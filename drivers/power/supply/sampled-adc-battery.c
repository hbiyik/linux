// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sampled ADC battery driver.
 *
 * Reads analog (IIO) and digital (GPIO) battery-monitoring inputs a
 * configurable number of times ("count"), spaced by a configurable
 * "sleep-period" in milliseconds. Every property read is a fully
 * synchronous, bounded-duration operation:
 *
 *   - Analog inputs (voltage, temperature) are arithmetically averaged
 *     over the sampling window.
 *   - Digital inputs (charge-finished, supplied/online, battery-low)
 *     are trusted only if they show at most one transition during the
 *     sampling window; the value after the (at most one) transition is
 *     used. Two or more transitions during the window means the input
 *     is reported as unconfirmed for that read.
 *   - If the charge-finished or supplied/online input changes during
 *     the window, the analog accumulators are reset and restart from
 *     that sample onward, since a change in charging regime can shift
 *     the instantaneous battery terminal voltage. battery-low changes
 *     do not reset the analog accumulators.
 *   - A candidate change on any digital input is only accepted once
 *     the same new value has been observed again at least
 *     "transient-reject" milliseconds after it was first seen. This
 *     absorbs brief transients (e.g. float-charge regulation ripple
 *     on a charge-status pin) without ever counting them as a
 *     transition, while still accepting a genuine, sustained change
 *     quickly. transient-reject defaults to 0 (disabled): any change
 *     is accepted immediately, as if this feature did not exist.
 *
 * Total worst-case block time for any property read is fixed at
 * (count - 1) * sleep_period milliseconds, regardless of how many of
 * the above inputs that property depends on. Transient rejection does
 * not add to this bound: a rejected candidate is simply never counted,
 * it does not extend the sampling window.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/slab.h>

enum sab_chan_type {
	SAB_VOLTAGE = 0,
	SAB_TEMP,
	SAB_MAX_CHAN_TYPE,
};

static const char *const sab_chan_name[SAB_MAX_CHAN_TYPE] = {
	[SAB_VOLTAGE] = "voltage",
	[SAB_TEMP]    = "temperature",
};

struct sab_digital_track {
	bool has_first;
	bool last_value;
	unsigned int transitions;

	bool has_pending;
	bool pending_value;
	u64 pending_since_ns;
};

struct sab_analog_track {
	s64 sum;
	unsigned int count;
};

struct sab_sample {
	int voltage_uv;
	bool voltage_valid;
	int temp;
	bool temp_valid;

	bool charged;
	bool charged_valid;
	bool supplied;
	bool supplied_valid;
	bool battery_low;
	bool battery_low_valid;
};

struct sab_device {
	struct device *dev;
	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	struct power_supply_battery_info *info;

	struct iio_channel *channel[SAB_MAX_CHAN_TYPE];
	struct gpio_desc *charge_finished;
	struct gpio_desc *battery_low;

	u32 sleep_period;
	u32 count;
	u64 transient_reject_ns;
};

static bool sab_digital_track_sample(struct sab_digital_track *t, bool raw,
				      u64 now_ns, u64 reject_ns)
{
	if (!t->has_first) {
		t->last_value = raw;
		t->has_first = true;
		return false;
	}

	if (raw == t->last_value) {
		t->has_pending = false;
		return false;
	}

	if (!reject_ns) {
		t->last_value = raw;
		t->transitions++;
		return true;
	}

	if (!t->has_pending || t->pending_value != raw) {
		t->has_pending = true;
		t->pending_value = raw;
		t->pending_since_ns = now_ns;
		return false;
	}

	if (now_ns - t->pending_since_ns < reject_ns)
		return false;

	t->last_value = raw;
	t->transitions++;
	t->has_pending = false;
	return true;
}

static bool sab_digital_track_result(struct sab_digital_track *t, bool *valid)
{
	*valid = t->has_first && (t->transitions <= 1);
	return t->last_value;
}

static void sab_analog_track_reset(struct sab_analog_track *t)
{
	t->sum = 0;
	t->count = 0;
}

static void sab_analog_track_sample(struct sab_analog_track *t, int value)
{
	t->sum += value;
	t->count++;
}

static int sab_analog_track_result(struct sab_analog_track *t)
{
	return t->count ? (int)div_s64(t->sum, t->count) : 0;
}

static int sab_read_channel(struct sab_device *sab, enum sab_chan_type channel,
			     int *result)
{
	int ret;

	ret = iio_read_channel_processed(sab->channel[channel], result);
	if (ret < 0)
		dev_err(sab->dev, "read channel error: %d\n", ret);
	else
		*result *= 1000;

	return ret;
}

static void sab_sample_all(struct sab_device *sab, struct sab_sample *out)
{
	struct sab_digital_track charged_track = { 0 };
	struct sab_digital_track supplied_track = { 0 };
	struct sab_digital_track batlow_track = { 0 };
	struct sab_analog_track voltage_track = { 0 };
	struct sab_analog_track temp_track = { 0 };
	u32 count = sab->count ? sab->count : 1;
	unsigned int i;

	memset(out, 0, sizeof(*out));

	for (i = 0; i < count; i++) {
		bool charged_raw = false, supplied_raw, batlow_raw = false;
		bool charged_transitioned = false, supplied_transitioned;
		u64 now_ns = ktime_get_ns();
		int ret;

		if (sab->charge_finished)
			charged_raw = gpiod_get_value_cansleep(sab->charge_finished);

		supplied_raw = power_supply_am_i_supplied(sab->psy) != 0;

		if (sab->battery_low)
			batlow_raw = gpiod_get_value_cansleep(sab->battery_low);

		if (sab->charge_finished)
			charged_transitioned = sab_digital_track_sample(&charged_track,
					charged_raw, now_ns, sab->transient_reject_ns);
		supplied_transitioned = sab_digital_track_sample(&supplied_track,
				supplied_raw, now_ns, sab->transient_reject_ns);
		if (sab->battery_low)
			sab_digital_track_sample(&batlow_track, batlow_raw,
					now_ns, sab->transient_reject_ns);

		if (charged_transitioned || supplied_transitioned) {
			sab_analog_track_reset(&voltage_track);
			sab_analog_track_reset(&temp_track);
		}

		if (sab->channel[SAB_VOLTAGE]) {
			int uv;

			ret = sab_read_channel(sab, SAB_VOLTAGE, &uv);
			if (ret == 0)
				sab_analog_track_sample(&voltage_track, uv);
		}

		if (sab->channel[SAB_TEMP]) {
			int t;

			ret = sab_read_channel(sab, SAB_TEMP, &t);
			if (ret == 0)
				sab_analog_track_sample(&temp_track, t);
		}

		if (i < count - 1 && sab->sleep_period)
			msleep(sab->sleep_period);
	}

	if (sab->charge_finished)
		out->charged = sab_digital_track_result(&charged_track, &out->charged_valid);
	else
		out->charged_valid = true; /* no such input: treat as N/A, never blocks status */

	out->supplied = sab_digital_track_result(&supplied_track, &out->supplied_valid);

	if (sab->battery_low)
		out->battery_low = sab_digital_track_result(&batlow_track, &out->battery_low_valid);

	if (voltage_track.count) {
		out->voltage_uv = sab_analog_track_result(&voltage_track);
		out->voltage_valid = true;
	}

	if (temp_track.count) {
		out->temp = sab_analog_track_result(&temp_track);
		out->temp_valid = true;
	}
}

static int sab_sample_status(struct sab_device *sab, struct sab_sample *sample,
			      int *status)
{
	if (!sample->supplied_valid)
		return -ENODATA;
	if (sab->charge_finished && !sample->charged_valid)
		return -ENODATA;

	if (!sample->supplied)
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (sab->charge_finished && sample->charged)
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		*status = POWER_SUPPLY_STATUS_CHARGING;

	return 0;
}

static int sab_resolve_temp(struct sab_device *sab, struct sab_sample *sample,
			     int *temp)
{
	if (sample->temp_valid) {
		*temp = sample->temp;
		return 0;
	}

	if (power_supply_battery_info_has_prop(sab->info,
			POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MIN) &&
	    power_supply_battery_info_has_prop(sab->info,
			POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MAX)) {
		*temp = (sab->info->temp_ambient_alert_min +
			 sab->info->temp_ambient_alert_max) / 2;
		return 0;
	}

	return -ENODATA;
}

static int sab_get_property(struct power_supply *psy,
			     enum power_supply_property psp,
			     union power_supply_propval *val)
{
	struct sab_device *sab = power_supply_get_drvdata(psy);
	struct sab_sample sample;
	int ret, status, temp;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;

	case POWER_SUPPLY_PROP_STATUS:
		sab_sample_all(sab, &sample);
		ret = sab_sample_status(sab, &sample, &status);
		val->intval = ret ? POWER_SUPPLY_STATUS_UNKNOWN : status;
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		sab_sample_all(sab, &sample);
		if (!sample.voltage_valid)
			return -ENODATA;
		val->intval = sample.voltage_uv;
		return 0;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (!sab->info)
			return -ENODATA;

		sab_sample_all(sab, &sample);

		if (!sample.voltage_valid)
			return -ENODATA;

		ret = sab_sample_status(sab, &sample, &status);
		if (ret < 0)
			return ret;

		ret = sab_resolve_temp(sab, &sample, &temp);
		if (ret < 0)
			return ret;

		val->intval = (status == POWER_SUPPLY_STATUS_CHARGING) ?
			power_supply_batinfo_ocv2cap(sab->info, sample.voltage_uv, temp) :
			power_supply_batinfo_ccv2cap(sab->info, sample.voltage_uv, temp);
		return 0;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (!sab->battery_low)
			return -ENODATA;

		sab_sample_all(sab, &sample);

		if (!sample.battery_low_valid)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		else if (sample.battery_low)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		return 0;

	default:
		return -EINVAL;
	}
}

static enum power_supply_property sab_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int sab_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = { };
	enum power_supply_property *properties;
	struct sab_device *sab;
	bool has_battery_info;
	unsigned int index;
	int ret, chan;

	sab = devm_kzalloc(&pdev->dev, sizeof(*sab), GFP_KERNEL);
	if (!sab)
		return -ENOMEM;

	sab->dev = &pdev->dev;
	platform_set_drvdata(pdev, sab);

	sab->sleep_period = 0;
	sab->count = 1;
	sab->transient_reject_ns = 0;
	device_property_read_u32(&pdev->dev, "sleep-period", &sab->sleep_period);
	device_property_read_u32(&pdev->dev, "count", &sab->count);
	{
		u32 transient_reject_ms = 0;

		device_property_read_u32(&pdev->dev, "transient-reject",
					  &transient_reject_ms);
		sab->transient_reject_ns = (u64)transient_reject_ms * NSEC_PER_MSEC;
	}

	for (chan = 0; chan < SAB_MAX_CHAN_TYPE; chan++) {
		sab->channel[chan] = devm_iio_channel_get(&pdev->dev, sab_chan_name[chan]);
		if (IS_ERR(sab->channel[chan])) {
			ret = PTR_ERR(sab->channel[chan]);
			sab->channel[chan] = NULL;

			if (ret == -EPROBE_DEFER)
				return ret;
			if (chan == SAB_VOLTAGE)
				return dev_err_probe(&pdev->dev, ret,
						"Failed to get voltage channel\n");
		}
	}

	sab->charge_finished = devm_gpiod_get_optional(&pdev->dev, "charged", GPIOD_IN);
	if (IS_ERR(sab->charge_finished))
		return dev_err_probe(&pdev->dev, PTR_ERR(sab->charge_finished),
				"Failed to get charged gpio\n");

	sab->battery_low = devm_gpiod_get_optional(&pdev->dev, "battery-low", GPIOD_IN);
	if (IS_ERR(sab->battery_low))
		return dev_err_probe(&pdev->dev, PTR_ERR(sab->battery_low),
				"Failed to get battery-low gpio\n");

	has_battery_info = device_property_present(&pdev->dev, "monitored-battery");

	index = ARRAY_SIZE(sab_props);
	properties = devm_kcalloc(&pdev->dev, ARRAY_SIZE(sab_props) + 2,
				   sizeof(*properties), GFP_KERNEL);
	if (!properties)
		return -ENOMEM;

	memcpy(properties, sab_props, sizeof(sab_props));
	if (has_battery_info)
		properties[index++] = POWER_SUPPLY_PROP_CAPACITY;
	if (sab->battery_low)
		properties[index++] = POWER_SUPPLY_PROP_CAPACITY_LEVEL;

	sab->psy_desc.name = dev_name(&pdev->dev);
	sab->psy_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	sab->psy_desc.properties = properties;
	sab->psy_desc.num_properties = index;
	sab->psy_desc.get_property = sab_get_property;

	psy_cfg.fwnode = dev_fwnode(&pdev->dev);
	psy_cfg.drv_data = sab;

	sab->psy = devm_power_supply_register(&pdev->dev, &sab->psy_desc, &psy_cfg);
	if (IS_ERR(sab->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(sab->psy),
				"Failed to register power-supply device\n");

	if (has_battery_info) {
		ret = power_supply_get_battery_info(sab->psy, &sab->info);
		if (ret)
			dev_warn(&pdev->dev, "Failed to get battery info: %d\n", ret);
	}

	return 0;
}

static const struct of_device_id sab_of_match[] = {
	{ .compatible = "sampled-adc-battery" },
	{ }
};
MODULE_DEVICE_TABLE(of, sab_of_match);

static struct platform_driver sab_driver = {
	.driver = {
		.name = "sampled-adc-battery",
		.of_match_table = sab_of_match,
	},
	.probe = sab_probe,
};
module_platform_driver(sab_driver);

MODULE_DESCRIPTION("Sampled ADC battery driver");
MODULE_LICENSE("GPL");
