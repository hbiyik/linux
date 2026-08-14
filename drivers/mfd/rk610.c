// SPDX-License-Identifier: GPL-2.0
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/rk610.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

static const struct mfd_cell rk610_devs[] = {
	{
		.name = "rk610-lvds",
		.of_compatible = "rockchip,rk610-lvds",
	},
};

static const struct regmap_config rk610_regmap_config = {
	.name = "rk610-core",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = RK610_MAX_REGISTER,
};

static void rk610_hw_reset(struct rk610 *rk610)
{
	gpiod_set_value_cansleep(rk610->reset_gpio, 1);
	msleep(100);
	gpiod_set_value_cansleep(rk610->reset_gpio, 0);
	msleep(100);
	gpiod_set_value_cansleep(rk610->reset_gpio, 1);
}

static int rk610_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct rk610 *rk610;
	unsigned int val;
	int ret;

	rk610 = devm_kzalloc(dev, sizeof(*rk610), GFP_KERNEL);
	if (!rk610)
		return -ENOMEM;

	rk610->dev = dev;
	rk610->client = client;
	i2c_set_clientdata(client, rk610);

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "failed to get/enable vdd supply\n");

	rk610->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(rk610->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(rk610->reset_gpio),
				     "failed to get reset GPIO\n");

	{
		struct clk_bulk_data *clks;

		ret = devm_clk_bulk_get_all_enabled(dev, &clks);
		if (ret < 0)
			return dev_err_probe(dev, ret, "failed to get/enable clocks\n");
	}

	rk610_hw_reset(rk610);
	msleep(100);

	rk610->regmap = devm_regmap_init_i2c(client, &rk610_regmap_config);
	if (IS_ERR(rk610->regmap))
		return dev_err_probe(dev, PTR_ERR(rk610->regmap),
				     "failed to init regmap\n");

	ret = regmap_read(rk610->regmap, RK610_C_PLL_CON0, &val);
	if (ret)
		return dev_err_probe(dev, ret, "chip did not respond\n");

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, rk610_devs,
				     ARRAY_SIZE(rk610_devs), NULL, 0, NULL);
}

static const struct of_device_id rk610_of_match[] = {
	{ .compatible = "rockchip,rk610" },
	{}
};
MODULE_DEVICE_TABLE(of, rk610_of_match);

static const struct i2c_device_id rk610_i2c_id[] = {
	{ "rk610", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, rk610_i2c_id);

static struct i2c_driver rk610_driver = {
	.driver = {
		.name = "rk610",
		.of_match_table = rk610_of_match,
	},
	.probe = rk610_i2c_probe,
	.id_table = rk610_i2c_id,
};
module_i2c_driver(rk610_driver);

MODULE_DESCRIPTION("Rockchip RK610 MFD core driver");
MODULE_LICENSE("GPL v2");
