// SPDX-License-Identifier: GPL-2.0
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_edid.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include <sound/hdmi-codec.h>

#include "rk610_hdmi.h"

#define DDC_SEGMENT_ADDR 0x30

struct rk610_hdmi_i2c {
	struct i2c_adapter adap;
	u8 segment;
	u8 offset;
	struct mutex lock;
};

struct rk610_hdmi {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct drm_bridge bridge;
	struct rk610_hdmi_i2c *i2c;
	struct i2c_adapter *ddc;
	int irq;
	struct clk *clk;
	struct drm_connector *connector;
	struct platform_device *audio_pdev;
	struct completion edid_done;
};

static inline struct rk610_hdmi *bridge_to_rk610_hdmi(struct drm_bridge *b)
{
	return container_of(b, struct rk610_hdmi, bridge);
}

static int rk610_hdmi_sys_power(struct rk610_hdmi *hdmi, bool up)
{
	u8 val = (1 << RK610_SYS_CLK_SHIFT) |
		 ((up ? RK610_SYS_PWR_ON : RK610_SYS_PWR_OFF)
		  << RK610_SYS_PWR_SHIFT) |
		 RK610_INT_POL;
	return regmap_write(hdmi->regmap, RK610_HDMI_REG_SYS_CTRL, val);
}

static int rk610_hdmi_soft_reset(struct rk610_hdmi *hdmi)
{
	int ret;

	ret = regmap_write(hdmi->regmap, RK610_HDMI_REG_SOFT_RESET, 0x00);
	if (ret)
		return ret;
	msleep(10);
	ret = regmap_write(hdmi->regmap, RK610_HDMI_REG_SOFT_RESET, 0x01);
	if (ret)
		return ret;
	msleep(100);

	return 0;
}

static int rk610_hdmi_driver_pll_bringup(struct rk610_hdmi *hdmi)
{
	int ret;

	ret = regmap_write(hdmi->regmap, RK610_HDMI_REG_PHY_DRIVER,
			   RK610_HDMI_VAL_DRIVER_NORMAL);
	if (ret)
		return ret;
	ret = regmap_write(hdmi->regmap, RK610_HDMI_REG_PHY_PRE_EMPHASIS,
			   RK610_HDMI_VAL_PHY_PRE_EMPHASIS);
	if (ret)
		return ret;
	ret = regmap_write(hdmi->regmap, RK610_HDMI_REG_E8, RK610_HDMI_VAL_E8);
	if (ret)
		return ret;
	ret = regmap_write(hdmi->regmap, RK610_HDMI_REG_E6, RK610_HDMI_VAL_E6);
	if (ret)
		return ret;
	return regmap_write(hdmi->regmap, RK610_HDMI_REG_PHY_PLL_CTRL,
			    RK610_HDMI_VAL_PHY_PLL_CTRL_ON);
}

static int rk610_hdmi_analog_bringup(struct rk610_hdmi *hdmi)
{
	int ret;

	ret = regmap_write(hdmi->regmap, RK610_HDMI_REG_PHY_BANDGAP_PWR, 0x00);
	if (ret)
		return ret;
	ret = regmap_write(hdmi->regmap, RK610_HDMI_REG_PHY_PLL_LDO_PWR, 0x00);
	if (ret)
		return ret;
	msleep(1);
	return regmap_write(hdmi->regmap, RK610_HDMI_REG_PHY_PLL_CTRL, 0x00);
}

enum rk610_hdmi_pwr_mode {
	RK610_HDMI_PWR_NORMAL,
	RK610_HDMI_PWR_LOWER,
	RK610_HDMI_PWR_SHUTDOWN
};

static void rk610_hdmi_set_pwr_mode(struct rk610_hdmi *hdmi,
				    enum rk610_hdmi_pwr_mode mode)
{
	struct regmap *rm = hdmi->regmap;

	switch (mode) {
	case RK610_HDMI_PWR_NORMAL:
		regmap_write(rm, RK610_HDMI_REG_PHY_PLL_TEST,
			     RK610_HDMI_VAL_PHY_PLL_TEST_NORMAL);
		regmap_write(rm, RK610_HDMI_REG_PHY_PLL_CTRL,
			     RK610_HDMI_VAL_PHY_PLL_CTRL_ON);
		regmap_write(rm, RK610_HDMI_REG_PHY_PLL_LDO_PWR,
			     RK610_HDMI_VAL_PHY_PLL_LDO_PWRDOWN(0));
		regmap_write(rm, RK610_HDMI_REG_PHY_BANDGAP_PWR,
			     RK610_HDMI_VAL_PHY_BANDGAP_PWRUP);
		regmap_write(rm, RK610_HDMI_REG_PHY_DRIVER,
			     RK610_HDMI_VAL_DRIVER_NORMAL);
		break;
	case RK610_HDMI_PWR_LOWER:
		regmap_write(rm, RK610_HDMI_REG_PHY_PLL_TEST,
			     RK610_HDMI_VAL_PHY_PLL_TEST_LOWER);
		regmap_write(rm, RK610_HDMI_REG_PHY_PLL_CTRL,
			     RK610_HDMI_VAL_PHY_PLL_CTRL_OFF);
		break;
	case RK610_HDMI_PWR_SHUTDOWN:
		regmap_write(rm, RK610_HDMI_REG_PHY_PLL_TEST,
			     RK610_HDMI_VAL_PHY_PLL_TEST_LOWER);
		regmap_write(rm, RK610_HDMI_REG_PHY_PLL_CTRL,
			     RK610_HDMI_VAL_PHY_PLL_CTRL_OFF);
		regmap_write(rm, RK610_HDMI_REG_PHY_PLL_LDO_PWR,
			     RK610_HDMI_VAL_PHY_PLL_LDO_PWRDOWN(1));
		regmap_write(rm, RK610_HDMI_REG_PHY_BANDGAP_PWR,
			     RK610_HDMI_VAL_PHY_BANDGAP_PWRDOWN);
		regmap_write(rm, RK610_HDMI_REG_PHY_DRIVER,
			     RK610_HDMI_VAL_DRIVER_SHUTDOWN);
		break;
	}
}

static enum drm_connector_status
rk610_hdmi_bridge_detect(struct drm_bridge *bridge,
			 struct drm_connector *connector)
{
	struct rk610_hdmi *hdmi = bridge_to_rk610_hdmi(bridge);
	unsigned int val;

	regmap_read(hdmi->regmap, RK610_HDMI_REG_HPD_STATUS, &val);

	return (val & RK610_HDMI_HPD_PLUG) ? connector_status_connected :
					     connector_status_disconnected;
}

#define RK610_HDMI_EDID_TIMEOUT_MS 500
#define RK610_HDMI_EDID_CHUNK_SIZE 32

static bool rk610_hdmi_edid_wait_ready(struct rk610_hdmi *hdmi)
{
	struct regmap *rm = hdmi->regmap;
	unsigned int val;
	int i;

	if (hdmi->irq > 0) {
		return wait_for_completion_timeout(
			       &hdmi->edid_done,
			       msecs_to_jiffies(RK610_HDMI_EDID_TIMEOUT_MS)) !=
		       0;
	}

	for (i = 0; i < 10; i++) {
		msleep(10);
		regmap_read(rm, RK610_HDMI_REG_INT_STATUS, &val);
		if (val & RK610_HDMI_EDID_EVENT)
			return true;
		msleep(100);
	}
	return false;
}

static int rk610_hdmi_i2c_read(struct rk610_hdmi *hdmi, struct i2c_msg *msg)
{
	struct regmap *rm = hdmi->regmap;
	unsigned int val;
	u8 *buf = msg->buf;
	int length = msg->len;
	unsigned int segment = hdmi->i2c->segment;
	unsigned int offset = hdmi->i2c->offset;
	int chunk_off;

	for (chunk_off = 0; chunk_off < length;
	     chunk_off += RK610_HDMI_EDID_CHUNK_SIZE) {
		int chunk_len = min_t(int, RK610_HDMI_EDID_CHUNK_SIZE,
				      length - chunk_off);
		int j;

		if (chunk_off > 0) {
			if (hdmi->irq > 0)
				reinit_completion(&hdmi->edid_done);

			regmap_write(rm, RK610_HDMI_REG_EDID_FIFO_ADDR, 0x00);
			regmap_write(rm, RK610_HDMI_REG_EDID_WORD_ADDR,
				     offset + chunk_off);
			regmap_write(rm, RK610_HDMI_REG_EDID_SEG, segment);
		}

		if (!rk610_hdmi_edid_wait_ready(hdmi))
			return -EAGAIN;

		regmap_write(rm, RK610_HDMI_REG_INT_STATUS,
			     RK610_HDMI_EDID_EVENT);

		for (j = 0; j < chunk_len; j++) {
			regmap_read(rm, RK610_HDMI_REG_EDID_FIFO_DATA, &val);
			buf[chunk_off + j] = val & 0xff;
		}
	}

	return 0;
}

static int rk610_hdmi_i2c_write(struct rk610_hdmi *hdmi, struct i2c_msg *msg)
{
	struct regmap *rm = hdmi->regmap;

	if (msg->len != 1 ||
	    (msg->addr != DDC_ADDR && msg->addr != DDC_SEGMENT_ADDR))
		return -EINVAL;

	if (msg->addr == DDC_SEGMENT_ADDR) {
		hdmi->i2c->segment = msg->buf[0];
		return 0;
	}

	hdmi->i2c->offset = msg->buf[0];

	regmap_write(rm, RK610_HDMI_REG_EDID_FIFO_ADDR, 0x00);
	regmap_write(rm, RK610_HDMI_REG_EDID_WORD_ADDR, hdmi->i2c->offset);
	regmap_write(rm, RK610_HDMI_REG_EDID_SEG, hdmi->i2c->segment);

	return 0;
}

static unsigned int rk610_hdmi_ddc_config(struct rk610_hdmi *hdmi)
{
	unsigned long ddc_clk_in = clk_get_rate(hdmi->clk);

	return (ddc_clk_in >> 2) / RK610_HDMI_DDC_SCL_RATE;
}

static int rk610_hdmi_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			       int num)
{
	struct rk610_hdmi *hdmi = i2c_get_adapdata(adap);
	struct rk610_hdmi_i2c *i2c = hdmi->i2c;
	unsigned int mask;
	int i, ret = 0;

	mutex_lock(&i2c->lock);

	i2c->segment = 0;
	i2c->offset = 0;

	{
		unsigned int ddc_config = rk610_hdmi_ddc_config(hdmi);

		regmap_write(hdmi->regmap, RK610_HDMI_REG_DDC_CFG_LO,
			     ddc_config & 0xff);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_DDC_CFG_HI,
			     (ddc_config >> 8) & 0xff);
	}

	if (hdmi->irq > 0)
		reinit_completion(&hdmi->edid_done);

	regmap_write(hdmi->regmap, RK610_HDMI_REG_INT_STATUS,
		     RK610_HDMI_EDID_EVENT);

	regmap_read(hdmi->regmap, RK610_HDMI_REG_INT_MASK, &mask);
	regmap_write(hdmi->regmap, RK610_HDMI_REG_INT_MASK,
		     mask | RK610_HDMI_EDID_EVENT);

	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_RD)
			ret = rk610_hdmi_i2c_read(hdmi, &msgs[i]);
		else
			ret = rk610_hdmi_i2c_write(hdmi, &msgs[i]);

		if (ret < 0)
			break;
	}

	if (!ret)
		ret = num;

	regmap_read(hdmi->regmap, RK610_HDMI_REG_INT_MASK, &mask);
	regmap_write(hdmi->regmap, RK610_HDMI_REG_INT_MASK,
		     mask & ~RK610_HDMI_EDID_EVENT);

	mutex_unlock(&i2c->lock);

	return ret;
}

static u32 rk610_hdmi_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm rk610_hdmi_i2c_algo = {
	.master_xfer = rk610_hdmi_i2c_xfer,
	.functionality = rk610_hdmi_i2c_func,
};

static struct i2c_adapter *rk610_hdmi_i2c_adapter(struct rk610_hdmi *hdmi)
{
	struct i2c_adapter *adap;
	struct rk610_hdmi_i2c *i2c;
	int ret;

	i2c = devm_kzalloc(hdmi->dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return ERR_PTR(-ENOMEM);

	mutex_init(&i2c->lock);

	adap = &i2c->adap;
	adap->owner = THIS_MODULE;
	adap->dev.parent = hdmi->dev;
	adap->dev.of_node = hdmi->dev->of_node;
	adap->algo = &rk610_hdmi_i2c_algo;
	strscpy(adap->name, "rk610-hdmi", sizeof(adap->name));
	i2c_set_adapdata(adap, hdmi);

	ret = devm_i2c_add_adapter(hdmi->dev, adap);
	if (ret)
		return ERR_PTR(ret);

	hdmi->i2c = i2c;

	return adap;
}

static const struct drm_edid *
rk610_hdmi_bridge_edid_read(struct drm_bridge *bridge,
			    struct drm_connector *connector)
{
	struct rk610_hdmi *hdmi = bridge_to_rk610_hdmi(bridge);

	if (rk610_hdmi_sys_power(hdmi, true))
		return NULL;

	if (rk610_hdmi_analog_bringup(hdmi))
		return NULL;

	return drm_edid_read_ddc(connector, bridge->ddc);
}

static void rk610_hdmi_config_avi(struct rk610_hdmi *hdmi, u8 vic)
{
	struct regmap *rm = hdmi->regmap;
	u8 info[9] = { 0x82, 0x02, 0x0d, 0x00, 0x00, 0x08, 0x00, vic, 0x00 };
	unsigned int sum = 0;
	int i;

	for (i = 0; i < 3; i++)
		sum += info[i];
	for (i = 4; i < 9; i++)
		sum += info[i];
	info[3] = (0x100 - (sum & 0xff)) & 0xff;

	regmap_write(rm, RK610_HDMI_REG_CONTROL_PACKET_BUF_INDEX,
		     RK610_HDMI_INFOFRAME_AVI);
	for (i = 0; i < 9; i++)
		regmap_write(rm, RK610_HDMI_REG_CONTROL_PACKET_ADDR + i,
			     info[i]);
}

static void rk610_hdmi_config_aai(struct rk610_hdmi *hdmi, u8 channels)
{
	struct regmap *rm = hdmi->regmap;
	u8 info[8] = { 0x84, 0x01, 0x0a, 0x00, channels, 0x00, 0x00, 0x00 };
	unsigned int sum = 0;
	int i;

	for (i = 0; i < 3; i++)
		sum += info[i];
	for (i = 4; i < 8; i++)
		sum += info[i];
	info[3] = (0x100 - (sum & 0xff)) & 0xff;

	regmap_write(rm, RK610_HDMI_REG_CONTROL_PACKET_BUF_INDEX,
		     RK610_HDMI_INFOFRAME_AAI);
	for (i = 0; i < 8; i++)
		regmap_write(rm, RK610_HDMI_REG_CONTROL_PACKET_ADDR + i,
			     info[i]);
}

static void rk610_hdmi_bridge_atomic_enable(struct drm_bridge *bridge,
					    struct drm_atomic_state *state)
{
	struct rk610_hdmi *hdmi = bridge_to_rk610_hdmi(bridge);
	struct drm_connector *connector;
	struct drm_crtc_state *crtc_state;
	struct drm_connector_state *conn_state;
	u8 vic;

	connector = drm_atomic_get_new_connector_for_encoder(state,
							     bridge->encoder);
	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (WARN_ON(!conn_state))
		return;
	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (WARN_ON(!crtc_state))
		return;

	vic = drm_match_cea_mode(&crtc_state->adjusted_mode);

	hdmi->connector = connector;

	if (rk610_hdmi_soft_reset(hdmi) ||
	    rk610_hdmi_driver_pll_bringup(hdmi)) {
		dev_err(hdmi->dev,
			"chip not responding during atomic_enable\n");
		return;
	}

	{
		const struct drm_display_mode *m = &crtc_state->adjusted_mode;
		unsigned int htotal = m->htotal;
		unsigned int hblank = m->htotal - m->hdisplay;
		unsigned int hdelay = m->htotal - m->hsync_start;
		unsigned int hduration = m->hsync_end - m->hsync_start;
		unsigned int vtotal = m->vtotal;
		unsigned int vblank = m->vtotal - m->vdisplay;
		unsigned int vdelay = m->vtotal - m->vsync_start;
		unsigned int vduration = m->vsync_end - m->vsync_start;
		u8 timing_ctl = RK610_TIMING_EXTERNAL_VIDEO;

		if (m->flags & DRM_MODE_FLAG_INTERLACE)
			timing_ctl |= RK610_TIMING_INTERLACE;
		if (m->flags & DRM_MODE_FLAG_PHSYNC)
			timing_ctl |= RK610_TIMING_HSYNC_POL;
		if (m->flags & DRM_MODE_FLAG_PVSYNC)
			timing_ctl |= RK610_TIMING_VSYNC_POL;

		regmap_write(hdmi->regmap, RK610_HDMI_REG_AV_MUTE, 0x03);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_CONTROL1, 0x01);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_CONTROL2, 0x30);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_CONTROL3, 0x08);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_TIMING_CTL,
			     timing_ctl);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_EXT_HTOTAL_L,
			     htotal & 0xff);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_EXT_HTOTAL_H,
			     (htotal >> 8) & 0xff);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_EXT_HBLANK_L,
			     hblank & 0xff);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_EXT_HBLANK_H,
			     (hblank >> 8) & 0xff);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_EXT_HDELAY_L,
			     hdelay & 0xff);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_EXT_HDELAY_H,
			     (hdelay >> 8) & 0xff);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_EXT_HDURATION_L,
			     hduration & 0xff);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_EXT_HDURATION_H,
			     (hduration >> 8) & 0xff);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_EXT_VTOTAL_L,
			     vtotal & 0xff);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_EXT_VTOTAL_H,
			     (vtotal >> 8) & 0xff);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_EXT_VBLANK,
			     vblank & 0xff);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_EXT_VDELAY,
			     vdelay & 0xff);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_VIDEO_EXT_VDURATION,
			     vduration & 0xff);
	}
	regmap_write(hdmi->regmap, RK610_HDMI_REG_OUTPUT_MODE,
		     RK610_HDMI_OUTPUT_HDMI);
	rk610_hdmi_config_avi(hdmi, vic);

	regmap_write(hdmi->regmap, RK610_HDMI_REG_PHY_PRE_EMPHASIS, 0x00);
	{
		unsigned int e1;

		regmap_read(hdmi->regmap, RK610_HDMI_REG_PHY_DRIVER, &e1);
		regmap_write(hdmi->regmap, RK610_HDMI_REG_PHY_DRIVER,
			     e1 | RK610_HDMI_VAL_PHY_DRIVER_TXEN(1));
	}

	rk610_hdmi_sys_power(hdmi, true);
	rk610_hdmi_sys_power(hdmi, false);
	rk610_hdmi_sys_power(hdmi, true);

	rk610_hdmi_set_pwr_mode(hdmi, RK610_HDMI_PWR_NORMAL);

	regmap_write(hdmi->regmap, RK610_HDMI_REG_AV_MUTE, 0x00);
}

static void rk610_hdmi_bridge_atomic_disable(struct drm_bridge *bridge,
					     struct drm_atomic_state *state)
{
	struct rk610_hdmi *hdmi = bridge_to_rk610_hdmi(bridge);

	regmap_write(hdmi->regmap, RK610_HDMI_REG_AV_MUTE, 0x03);
	rk610_hdmi_set_pwr_mode(hdmi, RK610_HDMI_PWR_LOWER);
}

struct rk610_hdmi_audio_rate {
	unsigned int rate;
	u8 sample_rate_val;
	u32 n_val;
};

static const struct rk610_hdmi_audio_rate rk610_hdmi_audio_rates[] = {
	{ 32000, RK610_AUDIO_SAMPLE_RATE_32K, RK610_AUDIO_N_32K },
	{ 44100, RK610_AUDIO_SAMPLE_RATE_441K, RK610_AUDIO_N_441K },
	{ 48000, RK610_AUDIO_SAMPLE_RATE_48K, RK610_AUDIO_N_48K },
	{ 88200, RK610_AUDIO_SAMPLE_RATE_882K, RK610_AUDIO_N_882K },
	{ 96000, RK610_AUDIO_SAMPLE_RATE_96K, RK610_AUDIO_N_96K },
	{ 176400, RK610_AUDIO_SAMPLE_RATE_1764K, RK610_AUDIO_N_1764K },
	{ 192000, RK610_AUDIO_SAMPLE_RATE_192K, RK610_AUDIO_N_192K },
};

static int rk610_hdmi_audio_hw_params(struct device *dev, void *data,
				      struct hdmi_codec_daifmt *daifmt,
				      struct hdmi_codec_params *params)
{
	struct rk610_hdmi *hdmi = dev_get_drvdata(dev);
	struct regmap *rm = hdmi->regmap;
	const struct rk610_hdmi_audio_rate *r = NULL;
	unsigned int i;

	if (daifmt->fmt != HDMI_I2S) {
		dev_err(dev,
			"unsupported audio format %u, only I2S is supported\n",
			daifmt->fmt);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(rk610_hdmi_audio_rates); i++) {
		if (rk610_hdmi_audio_rates[i].rate == params->sample_rate) {
			r = &rk610_hdmi_audio_rates[i];
			break;
		}
	}
	if (!r) {
		dev_err(dev, "unsupported audio sample rate %d\n",
			params->sample_rate);
		return -EINVAL;
	}

	regmap_write(rm, RK610_HDMI_REG_AUDIO_CTRL1,
		     RK610_AUDIO_CTS_SOURCE(RK610_AUDIO_CTS_SOURCE_INTERNAL) |
			     RK610_AUDIO_DOWN_SAMPLE(
				     RK610_AUDIO_DOWNSAMPLE_DISABLE) |
			     RK610_AUDIO_SOURCE(RK610_AUDIO_SOURCE_IIS) |
			     RK610_AUDIO_MCLK_ENABLE(1) |
			     RK610_AUDIO_MCLK_RATIO(RK610_AUDIO_MCLK_256FS));

	regmap_write(rm, RK610_HDMI_REG_AUDIO_SAMPLE_RATE, r->sample_rate_val);

	regmap_write(rm, RK610_HDMI_REG_AUDIO_I2S_MODE,
		     RK610_AUDIO_I2S_CHANNEL(RK610_AUDIO_I2S_CHANNEL_1_2) |
			     RK610_AUDIO_I2S_MODE_STANDARD);

	regmap_write(rm, RK610_HDMI_REG_AUDIO_N_H, (r->n_val >> 16) & 0xff);
	regmap_write(rm, RK610_HDMI_REG_AUDIO_N_M, (r->n_val >> 8) & 0xff);
	regmap_write(rm, RK610_HDMI_REG_AUDIO_N_L, r->n_val & 0xff);

	rk610_hdmi_config_aai(hdmi, 0x01);

	return 0;
}

static void rk610_hdmi_audio_shutdown(struct device *dev, void *data)
{
	struct rk610_hdmi *hdmi = dev_get_drvdata(dev);

	regmap_write(hdmi->regmap, RK610_HDMI_REG_AV_MUTE, 0x02);
}

static int rk610_hdmi_audio_mute_stream(struct device *dev, void *data,
					bool mute, int direction)
{
	struct rk610_hdmi *hdmi = dev_get_drvdata(dev);
	unsigned int av_mute;

	regmap_read(hdmi->regmap, RK610_HDMI_REG_AV_MUTE, &av_mute);
	if (mute)
		av_mute |= BIT(1);
	else
		av_mute &= ~BIT(1);
	regmap_write(hdmi->regmap, RK610_HDMI_REG_AV_MUTE, av_mute);

	return 0;
}

static int rk610_hdmi_audio_get_eld(struct device *dev, void *data,
				    uint8_t *buf, size_t len)
{
	struct rk610_hdmi *hdmi = dev_get_drvdata(dev);

	if (!hdmi->connector) {
		memset(buf, 0, len);
		return 0;
	}

	memcpy(buf, hdmi->connector->eld,
	       min(sizeof(hdmi->connector->eld), len));

	return 0;
}

static const struct hdmi_codec_ops rk610_hdmi_audio_codec_ops = {
	.hw_params = rk610_hdmi_audio_hw_params,
	.audio_shutdown = rk610_hdmi_audio_shutdown,
	.mute_stream = rk610_hdmi_audio_mute_stream,
	.get_eld = rk610_hdmi_audio_get_eld,
};

static int rk610_hdmi_audio_init(struct rk610_hdmi *hdmi)
{
	struct device *dev = hdmi->dev;
	struct hdmi_codec_pdata codec_data = {
		.ops = &rk610_hdmi_audio_codec_ops,
		.i2s = 1,
		.max_i2s_channels = 2,
	};

	if (!of_property_present(dev->of_node, "#sound-dai-cells"))
		return 0;

	hdmi->audio_pdev = platform_device_register_data(
		dev, HDMI_CODEC_DRV_NAME, PLATFORM_DEVID_AUTO, &codec_data,
		sizeof(codec_data));

	return PTR_ERR_OR_ZERO(hdmi->audio_pdev);
}

static int rk610_hdmi_bridge_attach(struct drm_bridge *bridge,
				    struct drm_encoder *encoder,
				    enum drm_bridge_attach_flags flags)
{
	return 0;
}

static const struct drm_bridge_funcs rk610_hdmi_bridge_funcs = {
	.attach = rk610_hdmi_bridge_attach,
	.atomic_enable = rk610_hdmi_bridge_atomic_enable,
	.atomic_disable = rk610_hdmi_bridge_atomic_disable,
	.detect = rk610_hdmi_bridge_detect,
	.edid_read = rk610_hdmi_bridge_edid_read,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
};

static irqreturn_t rk610_hdmi_irq(int irq, void *dev_id)
{
	struct rk610_hdmi *hdmi = dev_id;
	unsigned int stat;

	regmap_read(hdmi->regmap, RK610_HDMI_REG_INT_STATUS, &stat);
	if (!stat)
		return IRQ_NONE;

	regmap_write(hdmi->regmap, RK610_HDMI_REG_INT_STATUS, stat);

	if (stat & RK610_HDMI_EDID_EVENT)
		complete(&hdmi->edid_done);

	if (stat & RK610_HDMI_HPD_EVENT)
		drm_helper_hpd_irq_event(hdmi->bridge.dev);

	return IRQ_HANDLED;
}

static const struct regmap_config rk610_hdmi_regmap_config = {
	.name = "rk610-hdmi",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static int rk610_hdmi_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct rk610_hdmi *hdmi;
	int ret;

	hdmi = devm_drm_bridge_alloc(dev, struct rk610_hdmi, bridge,
				     &rk610_hdmi_bridge_funcs);
	if (IS_ERR(hdmi))
		return PTR_ERR(hdmi);

	hdmi->dev = dev;
	hdmi->client = client;
	i2c_set_clientdata(client, hdmi);
	init_completion(&hdmi->edid_done);

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to get/enable vdd supply\n");

	hdmi->regmap = devm_regmap_init_i2c(client, &rk610_hdmi_regmap_config);
	if (IS_ERR(hdmi->regmap))
		return dev_err_probe(dev, PTR_ERR(hdmi->regmap),
				     "failed to init regmap\n");

	hdmi->clk = devm_clk_get_enabled(dev, "refclk");
	if (IS_ERR(hdmi->clk))
		return dev_err_probe(dev, PTR_ERR(hdmi->clk),
				     "failed to get/enable refclk\n");

	hdmi->ddc = rk610_hdmi_i2c_adapter(hdmi);
	if (IS_ERR(hdmi->ddc))
		return dev_err_probe(dev, PTR_ERR(hdmi->ddc),
				     "failed to add ddc i2c adapter\n");
	hdmi->bridge.ddc = hdmi->ddc;

	hdmi->irq = client->irq;
	if (hdmi->irq > 0) {
		ret = devm_request_threaded_irq(dev, hdmi->irq, NULL,
						rk610_hdmi_irq, IRQF_ONESHOT,
						dev_name(dev), hdmi);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request irq\n");
	}

	ret = rk610_hdmi_soft_reset(hdmi);
	if (!ret)
		ret = rk610_hdmi_driver_pll_bringup(hdmi);
	if (!ret)
		ret = rk610_hdmi_sys_power(hdmi, false);
	if (ret)
		return dev_err_probe(dev, ret, "chip did not respond\n");

	ret = rk610_hdmi_audio_init(hdmi);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register audio codec\n");

	hdmi->bridge.of_node = dev->of_node;
	hdmi->bridge.type = DRM_MODE_CONNECTOR_HDMIA;
	hdmi->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID;
	if (hdmi->irq > 0)
		hdmi->bridge.ops |= DRM_BRIDGE_OP_HPD;

	drm_bridge_add(&hdmi->bridge);

	return 0;
}

static void rk610_hdmi_remove(struct i2c_client *client)
{
	struct rk610_hdmi *hdmi = i2c_get_clientdata(client);

	platform_device_unregister(hdmi->audio_pdev);
	drm_bridge_remove(&hdmi->bridge);
}

static const struct of_device_id rk610_hdmi_of_match[] = {
	{ .compatible = "rockchip,rk610-hdmi" },
	{}
};
MODULE_DEVICE_TABLE(of, rk610_hdmi_of_match);

static const struct i2c_device_id rk610_hdmi_id[] = { { "rk610_hdmi", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, rk610_hdmi_id);

static struct i2c_driver rk610_hdmi_driver = {
	.driver = {
		.name = "rk610-hdmi",
		.of_match_table = rk610_hdmi_of_match,
	},
	.probe = rk610_hdmi_probe,
	.remove = rk610_hdmi_remove,
	.id_table = rk610_hdmi_id,
};
module_i2c_driver(rk610_hdmi_driver);

MODULE_DESCRIPTION("Rockchip RK610 HDMI bridge driver");
MODULE_LICENSE("GPL v2");
