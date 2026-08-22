// SPDX-License-Identifier: GPL-2.0
/*
 * RK610 audio codec driver.
 *
 * The RK610 ("Jetta") multimedia companion chip's audio codec function
 * lives at its own independent I2C base address (0x60), confirmed
 * against the vendor GPL source and consistent with rk610-hdmi.c and
 * rk610-lvds.c, which are the chip's other independently-addressed
 * sub-functions (see include/linux/mfd/rk610.h for the full picture).
 *
 * Unlike a normal I2C register device, this chip folds the register
 * number directly into the low 5 bits of the I2C slave address on
 * every transaction (confirmed from the vendor driver:
 * i2c->addr = (i2c->addr & 0x60) | reg), rather than using a
 * conventional "write register address, then data" protocol. A custom
 * regmap_bus is used to replicate this exactly, so the rest of the
 * driver can use ordinary regmap_read()/regmap_write() calls.
 *
 * The analog front-end (input routing, mixer routing, bias/VMID
 * sequencing, anti-pop power-up) is configured once at probe time,
 * matching the vendor driver's rk610_codec_reg_set(). Behavior that
 * was a compile-time #define in the vendor driver (USE_MIC_IN,
 * USE_LPF, boot_depop, OUT_CAPLESS) is exposed as devicetree
 * properties instead, so it doesn't have to be a kernel rebuild away.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define RK610_CODEC_BASE_ADDR		0x60
#define RK610_CODEC_ADDR_MASK		0x60
#define RK610_CODEC_MAX_REG		0x1f

/* Register offsets, names matching the vendor header (rk610_codec.h) */
#define RK610_CODEC_R00			0x00 /* ADC high-pass filter / DSM */
#define RK610_CODEC_R04			0x04 /* soft mute / sidetone */
#define RK610_CODEC_R05			0x05 /* R interpolate vol MSB */
#define RK610_CODEC_R06			0x06 /* R interpolate vol LSB */
#define RK610_CODEC_R07			0x07 /* L interpolate vol MSB */
#define RK610_CODEC_R08			0x08 /* L interpolate vol LSB */
#define RK610_CODEC_R09			0x09 /* audio interface control */
#define RK610_CODEC_R0A			0x0a /* sample rate / clk control */
#define RK610_CODEC_R0B			0x0b /* decimation/interpolate enable */
#define RK610_CODEC_R0C			0x0c /* LIN volume */
#define RK610_CODEC_R0D			0x0d /* LIP volume */
#define RK610_CODEC_R0E			0x0e /* AL (mic) volume */
#define RK610_CODEC_R12			0x12 /* input select / mic boost */
#define RK610_CODEC_R13			0x13 /* left out mix */
#define RK610_CODEC_R14			0x14 /* right out mix */
#define RK610_CODEC_R15			0x15 /* LPF/DAC -> mixer routing */
#define RK610_CODEC_R17			0x17 /* AOL volume */
#define RK610_CODEC_R18			0x18 /* AOR volume */
#define RK610_CODEC_R19			0x19 /* AOM volume */
#define RK610_CODEC_R1A			0x1a /* MICBIAS / VMID ramp */
#define RK610_CODEC_R1C			0x1c /* ADC control (DEM etc.) */
#define RK610_CODEC_R1D			0x1d /* power mgr 1 */
#define RK610_CODEC_R1E			0x1e /* power mgr 2 */
#define RK610_CODEC_R1F			0x1f /* power mgr 3 */

/* R00 */
#define RK610_CODEC_R00_HPF_ENABLE	BIT(0)
#define RK610_CODEC_R00_DSM_ENABLE	BIT(1)
#define RK610_CODEC_R00_SCRAMBLE_EN	BIT(2)
#define RK610_CODEC_R00_DITHER_EN	BIT(3)
#define RK610_CODEC_R00_BCLKDIV_4	(0x1 << 4)

/* R04 */
#define RK610_CODEC_R04_MUTE_L		BIT(0)
#define RK610_CODEC_R04_MUTE_R		BIT(1)
#define RK610_CODEC_R04_MUTE		(RK610_CODEC_R04_MUTE_L | RK610_CODEC_R04_MUTE_R)

/* R09 */
#define RK610_CODEC_R09_I2S_MODE	0x2
#define RK610_CODEC_R09_16BIT_MODE	(0x0 << 2)
#define RK610_CODEC_R09_MASTER_MODE	BIT(6)

/* R0A */
#define RK610_CODEC_R0A_CLK_NO_DIV	BIT(6)
#define RK610_CODEC_R0A_CLK_ENABLE	BIT(7)

/* R0B */
#define RK610_CODEC_R0B_DEC_ENABLE	BIT(0)
#define RK610_CODEC_R0B_INT_ENABLE	BIT(1)

/* R12 */
#define RK610_CODEC_R12_MIC_INPUT	BIT(7)
#define RK610_CODEC_R12_MIC_BOOST_20DB	BIT(5)

/* R13/R14 */
#define RK610_CODEC_PGAMXVOL_0DB	0x5
#define RK610_CODEC_PGAMX_ENABLE	BIT(3)

/* R15 */
#define RK610_CODEC_R15_LDAMX_ENABLE	BIT(2)
#define RK610_CODEC_R15_RDAMX_ENABLE	BIT(3)
#define RK610_CODEC_R15_LLPFMX_ENABLE	BIT(6)
#define RK610_CODEC_R15_RLPFMX_ENABLE	BIT(7)
#define RK610_CODEC_R15_DAC_ROUTE	(RK610_CODEC_R15_LDAMX_ENABLE | RK610_CODEC_R15_RDAMX_ENABLE)
/* 0xc1: matches the vendor driver's literal USE_LPF value; not every
 * bit in this byte is named in the vendor header, so it is kept as a
 * verbatim constant rather than a partially-reconstructed OR chain.
 */
#define RK610_CODEC_R15_LPF_ROUTE	0xc1

/* R1A: MICBIAS 0.9x scale + slowest VMID ramp (pop-free startup with
 * cap-coupled output) - matches the vendor driver's literal value
 * exactly ("With Cap Output, VMID ramp up slow").
 */
#define RK610_CODEC_R1A_INIT		0x14

/* R1D/R1E: vendor literal init values (VMID/bias power-up sequence) */
#define RK610_CODEC_R1D_INIT		0x30
#define RK610_CODEC_R1E_INIT		0x40
/* R1D/R1E when entering standby/off: everything powered down */
#define RK610_CODEC_R1D_OFF		0xff
#define RK610_CODEC_R1E_OFF		0xff
#define RK610_CODEC_R1F_OFF		0xff

/* R1C */
#define RK610_CODEC_R1C_DEM_ENABLE	BIT(7)

/* R1F */
#define RK610_CODEC_R1F_PDMIXM_ENABLE	BIT(6)
#define RK610_CODEC_R1F_PDPAM_ENABLE	BIT(7)
/* base depop value, matches the vendor driver's literal 0x09 */
#define RK610_CODEC_R1F_DEPOP_BASE	0x09

struct rk610_codec {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *spk_gpio;
	unsigned int sysclk;

	bool use_mic_input;
	bool use_lpf_route;
	bool boot_depop;
	bool capless_output;
	u32 mic_volume;
};

/*
 * Custom regmap bus: this chip has no separate register-address byte.
 * Instead, the register number is folded into the low 5 bits of the
 * 7-bit I2C slave address for every single transaction, with the top
 * 2 bits fixed at the codec's own confirmed base address (0x60).
 *
 * I2C_SMBUS_BYTE (not I2C_SMBUS_BYTE_DATA) is deliberate - this chip
 * has no separate command/register byte at all. Every transaction is
 * a single raw byte, matching the vendor driver's own plain
 * i2c_master_send()/hw_write()-style single-byte transfer; the
 * register is entirely encoded in the address, not in a command byte.
 *
 * The computed per-register address is passed directly to
 * i2c_smbus_xfer() as its own parameter and is never written back
 * into the shared, persistent i2c_client's own .addr field. Mutating
 * that field would permanently move where the kernel considers this
 * device to live (visible in i2cdetect/sysfs as the device address
 * drifting to whatever register was last accessed), even though the
 * actual bus transactions themselves were always correctly addressed.
 */
static int rk610_codec_reg_write(void *context, unsigned int reg,
				  unsigned int val)
{
	struct i2c_client *i2c = context;
	u16 addr = (RK610_CODEC_BASE_ADDR & RK610_CODEC_ADDR_MASK) | reg;
	union i2c_smbus_data data;

	data.byte = val & 0xff;

	return i2c_smbus_xfer(i2c->adapter, addr, i2c->flags,
			       I2C_SMBUS_WRITE, data.byte, I2C_SMBUS_BYTE, NULL);
}

static int rk610_codec_reg_read(void *context, unsigned int reg,
				 unsigned int *val)
{
	struct i2c_client *i2c = context;
	u16 addr = (RK610_CODEC_BASE_ADDR & RK610_CODEC_ADDR_MASK) | reg;
	union i2c_smbus_data data;
	int ret;

	ret = i2c_smbus_xfer(i2c->adapter, addr, i2c->flags,
			      I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);
	if (ret < 0)
		return ret;

	*val = data.byte;
	return 0;
}

static const struct regmap_bus rk610_codec_regmap_bus = {
	.reg_write = rk610_codec_reg_write,
	.reg_read = rk610_codec_reg_read,
};

static const struct regmap_config rk610_codec_regmap_config = {
	.name = "rk610-codec",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = RK610_CODEC_MAX_REG,
	.cache_type = REGCACHE_MAPLE,
};

/* mclk -> sample rate coefficient table, from the vendor driver */
struct rk610_codec_coeff {
	u32 mclk;
	u32 rate;
	u8 sr;
	u8 usb;
};

static const struct rk610_codec_coeff rk610_codec_coeffs[] = {
	{ 12288000,  8000, 0x6,  0 }, { 11289600,  8000, 0x16, 0 },
	{ 12288000, 16000, 0xa,  0 }, { 11289600, 22050, 0x1a, 0 },
	{ 12288000, 32000, 0xc,  0 }, { 11289600, 44100, 0x10, 0 },
	{ 12288000, 48000, 0x0,  0 }, { 11289600, 88200, 0x1e, 0 },
	{ 12288000, 96000, 0xe,  0 }, { 12000000,  8000, 0x6,  1 },
	{ 12000000, 12000, 0x8,  1 }, { 12000000, 16000, 0xa,  1 },
	{ 12000000, 24000, 0x1c, 1 }, { 12000000, 32000, 0xa,  1 },
	{ 12000000, 44100, 0x11, 1 }, { 12000000, 48000, 0x0,  1 },
};

static int rk610_codec_get_coeff(unsigned int mclk, unsigned int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rk610_codec_coeffs); i++)
		if (rk610_codec_coeffs[i].mclk == mclk &&
		    rk610_codec_coeffs[i].rate == rate)
			return i;

	return -EINVAL;
}

/*
 * One-time analog front-end setup, run once at probe. Matches the
 * vendor driver's rk610_codec_reg_set(): VMID/bias power-up sequence,
 * input routing/gain, output mixer routing, default interface format,
 * digital filter mode, and anti-pop power sequencing. Without this,
 * the codec has no defined signal path at all - reset defaults alone
 * are not sufficient for correct operation, particularly for capture.
 */
static void rk610_codec_init_analog(struct rk610_codec *rk610)
{
	struct regmap *rm = rk610->regmap;

	regmap_write(rm, RK610_CODEC_R1D, RK610_CODEC_R1D_INIT);
	regmap_write(rm, RK610_CODEC_R1E, RK610_CODEC_R1E_INIT);

	regmap_write(rm, RK610_CODEC_R15, rk610->use_lpf_route ?
		     RK610_CODEC_R15_LPF_ROUTE : RK610_CODEC_R15_DAC_ROUTE);

	/* cap-coupled output, VMID ramp up slow (pop suppression) */
	regmap_write(rm, RK610_CODEC_R1A, RK610_CODEC_R1A_INIT);
	mdelay(10);

	/* line/mic input PGA gain, 0dB */
	regmap_write(rm, RK610_CODEC_R0C, 0x10);
	regmap_write(rm, RK610_CODEC_R0D, 0x10);

	if (rk610->use_mic_input) {
		u32 mic_vol = rk610->mic_volume;
		unsigned int r12 = 0x4c | RK610_CODEC_R12_MIC_INPUT;

		if (mic_vol > 0x07) {
			r12 |= RK610_CODEC_R12_MIC_BOOST_20DB;
			mic_vol -= 0x07;
		}
		regmap_write(rm, RK610_CODEC_R12, r12);
		regmap_write(rm, RK610_CODEC_R1C, RK610_CODEC_R1C_DEM_ENABLE);
		regmap_write(rm, RK610_CODEC_R0E, 0x10 | mic_vol);
	} else {
		regmap_write(rm, RK610_CODEC_R12, 0x4c);
		regmap_write(rm, RK610_CODEC_R0E, 0x10 | rk610->mic_volume);
	}

	/* PGA -> mixer routing disabled, 0dB gain */
	regmap_write(rm, RK610_CODEC_R13, RK610_CODEC_PGAMXVOL_0DB);
	regmap_write(rm, RK610_CODEC_R14, RK610_CODEC_PGAMXVOL_0DB);

	regmap_write(rm, RK610_CODEC_R04, RK610_CODEC_R04_MUTE);

	/* default sample-rate/clock and interface format; both are
	 * reconfigured properly by set_fmt/hw_params once the machine
	 * driver sets up the real stream, this just establishes a sane
	 * starting state.
	 */
	regmap_write(rm, RK610_CODEC_R0A, RK610_CODEC_R0A_CLK_NO_DIV);
	regmap_write(rm, RK610_CODEC_R09,
		     RK610_CODEC_R09_I2S_MODE | RK610_CODEC_R09_16BIT_MODE |
		     RK610_CODEC_R09_MASTER_MODE);
	regmap_write(rm, RK610_CODEC_R00,
		     RK610_CODEC_R00_HPF_ENABLE | RK610_CODEC_R00_DSM_ENABLE |
		     RK610_CODEC_R00_SCRAMBLE_EN | RK610_CODEC_R00_DITHER_EN |
		     RK610_CODEC_R00_BCLKDIV_4);

	regmap_write(rm, RK610_CODEC_R0B,
		     RK610_CODEC_R0B_DEC_ENABLE | RK610_CODEC_R0B_INT_ENABLE);

	if (rk610->boot_depop) {
		unsigned int r1f = RK610_CODEC_R1F_DEPOP_BASE |
				    RK610_CODEC_R1F_PDMIXM_ENABLE;

		if (!rk610->capless_output)
			r1f |= RK610_CODEC_R1F_PDPAM_ENABLE;

		regmap_write(rm, RK610_CODEC_R1F, r1f);
	}
}

static int rk610_codec_set_sysclk(struct snd_soc_dai *dai, int clk_id,
				   unsigned int freq, int dir)
{
	struct rk610_codec *rk610 = snd_soc_component_get_drvdata(dai->component);

	rk610->sysclk = freq;
	return 0;
}

static int rk610_codec_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct rk610_codec *rk610 = snd_soc_component_get_drvdata(dai->component);
	u16 iface = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBP_CFP:
		iface = 0x0040;
		break;
	case SND_SOC_DAIFMT_CBC_CFC:
		iface = 0x0000;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}

	regmap_write(rk610->regmap, RK610_CODEC_R09, iface);

	/* bring the speaker amp up now that the interface is configured */
	if (rk610->spk_gpio)
		gpiod_set_value_cansleep(rk610->spk_gpio, 1);

	return 0;
}

static int rk610_codec_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct rk610_codec *rk610 = snd_soc_component_get_drvdata(dai->component);
	unsigned int iface;
	int coeff;

	regmap_read(rk610->regmap, RK610_CODEC_R09, &iface);
	iface &= 0x1f3;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0008;
		break;
	default:
		return -EINVAL;
	}

	/* soft mute while we reprogram the interface, matches vendor behavior */
	regmap_write(rk610->regmap, RK610_CODEC_R04, RK610_CODEC_R04_MUTE);
	regmap_write(rk610->regmap, RK610_CODEC_R09, iface);

	coeff = rk610_codec_get_coeff(rk610->sysclk, params_rate(params));
	if (coeff >= 0) {
		const struct rk610_codec_coeff *c = &rk610_codec_coeffs[coeff];

		regmap_write(rk610->regmap, RK610_CODEC_R0A,
			     (c->sr << 1) | c->usb |
			     RK610_CODEC_R0A_CLK_NO_DIV |
			     RK610_CODEC_R0A_CLK_ENABLE);
	}

	return 0;
}

static int rk610_codec_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct rk610_codec *rk610 = snd_soc_component_get_drvdata(dai->component);

	if (mute) {
		regmap_write(rk610->regmap, RK610_CODEC_R17, 0xff);
		regmap_write(rk610->regmap, RK610_CODEC_R18, 0xff);
		regmap_write(rk610->regmap, RK610_CODEC_R19, 0xff);
		regmap_write(rk610->regmap, RK610_CODEC_R04, RK610_CODEC_R04_MUTE);
	} else {
		regmap_write(rk610->regmap, RK610_CODEC_R17, 0x00);
		regmap_write(rk610->regmap, RK610_CODEC_R18, 0x00);
		regmap_write(rk610->regmap, RK610_CODEC_R19, 0x00);
		regmap_write(rk610->regmap, RK610_CODEC_R04, 0x00);
	}

	return 0;
}

static const struct snd_soc_dai_ops rk610_codec_dai_ops = {
	.hw_params = rk610_codec_hw_params,
	.set_fmt = rk610_codec_set_fmt,
	.set_sysclk = rk610_codec_set_sysclk,
	.mute_stream = rk610_codec_mute_stream,
};

#define RK610_CODEC_RATES	SNDRV_PCM_RATE_8000_96000
#define RK610_CODEC_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_S20_3LE | \
				 SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_driver rk610_codec_dai = {
	.name = "rk610-codec-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = RK610_CODEC_RATES,
		.formats = RK610_CODEC_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = RK610_CODEC_RATES,
		.formats = RK610_CODEC_FORMATS,
	},
	.ops = &rk610_codec_dai_ops,
	.symmetric_rate = 1,
};

static int rk610_codec_set_bias_level(struct snd_soc_component *component,
				       enum snd_soc_bias_level level)
{
	struct rk610_codec *rk610 = snd_soc_component_get_drvdata(component);

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		if (rk610->spk_gpio)
			gpiod_set_value_cansleep(rk610->spk_gpio, 0);
		regmap_write(rk610->regmap, RK610_CODEC_R0A, 0);
		regmap_write(rk610->regmap, RK610_CODEC_R1D, RK610_CODEC_R1D_OFF);
		regmap_write(rk610->regmap, RK610_CODEC_R1E, RK610_CODEC_R1E_OFF);
		regmap_write(rk610->regmap, RK610_CODEC_R1F, RK610_CODEC_R1F_OFF);
		break;
	}

	return 0;
}

static const struct snd_soc_component_driver rk610_codec_component_driver = {
	.set_bias_level = rk610_codec_set_bias_level,
	.use_pmdown_time = 1,
	.endianness = 1,
};

static int rk610_codec_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct rk610_codec *rk610;

	rk610 = devm_kzalloc(dev, sizeof(*rk610), GFP_KERNEL);
	if (!rk610)
		return -ENOMEM;

	rk610->dev = dev;
	i2c_set_clientdata(i2c, rk610);

	rk610->spk_gpio = devm_gpiod_get_optional(dev, "speaker-enable", GPIOD_OUT_LOW);
	if (IS_ERR(rk610->spk_gpio))
		return dev_err_probe(dev, PTR_ERR(rk610->spk_gpio),
				     "failed to get speaker-enable gpio\n");

	rk610->use_mic_input = device_property_read_bool(dev, "rockchip,use-mic-input");
	rk610->use_lpf_route = device_property_read_bool(dev, "rockchip,use-lpf-route");
	rk610->boot_depop = device_property_read_bool(dev, "rockchip,boot-depop");
	rk610->capless_output = device_property_read_bool(dev, "rockchip,capless-output");

	/* 0x07 matches the vendor driver's own compiled-in default (Volume_Input) */
	rk610->mic_volume = 0x07;
	device_property_read_u32(dev, "rockchip,mic-volume", &rk610->mic_volume);
	if (rk610->mic_volume > 0x1f)
		rk610->mic_volume = 0x1f;

	rk610->regmap = devm_regmap_init(dev, &rk610_codec_regmap_bus, i2c,
					  &rk610_codec_regmap_config);
	if (IS_ERR(rk610->regmap))
		return dev_err_probe(dev, PTR_ERR(rk610->regmap),
				     "failed to init regmap\n");

	rk610_codec_init_analog(rk610);

	return devm_snd_soc_register_component(dev, &rk610_codec_component_driver,
						&rk610_codec_dai, 1);
}

static const struct of_device_id rk610_codec_of_match[] = {
	{ .compatible = "rockchip,rk610-codec" },
	{}
};
MODULE_DEVICE_TABLE(of, rk610_codec_of_match);

static const struct i2c_device_id rk610_codec_i2c_id[] = {
	{ "rk610-codec", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, rk610_codec_i2c_id);

static struct i2c_driver rk610_codec_driver = {
	.driver = {
		.name = "rk610-codec",
		.of_match_table = rk610_codec_of_match,
	},
	.probe = rk610_codec_i2c_probe,
	.id_table = rk610_codec_i2c_id,
};
module_i2c_driver(rk610_codec_driver);

MODULE_DESCRIPTION("RK610 audio codec driver");
MODULE_LICENSE("GPL");
