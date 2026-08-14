/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_MFD_RK610_H
#define __LINUX_MFD_RK610_H

#include <linux/regmap.h>

#define RK610_C_PLL_CON0		0x00
#define RK610_C_PLL_CON1		0x01
#define RK610_C_PLL_CON2		0x02
#define RK610_C_PLL_CON3		0x03
#define RK610_C_PLL_CON4		0x04
#define RK610_C_PLL_CON5		0x05
#define RK610_C_PLL_DISABLE_FRAC	BIT(0)
#define RK610_C_PLL_BYPASS_ENABLE	BIT(1)
#define RK610_C_PLL_POWER_ON		BIT(2)
#define RK610_C_PLL_LOCKED		BIT(7)

#define RK610_S_PLL_CON0		0x06
#define RK610_S_PLL_CON1		0x07
#define RK610_S_PLL_CON2		0x08

#define RK610_LVDS_CON0			0x09
#define RK610_LVDS_CON1			0x0a
#define RK610_LCD1_CON			0x0b
#define RK610_SCL_CON0			0x0c

#define RK610_TVE_CON			0x29
#define RK610_TVE_VDAC_R_BYPASS_EN	BIT(7)
#define RK610_TVE_CVBS_3CH_EN		BIT(6)

#define RK610_CCIR_RESET		0x2a
#define RK610_CLOCK_CON0		0x2b
#define RK610_CLOCK_CON1		0x2c
#define RK610_CODEC_CON			0x2d
#define RK610_I2C_CON			0x2e

#define RK610_MAX_REGISTER		RK610_I2C_CON

struct rk610 {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;

	struct gpio_desc *reset_gpio;
	struct clk *i2s_clk;
	struct clk *i2s_hclk;
	struct regulator *vcc25;
};

#endif /* __LINUX_MFD_RK610_H */
