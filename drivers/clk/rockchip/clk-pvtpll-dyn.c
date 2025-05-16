// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Author: Huseyin BIYIK <huseyinbiyik@hotmail.com>
 *
 */

#include <asm/div64.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/clk-provider.h>
#include <linux/iopoll.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/gcd.h>
#include <linux/clk/rockchip.h>
#include <linux/mfd/syscon.h>
#include "clk.h"

struct rockchip_clk_pvtpll {
	struct clk_hw hw;
	spinlock_t *lock;
	struct regmap *grf_regmap;
	struct rockchip_pvtpll_clock pvtclk;
	u8 ring_sel;
	u8 ring_max_len;
	u8 ring_min_len;
	unsigned long requested_rate;
	unsigned long parent_rate;
	struct rockchip_clk_provider *ctx;

#ifdef CONFIG_DEBUG_FS
    struct hlist_node   debug_node;
#endif
};

static int readreg(struct regmap *regmap, unsigned int addr, unsigned int shift,
		unsigned int mask, unsigned int *val)
{
	unsigned int regval;
	int err;
	pr_info("%s: reading %d[%d]\n", __func__, addr, shift);
	err = regmap_read(regmap, addr, &regval);
	if (err) {
		pr_err("%s: failed to read reg %d[%d]\n", __func__, addr, shift);
		return err;
	}
	*val = (regval >> shift) & mask;
	return 0;
}
;

static int writereg(struct regmap *regmap, unsigned int addr,
		unsigned int shift, unsigned int mask, unsigned int val)
{
	int err;

	val = (val << shift) | (mask << (shift + 16));
	err = regmap_write(regmap, addr, val);
	pr_info("%s: writing %d[%d]\n", __func__, addr, shift);
	if (err) {
		pr_err("%s: failed to write reg %d[%d]\n", __func__, addr, shift);
		return err;
	}
	return 0;
}
;

static int reg_pvtpll_isenabled(struct rockchip_clk_pvtpll *pvtpll,
		unsigned int *enabled)
{

	if (pvtpll->pvtclk.enable_mask
			&& readreg(pvtpll->grf_regmap, pvtpll->pvtclk.enable_addr,
					pvtpll->pvtclk.enable_shift, pvtpll->pvtclk.enable_mask,
					enabled)) {
		return -EIO;
	}

	if (enabled && pvtpll->pvtclk.start_mask
			&& readreg(pvtpll->grf_regmap, pvtpll->pvtclk.start_addr,
					pvtpll->pvtclk.start_shift, pvtpll->pvtclk.start_mask, enabled))
		return -EIO;

	return 0;
}
;

static int reg_enable_pvtpll(struct rockchip_clk_pvtpll *pvtpll,
		unsigned int value)
{
	unsigned int enabled;

	if (reg_pvtpll_isenabled(pvtpll, &enabled))
		return -EIO;

	if (enabled ^ value) {
		if (pvtpll->pvtclk.enable_mask
				&& writereg(pvtpll->grf_regmap, pvtpll->pvtclk.enable_addr,
						pvtpll->pvtclk.enable_shift, pvtpll->pvtclk.enable_mask,
						value))
			return -EIO;

		if (pvtpll->pvtclk.start_mask
				&& writereg(pvtpll->grf_regmap, pvtpll->pvtclk.start_addr,
						pvtpll->pvtclk.start_shift, pvtpll->pvtclk.start_mask, value))
			return -EIO;
	};

	return 0;
}
;

static int reg_select_ring(struct rockchip_clk_pvtpll *pvtpll, u8 value)
{
	if (pvtpll->pvtclk.ringsel_mask
			&& writereg(pvtpll->grf_regmap, pvtpll->pvtclk.ringsel_addr,
					pvtpll->pvtclk.ringsel_shift, pvtpll->pvtclk.ringsel_mask, value))
		return -EIO;

	return 0;
}
;

static int reg_set_ring_len(struct rockchip_clk_pvtpll *pvtpll, u8 value)
{
	return writereg(pvtpll->grf_regmap, pvtpll->pvtclk.ringlen_addr,
			pvtpll->pvtclk.ringlen_shift, pvtpll->pvtclk.ringlen_mask, value);
}
;

static int reg_get_ring_len(struct rockchip_clk_pvtpll *pvtpll,
		unsigned int *ringlen)
{
	return readreg(pvtpll->grf_regmap, pvtpll->pvtclk.ringlen_addr,
			pvtpll->pvtclk.ringlen_shift, pvtpll->pvtclk.ringlen_mask, ringlen);
}

static int reg_measure_pvtpll(struct rockchip_clk_pvtpll *pvtpll,
		unsigned long *rate)
{
	u32 count, count_reg, total_count;

	// sample up 2x the requested frequency in the total count status register width
	count = (pvtpll->pvtclk.total_mask * pvtpll->parent_rate)
			/ (pvtpll->requested_rate * 2);

	// sample registers are full 32bit registers without write mask
	if (regmap_read(pvtpll->grf_regmap, pvtpll->pvtclk.sample_addr, &count_reg))
		return -EIO;

	if (count != count_reg
			&& regmap_write(pvtpll->grf_regmap, pvtpll->pvtclk.sample_addr, count))
		return -EIO;

	// wait 2 times the measurement period so that the counters are populated
	// 2 * count * 10e6 / pvtpll->requested_rate
	msleep(1);

	if (readreg(pvtpll->grf_regmap, pvtpll->pvtclk.total_addr,
			pvtpll->pvtclk.total_shift, pvtpll->pvtclk.total_mask, &total_count))
		return -EIO;

	if (total_count == 0) {
		pr_err("%s: error measuring clock status counter: 0 \n", __func__);
		return -EIO;
	}

	*rate = (total_count * pvtpll->parent_rate) / count;
	return 0;
}
;

static unsigned long op_recalculate_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	unsigned long rate;

	struct rockchip_clk_pvtpll *pvtpll = container_of(hw,
			struct rockchip_clk_pvtpll, hw);

	if (reg_measure_pvtpll(pvtpll, &rate))
		return pvtpll->requested_rate;

	return rate;
}
;

static long op_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	struct rockchip_clk_pvtpll *pvtpll = container_of(hw,
			struct rockchip_clk_pvtpll, hw);
	pvtpll->requested_rate = rate;
	pvtpll->parent_rate = *parent_rate;
	return rate;
}
;

static void op_disable_pvtpll(struct clk_hw *hw)
{
	struct rockchip_clk_pvtpll *pvtpll = container_of(hw,
			struct rockchip_clk_pvtpll, hw);

	reg_enable_pvtpll(pvtpll, false);
}
;

static int op_enable_pvtpll(struct clk_hw *hw)
{
	struct rockchip_clk_pvtpll *pvtpll = container_of(hw,
			struct rockchip_clk_pvtpll, hw);

	if (!reg_enable_pvtpll(pvtpll, true) &&
			!reg_select_ring(pvtpll, pvtpll->ring_sel) &&
			!reg_set_ring_len(pvtpll, pvtpll->ring_max_len))
		return 0;
	return -EIO;
}

static int op_isenabled(struct clk_hw *hw)
{
	unsigned int enabled;

	struct rockchip_clk_pvtpll *pvtpll = container_of(hw,
			struct rockchip_clk_pvtpll, hw);

	if (reg_pvtpll_isenabled(pvtpll, &enabled))
		return -EIO;
	return enabled;
}

static int op_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	unsigned long cur_rate;
	unsigned int cur_len;
	int next_len = -ERANGE;

	struct rockchip_clk_pvtpll *pvtpll = container_of(hw,
			struct rockchip_clk_pvtpll, hw);
	//TODO: implement frequency tolerance, rather than matching the given freq.

	pvtpll->requested_rate = rate;
	pvtpll->parent_rate = parent_rate;

	while (1) {
		if (reg_measure_pvtpll(pvtpll, &cur_rate)
				&& reg_get_ring_len(pvtpll, &cur_len))
			return -EIO;

		if (next_len < 0)
			next_len = cur_len;

		if (cur_rate < rate && cur_len > pvtpll->ring_min_len) {
			next_len--;
		} else if (cur_rate > rate && cur_len < pvtpll->ring_max_len) {
			next_len++;
		}

		if (cur_rate == rate ||
				(cur_rate > rate && cur_len > next_len) ||
				(cur_rate < rate && cur_len < next_len)) {
			// accept frequency when it is magically is set
			// or first time when it is overshot when up-scalimg,
			// or first time when it is undershot when down-scalimg,
			return 0;
		} else if (cur_len != next_len) {
			// decrease the ring length when up-scaling
			// increase the ring length when down-scaling
			reg_set_ring_len(pvtpll, next_len);
		} else {
			// ring range limits are reached
			return -ERANGE;
		}

	}

	// we should never reach here
	return -EINVAL;
}
;

static const struct clk_ops pvtpll_clk_ops = {
		.recalc_rate = op_recalculate_rate,
		.round_rate = op_round_rate,
		.set_rate = op_set_rate,
		.enable = op_enable_pvtpll,
		.disable = op_disable_pvtpll,
		.is_enabled = op_isenabled,
};

struct clk* rockchip_clk_register_pvtpll(struct rockchip_clk_provider *ctx,
		struct rockchip_pvtpll_clock *pvtclk)
{
	struct clk_init_data init;
	struct regmap *grf;
	struct rockchip_clk_pvtpll *pvtpll = NULL;
	int err = 0;
	u32 pll_config[3];
	char pvtpll_name[20];
	char grf_dt_name[20];
	char pvtpll_dt_name[20];

	snprintf(grf_dt_name, sizeof(grf_dt_name), "rockchip,%s-grf", pvtclk->name);
	snprintf(pvtpll_dt_name, sizeof(pvtpll_dt_name), "rockchip,%s-pvtpll",
			pvtclk->name);
	snprintf(pvtpll_name, sizeof(pvtpll_name), "%s_pvtpll", pvtclk->name);

	grf = syscon_regmap_lookup_by_phandle(ctx->cru_node, grf_dt_name);
	if (IS_ERR(grf)) {
		pr_err("%s: failed to get %s regmap\n", __func__, grf_dt_name);
		return ERR_PTR(-EINVAL);
	}

	err = of_property_read_u32_array(ctx->cru_node, pvtpll_dt_name, pll_config, 3);
	if (err) {
		pr_err("%s: failed to get %s ring config %d\n", __func__,
				pvtpll_dt_name, err);
		return ERR_PTR(-EINVAL);
	}

	//TODO: kfree
	pvtpll = kzalloc(sizeof(*pvtpll), GFP_KERNEL);
	if (IS_ERR(pvtpll)) {
		pr_err("%s: error allocating pvtpll %s\n", __func__,
				pvtpll_name);
		err = -ENOMEM;
		goto error;
	}

	pvtpll->ctx = ctx;
	pvtpll->grf_regmap = grf;
	pvtpll->lock = &ctx->lock;
	pvtpll->ring_sel = pll_config[0];
	pvtpll->ring_min_len = pll_config[1];
	pvtpll->ring_max_len = pll_config[2];
	pvtpll->pvtclk = *pvtclk;
	pvtpll->hw.init = &init;

	init.name = pvtpll_name;
	init.flags = 0;
	init.parent_names = &pvtclk->parent_name;
	init.num_parents = 1;
	init.ops = &pvtpll_clk_ops;
	pvtpll->hw.init = &init;

	err = clk_hw_register(NULL, &pvtpll->hw);
	if (err) {
		pr_err("%s: failed to register pvtpll clock %s : %d\n",
				__func__, pvtclk->name, err);
		goto error;
	}

	return pvtpll->hw.clk;
	error:
	kfree(pvtpll);
	return ERR_PTR(err);
}
