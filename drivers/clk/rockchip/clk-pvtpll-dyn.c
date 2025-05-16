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


struct pvtpll_reg_variant0 {
	struct reg_field cfg_start;
	struct reg_field cfg_osc_en;
	struct reg_field cfg_out_polarity;
	struct reg_field cfg_osc_ring_sel;
	struct reg_field cfg_clk_div_ref;
	struct reg_field cfg_clk_div_osc;
	struct reg_field cfg_bypass;
	struct reg_field cfg_ring_len;
	struct reg_field cfg_cal_cnt;
	struct reg_field cfg_osc_cnt;
	struct reg_field cfg_osc_cnt_total;
};

struct rockchip_clk_pvtpll {
	struct clk_hw hw;
	spinlock_t *lock;
	struct regmap *grf_regmap;
	struct regmap_field *enable_reg;
	struct regmap_field *start_reg;
	struct regmap_field *ringsel_reg;
	struct regmap_field *ringlen_reg;
	struct regmap_field *count_reg;
	struct regmap_field *count_total_reg;
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


static struct pvtpll_reg_variant0 pvtpll_reg_variant0_layout = {
		.cfg_start = REG_FIELD(0x0, 0, 0),
		.cfg_osc_en = REG_FIELD(0x0, 1, 1),
		.cfg_out_polarity = REG_FIELD(0x0, 2, 2),
		.cfg_osc_ring_sel = REG_FIELD(0x0, 8, 10),
		.cfg_clk_div_ref = REG_FIELD(0x0, 11, 12),
		.cfg_clk_div_osc = REG_FIELD(0x0, 13, 14),
		.cfg_bypass = REG_FIELD(0x0, 15, 15),
		.cfg_ring_len = REG_FIELD(0x4, 0, 5),
		.cfg_cal_cnt = REG_FIELD(0x8, 0, 31),
		.cfg_osc_cnt = REG_FIELD(0x14, 0, 14),
		.cfg_osc_cnt_total = REG_FIELD(0x18, 0, 14),
};

// since regmap has no way of updating fields which has a write mask on higher 16bits
// we are doing a macho write here, ideally similar mechanism should be in regmap.c

static void regmap_field_write_highmask(struct regmap *map, struct regmap_field * field, u32 val)
{
	val = (val << field->shift) | (field->mask << (field->shift + 16));
	regmap_write(map, field->reg, val);
};

static bool reg_pvtpll_isenabled(struct rockchip_clk_pvtpll *pvtpll)
{
	u32 enabled = 1;

	if (pvtpll->enable_reg)
		enabled = regmap_field_test_bits(pvtpll->enable_reg, 1);
	if (enabled && pvtpll->start_reg)
		enabled = regmap_field_test_bits(pvtpll->start_reg, 1);

	return (bool)enabled;
};

static void reg_enable_pvtpll(struct rockchip_clk_pvtpll *pvtpll, bool value)
{
	if (!reg_pvtpll_isenabled(pvtpll)){
		if (pvtpll->enable_reg)
			regmap_field_write_highmask(pvtpll->grf_regmap, pvtpll->enable_reg, value);
		if (pvtpll->start_reg)
			regmap_field_write_highmask(pvtpll->grf_regmap, pvtpll->start_reg, value);	}
};

static void reg_select_ring(struct rockchip_clk_pvtpll *pvtpll, u8 value)
{
	if (pvtpll->ringsel_reg)
		regmap_field_write_highmask(pvtpll->grf_regmap, pvtpll->ringsel_reg, value);
};

static void reg_set_ring_len(struct rockchip_clk_pvtpll *pvtpll, u8 value)
{
	regmap_field_write_highmask(pvtpll->grf_regmap, pvtpll->ringlen_reg, value);
};

static int reg_measure_pvtpll(struct rockchip_clk_pvtpll *pvtpll, unsigned long *rate)
{
	int ret;
	u32 count, count_reg, total_count;

	// sample up 2x the requested frequency in the total count status register width
	count = (pvtpll->count_total_reg->mask * pvtpll->parent_rate) / (pvtpll->requested_rate * 2);
	ret = regmap_field_read(pvtpll->count_reg, &count_reg);
	if(ret){
		pr_err("%s: error getting clock config counter: %d \n", __func__, ret);
		return ret;
	}

	if (count != count_reg)
		regmap_field_write_highmask(pvtpll->grf_regmap, pvtpll->count_reg, count);

	// wait 2 times the measurement period so that the counters are populated
	// 2 * count * 10e6 / pvtpll->requested_rate
	msleep(1);

	ret = regmap_field_read(pvtpll->count_total_reg, &total_count);
	if(ret){
		pr_err("%s: error getting clock status counter: %d \n", __func__, ret);
		return ret;
	}
	if (total_count == 0){
		pr_err("%s: error measuring clock status counter: 0 \n", __func__);
		return -EINVAL;
	}

	*rate = (total_count * pvtpll->parent_rate) / count;
	return 0;
};

static unsigned long op_recalculate_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	unsigned long rate;
	int ret;

	struct rockchip_clk_pvtpll *pvtpll = container_of(hw,
			struct rockchip_clk_pvtpll, hw);

	ret = reg_measure_pvtpll(pvtpll, &rate);
	if (ret)
		return pvtpll->requested_rate;

	return rate;
};

static long op_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	struct rockchip_clk_pvtpll *pvtpll = container_of(hw,
			struct rockchip_clk_pvtpll, hw);
	pvtpll->requested_rate = rate;
	pvtpll->parent_rate = *parent_rate;
	return rate;
};

static void op_disable_pvtpll(struct clk_hw *hw)
{
	struct rockchip_clk_pvtpll *pvtpll = container_of(hw,
			struct rockchip_clk_pvtpll, hw);

	reg_enable_pvtpll(pvtpll, false);
};

static int op_enable_pvtpll(struct clk_hw *hw)
{
	struct rockchip_clk_pvtpll *pvtpll = container_of(hw,
			struct rockchip_clk_pvtpll, hw);

	reg_enable_pvtpll(pvtpll, true);
	reg_select_ring(pvtpll, pvtpll->ring_sel);
	reg_set_ring_len(pvtpll, pvtpll->ring_max_len);
	return 0;
}

static int op_isenabled(struct clk_hw *hw)
{

	struct rockchip_clk_pvtpll *pvtpll = container_of(hw,
			struct rockchip_clk_pvtpll, hw);

	return reg_pvtpll_isenabled(pvtpll);
}

static int op_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	unsigned long cur_rate;
	unsigned int cur_len;
	int ret;
	int next_len = -ERANGE;

	struct rockchip_clk_pvtpll *pvtpll = container_of(hw,
				struct rockchip_clk_pvtpll, hw);
	//TODO: implement frequency tolerance, rather than matching the given freq.

	pvtpll->requested_rate = rate;
	pvtpll->parent_rate = parent_rate;

	while (1) {
		ret = reg_measure_pvtpll(pvtpll, &cur_rate);
		if(ret)
			return ret;
		ret = regmap_field_read(pvtpll->ringlen_reg, &cur_len);
		if(ret){
			pr_err("%s: error getting ringlen: %d \n", __func__, ret);
			return ret;
		}

		if (next_len < 0)
			next_len = cur_len;

		if (cur_rate < rate && cur_len > pvtpll->ring_min_len){
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

static int init_regmap_variant0(struct device *dev,
		struct rockchip_clk_pvtpll *pvtpll)
{
	pvtpll->enable_reg = devm_regmap_field_alloc(dev, pvtpll->grf_regmap,
			pvtpll_reg_variant0_layout.cfg_osc_en);
	if (IS_ERR(pvtpll->enable_reg))
		return PTR_ERR(pvtpll->enable_reg);

	pvtpll->start_reg = devm_regmap_field_alloc(dev, pvtpll->grf_regmap,
			pvtpll_reg_variant0_layout.cfg_start);
	if (IS_ERR(pvtpll->start_reg))
		return PTR_ERR(pvtpll->start_reg);

	pvtpll->ringlen_reg = devm_regmap_field_alloc(dev, pvtpll->grf_regmap,
			pvtpll_reg_variant0_layout.cfg_ring_len);
	if (IS_ERR(pvtpll->ringlen_reg))
		return PTR_ERR(pvtpll->ringlen_reg);

	pvtpll->ringsel_reg = devm_regmap_field_alloc(dev, pvtpll->grf_regmap,
			pvtpll_reg_variant0_layout.cfg_osc_ring_sel);
	if (IS_ERR(pvtpll->ringsel_reg))
		return PTR_ERR(pvtpll->enable_reg);

	pvtpll->count_total_reg = devm_regmap_field_alloc(dev,
			pvtpll->grf_regmap,
			pvtpll_reg_variant0_layout.cfg_osc_cnt_total);
	if (IS_ERR(pvtpll->count_total_reg))
		return PTR_ERR(pvtpll->count_total_reg);

	return 0;
}

struct clk* rockchip_clk_register_pvtpll(struct device *dev,
		struct rockchip_clk_provider *ctx,
		const char *name, const char *parent_name, int variant)
{
	struct clk_init_data init;
	struct regmap *grf;
	struct device_node *np;
	struct rockchip_clk_pvtpll *pvtpll = NULL;
	int err = 0;
	u32 pll_config[3];
	char pvtpll_name[20];
	char grf_dt_name[20];
	char pvtpll_dt_name[20];

	if (variant != 0) {
		pr_err("%s: pvtpll variant %d is not supported\n", __func__,
				variant);
		return ERR_PTR(-EINVAL);
	}

	snprintf(grf_dt_name, sizeof(grf_dt_name), "rockchip,%s-grf", name);
	snprintf(pvtpll_dt_name, sizeof(pvtpll_dt_name), "rockchip,pvtpll-%s",
			name);
	snprintf(pvtpll_name, sizeof(pvtpll_name), "pvtpll_%s", name);

	np = of_parse_phandle(ctx->cru_node, grf_dt_name, 0);
	if (!np) {
		pr_err("%s: failed to get %s\n", __func__, grf_dt_name);
		return ERR_PTR(-EINVAL);
	}

	grf = syscon_node_to_regmap(np);
	if (IS_ERR(grf)) {
		pr_err("%s: failed to get %s regmap\n", __func__, grf_dt_name);
		return ERR_PTR(-EINVAL);
	}

	if (!of_property_read_u32_array(np, pvtpll_dt_name, pll_config, 3)) {
		pr_err("%s: failed to get %s ring config\n", __func__,
				pvtpll_dt_name);
		return ERR_PTR(-EINVAL);
	}

	pvtpll = devm_kzalloc(dev, sizeof(*pvtpll), GFP_KERNEL);
	if (IS_ERR(pvtpll)) {
		pr_err("%s: error allocating pvtpll %s\n", __func__,
				pvtpll_name);
		return ERR_PTR(-ENOMEM);
	}

	pvtpll->ctx = ctx;
	pvtpll->grf_regmap = grf;
	pvtpll->lock = &ctx->lock;
	pvtpll->ring_sel = pll_config[0];
	pvtpll->ring_min_len = pll_config[1];
	pvtpll->ring_max_len = pll_config[2];
	pvtpll->hw.init = &init;
	err = init_regmap_variant0(dev, pvtpll);
	if (err) {
		pr_err("%s: Error initing regmap fields for %s : %d\n",
				__func__, name, err);
		return ERR_PTR(err);
	}

	init.name = pvtpll_name;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.ops = &pvtpll_clk_ops;
	pvtpll->hw.init = &init;

	err = clk_hw_register(NULL, &pvtpll->hw);
	if (err) {
		pr_err("%s: failed to register pvtpll clock %s : %d\n",
				__func__, name, err);
		return ERR_PTR(err);
	}

	return pvtpll->hw.clk;
}
