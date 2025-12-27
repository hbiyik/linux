// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Hüseyin BIYIK
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/clk-provider.h>
#include <linux/pm_runtime.h>
#include <linux/bitfield.h>
#include "clk.h"
#include <linux/printk.h>

#define MHz			1000000

#define GRF_PVTPLL_CON0_L	0x00
#define GRF_START_FIRST		0
#define GRF_START_LAST		0
#define GRF_ENABLE_FIRST	1
#define GRF_ENABLE_LAST		1
#define GRF_RINGSEL_FIRST	8
#define GRF_RINGSEL_LAST	10

#define GRF_PVTPLL_CON0_H	0x04
#define GRF_RINGLEN_FIRST	0
#define GRF_RINGLEN_LAST	5

#define GRF_PVTPLL_CON1		0x08
#define GRF_COUNT_FIRST		0
#define GRF_COUNT_LAST		31

#define GRF_PVTPLL_STATUS	0x18
#define GRF_STATUS_FIRST	0
#define GRF_STATUS_LAST		13

struct rockchip_pvtpll_adaptive_clk {
	struct device *dev;
	struct regmap *regmap;
	struct clk_hw hw;
	struct clk *pvtpll_out;
	struct clk_bulk_data *clks;
	int num_clks;
	u32 min_rate;
	u32 max_rate;
	u32 ring_max_len;
	u32 ring_min_len;
	u32 ring;
	u32 enable[4];
	u32 start[4];
	u32 ringsel[4];
	u32 ringlen[4];
	u32 count[4];
	u32 status[4];
	u32 curlen;
	atomic_t isresumed;
};

struct rockchip_pvtpll_adaptive_otp_info {
	u16 min_freq;
	u16 max_freq;
	u8 volt;
	u8 length;
} __packed;

static void printcru(struct rockchip_pvtpll_adaptive_clk *pvtpll)
{
	void __iomem *base;

	base = ioremap(0xFD7C0000, SZ_4K);
	dev_info(pvtpll->dev, "%s: Gate 66: %d \n", __func__,
		 readl(base + 0x0908));
	dev_info(pvtpll->dev, "%s: Gate 67: %d \n", __func__,
		 readl(base + 0x090C));
	iounmap(base);
}

static void printpm(struct rockchip_pvtpll_adaptive_clk *pvtpll)
{
	void __iomem *base;

	base = ioremap(0xFD8D8000, SZ_4K);
	dev_info(pvtpll->dev, "%s: BISR_STS 4: %d \n", __func__,
		 readl(base + 0x280 + 4 * 4));
	iounmap(base);
}

static inline int
rockchip_pvtpll_adaptive_writereg(struct rockchip_pvtpll_adaptive_clk *pvtpll,
				  u32 *reg, u32 val)
{
	int ret;
	u32 regoffset = reg[0];
	u32 regfirst = reg[1];
	u32 reglast = reg[2];
	u32 writemaskoffset = reg[3];

	if (reglast < regfirst)
		return -EINVAL;

	if (writemaskoffset)
		val = (val << regfirst) |
		      (GENMASK(reglast, regfirst) << writemaskoffset);
	else if ((reglast - regfirst) != 31 || regfirst != 0)
		return -EINVAL;

	ret = regmap_write(pvtpll->regmap, regoffset, val);
	if (ret)
		return ret;

	return 0;
}

static inline int
rockchip_pvtpll_adaptive_readreg(struct rockchip_pvtpll_adaptive_clk *pvtpll,
				 u32 *reg, u32 *val)
{
	unsigned int regval;
	int ret;
	u32 regoffset = reg[0];
	u32 regfirst = reg[1];
	u32 reglast = reg[2];

	if (reglast < regfirst)
		return -EINVAL;

	ret = regmap_read(pvtpll->regmap, regoffset, &regval);
	if (ret)
		return ret;

	*val = (regval & GENMASK(reglast, regfirst)) >> regfirst;
	return 0;
}

static inline int
rockchip_pvtpll_adaptive_measure(struct rockchip_pvtpll_adaptive_clk *pvtpll,
				 u32 *rate)
{
	int ret;
	u32 val;

	ret = rockchip_pvtpll_adaptive_readreg(pvtpll, pvtpll->status, &val);
	if (ret)
		return ret;

	// delay 1us: 24/24Mhz seconds
	udelay(2);

	*rate = val * MHz;
	return 0;
}

static int rockchip_pvtpll_adaptive_pm_resume(struct device *dev)
{
	struct rockchip_pvtpll_adaptive_clk *pvtpll = dev_get_drvdata(dev);
	int ret;

	ret = clk_bulk_prepare_enable(pvtpll->num_clks, pvtpll->clks);
	if (ret)
		return ret;

	ret = rockchip_pvtpll_adaptive_writereg(pvtpll, pvtpll->ringsel,
						pvtpll->ring);
	if (ret)
		return ret;

	ret = rockchip_pvtpll_adaptive_writereg(pvtpll, pvtpll->ringlen,
						pvtpll->curlen);
	if (ret)
		return ret;

	ret = rockchip_pvtpll_adaptive_writereg(pvtpll, pvtpll->enable, 1);
	if (ret)
		return ret;

	ret = rockchip_pvtpll_adaptive_writereg(pvtpll, pvtpll->start, 1);
	if (ret)
		return ret;

	atomic_set(&pvtpll->isresumed, 1);
	return 0;
}

static int rockchip_pvtpll_adaptive_pm_suspend(struct device *dev)
{
	struct rockchip_pvtpll_adaptive_clk *pvtpll = dev_get_drvdata(dev);
	int ret;

	if (atomic_read(&pvtpll->isresumed) == 1) {
		ret = rockchip_pvtpll_adaptive_writereg(pvtpll, pvtpll->start,
							0);
		if (ret)
			return ret;

		ret = rockchip_pvtpll_adaptive_writereg(pvtpll, pvtpll->enable,
							0);
		if (ret)
			return ret;
		clk_bulk_disable_unprepare(pvtpll->num_clks, pvtpll->clks);
	}
	atomic_set(&pvtpll->isresumed, 0);
	return 0;
}

static int rockchip_pvtpll_adaptive_set_rate(struct clk_hw *hw,
					     unsigned long rate,
					     unsigned long parent_rate)
{
	struct rockchip_pvtpll_adaptive_clk *pvtpll;
	u32 cur_rate;
	u32 prev_rate;
	u32 cur_len;
	bool upscaling;
	int next_len = -1;
	int ret = 0;

	pvtpll = container_of(hw, struct rockchip_pvtpll_adaptive_clk, hw);

	ret = pm_runtime_resume_and_get(pvtpll->dev);
	if (ret)
		return ret;

	for (int i = 0; i < 255; i++) {
		ret = rockchip_pvtpll_adaptive_measure(pvtpll, &cur_rate);
		if (ret)
			goto exit;

		ret = rockchip_pvtpll_adaptive_readreg(pvtpll, pvtpll->ringlen,
						       &cur_len);
		if (ret)
			goto exit;

		/* init next length, prev_rate and rate direction */
		if (next_len < 0){
			next_len = cur_len;
			prev_rate = cur_rate;
			upscaling = rate > cur_rate ? true : false;
		}

		/* Adjust next length based on current rate */
		if (cur_rate < rate && cur_len > pvtpll->ring_min_len)
			next_len--;
		else if (cur_rate > rate && cur_len < pvtpll->ring_max_len)
			next_len++;

		/* Clamp to allowed range */
		if (next_len < pvtpll->ring_min_len)
			next_len = pvtpll->ring_min_len;
		if (next_len > pvtpll->ring_max_len)
			next_len = pvtpll->ring_max_len;

		dev_info(
			pvtpll->dev,
			"%s: curlen: %d, nextlen: %d: currate: %d, setrate: %ld\n",
			__func__, cur_len, next_len, cur_rate, rate);

		/* Use rate if, target rate is achieved or limits are reached*/
		if (cur_rate == rate || cur_len == next_len) {
			ret = 0;
			goto exit;
		}

		/*
		undershot when ring length trying to reduce to increase freq (next_len < cur_len)
		during downscaling or overshot when ring length trying to increase 
		to reduce the freq (next_len > cur_len) during up scaling.
		In this case next_len = prev_len, since algo will try to fit back
		in to the target rate
		*/
		if (!(upscaling ^ (next_len > cur_len))) {
			/*
			it is not possible to fit into the target perfectly
			check if the previous clock was closer to the target,
			if so revert to previous ring length
			*/
			pvtpll->curlen = cur_len;
			if (abs(rate - prev_rate) < abs(rate - cur_rate)) {
				pvtpll->curlen = next_len;
				ret = rockchip_pvtpll_adaptive_writereg(
					pvtpll, pvtpll->ringlen, next_len);
				if (ret)
					goto exit;
				dev_info(
					pvtpll->dev,
					"%s: rolled back len: %d, currate: %d, setrate: %ld\n",
					__func__, next_len, prev_rate, rate);
			}
			ret = 0;
			goto exit;
		}

		prev_rate = cur_rate;
		/* set the next ring length */
		ret = rockchip_pvtpll_adaptive_writereg(pvtpll, pvtpll->ringlen,
							next_len);
		if (ret)
			goto exit;
	}

	ret = -ETIMEDOUT;

exit:
	pm_runtime_mark_last_busy(pvtpll->dev);
	pm_runtime_put_autosuspend(pvtpll->dev);
	return ret;
}

static unsigned long
rockchip_pvtpll_adaptive_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct rockchip_pvtpll_adaptive_clk *pvtpll;
	pvtpll = container_of(hw, struct rockchip_pvtpll_adaptive_clk, hw);
	u32 rate;
	int ret;

	ret = pm_runtime_resume_and_get(pvtpll->dev);
	if(ret)
		return ret;

	ret = rockchip_pvtpll_adaptive_measure(pvtpll, &rate);

	pm_runtime_mark_last_busy(pvtpll->dev);
	pm_runtime_put_autosuspend(pvtpll->dev);

	if (ret)
		return 0;
	return rate;
}

static long rockchip_pvtpll_adaptive_round_rate(struct clk_hw *hw,
						unsigned long rate,
						unsigned long *prate)
{
	struct rockchip_pvtpll_adaptive_clk *pvtpll;
	pvtpll = container_of(hw, struct rockchip_pvtpll_adaptive_clk, hw);

	if (pvtpll->max_rate && rate > pvtpll->max_rate)
		rate = pvtpll->max_rate;

	if (pvtpll->min_rate && rate < pvtpll->min_rate)
		rate = pvtpll->min_rate;

	dev_info(pvtpll->dev, "%s: rounded %ld\n", __func__, rate);
	return rate;
}

static int rockchip_pvtpll_adaptive_enable(struct clk_hw *hw)
{
	struct rockchip_pvtpll_adaptive_clk *pvtpll;

	pvtpll = container_of(hw, struct rockchip_pvtpll_adaptive_clk, hw);

	if (!IS_ENABLED(CONFIG_PM))
		return rockchip_pvtpll_adaptive_pm_resume(pvtpll->dev);

	return 0;
}

static void rockchip_pvtpll_adaptive_disable(struct clk_hw *hw)
{
	struct rockchip_pvtpll_adaptive_clk *pvtpll;

	pvtpll = container_of(hw, struct rockchip_pvtpll_adaptive_clk, hw);
	if (!IS_ENABLED(CONFIG_PM))
		rockchip_pvtpll_adaptive_pm_suspend(pvtpll->dev);
}

static const struct clk_ops rockchip_pvtpll_adaptive_clk_ops = {
	.enable = rockchip_pvtpll_adaptive_enable,
	.disable = rockchip_pvtpll_adaptive_disable,
	.recalc_rate = rockchip_pvtpll_adaptive_recalc_rate,
	.round_rate = rockchip_pvtpll_adaptive_round_rate,
	.set_rate = rockchip_pvtpll_adaptive_set_rate,
};

static int
rockchip_pvtpll_adaptive_register(struct device *dev,
				  struct rockchip_pvtpll_adaptive_clk *pvtpll)
{
	struct clk_init_data init = {};

	init.parent_names = NULL;
	init.num_parents = 0;
	init.flags = CLK_GET_RATE_NOCACHE | CLK_VOLTAGE_FIRST;
	init.name = "pvtpll";
	init.ops = &rockchip_pvtpll_adaptive_clk_ops;

	pvtpll->hw.init = &init;

	of_property_read_string_index(dev->of_node, "clock-output-names", 0,
				      &init.name);

	pvtpll->pvtpll_out = devm_clk_register(dev, &pvtpll->hw);
	if (IS_ERR(pvtpll->pvtpll_out))
		return PTR_ERR(pvtpll->pvtpll_out);

	return of_clk_add_provider(dev->of_node, of_clk_src_simple_get,
				   pvtpll->pvtpll_out);
}

static int rockchip_pvtpll_adaptive_get_otp(
	struct device *dev, struct rockchip_pvtpll_adaptive_otp_info *opp_info)
{
	struct nvmem_cell *cell;
	void *buf;
	size_t len = 0;

	cell = nvmem_cell_get(dev, "opp-info");
	if (IS_ERR(cell))
		return PTR_ERR(cell);

	buf = nvmem_cell_read(cell, &len);
	if (IS_ERR(buf)) {
		nvmem_cell_put(cell);
		return PTR_ERR(buf);
	}

	if (len != sizeof(*opp_info)) {
		kfree(buf);
		nvmem_cell_put(cell);
		return -EINVAL;
	}

	memcpy(opp_info, buf, sizeof(*opp_info));
	kfree(buf);
	nvmem_cell_put(cell);
	return 0;
}

static int rockchip_pvtpll_adaptive_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct rockchip_pvtpll_adaptive_clk *pvtpll;
	struct rockchip_pvtpll_adaptive_otp_info opp_info;
	int ret = 0;

	pvtpll = devm_kzalloc(dev, sizeof(*pvtpll), GFP_KERNEL);
	if (!pvtpll){
		dev_err(dev, "%s: Can not alloc pvtpll\n", __func__);
		return -ENOMEM;
	}

	pvtpll->dev = dev;

	pvtpll->regmap = device_node_to_regmap(np);
	if (IS_ERR(pvtpll->regmap))
		return PTR_ERR(pvtpll->regmap);

	if (of_property_read_u32(np, "ring-max-len", &pvtpll->ring_max_len)) {
		dev_err(dev, "%s: ring-max-len is not provided\n", __func__);
		return -EINVAL;
	}

	if (of_property_read_u32(pdev->dev.of_node, "ring-min-len",
				 &pvtpll->ring_min_len)) {
		dev_err(dev, "%s: ring-min-len is not provided\n", __func__);
		return -EINVAL;
	}

	if (!pvtpll->ring_max_len ||
	    pvtpll->ring_min_len > pvtpll->ring_max_len) {
		dev_err(dev, "%s: invalid ring configuration: min %d, max: %d\n",
			__func__, pvtpll->ring_min_len, pvtpll->ring_max_len);
		return -EINVAL;
	}

	if (of_property_read_u32(np, "ring", &pvtpll->ring)){
		dev_err(dev, "%s: ring is not provided\n", __func__);
		return -EINVAL;
	}

	if (rockchip_pvtpll_adaptive_get_otp(dev, &opp_info))
    	memset(&opp_info, 0, sizeof(opp_info));

	if (of_property_read_u32(np, "max-rate", &pvtpll->max_rate))
		pvtpll->max_rate = opp_info.max_freq * MHz;

	if (of_property_read_u32(np, "min-rate", &pvtpll->min_rate))
		pvtpll->min_rate = opp_info.min_freq * MHz;

	if (of_property_read_u32_array(np, "reg-enable", pvtpll->enable,
				      4))
		memcpy(pvtpll->enable, (u32[]){ GRF_PVTPLL_CON0_L,
								 	   GRF_ENABLE_FIRST,
			       					   GRF_ENABLE_LAST,
									   16 }, sizeof(pvtpll->enable));

	if (of_property_read_u32_array(np, "reg-start",
				      	pvtpll->start, 4))
		memcpy(pvtpll->start, (u32[]){ GRF_PVTPLL_CON0_L,
									  GRF_START_FIRST,
			       					  GRF_START_LAST,
									  16 }, sizeof(pvtpll->start));

	if (of_property_read_u32_array(np, "reg-ringsel",
				      	pvtpll->ringsel, 4))
		memcpy(pvtpll->ringsel, (u32[]){ GRF_PVTPLL_CON0_L,
									    GRF_RINGSEL_FIRST,
			       					    GRF_RINGSEL_LAST,
									    16 }, sizeof(pvtpll->ringsel));

	if (of_property_read_u32_array(np, "reg-ringlen",
				      pvtpll->ringlen, 4))
		memcpy(pvtpll->ringlen, (u32[]){ GRF_PVTPLL_CON0_H,
									    GRF_RINGLEN_FIRST,
			       					    GRF_RINGLEN_LAST,
									    16 }, sizeof(pvtpll->ringlen));

	if (of_property_read_u32_array(np, "reg-count",
				      pvtpll->count, 4))
		memcpy(pvtpll->count, (u32[]){ GRF_PVTPLL_CON1,
									  GRF_COUNT_FIRST,
			       					  GRF_COUNT_LAST,
									  0 }, sizeof(pvtpll->count));

	if (of_property_read_u32_array(np, "reg-status",
				      	pvtpll->status, 4))
		memcpy(pvtpll->status, (u32[]){ GRF_PVTPLL_STATUS,
									   GRF_STATUS_FIRST,
			       					   GRF_STATUS_LAST,
									   0 }, sizeof(pvtpll->status));

	pvtpll->num_clks = devm_clk_bulk_get_all(dev, &pvtpll->clks);
	if (pvtpll->num_clks < 0)
    	return pvtpll->num_clks;

	platform_set_drvdata(pdev, pvtpll);

	ret = pm_runtime_set_active(dev);
	if (ret) {
		dev_err(dev, "failed to activate pm runtime: %d\n", ret);
		return ret;
	}

	ret = devm_pm_runtime_enable(dev);
	if (ret) {
		dev_err(dev, "failed to enable pm runtime: %d\n", ret);
		return ret;
	}

	pm_runtime_set_autosuspend_delay(dev, 50);
	pm_runtime_use_autosuspend(dev);

	pvtpll->curlen = pvtpll->ring_max_len;

	ret = rockchip_pvtpll_adaptive_register(dev, pvtpll);
	if (ret) {
		dev_err(dev, "failed to register clock: %d\n", ret);
		return ret;
	}

	dev_info(dev, "%s: probed\n", __func__);
	return 0;
}

static int rockchip_pvtpll_adaptive_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);

	return 0;
}

static const struct dev_pm_ops rockchip_pvtpll_adaptive_pm_ops = {
	SET_RUNTIME_PM_OPS(rockchip_pvtpll_adaptive_pm_suspend,
			   rockchip_pvtpll_adaptive_pm_resume,
			   NULL)
};

static const struct of_device_id rockchip_pvtpll_adaptive_match[] = {
	{
		.compatible = "rockchip,pvtpll-adaptive",
	},
	{}
};

MODULE_DEVICE_TABLE(of, rockchip_pvtpll_adaptive_match);

static struct platform_driver rockchip_pvtpll_adaptive_driver = {
	.driver = {
		.name = "rockchip-pvtpll-adaptive-clock",
		.of_match_table	= rockchip_pvtpll_adaptive_match,
		.pm = &rockchip_pvtpll_adaptive_pm_ops,
	},
	.probe	= rockchip_pvtpll_adaptive_probe,
	.remove	= rockchip_pvtpll_adaptive_remove,
};

module_platform_driver(rockchip_pvtpll_adaptive_driver);

MODULE_DESCRIPTION("Adaptive PVTPLL clock provider driver");
MODULE_LICENSE("GPL");
