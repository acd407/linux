// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2023 David Yang
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "sii9134.h"

struct histb_hdmi_map {
	void __iomem *base;
	unsigned int max_offset;
};

struct histb_hdmi_14_priv {
	void __iomem *base;

	struct device *dev;

	struct clk *clk;
	struct reset_control *rst;
	struct clk *clk_sii9134;
	struct reset_control *rst_sii9134;
	struct clk *clk_phy;
	struct reset_control *rst_phy;

	struct histb_hdmi_map map0;
	struct histb_hdmi_map map1;
	struct histb_hdmi_map map_phy;

	struct sii9134 sii9134;
};

/*
 * One config per bank: regmap debugfs is keyed on device+name.  Register
 * numbers are byte offsets into the window (SiI9134 register N sits at
 * base + 4 * N); the custom bus below performs the access width the wrapper
 * actually decodes.
 */
static const struct regmap_config histb_hdmi_14_bank0_regmap_config = {
	.reg_bits = 32,
	.val_bits = 8,
	.name = "bank0",
	.max_register = 0x3fc,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config histb_hdmi_14_bank1_regmap_config = {
	.reg_bits = 32,
	.val_bits = 8,
	.name = "bank1",
	.max_register = 0x3fc,
	.cache_type = REGCACHE_NONE,
};

/* on-chip HDMI PHY ("Taiyan", hi3798mv100): 32-bit regs at base + 0x1800 */
static const struct regmap_config histb_hdmi_14_phy_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.name = "phy",
	.max_register = 0x38,
	.cache_type = REGCACHE_NONE,
};

/*
 * The TX wrapper only answers full-width accesses: byte strobes are ignored,
 * so an 8-bit value needs a 32-bit read of the word and a 32-bit write whose
 * low byte carries the register value (exactly what the SDK's
 * DRV_ReadByte_8BA()/DRV_WriteByte_8BA() do).  regmap-mmio cannot express
 * that -- it demands reg_stride >= 4 for 32-bit values, and the core then
 * rejects every register number that is not a multiple of the stride.
 */
static int histb_hdmi_regmap_read(void *context, const void *reg, size_t reg_len,
				  void *val, size_t val_len)
{
	struct histb_hdmi_map *map = context;
	unsigned int offset = get_unaligned_le32(reg);
	u32 word;

	if (reg_len != 4 || (val_len != 1 && val_len != 4))
		return -EIO;

	if (offset > map->max_offset) {
		pr_warn_once("histb-hdmi: read out of window: off=%#x len=%zu\n",
			     offset, val_len);
		return -EINVAL;
	}

	word = readl(map->base + offset);
	if (val_len == 1)
		*(u8 *)val = word & 0xff;
	else
		put_unaligned_le32(word, val);

	return 0;
}

static int histb_hdmi_regmap_write(void *context, const void *data, size_t count)
{
	struct histb_hdmi_map *map = context;
	const u8 *buf = data;
	unsigned int offset = get_unaligned_le32(buf);
	size_t val_len = count - 4;
	u32 word;

	if (count < 5 || val_len > 4)
		return -EIO;

	if (offset > map->max_offset) {
		pr_warn_once("histb-hdmi: write out of window: off=%#x len=%zu\n",
			     offset, val_len);
		return -EINVAL;
	}

	word = val_len == 4 ? get_unaligned_le32(buf + 4) : buf[4];
	writel(word, map->base + offset);

	return 0;
}

static const struct regmap_bus histb_hdmi_regmap_bus = {
	.read = histb_hdmi_regmap_read,
	.write = histb_hdmi_regmap_write,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default = REGMAP_ENDIAN_NATIVE,
};

/*
 * The CRG clock gates for the HDMI TX (PERI_CRG67 bit0-5, PERI_CRG68
 * bit0) are not modelled as clocks yet; until they are, mirror the
 * SDK's SI_HW_ResetCtrl()/SI_HW_ResetPhy() behaviour and poke them
 * directly, otherwise the TX block answers every read with zero.
 */
#define HDMI_CRG_BASE		0xf8a22000
#define HDMI_CRG54		0x0d8	/* vo: bit0-6 cken, bit30 vou_srst_req */
#define HDMI_CRG67		0x10c	/* hdmitx ctrl: bit0-5 cken, bit8 bus_srst, bit9 srst */
#define HDMI_CRG68		0x110	/* hdmitx phy:  bit0 cken, bit4 srst */

static void histb_hdmi_14_crg_dump(struct device *dev, void __iomem *crg,
				   const char *when)
{
	u32 c67 = readl_relaxed(crg + HDMI_CRG67);
	u32 c68 = readl_relaxed(crg + HDMI_CRG68);

	dev_dbg(dev, "CRG %s: crg67=%08x (cken=%x srst=%x) crg68=%08x (cken=%x srst=%x) crg54=%08x\n",
		when, c67, c67 & 0x3f, (c67 >> 8) & 0x3, c68, c68 & 1, (c68 >> 4) & 1,
		readl_relaxed(crg + HDMI_CRG54));
}

static int histb_hdmi_14_runtime_suspend(struct device *dev)
{
	struct histb_hdmi_14_priv *priv = dev_get_drvdata(dev);

	/* NULL (absent/optional) clocks are tolerated by the clk core */
	clk_disable_unprepare(priv->clk_phy);
	clk_disable_unprepare(priv->clk_sii9134);
	clk_disable_unprepare(priv->clk);

	return 0;
}

static int histb_hdmi_14_runtime_resume(struct device *dev)
{
	struct histb_hdmi_14_priv *priv = dev_get_drvdata(dev);

	/*
	 * clk_prepare_enable(NULL) and reset_control_{assert,deassert}(NULL)
	 * are both no-ops, so absent optional clocks/resets are fine.
	 */
	return reset_control_assert(priv->rst_phy) ?:
	       reset_control_assert(priv->rst_sii9134) ?:
	       reset_control_assert(priv->rst) ?:
	       clk_prepare_enable(priv->clk) ?:
	       clk_prepare_enable(priv->clk_sii9134) ?:
	       clk_prepare_enable(priv->clk_phy) ?:
	       reset_control_deassert(priv->rst) ?:
	       reset_control_deassert(priv->rst_sii9134) ?:
	       reset_control_deassert(priv->rst_phy);
}

static const struct dev_pm_ops histb_hdmi_14_pm_ops = {
	.runtime_suspend = histb_hdmi_14_runtime_suspend,
	.runtime_resume = histb_hdmi_14_runtime_resume,
};

static void histb_hdmi_14_pm_cleanup(void *data)
{
	struct device *dev = data;
	struct histb_hdmi_14_priv *priv = dev_get_drvdata(dev);

	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);	/* balance the always-on reference */
	clk_disable_unprepare(priv->clk_phy);
	clk_disable_unprepare(priv->clk_sii9134);
	clk_disable_unprepare(priv->clk);
}

static int histb_hdmi_14_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_hdmi_14_priv *priv;
	void __iomem *crg;
	int irq;
	int ret;

	/* acquire resources */
	priv = devm_drm_bridge_alloc(dev, struct histb_hdmi_14_priv,
				     sii9134.bridge, &sii9134_bridge_funcs);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	/*
	 * The clock tree for HDMI/VO is not modelled yet (no CRG clock
	 * IDs, no clocks in DT), so all clocks are optional.  Missing
	 * clocks mean the gate is assumed always on; clk_prepare_enable()
	 * and clk_disable_unprepare() tolerate NULL.
	 */
	priv->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);
	priv->rst = devm_reset_control_get_optional(dev, NULL);
	if (IS_ERR(priv->rst))
		return PTR_ERR(priv->rst);

	priv->clk_sii9134 = devm_clk_get_optional(dev, "sii9134");
	if (IS_ERR(priv->clk_sii9134))
		return PTR_ERR(priv->clk_sii9134);
	/* DTS reset-names: "base" (idx 0, the unnamed one above), "bus", "phy" */
	priv->rst_sii9134 = devm_reset_control_get_optional(dev, "bus");
	if (IS_ERR(priv->rst_sii9134))
		return PTR_ERR(priv->rst_sii9134);

	priv->clk_phy = devm_clk_get_optional(dev, "phy");
	if (IS_ERR(priv->clk_phy))
		return PTR_ERR(priv->clk_phy);
	priv->rst_phy = devm_reset_control_get_optional(dev, "phy");
	if (IS_ERR(priv->rst_phy))
		return PTR_ERR(priv->rst_phy);

	crg = devm_ioremap(dev, HDMI_CRG_BASE, 0x200);
	if (crg)
		histb_hdmi_14_crg_dump(dev, crg, "before-resume");

	/*
	 * The SiI9134 INT output is level and storms while a sink holds
	 * HPD, so the driver polls instead of using interrupts (same as
	 * the SDK's production path).
	 */
	irq = -1;

	priv->dev = dev;
	platform_set_drvdata(pdev, priv);
	dev_set_drvdata(dev, priv);

	/* bring up device */
	ret = histb_hdmi_14_runtime_resume(dev);
	if (ret)
		return ret;

	/*
	 * Keep the device powered at all times: the bridge core accesses
	 * registers from HPD interrupts, detect and EDID reads that do
	 * not hold a runtime PM reference, and the downstream codec
	 * component likewise.  Runtime PM currently only covers system
	 * sleep; per-frame idle gating can come later.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get(dev);
	ret = devm_add_action_or_reset(dev, histb_hdmi_14_pm_cleanup, dev);
	if (ret)
		return ret;

	priv->map0.base = priv->base;
	priv->map0.max_offset = 0x3fc;
	priv->map1.base = priv->base + 0x400;
	priv->map1.max_offset = 0x3fc;
	priv->map_phy.base = priv->base + 0x1800;
	priv->map_phy.max_offset = 0x38;

	priv->sii9134.map = devm_regmap_init(dev, &histb_hdmi_regmap_bus,
					     &priv->map0,
					     &histb_hdmi_14_bank0_regmap_config);
	if (IS_ERR(priv->sii9134.map))
		return PTR_ERR(priv->sii9134.map);

	priv->sii9134.map1 = devm_regmap_init(dev, &histb_hdmi_regmap_bus,
					      &priv->map1,
					      &histb_hdmi_14_bank1_regmap_config);
	if (IS_ERR(priv->sii9134.map1))
		return PTR_ERR(priv->sii9134.map1);

	priv->sii9134.map_phy = devm_regmap_init(dev, &histb_hdmi_regmap_bus,
						 &priv->map_phy,
						 &histb_hdmi_14_phy_regmap_config);
	if (IS_ERR(priv->sii9134.map_phy))
		return PTR_ERR(priv->sii9134.map_phy);

	if (crg) {
		/*
		 * Release only: the state that actually scans out a
		 * picture leaves CRG67 = 0x3f and CRG68 = 0x01 (all five
		 * clock enables on, both resets released), and asserting
		 * hdmitx_ctrl_srst_req here takes the block down with it
		 * on this board.  Same values and 10 ms settling as the
		 * SDK's SI_HW_ResetCtrl()/SI_HW_ResetPhy().
		 */
		writel_relaxed(0x3f, crg + HDMI_CRG67);
		writel_relaxed(0x01, crg + HDMI_CRG68);
		usleep_range(10000, 12000);
		histb_hdmi_14_crg_dump(dev, crg, "after-cken");
	}

	priv->sii9134.dev = dev;
	return sii9134_probe(&priv->sii9134, irq);
}

static void histb_hdmi_14_remove(struct platform_device *pdev)
{
	struct histb_hdmi_14_priv *priv = platform_get_drvdata(pdev);

	/*
	 * Tear the bridge core down first; the always-on clock/resets
	 * and runtime PM are released afterwards by the devm action
	 * (histb_hdmi_14_pm_cleanup), then the devm-managed bridge.
	 */
	sii9134_remove(&priv->sii9134);
}

static const struct of_device_id histb_hdmi_14_of_match[] = {
	{ .compatible = "hisilicon,histb-hdmi-1-4", },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_hdmi_14_of_match);

static struct platform_driver histb_hdmi_14_driver = {
	.probe = histb_hdmi_14_probe,
	.remove = histb_hdmi_14_remove,
	.driver = {
		.name = "histb-hdmi-14",
		.of_match_table = histb_hdmi_14_of_match,
		.pm = &histb_hdmi_14_pm_ops,
	},
};

module_platform_driver(histb_hdmi_14_driver);

MODULE_DESCRIPTION("HiSilicon STB HDMI 1.4 Tx");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
