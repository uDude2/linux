/*
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Chao Xie <xiechao.mail@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/resource.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/platform_data/mv_usb.h>
#include <linux/usb/mv_usb2.h>

/* phy regs */
#define UTMI_REVISION		0x0
#define UTMI_CTRL		0x4
#define UTMI_PLL		0x8
#define UTMI_TX			0xc
#define UTMI_RX			0x10
#define UTMI_IVREF		0x14
#define UTMI_T0			0x18
#define UTMI_T1			0x1c
#define UTMI_T2			0x20
#define UTMI_T3			0x24
#define UTMI_T4			0x28
#define UTMI_T5			0x2c
#define UTMI_RESERVE		0x30
#define UTMI_USB_INT		0x34
#define UTMI_DBG_CTL		0x38
#define UTMI_OTG_ADDON		0x3c

/* For UTMICTRL Register */
#define UTMI_CTRL_USB_CLK_EN                    (1 << 31)
/* pxa168 */
#define UTMI_CTRL_SUSPEND_SET1                  (1 << 30)
#define UTMI_CTRL_SUSPEND_SET2                  (1 << 29)
#define UTMI_CTRL_RXBUF_PDWN                    (1 << 24)
#define UTMI_CTRL_TXBUF_PDWN                    (1 << 11)

#define UTMI_CTRL_INPKT_DELAY_SHIFT             30
#define UTMI_CTRL_INPKT_DELAY_SOF_SHIFT		28
#define UTMI_CTRL_PU_REF_SHIFT			20
#define UTMI_CTRL_ARC_PULLDN_SHIFT              12
#define UTMI_CTRL_PLL_PWR_UP_SHIFT              1
#define UTMI_CTRL_PWR_UP_SHIFT                  0

/* For UTMI_PLL Register */
#define UTMI_PLL_PLLCALI12_SHIFT		29
#define UTMI_PLL_PLLCALI12_MASK			(0x3 << 29)

#define UTMI_PLL_PLLVDD18_SHIFT			27
#define UTMI_PLL_PLLVDD18_MASK			(0x3 << 27)

#define UTMI_PLL_PLLVDD12_SHIFT			25
#define UTMI_PLL_PLLVDD12_MASK			(0x3 << 25)

#define UTMI_PLL_CLK_BLK_EN_SHIFT               24
#define CLK_BLK_EN                              (0x1 << 24)
#define PLL_READY                               (0x1 << 23)
#define KVCO_EXT                                (0x1 << 22)
#define VCOCAL_START                            (0x1 << 21)

#define UTMI_PLL_KVCO_SHIFT			15
#define UTMI_PLL_KVCO_MASK                      (0x7 << 15)

#define UTMI_PLL_ICP_SHIFT			12
#define UTMI_PLL_ICP_MASK                       (0x7 << 12)

#define UTMI_PLL_FBDIV_SHIFT                    4
#define UTMI_PLL_FBDIV_MASK                     (0xFF << 4)

#define UTMI_PLL_REFDIV_SHIFT                   0
#define UTMI_PLL_REFDIV_MASK                    (0xF << 0)

/* For UTMI_TX Register */
#define UTMI_TX_REG_EXT_FS_RCAL_SHIFT		27
#define UTMI_TX_REG_EXT_FS_RCAL_MASK		(0xf << 27)

#define UTMI_TX_REG_EXT_FS_RCAL_EN_SHIFT	26
#define UTMI_TX_REG_EXT_FS_RCAL_EN_MASK		(0x1 << 26)

#define UTMI_TX_TXVDD12_SHIFT                   22
#define UTMI_TX_TXVDD12_MASK                    (0x3 << 22)

#define UTMI_TX_CK60_PHSEL_SHIFT                17
#define UTMI_TX_CK60_PHSEL_MASK                 (0xf << 17)

#define UTMI_TX_IMPCAL_VTH_SHIFT                14
#define UTMI_TX_IMPCAL_VTH_MASK                 (0x7 << 14)

#define REG_RCAL_START                          (0x1 << 12)

#define UTMI_TX_LOW_VDD_EN_SHIFT                11

#define UTMI_TX_AMP_SHIFT			0
#define UTMI_TX_AMP_MASK			(0x7 << 0)

/* For UTMI_RX Register */
#define UTMI_REG_SQ_LENGTH_SHIFT                15
#define UTMI_REG_SQ_LENGTH_MASK                 (0x3 << 15)

#define UTMI_RX_SQ_THRESH_SHIFT                 4
#define UTMI_RX_SQ_THRESH_MASK                  (0xf << 4)

#define UTMI_OTG_ADDON_OTG_ON			(1 << 0)

enum mv_usb2_phy_type {
	PXA168_USB,
	PXA910_USB,
	MMP2_USB,
};

static struct mv_usb2_phy *the_phy;

struct mv_usb2_phy *mv_usb2_get_phy(void)
{
	return the_phy;
}
EXPORT_SYMBOL(mv_usb2_get_phy);

int mv_usb2_register_notifier(struct mv_usb2_phy *phy,
		struct notifier_block *nb)
{
	int ret;

	if (!phy)
		return -ENODEV;

	ret = atomic_notifier_chain_register(phy->extern_chip.head, nb);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL(mv_usb2_register_notifier);

int mv_usb2_unregister_notifier(struct mv_usb2_phy *phy,
		struct notifier_block *nb)
{
	int ret;

	if (!phy)
		return -ENODEV;

	ret = atomic_notifier_chain_unregister(phy->extern_chip.head, nb);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL(mv_usb2_unregister_notifier);

int mv_usb2_notify(struct mv_usb2_phy *phy, unsigned long val, void *v)
{
	int ret;

	if (!phy)
		return -ENODEV;

	ret = atomic_notifier_call_chain(phy->extern_chip.head, val, v);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL(mv_usb2_notify);

static unsigned int u2o_get(void __iomem *base, unsigned int offset)
{
	return readl_relaxed(base + offset);
}

static void u2o_set(void __iomem *base, unsigned int offset,
		unsigned int value)
{
	u32 reg;

	reg = readl_relaxed(base + offset);
	reg |= value;
	writel_relaxed(reg, base + offset);
	readl_relaxed(base + offset);
}

static void u2o_clear(void __iomem *base, unsigned int offset,
		unsigned int value)
{
	u32 reg;

	reg = readl_relaxed(base + offset);
	reg &= ~value;
	writel_relaxed(reg, base + offset);
	readl_relaxed(base + offset);
}

static void u2o_write(void __iomem *base, unsigned int offset,
		unsigned int value)
{
	writel_relaxed(value, base + offset);
	readl_relaxed(base + offset);
}

static int _usb_phy_init(struct mv_usb2_phy *mv_phy)
{
	struct platform_device *pdev = mv_phy->pdev;
	unsigned int loops = 0;
	void __iomem *base = mv_phy->base;

	dev_dbg(&pdev->dev, "phy init\n");

	/* Initialize the USB PHY power */
	if (mv_phy->type == PXA910_USB) {
		u2o_set(base, UTMI_CTRL, (1<<UTMI_CTRL_INPKT_DELAY_SOF_SHIFT)
			| (1<<UTMI_CTRL_PU_REF_SHIFT));
	}

	u2o_set(base, UTMI_CTRL, 1<<UTMI_CTRL_PLL_PWR_UP_SHIFT);
	u2o_set(base, UTMI_CTRL, 1<<UTMI_CTRL_PWR_UP_SHIFT);

	/* UTMI_PLL settings */
	u2o_clear(base, UTMI_PLL, UTMI_PLL_PLLVDD18_MASK
		| UTMI_PLL_PLLVDD12_MASK | UTMI_PLL_PLLCALI12_MASK
		| UTMI_PLL_FBDIV_MASK | UTMI_PLL_REFDIV_MASK
		| UTMI_PLL_ICP_MASK | UTMI_PLL_KVCO_MASK);

	u2o_set(base, UTMI_PLL, 0xee<<UTMI_PLL_FBDIV_SHIFT
		| 0xb<<UTMI_PLL_REFDIV_SHIFT | 3<<UTMI_PLL_PLLVDD18_SHIFT
		| 3<<UTMI_PLL_PLLVDD12_SHIFT | 3<<UTMI_PLL_PLLCALI12_SHIFT
		| 1<<UTMI_PLL_ICP_SHIFT | 3<<UTMI_PLL_KVCO_SHIFT);

	/* UTMI_TX */
	u2o_clear(base, UTMI_TX, UTMI_TX_REG_EXT_FS_RCAL_EN_MASK
		| UTMI_TX_TXVDD12_MASK | UTMI_TX_CK60_PHSEL_MASK
		| UTMI_TX_IMPCAL_VTH_MASK | UTMI_TX_REG_EXT_FS_RCAL_MASK
		| UTMI_TX_AMP_MASK);
	u2o_set(base, UTMI_TX, 3<<UTMI_TX_TXVDD12_SHIFT
		| 4<<UTMI_TX_CK60_PHSEL_SHIFT | 4<<UTMI_TX_IMPCAL_VTH_SHIFT
		| 8<<UTMI_TX_REG_EXT_FS_RCAL_SHIFT | 3<<UTMI_TX_AMP_SHIFT);

	/* UTMI_RX */
	u2o_clear(base, UTMI_RX, UTMI_RX_SQ_THRESH_MASK
		| UTMI_REG_SQ_LENGTH_MASK);
	u2o_set(base, UTMI_RX, 7<<UTMI_RX_SQ_THRESH_SHIFT
		| 2<<UTMI_REG_SQ_LENGTH_SHIFT);

	/* UTMI_IVREF */
	if (mv_phy->type == PXA168_USB)
		/* fixing Microsoft Altair board interface with NEC hub issue -
		 * Set UTMI_IVREF from 0x4a3 to 0x4bf */
		u2o_write(base, UTMI_IVREF, 0x4bf);

	/* toggle VCOCAL_START bit of UTMI_PLL */
	udelay(200);
	u2o_set(base, UTMI_PLL, VCOCAL_START);
	udelay(40);
	u2o_clear(base, UTMI_PLL, VCOCAL_START);

	/* toggle REG_RCAL_START bit of UTMI_TX */
	udelay(400);
	u2o_set(base, UTMI_TX, REG_RCAL_START);
	udelay(40);
	u2o_clear(base, UTMI_TX, REG_RCAL_START);
	udelay(400);

	/* Make sure PHY PLL is ready */
	loops = 0;
	while ((u2o_get(base, UTMI_PLL) & PLL_READY) == 0) {
		mdelay(1);
		loops++;
		if (loops > 100) {
			dev_warn(&pdev->dev, "calibrate timeout, UTMI_PLL %x\n",
				u2o_get(base, UTMI_PLL));
			break;
		}
	}

	if (mv_phy->type == PXA168_USB) {
		u2o_set(base, UTMI_RESERVE, 1 << 5);
		/* Turn on UTMI PHY OTG extension */
		u2o_write(base, UTMI_OTG_ADDON, 1);
	}

	return 0;
}

static int _usb_phy_shutdown(struct mv_usb2_phy *mv_phy)
{
	void __iomem *base = mv_phy->base;

	if (mv_phy->type == PXA168_USB)
		u2o_clear(base, UTMI_OTG_ADDON, UTMI_OTG_ADDON_OTG_ON);

	u2o_clear(base, UTMI_CTRL, UTMI_CTRL_RXBUF_PDWN);
	u2o_clear(base, UTMI_CTRL, UTMI_CTRL_TXBUF_PDWN);
	u2o_clear(base, UTMI_CTRL, UTMI_CTRL_USB_CLK_EN);
	u2o_clear(base, UTMI_CTRL, 1<<UTMI_CTRL_PWR_UP_SHIFT);
	u2o_clear(base, UTMI_CTRL, 1<<UTMI_CTRL_PLL_PWR_UP_SHIFT);

	return 0;
}

static int usb_phy_init(struct mv_usb2_phy *mv_phy)
{
	int i = 0;

	mutex_lock(&mv_phy->phy_lock);
	if (mv_phy->refcount++ == 0) {
		for (i = 0; i < mv_phy->clks_num; i++)
			clk_prepare_enable(mv_phy->clks[i]);
		_usb_phy_init(mv_phy);
	}
	mutex_unlock(&mv_phy->phy_lock);
	return 0;
}

static void usb_phy_shutdown(struct mv_usb2_phy *mv_phy)
{
	int i = 0;

	mutex_lock(&mv_phy->phy_lock);
	if (mv_phy->refcount++ == 0) {
		_usb_phy_shutdown(mv_phy);
		for (i = 0; i < mv_phy->clks_num; i++)
			clk_disable_unprepare(mv_phy->clks[i]);
	}
	mutex_unlock(&mv_phy->phy_lock);
}

static struct of_device_id usb_phy_dt_ids[] = {
	{ .compatible = "mrvl,pxa168-usb-phy",	.data = (void *)PXA168_USB},
	{ .compatible = "mrvl,pxa910-usb-phy",	.data = (void *)PXA910_USB},
	{ .compatible = "mrvl,mmp2-usb-phy",	.data = (void *)MMP2_USB},
	{}
};
MODULE_DEVICE_TABLE(of, usb_phy_dt_ids);

static int __devinit usb_phy_parse_dt(struct platform_device *pdev,
				struct mv_usb2_phy *mv_phy)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id =
			of_match_device(usb_phy_dt_ids, &pdev->dev);
	unsigned int clks_num;
	int i, ret;
	const char *clk_name;

	if (!np)
		return 1;

	clks_num = of_property_count_strings(np, "clocks");
	if (clks_num < 0) {
		dev_err(&pdev->dev, "failed to get clock number\n");
		return clks_num;
	}

	mv_phy->clks = devm_kzalloc(&pdev->dev,
		sizeof(struct clk *) * clks_num, GFP_KERNEL);
	if (mv_phy->clks == NULL) {
		dev_err(&pdev->dev,
			"failed to allocate mempory for clocks");
		return -ENOMEM;
	}

	for (i = 0; i < clks_num; i++) {
		ret = of_property_read_string_index(np, "clocks", i,
			&clk_name);
		if (ret) {
			dev_err(&pdev->dev, "failed to read clocks\n");
			return ret;
		}
		mv_phy->clks[i] = devm_clk_get(&pdev->dev, clk_name);
		if (IS_ERR(mv_phy->clks[i])) {
			dev_err(&pdev->dev, "failed to get clock %s\n",
				clk_name);
			return PTR_ERR(mv_phy->clks[i]);
		}
	}

	mv_phy->clks_num = clks_num;
	mv_phy->type = (unsigned int)(of_id->data);

	return 0;
}

static int __devinit usb_phy_probe(struct platform_device *pdev)
{
	struct mv_usb2_phy *mv_phy;
	struct resource *r;
	int ret, i;

	mv_phy = devm_kzalloc(&pdev->dev, sizeof(*mv_phy), GFP_KERNEL);
	if (mv_phy == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
	mutex_init(&mv_phy->phy_lock);

	ret = usb_phy_parse_dt(pdev, mv_phy);
	/* no CONFIG_OF */
	if (ret > 0) {
		struct mv_usb_phy_platform_data *pdata
				= pdev->dev.platform_data;
		const struct platform_device_id *id
				= platform_get_device_id(pdev);

		if (pdata == NULL || id == NULL) {
			dev_err(&pdev->dev,
				"missing platform_data or id_entry\n");
			return -ENODEV;
		}
		mv_phy->type = (unsigned int)(id->driver_data);
		mv_phy->clks_num = pdata->clknum;
		mv_phy->clks = devm_kzalloc(&pdev->dev,
			sizeof(struct clk *) * mv_phy->clks_num, GFP_KERNEL);
		if (mv_phy->clks == NULL) {
			dev_err(&pdev->dev,
				"failed to allocate mempory for clocks");
			return -ENOMEM;
		}
		for (i = 0; i < mv_phy->clks_num; i++)
			mv_phy->clks[i] = devm_clk_get(&pdev->dev,
							pdata->clkname[i]);
			if (IS_ERR(mv_phy->clks[i])) {
				dev_err(&pdev->dev, "failed to get clock %s\n",
					pdata->clkname[i]);
				return PTR_ERR(mv_phy->clks[i]);
			}
	} else if (ret < 0) {
		dev_err(&pdev->dev, "error parse dt\n");
		return ret;
	}
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no phy I/O memory resource defined\n");
		return -ENODEV;
	}
	mv_phy->base = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (mv_phy->base == NULL) {
		dev_err(&pdev->dev, "error map register base\n");
		return -EBUSY;
	}
	platform_set_drvdata(pdev, mv_phy);
	mv_phy->pdev = pdev;
	mv_phy->init = usb_phy_init;
	mv_phy->shutdown = usb_phy_shutdown;

	mv_phy->extern_chip.head = devm_kzalloc(&pdev->dev,
					sizeof(*mv_phy->extern_chip.head),
					GFP_KERNEL);
	if (mv_phy->extern_chip.head == NULL)
		return -ENOMEM;
	ATOMIC_INIT_NOTIFIER_HEAD(mv_phy->extern_chip.head);

	platform_set_drvdata(pdev, mv_phy);

	the_phy = mv_phy;

	dev_info(&pdev->dev, "mv usb2 phy initialized\n");

	return 0;
}

static int __devexit usb_phy_remove(struct platform_device *pdev)
{
	the_phy = NULL;

	return 0;
}

static struct platform_device_id usb_phy_ids[] = {
	{ .name = "pxa168-usb-phy",	.driver_data = PXA168_USB },
	{ .name = "pxa910-usb-phy",	.driver_data = PXA910_USB },
	{ .name = "mmp2-usb-phy",	.driver_data = MMP2_USB },
	{}
};

static struct platform_driver usb_phy_driver = {
	.probe	= usb_phy_probe,
	.remove = usb_phy_remove,
	.driver = {
		.name   = "pxa168-usb-phy",
		.of_match_table = usb_phy_dt_ids,
	},
	.id_table = usb_phy_ids,
};

static int __init mv_usb2_phy_init(void)
{
	return platform_driver_register(&usb_phy_driver);
}
arch_initcall(mv_usb2_phy_init);
