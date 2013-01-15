/*
 * Copyright (C) 2012 Marvell Inc.
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

#ifndef __MV_USB2_H
#define __MV_USB2_H

#include <linux/clk.h>

struct mv_usb2_phy {
	struct platform_device	*pdev;
	struct mutex		phy_lock;
	unsigned int		refcount;
	unsigned int		type;
	void __iomem		*base;
	struct clk		**clks;
	unsigned int		clks_num;

	int	(*init)(struct mv_usb2_phy *mv_phy);
	void	(*shutdown)(struct mv_usb2_phy *mv_phy);
};

struct mv_usb2_phy *mv_usb2_get_phy(void);

#endif
