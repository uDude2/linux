/**
 * io.h - DesignWare USB3 DRD IO Header
 *
 * Copyright (C) 2010-2012 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef __DRIVERS_USB_DWC3_IO_H
#define __DRIVERS_USB_DWC3_IO_H

#include <linux/io.h>

static inline u32 dwc3_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static inline void dwc3_writel(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset);
}

#endif /* __DRIVERS_USB_DWC3_IO_H */
