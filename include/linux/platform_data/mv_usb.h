/*
 * Copyright (C) 2011 Marvell International Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __MV_PLATFORM_USB_H
#define __MV_PLATFORM_USB_H

enum pxa_ehci_type {
	EHCI_UNDEFINED = 0,
	PXA_U2OEHCI,	/* pxa 168, 9xx */
	PXA_SPH,	/* pxa 168, 9xx SPH */
	MMP3_HSIC,	/* mmp3 hsic */
	MMP3_FSIC,	/* mmp3 fsic */
};

enum {
	MV_USB_MODE_OTG,
	MV_USB_MODE_HOST,
};

enum {
	VBUS_LOW	= 0,
	VBUS_HIGH	= 1 << 0,
};

#define MV_USB_HAS_VBUS_DETECTION	(1 << 0)
#define MV_USB_HAS_IDPIN_DETECTION	(1 << 1)
struct mv_usb_platform_data {
	unsigned int		clknum;
	char			**clkname;

	unsigned int		extern_attr;

	/* only valid for HCD. OTG or Host only*/
	unsigned int		mode;

	/* This flag is used for that needs id pin checked by otg */
	unsigned int    disable_otg_clock_gating:1;
	/* Force a_bus_req to be asserted */
	unsigned int    otg_force_a_bus_req:1;
};

struct mv_usb_phy_platform_data {
	unsigned int	clknum;
	char		**clkname;
};

#endif
