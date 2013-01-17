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

enum {
	EVENT_VBUS,
	EVENT_ID,
};

struct pxa_usb_vbus_ops {
	int (*get_vbus)(unsigned int *level);
	int (*set_vbus)(unsigned int level);
	int (*init)(void);
};

struct pxa_usb_idpin_ops {
	int (*get_idpin)(unsigned int *level);
	int (*init)(void);
};

struct pxa_usb_extern_ops {
	struct pxa_usb_vbus_ops		vbus;
	struct pxa_usb_idpin_ops	idpin;
};

struct mv_usb2_extern_chip {
	unsigned int id;
	struct pxa_usb_extern_ops ops;
	struct atomic_notifier_head *head;
};

struct mv_usb2_phy {
	struct platform_device	*pdev;
	struct mutex		phy_lock;
	unsigned int		refcount;
	unsigned int		type;
	void __iomem		*base;
	struct clk		**clks;
	unsigned int		clks_num;

	struct mv_usb2_extern_chip extern_chip;

	int	(*init)(struct mv_usb2_phy *mv_phy);
	void	(*shutdown)(struct mv_usb2_phy *mv_phy);
};

struct mv_usb2_phy *mv_usb2_get_phy(void);

#define mv_usb2_has_extern_call(phy, o, f, arg...)( \
{ \
	int ret;					\
	ret = (!phy ? 0 : ((phy->extern_chip.ops.o.f) ?	\
		1 : 0));				\
	ret;						\
} \
)

#define mv_usb2_extern_call(phy, o, f, args...)( \
{ \
	int ret;						\
	ret = (!phy ? -ENODEV : ((phy->extern_chip.ops.o.f) ?	\
		phy->extern_chip.ops.o.f(args) : -ENOIOCTLCMD));\
	ret;							\
} \
)

#define mv_usb2_set_extern_call(phy, o, f, p)( \
{ \
	int ret;							\
	ret = !phy ? -ENODEV : ((phy->extern_chip.ops.o.f) ?		\
		-EINVAL : ({phy->extern_chip.ops.o.f = p; 0; }));	\
	ret;								\
} \
)

extern int mv_usb2_register_notifier(struct mv_usb2_phy *phy,
					struct notifier_block *nb);
extern int mv_usb2_unregister_notifier(struct mv_usb2_phy *phy,
					struct notifier_block *nb);
extern int mv_usb2_notify(struct mv_usb2_phy *phy, unsigned long val, void *v);

#endif
