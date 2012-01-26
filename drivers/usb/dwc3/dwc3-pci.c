/**
 * dwc3-pci.c - PCI Specific glue layer
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

#include "core.h"

/* FIXME define these in <linux/pci_ids.h> */
#define PCI_VENDOR_ID_SYNOPSYS		0x16c3
#define PCI_DEVICE_ID_SYNOPSYS_HAPSUSB3	0xabcd

struct dwc3_pci {
	struct device		*dev;
	struct platform_device	*dwc3;
};

static int __devinit dwc3_pci_probe(struct pci_dev *pci,
		const struct pci_device_id *id)
{
	struct resource		res[2];
	struct platform_device	*dwc3;
	struct dwc3_pci		*glue;
	int			ret = -ENOMEM;
	int			devid;

	glue = kzalloc(sizeof(*glue), GFP_KERNEL);
	if (!glue) {
		dev_err(&pci->dev, "not enough memory\n");
		goto err0;
	}

	glue->dev	= &pci->dev;

	ret = pci_enable_device(pci);
	if (ret) {
		dev_err(&pci->dev, "failed to enable pci device\n");
		goto err1;
	}

	pci_set_power_state(pci, PCI_D0);
	pci_set_master(pci);

	devid = dwc3_get_device_id();
	if (devid < 0)
		goto err2;

	dwc3 = platform_device_alloc("dwc3", devid);
	if (!dwc3) {
		dev_err(&pci->dev, "couldn't allocate dwc3 device\n");
		goto err3;
	}

	memset(res, 0x00, sizeof(struct resource) * ARRAY_SIZE(res));

	res[0].start	= pci_resource_start(pci, 0);
	res[0].end	= pci_resource_end(pci, 0);
	res[0].name	= "dwc_usb3";
	res[0].flags	= IORESOURCE_MEM;

	res[1].start	= pci->irq;
	res[1].name	= "dwc_usb3";
	res[1].flags	= IORESOURCE_IRQ;

	ret = platform_device_add_resources(dwc3, res, ARRAY_SIZE(res));
	if (ret) {
		dev_err(&pci->dev, "couldn't add resources to dwc3 device\n");
		goto err4;
	}

	pci_set_drvdata(pci, glue);

	dma_set_coherent_mask(&dwc3->dev, pci->dev.coherent_dma_mask);

	dwc3->dev.dma_mask = pci->dev.dma_mask;
	dwc3->dev.dma_parms = pci->dev.dma_parms;
	dwc3->dev.parent = &pci->dev;
	glue->dwc3	= dwc3;

	ret = platform_device_add(dwc3);
	if (ret) {
		dev_err(&pci->dev, "failed to register dwc3 device\n");
		goto err4;
	}

	return 0;

err4:
	pci_set_drvdata(pci, NULL);
	platform_device_put(dwc3);

err3:
	dwc3_put_device_id(devid);

err2:
	pci_disable_device(pci);

err1:
	kfree(pci);

err0:
	return ret;
}

static void __devexit dwc3_pci_remove(struct pci_dev *pci)
{
	struct dwc3_pci	*glue = pci_get_drvdata(pci);

	dwc3_put_device_id(glue->dwc3->id);
	platform_device_unregister(glue->dwc3);
	pci_set_drvdata(pci, NULL);
	pci_disable_device(pci);
	kfree(glue);
}

static DEFINE_PCI_DEVICE_TABLE(dwc3_pci_id_table) = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_SYNOPSYS,
				PCI_DEVICE_ID_SYNOPSYS_HAPSUSB3),
	},
	{  }	/* Terminating Entry */
};
MODULE_DEVICE_TABLE(pci, dwc3_pci_id_table);

static struct pci_driver dwc3_pci_driver = {
	.name		= "dwc3-pci",
	.id_table	= dwc3_pci_id_table,
	.probe		= dwc3_pci_probe,
	.remove		= __devexit_p(dwc3_pci_remove),
};

MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 PCI Glue Layer");

static int __devinit dwc3_pci_init(void)
{
	return pci_register_driver(&dwc3_pci_driver);
}
module_init(dwc3_pci_init);

static void __exit dwc3_pci_exit(void)
{
	pci_unregister_driver(&dwc3_pci_driver);
}
module_exit(dwc3_pci_exit);
