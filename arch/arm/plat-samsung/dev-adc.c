/* linux/arch/arm/plat-samsung/dev-adc.c
 *
 * Copyright 2010 Maurus Cuelenaere
 *
 * S3C64xx series device definition for ADC device
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/platform_data/ntc_thermistor.h>

#include <mach/irqs.h>
#include <mach/map.h>

#include <plat/adc.h>
#include <plat/devs.h>
#include <plat/cpu.h>

static struct resource s3c_adc_resource[] = {
	[0] = {
		.start = SAMSUNG_PA_ADC,
		.end   = SAMSUNG_PA_ADC + SZ_256 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TC,
		.end   = IRQ_TC,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_ADC,
		.end   = IRQ_ADC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device s3c_device_adc = {
	.name		= "samsung-adc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s3c_adc_resource),
	.resource	= s3c_adc_resource,
};

/* NTC Thermistor. Attached to S3C-ADC in some Samsung SoC Devices */
struct platform_device s3c_device_adc_ntc_thermistor;
static int ntc_adc_num = -EINVAL; /* Uninitialized */
static struct s3c_adc_client *ntc_adc;

int __init s3c_adc_ntc_init(int port)
{
	int err = 0;

	if (port < 0 || port > 9)
		return -EINVAL;
	ntc_adc_num = port;

	ntc_adc = s3c_adc_register(&s3c_device_adc_ntc_thermistor, NULL, NULL,
				   0);
	if (IS_ERR(ntc_adc)) {
		err = PTR_ERR(ntc_adc);
		ntc_adc = NULL;
		return err;
	}

	return 0;
}

static int read_thermistor_uV(void)
{
	int val;
	s64 converted;

	WARN(ntc_adc == NULL || ntc_adc_num < 0,
	     "S3C NTC-ADC is not initialized for %s.\n", __func__);

	val = s3c_adc_read(ntc_adc, ntc_adc_num);

	converted = 3300000LL * (s64) val;
	converted >>= 12;

	return converted;
}

static struct ntc_thermistor_platform_data ntc_adc_pdata = {
	.read_uV	= read_thermistor_uV,
	.pullup_uV	= 3300000, /* voltage of vdd for ADC */
	.pullup_ohm	= 100000,
	.pulldown_ohm	= 100000,
	.connect	= NTC_CONNECTED_GROUND,
};

struct platform_device s3c_device_adc_ntc_thermistor = {
	.name			= "ncp15wb473",
	.dev			= {
		.platform_data = &ntc_adc_pdata,
	},
};
