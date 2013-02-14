/* linux/arch/arm/mach-exynos4/mach-hkdk4210.c
 *
 * Copyright (c) 2013 AgreeYa Mobility Co., Ltd.
 *		http://www.agreeyamobility.net
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/serial_core.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/max8997.h>
#include <linux/lcd.h>
#include <linux/rfkill-gpio.h>
#include <linux/platform_data/s3c-hsotg.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>

#include <video/platform_lcd.h>
#include <video/samsung_fimd.h>

#include <plat/regs-serial.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/sdhci.h>
#include <linux/platform_data/i2c-s3c2410.h>
#include <linux/platform_data/usb-ehci-s5p.h>
#include <plat/clock.h>
#include <plat/gpio-cfg.h>
#include <plat/backlight.h>
#include <plat/fb.h>
#include <plat/mfc.h>
#include <plat/hdmi.h>

#include <linux/platform_data/usb-exynos.h>
#include <mach/map.h>

#include <drm/exynos_drm.h>
#include "common.h"

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define HKDK4210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define HKDK4210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define HKDK4210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg hkdk4210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= HKDK4210_UCON_DEFAULT,
		.ulcon		= HKDK4210_ULCON_DEFAULT,
		.ufcon		= HKDK4210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= HKDK4210_UCON_DEFAULT,
		.ulcon		= HKDK4210_ULCON_DEFAULT,
		.ufcon		= HKDK4210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= HKDK4210_UCON_DEFAULT,
		.ulcon		= HKDK4210_ULCON_DEFAULT,
		.ufcon		= HKDK4210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= HKDK4210_UCON_DEFAULT,
		.ulcon		= HKDK4210_ULCON_DEFAULT,
		.ufcon		= HKDK4210_UFCON_DEFAULT,
	},
};

static struct regulator_consumer_supply __initdata ldo3_consumer[] = {
	REGULATOR_SUPPLY("vdd", "exynos4-hdmi"), /* HDMI */
	REGULATOR_SUPPLY("vdd_pll", "exynos4-hdmi"), /* HDMI */
};
static struct regulator_consumer_supply __initdata ldo6_consumer[] = {
};
static struct regulator_consumer_supply __initdata ldo7_consumer[] = {
};
static struct regulator_consumer_supply __initdata ldo8_consumer[] = {
	REGULATOR_SUPPLY("vdd_osc", "exynos4-hdmi"), /* HDMI */
};
static struct regulator_consumer_supply __initdata ldo9_consumer[] = {
	REGULATOR_SUPPLY("vqmmc", "exynos4-sdhci.2"),
	REGULATOR_SUPPLY("vmmc", "exynos4-sdhci.2"),
};
static struct regulator_consumer_supply __initdata ldo11_consumer[] = {
};
static struct regulator_consumer_supply __initdata ldo14_consumer[] = {
};
static struct regulator_consumer_supply __initdata ldo17_consumer[] = {
};
static struct regulator_consumer_supply __initdata buck1_consumer[] = {
	REGULATOR_SUPPLY("vdd_arm", NULL), /* CPUFREQ */
};
static struct regulator_consumer_supply __initdata buck2_consumer[] = {
	REGULATOR_SUPPLY("vdd_int", NULL), /* CPUFREQ */
};
static struct regulator_consumer_supply __initdata buck3_consumer[] = {
	REGULATOR_SUPPLY("vdd_g3d", "mali_drm"), /* G3D */
};
static struct regulator_consumer_supply __initdata buck7_consumer[] = {
	REGULATOR_SUPPLY("vcc", "platform-lcd"), /* LCD */
};

static struct regulator_init_data __initdata max8997_ldo1_data = {
	.constraints	= {
		.name		= "VDD_ABB_3.3V",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
};

static struct regulator_init_data __initdata max8997_ldo2_data	= {
	.constraints	= {
		.name		= "VDD_ALIVE_1.1V",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
};

static struct regulator_init_data __initdata max8997_ldo3_data = {
	.constraints	= {
		.name		= "VMIPI_1.1V",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo3_consumer),
	.consumer_supplies	= ldo3_consumer,
};

static struct regulator_init_data __initdata max8997_ldo4_data = {
	.constraints	= {
		.name		= "VDD_RTC_1.8V",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
};

static struct regulator_init_data __initdata max8997_ldo6_data = {
	.constraints	= {
		.name		= "VMIPI_1.8V",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo6_consumer),
	.consumer_supplies	= ldo6_consumer,
};

static struct regulator_init_data __initdata max8997_ldo7_data = {
	.constraints	= {
		.name		= "VDD_AUD_1.8V",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo7_consumer),
	.consumer_supplies	= ldo7_consumer,
};

static struct regulator_init_data __initdata max8997_ldo8_data = {
	.constraints	= {
		.name		= "VADC_3.3V",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo8_consumer),
	.consumer_supplies	= ldo8_consumer,
};

static struct regulator_init_data __initdata max8997_ldo9_data = {
	.constraints	= {
		.name		= "DVDD_SWB_2.8V",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo9_consumer),
	.consumer_supplies	= ldo9_consumer,
};

static struct regulator_init_data __initdata max8997_ldo10_data = {
	.constraints	= {
		.name		= "VDD_PLL_1.1V",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
};

static struct regulator_init_data __initdata max8997_ldo11_data = {
	.constraints	= {
		.name		= "VDD_AUD_3V",
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo11_consumer),
	.consumer_supplies	= ldo11_consumer,
};

static struct regulator_init_data __initdata max8997_ldo14_data = {
	.constraints	= {
		.name		= "AVDD18_SWB_1.8V",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo14_consumer),
	.consumer_supplies	= ldo14_consumer,
};

static struct regulator_init_data __initdata max8997_ldo17_data = {
	.constraints	= {
		.name		= "VDD_SWB_3.3V",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo17_consumer),
	.consumer_supplies	= ldo17_consumer,
};

static struct regulator_init_data __initdata max8997_ldo21_data = {
	.constraints	= {
		.name		= "VDD_MIF_1.2V",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
};

static struct regulator_init_data __initdata max8997_buck1_data = {
	.constraints	= {
		.name		= "VDD_ARM_1.2V",
		.min_uV		= 925000,
		.max_uV		= 1350000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(buck1_consumer),
	.consumer_supplies	= buck1_consumer,
};

static struct regulator_init_data __initdata max8997_buck2_data = {
	.constraints	= {
		.name		= "VDD_INT_1.1V",
		.min_uV		= 900000,
		.max_uV		= 1100000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(buck2_consumer),
	.consumer_supplies	= buck2_consumer,
};

static struct regulator_init_data __initdata max8997_buck3_data = {
	.constraints	= {
		.name		= "VDD_G3D_1.1V",
		.min_uV		= 900000,
		.max_uV		= 1100000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(buck3_consumer),
	.consumer_supplies	= buck3_consumer,
};

static struct regulator_init_data __initdata max8997_buck5_data = {
	.constraints	= {
		.name		= "VDDQ_M1M2_1.2V",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
};

static struct regulator_init_data __initdata max8997_buck7_data = {
	.constraints	= {
		.name		= "VDD_LCD_3.3V",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(buck7_consumer),
	.consumer_supplies	= buck7_consumer,
};

static struct max8997_regulator_data __initdata hkdk4210_max8997_regulators[] = {
	{ MAX8997_LDO1,		&max8997_ldo1_data },
	{ MAX8997_LDO2,		&max8997_ldo2_data },
	{ MAX8997_LDO3,		&max8997_ldo3_data },
	{ MAX8997_LDO4,		&max8997_ldo4_data },
	{ MAX8997_LDO6,		&max8997_ldo6_data },
	{ MAX8997_LDO7,		&max8997_ldo7_data },
	{ MAX8997_LDO8,		&max8997_ldo8_data },
	{ MAX8997_LDO9,		&max8997_ldo9_data },
	{ MAX8997_LDO10,	&max8997_ldo10_data },
	{ MAX8997_LDO11,	&max8997_ldo11_data },
	{ MAX8997_LDO14,	&max8997_ldo14_data },
	{ MAX8997_LDO17,	&max8997_ldo17_data },
	{ MAX8997_LDO21,	&max8997_ldo21_data },
	{ MAX8997_BUCK1,	&max8997_buck1_data },
	{ MAX8997_BUCK2,	&max8997_buck2_data },
	{ MAX8997_BUCK3,	&max8997_buck3_data },
	{ MAX8997_BUCK5,	&max8997_buck5_data },
	{ MAX8997_BUCK7,	&max8997_buck7_data },
};

static struct max8997_platform_data __initdata hkdk4210_max8997_pdata = {
	.num_regulators = ARRAY_SIZE(hkdk4210_max8997_regulators),
	.regulators	= hkdk4210_max8997_regulators,

	.wakeup	= true,
	.buck1_gpiodvs	= false,
	.buck2_gpiodvs	= false,
	.buck5_gpiodvs	= false,

	.ignore_gpiodvs_side_effect = true,
	.buck125_default_idx = 0x0,

	.buck125_gpios[0]	= EXYNOS4_GPX0(6),
	.buck125_gpios[1]	= EXYNOS4_GPX0(7),
	.buck125_gpios[2]	= EXYNOS4_GPX0(4),

	.buck1_voltage[0]	= 1350000,
	.buck1_voltage[1]	= 1300000,
	.buck1_voltage[2]	= 1250000,
	.buck1_voltage[3]	= 1200000,
	.buck1_voltage[4]	= 1150000,
	.buck1_voltage[5]	= 1100000,
	.buck1_voltage[6]	= 1000000,
	.buck1_voltage[7]	= 950000,

	.buck2_voltage[0]	= 1100000,
	.buck2_voltage[1]	= 1100000,
	.buck2_voltage[2]	= 1100000,
	.buck2_voltage[3]	= 1100000,
	.buck2_voltage[4]	= 1000000,
	.buck2_voltage[5]	= 1000000,
	.buck2_voltage[6]	= 1000000,
	.buck2_voltage[7]	= 1000000,

	.buck5_voltage[0]	= 1200000,
	.buck5_voltage[1]	= 1200000,
	.buck5_voltage[2]	= 1200000,
	.buck5_voltage[3]	= 1200000,
	.buck5_voltage[4]	= 1200000,
	.buck5_voltage[5]	= 1200000,
	.buck5_voltage[6]	= 1200000,
	.buck5_voltage[7]	= 1200000,
};

/* I2C0 */
static struct i2c_board_info i2c0_devs[] __initdata = {
	{
		I2C_BOARD_INFO("max8997", (0xCC >> 1)),
		.platform_data	= &hkdk4210_max8997_pdata,
		.irq		= IRQ_EINT(0),
	},
};

static struct s3c_sdhci_platdata hkdk4210_hsmmc2_pdata __initdata = {
	.cd_type	= S3C_SDHCI_CD_INTERNAL,
};

/* USB EHCI */
static struct s5p_ehci_platdata hkdk4210_ehci_pdata;

static void __init hkdk4210_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &hkdk4210_ehci_pdata;

	s5p_ehci_set_platdata(pdata);
}

/* USB OHCI */
static struct exynos4_ohci_platdata hkdk4210_ohci_pdata;

static void __init hkdk4210_ohci_init(void)
{
	struct exynos4_ohci_platdata *pdata = &hkdk4210_ohci_pdata;

	exynos4_ohci_set_platdata(pdata);
}

/* USB OTG */
static struct s3c_hsotg_plat hkdk4210_hsotg_pdata;

static struct gpio_led hkdk4210_gpio_leds[] = {
	{
		.name			= "hkdk4210::status",
		.default_trigger	= "heartbeat",
		.gpio			= EXYNOS4_GPX1(2),
		.active_low		= 1,
	},
};

static struct gpio_led_platform_data hkdk4210_gpio_led_info = {
	.leds		= hkdk4210_gpio_leds,
	.num_leds	= ARRAY_SIZE(hkdk4210_gpio_leds),
};

static struct platform_device hkdk4210_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &hkdk4210_gpio_led_info,
	},
};

#ifdef CONFIG_DRM_EXYNOS
static struct exynos_drm_fimd_pdata drm_fimd_pdata = {
	.panel	= {
		.timing	= {
			.left_margin	= 64,
			.right_margin	= 16,
			.upper_margin	= 64,
			.lower_margin	= 16,
			.hsync_len	= 48,
			.vsync_len	= 3,
			.xres		= 1024,
			.yres		= 600,
		},
	},
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC |
				VIDCON1_INV_VCLK,
	.default_win	= 0,
	.bpp		= 32,
};
#else
static struct s3c_fb_pd_win hkdk4210_fb_win0 = {
	.xres			= 1024,
	.yres			= 600,
	.max_bpp		= 32,
	.default_bpp		= 24,
	.virtual_x		= 1024,
	.virtual_y		= 2 * 600,
};

static struct fb_videomode hkdk4210_lcd_timing = {
	.left_margin	= 64,
	.right_margin	= 16,
	.upper_margin	= 64,
	.lower_margin	= 16,
	.hsync_len	= 48,
	.vsync_len	= 3,
	.xres		= 1024,
	.yres		= 600,
};

static struct s3c_fb_platdata hkdk4210_lcd_pdata __initdata = {
	.win[0]		= &hkdk4210_fb_win0,
	.vtiming	= &hkdk4210_lcd_timing,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC |
				VIDCON1_INV_VCLK,
	.setup_gpio	= exynos4_fimd0_gpio_setup_24bpp,
};
#endif

static struct platform_device *hkdk4210_devices[] __initdata = {
	&s3c_device_hsmmc2,
	&s3c_device_i2c0,
	&s3c_device_rtc,
	&s3c_device_usb_hsotg,
	&s3c_device_wdt,
	&s5p_device_ehci,
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc3,
	&s5p_device_fimc_md,
	&s5p_device_fimd0,
	&s5p_device_g2d,
	&s5p_device_hdmi,
	&s5p_device_i2c_hdmiphy,
	&s5p_device_jpeg,
	&s5p_device_mfc,
	&s5p_device_mfc_l,
	&s5p_device_mfc_r,
	&s5p_device_mixer,
#ifdef CONFIG_DRM_EXYNOS
	&exynos_device_drm,
#endif
	&exynos4_device_ohci,
#ifdef CONFIG_SATA_AHCI_PLATFORM
	&exynos4_device_ahci,
#endif
	&hkdk4210_leds_gpio,
};

/* I2C module and id for HDMIPHY */
static struct i2c_board_info hdmiphy_info = {
	I2C_BOARD_INFO("hdmiphy-exynos4210", 0x38),
};

static void s5p_tv_setup(void)
{
	/* Direct HPD to HDMI chip */
	gpio_request_one(EXYNOS4_GPX3(7), GPIOF_IN, "hpd-plug");
	s3c_gpio_cfgpin(EXYNOS4_GPX3(7), S3C_GPIO_SFN(0x3));
	s3c_gpio_setpull(EXYNOS4_GPX3(7), S3C_GPIO_PULL_NONE);
}

static void __init hkdk4210_map_io(void)
{
	exynos_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(hkdk4210_uartcfgs, ARRAY_SIZE(hkdk4210_uartcfgs));
}

static void __init hkdk4210_power_init(void)
{
	gpio_request(EXYNOS4_GPX0(0), "PMIC_IRQ");
	s3c_gpio_cfgpin(EXYNOS4_GPX0(0), S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(EXYNOS4_GPX0(0), S3C_GPIO_PULL_NONE);

	// system power enable (P3V3)
	gpio_request(EXYNOS4_GPX1(1), "p3v3_en");
	s3c_gpio_cfgpin(EXYNOS4_GPX1(1), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPX1(1), S3C_GPIO_PULL_NONE);
	gpio_set_value(EXYNOS4_GPX1(1), 1);
}

static void __init hkdk4210_reserve(void)
{
	s5p_mfc_reserve_mem(0x43000000, 8 << 20, 0x51000000, 8 << 20);
}

static void __init hkdk4210_machine_init(void)
{
	hkdk4210_power_init();

	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, i2c0_devs, ARRAY_SIZE(i2c0_devs));

	/*
	 * Since sdhci instance 2 can contain a bootable media,
	 * sdhci instance 0 is registered after instance 2.
	 */
	s3c_sdhci2_set_platdata(&hkdk4210_hsmmc2_pdata);

	hkdk4210_ehci_init();
	hkdk4210_ohci_init();
	s3c_hsotg_set_platdata(&hkdk4210_hsotg_pdata);

	s5p_tv_setup();
	s5p_i2c_hdmiphy_set_platdata(NULL);
	s5p_hdmi_set_platdata(&hdmiphy_info, NULL, 0);

#ifdef CONFIG_DRM_EXYNOS
	s5p_device_fimd0.dev.platform_data = &drm_fimd_pdata;
	exynos4_fimd0_gpio_setup_24bpp();
#else
	s5p_fimd0_set_platdata(&hkdk4210_lcd_pdata);
#endif

	platform_add_devices(hkdk4210_devices, ARRAY_SIZE(hkdk4210_devices));
}

MACHINE_START(HARDKERNEL, "ODROIDPC")
	/* Maintainer: Dongjin Kim <dongjin.kim@agreeyamobility.net> */
	.atag_offset	= 0x100,
	.smp		= smp_ops(exynos_smp_ops),
	.init_irq	= exynos4_init_irq,
	.map_io		= hkdk4210_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= hkdk4210_machine_init,
	.init_late	= exynos_init_late,
	.timer		= &exynos4_timer,
	.reserve	= &hkdk4210_reserve,
	.restart	= exynos4_restart,
MACHINE_END
