/*
 * arch/arm/mach-omap2/board-mapphone-cpcap-client.c
 *
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spi/cpcap.h>
#include <linux/leds-ld-cpcap.h>
#include <linux/leds-ld-cpcap-disp.h>
#include <linux/leds-cpcap-kpad.h>
#include <linux/leds-cpcap-display.h>
#include <linux/leds-cpcap-button.h>
#include <linux/cpcap_audio_platform_data.h>
#include <linux/pm_dbg.h>
#include <mach/gpio.h>
#include <plat/mux.h>
#include <plat/resource.h>
#include <plat/omap34xx.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif
/*
 * CPCAP devcies are common for different HW Rev.
 *
 */
static struct platform_device cpcap_3mm5_device = {
	.name   = "cpcap_3mm5",
	.id     = -1,
	.dev    = {
		.platform_data  = NULL,
	},
};

#ifdef CONFIG_CPCAP_USB
static struct platform_device cpcap_usb_device = {
	.name           = "cpcap_usb",
	.id             = -1,
	.dev.platform_data = NULL,
};

static struct platform_device cpcap_usb_det_device = {
	.name   = "cpcap_usb_det",
	.id     = -1,
	.dev    = {
		.platform_data  = NULL,
	},
};
#endif /* CONFIG_CPCAP_USB */

static struct platform_device cpcap_rgb_led = {
	.name           = LD_MSG_IND_DEV,
	.id             = -1,
	.dev.platform_data  = NULL,
};

#ifdef CONFIG_LEDS_AF_LED
static struct platform_device cpcap_af_led = {
	.name           = LD_AF_LED_DEV,
	.id             = -1,
	.dev            = {
		.platform_data  = NULL,
       },
};
#endif

static struct platform_device cpcap_bd7885 = {
	.name           = "bd7885",
	.id             = -1,
	.dev            = {
		.platform_data  = NULL,
       },
};

static struct platform_device cpcap_vio_active_device = {
	.name		= "cpcap_vio_active",
	.id		= -1,
	.dev		= {
		.platform_data = NULL,
	},
};

#ifdef CONFIG_PM_DBG_DRV
static struct platform_device cpcap_pm_dbg_device = {
	.name		= "cpcap_pm_dbg",
	.id		= -1,
	.dev		= {
		.platform_data = NULL,
	},
};

static struct pm_dbg_drvdata cpcap_pm_dbg_drvdata = {
	.pm_cd_factor = 1000,
};
#endif

static struct platform_device *cpcap_devices[] = {
#ifdef CONFIG_CPCAP_USB
	&cpcap_usb_device,
	&cpcap_usb_det_device,
#endif
	&cpcap_3mm5_device,
#ifdef CONFIG_LEDS_AF_LED
	&cpcap_af_led,
#endif
	&cpcap_bd7885
};


/*
 * CPCAP devcies whose availability depends on HW
 *
 */
static struct platform_device cpcap_button_led = {
	.name           = CPCAP_BUTTON_DEV,
	.id             = -1,
	.dev            = {
	.platform_data  = NULL,
	},
};

static struct disp_button_config_data btn_data;

static struct platform_device cpcap_display_led = {
	.name           = CPCAP_DISPLAY_DRV,
	.id             = -1,
	.dev            = {
		.platform_data  = NULL,
	},
};

#ifdef CONFIG_ARM_OF
static int __init cpcap_button_init(void)
{
	u8 device_available =1;
	return device_available;
}

static int __init ld_cpcap_disp_button_init(void)
{
	struct disp_button_config_data *pbtn_data = &btn_data;
	u8 device_available, ret;

	ret = -ENODEV;

	device_available = 1;
	pbtn_data->duty_cycle = 0xB8;
	pbtn_data->cpcap_mask = 0x3FF;
	pbtn_data->led_current =  0x0;
	pbtn_data->reg = CPCAP_REG_BLEDC;

	ret = 1;
	return ret;
}

static int __init is_disp_led_on(void)
{
	u8 device_available = 1;
	return device_available;
}

static int __init is_ld_cpcap_rgb_on(void)
{
	u8 device_available = 1;
	return device_available;
}

int is_cpcap_vio_supply_converter(void)
{
	return 1;
}

#endif /* CONFIG_ARM_OF */


void __init mapphone_cpcap_client_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cpcap_devices); i++)
		cpcap_device_register(cpcap_devices[i]);

	if (cpcap_button_init() > 0)
		cpcap_device_register(&cpcap_button_led);

	if (is_disp_led_on() > 0)
		cpcap_device_register(&cpcap_display_led);

	if (is_ld_cpcap_rgb_on() > 0)
		cpcap_device_register(&cpcap_rgb_led);

	if (!is_cpcap_vio_supply_converter())
		cpcap_device_register(&cpcap_vio_active_device);
}
