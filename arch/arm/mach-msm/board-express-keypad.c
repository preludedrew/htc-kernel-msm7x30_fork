/* linux/arch/arm/mach-msm/board-express-keypad.c
 *
 * Copyright (C) 2010-2011 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/gpio.h>

#include "board-express.h"
#include "proc_comm.h"
#include <linux/mfd/pmic8058.h>
#include <linux/input/pmic8058-keypad.h>

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_express."
module_param_named(keycaps, keycaps, charp, 0);

static struct gpio_event_direct_entry express_keypad_input_map[] = {
	{
		.gpio = EXPRESS_GPIO_KEYPAD_POWER_KEY,
		.code = KEY_POWER,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(EXPRESS_VOL_UP),
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(EXPRESS_VOL_DN),
		.code = KEY_VOLUMEDOWN,
	},
};

static void express_setup_input_gpio(void)
{
	uint32_t inputs_gpio_table[] = {
		PCOM_GPIO_CFG(EXPRESS_GPIO_KEYPAD_POWER_KEY, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),
	};

	config_gpio_table(inputs_gpio_table, ARRAY_SIZE(inputs_gpio_table));
}

static struct gpio_event_input_info express_keypad_input_info = {
	.info.func = gpio_event_input_func,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = express_keypad_input_map,
	.keymap_size = ARRAY_SIZE(express_keypad_input_map),
	.setup_input_gpio = express_setup_input_gpio,
};

static struct gpio_event_info *express_keypad_info[] = {
	&express_keypad_input_info.info,
};

static struct gpio_event_platform_data express_keypad_data = {
	.names = {
		"express-keypad",
		NULL,
	},
	.info = express_keypad_info,
	.info_count = ARRAY_SIZE(express_keypad_info),
};

static struct platform_device express_keypad_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &express_keypad_data,
	},
};

int __init express_init_keypad(void)
{
	KEY_LOGD("%s\n", __func__);
	return platform_device_register(&express_keypad_input_device);
}
