/* linux/arch/arm/mach-msm/board-express-panel.c
 *
 * Copyright (c) 2010 HTC.
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

#include <asm/io.h>
#include <asm/mach-types.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <mach/vreg.h>
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#include <mach/atmega_microp.h>
#include <mach/panel_id.h>
#include <mach/msm_panel.h>
#include <mach/debug_display.h>

#include "board-express.h"
#include "devices.h"
#include "proc_comm.h"

#define DEBUG_LCM

#ifdef DEBUG_LCM
#define LCMDBG(fmt, arg...)     printk("[DISP][lcm]%s"fmt, __func__, ##arg)
#else
#define LCMDBG(fmt, arg...)     {}
#endif

#define BRIGHTNESS_DEFAULT_LEVEL        102

int qspi_send_16bit(unsigned char id, unsigned data);
int qspi_send_9bit(struct spi_msg *msg);

#define LCM_GPIO_CFG(gpio, func) \
PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)
extern int panel_type;
static DEFINE_MUTEX(panel_lock);
static struct vreg *vreg_lcm_1v8;

static atomic_t lcm_init_done = ATOMIC_INIT(1);
static uint8_t last_val = BRIGHTNESS_DEFAULT_LEVEL;

#if 1
static int express_panel_power(int on_off)
{
	LCMDBG("%s(%d):\n", __func__, on_off);

	switch(panel_type){
	case PANEL_ID_EXP_LG:
	case PANEL_ID_EXP_LG_WS2:
		if (!!on_off) {
			vreg_enable(vreg_lcm_1v8);
			gpio_set_value(EXPRESS_LCM_3V3_EN, 1);
			hr_msleep(50);
			gpio_set_value(EXPRESS_LVDS_ON, 1);
		} else {
			gpio_set_value(EXPRESS_LVDS_ON, 0);
			hr_msleep(50);
			gpio_set_value(EXPRESS_LCM_3V3_EN, 0);
			vreg_disable(vreg_lcm_1v8);
		}
		break;
	case PANEL_ID_EXP_SMD:
	default:
		if (!!on_off) {
			vreg_enable(vreg_lcm_1v8);
			gpio_set_value(EXPRESS_LCM_3V3_EN, 1);
			hr_msleep(90);
			gpio_set_value(EXPRESS_LVDS_ON, 1);
		} else {
			gpio_set_value(EXPRESS_LVDS_ON, 0);
			hr_msleep(60);
			gpio_set_value(EXPRESS_LCM_3V3_EN, 0);
			vreg_disable(vreg_lcm_1v8);
		}
		break;
	}
	return 0;
}
#endif

struct lcm_cmd {
        uint8_t		cmd;
        uint8_t		data;
};

#define LCM_MDELAY	0x03

#define PWM_USER_DEF			142
#define PWM_USER_MIN			30
#define PWM_USER_DIM			20
#define PWM_USER_MAX			255

#define PWM_SAM_DEF			86
#define PWM_SAM_MIN			28
#define PWM_SAM_MAX			255

static uint32_t display_on_gpio_table[] = {

	LCM_GPIO_CFG(EXPRESS_LCD_PCLK, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_DE, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_VSYNC, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_HSYNC, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_G0, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_G1, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_G2, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_G3, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_G4, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_G5, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_G6, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_G7, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_B0, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_B1, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_B2, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_B3, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_B4, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_B5, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_B6, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_B7, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_R0, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_R1, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_R2, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_R3, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_R4, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_R5, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_R6, 1),
	LCM_GPIO_CFG(EXPRESS_LCD_R7, 1),
};

static uint32_t display_off_gpio_table[] = {
	LCM_GPIO_CFG(EXPRESS_LCD_PCLK, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_DE, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_VSYNC, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_HSYNC, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_G0, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_G1, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_G2, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_G3, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_G4, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_G5, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_G6, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_G7, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_B0, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_B1, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_B2, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_B3, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_B4, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_B5, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_B6, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_B7, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_R0, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_R1, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_R2, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_R3, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_R4, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_R5, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_R6, 0),
	LCM_GPIO_CFG(EXPRESS_LCD_R7, 0),
};

static int panel_gpio_switch(int on)
{
	config_gpio_table(
		!!on ? display_on_gpio_table : display_off_gpio_table,
		!!on ? ARRAY_SIZE(display_on_gpio_table) : ARRAY_SIZE(display_off_gpio_table));

	return 0;
}

static int express_shrink_pwm(int brightness, int user_def,
                int user_min, int user_max, int panel_def,
                int panel_min, int panel_max)
{
        if (brightness < PWM_USER_DIM) {
                return 0;
        }

        if (brightness < user_min) {
                return panel_min;
        }

        if (brightness > user_def) {
                brightness = (panel_max - panel_def) *
                        (brightness - user_def) /
                        (user_max - user_def) +
                        panel_def;
        } else {
                        brightness = (panel_def - panel_min) *
                        (brightness - user_min) /
                        (user_def - user_min) +
                        panel_min;
        }

        return brightness;
}

/*----------------------------------------------------------------------------*/
static int express_adjust_backlight(enum led_brightness val)
{
	uint8_t shrink_br = 0;
        uint8_t data[4] = {     /* PWM setting of microp, see p.8 */
                0x05,           /* Fading time; suggested: 5/10/15/20/25 */
                val,            /* Duty Cycle */
                0x00,           /* Channel H byte */
                0x20,           /* Channel L byte */
                };

	if(val == 0)
		data[0] = 0;

        mutex_lock(&panel_lock);
	shrink_br = express_shrink_pwm(val, PWM_USER_DEF,
                                PWM_USER_MIN, PWM_USER_MAX, PWM_SAM_DEF,
                                PWM_SAM_MIN, PWM_SAM_MAX);
        data[1] = shrink_br;

        PR_DISP_DEBUG("[lcm](%d), shrink_br=%d\n", val, shrink_br);
        microp_i2c_write(0x25, data, sizeof(data));
        last_val = shrink_br ? shrink_br: last_val;
        mutex_unlock(&panel_lock);

#if 0
        return shrink_br;
#else
	return val;
#endif
}

static int express_smd_panel_init(struct msm_lcdc_panel_ops *ops)
{
        LCMDBG("\n");

        mutex_lock(&panel_lock);
        express_panel_power(1);
		panel_gpio_switch(1);
        mutex_unlock(&panel_lock);

        return 0;
}

static int express_smd_panel_uninit(struct msm_lcdc_panel_ops *ops)
{
        LCMDBG("\n");

        mutex_lock(&panel_lock);
        express_panel_power(0);
		panel_gpio_switch(0);
        mutex_unlock(&panel_lock);

        return 0;
}

static int express_smd_panel_unblank(struct msm_lcdc_panel_ops *ops)
{
	hr_msleep(250);
	atomic_set(&lcm_init_done, 1);
	express_adjust_backlight(last_val);
        return 0;
}

static int express_smd_panel_blank(struct msm_lcdc_panel_ops *ops)
{
	express_adjust_backlight(0);
	atomic_set(&lcm_init_done, 0);
	hr_msleep(250);
        return 0;
}

static int express_smd_panel_shutdown(struct msm_lcdc_panel_ops *ops)
{
	express_adjust_backlight(0);
	hr_msleep(250);
	mutex_lock(&panel_lock);
	express_panel_power(0);
	atomic_set(&lcm_init_done, 0);
	mutex_unlock(&panel_lock);
        return 0;
}

static struct msm_lcdc_panel_ops express_smd_panel_ops = {
	.init		= express_smd_panel_init,
	.uninit		= express_smd_panel_uninit,
	.blank		= express_smd_panel_blank,
	.unblank	= express_smd_panel_unblank,
	.shutdown	= express_smd_panel_shutdown,
};

static struct msm_lcdc_timing express_smd_timing = {
        .clk_rate               = 40960000,
        .hsync_pulse_width      = 30,
        .hsync_back_porch       = 60,
        .hsync_front_porch      = 36,
        .hsync_skew             = 0,
        .vsync_pulse_width      = 10,
        .vsync_back_porch       = 11,
        .vsync_front_porch      = 10,
        .vsync_act_low          = 0,
        .hsync_act_low          = 0,
        .den_act_low            = 0,
};

/*----------------------------------------------------------------------------*/

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct panel_platform_data lgwsvga_data = {
        .fb_res = &resources_msm_fb[0],
        .power = express_panel_power,
        .gpio_switch = panel_gpio_switch,
};

static struct platform_device lgwsvga_panel = {
        .name = "panel-lgwsvga-ld070ws1-7x30",
        .id = -1,
        .dev = {
                .platform_data = &lgwsvga_data,
        },
};

static struct msm_fb_data express_lcdc_fb_data = {
	.xres		= 1024,
	.yres		= 600,
	.width		= 153,
	.height		= 90,
	.output_format	= 0,
};

static struct msm_lcdc_platform_data express_lcdc_platform_data = {
	.fb_id		= 0,
	.fb_data	= &express_lcdc_fb_data,
	.fb_resource	= &resources_msm_fb[0],
};

static struct platform_device express_lcdc_device = {
	.name	= "msm_mdp_lcdc",
	.id	= -1,
	.dev	= {
		.platform_data = &express_lcdc_platform_data,
	},
};

static struct msm_mdp_platform_data mdp_pdata = {
	.color_format	= MSM_MDP_OUT_IF_FMT_RGB888,
};
/*----------------------------------------------------------------------------*/

static void express_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness val)
{
	if (atomic_read(&lcm_init_done) == 0) {
		last_val = val ? val : last_val;
		LCMDBG(":lcm not ready, val=%d\n", val);
		return;
	}
	led_cdev->brightness = express_adjust_backlight(val);
}

static struct led_classdev express_backlight_led = {
	.name = "lcd-backlight",
	.brightness = LED_FULL,
	.brightness_set = express_brightness_set,
};

static int express_backlight_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &express_backlight_led);
	if (rc)
		LCMDBG("backlight: failure on register led_classdev\n");
	return 0;
}

static struct platform_device express_backlight_pdev = {
	.name = "express-backlight",
};

static struct platform_driver express_backlight_pdrv = {
	.probe          = express_backlight_probe,
	.driver         = {
		.name   = "express-backlight",
		.owner  = THIS_MODULE,
	},
};

static int __init express_backlight_init(void)
{
	return platform_driver_register(&express_backlight_pdrv);
}

/*----------------------------------------------------------------------------*/

int __init express_init_panel(void)
{
	int ret;
	//unsigned id;

        vreg_lcm_1v8 = vreg_get(0, "wlan2");
        if (IS_ERR(vreg_lcm_1v8))
                return PTR_ERR(vreg_lcm_1v8);
/*
        id = PCOM_GPIO_CFG(EXPRESS_LCM_3V3_EN, 1, GPIO_OUTPUT, GPIO_NO_PULL,
               GPIO_16MA);
        msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
        id = PCOM_GPIO_CFG(EXPRESS_LVDS_ON, 1, GPIO_OUTPUT, GPIO_NO_PULL,
               GPIO_16MA);
        msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
*/

	if(panel_type == PANEL_ID_EXP_LG || panel_type == PANEL_ID_EXP_LG_WS2) {
		ret = platform_device_register(&lgwsvga_panel);
		if(ret != 0)
			return ret;
	} else {
		msm_device_mdp.dev.platform_data = &mdp_pdata;
		ret = platform_device_register(&msm_device_mdp);
		if (ret != 0)
			return ret;

		express_lcdc_platform_data.timing = &express_smd_timing;
		express_lcdc_platform_data.panel_ops = &express_smd_panel_ops;

		ret = platform_device_register(&express_lcdc_device);
		if (ret != 0)
			return ret;

		ret = platform_device_register(&express_backlight_pdev);
	        if (ret)
				return ret;
	}

	return 0;
}

module_init(express_backlight_init);
