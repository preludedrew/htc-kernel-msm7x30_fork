/* linux/arch/arm/mach-msm/panel-lgwsvga-ld070ws1-7x30.c
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/leds.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_fb.h>
#include <linux/gpio.h>
#include <mach/msm_iomap.h>
#include <mach/msm_panel.h>
#include <mach/panel_id.h>
#include <mach/vreg.h>
#include <linux/spi/spi.h>
#include <mach/atmega_microp.h>
#include <mach/debug_display.h>
#include "devices.h"
#include "../../../drivers/video/msm/mdp_hw.h"

#define DEBUG_LCM
#ifdef DEBUG_LCM
#define LCMDBG(fmt, arg...)	printk("[DISP][lcm]%s"fmt, __func__, ##arg)
#else
#define LCMDBG(fmt, arg...)	{}
#endif

#define PWM_USER_DEF			143
#define PWM_USER_MIN			30
#define PWM_USER_DIM			20
#define PWM_USER_MAX			255

#define PWM_LG_DEF			80
#define PWM_LG_MIN			27
#define PWM_LG_MAX			255

#define LGWSVGA_DEFAULT_LEVEL        102

static DEFINE_MUTEX(panel_lock);
static struct vreg *vreg_lcm_1v8;
static atomic_t lcm_init_done = ATOMIC_INIT(1);
static uint8_t last_val_pwm = LGWSVGA_DEFAULT_LEVEL;
static int (*lgwsvga_power)(int on);
static int (*gpio_switch)(int on);

static const char *PanelVendor = "lg";
static const char *PanelNAME = "ld070ws1";
static const char *PanelSize = "wsvga";
static int color_enhancement = 0;
static int suspend_jiffies = 0;

static ssize_t panel_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", PanelVendor, PanelNAME, PanelSize);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(panel, 0444, panel_vendor_show, NULL);
static struct kobject *android_display;

static int display_sysfs_init(void)
{
	int ret ;
	printk(KERN_INFO "display_sysfs_init : kobject_create_and_add\n");
	android_display = kobject_create_and_add("android_display", NULL);
	if (android_display == NULL) {
		printk(KERN_INFO "display_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	printk(KERN_INFO "display_sysfs_init : sysfs_create_file\n");
	ret = sysfs_create_file(android_display, &dev_attr_panel.attr);
	if (ret) {
		printk(KERN_INFO "display_sysfs_init : sysfs_create_file " \
		"failed\n");
		kobject_del(android_display);
	}

	return 0 ;
}

struct mdp_reg lgwsvga_mdp_init_lut[] = {
	{0x94800, 0x000000, 0x0},
	{0x94804, 0x010001, 0x0},
	{0x94808, 0x010102, 0x0},
	{0x9480C, 0x020202, 0x0},
	{0x94810, 0x020203, 0x0},
	{0x94814, 0x030303, 0x0},
	{0x94818, 0x040404, 0x0},
	{0x9481C, 0x040404, 0x0},
	{0x94820, 0x050505, 0x0},
	{0x94824, 0x060605, 0x0},
	{0x94828, 0x070606, 0x0},
	{0x9482C, 0x070706, 0x0},
	{0x94830, 0x080807, 0x0},
	{0x94834, 0x090908, 0x0},
	{0x94838, 0x0A0908, 0x0},
	{0x9483C, 0x0A0A09, 0x0},
	{0x94840, 0x0B0B0A, 0x0},
	{0x94844, 0x0C0C0A, 0x0},
	{0x94848, 0x0D0D0B, 0x0},
	{0x9484C, 0x0D0D0B, 0x0},
	{0x94850, 0x0E0E0C, 0x0},
	{0x94854, 0x0F0F0D, 0x0},
	{0x94858, 0x10100E, 0x0},
	{0x9485C, 0x10110E, 0x0},
	{0x94860, 0x11110F, 0x0},
	{0x94864, 0x121210, 0x0},
	{0x94868, 0x131310, 0x0},
	{0x9486C, 0x141411, 0x0},
	{0x94870, 0x141512, 0x0},
	{0x94874, 0x151612, 0x0},
	{0x94878, 0x161613, 0x0},
	{0x9487C, 0x171714, 0x0},
	{0x94880, 0x181815, 0x0},
	{0x94884, 0x181915, 0x0},
	{0x94888, 0x191A16, 0x0},
	{0x9488C, 0x1A1B17, 0x0},
	{0x94890, 0x1B1C18, 0x0},
	{0x94894, 0x1C1C18, 0x0},
	{0x94898, 0x1C1D19, 0x0},
	{0x9489C, 0x1D1E1A, 0x0},
	{0x948A0, 0x1E1F1B, 0x0},
	{0x948A4, 0x1F201B, 0x0},
	{0x948A8, 0x20211C, 0x0},
	{0x948AC, 0x20221D, 0x0},
	{0x948B0, 0x21231E, 0x0},
	{0x948B4, 0x22241E, 0x0},
	{0x948B8, 0x23241F, 0x0},
	{0x948BC, 0x242520, 0x0},
	{0x948C0, 0x252621, 0x0},
	{0x948C4, 0x252722, 0x0},
	{0x948C8, 0x262822, 0x0},
	{0x948CC, 0x272923, 0x0},
	{0x948D0, 0x282A24, 0x0},
	{0x948D4, 0x292B25, 0x0},
	{0x948D8, 0x2A2C25, 0x0},
	{0x948DC, 0x2A2D26, 0x0},
	{0x948E0, 0x2B2E27, 0x0},
	{0x948E4, 0x2C2E28, 0x0},
	{0x948E8, 0x2D2F29, 0x0},
	{0x948EC, 0x2E3029, 0x0},
	{0x948F0, 0x2F312A, 0x0},
	{0x948F4, 0x30322B, 0x0},
	{0x948F8, 0x30332C, 0x0},
	{0x948FC, 0x31342D, 0x0},
	{0x94900, 0x32352D, 0x0},
	{0x94904, 0x33362E, 0x0},
	{0x94908, 0x34372F, 0x0},
	{0x9490C, 0x353830, 0x0},
	{0x94910, 0x363931, 0x0},
	{0x94914, 0x363A32, 0x0},
	{0x94918, 0x373B32, 0x0},
	{0x9491C, 0x383C33, 0x0},
	{0x94920, 0x393D34, 0x0},
	{0x94924, 0x3A3E35, 0x0},
	{0x94928, 0x3B3F36, 0x0},
	{0x9492C, 0x3C3F36, 0x0},
	{0x94930, 0x3D4037, 0x0},
	{0x94934, 0x3D4138, 0x0},
	{0x94938, 0x3E4239, 0x0},
	{0x9493C, 0x3F433A, 0x0},
	{0x94940, 0x40443B, 0x0},
	{0x94944, 0x41453B, 0x0},
	{0x94948, 0x42463C, 0x0},
	{0x9494C, 0x43473D, 0x0},
	{0x94950, 0x44483E, 0x0},
	{0x94954, 0x45493F, 0x0},
	{0x94958, 0x454A40, 0x0},
	{0x9495C, 0x464B40, 0x0},
	{0x94960, 0x474C41, 0x0},
	{0x94964, 0x484D42, 0x0},
	{0x94968, 0x494E43, 0x0},
	{0x9496C, 0x4A4F44, 0x0},
	{0x94970, 0x4B5045, 0x0},
	{0x94974, 0x4C5145, 0x0},
	{0x94978, 0x4D5246, 0x0},
	{0x9497C, 0x4E5347, 0x0},
	{0x94980, 0x4F5448, 0x0},
	{0x94984, 0x4F5549, 0x0},
	{0x94988, 0x50564A, 0x0},
	{0x9498C, 0x51574A, 0x0},
	{0x94990, 0x52584B, 0x0},
	{0x94994, 0x53594C, 0x0},
	{0x94998, 0x545A4D, 0x0},
	{0x9499C, 0x555B4E, 0x0},
	{0x949A0, 0x565C4F, 0x0},
	{0x949A4, 0x575D4F, 0x0},
	{0x949A8, 0x585E50, 0x0},
	{0x949AC, 0x595F51, 0x0},
	{0x949B0, 0x5A6052, 0x0},
	{0x949B4, 0x5B6153, 0x0},
	{0x949B8, 0x5C6254, 0x0},
	{0x949BC, 0x5D6355, 0x0},
	{0x949C0, 0x5D6455, 0x0},
	{0x949C4, 0x5E6556, 0x0},
	{0x949C8, 0x5F6657, 0x0},
	{0x949CC, 0x606758, 0x0},
	{0x949D0, 0x616859, 0x0},
	{0x949D4, 0x62695A, 0x0},
	{0x949D8, 0x636A5B, 0x0},
	{0x949DC, 0x646B5C, 0x0},
	{0x949E0, 0x656C5C, 0x0},
	{0x949E4, 0x666D5D, 0x0},
	{0x949E8, 0x676E5E, 0x0},
	{0x949EC, 0x686F5F, 0x0},
	{0x949F0, 0x697060, 0x0},
	{0x949F4, 0x6A7161, 0x0},
	{0x949F8, 0x6B7262, 0x0},
	{0x949FC, 0x6C7363, 0x0},
	{0x94A00, 0x6D7563, 0x0},
	{0x94A04, 0x6E7664, 0x0},
	{0x94A08, 0x6F7765, 0x0},
	{0x94A0C, 0x707866, 0x0},
	{0x94A10, 0x717967, 0x0},
	{0x94A14, 0x727A68, 0x0},
	{0x94A18, 0x737B69, 0x0},
	{0x94A1C, 0x747C6A, 0x0},
	{0x94A20, 0x757D6B, 0x0},
	{0x94A24, 0x767E6C, 0x0},
	{0x94A28, 0x777F6C, 0x0},
	{0x94A2C, 0x78806D, 0x0},
	{0x94A30, 0x79816E, 0x0},
	{0x94A34, 0x7A826F, 0x0},
	{0x94A38, 0x7B8370, 0x0},
	{0x94A3C, 0x7C8471, 0x0},
	{0x94A40, 0x7D8572, 0x0},
	{0x94A44, 0x7E8673, 0x0},
	{0x94A48, 0x7F8774, 0x0},
	{0x94A4C, 0x7F8875, 0x0},
	{0x94A50, 0x808976, 0x0},
	{0x94A54, 0x818A77, 0x0},
	{0x94A58, 0x828C78, 0x0},
	{0x94A5C, 0x838D79, 0x0},
	{0x94A60, 0x848E7A, 0x0},
	{0x94A64, 0x858F7B, 0x0},
	{0x94A68, 0x86907C, 0x0},
	{0x94A6C, 0x87917D, 0x0},
	{0x94A70, 0x88927D, 0x0},
	{0x94A74, 0x89937E, 0x0},
	{0x94A78, 0x8A947F, 0x0},
	{0x94A7C, 0x8B9580, 0x0},
	{0x94A80, 0x8C9681, 0x0},
	{0x94A84, 0x8D9782, 0x0},
	{0x94A88, 0x8E9883, 0x0},
	{0x94A8C, 0x8F9984, 0x0},
	{0x94A90, 0x909A85, 0x0},
	{0x94A94, 0x919B86, 0x0},
	{0x94A98, 0x929D87, 0x0},
	{0x94A9C, 0x939E88, 0x0},
	{0x94AA0, 0x949F89, 0x0},
	{0x94AA4, 0x95A08A, 0x0},
	{0x94AA8, 0x96A18B, 0x0},
	{0x94AAC, 0x97A28D, 0x0},
	{0x94AB0, 0x98A38E, 0x0},
	{0x94AB4, 0x99A48F, 0x0},
	{0x94AB8, 0x9AA590, 0x0},
	{0x94ABC, 0x9CA691, 0x0},
	{0x94AC0, 0x9DA792, 0x0},
	{0x94AC4, 0x9EA893, 0x0},
	{0x94AC8, 0x9FA994, 0x0},
	{0x94ACC, 0xA0AB95, 0x0},
	{0x94AD0, 0xA1AC96, 0x0},
	{0x94AD4, 0xA2AD97, 0x0},
	{0x94AD8, 0xA3AE98, 0x0},
	{0x94ADC, 0xA4AF99, 0x0},
	{0x94AE0, 0xA5B09A, 0x0},
	{0x94AE4, 0xA6B19B, 0x0},
	{0x94AE8, 0xA7B29C, 0x0},
	{0x94AEC, 0xA8B39E, 0x0},
	{0x94AF0, 0xA9B49F, 0x0},
	{0x94AF4, 0xAAB5A0, 0x0},
	{0x94AF8, 0xABB7A1, 0x0},
	{0x94AFC, 0xACB8A2, 0x0},
	{0x94B00, 0xADB9A3, 0x0},
	{0x94B04, 0xAEBAA4, 0x0},
	{0x94B08, 0xAFBBA5, 0x0},
	{0x94B0C, 0xB0BCA6, 0x0},
	{0x94B10, 0xB1BDA7, 0x0},
	{0x94B14, 0xB2BEA9, 0x0},
	{0x94B18, 0xB3BFAA, 0x0},
	{0x94B1C, 0xB4C0AB, 0x0},
	{0x94B20, 0xB5C1AC, 0x0},
	{0x94B24, 0xB6C3AD, 0x0},
	{0x94B28, 0xB7C4AE, 0x0},
	{0x94B2C, 0xB8C5AF, 0x0},
	{0x94B30, 0xB9C6B1, 0x0},
	{0x94B34, 0xBAC7B2, 0x0},
	{0x94B38, 0xBBC8B3, 0x0},
	{0x94B3C, 0xBCC9B4, 0x0},
	{0x94B40, 0xBDCAB5, 0x0},
	{0x94B44, 0xBECBB6, 0x0},
	{0x94B48, 0xBFCDB7, 0x0},
	{0x94B4C, 0xC0CEB9, 0x0},
	{0x94B50, 0xC1CFBA, 0x0},
	{0x94B54, 0xC2D0BB, 0x0},
	{0x94B58, 0xC3D1BC, 0x0},
	{0x94B5C, 0xC4D2BD, 0x0},
	{0x94B60, 0xC5D3BE, 0x0},
	{0x94B64, 0xC6D4C0, 0x0},
	{0x94B68, 0xC7D5C1, 0x0},
	{0x94B6C, 0xC8D7C2, 0x0},
	{0x94B70, 0xC9D8C3, 0x0},
	{0x94B74, 0xCAD9C4, 0x0},
	{0x94B78, 0xCBDAC5, 0x0},
	{0x94B7C, 0xCCDBC7, 0x0},
	{0x94B80, 0xCDDCC8, 0x0},
	{0x94B84, 0xCEDDC9, 0x0},
	{0x94B88, 0xCFDECA, 0x0},
	{0x94B8C, 0xD0DFCB, 0x0},
	{0x94B90, 0xD1E1CD, 0x0},
	{0x94B94, 0xD2E2CE, 0x0},
	{0x94B98, 0xD4E3CF, 0x0},
	{0x94B9C, 0xD5E4D0, 0x0},
	{0x94BA0, 0xD6E5D1, 0x0},
	{0x94BA4, 0xD7E6D2, 0x0},
	{0x94BA8, 0xD8E7D4, 0x0},
	{0x94BAC, 0xD9E8D5, 0x0},
	{0x94BB0, 0xDAEAD6, 0x0},
	{0x94BB4, 0xDBEBD7, 0x0},
	{0x94BB8, 0xDCECD8, 0x0},
	{0x94BBC, 0xDDEDD9, 0x0},
	{0x94BC0, 0xDEEEDA, 0x0},
	{0x94BC4, 0xDFEFDC, 0x0},
	{0x94BC8, 0xE0F0DD, 0x0},
	{0x94BCC, 0xE1F1DE, 0x0},
	{0x94BD0, 0xE2F3DF, 0x0},
	{0x94BD4, 0xE3F4E0, 0x0},
	{0x94BD8, 0xE4F5E1, 0x0},
	{0x94BDC, 0xE5F6E2, 0x0},
	{0x94BE0, 0xE6F7E3, 0x0},
	{0x94BE4, 0xE8F8E5, 0x0},
	{0x94BE8, 0xE9F9E6, 0x0},
	{0x94BEC, 0xEAFAE7, 0x0},
	{0x94BF0, 0xEBFCE8, 0x0},
	{0x94BF4, 0xECFDE9, 0x0},
	{0x94BF8, 0xEDFEEA, 0x0},
	{0x94BFC, 0xEEFFEB, 0x0}, 
	{0x90070, 0x00001F, 0x7},
};

struct mdp_reg lg_mdp_init_color[] = {
	{0x93400, 0x01E6, 0x0},
	{0x93404, 0xFFE1, 0x0},
	{0x93408, 0x0042, 0x0},
	{0x9340C, 0x0039, 0x0},
	{0x93410, 0x020B, 0x0},
	{0x93414, 0xFFC5, 0x0},
	{0x93418, 0x0002, 0x0},
	{0x9341C, 0x0012, 0x0},
	{0x93420, 0x01F3, 0x0},

	{0x93600, 0x0000, 0x0},
	{0x93604, 0x00FF, 0x0},
	{0x93608, 0x0000, 0x0},
	{0x9360C, 0x00FF, 0x0},
	{0x93610, 0x0000, 0x0},
	{0x93614, 0x00FF, 0x0},
	{0x93680, 0x0000, 0x0},
	{0x93684, 0x00FF, 0x0},
	{0x93688, 0x0000, 0x0},
	{0x9368C, 0x00FF, 0x0},
	{0x93690, 0x0000, 0x0},
	{0x93694, 0x00FF, 0x0},
	{0x90070, 0x001F, 0x8},
};


void lgwsvga_mdp_color_enhancement(struct mdp_device *mdp_dev)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);

	mdp->write_regs(mdp, lgwsvga_mdp_init_lut, ARRAY_SIZE(lgwsvga_mdp_init_lut));
	mdp->write_regs(mdp, lg_mdp_init_color, ARRAY_SIZE(lg_mdp_init_color));
}


static int
lgwsvga_panel_shrink_pwm(int brightness)
{
	if (brightness < PWM_USER_DIM) {
			return 0;
	}

	if (brightness < PWM_USER_MIN) {
			return PWM_LG_MIN;
	}

	if (brightness > PWM_USER_DEF) {
			brightness = (PWM_LG_MAX - PWM_LG_DEF) *
					(brightness - PWM_USER_DEF) /
					(PWM_USER_MAX - PWM_USER_DEF) +
					PWM_LG_DEF;
	} else {
					brightness = (PWM_LG_DEF - PWM_LG_MIN) *
					(brightness - PWM_USER_MIN) /
					(PWM_USER_DEF - PWM_USER_MIN) +
					PWM_LG_MIN;
	}

	return brightness;

}

static int lgwsvga_panel_power(int on)
{
	int ret = -EIO;

	if (lgwsvga_power) {
		ret = (*lgwsvga_power)(on);
		if (ret)
			goto power_fail;
	}

	if (gpio_switch) {
		ret = (*gpio_switch)(on);
		if (ret)
			goto power_fail;
	}
	return 0;

power_fail:
	return ret;
}

extern int qspi_send_9bit(struct spi_msg *msg);

#define LCM_CMD(_cmd, ...)					\
{                                                               \
        .cmd = _cmd,                                            \
        .data = (u8 []){__VA_ARGS__},                           \
        .len = sizeof((u8 []){__VA_ARGS__}) / sizeof(u8)        \
}

static int lgwsvga_adjust_backlight(enum led_brightness val)
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
	shrink_br = lgwsvga_panel_shrink_pwm(val);
        data[1] = shrink_br;

	PR_DISP_DEBUG("[lcm]%s:(%d), shrink_br=%d\n", __func__, val, shrink_br);
        microp_i2c_write(0x25, data, sizeof(data));
        last_val_pwm = shrink_br ? shrink_br: last_val_pwm;
        mutex_unlock(&panel_lock);

#if 0
        return shrink_br;
#else
	return val;
#endif
}

static struct msm_mdp_platform_data mdp_pdata = {
        .color_format = MSM_MDP_OUT_IF_FMT_RGB888,
};

static int lgwsvga_panel_init(struct msm_lcdc_panel_ops *ops)
{
	LCMDBG("%s()\n", __func__);

	mutex_lock(&panel_lock);
	lgwsvga_panel_power(1);
	mutex_unlock(&panel_lock);

	return 0;
}

static int lgwsvga_panel_uninit(struct msm_lcdc_panel_ops *ops)
{
        LCMDBG("\n");

        mutex_lock(&panel_lock);
        lgwsvga_panel_power(0);
        mutex_unlock(&panel_lock);

        return 0;
}

static int lgwsvga_panel_unblank(struct msm_lcdc_panel_ops *panel_data)
{
	int delay_time = 0;
	if (color_enhancement == 0) {
			lgwsvga_mdp_color_enhancement(mdp_pdata.mdp_dev);
			color_enhancement = 1;
	}

	delay_time = (jiffies - suspend_jiffies) * 1000 / HZ;

	if(delay_time < 500) {
		delay_time = 500 - delay_time;
		PR_DISP_INFO("resume delay_time:%d", delay_time);
		hr_msleep(delay_time);
	}
	atomic_set(&lcm_init_done, 1);
	lgwsvga_adjust_backlight(last_val_pwm);
        return 0;
}

static int lgwsvga_panel_blank(struct msm_lcdc_panel_ops *panel_data)
{
	lgwsvga_adjust_backlight(0);
	atomic_set(&lcm_init_done, 0);
	suspend_jiffies = jiffies;
        return 0;
}

static int lgwsvga_panel_shutdown(struct msm_lcdc_panel_ops *ops)
{
	lgwsvga_adjust_backlight(0);
	atomic_set(&lcm_init_done, 0);
	hr_msleep(250);
	mutex_lock(&panel_lock);
        lgwsvga_panel_power(0);
        mutex_unlock(&panel_lock);
	hr_msleep(400);
        return 0;
}

static struct msm_lcdc_panel_ops lgwsvga_lcdc_panel_ops = {
	.init 		= lgwsvga_panel_init,
	.uninit 	= lgwsvga_panel_uninit,
	.blank 		= lgwsvga_panel_blank,
	.unblank 	= lgwsvga_panel_unblank,
	.shutdown 	= lgwsvga_panel_shutdown,
};

static struct msm_lcdc_timing lgwsvga_lcdc_timing = {
	.clk_rate			= 48083000,	//In Bravoc, Radio will adjust clk rate from 24.57Mhz to 23.59Mhz to pass EMC testing
	.hsync_pulse_width	= 1,
	.hsync_back_porch	= 160,
	.hsync_front_porch	= 160,
	.hsync_skew			= 0,
	.vsync_pulse_width	= 1,
	.vsync_back_porch	= 23,
	.vsync_front_porch	= 12,
	.vsync_act_low		= 0,
	.hsync_act_low		= 0,
	.den_act_low		= 0,
};

static struct msm_fb_data lgwsvga_lcdc_fb_data = {
	.xres		= 1024,
	.yres		= 600,
	.width		= 153,
	.height		= 90,
	.output_format	= 0,
};

static struct msm_lcdc_platform_data lgwsvga_lcdc_platform_data = {
	.panel_ops	= &lgwsvga_lcdc_panel_ops,
	.timing		= &lgwsvga_lcdc_timing,
	.fb_id		= 0,
	.fb_data	= &lgwsvga_lcdc_fb_data,
};

static struct platform_device lgwsvga_lcdc_device = {
	.name	= "msm_mdp_lcdc",
	.id	= -1,
	.dev	= {
		.platform_data = &lgwsvga_lcdc_platform_data,
	},
};

void lgwsvga_brightness_set(struct led_classdev *led_cdev,
			enum led_brightness val)
{
	if (atomic_read(&lcm_init_done) == 0) {
		last_val_pwm= val ? val : last_val_pwm;
		LCMDBG(":lcm not ready, val=%d\n", val);
		return;
	}
	led_cdev->brightness = lgwsvga_adjust_backlight(val);

}

static struct led_classdev lgwsvga_backlight_led = {
	.name = "lcd-backlight",
	.brightness = LED_FULL,
	.brightness_set = lgwsvga_brightness_set,
};

static int lgwsvga_backlight_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &lgwsvga_backlight_led);
	if (rc)
		LCMDBG("backlight: failure on register led_classdev\n");
	return 0;
}

static struct platform_device lgwsvga_backlight = {
	.name = "lgwsvga-backlight",
};

static struct platform_driver lgwsvga_backlight_driver = {
	.probe		= lgwsvga_backlight_probe,
	.driver		= {
		.name	= "lgwsvga-backlight",
		.owner	= THIS_MODULE,
	},
};

static int __init lgwsvga_init_panel(void)
{
	int ret;

	vreg_lcm_1v8 = vreg_get(0, "gp13");
	if (IS_ERR(vreg_lcm_1v8))
			return PTR_ERR(vreg_lcm_1v8);

	msm_device_mdp.dev.platform_data = &mdp_pdata;
	ret = platform_device_register(&msm_device_mdp);
	if (ret != 0)
		return ret;

	lgwsvga_lcdc_platform_data.timing = &lgwsvga_lcdc_timing;
	lgwsvga_lcdc_platform_data.panel_ops = &lgwsvga_lcdc_panel_ops;

	ret = platform_device_register(&lgwsvga_lcdc_device);
	if (ret != 0)
		return ret;

	ret = platform_device_register(&lgwsvga_backlight);
		if (ret)
				return ret;

	return 0;
}

static int lgwsvga_probe(struct platform_device *pdev)
{
	int rc = -EIO;
	struct panel_platform_data *pdata;
	pdata = pdev->dev.platform_data;

	lgwsvga_power = pdata->power;
	gpio_switch = pdata->gpio_switch;
	lgwsvga_lcdc_platform_data.fb_resource = pdata->fb_res;

	last_val_pwm = LGWSVGA_DEFAULT_LEVEL;

	rc = lgwsvga_init_panel();
	if (rc)
		printk(KERN_ERR "%s fail %d\n", __func__, rc);

	display_sysfs_init();
	return rc;
}

static struct platform_driver lgwsvga_driver = {
	.probe = lgwsvga_probe,
	.driver = { .name = "panel-lgwsvga-ld070ws1-7x30" },
};

static int __init lgwsvga_7x30_init(void)
{
	return platform_driver_register(&lgwsvga_driver);
}

static int __init lgwsvga_backlight_7x30_init(void)
{
	return platform_driver_register(&lgwsvga_backlight_driver);
}

device_initcall(lgwsvga_7x30_init);
module_init(lgwsvga_backlight_7x30_init);
