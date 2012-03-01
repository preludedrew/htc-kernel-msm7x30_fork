/* linux/arch/arm/mach-msm/board-express-mmc.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>

#include <asm/gpio.h>
#include <asm/io.h>

#include <mach/vreg.h>
#include <mach/htc_pwrsink.h>

#include <asm/mach/mmc.h>
#include <mach/mpp.h>

#include "devices.h"
#include "board-express.h"
#include "proc_comm.h"
#include "board-common-wimax.h"

#ifdef CONFIG_WIMAX_SERIAL_MSM
#include <mach/msm_serial_wimax.h>
#include <linux/irq.h>

#define MSM_UART3_PHYS		0xACC00000
#define INT_UART3_IRQ		INT_UART3
#endif

#define EXPRESS_SDMC_CD_N_TO_SYS PM8058_GPIO_PM_TO_SYS(EXPRESS_SDMC_CD_N)

extern int msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
			unsigned int stat_irq, unsigned long stat_irq_flags);

/* ---- SDCARD ---- */

static uint32_t sdcard_on_gpio_table[] = {
	PCOM_GPIO_CFG(58, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* CLK */
	PCOM_GPIO_CFG(59, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(60, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(61, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(62, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT1 */
	PCOM_GPIO_CFG(63, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT0 */
};

static uint32_t sdcard_off_gpio_table[] = {
	PCOM_GPIO_CFG(58, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(59, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(60, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(61, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(63, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
};

static uint opt_disable_sdcard;

static uint32_t movinand_on_gpio_table[] = {
	PCOM_GPIO_CFG(64, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(65, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(66, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(67, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT1 */
	PCOM_GPIO_CFG(69, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT0 */
	PCOM_GPIO_CFG(115, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT4 */
	PCOM_GPIO_CFG(114, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT5 */
	PCOM_GPIO_CFG(113, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT6 */
	PCOM_GPIO_CFG(112, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT7 */
};
static int __init express_disablesdcard_setup(char *str)
{
	int cal = simple_strtol(str, NULL, 0);

	opt_disable_sdcard = cal;
	return 1;
}

__setup("board_express.disable_sdcard=", express_disablesdcard_setup);

static struct vreg *vreg_sdslot;	/* SD slot power */

struct mmc_vdd_xlat {
	int mask;
	int level;
};

static struct mmc_vdd_xlat mmc_vdd_table[] = {
	{ MMC_VDD_28_29,	2850 },
	{ MMC_VDD_29_30,	2900 },
};

static unsigned int sdslot_vdd = 0xffffffff;
static unsigned int sdslot_vreg_enabled;

static uint32_t express_sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int i;

	BUG_ON(!vreg_sdslot);

	if (vdd == sdslot_vdd)
		return 0;

	sdslot_vdd = vdd;

	if (vdd == 0) {
		printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
		config_gpio_table(sdcard_off_gpio_table,
				  ARRAY_SIZE(sdcard_off_gpio_table));
		vreg_disable(vreg_sdslot);
		sdslot_vreg_enabled = 0;
		return 0;
	}

	if (!sdslot_vreg_enabled) {
		mdelay(5);
		vreg_enable(vreg_sdslot);
		udelay(500);
		config_gpio_table(sdcard_on_gpio_table,
				  ARRAY_SIZE(sdcard_on_gpio_table));
		sdslot_vreg_enabled = 1;
	}

	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask == (1 << vdd)) {
			printk(KERN_INFO "%s: Setting level to %u\n",
				__func__, mmc_vdd_table[i].level);
			vreg_set_level(vreg_sdslot, mmc_vdd_table[i].level);
			return 0;
		}
	}

	printk(KERN_ERR "%s: Invalid VDD %d specified\n", __func__, vdd);
	return 0;
}

static unsigned int express_sdslot_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) gpio_get_value(EXPRESS_SDMC_CD_N_TO_SYS);
	return (!status);
}

#define EXPRESS_MMC_VDD		(MMC_VDD_28_29 | MMC_VDD_29_30)

static unsigned int express_sdslot_type = MMC_TYPE_SD;

static struct mmc_platform_data express_sdslot_data = {
	.ocr_mask	= EXPRESS_MMC_VDD,
	.status_irq	= MSM_GPIO_TO_INT(EXPRESS_SDMC_CD_N_TO_SYS),
	.status		= express_sdslot_status,
	.translate_vdd	= express_sdslot_switchvdd,
	.slot_type	= &express_sdslot_type,
	.dat0_gpio	= 63,
};

static unsigned int express_emmcslot_type = MMC_TYPE_MMC;
static struct mmc_platform_data express_movinand_data = {
	.ocr_mask	= EXPRESS_MMC_VDD,
	.slot_type	= &express_emmcslot_type,
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
};

/* ---- WIFI ---- */

static uint32_t wifi_on_gpio_table[] = {
	PCOM_GPIO_CFG(116, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(117, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(118, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(119, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(111, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(110, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(147, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), /* WLAN IRQ */
};

static uint32_t wifi_off_gpio_table[] = {
	PCOM_GPIO_CFG(116, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(117, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(118, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(119, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(111, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(110, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(147, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* WLAN IRQ */
};

/* BCM4329 returns wrong sdio_vsn(1) when we read cccr,
 * we use predefined value (sdio_vsn=2) here to initial sdio driver well
 */
static struct embedded_sdio_data express_wifi_emb_data = {
	.cccr	= {
		.sdio_vsn	= 2,
		.multi_block	= 1,
		.low_speed	= 0,
		.wide_bus	= 0,
		.high_power	= 1,
		.high_speed	= 1,
	},
};

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int
express_wifi_status_register(void (*callback)(int card_present, void *dev_id),
				void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int express_wifi_cd;	/* WiFi virtual 'card detect' status */

static unsigned int express_wifi_status(struct device *dev)
{
	return express_wifi_cd;
}

static struct mmc_platform_data express_wifi_data = {
	.ocr_mask		= MMC_VDD_28_29,
	.status			= express_wifi_status,
	.register_status_notify	= express_wifi_status_register,
	.embedded_sdio		= &express_wifi_emb_data,
};

int express_wifi_set_carddetect(int val)
{
	printk(KERN_INFO "%s: %d\n", __func__, val);
	express_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}
EXPORT_SYMBOL(express_wifi_set_carddetect);

static struct pm8058_gpio pmic_gpio_sleep_clk_output = {
	.direction      = PM_GPIO_DIR_OUT,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.output_value   = 0,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_S3,      /* S3 1.8 V */
	.out_strength   = PM_GPIO_STRENGTH_HIGH,
	.function       = PM_GPIO_FUNC_2,
};

#define ID_WIFI	0
#define ID_BT	1
#define CLK_OFF	0
#define CLK_ON	1
static DEFINE_SPINLOCK(express_w_b_slock);
int express_sleep_clk_state_wifi = CLK_OFF;
int express_sleep_clk_state_bt = CLK_OFF;

int express_wifi_bt_sleep_clk_ctl(int on, int id)
{
	int err = 0;
	unsigned long flags;

	printk(KERN_DEBUG "%s ON=%d, ID=%d\n", __func__, on, id);

	spin_lock_irqsave(&express_w_b_slock, flags);
	if (on) {
		if ((CLK_OFF == express_sleep_clk_state_wifi)
			&& (CLK_OFF == express_sleep_clk_state_bt)) {
			printk(KERN_DEBUG "EN SLEEP CLK\n");
			pmic_gpio_sleep_clk_output.function = PM_GPIO_FUNC_2;
			err = pm8058_gpio_config(
					EXPRESS_GPIO_WIFI_BT_SLEEP_CLK_EN,
					&pmic_gpio_sleep_clk_output);
			if (err) {
				spin_unlock_irqrestore(&express_w_b_slock,
							flags);
				printk(KERN_DEBUG "ERR EN SLEEP CLK, ERR=%d\n",
					err);
				return err;
			}
		}

		if (id == ID_BT)
			express_sleep_clk_state_bt = CLK_ON;
		else
			express_sleep_clk_state_wifi = CLK_ON;
	} else {
		if (((id == ID_BT) && (CLK_OFF == express_sleep_clk_state_wifi))
			|| ((id == ID_WIFI)
			&& (CLK_OFF == express_sleep_clk_state_bt))) {
			printk(KERN_DEBUG "DIS SLEEP CLK\n");
			pmic_gpio_sleep_clk_output.function
					= PM_GPIO_FUNC_NORMAL;
			err = pm8058_gpio_config(
					EXPRESS_GPIO_WIFI_BT_SLEEP_CLK_EN,
					&pmic_gpio_sleep_clk_output);
			if (err) {
				spin_unlock_irqrestore(&express_w_b_slock,
							flags);
				printk(KERN_DEBUG "ERR DIS SLEEP CLK, ERR=%d\n",
					err);
				return err;
			}
		} else {
			printk(KERN_DEBUG "KEEP SLEEP CLK ALIVE\n");
		}

		if (id)
			express_sleep_clk_state_bt = CLK_OFF;
		else
			express_sleep_clk_state_wifi = CLK_OFF;
	}
	spin_unlock_irqrestore(&express_w_b_slock, flags);

	return 0;
}
EXPORT_SYMBOL(express_wifi_bt_sleep_clk_ctl);

int express_wifi_power(int on)
{
	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (on) {
		config_gpio_table(wifi_on_gpio_table,
				  ARRAY_SIZE(wifi_on_gpio_table));
	} else {
		config_gpio_table(wifi_off_gpio_table,
				  ARRAY_SIZE(wifi_off_gpio_table));
	}

    express_wifi_bt_sleep_clk_ctl(on, ID_WIFI);
	gpio_set_value(EXPRESS_GPIO_WIFI_EN, on); /* WIFI_SHUTDOWN */

	mdelay(120);
	return 0;
}
EXPORT_SYMBOL(express_wifi_power);

int express_wifi_reset(int on)
{
	printk(KERN_INFO "%s: do nothing\n", __func__);
	return 0;
}

// /* ---------------- WiMAX GPIO Settings --------------- */
static uint32_t wimax_on_gpio_table[] = {
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_SDIO_D0, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT0 */	 
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_SDIO_D1, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT1 */
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_SDIO_D2, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_SDIO_D3, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_SDIO_CMD, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_SDIO_CLK_CPU, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(EXPRESS_GPIO_V_WIMAX_PVDD_EN, 0,  GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* PVDD_EN */
	PCOM_GPIO_CFG(EXPRESS_GPIO_V_WIMAX_DVDD_EN, 0,  GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* DVDD_EN */
	PCOM_GPIO_CFG(EXPRESS_GPIO_V_WIMAX_1V2_RF_EN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* 1V2RF_EN */
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_EXT_RST, 0, 	GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* WIMAX_EXT_RST */
};

static uint32_t wimax_off_gpio_table[] = {
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_SDIO_D0, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */	
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_SDIO_D1, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_SDIO_D2, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_SDIO_D3, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_SDIO_CMD, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */	
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_SDIO_CLK_CPU, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */	
	PCOM_GPIO_CFG(EXPRESS_GPIO_V_WIMAX_PVDD_EN, 0,  GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* PVDD_EN */
	PCOM_GPIO_CFG(EXPRESS_GPIO_V_WIMAX_DVDD_EN, 0,  GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* DVDD_EN */
	PCOM_GPIO_CFG(EXPRESS_GPIO_V_WIMAX_1V2_RF_EN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* 1V2RF_EN */
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_EXT_RST, 0, 	GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* WIMAX_EXT_RST */
};

static void wimax_on_pmic_gpio(void)
{
	int ret =0;
	printk(KERN_INFO "%s: enter\n", __func__);

	/* BT_RESET_N */
	ret = pm8058_mpp_config_digital_out(EXPRESS_WIMAX_DEBUG12,
			PM8058_MPP_DIG_LEVEL_S3, PM_MPP_DOUT_CTL_LOW);

	if (ret)
		printk(KERN_INFO "[ERROR]%s: Set up EXPRESS_WIMAX_DEBUG12 fail !!!\n", __func__);

}


static void (*wimax_status_cb)(int card_present, void *dev_id);
static void *wimax_status_cb_devid;
static int mmc_wimax_cd = 0;
static int mmc_wimax_hostwakeup_gpio = PM8058_GPIO_PM_TO_SYS(EXPRESS_WIMAX_HOST_WAKEUP);

static int mmc_wimax_status_register(void (*callback)(int card_present, void *dev_id), void *dev_id)
{
	if (wimax_status_cb)
		return -EAGAIN;
	printk("%s\n", __func__);
	wimax_status_cb = callback;
	wimax_status_cb_devid = dev_id;
	return 0;
}

static unsigned int mmc_wimax_status(struct device *dev)
{
	printk("%s\n", __func__);
	return mmc_wimax_cd;
}

void mmc_wimax_set_carddetect(int val)
{
	printk("%s: %d\n", __func__, val);
	mmc_wimax_cd = val;
	if (wimax_status_cb) {
		wimax_status_cb(val, wimax_status_cb_devid);
	} else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
}
EXPORT_SYMBOL(mmc_wimax_set_carddetect);

static unsigned int express_wimax_type = MMC_TYPE_SDIO_WIMAX;

static struct mmc_platform_data mmc_wimax_data = {
	.ocr_mask		= MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30,
	.status			= mmc_wimax_status,
	.register_status_notify	= mmc_wimax_status_register,
	.embedded_sdio		= NULL,
	//.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	//.msmsdcc_fmin   = 400000,
	//.msmsdcc_fmid   = 24000000,
	//.msmsdcc_fmax   = 48000000,
	//.nonremovable   = 1,
	.slot_type		= &express_wimax_type,
	.dummy52_required = 1,
};

struct _vreg
{
	const char *name;
	unsigned id;
};

int mmc_wimax_power(int on)
{
	printk("%s on=(%d)\n", __func__,on);

	if (on) {
	/* Power ON sequence */
		wimax_on_pmic_gpio();
		/* Step 1:Set PVDD output high */
		gpio_set_value(EXPRESS_GPIO_V_WIMAX_PVDD_EN, 1);  /* V_WIMAX_PVDD_EN */
		/* Step 2:delay 10 ms */
		mdelay(10);
		/* Step 3:Set DVDD output high */
		gpio_set_value(EXPRESS_GPIO_V_WIMAX_DVDD_EN, 1);  /* V_WIMAX_DVDD_EN */
		/* Step 4:delay 3 ms */
		mdelay(3);
		/* Step 5:Set 1V2_RD_EN output high */
		gpio_set_value(EXPRESS_GPIO_V_WIMAX_1V2_RF_EN, 1);  /* V_WIMAX_1V2_RF_EN */
		/* Step 6:delay 130 ms */
		mdelay(130);
		/* Step 7:SDIO/GPIO configuration */
		 config_gpio_table(wimax_on_gpio_table,
			  ARRAY_SIZE(wimax_on_gpio_table));

		/* Step 8:delay 3 ms */
		mdelay(3);
		/* Step 9:Set WIMAX_RST output high */
		gpio_set_value(EXPRESS_GPIO_WIMAX_EXT_RST, 1);  /* WIMAX_EXT_RSTz */

	} else {

 	/*Power OFF sequence*/
		/* Step 1:Set WIMAX_RST output low */
		gpio_set_value(EXPRESS_GPIO_WIMAX_EXT_RST, 0);  /* WIMAX_EXT_RSTz */
		/* Step 2:SDIO/GPIO configuration */
		config_gpio_table(wimax_off_gpio_table,
			  ARRAY_SIZE(wimax_off_gpio_table));
		/* Step 3:delay 5 ms */
		mdelay(5);
		/* Step 4:Set 1V2_RD_EN output low */
		gpio_set_value(EXPRESS_GPIO_V_WIMAX_1V2_RF_EN, 0);  /* V_WIMAX_1V2_RF_EN */
		/* Step 5:delay 3 ms */
		mdelay(3);
		/* Step 6:Set DVDD output low */
		gpio_set_value(EXPRESS_GPIO_V_WIMAX_DVDD_EN, 0);  /* V_WIMAX_DVDD_EN */
		/* Step 7:delay 3 ms */
		mdelay(3);
		/* Step 8:Set PVDD output low */
		gpio_set_value(EXPRESS_GPIO_V_WIMAX_PVDD_EN, 0);  /* V_WIMAX_PVDD_EN */

	}
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_power);

#ifdef CONFIG_WIMAX_SERIAL_MSM
static uint32_t wimax_uart_on_gpio_table[] = {
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_UART_SIN,  1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_UART_SOUT, 1, GPIO_INPUT,  GPIO_PULL_UP, GPIO_4MA),
};

static uint32_t wimax_uart_off_gpio_table[] = {	
	//PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_UART_SIN,  0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),
	//PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_UART_SOUT, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_4MA),
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_UART_SIN,  1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	PCOM_GPIO_CFG(EXPRESS_GPIO_WIMAX_UART_SOUT, 1, GPIO_INPUT,  GPIO_PULL_UP, GPIO_4MA),
};
#endif

int wimax_uart_switch = 0;
int mmc_wimax_uart_switch(int uart)
{
#ifdef CONFIG_WIMAX_SERIAL_MSM
	int ret=0;
#endif
	printk("%s uart:%d\n", __func__, uart);
	wimax_uart_switch = uart;

	if(uart == 0) { // Enable CPU wimax UART without microUSB
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_CPU_WIMAX_SW), uart?1:0);
	}else if (uart == 1) { // Disable CPU wimax UART with microUSB
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_CPU_WIMAX_SW), uart?1:0);
	}
#ifdef CONFIG_WIMAX_SERIAL_MSM
	else if (uart == 8) {
		printk(KERN_INFO "%s config wimax uart gpio on\n", __func__);
		config_gpio_table(wimax_uart_on_gpio_table,
			  ARRAY_SIZE(wimax_uart_on_gpio_table));	
	}else if (uart == 9) {
		printk(KERN_INFO "%s config wimax uart gpio off\n", __func__);
		config_gpio_table(wimax_uart_off_gpio_table,
			  ARRAY_SIZE(wimax_uart_off_gpio_table));
	}
	else if (wimax_uart_switch == 10 ) { // WIMAX UART debug thread.

		printk(KERN_INFO "%s config wimax uart gpio on\n", __func__);
		config_gpio_table(wimax_uart_on_gpio_table,
			  ARRAY_SIZE(wimax_uart_on_gpio_table));
    
		printk("%s : wimax_uart_switch %d\n", __func__, wimax_uart_switch);
		msm_serial_wimax_init(MSM_UART3_PHYS,MSM_UART3_PHYS, INT_UART3,
		&msm_device_uart3.dev, 23, MSM_GPIO_TO_INT(EXPRESS_GPIO_WIMAX_UART_SOUT));	
		
	}
	else if (wimax_uart_switch > 10 && wimax_uart_switch <=99) { // WIMAX UART debug thread.
		printk("%s : wimax_uart_switch %d\n", __func__, wimax_uart_switch);
		ret = msm_serial_wimax_thread(wimax_uart_switch);
		if(ret!=0)
		printk("%s : wimax_uart_switch ret=%d\n", __func__, ret);
	}
#endif
	return 0; 
}
EXPORT_SYMBOL(mmc_wimax_uart_switch);

int mmc_wimax_get_uart_switch(void)
{
	printk("%s uart:%d\n", __func__, wimax_uart_switch);
	return wimax_uart_switch?1:0; 	
}
EXPORT_SYMBOL(mmc_wimax_get_uart_switch);

int mmc_wimax_get_hostwakeup_gpio(void)
{
	return mmc_wimax_hostwakeup_gpio;
}
EXPORT_SYMBOL(mmc_wimax_get_hostwakeup_gpio);

int mmc_wimax_get_hostwakeup_IRQ_ID(void)
{
	return MSM_GPIO_TO_INT(mmc_wimax_get_hostwakeup_gpio());
}
EXPORT_SYMBOL(mmc_wimax_get_hostwakeup_IRQ_ID);

void mmc_wimax_enable_host_wakeup(int on)
{
	if (mmc_wimax_get_status())
	{	
		if (on) {
			if (!mmc_wimax_get_gpio_irq_enabled()) {
				printk("set GPIO%d as wakeup source on IRQ %d\n", mmc_wimax_get_hostwakeup_gpio(),mmc_wimax_get_hostwakeup_IRQ_ID());
				enable_irq(mmc_wimax_get_hostwakeup_IRQ_ID());
				enable_irq_wake(mmc_wimax_get_hostwakeup_IRQ_ID());
				mmc_wimax_set_gpio_irq_enabled(1);
			}
		}
		else {
			if (mmc_wimax_get_gpio_irq_enabled()) {
				printk("disable GPIO%d wakeup source\n", mmc_wimax_get_hostwakeup_gpio());
				disable_irq_wake(mmc_wimax_get_hostwakeup_IRQ_ID());				
				disable_irq_nosync(mmc_wimax_get_hostwakeup_IRQ_ID());
				mmc_wimax_set_gpio_irq_enabled(0);
			}
		}
	}
	else {
		printk("%s mmc_wimax_sdio_status is OFF\n", __func__);
	}
}
EXPORT_SYMBOL(mmc_wimax_enable_host_wakeup);

int __init express_init_mmc(unsigned int sys_rev)
{
	uint32_t id;
	wifi_status_cb = NULL;
	sdslot_vreg_enabled = 0;

	printk(KERN_INFO "%s\n", __func__);

	/* SDC1: Initial WiMAX */
	msm_add_sdcc(1, &mmc_wimax_data,0,0);	

	/* SDC2: MoviNAND */
	config_gpio_table(movinand_on_gpio_table,
			  ARRAY_SIZE(movinand_on_gpio_table));
	msm_add_sdcc(2, &express_movinand_data, 0, 0);

	/* initial WIFI_SHUTDOWN# */
	id = PCOM_GPIO_CFG(EXPRESS_GPIO_WIFI_EN, 0, GPIO_OUTPUT,
		GPIO_NO_PULL, GPIO_2MA),
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	gpio_set_value(EXPRESS_GPIO_WIFI_EN, 0);

	msm_add_sdcc(3, &express_wifi_data, 0, 0);

	if (opt_disable_sdcard) {
		printk(KERN_INFO "%s: SD-Card interface disabled\n", __func__);
		goto done;
	}

	vreg_sdslot = vreg_get(0, "gp10");
	if (IS_ERR(vreg_sdslot))
		return PTR_ERR(vreg_sdslot);

	set_irq_wake(MSM_GPIO_TO_INT(EXPRESS_SDMC_CD_N_TO_SYS), 1);

	msm_add_sdcc(4, &express_sdslot_data,
			MSM_GPIO_TO_INT(EXPRESS_SDMC_CD_N_TO_SYS),
			IORESOURCE_IRQ_LOWEDGE | IORESOURCE_IRQ_HIGHEDGE);
	printk(KERN_INFO "%s: %d\n", __func__, EXPRESS_SDMC_CD_N_TO_SYS);

done:

	return 0;
}

