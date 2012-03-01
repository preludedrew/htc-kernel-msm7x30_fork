/* linux/arch/arm/mach-msm/board-express-audio.c
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
 *
 */

#include <linux/android_pmem.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/delay.h>

#include <mach/gpio.h>
#include <mach/dal.h>
#include "board-express.h"
#include <mach/qdsp5v2/snddev_icodec.h>
#include <mach/qdsp5v2/snddev_ecodec.h>
#include <mach/qdsp5v2/audio_def.h>
#include <mach/qdsp5v2/voice.h>
#include <mach/htc_acoustic_7x30.h>
#include <linux/spi/spi_aic3254.h>
#include <mach/qdsp5v2/snddev_mi2s.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>

static struct mutex bt_sco_lock;
static struct mutex mic_lock;
static int curr_rx_mode;
static atomic_t aic3254_ctl = ATOMIC_INIT(0);
void express_back_mic_enable(int);

#define BIT_SPEAKER	(1 << 0)
#define BIT_HEADSET	(1 << 1)
#define BIT_RECEIVER	(1 << 2)
#define BIT_FM_SPK	(1 << 3)
#define BIT_FM_HS	(1 << 4)

static struct q5v2_hw_info q5v2_audio_hw[Q5V2_HW_COUNT] = {
	[Q5V2_HW_HANDSET] = {
		.max_gain[VOC_NB_INDEX] = 400,
		.min_gain[VOC_NB_INDEX] = -1600,
		.max_gain[VOC_WB_INDEX] = 400,
		.min_gain[VOC_WB_INDEX] = -1600,
	},
	[Q5V2_HW_HEADSET] = {
		.max_gain[VOC_NB_INDEX] = 900,
		.min_gain[VOC_NB_INDEX] = -1100,
		.max_gain[VOC_WB_INDEX] = 900,
		.min_gain[VOC_WB_INDEX] = -1100,
	},
	[Q5V2_HW_SPEAKER] = {
		.max_gain[VOC_NB_INDEX] = 1000,
		.min_gain[VOC_NB_INDEX] = -500,
		.max_gain[VOC_WB_INDEX] = 1000,
		.min_gain[VOC_WB_INDEX] = -500,
	},
	[Q5V2_HW_BT_SCO] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = -1500,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = -1500,
	},
	[Q5V2_HW_TTY] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = 0,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = 0,
	},
	[Q5V2_HW_HS_SPKR] = {
		.max_gain[VOC_NB_INDEX] = -500,
		.min_gain[VOC_NB_INDEX] = -2000,
		.max_gain[VOC_WB_INDEX] = -500,
		.min_gain[VOC_WB_INDEX] = -2000,
	},
	[Q5V2_HW_USB_HS] = {
		.max_gain[VOC_NB_INDEX] = 1000,
		.min_gain[VOC_NB_INDEX] = -500,
		.max_gain[VOC_WB_INDEX] = 1000,
		.min_gain[VOC_WB_INDEX] = -500,
	},
	[Q5V2_HW_HAC] = {
		.max_gain[VOC_NB_INDEX] = 1000,
		.min_gain[VOC_NB_INDEX] = -500,
		.max_gain[VOC_WB_INDEX] = 1000,
		.min_gain[VOC_WB_INDEX] = -500,
	},
};

static unsigned aux_pcm_gpio_off[] = {
	PCOM_GPIO_CFG(EXPRESS_GPIO_BT_PCM_OUT, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(EXPRESS_GPIO_BT_PCM_IN, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(EXPRESS_GPIO_BT_PCM_SYNC, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(EXPRESS_GPIO_BT_PCM_CLK, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
};

static unsigned aux_pcm_gpio_on[] = {
	PCOM_GPIO_CFG(EXPRESS_GPIO_BT_PCM_OUT, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(EXPRESS_GPIO_BT_PCM_IN, 1, GPIO_INPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(EXPRESS_GPIO_BT_PCM_SYNC, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(EXPRESS_GPIO_BT_PCM_CLK, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
};

static unsigned mi2s_clk_gpios_on[] = {
        PCOM_GPIO_CFG(EXPRESS_WCF_I2S_CLK, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
        PCOM_GPIO_CFG(EXPRESS_WCF_I2S_WS, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
        PCOM_GPIO_CFG(EXPRESS_WCA_MCLK, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static unsigned mi2s_clk_gpios_off[] = {
        PCOM_GPIO_CFG(EXPRESS_WCF_I2S_CLK, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
        PCOM_GPIO_CFG(EXPRESS_WCF_I2S_WS, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
        PCOM_GPIO_CFG(EXPRESS_WCA_MCLK, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
};

static unsigned mi2s_rx_data_lines_gpios_on[] = {
        PCOM_GPIO_CFG(EXPRESS_WCA_DATA_SD0, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
        PCOM_GPIO_CFG(EXPRESS_WCA_DATA_SD1, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
        PCOM_GPIO_CFG(EXPRESS_WCA_DATA_SD2, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
        PCOM_GPIO_CFG(EXPRESS_WCF_I2S_DATA, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static unsigned mi2s_rx_data_lines_gpios_off[] = {
        PCOM_GPIO_CFG(EXPRESS_WCA_DATA_SD0, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
        PCOM_GPIO_CFG(EXPRESS_WCA_DATA_SD1, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
        PCOM_GPIO_CFG(EXPRESS_WCA_DATA_SD2, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
        PCOM_GPIO_CFG(EXPRESS_WCF_I2S_DATA, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
};

void express_audio_2v85_enable(int en)
{
	struct vreg *vreg_2v85;
	int ret;
	vreg_2v85 = vreg_get(NULL, "gp7");

	if (IS_ERR(vreg_2v85)) {
		pr_aud_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "vreg_2v85", PTR_ERR(vreg_2v85));
		return;
	}

	if (en) {
		ret = vreg_enable(vreg_2v85);
	} else {
		ret = vreg_disable(vreg_2v85);
	}
	vreg_put(vreg_2v85);
	pr_aud_info("%s %d ret %d\n",__func__, en, ret);
}

void express_snddev_poweramp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		msleep(60);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_AUD_SPK_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_SPEAKER;
		mdelay(5);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_AUD_SPK_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_SPEAKER;
	}
}

void express_snddev_hsed_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		msleep(60);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_AUD_HP_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_HEADSET;
		mdelay(5);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_AUD_HP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_HEADSET;
	}
}

void express_snddev_hs_spk_pamp_on(int en)
{
	express_snddev_poweramp_on(en);
	express_snddev_hsed_pamp_on(en);
}

void express_snddev_receiver_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
#if 0
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_AUD_EP_EN), 1);
		mdelay(5);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_RECEIVER;
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_AUD_EP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_RECEIVER;
	}
#endif
}

void express_snddev_bt_sco_pamp_on(int en)
{
	static int bt_sco_refcount;
	pr_aud_info("%s %d\n", __func__, en);
	mutex_lock(&bt_sco_lock);
	if (en) {
		if (++bt_sco_refcount == 1) {
			config_gpio_table(aux_pcm_gpio_on,
					ARRAY_SIZE(aux_pcm_gpio_on));
		}
	} else {
		if (--bt_sco_refcount == 0) {
			config_gpio_table(aux_pcm_gpio_off,
					ARRAY_SIZE(aux_pcm_gpio_off));
			gpio_set_value(EXPRESS_GPIO_BT_PCM_OUT, 0);
			gpio_set_value(EXPRESS_GPIO_BT_PCM_SYNC, 0);
			gpio_set_value(EXPRESS_GPIO_BT_PCM_CLK, 0);
		}
	}
	mutex_unlock(&bt_sco_lock);
}

void express_snddev_usb_headset_on(int en)
{
	struct vreg *vreg_ncp;
	int ret;

	vreg_ncp = vreg_get(NULL, "ncp");
	if (IS_ERR(vreg_ncp)) {
		pr_aud_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(vreg_ncp));
		return;
	}

	if (en) {
		ret = vreg_enable(vreg_ncp);
	} else {
		ret = vreg_disable(vreg_ncp);
	}
	vreg_put(vreg_ncp);
	pr_aud_info("%s %d ret %d\n",__func__, en, ret);
}

/* power up internal/externnal mic shared GPIO */
void express_mic_bias_enable(int en, int shift)
{
	pr_aud_info("%s: %d system_rev = 0x%x\n", __func__, en, system_rev);

	if (en) {
		if(system_rev >= 0x80)
			express_audio_2v85_enable(en);

		pmic_hsed_enable(PM_HSED_CONTROLLER_2, PM_HSED_ENABLE_ALWAYS);
	} else {
		pmic_hsed_enable(PM_HSED_CONTROLLER_2, PM_HSED_ENABLE_OFF);

		if(system_rev >= 0x80)
			express_audio_2v85_enable(en);
	}
}

void express_snddev_imic_pamp_on(int en)
{
	pr_aud_info("%s: %d\n", __func__, en);

	if (en) {
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_ALWAYS);
		express_back_mic_enable(1);
	} else {
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);
		express_back_mic_enable(0);
	}
}

void express_snddev_emic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_AUD_MICPATH_SEL), 1);
	} else
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_AUD_MICPATH_SEL), 0);
}

void express_back_mic_enable(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_AUD_MICPATH_SEL),
			0);
		pmic_hsed_enable(PM_HSED_CONTROLLER_1, PM_HSED_ENABLE_ALWAYS);
	} else {
		pmic_hsed_enable(PM_HSED_CONTROLLER_1, PM_HSED_ENABLE_OFF);
	}
}

void express_snddev_fmspk_pamp_on(int en)
{
	if (en) {
		msleep(30);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_AUD_SPK_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_SPK;
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_AUD_SPK_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_SPK;
	}
}

void express_snddev_fmhs_pamp_on(int en)
{
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_AUD_HP_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_HS;
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(EXPRESS_AUD_HP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_HS;
	}
}

int express_get_rx_vol(uint8_t hw, int network, int level)
{
	struct q5v2_hw_info *info;
	int vol, maxv, minv;

	info = &q5v2_audio_hw[hw];
	maxv = info->max_gain[network];
	minv = info->min_gain[network];
	vol = minv + ((maxv - minv) * level) / 100;
	pr_aud_info("%s(%d, %d, %d) => %d\n", __func__, hw, network, level, vol);
	return vol;
}

void express_rx_amp_enable(int en)
{
	if (curr_rx_mode != 0) {
		atomic_set(&aic3254_ctl, 1);
		pr_aud_info("%s: curr_rx_mode 0x%x, en %d\n",
			__func__, curr_rx_mode, en);
		if (curr_rx_mode & BIT_SPEAKER)
			express_snddev_poweramp_on(en);
		if (curr_rx_mode & BIT_HEADSET)
			express_snddev_hsed_pamp_on(en);
		if (curr_rx_mode & BIT_RECEIVER)
			express_snddev_receiver_pamp_on(en);
		if (curr_rx_mode & BIT_FM_SPK)
			express_snddev_fmspk_pamp_on(en);
		if (curr_rx_mode & BIT_FM_HS)
			express_snddev_fmhs_pamp_on(en);
		atomic_set(&aic3254_ctl, 0);;
	}
}

int express_mi2s_clk_enable(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		config_gpio_table(mi2s_clk_gpios_on,
				ARRAY_SIZE(mi2s_clk_gpios_on));
	} else {
		config_gpio_table(mi2s_clk_gpios_off,
				ARRAY_SIZE(mi2s_clk_gpios_off));
	}
	return 0;
}

int express_mi2s_data_enable(int en, u32 direction, u8 sd_line_mask)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		config_gpio_table(mi2s_rx_data_lines_gpios_on,
				ARRAY_SIZE(mi2s_rx_data_lines_gpios_on));
	} else {
		config_gpio_table(mi2s_rx_data_lines_gpios_on,
				ARRAY_SIZE(mi2s_rx_data_lines_gpios_off));
	}
	return 0;
}

static struct q5v2audio_mi2s_ops mi2sops = {
	.mi2s_clk_enable = express_mi2s_clk_enable,
	.mi2s_data_enable = express_mi2s_data_enable,
};

int express_support_aic3254(void)
{
	return 1;
}

int express_support_back_mic(void)
{
	return 1;
}

int express_support_receiver(void)
{
	return 0;
}
/*
void express_get_acoustic_tables(struct acoustic_tables *tb){
	strcpy(tb->aic3254, "IOTable.txt\0");
}
*/
static struct q5v2audio_analog_ops ops = {
	.speaker_enable	= express_snddev_poweramp_on,
	.headset_enable	= express_snddev_hsed_pamp_on,
	.handset_enable	= express_snddev_receiver_pamp_on,
	.headset_speaker_enable	= express_snddev_hs_spk_pamp_on,
	.bt_sco_enable	= express_snddev_bt_sco_pamp_on,
	.usb_headset_enable = express_snddev_usb_headset_on,
	.int_mic_enable = express_snddev_imic_pamp_on,
	.ext_mic_enable = express_snddev_emic_pamp_on,
	.fm_headset_enable = express_snddev_fmhs_pamp_on,
	.fm_speaker_enable = express_snddev_fmspk_pamp_on,
};

static struct q5v2audio_ecodec_ops eops = {
	.bt_sco_enable  = express_snddev_bt_sco_pamp_on,
};

static struct q5v2voice_ops vops = {
	.get_rx_vol = express_get_rx_vol,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = express_mic_bias_enable,
	.support_aic3254 = express_support_aic3254,
	.support_back_mic = express_support_back_mic,
	.support_receiver = express_support_receiver,
	.enable_back_mic =  express_back_mic_enable,
/*
	.get_acoustic_tables = express_get_acoustic_tables,
*/
};

static struct aic3254_ctl_ops cops = {
	.rx_amp_enable = express_rx_amp_enable,
};

void __init express_audio_init(void)
{
	struct pm8058_gpio audio_pwr = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
		.vin_sel        = 6,
	};

	mutex_init(&bt_sco_lock);
	mutex_init(&mic_lock);

#ifdef CONFIG_MSM7KV2_AUDIO
	htc_7x30_register_analog_ops(&ops);
	htc_7x30_register_ecodec_ops(&eops);
	htc_7x30_register_voice_ops(&vops);
	htc_7x30_register_mi2s_ops(&mi2sops);
	acoustic_register_ops(&acoustic);
#endif
        aic3254_register_ctl_ops(&cops);

	/* Init PMIC GPIO */
	pm8058_gpio_config(EXPRESS_AUD_SPK_EN, &audio_pwr);
	pm8058_gpio_config(EXPRESS_AUD_HP_EN, &audio_pwr);
	pm8058_gpio_config(EXPRESS_AUD_MICPATH_SEL, &audio_pwr);

	/* Rest AIC3254 */
	gpio_set_value(EXPRESS_AUD_CODEC_RST, 0);
	mdelay(1);
	gpio_set_value(EXPRESS_AUD_CODEC_RST, 1);

	mutex_lock(&bt_sco_lock);
	config_gpio_table(aux_pcm_gpio_off, ARRAY_SIZE(aux_pcm_gpio_off));
	gpio_set_value(EXPRESS_GPIO_BT_PCM_OUT, 0);
	gpio_set_value(EXPRESS_GPIO_BT_PCM_SYNC, 0);
	gpio_set_value(EXPRESS_GPIO_BT_PCM_CLK, 0);
	mutex_unlock(&bt_sco_lock);
}
