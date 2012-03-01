/* linux/arch/arm/mach-msm/board-express.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_EXPRESS_H
#define __ARCH_ARM_MACH_MSM_BOARD_EXPRESS_H

#include <mach/board.h>

#define EXPRESS_PROJECT_NAME	"express"

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define MSM_LINUX_BASE1		0x05000000
#define MSM_LINUX_SIZE1		0x0B000000
#define MSM_LINUX_BASE2		0x20000000
#define MSM_LINUX_SIZE2		0x0FB00000
#define MSM_LINUX_BASE3		0x40000000
#define MSM_LINUX_SIZE3		0x10000000
#define MSM_LINUX_BASE4		0x60000000
#define MSM_LINUX_SIZE4		0x08800000
#define MSM_MEM_256MB_OFFSET	0x10000000

#define MSM_FB1_BASE		0x2FB00000
#define MSM_FB1_SIZE		0x00500000

#define MSM_GPU_MEM_BASE	0x68800000
#define MSM_GPU_MEM_SIZE	0x00500000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE    0x000F0000

/* CIQ */
#ifdef CONFIG_BUILD_CIQ
#define MSM_PMEM_CIQ_BASE   MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE
#define MSM_PMEM_CIQ_SIZE   SZ_64K
#define MSM_PMEM_CIQ1_BASE  MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ1_SIZE  MSM_PMEM_CIQ_SIZE
#define MSM_PMEM_CIQ2_BASE  MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ2_SIZE  MSM_PMEM_CIQ_SIZE
#define MSM_PMEM_CIQ3_BASE  MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ3_SIZE  MSM_PMEM_CIQ_SIZE
#endif

#define MSM_PMEM_ADSP_BASE  	0x68D00000
#define MSM_PMEM_ADSP_SIZE	0x02800000
#define PMEM_KERNEL_EBI1_BASE   0x6B500000
#define PMEM_KERNEL_EBI1_SIZE   0x00600000

#define MSM_PMEM_CAMERA_BASE	0x6BB00000
#define MSM_PMEM_CAMERA_SIZE	0x00000000

#define MSM_PMEM_MDP_BASE	0x6BB00000
#define MSM_PMEM_MDP_SIZE	0x04000000

#define MSM_FB_BASE		0x6FB00000
#define MSM_FB_SIZE		0x00500000

/* GPIO definition */

/* Direct Keys */
#define EXPRESS_GPIO_KEYPAD_POWER_KEY  (46)

/* Battery */
#define EXPRESS_GPIO_ADP_9V            (33)

/* Wifi */
#define EXPRESS_GPIO_WIFI_IRQ          (147)
#define EXPRESS_GPIO_WIFI_EN           (133)

#define EXPRESS_GPIO_UART2_RX          (51)
#define EXPRESS_GPIO_UART2_TX          (52)

/* WiMax */
#define EXPRESS_GPIO_WIMAX_UART_SIN        (54)
#define EXPRESS_GPIO_WIMAX_UART_SOUT       (53)
#define EXPRESS_GPIO_V_WIMAX_1V2_RF_EN     (150)
#define EXPRESS_GPIO_WIMAX_EXT_RST         (143)
#define EXPRESS_GPIO_V_WIMAX_DVDD_EN       (154)
#define EXPRESS_GPIO_V_WIMAX_PVDD_EN       (36)
#define EXPRESS_GPIO_WIMAX_SDIO_D0         (43)
#define EXPRESS_GPIO_WIMAX_SDIO_D1         (42)
#define EXPRESS_GPIO_WIMAX_SDIO_D2         (41)
#define EXPRESS_GPIO_WIMAX_SDIO_D3         (40)
#define EXPRESS_GPIO_WIMAX_SDIO_CMD        (39)
#define EXPRESS_GPIO_WIMAX_SDIO_CLK_CPU    (38)

/* Sensors */
#define EXPRESS_GPIO_COMPASS_INT       (42)

#define EXPRESS_COMPASS_LAYOUTS		{ \
	{ { 0,  1, 0}, {-1,  0,  0}, {0, 0,  1} }, \
	{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
	{ {-1,  0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
	{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}

/* Microp */
#define EXPRESS_GPIO_UP_RESET_N        PMGPIO(10)
#define EXPRESS_GPIO_UP_INT_N          PMGPIO(26)

/* TP */
#define EXPRESS_GPIO_TP_ATT_N		(26)
#define EXPRESS_GPIO_TP_3V3_ENABLE	(55)
#define EXPRESS_GPIO_SPI_ENABLE		(83)



/* LCD */
#define EXPRESS_LCD_PCLK			(90)
#define EXPRESS_LCD_DE                    (91)
#define EXPRESS_LCD_VSYNC                 (92)
#define EXPRESS_LCD_HSYNC                 (93)

#define EXPRESS_LCD_G0                    (18)
#define EXPRESS_LCD_G1                    (19)
#define EXPRESS_LCD_G2                    (94)
#define EXPRESS_LCD_G3                    (95)
#define EXPRESS_LCD_G4                    (96)
#define EXPRESS_LCD_G5                    (97)
#define EXPRESS_LCD_G6                    (98)
#define EXPRESS_LCD_G7                    (99)

#define EXPRESS_LCD_B0                    (20)
#define EXPRESS_LCD_B1                    (21)
#define EXPRESS_LCD_B2                    (22)
#define EXPRESS_LCD_B3                    (100)
#define EXPRESS_LCD_B4                    (101)
#define EXPRESS_LCD_B5                    (102)
#define EXPRESS_LCD_B6                    (103)
#define EXPRESS_LCD_B7                    (104)

#define EXPRESS_LCD_R0                    (23)
#define EXPRESS_LCD_R1                    (24)
#define EXPRESS_LCD_R2                    (25)
#define EXPRESS_LCD_R3                    (105)
#define EXPRESS_LCD_R4                    (106)
#define EXPRESS_LCD_R5                    (107)
#define EXPRESS_LCD_R6                    (108)
#define EXPRESS_LCD_R7                    (109)
#define EXPRESS_LCM_3V3_EN                (181)
#define EXPRESS_LVDS_ON                   (80)
#define EXPRESS_LCM_ID			  		  (79)

/* DTV */
#define EXPRESS_DTV_PCLK			(124)
#define EXPRESS_DTV_DE                    (125)
#define EXPRESS_DTV_VSYNC                 (126)
#define EXPRESS_DTV_HSYNC                 (127)

#define EXPRESS_DTV_G0                    (128)
#define EXPRESS_DTV_G1                    (129)
#define EXPRESS_DTV_G2                    (130)
#define EXPRESS_DTV_G3                    (131)
#define EXPRESS_DTV_G4                    (132)
#define EXPRESS_DTV_G5                    (160)
#define EXPRESS_DTV_G6                    (161)
#define EXPRESS_DTV_G7                    (162)

#define EXPRESS_DTV_B0                    (163)
#define EXPRESS_DTV_B1                    (164)
#define EXPRESS_DTV_B2                    (165)
#define EXPRESS_DTV_B3                    (166)
#define EXPRESS_DTV_B4                    (167)
#define EXPRESS_DTV_B5                    (168)
#define EXPRESS_DTV_B6                    (169)
#define EXPRESS_DTV_B7                    (170)

#define EXPRESS_DTV_R0                    (171)
#define EXPRESS_DTV_R1                    (172)
#define EXPRESS_DTV_R2                    (173)
#define EXPRESS_DTV_R3                    (174)
#define EXPRESS_DTV_R4                    (175)
#define EXPRESS_DTV_R5                    (176)
#define EXPRESS_DTV_R6                    (177)
#define EXPRESS_DTV_R7                    (178)

/* Audio */
#define EXPRESS_AUD_CODEC_RST          (34)
#define EXPRESS_AUD_MIC_BIAS           (56)
#define EXPRESS_AUD_A1026_INT          (120)
#define EXPRESS_AUD_A1026_RESET        (122)
#define EXPRESS_AUD_A1026_WAKEUP       (123)
#define EXPRESS_WCA_MCLK               (120)
#define EXPRESS_WCA_DATA_SD0           (121)
#define EXPRESS_WCA_DATA_SD1           (122)
#define EXPRESS_WCA_DATA_SD2           (123)
#define EXPRESS_WCF_I2S_WS             (144)
#define EXPRESS_WCF_I2S_CLK            (145)
#define EXPRESS_WCF_I2S_DATA           (146)

/* BT */
#define EXPRESS_GPIO_BT_HOST_WAKE      (44)
#define EXPRESS_GPIO_BT_CHIP_WAKE      (50)
#define EXPRESS_GPIO_BT_UART1_RTS      (134)
#define EXPRESS_GPIO_BT_UART1_CTS      (135)
#define EXPRESS_GPIO_BT_UART1_RX       (136)
#define EXPRESS_GPIO_BT_UART1_TX       (137)
#define EXPRESS_GPIO_BT_PCM_OUT        (138)
#define EXPRESS_GPIO_BT_PCM_IN         (139)
#define EXPRESS_GPIO_BT_PCM_SYNC       (140)
#define EXPRESS_GPIO_BT_PCM_CLK        (141)
#define EXPRESS_GPIO_BT_SHUTDOWN_N     (148)

/* USB */
#define EXPRESS_GPIO_USB_ID2_PIN       (86)
#define EXPRESS_GPIO_USB_MHL_SEL       (56)
#define EXPRESS_GPIO_MHL_RESET         (89)

/* Camera */

#define EXPRESS_CAM_PWD			 	 (37)
#define EXPRESS_CAM_RST              (30)
#define EXPRESS_CLK_SWITCH     PMGPIO(4)/* EXPRESS_CLK_SWITCH Pin Express*/
#define EXPRESS_CAM2_PWD       PMGPIO(3)/* EXPRESS_CAM2_PWD Pin Express*/
#define EXPRESS_CAM2_RST               (31)

/* MHL */
#define EXPRESS_MHL_RSTz               (49)
#define EXPRESS_MHL_INT	               (180)
#define EXPRESS_MHL_3V3_EN             (35)
#define EXPRESS_MHL_SW                 (56)
#define EXPRESS_MHL_I2C_TPI            (0x72)
#define EXPRESS_MHL_I2C_CBUS           (0xC8)

#define EXPRESS_SPI_CLK                (45)
#define EXPRESS_SPI_DO                 (47)
#define EXPRESS_SPI_DI                 (48)
#define EXPRESS_SPI_CS2                (87)

/* Accessory */
#define EXPRESS_H2W_IO1_CLK		(52)
#define EXPRESS_H2W_IO2_DAT		(51)

/* PMIC */
#define PMIC_GPIO_INT                (27)
#define EXPRESS_GPIO_PS_HOLD           (29)

/* PMIC GPIO definition */
#define PMGPIO(x) (x-1)
#define EXPRESS_WIMAX_HOST_WAKEUP         PMGPIO(8)
#define EXPRESS_MCHG_EN_N	 PMGPIO(11)
#define EXPRESS_AUD_MICPATH_SEL  PMGPIO(12)
#define EXPRESS_ISET		 PMGPIO(14)
#define EXPRESS_AUD_SPK_EN       PMGPIO(16)
#define EXPRESS_CSA_XRES         PMGPIO(17)
#define EXPRESS_MHL_SW_XB         PMGPIO(18)
#define EXPRESS_AUD_HP_EN        PMGPIO(19)
#define EXPRESS_CSA_INTz         PMGPIO(20)
#define EXPRESS_GPIO_USB_ID_PIN  PMGPIO(21)
#define EXPRESS_LED_3V3_EN       PMGPIO(22)
#define EXPRESS_SDMC_CD_N        PMGPIO(23)
#define EXPRESS_VOL_DN           PMGPIO(24)
#define EXPRESS_SLOW_CHG         PMGPIO(31)
#define EXPRESS_9V_AC_DETECT           PMGPIO(32)
#define EXPRESS_VOL_UP           PMGPIO(25)
#define EXPRESS_CPU_WIMAX_SW     PMGPIO(35)
#define EXPRESS_COMPASS_INT	PMGPIO(36)
#define EXPRESS_GPIO_WIFI_BT_SLEEP_CLK_EN	PMGPIO(38)
#define EXPRESS_SLEEP_CLK2       PMGPIO(39)
#define EXPRESS_TP_ATT_PMIC	   PMGPIO(1)
#define EXPRESS_TP_ATT		   PMGPIO(2)
#define EXPRESS_AUD_REMO_EN	PMGPIO(5)
#define EXPRESS_AUD_REMO_PRES	PMGPIO(13)
#define EXPRESS_AUD_HP_DETz	PMGPIO(15)
#define EXPRESS_H2W_CABLE_IN1	PMGPIO(9)
#define EXPRESS_H2W_CABLE_IN2	PMGPIO(33)

/* PMIC MPP definition */
#define PMMPP(x) (x-1)
#define EXPRESS_EXT_3V3_EN		PMMPP(3)
#define EXPRESS_BTRST	PMMPP(4)
#define EXPRESS_WIMAX_DEBUG12	PMMPP(10)

#ifdef CONFIG_MICROP_COMMON
void __init express_microp_init(void);
#endif
int express_init_mmc(unsigned int sys_rev);
void __init express_audio_init(void);
int __init express_init_keypad(void);
int __init express_wifi_init(void);
int __init express_init_panel(void);
#ifdef CONFIG_WIMAX_SERIAL_MSM
void express_dump_uart_reg(int r);
#endif
#endif /* __ARCH_ARM_MACH_MSM_BOARD_EXPRESS_H */
