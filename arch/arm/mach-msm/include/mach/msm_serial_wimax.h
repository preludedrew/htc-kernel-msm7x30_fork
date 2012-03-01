/*
 * Copyright (C) 2009 Google, Inc.
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

//#ifndef __ASM_ARCH_MSM_SERIAL_WIMAX_H
//#define __ASM_ARCH_MSM_SERIAL_WIMAX_H

void msm_serial_wimax_init(unsigned int uart_base,unsigned int gsbi_base, int irq,
			   struct device *clk_device, int signal_irq, int wakeup_irq);
int msm_serial_wimax_thread(int num);


//#endif
