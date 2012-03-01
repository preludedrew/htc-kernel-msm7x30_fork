/*
 * drivers/serial/msm_serial_debuger.c
 *
 * Serial Debugger Interface for MSM7K
 *
 * Copyright (C) 2008 Google, Inc.
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

#include <stdarg.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/console.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/kernel_debugger.h>
#include <linux/kernel_stat.h>
#include <linux/irq.h>
#include <linux/delay.h>

#include <mach/system.h>
#include <mach/fiq.h>

#include <mach/msm_serial_wimax.h>
#include "msm_serial.h"
#include "msm_serial_hs_hwreg.h"
#include <mach/msm_iomap.h>

static unsigned int debug_port_base;
static unsigned int uart_clk_port_base;
static unsigned int gsbi_port_base;
static struct clk *debug_clk;

#define MAXRXCOUNT 32

static inline void wimax_msm_write(unsigned int val, unsigned int off)
{
	__raw_writel(val, debug_port_base + off);
}

static inline unsigned int wimax_msm_read(unsigned int off)
{
	return __raw_readl(debug_port_base + off);
}

static inline void wimax_msm_write_clk(unsigned int val, unsigned int off)
{
	__raw_writel(val, uart_clk_port_base + off);
}

static inline unsigned int wimax_msm_read_clk(unsigned int off)
{
	return __raw_readl(uart_clk_port_base + off);
}

int msm_serial_wimax_dump_register(void)
{
	unsigned char tempc;
	printk("%s =========================================  +\n", __func__);		
	tempc = wimax_msm_read(UART_MR1);
	printk("%s:  UART_MR1=%x+ \n", __func__,tempc);
	tempc = wimax_msm_read(UART_MR2);
	printk("%s:  UART_MR2=%x+ \n", __func__,tempc);	
	tempc = wimax_msm_read(UART_IPR);
	printk("%s:  UART_IPR=%x+ \n", __func__,tempc);	
	tempc = wimax_msm_read(UART_TFWR);
	printk("%s:  UART_TFWR=%x+ \n", __func__,tempc);	
	tempc = wimax_msm_read(UART_RFWR);
	printk("%s:  UART_RFWR=%x+ \n", __func__,tempc);	
	tempc = wimax_msm_read(UART_HCR);
	printk("%s:  UART_HCR=%x+ \n", __func__,tempc);
	printk("%s ---------------------------------------------- \n", __func__);		
	tempc = wimax_msm_read(UART_MREG);
	printk("%s:  UART_MREG=%x+ \n", __func__,tempc);	
	tempc = wimax_msm_read(UART_NREG);
	printk("%s:  UART_NREG=%x+ \n", __func__,tempc);	
	tempc = wimax_msm_read(UART_DREG);
	printk("%s:  UART_DREG=%x+ \n", __func__,tempc);	
	tempc = wimax_msm_read(UART_MNDREG);
	printk("%s:  UART_MNDREG=%x+ \n", __func__,tempc);	
	tempc = wimax_msm_read(UART_SIM_CFG);
	printk("%s:  UART_SIM_CFG=%x+ \n", __func__,tempc);	
	printk("%s ---------------------------------------------- \n", __func__);	
	tempc = wimax_msm_read(UART_MISR_MODE);
	printk("%s:  UART_MISR_MODE=%x+ \n", __func__,tempc);	
	tempc = wimax_msm_read(UART_MISR_RESET);
	printk("%s:  UART_MISR_RESET=%x+ \n", __func__,tempc);
	tempc = wimax_msm_read(UART_MISR_EXPORT);
	printk("%s:  UART_MISR_EXPORT=%x+ \n", __func__,tempc);
	tempc = wimax_msm_read(UART_MISR_VAL);
	printk("%s:  UART_MISR_VAL=%x+ \n", __func__,tempc);
	tempc = wimax_msm_read(UART_TEST_CTRL);
	printk("%s:  UART_TEST_CTRL=%x+ \n", __func__,tempc);
	tempc = wimax_msm_read(UART_TEST_WR_ADDR);
	printk("%s:  UART_TEST_WR_ADDR=%x+ \n", __func__,tempc);
	tempc = wimax_msm_read(UART_TEST_RD_ADDR);
	printk("%s:  UART_TEST_RD_ADDR=%x+ \n", __func__,tempc);
	printk("%s ---------------------------------------------- \n", __func__);	
	tempc = wimax_msm_read(UART_SR);
	printk("%s:  UART_SR=%x+ \n", __func__,tempc);
	tempc = wimax_msm_read(UART_MISR);
	printk("%s:  UART_MISR=%x+ \n", __func__,tempc);
	tempc = wimax_msm_read(UART_ISR);
	printk("%s:  UART_ISR=%x+ \n", __func__,tempc);
	tempc = wimax_msm_read(UART_TEST_RD_DATA);
	printk("%s:  UART_TEST_RD_DATA=%x+ \n", __func__,tempc);	
	printk("%s =========================================  -\n", __func__);	

	return 0;
}

#define UART3_NS_REG_OFFSET 0x468

int msm_serial_wimax_uart_hw_init(void)
{
	unsigned char tempc;
	printk("%s + \n", __func__);
	
	tempc = wimax_msm_read_clk(UART3_NS_REG_OFFSET);
	printk("%s : uart_ns_reg=0x%x  \n", __func__,tempc);	
	wimax_msm_write_clk(0x30,UART3_NS_REG_OFFSET);  /* Open UART3 clock */


	wimax_msm_write(0x00,UART_SIM_CFG);
	/* Ensure that IRDA mode is disabled */
	//memset((void *)&pUartRegs[port]->uart_irda, 0, sizeof(pUartRegs[port]->uart_irda));
	wimax_msm_write(0x00,UART_IRDA);
	wimax_msm_write(0x00,UART_IPR);	

	/* drain anything in the UART */
	//pUartRegs[port]->uart_mreg.data = UART_MVR_V;
	//pUartRegs[port]->uart_nreg.uart_nreg = UART_NVR_V;
	//pUartRegs[port]->uart_dreg.data = UART_DVR_V;
	//pUartRegs[port]->uart_mndreg.dreg_lsb = D_VAL & 0x03;
	//pUartRegs[port]->uart_mndreg.nreg_lsb = N_MINUS_M_VAL & 0x07;
	//pUartRegs[port]->uart_mndreg.mreg_lsb = M_VAL & 0x01;
	/* setup clock dividers */
	
#if defined(CONFIG_ARCH_QSD8X50) || defined(CONFIG_ARCH_MSM7X30)
	if (clk_get_rate(debug_clk) == 19200000) {

		//printk(KERN_INFO "uart_hw_init : MNDREG= 0x1A \r\n");
		/* clock is TCXO (19.2MHz) */
		//msm_write(0x06, UART_MREG);
		//msm_write(0xF1, UART_NREG);
		//msm_write(0x0F, UART_DREG);
		//msm_write(0x1A, UART_MNDREG);

		// This is sync from express hboot by steven.
		printk("%s : MNDREG= 0x60 0x1D 0xFA 0x1C \n", __func__);
		/* clock is TCXO (19.2MHz) */
		wimax_msm_write(0x60, UART_MREG);
		wimax_msm_write(0x1D, UART_NREG);
		wimax_msm_write(0xFA, UART_DREG);
		wimax_msm_write(0x1C, UART_MNDREG);		
	} else
#endif
	{
		printk("%s : MNDREG= 0xC0 0xB2 0x7D 0x1C \n", __func__);	
		/* clock must be TCXO/4 */
		wimax_msm_write(0xC0, UART_MREG);
		wimax_msm_write(0xB2, UART_NREG);
		wimax_msm_write(0x7D, UART_DREG);
		wimax_msm_write(0x1C, UART_MNDREG);
	}


	
	//pUartRegs[port]->uart_imr = 0;
	wimax_msm_write(0x00,UART_IMR);
	
	/* Receive watermark at zero.  This means an interrupt would be generated
	 * whenever any (more than zero) characters are in the Rx FIFO.  Since we
	 * don't use Rx interrupts, this doesn't really matter.
	 */

	//pUartRegs[port]->uart_rfwr.rfw = 0;

	/* Transmit watermark.  We DO use this, to determine if there is
	 * room for one more character in the Tx FIFO.
	 */
	//pUartRegs[port]->uart_tfwr.tfw = UART_TXWAT_VAL;

	/* rx interrupt on every character -- keep it simple */
	wimax_msm_write(0, UART_RFWR);

	/* Do not enable hardware flow control.  It works, but it causes
	 * more trouble than it's worth with improperly-wired cables.
	 * There's really no need for it anyway, since the main data flow
	 * is from the host to the phone, and the phone (having nothing
	 * better to do than service the UART) will always keep up.
	 */

	//memset((void *)&pUartRegs[port]->uart_mr1, 0, sizeof(pUartRegs[port]->uart_mr1));
	wimax_msm_write(0x00,UART_MR1);
	//pUartRegs[port]->uart_mr2.bits_per_char = 3;	// 8BPC
	//pUartRegs[port]->uart_mr2.stop_bit_len = 1;	// 1SB
	//msm_write(UART_MR2_BITS_PER_CHAR, UART_MR2);	
	wimax_msm_write(UART_MR2_STOP_BIT_LEN_ONE | UART_MR2_BITS_PER_CHAR, UART_MR2);		
	//pUartRegs[port]->uart_csr = UART_CSR_1152K_BPS;
	wimax_msm_write(UART_CSR_115200, UART_CSR);
	
	//pUartRegs[port]->uart_cr = BIT_UART_CR_CH_CMD_RESET_ERROR;	// CR_RESET_ERR
	//pUartRegs[port]->uart_cr = BIT_UART_CR_CH_CMD_RESET_RX;	// CR_RESET_RX
	//pUartRegs[port]->uart_cr = BIT_UART_CR_CH_CMD_RESET_RX;	// CR_RESET_RX
	//pUartRegs[port]->uart_cr = BIT_UART_CR_CH_CMD_RESET_TX;	// CR_RESET_TX
	//pUartRegs[port]->uart_cr = BIT_UART_CR_CH_CMD_RESET_TX;	// CR_RESET_TX
	/* reset everything */
	wimax_msm_write(UART_CR_TX_DISABLE, UART_CR);
	wimax_msm_write(UART_CR_RX_DISABLE, UART_CR);

	wimax_msm_write(UART_CR_CMD_RESET_RX, UART_CR);
	wimax_msm_write(UART_CR_CMD_RESET_RX, UART_CR);
	wimax_msm_write(UART_CR_CMD_RESET_TX, UART_CR);
	wimax_msm_write(UART_CR_CMD_RESET_TX, UART_CR);
	wimax_msm_write(UART_CR_CMD_RESET_ERR, UART_CR);
	wimax_msm_write(UART_CR_CMD_RESET_BREAK_INT, UART_CR);
	wimax_msm_write(UART_CR_CMD_RESET_CTS, UART_CR);
	wimax_msm_write(UART_CR_CMD_SET_RFR, UART_CR);
	//pUartRegs[port]->uart_cr = BIT_UART_CR_RX_EN;
	//pUartRegs[port]->uart_cr = BIT_UART_CR_TX_EN;
	/* enable TX and RX */
	wimax_msm_write(UART_CR_TX_ENABLE, UART_CR);
	wimax_msm_write(UART_CR_RX_ENABLE, UART_CR);

	/* Wait while the UART's reciever drains.  After resetting the UART, we
	 * may have a couple of spurrious characters in the RX FIFO.  This can
	 * happen especially when the phone is not plugged into a DIP, thus
	 * causing the UART to re-recognize the start of break condtion.
	 */
	while ( (wimax_msm_read(UART_SR) & UART_SR_RX_READY) ) {
		tempc  = wimax_msm_read(UART_RF);
		printk("%s :@@%c",__func__, tempc);
	}

	printk("%s - \n", __func__);
	return 0;
}

void msm_serial_wimax_init(unsigned int uart_base,unsigned int gsbi_base, int irq,
			   struct device *clk_device, int signal_irq, int wakeup_irq)			   
{
	//int ret;
	void *port;
	void *port_gsbi;	
	printk("%s + uart_base = 0x%x\n", __func__,uart_base);
	debug_clk = clk_get(clk_device, "uart_clk");
	if (debug_clk)
		clk_enable(debug_clk);

	port = ioremap(uart_base, 4096);
	if (!port)
		return;
	debug_port_base = (unsigned int) port;

	port_gsbi=ioremap(gsbi_base, 4);
	if (!port_gsbi)
		return;
	gsbi_port_base = (unsigned int) port_gsbi;

	uart_clk_port_base = (unsigned int) MSM_CLK_CTL_BASE ;
	printk("%s : uart_clk_port_base = 0x%x\n", __func__,uart_clk_port_base);
	printk("%s : debug_port_base = 0x%x\n", __func__,debug_port_base);	
	msm_serial_wimax_uart_hw_init();
	printk("%s - \n", __func__);	
}


//**************************************************
//FUNCTION: msm_serial_wimax_thread
//
//INPUT: (int num)
//10~20: Polling UART RX FIFO.
//21~22: TX test
//97: DUMP UART REGISTER
//98: RESET UART TX/RX
//99: UART HW init
//**************************************************


int msm_serial_wimax_thread(int num)
{

	unsigned int sr;
	int count =0,count2=0;
	int error =0;
	unsigned int tempi;
	printk("%s + num= %d\n", __func__,num);

	if(num >= 10 && num <=20)
	{
		while(count < 1000)
		{
			mdelay((num-10));
			count2=0;
			while((sr = wimax_msm_read(UART_SR))  & UART_SR_RX_READY)
			{
				printk("%s : RX sr=0x%8x count=%d\n", __func__,sr,count);
				tempi = wimax_msm_read(UART_RF);
				printk("%s : @@(%c) \n", __func__,tempi);

				if(sr & UART_SR_RX_BREAK)
				{
					error =1;
					printk("%s : GOT UART_SR_RX_BREAK!!\n", __func__);					
				}
				if(sr & UART_SR_PAR_FRAME_ERR )
				{
					error =1;
					printk("%s : GOT UART_SR_PAR_FRAME_ERR!!\n", __func__);	
				}
				if(sr & UART_SR_HUNT_CHAR)
				{
					error =1;
					printk("%s : GOT UART_SR_HUNT_CHAR!!\n", __func__);
				}

				if(sr & UART_SR_OVERRUN)
				{
					printk("%s : GOT UART_SR_OVERRUN!!\n", __func__);
				}

				if(error ==1)
				{	
					printk("%s : reset rx!!\n", __func__);
					/* reset everything */
					//msm_write(UART_CR_TX_DISABLE, UART_CR);
					wimax_msm_write(UART_CR_RX_DISABLE, UART_CR);

					wimax_msm_write(UART_CR_CMD_RESET_RX, UART_CR);
					wimax_msm_write(UART_CR_CMD_RESET_RX, UART_CR);
					//msm_write(UART_CR_CMD_RESET_TX, UART_CR);
					//msm_write(UART_CR_CMD_RESET_TX, UART_CR);
					wimax_msm_write(UART_CR_CMD_RESET_ERR, UART_CR);
					wimax_msm_write(UART_CR_CMD_RESET_BREAK_INT, UART_CR);
					wimax_msm_write(UART_CR_CMD_RESET_CTS, UART_CR);
					wimax_msm_write(UART_CR_CMD_SET_RFR, UART_CR);

					//msm_write(UART_CR_TX_ENABLE, UART_CR);
					wimax_msm_write(UART_CR_RX_ENABLE, UART_CR);
					error =0;
				}
			}
			if(!(sr & UART_SR_RX_READY))
				printk("%s : NORX sr=0x%8x count=%d\n", __func__,sr,count);

			count++;
		}	

	}
	else if (num == 21)
	{
		printk("%s : start sending cmd 21\n", __func__);
		mdelay(2000);
		wimax_msm_write(0x63, UART_TF);//c
		if((sr = wimax_msm_read(UART_SR))  & UART_SR_TX_READY)
		{
			printk("%s : TX sr=0x%8x \n", __func__,sr);	
		}
		//msm_write(0x6D, UART_TF);//m
		//msm_write(0x64, UART_TF);//d
		//msm_write(0x00, UART_TF);//
		//msm_write(0x22, UART_TF);//"
		//msm_write(0x73, UART_TF);//s
		//msm_write(0x68, UART_TF);//h
		//msm_write(0x6F, UART_TF);//o
		//msm_write(0x77, UART_TF);//w
		//msm_write(0x76, UART_TF);//v
		//msm_write(0x65, UART_TF);//e
		//msm_write(0x72, UART_TF);//r
		//msm_write(0x73, UART_TF);//s
		//msm_write(0x69, UART_TF);//i
		//msm_write(0x6F, UART_TF);//o
		//msm_write(0x6E, UART_TF);//n
		//msm_write(0x22, UART_TF);//"
		//msm_write(0x0D, UART_TF);//\r
		printk("%s : end sending cmd\n", __func__);

	}
	else if (num == 22)
	{	
		printk("%s : start sending cmd 22\n", __func__);
		mdelay(2000);
		wimax_msm_write(0x63, UART_TF);//c
		wimax_msm_write(0x6D, UART_TF);//m
		wimax_msm_write(0x64, UART_TF);//d
		if((sr = wimax_msm_read(UART_SR))  & UART_SR_TX_READY)
		{
			printk("%s : TX sr=0x%8x \n", __func__,sr);
		}
		printk("%s : end sending cmd\n", __func__);
	}
	else if(num ==97)
	{
		msm_serial_wimax_dump_register();
	}	
	else if(num ==98)
	{
		printk("%s : reset rx/tx\n", __func__);	
		/* reset everything */
		wimax_msm_write(UART_CR_TX_DISABLE, UART_CR);
		wimax_msm_write(UART_CR_RX_DISABLE, UART_CR);

		wimax_msm_write(UART_CR_CMD_RESET_RX, UART_CR);
		wimax_msm_write(UART_CR_CMD_RESET_RX, UART_CR);
		wimax_msm_write(UART_CR_CMD_RESET_TX, UART_CR);
		wimax_msm_write(UART_CR_CMD_RESET_TX, UART_CR);
		wimax_msm_write(UART_CR_CMD_RESET_ERR, UART_CR);
		wimax_msm_write(UART_CR_CMD_RESET_BREAK_INT, UART_CR);
		wimax_msm_write(UART_CR_CMD_RESET_CTS, UART_CR);
		wimax_msm_write(UART_CR_CMD_SET_RFR, UART_CR);

		wimax_msm_write(UART_CR_TX_ENABLE, UART_CR);
		wimax_msm_write(UART_CR_RX_ENABLE, UART_CR);
	}	
	else if(num ==99)
	{
		msm_serial_wimax_uart_hw_init();
	}
	else
	{
		printk("%s don't support this number  %d\n", __func__,num);
	}

	printk("%s -\n", __func__);
	return 0;
}
