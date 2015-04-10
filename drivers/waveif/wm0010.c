/*
 * wm0010.c  --  WM0010 ALSA Soc Audio driver
 *
 * Copyright 2010 Wolfson Microelectronics PLC.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#define DEBUG

#ifdef	DEBUG
#define	DBG_PRINTK(x, ...)	printk("wm0010(%d):" x, __LINE__, ##__VA_ARGS__)
#endif

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/irqreturn.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/firmware.h>
#include <linux/ihex.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/gfp.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/spinlock_types.h>

#include <linux/clk.h>

#include <linux/waveif.h>

#include <mach/msm_iomap.h>

#ifndef _WM0010_H_
#include "wm0010.h"
#endif

/**********************************************************************/
/* macros															  */
/**********************************************************************/

#define	GP_CLK_SHARE

/*#define	USE_IRQ_READY				1	*/	/* use interrupt to process write() */
#ifndef	USE_IRQ_READY							/* no interrupt to wait ready */
#define	READY_POLL_INTERVAL				1		/* poll ready 1ms interval */
#define	READY_POLL_MAX_NUM				25		/* max polling times is 25 before write */
#define READY_POLL_MAX_NUM2				10		/* poll 10 times becomes ready after download firmware */
#endif
#define	USE_SUSPEND						1		/* enable suspend/resume */

/*#define	SPI_LOG_ON					1	*/	/* enable logs for SPI controls */

#define	DEBUG_SLOW_WRITE				1		/* read/write uses 26MHz->(slow) clock */
/*#define	MONITOR_IRQ					1	*/	/* monitor XIRQ */

#define	CTRL_SPI_CLK_LOW				1		/* set SPI signals GPIO low(CS set to HIGH) while suspend */

#define	VREG_CORE_OFF					1		/* power off CORE while RESET */

#define	MBP_MODE_CLK_B		GPIO_CFG(98, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA)	/* CLK_B */
#define	MBP_MODE_RESET		GPIO_CFG(100, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)	/* RESET */
#define	MBP_MODE_REQUEST	GPIO_CFG(162, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)	/* REQUEST */
#define	MBP_MODE_READY		GPIO_CFG(99, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)	/* READY */
#define	MBP_MODE_XIRQ		GPIO_CFG(92, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)		/* XIRQ */

#define	DSP_POWER_CORE	"gp3"							/* CORE */
#define	DSP_POWER_IO	"lvsw1"							/* IO */

#define	BOOT_IMAGE_1ST	"wm0010_stage2_BOOT.bin"		/* firmware of 1st stage */

#define	BOOT_IMAGE_2ND	"EIF_VDSP.fw"					/* firmware of 2nd stage */


#define	SPI_CLK_MAX_1ST	(1638400)			/* 1st stage */
#define	SPI_CLK_MAX_2ND	(26331429)			/* 2nd stage */

#ifdef	DEBUG_SLOW_WRITE
#define	SPI_CLK_MAX_RW	400000				/* Cmd/Status */
#else
#define	SPI_CLK_MAX_RW	SPI_CLK_MAX_2ND		/* Cmd/Status, native speed */
#endif
#define	RATE_CLOCK_B	12288000			/* GP_CLK_B setting for CLKIN */





/**********************************************************************/
/* prototypes		 												  */
/**********************************************************************/
int setup_remove(struct wm0010_pdata *pdata);
int setup_probe(struct wm0010_pdata *pdata);

/**********************************************************************/
/* globals															  */
/**********************************************************************/
static DEFINE_SPINLOCK(rw_lock);			/* exclusive access read/write */

static struct spi_device *wm0010 = NULL;	/* spi_device* created in probe */

#ifdef	GP_CLK_SHARE

#include <mach/msm_iomap.h>

typedef int8_t		VibeInt8;
typedef u_int8_t	VibeUInt8;
typedef int16_t		VibeInt16;
typedef u_int16_t	VibeUInt16;
typedef int32_t		VibeInt32;
typedef u_int32_t	VibeUInt32;
typedef u_int8_t	VibeBool;
typedef VibeInt32	VibeStatus;

#define uint32 uint32_t
typedef unsigned long L4_Word_t;
typedef L4_Word_t word_t;
typedef word_t L4_Bool_t;

#define __inpdw(port)       (*((volatile uint32 *) (port)))
#define in_dword_masked(addr, mask) (__inpdw(addr) & (mask))
#define __outpdw(port, val) (*((volatile uint32 *) (port)) = ((uint32) (val)))
#define out_dword(addr, val) __outpdw(addr,val)
#define out_dword_masked_ns(io, mask, val, current_reg_content) \
  out_dword( io, ((current_reg_content & (uint32)(~(mask))) | \
                 ((uint32)((val) & (mask)))) )

#define __msmhwio_outm(hwiosym, mask, val) HWIO_##hwiosym##_OUTM(mask, val)
#define HWIO_OUTM(hwiosym, mask, val) __msmhwio_outm(hwiosym, mask, val)

#define HWIO_GP_NS_REG_RMSK                                                     0xffffffff
#define HWIO_GP_NS_REG_MNCNTR_EN_BMSK                                                0x100
#define HWIO_GP_NS_REG_MNCNTR_EN_SHFT                                                  0x8
#define HWIO_GP_NS_REG_MNCNTR_RST_BMSK                                                0x80
#define HWIO_GP_NS_REG_MNCNTR_RST_SHFT                                                 0x7
#define HWIO_GP_NS_REG_MNCNTR_MODE_BMSK                                               0x60
#define HWIO_GP_NS_REG_MNCNTR_MODE_SHFT                                                0x5
#define HWIO_GP_NS_REG_GP_N_VAL_BMSK                                            0xffff0000
#define HWIO_GP_NS_REG_GP_N_VAL_SHFT                                                  0x10
#define HWIO_GP_NS_REG_GP_ROOT_ENA_BMSK                                              0x800
#define HWIO_GP_NS_REG_GP_ROOT_ENA_SHFT                                                0xb
#define HWIO_GP_NS_REG_GP_CLK_INV_BMSK                                               0x400
#define HWIO_GP_NS_REG_GP_CLK_INV_SHFT                                                 0xa
#define HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_BMSK                                        0x200
#define HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_SHFT                                          0x9
#define HWIO_GP_NS_REG_PRE_DIV_SEL_BMSK                                               0x18
#define HWIO_GP_NS_REG_PRE_DIV_SEL_SHFT                                                0x3
#define HWIO_GP_NS_REG_SRC_SEL_BMSK                                                    0x7
#define HWIO_GP_NS_REG_SRC_SEL_SHFT                                                      0

#define HWIO_GP_MD_REG_RMSK                                                     0xffffffff
#define HWIO_GP_MD_REG_M_VAL_BMSK                                               0xffff0000
#define HWIO_GP_MD_REG_M_VAL_SHFT                                                     0x10
#define HWIO_GP_MD_REG_D_VAL_BMSK                                                   0xffff
#define HWIO_GP_MD_REG_D_VAL_SHFT                                                        0
#define CLK_CTL_REG_BASE                                                        (MSM_CLK_CTL_BASE + 0x00000000)
#define HWIO_GP_MD_REG_ADDR                                                     (CLK_CTL_REG_BASE      + 0x00000058)
#define HWIO_GP_MD_REG_OUT(v)                                                   \
        out_dword(HWIO_GP_MD_REG_ADDR,v)
#define HWIO_GP_MD_REG_OUTM(m,v)                                                \
        out_dword_masked_ns(HWIO_GP_MD_REG_ADDR,m,v,HWIO_GP_MD_REG_IN);
#define HWIO_GP_MD_REG_IN                                                       \
        in_dword_masked(HWIO_GP_MD_REG_ADDR, HWIO_GP_MD_REG_RMSK)

#define HWIO_GP_NS_REG_ADDR                                                     (CLK_CTL_REG_BASE      + 0x0000005c)
#define HWIO_GP_NS_REG_OUT(v)                                                   \
        out_dword(HWIO_GP_NS_REG_ADDR,v)
#define HWIO_GP_NS_REG_OUTM(m,v)                                                \
        out_dword_masked_ns(HWIO_GP_NS_REG_ADDR,m,v,HWIO_GP_NS_REG_IN); 
#define HWIO_GP_NS_REG_IN                                                       \
        in_dword_masked(HWIO_GP_NS_REG_ADDR, HWIO_GP_NS_REG_RMSK)

static int clk_12288(void)
{
	/* set clk default for wolfson */
	/* M/N:D counter */
	//printk("[hap log]%s : hwio en_bmsk start \n",__FUNCTION__);
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 0 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT);

	//printk("[hap log]%s: n_val_bmsk start\n",__FUNCTION__);
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_N_VAL_BMSK, 0 << HWIO_GP_NS_REG_GP_N_VAL_SHFT);

	//printk("[hap log]%s : HWIO_GP_NS_REG_GP_ROOT_ENA_BMSK start \n",__FUNCTION__);
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_ROOT_ENA_BMSK, 0 << HWIO_GP_NS_REG_GP_ROOT_ENA_SHFT);

	//printk("[hap log]%s : HWIO_GP_NS_REG_GP_CLK_INV_BMSK start \n",__FUNCTION__);
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_CLK_INV_BMSK, 0 << HWIO_GP_NS_REG_GP_CLK_INV_SHFT);

	//printk("[hap log]%s : HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_BMSK start \n",__FUNCTION__);
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_BMSK, 0 << HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_SHFT);

	//printk("[hap log]%s : HWIO_GP_NS_REG_MNCNTR_RST_BMSK start \n",__FUNCTION__);
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_RST_BMSK, 0 << HWIO_GP_NS_REG_MNCNTR_RST_SHFT);

	/* Dual-edge mode => 2*/
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_MODE_BMSK, 0 << HWIO_GP_NS_REG_MNCNTR_MODE_SHFT); 

	/* P: 0 => Freq/1, 1 => Freq/2, 3 => Freq/4 */
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_PRE_DIV_SEL_BMSK, 1 << HWIO_GP_NS_REG_PRE_DIV_SEL_SHFT);

	/* S : 0 => TXCO(19.2MHz), 1 => Sleep XTAL(32kHz) */
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_SRC_SEL_BMSK, 6 << HWIO_GP_NS_REG_SRC_SEL_SHFT); 

	//printk("[hap log]%s: m_val_bmsk start\n",__FUNCTION__);
	HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_M_VAL_BMSK, 0 << HWIO_GP_MD_REG_M_VAL_SHFT);

	//printk("[hap log]%s: d_val_bmsk start\n",__FUNCTION__);
	HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_D_VAL_BMSK, 0 << HWIO_GP_MD_REG_D_VAL_SHFT);

	/* Enable M/N counter */
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 0 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT); 

	return 0;
}
#endif

/**
 *	wm0010_read
 *
 *	read the status from DSP
 *
 *	@param	*f		[in]	file descriptor
 *	@param	*data	[in]	buffer
 *	@param	len		[in]	max size
 *	@param	*off	[in]	read offset in buffer
 *	@return	>0	bytes read<br>
 *			==0	canceled by Close() is invoked<br>
 *			<0	error
 */
static ssize_t wm0010_read(struct file *f, char __user *data, size_t len, loff_t *off)
{
	u8 *buf = kzalloc(len, GFP_KERNEL);
	struct spi_message m;
	struct spi_transfer t;
	int ret;
	unsigned long flags;
	struct wm0010_pdata *pdata = dev_get_platdata(&wm0010->dev);

//	DBG_PRINTK("%s: *f=%p, *data=%p, len=%d, *off=%p\n", __func__, f, data, len, off);
	DBG_PRINTK("%s\n", __func__);

	if (!f) {
		kfree(buf);
		return -EBADF;
	}
	if (!buf) {
		return -ENOMEM;
	}

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));
	t.rx_buf = buf;
	t.len = len;
	t.bits_per_word = 8;
#ifdef	DEBUG_SLOW_WRITE
	t.speed_hz = SPI_CLK_MAX_RW;
#endif
	spi_message_add_tail(&t, &m);

	/* wait for REQUEST */
	wait_event_interruptible(pdata->wq_request, pdata->request_raised);
	if (pdata->cancel_read) {
		DBG_PRINTK("%s() canceled...\n", __func__);
		kfree(buf);
		pdata->cancel_read = 0;
		return 0;
	}
	if (pdata->request_raised == 0) {
		DBG_PRINTK("%s() interrupted...\n", __func__);
		kfree(buf);
		return -EINTR;
	}

	spin_lock_irqsave(&rw_lock, flags);
	pdata->request_raised--;
	spin_unlock_irqrestore(&rw_lock, flags);

	ret = spi_sync(wm0010, &m);
	if (ret != 0) {
		goto err;
	}

	if (copy_to_user(data, buf, len)) {
		ret = -EFAULT;
		goto err;
	}

	kfree(buf);
	DBG_PRINTK("%s OK\n", __func__);

	return len;

err:
	DBG_PRINTK("%s ERR=%d\n", __func__, ret);
	kfree(buf);
	return ret;
}

/**
 *	request_isr
 *
 *	REQUEST ISR<br>
 *
 *	@param	irq		[in]	irq number
 *	@param	data	[in]	parameter
 *	@return	IRQ_HANDLED<br>
 */
static irqreturn_t request_isr(int irq, void *data)
{
	unsigned long flags;
	struct wm0010_pdata *pdata = dev_get_platdata(&wm0010->dev);

	spin_lock_irqsave(&rw_lock, flags);
	pdata->request_raised++;
	spin_unlock_irqrestore(&rw_lock, flags);

	wake_up_interruptible(&pdata->wq_request);

	return IRQ_HANDLED;
}

#ifdef	MONITOR_IRQ
static irqreturn_t xirq_isr(int irq, void *data)
{
	static int cnt = 0;

	cnt++;
	DBG_PRINTK("%s - %d\n", __func__, cnt);
	return IRQ_HANDLED;
}
#endif

#ifdef	USE_IRQ_READY
static irqreturn_t ready_isr(int irq, void *data)
{
	unsigned long flags;
	struct wm0010_pdata *pdata = dev_get_platdata(&wm0010->dev);

	spin_lock_irqsave(&rw_lock, flags);
	pdata->ready_raised++;
	spin_unlock_irqrestore(&rw_lock, flags);

	wake_up_interruptible(&pdata->wq_ready);

	return IRQ_HANDLED;
}
#endif

/**
 *	cleanup_config
 *
 *	off the setting of GPIO<br>
 *	any errors are ignored.
 *
 *	@param	pdata	[in]	driver parameter
 *	@return	0	success
 */
int cleanup_config(struct wm0010_pdata *pdata)
{
	int ret = 0;

	DBG_PRINTK("%s\n", __func__);
#ifdef	USE_IRQ_READY
	disable_irq(pdata->gpio_ready_irq);
#endif

	if (pdata->gpio_request_irq) {
		free_irq(pdata->gpio_request_irq, NULL);
		pdata->gpio_request_irq = 0;
	}
#ifdef	USE_IRQ_READY
	if (pdata->gpio_ready_irq) {
		free_irq(pdata->gpio_ready_irq, NULL);
		pdata->gpio_ready_irq = 0;
	}
#endif
#ifdef	MONITOR_IRQ
	if (pdata->gpio_xirq_irq) {
		free_irq(pdata->gpio_xirq_irq, NULL);
		pdata->gpio_xirq_irq = 0;
	}
#endif
	if (pdata->pin_ready) {
		gpio_free(pdata->pin_ready);
		pdata->pin_ready = 0;
	}

	if (pdata->pin_request) {
		gpio_free(pdata->pin_request);
		pdata->pin_request = 0;
	}

	if (pdata->pin_reset) {
		gpio_free(pdata->pin_reset);
		pdata->pin_reset = 0;
	}

	if (pdata->pin_xirq) {
		gpio_free(pdata->pin_xirq);
		pdata->pin_xirq = 0;
	}
#if	0
	ret = gpio_tlmm_config(MBP_MODE_RESET, GPIO_CFG_DISABLE);
	if (ret) {
		DBG_PRINTK("gpio_tlmm_config(DSP_RESET, DIS) = %d\n", ret);
	}

	ret = gpio_tlmm_config(MBP_MODE_REQUEST, GPIO_CFG_DISABLE);
	if (ret) {
		DBG_PRINTK("gpio_tlmm_config(REQUEST, DIS) = %d\n", ret);
	}

	ret = gpio_tlmm_config(MBP_MODE_READY, GPIO_CFG_DISABLE);
	if (ret) {
		DBG_PRINTK("gpio_tlmm_config(READY, DIS) = %d\n", ret);
	}

	ret = gpio_tlmm_config(MBP_MODE_XIRQ, GPIO_CFG_DISABLE);
	if (ret) {
		DBG_PRINTK("gpio_tlmm_config(XIRQ, DIS) = %d\n", ret);
	}
#endif
DBG_PRINTK("%s()\n", __func__);
	return ret;
}

/**
 *	setup_probe
 *
 *	initialize GPIO, it called from probe.
 *
 *	@param	pdata	[in]	driver parameter
 *	@return	0		Success<br>
 *			-EINVAL	fail
 */
int setup_probe(struct wm0010_pdata *pdata)
{
	int ret;
#if	1
	ret = gpio_tlmm_config(MBP_MODE_RESET, GPIO_CFG_ENABLE);
	if (ret) {
		DBG_PRINTK("gpio_tlmm_config(DSP_RESET, ENA) = %d\n", ret);
		goto ON_ERR;
	}

	ret = gpio_tlmm_config(MBP_MODE_REQUEST, GPIO_CFG_ENABLE);
	if (ret) {
		DBG_PRINTK("gpio_tlmm_config(REQUEST, ENA) = %d\n", ret);
		goto ON_ERR;
	}

	ret = gpio_tlmm_config(MBP_MODE_READY, GPIO_CFG_ENABLE);
	if (ret) {
		DBG_PRINTK("gpio_tlmm_config(READY, ENA) = %d\n", ret);
		goto ON_ERR;
	}

	ret = gpio_tlmm_config(MBP_MODE_XIRQ, GPIO_CFG_ENABLE);
	if (ret) {
		DBG_PRINTK("gpio_tlmm_config(XIRQ, ENA) = %d\n", ret);
		goto ON_ERR;
	}
#endif
//	pdata->pin_reset = GPIO_PIN(MBP_MODE_RESET);
//	pdata->pin_request = GPIO_PIN(MBP_MODE_REQUEST);
//	pdata->pin_ready = GPIO_PIN(MBP_MODE_READY);
//	pdata->pin_xirq = GPIO_PIN(MBP_MODE_XIRQ);

#ifdef	USE_IRQ_READY
//	pdata->gpio_ready_irq = gpio_to_irq(pdata->pin_ready);
#endif

	pdata->clk_b = clk_get(NULL, "gp_clk");	/* GP_CLK(GP_CLK_A, GP_CLK_B common) */
	if (IS_ERR(pdata->clk_b)) {
		pr_err("%s: error\n", __func__);
		goto ON_ERR;
	}
	/* CORE first */
	pdata->vreg_core = vreg_get(NULL, DSP_POWER_CORE);
	if (IS_ERR(pdata->vreg_core)) {
		DBG_PRINTK("vreg_get(CORE) = 0x%p\n", pdata->vreg_core);
		goto ON_ERR;
	}

	/* IO second */
	pdata->vreg_io = vreg_get(NULL, DSP_POWER_IO);
	if (IS_ERR(pdata->vreg_io)) {
		DBG_PRINTK("vreg_get(IO) = 0x%p\n", pdata->vreg_io);
		goto ON_ERR;
	}

	return 0;

ON_ERR:
	setup_remove(pdata);
	return -EINVAL;
}

/**
 *	setup_remove
 *
 *	.remove OR .probe use this to handle error<br>
 *	release resouces.
 *
 *	@param	pdata	[in]	driver paramter
 *	@return	0	success
 */
int setup_remove(struct wm0010_pdata *pdata)
{
	if (pdata->clk_b) {
		clk_put(pdata->clk_b);	/* GP_CLK(GP_CLK_A, GP_CLK_B common) */
		pdata->clk_b = NULL;
	}

	if (pdata->vreg_io) {
		vreg_put(pdata->vreg_io);
		pdata->vreg_io = NULL;
	}
	if (pdata->vreg_core) {
		vreg_put(pdata->vreg_core);
		pdata->vreg_core = NULL;
	}

#ifdef	MONITOR_IRQ
	pdata->gpio_xirq_irq = 0;
#endif
	pdata->gpio_request_irq = 0;
#ifdef	USE_IRQ_READY
	pdata->gpio_ready_irq = 0;
#endif
	pdata->pin_ready = 0;
	pdata->pin_request = 0;
	pdata->pin_reset = 0;
	pdata->pin_xirq = 0;

#if	1
	{
		int ret;
		ret = gpio_tlmm_config(MBP_MODE_RESET, GPIO_CFG_DISABLE);
		if (ret) {
			DBG_PRINTK("gpio_tlmm_config(DSP_RESET, DIS) = %d\n", ret);
		}

		ret = gpio_tlmm_config(MBP_MODE_REQUEST, GPIO_CFG_DISABLE);
		if (ret) {
			DBG_PRINTK("gpio_tlmm_config(REQUEST, DIS) = %d\n", ret);
		}

		ret = gpio_tlmm_config(MBP_MODE_READY, GPIO_CFG_DISABLE);
		if (ret) {
			DBG_PRINTK("gpio_tlmm_config(READY, DIS) = %d\n", ret);
		}

		ret = gpio_tlmm_config(MBP_MODE_XIRQ, GPIO_CFG_DISABLE);
		if (ret) {
			DBG_PRINTK("gpio_tlmm_config(XIRQ, DIS) = %d\n", ret);
		}
	}
#endif
	return 0;
}

/**
 *	setup_config
 *
 *	setup GPIO config
 *
 *	@param	pdata	[in]	driver paramter
 *	@return	0	success<br>
 *			<0	fail
 */
int setup_config(struct wm0010_pdata *pdata)
{
	int ret;

	DBG_PRINTK("%s\n", __func__);
#if	0
	ret = gpio_tlmm_config(MBP_MODE_RESET, GPIO_CFG_ENABLE);
	if (ret) {
		DBG_PRINTK("gpio_tlmm_config(DSP_RESET, ENA) = %d\n", ret);
		goto ON_ERR;
	}

	ret = gpio_tlmm_config(MBP_MODE_REQUEST, GPIO_CFG_ENABLE);
	if (ret) {
		DBG_PRINTK("gpio_tlmm_config(REQUEST, ENA) = %d\n", ret);
		goto ON_ERR;
	}

	ret = gpio_tlmm_config(MBP_MODE_READY, GPIO_CFG_ENABLE);
	if (ret) {
		DBG_PRINTK("gpio_tlmm_config(READY, ENA) = %d\n", ret);
		goto ON_ERR;
	}

	ret = gpio_tlmm_config(MBP_MODE_XIRQ, GPIO_CFG_ENABLE);
	if (ret) {
		DBG_PRINTK("gpio_tlmm_config(XIRQ, ENA) = %d\n", ret);
		goto ON_ERR;
	}
#endif
	pdata->pin_request = GPIO_PIN(MBP_MODE_REQUEST);
	ret = gpio_request(pdata->pin_request, "DSP Request");
	if (ret < 0) {
		DBG_PRINTK("Failed to request GPIO for DSP Request\n");
		pdata->pin_request = 0;
		goto ON_ERR;
	}
	pdata->pin_ready = GPIO_PIN(MBP_MODE_READY);
	ret = gpio_request(pdata->pin_ready, "DSP Ready");
	if (ret < 0) {
		DBG_PRINTK("Failed to request GPIO for DSP Ready\n");
		pdata->pin_ready = 0;
		goto ON_ERR;
	}
	pdata->pin_xirq = GPIO_PIN(MBP_MODE_XIRQ);
	ret = gpio_request(pdata->pin_xirq, "DSP XIRQ");
	if (ret < 0) {
		DBG_PRINTK("Failed to request GPIO for DSP XIRQ\n");
		pdata->pin_xirq = 0;
		goto ON_ERR;
	}

	pdata->pin_reset = GPIO_PIN(MBP_MODE_RESET);
	ret = gpio_request(pdata->pin_reset, "DSP reset");
	if (ret < 0) {
		DBG_PRINTK("Failed to request GPIO for DSP reset\n");
		pdata->pin_reset = 0;
		goto ON_ERR;
	}

	ret = gpio_direction_input(pdata->pin_xirq);
	if (ret < 0) {
		DBG_PRINTK("Failed to set GPIO direction for DSP XIRQ\n");
		goto ON_ERR;
	}

	ret = gpio_direction_input(pdata->pin_request);
	if (ret < 0) {
		DBG_PRINTK("Failed to set GPIO direction for DSP request\n");
		goto ON_ERR;
	}

	ret = gpio_direction_input(pdata->pin_ready);
	if (ret < 0) {
		DBG_PRINTK("Failed to set GPIO direction for DSP ready\n");
		goto ON_ERR;
	}

	ret = gpio_direction_output(pdata->pin_reset, 0);
	if (ret < 0) {
		DBG_PRINTK("Failed to set GPIO direction for DSP reset\n");
		goto ON_ERR;
	}

	pdata->gpio_request_irq = gpio_to_irq(pdata->pin_request);
	ret = request_irq(pdata->gpio_request_irq, request_isr, IRQF_TRIGGER_RISING, "DSP_REQUEST", NULL);
	if (ret < 0) {
		DBG_PRINTK("Failed to bind IRQ_REQUEST: %d\n", ret);
		pdata->gpio_request_irq = 0;
		goto ON_ERR;
	}
	disable_irq(pdata->gpio_request_irq);
#ifdef	MONITOR_IRQ
	pdata->gpio_xirq_irq = gpio_to_irq(pdata->pin_xirq);
	ret = request_irq(pdata->gpio_xirq_irq, xirq_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "XIRQ_MON", NULL);
	if (ret < 0) {
		DBG_PRINTK("Failed to bind IRQ_XIRQ: %d\n", ret);
		pdata->gpio_xirq_irq = 0;
		goto ON_ERR;
	}
	disable_irq(pdata->gpio_xirq_irq);
#endif

	return 0;

ON_ERR:
DBG_PRINTK("%s()\n", __func__);
	cleanup_config(pdata);
	return -ENOMEM;
}

/**
 *	wm0010_write
 *
 *	write() sends command/parameter to DSP.
 *
 *	@param	*f		[in]	file descriptor
 *	@param	*data	[in]	buffer
 *	@param	len		[in]	data size
 *	@param	*off	[in]	offset in buffer
 *	@return	>0	written size<br>
 *			<=0	error
 */
static ssize_t wm0010_write(struct file *f, const char __user *data, size_t len, loff_t *off)
{
	u8 *buf = kzalloc(len, GFP_KERNEL);
	struct spi_message m;
	struct spi_transfer t;
	int ret;
#ifdef	USE_IRQ_READY
	unsigned long flags;
#endif

	struct wm0010_pdata *pdata = dev_get_platdata(&wm0010->dev);

	DBG_PRINTK("%s\n", __func__);

	if (!buf) {
		return -ENOMEM;
	}

	if (copy_from_user(buf, data, len)) {
		ret = -EFAULT;
		goto err;
	}

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));
	t.tx_buf = buf;
	t.len = len;
	t.bits_per_word = 8;
#ifdef	DEBUG_SLOW_WRITE
	t.speed_hz = SPI_CLK_MAX_RW;
#endif
	spi_message_add_tail(&t, &m);

#ifdef	USE_IRQ_READY
	wait_event_interruptible(pdata->wq_ready, pdata->ready_raised);
	spin_lock_irqsave(&rw_lock, flags);
	pdata->ready_raised--;
	spin_unlock_irqrestore(&rw_lock, flags);
#else
/* READY signal polling */
	{
		int i;
		int v;
		for (i = 0; i < READY_POLL_MAX_NUM; i++) {
			v = gpio_get_value(pdata->pin_ready);
			if (v == 1) {
				break;
			}
			msleep(READY_POLL_INTERVAL);
		}
		if (v == 0) {
			DBG_PRINTK("%s() READY is LOW(%d).\n", __func__, v);
			ret = -EINVAL;
			goto err;
		}
		DBG_PRINTK("%s() READY comes HIGH(i=%d).\n", __func__, i);

	}
#endif
	ret = spi_sync(wm0010, &m);
	if (ret != 0)
		goto err;

	kfree(buf);
#if	1
	{
		int v_rdy = gpio_get_value(pdata->pin_ready);
		int v_rst = gpio_get_value(pdata->pin_reset);
		int v_req = gpio_get_value(pdata->pin_request);
		int v_irq = gpio_get_value(pdata->pin_xirq);
		DBG_PRINTK("RDY:%d RST:%d REQ:%d XIQ:%d\n", v_rdy, v_rst, v_req, v_irq);
	}
#endif
	DBG_PRINTK("%s OK\n", __func__);
	return len;

err:
	DBG_PRINTK("%s ERR=%d\n", __func__, ret);
#if	1
	{
		int v_rdy = gpio_get_value(pdata->pin_ready);
		int v_rst = gpio_get_value(pdata->pin_reset);
		int v_req = gpio_get_value(pdata->pin_request);
		int v_irq = gpio_get_value(pdata->pin_xirq);
		DBG_PRINTK("RDY:%d RST:%d REQ:%d XIQ:%d\n", v_rdy, v_rst, v_req, v_irq);
	}
#endif
	kfree(buf);
	return ret;
}

/**
 *	wm0010_open
 *
 *	wm0010 initialize DSP and variables
 *
 *	@param	inode	[in]	inode
 *	@param	filp	[in]	filp
 *	@return	status<br>
 *			==0	OK<br>
 *			<0	error
 */
static int wm0010_open(struct inode *inode, struct file *filp)
{
	int	ret;
	struct wm0010_pdata *pdata = dev_get_platdata(&wm0010->dev);

	DBG_PRINTK("%s\n", __func__);

	/* */
	if (filp == NULL) {
		return -ENOMEM;
	}
	if (!try_module_get(THIS_MODULE)) {
		return -ENODEV;
	}

	/* runnning == 0 means enabled first */
	if (pdata->running++) {	/* 2nd enabler does not load firmware */
		return 0;
	}

#ifdef	VREG_CORE_OFF
	ret = vreg_enable(pdata->vreg_core);
	if (ret) {
		dev_err(&wm0010->dev, "vreg_enable(CORE)=%d\n", ret);
	}
#endif

#ifdef	GP_CLK_SHARE
	clk_12288();
	DBG_PRINTK("%s: clk_12288() done\n", __func__);
	usleep(1000);				/* 1ms wait to be stable */
	DBG_PRINTK("%s: usleep(1000) done\n", __func__);
#endif

/*	GPIO PIN, vreg_get()	*/

	ret = setup_config(pdata);
	if (ret) {
		DBG_PRINTK("%s\n", __func__);
		return ret;
	}

	ret = clk_enable(pdata->clk_b);				/* enable CLKIN:GP_CLK_B */
	if (ret) {
		pr_err("%s: ERROR: clk_enable() = %d\n", __func__, ret);
	}

/*	GPIO, XRESET LOW, CLKIN 1 cycle	*/
	gpio_set_value(pdata->pin_reset, 0);

#ifdef	USE_IRQ_READY
	enable_irq(pdata->gpio_ready_irq);
#endif
	enable_irq(pdata->gpio_request_irq);
#ifdef	MONITOR_IRQ
	enable_irq(pdata->gpio_xirq_irq);
#endif

	ret = wm0010_boot();
	if (ret) {
#ifdef	MONITOR_IRQ
		disable_irq(pdata->gpio_xirq_irq);
#endif
#ifdef	USE_IRQ_READY
		disable_irq(pdata->gpio_ready_irq);
#endif
		disable_irq(pdata->gpio_request_irq);
		goto ON_ERROR;
	}

DBG_PRINTK("%s() SUCCESS\n", __func__);
	return 0;

ON_ERROR:
DBG_PRINTK("%s() ON_ERROR:\n", __func__);

	clk_disable(pdata->clk_b);
	cleanup_config(pdata);
	return -ENOMEM;
}

/**
 *	wm0010_release
 *
 *	disable DSP<br>
 *
 *	@param	inode	[in]	inode
 *	@param	filp	[in]	filp
 *	@return	error code<br>
 *			0	success<br>
 *			<0	fail
 */
static int wm0010_release(struct inode *inode, struct file *filp)
{
	struct wm0010_pdata *pdata = (struct wm0010_pdata *)dev_get_platdata(&wm0010->dev);

	DBG_PRINTK("%s suspend=%d, resume=%d\n", __func__, pdata->suspend, pdata->resume);

	if (filp == NULL) {
		return -EBADF;
	}

	pdata->running--;
	/* NOP if caller is not last user */
	if (pdata->running) {	/* there are any user remains */
		return 0;
	}

#ifdef	MONITOR_IRQ
	disable_irq(pdata->gpio_xirq_irq);
#endif
#ifdef	USE_IRQ_READY
	disable_irq(pdata->gpio_ready_irq);
#endif
	disable_irq(pdata->gpio_request_irq);

/*	GPIO, XRESET LOW */
	gpio_set_value(pdata->pin_reset, 0);

/*	disalbe CLKIN */
	clk_disable(pdata->clk_b);

#ifdef	VREG_CORE_OFF
	{
		int ret;
		ret = vreg_disable(pdata->vreg_core);
		if (ret) {
			dev_err(&wm0010->dev, "%s(): vreg_disable(CORE) = %d\n", __func__, ret);
		}
	}
#endif	/*	VREG_CORE_OFF */

/*	release GPIO */
	cleanup_config(pdata);

	module_put(THIS_MODULE);

	return 0;
}

static int wm0010_ioctl(
	struct inode *inode,
	struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	int ret = 0;
	struct wm0010_pdata *pdata = dev_get_platdata(&wm0010->dev);

	DBG_PRINTK("%s\n", __func__);

	if (filp == NULL) {
		return -EBADF;
	}

	switch (cmd) {
	case WAVEIF_CANCEL_READ:
		{
			DBG_PRINTK("WAVEIF_CANCEL_READ\n");
			pdata->cancel_read = 1;		/* mark cancel request */
			request_isr(pdata->gpio_request_irq, NULL);
			ret = 0;
		}
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return  ret;
}

/* wm0010 driver FOPS */
static struct file_operations wm0010_fops = {
	.read		= wm0010_read,				/* read Entry */
	.write		= wm0010_write,				/* write Entry */
	.ioctl		= wm0010_ioctl,				/* ioctl Entry */
	.open		= wm0010_open,				/* open Entry */
	.release	= wm0010_release,			/* release Entry */
};

/* driver definition */
static struct miscdevice wm0010_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,			/* auto */
	.name = "wm0010",						/* Driver name */
	.fops = &wm0010_fops,					/* FOPS */
};

/**
 *	wm0010_halt
 *
 *	halt the driver
 *
 *	@return	0	success
 */
int wm0010_halt(void)
{
	struct wm0010_pdata *pdata = (struct wm0010_pdata *)dev_get_platdata(&wm0010->dev);

	DBG_PRINTK("%s\n", __func__);

	/* Assert reset */
	gpio_set_value(pdata->pin_reset, 0);

	pdata->running = 0;
	return 0;
}
//EXPORT_SYMBOL_GPL(wm0010_halt);

/**
 * wm0010_boot
 *
 *	load the firmware
 *
 *	(1)	RESET off
 *	(2)	usleep(5); wait for DSP wakeup
 *	(3)	send Boot Firmware, SPI
 *	(4) 2ms, wait PLL lock
 *	(5) Send 2nd Firmware
 *
 *
<PRE><TT>
    [Stage 1]

            [n]   0    1        n-1
    MOSI    +----+----+----+   +----+
            |Len |Data|Data|...|Data|   Len is number of bytes Data except Len itself.
            +----+----+----+   +----+
    End B/L  B    L->B L->B     L->B

    Ex.[Firmware Binary Format]

    00000000    0000030e                                // Len = ((0x0c34) / 4) + 1
    00000004    00000506 ff22444f e0000000 40000063     // Data, ...
    ...
    00000c34    3fff8120

    Firmware Binary Format
    ----------------------
    (0-3 th word)
    00000000  06 05 00 00 4F 44 22 FF 00 00 00 E0 63 00 00 40
    (4-7 th word)
    00000010  FC 00 00 40 00 00 00 00 0C 00 00 02 13 0C 32 20

    ...

    (776-779 th word)
    00000c20  43 00 00 00 00 00 FF 3F 00 00 FF 3F 00 80 FF 3F
    (780 th word)
    00000c30  20 81 FF 3F

    [Stage 2]

    SPI Signal Format
    -----------------

    SS  ~~~~|_______________________|~~~|_______________________|~...~~|_________|~~|_________|~

    MOSI    +----+----+----+   +----+   +----+----+----+   +----+      +----+----+  +----+----+
            |Addr|Data|Data|...|Data|   |Addr|Data|Data|...|Data| ...  |Addr|ADDR|  |Addr|0000|
            +----+----+----+   +----+   +----+----+----+   +----+      +----+----+  +----+----+
    End B/L  B    L->B L->B     L->B     B    L->B L->B     L->B        B    L->B    B    L->B

    Firmware Binary Format
    ----------------------

    (1st Data Frame)
    00000000  60 00 00 00 00 10 00 C5 49 10 D5 49 20 E5 49 30
              [Addr:B   ][len:B][Data1:L  ] [Data2:L  ]
    00000010  F5 49 00 34 00 00 00 00
                              ][Pad/4]
    (2nd Data Frame)
    00000018                          60 00 00 10 00 10 00 00
                                      [Addr:B   ][len:B][Data...
    00000020  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
    00000030  60 00 00 20 00 10 00 00 00 00 00 00 00 00 00 00
    00000040  00 00 00 00 00 00 00 00 60 00 00 30 00 10 00 00

    ...

    (N th Data Frame)
    00000210  60 00 01 60 00 0A 09 A0 AB 09 B0 BB 09 00 35 00
              [Addr:B   ][Len:B][Data.....] [Data.....][Data]   :: no padding
    (N+1 th Data Frame)
    00000220  60 00 01 80 00 06 20 D2 61 A0 02 00
    (N+2 th Data Frame)
    0000022c                                      60 00 01 C0
    00000230  00 06 20 D3 61 A0 02 00 60 00 02 00 00 06 20 D4

    ...

    000031a0  00 00 00 00
    (last-1 th data Frame)
    000031a4              60 00 23 50 00 10 FF FF FF FF 00 00
    000031b0  00 00 FF FF FF FF 00 00 00 00 00 00 
    (last th data Frame)
    000031bc                                      00 00 00 00
                                                  [Addr:0000]
    000031c0  00 04 80 06 00 60 00 00
             [Len:4][StartAddr:L][Pad]
    (End mark Frame)
    000031ca                          00 00 00 00 00 00
                                      [Addr:0000][len:0]
</TT></PRE>
 *
 *	@return	0	success<br>
 *			<0	fail
 */
int wm0010_boot(void)
{
	struct wm0010_pdata *pdata = (struct wm0010_pdata *)dev_get_platdata(&wm0010->dev);
	int	ret;
	int	rtycnt;
	const struct firmware *fw = NULL;
	struct spi_message m;
	struct spi_transfer t;
	u32 *img32 = NULL;
	u32 *img32_in;
	u32 *out32 = NULL;
	u8	*out = NULL;
	u32 len;
	int i;
	int j;
	int done;
	int pad_size;
	int allocated_size = 0;			/* allocated buffer size */
	unsigned int addr;				/* address field in Big Endian */
	int	rec_len;					/* record length */
	int aligned_size = 0;			/* aligned size it includes address */

	DBG_PRINTK("wm0010_boot\n");

	if (!wm0010) {
		pr_err("No WM0010 registered\n");
		return -EINVAL;
	}

	gpio_set_value(pdata->pin_reset, 0);
	usleep(1);
//	msleep(20);
	/* Release reset */
	gpio_set_value(pdata->pin_reset, 1);

	usleep(5);	// GP_CL
//	msleep(20);
//	usleep(50);	// GP_CL try 50us or 20ms?

	rtycnt = 0;
	for (;;) {
		int irq = gpio_get_value(pdata->pin_xirq);
		if (irq == 0) {
			DBG_PRINTK("IRQ reset OK : time = %d\n", (rtycnt * 20));
			break;
		} else {
			rtycnt++;
			msleep(20);
		}
	}
	
	/*
		First the bootloader
	*/

	ret = request_firmware(&fw, BOOT_IMAGE_1ST, &wm0010->dev);
	if (ret != 0) {
		dev_err(&wm0010->dev, "Failed to request bootloader: %d\n", ret);
		goto abort;
	}

#ifdef	SPI_LOG_ON
	dev_dbg(&wm0010->dev, "Downloading %d byte first stage\n", fw->size);
#endif
	/* Length is in DWORDs and needs to be a multiple of 64 */
	len = (fw->size + 3) / 4;	/* make it 4 bytes aligned size */
	img32 = kzalloc((len + 1) * 4, GFP_KERNEL);
	if (!img32) {
		ret = -ENOMEM;
		dev_err(&wm0010->dev, "Failed to allocate download image\n");
		goto abort;
	}

	out = kzalloc((len + 1) * 4, GFP_KERNEL);
	if (!out) {
		ret = -ENOMEM;
		dev_err(&wm0010->dev, "Failed to allocate output buffer\n");
		goto abort;
	}

	/* Image needs to be byte swapped for ROM bootloader */
	img32_in = (u32 *)(&fw->data[0]);
	for (i = 0; i < len; i++) {	/* convert 4 bytes word Endian without length on top */
		img32[i + 1] = cpu_to_be32(le32_to_cpu(img32_in[i]));
	}
	spi_message_init(&m);

	/* set data length on TOP */
	img32[0] = cpu_to_be32(len);
	memset(&t, 0, sizeof(t));
	t.rx_buf = out;
	t.tx_buf = img32;
	t.len = (len + 1) * 4;
	t.bits_per_word = 8;
	t.speed_hz = SPI_CLK_MAX_1ST;
	spi_message_add_tail(&t, &m);

	ret = spi_sync(wm0010, &m);
	if (ret != 0) {
		dev_err(&wm0010->dev, "Initial download failed: %d\n", ret);
		goto abort;
	}

#ifdef	SPI_LOG_ON
	DBG_PRINTK("check error after spi\n");
#endif
	/* Look for errors from the boot ROM */
	for (i = 0; i < fw->size + 4; i++) {	/* length and data. without padding */
		if ((i > 0) && out[i] && (out[i] != 0x55)) {
			ret = -EBUSY;
			dev_err(&wm0010->dev, "Boot ROM error: %x in %d\n", out[i], i);
			goto abort;
		}
	}

#ifdef	SPI_LOG_ON
	DBG_PRINTK("1st Boot ROM OK\n");
#endif
	release_firmware(fw);
	fw = NULL;
	kfree(img32);
	img32 = NULL;
	kfree(out);
	out = NULL;

	/* wait 2ms PLL clock stable */
	msleep(2);	/* PLL */

#if	1
	{
		int v_rdy = gpio_get_value(pdata->pin_ready);
		int v_rst = gpio_get_value(pdata->pin_reset);
		int v_req = gpio_get_value(pdata->pin_request);
		int v_irq = gpio_get_value(pdata->pin_xirq);
		DBG_PRINTK("RDY:%d RST:%d REQ:%d XIQ:%d\n", v_rdy, v_rst, v_req, v_irq);
	}
#endif

	/*
		2nd BOOT
	*/
	ret = request_firmware(&fw, BOOT_IMAGE_2ND, &wm0010->dev);
	if (ret != 0) {
		dev_err(&wm0010->dev, "Failed to request application: %d\n", ret);
		goto abort;
	}

	done = 0;
	out32 = NULL;
	for (i = 0; i < fw->size;) {
#ifdef	SPI_LOG_ON
		DBG_PRINTK("2nd Boot - %d\n", i);
#endif
		addr = be32_to_cpu(*(unsigned int *)&fw->data[i]);

		i += sizeof(unsigned int);
		rec_len = be16_to_cpu(*(unsigned short *)&fw->data[i]);

		i += sizeof(unsigned short);
		aligned_size = ((rec_len + 3) & 0xfffffffc) + sizeof(unsigned int);	/* 4byte align + addr */

		if (allocated_size < aligned_size) {
			if (img32) {
				kfree(img32);
				kfree(out32);
				img32 = NULL;
				out32 = NULL;
			}
			/* keep track allocated size */
			allocated_size = aligned_size;
			img32 = (unsigned int *)kzalloc(allocated_size, GFP_KERNEL);
			if (!img32) {
				dev_err(&wm0010->dev, "Falied to allocate img32\n");
			}
			out32 = (unsigned int *)kzalloc(allocated_size, GFP_KERNEL);
			if (!out32) {
				dev_err(&wm0010->dev, "Falied to allocate out32\n");
			}
		}

		/* set address field */
		img32[0] = cpu_to_be32(addr);
		/* set 0x00000000 if pad there */
		img32_in = (unsigned int *)&fw->data[i];
		for (j = 0; j < (rec_len + 3) / 4; j++) {
			img32[1 + j] = cpu_to_be32(le32_to_cpu(img32_in[j]));
		}
		/* skip data */
		i += rec_len;

		spi_message_init(&m);

		/* transmit a recoed */
		memset(&t, 0, sizeof(t));
		t.len = aligned_size;
		t.tx_buf = img32;
		t.rx_buf = out32;
		t.bits_per_word = 8;
		t.cs_change = 0;
		spi_message_add_tail(&t, &m);

		ret = spi_sync(wm0010, &m);
		if (ret != 0) {
			dev_err(&wm0010->dev, "2nd Boot Failed: addr=0x%08x, len=%d\n", addr, rec_len);
		}

		for (j = 0; j < (rec_len + 3) / 4; j++) {
			switch (be32_to_cpu(out32[j])) {
			case 0xe0e0e0e0:
				ret = -EIO;
				dev_err(&wm0010->dev, "ROM error reported in stage 2\n");
				goto abort;

			case 0x55555555:
				ret = -EBUSY;
				dev_err(&wm0010->dev, "ROM bootloader running in stage 2\n");
				goto abort;

			case 0xffffffff:
				ret = -EIO;
				dev_err(&wm0010->dev, "Application started early\n");
				goto abort;

			case 0xfffffffc:
				ret = -EIO;
				dev_err(&wm0010->dev, "Application start failed\n");
				goto abort;

			case 0xfffffffe:
				ret = -EIO;
				dev_err(&wm0010->dev, "Device reports SPI error\n");
				goto abort;

			case 0xfffffffb:
				ret = -EIO;
				dev_err(&wm0010->dev, "Device reports SPI read overflow\n");
				goto abort;

			case 0xfffffffd:
				ret = -EIO;
				dev_err(&wm0010->dev, "Bootloader exited\n");
				goto abort;

			case 0xfffffffa:
				ret = -EIO;
				dev_dbg(&wm0010->dev, "Bootloader running\n");
				break;

			case 0xfffffff9:
				ret = -EIO;
				dev_err(&wm0010->dev, "SPI underclocked\n");
				goto abort;

			default:
#ifdef	SPI_LOG_ON
				if ((j > 0) && (be32_to_cpu(out32[j]) != done)) {
					dev_dbg(&wm0010->dev, "%d(0x%x) transfers reported, %d done\n",
							be32_to_cpu(out32[j]),
							be32_to_cpu(out32[j]), done);
				} else {
					dev_dbg(&wm0010->dev, "%d transfers done\n", done);
				}
#endif
				break;
			}
		}

		/* skip padding */
		pad_size = ((rec_len + sizeof(unsigned short) + 3) & 0xfffffffc) - (rec_len + sizeof(unsigned short));
		i += pad_size;

		done++;
	}

/* end data */
	dev_dbg(&wm0010->dev, "Done %d records\n", done);

	if (img32) {
		kfree(img32);
		img32 = NULL;
		kfree(out32);
		out32 = NULL;
	}

	release_firmware(fw);
	fw = NULL;

	/*
		Firmware download is completed then READY changes LOW->HIGH
	*/
	{
		int i;
		int v;
		for (i = 0; i < READY_POLL_MAX_NUM2; i++) {
			v = gpio_get_value(pdata->pin_ready);
			if (v == 1) {
				break;
			}
			msleep(READY_POLL_INTERVAL);
		}
		if (v == 0) {
			DBG_PRINTK("READY is LOW(%d).\n", v);
			ret = -EINVAL;
			goto abort;
		}
		DBG_PRINTK("READY:Hi %d ms.\n", (i * 20));
    }

	DBG_PRINTK("1st/2nd Boot OK\n");
	return 0;
abort:
	if (fw) {
		release_firmware(fw);
	}
	if (img32) {
		kfree(img32);
	}
	if (out32) {
		kfree(out32);
	}
	if (out) {
		kfree(out);
	}
	/* Put the chip back into reset */
	wm0010_halt();
	return ret;
}

//EXPORT_SYMBOL_GPL(wm0010_boot);

/*
 *	wm0010_resume
 *
 *	resume from suspend<br>
 *	If DSP is active(resumed from suspended during phone call active) 
 *	set DSP power CORE=ON,IO=ON, RESET signal to LOW.
 *
 *	@param	spi	[in]	spi_device*
 *	@return	0	success
 */
static int wm0010_resume(struct spi_device *spi)
{
	struct wm0010_pdata *pdata = dev_get_platdata(&spi->dev);
#ifdef	USE_SUSPEND
	int ret = 0;

	dev_dbg(&spi->dev, "%s()\n", __func__);

	/* DSP inactive before suspended */
	if (!pdata->running) {
#ifdef	CTRL_SPI_CLK_LOW
		dev_err(&spi->dev, "resume SPI signals\n");
		/* spi_clk */
		ret = gpio_tlmm_config(GPIO_CFG(45, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		if (ret) {
			dev_err(&spi->dev, "config(45)=%d\n", ret);
		}
		/* spi_cs0 */
		ret = gpio_tlmm_config(GPIO_CFG(46, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (ret) {
			dev_err(&spi->dev, "config(46)=%d\n", ret);
		}
		/* spi_mosi */
		ret = gpio_tlmm_config(GPIO_CFG(47, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		if (ret) {
			dev_err(&spi->dev, "config(47)=%d\n", ret);
		}
		/* spi_miso */
		ret = gpio_tlmm_config(GPIO_CFG(48, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (ret) {
			dev_err(&spi->dev, "config(48)=%d\n", ret);
		}
		/* GP_CLK_B */
		ret = gpio_tlmm_config(GPIO_CFG(98, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), GPIO_CFG_ENABLE);
		if (ret) {
			dev_err(&spi->dev, "config(98)=%d\n", ret);
		}
#endif	/* CTRL_SPI_CLK_LOW */

	/* setup GPIO settings */
		ret = setup_config(pdata);
		if (ret) {
			dev_err(&spi->dev, "%s(): setup_config() = %d\n", __func__, ret);
		}
		/* Assert RESET */
		gpio_set_value(pdata->pin_reset, 0);
		usleep(1);

		ret = vreg_set_level(pdata->vreg_core, 1200);	/* core 1.2V */
		if (ret) {
			dev_err(&spi->dev, "vreg_set_level(CORE, 1200) = %d\n", ret);
		}
		ret = vreg_enable(pdata->vreg_core);
		if (ret) {
			dev_err(&spi->dev, "%s(): vreg_enable(CORE) = %d\n", __func__, ret);
		}
		ret = vreg_set_level(pdata->vreg_io, 1800);	/* IO 1.8V */
		if (ret) {
			dev_err(&spi->dev, "vreg_set_level(IO, 1800) = %d\n", ret);
		}
		ret = vreg_enable(pdata->vreg_io);
		if (ret) {
			dev_err(&spi->dev, "%s(): vreg_enable(IO) = %d\n", __func__, ret);
		}

#ifdef	VREG_CORE_OFF
		ret = vreg_disable(pdata->vreg_core);
		if (ret) {
			dev_err(&spi->dev, "%s(): vreg_disable(CORE) = %d\n", __func__, ret);
		}
#endif	/*	VREG_CORE_OFF */
		cleanup_config(pdata);
	}
#endif
	pdata->resume++;
	dev_dbg(&spi->dev, "%s()\n", __func__);
	return 0;
}

/*
 *	wm0010_suspend
 *
 *	goto suspend<br>
 *	power and SPI signals are kept when DSP active.
 *
 *	@param	spi	[in]		spi_device*
 *	@param	mesg	[in]	pm_message_t
 *	@return	0	success
 */
static int wm0010_suspend(struct spi_device *spi, pm_message_t mesg)
{
	int ret = 0;
	struct wm0010_pdata *pdata = dev_get_platdata(&spi->dev);

	/* DSP is active */
	if (pdata->running) {
		pdata->suspend_withrun++;
	} else {	/* DSP is not active */
		pdata->suspend++;
	}
#ifdef	USE_SUSPEND

	dev_dbg(&spi->dev, "%s(), msg=%d\n", __func__, mesg.event);

	/* power off after close invoked. */
	if (!pdata->running) {
#ifndef	VREG_CORE_OFF
		vreg_enable(pdata->vreg_core);
#endif	/* VREG_CORE_OFF */
		vreg_disable(pdata->vreg_io);
		vreg_disable(pdata->vreg_core);

#ifdef	CTRL_SPI_CLK_LOW
		dev_err(&spi->dev, "suspend SPI signals\n");
		/* spi_clk */
		ret = gpio_tlmm_config(GPIO_CFG(45, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (ret) {
			dev_err(&spi->dev, "config(45)=%d\n", ret);
		}
		/* spi_cs0 */
		ret = gpio_tlmm_config(GPIO_CFG(46, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (ret) {
			dev_err(&spi->dev, "config(46)=%d\n", ret);
		}
		/* spi_mosi */
		ret = gpio_tlmm_config(GPIO_CFG(47, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (ret) {
			dev_err(&spi->dev, "config(47)=%d\n", ret);
		}
		/* spi_miso */
		ret = gpio_tlmm_config(GPIO_CFG(48, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (ret) {
			dev_err(&spi->dev, "config(48)=%d\n", ret);
		}
		/* GP_CLK_B */
		ret = gpio_tlmm_config(GPIO_CFG(98, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (ret) {
			dev_err(&spi->dev, "config(98)=%d\n", ret);
		}

		gpio_set_value(45, 0);	/* spi_clk = LOW */
		gpio_set_value(46, 1);	/* spi_cs0 = HIGH */
		gpio_set_value(47, 0);	/* spi_mosi = LOW */
		gpio_set_value(48, 0);	/* spi_miso = LOW */
		gpio_set_value(98, 0);	/* GP_CLK_B = LOW */

		ret = 0;
#endif	/* CTRL_SPI_CLK_LOW */
	}
#endif
	dev_dbg(&spi->dev, "%s(), msg=%d\n", __func__, mesg.event);
	return ret;
}

#if 0
static int wm0010_resume_pm(struct device *spi)
{
	pr_err("%s()\n", __func__);
	return 0;
}

static int wm0010_suspend_pm(struct device *spi){
	pr_err("%s()\n", __func__);
	return 0;
}
#endif

/**
 *	wm0010_spi_probe
 *
 *	Probe<br>
 *	initialize DSP
 *
 *	@param	spi	[in]	spi_device
 *	@return	0	success<br>
 *			<0	fail
 */
static int __devinit wm0010_spi_probe(struct spi_device *spi)
{
	int ret;

	struct wm0010_pdata *pdata = NULL;

	DBG_PRINTK("wm0010_spi_probe\n");

	pdata = (struct wm0010_pdata *)kzalloc(sizeof(struct wm0010_pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&spi->dev, "Please provide valid platform data\n");
		return -EINVAL;
	}

	pdata->pin_reset = 0;
	pdata->pin_request = 0;
	pdata->pin_ready = 0;
	pdata->pin_xirq = 0;
	pdata->gpio_request_irq = 0;
#ifdef	USE_IRQ_READY
	pdata->gpio_ready_irq = 0;
#endif
#ifdef	MONITOR_IRQ
	pdata->gpio_xirq_irq = 0;
#endif
	pdata->vreg_core = NULL;
	pdata->vreg_io = NULL;
	pdata->clk_b = NULL;

	pdata->request_raised = 0;
	pdata->ready_raised = 0;
	
	pdata->suspend = 0;
	pdata->suspend_withrun = 0;
	pdata->resume = 0;

	pdata->running = 0;
	pdata->cancel_read = 0;

	init_waitqueue_head(&pdata->wq_request);
#ifdef	USE_IRQ_READY
	init_waitqueue_head(&pdata->wq_ready);
#endif

	setup_probe(pdata);

	/* power off CORE because CORE is on Power-On-Default */
	ret = setup_config(pdata);
	if (ret) {
		dev_err(&spi->dev, "setup_config() = %d\n", ret);
		setup_remove(pdata);
		kfree(pdata);
		return ret;
	}

	/* assert reset */
	gpio_set_value(pdata->pin_reset, 0);
	usleep(1);

	ret = vreg_set_level(pdata->vreg_core, 1200);	/* core 1.2V */
	if (ret) {
		dev_err(&spi->dev, "vreg_set_level(CORE, 1200) = %d\n", ret);
		goto ON_ERR;
	}
	ret = vreg_enable(pdata->vreg_core);
	if (ret) {
		dev_err(&spi->dev, "vreg_enable(CORE) = %d\n", ret);
		goto ON_ERR;
	}
	ret = vreg_set_level(pdata->vreg_io, 1800);	/* IO 1.8V */
	if (ret) {
		dev_err(&spi->dev, "vreg_set_level(IO, 1800) = %d\n", ret);
		goto ON_ERR;
	}
	ret = vreg_enable(pdata->vreg_io);
	if (ret) {
		dev_err(&spi->dev, "vreg_enable(IO) = %d\n", ret);
		goto ON_ERR;
	}

	// disable CLK_B */
	ret = clk_enable(pdata->clk_b);				/* enable clock */
	if (ret) {
		dev_err(&spi->dev, "clk_enable() = %d\n", ret);
		goto ON_ERR;
	}
	clk_disable(pdata->clk_b);					/* disable clock */

	ret = cleanup_config(pdata);
	if (ret) {
		dev_err(&spi->dev, "cleanup_config()=%d\n", ret);
		setup_remove(pdata);
		kfree(pdata);
		return ret;
	}

	wm0010_miscdev.parent = &spi->dev;
	ret = misc_register(&wm0010_miscdev);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register miscdev: %d\n", ret);
		setup_remove(pdata);
		kfree(pdata);
		return ret;
	}

#ifdef	VREG_CORE_OFF
	vreg_disable(pdata->vreg_core);
#endif	/*	VREG_CORE_OFF */

	wm0010 = spi;
	spi->dev.platform_data = pdata;
	spi->chip_select = 0;						/* CS0:DSP */

#ifdef	CTRL_SPI_CLK_LOW
	dev_err(&spi->dev, "Probe SPI signals\n");
	/* spi_clk */
	ret = gpio_tlmm_config(GPIO_CFG(45, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	if (ret) {
		dev_err(&spi->dev, "config(45)=%d\n", ret);
	}
	/* spi_cs0 */
	ret = gpio_tlmm_config(GPIO_CFG(46, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (ret) {
		dev_err(&spi->dev, "config(46)=%d\n", ret);
	}
	/* spi_mosi */
	ret = gpio_tlmm_config(GPIO_CFG(47, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	if (ret) {
		dev_err(&spi->dev, "config(47)=%d\n", ret);
	}
	/* spi_miso */
	ret = gpio_tlmm_config(GPIO_CFG(48, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (ret) {
		dev_err(&spi->dev, "config(48)=%d\n", ret);
	}
	/* GP_CLK_B */
	ret = gpio_tlmm_config(GPIO_CFG(98, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), GPIO_CFG_ENABLE);
	if (ret) {
		dev_err(&spi->dev, "config(98)=%d\n", ret);
	}
#endif	/* CTRL_SPI_CLK_LOW */

	return 0;

ON_ERR:
	clk_disable(pdata->clk_b);
	vreg_disable(pdata->vreg_io);
	vreg_disable(pdata->vreg_core);
	cleanup_config(pdata);
	setup_remove(pdata);
	kfree(pdata);

	return ret;
}

/**
 *	wm0010_spi_remove
 *
 *	remove<br>
 *	deregister driver it was registered in Probe
 *
 *	@param	spi	[in]	spi_device*
 *	@return	0	success<br>
 */
static int __devexit wm0010_spi_remove(struct spi_device *spi)
{
	struct wm0010_pdata *pdata = dev_get_platdata(&spi->dev);

	dev_err(&spi->dev, "%s\n", __func__);

	if (pdata) {
		setup_config(pdata);

		vreg_disable(pdata->vreg_io);
		vreg_disable(pdata->vreg_core);

		cleanup_config(pdata);

		setup_remove(pdata);

		kfree(pdata);
	}

	wm0010 = NULL;
	misc_deregister(&wm0010_miscdev);

	return 0;
}


/* wm0010 driver as spi child driver */
static struct spi_driver wm0010_spi_driver = {
	.driver = {
		.name	= "wm0010_wolfson",						/* device object which connect to */
		.bus 	= &spi_bus_type,						/* driver bus type */
		.owner	= THIS_MODULE,							/* owner */
	},
	.probe		= wm0010_spi_probe,						/* Probe Entry */
	.remove		= __devexit_p(wm0010_spi_remove),		/* Remove Entry */
	.suspend	= wm0010_suspend,						/* Suspend Entry */
	.resume		= wm0010_resume,						/* Resume Entry */
};

/**
 *	wm0010_init
 *
 *	Initialize driver<br>
 *	register with spi_register_driver
 *
 *	@return	0	success<br>
 *			<0	fail
 */
static int __init wm0010_init(void)
{
	int ret;
	DBG_PRINTK("wm0010_init\n");

	ret = spi_register_driver(&wm0010_spi_driver);
	if (ret) {
		DBG_PRINTK("wm0010_init ret=%d\n", ret);
	}
	return ret;
}
module_init(wm0010_init);

/**
 *	wm0010_exit
 *
 *	finalize driver<br>
 *	deregister spi_register_driver
 */
static void __exit wm0010_exit(void)
{
	DBG_PRINTK("wm0010_exit\n");
	spi_unregister_driver(&wm0010_spi_driver);
}
module_exit(wm0010_exit);

MODULE_DESCRIPTION("ASoC WM0010 driver");
MODULE_AUTHOR("Fujitsu");
MODULE_LICENSE("GPL");
