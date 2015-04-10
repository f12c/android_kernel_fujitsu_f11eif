/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/


#ifndef _FJ_WALKMOTION_H
#define _FJ_WALKMOTION_H


#include <linux/ioctl.h>

#define FJ_WM_IOC_MAGIC '~'

/* Initialize */
#define FJ_WM_IOCT_INITIALIZE 		_IO(FJ_WM_IOC_MAGIC, 0)
/* Cancel initialize */
#define FJ_WM_IOCT_CANCELINITIALIZE	_IO(FJ_WM_IOC_MAGIC, 1)
/* Request IRQ */
#define FJ_WM_IOCS_REQUESTMOTIONIRQ	_IOW(FJ_WM_IOC_MAGIC, 2, unsigned int)
/* Cancel request IRQ */
#define FJ_WM_IOCT_CANCELMOTIONIRQ	_IO(FJ_WM_IOC_MAGIC, 3)
/* Set interrupt terminal */
#define FJ_WM_IOCS_SETSCIFACONTROL	_IOW(FJ_WM_IOC_MAGIC, 4, unsigned int)

/* Detection of high edge */
#define FJ_WM_EDGE_HIGH			1
/* Detection of low edge */
#define FJ_WM_EDGE_LOW			0
/* UART port */
#define FJ_WM_MODE_GPIO			1
/* GPIO port */
#define FJ_WM_MODE_UART			0

/* Walk Motion MC Platform Data */
struct fj_wm_platform_data {
	/* Motion IRQ */
	int motion_irq;
	/* Delay */
	int mc_init_delay;	
};

#endif /** _FJ_WALKMOTION_H */
