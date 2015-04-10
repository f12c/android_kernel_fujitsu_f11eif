/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#ifndef _WM0010_H_
#define _WM0010_H_

#include <linux/clk.h>
#include <mach/vreg.h>
#include <linux/spinlock.h>

int wm0010_boot(void);
int wm0010_halt(void);

/// private data
struct wm0010_pdata {
	int pin_reset;				/* RESET pin number */
	int pin_request;			/* REQUEST pin number */
	int gpio_request_irq;		/* REQUEST IRQ number */
	int gpio_xirq_irq;			/* XIRQ IRQ number */
	int pin_ready;				/* READY pin number */
	int pin_xirq;				/* XIRQ pin number */
	struct vreg *vreg_core;		/* CORE */
	struct vreg *vreg_io;		/* IO */
	struct clk* clk_b;			/* GP_CLK_B */

	int request_raised;			/* REQUEST interrupt count */
	int ready_raised;			/* READY interrupt count */
	wait_queue_head_t	wq_request;	/* REQUEST interrupt event queue */

	/* request count */
	int suspend;				/* suspend invoked while DSP in active */
	int suspend_withrun;		/* suspend invoked while DSP active */
	int resume;					/* resume requested */

	int cancel_read;			/* IOCTL_CANCEL_READ requested */
	int running;				/* Opened */
};

#endif
