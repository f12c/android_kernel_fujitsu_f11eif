/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#include <linux/device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/walkmotion.h>
#include <linux/ioctl.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/delay.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>

/* Driver name */
#define DRIVER_NAME "fj-walkmotion"
/* Delay */
#define FJ_WM_DEF_MC_INIT_DELAY 20

/** State bit */
/** Not initialize */
#define STS_BIT_MC_UNINIT           0x01
/** Initializing */
#define STS_BIT_MC_INIT_ONGOING     0x02
/** Initialized */
#define STS_BIT_MC_INIT             0x04
/** Polling IRQ */
#define STS_BIT_MOTION_POLLING      0x10
/** Initializer */
#define FJ_WM_STS_INITIALIZER       0x00

/** Not initialize */
#define FJ_WM_STS_MC_UNINITIALIZED   (STS_BIT_MC_UNINIT)
/** Initializing */
#define FJ_WM_STS_MC_INIT_ONGOING    (STS_BIT_MC_INIT_ONGOING)
/** Wait */
#define FJ_WM_STS_MC_INIT_WAIT \
                           (STS_BIT_MC_INIT_ONGOING|STS_BIT_MOTION_POLLING)
/** Initialized */
#define FJ_WM_STS_MC_INITIALIZED     (STS_BIT_MC_INIT)
/** Polling IRQ */
#define FJ_WM_STS_MOTION_IRQ_POLLING (STS_BIT_MC_INIT|STS_BIT_MOTION_POLLING)

#define FJ_WM_GPIO_RESET          93   /** Reset */
#define GPIO_HIGH                  1   /** High */
#define GPIO_LOW                   0   /** Low */

#define RC5T712_IOSEL2           0x21
#define RC5T712_IOOUT1           0x23
#define RC5T712_IOOUT2           0x24
#define RC5T712_GPPUPD2H         0x2C
#define RC5T712_UART1SEL         0x60
#define RC5T712_UART1PUPD        0x61
#define RC5T712_UART1TXMODE      0x62

#define debug_printk(format, arg...) \
    if (0) printk("%s" format, KERN_DEBUG, ##arg)

/* Walk motion data */
struct fj_wm_data {
	/** Driver state */
	int                state;
	/** State lock */
	struct mutex       state_lock;
	/** Motion IRQ wait queue */
	wait_queue_head_t  weit_queue_motion_irq;
	/** Motion IRQ */
	int                motion_irq;
	/** Delay */
	int                mc_init_delay;
	/** Device for debug */
	struct device      *dbg_dev;
	/** enable/disable irq flag */
	int                irq_flag;
};
static struct fj_wm_data *wm_data;

static void fj_wm_mc_init(void);
static void fj_wm_motion_irq(struct work_struct *);
static DECLARE_WORK(work_queue_motion_irq, fj_wm_motion_irq);

/** Interrupt handler
 *
 * @param irq  : Not use
 * @param data : Not use
 * @return IRQ_HANDLED
 */
static irqreturn_t fj_wm_irq_handler(int irq, void *data)
{
	debug_printk("%s : start\n", __func__);
	schedule_work(&work_queue_motion_irq);

	return IRQ_HANDLED;
}

/** Observe motion irq
 *
 * @param ws : Not use
 * @return void
 */
static void fj_wm_motion_irq(struct work_struct *ws)
{
	debug_printk("%s : start\n", __func__);
	mutex_lock(&wm_data->state_lock);
	/* State check */
	if (!(wm_data->state & STS_BIT_MOTION_POLLING)) {
		/* Finish observe */
		mutex_unlock(&wm_data->state_lock);
		debug_printk("%s : ign irq (state : %#04x)\n", __func__, wm_data->state);
		return;
	} else if (wm_data->state & STS_BIT_MC_INIT_ONGOING) {
		debug_printk("%s : first motion_irq\n", __func__);
		/* Change state to FJ_WM_STS_MC_INITIALIZED */
		wm_data->state = FJ_WM_STS_MC_INITIALIZED;
	} else {
		debug_printk("%s : normal motion_irq\n", __func__);
		/* Clear STS_BIT_MOTION_POLLING */
		wm_data->state &= ~STS_BIT_MOTION_POLLING;
	}

	wake_up_interruptible(&wm_data->weit_queue_motion_irq);

	mutex_unlock(&wm_data->state_lock);
}

/** Walk motion MC initialization
 *
 * @param ws : Work struct
 * @return void
 */
static void fj_wm_mc_init(void)
{
	
	debug_printk("%s : start\n", __func__);
	mutex_lock(&wm_data->state_lock);

	gpio_set_value(FJ_WM_GPIO_RESET, GPIO_LOW);

	mutex_unlock(&wm_data->state_lock);

	/* Wait for 20ms */
	msleep(wm_data->mc_init_delay);

	mutex_lock(&wm_data->state_lock);

	gpio_set_value(FJ_WM_GPIO_RESET, GPIO_HIGH);

	/* Change state to FJ_WM_STS_MC_INIT_WAIT */
	wm_data->state = FJ_WM_STS_MC_INIT_WAIT;

	mutex_unlock(&wm_data->state_lock);

	return;
}


/** Get driver resource
 *
 * @param inode : Not use
 * @param file  : Not use
 * @return 0 Success
 */
static int fj_wm_open(struct inode *inode, struct file *file)
{
	debug_printk("%s : start\n", __func__);
	/* Have nothing to do */
	return 0;
}

/** Release driver resource
 * @param inode : Not use
 * @param file  : Not use
 * @return 0 Success
 */
static int fj_wm_release(struct inode *inode, struct file *file)
{
	/* Release IRQ resource */
	debug_printk("%s : start\n", __func__);
	if(wm_data->irq_flag == 1) {
		free_irq(wm_data->motion_irq, wm_data);
		wm_data->irq_flag = 0;
		debug_printk("%s : irq_flag = %d\n", __func__, wm_data->irq_flag);
	}
	
	mutex_lock(&wm_data->state_lock);
	wm_data->state = FJ_WM_STS_INITIALIZER;
	mutex_unlock(&wm_data->state_lock);
	
	return 0;
}

/** Control device
 *
 * @param inode : Not use
 * @param file  : Not use
 * @param cmd   : Control command
 * @param arg   : Argument
 * @return 0 Success
 */
static int
fj_wm_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	                                                      unsigned long arg)
{
	int ret;

	switch (cmd) {
	/** Initialize MC */
	case FJ_WM_IOCT_INITIALIZE:
		debug_printk("FJ_WM_IOCT_INITIALIZE : start\n");

		mutex_lock(&wm_data->state_lock);
		
		/* Check state */
		if (wm_data->state & STS_BIT_MC_INIT_ONGOING) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCT_INITIALIZE : illegal state (state : "
				                                   "%#04x)\n", wm_data->state);
			return -EINVAL;
		}
		
		/* Change state to FJ_WM_STS_MC_INIT_ONGOING */
		wm_data->state = FJ_WM_STS_MC_INIT_ONGOING;

		mutex_unlock(&wm_data->state_lock);

		fj_wm_mc_init();

		if(wm_data->irq_flag == 0) {
			ret = request_irq(wm_data->motion_irq, fj_wm_irq_handler,
			                            IRQF_TRIGGER_RISING, "fj_wm", wm_data);
			if (ret < 0) {
				mutex_lock(&wm_data->state_lock);
				wm_data->state = FJ_WM_STS_MC_UNINITIALIZED;
				mutex_unlock(&wm_data->state_lock);
				printk(KERN_ERR "FJ_WM_IOCT_INITIALIZE : request_irq failed "
					                                      "(ret : %d)\n", ret);
				return ret;
			}
			wm_data->irq_flag = 1;
			debug_printk("FJ_WM_IOCT_INITIALIZE : irq_flag = %d\n",
				                                            wm_data->irq_flag);
		}
		
		/* Wait IRQ */
		if ((ret = wait_event_interruptible_timeout(
			                        wm_data->weit_queue_motion_irq,
			                        wm_data->state == FJ_WM_STS_MC_INITIALIZED,
			                        msecs_to_jiffies(250))) <= 0) {
			/* If canceled */
			
			mutex_lock(&wm_data->state_lock);
			/* Change state to FJ_WM_STS_MC_UNINITIALIZED */
			wm_data->state = FJ_WM_STS_MC_UNINITIALIZED;
			mutex_unlock(&wm_data->state_lock);
			                    	
			printk(KERN_ERR "FJ_WM_IOCT_INITIALIZE : wait_event_"
				           "interruptible_timeout canceled (ret : %d)\n", ret);
			return -ECANCELED;
		}

		break;

	/** Cancel initialization MC */
	case FJ_WM_IOCT_CANCELINITIALIZE:
		debug_printk("FJ_WM_IOCT_CANCELINITIALIZE : start\n");

		mutex_lock(&wm_data->state_lock);

		/* Check state */
		if (!(wm_data->state & STS_BIT_MC_INIT_ONGOING)) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCT_CANCELINITIALIZE : illegal state "
				                          "(state : %#04x)\n", wm_data->state);
			return -EINVAL;
		} else if (wm_data->state & STS_BIT_MOTION_POLLING) {
			/* Wake up */
			wake_up_interruptible(&wm_data->weit_queue_motion_irq);
		}

		/* Clear motion IRQ */
		if(wm_data->irq_flag == 1) {
			free_irq(wm_data->motion_irq, wm_data);
			wm_data->irq_flag = 0;
			debug_printk("FJ_WM_IOCT_CANCELINITIALIZE : irq_flag = %d\n",
				                                            wm_data->irq_flag);
		}

		mutex_unlock(&wm_data->state_lock);

		break;

	/** Request motiot IRQ */
	case FJ_WM_IOCS_REQUESTMOTIONIRQ:
		debug_printk("FJ_WM_IOCS_REQUESTMOTIONIRQ : start\n");

		mutex_lock(&wm_data->state_lock);

		/* Check state */
		if (wm_data->state != FJ_WM_STS_MC_INITIALIZED) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCS_REQUESTMOTIONIRQ : illegal state "
				                          "(state : %#04x)\n", wm_data->state);
			return -EINVAL;
		}

		/* Clear motion IRQ */
		if(wm_data->irq_flag == 1) {
			free_irq(wm_data->motion_irq, wm_data);
			wm_data->irq_flag = 0;
			debug_printk("FJ_WM_IOCS_REQUESTMOTIONIRQ : irq_flag = %d\n",
				                                            wm_data->irq_flag);
		}

		if (arg == FJ_WM_EDGE_HIGH) {
			/* High-edge detection */
			debug_printk("FJ_WM_IOCS_REQUESTMOTIONIRQ : FJ_WM_EDGE_HIGH\n");
			if(wm_data->irq_flag == 0) {
				ret = request_irq(wm_data->motion_irq, fj_wm_irq_handler,
				                        IRQF_TRIGGER_RISING, "fj_wm", wm_data);
				if (ret < 0) {
					mutex_unlock(&wm_data->state_lock);
					printk(KERN_ERR "FJ_WM_IOCS_REQUESTMOTIONIRQ"
						 "(FJ_WM_EDGE_HIGH) : request_irq failed (ret : %d)\n",
						                                                  ret);
					return  ret;
				}
				wm_data->irq_flag = 1;
				debug_printk("FJ_WM_IOCS_REQUESTMOTIONIRQ(FJ_WM_EDGE_HIGH) : "
					                     "irq_flag = %d\n", wm_data->irq_flag);
			}
		} else if (arg == FJ_WM_EDGE_LOW) {
			/* Low-edge detection */
			debug_printk("FJ_WM_IOCS_REQUESTMOTIONIRQ : FJ_WM_EDGE_LOW\n");
			
			if(wm_data->irq_flag == 0) {
				ret = request_irq(wm_data->motion_irq, fj_wm_irq_handler,
				                       IRQF_TRIGGER_FALLING, "fj_wm", wm_data);
				if (ret < 0) {
					mutex_unlock(&wm_data->state_lock);
					printk(KERN_ERR "FJ_WM_IOCS_REQUESTMOTIONIRQ"
						  "(FJ_WM_EDGE_LOW) : request_irq failed (ret : %d)\n",
						                                                  ret);
					return  ret;
				}
				wm_data->irq_flag = 1;
				debug_printk("FJ_WM_IOCS_REQUESTMOTIONIRQ(FJ_WM_EDGE_LOW) : "
					                     "irq_flag = %d\n", wm_data->irq_flag);
			}
		} else {				/* Invalid arg */
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCS_REQUESTMOTIONIRQ : invalid argument "
				                                         "(arg : %ld)\n", arg);
			return -EINVAL;
		}

		wm_data->state |= STS_BIT_MOTION_POLLING; /* Set polling bit */

		mutex_unlock(&wm_data->state_lock);

		/* Sleep */
		if ((ret = wait_event_interruptible(wm_data->weit_queue_motion_irq,
			                    !(wm_data->state & STS_BIT_MOTION_POLLING)))) {
			/* If canceld request motion IRQ */

			mutex_lock(&wm_data->state_lock);
			/* Clear STS_BIT_MOTION_POLLING */
			wm_data->state &= ~STS_BIT_MOTION_POLLING;

			mutex_unlock(&wm_data->state_lock);
			printk("FJ_WM_IOCS_REQUESTMOTIONIRQ : wait_event_interruptible "
				                                 "canceled (ret : %d)\n", ret);
			return -ECANCELED;
		}
		
		break;

	/** Cancel request motion IRQ */
	case FJ_WM_IOCT_CANCELMOTIONIRQ:
		debug_printk("FJ_WM_IOCT_CANCELMOTIONIRQ : start\n");

		mutex_lock(&wm_data->state_lock);

		/* Check state */
		if (!(wm_data->state & STS_BIT_MOTION_POLLING)) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCT_CANCELMOTIONIRQ : illegal state(1) "
				                          "(state : %#04x)\n", wm_data->state);
			return -EINVAL; /* Error */
		}
		if (wm_data->state & STS_BIT_MC_INIT_ONGOING) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCT_CANCELMOTIONIRQ : illegal state(2) "
				                          "(state : %#04x)\n", wm_data->state);
			return -EINVAL; /* Error */
		}

		wm_data->state &= ~STS_BIT_MOTION_POLLING;
		
		wake_up_interruptible(&wm_data->weit_queue_motion_irq);

		mutex_unlock(&wm_data->state_lock);

		break;

	/** Set terminal */
	case FJ_WM_IOCS_SETSCIFACONTROL:
		debug_printk("FJ_WM_IOCS_SETSCIFACONTROL : start\n");

		mutex_lock(&wm_data->state_lock);

		/* Check state */
		if (!(wm_data->state & STS_BIT_MC_INIT)) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCS_SETSCIFACONTROL : illegal state "
				                          "(state : %#04x)\n", wm_data->state);
			return -EINVAL; /* Error */
		}

		if (!(arg == FJ_WM_MODE_UART || arg == FJ_WM_MODE_GPIO)) {
			mutex_unlock(&wm_data->state_lock);
			printk(KERN_ERR "FJ_WM_IOCS_SETSCIFACONTROL : invalid  argument "
				                                         "(arg = %ld)\n", arg);
			return -EINVAL;
		}

		mutex_unlock(&wm_data->state_lock);

		break;

	default:
		printk(KERN_ERR "%s : invalid command (cmd : %d)\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

/** Initialize file operations */
static struct file_operations fj_wm_fileops = {
	.owner		= THIS_MODULE,
	.open		= fj_wm_open,		/** open */
	.release	= fj_wm_release,	/** release */
	.ioctl		= fj_wm_ioctl		/** ioctl */
};

/** Initialize miscdevice */
static struct miscdevice fj_wm_device = {
	.minor = MISC_DYNAMIC_MINOR,		/** Minor number */
	.name = "fj_wm_ctl",			/** Device node */
	.fops = &fj_wm_fileops			/** File operations */
};

/** Set up module
 *
 * @param pdev : Not use
 * @return 0  Success
 */
static int __devinit
fj_walkmotion_probe(struct platform_device *pdev)
{
	struct fj_wm_platform_data *pdata;
	int ret;

	debug_printk("%s : start\n", __func__);
	wm_data = kzalloc(sizeof(*wm_data), GFP_KERNEL);
	if (!wm_data) {
		printk(KERN_ERR "%s : could not allocate memory\n", __func__);
		return -ENOMEM;
	}

	/* Register miscdevice */
	ret = misc_register(&fj_wm_device);
	if (ret) {
		printk(KERN_ERR "%s : could not register misc_device (ret : %d)\n",
			                                                    __func__, ret);
		goto err_misc;
	}

	mutex_init(&wm_data->state_lock);			/* Initialize */

	/* Initialize work queue */
	INIT_WORK(&work_queue_motion_irq, fj_wm_motion_irq);
	/* Initialize wait queue */
	init_waitqueue_head(&wm_data->weit_queue_motion_irq);

	pdata = pdev->dev.platform_data;
	if (pdata->mc_init_delay < 0) {
		wm_data->mc_init_delay = FJ_WM_DEF_MC_INIT_DELAY;
	} else {
		wm_data->mc_init_delay = pdata->mc_init_delay;
	}
	wm_data->motion_irq = pdata->motion_irq;

	wm_data->state = FJ_WM_STS_INITIALIZER;
	wm_data->dbg_dev = &pdev->dev;
	wm_data->irq_flag = 0;

	return 0;

err_misc:
	kfree(wm_data);
	return ret;
}

/** Remove module
 *
 * @param pdev : not use
 * @return 0  success
 */
static int __devexit fj_walkmotion_remove(struct platform_device *pdev)
{
	debug_printk("%s : start\n", __func__);
	cancel_work_sync(&work_queue_motion_irq);
	misc_deregister(&fj_wm_device);
	kfree(wm_data);

	return 0;
}

/** Walk motion driver */
static struct platform_driver fj_walkmotion_driver = {
	.driver.name	= DRIVER_NAME,
	.driver.owner	= THIS_MODULE,
	.probe		= fj_walkmotion_probe,
	.remove		= __devexit_p(fj_walkmotion_remove),
};

/** Init module
 *
 * @return 0  Success
 *         !0 Fail
 */
static int __init fj_walkmotion_init(void)
{
	debug_printk("%s : start", __func__);
	return platform_driver_register(&fj_walkmotion_driver);
}
module_init(fj_walkmotion_init);

/** Exit module
 *
 * @return void
 */
static void __exit fj_walkmotion_exit(void)
{
	debug_printk("%s : start\n", __func__);
	platform_driver_unregister(&fj_walkmotion_driver);
}
module_exit(fj_walkmotion_exit);

MODULE_ALIAS("platform:fj-walkmotion");
MODULE_AUTHOR("FUJITSU LIMITED");
MODULE_DESCRIPTION("Fujitsu Walk Motion MC Driver");
MODULE_LICENSE("GPL");
