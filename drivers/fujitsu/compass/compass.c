/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/compass.h>
#include "hscdtd002a.h"

static const char COMPASS_CLASS_NAME[] = "compass";
#define COMPASS_DRIVER_NAME COMPASS_CLASS_NAME

static dev_t compass_devno = COMPASS_DEVNO;
static struct class *compass_class = NULL;
static struct cdev compass_cdev;

static struct file_operations compass_fops = {
	.open		= hscd_open,
	.release	= hscd_release,
	.ioctl		= hscd_ioctl,
};

static const struct i2c_device_id compass_id2[] = {
		{"compass", 4},	//Ver2
};
static const struct i2c_device_id compass_id3[] = {
		{"compass", 0},	//Ver3
};


/*--------------------------------------------------------------------*
 * Function:When the device starts,
 *           character device  register and Initialization of hardware*
 *--------------------------------------------------------------------*/
static int compass_probe
(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev;
	int result;

	compass_printk(KERN_INFO, "%s: module loaded\n", COMPASS_CLASS_NAME);

	hscd->reg.st_client = client;	// Client information is secured.

	// majors device number is automatically allocated
	// register a range of minor device numbers
	result = alloc_chrdev_region (&compass_devno, COMPASS_DEVNO_MINOR,
			COMPASS_NDEVICES, COMPASS_DRIVER_NAME);
	if (unlikely(result < 0)) {
		compass_printk(KERN_ERR, "alloc_chrdev_region() failed (%d).\n", 
																	result);
		goto Exit;
	}

	compass_printk(KERN_INFO, "%s device number: %u.%u\n",
		COMPASS_CLASS_NAME, MAJOR(compass_devno), MINOR(compass_devno));

	// compass_cdev of character device to initialize and register the system
	cdev_init(&compass_cdev, &compass_fops);
	compass_cdev.owner = THIS_MODULE;
	result = cdev_add(&compass_cdev, compass_devno, COMPASS_NDEVICES);
	if (unlikely(result < 0)) {
		compass_printk(KERN_ERR, "cdev_add() failed.\n");
		goto Error1;
	}

	// create a device class
	compass_class = class_create(THIS_MODULE, COMPASS_CLASS_NAME);
	if (IS_ERR(compass_class)) {
		result = PTR_ERR(compass_class);	// result <- -errno
		compass_printk(KERN_ERR, "class_create() failed. (%d)\n", result);
		goto Error2;
	}

	// Create a class devices, sysfs to register.
	// /sys/bus/platform/drivers/compass/
	dev = device_create (compass_class, NULL, compass_devno,
		hscd, "%s", COMPASS_DRIVER_NAME);
	if (unlikely(IS_ERR (dev))) {
		result = PTR_ERR(dev);	// result <- -errno
		compass_printk(KERN_ERR, "device_create() failed (%d).\n", result);
		goto Error3;
	}

	hscd_module_init();		// HSCD module initialization

Exit:
	compass_printk(KERN_DEBUG, "%s returned %d\n", __FUNCTION__, result);
	return result;

Error3:
	class_destroy(compass_class);
Error2:
	compass_class = NULL;
	cdev_del(&compass_cdev);
Error1:
	unregister_chrdev_region(compass_devno, COMPASS_NDEVICES);
	goto Exit;
}

/*--------------------------------------------------------------------*
 * Function:When the device ends,
 *           hardware is stopped and character device is deleted.     *
 *--------------------------------------------------------------------*/
static int compass_remove(struct i2c_client *client)
{
	compass_printk(KERN_DEBUG, "%s\n", __FUNCTION__);

	hscd_module_term();		// HSCD module release

	device_destroy(compass_class, compass_devno);
	if (compass_class != NULL) {
		class_destroy(compass_class);
		compass_class = NULL;
	}
	cdev_del(&compass_cdev);
	unregister_chrdev_region(compass_devno, COMPASS_NDEVICES);
	return OK;
}

static struct i2c_driver compass_driver2 = {
	.probe = compass_probe,
	.remove = compass_remove,
	.id_table = compass_id2,
	.driver = {
		.name = COMPASS_DRIVER_NAME,
	},
};
static struct i2c_driver compass_driver3 = {
	.probe = compass_probe,
	.remove = compass_remove,
	.id_table = compass_id3,
	.driver = {
		.name = COMPASS_DRIVER_NAME,
	},
};

/*--------------------------------------------------------------------*
 * Function:When the device starts,                                   *
 *           magnetism sensor driver is registered in the kernel.     *
 *--------------------------------------------------------------------*/
static int __devinit compass_init(void)
{
	int res;

	compass_printk(KERN_INFO, "Initializing %s module\n", COMPASS_CLASS_NAME);

	if(system_rev >= 0x0c) {
		res = i2c_add_driver(&compass_driver2);
	} else {
		res = i2c_add_driver(&compass_driver3);
	}
	return res;
}

/*--------------------------------------------------------------------*
 * Function:When the device ends,                                     *
 *           magnetism sensor driver is deleted from the kernel.      *
 *--------------------------------------------------------------------*/
static void __exit compass_exit(void)
{
	compass_printk(KERN_INFO, "Exiting %s module\n", COMPASS_CLASS_NAME);

	if(system_rev >= 0x0c) {
		i2c_del_driver (&compass_driver2);
	} else {
		i2c_del_driver (&compass_driver3);
	}
}

module_init (compass_init);
module_exit (compass_exit);

MODULE_AUTHOR ("FUJITSU,2011");
MODULE_DESCRIPTION ("compass driver");
MODULE_LICENSE ("GPL");
