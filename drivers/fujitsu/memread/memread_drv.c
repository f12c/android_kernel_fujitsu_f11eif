/*
  memread Driver

  Copyright (C) 2010 TOSHIBA CORPORATION Mobile Communication Company.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/current.h>

static struct cdev mreadDevs;
static int mread_major = 0;
static struct class* mread_class;
static atomic_t mread_available = ATOMIC_INIT(2);
static pid_t app_pid = 0;

/*---------------------------------------------------------------------------
    mread_open
---------------------------------------------------------------------------*/
int mread_open(struct inode* inode,struct file* file)
{

	atomic_dec(&mread_available);
	if(atomic_read(&mread_available) < 0) {
		atomic_inc(&mread_available);
		printk("memread atomic error \n");
		return(-EBUSY);
	}
	return 0;
}

/*---------------------------------------------------------------------------
    mread_release
---------------------------------------------------------------------------*/
int mread_release(struct inode *inode, struct file *file)
{

	atomic_inc(&mread_available);
	return 0;
}

/*---------------------------------------------------------------------------
    mread_ioctl
---------------------------------------------------------------------------*/
/* IOCTL CMD */
#define MEMREAD_IOCTL_MAGIC 'm'

#define MEMREAD_IOCTL_00   _IOC( _IOC_NONE,MEMREAD_IOCTL_MAGIC, 0, 0 )
#define MEMREAD_IOCTL_01   _IOC( _IOC_NONE,MEMREAD_IOCTL_MAGIC, 1, 0 )
/* struct CMD01 */
typedef struct {
	uint32_t mem_len;
	char * mem_addr;
	char * mem_buf;
} mread_ioctl_cmd_01;
static long mread_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret;
	

	switch(cmd) {
		case MEMREAD_IOCTL_00:
			app_pid = (pid_t)arg;
			ret = 0;
			break;
		case MEMREAD_IOCTL_01:
			{
				mread_ioctl_cmd_01 __user *argp = (mread_ioctl_cmd_01 __user *)arg;
				mread_ioctl_cmd_01 param;
				if (copy_from_user(&param, argp,sizeof(mread_ioctl_cmd_01))) {
					printk("memread_drv:%s: copy_from_user failed 1\n", __func__ );
					ret = (-EFAULT);
				}
				else if (copy_to_user(param.mem_buf,param.mem_addr,param.mem_len)) {
					printk("memread_drv:%s: copy_to_user failed 3\n", __func__ );
					ret = (-EFAULT);
				}
				else {
					ret = 0;
				}
			}
			if(atomic_read(&mread_available) <= 0) {
				printk("memread MREAD_IOCTL_CMD_01 INFO\n");
			}
			break;
		default:
			ret = (-EINVAL);
			break;
	}
	return ret;
}

static struct file_operations mread_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.open = mread_open,
	.unlocked_ioctl = mread_ioctl,
	.release = mread_release,
};
/*---------------------------------------------------------------------------
    mread_setup_cdev
---------------------------------------------------------------------------*/
static void mread_setup_cdev(struct cdev *dev, int minor,
	struct file_operations *fops)
{
	int err, devno = MKDEV(mread_major, minor);
	cdev_init(dev, fops);
	dev->owner = THIS_MODULE;
	dev->ops = fops;
	err = cdev_add (dev, devno, 1);
	if (err)
		printk ("memread Error %d adding rfs%d\n", err, minor);

	if (IS_ERR(device_create(mread_class, NULL, devno, NULL, "mread")))
		printk("memread can't create device\n");
}

static int memread_pdev_probe(struct platform_device *pdev)
{
	int result = 0;
	dev_t dev;
	

	dev = MKDEV(mread_major, 0);

	if (mread_major) {
		result = register_chrdev_region(dev, 2, "mread");
	}
	else {
		result = alloc_chrdev_region(&dev, 0, 2, "mread");
		mread_major = MAJOR(dev);
	}
	if (result < 0) {
		printk("memread error fail to get major %d\n", mread_major);
		return result;
	}
	mread_class = class_create(THIS_MODULE, "mread");
	if(IS_ERR(mread_class)) {
		return PTR_ERR(mread_class);
	}

	mread_setup_cdev(&mreadDevs, 0, &mread_fops);

	return 0;
}

static int memread_pdev_suspend(struct platform_device *pdev, pm_message_t state)
{
    return 0;
}
static int memread_pdev_resume(struct platform_device *pdev)
{
	int sts;

	if(app_pid) {
	    sts = kill_pid( get_pid(task_pid(find_task_by_vpid(app_pid))), SIGUSR2, 1);
	}
	else {
		printk( KERN_INFO "memread resume pid = 0\n");
	}
    return 0;
}
static int memread_pdev_remove(struct platform_device *pdev)
{
	cdev_del(&mreadDevs);
	cdev_del(&mreadDevs + 1);
	unregister_chrdev_region(MKDEV(mread_major, 0), 2);
	device_destroy(mread_class, MKDEV(mread_major, 0));
	class_destroy(mread_class);
	return 0;
}

static struct platform_driver _memread_pdev = {
    .probe      = memread_pdev_probe,
    .remove     = memread_pdev_remove,
    .suspend    = memread_pdev_suspend,
    .resume     = memread_pdev_resume,
    .driver     = {
        .name  = "mread",
        .owner = THIS_MODULE,
    },
};

/*---------------------------------------------------------------------------
    mread_init
---------------------------------------------------------------------------*/
static int __init mread_init(void)
{
    platform_driver_register( &_memread_pdev );
    return 0;
}

/*---------------------------------------------------------------------------
    mread_exit
---------------------------------------------------------------------------*/
static void __exit mread_exit(void)
{

    platform_driver_unregister( &_memread_pdev );
}

module_init(mread_init);
module_exit(mread_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("memory read driver");
MODULE_ALIAS("platform:memread_driver");
