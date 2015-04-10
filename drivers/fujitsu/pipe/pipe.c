/*
  pipe Driver

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
/*
 * Copyright(C) 2011 FUJITSU LIMITED
*/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <mach/gpio.h>

#include "smd_private.h"

MODULE_LICENSE("GPL");

#define DRIVER_NAME "pipe"

/*----------------------------------------------------------
 *	pipe_ioctl para
 *---------------------------------------------------------*/
/* command pipe_ioctl */
#define PIPE_IOCTL_MAGIC 'p'

#define PIPE_IOCTL_00   _IOC( _IOC_NONE,PIPE_IOCTL_MAGIC, 0, 0 )
#define PIPE_IOCTL_01   _IOC( _IOC_NONE,PIPE_IOCTL_MAGIC, 1, 0 )
#define PIPE_IOCTL_02   _IOC( _IOC_NONE,PIPE_IOCTL_MAGIC, 2, 0 )
/* struct pipe_ioctl */
typedef struct {
	unsigned int pipe_user_id;
	unsigned int * ppipe_smem_id;
} pipe_ioctl_cmd0;

typedef struct {
	char * pipe_user_buf;
	unsigned int pipe_user_size;
	unsigned int pipe_smem_id;
	unsigned int pipe_smem_size;
	unsigned int pipe_smem_offset;
} pipe_ioctl_cmd1;

typedef union {
	pipe_ioctl_cmd0 pipe_cmd0_union;
	pipe_ioctl_cmd1 pipe_cmd1_union;
} pipe_ioctl_cmd_union;

#define SMEM_ID_TBL_MAX     (7)
static const unsigned int _smem_id_tbl[SMEM_ID_TBL_MAX]
= { 13,
	14,
	15,
	16,
	17,
	SMEM_OEM_008 , 
	SMEM_OEM_009   
};

/* pipe local para */

unsigned char pipe_local_buf[ (SMEM_V1_OEM_008_SIZE < SMEM_V1_OEM_009_SIZE) ? SMEM_V1_OEM_009_SIZE : SMEM_V1_OEM_008_SIZE ];
static unsigned int pipe_dev_count = 1;
static int pipe_major = 0;
static int pipe_minor = 0;
module_param(pipe_major, int, 0);
static struct class *pipe_class = NULL;
static struct cdev pipe_cdev;
//static long pipe_size_check(pipe_ioctl_cmd1);

/*----------------------------------------------------------
 *	pipe_open
 *---------------------------------------------------------*/
static int pipe_open (struct inode *inode, struct file *filp)
{
	return 0;
}


/*----------------------------------------------------------
 *	pipe_write
 *---------------------------------------------------------*/
/*
static ssize_t pipe_write(struct file * file, char * buff, size_t count, loff_t *pos)
{
	return (-EIO);
}
*/
/*----------------------------------------------------------
 *	pipe_read
 *---------------------------------------------------------*/
/*
static ssize_t pipe_read(struct file * file, char * buff, size_t count, loff_t *pos)
{
	return (-EIO);
}
*/
/*----------------------------------------------------------
 *	pipe_release
 *---------------------------------------------------------*/
static int pipe_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/*----------------------------------------------------------
 *	pipe_size_check
 *---------------------------------------------------------*/
static long pipe_size_check(pipe_ioctl_cmd1 pa)
{
	long ret = 0;
	
	if( (sizeof(pipe_local_buf) < pa.pipe_user_size) ||
		(sizeof(pipe_local_buf) < pa.pipe_smem_size) ||
		(pa.pipe_smem_size < (pa.pipe_smem_offset + pa.pipe_user_size)) ) {
			printk(KERN_ERR "pipe_ioctl: failed pipe_size %x smem_size %x param.smem_offset + pipe_user_size %x\n"
								,pa.pipe_user_size, pa.pipe_smem_size,(pa.pipe_smem_offset + pa.pipe_user_size));
		ret = (-EINVAL);
	}
	return ret;
}
/*----------------------------------------------------------
 *	pipe_ioctl
 *---------------------------------------------------------*/
static long pipe_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

	pipe_ioctl_cmd_union __user *argp = (pipe_ioctl_cmd_union __user *)arg;
	pipe_ioctl_cmd_union param;
	unsigned int pipe_smem_id;
	unsigned char *smem_ptr = NULL;
	long ret;
	int i;

	if (copy_from_user(&param, argp,sizeof(pipe_ioctl_cmd_union))) {
		printk(KERN_ERR "pipe_ioctl:%s: copy_from_user failed 1\n", __func__ );
		return (-EFAULT);
	}
	switch(cmd) {
		case PIPE_IOCTL_00:
			if (param.pipe_cmd0_union.pipe_user_id < SMEM_ID_TBL_MAX){
				pipe_smem_id = _smem_id_tbl[param.pipe_cmd0_union.pipe_user_id];

			}else{
				pipe_smem_id = 0xFFFF;
				printk(KERN_ERR "PIPE_IOCTL_CMD_00: user id invalid. uid=%d\n", param.pipe_cmd0_union.pipe_user_id);
				return (-EFAULT);
			}
			if (copy_to_user(param.pipe_cmd0_union.ppipe_smem_id, &pipe_smem_id, sizeof(pipe_smem_id))) {
				printk(KERN_ERR "PIPE_IOCTL_CMD_00: copy_to_user failed. uid=%d\n",param.pipe_cmd0_union.pipe_user_id);
				return (-EFAULT);
			}
			break;
		case PIPE_IOCTL_01:
			ret = pipe_size_check(param.pipe_cmd1_union);
			if(ret)
				return ret;
			if(copy_from_user(pipe_local_buf, param.pipe_cmd1_union.pipe_user_buf, param.pipe_cmd1_union.pipe_user_size)) {
				printk(KERN_ERR "PIPE_IOCTL_CMD_01: copy_from_user failed.\n");
				return (-EFAULT);
			}
			/* SMEM access*/
			smem_ptr = (unsigned char *)smem_alloc_vendor1(param.pipe_cmd1_union.pipe_smem_id);
			if(smem_ptr == NULL){
				printk(KERN_ERR "PIPE_IOCTL_CMD_01: failed to get smem_id=%x\n", param.pipe_cmd1_union.pipe_smem_id);
				return (-EIO);
			}

			/* data stor */
			smem_ptr += param.pipe_cmd1_union.pipe_smem_offset;
			for(i = 0; i < param.pipe_cmd1_union.pipe_user_size; i++){
				*smem_ptr++ = pipe_local_buf[i];
			}
			break;
		case PIPE_IOCTL_02:
			ret = pipe_size_check(param.pipe_cmd1_union);
			if(ret)
				return ret;
			/* SMEM access*/
			smem_ptr = (unsigned char *)smem_alloc_vendor1(param.pipe_cmd1_union.pipe_smem_id);
			if(smem_ptr == NULL){
				printk(KERN_ERR "PIPE_IOCTL_CMD_02: failed to get pipe_smem_id=%x\n", param.pipe_cmd1_union.pipe_smem_id);
				return (-EIO);
			}

			/* data stor */
			smem_ptr += param.pipe_cmd1_union.pipe_smem_offset;
			if(copy_to_user(param.pipe_cmd1_union.pipe_user_buf, smem_ptr, param.pipe_cmd1_union.pipe_user_size)) {
				printk(KERN_ERR "PIPE_IOCTL_CMD_02: copy_to_user failed.\n");
				return (-EFAULT);
			}
			break;
		default:
			printk(KERN_ERR "pipe_ioctl:illegal command. %d\n",cmd);
			return (-EINVAL);
	}
	return 0;
}

/*
 * Our various sub-devices.
 */
static struct file_operations pipe_ops = {
	.owner   = THIS_MODULE,
	.open    = pipe_open,
	.release = pipe_release,
	.unlocked_ioctl = pipe_ioctl,
};

/*----------------------------------------------------------
 *	pipe_init
 *---------------------------------------------------------*/
static int pipe_init(void)
{
	dev_t dev = MKDEV(pipe_major, 0);
	int alloc_ret = 0;
	int cdev_err = 0;
    struct device *class_dev = NULL;

	/*
	* register major number
	*/
	/* reserve major number */
	if (pipe_major) {
		alloc_ret = register_chrdev_region(dev, pipe_dev_count, DRIVER_NAME);
		if (alloc_ret < 0) {
			printk(KERN_ERR "pipe: unable to get major %d\n", pipe_major);
			goto error;
		}
		if (pipe_major == 0)
			pipe_major = alloc_ret;
	}
	else {
		alloc_ret = alloc_chrdev_region(&dev, pipe_minor, pipe_dev_count, DRIVER_NAME);
		if (alloc_ret) {
			printk(KERN_ERR "pipe: unable to get major \n");
			goto error;
		}
		pipe_major = MAJOR(dev);
	}

	/* register system call handler(fops) */
	cdev_init(&pipe_cdev, &pipe_ops);

	/* register to kernel */
	pipe_cdev.owner = THIS_MODULE;
	pipe_cdev.ops = &pipe_ops;
	cdev_err = cdev_add (&pipe_cdev, MKDEV(pipe_major, pipe_minor), pipe_dev_count);

	if (cdev_err) {
		goto error;
	}

	/* register class */
	pipe_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(pipe_class)) {
		goto error;
	}
	class_dev = device_create(pipe_class, NULL, MKDEV(pipe_major, pipe_minor), NULL, DRIVER_NAME);

	if (IS_ERR(class_dev))
		printk(KERN_ERR "pipe: can't create device\n");

	return 0;
  
error:
	if (cdev_err == 0)
		cdev_del(&pipe_cdev);
	
	if (alloc_ret == 0)
		unregister_chrdev_region(MKDEV(pipe_major, 0), pipe_dev_count);

	return -1;
}


/*----------------------------------------------------------
 *	pipe_exit
 *---------------------------------------------------------*/
static void pipe_exit(void)
{
	/* unregister class */
	device_destroy(pipe_class, MKDEV(pipe_major, 0));
	class_destroy(pipe_class);

	/* unregister device */
	cdev_del(&pipe_cdev);
	unregister_chrdev_region(MKDEV(pipe_major, 0), pipe_dev_count);
}

module_init(pipe_init);
module_exit(pipe_exit);
