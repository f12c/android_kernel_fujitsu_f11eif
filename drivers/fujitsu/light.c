/* 
  Light Sensor Driver

  Protocol driver for Light sensors.
  Copyiiright (C) 2010 TOSHIBA CORPORATION Mobile Communication Company.

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
 * Copyright (c) 2010 Yamaha Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/


#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/platform_device.h>


#include <linux/moduleparam.h>

#include <linux/slab.h>     /* kmalloc() */
#include <linux/fs.h>       /* everything... */
#include <linux/errno.h>    /* error codes */
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <asm/page.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <linux/i2c.h>
#include <mach/vreg.h>
#include "light.h"

#include "../arch/arm/mach-msm/proc_comm.h"

/* for I2C remote mutex */
#include "../arch/arm/mach-msm/smd_private.h"

/* FUJITSU:2011-11-07 EARLYSUSPEND start */
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#endif
/* FUJITSU:2011-11-07 EARLYSUSPEND end */
//Flags for parallel userspace and driver handling

/*===========================================================================
  MACROS
  ===========================================================================*/
#define ALS_DURTY_HACK 1
#define ALS_FIX_TEST 1

/* for debugging */
#define DEBUG 1

#if DEBUG
#define LIGHT_DBG(x...)  printk(x)
#else
#define LIGHT_DBG(x...) 
#endif

/* for device interface initialize */
#define SENSOR_NAME 			"light"
/* FUJITSU:2011-05-30 ALS polling interval time change start */
#define SENSOR_DEFAULT_DELAY    (500)   /* 500ms:sensor polling interval */
/* FUJITSU:2011-05-30 ALS polling interval time change end */
#define SENSOR_LED_OFF_DELAY	(150)	/*  150ms:LED  */
#define SENSOR_MAX_DELAY		(2000)	/* 2000ms:sensor polling interval max */
#define ABS_STATUS				(ABS_BRAKE)
#define ABS_WAKE				(ABS_MISC)
#define ABS_CONTROL_REPORT		(ABS_THROTTLE)

#define LIGHT_DEFAULT_VAL		600


extern struct i2c_adapter *i2c_proximity;	// i2c_adp desc (proximity driver)
extern int i2c_proximity_ready;				// i2c_adp access ready


/*===========================================================================

  GLOBAL VARIABLES

  ===========================================================================*/

static struct input_dev *input_data = NULL;
static int light_suspended = 0;
/* FUJITSU:2011-11-07 EARLYSUSPEND start */
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend  e_sus_fcn;
void light_early_suspend(struct early_suspend *h);
void light_late_resume(struct early_suspend *h);
#endif
/* FUJITSU:2011-11-07 EARLYSUSPEND end */

static struct timer_list tmdrv_timer;
static struct work_struct g_proxi_work_data;
static atomic_t g_led_on_off;
static atomic_t g_timer_is_pending;
//static int g_ls_op_cnt = 0; //chaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaange
static int g_ls_op_cnt = 0;
static int g_ls_probe_init = 0;
static int g_inside_sw = 0;

static int task_flg = 0;

/* FUJITSU:2011-05-30 ALS lux global variables start */
int g_light_lux = 0;
/* FUJITSU:2011-05-30 ALS lux global variables end */

struct sensor_data {
    struct input_dev *input_device;
    struct mutex mutex;
    int enabled;
    int delay;
#if DEBUG
    int suspend;
#endif
};


static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;
static struct sensor_data *data = NULL;

extern int prodrv_sns_als_data_read(int *lux);


/*--------------------------------------------------------------------
func  :	sensor input device opening manager
In/Out:	-
return: 	0: normal
 --------------------------------------------------------------------*/
static int
light_open_sensor(void)
{
   LIGHT_DBG("[LIGHT] OPEN called  [%s] start, line [%d]===\n",
   												 __FUNCTION__,__LINE__);
   mutex_lock(&data->mutex);
   if (0 >= g_ls_op_cnt)
   {
   LIGHT_DBG("[LIGHT] OPEN called  [%s] start, line [%d]===, op_cnt[%d],inside_sw[%d]\n",
   												 __FUNCTION__,__LINE__,g_ls_op_cnt,g_inside_sw);
	   if (0 == g_inside_sw)
	   {
			// sensor access interval timer handler start.
	   		schedule_work(&g_proxi_work_data);
	   }
   }
   g_ls_op_cnt ++;
   LIGHT_DBG("[LIGHT] OPEN called  [%s] start, line [%d]===, op_cnt[%d],inside_sw[%d]\n",
   												 __FUNCTION__,__LINE__,g_ls_op_cnt,g_inside_sw);
   mutex_unlock(&data->mutex);
   return 0;
}

/*--------------------------------------------------------------------
func  :	sensor input device closing manager
In/Out:	*dev :device pointer
return: 	0: normal
       	   -1: error (underrun)
 --------------------------------------------------------------------*/
static int
light_close_sensor(void)
{

    LIGHT_DBG("[LIGHT] OPEN called  [%s] start, line [%d]===\n",
    											 __FUNCTION__,__LINE__);
	mutex_lock(&data->mutex);
	if (0 == g_ls_op_cnt) 
	{
		mutex_unlock(&data->mutex);
		return -1;
	}
	// device count declement
	g_ls_op_cnt --;
	mutex_unlock(&data->mutex);
	return 0;
}


/*    Sysfs Interface  FUNCTIONS                                             */

/* ----------------------------------------------------------------- */
/*
 * sysfs: delay interface (illumi sensor polling time(output))
 */
static ssize_t
sensor_delay_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int delay;

	LIGHT_DBG("[LIGHT] sensor_delay_show enter\n");
	if(NULL == data) {
		LIGHT_DBG("[LIGHT] sensor_delay_show null error\n");
		return 0;
	}

    mutex_lock(&data->mutex);

    delay = data->delay;

    mutex_unlock(&data->mutex);

    return sprintf(buf, "%d\n", delay);
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: delay interface (illumi sensor polling time(input))
 */
static ssize_t
sensor_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int value = simple_strtoul(buf, NULL, 10);

	LIGHT_DBG("[LIGHT] sensor_delay_store enter\n");
	if(NULL == data) {
		LIGHT_DBG("[LIGHT] sensor_delay_store null error\n");
		return 0;
	}

    if (value < 0) {
        return count;
    }

    if (SENSOR_MAX_DELAY < value) {
        value = SENSOR_MAX_DELAY;
    }

    mutex_lock(&data->mutex);

    data->delay = value;

    input_report_abs(input_data, ABS_CONTROL_REPORT,
    				 (data->enabled<<16) | value);
#if ALS_FIX_TEST
    input_sync(input_data);
#endif
    mutex_unlock(&data->mutex);

    return count;
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: enable interface (illumi sensor acces flag(output))
 */
static ssize_t
sensor_enable_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int enabled;

	LIGHT_DBG("[LIGHT] sensor_enable_show enter\n");
	if(NULL == data) {
		LIGHT_DBG("[LIGHT] sensor_enable_show null error\n");
		return 0;
	}

    mutex_lock(&data->mutex);

    enabled = data->enabled;

    mutex_unlock(&data->mutex);

    return sprintf(buf, "%d\n", enabled);
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: enable interface (illumi sensor acces flag(input))
 */
static ssize_t
sensor_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int value = simple_strtoul(buf, NULL, 10);

	LIGHT_DBG("[LIGHT] sensor_enable_store enter\n");
	if(NULL == data) {
		LIGHT_DBG("[LIGHT] sensor_enable_store null error\n");
		return 0;
	}

    if (value != 0 && value != 1) {
        return count;
    }

    mutex_lock(&data->mutex);

    if (data->enabled && !value) {
	light_suspended = 1;
    }
    if (!data->enabled && value) {
	light_suspended = 0;
    }
    data->enabled = value;

    input_report_abs(input_data, ABS_CONTROL_REPORT,
    				 (value<<16) | data->delay);
#if ALS_FIX_TEST
    input_sync(input_data);
#endif
    mutex_unlock(&data->mutex);

    return count;
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: wake interface (illumi sensor wake(input))
 */
 static ssize_t
sensor_wake_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    static int cnt = 1;

	LIGHT_DBG("[LIGHT] sensor_wake_store enter\n");
	if(NULL == input_data) {
		LIGHT_DBG("[LIGHT] sensor_wake_store null error\n");
		return 0;
	}

    input_report_abs(input_data, ABS_WAKE, cnt++);
#if ALS_FIX_TEST
    input_sync(input_data);
#endif
    return count;
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: data interface (illumi sensor lux value get(output))
 */
 static ssize_t
sensor_data_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    unsigned long flags;
    int x;

	LIGHT_DBG("[LIGHT] sensor_data_show enter\n");
	if(NULL == input_data) {
		LIGHT_DBG("[LIGHT] sensor_data_show null error\n");
		return 0;
	}

#if ALS_DURTY_HACK
/* ---------------------------------------------------------------------------- */
 /*
 * durty hack for als working
 *
 */
    //schedule_work(&g_proxi_work_data);
    if(task_flg >= 0 && task_flg < 2) {
        task_flg++;
    }
    if(task_flg == 1) {
            light_open_sensor();
    }
/* ------------------------------ end ------------------------------- */
#endif

    spin_lock_irqsave(&input_data->event_lock, flags);

/* FUJITSU:2011-12-01 ICS start */
#if 1
    x = input_data->abs[ABS_X];
#else
    x = input_data->absinfo[ABS_X].value;
#endif
/* FUJITSU:2011-12-01 ICS end */

    spin_unlock_irqrestore(&input_data->event_lock, flags);

    return sprintf(buf, "%d\n", x);
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: status interface (illumi sensor status(output))
 */
static ssize_t
sensor_status_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    unsigned long flags;
    int status;

	LIGHT_DBG("[LIGHT] sensor_status_show enter\n");
	if(NULL == input_data) {
		LIGHT_DBG("[LIGHT] sensor_status_show null error\n");
		return 0;
	}

    spin_lock_irqsave(&input_data->event_lock, flags);

/* FUJITSU:2011-12-01 ICS start */
#if 1
    status = input_data->abs[ABS_STATUS];
#else
    status = input_data->absinfo[ABS_STATUS].value;
#endif
/* FUJITSU:2011-12-01 ICS end */

    spin_unlock_irqrestore(&input_data->event_lock, flags);

    return sprintf(buf, "%d\n", status);
}


static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
        sensor_delay_show, sensor_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
        sensor_enable_show, sensor_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
        NULL, sensor_wake_store);
static DEVICE_ATTR(data, S_IRUGO, sensor_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, sensor_status_show, NULL);

static struct attribute *sensor_attributes[] = {
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_wake.attr,
    &dev_attr_data.attr,
    &dev_attr_status.attr,
    NULL
};

static struct attribute_group sensor_attribute_group = {
    .attrs = sensor_attributes
};


/* ----------------------------------------------------------------- */
/*
 * suspend interface
 */
static int
sensor_suspend(struct platform_device *pdev, pm_message_t state)
{

	// illumi light suspend flag ON
	//light_suspended = 1;
	LIGHT_DBG("[LIGHT] sensor_suspend\n");

    return 0;
}


/* ----------------------------------------------------------------- */
/*
 * resume interface
 */
static int
sensor_resume(struct platform_device *pdev)
{

	// illumi light suspend flag OFF
	//light_suspended = 0;
	LIGHT_DBG("[LIGHT] sensor_resume \n");

    return 0;
}




/* ----------------------------------------------------------------- */
/*
 *  LED control mode setting interface
 *
 *    !0 set =>  LED ON and illumi sensor off (value hold)
 *    0  set =>  LED OFF and illumi sensor on
 */
void led_notify_light(int on_off)
{
	static int prev_status = -1;

	atomic_set(&g_led_on_off, on_off);

	if(prev_status == atomic_read(&g_led_on_off)){
		return;
	}
    else {
		if(!atomic_read(&g_led_on_off)) {
			mod_timer(&tmdrv_timer, jiffies + msecs_to_jiffies(SENSOR_LED_OFF_DELAY));
			atomic_set(&g_timer_is_pending,1);
		}
	}
	prev_status = atomic_read(&g_led_on_off);

}

EXPORT_SYMBOL(led_notify_light);

/* ----------------------------------------------------------------- */
/*
 *  illumi sensor intervals timer bottom half handler
 *
 */
static void 
light_work_bh(struct work_struct *work)
{

	int lux_val=0;
	int res;

	mutex_lock(&data->mutex);
	g_inside_sw = 1;
	mutex_unlock(&data->mutex);

	LIGHT_DBG("[LIGHT] light_work_bh suspend flag:%d\n",light_suspended);
	LIGHT_DBG("[LIGHT] light_work_bh i2c ready   :%d\n",i2c_proximity_ready);
	LIGHT_DBG("[LIGHT] light_work_bh data-delay  :%d\n",data->delay);
	LIGHT_DBG("[LIGHT] light_work_bh data-enabled:%d\n",data->enabled);

	if (0 == light_suspended && 1 == i2c_proximity_ready) {
		if(!atomic_read(&g_led_on_off)) //LED is OFF
		{
			// LED OFF => HW value
			// proximity driver's illumi values get
			res = prodrv_sns_als_data_read(&lux_val);
			if( res < 0 ) {
				// error
				LIGHT_DBG("[LIGHT] light_work_bh lux-get error\n");
				//  save value

/* FUJITSU:2011-12-01 ICS start */
#if 1
			    lux_val = input_data->abs[ABS_X];
#else
			    lux_val = input_data->absinfo[ABS_X].value;
#endif
/* FUJITSU:2011-12-01 ICS end */

			}
		}
		else {
			// LED ON => save value
/* FUJITSU:2011-12-01 ICS start */
#if 1
		    lux_val = input_data->abs[ABS_X];
#else
		    lux_val = input_data->absinfo[ABS_X].value;
#endif
/* FUJITSU:2011-12-01 ICS end */
		}

/* FUJITSU:2011-05-30 ALS lux global variables start */
		g_light_lux = lux_val;
/* FUJITSU:2011-05-30 ALS lux global variables end */

		// report buffer write
#if ALS_FIX_TEST
		input_report_abs(input_data, ABS_X, lux_val + 1);
#else
		input_report_abs(input_data, ABS_X, lux_val);
#endif
		input_sync(input_data);
	}

	mutex_lock(&data->mutex);
	if (g_ls_op_cnt > 0)
//if (1)
	{
		if(atomic_read(&g_timer_is_pending))
		{
			mod_timer(&tmdrv_timer,jiffies + msecs_to_jiffies(SENSOR_LED_OFF_DELAY));
			LIGHT_DBG("[LIGHT] light_work_bh LED timer set:%d sec\n",SENSOR_LED_OFF_DELAY);
		}
		else
		{
/* FUJITSU:2011-05-10 ALS/PS CORRECT start */
			mod_timer(&tmdrv_timer,jiffies + msecs_to_jiffies(data->delay));
			LIGHT_DBG("[LIGHT] light_work_bh wait timer set:%d sec\n",data->delay);
/* FUJITSU:2011-05-10 ALS/PS CORRECT end */
		}
	}
	g_inside_sw = 0;
	mutex_unlock(&data->mutex);
}

/* ----------------------------------------------------------------- */
/*
 *  illumi sensor intervals timer handler
 *
 */
static void 
light_timer_func(unsigned long ptr)
{
	LIGHT_DBG("[LIGHT] schedule work is called===\n");
	atomic_set(&g_timer_is_pending,0);
	// bottom half function dispatch (light_work_bh())
	schedule_work(&g_proxi_work_data);
}

/* FUJITSU:2011-11-07 EARLYSUSPEND start */
#ifdef CONFIG_HAS_EARLYSUSPEND
void light_early_suspend(struct early_suspend *h)
{
	light_suspended = 1;
	LIGHT_DBG("[LIGHT] light_early_suspend!!!!\n");
}
void light_late_resume(struct early_suspend *h)
{
	light_suspended = 0;
	LIGHT_DBG("[LIGHT] light_late_resume \n");
}

#endif
/* FUJITSU:2011-11-07 EARLYSUSPEND end */

/* ----------------------------------------------------------------- */
/*
 *  sensor input device open
 *
 */
static int 
input_open (struct input_dev *dev)
{
	if (0 == g_ls_probe_init)
	{
		printk(KERN_ERR
			   "[LIGHT] input_open: Not been able to start due to initialization\n");
		return -1;
	}

	LIGHT_DBG("\n[LIGHT] in OPEN called  [%s] start, line [%d]===\n",
													 __FUNCTION__,__LINE__);
	light_open_sensor();
	return 0;
}

/* ----------------------------------------------------------------- */
/*
 *  sensor input device close
 *
 */
static void 
input_close (struct input_dev *dev)
{
	if (0 == g_ls_probe_init)
	{
		printk(KERN_ERR
			   "[LIGHT] input_close: Not been initialized,close notn");
		return;
	}

	LIGHT_DBG("\n[LIGHT] in CLOSE called  [%s] start, line [%d]===\n",
													 __FUNCTION__,__LINE__);
	light_close_sensor();
       task_flg = 0;
	return;
}


/* ----------------------------------------------------------------- */
/*
 *  sensor probe
 *
 */
static int
sensor_probe(struct platform_device *pdev)
{
	int input_registered = 0;
	int sysfs_created = 0;
	int rt;

	LIGHT_DBG("\n[LIGHT] in PROBE called  [%s] start, line [%d]===\n",
													 __FUNCTION__,__LINE__);

	// device memory allocate
	data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	if (!data) {
		rt = -ENOMEM;
		goto err;
	}
	
	// device memory initialize
	data->enabled = 1;
	data->delay = SENSOR_DEFAULT_DELAY;

	// ligth device alloate
	input_data = input_allocate_device();
	if (!input_data) {
		rt = -ENOMEM;
		printk(KERN_ERR
			   "[LIGHT] sensor_probe: Failed to allocate input_data device\n");
		goto err;
	}
	data->input_device = input_data;

	// data store device member
	input_set_drvdata(input_data,data);

	// HAL interface event bit setting
	set_bit(EV_ABS, input_data->evbit);
	set_bit(ABS_X,     input_data->absbit);
/* FUJITSU:2012-03-06 ACE add start */
	input_set_capability(input_data, EV_ABS, ABS_WAKE);
	input_set_capability(input_data, EV_ABS, ABS_CONTROL_REPORT);
/* FUJITSU:2012-03-06 ACE add end */
	input_set_abs_params(input_data, ABS_X, -1872, 1872, 0, 0);

	input_data->name = SENSOR_NAME;
	input_data->open = input_open;
	input_data->close = input_close;

	// device register
	rt = input_register_device(input_data);
	if (rt) {
		printk(KERN_ERR
			   "[LIGHT] sensor_probe: Unable to register input_data device: %s\n",
			   input_data->name);
		goto err;
	}
	input_registered = 1;

	// sysfs device setting
	rt = sysfs_create_group(&input_data->dev.kobj,
			&sensor_attribute_group);
	if (rt) {
		printk(KERN_ERR
			   "[LIGHT] sensor_probe: sysfs_create_group failed[%s]\n",
			   input_data->name);
		goto err;
	}
	sysfs_created = 1;

	mutex_init(&data->mutex);
	this_data = input_data;

	//initialize work queue(timer functions)
#if 0
	INIT_DELAYED_WORK(&g_proxi_work_data, light_work_bh);
#else
	INIT_WORK(&g_proxi_work_data, light_work_bh);
#endif
	setup_timer(&tmdrv_timer, light_timer_func, 0);
	g_ls_probe_init = 1;

/* FUJITSU:2011-11-07 EARLYSUSPEND start */
#ifdef CONFIG_HAS_EARLYSUSPEND
	e_sus_fcn.suspend = light_early_suspend;
	e_sus_fcn.resume = light_late_resume;
	e_sus_fcn.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&e_sus_fcn);
#endif
/* FUJITSU:2011-11-07 EARLYSUSPEND end */

	return 0;

	err:
	if (data != NULL) {
		printk(KERN_CRIT "[LIGHT] sensor_probe: failed to initilie the light driver\n");
		if (input_data != NULL) {

			if (sysfs_created) {
				sysfs_remove_group(&input_data->dev.kobj,
						&sensor_attribute_group);
			}

			if (input_registered) {
				input_unregister_device(input_data);
			}
			else {
				input_free_device(input_data);
			}
			input_data = NULL;
		}
		kfree(data);
	}

	return rt;
}

/* ----------------------------------------------------------------- */
/*
 *  sensor remove
 *
 */
static int
sensor_remove(struct platform_device *pdev)
{
    struct sensor_data *data;

	g_ls_probe_init = 0;
    if (this_data != NULL) {
        data = input_get_drvdata(this_data);

        sysfs_remove_group(&this_data->dev.kobj,
                &sensor_attribute_group);

		// device I/O unregister
        input_unregister_device(this_data);
        if (data != NULL) {
            kfree(data);
        }
    }

    return 0;
}


/*--------------------------------------------------------------------
  device module attribute (device desc.) setting
  ------------------------------------------------------------------*/
static struct platform_driver sensor_driver = {
    .probe      = sensor_probe,
    .remove     = sensor_remove,
    .suspend    = sensor_suspend,
    .resume     = sensor_resume,
    .driver = {
        .name   = SENSOR_NAME,
        .owner  = THIS_MODULE,
    },
};


/* ----------------------------------------------------------------- */
/*
 * sensor initializeation
 *
 */
static int __init sensor_init(void)
{
	atomic_set(&g_led_on_off,0);
	atomic_set(&g_timer_is_pending,0);

	// device register
	sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
	if (IS_ERR(sensor_pdev)) {
		return -1;
	} 
	platform_driver_register(&sensor_driver);

	return 0;
}
module_init(sensor_init);

/* ----------------------------------------------------------------- */
/*
 * sensor shutdown
 *
 */
static void __exit sensor_exit(void)
{
    platform_driver_unregister(&sensor_driver);
    platform_device_unregister(sensor_pdev);
}
module_exit(sensor_exit);

MODULE_AUTHOR("Yamaha Corporation");
MODULE_LICENSE( "GPL" );
MODULE_VERSION("1.2.0");
