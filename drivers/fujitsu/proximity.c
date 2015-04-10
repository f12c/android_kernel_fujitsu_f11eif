/*
  Proximty Sensor Driver
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
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA */
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
 * MA  02110-1301, USA. */
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
#include <linux/kthread.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <mach/gpio.h>
#include "proximity.h"
#include <asm/uaccess.h>
#include <asm/page.h>
#include <linux/i2c.h>
#include <mach/pmic.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include "prox_gpio_hw.h"
/* FUJITSU:2012-01-16 bsp-sensor add start */
#include "proc_comm.h"
/* FUJITSU:2012-01-16 bsp-sensor add end */

/*===========================================================================
  MACROS
  ===========================================================================*/
#define ALS_FIX_TEST 0
#define testing1 1
#define testing2 1
#define testing3 0

#define EIFTEST 1

/* for debugging */
#define DEBUG 0
#define DEBUG_SUPPORT_TEST 0	// tuning
#define DEBUG_MONITOR 0

#if DEBUG
#define PROX_DBG(x...)  printk(KERN_DEBUG x)
#else
#define PROX_DBG(x...)  
#endif

#if DEBUG_MONITOR
#define PROX_DBG_MONITOR(x...)  printk(x)
#else
#define PROX_DBG_MONITOR(x...)  
#endif

#define PROX_PRINT(x...) printk(x)

/* for device interface initialize */
#define SENSOR_NAME				"proximity"
#define SENSOR_DEFAULT_DELAY	(200)   /* 200 ms */
#define SENSOR_MAX_DELAY		(2000)  /* 2000 ms */
#define ABS_STATUS				(ABS_BRAKE)
#define ABS_WAKE				(ABS_MISC)
#define ABS_CONTROL_REPORT		(ABS_THROTTLE)
/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
#define PROX_GPIO_REV0			22		/* GPIO_REV03_20110322 */
#define PROX_GPIO_REV1			112		
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */
#define WAKE_ON					1

//
/* for I2C */
#define PROX_I2C_ADAPTER		4			/* 1-X QUPB */

#define SNS_STANDBY_TIME		10

#define I2C_RTRY_WAIT			20
#define I2C_RTRY_CNT			10

#define I2C_WRITE_BUFSIZE		64
#define READ_SIZE				2

#define I2C_TRANS_WRITE			1
#define I2C_TARNS_READ			2

#define I2C_READ_POS			1

#define PROX_SLAVE_ADDRESS	  	(0x38)		/* BH1772GLC	*/

#define GPIOF_ENABLE_WAKE		0x40000000

#define	INT_CLEAR				0x80

#define	PROX_REG				0x00
#define	PROX_1					0x01
#define	PROX_VO_DET				0x01

#define	GAIN_REG				0x01
#define	GAIN_REG_INIT			0x00
#define	GAIN_LED				0x08

#define	HYS_REG					0x02
#define HYS_REG_INIT			0x00
#define HYS_HYSF				0x01
#define HYS_HYSF_1				0x02
#define HYS_HYSF_2				0x04
#define HYS_HYSF_3				0x08

#define	HYS_HYSC				0x20
#define	HYS_HYSC_1				0x40
#define	HYS_HYSD				0x80

#define	CYCLE_REG				0x03
#define	CYCLE_REG_INIT			0x00
#define	CYCLE_OSC				0x04
#define	CYCLE_CYCL				0x08
#define	CYCLE_CYCL_1			0x10
#define	CYCLE_CYCL_2			0x20
#define	CYCLE_256MS				0x2C	/*K01_PROX_001*/

#define	OPMOD_REG				0x04
#define	OPMOD_REG_INIT			0x00
#define	OPMOD_SSD				0x01
#define	OPMOD_VCON				0x02
#define	OPMOD_ASD				0x10

#define	CON_REG					0x06
#define	CON_REG_INIT			0x00
#define	CON_OCON				0x08
#define	CON_OCON_1				0x10

#define PROX_REG_NEAR			1
#define PROX_REG_FAR			0

/* for notification */
#define PROX_VAL_NEAR			0
#define PROX_VAL_FAR			1

/* for sensor status */
#define PROX_STATE_WAKE			0
#define PROX_STATE_SUSPEND		1
#define PROX_STATE_WAIT_I2C		2

/* for sensor settings */
#define NV_OEM_TOP_ITEMS_I		10000
#define NV_PROX_SENSE_A_I		(NV_OEM_TOP_ITEMS_I+106)
#define NV_PROX_SENSE_B_I		(NV_OEM_TOP_ITEMS_I+107)

#define PROX_SENS_MAX_NUM		31
/* FUJITSU:2012-01-26 Next-i del start */
///* FUJITSU:2011-12-07 ALS/PS CORRECT start */
//#define PROX_SENS_H				85		// default 104
//#define PROX_SENS_L				84		// default 76
///* FUJITSU:2011-12-07 ALS/PS CORRECT end */
/* FUJITSU:2012-01-26 Next-i del end */
#define PROX_TEST_SENS_NUM		32

/* for sensor delay */
#define PROX_DETECT_NON			0
#define PROX_DETECTING			1
#define PROX_DETECTED			2



/*--------------------------------------------------------------------
I2C register (BH1772GLC)
--------------------------------------------------------------------*/
//#define I2C_CHANNEL					0
//#define I2C_WRITE_BIT					0x00
//#define I2C_READ_BIT					0x01
//#define I2C_PROX_ADDRESS				0x70

#define I2C_PROX_ALS_CONTROL			0x40
#define I2C_PROX_PS_CONTROL				0x41
#define I2C_PROX_I_LED					0x42
//
#define I2C_PROX_ALS_PS_MEAS			0x44
#define I2C_PROX_PS_MEAS_RATE			0x45
#define I2C_PROX_ALS_MEAS_RATE		0x46
//
//
#define I2C_PROX_ALS_DATA_0				0x4C
#define I2C_PROX_ALS_DATA_1				0x4D
#define I2C_PROX_ALS_PS_STATUS		0x4E
#define I2C_PROX_PS_DATA_LED1			0x4F
//
//
#define I2C_PROX_INTERRUPT				0x52
#define I2C_PROX_PS_TH_H_LED1			0x53
//
//
//#define I2C_PROX_ALS_TH_UP_0			0x56
//#define I2C_PROX_ALS_TH_UP_1			0x57
//#define I2C_PROX_ALS_TH_LOW_0			0x58
//#define I2C_PROX_ALS_TH_LOW_1			0x59
#define I2C_PROX_ALS_SENSITIVITY		0x5A
#define I2C_PROX_PERSISTENCE			0x5B
#define I2C_PROX_PS_TH_L_LED1			0x5C
//
//
/*--------------------------------------------------------------------
I2C status (BH1772GLC)
--------------------------------------------------------------------*/
// I2C_PROX_ALS_CONTROL MODE
#define ALS_MODE_STANDBY				0x00
#define ALS_MODE_FORCED					0x02
#define ALS_MODE_STANDALONE				0x03

// I2C_PROX_PS_CONTROL MODE
#define PS_MODE_STANDBY					0x00
#define PS_MODE_FORCED					0x02
#define PS_MODE_STANDALONE				0x03

// I2C_PROX_I_LED LED current
#define LED_CURRENT_5mA					0x18
#define LED_CURRENT_10mA				0x19
#define LED_CURRENT_20mA				0x1A
#define LED_CURRENT_50mA				0x1B
#define LED_CURRENT_100mA				0x1C
#define LED_CURRENT_150mA				0x1D
#define LED_CURRENT_200mA				0x1E

// I2C_PROX_ALS_PS_MEAS
#define PS_TRIGGER						0x01
#define ALS_TRIGGER						0x02

// I2C_PROX_ALS_MEAS_RATE
#define ALS_RATE_ENABLE					0x00
#define ALS_RATE_DISABLE				0x80

#define ALS_MEAS_RATE_100ms				0x00
#define ALS_MEAS_RATE_200ms				0x01
#define ALS_MEAS_RATE_500ms				0x02
#define ALS_MEAS_RATE_1000ms			0x03
#define ALS_MEAS_RATE_2000ms			0x04

// I2C_PROX_PS_MEAS_RATE
#define PS_MEAS_RATE_10ms				0x00
#define PS_MEAS_RATE_20ms				0x01
#define PS_MEAS_RATE_30ms				0x02
#define PS_MEAS_RATE_50ms				0x03
#define PS_MEAS_RATE_70ms				0x04
#define PS_MEAS_RATE_100ms				0x05
#define PS_MEAS_RATE_200ms				0x06
#define PS_MEAS_RATE_500ms				0x07
#define PS_MEAS_RATE_1000ms				0x08
#define PS_MEAS_RATE_2000ms				0x09

// I2C_PROX_INTERRUPT
#define INT_MODE_NONE					0x00
#define INT_MODE_PS						0x01
#define INT_MODE_ALS					0x02
#define INT_MODE_ALL					0x03

#define INT_POL_ACTLOW					0x00
#define INT_POL_INACTLOW				0x04

#define INT_OUTMODE_LATCH				0x00
#define INT_OUTMODE_NOLATCH				0x08

#define	INT_PSHIST_HI					0x00
#define INT_PSHIST_HILOW				0x10

#define	FUJITSU_INTERRUPT_SET			\
	(INT_PSHIST_HILOW | INT_OUTMODE_NOLATCH | INT_POL_ACTLOW | INT_MODE_PS)
#define	FUJITSU_INTERRUPT_RESET			\
	(INT_PSHIST_HILOW | INT_OUTMODE_NOLATCH | INT_POL_ACTLOW | INT_MODE_NONE)
#define	FUJITSU_INT_SET_F2N				\
	(INT_PSHIST_HILOW | INT_OUTMODE_NOLATCH | INT_POL_ACTLOW | INT_MODE_PS)
#define	FUJITSU_INT_SET_N2F				\
	(INT_PSHIST_HILOW | INT_OUTMODE_NOLATCH | INT_POL_INACTLOW | INT_MODE_PS)
#define	FUJITSU_INT_RESET_F2N			\
	(INT_PSHIST_HILOW | INT_OUTMODE_NOLATCH | INT_POL_ACTLOW | INT_MODE_NONE)
#define	FUJITSU_INT_RESET_N2F			\
	(INT_PSHIST_HILOW | INT_OUTMODE_NOLATCH | INT_POL_INACTLOW | INT_MODE_NONE)

/* FUJITSU:2011-11-29 PS CONTROL change start */
#define PROX_ALS_SENS_DEF				0x35
/* FUJITSU:2011-11-29 PS CONTROL change end */

// I2C_PROX_ALS_PS_STATUS
#define	STS_ALSINT_BIT					0x80
#define	STS_ALSDATA_BIT					0x40
#define	STS_PSINT_BIT					0x02
#define	STS_PSDATA_BIT					0x01

#ifndef EIFTEST
/* FUJITSU:2011-09-15 panel color read  start */
//#define PROX_NVID_BASE		10151
/* FUJITSU:2011-09-15 panel color read  end */
#else
#define PROX_NVID_BASE		10031
#define PROX_NVDATA_BASE	0xf8;
#endif

/* FUJITSU:2011-09-27 start */
#define PROX_NV_PS_TH_H_ID_BASE			10153
#define PROX_NV_PS_TH_L_ID_BASE			10154
#define PROX_NV_LIGHT_HOSEI_ID_BASE		10155
/* FUJITSU:2011-09-27 end */


/*===========================================================================

  GLOBAL VARIABLES

  ===========================================================================*/
/*============================================================
test
===========================================================*/
#if 0
static struct wake_lock prox_wake_lock;
#endif
/*============================================================
test
===========================================================*/


struct i2c_adapter *i2c_proximity;		// i2c_adp desc.
int i2c_proximity_ready = 0;	// 1:ready , 0:not ready

static atomic_t g_prox_state;
static atomic_t g_wake;
static atomic_t g_suspend_off;

static unsigned int g_proxi_irq;
static struct work_struct g_proxi_work_data;

static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;

/* FUJITSU:2011-05-10 ALS/PS CORRECT start */
static unsigned char 	g_i_led;			// sensor register value
static unsigned char 	g_als_sens;			// sensor register value
static unsigned char 	g_ps_hi;			// sensor register value
static unsigned char 	g_ps_low;			// sensor register value
static int				g_als_correct;		// ALS sensitive lux x correct
/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
static int				g_gpio_number;		// sensor gpio num
static int				g_gpio_pullup;		// sensor gpio pullup
static int				g_gpio_strength;	// sensor gpio device strength
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

/* FUJITSU:2011-05-25 panel color read  start */
static unsigned g_prox_nvid;
static unsigned g_prox_nvdata;
/* FUJITSU:2011-05-25 panel color read end    */
#ifndef EIFTEST
/* FUJITSU:2011-09-27 start */
static unsigned g_prox_nv_ps_th_h_id;
static unsigned g_prox_nv_ps_th_l_id;
static unsigned g_prox_nv_light_hosei_id;
static unsigned g_prox_nvdata_ps_th_h;
static unsigned g_prox_nvdata_ps_th_l;
static unsigned g_prox_nvdata_light_hosei;
/* FUJITSU:2011-09-27 end */
#endif

/* FUJITSU:2011-11-29 PS CONTROL change start */
static int				prox_lux_data;
/* FUJITSU:2011-11-29 PS CONTROL change end */

struct prox_correct_t {
	unsigned char	index;
	unsigned char	i_led;
	unsigned char	ps_hi;
	unsigned char	ps_low;
	unsigned char	als_sens;
	int				als_correct;
};

/* FUJITSU:2012-01-26 Next-i change start */
struct prox_correct_t	prox_correct[4] = {
#if defined(CONFIG_MACH_F11EIF)
//	{   0, LED_CURRENT_20mA, 87, 66, 0x6a, 5},// B(2x5)
//	{   1, LED_CURRENT_20mA, 87, 66, 0x42, 2},// P(1.25x2)
	{   0, LED_CURRENT_20mA, 60, 39, 0x6a, 5},// B(2x5)
	{   1, LED_CURRENT_20mA, 60, 39, 0x42, 2},// P(1.25x2)
/* FUJITSU:2012-03-06 ACE change start */
	//{0xff, LED_CURRENT_20mA, 87, 66, 0x6a, 5}, // o(2x5)
	{0xff, LED_CURRENT_20mA, 44, 32, 0x6a, 5},// B(2x5)
/* FUJITSU:2012-03-06 ACE change end */
#elif defined(CONFIG_MACH_F12SKY)
	{	0, LED_CURRENT_20mA, 95, 94, 0xc8, 1},// SK-White
	{	1, LED_CURRENT_20mA, 95, 94, 0xe9, 1},// SK-PINK
	{	2, LED_CURRENT_20mA, 95, 94, 0xe9, 1},// SK-COLABO
#elif defined(CONFIG_MACH_F12APON)
	{	3, LED_CURRENT_20mA, 85, 84, 0x6a, 1},// AD		
	{	4, LED_CURRENT_20mA, 85, 84, 0x6a, 1},// AK-BLACK
	{	5, LED_CURRENT_20mA, 85, 84, 0x6a, 1},// AK-RED	
#elif defined(CONFIG_MACH_F09D)
/* FUJITSU:2012-02-08 ACE change start */
	{	0, LED_CURRENT_20mA, 95, 94, 0xc1, 1},// ACE-PINK
	{	1, LED_CURRENT_20mA, 95, 94, 0xcc, 1},// ACE-GOLD
	{0xff, LED_CURRENT_20mA, 95, 94, 0xcc, 1},// default
/* FUJITSU:2012-02-08 ACE change end */
#else
	{	0, LED_CURRENT_20mA, 85, 84, 0x6a, 1},
	{	1, LED_CURRENT_20mA, 85, 84, 0x6a, 1},
	{	2, LED_CURRENT_20mA, 85, 84, 0x6a, 1},
#endif
	{0xff, LED_CURRENT_20mA, 85, 84, 0x6a, 1} // default
};
/* FUJITSU:2012-01-26 Next-i change end */

/*===========================================================================

  STRUCTS

  ===========================================================================*/
struct sensor_data {
	struct mutex mutex;
	int enabled;
	int delay;
};

struct prox_detect_state {
	int           proxy_lux_save;
	int           proxy_distreg_save;
	char          proxy_distance_save[8];
	unsigned char proxy_val;
	unsigned char proxy_state;
	unsigned char proxy_intr_save;
};
static struct prox_detect_state prox_detect;


/*===========================================================================

  LOCAL FUNCTION PROTOTYPES

  ===========================================================================*/
static int prodrv_sns_reg_init(void);
static int  prodrv_sns_i2c_write( unsigned char reg, unsigned char data);
static int prodrv_sns_i2c_read(unsigned short reg, unsigned char *data, uint32_t len);
static void prodrv_sns_ssd(void);
int prodrv_sns_ON(void);
void prodrv_sns_OFF(void);
static int prodrv_sns_request_irqs(void);
int prodrv_sns_als_data_read(int *lux);
void prodrv_sns_report_distance(void);
static int prodrv_sns_convert_data(int data, char *val);
struct i2c_adapter * prodev_i2c_search_adapter(void);
static void prodrv_sns_work_bh(struct work_struct *work);
static void prodrv_sns_work_bh_sub(void);


/* FUJITSU:2011-05-25 panel color read start  */
extern int msm_prodrv_read_nvitem(unsigned *id, unsigned *data);
/* FUJITSU:2011-05-25 panel color read end    */

/* FUJITSU:2011-08-29 KOUTEI TAIOU start  */
extern int factory_mode(void);
/* FUJITSU:2011-05-29 KOUTEI TAIOU end  */

/*===========================================================================

  LOCAL FUNCTIONS

  ===========================================================================*/
/* FUJITSU:2012-01-16 bsp-sensor add start */
/* FUJITSU:2011-12-01 add PROXIMITY setting start */


int msm_prodrv_read_nvitem(unsigned *id, unsigned *data)
{
    int rc;

    if(id == NULL || data == NULL)
        return -1;

    /* read NV  */
   rc = msm_proc_comm(PCOM_OEM_011, id, data);

    return rc;
}
EXPORT_SYMBOL(msm_prodrv_read_nvitem);


/* FUJITSU:2011-12-01 add PROXIMITY setting end */
/* FUJITSU:2012-01-16 bsp-sensor add end */

/*    Sysfs Interface  FUNCTIONS                                             */

/* ----------------------------------------------------------------- */
/*
 * sysfs: delay interface (proximity delay(output))
 */
static ssize_t
prodrv_sysfs_delay_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int delay;
	
	PROX_DBG("[PROXIMITY] prodrv_sysfs_delay_show \n");
	
	if(NULL == data) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_delay_show NULL \n");
		return 0;
	}
	
	mutex_lock(&data->mutex);

	delay = data->delay;

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", delay);
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: delay interface (proximity delay(input))
 */
static ssize_t
prodrv_sysfs_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int value = simple_strtoul(buf, NULL, 10);
	int enabled;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_delay_store \n");

	if(NULL == data) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_delay_store NULL \n");
		return 0;
	}
	
	if (value < 0) {
		return count;
	}

	if (SENSOR_MAX_DELAY < value) {
		value = SENSOR_MAX_DELAY;
	}

	mutex_lock(&data->mutex);

	enabled = data->enabled;
	data->delay = value;

	input_report_abs(input_data, ABS_CONTROL_REPORT, (enabled<<16) | value);
#if ALS_FIX_TEST
    input_sync(input_data);
#endif
	mutex_unlock(&data->mutex);

	return count;
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: enable interface (proximity enable(output))
 */
static ssize_t
prodrv_sysfs_enable_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int enabled;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_show \n");
	
	if(NULL == data) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_show NULL \n");
		return 0;
	}
	
	mutex_lock(&data->mutex);

	enabled = data->enabled;

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", enabled);
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: enable interface (proximity enable(input))
 */
static ssize_t
prodrv_sysfs_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{

    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);

	int value = simple_strtoul(buf, NULL, 10);
	int		init_sts;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_store \n");
	
	if ((value != 0) && (value != 1)) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_store return err \n");
		return count;
	}

	PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_store value:%d \n", value);
	//if(value) {
	if(1) {
		/*if it has been set to 1, call is incoming*/
		atomic_set(&g_wake,true);
		PROX_DBG_MONITOR("[PROXIMITY] Sensor Enable set\n");

		prox_detect.proxy_intr_save = FUJITSU_INTERRUPT_SET;
		init_sts = prodrv_sns_i2c_write(I2C_PROX_INTERRUPT,
				FUJITSU_INTERRUPT_SET);

		init_sts = prodrv_sns_i2c_write(I2C_PROX_PS_CONTROL, 
				PS_MODE_STANDALONE);	

		msleep(15);

		init_sts = prodrv_sns_i2c_write(I2C_PROX_PERSISTENCE, 0x03);
		if(0 > init_sts) {
			PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_store PS presistence 3 ret[%d] \n", init_sts);
		}
	}
	else {

		init_sts = prodrv_sns_i2c_write(I2C_PROX_PERSISTENCE, 0x01);
		if(0 > init_sts) {
			PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_store PS presistence 1 ret[%d] \n", init_sts);
		}

		atomic_set(&g_wake,false);
		PROX_DBG_MONITOR("[PROXIMITY] Sensor Enable reset\n");

		prox_detect.proxy_intr_save = FUJITSU_INTERRUPT_RESET;
		init_sts = prodrv_sns_i2c_write(I2C_PROX_INTERRUPT,
				FUJITSU_INTERRUPT_RESET);

		init_sts = prodrv_sns_i2c_write(I2C_PROX_PS_CONTROL, PS_MODE_STANDBY);	

	}
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_enable_store I2C_PROX_INTERRUPT"
				" FALSE(%02x) \n", prox_detect.proxy_intr_save);
	}

    mutex_lock(&data->mutex);
#if testing3
    data->enabled = 1;
#else
    data->enabled = value;
#endif
#if testing2
    input_report_abs(input_data, ABS_CONTROL_REPORT,
    				 (value<<16) | data->delay);
#if ALS_FIX_TEST
    input_sync(input_data);
#endif
#endif
    mutex_unlock(&data->mutex);
/* FUJITSU:2012-03-06 ACE add end */
	return count;
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: wake interface (wake(input))
 */
static ssize_t
prodrv_sysfs_wake_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	static int cnt = 1;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_wake_store \n");

	input_report_abs(input_data, ABS_WAKE, cnt++);
#if ALS_FIX_TEST
	input_sync(input_data);
#endif
	return count;
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: data interface (distance data(output))
 */
static ssize_t
prodrv_sysfs_data_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int x;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_data_show \n");
	
	spin_lock_irqsave(&input_data->event_lock, flags);

/* FUJITSU:2011-12-01 ICS start */
#if 1
	x = input_data->abs[ABS_X];
#else
	x = input_data->absinfo[ABS_X].value;
#endif
/* FUJITSU:2011-12-01 ICS end */
	spin_unlock_irqrestore(&input_data->event_lock, flags);

	//  report distance
	prodrv_sns_report_distance();

	PROX_DBG("[PROXIMITY] prodrv_sysfs_data_show(distance=%s) \n"
										,prox_detect.proxy_distance_save);

	return sprintf(buf, "%s\n",prox_detect.proxy_distance_save);
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: dfdata interface (proximity flags(output))
 */
static ssize_t
prodrv_sysfs_dfdata_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int x;
#if DEBUG_MONITOR
	int	lux;
#endif

	PROX_DBG("[PROXIMITY] prodrv_sysfs_dfdata_show \n");

/* FUJITSU:2011-10-13 RESUME start  */
	prodrv_sns_work_bh_sub();
/* FUJITSU:2011-10-13 RESUME end  */
	
	spin_lock_irqsave(&input_data->event_lock, flags);

/* FUJITSU:2012-01-26 Next-i del start */
/* FUJITSU:2011-08-29 KOUTEI TAIOU start  */
//	if((system_rev > 0x02) || (0 != factory_mode())) {
/* FUJITSU:2011-08-29 KOUTEI TAIOU end  */
/* FUJITSU:2012-01-26 Next-i del end */
/* FUJITSU:2011-12-01 ICS start */
#if 0
		x = input_data->absinfo[ABS_X].value;
#else
		x = input_data->abs[ABS_X];
		//x = 1;
#endif
/* FUJITSU:2011-12-01 ICS end */
/* FUJITSU:2012-01-26 Next-i del start */
//	}
//	else {
//		x = 1;
//	}
/* FUJITSU:2012-01-26 Next-i del end */
	
	spin_unlock_irqrestore(&input_data->event_lock, flags);

#if DEBUG_MONITOR
	// debug mode (for sensor HW test)
	prodrv_sns_als_data_read(&lux);
	prodrv_sns_report_distance();
	PROX_DBG_MONITOR("[PROXIMITY] Sensor status (Lux=%d distance=%s) \n",
										lux,prox_detect.proxy_distance_save);
#endif	

	PROX_DBG("[PROXIMITY] prodrv_sysfs_dfdata_show:%d \n",x);
	
	return sprintf(buf, "%d\n", x);
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: drdata interface (prox-flags and dist-register(output))
 */
static ssize_t
prodrv_sysfs_drdata_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int x;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_drdata_show \n");

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	prodrv_sns_work_bh_sub();
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	spin_lock_irqsave(&input_data->event_lock, flags);

/* FUJITSU:2011-12-01 ICS start */
#if 1
	x = input_data->abs[ABS_X];
#else
	x = input_data->absinfo[ABS_X].value;
#endif
/* FUJITSU:2011-12-01 ICS end */

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	//  report ps sensor register
	prodrv_sns_report_distance();

	PROX_DBG("[PROXIMITY] prodrv_sysfs_drdata_show:%d \n"
									,prox_detect.proxy_distreg_save);
	
	return sprintf(buf, "%02x%02x\n", x,prox_detect.proxy_distreg_save);
}

/* ----------------------------------------------------------------- */
/*
 * sysfs: status interface (status(output))
 */
static ssize_t
prodrv_sysfs_status_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int status;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_status_show \n");
	
	spin_lock_irqsave(&input_data->event_lock, flags);

/* FUJITSU:2011-12-01 ICS start */
#if 1
	status = input_data->abs[ABS_STATUS];
#else
	status = input_data->absinfo[ABS_STATUS].value;
#endif
/* FUJITSU:2011-12-01 ICS end */

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	PROX_DBG("[PROXIMITY] prodrv_sysfs_status_show:%d \n",status);
	
	return sprintf(buf, "%d\n", status);
}


/* ----------------------------------------------------------------- */
/*
 * sysfs: ktdata interface (koutei test data(output))
 */
static ssize_t
prodrv_sysfs_ktdata_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int x;
	int	lux;

	PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_show \n");

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	prodrv_sns_work_bh_sub();
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	spin_lock_irqsave(&input_data->event_lock, flags);

/* FUJITSU:2011-12-01 ICS start */
#if 1
	x = input_data->abs[ABS_X];
#else
	x = input_data->absinfo[ABS_X].value;
#endif
/* FUJITSU:2011-12-01 ICS end */

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	// debug mode (for sensor HW test)
	prodrv_sns_als_data_read(&lux);
	prodrv_sns_report_distance();
	PROX_DBG("[PROXIMITY] Sensor (Lux=%d dist=%s reg=%d I_LED=%02x ALS=%02x) \n",
		lux,prox_detect.proxy_distance_save,prox_detect.proxy_distreg_save,
		g_i_led,g_als_sens);

	PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_show:%d \n",x);

/* FUJITSU:2011-11-29 PS CONTROL change start */
/* FUJITSU:2011-09-27 start */
	return sprintf(buf, "%d %s %d %d %d %02x %02x %x\n", x,
		prox_detect.proxy_distance_save,prox_detect.proxy_distreg_save,
		lux, prox_lux_data, g_i_led, g_als_sens, g_als_correct);
/* FUJITSU:2011-09-27 end */
/* FUJITSU:2011-11-29 PS CONTROL change end */
}

/* FUJITSU:2011-06-13 [CTS] start */
#if DEBUG_SUPPORT_TEST
/* ----------------------------------------------------------------- */
/*
 * sysfs: ktdata interface (koutei test data(input))
 */
static ssize_t
prodrv_sysfs_ktdata_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int value = simple_strtoul(buf, NULL, 10);
	unsigned char value_led;
	unsigned char value_sens;
	int		init_sts;	/* FUJITSU:2011-08-29 KOUTEI TAIOU add  */
	
	value_led  = (unsigned char)(value & 0x00ff);
	value_sens = (unsigned char)((value & 0xff00) >> 8);

	PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_store \n");
	
	if ((value_led < 0x18) || (value_led > 0x1e)) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_store return err \n");
		return count;
	}

	PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_store val:%d "
			"(led=%02x,sens=%02x)\n",
			 value,value_led,value_sens);

	g_i_led = value_led;
	g_als_sens = value_sens;
		
/* FUJITSU:2011-08-29 KOUTEI TAIOU start  */
	init_sts = prodrv_sns_i2c_write(I2C_PROX_I_LED, g_i_led);	
	PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_store I_LED = %02x\n",g_i_led);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_store I_LED FALSE \n");
		return false;
	}
	init_sts = prodrv_sns_i2c_write(I2C_PROX_ALS_SENSITIVITY,g_als_sens);
	PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_store ALS_SENS = %02x\n",g_als_sens);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sysfs_ktdata_store ALS_CONTROL FALSE \n");
		return false;
	}
/* FUJITSU:2011-08-29 KOUTEI TAIOU end  */

	return count;
}
#endif
/* FUJITSU:2011-06-13 [CTS] end */

/* FUJITSU:2011-09-27 start */
/* ----------------------------------------------------------------- */
/*
 * sysfs: alsdata interface (koutei test data(output))
 */
static ssize_t
prodrv_sysfs_alsdata_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	PROX_DBG("[PROXIMITY] prodrv_sysfs_alsdata_show \n");

	prodrv_sns_report_distance();
	PROX_DBG("[PROXIMITY] Sensor (reg=%d) \n",
		prox_detect.proxy_distreg_save);

	return sprintf(buf, "%x\n", prox_detect.proxy_distreg_save);
}
/* FUJITSU:2011-09-27 end */

/*--------------------------------------------------------------------
  device attribute (sysfs-interface) setting
  ------------------------------------------------------------------*/
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		prodrv_sysfs_delay_show, prodrv_sysfs_delay_store);
static DEVICE_ATTR(enable, S_IWUGO|S_IRUGO|S_IWUSR|S_IWGRP,
		prodrv_sysfs_enable_show, prodrv_sysfs_enable_store);
/* FUJITSU:2012-03-21 ACE change start */
///* FUJITSU:2012-03-06 ACE change start */
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
//static DEVICE_ATTR(wake, S_IWUGO|S_IWUSR|S_IWGRP,
///* FUJITSU:2012-03-06 ACE change end */
/* FUJITSU:2012-03-21 ACE change end */
		NULL, prodrv_sysfs_wake_store);
/* FUJITSU:2011-06-13 [CTS] start */
static DEVICE_ATTR(data,   S_IRUGO, prodrv_sysfs_data_show,   NULL);
static DEVICE_ATTR(status, S_IRUGO, prodrv_sysfs_status_show, NULL);
static DEVICE_ATTR(dfdata, S_IRUGO, prodrv_sysfs_dfdata_show, NULL);
static DEVICE_ATTR(drdata, S_IRUGO, prodrv_sysfs_drdata_show, NULL);

#if DEBUG_SUPPORT_TEST
static DEVICE_ATTR(ktdata, S_IWUGO|S_IRUGO|S_IWUSR|S_IWGRP,
		prodrv_sysfs_ktdata_show, prodrv_sysfs_ktdata_store);
#else
static DEVICE_ATTR(ktdata, S_IRUGO, prodrv_sysfs_ktdata_show, NULL);
#endif
/* FUJITSU:2011-06-13 [CTS] end */

/* FUJITSU:2011-09-27 start */
static DEVICE_ATTR(alsdata, S_IRUGO, prodrv_sysfs_alsdata_show, NULL);
/* FUJITSU:2011-09-27 end */

static struct attribute *sensor_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	&dev_attr_status.attr,
	&dev_attr_dfdata.attr,
	&dev_attr_drdata.attr,
	&dev_attr_ktdata.attr,
	&dev_attr_alsdata.attr,		/* FUJITSU:2011-09-27 add */
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
prodrv_sns_suspend(struct platform_device *pdev, pm_message_t state)
{

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	if(atomic_read(&g_wake)) {

/* FUJITSU:2011-06-13 forced SUSPEND mode start */
		atomic_set(&g_suspend_off,true);
		atomic_set(&g_prox_state,PROX_STATE_SUSPEND);
		/**/
		if( prox_detect.proxy_val == PROX_VAL_NEAR ) {
			PROX_DBG("[PROXIMITY] prodrv_sns_suspend WAKE_ON / INTR-mode\n");
			PROX_PRINT(KERN_DEBUG "proximity: suspend(I)\n");
		}
		else {
			PROX_DBG("[PROXIMITY] prodrv_sns_suspend WAKE_ON "
					"FAR(forced INTR)\n");
			PROX_PRINT(KERN_DEBUG "proximity: suspend(I1)\n");
		}
		return 0;
/* FUJITSU:2011-06-13 forced SUSPEND mode end */
	} 
	else {
		PROX_DBG("[PROXIMITY] prodrv_sns_suspend WAKE_OFF (sleep)\n");
		PROX_PRINT(KERN_DEBUG "proximity: suspend(S2)\n");
	}
	// normal suspend mode (not interrupt)
	prodrv_sns_OFF();

/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	atomic_set(&g_prox_state,PROX_STATE_SUSPEND);

	return 0;
}


/* ----------------------------------------------------------------- */
/*
 * resume interface
 */
static int
prodrv_sns_resume(struct platform_device *pdev)
{
	int sts;

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	/* implement resume of the sensor */
	if(atomic_read(&g_suspend_off)) {
		// intr-mode

		atomic_set(&g_suspend_off,false);

/* FUJITSU:2011-06-15 no-irq suspend status change start */
		if(atomic_read(&g_prox_state) == PROX_STATE_SUSPEND) {
			PROX_PRINT(KERN_DEBUG "proximity: resume(Iw)\n");
			atomic_set(&g_prox_state,PROX_STATE_WAKE);
		} else {
			PROX_PRINT(KERN_DEBUG "proximity: resume(I)\n");
		}
/* FUJITSU:2011-06-15 no-irq suspend status change end */
	}
	else {
		// normal-suspend mode
		PROX_PRINT(KERN_DEBUG "proximity: resume(S)\n");

		sts = prodrv_sns_ON();
		atomic_set(&g_prox_state,PROX_STATE_WAKE);
		if(sts != true) {
			PROX_PRINT(KERN_ERR "proximity: resume prodrv_sns_ON error\n");
			return -EBUSY;
		}
	}
	i2c_proximity_ready = 1;		// I2C proximity ALS access ready
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	return 0;
}

/* ----------------------------------------------------------------- */
/*
 * probe interface
 */
static int
prodrv_sns_probe(struct platform_device *pdev)
{
	struct sensor_data *data = NULL;
	static struct input_dev *input_data = NULL;
	int input_registered = 0;
	int  sysfs_created = 0;
	int rt;
/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	int sts;
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */
/* FUJITSU:2011-05-26 ALS/PS CORRECT replace start */
	int idx;
	unsigned char nvindex = 0xff;
/* FUJITSU:2011-05-26 ALS/PS CORRECT replace end */

	PROX_DBG("[PROXIMITY] prodrv_sns_probe \n");

/* FUJITSU:2011-05-26 ALS/PS CORRECT replace start */
	// body color read
	g_prox_nvid = PROX_NVID_BASE;
#ifdef EIFTEST
	g_prox_nvdata = PROX_NVDATA_BASE;
#endif
	rt = msm_prodrv_read_nvitem(&g_prox_nvid, &g_prox_nvdata);
	if(rt == 0 )
	{
/* FUJITSU:2011-09-13 panel color read  start */
#ifndef EIFTEST
		nvindex = (unsigned char)((g_prox_nvdata  & 0x000000ff ));
		//nvindex = (unsigned char)((0x04030201  & 0x000000ff ));
#else
		nvindex = (unsigned char)((g_prox_nvid >> 24) & 0x000000ff );
		//nvindex = (unsigned char)((0x08070605 >> 24) & 0x000000ff );
#endif
/* FUJITSU:2012-01-16 bsp-sensor mod start */
		PROX_DBG( "[PROXIMITY] prodrv_color_info : nv read success idx[%d] id[%08x] data[%08x] \n", nvindex, g_prox_nvid, g_prox_nvdata );
/* FUJITSU:2012-01-16 bsp-sensor mod end */
/* FUJITSU:2011-09-13 panel color read  end */
	}
	else
	{
/* FUJITSU:2012-01-16 bsp-sensor mod start */
		PROX_PRINT( KERN_ERR "proximity: probe NV read error [%d] \n", rt );
/* FUJITSU:2012-01-16 bsp-sensor mod end */
	}

#if DEBUG_SUPPORT_TEST
	PROX_DBG("[PROXIMITY] prodrv_sns_init tune mode\n");
	nvindex = 0xff;	// default
#endif

	// select of housing
	for(idx=0;prox_correct[idx].index != 0xff;idx++) {
		if(prox_correct[idx].index == nvindex) {
			break;
		}
	}

	PROX_DBG("[PROXIMITY] idx= %d \n", idx);
	// register and correction initialize
	g_i_led = prox_correct[idx].i_led;
/* FUJITSU:2011-09-27 start */
	g_ps_hi = prox_correct[idx].ps_hi;
	g_ps_low = prox_correct[idx].ps_low;
	g_als_correct = prox_correct[idx].als_correct;
#ifndef EIFTEST
/* FUJITSU:2012-01-26 Next-i change start */
//	if(idx == 3 || idx == 4 || idx == 5){
	if(prox_correct[idx].index != 0xff){
/* FUJITSU:2012-01-26 Next-i change end */
		g_prox_nv_ps_th_h_id = PROX_NV_PS_TH_H_ID_BASE;
		rt = msm_prodrv_read_nvitem(&g_prox_nv_ps_th_h_id, &g_prox_nvdata_ps_th_h);
		if(rt == 0 )
		{
			g_ps_hi = (unsigned char)((g_prox_nvdata_ps_th_h  & 0x000000ff ));
/* FUJITSU:2012-01-16 bsp-sensor mod start */
			PROX_DBG( "[PROXIMITY] prodrv_ps_th_h_info : nv read success hi[%d] hid[%08x] hdata[%08x] \n", g_ps_hi, g_prox_nv_ps_th_h_id, g_prox_nvdata_ps_th_h );
/* FUJITSU:2012-01-16 bsp-sensor mod end */
		}else{
/* FUJITSU:2012-01-16 bsp-sensor mod start */
			PROX_PRINT(KERN_ERR "proximity: probe NV ps_th_l read error [%d] \n", rt);
/* FUJITSU:2012-01-16 bsp-sensor mod end */
		}

		g_prox_nv_ps_th_l_id = PROX_NV_PS_TH_L_ID_BASE;
		rt = msm_prodrv_read_nvitem(&g_prox_nv_ps_th_l_id, &g_prox_nvdata_ps_th_l);
		if(rt == 0 )
		{
			g_ps_low = (unsigned char)((g_prox_nvdata_ps_th_l  & 0x000000ff ));
/* FUJITSU:2012-01-16 bsp-sensor mod start */
			PROX_DBG( "[PROXIMITY] prodrv_ps_th_l_info : nv read success lw[%d] lid[%08x] ldata[%08x] \n", g_ps_low, g_prox_nv_ps_th_l_id, g_prox_nvdata_ps_th_l );
/* FUJITSU:2012-01-16 bsp-sensor mod end */
		}else{
/* FUJITSU:2012-01-16 bsp-sensor mod start */
			PROX_PRINT(KERN_ERR "proximity: probe NV ps_th_l read error [%d] \n", rt);
/* FUJITSU:2012-01-16 bsp-sensor mod end */
		}

		if(0 == factory_mode()){
			g_prox_nv_light_hosei_id = PROX_NV_LIGHT_HOSEI_ID_BASE;
			rt = msm_prodrv_read_nvitem(&g_prox_nv_light_hosei_id, &g_prox_nvdata_light_hosei);
			if(rt == 0 )
			{
				g_als_correct = (unsigned char)((g_prox_nvdata_light_hosei  & 0x000000ff ));
/* FUJITSU:2012-01-16 bsp-sensor mod start */
				PROX_DBG( "[PROXIMITY] prodrv_light_hosei_info : nv read success cor[%d] hsid[%08x] hsdata[%08x] \n", g_als_correct, g_prox_nv_light_hosei_id, g_prox_nvdata_light_hosei );
/* FUJITSU:2012-01-16 bsp-sensor mod end */
			}else{
/* FUJITSU:2012-03-12 ACE change start */
/* FUJITSU:2012-01-16 bsp-sensor mod start */
//			PROX_PRINT(KERN_ERR "proximity: probe NV light_hosei read error [%d] \n", rt);
				PROX_DBG("[PROXIMITY] probe NV light_hosei read nothing [%d] then default cor[%d] \n", rt, g_als_correct);
/* FUJITSU:2012-01-16 bsp-sensor mod end */
/* FUJITSU:2012-03-12 ACE change end */
			}
		}
	}
/* FUJITSU:2011-09-27 end */
#endif
	g_als_sens = prox_correct[idx].als_sens;
	PROX_DBG("[PROXIMITY] prodrv_sns_init global"
			"(led=%d,ps_hi=%d,ps_low=%d,als_sen=%d,als_corr=%d\n"
			,g_i_led,g_ps_hi,g_ps_low,g_als_sens,g_als_correct);

	PROX_PRINT(KERN_INFO "proximity: probe(%d)\n",nvindex);

/* FUJITSU:2011-05-26 ALS/PS CORRECT replace end */

	// GPIO initialize
/* FUJITSU:2011-05-26 checker start */
	rt = gpio_request(g_gpio_number, SENSOR_NAME);
	if (rt) {
		/* gpio error */
		PROX_DBG("[PROXIMITY] prodrv_sns_probe: gpio_request error \n");
		goto err;
	}
/* FUJITSU:2011-05-26 checker end */

	// I2C initialize
/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	sts = prodrv_sns_ON();
	if(sts != true) {
		rt = -EBUSY;
		goto err;
	}
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	// driver management memory allocate
	data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	if (!data) {
		rt = -ENOMEM;
		goto err;
	}
	data->enabled = 0;
	data->delay = SENSOR_DEFAULT_DELAY;

	// proximity device allocate 
	input_data = input_allocate_device();
	if (!input_data) {
		rt = -ENOMEM;
		goto err;
	}

	set_bit(EV_ABS, input_data->evbit);
	input_set_capability(input_data, EV_ABS, ABS_X);
/* FUJITSU:2012-03-06 ACE add start */
	input_set_capability(input_data, EV_ABS, ABS_WAKE);
	input_set_capability(input_data, EV_ABS, ABS_CONTROL_REPORT);
/* FUJITSU:2012-03-06 ACE add end */
/* FUJITSU:2011-12-01 ICS start */
	input_set_abs_params(input_data, ABS_X, 0, 1, 0, 0);
/* FUJITSU:2011-12-01 ICS end */

	input_data->name = SENSOR_NAME;

	// proximity device register
	rt = input_register_device(input_data);
	if (rt) {
		goto err;
	}
	input_registered = 1;

	// data store device member
	input_set_drvdata(input_data,data);

	// proximity sysfs initialize
	rt = sysfs_create_group(&input_data->dev.kobj,
			&sensor_attribute_group);
	if (rt) {
		goto err;
	}
	sysfs_created = 1;

	mutex_init(&data->mutex);

#if testing1
	this_data = input_data;
	input_report_abs(input_data, ABS_X, PROX_VAL_FAR);
	input_sync(input_data);	
#endif
	// proximity interrupt initialize
	prodrv_sns_request_irqs();

	return 0;

err:
	if (data != NULL) {
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
	PROX_PRINT(KERN_ERR "proximity: probe error (%d)\n",rt);

	return rt;
}

/* ----------------------------------------------------------------- */
/*
 * remove interface
 */
static int
prodrv_sns_remove(struct platform_device *pdev)
{
	struct sensor_data *data;

	PROX_DBG("[PROXIMITY] prodrv_sns_remove \n");
	
	if (this_data != NULL) {
		data = input_get_drvdata(this_data);
		sysfs_remove_group(&this_data->dev.kobj,
				&sensor_attribute_group);
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
	.probe	  = prodrv_sns_probe,
	.remove	 = prodrv_sns_remove,
	.suspend	= prodrv_sns_suspend,
	.resume	 = prodrv_sns_resume,
	.driver = {
		.name   = SENSOR_NAME,
		.owner  = THIS_MODULE,
	},
};

/* ----------------------------------------------------------------- */
/*
 * proximity sensor active-mode setting function
 */
int
prodrv_sns_ON(void)
{
	int ok_flag = true;

	PROX_DBG("[PROXIMITY] prodrv_sns_ON \n");

	// proximity status initialize
	prox_detect.proxy_val = PROX_VAL_NEAR;
	prox_detect.proxy_state = PROX_DETECT_NON;
	prox_detect.proxy_lux_save = 0;
	memset(prox_detect.proxy_distance_save,0,
		sizeof(prox_detect.proxy_distance_save));
	prox_detect.proxy_distreg_save =0;

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	// proximty interrupt (GPIO) signal setting
#ifndef CONFIG_GPIOLIB
	gpio_configure(g_gpio_number, GPIOF_ENABLE_WAKE | GPIOF_INPUT);
#else
	gpio_direction_input(g_gpio_number);
#endif
	
	//added to set it to no pull as per spec
	gpio_tlmm_config(
		GPIO_CFG(g_gpio_number, 0, GPIO_INPUT, g_gpio_pullup, g_gpio_strength)
																,GPIO_ENABLE);
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	msleep(SNS_STANDBY_TIME);

	// i2c register initialize
	ok_flag = prodrv_sns_reg_init();
	if(ok_flag!=true) {
#ifndef CONFIG_GPIOLIB
		gpio_configure(g_gpio_number, GPIOF_DRIVE_OUTPUT);
#else
		gpio_direction_output(g_gpio_number, 1);
#endif
	}
#if 0
       // Use wake lock to stop suspending during calls.
       wake_lock(&prox_wake_lock);
       pr_crit("[PROXIMITY] get wake lock"); 
#endif
	return ok_flag;
}

/* ----------------------------------------------------------------- */
/*
 * proximity sensor standby-mode setting function
 */
void
prodrv_sns_OFF(void)
{
	PROX_DBG("[PROXIMITY] prodrv_sns_OFF \n");

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	// GPIO register finalize
#ifndef CONFIG_GPIOLIB
	gpio_configure(g_gpio_number, GPIOF_DRIVE_OUTPUT);
#else
	gpio_direction_output(g_gpio_number, 1);
#endif
	
	gpio_tlmm_config(
		GPIO_CFG(g_gpio_number, 0, GPIO_OUTPUT, g_gpio_pullup, g_gpio_strength)
																,GPIO_DISABLE);
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	// i2c register finalize
	prodrv_sns_ssd();

	// driver vars reset
	prox_detect.proxy_val = PROX_VAL_NEAR;
	prox_detect.proxy_state = PROX_DETECT_NON;
	prox_detect.proxy_lux_save = 0;
	memset(prox_detect.proxy_distance_save,0,
		sizeof(prox_detect.proxy_distance_save));
	prox_detect.proxy_distreg_save =0;
#if 0
       wake_unlock(&prox_wake_lock);
       pr_crit("[PROXIMITY] release wake lock"); 
#endif
}

/* ----------------------------------------------------------------- */
/*
 * proximity sensor I2C active-mode setting function
 */
static int
prodrv_sns_reg_init(void)
{
	int		init_sts;

	// ALS setting (Illuminate setting)
	// proximity sensor device (ALS meas mode) initialize
	init_sts = prodrv_sns_i2c_write(I2C_PROX_ALS_SENSITIVITY,g_als_sens);
	PROX_DBG("[PROXIMITY] prodrv_sns_reg_init ALS_SENS = %02x\n",g_als_sens);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init ALS_CONTROL FALSE \n");
		return false;
	}

	// proximity sensor device (ALS meas mode) initialize
	init_sts = prodrv_sns_i2c_write(I2C_PROX_ALS_MEAS_RATE, 
						ALS_RATE_ENABLE | ALS_MEAS_RATE_500ms);	// ALS interval
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init ALS_CONTROL FALSE \n");
		return false;
	}

	// proximity sensor device (ALS control mode) initialize
	init_sts = prodrv_sns_i2c_write(I2C_PROX_ALS_CONTROL, ALS_MODE_STANDALONE);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init ALS_CONTROL FALSE \n");
		return false;
	}
	i2c_proximity_ready = 1;		// I2C proximity ALS access ready

	// PS setting (proximity setting)
	// proximity sensor device (LED current) initialize
	init_sts = prodrv_sns_i2c_write(I2C_PROX_I_LED, g_i_led);	
	PROX_DBG("[PROXIMITY] prodrv_sns_reg_init I_LED = %02x\n",g_i_led);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init I_LED FALSE \n");
		return false;
	}

	// proximity sensor device (PS interrupt threshold [HI]) initialize
/* FUJITSU:2011-05-10 ALS/PS CORRECT start */
	init_sts = prodrv_sns_i2c_write(I2C_PROX_PS_TH_H_LED1, g_ps_hi);
/* FUJITSU:2011-05-10 ALS/PS CORRECT end */
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init INTERRUPT HI TH  \n");
		return false;
	}

	// proximity sensor device (PS interrupt threshold [LOW]) initialize
/* FUJITSU:2011-05-10 ALS/PS CORRECT start */
	init_sts = prodrv_sns_i2c_write(I2C_PROX_PS_TH_L_LED1, g_ps_low);
/* FUJITSU:2011-05-10 ALS/PS CORRECT end */
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init INTERRUPT LO TH  \n");
		return false;
	}

	// proximity sensor device (PS meas rate) initialize
	init_sts = prodrv_sns_i2c_write(I2C_PROX_PS_MEAS_RATE, PS_MEAS_RATE_50ms);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init PS meas rate\n");
		return false;
	}

	// proximity sensor device (ALS/PS persistence) initialize
	// proximity detect rate = meas rate x persistence = 50ms x 3 = 150ms
	init_sts = prodrv_sns_i2c_write(I2C_PROX_PERSISTENCE, 0x03);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init PS presistence\n");
		return false;
	}

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	// proximity sensor device (interrupt mode) default set (default FAR->NEAR)
	prox_detect.proxy_intr_save = FUJITSU_INTERRUPT_RESET;

	// proximity sensor device (interrupt mode) initialize (disable intr.)
	init_sts = prodrv_sns_i2c_write(I2C_PROX_INTERRUPT, FUJITSU_INTERRUPT_RESET);
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init INTERRUPT FALSE \n");
		return false;
	}
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

	// proximity sensor device (PS control mode) initialize
/* FUJITSU:2011-11-29 PS CONTROL change start */
	init_sts = prodrv_sns_i2c_write(I2C_PROX_PS_CONTROL, PS_MODE_STANDBY);	
/* FUJITSU:2011-11-29 PS CONTROL change end */
	if(0 > init_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_reg_init PS_CONTROL FALSE \n");
		return false;
	}

	return true;
}

/* ----------------------------------------------------------------- */
/*
 * proximity sensor I2C standby-mode setting function
 * @param   reg    [in] register address
 * @param   data   [in] write data buffer
 * @return  0      :    normal end
 *          !0     :    i2c_transfer error
 */
static void
prodrv_sns_ssd(void)
{
	int		final_sts;

	PROX_DBG("[PROXIMITY] prodrv_sns_ssd \n");

	// proximity sensor device (PS) standby
	final_sts = prodrv_sns_i2c_write(I2C_PROX_PS_CONTROL, PS_MODE_STANDBY);	
	if(0 > final_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_ssd PS_CONTROL FALSE \n");
	}

	i2c_proximity_ready = 0;	// I2C proximity ALS access not ready

	// proximity sensor device (ALS) standby
	final_sts = prodrv_sns_i2c_write(I2C_PROX_ALS_CONTROL, ALS_MODE_STANDBY);
	if(0 > final_sts) {
		PROX_DBG("[PROXIMITY] prodrv_sns_ssd ALS_CONTROL FALSE \n");
	}
}

/* ----------------------------------------------------------------- */
/*
 * does i2c write to the proximity sensor
 * @param   reg    [in] register address
 * @param   data   [in] write data buffer
 * @return  0      :    normal end
 *          !0     :    i2c_transfer error
 */
static int
prodrv_sns_i2c_write(unsigned char reg, unsigned char data)
{
	struct i2c_msg msg;
	u_int8_t buf[I2C_WRITE_BUFSIZE];
	int ret = 0;
/* FUJITSU:2011-10-20 I2C ACCESS start */
	int retry_count = 0;
/* FUJITSU:2011-10-20 I2C ACCESS end */

	msg.addr = PROX_SLAVE_ADDRESS;
	msg.flags = 0;
	buf[0] = reg;
	buf[1] = data;

	msg.buf  = buf;
	msg.len  = 2;

/* FUJITSU:2011-10-20 I2C ACCESS start */
	for (;;) {
		if ((ret = i2c_transfer(i2c_proximity, &msg, I2C_TRANS_WRITE)) >= 0) {
			break;				// Normal termination
		}

		PROX_PRINT(KERN_ERR "proximity: prodrv_sns_i2c_write I2C error"
				"reg 0x%02X ret(%d) retry_count %d\n", reg, ret,retry_count);

		if (ret == -ETIMEDOUT) {
			return ret;
		}

		if (retry_count >= I2C_RTRY_CNT) {
			PROX_PRINT(KERN_ERR "proximity: prodrv_sns_i2c_write retry over \n");
			return ret;
		}
		retry_count++;
		msleep(I2C_RTRY_WAIT);
	}
/* FUJITSU:2011-10-20 I2C ACCESS end */

	return ret;
}

/* ----------------------------------------------------------------- */
/*
 * does i2c read from the proximity sensor
 * @param   reg    [in] register address
 * @param   data   [out]read data buffer
 * @param   len    [out]read data buffer length
 * @return  0      :    normal end
 *          !0     :    i2c_transfer error
 */
static int
prodrv_sns_i2c_read(unsigned short reg, unsigned char *data, uint32_t len)
{
	struct i2c_msg msg[READ_SIZE];
	u_int8_t msgbuf[READ_SIZE];
	int ret = 0;
/* FUJITSU:2011-10-20 I2C ACCESS start */
	int retry_count = 0;
/* FUJITSU:2011-10-20 I2C ACCESS end */

	memcpy(msgbuf, &reg, sizeof(reg));

	msg[0].addr  = PROX_SLAVE_ADDRESS;
	msg[0].flags = 0;
	msg[0].buf   = msgbuf;
	msg[0].len   = 1;

	msg[1].addr  = PROX_SLAVE_ADDRESS;
	msg[1].flags = I2C_M_RD;
	msg[1].buf   = data;
	msg[1].len   = len;

/* FUJITSU:2011-10-20 I2C ACCESS start */
	for (;;) {
		if ((ret = i2c_transfer(i2c_proximity, msg, I2C_TARNS_READ)) >= 0) {
			break;				// Normal termination
		}

		PROX_PRINT(KERN_ERR "proximity: prodrv_sns_i2c_read I2C error"
				"reg 0x%02X ret(%d) retry_count %d\n", reg, ret,retry_count);

		if (ret == -ETIMEDOUT) {
			return ret;
		}

		if (retry_count >= I2C_RTRY_CNT) {
			PROX_PRINT(KERN_ERR "proximity: prodrv_sns_i2c_read retry over \n");
			return ret;
		}
		retry_count++;
		msleep(I2C_RTRY_WAIT);
	}
/* FUJITSU:2011-10-20 I2C ACCESS end */

	return ret;
}

/* ----------------------------------------------------------------- */
/*
 * proximity ALS register get
 * @param   lux    [out]illumi sensor data (Lux)
 * @return  0      :    normal end
 *          !0     :    error
 */
int
prodrv_sns_als_data_read(int *lux)
{
	unsigned char data1[READ_SIZE];
	int 		res;

//	PROX_DBG("[PROXIMITY] prodrv_sns_als_data_read \n");
	
	/*--------------------------------------------------------------------
		ALS standalone mode
	--------------------------------------------------------------------*/
	*lux=0;

	res = prodrv_sns_i2c_read(I2C_PROX_ALS_PS_STATUS, data1, 1);
	if (res < 0){
		PROX_PRINT(KERN_ERR "proximity: als status I2C access error\n");
		PROX_DBG("[PROXIMITY] prodrv_sns_als_data_read PS status failed\n");
		return res;
	}

	if( data1[0] & STS_ALSDATA_BIT ) {
		// Illumi sensor value change
		
		// Illumi sensor value(Hi/Low) get
		res = prodrv_sns_i2c_read(I2C_PROX_ALS_DATA_0, data1, 2);
		if (res < 0){
			PROX_PRINT(KERN_ERR "proximity: als data I2C access error\n");
			PROX_DBG("[PROXIMITY] prodrv_sns_als_data_read ALS read failed\n");
			return res;
		}
		*lux = (data1[1] << 8) | data1[0];
		prox_lux_data = *lux;

/* FUJITSU:2012-01-26 Next-i change start */
#if defined(CONFIG_MACH_F11SKY)
		*lux = *lux * g_als_correct;
#else
		if(0 == factory_mode()) {
/* FUJITSU:2011-11-29 PS CONTROL change start */
/* FUJITSU:2011-05-10 ALS/PS CORRECT start */
		// Illumi sensor value software correct 
/* FUJITSU:2012-03-06 ACE change start */
//		*lux = *lux * g_als_correct * PROX_ALS_SENS_DEF / g_als_sens;
		*lux = *lux * g_als_correct * g_als_sens / PROX_ALS_SENS_DEF;
/* FUJITSU:2012-03-06 ACE change end */
/* FUJITSU:2011-05-10 ALS/PS CORRECT end */
/* FUJITSU:2011-11-29 PS CONTROL change end */
		}
#endif
/* FUJITSU:2012-01-26 Next-i change end */

		prox_detect.proxy_lux_save = *lux;

	}
	else {
		// Illumi sensor value not change (save data value)
		*lux = prox_detect.proxy_lux_save;

	}

	return 0;
}
EXPORT_SYMBOL(prodrv_sns_als_data_read);

/* ----------------------------------------------------------------- */
/*
 * does distance report save function
 */
void prodrv_sns_report_distance(void)
{
	int res;
	int dat;
	unsigned char data1[READ_SIZE];
 
	// proximity sensor value get
	res = prodrv_sns_i2c_read(I2C_PROX_PS_DATA_LED1, data1, 1);
	if (res < 0){
		PROX_DBG("[PROXIMITY] prodrv_sns_report_distance read failed\n");
		return;
	}
	dat = data1[0];

	prox_detect.proxy_distreg_save = dat;
	// conv. to distance[cm]
	res = prodrv_sns_convert_data(dat, prox_detect.proxy_distance_save);
	if (res < 0){
		PROX_PRINT(KERN_ERR "proximiy: distance convert error (%d)\n",dat);
		PROX_DBG("[PROXIMITY] prodrv_sns_report_distance data-conver failed\n");
		return;
	}

	PROX_DBG("[PROXIMITY] prodrv_sns_report_distance dat=%d dist=%s\n",dat,
			prox_detect.proxy_distance_save);

	return;
}

/* ----------------------------------------------------------------- */
/*
 * does proximiy sensor distance convert
 */
#define DIST_LIST_MAX 256
static int prodrv_sns_convert_data(int data, char *val)
{
	// scale:cm
	static char distance_list[DIST_LIST_MAX][8] = {
		"22.000","21.506","21.024","20.552","20.091","19.641","19.200","18.770",//   0-
		"18.349","17.937","17.535","17.142","16.758","16.382","16.014","15.655",//   8-
		"15.304","14.961","14.625","14.297","13.977","13.663","13.357","13.057",//  16-
		"12.765","12.478","12.198","11.925","11.657","11.396","11.140","10.891",//  24-
		"10.646","10.408","10.174", "9.946", "9.723", "9.505", "9.292", "9.083",//  32-
		 "8.880", "8.681", "8.486", "8.296", "8.110", "7.928", "7.750", "7.576",//  40-
		 "7.406", "7.240", "7.078", "6.919", "6.764", "6.612", "6.464", "6.319",//  48-
		 "6.177", "6.039", "5.903", "5.771", "5.641", "5.515", "5.391", "5.270",//  56-
		 "5.152", "5.037", "4.924", "4.813", "4.705", "4.600", "4.496", "4.396",//  64-
		 "4.297", "4.201", "4.106", "4.014", "3.924", "3.836", "3.750", "3.666",//  72-
		 "3.584", "3.504", "3.425", "3.348", "3.273", "3.200", "3.128", "3.058",//  80-
		 "2.989", "2.922", "2.857", "2.793", "2.730", "2.669", "2.609", "2.550",//  88-
		 "2.493", "2.437", "2.382", "2.329", "2.277", "2.226", "2.176", "2.127",//  96-
		 "2.079", "2.033", "1.987", "1.942", "1.899", "1.856", "1.815", "1.774",// 104-
		 "1.734", "1.695", "1.657", "1.620", "1.584", "1.548", "1.513", "1.480",// 112-
		 "1.446", "1.414", "1.382", "1.351", "1.321", "1.291", "1.262", "1.234",// 120-
		 "1.206", "1.179", "1.153", "1.127", "1.102", "1.077", "1.053", "1.029",// 128-
		 "1.006", "0.983", "0.961", "0.940", "0.919", "0.898", "0.878", "0.858",// 136-
		 "0.839", "0.820", "0.802", "0.784", "0.766", "0.749", "0.732", "0.716",// 144-
		 "0.700", "0.684", "0.669", "0.654", "0.639", "0.625", "0.611", "0.597",// 152-
		 "0.584", "0.570", "0.558", "0.545", "0.533", "0.521", "0.509", "0.498",// 160
		 "0.487", "0.476", "0.465", "0.455", "0.444", "0.434", "0.425", "0.415",// 168
		 "0.406", "0.397", "0.388", "0.379", "0.371", "0.362", "0.354", "0.346",// 176
		 "0.338", "0.331", "0.323", "0.316", "0.309", "0.302", "0.295", "0.289",// 184
		 "0.282", "0.276", "0.270", "0.264", "0.258", "0.252", "0.246", "0.241",// 192
		 "0.235", "0.230", "0.225", "0.220", "0.215", "0.210", "0.205", "0.201",// 200
		 "0.196", "0.192", "0.187", "0.183", "0.179", "0.175", "0.171", "0.167",// 208
		 "0.163", "0.160", "0.156", "0.153", "0.149", "0.146", "0.143", "0.139",// 216
		 "0.136", "0.133", "0.130", "0.127", "0.124", "0.122", "0.119", "0.116",// 224
		 "0.114", "0.111", "0.109", "0.106", "0.104", "0.101", "0.099", "0.097",// 232
		 "0.095", "0.093", "0.090", "0.088", "0.086", "0.084", "0.083", "0.081",// 240
		 "0.079", "0.077", "0.075", "0.074", "0.072", "0.070", "0.069", "0.067"	// 248
	};

	if((data >=  DIST_LIST_MAX) || (data < 0)){
		return -1;
	}

	memcpy(val,distance_list[data],8);
	return 0;
}

/* ----------------------------------------------------------------- */
/*
 * does proximity block hander
 */
/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
static void
prodrv_sns_work_bh(struct work_struct *work)
{
	int res;
	unsigned char data1[READ_SIZE];
	
	PROX_DBG("[PROXIMITY] prodrv_sns_work_bh \n");

	msleep(SNS_STANDBY_TIME);

	// proximity sensor device (interrupt mode) read-reset
	res = prodrv_sns_i2c_read(I2C_PROX_INTERRUPT, data1, 1);
	if(0 > res) {
		PROX_PRINT(KERN_ERR "proximity: intr-read I2C access error\n");
		PROX_DBG("[PROXIMITY] prodrv_sns_work_bh INTERRUPT read FALSE \n");
	}

	PROX_DBG("[PROXIMITY] prodrv_sns_work_bh WAKE \n");
	atomic_set(&g_prox_state,PROX_STATE_WAKE);

	// status setting
	prodrv_sns_work_bh_sub();

	enable_irq(g_proxi_irq);
}

static void
prodrv_sns_work_bh_sub(void)
{
	int res;
	unsigned char data1[READ_SIZE];

	PROX_DBG("[PROXIMITY] prodrv_sns_work_bh_sub \n");
#if ALS_FIX_TEST
	if(atomic_read(&g_prox_state)) {
#else
	if(atomic_read(&g_prox_state) != PROX_STATE_SUSPEND) {
#endif
		// PS status read
		res = prodrv_sns_i2c_read(I2C_PROX_ALS_PS_STATUS, data1, 1);
		if (res < 0){
			PROX_PRINT(KERN_ERR "proximity: ps status I2C access error\n");
			PROX_DBG("[PROXIMITY] prodrv_sns_work_bh_sub PS status read  "
					"failed  \n");
			return;
		}

		// NEAR or FAR check
		if(data1[0] & STS_PSINT_BIT) {
			// bit ON (PS intrrupt ACTIVE) = NEAR
			PROX_DBG("[PROXIMITY] prodrv_sns_work_bh_sub NEAR (PS_STA=%02x) \n"
					,data1[0]);

			// save state
			prox_detect.proxy_val = PROX_VAL_NEAR;
			prox_detect.proxy_state = PROX_DETECTED;

			// proximity sensor device (interrupt mode) change
			prox_detect.proxy_intr_save = FUJITSU_INT_SET_N2F;
	
		}
		else {
			// bit OFF (PS intrrupt INACTIVE) = FAR
			PROX_DBG("[PROXIMITY] prodrv_sns_work_bh_sub FAR (PS_STA=%02x) \n"
					,data1[0]);

			// save state
			prox_detect.proxy_val = PROX_VAL_FAR;
			prox_detect.proxy_state = PROX_DETECT_NON;
			
			// proximity sensor device (interrupt mode) change
			prox_detect.proxy_intr_save = FUJITSU_INT_SET_F2N;
	
		}

		// distance data get
		prodrv_sns_report_distance();
#if testing1
#if 0
		// report save
		input_report_abs(this_data, ABS_X, 1);
		input_sync(this_data);

#else
		// report save
		input_report_abs(this_data, ABS_X, prox_detect.proxy_val);
		input_sync(this_data);
#endif
#endif
	}


}
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */

/* ----------------------------------------------------------------- */
/*
 * does proximity irq handler
 */
static irqreturn_t
prodrv_sns_irq_handler(int irq, void *p)
{
/* FUJITSU:2011-07-13 INTR change start */
/* FUJITSU:2012-01-26 Next-i change start */
	if((atomic_read(&g_wake)) /*&& (system_rev > 0x02)*/){
/* FUJITSU:2012-01-26 Next-i change end */
		disable_irq_nosync(g_proxi_irq);
		schedule_work(&g_proxi_work_data);
	}
/* FUJITSU:2011-07-13 INTR change end */
	return IRQ_HANDLED;
}


/* ----------------------------------------------------------------- */
/*
 * proximity irq  setting
 * @param   (none)
 * @return  0      :  normal end
 *          !0     :  irq initialize error
 */
static int
prodrv_sns_request_irqs(void)
{
	int err;
/* FUJITSU:2011-07-13 INTR change start */
	unsigned long req_flags = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING;
/* FUJITSU:2011-07-13 INTR change end */

	err = g_proxi_irq = gpio_to_irq(g_gpio_number);
	err = request_irq(g_proxi_irq, prodrv_sns_irq_handler,
			req_flags, SENSOR_NAME, NULL);
	if (err) {
		PROX_PRINT(KERN_ERR "proximity: irq init fail\n");
		return err;
	}
/* FUJITSU:2011-12-01 ICS start */
#if 1
	set_irq_wake(g_proxi_irq, WAKE_ON);
#else
	irq_set_irq_wake(g_proxi_irq, WAKE_ON);
#endif
/* FUJITSU:2011-12-01 ICS end */
	return 0;
}

/* ----------------------------------------------------------------- */
/*
 * i2c adapter setting
 * @param   (none)
 * @return  !0       : I2C adapter descriptor
 *           0(NULL) : ERROR
 */
struct i2c_adapter * prodev_i2c_search_adapter(void)
{
	struct i2c_adapter *	adptmp = NULL;
/* FUJITSU:2011-05-10 PROXIMITY I2C adapter-set change start */
	i2c_proximity = i2c_get_adapter(0);
	adptmp = i2c_proximity;

/* FUJITSU:2011-05-10 PROXIMITY I2C adapter-set change end */

	return adptmp;
}


/* ----------------------------------------------------------------- */
/*
 * sensor initializeation
 * @param   (none)
 * @param   data   [in] write data buffer
 * @return  0      :    normal end
 *          !0     :    init error
 */
static int __init prodrv_sns_init(void)
{

	PROX_DBG("[PROXIMITY] prodrv_sns_init \n");
	
	i2c_proximity = prodev_i2c_search_adapter();
/* FUJITSU:2011-04-21 NULL_check start */
	if (NULL == i2c_proximity){
		PROX_PRINT(KERN_ERR "proximity: i2c init fail\n");
		return -1;
	}
/* FUJITSU:2011-04-21 NULL_check end */

/* FUJITSU:2011-05-11 SENSOR not SUSPEND start */
	// interrupt GPIO configure
	PROX_DBG("[PROXIMITY] prodrv_sns_init system_rev(%d)\n",system_rev);
	g_gpio_number = PROX_GPIO_REV1;
	g_gpio_pullup = GPIO_PULL_UP;
	g_gpio_strength = GPIO_2MA;
	PROX_DBG("[PROXIMITY] prodrv_sns_init config(num=%d,pull=%d,str=%d\n"
			,g_gpio_number,g_gpio_pullup,g_gpio_strength);
/* FUJITSU:2011-05-11 SENSOR not SUSPEND end */


	INIT_WORK(&g_proxi_work_data, prodrv_sns_work_bh);
	atomic_set(&g_wake,false);
	atomic_set(&g_suspend_off,false);
	atomic_set(&g_prox_state,PROX_STATE_WAKE);

	sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
	if (IS_ERR(sensor_pdev)) {
		PROX_PRINT(KERN_ERR "proximity: sensor device registraton error (%s)\n",SENSOR_NAME);
		return -1;
	}
#if 0
	wake_lock_init(&prox_wake_lock, WAKE_LOCK_SUSPEND, "taos");///////////////////////////////////////////////////test2013-05-31
#endif
	return platform_driver_register(&sensor_driver);
}
module_init(prodrv_sns_init);

/* ----------------------------------------------------------------- */
/*
 * sensor shutdown
 * @param   (none)
 * @return  (none)
 */
static void __exit prodrv_sns_exit(void)
{
	PROX_DBG("[PROXIMITY] prodrv_sns_exit \n");
	
	prodrv_sns_OFF();
	platform_driver_unregister(&sensor_driver);
	platform_device_unregister(sensor_pdev);
}
module_exit(prodrv_sns_exit);

MODULE_AUTHOR("Yamaha Corporation");
MODULE_LICENSE( "GPL" );
MODULE_VERSION("1.0.0");
