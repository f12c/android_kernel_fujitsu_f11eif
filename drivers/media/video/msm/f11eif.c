/*
 * Copyright(C) 2011 FUJITSU LIMITED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
//#include <linux/adxl345.h>
#include <media/msm_camera.h>
#include <mach/camera.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
//#include <plat/pm.h>
//#include <mach/pm.h>
//#include "sub_pmic.h"
#include "camsensor_gpio.h"
//#include "tsb_model.h"
#include "../../../arch/arm/mach-msm/smd_private.h"
//#include "pm.h"

// EXSAMPLE Todo:
#define F11EIF_DRV_NAME  "m6mo_cam"

#if 1
#define LOGI(fmt, args...)      printk(KERN_DEBUG "f11eif: " fmt, ##args)
#else
#define LOGI(fmt, args...)      do{}while(0)
#endif
#define LOGE(fmt, args...)      printk(KERN_ERR "f11eif: " fmt, ##args)

// GPIO
#define RESET                   25
//#define H_STBY                  1
#define MCLK                    15
//#define R_STBY                  151
//#define CAM_18V                 29
#define CAM_INT                 24
#define CAM_I2C_SDA             108
#define CAM_I2C_SCL             109

/*===================================================================*
    LOCAL DECLARATIONS
 *===================================================================*/
struct f11eif_ctrl {
    const struct msm_camera_sensor_info *sensordata;
//    int model;
//    int led;
};

DEFINE_MUTEX(f11eif_mtx);

struct f11eif_work_t {
	struct work_struct work;
};
static DECLARE_WAIT_QUEUE_HEAD(f11eif_wait_queue);
static struct f11eif_work_t *f11eif_sensorw;
static struct f11eif_ctrl *f11eif_ctrl = NULL;
static struct i2c_client *f11eif_i2c_client = NULL;

/*===================================================================*
    EXTERNAL DECLARATIONS
 *===================================================================*/
extern int _I2C_LOG_;

//////////////////////////////////
// LED Control
//////////////////////////////////

// led is not support
#if 0
static void f11eif_led_control(int ctrl)
{
    switch (ctrl) {
    case LED_OFF:
        if(f11eif_ctrl->model == TSB_MODEL_NO_0 || f11eif_ctrl->model == TSB_MODEL_NO_1) {
            pmic_set_led_intensity(LED_LCD, 0);
            pmic_set_led_intensity(LED_KEYPAD, 0);
        } else {
            pmic_secure_mpp_config_i_sink(PM_MPP_13,f11eif_ctrl->led,
                                          PM_MPP__I_SINK__SWITCH_DIS);
            pmic_secure_mpp_config_i_sink(PM_MPP_18,f11eif_ctrl->led,
                                          PM_MPP__I_SINK__SWITCH_DIS);
        }
        if ( vreg_disable(vreg_get(NULL, "boost"))) // VREG_5V
            LOGE("%s: vreg_5Vdisable failed !\n", __func__);
        break;
    case LED_LOW:
    case LED_HIGH:
        if ( vreg_enable(vreg_get(NULL, "boost")))  // VREG_5V
            LOGE("%s: vreg_5V enable failed !\n", __func__);
        mdelay(1);

        if(f11eif_ctrl->model == TSB_MODEL_NO_0 || f11eif_ctrl->model == TSB_MODEL_NO_1) {
            pmic_set_led_intensity(LED_LCD, 1);
            pmic_set_led_intensity(LED_KEYPAD, 1);
        } else {
            f11eif_ctrl->led = ctrl == LED_LOW ? PM_MPP__I_SINK__LEVEL_10mA :
                                                  PM_MPP__I_SINK__LEVEL_15mA;
            pmic_secure_mpp_config_i_sink(PM_MPP_13, f11eif_ctrl->led,
                                          PM_MPP__I_SINK__SWITCH_ENA);
            pmic_secure_mpp_config_i_sink(PM_MPP_18, f11eif_ctrl->led,
                                          PM_MPP__I_SINK__SWITCH_ENA);
        }
        break;
    }
}

////////////////////////////////
// SPI SYNC
////////////////////////////////
static int f11eif_spi(uint8_t *ptx, uint8_t *prx, uint32_t len)
{
    struct spi_message  msg;
    struct spi_transfer xfer;
    int rc = 0;

    spi_message_init(&msg);
    memset((void*)&xfer, 0, sizeof(xfer));
    xfer.tx_buf = ptx;
    xfer.len = len;
    xfer.rx_buf = prx;
    xfer.bits_per_word = 16;
    xfer.speed_hz = 26330000;
    spi_message_add_tail(&xfer, &msg);
    rc = spi_sync(f11eif_spi_dev, &msg);
    if (rc < 0) LOGE(" - spi_sync(): Send Error (%d)\n", rc);
    return rc;
}

////////////////////////////////
// DL
////////////////////////////////
static int f11eif_dl(struct cfg_dl *dl)
{
    int i;
    uint8_t rx[2];
    int32_t sz = dl->len[0] + dl->len[1] + dl->len[2] + dl->len[3];
    uint8_t *pMem = kzalloc(sz, GFP_KERNEL);
    uint8_t *p[4];

    if(!pMem) {
        LOGE(" -%s kzalloc() Failed!\n",__func__);
        return -ENOMEM;
    }

    p[0] = pMem;
    p[1] = p[0] + dl->len[0];
    p[2] = p[1] + dl->len[1];
    p[3] = p[2] + dl->len[2];

    for(i=0;i<4;++i) {
        if (copy_from_user(p[i], dl->dt[i], dl->len[i])) {
            LOGE(" * copy_from_user (%d)Error !\n", i);
            goto f11eif_dl_exit1;
        }
        LOGI(" >> %03d.bin DL Start (Size:%d) >>\n", i, dl->len[i]);
        if (f11eif_spi(p[i], NULL, dl->len[i])) {
            LOGE(" * %03d.bin SPI Send Error ! \n", i);
            goto f11eif_dl_exit1;
        }
        if (!i) {
            mdelay(1);
            continue;
        }
        mdelay(5);
        if (f11eif_spi(NULL, rx, sizeof(rx))) {
            LOGE(" * %03d.bin DL Receive Error ! \n", i);
            goto f11eif_dl_exit1;
        }
        if (rx[0] != 0x07 || rx[1] != 0x01) {
            LOGE(" * %03d.bin DL Response Error ! %02X%02X\n", i, rx[0], rx[1]);
            goto f11eif_dl_exit1;
        }
        LOGI(" << %03d.bin DL Complete. <<\n", i);
        mdelay(1);
    }
    kfree(pMem);
    mdelay(9);
    return 0;

f11eif_dl_exit1:
    kfree(pMem);
    return -1;
}
#endif

////////////////////////////////
// I2C
////////////////////////////////
int f11eif_i2c_write(struct CameraSensorI2CCmdTypeIsp* pI2CCmd)
{
	int rc = 0;
	// Before 1-3
	if( system_rev >= 0x0C ) {
		rc = camsensor_i2c_write(f11eif_i2c_client, pI2CCmd);
	} else {
	// After 2-1
		rc = camsensor_gpioi2c_write_isp(pI2CCmd);
	}
	return rc;
}

int f11eif_i2c_read(struct CameraSensorI2CCmdTypeIsp* pI2CCmd)
{
	int rc = 0;
	// Before 1-3
	if( system_rev >= 0x0C ) {
		rc = camsensor_i2c_read(f11eif_i2c_client, pI2CCmd);
	} else {
	// After 2-1
		rc = camsensor_gpioi2c_read_isp(pI2CCmd);
	}
	return rc;
}

int f11eif_i2c_write_normal(struct CameraSensorI2CCmdType* pI2CCmd)
{
	int rc = 0;
	// Before 1-3
	if( system_rev >= 0x0C ) {
		rc = camsensor_i2c_write_normal(f11eif_i2c_client, pI2CCmd);
	} else {
	// After 2-1
		rc = camsensor_gpioi2c_write(pI2CCmd);
	}
	return rc;
}

int f11eif_i2c_read_normal(struct CameraSensorI2CCmdType* pI2CCmd)
{
	int rc = 0;
	// Before 1-3
	if( system_rev >= 0x0C ) {
		rc = camsensor_i2c_read_normal(f11eif_i2c_client, pI2CCmd);
	} else {
	// After 2-1
		rc = camsensor_gpioi2c_read(pI2CCmd);
	}
	return rc;
}

#if 0 // M6Mo Firm DL
#include "m6mo_dl_tp.c"
#endif

////////////////////////////////
// Power ON
////////////////////////////////
#define VREG_TABLE  4
static struct vreg_info {
    char *name;
    unsigned int lvl;
    struct vreg *vreg;
} vreg_info[VREG_TABLE] = {
    {"gp5", 1200, NULL},	// VREG_L23 set level 1.2V
    {"gp10", 1800, NULL},	// VREG_L16 set level 1.8V
    {"wlan", 2800, NULL},	// VREG_L13 set level 2.8V
    {"gp2", 2800, NULL},	// VREG_L11 set level 2.8V
};

static int f11eif_sensor_poweron(void)
{
    int i;
//	int rc = 0;
	LOGI("+%s()\n", __func__);

    gpio_tlmm_config( GPIO_CFG(CAM_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );

    for(i=0;i<VREG_TABLE;++i) {
        LOGI("-%s vreg(%s)\n", __func__, vreg_info[i].name);
        vreg_info[i].vreg = vreg_get(NULL, vreg_info[i].name);
        if (!vreg_info[i].vreg) {
            LOGE("%s: * vreg_get(%s) failed !\n", __func__, vreg_info[i].name);
            return -1;
        }
        if (vreg_set_level(vreg_info[i].vreg, vreg_info[i].lvl)) {
            LOGE("%s: * %s set level failed !\n", __func__, vreg_info[i].name);
            return -1;
        }
        if (vreg_enable(vreg_info[i].vreg)) {
            LOGE("%s: * vreg %s enable failed !\n", __func__, vreg_info[i].name);
            return -1;
        }
        mdelay(1);
    }

	LOGI("+%s(Clock)\n", __func__);

	// Clock
    gpio_tlmm_config( GPIO_CFG(MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), GPIO_CFG_ENABLE );
	LOGI("+%s(Clock1)\n", __func__);
    msm_camio_clk_rate_set( 24576000 );
	msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
	LOGI("+%s(Clock2)\n", __func__);
//    msm_camio_camif_pad_reg_reset();
//	LOGI("+%s(Clock3)\n", __func__);

	mdelay(1);

	LOGI("+%s(Reset)\n", __func__);
    // Reset
    gpio_tlmm_config( GPIO_CFG(RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
//	rc = gpio_request( RESET, "msm_cam_f11eif" );
//	LOGI("Reset Result %d\n", rc);
    gpio_set_value( RESET, 1 );

    mdelay(5);
	
	LOGI("+%s(Reset1)\n", __func__);
	
	// After 2-1
   	if( system_rev < 0x0C ) {
		LOGI("+%s(After 2-1) GPIO CFG\n", __func__);
		gpio_tlmm_config(GPIO_CFG(CAM_I2C_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(CAM_I2C_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
   	}

	LOGI("+%s(End)\n", __func__);
    return 0;
}

////////////////////////////////
// Power OFF
////////////////////////////////
static void f11eif_sensor_poweroff(void)
{
    int i;
    int value;

    LOGI("+%s()\n", __func__);

	// After 2-1
   	if( system_rev < 0x0C ) {
		LOGI("+%s(After 2-1) GPIO CFG\n", __func__);
		gpio_tlmm_config(GPIO_CFG(CAM_I2C_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(CAM_I2C_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(CAM_I2C_SDA, 1);	// KO_I2C3_SDA
		gpio_set_value(CAM_I2C_SCL, 1);	// KO_I2C3_SCL
		value = gpio_get_value(CAM_I2C_SDA);
		LOGI("+GPIO108 (%d)\n", value );
		value = gpio_get_value(CAM_I2C_SCL);
		LOGI("+GPIO109 (%d)\n", value );
   	}

	// Reset
    gpio_set_value( RESET, 0 );

    mdelay(5);

	// Clock
    gpio_tlmm_config( GPIO_CFG(MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );

    mdelay(5);

    for(i=VREG_TABLE-1; i>=0; --i) {
        LOGI("-%s vreg(%s)\n", __func__, vreg_info[i].name);
        vreg_info[i].vreg = vreg_get(NULL, vreg_info[i].name);
        if (!vreg_info[i].vreg) {
            LOGE("%s: * vreg_get(%s) failed !\n", __func__, vreg_info[i].name);
        }
        if (vreg_disable(vreg_info[i].vreg)) {
            LOGE("%s: * vreg %s disable failed !\n", __func__, vreg_info[i].name);
        }
        vreg_put(vreg_info[i].vreg);
        vreg_info[i].vreg = NULL;
        mdelay(1);
    }

    gpio_tlmm_config( GPIO_CFG(CAM_INT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );

}

//=====================================================================
// Driver Function
//=====================================================================
//---------------------------------------------------------------------
// msm_open_control
//---------------------------------------------------------------------
int f11eif_sensor_init(const struct msm_camera_sensor_info *data)
{
    LOGI("+%s()\n", __func__);
    
    f11eif_ctrl = kzalloc(sizeof(struct f11eif_ctrl), GFP_KERNEL);
    if (!f11eif_ctrl) {
        LOGE(" -%s kzalloc() Failed!\n",__func__);
        return -ENOMEM;
    }

    if (data)
        f11eif_ctrl->sensordata = data;

	// Model is nessesary?
//    f11eif_ctrl->model = tsb_model_get_model_no();

    // Sensor Power ON
    if (f11eif_sensor_poweron() < 0) {
        kfree(f11eif_ctrl);
        LOGI("-%s Failed.\n", __func__);
        return -1;
    }
#if 0 //tp
    LOGI("-M6MO_DL START!!.\n");
    // [M6MO_DL]download start!!
    if (f11eif_m6mo_dl() < 0) {
        kfree(f11eif_ctrl);
        LOGE("-%s Failed.\n", __func__);
        return -1;
    }
    // [M6MO_DL]download end!!
    LOGI("-M6MO_DL END!!.\n");

    LOGI("-CHKSUM START!!.\n");
    // [M6MO_DL]checksum calc start!!
    if (f11eif_m6mo_checksum() < 0) {
        kfree(f11eif_ctrl);
        LOGE("-%s Failed.\n", __func__);
        return -1;
    }
    // [M6MO_DL]checksum calc end!!
    LOGI("-CHKSUM END!!.\n");
    // Sensor Power Off
    f11eif_sensor_poweroff();
    mdelay(10);
    // Sensor Power ON
    if (f11eif_sensor_poweron() < 0) {
        kfree(f11eif_ctrl);
        LOGI("-%s Failed.\n", __func__);
        return -1;
    }
    LOGI("-POWERON RESTART END!!.\n");
#endif

    return 0;
}

static int32_t f11eif_video_config(int mode)
{
    struct msm_camera_csi_params f11eif_csi_params;
    int32_t rc = 0;
    LOGI("-%s top.\n", __func__);
    msm_camio_vfe_clk_rate_set(192000000);
    f11eif_csi_params.data_format = CSI_10BIT;
    f11eif_csi_params.lane_cnt = 2;
    f11eif_csi_params.lane_assign = 0xe4;
    f11eif_csi_params.dpcm_scheme = 0;
    f11eif_csi_params.settle_cnt = 0x18;
    rc = msm_camio_csi_config(&f11eif_csi_params);
    if (rc < 0)
        LOGE("-%s msm_camio_csi_config Failed(%d).\n", __func__, rc);
    msleep(10);

    return rc;
}

static int32_t f11eif_set_sensor_mode(int mode,
    int res)
{
    int32_t rc = 0;
    LOGI("-%s top.\n", __func__);
    switch (mode) {
    case SENSOR_PREVIEW_MODE:
        LOGI("-%s SENSOR_PREVIEW_MODE.\n", __func__);
        rc = f11eif_video_config(mode);
        break;
//    case SENSOR_SNAPSHOT_MODE:
//        rc = f11eif_snapshot_config(mode);
//        break;
//    case SENSOR_RAW_SNAPSHOT_MODE:
//        rc = f11eif_raw_snapshot_config(mode);
//        break;
    default:
        rc = -EINVAL;
        break;
    }
    LOGI("-%s end(%d).\n", __func__, rc);
    return rc;
}

//---------------------------------------------------------------------
// msm_ioctl_control()
//---------------------------------------------------------------------
int f11eif_sensor_config(void __user *argp)
{
    struct CameraSensorI2CCmdTypeIsp   I2CCmd;
	struct CameraSensorI2CCmdType      I2CCmdNormal;
    struct sensor_cfg_data cfg;
    uint32_t *smem_ptr = NULL;
    int   rc = 0;
	int   cpl = 0;

//    LOGI("+%s()\n", __func__);
    
    if (copy_from_user(&cfg, (void *)argp, sizeof(struct sensor_cfg_data)))
        return -EFAULT;

    mutex_lock(&f11eif_mtx);
    switch (cfg.cfgtype) {
//    case CFG_PWR_UP:
//        adxl345_stop_ap();
//        rc = f11eif_dl(&cfg.cfg.dl);
//        adxl345_start_ap();
//        break;
//
    case CFG_GET_TEMP:
//        smem_ptr = (uint32_t *)smem_alloc(SMEM_OEM_013, sizeof(int)); 
        smem_ptr = (uint32_t *)smem_alloc_vendor1(SMEM_OEM_004); 
        if(smem_ptr == NULL){
            LOGE("+%s (CFG_GET_TEMP) smem_ptr is null!\n", __func__);
            rc = -EINVAL;
        } else {
            cfg.cfg.temp = *smem_ptr;
            LOGI("+%s (CFG_GET_TEMP:%d)\n", __func__, cfg.cfg.temp);
            if (copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data)))
                rc = -EFAULT;
        }
        break;
//
//    case CFG_SET_LED:
//        f11eif_led_control(cfg.cfg.led);
//        break;
//
    case CFG_PWR_UP:
        LOGI("-%s (CFG_PWR_UP)\n", __func__);
        // Sensor Power OFF
        f11eif_sensor_poweroff();

        mdelay(10);

        // Sensor Power ON
        rc =f11eif_sensor_poweron();
        if(rc < 0) {
            LOGE("-%s CFG_PWR_UP Failed!\n", __func__);
        }
        break;

    case CFG_COMMAND:
        LOGI("-%s (CFG_COMMAND)\n", __func__);
        _I2C_LOG_ = cfg.rs;
        I2CCmd.slave_addr = 0x1F;
    	I2CCmd.category   = cfg.cfg.cmd_isp.category;
    	I2CCmd.byte       = cfg.cfg.cmd_isp.byte;
        I2CCmd.pwdata     = cfg.cfg.cmd_isp.wvalue;
        I2CCmd.wlen       = cfg.cfg.cmd_isp.txlen;
        I2CCmd.prdata     = cfg.cfg.cmd_isp.rvalue;
        I2CCmd.rlen       = cfg.cfg.cmd_isp.rxlen;
        
		if (!cfg.cfg.cmd_isp.rxlen) {
        	rc = f11eif_i2c_write(&I2CCmd);
		} else {
        	rc = f11eif_i2c_read(&I2CCmd);
			if( !rc ) {
				for( cpl=0; cpl<cfg.cfg.cmd_isp.rxlen; cpl++ ) {
					cfg.cfg.cmd_isp.rvalue[cpl] = I2CCmd.prdata[cpl];
//					LOGI("+i2c_read (%02x)\n", cfg.cfg.cmd_isp.rvalue[cpl] );
				}
	            rc = copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data));
	            if(rc != 0){
	            	LOGE("-%s copy_to_user() Failed!(%d)\n", __func__, rc);
	                rc = -EFAULT;
				}
			}
		}

        _I2C_LOG_ = 1;
        break;

    case CFG_COMMAND_NORMAL:
        LOGI("-%s (CFG_COMMAND_NORMAL)\n", __func__);
        _I2C_LOG_ = cfg.rs;
        I2CCmdNormal.slave_addr = 0x1F;
        I2CCmdNormal.pwdata     = cfg.cfg.cmd.wvalue;
        I2CCmdNormal.wlen       = cfg.cfg.cmd.txlen;
        I2CCmdNormal.prdata     = cfg.cfg.cmd.rvalue;
        I2CCmdNormal.rlen       = cfg.cfg.cmd.rxlen;

		if (!cfg.cfg.cmd.rxlen) {
        	rc = f11eif_i2c_write_normal(&I2CCmdNormal);
		} else {
        	rc = f11eif_i2c_read_normal(&I2CCmdNormal);
			if( !rc ) {
				for( cpl=0; cpl<cfg.cfg.cmd.rxlen; cpl++ ) {
					cfg.cfg.cmd.rvalue[cpl] = I2CCmdNormal.prdata[cpl];
					LOGI("+i2c_read (%02x)\n", cfg.cfg.cmd.rvalue[cpl] );
				}
			}
		}
        if (!rc)
            if (copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data)))
                rc = -EFAULT;
        _I2C_LOG_ = 1;
        break;
	
    case CFG_SET_MODE:
        LOGI("-%s (CFG_SET_MODE)\n", __func__);
        rc = f11eif_set_sensor_mode(cfg.mode, cfg.rs);
        break;

    case CFG_CAM_INT:
        LOGI("-%s (CFG_CAM_INT)\n", __func__);
        cfg.cfg.cam_int = gpio_get_value(CAM_INT);
		LOGI("+cam_int (%d)\n", cfg.cfg.cam_int );
        if (copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data)))
        	rc = -EFAULT;
        break;

    default:
        LOGI("-%s ERR root:%d\n", __func__, cfg.cfgtype);
        rc = -EINVAL;
        break;
    }
    mutex_unlock(&f11eif_mtx);

    if (rc) LOGI("-%s Done.(%d)\n", __func__, rc);
    return rc;
}

//---------------------------------------------------------------------
// msm_release_control()
//---------------------------------------------------------------------
int f11eif_sensor_release(void)
{
    LOGI("+%s\n", __func__);

    mutex_lock(&f11eif_mtx);

//    f11eif_led_control(LED_OFF);
    f11eif_sensor_poweroff();
    kfree(f11eif_ctrl);

    mutex_unlock(&f11eif_mtx);

    LOGI("-%s Done.\n", __func__);
    return 0;
}

/////////////////////////////////////
// Sensor Driver Setup (Kernel Init)
/////////////////////////////////////

// I2C Driver for 1-2Ver. 

static const struct i2c_device_id f11eif_i2c_id[] = {
	{ F11EIF_DRV_NAME, 0},
	{ },
};

#if 0
static int __devinit f11eif_spi_probe(struct spi_device *spi)
{
    int rc = 0;

    f11eif_spi_dev = spi;
    spi->bits_per_word = 16;
    rc = spi_setup(spi);
    if (rc) LOGE(" - spi_setup Error !\n");
    return rc;
}

static int __devexit f11eif_spi_remove(struct spi_device *spi)
{
    LOGI("+%s\n", __func__);
    return 0;
}

static struct spi_driver f11eif_spi_driver = {
    .driver     = {
        .name   = "f11eif",
        .owner  = THIS_MODULE,
    },
    .probe      = f11eif_spi_probe,
    .remove     = __devexit_p(f11eif_spi_remove),
};
#endif

static int f11eif_init_client(struct i2c_client *client)
{
    LOGI("+%s()\n", __func__);
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&f11eif_wait_queue);
	return 0;
}


static int __devinit f11eif_i2c_probe( struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;

    LOGI("+%s()\n", __func__);
    
	if( !i2c_check_functionality(client->adapter, I2C_FUNC_I2C) ) {
		LOGI("i2c_check_functionality failed\n");
		return -1;
	}

	f11eif_sensorw = kzalloc(sizeof(struct f11eif_work_t), GFP_KERNEL);
	if (!f11eif_sensorw) {
		LOGI("kzalloc failed.\n");
		rc = -ENOMEM;
		return rc;
	}

	i2c_set_clientdata(client, f11eif_sensorw);
	f11eif_init_client(client);
	f11eif_i2c_client = client;

	msleep(50);

	LOGI("f11eif_probe successed! rc = %d\n", rc);
	return 0;

}	

static struct i2c_driver f11eif_i2c_driver = {
	.id_table = f11eif_i2c_id,
	.probe  = f11eif_i2c_probe,
	.remove = __exit_p(f11eif_i2c_remove),
	.driver = {
		.name = F11EIF_DRV_NAME,
	},
};

static int f11eif_sensor_probe(const struct msm_camera_sensor_info *info,
                                struct msm_sensor_ctrl *s)
{

	int rc = 0;
    LOGI("+%s()\n", __func__);
	rc = i2c_add_driver( &f11eif_i2c_driver );
	if( rc < 0 ) {
		rc = -ENOTSUPP;
		return rc;
	}

    LOGI("+%s(1)\n", __func__);
    s->s_init = f11eif_sensor_init;
    s->s_release = f11eif_sensor_release;
    s->s_config  = f11eif_sensor_config;

	s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle  = 0;

    return 0;
}

static int __f11eif_probe(struct platform_device *pdev)
{
    LOGI("+%s()\n", __func__);
    return msm_camera_drv_start(pdev, f11eif_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
    .probe = __f11eif_probe,
    .driver = {
        .name = "msm_camera_f11eif",
        .owner = THIS_MODULE,
    },
};

static int __init f11eif_init(void)
{
    LOGI("+%s()\n", __func__);
    return platform_driver_register(&msm_camera_driver);
}

module_init(f11eif_init);
