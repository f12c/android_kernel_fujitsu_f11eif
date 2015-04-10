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
#include <linux/regulator/consumer.h> /* FUJITSU : 2012-1-19 add camea */
//#include <linux/adxl345.h>
#include <media/msm_camera.h>
#include <mach/camera.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
//#include <plat/pm.h>
//#include <mach/pm.h>
//#include "sub_pmic.h"
#ifndef Camsensor_gpio_back_H
#include "camsensor_gpio.h"
#endif // Camsensor_gpio_back_H
//#include "tsb_model.h"
#include "../../../arch/arm/mach-msm/smd_private.h"
//#include "pm.h"

#define M6MORL_DRV_NAME  "m6morl_imx046es"

#define LOGI(fmt, args...)      printk(KERN_INFO "m6morl_imx046es: " fmt, ##args)
//#define LOGI(fmt, args...)      do{}while(0)
#define LOGD(fmt, args...)      printk(KERN_DEBUG "m6morl_imx046es: " fmt, ##args)
//#define LOGD(fmt, args...)      do{}while(0)
#define LOGE(fmt, args...)      printk(KERN_ERR "m6morl_imx046es: " fmt, ##args)

static bool is_use_qup_i2c = false;

// GPIO
#define RESET                   25
#define MCLK                    15
#define CAM_INT                 24
#define CAM_I2C_SDA             108
#define CAM_I2C_SCL             109

// temp
#define SUPPORTED_READ_TEMP

/*===================================================================*
    LOCAL DECLARATIONS
 *===================================================================*/
struct m6morl_imx046es_ctrl {
    const struct msm_camera_sensor_info *sensordata;
//    int model;
//    int led;
};

DEFINE_MUTEX(m6morl_imx046es_mtx);

struct m6morl_imx046es_work_t {
	struct work_struct work;
};
static DECLARE_WAIT_QUEUE_HEAD(m6morl_imx046es_wait_queue);
static struct m6morl_imx046es_work_t *m6morl_imx046es_sensorw;
static struct m6morl_imx046es_ctrl *m6morl_imx046es_ctrl = NULL;
static struct i2c_client *m6morl_imx046es_i2c_client = NULL;

/*===================================================================*
    EXTERNAL DECLARATIONS
 *===================================================================*/
extern int factory_mode(void);
//extern int _I2C_WAIT_;
int _I2C_WAIT_ = 12;
extern int _I2C_LOG_;

////////////////////////////////
// I2C
////////////////////////////////
int m6morl_imx046es_i2c_write(struct CameraSensorI2CCmdTypeIsp* pI2CCmd)
{
	int rc = 0;
	if( is_use_qup_i2c ) {
		rc = camsensor_i2c_write(m6morl_imx046es_i2c_client, pI2CCmd);
	} else {
		rc = camsensor_gpioi2c_write_isp(pI2CCmd);
	}
	return rc;
}

int m6morl_imx046es_i2c_read(struct CameraSensorI2CCmdTypeIsp* pI2CCmd)
{
	int rc = 0;
	if( is_use_qup_i2c ) {
		rc = camsensor_i2c_read(m6morl_imx046es_i2c_client, pI2CCmd);
	} else {
		rc = camsensor_gpioi2c_read_isp(pI2CCmd);
	}
	return rc;
}

int m6morl_imx046es_i2c_write_normal(struct CameraSensorI2CCmdType* pI2CCmd)
{
	int rc = 0;
	if( is_use_qup_i2c ) {
		rc = camsensor_i2c_write_normal(m6morl_imx046es_i2c_client, pI2CCmd);
	} else {
		rc = camsensor_gpioi2c_write(pI2CCmd);
	}
	return rc;
}

int m6morl_imx046es_i2c_read_normal(struct CameraSensorI2CCmdType* pI2CCmd)
{
	int rc = 0;
	if( is_use_qup_i2c ) {
		rc = camsensor_i2c_read_normal(m6morl_imx046es_i2c_client, pI2CCmd);
	} else {
		rc = camsensor_gpioi2c_read(pI2CCmd);
	}
	return rc;
}

////////////////////////////////
// Power ON
////////////////////////////////
#define VREG_TABLE  4
/* FUJITSU : 2012-01-19 chg camera start */
#if 0 //changed by jellicle
static struct regulator_bulk_data regs[VREG_TABLE] = {
/* FUJITSU:2011_12_27 add camera start */
	{ .supply = "gp5",  .min_uV = 1200000, .max_uV = 1200000 },
	{ .supply = "gp10", .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "wlan", .min_uV = 2800000, .max_uV = 2800000 },
	{ .supply = "gp2",  .min_uV = 2800000, .max_uV = 2800000 },
/* FUJITSU:2011_12_27 add camera end */
};
#else
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
#endif
/* FUJITSU : 2012-01-19 chg camera end */

static int m6morl_imx046es_sensor_poweron(void)
{
    int i;
	//int rc;	/* FUJITSU : 2012-01-19 add camera */
	
	LOGI("+%s()\n", __func__);

    gpio_tlmm_config( GPIO_CFG(CAM_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );

    for(i=0;i<VREG_TABLE;++i) {
/* FUJITSU : 2012-01-19 chg camera start */
#if 0 //changed by jellicle
    	rc = regulator_bulk_enable(1, &regs[i]);
		if (rc) {
			LOGE("%s:regulator_bulk_enable enable failed !\n", __func__);
            goto _sensor_poweron_fail;
        }
		mdelay(1);
#else
        LOGD("-%s vreg(%s)\n", __func__, vreg_info[i].name);
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
            goto _sensor_poweron_fail;
        }
        mdelay(1);
#endif
/* FUJITSU : 2012-01-19 chg camera end */
    }

	// Clock
    gpio_tlmm_config( GPIO_CFG(MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), GPIO_CFG_ENABLE );
    msm_camio_clk_rate_set( 24576000 );
	msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
	mdelay(1);

    // Reset
    gpio_tlmm_config( GPIO_CFG(RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
    gpio_set_value( RESET, 1 );
    mdelay(5);
	
	if( !is_use_qup_i2c ) {
		LOGI("+%s (i2c gpio cfg)\n", __func__);
		gpio_tlmm_config(GPIO_CFG(CAM_I2C_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(CAM_I2C_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
   	}

	// Max Clock Stretch(us) = _I2C_WAIT_ * 1000 * 3 
	if( factory_mode() ) {
		_I2C_WAIT_ = 3333;
	} else {
		_I2C_WAIT_ = 12;
	}

	LOGI("-%s Done(i2c wait:%d).\n", __func__, _I2C_WAIT_);
    return 0;

_sensor_poweron_fail:
    for(i=0;i<VREG_TABLE;++i) {
/* FUJITSU : 2012-01-19 chg camera start */
#if 0 //changed by jellicle
    	regulator_bulk_disable(1, &regs[i]);
#else
        vreg_disable(vreg_info[i].vreg);
#endif
/* FUJITSU : 2012-01-19 chg camera end */
    }

    LOGE("-%s Failed...\n", __func__);
    return -1;
}

////////////////////////////////
// Power OFF
////////////////////////////////
static void m6morl_imx046es_sensor_poweroff(void)
{
    int i;

    LOGI("+%s()\n", __func__);

	if( !is_use_qup_i2c ) {
		LOGI("+%s(After 2-1) GPIO CFG\n", __func__);
		gpio_tlmm_config(GPIO_CFG(CAM_I2C_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(CAM_I2C_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(CAM_I2C_SDA, 1);	// KO_I2C3_SDA
		gpio_set_value(CAM_I2C_SCL, 1);	// KO_I2C3_SCL
   	}

	// Reset
    gpio_set_value( RESET, 0 );
    mdelay(5);

	// Clock
    gpio_tlmm_config( GPIO_CFG(MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
    mdelay(5);

    for(i=VREG_TABLE-1; i>=0; --i) {
/* FUJITSU : 2012-01-19 chg camera start */
#if 0 //changed by jellicle
    	regulator_bulk_disable(1, &regs[i]);
		mdelay(5);
#else
        LOGD("-%s vreg(%s)\n", __func__, vreg_info[i].name);
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
#endif
/* FUJITSU : 2012-01-19 chg camera end */
    }

    gpio_tlmm_config( GPIO_CFG(CAM_INT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );

    LOGI("-%s Done.\n", __func__);
}

//=====================================================================
// Driver Function
//=====================================================================
//---------------------------------------------------------------------
// msm_open_control
//---------------------------------------------------------------------
int m6morl_imx046es_sensor_init(const struct msm_camera_sensor_info *data)
{
    LOGI("+%s()\n", __func__);
    
    m6morl_imx046es_ctrl = kzalloc(sizeof(struct m6morl_imx046es_ctrl), GFP_KERNEL);
    if (!m6morl_imx046es_ctrl) {
        LOGE(" -%s kzalloc() Failed!\n",__func__);
        return -ENOMEM;
    }

    if (data)
        m6morl_imx046es_ctrl->sensordata = data;

    // Sensor Power ON
    if (m6morl_imx046es_sensor_poweron() < 0) {
        kfree(m6morl_imx046es_ctrl);
        LOGE("-%s Failed.\n", __func__);
        return -1;
    }

    return 0;
}

static int32_t m6morl_imx046es_video_config(int mode)
{
    struct msm_camera_csi_params m6morl_imx046es_csi_params;
    int32_t rc = 0;
    LOGI("-%s top.\n", __func__);
    msm_camio_vfe_clk_rate_set(192000000);
    m6morl_imx046es_csi_params.data_format = CSI_10BIT;
    m6morl_imx046es_csi_params.lane_cnt = 2;
    m6morl_imx046es_csi_params.lane_assign = 0xe4;
    m6morl_imx046es_csi_params.dpcm_scheme = 0;
    m6morl_imx046es_csi_params.settle_cnt = 0x18;
    rc = msm_camio_csi_config(&m6morl_imx046es_csi_params);
    if (rc < 0)
        LOGE("-%s msm_camio_csi_config Failed(%d).\n", __func__, rc);
    msleep(10);

    return rc;
}

static int32_t m6morl_imx046es_set_sensor_mode(int mode,
    int res)
{
    int32_t rc = 0;
    LOGD("-%s top.\n", __func__);
    switch (mode) {
    case SENSOR_PREVIEW_MODE:
        LOGI("-%s SENSOR_PREVIEW_MODE.\n", __func__);
        rc = m6morl_imx046es_video_config(mode);
        break;
    case SENSOR_SNAPSHOT_MODE:
    case SENSOR_RAW_SNAPSHOT_MODE:
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
int m6morl_imx046es_sensor_config(void __user *argp)
{
    struct CameraSensorI2CCmdTypeIsp   I2CCmd;
	struct CameraSensorI2CCmdType      I2CCmdNormal;
    struct sensor_cfg_data cfg;
#ifdef SUPPORTED_READ_TEMP
    uint32_t *smem_ptr = NULL;
#endif
    int rc = 0;
	int cpl = 0;

    LOGD("+%s()\n", __func__);
    
    if (copy_from_user(&cfg, (void *)argp, sizeof(struct sensor_cfg_data)))
        return -EFAULT;

    mutex_lock(&m6morl_imx046es_mtx);
    switch (cfg.cfgtype) {

    case CFG_GET_TEMP:
#ifdef SUPPORTED_READ_TEMP
        smem_ptr = (uint32_t *)smem_alloc_vendor1(SMEM_OEM_004); 
        if(smem_ptr == NULL){
            LOGE("+%s (CFG_GET_TEMP) smem_ptr is null!\n", __func__);
            rc = -EINVAL;
            break;
        }
        if( system_rev >= 0x0C ) {
            cfg.cfg.temp = 0x70;  // normal temperature.
        } else {
            cfg.cfg.temp = *smem_ptr;
        }
#else
        cfg.cfg.temp = 0x70;  // normal temperature.
#endif
        LOGI("+%s (CFG_GET_TEMP:%d)\n", __func__, cfg.cfg.temp);
        if (copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data)))
            rc = -EFAULT;
        break;

    case CFG_PWR_UP:
        LOGI("-%s (CFG_PWR_UP)\n", __func__);
        // Sensor Power OFF
        m6morl_imx046es_sensor_poweroff();

        mdelay(10);

        // Sensor Power ON
        rc =m6morl_imx046es_sensor_poweron();
        if(rc < 0) {
            LOGE("-%s CFG_PWR_UP Failed!\n", __func__);
        }
        break;

    case CFG_COMMAND:
        LOGD("-%s (CFG_COMMAND)\n", __func__);
        _I2C_LOG_ = cfg.rs;
        I2CCmd.slave_addr = 0x1F;
    	I2CCmd.category   = cfg.cfg.cmd_isp.category;
    	I2CCmd.byte       = cfg.cfg.cmd_isp.byte;
        I2CCmd.pwdata     = cfg.cfg.cmd_isp.wvalue;
        I2CCmd.wlen       = cfg.cfg.cmd_isp.txlen;
        I2CCmd.prdata     = cfg.cfg.cmd_isp.rvalue;
        I2CCmd.rlen       = cfg.cfg.cmd_isp.rxlen;
        
		if (!cfg.cfg.cmd_isp.rxlen) {
        	rc = m6morl_imx046es_i2c_write(&I2CCmd);
			if (rc) {
				LOGE("<write err> [c:%02x b:%02x l:%d]\n", cfg.cfg.cmd_isp.category, cfg.cfg.cmd_isp.byte, cfg.cfg.cmd_isp.txlen);
			}
		} else {
        	rc = m6morl_imx046es_i2c_read(&I2CCmd);
			if( !rc ) {
				for( cpl=0; cpl<cfg.cfg.cmd_isp.rxlen; cpl++ ) {
					cfg.cfg.cmd_isp.rvalue[cpl] = I2CCmd.prdata[cpl];
					LOGD("+i2c_read (%02x)\n", cfg.cfg.cmd_isp.rvalue[cpl] );
				}
	            rc = copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data));
	            if(rc != 0){
	            	LOGE("-%s copy_to_user() Failed!(%d)\n", __func__, rc);
	                rc = -EFAULT;
				}
			} else {
				LOGE("<read err> [c:%02x b:%02x l:%d]\n", cfg.cfg.cmd_isp.category, cfg.cfg.cmd_isp.byte, cfg.cfg.cmd_isp.rxlen);
			}
		}

        _I2C_LOG_ = 1;
        break;

    case CFG_COMMAND_NORMAL:
        LOGD("-%s (CFG_COMMAND_NORMAL)\n", __func__);
        _I2C_LOG_ = cfg.rs;
        I2CCmdNormal.slave_addr = 0x1F;
        I2CCmdNormal.pwdata     = cfg.cfg.cmd.wvalue;
        I2CCmdNormal.wlen       = cfg.cfg.cmd.txlen;
        I2CCmdNormal.prdata     = cfg.cfg.cmd.rvalue;
        I2CCmdNormal.rlen       = cfg.cfg.cmd.rxlen;

		if (!cfg.cfg.cmd.rxlen) {
        	rc = m6morl_imx046es_i2c_write_normal(&I2CCmdNormal);
		} else {
        	rc = m6morl_imx046es_i2c_read_normal(&I2CCmdNormal);
			if( !rc ) {
				for( cpl=0; cpl<cfg.cfg.cmd.rxlen; cpl++ ) {
					cfg.cfg.cmd.rvalue[cpl] = I2CCmdNormal.prdata[cpl];
					LOGD("+i2c_read (%02x)\n", cfg.cfg.cmd.rvalue[cpl] );
				}
			}
		}
        if (!rc)
            if (copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data)))
                rc = -EFAULT;
        _I2C_LOG_ = 1;
        break;
	
    case CFG_SET_MODE:
        LOGD("-%s (CFG_SET_MODE)\n", __func__);
        rc = m6morl_imx046es_set_sensor_mode(cfg.mode, cfg.rs);
        break;

    case CFG_CAM_INT:
        cfg.cfg.cam_int = gpio_get_value(CAM_INT);
        LOGD("-%s (CFG_CAM_INT:%d)\n", __func__, cfg.cfg.cam_int);
        if (copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data)))
        	rc = -EFAULT;
        break;

    default:
        LOGE("-%s ERR root:%d\n", __func__, cfg.cfgtype);
        rc = -EINVAL;
        break;
    }
    mutex_unlock(&m6morl_imx046es_mtx);

    if (rc) LOGI("-%s Done.(%d)\n", __func__, rc);
    return rc;
}

//---------------------------------------------------------------------
// msm_release_control()
//---------------------------------------------------------------------
int m6morl_imx046es_sensor_release(void)
{
    LOGI("+%s\n", __func__);

    mutex_lock(&m6morl_imx046es_mtx);

    m6morl_imx046es_sensor_poweroff();
    kfree(m6morl_imx046es_ctrl);

    mutex_unlock(&m6morl_imx046es_mtx);

    LOGI("-%s Done.\n", __func__);
    return 0;
}

/////////////////////////////////////
// Sensor Driver Setup (Kernel Init)
/////////////////////////////////////

// I2C Driver for 1-2Ver. 
static const struct i2c_device_id m6morl_imx046es_i2c_id[] = {
	{ M6MORL_DRV_NAME, 0},
	{ },
};

static int m6morl_imx046es_init_client(struct i2c_client *client)
{
    LOGI("+%s()\n", __func__);
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&m6morl_imx046es_wait_queue);
	return 0;
}


static int __devinit m6morl_imx046es_i2c_probe( struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;

    LOGI("+%s()\n", __func__);
    
	if( !i2c_check_functionality(client->adapter, I2C_FUNC_I2C) ) {
		LOGE("i2c_check_functionality failed\n");
		return -1;
	}

	m6morl_imx046es_sensorw = kzalloc(sizeof(struct m6morl_imx046es_work_t), GFP_KERNEL);
	if (!m6morl_imx046es_sensorw) {
		LOGE("kzalloc failed.\n");
		rc = -ENOMEM;
		return rc;
	}

	i2c_set_clientdata(client, m6morl_imx046es_sensorw);
	m6morl_imx046es_init_client(client);
	m6morl_imx046es_i2c_client = client;

	msleep(50);

	LOGI("m6morl_imx046es_probe successed! rc = %d\n", rc);
	return 0;

}

static struct i2c_driver m6morl_imx046es_i2c_driver = {
	.id_table = m6morl_imx046es_i2c_id,
	.probe  = m6morl_imx046es_i2c_probe,
	.remove = __exit_p(m6morl_imx046es_i2c_remove),
	.driver = {
		.name = M6MORL_DRV_NAME,
	},
};

static int m6morl_imx046es_sensor_probe(const struct msm_camera_sensor_info *info,
                                struct msm_sensor_ctrl *s)
{
	int rc = 0;

#ifdef CONFIG_MACH_F11EIF
	if( system_rev >= 0x0C ) {
		// Before 1-3
		is_use_qup_i2c = true;
	} else {
		// After 2-1
		is_use_qup_i2c = false;
	}
#else
	is_use_qup_i2c = false;
#endif
    LOGI("+%s() i2c(%d)\n", __func__, is_use_qup_i2c);

	if( is_use_qup_i2c ) {
		rc = i2c_add_driver( &m6morl_imx046es_i2c_driver );
		if( rc < 0 ) {
			rc = -ENOTSUPP;
			return rc;
		}
	}

    s->s_init = m6morl_imx046es_sensor_init;
    s->s_release = m6morl_imx046es_sensor_release;
    s->s_config  = m6morl_imx046es_sensor_config;

	s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle  = 90;

    return 0;
}

static int __m6morl_imx046es_probe(struct platform_device *pdev)
{
/* FUJITSU : 2012-01-19 add camera start */
	//int rc;

	//struct device *dev = &pdev->dev;
/* FUJITSU : 2012-01-19 add camera end */
	
    LOGI("+%s()\n", __func__);

/* FUJITSU : 2012-01-19 add camera start */
	/* Use gp6 and gp16 if and only if dev name matches. */
	//rc = regulator_bulk_get(dev, VREG_TABLE, regs);

	//if (rc) {
	//	dev_err(dev, "%s: could not get regulators: %d\n",
	//			__func__, rc);
	//	return -1;
	//}

	//rc = regulator_bulk_set_voltage(VREG_TABLE, regs);

	//if (rc) {
	//	dev_err(dev, "%s: could not set voltages: %d\n",
	//			__func__, rc);
	//	goto reg_free;
	//}
/* FUJITSU : 2012-01-19 add camera end */

    return msm_camera_drv_start(pdev, m6morl_imx046es_sensor_probe);

/* FUJITSU : 2012-01-19 add camera start */
//reg_free:
//	regulator_bulk_free(VREG_TABLE, regs);
//	return -1;
/* FUJITSU : 2012-01-19 add camera end */
}

static struct platform_driver msm_camera_driver = {
    .probe = __m6morl_imx046es_probe,
    .driver = {
        .name = "msm_camera_m6morl_imx046es",
        .owner = THIS_MODULE,
    },
};

static int __init m6morl_imx046es_init(void)
{
    LOGI("+%s()\n", __func__);
    return platform_driver_register(&msm_camera_driver);
}

module_init(m6morl_imx046es_init);
