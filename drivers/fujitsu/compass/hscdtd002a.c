/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/irq.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/gpio.h>

#include <linux/ctype.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include "hscdtd002a.h"

#define debug(format, arg...)	\
	if (0)	compass_printk(KERN_DEBUG, format, ##arg)


const char HSCD_DEVNAME[] = "HSCD";

// HSCD Peculiar instance data
hscd_t hscd[1];

DECLARE_MUTEX(hscd_sem);		// For exclusive control

/*--------------------------------------------------------------------
HSCD Interruption terminal (HSCD.DRDY -> JIKI_DRDY)
--------------------------------------------------------------------*/
#define JIKI_DRDY_OFF  0x00

// When the interruption demand is generated, the flag becomes one.
// It is flag automatically clearness output data is read.
// GPIO True where interruption has been generated.
#define GPIO_ISACTIVE()   (gpio_get_value(JIKI_DRDY_GPIO))

//----------------------------------------------------------------


/*--------------------------------------------------------------------
Prototype of foreign function not declared in header file
--------------------------------------------------------------------*/
#ifdef	CONFIG_BRUTUS
extern int case_getstatus(uint8_t * isOpen);
#endif	// CONFIG_BRUTUS


/*--------------------------------------------------------------------
Function  :The frequency is totaled the present error/retrying.
Attention :Execute it while excluded.
--------------------------------------------------------------------*/
#define hscd_update_error_count()						\
	do {												\
		hscd->total_error_count += hscd->error_count;	\
		hscd->total_retry_count += hscd->retry_count;	\
		hscd->error_count = hscd->retry_count = 0;		\
	} while(false)


/*== HSCD Basic control function (Reading and writing of register)==*/

/*--------------------------------------------------------------------
hscd_read_(verify_)regs() : reg_data[] Number of arguments of minimum elements.
--------------------------------------------------------------------*/
#define READREGS_MINBUFSIZE     3

static struct mutex xfer_lock;

/*--------------------------------------------------------------------
Function:HSCD of RANDOM READ  register is read.
@param	: (1)start_reg:The first register number.
          (2)n_regs:Number of read registers.
          (3)hscd->max_retry_count:Maximum retrying frequency.
output  :reg_data[0 - n_regs-1]:Read data.
I/O	    :When the reading error occurs, the following variable is updated.
          (1)hscd->error_count:+1 
          (2)hscd->retry_count:The generation frequency is multiplied retrying.
@retval :(1)0:Normal termination.
       	  (2)-EBUSY:I2C Communication fault (time out).
--------------------------------------------------------------------*/
static int hscd_read_regs(uint8_t start_reg, uint8_t reg_data[], uint16_t n_regs)
{
	bool error = false;
	unsigned retry_count = 0;
	int ret;
	struct i2c_msg msg[2];
	uint8_t buf0[2];

	debug("%s(start:0x%02X n:%u)\n", __func__, start_reg, n_regs);

	//i2c_transfer Parameter setting
	msg[0].addr = hscd->reg.st_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf0;
	buf0[0] = start_reg;		// Reading beginning position setting

	msg[1].addr = hscd->reg.st_client->addr;
	msg[1].flags = I2C_M_RD;	// Read the register value
	msg[1].len = n_regs;		// Reading number
	msg[1].buf = reg_data;		// Area where reading result is put

	for (;;) {
		debug("%s i2c_transfer start \n", __func__);

		mutex_lock(&xfer_lock);
		if ((ret = i2c_transfer(hscd->reg.st_client->adapter, msg, 2)) >= 0) {
			debug("%s i2c_transfer end \n", __func__);
			mutex_unlock(&xfer_lock);
			break;				// Normal termination
		}
		mutex_unlock(&xfer_lock);

		debug("%s i2c_transfer end \n", __func__);

		// When the error occurs
		error = true;
		if (retry_count >= hscd->max_retry_count) {
			// When ..retrying.. frequency reaches the upper bound:error
			compass_printk(KERN_ERR, "%s: I2C receive error (%d)\n",
														HSCD_DEVNAME, ret);
			goto Exit;
		}
		// It retries. 
		retry_count++;
	}

Exit:
	if (error) {
		// The frequency is totaled error/retrying. 
		hscd->error_count++;
		hscd->retry_count += retry_count;
	}
	return ret;
}

/*--------------------------------------------------------------------
Function  :HSCD of RANDOM READ register is read.
        	It repeats until the same data can be acquired reading
        	two or more times, and continuously two times.
        	 (The maximum of retrying hscd->max_retry_count)
@param  :(1) start_reg:The first register number.
          (2) n_regs:Number of read registers.
          (3) hscd->max_retry_count:Maximum retrying frequency.
output  :reg_data[0 - n_regs-1]:Read data.
I/O :When the reading error occurs, the following variable is updated.
        (1) hscd->error_count:The error generation frequency is multiplied.
        (2) hscd->retry_count:The generation frequency is multiplied retrying.
@retval:(1) 0:Normal termination.
         (2) -EBUSY:I2C Communication fault (time out).
         (3) -EIO:It doesn't succeed in the reading collation even 
                   if ..retrying.. frequency reaches the upper bound. 
--------------------------------------------------------------------*/
static int hscd_read_verify_regs(uint8_t start_reg,
								 uint8_t reg_data[], uint8_t n_regs)
{
	unsigned read_count = 0;	// Read number
	const unsigned max_read_count = hscd->max_retry_count;
	int ret;

	for (;;) {
		ret = hscd_read_regs(start_reg, reg_data, n_regs);
		if (ret >= 0)
			break;
		
		read_count++;
		if (read_count >= max_read_count) {
			compass_printk(KERN_ERR, "%s: %s error (%d)\n",
								HSCD_DEVNAME, __func__, ret);
			goto RetryFailed;
		}
		
	}
	// Collation success
	ret = OK;

Exit:
	if (read_count > 0) {
		// When retrying
		hscd->retry_count += read_count;
		hscd->error_count++;
	}

	return ret;

RetryFailed:					// Exaggerated ..retrying.. frequency

	ret = -EIO;
	goto Exit;
}

/*--------------------------------------------------------------------
Function  :HSCD of WRITE It writes it in the register.
@param  :(1) start_reg:The first register number.
          (2) reg_data[0 - n_regs-1]:Data written in register.
          (3) n_regs (<=HSCD_MAX_CONTIGUOUS_REGS):Number of registers.
          (4) hscd->max_retry_count:Maximum retrying frequency.
I/O:When the I/O error occurs, the following variable is updated. .
        (1) hscd->error_count
        (2) hscd->retry_count
@retval:(1) 0:Normal termination.
         (2) -EINVAL:n_regs is too large.
         (3) -EBUSY:I2C Communication fault (time out).
--------------------------------------------------------------------*/
static int hscd_write_regs(uint8_t start_reg,
						   const uint8_t reg_data[], uint16_t n_regs)
{
	// buffer[]:Array that adds slave addr(W) 
	//			 and start_reg in front of reg_data[]. 
	uint8_t buffer[HSCD_MAX_CONTIGUOUS_REGS + 2];
	uint8_t *dest;
	unsigned i;
	int ret;
	unsigned retry_count = 0;

	debug("%s(0x%02X %p %u)\n", __func__, start_reg, reg_data, n_regs);

	if (n_regs > HSCD_MAX_CONTIGUOUS_REGS) {
		ret = -EINVAL;
		debug("Too many registers (> %u).\n", HSCD_MAX_CONTIGUOUS_REGS);
		goto Error;
	}

	dest = buffer;
	*dest++ = start_reg;
	for (i = 0; i < n_regs; i++)
		*dest++ = reg_data[i];

	for (;;) {
		mutex_lock(&xfer_lock);
		ret = i2c_master_send(hscd->reg.st_client, buffer, n_regs + 1);
		if (ret >= 0) {
			mutex_unlock(&xfer_lock);
			break;				// Normal termination
		}
		mutex_unlock(&xfer_lock);
		// When the error occurs
		if (retry_count >= hscd->max_retry_count) {
			// When ..retrying.. frequency reaches the upper bound:error
			compass_printk(KERN_ERR, "%s: I2C send error (%d)\n", 
													HSCD_DEVNAME, ret);
			//ret = -EIO;
			goto Error;
		}
		// It retries. 
		retry_count++;
	}

	if (retry_count > 0) {
Error:
		hscd->error_count++;
		hscd->retry_count += retry_count;
	}
	debug("%s() -> %d\n", __func__, ret);
	return ret;
}

/*--------------------------------------------------------------------
Function  :HSCD of WRITE It writes it in the register.
        Whether it was possible to write it correctly by reading afterwards,
        returning, and collating it is confirmed. 
        When it is not possible to write it, it retries. 
@param  :(1) start_reg:The first register number.
          (2) reg_data[0 - n_regs-1]:Data written in register.
          (3) n_regs (<=HSCD_MAX_CONTIGUOUS_REGS):Number of registers.
          (4) verify_mask (0x00):Mask that specifies collated bit about 
                                    reg_data[] each byte. 
              It is checked whether the bit of one in this is corresponding. 
        (5) hscd->max_retry_count:Maximum retrying frequency.
I/O:When the I/O error occurs, the following variable is updated. 
        (1) hscd->error_count
        (2) hscd->retry_count
@retval:(1) 0:Normal termination.
         (2) -EINVAL:n_regs is too large.
         (3) -EBUSY:I2C Communication fault (time out).
         (4) -EIO:It doesn't succeed in the writing collation even 
                   if ..retrying.. frequency reaches the upper bound. 
--------------------------------------------------------------------*/
static int hscd_write_verify_regs(uint8_t start_reg, const uint8_t reg_data[],
								  uint16_t n_regs, uint8_t verify_mask)
{
	uint8_t buffer[HSCD_MAX_CONTIGUOUS_REGS];
	unsigned retry_count = 0;
	unsigned i;
	int ret;

	debug("%s(0x%02X %p %u 0x%02X)\n",
		__func__, start_reg, reg_data, n_regs, verify_mask);

	if ((verify_mask & 0xFF) == 0) {
		ret = -EINVAL;
		goto Error;
	}

	for (;;) {
		// reg_data[] writes in the register.
		ret = hscd_write_regs(start_reg, reg_data, n_regs);
		if (ret < 0)
			goto Error;

		// The register is read buffer[].
		if ((ret = hscd_read_regs(start_reg, buffer, n_regs)) < 0)
			goto Error;

		// collates buffer[0 - n_regs-1] and reg_data[0 - n_regs-1].
		// (Only the bit specified verify_mask.)
		for (i = 0;; i++) {
			if (i >= n_regs)
				goto Exit;		// Colation success
			if ((buffer[i]^reg_data[i])&verify_mask)
				break;			// disagreement
		}
		// Collation failure
		if (retry_count >= hscd->max_retry_count) {
			compass_printk(KERN_ERR, "%s: Reg write-verify error.\n", 
															HSCD_DEVNAME);
			ret = -EIO;
			goto Error;
		}
		retry_count++;
	}

Exit:
	if (retry_count > 0) {
Error:
		hscd->error_count++;
		hscd->retry_count += retry_count;
	}
	return ret;
}

/*--------------------------------------------------------------------
Function  :The operational mode of HSCD is set.
@param    :mode:HSCD_MODE_{STANBY,FORCESTATE}.
Attention :When it is active.Begin the temperature survey at the same time. 
@retval   :Success:0,Error: -errno.
error     :(1) EINVAL:mode is illegal.
            (2) other :hscd_write_verify_regs() 
--------------------------------------------------------------------*/
static int hscd_setmode(uint8_t mode)
{
	int ret;
	uint8_t reg_data;

	if (mode == HSCD_MODE_STANBY_V2 || mode == HSCD_MODE_STANBY_V3) {
		// changes to an active mode
		ret = hscd_write_verify_regs(HSCD_CNTL1, &mode, 1, 0x80);
		if (ret < 0)
			goto Exit;
		//CNTL3 clear
		reg_data = HSCD_CNTL3_STANBY;
		ret = hscd_write_regs(HSCD_CNTL3, &reg_data, 1);
		if (ret < 0)
			goto Exit;
		//CNTL2 clear
		if(system_rev >= 0x0c || system_rev == 0x0a) {
			reg_data = HSCD_CNTL2_CLEAR_V2;
		} else {
			reg_data = HSCD_CNTL2_CLEAR_V3;
		}
		ret = hscd_write_verify_regs(HSCD_CNTL2, &reg_data, 1, 0xFF);
		if (ret < 0)
			goto Exit;
	} else if (mode == HSCD_MODE_FORCESTATE_V2 
				||mode == HSCD_MODE_FORCESTATE_V3) {
		// changes to an active mode
		if (system_rev >= 0x0c || system_rev == 0x0a) {
			mode = HSCD_MODE_FORCESTATE_V2;
		} else {
			if (hscd->measurement.measurement_range == HSCD_MEASUREMENT_RANGE_12) {
				mode = HSCD_FORCESTATE_12BIT;
			} else if (hscd->measurement.measurement_range 
							== HSCD_MEASUREMENT_RANGE_13) {
				mode = HSCD_FORCESTATE_13BIT;
			} else {
				//error
				ret = -EINVAL;
				goto Exit;
			}
		}
		
		ret = hscd_write_verify_regs(HSCD_CNTL1, &mode, 1, 0x80);
		if (ret < 0)
			goto Exit;
		
		// CNTL2 is DRDY ON
		if(system_rev >= 0x0c || system_rev == 0x0a) {
			reg_data = HSCD_CNTL2_DRDYON_V2;
		} else {
			reg_data = HSCD_CNTL2_DRDYON_V3;
		}
		ret = hscd_write_verify_regs(HSCD_CNTL2, &reg_data, 1, 0x08);
		if (ret < 0)
			goto Exit;

		reg_data = HSCD_CNTL3_FORCE;
		ret = hscd_write_regs(HSCD_CNTL3, &reg_data, 1);
		if (ret < 0)
			goto Exit;
	} else {
		//error
		ret = -EINVAL;
		goto Exit;
	}

	HSCD_SETMODE(mode);

Exit:
	debug("%s(0x%02X) -> %d\n", __func__, mode, ret);

	return ret;
}

/*--------------------------------------------------------------------
Function  :HSCD is changed to powerdown mode. 
--------------------------------------------------------------------*/
#define hscd_stanby()      \
	if (system_rev >= 0x0c || system_rev == 0x0a) { \
		hscd_setmode(HSCD_MODE_STANBY_V2); \
	} else { \
		hscd_setmode(HSCD_MODE_STANBY_V3); \
	}

/*--------------------------------------------------------------------
Function  :register of hscd is read. 
output    :hscd->reg:content of shcd
@retval   :Success:0,Error: -errno.
error     :shcd_{read,write}_verify_regs() 
--------------------------------------------------------------------*/
static int hscd_read_reg_all(void)
{
	int ret;
	uint8_t buffer[HSCD_MAX_CONTIGUOUS_REGS];

	debug("%s start\n", __func__);

	// read TEMP
	ret = hscd_read_verify_regs(HSCD_TEMP, &hscd->reg.temp, 1);
	if (ret < 0)
		goto Exit;

	// read DATA[XYZ]
	ret = hscd_read_verify_regs(HSCD_XLSB, buffer, 6);
	if (ret < 0)
		goto Exit;
	// MSB/LSB of buffer is replaced
	hscd->reg.data.x = buffer[0] + (buffer[1] << 8);
	hscd->reg.data.y = buffer[2] + (buffer[3] << 8);
	hscd->reg.data.z = buffer[4] + (buffer[5] << 8);

	// read STAT
	ret = hscd_read_verify_regs(HSCD_STAT, &hscd->reg.stat, 1);
	if (ret < 0)
		goto Exit;
	// read CNTL1
	ret = hscd_read_verify_regs(HSCD_CNTL1, &hscd->reg.cntl1, 1);
	if (ret < 0)
		goto Exit;
	// read CNTL2
	ret = hscd_read_verify_regs(HSCD_CNTL2, &hscd->reg.cntl2, 1);
	if (ret < 0)
		goto Exit;
	// read CNTL3
	ret = hscd_read_verify_regs(HSCD_CNTL3, &hscd->reg.cntl3, 1);
	if (ret < 0)
		goto Exit;
	// read CNTL4
	if(system_rev >= 0x0c || system_rev == 0x0a) {
		
		ret = hscd_read_verify_regs(HSCD_CNTL4_V2, &hscd->reg.cntl4, 1);
		if (ret < 0)
			goto Exit;
	} else {
		ret = hscd_read_verify_regs(HSCD_CNTL4_V3, &hscd->reg.cntl4, 1);
		if (ret < 0)
			goto Exit;
	}
	// read PRET
	ret = hscd_read_verify_regs(HSCD_PRET, &hscd->reg.pret, 1);
	if (ret < 0)
		goto Exit;
	// read AMP
	if(system_rev >= 0x0c || system_rev == 0x0a) {
		ret = hscd_read_verify_regs(HSCD_AMP, &hscd->reg.amp, 1);
		if (ret < 0)
			goto Exit;
	}

	// read OFF[XYZ]
	ret = hscd_read_verify_regs(HSCD_XOFFLSB, buffer, 6);
	if (ret < 0)
		goto Exit;
	// MSB/LSB of buffer is replaced
	hscd->reg.off.x = buffer[0] + (buffer[1] << 8);
	hscd->reg.off.y = buffer[2] + (buffer[3] << 8);
	hscd->reg.off.z = buffer[4] + (buffer[5] << 8);

Exit:
	debug("%s -> %d\n", __func__, ret);

	return ret;
}

/*--------------------------------------------------------------------
Function  :Offset value of HSCD writes in the OFF[XYZ] register. 
@param    :*off:Offset value.
output    :hscd->reg.off:If it is a success, *off is written.
@retval   :Success:0,Error: -errno.
error     :hscd_write_verify_regs()
--------------------------------------------------------------------*/
static int hscd_set_sensor_offset(const hscd_xyz_t * off)
{
	int ret;
	uint8_t buffer[HSCD_MAX_CONTIGUOUS_REGS];

	// MSB/LSB of xyz is replaced
	buffer[0] = (uint8_t) (off->x & 0x00FF);
	buffer[1] = (uint8_t) ((off->x >> 8) & 0x000F);
	buffer[2] = (uint8_t) (off->y & 0x00FF);
	buffer[3] = (uint8_t) ((off->y >> 8) & 0x000F);
	buffer[4] = (uint8_t) (off->z & 0x00FF);
	buffer[5] = (uint8_t) ((off->z >> 8) & 0x000F);

	ret = hscd_write_verify_regs(HSCD_XOFFLSB, buffer, sizeof(*off), 0xFF);
	if (ret == OK)
		hscd->reg.off = *off;

	debug("%s(X:%u Y:%u Z:%u) -> %u %d\n",
		__func__, off->x, off->y, off->z, *buffer, ret);

	return ret;
}

/*--------------------------------------------------------------------
Function  :Amplifier gain value of HSCD writes in the AMP register.
@param    :*amp:Amplifier gain value
outpue    :hscd->reg.amp:If it is a success, *amp is written.
@retval   :Success:0,Error: -errno.
error     :hscd_write_verify_regs() 
--------------------------------------------------------------------*/
static int hscd_set_sensor_ampgain(const uint8_t *amp)
{
	int ret;

	if(system_rev >= 0x0c || system_rev == 0x0a) {
		ret = hscd_write_verify_regs(HSCD_AMP, amp, 1, 0xC0);
	} else {
		ret = hscd_write_verify_regs(HSCD_CNTL4_V3, amp, 1, 0xC0);
	}
	if (ret == OK)
		hscd->reg.amp = *amp;

	debug("%s(amp:%u) -> %d\n", __func__, *amp, ret);

	return ret;
}


/*================== Initialization/Termination ====================*/

/*--------------------------------------------------------------------
Function  :Port that HSCD uses is initialized. 
--------------------------------------------------------------------*/
static void hscd_port_init(void)
{
	debug("%s\n", __func__);

	gpio_request(JIKI_DRDY_GPIO, "compass");
	gpio_direction_input(JIKI_DRDY_GPIO);
}

/*--------------------------------------------------------------------
Function  :Reset of the software of HSCD is executed. 
@param    :keep:true ,no change. 
                  false,Reset of the software is executed. 
@retval   :Success:2,Error: -errno.
error     : I/O error
--------------------------------------------------------------------*/
static int hscd_chip_reset(bool keep)
{
	uint8_t regs[READREGS_MINBUFSIZE];
	bool error = false;
	unsigned retry_count = 0;
	int ret = OK;
	uint8_t cntl3 = HSCD_CNTL3_SRST;

	debug("%s start\n", __func__);

	// CNTL3:SRST bit=1
	ret = hscd_write_regs(HSCD_CNTL3, &cntl3, 1);
	if (ret < 0) {
		compass_printk(KERN_ERR, "Error: SoftReset  write cntl3 cntl3=%x.\n",
																		cntl3);
		goto Exit;
	}

	udelay(HSCD_Trnw);			// Reset waiting

	for (;;) {
		// if register of CNTL3 is zero,Reset complete
		if ((ret = hscd_read_regs(HSCD_CNTL3, regs, 1)) >= 0) {
			if (0x00 == regs[0]) {
				break;			// Success
			} else {
				compass_printk(KERN_ERR, 
					"SoftReset  read cntl3 NG cntl3=%x.\n", regs[0]);
			}

		}
		// At the error
		error = true;
		if (retry_count >= hscd->max_retry_count) {
			// when retries reach upper limit:error
			compass_printk(KERN_ERR,
						   "Error: Unable to clear SoftReset.\n");
			//ret = -EIO;
			break;
		}
		// retries
		retry_count++;
	}

Exit:
	if (error) {
		// error/retries total.
		hscd->error_count++;
		hscd->retry_count += retry_count;
	}

	debug("%s() -> %d\n", __func__, ret);

	return ret;
}

/*--------------------------------------------------------------------
Function  :Execution of offset calibration of HSCD
@param    :None
@retval   :Success:2,Error: -errno.
error     :(1)  I/O error
            (2) other:hscd_write_verify_regs()
--------------------------------------------------------------------*/
static int hscd_offsetcalib(void)
{
	uint8_t regs[READREGS_MINBUFSIZE];
	bool error = false;
	unsigned retry_count = 0;
	int ret;
	uint8_t cntl3 = HSCD_CNTL3_OCL;
	uint8_t mode;

	debug("%s start\n", __func__);

	if(system_rev >= 0x0c || system_rev == 0x0a) {
		mode = HSCD_MODE_FORCESTATE_V2;
	} else {
		mode = HSCD_MODE_FORCESTATE_V3;
	}
	
	// changes to an active mode.
	ret = hscd_write_verify_regs(HSCD_CNTL1, &mode, 1, 0x80);
	if (ret < 0)
		goto Exit;

	// The offset calibration is set. 
	ret = hscd_write_regs(HSCD_CNTL3, &cntl3, 1);
	if (ret < 0)
		goto Exit;
	mdelay(HSCD_Twat);			// Measurement waiting time

	// Could be measured ?
	for (;;) {
		// if register of CNTL3 is zero,Reset complete
		if ((ret = hscd_read_regs(HSCD_CNTL3, regs, 1)) >= 0) {
			if (0x00 == (regs[0] & 0x01)) {
				// Success
				debug("%s OFFSET CALIBRATION SUCCESS\n", __func__);
				break;
			} else {
				compass_printk(KERN_ERR,
					"OFFSET CALIBRATION NG cntl3=%x.\n", regs[0]);
			}
		}
		// At the error
		error = true;
		if (retry_count >= hscd->max_retry_count) {
			// when retries reach upper limit:error
			compass_printk(KERN_ERR, "Error: Unable to OFFSET CALIBRATION.\n");
			//ret = -EIO;
			break;
		}
		// retries
		retry_count++;
	}

Exit:
	if (error) {
		// error/retries total.
		hscd->error_count++;
		hscd->retry_count += retry_count;
	}

	debug("%s  -> %d\n", __func__, ret);

	return ret;
}



/*--------------------------------------------------------------------
Function  :Offset and amplifier gain value of HSCD 
            are reset in the following value. 
           Offset         :(0x0000, 0x0000, 0x0000)
           Amplifier gain :(EHXGA,EHYGA,EHZGA)
@retval   :Success:0,Error: -errno.
error     :hscd_write_verify_regs() 
--------------------------------------------------------------------*/
static int hscd_adjustment_reset(void)
{
	hscd_xyz_t off;
	uint8_t amp;
	int ret;

	off.x = off.y = off.z = 0x0000;	// Offset:0
	amp = 0x80;						// Gain  :1.0(default)
	if ((ret = hscd_set_sensor_offset(&off)) > 0) {
		ret = hscd_set_sensor_ampgain(&amp);
	}

	return ret;
}

/*--------------------------------------------------------------------
Function  :HSCD reset and initialize the driver.
@param    :first:Initialization : true
                   Reinitialize   : false
--------------------------------------------------------------------*/
static void hscd_driver_init(bool first)
{
	if (first) {
		mutex_init(&xfer_lock);
		init_waitqueue_head(&hscd->measure_end_queue);
		hscd_port_init();	// GPIO97 Initialization
	}

	// Initialize hardware.
	hscd_chip_reset(false);		// HSCD Reset

	// Set the default collation and retry
	hscd->max_retry_count = 10;

	// *hscd Reset.
	HSCD_SETSTATE(HSCD_STAT_IDLE);	// Idle state

	// past measuring data is invalidated. 
	hscd->reg.temp = 0;
	hscd->reg.data.x = 0;
	hscd->reg.data.y = 0;
	hscd->reg.data.z = 0;

	hscd_adjustment_reset();	// Initialize the settings.

	// read register
	hscd_read_reg_all();
}

/*--------------------------------------------------------------------
Function  :Driver's termination. 
--------------------------------------------------------------------*/
#define hscd_driver_term()    hscd_driver_init(false)

/*--------------------------------------------------------------------
Function  :Module initialization.
--------------------------------------------------------------------*/
void hscd_module_init(void)
{
	hscd_driver_init(true);
}

/*--------------------------------------------------------------------
Function  :Module termination.
--------------------------------------------------------------------*/
void hscd_module_term(void)
{
	hscd_driver_term();
	gpio_free(JIKI_DRDY_GPIO);
}


/*====================== Interrupt Processing=======================*/

/*--------------------------------------------------------------------
Function  :interrupt handler of HSCD.
@param    :(1) irq:IRQ Number.
            (2) data:request_irq() ,Device-specific data (not used).
output    :hscd->measurement
            (1) time:End time measurement
@retval   :HSCD Interrupt: IRQ_HANDLED,other: IRQ_NONE.
--------------------------------------------------------------------*/
static irqreturn_t hscd_isr(int irq, void *data)
{
	compass_data_t *const m = &hscd->measurement;

	debug("%s(%d, %p)\n", __func__, irq, data);

	// If GPIO97 has not been generated, disregards. 
	if (unlikely(!GPIO_ISACTIVE()))
		return IRQ_NONE;

	HSCD_SETSTATE(HSCD_STAT_MEASURED);	// to measurement completion state.

	// Recording Time.
	do_gettimeofday(&m->time);

#ifdef	CONFIG_BRUTUS
	//get case state
	case_getstatus(&m->case_open);
	hscd->reg.case_open = m->case_open;
#endif	// CONFIG_BRUTUS

	// The task is restarted. 
	wake_up_all(&hscd->measure_end_queue);

	debug("%s end\n", __func__);

	return IRQ_HANDLED;
}

/*--------------------------------------------------------------------
Function  :Clear interrupt flag HSCD.
output    :hscd->reg.data
@retval   :Success:OK,Error: -errno.
error     :(1) error number:I/O error.
            (2) other:hscd_read_regs()
--------------------------------------------------------------------*/
static int hscd_clear_interrupt(void)
{
	bool error = false;
	unsigned retry_count = 0;
	int ret = OK;
	unsigned nRegs;
	uint8_t buffer[HSCD_MAX_CONTIGUOUS_REGS];

	debug("%s start\n", __func__);

	if (likely(GPIO_ISACTIVE())) {

		debug("%s gpio active\n", __func__);

		for (;;) {
			// When the GPIO97 signal is High:
			// Clear interrupt.
			nRegs = sizeof(hscd->reg.data);
			if ((ret = hscd_read_regs(HSCD_XLSB, buffer, nRegs)) >= 0) {

				hscd_read_verify_regs(HSCD_STAT, &hscd->reg.stat, 1);
				debug("stat=%x.\n", hscd->reg.stat);

				// check GPIO97 is Low .
				if (likely(!GPIO_ISACTIVE()))
					break;		// Success
			}
			// At the error
			error = true;
			if (retry_count >= hscd->max_retry_count) {
				// when retries reach upper limit:error
				compass_printk(KERN_ERR, "Error: Unable to clear GPIO97.\n");
				break;
			}
			// retries
			retry_count++;
		}
	}

	if (error) {
		// error/retries total.
		hscd->error_count++;
		hscd->retry_count += retry_count;
	}

	debug("%s() -> %d\n", __func__, ret);

	return ret;
}


/*===================== Magnetic measurements ======================*/

/*--------------------------------------------------------------------
Function  :start the measurement of magnetic.
@retval   :(1) 0:Success.
            (2) -EBUSY:I2C Communication error (time out).
            (3) -EIO  :I/O error.
--------------------------------------------------------------------*/
static int hscd_measure_start(void)
{
	int ret;

	debug("%s\n", __func__);

	// Clear interrupt HSCD.
	if ((ret = hscd_clear_interrupt()) >= 0) {
		// Measurement start.
		HSCD_SETSTATE(HSCD_STAT_MEASURING);
		if(system_rev >= 0x0c || system_rev == 0x0a) {
			ret = hscd_setmode(HSCD_MODE_FORCESTATE_V2);
		} else {
			ret = hscd_setmode(HSCD_MODE_FORCESTATE_V3);
		}
	}
	return ret;
}

/*--------------------------------------------------------------------
Function  :measuring data of terrestrial magnetism read.
output    :hscd->measurement:Result of a measurement.
@retval   :hscd_read_verify_regs() 
--------------------------------------------------------------------*/
static int hscd_measure_end(void)
{
	int ret;
	unsigned nRegs;
	uint8_t buffer[HSCD_MAX_CONTIGUOUS_REGS];
	compass_data_t *const m = &hscd->measurement;

	debug("%s\n", __func__);

	nRegs = sizeof(hscd->reg.data);
	ret = hscd_read_verify_regs(HSCD_XLSB, buffer, nRegs);
	if (ret >= 0) {
		// MSB/LSB of buffer is replaced
		hscd->reg.data.x = buffer[0] + (buffer[1] << 8);
		hscd->reg.data.y = buffer[2] + (buffer[3] << 8);
		hscd->reg.data.z = buffer[4] + (buffer[5] << 8);

		debug("%s():DATA:(0x%04X 0x%04X 0x%04X)\n",
						__func__, hscd->reg.data.x,
						hscd->reg.data.y, hscd->reg.data.z);

		// The measuring data is copied onto hscd->measurement. 
		m->magnetism[0] = hscd->reg.data.x;	// DATAX
		m->magnetism[1] = hscd->reg.data.y;	// DATAY
		m->magnetism[2] = hscd->reg.data.z;	// DATAZ

		// Measurement data acquisition completion.
		HSCD_SETSTATE(HSCD_STAT_AVAILABLE);
	}

	return ret;
}

/*--------------------------------------------------------------------
Function  :start the measurement of magnetic,and Getting Results.
output    :result of a measurement, 
			and opening and shutting and state of the turn of the case.
	        *measurement
	        hscd->measurement
	        hscd->reg.temp
	        hscd->reg.data
@retval   :(1) 0:success
            (2) -EBUSY:I2C Communication error (time out).
            (3) -EIO:I/O error retries reach upper limit.
            (4) EINTR:Received a signal while waiting for measurement.
            (5) EFAULT:measurement adress Injustice.
--------------------------------------------------------------------*/
static int hscd_ioctl_measure(compass_data_t __user * measurement)
{
	int ret;
	int measurement_count = 0;

	debug("%s(%p) start\n", __func__, measurement);

	if ((ret = down_interruptible(&hscd_sem)) >= 0) {
		// Exclusive control start
		
		ret = copy_from_user(&hscd->measurement.measurement_range,
						&measurement->measurement_range,
						sizeof(measurement->measurement_range));
		if (ret != 0) {
			debug("Can't copy to user area.%u\n", ret);
			ret = -EFAULT;
			goto Start_err;
		}
		
		debug("HSCD_STATE0:%d\n", HSCD_STATE());
		if ((ret = hscd_measure_start()) < 0)
			goto Start_err;

		// Measurement start success
  		enable_irq(HSCD_IRQ());		// GPIO97 Permission
		
		// Waiting for  measurement complete
		debug("loop in\n");

		for (;;) {
			const long timeout = (HSCD_MAX_MEASUREMENT_TIME * HZ / 1000000) + 1;

			//STAT Reading confirmation
			debug("HSCD_STATE:%d\n", HSCD_STATE());

			//During the measurement:wait
			ret = wait_event_interruptible_timeout(hscd->measure_end_queue,
				(HSCD_STATE() == HSCD_STAT_MEASURED), timeout);
			if (ret > 0)
				break;			// Measurements completed
			if (ret < 0)
				goto Exit;
			// time out:condition is revalued. 
			debug("HSCD_STATE:%d \n", HSCD_STATE());
			debug("%s: wait_event_interruptible_timeout ret=%d\n",
													__func__, ret);
			debug("%s: timeout(%ld)\n", __func__, timeout);
			
			measurement_count++;
			if (measurement_count > HSCD_MAX_MEASUREMENT_COUNT) {
				compass_printk(KERN_ERR, 
					"%s: measurement wait event error.\n", HSCD_DEVNAME);
				ret = -EBUSY;
				goto Exit;
			}
		}

		debug("loop out\n");

		if ((ret = hscd_measure_end()) < 0)
			goto Exit;

		HSCD_SETSTATE(HSCD_STAT_IDLE);

		// Return the number of retries and error
		hscd_update_error_count();
		hscd->measurement.total_error_count = hscd->total_error_count;
		hscd->measurement.total_retry_count = hscd->total_retry_count;
		if (copy_to_user(measurement, &hscd->measurement, 
			sizeof(*measurement)) != 0) {
			debug("Can't copy to user area.%u\n", ret);
			ret = -EFAULT;
		}

Exit:
  		disable_irq(HSCD_IRQ());
		
Start_err:
		hscd_update_error_count();
		up(&hscd_sem);			// Exclusive control end
	}

	debug("%s ->%d\n", __func__, ret);

	return ret;
}


/*========================= File Operations ========================*/

/*--------------------------------------------------------------------
Function  :HSCD device is opened.  
@param    :(1) inode:inode Information of Device file (Unused).
            (2) file:open file Structure.
@retval   :(1) 0:success.
            (2) -EACCES:mode is (O_RDONLY) except.
            (3) -EINTR:Interrupt occurs during execution.
            (4) other : request_irq() 
--------------------------------------------------------------------*/
int hscd_open(struct inode *inode, struct file *file)
{
	int ret = OK;
	uint8_t mode;

	debug("%s start\n", __func__);
	
	if(system_rev >= 0x0c || system_rev == 0x0a) {
		mode = HSCD_MODE_FORCESTATE_V2;
	} else {
		mode = HSCD_MODE_FORCESTATE_V3;
	}

	// mode is (O_RDONLY) excep EACCES.
	if (file->f_mode & FMODE_WRITE) {
		ret = -EACCES;
		debug("%s: Write operation not allowed.\n", HSCD_DEVNAME);
		goto Exit1;
	}

	if ((ret = down_interruptible(&hscd_sem)) >= 0) {
		// Exclusive control start
		if (hscd->use_count == 0) {
			// When not in use:Initialization
			hscd_driver_init(false);

			// Handler registration
			ret = request_irq(HSCD_IRQ(), hscd_isr, IRQ_TYPE_EDGE_RISING,
							HSCD_DEVNAME, &hscd);
			if (unlikely(ret < 0)) {
				debug("%s: Can't use IRQ%u.\n", HSCD_DEVNAME, HSCD_IRQ());
				goto Exit2;
			}
  			disable_irq(HSCD_IRQ());
		}
		hscd->use_count++;

		// changes to an active mode
		ret = hscd_write_verify_regs(HSCD_CNTL1, &mode, 1, 0x80);
		if (ret < 0) {
			goto Exit2;
		} else {
			ret = OK;
		}

Exit2:
		up(&hscd_sem);		// Exclusive control end
	}

Exit1:
	debug("%s() -> %d\n", __func__, ret);
	return ret;
}

/*--------------------------------------------------------------------
Function  :HSCD device is close
@param    :(1) inode:inode Information of Device file (Unused).
            (2) file:close file Structure.
@retval   :(1) 0:success.
            (2) -EINTR:Interrupt occurs during execution.
--------------------------------------------------------------------*/
int hscd_release(struct inode *inode, struct file *file)
{
	int ret;

	debug("%s\n", __func__);

	if ((ret = down_interruptible(&hscd_sem)) >= 0) {
		//  Exclusive control start
		if (--hscd->use_count == 0) {
			// When becoming unused
			// GPIO97 Interruption prohibition,IRQ Liberating.
  			disable_irq(HSCD_IRQ());
			free_irq(HSCD_IRQ(), &hscd);

			// HSCD Reset
			hscd_driver_term();
		}
		
		if(hscd->use_count < 0) {
			hscd->use_count = 0;
		}
		up(&hscd_sem);			//  Exclusive control end
	}
	return ret;
}


/*=========================== IOCTL ===========================*/

/*--------------------------------------------------------------------
Function  :Data written in the register of HSCD.
@param    :member of *regs.
           mask:Select a group mask register to read.
             The combination of the following flags.
           HSCD_REGMASK_OFFSET:Sensor offset (OFF[XYZ])
           HSCD_REGMASK_AMPGAIN:Sensor amplifier gain (AMP)
           off (mask.HSCD_REGMASK_OFFSET = ON )
           amp (mask.HSCD_REGMASK_AMPGAIN = ON )
@retval   :(1) 0:success.
            (2) -EBUSY:I2C Communication fault (time out).
            (3) -EIO:I/O error retries reach upper limit.
            (4) EINTR:Received a signal while waiting for measurement.
            (5) EFAULT:regs adress Injustice.
--------------------------------------------------------------------*/
static int hscd_ioctl_setregs(const hscd_registers_t __user * regs)
{
	hscd_registers_t src;
	int ret;

	debug("%s\n", __func__);

	if ((ret = down_interruptible(&hscd_sem)) >= 0) {
		// Exclusive control start

		// src <- *regs
		if (copy_from_user(&src, regs, sizeof(src)) != 0) {
			debug("%s Can't copy from user area.\n", __func__);
			ret = -EFAULT;
			goto Exit;
		}
		if (src.mask & HSCD_REGMASK_OFFSET) {
			// If the specified offset
			if ((ret = hscd_set_sensor_offset(&src.off)) < 0)
				goto Exit;
			hscd->reg.mask |= HSCD_REGMASK_OFFSET;
		}
		if (src.mask & HSCD_REGMASK_AMPGAIN) {
			// If the amp is given
			if ((ret = hscd_set_sensor_ampgain(&src.amp)) < 0)
				goto Exit;
			hscd->reg.mask |= HSCD_REGMASK_AMPGAIN;
		}

		ret = 0;
Exit:
		hscd_update_error_count();
		up(&hscd_sem);			// Exclusive control end
	}
	return ret;
}

/*--------------------------------------------------------------------
Function  :Get HSCD register contents and status of the case.
@param    :regs->mask:Select a group mask register to read.
	        The combination of the following flags.
	        HSCD_REGMASK_DATA:Previous measurements (TEMP,DATA[XYZ])
	        HSCD_REGMASK_OFFSET:Sensor offset (OFF[XYZ])
	        HSCD_REGMASK_AMPGAIN:Sensor amplifier gain (AMP)
output    :member of *regs.
	        temp,data:TEMP,The contents of the register of DATA[XYZ].
	          (mask.HSCD_REGMASK_DATA = ON)
	        off:The contents of the register of OFF[XYZ].
	          (mask.HSCD_REGMASK_OFFSET = ON)
	        amp:The contents of the register of AMP.
	          (mask.HSCD_REGMASK_AMPGAIN = ON)
@retval   :(1) 0:success.
	        (2) -EBUSY:I2C Communication fault (time out).
	        (3) -EIO:I/O error retries reach upper limit.
	        (4) EINTR:Received a signal while waiting for Exclusive control.
	        (5) EFAULT:regs adress Injustice.
--------------------------------------------------------------------*/
static int hscd_ioctl_getregs(hscd_registers_t __user * regs)
{
	uint8_t mask, buffer[HSCD_MAX_CONTIGUOUS_REGS];
	unsigned nRegs;
	int ret;
	debug("%s\n", __func__);

	if ((ret = down_interruptible(&hscd_sem)) >= 0) {
		// Exclusive control start

		// mask <- regs->mask
		if (copy_from_user(&mask, &regs->mask, sizeof(mask)) != 0) {
			debug("%s Can't copy from user area.\n", __func__);
			goto CopyFailed;
		}

		if (mask & HSCD_REGMASK_DATA) {
			// Register read range
			nRegs = sizeof(regs->temp);
			ret = hscd_read_verify_regs(HSCD_TEMP, buffer, nRegs);
			if (ret < 0)
				goto Exit;
			// To store the contents read.
			memcpy(&hscd->reg.temp, buffer, nRegs);
			if (copy_to_user(&regs->temp, buffer, nRegs) != 0)
				goto CopyFailed;

			nRegs = sizeof(regs->data);
			ret = hscd_read_verify_regs(HSCD_XLSB, buffer, nRegs);
			if (ret < 0)
				goto Exit;
			// buffer in MSB / LSB swap the set xyz
			hscd->reg.data.x = buffer[0] + (buffer[1] << 8);
			hscd->reg.data.y = buffer[2] + (buffer[3] << 8);
			hscd->reg.data.z = buffer[4] + (buffer[5] << 8);
			if (copy_to_user(&regs->data, &hscd->reg.data, nRegs) != 0)
				goto CopyFailed;
			// set flag of hscd->reg.mask
			hscd->reg.mask |= HSCD_REGMASK_DATA;
		}

		if (mask & HSCD_REGMASK_OFFSET) {
			// Register read range.
			nRegs = sizeof(regs->off);
			ret = hscd_read_verify_regs(HSCD_XOFFLSB, buffer, nRegs);
			if (ret < 0)
				goto Exit;
			// buffer in MSB / LSB swap the set xyz
			hscd->reg.off.x = buffer[0] + (buffer[1] << 8);
			hscd->reg.off.y = buffer[2] + (buffer[3] << 8);
			hscd->reg.off.z = buffer[4] + (buffer[5] << 8);
			memcpy(&hscd->reg.off, buffer, nRegs);
			if (copy_to_user(&regs->off, &hscd->reg.off, nRegs) != 0)
				goto CopyFailed;
			// set flag of hscd->reg.mask
			hscd->reg.mask |= HSCD_REGMASK_OFFSET;
		}

		if (mask & HSCD_REGMASK_AMPGAIN) {
			// Register read range
			nRegs = sizeof(regs->amp);
			if(system_rev >= 0x0c || system_rev == 0x0a) {
				ret = hscd_read_verify_regs(HSCD_AMP, buffer, nRegs);
			} else {
				ret = hscd_read_verify_regs(HSCD_CNTL4_V3, buffer, nRegs);
			}
			if (ret < 0)
				goto Exit;
			memcpy(&hscd->reg.amp, buffer, nRegs);
			// To store the contents read
			if (copy_to_user(&regs->amp, buffer, nRegs) != 0)
				goto CopyFailed;
			// set flag of hscd->reg.mask
			hscd->reg.mask |= HSCD_REGMASK_AMPGAIN;
		}
#ifdef	CONFIG_BRUTUS
		// Return the status of the case opening.
		case_getstatus(&hscd->reg.case_open);
		if (copy_to_user(&regs->case_open, &hscd->reg.case_open, 1) != 0) {
			goto CopyFailed;
		}
#endif	// CONFIG_BRUTUS

		ret = 0;
		goto Exit;

CopyFailed:
		debug("%s Can't copy user area.\n", __func__);
		ret = -EFAULT;

Exit:
		hscd_update_error_count();
		up(&hscd_sem);			// Exclusive control end
	}
	return ret;
}

/*--------------------------------------------------------------------
Function  :offset calibration of HSCD is executed. 
output    :member of *regs.
            off:The contents of the register of OFF[XYZ].
@retval   :(1) 0:success.
	        (2) -EBUSY:I2C Communication fault (time out).
	        (3) -EIO:I/O error retries reach upper limit.
	        (4) EINTR:Received a signal while waiting for Exclusive control.
	        (5) EFAULT:regs adress Injustice.
--------------------------------------------------------------------*/
static int hscd_ioctl_offcal(hscd_registers_t __user * regs)
{
	uint8_t buffer[HSCD_MAX_CONTIGUOUS_REGS];
	unsigned nRegs;
	int ret;

	debug("%s\n", __func__);

	if ((ret = down_interruptible(&hscd_sem)) >= 0) {
		// Exclusive control start

		// ffset calibration is executed
		if ((ret = hscd_offsetcalib()) > 0) {
			// success
			// Register read range.
			nRegs = sizeof(regs->off);
			ret = hscd_read_verify_regs(HSCD_XOFFLSB, buffer, nRegs);
			if (ret < 0)
				goto Exit;
			// buffer in MSB / LSB swap the set xyz
			hscd->reg.off.x = buffer[0] + (buffer[1] << 8);
			hscd->reg.off.y = buffer[2] + (buffer[3] << 8);
			hscd->reg.off.z = buffer[4] + (buffer[5] << 8);
			memcpy(&hscd->reg.off, buffer, nRegs);
			if (copy_to_user(&regs->off, &hscd->reg.off, nRegs) != 0)
				goto CopyFailed;
		}

		ret = 0;
		goto Exit;

CopyFailed:
		debug("%s Can't copy to user area.\n", __func__);
		ret = -EFAULT;

Exit:
		hscd_update_error_count();
		up(&hscd_sem);			// Exclusive control end
	}
	return ret;
}

/* FUJITSU:2011-04-27 Ver3 correction start */
/*--------------------------------------------------------------------
Function  :selftest of HSCD is executed. 
@param    :selfdata->testmode:select test mode.
output    :member of *selfdata.
	        test_ret:result value of self test
	        err_code:STBB value of self test
	          (test_mode = modeB)
@retval   :(1) 0:success.
	        (2) -EBUSY:I2C Communication fault (time out).
	        (3) -EIO:I/O error retries reach upper limit.
	        (4) -EINTR:Received a signal while waiting for Exclusive control.
	        (5) -EFAULT:regs adress Injustice.
--------------------------------------------------------------------*/
static int hscd_ioctl_selftest(selftest_data_t __user * selfdata)
{
	int8_t mode;
	int ret;
	uint8_t reg_data, check_data, buffer[HSCD_MAX_CONTIGUOUS_REGS];
	
	debug("%s start\n",__func__);
	
	if ((ret = down_interruptible(&hscd_sem)) >= 0) {
		// Exclusive control start
		
		// mode <- *selfdata->mode
		if (copy_from_user(&mode, &selfdata->test_mode, sizeof(mode)) != 0) {
			debug("%sCan't copy from user area.\n", __func__);
			ret = -EFAULT;
			goto Exit;
		}
		
		if (mode == MODE_A) {
			// SelfTest modeA
			ret = hscd_read_verify_regs(HSCD_STBA, buffer,1);
			if (ret < 0) {
				goto Exit;
			}
			
			// STBA check
			memcpy(&check_data, buffer, sizeof(check_data));
			if (check_data != HSCD_SELFTEST_DEF) {
				ret = -EFAULT;
				goto Exit;
			}
			
			// STCA write
			reg_data = HSCD_CNTL3_STCA;
			ret = hscd_write_regs(HSCD_CNTL3, &reg_data, 1);
			if (ret < 0) {
				goto Exit;
			}
			// STBA check1
			ret = hscd_read_regs(HSCD_STBA, buffer,1);
			if (ret < 0) {
				goto Exit;
			}
			memcpy(&check_data, buffer, sizeof(check_data));
			if (check_data != HSCD_SELFTEST_PASS) {
				selfdata->test_ret = SELF_TEST_FAIL;
				ret = 0;
				goto Exit;
			}
			
			// STBA check2
			ret = hscd_read_verify_regs(HSCD_STBA, buffer,1);
			if (ret < 0) {
				goto Exit;
			}
			memcpy(&check_data, buffer, sizeof(check_data));
			if (check_data != HSCD_SELFTEST_DEF) {
				selfdata->test_ret = SELF_TEST_FAIL;
			} else {
				selfdata->test_ret = SELF_TEST_PASS;
			}
			
		} else if (mode == MODE_B) {
			// SelfTest modeB
			ret = hscd_read_verify_regs(HSCD_STBB, buffer,1);
			if (ret < 0) {
				goto Exit;
			}
			
			// STBB check
			memcpy(&check_data, buffer, sizeof(check_data));
			if (check_data != HSCD_SELFTEST_DEF) {
				ret = -EFAULT;
				goto Exit;
			}
			
			// STCB write
			reg_data = HSCD_CNTL3_STCB;
			ret = hscd_write_regs(HSCD_CNTL3, &reg_data, 1);
			if (ret < 0) {
				goto Exit;
			}
			// Waiting time
			mdelay(HSCD_Twat);
			// STBB check1
			ret = hscd_read_regs(HSCD_STBB, buffer,1);
			if (ret < 0) {
				goto Exit;
			}
			
			//error_code get
			if (copy_to_user(&selfdata->err_code, 
					buffer, sizeof(selfdata->err_code)) != 0) {
				debug("%s Can't copy user area.\n", __func__);
				ret = -EFAULT;
				goto Exit;
			}
			memcpy(&check_data, buffer, sizeof(check_data));
			if (check_data != HSCD_SELFTEST_PASS) {
				selfdata->test_ret = SELF_TEST_FAIL;
				ret = 0;
				goto Exit;
			}
			
			// STBB check2
			ret = hscd_read_verify_regs(HSCD_STBB, buffer,1);
			if (ret < 0) {
				goto Exit;
			}
			memcpy(&check_data, buffer, sizeof(check_data));
			if (check_data != HSCD_SELFTEST_DEF) {
				selfdata->test_ret = SELF_TEST_FAIL;
			} else {
				selfdata->test_ret = SELF_TEST_PASS;
			}
		} else {
			ret = -EFAULT;
		}
		
Exit:		
		up(&hscd_sem);			// Exclusive control end
	}
	
	return ret;
}

int hscd_ioctl_measure_temp(compass_data_t __user * measurement) {
	
	int ret,retry_count;
	uint8_t reg_data, buffer;
	compass_data_t *const m = &hscd->measurement;
	
	retry_count = 0;
	
	// changes to an active mode
	if (system_rev >= 0x0c || system_rev == 0x0a) {
		reg_data = HSCD_MODE_FORCESTATE_V2;
	} else {
		reg_data = HSCD_MODE_FORCESTATE_V3;
	}
	ret = hscd_write_verify_regs(HSCD_CNTL1, &reg_data, 1, 0x80);
	if (ret < 0)
		goto Exit;
	
	// CNTL3 is temperature and forcestate measurement
	reg_data = HSCD_CNTL3_TCS;
	ret = hscd_write_regs(HSCD_CNTL3, &reg_data, 1);
	if (ret < 0)
		goto Exit;
	
	if (system_rev >= 0x0c || system_rev == 0x0a) {
		mdelay(HSCD_Twat);		// Waiting time
		ret = hscd_read_verify_regs(HSCD_TEMP, &hscd->reg.temp,
											sizeof(hscd->reg.temp));
		if (ret < 0)
			goto Exit;
	} else {
		mdelay(1);		// Waiting time
		for(;;) 
		{
			ret = hscd_read_verify_regs(HSCD_STAT, 
						&buffer, sizeof(hscd->reg.stat));
			if (ret < 0)
				goto Exit;
			if ((buffer & HSCD_STAT_TRDY) == HSCD_STAT_TRDY) {
				ret = hscd_read_verify_regs(HSCD_TEMP, &hscd->reg.temp,
												sizeof(hscd->reg.temp));
				if (ret < 0)
					goto Exit;
				break;
			}
			
			if (retry_count > 3) {
				compass_printk(KERN_ERR,
					"%s: temp data get error.\n", HSCD_DEVNAME);
				ret = -EIO;
				goto Exit;
			}
			mdelay(1);		// Waiting time
			retry_count++;
		}
	}
	
	// The measuring data is copied onto hscd->measurement. 
	m->temperature = hscd->reg.temp;	// TEMP
	if (copy_to_user(measurement, &hscd->measurement, 
			sizeof(*measurement)) != 0) {
			compass_printk(KERN_ERR," %s Can't copy to user area.\n",__func__);
			ret = -EFAULT;
			goto Exit;
	}
	ret = 0;
	return ret;
Exit:
	return ret;
}

/*--------------------------------------------------------------------
Function  :ioctl() System call processing
@param    :(1) inode :inode Information of Device file (Unused).
	        (2) file :file descriptor of ioctl() file Structure (Unused).
	        (3) command:command number.
	            COMPASS_IOC_GETDATA:Magnetic measurements
	            COMPASS_IOC_GETREGS:Register of HSCD to read the content.
	            COMPASS_IOC_SETREGS:Write data register of HSCD.
	            COMPASS_IOC_GETDATASTOP:exit the magnetic measurements
	            COMPASS_IOC_SELFTEST:Self Test
	        (4) arg:Command Arguments.
	            COMPASS_IOC_GETDATA (Type:compass_data_t*):
	            COMPASS_IOC_GETREGS (Type:hscd_registers_t*):
	            COMPASS_IOC_SETREGS (Type:const hscd_registers_t*):
	            COMPASS_IOC_GETDATASTOP  (Type:compass_data_t*):
	            COMPASS_IOC_SELFTEST (Type:selftest_data_t*)
@retval   :(1) 0:success.
	        (2) -EINVAL:Undefined command number.
	        (3) -EBUSY:I2C Communication fault (time out).
	        (4) -EIO:I/O error retries reach upper limit.
	        (5) -EFAULT:arg adress Injustice.
--------------------------------------------------------------------*/
int hscd_ioctl(struct inode *inode, struct file *file,
			   unsigned command, u_long arg)
{
	int ret;

	debug("%s(0x%08X, 0x%08lX)\n", __func__, command, arg);

	switch (command) {
	case COMPASS_IOC_GETDATA:
		ret = hscd_ioctl_measure((compass_data_t __user *)arg);
		break;
	case COMPASS_IOC_TEMP_COR:
		ret = hscd_ioctl_measure_temp((compass_data_t __user *)arg);
		break;
	case COMPASS_IOC_GETREGS:
		ret = hscd_ioctl_getregs((hscd_registers_t __user *)arg);
		break;
	case COMPASS_IOC_SETREGS:
		ret = hscd_ioctl_setregs((const hscd_registers_t __user *)arg);
		break;
	case COMPASS_IOC_OFFCAL:
		ret = hscd_ioctl_offcal((hscd_registers_t __user *)arg);
		break;
	case COMPASS_IOC_GETDATASTOP:
		ret = 0;
		break;
	case COMPASS_IOC_SELFTEST:
		if(system_rev >= 0x0c || system_rev == 0x0a) {
			ret = -EINVAL;
		} else {
			ret = hscd_ioctl_selftest((selftest_data_t __user *)arg);
		}
		break;
	default:
		ret = -EINVAL;		// Undefined command number
	}

	debug("%s() -> %d\n", __func__, ret);

	return ret;
}

MODULE_AUTHOR ("FUJITSU,2011");
MODULE_DESCRIPTION ("hscdtd002a driver");
MODULE_LICENSE ("GPL");