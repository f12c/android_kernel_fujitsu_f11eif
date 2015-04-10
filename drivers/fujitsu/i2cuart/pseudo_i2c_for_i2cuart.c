/* 
 *
 * Copyright (c) 2008-2010, TOSHIBA Corporation All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>
#include <linux/delay.h>
#include "pseudo_i2c_for_i2cuart.h"

#define SCL_GPIO 0
#define SDA_GPIO 1

#define CAP_I2C_LOOP_CNT_SPBIG      (100000)
#define CAP_I2C_LOOP_CNT_BIG        (100)
#define CAP_I2C_LOOP_CNT_MED        (256)
#define CAP_I2C_LOOP_CNT_SMALL      (32)

#define SDA_LOW()           gpio_direction_output(SDA_GPIO, 0)
#define SDA_HIGH()          gpio_direction_input(SDA_GPIO)
#define SCL_LOW()           gpio_direction_output(SCL_GPIO, 0)
#define SCL_HIGH()          gpio_direction_input(SCL_GPIO)
#define SCL_and_SDA_HIGH()  gpio_direction_input(SCL_GPIO);\
                            gpio_direction_input(SDA_GPIO)

#define SCL_IS_HIGH()       (gpio_get_value(SCL_GPIO) != 0)
#define SCL_IS_LOW()        (gpio_get_value(SCL_GPIO) == 0)
#define SDA_IS_HIGH()       (gpio_get_value(SDA_GPIO) != 0)
#define SDA_IS_LOW()        (gpio_get_value(SDA_GPIO) == 0)
#define SDA_SCL_IS_HIGH()   (SCL_IS_HIGH() && SDA_IS_HIGH())

/* SDA set, SDA check  */
#define SDA_EQUAL(val)      (val ? SDA_IS_HIGH() : SDA_IS_LOW())

/*--------- Wait ---------*/
#define WAIT_HIGH()         //clk_pause(10)
#define WAIT_LOW()          //clk_pause(10)

#define TRUE true
#define FALSE false


static bool cap_i2c_init_flag  = FALSE;


static void cap_i2c_stop(void)
{

    /* SDA -->drive LOW */
    SDA_LOW();
    WAIT_LOW();

    /* SCL -->pull-up HIGH */
    SCL_HIGH();
    WAIT_HIGH();

    /* SDA -->pull-up HIGH */
    SDA_HIGH();
    WAIT_HIGH();

    return;
}

static bool cap_i2c_chk_ack(void)
{
    unsigned int cnt;
	bool         rc;

    /* SDA -->pull-up HIGH */
    SDA_HIGH();

    /* SCL --> pull-up HIGH */
    SCL_HIGH();

    /* wait SCL --> HIGH */
    for( cnt = CAP_I2C_LOOP_CNT_BIG; cnt; cnt-- ){
        if( SCL_IS_HIGH() ) {
            break;
        }
    }

    if( cnt == 0 ) {
        printk( KERN_ERR "%s : wait SCL High cnt =%d \n", __func__, cnt );
        return FALSE;
    }

    /* check SDA */
    for( cnt = CAP_I2C_LOOP_CNT_BIG; cnt; cnt-- ){
        if( SDA_IS_LOW() ) {
            break;
        }
    }

    rc = SDA_IS_LOW();  /* ACK(LOW)? */

    /* SCL --> Drive LOW, SDA --> pull-up HIGH */
    SCL_LOW();

    return rc;
}

static void cap_i2c_send_ack(void)
{
    int cnt;

    /* SDA -->pull-up LOW */
    SDA_LOW();

    /* SCL --> pull-up HIGH */
    SCL_HIGH();

    /* wait SCL --> HIGH */
    for( cnt = CAP_I2C_LOOP_CNT_SPBIG; cnt; cnt-- ){
        if( SCL_IS_HIGH() ) {
            break;
        }
    }

    SCL_LOW();
    
    return;
}


static void cap_i2c_send_nak(void)
{
    int cnt;

    /* SDA -->pull-up HIGH */
    SDA_HIGH();

    /* SCL --> pull-up HIGH */
    SCL_HIGH();

    /* wait SCL --> HIGH */
    for( cnt = CAP_I2C_LOOP_CNT_SPBIG; cnt; cnt-- ){
        if(SCL_IS_HIGH()) {
            break;
        }
    }

    SCL_LOW();
    SDA_LOW();

    return;
}

static bool cap_i2c_restart(void)
{

    /*** restart ***/
    /* SDA --> pull-up HIGH */
    SDA_HIGH();

    /* SCL --> pull-up HIGH */
    SCL_HIGH();

    /* SCL high? */
    if( SCL_IS_LOW() ) {
        printk( KERN_ERR "%s : error SCL is Low \n", __func__ );
        goto exit;
    }

    /* SDA --> drive LOW */
    SDA_LOW();

    /* SCL high? */
    if( SCL_IS_LOW() ) {
        printk( KERN_ERR "%s : error SCL is Low \n", __func__ );
        goto fault;
    }

    /* SCL --> Drive LOW */
    SCL_LOW();

    /* start done. SCL and SDA LOW */
    return TRUE;

 fault: ;    /* exit with direction input */
    SCL_and_SDA_HIGH();

 exit: ;     /* exit only */
    return FALSE;
}

static bool cap_i2c_start(void)
{
    unsigned int    cnt;
    bool            stat;

    stat = FALSE;
    SDA_HIGH();
    SCL_HIGH();
    for(cnt = CAP_I2C_LOOP_CNT_BIG; (cnt && !stat); cnt--){
        stat = SDA_SCL_IS_HIGH();
    }
    if ( !stat ) {
        printk( KERN_ERR "%s : error SCL/SDA is not HIGH\n", __func__ );
        goto exit;
    }

    /* check bus free? */
    stat = TRUE;

    for(cnt = CAP_I2C_LOOP_CNT_BIG; (cnt && stat); cnt--){
        stat = SDA_SCL_IS_HIGH();
    }
    if( !stat ) {
        printk( KERN_ERR "%s : error SCL/SDAL is not HIGH\n", __func__ );
        goto exit; /* someone uses the i2c bus... */
    }

    /* SDA --> drive LOW */
    SDA_LOW();

    /* SCL high? */
    if( SCL_IS_LOW() ) {
        printk( KERN_ERR "%s : error SCL is Low\n", __func__ );
        goto fault;
    }

    /* SCL --> drive LOW */
    SCL_LOW();

    /* start done. SCL and SDA LOW */
    return TRUE;

fault: ;    /* exit with direction input */
    SCL_and_SDA_HIGH();

exit: ;     /* exit only */
    return FALSE;
}


static bool cap_i2c_recv_byte(uint8_t *val)
{
    uint8_t mask, data;
    int     cnt;


    /* ensure SDA pull-up high */
    SDA_HIGH();

    mask = 0x80;
    data = 0;

    while(mask){

        /* SCL --> pull-up HIGH */
        SCL_HIGH();
        /* wait SCL turn into HIGH */
        for( cnt = CAP_I2C_LOOP_CNT_MED; cnt; cnt-- ){
            if(SCL_IS_HIGH()) {
                break;
            }
        }
        if(cnt == 0) {
            printk( KERN_ERR "%s : error SCL is not HIGH. cnt = %d\n", __func__, cnt );
            goto fault;
        }

        /* check SDA */
        if(SDA_IS_HIGH()) {
            data |= mask;
        }

        /* SCL --> drive LOW */
        SCL_LOW();
        mask >>= 1;

    }

    /* Done, SCL is in LOW. */
    *val = data;
    return TRUE;

 fault: ;
    cap_i2c_stop();
    return FALSE;
}

static bool cap_i2c_send_byte(uint8_t val)
{
    uint8_t mask;
    int     cnt;


    mask = 0x80;
    
    while(mask){

        if(val & mask) {
            // SDA : 1
            SDA_HIGH();
            /* SCL --> pull-up HIGH */
            SCL_HIGH();

            /* wait SCL turn into HIGH */
            for( cnt = CAP_I2C_LOOP_CNT_BIG; cnt; cnt-- ){
                if(SCL_IS_HIGH()) {
                    break;
                }
            }
            if(cnt == 0) {
                printk( KERN_ERR "%s : error SCL is not HIGH. cnt = %d\n", __func__, cnt );
                goto fault;
            }
            /* check SDA */
            if( !SDA_IS_HIGH() ) {
                printk( KERN_ERR "%s : error SDA is not HIGH.\n", __func__ );
                goto exit;  // arbitration. lose game... 
            }
        } else {
            // SDA : 0
            SDA_LOW();

            /* SCL --> pull-up HIGH */
            SCL_HIGH();

            /* wait SCL turn into HIGH */
            for( cnt = CAP_I2C_LOOP_CNT_BIG; cnt; cnt-- ){
                if( SCL_IS_HIGH() ) {
                    break;
                }
            }
            if( cnt == 0 ) {
                printk( KERN_ERR "%s : error SCL is not HIGH. cnt = %d\n", __func__, cnt );
                goto fault;
            }

            if( !SDA_IS_LOW() ) {
                printk( KERN_ERR "%s : error SCL is not HIGH.\n", __func__ );
                goto exit;  // arbitration. lose game... 
            }
        }

        /* SCL --> drive LOW */
        SCL_LOW();
        mask >>= 1;
    }


    /* Done, SCL is in LOW. */
    return TRUE;

 fault: ;
    cap_i2c_stop();

 exit: ;
    SCL_and_SDA_HIGH();

    return FALSE;
}


static bool cap_i2c_read_bytes_internal( I2C_INFO *cmd_ptr )
{

  //    i2c_options_type    option = cmd_ptr->options;
    uint8_t    slave = (cmd_ptr->slave_addr << 1);
    uint8_t    *data = cmd_ptr->buf_ptr;
    int         size = (int)cmd_ptr->len;
    bool        rc = FALSE;
    int         idx = 0;
    uint8_t     reg;
	uint8_t     buf;
    
    buf = 0;

    // Send Start Condition
    if( !(rc = cap_i2c_start()) ) {
        goto exit;
    }

    /* Send Slave Address (Write) */
    if( !(rc = cap_i2c_send_byte(slave & ~1)) ) {
        printk(KERN_ERR "cap_i2c_read_bytes_internal error!!! slave address send\n");
        goto exit;
    }

    // -> Wait ACK
    if( !(rc = cap_i2c_chk_ack()) ) {
        printk(KERN_ERR "cap_i2c_read_bytes_internal error!!! ack check\n");
        goto fault;
    }

    /* Send Register Address */
    reg = (uint8_t)(cmd_ptr->offset & 0xff);
    if( !(rc = cap_i2c_send_byte(reg)) ) {
        printk(KERN_ERR "cap_i2c_read_bytes_internal error!!! address send\n");
        goto exit;
    }

    // -> Wait ACK
    if( !(rc = cap_i2c_chk_ack()) ) {
        printk(KERN_ERR "cap_i2c_read_bytes_internal error!!! ack check\n");
        goto fault;
    }

    /* Send Restart condition */
    if( !(rc = cap_i2c_restart()) ) {
        printk(KERN_ERR "cap_i2c_read_bytes_internal error!!! restart\n");
        goto exit;
    }

    /* Send Slave Address Again (Read) */
    if( !(rc = cap_i2c_send_byte(slave | 1)) ) {
        printk(KERN_ERR "cap_i2c_read_bytes_internal error!!! slave address send\n");
        goto exit;
    }

    // -> Wait ACK
    if( !(rc = cap_i2c_chk_ack()) ) {
        printk(KERN_ERR "cap_i2c_read_bytes_internal error!!! ack check\n");
        goto fault;
    }

    /* Receive Register Data */
    for(idx = 0; idx < size; idx++) {
        /* Receive 1 byte data */
        rc = cap_i2c_recv_byte(&buf);

//        printk(KERN_ERR "cap_i2c_read_bytes_internal recv data = 0x%x\n",buf);

        data[idx] = buf;

        if ( (idx + 1) < size ) {
            cap_i2c_send_ack();
        } else {
            cap_i2c_send_nak();
        }
    }

    /* Check No Ack (Data End) */
    cap_i2c_stop();
    return TRUE;

fault: ;
    /* Send Stop Condition */
    cap_i2c_stop();

exit: ;
  return rc;
}

static bool cap_i2c_write_bytes_internal( I2C_INFO *cmd_ptr )
{
    uint8_t     slave = (cmd_ptr->slave_addr << 1);
    uint8_t    *data = cmd_ptr->buf_ptr;
    uint32_t    size = (int)cmd_ptr->len;
    bool        ret = FALSE;
    uint32_t    idx = 0;
	uint8_t     reg;

    // Send Start Condition
    if( !(ret = cap_i2c_start()) ) {
        printk( KERN_ERR "%s : send start condition error \r\n", __func__ );
        goto exit;
    }

    // Send Slave Address (7bit)
    if( !(ret = cap_i2c_send_byte(slave & ~1)) ) {
        printk( KERN_ERR "%s : send slave address error\r\n", __func__ );
        goto exit;
    }

    // -> Wait ACK
    if( !(ret = cap_i2c_chk_ack()) ) {
        printk( KERN_ERR "%s : send slave address ack check error\r\n", __func__ );
        goto fault;
    }

    // Send Data Address1 (8bit)
    reg = (uint8_t)(cmd_ptr->offset & 0xff);
    if( !(ret = cap_i2c_send_byte(reg)) ) {
        printk( KERN_ERR "%s : send data address1 error \r\n", __func__ );
        goto exit;
    }

    // -> Wait ACK
    if( !(ret = cap_i2c_chk_ack()) ) {
        printk( KERN_ERR "%s : send data address1 ack check error\r\n", __func__ );
        goto fault;
    }

    // Send Data Bytes (8bit * size)
    for(idx=0; idx<size; idx++){
        // Send Data 1byte
        if( !(ret = cap_i2c_send_byte(data[idx])) ) {
            printk( KERN_ERR "%s : send data bytes error\r\n", __func__ );
            goto exit;
        }

        // -> Wait ACK
        if( !(ret = cap_i2c_chk_ack()) ) {
            printk( KERN_ERR "%s : send data bytes ach check error\r\n", __func__ );
            goto fault;
        }
    }

fault: ;
  cap_i2c_stop();

exit: ;
  return ret;
}

bool pse_i2c_init(void)
{
    uint32_t    cnt;
    uint32_t    val;
    bool        ret = FALSE;
    
    if( cap_i2c_init_flag == TRUE ) {
        printk( KERN_ERR "%s : capI2C driver is already initialised\n", __func__ );
        goto err;
    }

    /* clear gpio output enable (SCL, SDA to HIGH) */
    SCL_and_SDA_HIGH();

    /* scl and sda are in high level? */
    cnt = CAP_I2C_LOOP_CNT_MED;
    do{
        val = SDA_SCL_IS_HIGH();
        cnt--;
    } while (cnt && !val);
    
    if( cnt == 0 ) {
        printk( KERN_ERR "%s : SDA and SCL could not set to High \n", __func__ );
        goto err;
    }

    cap_i2c_init_flag = TRUE;
    ret = TRUE;
    
err:
    return ret;
}

bool pse_i2c_read( I2C_INFO *cmd_ptr )
{
    bool ret = FALSE;

    
    if( cap_i2c_init_flag != TRUE ) {
        printk( KERN_ERR "%s : capI2C driver is not initialised\n", __func__ );
        goto err;
    }

    ret = cap_i2c_read_bytes_internal(cmd_ptr);

err:
    return ret;
}

bool pse_i2c_write( I2C_INFO *cmd_ptr )
{
    bool ret = FALSE;

    if( cap_i2c_init_flag != TRUE ) {
        printk( KERN_ERR "%s : capI2C driver is not initialised\n", __func__ );
        goto err;
    }

    ret = cap_i2c_write_bytes_internal(cmd_ptr);

err:
    return ret;
}

MODULE_AUTHOR ("FUJITSU,2011");
MODULE_DESCRIPTION ("pseudo i2cuart driver");
MODULE_LICENSE ("GPL");
