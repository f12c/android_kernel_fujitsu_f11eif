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

/*============================================================================

                    Camera Sensor GPIO Driver Source File

============================================================================*/

/*============================================================================
                        INCLUDE FILES
============================================================================*/
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>
#include "camsensor_gpio.h"

/*============================================================================
                        INTERNAL FEATURES
============================================================================*/

/*============================================================================
                        CONSTANTS
============================================================================*/
#define TRUE                    1
#define FALSE                   0

#define _DUMP                   1

#define SCL                     109          // I2C SCL GPIO No
#define SDA                     108          // I2C SDA GPIO No

#if 0
#define SCL_Low()               gpio_configure(SCL, GPIOF_DRIVE_OUTPUT)
#define SCL_High()              gpio_configure(SCL, GPIOF_INPUT)
#define SDA_Low()               gpio_configure(SDA, GPIOF_DRIVE_OUTPUT)
#define SDA_High()              gpio_configure(SDA, GPIOF_INPUT)
#else
#define SCL_Low()               gpio_direction_output(SCL, 0)
#define SCL_High()              gpio_direction_input(SCL)
#define SDA_Low()               gpio_direction_output(SDA, 0)
#define SDA_High()              gpio_direction_input(SDA)
#endif

#define SDA_Read()              gpio_get_value(SDA)
#define SCL_Read()              gpio_get_value(SCL)

//#define LOGI(fmt, args...)      printk(KERN_INFO "camI2C: " fmt, ##args)
#define LOGE(fmt, args...)      printk(KERN_ERR "camI2C: " fmt, ##args)
#define LOGI(fmt, args...)      do {} while (0)
//#define LOGE(fmt, args...)      do {} while (0)

/*============================================================================
                        EXTERNAL VARIABLES DEFINITIONS
============================================================================*/
int _I2C_LOG_ = 0;

/*============================================================================
                        INTERNAL API DECLARATIONS
============================================================================*/
//void WaitHighSpeed(int us) {usleep(us);}
static void WaitHighSpeed(int us) {}
static void ( *Wait )( int us ) = WaitHighSpeed;

static boolean SCL_IsHigh( void )
{
    int cnt;

    // Check SCL High
//    for(cnt = 1000 * 2; cnt; cnt--)
    for(cnt = 1000 * 12; cnt; cnt--) // 35ms
    {      
        udelay(3);
        if( SCL_Read() )    return TRUE;    // SCL High
    }
    return FALSE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_START_CONDITION
===========================================================================*/
static boolean camsensor_gpio_start_condition(void)
{
    // In : SCL In (Pull-UP), SDA In (Pull-UP)
    // Out: SCL Out (Drive-Low), SDA Out (Drive-Low)

    // Bus free Check
    if (!SCL_IsHigh())      // SCL Low!
    {
        LOGE("start_condition Error (SCL Low) !\n");
        return FALSE;
    }
    // SDA Low (Drive-Low)
    SDA_Low();
    Wait(5);

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(2);

    return TRUE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_STOP_CONDITION
===========================================================================*/
static boolean camsensor_gpio_stop_condition(void)
{
    // In : SCL Out (Drive-Low), SDA In/Out
    // Out: SCL In (Pull-UP), SDA In (Pull-UP)

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(2);

    // SDA Low (Drive-Low)
    SDA_Low();
    Wait(2);

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High wait
    if (!SCL_IsHigh())      // SCL Low!
    {
        LOGE("stop_condition Error (SCL Low) !\n");
    }
    Wait(3);

    // SDA HIGH (Pull-UP)
    SDA_High();
    Wait(2);
    return TRUE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_RESTART_CONDITION
===========================================================================*/
static boolean camsensor_gpio_restart_condition(void)
{
    // In : SCL Out (Drive-Low), SDA Out (Drive-Low)
    // Out: SCL Out (Drive-Low), SDA Out (Drive-Low)

    // SDA High (Pull-UP)
    SDA_High();
    Wait(2);

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High Check
    if (!SCL_IsHigh())
    {
        LOGE("restart_condition Error (SCL Low) !\n");
        return FALSE;
    }
    Wait(3);

    // SDA Low (Drive-Low)
    SDA_Low();
    Wait(3);

    // SCL Low (Drive Low)
    SCL_Low();
    Wait(3);

    return TRUE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_CHK_ACK
===========================================================================*/
static boolean camsensor_gpio_chk_ack(void)
{
    int sda;

    // In : SCL Out (Drive-Low), SDA In (Pull-UP)
    // Out: SCL Out (Drive-Low), SDA In (Pull-UP)

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High Check
    if (!SCL_IsHigh())
    {
        LOGE("chk_ack Error (SCL Low) !\n");
        return FALSE;
    }

    // SDA Signal Read
    sda = SDA_Read();
    Wait(3);

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(3);
 
    // response check
    if (sda)
    {                       // Nack recv
        LOGE("#chk_ack Error (Nack. Receive)!\n");
        return FALSE;
    }

    return TRUE;            // Ack Recv
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_SEND_NACK
===========================================================================*/
static boolean camsensor_gpio_send_ack(uint16_t len)
{
    // In : SCL Out (Drive-Low), SDA In/Out
    // Out: SCL Out (Drive-Low), SDA Out (Drive-Low)

    if( !len )
    {   // Nack
        // SDA High (Pull-UP)
        SDA_High();
        Wait(2);
    }
    else
    {   // Ack
        // SDA Low (Drive-Low)
        SDA_Low();
        Wait(2);
    }

    // SCL High (Pull-UP)
    SCL_High();
    Wait(5);

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(2);

    // SDA Low (Drive-Low)
    SDA_Low();
    Wait(2);

    return TRUE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_SEND_BYTE
===========================================================================*/
static boolean camsensor_gpio_send_byte(uint8_t val)
{
    int     dir;
    uint8_t mask;

    // In : SCL Out (Drive-Low), SDA In/Out
    // Out: SCL Out (Drive-Low), SDA In (Pull-UP)

    mask = 0x80;
    // MSB Bit Output
    dir = val & mask;
    if (dir)
    {
        // SDA High (Pull-UP)
        SDA_High();
    }
    else
    {
        // SDA Low (Drive-Low)
        SDA_Low();
    }
    Wait(2);

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High Check
    if (!SCL_IsHigh())
    {
        LOGE("send_byte SCL Error !\n");
        return FALSE;
    }
    Wait(3);

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(3);

    // 7bit output
    mask >>= 1;
    while( mask )
    {
        // SDA 1Bit out
        dir = val & mask;
        if (dir)
        {
            // SDA High (Pull-UP)
            SDA_High();
        }
        else
        {
            // SDA Low (Drive-Low)
            SDA_Low();
        }
        Wait(2);

        // SCL High (Pull-UP) 
        SCL_High();
        Wait(4);

        // SCL Low (Drive-Low)
        SCL_Low();
        Wait(2);

        mask >>= 1;
    
    }

    // SDA High (Pull-UP)
    SDA_High();
    Wait(4);

    return TRUE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_RECV_BYTE
===========================================================================*/
static boolean camsensor_gpio_recv_byte(uint8_t *val)
{
    int     sda;
    uint8_t mask = 0x80;
    uint8_t data = 0x00;

    // In : SCL Out (Drive-Low), SDA Out (Drive-Low)
    // Out: SCL Out (Drive-Low), SDA In (Pull-UP)

    // SDA Hige (Pull-UP)
    SDA_High();
    Wait(2);

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High Check
    if (!SCL_IsHigh())
    {
        LOGE("recv_byte SCL Error !\n");
        return FALSE;
    }
    Wait(3);

    // SDA in
    sda = SDA_Read();
    if (sda)
        data |= mask;

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(3);

    // 7bit output
    mask >>= 1;
    while(mask)
    {
        // SCL High (Pull-UP)
        SCL_High();
        Wait(2);

        // SDA in
        sda = SDA_Read();
        if (sda)
          data |= mask;
        Wait(2);

        // SCL Low (Drive-Low)
        SCL_Low();
        Wait(4);

        mask >>= 1;
    }
    // Done, SCL is in LOW.
    *val = data;

    Wait(2);

    return TRUE;
}

/*============================================================================
                        EXTERNAL API DEFINITIONS
============================================================================*/

/*===========================================================================
    FUNCTION      CAMSENSOR_GPIOI2C_WRITE
===========================================================================*/
int camsensor_gpioi2c_write(struct CameraSensorI2CCmdType *pI2CCmd)
{
    int rc = TRUE;
    uint16_t len;
    uint8_t  *pd;
    char    tszDump[256] = "";
    int     idx = 0;
	uint8_t slave;

    // 1.Start Condition
    if( !(rc = camsensor_gpio_start_condition()) )  goto fault;
                 
	slave = pI2CCmd->slave_addr << 1;
	
    // 2.SEND - Slave Address(Camera Device)
    if( !(rc = camsensor_gpio_send_byte(slave)) ) goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        LOGE("write. fault Slave addr\n");
        goto fault;
    }

    // 3.SEND - Data
    for(len = pI2CCmd->wlen, pd = pI2CCmd->pwdata ; len > 0 ; --len, ++pd)
    {
        if( !(rc = camsensor_gpio_send_byte(*pd)) )    goto fault;
        // Chk Acknowledge
        if( !(rc = camsensor_gpio_chk_ack()) )
        {
            LOGE("write. fault Data\n");
            goto fault;
        }
    }

    // 4.Stop Condition
    camsensor_gpio_stop_condition();
    goto exit;

fault : ;
    camsensor_gpio_stop_condition();

exit : ;
#if _DUMP
    for(len = 1, pd = pI2CCmd->pwdata+1 ; len < pI2CCmd->wlen && len < 12; ++len, ++pd)
    {
        idx += sprintf(&tszDump[idx], "%02X ", *pd);
    }
#endif
    if (_I2C_LOG_ != 0 || !rc)
        LOGI("write C:%02X T:%s (%s)\n",
               *pI2CCmd->pwdata, tszDump, rc ? "Ok" : "Error");

    return rc ? RET_OK : RET_ERR;
}

/*===========================================================================
    FUNCTION      CAMSENSOR_GPIOI2C_READ
===========================================================================*/
int camsensor_gpioi2c_read(struct CameraSensorI2CCmdType *pI2CCmd)
{
    int rc = TRUE;
    uint16_t len;
    uint8_t  *pd;
    char    tszDump[256] = "";
//    int     idx = 0;
	uint8_t slave;

    // 1.Start Condition    
    if( !(rc = camsensor_gpio_start_condition()) )  goto fault;

	slave = pI2CCmd->slave_addr << 1;
	
    // 2. SEND - Slave Address(Camera Device)
    if( !(rc = camsensor_gpio_send_byte(slave)) ) goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        LOGE("read. fault Slave[W] addr\n");
        goto fault;
    } 

    // 3.SEND - Data
    for(len = pI2CCmd->wlen, pd = pI2CCmd->pwdata ; len > 0 ; --len, ++pd)
    {
        if( !(rc = camsensor_gpio_send_byte(*pd)) )    goto fault;
        // Chk Acknowledge
        if( !(rc = camsensor_gpio_chk_ack()) )
        {
            LOGE("raed. fault Write Data\n");
            goto fault;
        }
    }

    // 4.Restart Condition
    if( !(rc = camsensor_gpio_restart_condition()) )    goto fault;

    // 5.SEND - Slave Address again(Camera Device)
    if( !(rc = camsensor_gpio_send_byte(slave | 0x01)) )  goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        LOGE("read. fault Slave[R] addr\n");
        goto fault;
    }

    // 6.RCV - Data
    for(len = pI2CCmd->rlen, pd = pI2CCmd->prdata ; len > 0 ; --len, ++pd)
    {
        if( !(rc = camsensor_gpio_recv_byte(pd)) )    goto fault;
        // Send Ack
        if( !(rc = camsensor_gpio_send_ack(len-1)) )
        {
            LOGE("read. fault ack \n");
            goto fault;
        }
    }

    // 7.Stop Condition
    camsensor_gpio_stop_condition();

//#if _DUMP
//    if (pI2CCmd->wlen > 1)
//        idx = sprintf(&tszDump[idx], "T:", *pd);
//    for(len = 1, pd = pI2CCmd->pwdata+1 ; len < pI2CCmd->wlen && len < 8; ++len, ++pd)
//    {
//        idx += sprintf(&tszDump[idx], "%02X ", *pd);
//    }
//    idx += sprintf(&tszDump[idx], "R:", *pd);
//    for(len = 0, pd = pI2CCmd->prdata ; len < pI2CCmd->rlen && len < 12; ++len, ++pd)
//    {
//        idx += sprintf(&tszDump[idx], "%02X ", *pd);
//    }
//#endif

    goto exit;

fault : ;
    camsensor_gpio_stop_condition();
    sprintf(tszDump, "%02X ",*pI2CCmd->prdata);

exit : ;
    if (_I2C_LOG_ != 0 || !rc)
        LOGI("read  C:%02X %s (%s)\n", 
             *pI2CCmd->prdata, tszDump, rc ? "Ok" : "Error");

    return rc ? RET_OK : RET_ERR;
}

/*===========================================================================
    FUNCTION      CAMSENSOR_GPIOI2C_WRITE_ISP
===========================================================================*/
int camsensor_gpioi2c_write_isp(struct CameraSensorI2CCmdTypeIsp *pI2CCmd)
{
    int rc = TRUE;
    uint16_t len;
    uint8_t  *pd;
//    char    tszDump[64] = "";
//    int     idx = 0;
	char    req_data[12]; // Max send buffer
	char*   req_buf;
	int     i = 0;
	uint8_t slave;

    // 1.Start Condition
    if( !(rc = camsensor_gpio_start_condition()) )  goto fault;
                                       
	slave = pI2CCmd->slave_addr << 1;
	
    // 2.SEND - Slave Address(Camera Device)
    if( !(rc = camsensor_gpio_send_byte(slave)) ) goto fault;
	
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        LOGE("write. fault Slave addr\n");
        goto fault;
    }

	// for M6Mo Parameter
	req_data[0] = 4 + pI2CCmd->wlen;
	req_data[1] = 0x02; // 0x02 is Write cmd.
	req_data[2] = pI2CCmd->category;
	req_data[3] = pI2CCmd->byte;
	req_buf = &req_data[4];
    LOGI("<write> s:%02x [c:%02x b:%02x]\n", slave, pI2CCmd->category, pI2CCmd->byte);

	for( i=0; i<pI2CCmd->wlen; i++ ) {
		req_buf[i] = pI2CCmd->pwdata[i];
    	LOGI("<write> - data:%02x\n", req_buf[i]);
	}
	pI2CCmd->wlen += 4; // Length is no send. because +3byte.
	
    // 3.SEND - Data
    for(len = pI2CCmd->wlen, pd = req_data ; len > 0 ; --len, ++pd)
    {
        if( !(rc = camsensor_gpio_send_byte(*pd)) )    goto fault;
        // Chk Acknowledge
        if( !(rc = camsensor_gpio_chk_ack()) )
        {
            LOGE("write. fault Data\n");
            goto fault;
        }
    }

    // 4.Stop Condition
    camsensor_gpio_stop_condition();
    goto exit;

fault : ;
    camsensor_gpio_stop_condition();

exit : ;
//#if _DUMP
//    for(len = 0, pd = req_data ; len < pI2CCmd->wlen && len < 12; ++len, ++pd)
//    {
//        idx += sprintf(&tszDump[idx], "%02X ", *pd);
//    }
//#endif
    if (_I2C_LOG_ != 0 || !rc)
        LOGI("write C:%02X (%s)\n",
               *pI2CCmd->pwdata, rc ? "Ok" : "Error");

    return rc ? RET_OK : RET_ERR;
}

/*===========================================================================
    FUNCTION      CAMSENSOR_GPIOI2C_READ_ISP
===========================================================================*/
int camsensor_gpioi2c_read_isp(struct CameraSensorI2CCmdTypeIsp *pI2CCmd)
{
    int rc = TRUE;
    uint16_t len;
    uint8_t  *pd;
//    char    tszDump[72] = "";
//    int     idx = 0;
	char    req_data[12];
	char    read_data[256];
//	char*   req_buf;
	int     i = 0;
	uint8_t slave;

    // 1.Start Condition    
    if( !(rc = camsensor_gpio_start_condition()) )  goto fault;

	slave = pI2CCmd->slave_addr << 1;
	
    // 2. SEND - Slave Address(Camera Device)
    if( !(rc = camsensor_gpio_send_byte(slave)) ) goto fault;
	
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        LOGE("read. fault Slave[W] addr\n");
        goto fault;
    } 

	// for M6Mo Parameter
	req_data[0] = 5;
	req_data[1] = 0x01; // 0x01 is Read cmd.
	req_data[2] = pI2CCmd->category;
	req_data[3] = pI2CCmd->byte;
	req_data[4] = pI2CCmd->rlen;
    LOGI("<read:w> s:%02x [c:%02x b:%02x] l:%d\n", slave, pI2CCmd->category, pI2CCmd->byte, pI2CCmd->rlen);
	pI2CCmd->wlen = 5; // Length is no send. because +4byte.

    // 3.SEND - Data
    for(len = pI2CCmd->wlen, pd = req_data ; len > 0 ; --len, ++pd)
    {
        if( !(rc = camsensor_gpio_send_byte(*pd)) )    goto fault;
        // Chk Acknowledge
        if( !(rc = camsensor_gpio_chk_ack()) )
        {
            LOGE("raed. fault Write Data\n");
            goto fault;
        }
    }

#if 1
	// 4-1.Stop Condition
    if( !(rc = camsensor_gpio_stop_condition()) )    goto fault;
    Wait(10);
	// 4-2.Start Condition
    if( !(rc = camsensor_gpio_start_condition()) )    goto fault;

#else
	// 4.Restart Condition
    if( !(rc = camsensor_gpio_restart_condition()) )    goto fault;
#endif
	
    // 5.SEND - Slave Address again(Camera Device)
    if( !(rc = camsensor_gpio_send_byte(slave | 0x01)) )  goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        LOGE("read. fault Slave[R] addr\n");
        goto fault;
    }

    // 6.RCV - Data
    for(len = pI2CCmd->rlen + 1, pd = read_data ; len > 0 ; --len, ++pd)
    {
        if( !(rc = camsensor_gpio_recv_byte(pd)) )    goto fault;
        // Send Ack
        if( !(rc = camsensor_gpio_send_ack(len-1)) )
        {
            LOGE("read. fault ack \n");
            goto fault;
        }
    }

    // 7.Stop Condition
    camsensor_gpio_stop_condition();

    for( i=0; i<pI2CCmd->rlen; i++ ) {
    	pI2CCmd->prdata[i] = read_data[i+1];
    	LOGI("<read:r> r:%d [c:%02x b:%02x]0x%02x\n", rc, pI2CCmd->category, pI2CCmd->byte, pI2CCmd->prdata[i]);
    }
	
//#if _DUMP
//    if (pI2CCmd->wlen > 1)
//        idx = sprintf(&tszDump[idx], "T:", *pd);
//    for(len = 0, pd = req_data ; len < pI2CCmd->wlen && len < 8; ++len, ++pd)
//    {
//        idx += sprintf(&tszDump[idx], "%02X ", *pd);
//    }
//    idx += sprintf(&tszDump[idx], "R:", *pd);
//    for(len = 0, pd = pI2CCmd->prdata ; len < pI2CCmd->rlen && len < 12; ++len, ++pd)
//    {
//        idx += sprintf(&tszDump[idx], "%02X ", *pd);
//    }
//#endif

    goto exit;

fault : ;
    camsensor_gpio_stop_condition();
//    sprintf(tszDump, "%02X ",*req_data);

exit : ;
    if (_I2C_LOG_ != 0 || !rc)
        LOGI("read  C:%02X (%s)\n", 
             *req_data, rc ? "Ok" : "Error");

    return rc ? RET_OK : RET_ERR;
}

/*===========================================================================
    FUNCTION      CAMSENSOR_GPIOI2C_READ
===========================================================================*/
int camsensor_i2c_read( struct i2c_client* client, struct CameraSensorI2CCmdTypeIsp* pI2CCmd )
{

    int rc = 0;
    struct i2c_msg msg[2];
    unsigned char read_data[256];
    unsigned char req_data[5];
    int i = 0;

	if (!client->adapter) {
        LOGE( "- %s err:client->adapter is NULL\n", __func__ );
		return -ENODEV;
	}

    req_data[0] = 5;
    req_data[1] = 0x01;
    req_data[2] = pI2CCmd->category;
    req_data[3] = pI2CCmd->byte;
    req_data[4] = pI2CCmd->rlen;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 5;
    msg[0].buf = req_data;
    
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = pI2CCmd->rlen + 1;
    msg[1].buf = read_data;

    rc = i2c_transfer(client->adapter, msg, 2);
    if( rc < 0 ) {
        LOGE( "I2C Transfer Err %d Read8:[%02x:0x%x-0x%x]\n", rc, client->addr, pI2CCmd->category, pI2CCmd->byte );
        rc = -EIO;
        return rc;
    }
    
    for( i=0; i<pI2CCmd->rlen; i++ ) {
    	pI2CCmd->prdata[i] = read_data[i+1];
    	LOGI("<read:r> r:%d [c:%02x b:%02x]0x%02x\n", rc, pI2CCmd->category, pI2CCmd->byte, pI2CCmd->prdata[i]);
    }

    LOGI( "I2C Transfer OK %d Read:[0x%x-0x%x] %02x\n", rc, pI2CCmd->category, pI2CCmd->byte, pI2CCmd->prdata[0] );
	
    return 0;

}

/*===========================================================================
    FUNCTION      CAMSENSOR_GPIOI2C_WRITE
===========================================================================*/
int camsensor_i2c_write( struct i2c_client* client, struct CameraSensorI2CCmdTypeIsp* pI2CCmd )
{

    int rc = 0;
    struct i2c_msg msg;
    unsigned char req_data[256];
    int i = 0;

	if (!client->adapter) {
        LOGE( "- %s err:client->adapter is NULL\n", __func__ );
		return -ENODEV;
	}

    req_data[0] = 4 + pI2CCmd->wlen;
    req_data[1] = 0x02;
    req_data[2] = pI2CCmd->category;
    req_data[3] = pI2CCmd->byte;
    LOGI("<write> s:%02x [c:%02x b:%02x]\n", client->addr, pI2CCmd->category, pI2CCmd->byte);
    
    for( i=0; i<pI2CCmd->wlen; i++ ) {
    	req_data[i+4] = pI2CCmd->pwdata[i];
    	LOGI("<write> - data:%02x\n", req_data[i+4]);
    }
    
    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = 4 + pI2CCmd->wlen;  /* add address bytes */
    msg.buf = req_data;

    rc = i2c_transfer(client->adapter, &msg, 1);
    if( rc < 0 ) {
        LOGE("I2C write8 error %d(%02x:%02x,%02x)\n", rc, client->addr, pI2CCmd->category, pI2CCmd->byte);
        rc = -EIO;
        return rc;
    }
    
    LOGI( "I2C Transfer OK %d write:[0x%x-0x%x] %02x\n", rc, pI2CCmd->category, pI2CCmd->byte, req_data[4] );

	return 0;
}

/*===========================================================================
    FUNCTION      CAMSENSOR_GPIOI2C_READ_NORMAL
===========================================================================*/
int camsensor_i2c_read_normal( struct i2c_client* client, struct CameraSensorI2CCmdType* pI2CCmd )
{

    int rc = 0;
    struct i2c_msg msg[2];

    unsigned char  read_data[256];
    unsigned char  req_data[256];
    int i = 0;

	if (!client->adapter) {
        LOGE( "- %s err:client->adapter is NULL\n", __func__ );
		return -ENODEV;
	}

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = pI2CCmd->wlen;
    for( i=0; i<pI2CCmd->wlen; i++ ) {
	    LOGI( "I2C Transfer_normal 0x%02x \n", pI2CCmd->pwdata[i] );
    	req_data[i] = pI2CCmd->pwdata[i];
    }
    msg[0].buf = req_data;

    
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = pI2CCmd->rlen;
    msg[1].buf = read_data;

    rc = i2c_transfer(client->adapter, msg, 2);
    if( rc < 0 ) {
        LOGE( "I2C Transfer Err %d Read(%02x,%02x)\n", rc, msg[1].addr, msg[1].len );
        rc = -EIO;
        return rc;
    }
    
	for( i=0; i<pI2CCmd->rlen; i++ ) {
    	pI2CCmd->prdata[i] = read_data[i];
    }
    
    LOGI( "I2C Transfer OK %d Read: 0x%02x\n", rc, pI2CCmd->prdata[0] );
	
    return 0;

}

/*===========================================================================
    FUNCTION      CAMSENSOR_GPIOI2C_WRITE_NORMAL
===========================================================================*/
int camsensor_i2c_write_normal( struct i2c_client* client, struct CameraSensorI2CCmdType* pI2CCmd )
{

    int rc = 0;
    struct i2c_msg msg;
    unsigned char req_data[256];
    int i = 0;

	if (!client->adapter) {
        LOGE( "- %s err:client->adapter is NULL\n", __func__ );
		return -ENODEV;
	}

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = pI2CCmd->wlen;  /* add address bytes */
    for( i=0; i<pI2CCmd->wlen; i++ ) {
	    LOGI( "I2C Transfer_normal 0x%02x \n", pI2CCmd->pwdata[i] );
    	req_data[i] = pI2CCmd->pwdata[i];
    }
    msg.buf = req_data;

    rc = i2c_transfer(client->adapter, &msg, 1);
    if( rc < 0 ) {
        LOGE("I2C write8 error %d(%02x,%02x)\n", rc, msg.addr, msg.len);
        rc = -EIO;
        return rc;
    }
    
    LOGI( "I2C Transfer OK %d write \n", rc );

	return 0;
}
