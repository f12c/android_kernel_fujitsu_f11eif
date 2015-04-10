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
    INCLUDE FILES FOR MODULE
============================================================================*/
#include <linux/clk.h>
#include <mach/clk.h>
#include <mach/gpio.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

/*===========================================================================
    LOCAL FUNCTIONS PROTOTYPES
============================================================================*/
/* FUJITSU:2011-04-14 add backlight-ctrl module --> */
#define BD6184_I2C_SLAVE_ADDR 0x76
static struct i2c_adapter *i2c_bkl;

typedef enum {
    BKL_STATE_OFF,
    BKL_STATE_1ST_ON,
    BKL_STATE_ON
} mddi_S6D16A2X01_bkl_state_e;

mddi_S6D16A2X01_bkl_state_e mddi_S6D16A2X01_bkl_state;
/* FUJITSU:2011-04-14 add backlight-ctrl module <-- */

#define MDDI_COMMAND_MAX_NUM    20

typedef struct {
    uint32 cmd;
    uint32 value;
} drv_mddi_cmd_table;

//Display Driver State
typedef enum {
    LCD_STATE_INIT,
    LCD_STATE_OFF,
    LCD_STATE_WAIT_UPDATE,
    LCD_STATE_WAIT_DISPLAY_ON,
    LCD_STATE_ON
} lcd_state_e;

//Dipslay Driver
typedef enum {
    LCD_OFF,
    LCD_ON
} lcd_info_e;

static void drv_display_control(lcd_info_e lcd_info);
static void drv_display_reset(void);
/* FUJITSU:2011-06-22 LCDPanelPowerControl start */
static void drv_display_reset_low(void);
static void drv_display_reset_high(void);
/* FUJITSU:2011-06-22 LCDPanelPowerControl end */
static void drv_display_poweron(void);
/* FUJITSU:2011-06-22 LCDPanelPowerControl start */
//static void drv_display_deep_standby_in(void);
//static void drv_display_deep_standby_out(void);
/* FUJITSU:2011-06-22 LCDPanelPowerControl end */
static void drv_display_mddi_cmd(drv_mddi_cmd_table *cmd_table);
/* FUJITSU:2011-06-22 LCDPanelPowerControl start */
static void drv_display_sleep_in(void);
//static void drv_display_sleep_out(void);
/* FUJITSU:2011-06-22 LCDPanelPowerControl end */

/* FUJITSU:2011-04-15 add sandstorm blocker --> */
static void drv_display_delay_on(struct work_struct *ignored);
static DECLARE_WORK(display_on_wq, drv_display_delay_on);
static void drv_display_displayon(void);
void mddi_panel_fullscrn_update_notify(void);
void mddi_panel_updatedone_notify(void);
/* FUJITSU:2011-04-15 add sandstorm blocker <-- */

static lcd_state_e lcd_state = LCD_STATE_INIT;

//command
static drv_mddi_cmd_table cmd_B2_PR[] =
{
{0xB2, 0x00000000},
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_B4[] =
{
{0xB4, 0x00000000},
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_B7[] =
{
{0xB7, 0x00111100},
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_B8[] =
{
{0xB8,  0x00002306},
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_2A[] =
{
{0x2A, 0xDF010000},
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_2B[] =
{
{0x2B, 0x1F030000},
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_F4[] =
{
/* FUJITSU:2011-05-17 LCDPanelSetting start */
//{0xF4, 0x004F2C30},
{0xF4, 0x004F2230},
/* FUJITSU:2011-05-17 LCDPanelSetting end */
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_F5[] =
{
{0xF5, 0x00000001},
{0xF5, 0x03030103},
{0xF5, 0x00000303},
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_F2[] =
{
{0xF2, 0x08038210},
{0xF2, 0xFF080808},
{0xF2, 0x00000003},
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_F3[] =
{
{0xF3, 0x00AC2000},
/* FUJITSU:2011-05-17 LCDPanelSetting start */
//{0xF3, 0x55555544},
{0xF3, 0x33334444},
/* FUJITSU:2011-05-17 LCDPanelSetting end */
{0xF3, 0x95921210},
{0xF3, 0x00000175},
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_F6[] =
{
{0xF6, 0x000000A1},
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_F7[] =
{
/* FUJITSU:2011-06-29 LCDPanelSetting start */
{0xF7, 0x8D000101},
{0xF7, 0x21050701},
{0xF7, 0x010C0105},
{0xF7, 0x00000006},
{0xFFFFFFFF, 0xFFFFFFFF}
/* FUJITSU:2011-06-29 LCDPanelSetting end */
};

static drv_mddi_cmd_table cmd_F8[] =
{
/* FUJITSU:2011-06-29 LCDPanelSetting start */
//{0xF8, 0x84000101},
//{0xF8, 0x1F040501},
{0xF8, 0x8D000101},
{0xF8, 0x21050701},
{0xF8, 0x010C0105},
{0xF8, 0x00000006},
{0xFFFFFFFF, 0xFFFFFFFF}
/* FUJITSU:2011-06-29 LCDPanelSetting end */
};

static drv_mddi_cmd_table cmd_F9[] =
{
{0xF9, 0x04030201},
{0xF9, 0x08070605},
{0xF9, 0x0C0B0A09},
{0xF9, 0x100F0E0D},
{0xF9, 0x000022C0},
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_FA[] =
{
/* FUJITSU:2011-06-29 LCDPanelSetting start */
//{0xFA, 0x292D350A},
//{0xFA, 0x2745544C},
//{0xFA, 0x272B3419},
//{0xFA, 0x0A0D1624},
//{0xFA, 0x3C292D35},
//{0xFA, 0x16223A3E},
//{0xFA, 0x24252832},
//{0xFA, 0x350A0D16},
//{0xFA, 0x2A2C292D},
//{0xFA, 0x2D0F1524},
//{0xFA, 0x101A1B22},
//{0xFA, 0x0000000D},
//{0xFFFFFFFF, 0xFFFFFFFF}
{0xFA, 0x292D350A},
{0xFA, 0x2C404646},
{0xFA, 0x3D393F21},
{0xFA, 0x0A102644},
{0xFA, 0x36292D35},
{0xFA, 0x1E27383A},
{0xFA, 0x443B363D},
{0xFA, 0x350A1026},
{0xFA, 0x26261C2D},
{0xFA, 0x38171A22},
{0xFA, 0x203C3332},
{0xFA, 0x00000010},
{0xFFFFFFFF, 0xFFFFFFFF}
/* FUJITSU:2011-06-29 LCDPanelSetting end */
};

static drv_mddi_cmd_table cmd_FB[] =
{
/* FUJITSU:2011-06-29 LCDPanelSetting start */
//{0xFB, 0x292D350A},
//{0xFB, 0x2745544C},
//{0xFB, 0x272B3418},
//{0xFB, 0x0A0D1624},
//{0xFB, 0x3C292D35},
//{0xFB, 0x15223A3E},
//{0xFB, 0x24252832},
//{0xFB, 0x350A0D16},
//{0xFB, 0x2A2C292D},
//{0xFB, 0x2D0E1524},
//{0xFB, 0x101A1B22},
//{0xFB, 0x0000000D},
//{0xFFFFFFFF, 0xFFFFFFFF}
{0xFB, 0x292D350A},
{0xFB, 0x2C404646},
{0xFB, 0x3D393F21},
{0xFB, 0x0A102644},
{0xFB, 0x36292D35},
{0xFB, 0x1E27383A},
{0xFB, 0x443B363D},
{0xFB, 0x350A1026},
{0xFB, 0x26261C2D},
{0xFB, 0x38171A22},
{0xFB, 0x203C3332},
{0xFB, 0x00000010},
{0xFFFFFFFF, 0xFFFFFFFF}
/* FUJITSU:2011-06-29 LCDPanelSetting end */
};

/* FUJITSU:2011-05-17 LCDPanelSetting start */
//Tearing effect line on
static drv_mddi_cmd_table cmd_35[] =
{
{0x35,  0x00000000},
{0xFFFFFFFF, 0xFFFFFFFF}
};
/* FUJITSU:2011-05-17 LCDPanelSetting end */

//password lock command
static drv_mddi_cmd_table cmd_B2_PL[] =
{
{0xB2,  0x00000003},
{0xFFFFFFFF, 0xFFFFFFFF}
};

//sleep out command and wait
static drv_mddi_cmd_table cmd_11[] =
{
{0x11,  0x00000000},
{0x00,  150},   //wait 120ms(min)   //CHECK:adjust wait time
{0xFFFFFFFF, 0xFFFFFFFF}
};

/* FUJITSU:2011-05-17 LCDPanelSetting start */
//sleep in command and wait
static drv_mddi_cmd_table cmd_10[] =
{
{0x10,  0x00000000},
{0x00,  52},   //wait 51ms(min)   //CHECK:adjust wait time
{0xFFFFFFFF, 0xFFFFFFFF}
};
/* FUJITSU:2011-05-17 LCDPanelSetting end */

//display on command
static drv_mddi_cmd_table cmd_29[] =
{
{0x29,  0x00000000},
{0xFFFFFFFF, 0xFFFFFFFF}
};

/* FUJITSU:2011-05-17 LCDPanelSetting start */
//display off command
static drv_mddi_cmd_table cmd_28[] =
{
{0x28,  0x00000000},
{0xFFFFFFFF, 0xFFFFFFFF}
};
/* FUJITSU:2011-05-17 LCDPanelSetting end */

/* FUJITSU:2011-06-22 LCDPanelPowerControl start */
#if 0
//deep stand by in command
static drv_mddi_cmd_table cmd_BB_DSTBIN[] =
{
{0xBB,  0x00000080},
{0x00,  120},   //wait 100ms(min) //CHECK:adjust wait time
{0xFFFFFFFF, 0xFFFFFFFF}
};

//deep stand by register initialize command
static drv_mddi_cmd_table cmd_BB_DSTBINIT[] =
{
{0xBB,  0x00000000},
{0xFFFFFFFF, 0xFFFFFFFF}
};
#endif
/* FUJITSU:2011-06-22 LCDPanelPowerControl end */

static void drv_display_mddi_cmd(drv_mddi_cmd_table *cmd_table)
{
    uint32 i = 0;
    uint32 value[MDDI_COMMAND_MAX_NUM] = {0};
    uint32 cmd;
    
    int ret = 0;
    
    cmd = cmd_table[i].cmd;
    
    while (cmd_table[i].cmd != 0xFFFFFFFF || cmd_table[i].value != 0xFFFFFFFF ) {
        if(i >= MDDI_COMMAND_MAX_NUM){
            printk(KERN_ERR "[DD]%s: command buffer overflow cmd=(0x%08X)\n", __func__, cmd_table[i].cmd);
            break;
        }
        if (cmd_table[i].cmd == 0 && cmd_table[i].value) {
            mddi_wait(cmd_table[i].value);
        }else if(cmd == cmd_table[i].cmd && cmd == cmd_table[i+1].cmd){
            value[i] = cmd_table[i].value;
        }else{
            value[i] = cmd_table[i].value;
            ret = mddi_host_register_multiwrite(cmd_table[i].cmd, value, i+1, TRUE, NULL, MDDI_HOST_PRIM);
            if (ret != 0) {
                printk(KERN_ERR "[LCD]%s: mddi_host_register_multiwrite() cmd=0x%08X, value[%d]=0x%08X, count=%d  ret=%d\n", __func__, cmd_table[i].cmd, i, value[i], i+1, ret);
            }
        }
        cmd = cmd_table[i].cmd;
        i++;
    }
    return;
}

/* FUJITSU:2011-04-15 add sandstorm blocker --> */
void mddi_panel_fullscrn_update_notify(void)
{
    if (lcd_state == LCD_STATE_WAIT_UPDATE) {
        printk(KERN_INFO "[LCD]%s FullScreen Update..\n",__func__);
        lcd_state = LCD_STATE_WAIT_DISPLAY_ON;
    }
}

void mddi_panel_updatedone_notify(void)
{
    if (lcd_state == LCD_STATE_WAIT_DISPLAY_ON) {
        printk(KERN_INFO "[LCD]%s FullScreen Updated! set work queue.\n",__func__);
        schedule_work(&display_on_wq);
    }
}


static void drv_display_delay_on(struct work_struct *ignored)
{
    if (lcd_state == LCD_STATE_WAIT_DISPLAY_ON) {
        drv_display_displayon();
        printk(KERN_INFO "[LCD]%s Display ON\n",__func__);
        lcd_state = LCD_STATE_ON;
        //wait 20ms
        mddi_wait(20);
    }
    else {
        printk(KERN_ERR "%s: lcd_state (%d).\n", __func__, lcd_state);
    }
}

static void drv_display_displayon(void)
{
    drv_display_mddi_cmd(cmd_29);
}
/* FUJITSU:2011-04-15 add sandstorm blocker <-- */

static void drv_display_reset(void)
{
    
    //gpio init
    gpio_tlmm_config(GPIO_CFG(33, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    
    //Reset
    gpio_set_value(33, 0);
    mddi_wait(15);  //min 10ms  //CHECK:adjust wait time
    gpio_set_value(33, 1);
    mddi_wait(15);  //min 10ms  //CHECK:adjust wait time
    
}

/* FUJITSU:2011-06-22 LCDPanelPowerControl start */
static void drv_display_reset_low(void)
{
    //gpio init
    gpio_tlmm_config(GPIO_CFG(33, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    
    gpio_set_value(33, 0);
    
}

static void drv_display_reset_high(void)
{
    //gpio init
    gpio_tlmm_config(GPIO_CFG(33, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    
    mddi_wait(1);  //min 10us
    gpio_set_value(33, 1);
    mddi_wait(15);
}
/* FUJITSU:2011-06-22 LCDPanelPowerControl end */

static void drv_display_poweron(void)
{
    
    //panel reset
/* FUJITSU:2011-06-22 LCDPanelPowerControl start */
    //drv_display_reset();
/* FUJITSU:2011-06-22 LCDPanelPowerControl end */
    
    //LCD Command start
    drv_display_mddi_cmd(cmd_B2_PR);
    drv_display_mddi_cmd(cmd_B4);
    drv_display_mddi_cmd(cmd_B7);
    drv_display_mddi_cmd(cmd_B8);
    drv_display_mddi_cmd(cmd_2A);
    drv_display_mddi_cmd(cmd_2B);
    
    drv_display_mddi_cmd(cmd_F4);
    drv_display_mddi_cmd(cmd_F5);
    drv_display_mddi_cmd(cmd_F2);
    drv_display_mddi_cmd(cmd_F3);
    drv_display_mddi_cmd(cmd_F6);
    drv_display_mddi_cmd(cmd_F7);
    drv_display_mddi_cmd(cmd_F8);
    drv_display_mddi_cmd(cmd_F9);
    drv_display_mddi_cmd(cmd_FA);
    drv_display_mddi_cmd(cmd_FB);
    
/* FUJITSU:2011-05-17 LCDPanelSetting start */
    drv_display_mddi_cmd(cmd_35);
/* FUJITSU:2011-05-17 LCDPanelSetting end */
    drv_display_mddi_cmd(cmd_B2_PL);
    drv_display_mddi_cmd(cmd_11);
/* FUJITSU:2011-06-22 LCDPanelPowerControl start */
//    drv_display_mddi_cmd(cmd_29);
/* FUJITSU:2011-06-22 LCDPanelPowerControl end */
    
}

/* FUJITSU:2011-05-17 LCDPanelSetting start */
static void drv_display_sleep_in(void)
{
    drv_display_mddi_cmd(cmd_28);
    drv_display_mddi_cmd(cmd_10);
}
/* FUJITSU:2011-05-17 LCDPanelPowerControl end */

/* FUJITSU:2011-06-22 LCDPanelPowerControl start */
#if 0
static void drv_display_deep_standby_in(void)
{
    drv_display_mddi_cmd(cmd_B2_PR);
    drv_display_mddi_cmd(cmd_BB_DSTBIN);
}
#endif

#if 0
static void drv_display_sleep_out(void)
{
    drv_display_mddi_cmd(cmd_B2_PR);
    drv_display_mddi_cmd(cmd_B4);
    drv_display_mddi_cmd(cmd_B7);
    drv_display_mddi_cmd(cmd_B8);
    drv_display_mddi_cmd(cmd_2A);
    drv_display_mddi_cmd(cmd_2B);
    
    drv_display_mddi_cmd(cmd_F4);
    drv_display_mddi_cmd(cmd_F5);
    drv_display_mddi_cmd(cmd_F2);
    drv_display_mddi_cmd(cmd_F3);
    drv_display_mddi_cmd(cmd_F6);
    drv_display_mddi_cmd(cmd_F7);
    drv_display_mddi_cmd(cmd_F8);
    drv_display_mddi_cmd(cmd_F9);
    drv_display_mddi_cmd(cmd_FA);
    drv_display_mddi_cmd(cmd_FB);
    
    drv_display_mddi_cmd(cmd_35);
    drv_display_mddi_cmd(cmd_B2_PL);
    drv_display_mddi_cmd(cmd_11);
}
#endif

#if 0
static void drv_display_deep_standby_out(void)
{
    
    drv_display_reset();
    drv_display_mddi_cmd(cmd_B2_PR);
    drv_display_mddi_cmd(cmd_BB_DSTBINIT);
/* FUJITSU:2011-05-17 LCDPanelSetting start */
#if 0	//	20110516:remove
    drv_display_mddi_cmd(cmd_B4);
    drv_display_mddi_cmd(cmd_B7);
    drv_display_mddi_cmd(cmd_B8);
    
    drv_display_mddi_cmd(cmd_F4);
    drv_display_mddi_cmd(cmd_F5);
    drv_display_mddi_cmd(cmd_F2);
    drv_display_mddi_cmd(cmd_F3);
    drv_display_mddi_cmd(cmd_F6);
    drv_display_mddi_cmd(cmd_F7);
    drv_display_mddi_cmd(cmd_F8);
    drv_display_mddi_cmd(cmd_F9);
    drv_display_mddi_cmd(cmd_FA);
    drv_display_mddi_cmd(cmd_FB);
    
    drv_display_mddi_cmd(cmd_B2_PL);
    drv_display_mddi_cmd(cmd_11);
/* FUJITSU:2011-04-15 add sandstorm blocker --> */
//    drv_display_mddi_cmd(cmd_29); display_on delete
/* FUJITSU:2011-04-15 add sandstorm blocker --> */
#else
    drv_display_mddi_cmd(cmd_B2_PL);
#endif	//	20110516:remove
/* FUJITSU:2011-05-17 LCDPanelPowerControl end */
    
}
#endif
/* FUJITSU:2011-06-22 LCDPanelPowerControl end */

static void drv_display_control(lcd_info_e lcd_info)
{
    printk(KERN_DEBUG "[LCD]%s(%d): enter state=%d\n",__func__,lcd_info,lcd_state);
    
    switch (lcd_state) {
        case LCD_STATE_INIT:
/* FUJITSU:2011-06-06 delete for boot seq --> */
            if (system_rev >= 0x0c) {
                printk(KERN_DEBUG "[LCD]%s: power_on\n",__func__);
/* FUJITSU:2011-06-22 LCDPanelPowerControl start */
				drv_display_reset();
/* FUJITSU:2011-06-22 LCDPanelPowerControl end */
                drv_display_poweron();
	            lcd_state = LCD_STATE_WAIT_UPDATE;
            }
/* FUJITSU:2011-06-06 delete for boot seq <-- */
/* FUJITSU:2011-06-22 LCDPanelPowerControl start */
			else
			{
    	        lcd_state = LCD_STATE_ON;
			}
/* FUJITSU:2011-06-22 LCDPanelPowerControl end */
            break;
        case LCD_STATE_OFF:
            if (lcd_info == LCD_ON) {
/* FUJITSU:2011-06-22 resume problem start */
				mddi_host_client_cnt_reset();
/* FUJITSU:2011-06-22 resume problem end */
/* FUJITSU:2011-06-22 LCDPanelPowerControl start */
                //drv_display_deep_standby_out();
				drv_display_reset_high();
	            drv_display_poweron();
				//drv_display_sleep_out();
/* FUJITSU:2011-06-22 LCDPanelSetting end */
/* FUJITSU:2011-04-15 add sandstorm blocker --> */
//            lcd_state = LCD_STATE_ON;
                lcd_state = LCD_STATE_WAIT_UPDATE;
/* FUJITSU:2011-04-15 add sandstorm blocker --> */
            }else{
                //NOP
            }
            break;
        case LCD_STATE_ON:
            if (lcd_info == LCD_OFF) {
/* FUJITSU:2011-07-01 resume problem start */
				mddi_host_client_cnt_reset();
/* FUJITSU:2011-07-01 resume problem end */
/* FUJITSU:2011-06-22 LCDPanelPowerControl start */
				drv_display_sleep_in();
				drv_display_reset_low();
                //drv_display_deep_standby_in();
/* FUJITSU:2011-06-22 LCDPanelPowerControl end */
                lcd_state = LCD_STATE_OFF;
            }else{
                //NOP
            }
            break;
        case LCD_STATE_WAIT_DISPLAY_ON:
        case LCD_STATE_WAIT_UPDATE:
        default:
            break;
    }
    printk(KERN_DEBUG "[LCD]%s: leave state=%d\n",__func__, lcd_state);
}

static int mddi_lcd_on(struct platform_device *pdev)
{
    drv_display_control(LCD_ON);
    return 0;
}

static int mddi_lcd_off(struct platform_device *pdev)
{
    drv_display_control(LCD_OFF);
    return 0;
}

static int __devinit mddi_lcd_probe(struct platform_device *pdev)
{
    printk(KERN_DEBUG "[LCD]%s: enter\n", __func__);
    msm_fb_add_device(pdev);
    printk(KERN_DEBUG "[LCD]%s: leave\n", __func__);
    return 0;
}

/* FUJITSU:2011-04-14 add backlight-ctrl module --> */
static int _mddi_S6D16A2X01_BD6184_i2c_write(unsigned char addr,unsigned char data)
{
    struct i2c_msg msg;
    u_int8_t buf[8];
    int ret = 0;
    int try = 0;/* FUJITSU:2011-08-25 i2c-retry */

    msg.addr  = BD6184_I2C_SLAVE_ADDR;
    msg.buf   = buf;
    msg.len   = 2;
    msg.flags = 0;
    
    buf[0] = addr;
    buf[1] = data;
    
    ret = i2c_transfer(i2c_bkl, &msg, 1);
    if (ret < 0) {
        printk(KERN_ERR "[BKL]%s I2C(addr:%x,data:%x) ERROR ret = %d\n",__func__,addr,data,ret);
/* FUJITSU:2011-08-25 i2c-retry start */
        for (try = 0; try < 50; try++) {
            msleep(5);
            ret = i2c_transfer(i2c_bkl, &msg, 1);
            if (ret >= 0 ) {
                printk(KERN_ERR "[BKL]%s I2C(addr:%x,data:%x) ERROR ret = %d retry(%d):OK\n",__func__,addr,data,ret,try);
                break;
            }
        }
        if(ret < 0){
            printk(KERN_ERR "[BKL]%s I2C(addr:%x,data:%x) ERROR ret = %d retry(%d):NG\n",__func__,addr,data,ret,try);
        }
/* FUJITSU:2011-08-25 i2c-retry end */
    }
    else {
        /* I2C transfer successful. return success(0) */
        ret = 0;
    }
    
    return ret;
}

static void set_backlight(struct msm_fb_data_type *mfd)
{
    int ret = 0;
    int32 level = 0;
    
    level = mfd->bl_level;
    
    if(level) {
        /* BKL ON */
        if(mddi_S6D16A2X01_bkl_state == BKL_STATE_OFF) {
            //initial configure
            ret |= _mddi_S6D16A2X01_BD6184_i2c_write(0x01,0x0F);
            
            /* NOTICE : light turn on sharply at 1st time */
            ret |= _mddi_S6D16A2X01_BD6184_i2c_write(0x09,0x00); //TLH set sharply
            
            /* Intensity */
            ret |= _mddi_S6D16A2X01_BD6184_i2c_write(0x03,level);
            
            /* MLED Power On */
            ret |= _mddi_S6D16A2X01_BD6184_i2c_write(0x02,0x01);
            
            if(ret != 0) {
                printk(KERN_ERR "[BKL]:I2C set failed. state change skipped.\n");
            }
            else {
                mddi_S6D16A2X01_bkl_state = BKL_STATE_1ST_ON;
                printk(KERN_INFO "[BKL]%s(%d) Backlight turn ON.\n",__func__,level);
            }
        }
        else {
            if(mddi_S6D16A2X01_bkl_state == BKL_STATE_1ST_ON){
                /* 2nd time. slope reset */
/* FUJITSU:2011-06-23 change slope time settings start */
                _mddi_S6D16A2X01_BD6184_i2c_write(0x09,0x66);
//                _mddi_S6D16A2X01_BD6184_i2c_write(0x09,0x00);
/* FUJITSU:2011-06-23 change slope time settings end */
                mddi_S6D16A2X01_bkl_state = BKL_STATE_ON;
            }
            /* Intensity */
            _mddi_S6D16A2X01_BD6184_i2c_write(0x03,level);
        }
    }
    else {
        /* BKL OFF */
        
        /* MLED PowerOff */
        _mddi_S6D16A2X01_BD6184_i2c_write(0x02,0x00);
        
        mddi_S6D16A2X01_bkl_state = BKL_STATE_OFF;
        printk(KERN_INFO "[BKL]%s(%d) Backlight turn OFF.\n",__func__,level);
    }
    
    return;
}
/* FUJITSU:2011-04-14 add backlight-ctrl module <-- */

static struct platform_driver this_driver = {
    .probe  = mddi_lcd_probe,
    .shutdown = NULL,
    .driver = {
        .name   = "mddi_S6D16A2X01",
    },
};

static struct msm_fb_panel_data mddi_S6D16A2X01_panel_data = {
    .on   = mddi_lcd_on,
    .off  = mddi_lcd_off,
    .set_backlight  = set_backlight
};

static struct platform_device this_device = {
    .name   = "mddi_S6D16A2X01",
    .id  = 0,
    .dev  = {
        .platform_data = &mddi_S6D16A2X01_panel_data,
    }
};

static int __init mddi_S6D16A2X01_init(void)
{
    int ret;
    struct msm_panel_info *pinfo;
    
    printk(KERN_DEBUG "[LCD]%s: enter\n", __func__);
    
    lcd_state = LCD_STATE_INIT;
    
/* FUJITSU:2011-04-14 add backlight-ctrl module --> */
/* FUJITSU:2011-05-11 add i2c adapter chg --> */
    if (system_rev >= 0x0c) { //CHECK:
        /* use QUP-I2C */
        i2c_bkl = i2c_get_adapter(4);
        printk(KERN_DEBUG "[LCD]%s: i2c_adapter QUP\n",__func__);
    }
    else {
        /*  */
        i2c_bkl = i2c_get_adapter(0);
        printk(KERN_DEBUG "[LCD]%s: i2c_adapter Legacy\n",__func__);
    }
/* FUJITSU:2011-05-11 add i2c adapter chg <-- */
    if (!i2c_bkl) {
        printk(KERN_ERR "[LCD]%s i2c_get_adapter(4) failure.\n",__func__);
    }
    mddi_S6D16A2X01_bkl_state = BKL_STATE_OFF;
/* FUJITSU:2011-04-14 add backlight-ctrl module <-- */
    
    ret = platform_driver_register(&this_driver);
    
    if (!ret) {
        pinfo = &mddi_S6D16A2X01_panel_data.panel_info;
        pinfo->xres                       = 480;
        pinfo->yres                       = 800;
        pinfo->type                       = MDDI_PANEL;
        pinfo->pdest                      = DISPLAY_1;
        pinfo->mddi.vdopkt                = MDDI_DEFAULT_PRIM_PIX_ATTR;
        pinfo->wait_cycle                 = 0;
        pinfo->bpp                        = 24;
        pinfo->fb_num                     = 2;
        pinfo->clk_rate                   = 192000000;
        pinfo->clk_min                    = 192000000;
        pinfo->clk_max                    = 192000000;
/* FUJITSU:2011-05-17 LCDPanelSetting start */
        pinfo->lcd.vsync_enable           = TRUE;
/* FUJITSU:2011-05-17 LCDPanelSetting end */
/* FUJITSU:2011-06-29 LCDPanelSetting start */
        //pinfo->lcd.refx100                = 6322;
        pinfo->lcd.refx100                = 6740;
/* FUJITSU:2011-06-29 LCDPanelSetting end */
        pinfo->lcd.v_back_porch           = 6;
        pinfo->lcd.v_front_porch          = 8;
        pinfo->lcd.v_pulse_width          = 2;
/* FUJITSU:2011-05-17 LCDPanelSetting start */
        pinfo->lcd.hw_vsync_mode          = TRUE;
/* FUJITSU:2011-05-17 LCDPanelSetting end */
        pinfo->lcd.vsync_notifier_period  = 0;
        pinfo->lcd.rev                    = 2;
        pinfo->bl_min                     = 1;
//        pinfo->bl_max                     = 99; * FUJITSU:2011-05-30 change backlight max  */
        pinfo->bl_max                     = 128;
        
        ret = platform_device_register(&this_device);
        if (ret) {
            platform_driver_unregister(&this_driver);
        }
    }
    printk(KERN_INFO "[LCD]%s: leave(%d)\n", __func__,ret);
    return ret;
}

module_init(mddi_S6D16A2X01_init);
