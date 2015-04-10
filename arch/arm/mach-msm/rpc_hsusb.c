/* linux/arch/arm/mach-msm/rpc_hsusb.c
 *
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#include <linux/err.h>
#include <mach/rpc_hsusb.h>
#include <asm/mach-types.h>
/* FUJITSU:2011-04-20 USB start */
#include <linux/i2c.h>
#include <linux/leds-pmic8058.h>
/* FUJITSU:2011-04-20 USB end */

static struct msm_rpc_endpoint *usb_ep;
static struct msm_rpc_endpoint *chg_ep;

#define MSM_RPC_CHG_PROG 0x3000001a
/* FUJITSU:2011-04-20 USB start */
#define TCHARGE_I2C_SLAVE_ADDR                    0x37
#define TCHARGE_MAX_ERR_COUNT                     5    /* 5 times */
#define TCHARGE_OUT( addr, data ) tcharge_out( addr, data );
#define TCHARGE_IN( addr ) tcharge_in( addr );
void tcharge_hw_init_temp( void );
extern int msm_hsusb_chg_status(void);
extern void msm_hsusb_chg_set_status(int chg);
typedef void (*msm_batt_cbfunc)(void);
static msm_batt_cbfunc msm_batt_update_status_cb_hsusb = NULL;
////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---------------------------------------------------------------------------
  Charge (output) current - (address: 00h) - Non-Volatile & Volatile(mirror)
---------------------------------------------------------------------------*/
#define TCHARGE_CHG_CURR_ADDR                     0x00 /* address */

/* bit[7-5]: Fast Charge Current */
#define TCHARGE_FAST_CURR_M                       0xE0 /* mask */
#define TCHARGE_FAST_450MA_V                      0x00 /* 450mA */
#define TCHARGE_FAST_550MA_V                      0x20 /* 550mA */
#define TCHARGE_FAST_650MA_V                      0x40 /* 650mA */
#define TCHARGE_FAST_750MA_V                      0x60 /* 750mA */
#define TCHARGE_FAST_850MA_V                      0x80 /* 850mA */
#define TCHARGE_FAST_950MA_V                      0xA0 /* 950mA */
#define TCHARGE_FAST_1050MA_V                     0xC0 /* 1050mA */
#define TCHARGE_FAST_1150MA_V                     0xE0 /* 1150mA */

/* bit[4:3]: Pre-Charge Charge Current */
#define TCHARGE_PRE_CHARGE_CURR_M                 0x18 /* mask */
#define TCHARGE_PRE_50MA_V                        0x00 /* 50mA */
#define TCHARGE_PRE_100MA_V                       0x08 /* 100mA */
#define TCHARGE_PRE_150MA_V                       0x10 /* 150mA */
#define TCHARGE_PRE_200MA_V                       0x18 /* 200mA */

/* bit[2:1]: Termination Current */
#define TCHARGE_TERMINATION_CURR_M                0x06 /* mask */
#define TCHARGE_TERMINATION_50MA_V                0x00 /* 50mA */
#define TCHARGE_TERMINATION_100MA_V               0x02 /* 100mA */
#define TCHARGE_TERMINATION_150MA_V               0x04 /* 150mA */
#define TCHARGE_TERMINATION_200MA_V               0x06 /* 200mA */

/* bit[0]: Trickle-Charge Safety Timer */
#define TCHARGE_TRICKLE_ST_M                      0x01 /* mask */
#define TCHARGE_TRICKLE_ST_DIS_V                  0x00 /* Neither the pre-charge nor the complete-charge timers 
                                                          tun during trickle charging (Vbatt<2.1) */
#define TCHARGE_TRICKLE_ST_ENA_V                  0x01 /* Both timers begin running when charging is enabled and 
                                                          input voltage is valid, independent of battery voltage */

/*---------------------------------------------------------------------------
  Input current limit - (address: 01h) - Non-Volatile & Volatile(mirror)
---------------------------------------------------------------------------*/
#define TCHARGE_INPUT_LIMIT_ADDR                  0x01 /* address */

/* bit[7-5]: AC Input Current Limit */
#define TCHARGE_AC_LIMIT_M                        0xE0 /* mask */
#define TCHARGE_AC_300MA_V                        0x00 /* 300mA */
#define TCHARGE_AC_550MA_V                        0x20 /* 550mA */
#define TCHARGE_AC_650MA_V                        0x40 /* 650mA */
#define TCHARGE_AC_750MA_V                        0x60 /* 750mA */
#define TCHARGE_AC_850MA_V                        0x80 /* 850mA */
#define TCHARGE_AC_950MA_V                        0xA0 /* 950mA */
#define TCHARGE_AC_1050MA_V                       0xC0 /* 1050mA */
#define TCHARGE_AC_1150MA_V                       0xE0 /* 1150mA */

/* bit[4]: USB Input Current Limit */
#define TCHARGE_USB_LIMIT_M                       0x10 /* mask */
#define TCHARGE_USB_RESERVED_V                    0x00 /* Reserved */
#define TCHARGE_90_475MA_V                        0x10 /* 90/475mA */

/* bit[3]: Reserved */

/* bit[2]: Automatic Input Current Limit */
#define TCHARGE_AUTO_LIMIT_M                      0x04 /* mask */
#define TCHARGE_AUTO_DISABLE_V                    0x04 /* Disable */
#define TCHARGE_AUTO_ENABLE_V                     0x00 /* Enable */

/* bit[1:0]: Automatic Input Current Limit Threshold */
#define TCHARGE_CURR_THRES_M                      0x03 /* mask */
#define TCHARGE_THRES_4P25V_V                     0x00 /* 4.25V */
#define TCHARGE_THRES_4P50V_V                     0x01 /* 4.50V */
#define TCHARGE_THRES_4P75V_V                     0x02 /* 4.75V */
#define TCHARGE_THRES_5P00V_V                     0x03 /* 5.00V */

/*---------------------------------------------------------------------------
  Float Voltage - (address: 02h) - Non-Volatile & Volatile(mirror)
---------------------------------------------------------------------------*/
#define TCHARGE_FLOAT_VOLTAGE_ADDR                0x02 /* address */

/* bit[7]: Internal (IC) Temperature limit */
#define TCHARGE_TEMP_LIMIT_M                      0x80 /* mask */
#define TCHARGE_TEMP_ACTIVE_V                     0x00 /* Status bit active only */
#define TCHARGE_TEMP_SUSPENTION_V                 0x80 /* Status bit active and charge suspension */

/* bit[6-0]: Float Voltage(V) */
#define TCHARGE_FLOAT_VOLTAGE_M                   0x7F /* mask */
/* setting rule of Float Voltage
        (lower limit)
        3.46V ... 0x00
        3.46V ... 0x01
        3.48V ... 0x02
        3.48V ... 0x03
        3.50V ... 0x04
        3.50V ... 0x05
         ...
        4.20V ... 0x4A
        4.20V ... 0x4B
        4.22V ... 0x4C
        4.22V ... 0x4D
         ...
        4.70V ... 0x7C
        4.70V ... 0x7D
        4.72V ... 0x7E
        4.72V ... 0x7F
        (higher limit)
*/

/*---------------------------------------------------------------------------
  Control Register A - (address: 03h) - Non-Volatile & Volatile(mirror)
---------------------------------------------------------------------------*/
#define TCHARGE_CONT_REG_A_ADDR                   0x03 /* address */

/* bit[7]: Automatic Recharge */
#define TCHARGE_AUTO_RECHARGE_M                   0x80 /* mask */
#define TCHARGE_RECHARGE_ENABLE_V                 0x00 /* Enabled */
#define TCHARGE_RECHARGE_DISABLE_V                0x80 /* Disable */

/* bit[6]: Current Termination */
#define TCHARGE_CURR_TERMINATION_M                0x40 /* mask */
#define TCHARGE_TERMINATE_ALLOWED_V               0x00 /* Allowed to end a charge cycle */
#define TCHARGE_TERMINATE_NOTALLOWED_V            0x40 /* Not allowed to end a charge cycle */

/* bit[5-3]: Pre-charge to Fast-charge Voltage Threshold */
#define TCHARGE_PREFAST_VOLTAGE_M                 0x38 /* mask */
#define TCHARGE_PREFASH_2P2V_V                    0x00 /* 2.2V */
#define TCHARGE_PREFASH_2P4V_V                    0x08 /* 2.4V */
#define TCHARGE_PREFASH_2P5V_V                    0x10 /* 2.5V */
#define TCHARGE_PREFASH_2P6V_V                    0x18 /* 2.6V */
#define TCHARGE_PREFASH_2P9V_V                    0x20 /* 2.9V */
#define TCHARGE_PREFASH_3P0V_V                    0x28 /* 3.0V */
#define TCHARGE_PREFASH_3P1V_V                    0x30 /* 3.1V */
#define TCHARGE_PREFASH_3P4V_V                    0x38 /* 3.4V */

/* bit[2]: SYSON UVLO */
#define TCHARGE_SYSON_UVLO_M                      0x04 /* mask */
#define TCHARGE_UVLO_3P50V_V                      0x00 /* SYSON UVLO = DCIN UVLO = 3.5V(typ.) */
#define TCHARGE_UVLO_4P75V_V                      0x04 /* SYSON UVLO = 4.75V(typ.) */

/* bit[1]: Automatic Power Source Detection */
#define TCHARGE_AUTOPWR_DETECT_M                  0x02 /* mask */
#define TCHARGE_AUTOPWR_DISABLE_V                 0x02 /* Disable */
#define TCHARGE_AUTOPWR_ENABLE_V                  0x00 /* Enable */

/* bit[0]: Trickle Charging */
#define TCHARGE_TRICKLE_CHARGE_M                  0x01 /* mask */
#define TCHARGE_TRICKLE_ENABLE_V                  0x00 /* Enables trickle charging */
#define TCHARGE_TRICKLE_DISABLE_V                 0x01 /* Disable trickle charging */

/*---------------------------------------------------------------------------
  Control Register B - (address: 04h) - Non-Volatile & Volatile(mirror)
---------------------------------------------------------------------------*/
#define TCHARGE_CONT_REG_B_ADDR                   0x04 /* address */

/* bit[7]: STAT Output Mode */
#define TCHARGE_STAT_MODE_M                       0x80 /* mask */
#define TCHARGE_STAT_CHARGING_V                   0x00 /* STAT Output indicates charging state */
#define TCHARGE_STAT_UVOV_V                       0x80 /* STAT Output indicates input UV/OV condition */

/* bit[6]: Battery OV */
#define TCHARGE_BATTERY_OV_M                      0x40 /* mask */
#define TCHARGE_OV_NOTCAUSE_V                     0x00 /* Battery OV does not causes charge cycle to end */
#define TCHARGE_OV_CAUSE_V                        0x40 /* Battery OV does cause charge cycle to end */

/* bit[5]: Not used */

/* bit[4]: Pre-charge to fast-charge Threshold */
#define TCHARGE_PREFAST_THRESH_M                  0x10 /* mask */
#define TCHARGE_PREFASH_ENABLE_V                  0x00 /* Enabled */
#define TCHARGE_PREFASH_DISABLE_V                 0x10 /* Disabled */

/* bit[3]: Charge Safety Timer */
#define TCHARGE_CHARGE_SFTIMER_M                  0x08 /* mask */
#define TCHARGE_SFTIMER_ENABLE_V                  0x00 /* Enabled */
#define TCHARGE_SFTIMER_DISABLE_V                 0x08 /* Disabled */

/* bit[2]: OTG Watchdog Timer */
#define TCHARGE_OTG_WDTIMER_M                     0x04 /* mask */
#define TCHARGE_OTGWD_DISABLE_V                   0x00 /* Disabled */
#define TCHARGE_OTGWD_ENABLE_V                    0x04 /* Enabled */

/* bit[1]: Charge Watchdog Timer */
#define TCHARGE_CHARGE_WDTIMER_M                  0x02 /* mask */
#define TCHARGE_CHGWD_DISABLE_V                   0x00 /* Disabled */
#define TCHARGE_CHGWD_ENABLE_V                    0x02 /* Enabled */

/* bit[0]: Interrupt(IRQ) Output */
#define TCHARGE_IRQ_OUTPUT_M                      0x01 /* mask */
#define TCHARGE_IRQ_DISABLE_V                     0x00 /* Disabled */
#define TCHARGE_IRQ_ENABLE_V                      0x01 /* Enabled */

/*---------------------------------------------------------------------------
  Pin Control - (address: 05h) - Non-Volatile & Volatile(mirror)
---------------------------------------------------------------------------*/
#define TCHARGE_PIN_CONT_ADDR                     0x05 /* address */

/* bit[7]: Reserved */

/* bit[6]: Automatic Charging Control */
#define TCHARGE_AUTO_CHARGE_M                     0x40 /* mask */
#define TCHARGE_AUTOCHG_NOTENABLE_V               0x00 /* Automatic charging is not enable; bits[3:2] control charging */
#define TCHARGE_AUTOCHG_ENABLE_V                  0x40 /* Automatic charging is enabled regardless of bits[3:2] */

/* bit[5]: "Dead-battery" Voltage Threshold */
#define TCHARGE_DEADBATT_THRESH_M                 0x20 /* mask */
#define TCHARGE_DEADBATT_3P47V_V                  0x00 /* AUXPWR < 3.47V */
#define TCHARGE_DEADBATT_3P54V_V                  0x20 /* AUXPWR < 3.54V */

/* bit[4]: Safety Timer Operation */
#define TCHARGE_SFTIMER_OPERAT_M                  0x10 /* mask */
#define TCHARGE_SFTIMER_SETTINGS_V                0x00 /* 36- and 240-minute safety timers active when WD timer is off and
                                                        when permanent operation is selected, "0" in 04[3] */
#define TCHARGE_SFTIMER_NOSET_V                   0x10 /* 36--minute safety timers active when device not in SW control */

/* bit[3:2]: Enable (EN) Control */
#define TCHARGE_EN_CONTROL_M                      0x0C /* mask */
#define TCHARGE_EN_CTRL0_V                        0x00 /* "0" in 31[4] turns on (enables) charger */
#define TCHARGE_EN_CTRL1_V                        0x04 /* "0" in 31[4] turns off (disables) charger */
#define TCHARGE_EN_ACTLOW_V                       0x08 /* EN pin activate low; Reg 31[4] has no effect */
#define TCHARGE_EN_ACTHIGH_V                      0x0C /* EN pin activate high; Reg 31[4] has no effect */

/* bit[1:0]: OTG and USB5/1/AC Pin Control */
#define TCHARGE_USB51AC_PINCONTROL_M              0x03 /* mask */
#define TCHARGE_USB51AC_I2CREQ_V                  0x00 /* Requires I2C - Pin has no effect USB level determined by 31[3:2]
                                                          OTG mode controlled by 31[1] */
#define TCHARGE_USB51AC_PINCONT_V                 0x01 /* Pin controls USB5/1/AC level "0"=100mA, "1"=500mA, floating=AC
                                                          OTG mode controlled by 31[1] */
#define TCHARGE_USB51AC_OTGLO_V                   0x02 /* Pin controls OTG function - active low
                                                          USB level determined by 31[3:2] */
#define TCHARGE_USB51AC_OTGHI_V                   0x03 /* Pin controls OTG function - active high
                                                          USB level determined by 31[3:2] */

/*---------------------------------------------------------------------------
  OTG Control - (address: 06h) - Non-Volatile & Volatile(mirror)
---------------------------------------------------------------------------*/
#define TCHARGE_OTG_CONT_ADDR                     0x06 /* address */

/* bit[7]: Battery Missing Detection */
#define TCHARGE_BATT_DETECT_M                     0x80 /* mask */
#define TCHARGE_BATT_ENABLE_V                     0x80 /* Enabled */
#define TCHARGE_BATT_DISABLE_V                    0x00 /* Disabled */

/* bit[6]: Automatic Recharge Threshold */
#define TCHARGE_RECHARGE_THRESH_M                 0x40 /* mask */
#define TCHARGE_RECHARGE_70MV_V                   0x00 /* 70mV */
#define TCHARGE_RECHARGE_140MV_V                  0x40 /* 140mV */

/* bit[5]: SYSON Control (SMB329 only) */
#define TCHARGE_SYSON_CONTROL_M                   0x20 /* mask */
#define TCHARGE_SYSON_ENABLE_V                    0x00 /* Enabled */
#define TCHARGE_SYSON_DISABLE_V                   0x20 /* Disabled */

/* bit[4:3]: OTG Current Limit (Out of Battery) */
#define TCHARGE_OTG_LIMIT_M                       0x18 /* mask */
#define TCHARGE_OTG_950MA_V                       0x00 /* 950mA */
#define TCHARGE_OTG_350MA_V                       0x08 /* 350mA */
#define TCHARGE_OTG_450MA_V                       0x10 /* 450mA */
#define TCHARGE_OTG_550MA_V                       0x18 /* 550mA */

/* bit[2:0]: OTG Mode UVLO Threshold */
#define TCHARGE_OTG_UVLOTHRESH_M                  0x07 /* mask */
#define TCHARGE_OTG_UVLO2P75V_V                   0x02 /* 2.75V */
#define TCHARGE_OTG_UVLO2P86V_V                   0x03 /* 2.86V */
#define TCHARGE_OTG_UVLO3P19V_V                   0x04 /* 3.19V */
#define TCHARGE_OTG_UVLO3P30V_V                   0x05 /* 3.30V */
#define TCHARGE_OTG_UVLO3P41V_V                   0x06 /* 3.41V */
#define TCHARGE_OTG_UVLO3P70V_V                   0x07 /* 3.70V */

/*---------------------------------------------------------------------------
  I2C Bus/Slave Address - 8bit (address: 07h) - Non-Volatile
---------------------------------------------------------------------------*/
#define TCHARGE_BUS_SLAVE_ADDR                    0x07 /* address */

/* bit[7:4]: I2C Slave Address */
#define TCHARGE_I2C_SLAVEADDR_M                   0xF0 /* mask */
#define TCHARGE_I2CADDR_0000_V                    0x00 /* 0000 */
#define TCHARGE_I2CADDR_0001_V                    0x10 /* 0001 */
#define TCHARGE_I2CADDR_0010_V                    0x20 /* 0010 */
#define TCHARGE_I2CADDR_0011_V                    0x30 /* 0011 */
#define TCHARGE_I2CADDR_0100_V                    0x40 /* 0100 */
#define TCHARGE_I2CADDR_0101_V                    0x50 /* 0101 */
#define TCHARGE_I2CADDR_0110_V                    0x60 /* 0110 */
#define TCHARGE_I2CADDR_0111_V                    0x70 /* 0111 */
#define TCHARGE_I2CADDR_1000_V                    0x80 /* 1000 */
#define TCHARGE_I2CADDR_1001_V                    0x90 /* 1001 */
#define TCHARGE_I2CADDR_1010_V                    0xA0 /* 1010 */
#define TCHARGE_I2CADDR_1011_V                    0xB0 /* 1011 */
#define TCHARGE_I2CADDR_1100_V                    0xC0 /* 1100 */
#define TCHARGE_I2CADDR_1101_V                    0xD0 /* 1101 */
#define TCHARGE_I2CADDR_1110_V                    0xE0 /* 1110 */
#define TCHARGE_I2CADDR_1111_V                    0xF0 /* 1111 */

/* bit[3:1]: I2C BUS Address */
#define TCHARGE_I2C_SLAVEBUS_M                    0x0E /* mask */
#define TCHARGE_I2CBUS_000_V                      0x00 /* 0000 */
#define TCHARGE_I2CBUS_001_V                      0x01 /* 0001 */
#define TCHARGE_I2CBUS_010_V                      0x02 /* 0010 */
#define TCHARGE_I2CBUS_011_V                      0x03 /* 0011 */
#define TCHARGE_I2CBUS_100_V                      0x04 /* 0100 */
#define TCHARGE_I2CBUS_101_V                      0x05 /* 0101 */
#define TCHARGE_I2CBUS_110_V                      0x06 /* 0110 */
#define TCHARGE_I2CBUS_111_V                      0x07 /* 0111 */

/* bit[0]: Voltatile Writes Permission */
#define TCHARGE_WRITE_PERMISSION_M                0x01 /* mask */
#define TCHARGE_WRITE_NOTALLOW_V                  0x00 /* No volatile writes to registers h00-h06 */
#define TCHARGE_WRITE_ALLOW_V                     0x01 /* Allow volatile writes to registers h00-h06 */

/*---------------------------------------------------------------------------
  Clear IRQ - (address: 30h) - Volatile(write only)
---------------------------------------------------------------------------*/
#define TCHARGE_CLEAR_IRQ_ADDR                    0x30 /* address */

/* Clear Interrupts : Write any data */
#define TCHARGE_CLEAR_IRQ_M                       0xFF /* mask */
#define TCHARGE_CLEAR_IRQ_V                       0xFF /* Clear Interrupts : Write any data */

/*---------------------------------------------------------------------------
  Command Register - (address: 31h) - Volatile(power up value is 000000, read/write)
---------------------------------------------------------------------------*/
#define TCHARGE_COMMAND_REG_ADDR                  0x31 /* address */

/* bit[7]: Voltatile Writes */
#define TCHARGE_VOLATILE_WRITE_M                  0x80 /* mask */
#define TCHARGE_VOLATILE_NOTALLOW_V               0x00 /* Do not allow volatile writes to 00h to 06h */
#define TCHARGE_VOLATILE_ALLOW_V                  0x80 /* Allow volatile writes */

/* bit[6]: POR */
#define TCHARGE_POR_M                             0x40 /* mask */
#define TCHARGE_POR_NOACTION_V                    0x00 /* No action */
#define TCHARGE_POR_RESET_V                       0x40 /* Internal Reset (Command ergisters are cleared & NV default values are reloaded) */

/* bit[5]: Fast Charge Settings */
#define TCHARGE_FAST_CHARGE_M                     0x20 /* mask */
#define TCHARGE_FORCE_RPECHG_V                    0x00 /* Force pre-change current settings */
#define TCHARGE_ALLOW_FASTCHG_V                   0x20 /* Allow fast-charge current settings */

/* bit[4]: Battery Charge Enable */
#define TCHARGE_BATTERY_CHARGE_M                  0x10 /* mask */
#define TCHARGE_CHARGE_0ENABLE_V                  0x00 /* Enabled for 05[2]=0, Disabled for 05[2]=1 */
#define TCHARGE_CHARGE_1ENABLE_V                  0x10 /* Enabled for 05[2]=1, Disabled for 05[2]=0 */

/* bit[3]: USB5/1 Mode */
#define TCHARGE_USB51_MODE_M                      0x08 /* mask */
#define TCHARGE_USB51_100MA_V                     0x00 /* USB 100mA Mode */
#define TCHARGE_USB51_500MA_V                     0x08 /* USB 500mA Mode */

/* bit[2]: USB5/1/AC Mode */
#define TCHARGE_USB51AC_MODE_M                    0x04 /* mask */
#define TCHARGE_USB51AC_USB51_V                   0x00 /* USB5/1 Current Limit */
#define TCHARGE_USB51AC_AC_V                      0x04 /* AC Mode Current Limit */

/* bit[1]: OTG Mode */
#define TCHARGE_OTG_MODE_M                        0x02 /* mask */
#define TCHARGE_OTG_DISABLE_V                     0x00 /* Disabled */
#define TCHARGE_OTG_ENABLE_V                      0x02 /* Enabled */

/* bit[0]: STAT Output */
#define TCHARGE_STAT_OUTPUT_M                     0x01 /* mask */
#define TCHARGE_STAT_ENABLE_V                     0x00 /* STAT Output Enabled (IRQ still allowed) */
#define TCHARGE_STAT_DISABLE_V                    0x01 /* STAT Output Disabled (IRQ still allowed) */

/*---------------------------------------------------------------------------
  Command Register - (address: 33h) - Volatile(read only)
---------------------------------------------------------------------------*/
#define TCHARGE_COMMAND_REG2_ADDR                 0x33 /* address */

/* bit[1]: Battery Temperature */
#define TCHARGE_BATTTEMP_HOT_M                    0x02 /* mask */
#define TCHARGE_BATTTEMP_HOT_V                    0x02 /* Battery Too Hot */

/* bit[0]: Battery Temperature */
#define TCHARGE_BATTTEMP_COLD_M                   0x01 /* mask */
#define TCHARGE_BATTTEMP_COLD_V                   0x01 /* Battery Too Cold */

/*---------------------------------------------------------------------------
  Power Source Detection Status - (address: 34h) - Volatile(read only)
---------------------------------------------------------------------------*/
#define TCHARGE_POW_DETECT_ADDR                   0x34 /* address */

/* bit[7:4]: AC Input Current Limit Status */
#define TCHARGE_AC_LIMITR_M                       0xF0 /* mask */
#define TCHARGE_AC_275MAR_V                       0x80 /* 275mA */
#define TCHARGE_AC_300MAR_V                       0x00 /* 300mA */
#define TCHARGE_AC_550MAR_V                       0x10 /* 550mA */
#define TCHARGE_AC_650MAR_V                       0x20 /* 650mA */
#define TCHARGE_AC_750MAR_V                       0x30 /* 750mA */
#define TCHARGE_AC_850MAR_V                       0x40 /* 850mA */
#define TCHARGE_AC_950MAR_V                       0x50 /* 950mA */
#define TCHARGE_AC_1050MAR_V                      0x60 /* 1050mA */
#define TCHARGE_AC_1150MAR_V                      0x70 /* 1150mA */

/* bit[2]: Power Source Detection Function Status */
#define TCHARGE_FUNCTION_STATUS_M                 0x04 /* mask */
#define TCHARGE_FUNCTION_NOTBUSY_V                0x00 /* Function not busy */
#define TCHARGE_FUNCTION_BUSY_V                   0x04 /* Function busy */

/* bit[1]: Power Source Detection Status */
#define TCHARGE_DETECT_STATUS_M                   0x02 /* mask */
#define TCHARGE_DETECT_NOTCOMPLETE_V              0x00 /* Source detection not completed */
#define TCHARGE_DETECT_COMPLETE_V                 0x02 /* Source detection completed */

/* bit[0]: Power Source Type */
#define TCHARGE_POWER_TYPE_M                      0x01 /* mask */
#define TCHARGE_POWER_USB_V                       0x00 /* USB Charger */
#define TCHARGE_POWER_DEDICATED_V                 0x01 /* Detected Charger */

/*---------------------------------------------------------------------------
  Battery Status Register A - (address: 35h) - Volatile(read only)
---------------------------------------------------------------------------*/
#define TCHARGE_BATT_STATUS_A_ADDR                0x35 /* address */

/* bit[6]: State of OTG or USB5/1/AC pin */
#define TCHARGE_PIN_STATE_M                       0x40 /* mask */
#define TCHARGE_PIN_LOW_V                         0x00 /* Logic Low */
#define TCHARGE_PIN_HIGH_V                        0x40 /* Logic High */

/* bit[5]: USB5/1 Mode */
#define TCHARGE_USB_MODE_M                        0x20 /* mask */
#define TCHARGE_USB_100MA_V                       0x00 /* USB 100mA Mode */
#define TCHARGE_USB_500MA_V                       0x20 /* USB 500mA Mode */

/* bit[4]: USB/AC Mode */
#define TCHARGE_USBAC_MODE_M                      0x10 /* mask */
#define TCHARGE_USBAC_USB_V                       0x00 /* USB Mode */
#define TCHARGE_USBAC_AC_V                        0x10 /* AC Mode */

/* bit[3]: Status */
#define TCHARGE_RECHARGE_SIGNAL_M                 0x08 /* mask */
#define TCHARGE_RECHARGE_SIGNAL_V                 0x08 /* Re-charge interrupt Signal (IRQ) */

/* bit[2]: Status */
#define TCHARGE_LIMIT_REACH_M                     0x04 /* mask */
#define TCHARGE_LIMIT_REACH_V                     0x04 /* Internal Temperature Limit Reached */

/* bit[1]: Status */
#define TCHARGE_OTG_PROGRESS_M                    0x02 /* mask */
#define TCHARGE_OTG_PROGRESS_V                    0x02 /* OTG Mode in Progress */

/* bit[0]: Status */
#define TCHARGE_OTG_UVLO_M                        0x01 /* mask */
#define TCHARGE_OTG_UVLO_V                        0x01 /* OTG Battery UVLO */

/*---------------------------------------------------------------------------
  Battery Status Register B - (address: 36h) - Volatile(read only)
---------------------------------------------------------------------------*/
#define TCHARGE_BATT_STATUS_B_ADDR                0x36 /* address */

/* bit[7]: Charge Termination */
#define TCHARGE_FULLBATT_ONCE_M                   0x80 /* mask */
#define TCHARGE_FULLBATT_ONCE_V                   0x80 /* At least one charge cycle has occurred and terminated */

/* bit[6]: Charge Termination */
#define TCHARGE_CHARGE_LOW_M                      0x40 /* mask */
#define TCHARGE_CHARGE_LOW_V                      0x40 /* Charge current is lower than termination current threshold */

/* bit[5:4]: Safety Timer Status */
#define TCHARGE_SFTIMER_STATUS_M                  0x30 /* mask */
#define TCHARGE_NO_TIMEOUT_V                      0x00 /* No safety timeouts occurred */
#define TCHARGE_PRECHG_EXPIRE_V                   0x10 /* Pre-charge Safety timer expired */
#define TCHARGE_COMCHG_EXPIRE_V                   0x20 /* Complete-charge Safety timer expired */
#define TCHARGE_WAITING_CHARGE_V                  0x30 /* Waiting to begin charging */

/* bit[3]: Charger Error IRQ */
#define TCHARGE_CHARGE_ERROR_M                    0x08 /* mask */
#define TCHARGE_NO_ERROR_V                        0x00 /* No Charger Error Interrupt Signal */
#define TCHARGE_CHGARGE_ERROR_V                   0x08 /* Interrupt Signal because of Charger Error */

/* bit[2:1]: Charging Status */
#define TCHARGE_CHARGING_STATUS_M                 0x06 /* mask */
#define TCHARGE_NO_CHARGING_V                     0x00 /* No Charging (from switcher) */
#define TCHARGE_PRE_CHARGING_V                    0x02 /* Pre-charging */
#define TCHARGE_FAST_CHARGING_V                   0x04 /* Fast-charging */
#define TCHARGE_TAPER_CHARGING_V                  0x06 /* Taper-charing */

/* bit[0]: Charging Enabled/Disable */
#define TCHARGE_CHARGING_STATE_M                  0x01 /* mask */
#define TCHARGE_CHARGER_ENABLE_V                  0x01 /* Charger Enabled */
#define TCHARGE_CHARGER_DISABLE_V                 0x00 /* Charger Disabled */

/*---------------------------------------------------------------------------
  Battery Status Register C - (address: 37h) - Volatile(read only)
---------------------------------------------------------------------------*/
#define TCHARGE_BATT_STATUS_C_ADDR                0x37 /* address */

/* bit[7]: Various Status */
#define TCHARGE_WATCHDOG_INT_M                    0x80 /* mask */
#define TCHARGE_WATCHDOG_INT_V                    0x80 /* Watchdog interrupt */

/* bit[6]: Various Status */
#define TCHARGE_OTG_REACH_M                       0x40 /* mask */
#define TCHARGE_OTG_REACH_V                       0x40 /* OTG current limit reached */

/* bit[5]: Various Status */
#define TCHARGE_VOLTAGE_BELOW_M                   0x20 /* mask */
#define TCHARGE_VOLTAGE_BELOW_V                   0x20 /* Input Voltage is below threshold (collapse) */

/* bit[4]: Various Status */
#define TCHARGE_BATTERY_MISSING_M                 0x10 /* mask */
#define TCHARGE_BATTERY_MISSING_V                 0x10 /* Battery missing */

/* bit[3]: Various Status */
#define TCHARGE_BATTERY_OVLO_M                    0x08 /* mask */
#define TCHARGE_BATTERY_OVLO_V                    0x08 /* Battery OVLO */

/* bit[2]: Various Status */
#define TCHARGE_INPUT_OVLO_M                      0x04 /* mask */
#define TCHARGE_INPUT_OVLO_V                      0x04 /* Input OVLO */

/* bit[1]: Various Status */
#define TCHARGE_UNDER_VBATT_M                     0x02 /* mask */
#define TCHARGE_UNDER_VBATT_V                     0x02 /* Input UVLO or DCIN < VBATT */

/* bit[0]: Various Status */
#define TCHARGE_TRICKLE_CHARGE_M                  0x01 /* mask */
#define TCHARGE_TRICKLE_CHARGE_V                  0x01 /* Trickle Charge Mode (VBATT < 2.0V) */

/*---------------------------------------------------------------------------
  Version - (address: 3Bh) - Non-Volatile(read only)
---------------------------------------------------------------------------*/
#define TCHARGE_VERSION_ADDR                      0x3B /* address */

/* bit[7:5]: Vendor Code */
#define TCHARGE_VENDOR_CODE_M                     0xE0 /* mask */

/* bit[2:0]: Revision Code */
#define TCHARGE_REVISION_CODE_M                   0x07 /* mask */
////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int g_chgtype; /* cange type */
/* FUJITSU:2011-04-20 USB end */

struct msm_chg_rpc_ids {
	unsigned long	vers_comp;
	unsigned	chg_usb_charger_connected_proc;
	unsigned	chg_usb_charger_disconnected_proc;
	unsigned	chg_usb_i_is_available_proc;
	unsigned	chg_usb_i_is_not_available_proc;
};

struct msm_hsusb_rpc_ids {
	unsigned long	prog;
	unsigned long	vers_comp;
	unsigned long	init_phy;
	unsigned long	vbus_pwr_up;
	unsigned long	vbus_pwr_down;
	unsigned long	update_product_id;
	unsigned long	update_serial_num;
	unsigned long	update_is_serial_num_null;
	unsigned long	reset_rework_installed;
	unsigned long	enable_pmic_ulpi_data0;
	unsigned long	disable_pmic_ulpi_data0;
/* FUJITSU:2011-05-23 USB start */
	unsigned long	cable_det_notifi_init_comp_proc;
	unsigned long	cable_det_notifi_disconnect_proc;
	unsigned long	cable_det_notifi_suspend_proc;
	unsigned long	cable_det_notifi_resume_proc;
	unsigned long	cable_det_notifi_usb_vdd_off_proc;
	unsigned long	cable_det_notifi_earphone_status_proc;
	unsigned long	cable_det_notifi_usb_ctrl_power_proc;
/* FUJITSU:2011-05-23 USB end */
};

static struct msm_hsusb_rpc_ids usb_rpc_ids;
static struct msm_chg_rpc_ids chg_rpc_ids;

/* FUJITSU:2011-07-07 USB start */
int g_cradle_stat;	/* enable 1:disable 0 */
int g_cradle_otg_charge_type = -1;
/* FUJITSU:2011-07-07 USB end */

static int msm_hsusb_init_rpc_ids(unsigned long vers)
{
	if (vers == 0x00010001) {
		usb_rpc_ids.prog			= 0x30000064;
		usb_rpc_ids.vers_comp			= 0x00010001;
		usb_rpc_ids.init_phy			= 2;
		usb_rpc_ids.vbus_pwr_up			= 6;
		usb_rpc_ids.vbus_pwr_down		= 7;
		usb_rpc_ids.update_product_id		= 8;
		usb_rpc_ids.update_serial_num		= 9;
		usb_rpc_ids.update_is_serial_num_null	= 10;
		usb_rpc_ids.reset_rework_installed	= 17;
		usb_rpc_ids.enable_pmic_ulpi_data0	= 18;
		usb_rpc_ids.disable_pmic_ulpi_data0	= 19;
/* FUJITSU:2011-05-24 USB start */
		usb_rpc_ids.cable_det_notifi_init_comp_proc       = 28;
		usb_rpc_ids.cable_det_notifi_disconnect_proc      = 29;
		usb_rpc_ids.cable_det_notifi_suspend_proc         = 30;
		usb_rpc_ids.cable_det_notifi_resume_proc          = 31;
		usb_rpc_ids.cable_det_notifi_usb_vdd_off_proc     = 32;
		usb_rpc_ids.cable_det_notifi_earphone_status_proc = 33;
		usb_rpc_ids.cable_det_notifi_usb_ctrl_power_proc  = 34;
/* FUJITSU:2011-05-24 USB end */
		return 0;
	} else if (vers == 0x00010002) {
		usb_rpc_ids.prog			= 0x30000064;
		usb_rpc_ids.vers_comp			= 0x00010002;
		usb_rpc_ids.init_phy			= 2;
		usb_rpc_ids.vbus_pwr_up			= 6;
		usb_rpc_ids.vbus_pwr_down		= 7;
		usb_rpc_ids.update_product_id		= 8;
		usb_rpc_ids.update_serial_num		= 9;
		usb_rpc_ids.update_is_serial_num_null	= 10;
		usb_rpc_ids.reset_rework_installed	= 17;
		usb_rpc_ids.enable_pmic_ulpi_data0	= 18;
		usb_rpc_ids.disable_pmic_ulpi_data0	= 19;
/* FUJITSU:2011-05-24 USB start */
		usb_rpc_ids.cable_det_notifi_init_comp_proc       = 28;
		usb_rpc_ids.cable_det_notifi_disconnect_proc      = 29;
		usb_rpc_ids.cable_det_notifi_suspend_proc         = 30;
		usb_rpc_ids.cable_det_notifi_resume_proc          = 31;
		usb_rpc_ids.cable_det_notifi_usb_vdd_off_proc     = 32;
		usb_rpc_ids.cable_det_notifi_earphone_status_proc = 33;
		usb_rpc_ids.cable_det_notifi_usb_ctrl_power_proc  = 34;
/* FUJITSU:2011-05-24 USB end */
		return 0;
	} else {
		pr_err("%s: no matches found for version\n",
			__func__);
		return -ENODATA;
	}
}

static int msm_chg_init_rpc(unsigned long vers)
{
	if (((vers & RPC_VERSION_MAJOR_MASK) == 0x00010000) ||
	    ((vers & RPC_VERSION_MAJOR_MASK) == 0x00020000) ||
	    ((vers & RPC_VERSION_MAJOR_MASK) == 0x00030000) ||
	    ((vers & RPC_VERSION_MAJOR_MASK) == 0x00040000)) {
		chg_ep = msm_rpc_connect_compatible(MSM_RPC_CHG_PROG, vers,
						     MSM_RPC_UNINTERRUPTIBLE);
		if (IS_ERR(chg_ep))
			return -ENODATA;
		chg_rpc_ids.vers_comp				= vers;
		chg_rpc_ids.chg_usb_charger_connected_proc 	= 7;
		chg_rpc_ids.chg_usb_charger_disconnected_proc 	= 8;
		chg_rpc_ids.chg_usb_i_is_available_proc 	= 9;
		chg_rpc_ids.chg_usb_i_is_not_available_proc 	= 10;
		return 0;
	} else
		return -ENODATA;
}

/* rpc connect for hsusb */
int msm_hsusb_rpc_connect(void)
{

	if (usb_ep && !IS_ERR(usb_ep)) {
		pr_debug("%s: usb_ep already connected\n", __func__);
		return 0;
	}

	/* Initialize rpc ids */
	if (msm_hsusb_init_rpc_ids(0x00010001)) {
		pr_err("%s: rpc ids initialization failed\n"
			, __func__);
		return -ENODATA;
	}

	usb_ep = msm_rpc_connect_compatible(usb_rpc_ids.prog,
					usb_rpc_ids.vers_comp,
					MSM_RPC_UNINTERRUPTIBLE);

	if (IS_ERR(usb_ep)) {
		pr_err("%s: connect compatible failed vers = %lx\n",
			 __func__, usb_rpc_ids.vers_comp);

		/* Initialize rpc ids */
		if (msm_hsusb_init_rpc_ids(0x00010002)) {
			pr_err("%s: rpc ids initialization failed\n",
				__func__);
			return -ENODATA;
		}
		usb_ep = msm_rpc_connect_compatible(usb_rpc_ids.prog,
					usb_rpc_ids.vers_comp,
					MSM_RPC_UNINTERRUPTIBLE);
	}

	if (IS_ERR(usb_ep)) {
		pr_err("%s: connect compatible failed vers = %lx\n",
				__func__, usb_rpc_ids.vers_comp);
		return -EAGAIN;
	} else
		pr_debug("%s: rpc connect success vers = %lx\n",
				__func__, usb_rpc_ids.vers_comp);

	return 0;
}
EXPORT_SYMBOL(msm_hsusb_rpc_connect);

/* rpc connect for charging */
int msm_chg_rpc_connect(void)
{
	uint32_t chg_vers;

	if (machine_is_msm7201a_surf() || machine_is_msm7x27_surf() ||
	    machine_is_qsd8x50_surf() || machine_is_msm7x25_surf() ||
	    machine_is_qsd8x50a_surf())
		return -ENOTSUPP;

	if (chg_ep && !IS_ERR(chg_ep)) {
		pr_debug("%s: chg_ep already connected\n", __func__);
		return 0;
	}

	chg_vers = 0x00040001;
	if (!msm_chg_init_rpc(chg_vers))
		goto chg_found;

	chg_vers = 0x00030001;
	if (!msm_chg_init_rpc(chg_vers))
		goto chg_found;

	chg_vers = 0x00020001;
	if (!msm_chg_init_rpc(chg_vers))
		goto chg_found;

	chg_vers = 0x00010001;
	if (!msm_chg_init_rpc(chg_vers))
		goto chg_found;

	pr_err("%s: connect compatible failed \n",
			__func__);
	return -EAGAIN;

chg_found:
	pr_debug("%s: connected to rpc vers = %x\n",
			__func__, chg_vers);
	return 0;
}
EXPORT_SYMBOL(msm_chg_rpc_connect);

/* rpc call for phy_reset */
int msm_hsusb_phy_reset(void)
{
	int rc = 0;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		pr_err("%s: phy_reset rpc failed before call,"
			"rc = %ld\n", __func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	rc = msm_rpc_call(usb_ep, usb_rpc_ids.init_phy,
				&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		pr_err("%s: phy_reset rpc failed! rc = %d\n",
			__func__, rc);
	} else
		pr_debug("msm_hsusb_phy_reset\n");

	return rc;
}
EXPORT_SYMBOL(msm_hsusb_phy_reset);

/* rpc call for vbus powerup */
int msm_hsusb_vbus_powerup(void)
{
	int rc = 0;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		pr_err("%s: vbus_powerup rpc failed before call,"
			"rc = %ld\n", __func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	rc = msm_rpc_call(usb_ep, usb_rpc_ids.vbus_pwr_up,
		&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		pr_err("%s: vbus_powerup failed! rc = %d\n",
			__func__, rc);
	} else
		pr_debug("msm_hsusb_vbus_powerup\n");

	return rc;
}
EXPORT_SYMBOL(msm_hsusb_vbus_powerup);

/* rpc call for vbus shutdown */
int msm_hsusb_vbus_shutdown(void)
{
	int rc = 0;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		pr_err("%s: vbus_shutdown rpc failed before call,"
			"rc = %ld\n", __func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	rc = msm_rpc_call(usb_ep, usb_rpc_ids.vbus_pwr_down,
		&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		pr_err("%s: vbus_shutdown failed! rc = %d\n",
			__func__, rc);
	} else
		pr_debug("msm_hsusb_vbus_shutdown\n");

	return rc;
}
EXPORT_SYMBOL(msm_hsusb_vbus_shutdown);

int msm_hsusb_send_productID(uint32_t product_id)
{
	int rc = 0;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
		uint32_t product_id;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		pr_err("%s: rpc connect failed: rc = %ld\n",
			__func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	req.product_id = cpu_to_be32(product_id);
	rc = msm_rpc_call(usb_ep, usb_rpc_ids.update_product_id,
				&req, sizeof(req),
				5 * HZ);
	if (rc < 0)
		pr_err("%s: rpc call failed! error: %d\n",
			__func__, rc);
	else
		pr_debug("%s: rpc call success\n" , __func__);

	return rc;
}
EXPORT_SYMBOL(msm_hsusb_send_productID);

int msm_hsusb_send_serial_number(const char *serial_number)
{
	int rc = 0, serial_len;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
		uint32_t length;
		char serial_num[20];
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		pr_err("%s: rpc connect failed: rc = %ld\n",
			__func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	serial_len  = strlen(serial_number)+1;
	strncpy(req.serial_num, serial_number, 20);
	req.length = cpu_to_be32(serial_len);
	rc = msm_rpc_call(usb_ep, usb_rpc_ids.update_serial_num,
				&req, sizeof(req),
				5 * HZ);
	if (rc < 0)
		pr_err("%s: rpc call failed! error: %d\n",
			__func__, rc);
	else
		pr_debug("%s: rpc call success\n", __func__);

	return rc;
}
EXPORT_SYMBOL(msm_hsusb_send_serial_number);

int msm_hsusb_is_serial_num_null(uint32_t val)
{
	int rc = 0;
	struct hsusb_phy_start_req {
			struct rpc_request_hdr hdr;
			uint32_t value;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		pr_err("%s: rpc connect failed: rc = %ld\n",
			__func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}
	if (!usb_rpc_ids.update_is_serial_num_null) {
		pr_err("%s: proc id not supported \n", __func__);
		return -ENODATA;
	}

	req.value = cpu_to_be32(val);
	rc = msm_rpc_call(usb_ep, usb_rpc_ids.update_is_serial_num_null,
				&req, sizeof(req),
				5 * HZ);
	if (rc < 0)
		pr_err("%s: rpc call failed! error: %d\n" ,
			__func__, rc);
	else
		pr_debug("%s: rpc call success\n", __func__);

	return rc;
}
EXPORT_SYMBOL(msm_hsusb_is_serial_num_null);

int msm_chg_usb_charger_connected(uint32_t device)
{
	int rc = 0;
	struct hsusb_start_req {
		struct rpc_request_hdr hdr;
		uint32_t otg_dev;
	} req;

	if (!chg_ep || IS_ERR(chg_ep))
		return -EAGAIN;
	req.otg_dev = cpu_to_be32(device);
	rc = msm_rpc_call(chg_ep, chg_rpc_ids.chg_usb_charger_connected_proc,
			&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		pr_err("%s: charger_connected failed! rc = %d\n",
			__func__, rc);
	} else
		pr_debug("msm_chg_usb_charger_connected\n");

	return rc;
}
EXPORT_SYMBOL(msm_chg_usb_charger_connected);

int msm_chg_usb_i_is_available(uint32_t sample)
{
	int rc = 0;
	struct hsusb_start_req {
		struct rpc_request_hdr hdr;
		uint32_t i_ma;
	} req;

	if (!chg_ep || IS_ERR(chg_ep))
		return -EAGAIN;
	req.i_ma = cpu_to_be32(sample);
	rc = msm_rpc_call(chg_ep, chg_rpc_ids.chg_usb_i_is_available_proc,
			&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		pr_err("%s: charger_i_available failed! rc = %d\n",
			__func__, rc);
	} else
		pr_debug("msm_chg_usb_i_is_available(%u)\n", sample);

	return rc;
}
EXPORT_SYMBOL(msm_chg_usb_i_is_available);

int msm_chg_usb_i_is_not_available(void)
{
	int rc = 0;
	struct hsusb_start_req {
		struct rpc_request_hdr hdr;
	} req;

	if (!chg_ep || IS_ERR(chg_ep))
		return -EAGAIN;
	rc = msm_rpc_call(chg_ep, chg_rpc_ids.chg_usb_i_is_not_available_proc,
			&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		pr_err("%s: charger_i_not_available failed! rc ="
			"%d \n", __func__, rc);
	} else
		pr_debug("msm_chg_usb_i_is_not_available\n");

	return rc;
}
EXPORT_SYMBOL(msm_chg_usb_i_is_not_available);

int msm_chg_usb_charger_disconnected(void)
{
	int rc = 0;
	struct hsusb_start_req {
		struct rpc_request_hdr hdr;
	} req;

	if (!chg_ep || IS_ERR(chg_ep))
		return -EAGAIN;
	rc = msm_rpc_call(chg_ep, chg_rpc_ids.chg_usb_charger_disconnected_proc,
			&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		pr_err("%s: charger_disconnected failed! rc = %d\n",
			__func__, rc);
	} else
		pr_debug("msm_chg_usb_charger_disconnected\n");

	return rc;
}
EXPORT_SYMBOL(msm_chg_usb_charger_disconnected);

/* rpc call to close connection */
int msm_hsusb_rpc_close(void)
{
	int rc = 0;

	if (IS_ERR(usb_ep)) {
		pr_err("%s: rpc_close failed before call, rc = %ld\n",
			__func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	rc = msm_rpc_close(usb_ep);
	usb_ep = NULL;

	if (rc < 0) {
		pr_err("%s: close rpc failed! rc = %d\n",
			__func__, rc);
		return -EAGAIN;
	} else
		pr_debug("rpc close success\n");

	return rc;
}
EXPORT_SYMBOL(msm_hsusb_rpc_close);

/* rpc call to close charging connection */
int msm_chg_rpc_close(void)
{
	int rc = 0;

	if (IS_ERR(chg_ep)) {
		pr_err("%s: rpc_close failed before call, rc = %ld\n",
			__func__, PTR_ERR(chg_ep));
		return -EAGAIN;
	}

	rc = msm_rpc_close(chg_ep);
	chg_ep = NULL;

	if (rc < 0) {
		pr_err("%s: close rpc failed! rc = %d\n",
			__func__, rc);
		return -EAGAIN;
	} else
		pr_debug("rpc close success\n");

	return rc;
}
EXPORT_SYMBOL(msm_chg_rpc_close);

int msm_hsusb_reset_rework_installed(void)
{
	int rc = 0;
	struct hsusb_start_req {
		struct rpc_request_hdr hdr;
	} req;
	struct hsusb_rpc_rep {
		struct rpc_reply_hdr hdr;
		uint32_t rework;
	} rep;

	memset(&rep, 0, sizeof(rep));

	if (!usb_ep || IS_ERR(usb_ep)) {
		pr_err("%s: hsusb rpc connection not initialized, rc = %ld\n",
			__func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	rc = msm_rpc_call_reply(usb_ep, usb_rpc_ids.reset_rework_installed,
				&req, sizeof(req),
				&rep, sizeof(rep), 5 * HZ);

	if (rc < 0) {
		pr_err("%s: rpc call failed! error: (%d)"
				"proc id: (%lx)\n",
				__func__, rc,
				usb_rpc_ids.reset_rework_installed);
		return rc;
	}

	pr_info("%s: rework: (%d)\n", __func__, rep.rework);
	return be32_to_cpu(rep.rework);
}
EXPORT_SYMBOL(msm_hsusb_reset_rework_installed);

static int msm_hsusb_pmic_ulpidata0_config(int enable)
{
	int rc = 0;
	struct hsusb_start_req {
		struct rpc_request_hdr hdr;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		pr_err("%s: hsusb rpc connection not initialized, rc = %ld\n",
			__func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	if (enable)
		rc = msm_rpc_call(usb_ep, usb_rpc_ids.enable_pmic_ulpi_data0,
					&req, sizeof(req), 5 * HZ);
	else
		rc = msm_rpc_call(usb_ep, usb_rpc_ids.disable_pmic_ulpi_data0,
					&req, sizeof(req), 5 * HZ);

	if (rc < 0)
		pr_err("%s: rpc call failed! error: %d\n",
				__func__, rc);
	return rc;
}

int msm_hsusb_enable_pmic_ulpidata0(void)
{
	return msm_hsusb_pmic_ulpidata0_config(1);
}
EXPORT_SYMBOL(msm_hsusb_enable_pmic_ulpidata0);

int msm_hsusb_disable_pmic_ulpidata0(void)
{
	return msm_hsusb_pmic_ulpidata0_config(0);
}
EXPORT_SYMBOL(msm_hsusb_disable_pmic_ulpidata0);


/* wrapper for sending pid and serial# info to bootloader */
int usb_diag_update_pid_and_serial_num(uint32_t pid, const char *snum)
{
	int ret;

	ret = msm_hsusb_send_productID(pid);
	if (ret)
		return ret;

	if (!snum) {
		ret = msm_hsusb_is_serial_num_null(1);
		if (ret)
			return ret;
	}

	ret = msm_hsusb_is_serial_num_null(0);
	if (ret)
		return ret;
	ret = msm_hsusb_send_serial_number(snum);
	if (ret)
		return ret;

	return 0;
}


#ifdef CONFIG_USB_GADGET_MSM_72K
/* charger api wrappers */
int hsusb_chg_init(int connect)
{
	if (connect)
		return msm_chg_rpc_connect();
	else
		return msm_chg_rpc_close();
}
EXPORT_SYMBOL(hsusb_chg_init);

void hsusb_chg_vbus_draw(unsigned mA)
{
/* FUJITSU:2011-04-20 USB start */
#ifdef CONFIG_MACH_F11EIF
	if(system_rev < 0x0c)
#endif /* CONFIG_MACH_F11EIF */
		msm_chg_usb_i_is_available(mA);
/* FUJITSU:2011-04-20 USB end */
}
EXPORT_SYMBOL(hsusb_chg_vbus_draw);

void hsusb_chg_connected(enum chg_type chgtype)
{
	char *chg_types[] = {"STD DOWNSTREAM PORT",
			"CARKIT",
			"DEDICATED CHARGER",
			"INVALID"};

	if (chgtype == USB_CHG_TYPE__INVALID) {
		msm_chg_usb_i_is_not_available();
		msm_chg_usb_charger_disconnected();
/* FUJITSU:2011-04-20 USB start */
		g_chgtype=0;
/* FUJITSU:2011-04-20 USB end */
		return;
	}
/* FUJITSU:2011-04-20 USB start */
	switch(chgtype)
	{
	case 0:
		g_chgtype=2;
		break;
	case 2:
		g_chgtype=1;
		break;
	default:
		g_chgtype=0;
		break;
	}	
	pr_info("\nCharger Type: %s g[%d]\n", chg_types[chgtype],g_chgtype);
#ifdef CONFIG_MACH_F11EIF
        if(system_rev >= 0x0c){
		/*Battry ON */
		pr_info("%s: usb Battry Onstart\n", __func__);
		tcharge_hw_init_temp();
		if(msm_batt_update_status_cb_hsusb)
			msm_batt_update_status_cb_hsusb();
		else
			printk(KERN_ERR "%s:msm_batt_update_status_cb_hsusb is NULL.\n", __func__);
		pr_info("%s: usb Battry On end\n", __func__);
	}else{
#endif /* CONFIG_MACH_F11EIF */
/* FUJITSU:2011-07-07 USB start */
		pr_info("%s:cradle_connect stat:[%d]\n", __func__,g_cradle_stat);
		if(g_cradle_stat==1){
			if(chgtype==0){
				msm_chg_usb_charger_connected(chgtype);
				pr_info("%s: msm_chg_usb_charger_connected:[%d]\n", __func__,chgtype);
				g_cradle_otg_charge_type=-1;
			}else{
				pr_info("%s: msm_chg_usb_charger skip:[%d]\n", __func__,chgtype);
				g_cradle_otg_charge_type=chgtype;
			}
		}else{
			msm_chg_usb_charger_connected(chgtype);
			pr_info("%s: msm_chg_usb_charger_connected:[%d]\n", __func__,chgtype);
			g_cradle_otg_charge_type=-1;
		}
/* FUJITSU:2011-07-07 USB end */
#ifdef CONFIG_MACH_F11EIF
	}
#endif /* CONFIG_MACH_F11EIF */
/* FUJITSU:2011-04-20 USB end */
}
EXPORT_SYMBOL(hsusb_chg_connected);
#endif
/* FUJITSU:2011-04-04 USB start */
int hsusb_cable_det_notifi_initialize_complete(uint32_t drv)
{
	int rc = 0;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
		uint32_t drv;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		printk(KERN_ERR "%s: hsusb_cable_det_notifi_initialize_complete rpc failed before call,"
			"rc = %ld\n", __func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	req.drv = cpu_to_be32(drv);
	rc = msm_rpc_call(usb_ep, usb_rpc_ids.cable_det_notifi_init_comp_proc,
		&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		printk(KERN_ERR "%s: hsusb_cable_det_notifi_initialize_complete failed! rc = %d\n",
			__func__, rc);
	} else 
		printk(KERN_INFO "hsusb_cable_det_notifi_initialize_complete\n");

	return rc;
}
EXPORT_SYMBOL(hsusb_cable_det_notifi_initialize_complete);

int hsusb_cable_det_notifi_earphone_status(int status)
{
	int rc = 0;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
		int status;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		printk(KERN_ERR "%s: hsusb_cable_det_notifi_earphone_status rpc failed before call,"
			"rc = %ld\n", __func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	req.status = cpu_to_be32(status);
	rc = msm_rpc_call(usb_ep, usb_rpc_ids.cable_det_notifi_earphone_status_proc,
		&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		printk(KERN_ERR "%s: hsusb_cable_det_notifi_earphone_status failed! rc = %d\n",
			__func__, rc);
	} else
		printk(KERN_INFO "hsusb_cable_det_notifi_earphone_status\n");

	return rc;
}
EXPORT_SYMBOL(hsusb_cable_det_notifi_earphone_status);

int hsusb_cable_det_notifi_disconnect(void)
{
	int rc = 0;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
		//uint32_t drv;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		printk(KERN_ERR "%s: hsusb_cable_det_notifi_disconnect rpc failed before call,"
			"rc = %ld\n", __func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	//req.drv = cpu_to_be32(drv);
	rc = msm_rpc_call(usb_ep, usb_rpc_ids.cable_det_notifi_disconnect_proc,
		&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		printk(KERN_ERR "%s: hsusb_cable_det_notifi_disconnect failed! rc = %d\n",
			__func__, rc);
	} else
		printk(KERN_INFO "hsusb_cable_det_notifi_disconnect\n");


	return rc;
}
EXPORT_SYMBOL(hsusb_cable_det_notifi_disconnect);

int hsusb_cable_det_notifi_suspend(uint32_t drv)
{
	int rc = 0;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
		uint32_t drv;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		printk(KERN_ERR "%s: hsusb_cable_det_notifi_suspend rpc failed before call,"
			"rc = %ld\n", __func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	req.drv = cpu_to_be32(drv);
	rc = msm_rpc_call(usb_ep, usb_rpc_ids.cable_det_notifi_suspend_proc,
		&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		printk(KERN_ERR "%s: hsusb_cable_det_notifi_suspend failed! rc = %d\n",
			__func__, rc);
	} else
		printk(KERN_INFO "hsusb_cable_det_notifi_suspend\n");

/* FUJITSU:2011-04-20 USB start */
	printk(KERN_INFO "%s chgtype [%d]\n",__func__,g_chgtype);
	g_chgtype=0;
	printk(KERN_INFO "%s chgtype [%d]\n",__func__,g_chgtype);
/* FUJITSU:2011-04-20 USB end */
	return rc;
}
EXPORT_SYMBOL(hsusb_cable_det_notifi_suspend);

int hsusb_cable_det_notifi_resume(uint32_t drv)
{
	int rc = 0;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
		uint32_t drv;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		printk(KERN_ERR "%s: hsusb_cable_det_notifi_resume rpc failed before call,"
			"rc = %ld\n", __func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	req.drv = cpu_to_be32(drv);
	rc = msm_rpc_call(usb_ep, usb_rpc_ids.cable_det_notifi_resume_proc,
		&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		printk(KERN_ERR "%s: hsusb_cable_det_notifi_resume failed! rc = %d\n",
			__func__, rc);
	} else
		printk(KERN_INFO "hsusb_cable_det_notifi_resume\n");

	return rc;
}
EXPORT_SYMBOL(hsusb_cable_det_notifi_resume);

int hsusb_cable_usb_vdd_off(void)
{
	int rc = 0;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
		//uint32_t drv;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		printk(KERN_ERR "%s: hsusb_cable_usb_vdd_off rpc failed before call,"
			"rc = %ld\n", __func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	//req.drv = cpu_to_be32(drv);
	rc = msm_rpc_call(usb_ep, usb_rpc_ids.cable_det_notifi_usb_vdd_off_proc,
		&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		printk(KERN_ERR "%s: hsusb_cable_usb_vdd_off failed! rc = %d\n",
			__func__, rc);
	} else
		printk(KERN_INFO "hsusb_cable_usb_vdd_off\n");

	return rc;
}
EXPORT_SYMBOL(hsusb_cable_usb_vdd_off);
/* FUJITSU:2011-04-04 USB end */
/* FUJITSU:2011-05-23 USB start */
int hsusb_notifi_usb_ctrl_power(uint32_t ctrl)
/* ctrl on:1 off:0 */
{
	int rc = 0;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
		int power_ctrl;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		printk(KERN_ERR "%s: rpc failed before call,"
			"rc = %ld\n", __func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	req.power_ctrl = cpu_to_be32(ctrl);
	rc = msm_rpc_call(usb_ep, usb_rpc_ids.cable_det_notifi_usb_ctrl_power_proc,
		&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		printk(KERN_ERR "%s: failed! pram:[%d] rc = %d\n",
			__func__, ctrl, rc);
	} else
		printk(KERN_INFO "%s:pram:[%d]\n",__func__, ctrl);

	return rc;
}
EXPORT_SYMBOL(hsusb_notifi_usb_ctrl_power);
/* FUJITSU:2011-05-23 USB end */
/* FUJITSU:2011-04-20 USB start */
void tcharge_hw_init_temp( void )
{
    int ret = 0;
    struct i2c_adapter *i2c;
    struct i2c_msg msg[2];
    unsigned char buffer[8];
    unsigned char rbuffer[8];
    unsigned char temp_buf = 0;

    //power on
    msg[0].addr    = TCHARGE_I2C_SLAVE_ADDR;
    msg[0].buf     = buffer;
    msg[0].flags   = 0;

    msg[1].addr    = TCHARGE_I2C_SLAVE_ADDR;
    msg[1].buf     = rbuffer;
    msg[1].len     = 1;
    msg[1].flags   = 1;
   
    //i2c qup adapter id=4
    i2c = i2c_get_adapter(4);
  /* Command Register - (address: 31h) -----------------------------------*/
    buffer[0] = TCHARGE_COMMAND_REG_ADDR;
    msg[0].len     = 1;
    ret = i2c_transfer(i2c, msg, 2);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
    temp_buf=rbuffer[0];
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:31h read is 0x%02x\n", __func__, temp_buf);
  /* Bit[7] Voltatile Writes */
  /* bit[3]: USB5/1 Mode */
  /* bit[2]: USB5/1/AC Mode */
  temp_buf = (( temp_buf & ~TCHARGE_VOLATILE_WRITE_M ) | TCHARGE_VOLATILE_ALLOW_V );
  temp_buf = (( temp_buf & ~TCHARGE_USB51_MODE_M ) | TCHARGE_USB51_500MA_V );
  temp_buf = (( temp_buf & ~TCHARGE_USB51AC_MODE_M ) | TCHARGE_USB51AC_USB51_V );
    buffer[1] = temp_buf;
    msg[0].len     = 2;
    ret = i2c_transfer(i2c, msg, 1);
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:31h write is 0x%02x\n", __func__, temp_buf);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
  /* Charge (output) current - (address: 00h) ----------------------------*/
    buffer[0] = TCHARGE_CHG_CURR_ADDR;
    msg[0].len     = 1;
    ret = i2c_transfer(i2c, msg, 2);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
    temp_buf=rbuffer[0];
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:00h read is 0x%02x\n", __func__, temp_buf);
  /* Bit[7][6][5] Fast Charge Current */
  /* Bit[4][3] Pre-Charge Charge Current */
  /* Bit[2][1] Termination Currentt */
  /* Bit[0] Trickle-Charge Safety Timer */
  temp_buf = (( temp_buf & ~TCHARGE_FAST_CURR_M ) | TCHARGE_FAST_850MA_V );
  temp_buf = (( temp_buf & ~TCHARGE_PRE_CHARGE_CURR_M ) | TCHARGE_PRE_200MA_V );
  temp_buf = (( temp_buf & ~TCHARGE_TERMINATION_CURR_M ) | TCHARGE_TERMINATION_50MA_V );
  temp_buf = (( temp_buf & ~TCHARGE_TRICKLE_ST_M ) | TCHARGE_TRICKLE_ST_ENA_V );
    buffer[1] = temp_buf;
    msg[0].len     = 2;
    ret = i2c_transfer(i2c, msg, 1);
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:00h write is 0x%02x\n", __func__, temp_buf);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }

  /* Input current limit - (address: 01h) --------------------------------*/
  buffer[0] = TCHARGE_INPUT_LIMIT_ADDR;
    msg[0].len     = 1;
    ret = i2c_transfer(i2c, msg, 2);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
    temp_buf=rbuffer[0];
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:01h read is 0x%02x\n", __func__, temp_buf);
  /* Bit[1][0] Automatic Input Current Limit Threshold */
  temp_buf = (( temp_buf & ~TCHARGE_CURR_THRES_M ) | TCHARGE_THRES_4P25V_V );
    buffer[1] = temp_buf;
    msg[0].len     = 2;
    ret = i2c_transfer(i2c, msg, 1);
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:01h write is 0x%02x\n", __func__, temp_buf);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
  /* Float Voltage - (address: 02h) --------------------------------------*/
  buffer[0] = TCHARGE_FLOAT_VOLTAGE_ADDR;
    msg[0].len     = 1;
    ret = i2c_transfer(i2c, msg, 2);
    if(ret < 0){
        printk(KERN_DEBUG "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
    temp_buf=rbuffer[0];
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:02h read is 0x%02x\n", __func__, temp_buf);
  /* Bit[7] Internal (IC) Temperature limit */
  /* Bit[6][5][4][3][2][1][0] Float Voltage(V): 4.16V */
  temp_buf = (( temp_buf & ~TCHARGE_TEMP_LIMIT_M ) | TCHARGE_TEMP_ACTIVE_V );
  temp_buf = (( temp_buf & ~TCHARGE_FLOAT_VOLTAGE_M ) | 0x46 );
    buffer[1] = temp_buf;
    msg[0].len     = 2;
    ret = i2c_transfer(i2c, msg, 1);
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:02h write is 0x%02x\n", __func__, temp_buf);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
  /* Control Register A - (address: 03h) ---------------------------------*/
  buffer[0] = TCHARGE_CONT_REG_A_ADDR;
    msg[0].len     = 1;
    ret = i2c_transfer(i2c, msg, 2);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
    temp_buf=rbuffer[0];
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:03h read is 0x%02x\n", __func__, temp_buf);
  /* Bit[7] Automatic Recharge */
  temp_buf = (( temp_buf & ~TCHARGE_AUTO_RECHARGE_M ) | TCHARGE_RECHARGE_DISABLE_V );
  /* Bit[6]: Current Termination */
  temp_buf = (( temp_buf & ~TCHARGE_CURR_TERMINATION_M ) | TCHARGE_TERMINATE_NOTALLOWED_V );
  /* Bit[5][4][3] Pre-charge to Fast-charge Voltage Threshold */
  temp_buf = (( temp_buf & ~TCHARGE_PREFAST_VOLTAGE_M ) | TCHARGE_PREFASH_2P6V_V );
  /* Bit[2] SYSON UVLO */
  temp_buf = (( temp_buf & ~TCHARGE_SYSON_UVLO_M ) | TCHARGE_UVLO_3P50V_V );
  /* Bit[1] Automatic Power Source Detection */
  temp_buf = (( temp_buf & ~TCHARGE_AUTOPWR_DETECT_M ) | TCHARGE_AUTOPWR_DISABLE_V );
  /* Bit[0] Trickle Charging */
  temp_buf = (( temp_buf & ~TCHARGE_TRICKLE_CHARGE_M ) | TCHARGE_TRICKLE_ENABLE_V );
    buffer[1] = temp_buf;
    msg[0].len     = 2;
    ret = i2c_transfer(i2c, msg, 1);
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:03h write is 0x%02x\n", __func__, temp_buf);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
  /* Control Register B - (address: 04h) ---------------------------------*/
  buffer[0] = TCHARGE_CONT_REG_B_ADDR;
    msg[0].len     = 1;
    ret = i2c_transfer(i2c, msg, 2);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
    temp_buf=rbuffer[0];
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:04h read is 0x%02x\n", __func__, temp_buf);
  /* Bit[7] STAT Output Mode */
  /* Bit[6] Battery OV (Battery OV does/doesn't causes charge cycle to end) */
  temp_buf = (( temp_buf & ~TCHARGE_BATTERY_OV_M ) | TCHARGE_OV_NOTCAUSE_V );
    buffer[1] = temp_buf;
    msg[0].len     = 2;
    ret = i2c_transfer(i2c, msg, 1);
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:04h write is 0x%02x\n", __func__, temp_buf);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
  /* Pin Control - (address: 05h) ----------------------------------------*/
  buffer[0] = TCHARGE_PIN_CONT_ADDR;
    msg[0].len     = 1;
    ret = i2c_transfer(i2c, msg, 2);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
    temp_buf=rbuffer[0];
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:05h read is 0x%02x\n", __func__, temp_buf);
  /* Bit[6] Automatic Charging Control: Automatic charging is not enable; bits[3:2] control charging */
  temp_buf = (( temp_buf & ~TCHARGE_AUTO_CHARGE_M ) | TCHARGE_AUTOCHG_NOTENABLE_V );
  /* Bit[3][2] Enable (EN) Control: "0" in 31[4] turns on (enables) charger */
  temp_buf = (( temp_buf & ~TCHARGE_EN_CONTROL_M ) | TCHARGE_EN_CTRL0_V );
  /* Bit[1][0] OTG and USB5/1/AC Pin Control: Requires I2C */
  temp_buf = (( temp_buf & ~TCHARGE_USB51AC_PINCONTROL_M ) | TCHARGE_USB51AC_I2CREQ_V );
    buffer[1] = temp_buf;
    msg[0].len     = 2;
    ret = i2c_transfer(i2c, msg, 1);
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:05h write is 0x%02x\n", __func__, temp_buf);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }

  /* OTG Control - (address: 06h) ----------------------------------------*/
  buffer[0] = TCHARGE_OTG_CONT_ADDR;
    msg[0].len     = 1;
    ret = i2c_transfer(i2c, msg, 2);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
    temp_buf=rbuffer[0];
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:06h read is 0x%02x\n", __func__, temp_buf);
  /* Bit[7] Battery Missing Detection */
  temp_buf = (( temp_buf & ~TCHARGE_BATT_DETECT_M ) | TCHARGE_BATT_DISABLE_V );
  /* Bit[6] Automatic Recharge Threshold */
  temp_buf = (( temp_buf & ~TCHARGE_RECHARGE_THRESH_M ) | TCHARGE_RECHARGE_140MV_V );
  /* Bit[5] SYSON Control (SMB329 only) */
  temp_buf = (( temp_buf & ~TCHARGE_SYSON_CONTROL_M ) | TCHARGE_SYSON_ENABLE_V );
    buffer[1] = temp_buf;
    msg[0].len     = 2;
    ret = i2c_transfer(i2c, msg, 1);
    printk(KERN_DEBUG "%s:[TCHG-DEBUG] addr:06h write is 0x%02x\n", __func__, temp_buf);
    if(ret < 0){
        printk(KERN_ERR "[DD](%s):I2C ERROR [%d] ret = %d\n",__func__, __LINE__, ret);
        return;
    }
    printk(KERN_DEBUG "[DD](%s):I2C OK [%d] ret = %d\n",__func__, __LINE__, ret);
    return;
}

int msm_hsusb_chg_status(void)
{
	return g_chgtype;
}
EXPORT_SYMBOL(msm_hsusb_chg_status);
void msm_hsusb_chg_set_status(chg)
{
	if(g_chgtype!=0)
		if(chg == 0)
			if(g_chgtype == 1)
				return;
	g_chgtype=chg;
	return;
}
EXPORT_SYMBOL(msm_hsusb_chg_set_status);
void msm_reg_battfunc_hsusb(msm_batt_cbfunc func)
{
	msm_batt_update_status_cb_hsusb = func;
	return;
}
EXPORT_SYMBOL(msm_reg_battfunc_hsusb);
/* FUJITSU:2011-04-20 USB end */
