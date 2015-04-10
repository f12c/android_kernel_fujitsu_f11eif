/*
** =========================================================================
** File:
**	 ImmVibeSPI.c
**
** Description: 
**	 Device-dependent functions called by Immersion TSP API
**	 to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2010 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/delay.h>

/* FUJITSU:2011-04-29 VIB for 2-2 start */
#include "tspdrv.h"
#include <linux/clk.h>
#include <mach/msm_iomap.h>

#define __msmhwio_outm(hwiosym, mask, val) HWIO_##hwiosym##_OUTM(mask, val)
#define HWIO_OUTM(hwiosym, mask, val) __msmhwio_outm(hwiosym, mask, val)
/* FUJITSU:2011-04-29 VIB for 2-2 end */

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

/*
** This SPI supports only one actuator.
*/
#define NUM_ACTUATORS 1

//#define PWM_DUTY_MAX	579 /* 13MHz / (579 + 1) = 22.4kHz */

#define PWM_DUTY_MAX	39 /* FUJITSU: To match approx. expected freq. 40us */

/* FUJITSU:2011-04-26 modify VIB GPIO port and PWM_DUTY start */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)	 (pm_gpio + NR_GPIO_IRQS)
#define PMIC_GPIO_HAP_ENABLE 14 /* PMIC GPIO Number 15 */
#define PMIC_CVIB_LDO_ENABLE 18 /* PMIC GPIO Number 19 */
/* FUJITSU:2011-04-26 modify VIB GPIO port and PWM_DUTY end */

/* FUJITSU:2011-04-29 VIB for < 2-2 start */
#define HAP_LVL_SHFT_PMIC_GPIO 23 /* PMIC GPIO Number 24 */
/* FUJITSU:2011-04-29 VIB for < 2-2 end */

/* FUJITSU:2011-04-29 VIB for 2-2 start */
#define HAP_LVL_SHFT_MSM_GPIO 16
#define VIB_NAME "vib"
/* FUJITSU:2011-04-29 VIB for 2-2 end */

struct pwm_device *pwm;

static bool g_bAmpEnabled = false;

/* FUJITSU:2011-04-29 VIB for 2-2 start */

struct clk *android_vib_clk; /* gp_clk */

// immergion sample
#define GP_CLK_M_DEFAULT 21
#define GP_CLK_N_DEFAULT 18000
#define GP_CLK_D_DEFAULT 9000 /* 50% duty cycle */ 

#define IMM_PWM_MULTIPLIER 17778 /* Must be integer */
//#define GP_CLK_M_DEFAULT 30
//#define GP_CLK_N_DEFAULT 20000
//#define GP_CLK_D_DEFAULT 10000 /* 50% duty cycle */ 

#define uint32 uint32_t
typedef unsigned long L4_Word_t;
typedef L4_Word_t word_t;
typedef word_t L4_Bool_t;

#ifndef INTLOCK
#if !defined _ARM_ASM_

  #ifdef __cplusplus
    extern "C" {
  #endif
//      L4_Word_t int_lock_ext( void );
//      void int_free_ext( void );
  #ifdef __cplusplus
    }
  #endif

//  #define  INTLOCK( )  { L4_Word_t sav = int_lock_ext();
//  #define  INTFREE( )  if(!sav) (void)int_free_ext();}

#endif

#if !defined _ARM_ASM_

//  #define  INTLOCK_SAV( sav_var )  sav_var = int_lock_ext()
//  #define  INTFREE_SAV( sav_var )  if(!sav_var) (void)int_free_ext()

#endif
#endif

#undef HWIO_INTLOCK
#undef HWIO_INTFREE

#if defined(FEATURE_WINCE_OAL) || defined(FEATURE_WINCE_BOOTLOADER) || defined(FEATURE_WINCE) || defined(_ARM_ASM_)

  #define HWIO_INTLOCK()
  #define HWIO_INTFREE()

#elif defined(IMAGE_QDSP6_PROC)
#error code not present
#elif defined(INTLOCK_SAV)

  #define HWIO_INTLOCK()
  #define HWIO_INTFREE()

//  #define HWIO_INTLOCK() { uint32 hwio_isav; INTLOCK_SAV(hwio_isav)
//  #define HWIO_INTFREE()   INTFREE_SAV(hwio_isav); }

#else

  #define HWIO_INTLOCK()
  #define HWIO_INTFREE()

//  #define HWIO_INTLOCK()   INTLOCK()
//  #define HWIO_INTFREE()   INTFREE()

#endif

#define __inpdw(port)       (*((volatile uint32 *) (port)))
#define in_dword_masked(addr, mask) (__inpdw(addr) & (mask))
#define __outpdw(port, val) (*((volatile uint32 *) (port)) = ((uint32) (val)))
#define out_dword(addr, val) __outpdw(addr,val)
#define out_dword_masked_ns(io, mask, val, current_reg_content) \
  out_dword( io, ((current_reg_content & (uint32)(~(mask))) | \
                 ((uint32)((val) & (mask)))) )

#define CLK_CTL_REG_BASE                                                        (MSM_CLK_CTL_BASE + 0x00000000)
#define HWIO_GP_MD_REG_ADDR                                                     (CLK_CTL_REG_BASE      + 0x00000058)
#define HWIO_GP_MD_REG_PHYS                                                     (CLK_CTL_REG_BASE_PHYS + 0x00000058)
#define HWIO_GP_MD_REG_RMSK                                                     0xffffffff
#define HWIO_GP_MD_REG_SHFT                                                              0
#define HWIO_GP_MD_REG_IN                                                       \
        in_dword_masked(HWIO_GP_MD_REG_ADDR, HWIO_GP_MD_REG_RMSK)
#define HWIO_GP_MD_REG_INM(m)                                                   \
        in_dword_masked(HWIO_GP_MD_REG_ADDR, m)
#define HWIO_GP_MD_REG_OUT(v)                                                   \
        out_dword(HWIO_GP_MD_REG_ADDR,v)

#if 0
#define HWIO_GP_MD_REG_OUTM(m,v)                                                \
        HWIO_INTLOCK(); \
        out_dword_masked_ns(HWIO_GP_MD_REG_ADDR,m,v,HWIO_GP_MD_REG_IN); \
        HWIO_INTFREE()
#else
#define HWIO_GP_MD_REG_OUTM(m,v)                                                \
        out_dword_masked_ns(HWIO_GP_MD_REG_ADDR,m,v,HWIO_GP_MD_REG_IN); 
#endif

#define HWIO_GP_MD_REG_M_VAL_BMSK                                               0xffff0000
#define HWIO_GP_MD_REG_M_VAL_SHFT                                                     0x10
#define HWIO_GP_MD_REG_D_VAL_BMSK                                                   0xffff
#define HWIO_GP_MD_REG_D_VAL_SHFT                                                        0

#define HWIO_GP_NS_REG_ADDR                                                     (CLK_CTL_REG_BASE      + 0x0000005c)
#define HWIO_GP_NS_REG_PHYS                                                     (CLK_CTL_REG_BASE_PHYS + 0x0000005c)
#define HWIO_GP_NS_REG_RMSK                                                     0xffffffff
#define HWIO_GP_NS_REG_SHFT                                                              0
#define HWIO_GP_NS_REG_IN                                                       \
        in_dword_masked(HWIO_GP_NS_REG_ADDR, HWIO_GP_NS_REG_RMSK)
#define HWIO_GP_NS_REG_INM(m)                                                   \
        in_dword_masked(HWIO_GP_NS_REG_ADDR, m)
#define HWIO_GP_NS_REG_OUT(v)                                                   \
        out_dword(HWIO_GP_NS_REG_ADDR,v)

#if 0
#define HWIO_GP_NS_REG_OUTM(m,v)                                                \
        HWIO_INTLOCK(); \
        out_dword_masked_ns(HWIO_GP_NS_REG_ADDR,m,v,HWIO_GP_NS_REG_IN); \
        HWIO_INTFREE()
#else
#define HWIO_GP_NS_REG_OUTM(m,v)                                                \
        out_dword_masked_ns(HWIO_GP_NS_REG_ADDR,m,v,HWIO_GP_NS_REG_IN); 
#endif

#define HWIO_GP_NS_REG_GP_N_VAL_BMSK                                            0xffff0000
#define HWIO_GP_NS_REG_GP_N_VAL_SHFT                                                  0x10
#define HWIO_GP_NS_REG_GP_ROOT_ENA_BMSK                                              0x800
#define HWIO_GP_NS_REG_GP_ROOT_ENA_SHFT                                                0xb
#define HWIO_GP_NS_REG_GP_CLK_INV_BMSK                                               0x400
#define HWIO_GP_NS_REG_GP_CLK_INV_SHFT                                                 0xa
#define HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_BMSK                                        0x200
#define HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_SHFT                                          0x9
#define HWIO_GP_NS_REG_MNCNTR_EN_BMSK                                                0x100
#define HWIO_GP_NS_REG_MNCNTR_EN_SHFT                                                  0x8
#define HWIO_GP_NS_REG_MNCNTR_RST_BMSK                                                0x80
#define HWIO_GP_NS_REG_MNCNTR_RST_SHFT                                                 0x7
#define HWIO_GP_NS_REG_MNCNTR_MODE_BMSK                                               0x60
#define HWIO_GP_NS_REG_MNCNTR_MODE_SHFT                                                0x5
#define HWIO_GP_NS_REG_PRE_DIV_SEL_BMSK                                               0x18
#define HWIO_GP_NS_REG_PRE_DIV_SEL_SHFT                                                0x3
#define HWIO_GP_NS_REG_SRC_SEL_BMSK                                                    0x7
#define HWIO_GP_NS_REG_SRC_SEL_SHFT                                                      0

/* Variable for setting PWM in Force Out Set */
VibeInt32 g_nForce_32 = 0;

/* Global variables for LRA PWM M,N and D values.  */
VibeInt32 g_nLRA_GP_CLK_M = GP_CLK_M_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_N = GP_CLK_N_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_D = GP_CLK_D_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_PWM_MUL = IMM_PWM_MULTIPLIER;

/* FUJITSU:2011-07-09 VIB for gp_clk start */
extern int vib_set_flag;
/* FUJITSU:2011-07-09 VIB for gp_clk end */

static int vibe_set_pwm_freq(int nForce)
{
	/* FUJITSU:2011-07-09 VIB for gp_clk */
	if(vib_set_flag != 0){
		DbgOut((KERN_DEBUG "don't set pwm.\n"));
	}
	else if(nForce == 0) //Generate Clock with appropriate frequency(around 22KHz) and 50% duty cycle.
	{
		/* Put the MND counter in reset mode for programming */
	
		/* M/N:D counter */
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 0 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT);
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_N_VAL_BMSK, ~(g_nLRA_GP_CLK_N - g_nLRA_GP_CLK_M) << HWIO_GP_NS_REG_GP_N_VAL_SHFT);
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_ROOT_ENA_BMSK, 1 << HWIO_GP_NS_REG_GP_ROOT_ENA_SHFT);
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_CLK_INV_BMSK, 0 << HWIO_GP_NS_REG_GP_CLK_INV_SHFT);
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_BMSK, 1 << HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_SHFT);
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_RST_BMSK, 0 << HWIO_GP_NS_REG_MNCNTR_RST_SHFT);
		/* Dual-edge mode => 2*/
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_MODE_BMSK, 2 << HWIO_GP_NS_REG_MNCNTR_MODE_SHFT); 
		/* P: 0 => Freq/1, 1 => Freq/2, 3 => Freq/4 */
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_PRE_DIV_SEL_BMSK, 0 << HWIO_GP_NS_REG_PRE_DIV_SEL_SHFT);
		/* S : 0 => TXCO(19.2MHz), 1 => Sleep XTAL(32kHz) */
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_SRC_SEL_BMSK, 0 << HWIO_GP_NS_REG_SRC_SEL_SHFT); 
		HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_M_VAL_BMSK, g_nLRA_GP_CLK_M << HWIO_GP_MD_REG_M_VAL_SHFT);
		HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_D_VAL_BMSK, ( ~((VibeInt16)g_nLRA_GP_CLK_D << 1) ) << HWIO_GP_MD_REG_D_VAL_SHFT);

		/* Enable M/N counter */
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 1 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT); 
	}
	else //Clock is already running, so control only D register here.
	{	
		g_nForce_32 = ((nForce * (g_nLRA_GP_CLK_D - 2)) / 127 ) + g_nLRA_GP_CLK_D;
		HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_D_VAL_BMSK, ( ~((VibeInt16)g_nForce_32 << 1) ) << HWIO_GP_MD_REG_D_VAL_SHFT);
	}
	return VIBE_S_SUCCESS;

}

/* FUJITSU:2011-04-29 VIB for 2-2 end */

/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
//#error Please review the code between the #if and #endif

	if (g_bAmpEnabled)
	{
		DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpDisable.\n"));

		g_bAmpEnabled = false;

#if 0
		mhn_gpio_set_level(GPIO_EN, GPIO_LEVEL_LOW);
		mz_ops.bstat &= ~HN_BATTERY_MOTOR;
#endif

		/* FUJITSU:2011-04-29 VIB start */
		if(0x0B <= system_rev) {
			pwm_disable(pwm);
		}
		else{
			clk_disable(android_vib_clk);
		}
		/* FUJITSU:2011-04-29 VIB end */

	   	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HAP_ENABLE), 0);

		/* FUJITSU:2011-04-29 VIB start */
		if(0x0B <= system_rev) {
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(HAP_LVL_SHFT_PMIC_GPIO), 0);
		}
		else{
			if (gpio_tlmm_config(GPIO_CFG(HAP_LVL_SHFT_MSM_GPIO, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE)){
	
				pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
				       __func__, HAP_LVL_SHFT_MSM_GPIO);
			}
			gpio_set_value(HAP_LVL_SHFT_MSM_GPIO, 0);
		}	
		/* FUJITSU:2011-04-29 VIB end */

		/* FUJITSU:2011-04-26 VIB start */
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_CVIB_LDO_ENABLE), 0);
		/* FUJITSU:2011-04-26 VIB end */
	}

	return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
//#error Please review the code between the #if and #endif

	int rc = 0;

	/* FUJITSU:2011-07-09 VIB for gp_clk share*/
	if ( !g_bAmpEnabled && (vib_set_flag == 0) )
	{
		DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpEnable.\n"));

		g_bAmpEnabled = true;mdelay(1);

#if 0
		/* 
		** Ensure the PWM frequency is at the expected value. These 2 lines of code
		** can be removed if no other application alters the PWM frequency.
		*/
		PWM_CTRL  = 0;				  /* 13Mhz / (0 + 1) = 13MHz */
		PWM_PERIOD = PWM_DUTY_MAX;	  /* 13Mhz / (PWM_DUTY_MAX + 1) = 22.4kHz */

		/* Set duty cycle to 50% */
		PWM_DUTY = (PWM_DUTY_MAX+1)>>1; /* Duty cycle range = [0, PWM_DUTY_MAX] */

		/* Enable amp */
		mhn_gpio_set_level(GPIO_EN, GPIO_LEVEL_HIGH);
		mz_ops.bstat |= HN_BATTERY_MOTOR;
#endif

		/* FUJITSU:2011-04-26 VIB start */
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_CVIB_LDO_ENABLE), 1);
		/* FUJITSU:2011-04-26 VIB end */

		/* FUJITSU:2011-04-29 VIB start */
		if(0x0B <= system_rev) {
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(HAP_LVL_SHFT_PMIC_GPIO), 1);
			// Start 50% duty cycle
			rc = pwm_config(pwm, ((PWM_DUTY_MAX+1) >> 1), PWM_DUTY_MAX+1);
			if (rc < 0)printk("pwm_config ERROR!\n");
			rc = pwm_enable(pwm);
			if (rc < 0)printk("pwm_enable ERROR!\n");

			mdelay(1);
		}
		else{
			mdelay(1);
			if (gpio_tlmm_config(GPIO_CFG(HAP_LVL_SHFT_MSM_GPIO, 3, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE)){

				pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
				       __func__, HAP_LVL_SHFT_MSM_GPIO);
			}
			gpio_set_value(HAP_LVL_SHFT_MSM_GPIO, 1);

			clk_enable(android_vib_clk);
			// Start 50% duty cycle
			vibe_set_pwm_freq(0);
		}
		/* FUJITSU:2011-04-29 VIB end */

		/* FUJITSU:2011-04-26 VIB start */
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HAP_ENABLE), 1);
		/* FUJITSU:2011-04-26 VIB end */

	}

	return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
//#error Please review the code between the #if and #endif

	DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Initialize.\n"));

	/* FUJITSU:2011-07-06 VIB start */
	// g_bAmpEnabled = true;   /* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */
	g_bAmpEnabled = false;
	/* FUJITSU:2011-07-06 VIB end */

	/* FUJITSU:2011-04-29 VIB start */
	if(0x0B <= system_rev) {
		pwm = pwm_request(0, "ISA1000");
	}
	else{
		android_vib_clk = clk_get(NULL,"gp_clk");
		if(IS_ERR(android_vib_clk)) {
			printk("android vib clk failed!!!\n");
		}
		else {
			//printk("THNAK YOU!!\n");
		}
	}
	/* FUJITSU:2011-04-29 VIB end */

	/* 
	** Disable amp.
	** If multiple actuators are supported, please make sure to call
	** ImmVibeSPI_ForceOut_AmpDisable for each actuator (provide the actuator index as
	** input argument).
	*/
	ImmVibeSPI_ForceOut_AmpDisable(0);

	/* FUJITSU:2011-07-06 VIB start */
   	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HAP_ENABLE), 0);

	if(0x0B <= system_rev) {
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(HAP_LVL_SHFT_PMIC_GPIO), 0);
	}
	else{
		if (gpio_tlmm_config(GPIO_CFG(HAP_LVL_SHFT_MSM_GPIO, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE)){
				pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
				       __func__, HAP_LVL_SHFT_MSM_GPIO);
			}
		gpio_set_value(HAP_LVL_SHFT_MSM_GPIO, 0);
	}	
	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_CVIB_LDO_ENABLE), 0);
	/* FUJITSU:2011-07-06 VIB end */

#if 0
	/* 
	** PWM frequency:
	** The PWM frequency must be set to a fixed value and shouldn't change
	** during the lifetime of the app. The ideal solution would be to use a
	** frequency value between 20kHz and 50kHz. A frequency value slightly
	** outside of the above limits should still work and be compliant with
	** TSP requirements (please refer to the TSP integration guide for
	** further information).
	*/

	/* 22.4kHz PWM, duty cycle 50% */
	PWM_CTRL = 0;				   /* 13Mhz / (0 + 1) = 13MHz */
	PWM_PERIOD = PWM_DUTY_MAX;	  /* 13Mhz / (PWM_DUTY_MAX + 1) = 22.4kHz */
	PWM_DUTY = (PWM_DUTY_MAX+1)>>1; /* Duty cycle range = [0, PWM_DUTY_MAX] */
#endif

	return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
//#error Please review the code between the #if and #endif

	DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Terminate.\n"));

	/* 
	** Disable amp.
	** If multiple actuators are supported, please make sure to call
	** ImmVibeSPI_ForceOut_AmpDisable for each actuator (provide the actuator index as
	** input argument).
	*/
	ImmVibeSPI_ForceOut_AmpDisable(0);

#if 0
	/* Set PWM frequency */
	PWM_CTRL  = 0;				  /* 13Mhz / (0 + 1) = 13MHz */
	PWM_PERIOD = PWM_DUTY_MAX;	  /* 13Mhz / (PWM_DUTY_MAX + 1) = 22.4kHz */

	/* Set duty cycle to 50% */
	PWM_DUTY = (PWM_DUTY_MAX+1)>>1; /* Duty cycle range = [0, PWM_DUTY_MAX] */
#endif

	/* FUJITSU:2011-04-29 VIB start */
	if(0x0B <= system_rev) {
		pwm_free(pwm);
	}
	/* FUJITSU:2011-04-29 VIB end */

	return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set PWM duty cycle
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
//#error Please review the code between the #if and #endif

	/* FUJITSU:2011-04-29 VIB < 2-2 start */
	VibeUInt32 nTmp;
	unsigned int duty;
	int rc = 0;
	/* FUJITSU:2011-04-29 VIB < 2-2 end */
	VibeInt8 nForce;
	duty = 0;
	switch (nOutputSignalBitDepth)
	{
		case 8:
			/* pForceOutputBuffer is expected to contain 1 byte */
			if (nBufferSizeInBytes != 1){
				DbgOut((KERN_ERR "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d \n",
						nBufferSizeInBytes ));
				return VIBE_E_FAIL;
			}
			nForce = pForceOutputBuffer[0];
			break;
		case 16:
			/* pForceOutputBuffer is expected to contain 2 byte */
			if (nBufferSizeInBytes != 2) return VIBE_E_FAIL;

			/* Map 16-bit value to 8-bit */
			nForce = ((VibeInt16*)pForceOutputBuffer)[0] >> 8;
			break;
		default:
			/* Unexpected bit depth */
			return VIBE_E_FAIL;
	}

printk(KERN_DEBUG "[TSPDRV] system_rev = %d\n", system_rev); 

	if (nForce == 0)
	{
		/* Set 50% duty cycle */

		/* FUJITSU:2011-04-29 VIB start */
		if(0x0B <= system_rev) {
			duty = (PWM_DUTY_MAX+1)>>1;
		}
		else{
			ImmVibeSPI_ForceOut_AmpDisable(0);
		}
		/* FUJITSU:2011-04-29 VIB end */
	}
	else
	{

		/* FUJITSU:2011-04-29 VIB start */
		if(0x0B <= system_rev) {
			/* Map force from [-127, 127] to ]0, PWM_DUTY_MAX[ */
			/* Approximate it by mapping from [-128, 128] to [0, PWM_DUTY_MAX+1] */
			nTmp = ((nForce+128) * (PWM_DUTY_MAX+1)) >> 8;

			/* 
			** The above mapping will output PWM duty cycle in [0, PWM_DUTY_MAX] range.
			** For PWM_DUTY_MAX == 579, max duty cycle is 98.6%.
			** As long as we don't hit the 100% mark, we're OK
			*/
			duty = nTmp;
		}
		else{
			ImmVibeSPI_ForceOut_AmpEnable(0);
			vibe_set_pwm_freq(nForce);
		}
		/* FUJITSU:2011-04-29 VIB end */
	}

	/* FUJITSU:2011-05-31 VIB start */
	if(0x0B <= system_rev) {
		rc = pwm_config(pwm, duty, PWM_DUTY_MAX+1);
		if (rc < 0)printk("pwm_config ERROR!\n");
	}
	/* FUJITSU:2011-05-31 VIB end */

	return VIBE_S_SUCCESS;
}

/*
** Called to set force output frequency parameters
*/
//IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
//{
//#error Please review the code between the #if and #endif

#if 0 
	#error  "The OEM must handle different frequency parameters here"
#endif

//	return VIBE_S_SUCCESS;
//}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
//#error Please review the code between the #if and #endif

#if 0   /* The following code is provided as a sample. Please modify as required. */
	if ((!szDevName) || (nSize < 1)) return VIBE_E_FAIL;

	DbgOut((KERN_DEBUG "ImmVibeSPI_Device_GetName.\n"));

	strncpy(szDevName, "Generic Linux Device", nSize-1);
	szDevName[nSize - 1] = '\0';	/* make sure the string is NULL terminated */
#endif

	return VIBE_S_SUCCESS;
}
