/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/
#ifndef hscd_h
#define hscd_h
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/compass.h>

/*--------------------------------------------------------------------
Interrupt
--------------------------------------------------------------------*/

#define JIKI_DRDY_GPIO 97

#define HSCD_IRQ()  gpio_to_irq(JIKI_DRDY_GPIO)

/*--------------------------------------------------------------------
Register address
--------------------------------------------------------------------*/
#define HSCD_STBA    0x0B // R:  Self test modeA
#define HSCD_STBB    0x0C // R:  Self test modeB
#define HSCD_INFOLSB 0x0D // R:  Information register(LSB)
#define HSCD_INFOMSB 0x0E // R:  Information register(MSB)
#define HSCD_WIA     0x0F // R:  "Who I Am" register
#define HSCD_XLSB    0x10 // R:  X Axis Output data register(LSB)
#define HSCD_XMSB    0x11 // R:  X Axis Output data register(MSB)
#define HSCD_YLSB    0x12 // R:  Y Axis Output data register(LSB)
#define HSCD_YMSB    0x13 // R:  Y Axis Output data register(MSB)
#define HSCD_ZLSB    0x14 // R:  Z Axis Output data register(LSB)
#define HSCD_ZMSB    0x15 // R:  Z Axis Output data register(MSB)
#define HSCD_INS     0x16 // R:  Interrupt Source register
#define HSCD_STAT    0x18 // R:  Status register
#define HSCD_INL     0x1A // R:  Interrupt Latch register
#define HSCD_CNTL1   0x1B // RW: Control register1
#define HSCD_CNTL2   0x1C // RW: Control register2
#define HSCD_CNTL3   0x1D // RW: Control register3
#define HSCD_CNTL4_V2   0x28 // RW: Control register4(ver2)
#define HSCD_CNTL4_V3   0x1E // RW: Control register4(ver3)
#define HSCD_INC_V2     0x1E // RW: Interrupt Control register(ver2)
#define HSCD_INC_V3     0x1F // RW: Interrupt Control register(ver3)
#define HSCD_AMP     0x1F // RW: AMP Control register(ver2)

#define HSCD_XOFFLSB 0x20 // RW: X Offset Drift value register(LSB)
#define HSCD_XOFFMSB 0x21 // RW: X Offset Drift value register(MSB)
#define HSCD_YOFFLSB 0x22 // RW: Y Offset Drift value register(LSB)
#define HSCD_YOFFMSB 0x23 // RW: Y Offset Drift value register(MSB)
#define HSCD_ZOFFLSB 0x24 // RW: Z Offset Drift value register(LSB)
#define HSCD_ZOFFMSB 0x25 // RW: Z Offset Drift value register(MSB)
#define HSCD_ITHRLSB 0x26 // RW: Interrupt Threshold register(LSB)
#define HSCD_ITHRMSB 0x27 // RW: Interrupt Threshold register(MSB)
#define HSCD_PRET    0x30 // RW: Preset time register
#define HSCD_TEMP    0x31 // RW: Temperature data register

// The maximum number of consecutive register numbers (XLSB-YMSB=6)
// (I2C It uses it for the size decision in the sending and receiving buffer.)
#define HSCD_MAX_CONTIGUOUS_REGS (HSCD_ZMSB - HSCD_XLSB + 1)

/*--------------------------------------------------------------------
Register Content
--------------------------------------------------------------------*/
// CNTL1 Register Content Ver2
#define HSCD_MODE_STANBY_V2      0x02 // Standby Mode(ver2)
#define HSCD_MODE_NORMAL_V2      0x80 // Active normality Mode(ver2)
#define HSCD_MODE_FORCESTATE_V2  0x82 // Active forcestate Mode(ver2)
// CNTL1 Register Content Ver3
#define HSCD_MODE_STANBY_V3      0x22 // Standby Mode(ver3)
#define HSCD_MODE_NORMAL_V3      0xA0 // Active normality Mode(ver3)
#define HSCD_MODE_FORCESTATE_V3  0xA2 // Active forcestate Mode(ver3)
#define HSCD_FORCESTATE_12BIT    0xA2 // Range 12bit
#define HSCD_FORCESTATE_13BIT    0xE2 // Range 13bit

// CNTL2 Register Content Ver2
#define HSCD_CNTL2_CLEAR_V2      0x00 // DRDY OFF(ver2)
#define HSCD_CNTL2_DRDYON_V2     0x08 // DRDY ON (DRDY active HIGH)(ver2)
// CNTL2 Register Content Ver3
#define HSCD_CNTL2_CLEAR_V3      0x76 // DRDY OFF(ver3)
#define HSCD_CNTL2_DRDYON_V3     0x7E // DRDY ON (DRDY active HIGH)(ver3)

// CNTL3 Register Content
#define HSCD_CNTL3_STANBY     0x00 // Standby Mode
#define HSCD_CNTL3_SRST       0x80 // Soft reset
#define HSCD_CNTL3_FORCE      0x40 // Active forcestate Mode Measurement
#define HSCD_CNTL3_STCA       0x20 // self test modeA
#define HSCD_CNTL3_STCB       0x10 // self test modeB
#define HSCD_CNTL3_TCS        0x02 // Active temperature Measurement
#define HSCD_CNTL3_OCL        0x01 // Active offset calibration Measurement


#define HSCD_SELFTEST_DEF     0x55 // self test default value
#define HSCD_SELFTEST_PASS    0xAA // self test pass value

#define HSCD_STAT_TRDY        0x80 // temp measure completion

/*--------------------------------------------------------------------
HSCD Peculiar instance data
--------------------------------------------------------------------*/
typedef struct hscd {
	wait_queue_head_t measure_end_queue; // Present measurement completion queue
	//wait_queue_head_t measure_start_queue; // Next measurement beginning queue
	compass_data_t measurement; // The last result of a measurement

	// Number of clients of measuring data reading waiting
	// unsigned wait_count;     // Unused

	int use_count;              // Frequency that has been opened now
	unsigned max_retry_count;   // Maximum retrying frequency

	unsigned error_count;       // This time ioctl() error frequency
	unsigned retry_count;       // This time ioctl() retrying frequency
	u_long total_error_count;   // Total when system starts
	u_long total_retry_count;   // Total when system starts

	// State of operation
	volatile uint8_t state;		// Status
#define HSCD_STAT_IDLE		0	// State of idol
#define HSCD_STAT_MEASURING	1	// HSCD measuring
#define HSCD_STAT_MEASURED	2	// Measurement completion
#define HSCD_STAT_AVAILABLE	3	// The measuring data can be read
	//uint8_t verify;		// HSCD Collation when register is read and written

	// HSCD register
	hscd_registers_t reg;
} hscd_t;

// Mode (MS1.MODE[1:0]) get.
#define HSCD_MODE()           (hscd->reg.cntl1 & HSCD_MODE_MASK)

// Mode (MS1.MODE[1:0]) set.
#define HSCD_SETMODE(mode)    (hscd->reg.cntl1 = (mode))

// State of operation
#define HSCD_STATE()              (hscd->state)
#define HSCD_SETSTATE(newState)   (HSCD_STATE() = (newState))

/*--------------------------------------------------------------------
HSCD Timing data
--------------------------------------------------------------------*/
// Reset waiting (us)
#define HSCD_Trnw 500

// Waiting time when mode is shifted (ms)
#define HSCD_Twat 5

// Measurement time maximum value (us)
//#define HSCD_MAX_MEASUREMENT_TIME 15100
#define HSCD_MAX_MEASUREMENT_TIME 5000

#define HSCD_MAX_MEASUREMENT_COUNT 4

/*--------------------------------------------------------------------
--------------------------------------------------------------------*/
#ifdef __KERNEL__
extern hscd_t hscd[1];

int hscd_ioctl(struct inode*, struct file*, unsigned, u_long);
int hscd_open(struct inode*, struct file*);
int hscd_release(struct inode*, struct file*);
void hscd_module_init(void);
void hscd_module_term(void);

//void hscd_self_test(hscd_t *hscd);
#endif /* __KERNEL__ */
#endif /* hscd_h */
