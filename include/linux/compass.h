/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/
#ifndef _COMPASS_H
#define _COMPASS_H
#ifdef __KERNEL__
#define COMPASS_DEBUG
#else /* __KERNEL__ */
#include <stdint.h>
#include <sys/time.h>
#endif /* __KERNEL__ */
#include <linux/ioctl.h>

/*--------------------------------------------------------------------
Device Number
--------------------------------------------------------------------*/
#define COMPASS_DEVNO_MAJOR 0 // Major Number (Automatic Numbering)
#define COMPASS_DEVNO_MINOR 0 // Minor Number

#define COMPASS_DEVNO MKDEV(COMPASS_DEVNO_MAJOR, COMPASS_DEVNO_MINOR)

#define COMPASS_NDEVICES    1 // Number of Devices

/*--------------------------------------------------------------------
magnetism measurement data
--------------------------------------------------------------------*/
typedef struct compass_data {
  struct timeval time;    // Measuring Time

  // Measurement data of hscd
  int8_t temperature;    // Temperature magnetic sensors (TEMP)
  int16_t magnetism[3];   // Magnetic vector (DATA[XYZ])

  // condition of case
  uint8_t case_open;     // condition of case (open <=> true)

  // Number of errors after device opened.
  u_long total_error_count; // Number of errors
  u_long total_retry_count; // Number of retry

  int8_t measurement_range;  // Measurement range

#define HSCD_MEASUREMENT_RANGE_12    0  // 12bit
#define HSCD_MEASUREMENT_RANGE_13    1  // 13bit

} compass_data_t;

/*--------------------------------------------------------------------
hscd Register
--------------------------------------------------------------------*/
// Parameters for 3-axis
typedef union hscd_xyz {
  int16_t xyz[3];
  struct { int16_t x, y, z; };
} hscd_xyz_t;

// Register all of hscd
//   Interrupt Source INS 
//   Interrupt latches INL 
//   Interrupt Control INC 
//   Interrupt Threshold Register ITHR
//   not use the interrupt line is not connected.
typedef struct hscd_registers {
  uint8_t mask;     // Select read / write registers.
#define HSCD_REGMASK_DATA     0x01 // Measurement data (TEMP,DATA[XYZ])
#define HSCD_REGMASK_OFFSET   0x02 // Sensor offset (OFF[XYZ])
#define HSCD_REGMASK_AMPGAIN  0x04 // Sensor amplifier gain (AMP)
#define HSCD_REGMASK_ADJUST   \
  (HSCD_REGMASK_OFFSET | HSCD_REGMASK_AMPGAIN)
#define HSCD_REGMASK_ALL      \
  (HSCD_REGMASK_DATA | HSCD_REGMASK_ADJUST)

  // Status and sensor measurements
  int8_t temp;     // Temperature sensor measurements (TEMP)
  hscd_xyz_t data;  //magnetism sensor measurements (DATAX, DATAY, DATAZ)

  // Mode and set value register
  uint8_t ins;      // Interrupt Source (INS Unused)
  uint8_t stat;     // Status
  uint8_t inl;      // Interrupt latches (INL Unused)
  uint8_t cntl1;    // Mode (CNTL1)
  uint8_t cntl2;    // Mode (CNTL2)
  uint8_t cntl3;    // Mode (CNTL3)
  uint8_t cntl4;    // Mode (CNTL4)
  uint8_t inc;      // Interrupt Control (INC Unused)
  uint16_t ithr;    // Interrupt Threshold (ITHR Unused)
  uint8_t pret;     // Preset time (PRET)
  
  uint8_t amp;     // magnetism sensor amplifier gain (AMP)
  hscd_xyz_t off;   // magnetism sensor offset (OFFX, OFFY, OFFZ)

  // condition of case
  uint8_t case_open;     // condition of case (open <=> true)

  struct i2c_client* st_client;  //I2C Client information

} hscd_registers_t;

/*--------------------------------------------------------------------
Self Test
--------------------------------------------------------------------*/
typedef struct selftest_data {
  // setting value
  int8_t test_mode;    // test mode(A or B)
#define MODE_A           0  // test mode A
#define MODE_B           1  // test mode B
	
  // return value
  int8_t test_ret;     // test result(pass or fail)
#define SELF_TEST_PASS   0  // pass
#define SELF_TEST_FAIL   1  // fail

  uint8_t err_code;     // error code(Bmode fail only)
	
} selftest_data_t;

/*--------------------------------------------------------------------
IOCTL
--------------------------------------------------------------------*/
#define COMPASS_IOC_MAGIC   0xA2

#define COMPASS_IOC_GETDATA     _IOWR(COMPASS_IOC_MAGIC, 0x01, compass_data_t)
#define COMPASS_IOC_TEMP_COR    _IOWR(COMPASS_IOC_MAGIC, 0x02, compass_data_t)
#define COMPASS_IOC_OFFCAL      _IOW(COMPASS_IOC_MAGIC, 0x03, hscd_registers_t)
#define COMPASS_IOC_GETREGS     _IOR(COMPASS_IOC_MAGIC, 0x04, hscd_registers_t)
#define COMPASS_IOC_SETREGS     _IOW(COMPASS_IOC_MAGIC, 0x05, hscd_registers_t)
#define COMPASS_IOC_GETDATASTOP _IOW(COMPASS_IOC_MAGIC, 0x06, compass_data_t)
#define COMPASS_IOC_SELFTEST    _IOWR(COMPASS_IOC_MAGIC, 0x07, selftest_data_t)

/*====================== Definition for driver =====================*/
#ifdef __KERNEL__
/*--------------------------------------------------------------------
General-purpose macro definition
--------------------------------------------------------------------*/
#define OK      0

/*--------------------------------------------------------------------
Function  :Structure member's address is returned.
@param    : (1) type:Structure member's type.
            (2) base:Base address of structure.
            (3) offset:Byte offset of the structure member.
@retval   :Member's address.
--------------------------------------------------------------------*/
#define STRUCT_MEMBER(type, base, offset) \
  ((type*)((char*)(base) + (offset)))

/*--------------------------------------------------------------------
Log output
--------------------------------------------------------------------*/
#define compass_printk(logLevel, format, arg...) \
  printk("%s" format, logLevel, ##arg)

#endif /* __KERNEL__ */

/*--------------------------------------------------------------------
--------------------------------------------------------------------*/

#endif /* _LINUX_COMPASS_H */
