/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/


#ifndef _I2CUART_H_
#define _I2CUART_H_

/* for Device Node */
#define I2CUART_PATHNAME	"/dev/i2cuart"

/* for IOCTL */
#define I2CUART_IOCTL_FLUSH		1


struct i2cuart_platform_data {
	int irq;
};

#endif	/* _I2CUART_H_ */
