#ifndef	__WAVEIF_H__
#define	__WAVEIF_H__

//#include <linux/types.h>
#include <linux/ioctl.h>

//#define WAVEIF_LOAD_FIRM  _IOW('w', 0x01, int)
//#define WAVEIF_RESET      _IOW('w', 0x02, int)
//#define WAVEIF_TEST_BOOT  _IOW('w', 0x03, int)
//#define	WAVEIF_GPIO_RESET _IOW('w', 0x04, int)	// 1:set, 0:reset
//#define	WAVEIF_CLK_B	  _IOW('w', 0x05, int)	// 1:enable, 0:disable

#define	WAVEIF_CANCEL_READ	_IOW('w', 0x06, int)	//

#endif	//	__WAVEIF_H__
