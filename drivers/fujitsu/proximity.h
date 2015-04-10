/* 
  Proximty Sensor Driver

  This is the driver code for  Proximty sensor
  Copyiiright (C) 2010 TOSHIBA CORPORATION Mobile Communication Company.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
*/
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/


#ifndef _LINUX_PROXIMITY_H
#define _LINUX_PROXIMITY_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define	TVTUNER_IOCTL_BASE	'T'

struct tvtuner_info {
	__u32 options;		/* Options the card/driver supports */
	__u32 firmware_version;	/* Firmware version of the card */
	__u8  identity[32];	/* Identity of the board */
};


/* IOCTL*/
#define	TVTUNER_IOCTL_GETSUPPORT    _IOR(TVTUNER_IOCTL_BASE,  0, struct tvtuner_info)
#define	IOCTL_PRODRV_ON             _IOWR(TVTUNER_IOCTL_BASE,  1, int)
#define	IOCTL_PRODRV_OFF            _IOWR(TVTUNER_IOCTL_BASE,  2, int)





#ifdef __KERNEL__


#endif	/* __KERNEL__ */

#endif /* _LINUX_PROXIMITY_H */
