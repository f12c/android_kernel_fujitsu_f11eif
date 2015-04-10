/*
 * COPYRIGHT(C) 2011 FUJITSU LIMITED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
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
#ifndef _CYTTSP_IF_H_
#define _CYTTSP_IF_H_

#include <linux/ioctl.h>

/* DeviceStatus */
enum devicepowerstatus
	{
	DEEPSLEEP        = 0x00,   /* DeepSleep     */
	WAKEUP,                    /* WakeUp        */
	HARDRESET                  /* HardReset     */
	};

#define CYTTSP_IOCTL_MAGIC 'c'

#define IOCTL_SET_POWERMODE   _IOR(CYTTSP_IOCTL_MAGIC, 1, int)
#define IOCTL_GET_FIRMVERSION _IOW(CYTTSP_IOCTL_MAGIC, 2,  char * )
#define IOCTL_DO_CALIBRATION  _IO(CYTTSP_IOCTL_MAGIC, 3)

#endif // _CYTTSP_IF_H_
