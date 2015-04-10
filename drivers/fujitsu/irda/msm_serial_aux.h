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

#ifndef __MSM_SERIAL_AUX__
#define __MSM_SERIAL_AUX__

/*
 * define IOCTL macro
 */
#define MSM_AUX_IOC_TYPE  0x56			/* 8bit */
#define MSM_AUX_IOCTL_CHG_IRDA         _IO  ( MSM_AUX_IOC_TYPE, 0 )
#define MSM_AUX_IOCTL_CHG_UART         _IO  ( MSM_AUX_IOC_TYPE, 1 )
#define MSM_AUX_IOCTL_GET_IRDA         _IOR ( MSM_AUX_IOC_TYPE, 2, unsigned long )

/* UART MemBase */
#if 0
#define THIS_UART_MEMBASE	0xACB00000	/* MSM1				*/
#else
#define THIS_UART_MEMBASE	0xA3200000	/* DM2				*/
#endif
#define THIS_UART_MEMSIZE   4096		/* align page size */

/* UART DM regs offset */
#define UART_MR1			0x0000
#define UART_MR2			0x0004
#define UART_SR				0x0008
#define UART_TF				0x000C
#define UART_CR				0x0010
#define UART_IMR			0x0014
#define UART_IPR			0x0018
#define UART_TFWR			0x001C
#define UART_RFWR			0x0020
#define UART_HCR			0x0024
#define UART_MREG			0x0028
#define UART_NREG			0x002C
#define UART_DREG			0x0030
#define UART_MNDREG			0x0034
#define UART_IRDA			0x0038
#define UART_MISR_MODE		0x0040
#define UART_MISR_RESET		0x0044
#define UART_MISR_EXPORT	0x0048
#define UART_MISR_VAL		0x004C
#define UART_TEST_CTRL		0x0050

#define UART_RF				0x000C
#define UART_MISR			0x0010
#define UART_ISR			0x0014


#endif /* __MSM_SERIAL_AUX__ */

