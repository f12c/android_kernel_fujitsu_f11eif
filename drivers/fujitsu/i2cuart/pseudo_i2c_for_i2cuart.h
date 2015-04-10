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


#ifndef PSEUDO_I2C_FOR_I2CUART_H
#define PSEUDO_I2C_FOR_I2CUART_H

/* I2C Control Informatino Sturcture */
typedef struct{
    uint8_t     slave_addr;     /* slave address                      */
    uint8_t     *buf_ptr;       /* pointer to buffer in caller space  */
    uint16_t    offset;         /* offset in I2C device to read/write */
    uint16_t    len;            /* count of bytes to transfer         */
} I2C_INFO;

/* prototypes                       */
#ifdef __cplusplus
extern "C" {
#endif

extern bool pse_i2c_init(void);
extern bool pse_i2c_read( I2C_INFO *cmd_ptr );
extern bool pse_i2c_write( I2C_INFO *cmd_ptr );

#ifdef __cplusplus
}
#endif


#endif /* PSEUDO_I2C_FOR_I2CUART_H */
