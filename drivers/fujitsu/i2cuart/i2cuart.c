/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*  INCLUDE                                                                   */
/*----------------------------------------------------------------------------*/
#include <asm/system.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>
#include <mach/pmic.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/hrtimer.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>

#include <linux/clk.h>
#include <linux/i2cuart.h>

/*----------------------------------------------------------------------------*/
/*  OPTIONS                                                                   */
/*----------------------------------------------------------------------------*/
/* for logging */
#define I2CUART_DEBUG	0
#define I2CUART_DEBUG2	0

#if I2CUART_DEBUG
#define I2CUART_DBG(x...)  printk(x)
#else
#define I2CUART_DBG(x...)
#endif

#if I2CUART_DEBUG2
#define I2CUART_DBG2(x...)  printk(x)
#else
#define I2CUART_DBG2(x...)
#endif


#define I2C_PSEUDO 1
#if I2C_PSEUDO
#include "pseudo_i2c_for_i2cuart.h"
#endif

/*----------------------------------------------------------------------------*/
/*  MACROS                                                                    */
/*----------------------------------------------------------------------------*/

#define MYNAME					"i2cuart"

#define I2CUART_SLAVE_ADDRESS	(0x6C)
#define I2C_TRANS_WRITE			(1)
#define I2C_TARNS_READ			(2)
#define I2C_RETRY_MAX			(5)

/*-------- MAX-IC register --------*/
/* interrupts */
#define REG_THR					(0x00)
#define REG_RHR					(0x00)
#define REG_INT_IRQEN			(0x01)
#define REG_INT_ISR				(0x02)
#define REG_INT_LSRTEN			(0x03)
#define REG_INT_LSR				(0x04)
#define REG_INT_SPCEN			(0x05)
#define REG_INT_SPC				(0x06)
#define REG_INT_STSEN			(0x07)
#define REG_INT_STS				(0x08)

/* uart modes */
#define REG_MODE1				(0x09)
#define REG_MODE2				(0x0A)
#define REG_LCR					(0x0B)

/* fifo control */
#define REG_FLOW_LVL			(0x0F)
#define REG_FIFO_TRG			(0x10)
#define REG_FIFO_TXLVL			(0x11)
#define REG_FIFO_RXLVL			(0x12)

/* flow control */
#define REG_FLOW				(0x13)

/* clock config */
#define REG_PLLCFG				(0x1A)
#define REG_BRGCFG				(0x1B)
#define REG_DIVLSB				(0x1C)
#define REG_DIVMSB				(0x1D)
#define REG_CLKSRC				(0x1E)

/* register value */
#define REG_INT_IRQEN_RX		(0x08)
#define REG_INT_ISR_RX			(0x08)
#define REG_INT_RFIFO_VAL		(0x40)
#define REG_INT_ISR_RFIFO_VAL	(0x40)
#define REG_INT_LSR_RXBRK_VAL	(0x10)
#define REG_INT_SPCEN_BRK_VAL	(0x10)
#define REG_INT_LSR_RXOVR_VAL	(0x02)

#define REG_MODE2_RST_VAL		(0x01)
#define REG_MODE2_FIFORST_VAL	(0x02)
#define REG_MODE2_RFIFO_INV_VAL	(0x08)

#define REG_FLOW_SWFLOW_VAL		(0xF8)
#define REG_FLOW_CTSRTS_VAL		(0x03)

#define REG_LCR_LENGTH_VAL		(0x03)
#define REG_LCR_STOPB_VAL		(0x04)

#define REG_DIVLSB_INITVAL		(0x01)

#define I2CUART_FIFO_SIZE		(128)
#define I2CUART_RESUME_DELAY	(20)	/* 20msec */

#define I2C_WRITE_BUFSIZE		(1024)
#define I2C_READ_BUFSIZE		(2048)
#define I2CUART_RINGBUFF_SIZE	(3072)

#define I2CUART_CHIP_BUFF_SIZE	(128)
#define I2CUART_WORKQUEUE_DELAY	(1000)

/*----------------------------------------------------------------------------*/
/*  LOCAL FUNCTION PROTOTYPES                                                 */
/*----------------------------------------------------------------------------*/
/*---------------------------  FILE OPERATIONS -------------------------------*/
static int i2cuart_fop_open(struct inode *inode, struct file *file);

static int i2cuart_fop_release(struct inode *inode, struct file *file);

static ssize_t i2cuart_fop_write(struct file *file, const char __user *user,
								size_t size, loff_t *loff);

static ssize_t i2cuart_fop_read(struct file *file, char __user *user,
								size_t size, loff_t *loff);

static int i2cuart_fop_ioctl(struct inode *node, struct file *file,
							unsigned int cmd, unsigned long arg);

static unsigned int i2cuart_fop_poll(struct file *file,
									struct poll_table_struct *ptable);

/*----------------------------  SUB FUNCTIONS -------------------------------*/
static irqreturn_t i2cuart_irq_handler(int irq, void *data);

static int i2cuart_flush(void);

static int i2cuart_reset(bool recovery);

static int i2cuart_set_termios(void);

static int i2cuart_i2c_write(unsigned char reg, char *data, size_t len);

static int i2cuart_i2c_read(unsigned char reg, char *data, size_t len);

static int i2cuart_request_irq(void);

static int i2cuart_data_to_buffer( int iSize );

static int i2cuart_data_from_buffer( char *pcBuf, int iSize );

static void i2cuart_work_bh(struct work_struct *work);

/*--------------------------- MODULE for kernel ------------------------------*/
static int i2cuart_probe(struct platform_device *pdev);
static int i2cuart_remove(struct platform_device *pdev);
static int i2cuart_suspend(struct platform_device *pdev, pm_message_t state);
static int i2cuart_resume(struct platform_device *pdev);
static int __init i2cuart_init(void);
static void __exit i2cuart_exit(void);

/*----------------------------------------------------------------------------*/
/*  VARIABLES                                                                 */
/*----------------------------------------------------------------------------*/
static struct i2c_adapter *gp_i2c_adapter;		/* I2C info					*/
static unsigned int g_irq;						/* IRQ number				*/
static 	wait_queue_head_t	g_wait_queue_irq;	/* IRQ wait					*/
static	u_int8_t g_wbuf[I2C_WRITE_BUFSIZE];		/* copy buf from user space */
static	u_int8_t g_tmpbuf[I2C_WRITE_BUFSIZE+1];	/* buf for uart->I2C		*/
static	u_int8_t g_rbuf[I2C_READ_BUFSIZE];		/* copy buf to user space	*/
static	u_int8_t g_open;						/* open conter				*/
static struct mutex g_i2ctxrx_mutex;			/* Mutex for Pedo-I2C		*/
static struct mutex g_fop_mutex;				/* Mutex for fop			*/
static struct mutex g_work_mutex;				/* Mutex for work			*/

/* for RingBuffer Control */
static char g_cTmpRdBuf[I2CUART_RINGBUFF_SIZE + 1];	/* Ring Buffer Area			*/
static unsigned short g_usRidx, g_usWidx;		/* Index					*/
static int g_iBufDataSize;						/* DataSize in Buffer		*/
static struct mutex g_i2cuart_en_mutex;			/* for Readable Flag Access */
static struct mutex g_i2cuart_rw_mutex;			/* for Ring Buffer Access	*/
static int g_i2cuart_readable;					/* for Notifying to HAL		*/

static struct clk *gp_i2cuart_clk;
static struct work_struct g_i2cuart_work_data;

/* file operations handler */
static struct file_operations g_i2cuart_fops = {
	.owner		=	THIS_MODULE,
	.open		=	i2cuart_fop_open,
	.release	=	i2cuart_fop_release,
	.write		=	i2cuart_fop_write,
	.read		=	i2cuart_fop_read,
	.poll		=	i2cuart_fop_poll,
	.ioctl		=	i2cuart_fop_ioctl
};

/* device inf */
static struct miscdevice i2cuart_ioctl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "i2cuart",
	.fops = &g_i2cuart_fops
};

/* platform driver inf */
static struct platform_driver g_i2cuart_platform_driver = {
	.probe	= i2cuart_probe,
	.remove	= i2cuart_remove,
	.suspend	= i2cuart_suspend,
	.resume	 = i2cuart_resume,
	.driver	= {
	.name	= MYNAME,
	.owner	= THIS_MODULE,
	}
};

/*----------------------------------------------------------------------------*/
/*  FILE OPERATIONS                                                           */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*  @abstract : open device                                                   */
/*  @param    : inode -- not use                                              */
/*              file  -- not use                                              */
/*  @return   : 0 ==  -- success                                              */
/*              0 >   -- error                                                */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static int i2cuart_fop_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	
	I2CUART_DBG2("%s start\n", __func__);

	if (g_open == 0) {
		ret = i2cuart_request_irq();
	}
	g_open++;

	if (ret > 0) {
		ret = 0;
	}

	return ret;
}

/*----------------------------------------------------------------------------*/
/*  @abstract : close device                                                  */
/*  @param    : inode -- not use                                              */
/*              file  -- not use                                              */
/*  @return   : 0                                                             */
/*  @note     : release IRQ,reset MAX-IC                                      */
/*----------------------------------------------------------------------------*/
static int i2cuart_fop_release(struct inode *inode, struct file *file)
{
	I2CUART_DBG2("%s start\n", __func__);

	if (g_open > 0) {
		g_open--;
	}
	if (g_open == 0) {
		free_irq(g_irq, gp_i2c_adapter);
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
/*  @abstract : send data                                                     */
/*  @param    : file  -- not use                                              */
/*              user  -- user's data buffer                                   */
/*              size  -- number of user's data byte                           */
/*              loff  -- not use                                              */
/*  @return   : 0 <=  -- TXed data bytes                                      */
/*              0 >   -- error                                                */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static ssize_t i2cuart_fop_write(struct file *file, const char __user *user,
								size_t size, loff_t *loff)
{
	int ret = 0;
	int index = 0;
	int w_size = 0;
	int rest_size = size;
	uint8_t buf = 0;


	I2CUART_DBG("%s start\n", __func__);

	if (size > I2C_WRITE_BUFSIZE) {
		printk("%s Too large area... size is %d\n", __func__,size);
		return (-EFAULT);
	}

	mutex_lock(&g_fop_mutex);

	memset(g_wbuf, '\0', I2C_WRITE_BUFSIZE);

	if (copy_from_user(g_wbuf, user, size) != 0) {
		printk("%s Can't copy from user area.\n", __func__);
		mutex_unlock(&g_fop_mutex);
		return (-EFAULT);
	}

	for(;index < size;) {
		/* calculate writable byte count */
		ret = i2cuart_i2c_read(REG_FIFO_TXLVL, &buf, 1);
		if ( (ret < 0) || (buf >= I2CUART_FIFO_SIZE) ) {
			printk("%s Can't get TxFIFO size. ret[%d] buf[0x%x] retval[%d] \n", __func__, ret, buf, ((ret < 0) ? ret : (-1)));
			mutex_unlock(&g_fop_mutex);
			return ((ret < 0) ? ret : (-1));
		}
		if(rest_size > (I2CUART_FIFO_SIZE - buf)) {
			w_size = I2CUART_FIFO_SIZE - buf;
			rest_size -= w_size;
		} else {
			w_size = rest_size;
			rest_size = 0;
		}

		I2CUART_DBG2("%s Req size %d, FIFO :%d, w_size %d, rest_size %d.\n",
						__func__, size, buf, w_size, rest_size);

		/* TxDisable  */
		buf = 0x82;
		ret = i2cuart_i2c_write(REG_MODE1, &buf, 1);
		if (ret < 0) {
			printk("%s Can't Write REG_MODE1 ret[%d].\n", __func__,ret);
			mutex_unlock(&g_fop_mutex);
			return (-EFAULT);
		}
		
		ret = i2cuart_i2c_write(REG_THR, &g_wbuf[index], w_size);
		if (ret < 0) {
			printk("%s Can't send idx[%d] ret[%d].\n", __func__,index,ret);
			/* Tx Enable */
			buf = 0x80;
			ret = i2cuart_i2c_write(REG_MODE1, &buf, 1);
			if (ret < 0) {
				printk("%s Can't Enable Tx. ret[%d]\n", __func__,ret);
			}
			mutex_unlock(&g_fop_mutex);
			return ret;
		}

		/* increase w-index */
		index += w_size;

		/* Tx Enable */
		buf = 0x80;
		ret = i2cuart_i2c_write(REG_MODE1, &buf, 1);
		if (ret < 0) {
			printk("%s Can't Write REG_MODE1 ret[%d].\n", __func__,ret);
			mutex_unlock(&g_fop_mutex);
			return (-EFAULT);
		}
		/* if all datas transferd,break */
		if(rest_size <= 0) {
			break;
		}
	}

	I2CUART_DBG2("%s data[%s] size[%d]\n", __func__, g_wbuf, size);

	mutex_unlock(&g_fop_mutex);
	return size;
}

/*----------------------------------------------------------------------------*/
/*  @abstract : read data                                                     */
/*  @param    : file  -- not use                                              */
/*              user  -- user's data buffer                                   */
/*              size  -- number of user's data byte                           */
/*              loff  -- not use                                              */
/*  @return   : 0 <=  -- RXed data bytes                                      */
/*              0 >   -- error                                                */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static ssize_t i2cuart_fop_read(struct file *file, char __user *user,
								size_t size, loff_t *loff)
{
	int ret;

	I2CUART_DBG("%s start\n", __func__);

	if (size > I2C_READ_BUFSIZE) {
		printk("%s Too large area size is %d\n", __func__,size);
		return (-EFAULT);
	}

	mutex_lock(&g_fop_mutex);

	memset(g_rbuf, '\0', I2C_READ_BUFSIZE);
	ret = i2cuart_data_from_buffer( (char *)g_rbuf, size );

	mutex_lock( &g_i2cuart_en_mutex );
	if ( g_iBufDataSize == 0 ){
		g_i2cuart_readable = 0;
	}
	mutex_unlock( &g_i2cuart_en_mutex );

	if (ret < 0) {
		printk("%s failed getting buff-data. ret[%d]\n", __func__,ret);
		mutex_unlock(&g_fop_mutex);
		return ret;
	}

	if (copy_to_user(user, g_rbuf, size) != 0) {
		printk("%s Can't copy to user area.\n", __func__);
		mutex_unlock(&g_fop_mutex);
		return (-EFAULT);
	}
	I2CUART_DBG2( "%s data[%s] size[%d] ret[%d]\n",
				__func__, g_rbuf, size, ret);
	mutex_unlock(&g_fop_mutex);

	return ret;
}

/*----------------------------------------------------------------------------*/
/*  @abstract : control device                                                */
/*  @param    : inode -- not use                                              */
/*              file  -- not use                                              */
/*              cmd   -- control comand                                       */
/*              arg   -- not use                                              */
/*  @return   : 0 ==  -- command succeed                                      */
/*              0 >   -- error                                                */
/*  @note     : only support UART-FLUSH                                       */
/*----------------------------------------------------------------------------*/
static int i2cuart_fop_ioctl(struct inode *inode, struct file *file,
							unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	I2CUART_DBG2("%s start\n", __func__);

	switch (cmd) {
	case I2CUART_IOCTL_FLUSH:
		mutex_lock(&g_fop_mutex);
		ret = i2cuart_flush();
		mutex_unlock(&g_fop_mutex);
		break;

	default:
		ret = (-1);
		printk("%s illegal command %d .\n", __func__ ,cmd);
		break;
	}

	return ret;
}

/*----------------------------------------------------------------------------*/
/*  @abstract : poll device                                                   */
/*  @param    : file   -- not use                                             */
/*              ptable -- not use                                             */
/*  @return   :        -- poll event bit                                      */
/*  @note     : judge exist read-data or not                                  */
/*----------------------------------------------------------------------------*/
static unsigned int i2cuart_fop_poll(struct file *file,
									struct poll_table_struct *ptable)
{
	unsigned int mask = 0;

	I2CUART_DBG("%s start\n", __func__);

	poll_wait(file, &g_wait_queue_irq, ptable);

	if (g_i2cuart_readable == 1){
		mask = (POLLIN | POLLRDNORM);
	} else if (g_i2cuart_readable == 2){
		mask = POLLPRI;
		printk("[I2CUART] Notify full mask[0x%x]\n", mask);
	} else {
		I2CUART_DBG("%s Not Readable\n", __func__);
	}

	return mask;
}

/*----------------------------------------------------------------------------*/
/*  SUB FUNCTIONS                                                             */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*  @abstract : copy to Ring Buffer                                           */
/*  @param    : iSize -- Read size(indicated for upper layer)                 */
/*  @return   : read size                                                     */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static int i2cuart_data_to_buffer( int iSize )
{
	int iRet = 0;
	int iRest = 0;
	int iReadSize = 0;
	char cFIFOSize = 0;

	I2CUART_DBG("%s start\n", __func__);

	mutex_lock( &g_i2cuart_rw_mutex );

	I2CUART_DBG("start ridx[%d] widx[%d] size[%d] req[%d] \n",
				g_usRidx, g_usWidx, g_iBufDataSize, iSize );

	if ( (g_iBufDataSize >= I2CUART_RINGBUFF_SIZE) || ( iSize == 0 ) ){
		mutex_unlock( &g_i2cuart_rw_mutex );
		I2CUART_DBG("%s return1\n", __func__);
		return 0;
	}

	iRet = i2cuart_i2c_read( REG_FIFO_RXLVL,&cFIFOSize,1 );
	I2CUART_DBG("i2cuart_data_to_buffer RxFIFOLvl %d\n",cFIFOSize);

	if ( (iRet > 0) && (cFIFOSize > 0) ){
		if ( iSize > cFIFOSize ){
			iReadSize = cFIFOSize;
		} else {
			iReadSize = iSize;
		}

	} else {
		mutex_unlock( &g_i2cuart_rw_mutex );
		if ( iRet <= 0 ){
			I2CUART_DBG("%s return2 FIFO is Err [0x%x] \n", __func__, iRet);
			return (-1);
		} else {
			I2CUART_DBG("%s return2 FIFO is Empty\n", __func__);
			return 0;
		}
	}

	if ( g_usWidx < g_usRidx ){
		if ( g_usWidx + iReadSize > g_usRidx ){
			iRet = i2cuart_i2c_read( REG_RHR, &g_cTmpRdBuf[g_usWidx],
									g_usRidx - g_usWidx );
			if ( iRet > 0 ){
				iRet = g_usRidx - g_usWidx;
				g_iBufDataSize += g_usRidx - g_usWidx;
				g_usWidx = g_usRidx;
			}
		} else {
			iRet = i2cuart_i2c_read( REG_RHR,
									&g_cTmpRdBuf[g_usWidx],
									iReadSize );
			if ( iRet > 0 ){
				iRet = iReadSize;
				g_iBufDataSize += iReadSize;
				g_usWidx += iReadSize;
			}
		}
	} else {
		if ( g_usWidx + iReadSize > I2CUART_RINGBUFF_SIZE ){
			/* 1st */
			iRet = i2cuart_i2c_read( REG_RHR, &g_cTmpRdBuf[g_usWidx],
					I2CUART_RINGBUFF_SIZE - g_usWidx );

			if ( iRet > 0 ){
				iRest = iReadSize - (I2CUART_RINGBUFF_SIZE - g_usWidx);
				g_iBufDataSize += (I2CUART_RINGBUFF_SIZE - g_usWidx);
				g_usWidx = 0;

				if ( iRest > g_usRidx ){
					iRet = i2cuart_i2c_read( REG_RHR,
											&g_cTmpRdBuf[0],
											g_usRidx );
					if ( iRet > 0 ){
						iRet = iRest + g_usRidx;
						g_iBufDataSize += g_usRidx;
						g_usWidx = g_usRidx;
					}
				} else {
					iRet = i2cuart_i2c_read( REG_RHR,
											&g_cTmpRdBuf[0],
											iRest );
					if ( iRet > 0 ){
						iRet = iReadSize;
						g_iBufDataSize += iRest;
						g_usWidx = iRest;
					}
				}
			}
		} else {
			iRet = i2cuart_i2c_read( REG_RHR,
									&g_cTmpRdBuf[g_usWidx],
									iReadSize );

			if ( iRet > 0 ){
				g_iBufDataSize += iReadSize;
				g_usWidx += iReadSize;
				iRet = iReadSize;
				if ( g_usWidx >= I2CUART_RINGBUFF_SIZE ){
					g_usWidx = 0;
				}
			}
		}
	}

	I2CUART_DBG("end ridx[%d] widx[%d] size[%d] ret[%d] \n",
				g_usRidx, g_usWidx, g_iBufDataSize, iRet );

	mutex_unlock( &g_i2cuart_rw_mutex );

	I2CUART_DBG("%s end\n", __func__);

	return iRet;
}


/*----------------------------------------------------------------------------*/
/*  @abstract : copy from Ring Buffer for Read Operation                      */
/*  @param    : pcBuf -- read buf                                             */
/*            : iSize -- Read size                                            */
/*  @return   : read  size                                                    */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static int i2cuart_data_from_buffer( char *pcBuf, int iSize )
{
	int i;
	int iMax = 0;

	mutex_lock( &g_i2cuart_rw_mutex );

	I2CUART_DBG("i2cuart_data_from_buffer() start ridx[%d] widx[%d] size[%d] req[%d]\n",
				g_usRidx, g_usWidx, g_iBufDataSize, iSize );
	if ( (g_iBufDataSize == 0) || ( iSize == 0 ) ){
		mutex_unlock( &g_i2cuart_rw_mutex );
		return 0;
	}

	if ( iSize > g_iBufDataSize ){
		iMax = g_iBufDataSize;
	} else {
		iMax = iSize;
	}

	for( i = 0; i < iMax; i++ ){
		pcBuf[i] = g_cTmpRdBuf[g_usRidx];
		g_usRidx++;
		g_iBufDataSize--;

		if ( g_usRidx == I2CUART_RINGBUFF_SIZE ){
			g_usRidx = 0;
		}
		if ( g_usRidx == g_usWidx ){
			i++;
			break;
		}
	}

	I2CUART_DBG( "i2cuart_data_from_buffer() end ridx[%d] widx[%d] size[%d] ret[%d] \n",
			g_usRidx, g_usWidx, g_iBufDataSize, i );

	mutex_unlock( &g_i2cuart_rw_mutex );

	return i;
}

/*----------------------------------------------------------------------------*/
/*  @abstract : read in context                                               */
/*  @param    : work -- work data                                             */
/*  @return   : -                                                             */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static void i2cuart_work_bh(struct work_struct *work)
{
	int ret	= 0;
	uint8_t buf[3]	= {0,0,0};
	int getsize = 0;

	mutex_lock( &g_work_mutex );
	I2CUART_DBG( "%s IRQ_SCHD start readable[%d] restbuf[%d] ridx[%d] widx[%d] \n",
			__func__, g_i2cuart_readable, g_iBufDataSize, g_usRidx, g_usWidx );

	ret = i2cuart_i2c_read(REG_INT_ISR, buf, 3);
	if (ret < 0) {
		printk("%s Can't Read REG_INT_LSR value 0x%x,ret[%d]\n", __func__,buf[2],ret);
	}
	I2CUART_DBG( "%s ISR [0x%x] \n", __func__, buf[0] );
	I2CUART_DBG( "%s LSR [0x%x] \n", __func__, buf[2] );

	if (!(buf[2] & REG_INT_LSR_RXOVR_VAL)) {
		usleep( I2CUART_WORKQUEUE_DELAY );
		ret = 0;
		do{
			ret = i2cuart_data_to_buffer( I2CUART_RINGBUFF_SIZE );
			I2CUART_DBG2( "i2cuart_work_bh read[%d] \n", ret );
			if ( ret > 0 ){
				getsize += ret;
			}
			if ( (g_iBufDataSize >= I2CUART_RINGBUFF_SIZE) || (ret == I2CUART_CHIP_BUFF_SIZE) ){
				buf[2] |= REG_INT_LSR_RXOVR_VAL;
				I2CUART_DBG2("[I2CUART] Traffic high rbuf[%d] gbuf[%d] ridx[%d] widx[%d] \n", g_iBufDataSize, ret, g_usRidx, g_usWidx);
				break;
			}
		} while (ret != 0);
	}

	I2CUART_DBG("%s bufend readable[%d] restbuf[%d] ridx[%d] widx[%d] \n",
				__func__, g_i2cuart_readable, g_iBufDataSize, g_usRidx, g_usWidx );

	if ( (g_iBufDataSize > 0) || (buf[2] & REG_INT_LSR_RXOVR_VAL) ) {
		mutex_lock( &g_i2cuart_en_mutex );
		if (buf[2] & REG_INT_LSR_RXOVR_VAL) {
			g_i2cuart_readable = 2;
		} else {
			g_i2cuart_readable = 1;
		}
		mutex_unlock( &g_i2cuart_en_mutex );

		wake_up_interruptible(&g_wait_queue_irq);
	}
	mutex_unlock( &g_work_mutex );
}

/*----------------------------------------------------------------------------*/
/*  @abstract : interrupt handler                                             */
/*  @param    : irq  -- not use                                               */
/*              data -- not use                                               */
/*  @return   : IRQ_HANDLED                                                   */
/*  @note     :                                                               */
/*----------------------------------------------------------------------------*/
static irqreturn_t i2cuart_irq_handler(int irq, void *data)
{
	I2CUART_DBG("%s \n", __func__);

	schedule_work( &g_i2cuart_work_data);

	return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
/*  @abstract : Tx/Rx FIFO Reset                                              */
/*  @param    : -                                                             */
/*  @return   : -                                                             */
/*  @note     : method of ioctl                                               */
/*----------------------------------------------------------------------------*/
static int i2cuart_flush(void)
{
	int ret;
	uint8_t buf;
	int iRetVal = 0;

	mutex_lock( &g_work_mutex );
	cancel_work_sync(&g_i2cuart_work_data);
	I2CUART_DBG2("%s start\n", __func__);

	/* RxDisable  */
	buf = (uint8_t)0x81;
	ret = i2cuart_i2c_write(REG_MODE1, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Write REG_MODE1 ret[%d].\n", __func__,ret);
		iRetVal = ret;
	}

	/* reset Tx/Rx FIFO */
	buf = (uint8_t)0x0A;
	ret = i2cuart_i2c_write(REG_MODE2, &buf, 1);
	if (ret < 0) {
		printk("%s can't reset FIFO. ret[%d]\n", __func__,ret);
		iRetVal = ret;
	}

	/* after FIFO reset, for normal operation */
	buf = 0x08;
	ret = i2cuart_i2c_write(REG_MODE2, &buf, 1);
	if (ret < 0) {
		printk("%s can't recovery from FIFO reset ret[%d].\n", __func__,ret);
		iRetVal = ret;
	}

	/* Interrupt status clear */
	ret = i2cuart_i2c_read(REG_INT_ISR, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Read REG_INT_ISR value[0x%x] ret[%d]\n", __func__,buf,ret);
		iRetVal = ret;
	}
/* FUJITSU:2011-08-29 delete register read start */
#if 0
	ret = i2cuart_i2c_read(REG_INT_LSR, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Read REG_INT_LSR value[0x%x] ret[%d]\n", __func__,buf,ret);
		iRetVal = ret;
	}
	ret = i2cuart_i2c_read(REG_INT_SPC, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Read REG_INT_SPC value[0x%x] ret[%d]\n", __func__,buf,ret);
		iRetVal = ret;
	}
	ret = i2cuart_i2c_read(REG_INT_STS, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Read REG_INT_STS value[0x%x] ret[%d]\n", __func__,buf,ret);
		iRetVal = ret;
	}
#endif
/* FUJITSU:2011-08-29 delete register read end */

	/* Reset My RingBuffCtrl's Param */
	mutex_lock( &g_i2cuart_rw_mutex );
	g_usRidx = 0;
	g_usWidx = 0;
	g_iBufDataSize = 0;
	g_i2cuart_readable = 0;
	memset( &g_cTmpRdBuf[0], '\0', I2CUART_RINGBUFF_SIZE + 1);
	mutex_unlock( &g_i2cuart_rw_mutex );


	/* RxEnable */
	buf = (uint8_t)0x80;
	ret = i2cuart_i2c_write(REG_MODE1, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Write REG_MODE1 ret[%d].\n", __func__,ret);
		iRetVal = ret;
	}

	schedule_work(&g_i2cuart_work_data);
	mutex_unlock( &g_work_mutex );

	return iRetVal;	
}


/*----------------------------------------------------------------------------*/
/*  @abstract : reset device(MAX-IC)                                          */
/*  @param    : -                                                             */
/*  @return   : -                                                             */
/*  @note     : set reset register, and read DIVLSB for reset-confirm         */
/*----------------------------------------------------------------------------*/
static int i2cuart_reset(bool recovery)
{
	int ret;
	uint8_t buf;

	I2CUART_DBG("%s start\n", __func__);

	/* reset MAX-IC */
	buf = (uint8_t)REG_MODE2_RST_VAL;
	ret = i2cuart_i2c_write(REG_MODE2, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Write REG_MODE2 ret[%d].\n", __func__,ret);
		return ret;
	}

	mdelay(5);

	/* confirm reset complete */
	ret = i2cuart_i2c_read(REG_DIVLSB, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Read REG_DIVLSB value[0x%x] ret[%d]\n", __func__,buf,ret);
		return ret;
	}

	if (buf != REG_DIVLSB_INITVAL) {
		/* try again...? */
		printk("%s Can't Reset DIVLSB[0x%x].\n", __func__,buf);
		return -1;
	}

	if (recovery) {
		/* after reset, for normal operation */
		buf = (uint8_t)0x00;
		ret = i2cuart_i2c_write(REG_MODE2, &buf, 1);
		if (ret < 0) {
			printk("%s can't recovery from reset ret[%d].\n", __func__,ret);
			return ret;
		}
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
/*  @abstract : set UART-port of MAX-IC                                       */
/*  @param    : -                                                             */
/*  @return   : -                                                             */
/*  @note     :                                                               */
/*----------------------------------------------------------------------------*/
static int i2cuart_set_termios(void)
{
	uint8_t buf;
	int ret;

	/* BRGCFG */
	ret = i2cuart_i2c_read(REG_BRGCFG, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Read REG_BRGCFG value[0x%x] ret[%d\n", __func__,buf,ret);
		goto Exit;
	}

	buf |= (uint8_t)0x25;	/* 4x */
	ret = i2cuart_i2c_write(REG_BRGCFG, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Write REG_BRGCFG ret[%d].\n", __func__,ret);
		goto Exit;
	}

	/* DIVMSB */
	buf = (uint8_t)0x00;
	ret = i2cuart_i2c_write(REG_DIVMSB, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Write REG_DIVMSB ret[%d].\n", __func__,ret);
		goto Exit;
	}

	/* DIVLSB */
	buf = (uint8_t)0x01;
	ret = i2cuart_i2c_write(REG_DIVLSB, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Write REG_DIVLSB ret[%d].\n", __func__,ret);
		goto Exit;
	}

	/* CLKSRC */
	buf = (uint8_t)0x98;
	ret = i2cuart_i2c_write(REG_CLKSRC, &buf, 1);
	I2CUART_DBG("REG_CLKSRC [0x%x].\n", buf);
	if (ret < 0) {
		printk("%s Can't Write REG_CLKSRC ret[%d].\n", __func__,ret);
		goto Exit;
	}

	/* RxDisable + EnableIRQ */
	buf = (uint8_t)0x81;
	ret = i2cuart_i2c_write(REG_MODE1, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Write REG_MODE1 ret[%d].\n", __func__,ret);
		goto Exit;
	}

	/* FIFOReset + EmptyCtrl */
	buf = (uint8_t)0x0A;
	ret = i2cuart_i2c_write(REG_MODE2, &buf, 1);
	if (ret < 0) {
		printk("%s can't reset FIFO ret[%d].\n", __func__,ret);
		goto Exit;
	}

	/* EmptyCtrl */
	buf = (uint8_t)0x08;
	ret = i2cuart_i2c_write(REG_MODE2, &buf, 1);
	if (ret < 0) {
		printk("%s can't reset FIFO ret[%d].\n", __func__,ret);
		goto Exit;
	}

	/* Interrupt status clear */
	ret = i2cuart_i2c_read(REG_INT_ISR, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Read REG_INT_ISR value[0x%x] ret[%d]\n", __func__,buf,ret);
		goto Exit;
	}
	ret = i2cuart_i2c_read(REG_INT_LSR, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Read REG_INT_LSR value[0x%x] ret[%d]\n", __func__,buf,ret);
		goto Exit;
	}
	ret = i2cuart_i2c_read(REG_INT_SPC, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Read REG_INT_SPC value[0x%x] ret[%d]\n", __func__,buf,ret);
		goto Exit;
	}
	ret = i2cuart_i2c_read(REG_INT_STS, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Read REG_INT_STS value[0x%x] ret[%d]\n", __func__,buf,ret);
		goto Exit;
	}

	/* Enable RxOverrun Int */
	buf = (uint8_t)0x02;
	ret = i2cuart_i2c_write(REG_INT_LSRTEN, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Write REG_INT_LSRTEN ret[%d].\n", __func__,ret);
		goto Exit;
	}

	/* Enable RxEmtyIEn */
	buf = (uint8_t)0x41;
	ret = i2cuart_i2c_write(REG_INT_IRQEN, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Write REG_INT_IRQEN ret[%d].\n", __func__,ret);
		goto Exit;
	}

	/* RxEnable + EnableIRQ */
	buf = (uint8_t)0x80;
	ret = i2cuart_i2c_write(REG_MODE1, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Write REG_MODE1 ret[%d].\n", __func__,ret);
	}

	/* UART Param data bit length:8bit */
	buf = (uint8_t)0x03;
	ret = i2cuart_i2c_write(REG_LCR, &buf, 1);
	if (ret < 0) {
		printk("%s Can't Write REG_LCR ret[%d].\n", __func__,ret);
		goto Exit;
	}

Exit:
	return ret;
}

/*----------------------------------------------------------------------------*/
/*  @abstract : write to MAX-IC                                               */
/*  @param    : reg  -- MAX-IC register offset                                */
/*              data --                                                       */
/*              len  --                                                       */
/*  @return   : 0    -- success                                               */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static int i2cuart_i2c_write(unsigned char reg, char *data, size_t len)
{

	if(system_rev >= 0x0c) {
		I2C_INFO inf;
		bool ret;
		int i = 0;

		mutex_lock( &g_i2ctxrx_mutex );

		inf.slave_addr	= I2CUART_SLAVE_ADDRESS;
		inf.buf_ptr		= data;
		inf.offset		= reg;
		inf.len			= len;

		for(i = 0; i < I2C_RETRY_MAX; i++) {
			ret = pse_i2c_write(&inf);
			if (ret) {
				if ( i > 0 ){
					printk("%s pse_i2c_write() succeed at last. (try : %d, reg : %d)\n", __func__, i, reg);
				}
				break;
			}
			printk("%s pse_i2c_write() retrying. (try : %d, reg : %d)\n", __func__, i, reg);
		}
		if (!ret) {
			printk("I2C write NG \n");
			mutex_unlock( &g_i2ctxrx_mutex );
			return -1;
		}

		mutex_unlock( &g_i2ctxrx_mutex );
		return len;

	} else {
		struct i2c_msg msg;
		int ret = 0;
		int i = 0;

		memset(g_tmpbuf, 0x00, sizeof(g_tmpbuf));

		msg.addr	= I2CUART_SLAVE_ADDRESS;
		msg.flags	= 0;
		msg.buf		= g_tmpbuf;
		msg.buf[0]	= reg;

		memcpy(&msg.buf[1], data, len);
		msg.len  = len + 1;

		for(i = 0; i < I2C_RETRY_MAX; i++) {
			ret = i2c_transfer(gp_i2c_adapter, &msg, I2C_TRANS_WRITE);
			if(ret >= 0) {
				if ( i > 0 ){
					printk("%s i2c_transfer() succeed at last. (try : %d, reg : %d)\n", __func__, i, reg);
				}
				break;
			} else if ( ret == -ETIMEDOUT ){
				printk("%s i2c_transfer() timed out. (try : %d, reg : %d)\n", __func__, i, reg);
				break;
			}
			printk("%s i2c_transfer() retrying. (try : %d, reg : %d, err : %d)\n", __func__, i, reg, ret);
		}
		return ret;
	}
}

/*----------------------------------------------------------------------------*/
/*  @abstract : read from MAX-IC                                              */
/*  @param    : reg  -- MAX-IC register offset                                */
/*              data --                                                       */
/*              len  --                                                       */
/*  @return   : 0    -- success                                               */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static int i2cuart_i2c_read(unsigned char reg, char *data, size_t len)
{
	
	if(system_rev >= 0x0c) {
		I2C_INFO inf;
		bool ret;
		int i = 0;

		mutex_lock( &g_i2ctxrx_mutex );

		inf.slave_addr	= I2CUART_SLAVE_ADDRESS;
		inf.buf_ptr		= data;
		inf.offset		= reg;
		inf.len			= len;

		for(i = 0; i < I2C_RETRY_MAX; i++) {
			ret = pse_i2c_read(&inf);
			if (ret) {
				if ( i > 0 ){
					printk("%s pse_i2c_read() succeed at last. (try : %d, reg : %d)\n", __func__, i, reg);
				}
				break;
			}
			printk("%s pse_i2c_read() retrying. (try : %d, reg : %d)\n", __func__, i, reg);
		}
		if (!ret) {
			printk("I2C read NG \n");
			mutex_unlock( &g_i2ctxrx_mutex );
			return -1;
		}

		mutex_unlock( &g_i2ctxrx_mutex );

		return len;

	} else {

		struct i2c_msg msg[2];
		u_int8_t msgbuf;
		int ret = 0;
		int i = 0;

		I2CUART_DBG("%s start\n", __func__);

		msgbuf			=  (u_int8_t)reg;
		msg[0].addr		= I2CUART_SLAVE_ADDRESS;
		msg[0].flags	= 0;
		msg[0].buf		= &msgbuf;
		msg[0].len		= 1;
		msg[1].addr		= I2CUART_SLAVE_ADDRESS;
		msg[1].flags	= I2C_M_RD;
		msg[1].buf		= data;
		msg[1].len		= len;

		for(i = 0; i < I2C_RETRY_MAX; i++) {
			ret = i2c_transfer(gp_i2c_adapter, msg, I2C_TARNS_READ);
			if(ret >= 0) {
				if ( i > 0 ){
					printk("%s i2c_transfer() succeed at last. (try : %d, reg : %d)\n", __func__, i, reg);
				}
				break;
			} else if ( ret == -ETIMEDOUT ){
				printk("%s i2c_transfer() timed out. (try : %d, reg : %d)\n", __func__, i, reg);
				break;
			}
			printk("%s i2c_transfer() retrying. (try : %d, reg : %d, err : %d)\n", __func__, i, reg, ret);
		}

		return ret;
	}
}

/*----------------------------------------------------------------------------*/
/*  @abstract : setup IRQ Handler(KO_UART_NIRQ4)                              */
/*  @param    : -                                                             */
/*  @return   : -                                                             */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static int i2cuart_request_irq(void)
{
	int err;
	unsigned long req_flags = (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW);

	err = request_any_context_irq(g_irq, i2cuart_irq_handler, req_flags,
						MYNAME, gp_i2c_adapter);

	I2CUART_DBG("%s returned %d\n", __func__, err);

	return err;
}

/*----------------------------------------------------------------------------*/
/*  MODULE for kernel                                                         */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*  @abstract : setup driver                                                  */
/*  @param    : pdev -- device info                                           */
/*  @return   : 0    -- success                                               */
/*              !0   -- fail                                                  */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static int i2cuart_probe(struct platform_device *pdev)
{

	struct i2cuart_platform_data *pdata;
	int result = 0;
	int mem;

	pdata = pdev->dev.platform_data;
	g_irq = pdata->irq;

	/* get I2Cadapter */
	if(system_rev >= 0x0c) {
		gp_i2c_adapter = i2c_get_adapter(4);
	} else {
		gp_i2c_adapter = i2c_get_adapter(0);
	}

	/* GP_MN */
	mem = (int)ioremap( 0xACE00000, 0x0100 );

	writel( 0x00000001, mem + (0x004c) );	/* GP_MN_CLK_MDIV *//* 4x */
	writel( 0x0000fffc, mem + (0x0050) );	/* GP_MN_CLK_NDIV */
	writel( 0x00000002, mem + (0x0054) );	/* GP_MN_CLK_DUTY */

	mutex_init( &g_i2ctxrx_mutex );
	mutex_init( &g_fop_mutex );

	mutex_init( &g_i2cuart_en_mutex );
	mutex_init( &g_i2cuart_rw_mutex );
	g_usRidx = 0;
	g_usWidx = 0;
	g_iBufDataSize = 0;
	memset( &g_cTmpRdBuf[0], '\0', I2CUART_RINGBUFF_SIZE + 1);

	INIT_WORK(&g_i2cuart_work_data, i2cuart_work_bh);

	/* init wait queue */
	init_waitqueue_head(&g_wait_queue_irq);

	gp_i2cuart_clk = clk_get( &pdev->dev, "tcxo_div4_clk" );
	clk_enable( gp_i2cuart_clk );

	/* reset MAX-IC */
	gpio_set_value( 104, 0 );
	mdelay(1);
	gpio_set_value( 104, 1 );
	mdelay(5);
	gpio_get_value( 104 );

	if(system_rev >= 0x0c) {
		pse_i2c_init();
	}

	result = i2cuart_set_termios();
	if (result < 0) {
		I2CUART_DBG2("%s : i2cuart_set_termios() failed\n", __func__);
		return result;
	}

	g_open = 0;
	memset(g_wbuf, '\0', I2C_WRITE_BUFSIZE);
	memset(g_rbuf, '\0', I2C_READ_BUFSIZE);

	result = misc_register(&i2cuart_ioctl_device);
	if (result) {
		printk("misc_register() failed result[%d].\n", result);
	}

	mutex_init( &g_work_mutex );

	I2CUART_DBG2("%s returned %d\n", __func__, result);
	return result;

}

/*----------------------------------------------------------------------------*/
/*  @abstract : remove driver                                                 */
/*  @param    : pdev -- device info                                           */
/*  @return   : 0    -- success                                               */
/*              !0   -- fail                                                  */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static int i2cuart_remove(struct platform_device *pdev)
{
	int ret = 0;
	
	I2CUART_DBG2("%s start\n", __func__);

	ret = i2cuart_reset(false);
	if (ret < 0) {
		printk("%s : i2cuart_reset() failed ret[%d]\n", __func__,ret);
	}

	clk_disable( gp_i2cuart_clk );

	misc_deregister(&i2cuart_ioctl_device);

	return 0;
}

/*----------------------------------------------------------------------------*/
/*  @abstract : suspend module                                                */
/*  @param    : pdev  -- device info                                          */
/*              state -- event info                                           */
/*  @return   : 0                                                             */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static int i2cuart_suspend(struct platform_device *pdev, pm_message_t state)
{
	clk_disable(gp_i2cuart_clk);
	I2CUART_DBG2("%s\n", __func__);

	return 0;
}

/*----------------------------------------------------------------------------*/
/*  @abstract : resume module                                                 */
/*  @param    : -                                                             */
/*  @return   : 0                                                             */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static int i2cuart_resume(struct platform_device *pdev)
{
	clk_enable(gp_i2cuart_clk);
	mdelay(I2CUART_RESUME_DELAY);
	I2CUART_DBG2("%s\n", __func__);

	return 0;
}

/*----------------------------------------------------------------------------*/
/*  @abstract : init driver module                                            */
/*  @param    : -                                                             */
/*  @return   : 0  -- success                                                 */
/*             !0  -- fail                                                    */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static int i2cuart_init(void)
{
	int ret = 0;

	ret = platform_driver_probe(&g_i2cuart_platform_driver, i2cuart_probe);
	I2CUART_DBG("%s end[0x%x]\n", __func__,ret);
	return ret;
}

/*----------------------------------------------------------------------------*/
/*  @abstract : terminate driver module                                       */
/*  @param    : -                                                             */
/*  @return   : 0  -- success                                                 */
/*              !0 -- fail                                                    */
/*  @note     : -                                                             */
/*----------------------------------------------------------------------------*/
static void i2cuart_exit(void)
{
	platform_driver_unregister(&g_i2cuart_platform_driver);
	I2CUART_DBG("%s end\n", __func__);
}

module_init(i2cuart_init);
module_exit(i2cuart_exit);

MODULE_AUTHOR ("FUJITSU,2011");
MODULE_DESCRIPTION ("i2cuart driver");
MODULE_LICENSE ("GPL");
