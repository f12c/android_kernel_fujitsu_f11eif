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
/* FUJITSU:2011-05-26 USB start */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cryptohash.h>
/*-----*/
MODULE_DESCRIPTION("Fujitsu Debug USB Driver");
MODULE_LICENSE("GPL");

/*-----*/
#define NV_OPEFLAG_I			10035
#define PID__ADB_MSC________________	0x12AB
#define PID__ADB_MSC_DIG_MDM_GPS_RMN	0x12B2

#define ENABLE_PWSTRING_IF    // disable pwstring

/*-----*/
extern void android_debugusb_set_port(int product_id);
extern int android_get_current_pid(void);
/* FUJITSU:2011-05-11 USB start */
extern int msm_usb_read_nvitem(unsigned int id, unsigned int *data);
extern int msm_usb_write_nvitem(unsigned int id, unsigned int *data);
/* FUJITSU:2011-05-11 USB end */

unsigned long debugusb_get_pid_from_nv(void);

#define MAX_STRING_LEN  65

/*-----*/
static int debugusb_set_pid(const char *val, struct kernel_param *kp);
static int debugusb_get_pid(char *buffer, struct kernel_param *kp);
static int debugusb_set_password(const char *val, struct kernel_param *kp);
static int debugusb_get_password(char *buffer, struct kernel_param *kp);

static int product_id = 0;
module_param_call(product_id, debugusb_set_pid, debugusb_get_pid, &product_id, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);	//0644
MODULE_PARM_DESC(product_id, "USB debug product id");

/*-----*/
static char password_string[MAX_STRING_LEN] = {0};
#ifdef ENABLE_PWSTRING_IF
/* FUJITSU:2011-06-10 USB start */
//static unsigned int desired_password[5] = {0x4d347823,0x685235e1,0x99803e04,0xcd56a201,0x5c5ec4f2};
static unsigned int desired_password[5] = {0x2543405f,0x8bd90dc7,0x5de9ec18,0x9f8102c1,0x8d8e8efd};
/* FUJITSU:2011-06-10 USB end */
#endif // ENABLE_PWSTRING_IF
static unsigned int output_password[5] = {0}; 
static struct kparam_string kps_pass = {
	.maxlen = MAX_STRING_LEN,
	.string = password_string
};

module_param_call(password_string, debugusb_set_password, debugusb_get_password, &kps_pass, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);	//0644
MODULE_PARM_DESC(password_string, "Password string");

unsigned long debugusb_get_pid_from_nv(void)
{
	unsigned long pid	= 0;
/* FUJITSU:2011-05-11 USB start */
	unsigned int nv_ope	= 0;
/* FUJITSU:2011-05-11 USB end */

//debug log
pr_info("[gadget/debug_usb.c] %s\n", __func__);

	if (msm_usb_read_nvitem(NV_OPEFLAG_I, &nv_ope) != 0) {
		goto out;
	}
	if (nv_ope == 0x02) {
		pid = PID__ADB_MSC_DIG_MDM_GPS_RMN;
	}
out:
pr_info("[gadget/debug_usb.c] %s pid=0x%lx nv_ope=0x%x\n", __func__,pid,nv_ope);
	return pid;
}

static int debugusb_set_nv(unsigned long pid, struct kernel_param *kp)
{
	int		ret 		= 0;
/* FUJITSU:2011-05-11 USB start */
	unsigned int	nv_ope 		= 0;
/* FUJITSU:2011-05-11 USB end */
	#ifdef ENABLE_PWSTRING_IF
	unsigned int	buffer [5] 	= {0};	
	int 		loop 		= 0;
	#endif // ENABLE_PWSTRING_IF


//debug log
pr_info("[gadget/debug_usb.c] %s pid=0x%lx\n", __func__,pid);

	switch (pid) {
	case PID__ADB_MSC_DIG_MDM_GPS_RMN:
		nv_ope = 2;
		break;
	case PID__ADB_MSC________________:
		nv_ope = 0;
		break;
	default:	/* other pid */
		goto out;
	}

pr_info("[gadget/debug_usb.c] %s nv_ope=%04x\n", __func__,nv_ope);
	
	#ifdef ENABLE_PWSTRING_IF
	debugusb_get_password((char *)buffer, kp);
	for (loop =0; loop<5; loop ++) {
		if( buffer[loop] != desired_password[loop] ) {
			goto out;
		}
	}
	#endif // ENABLE_PWSTRING_IF

	if (msm_usb_write_nvitem(NV_OPEFLAG_I, &nv_ope) != 0) {
		goto out;
	}

	ret = 1;	/* suceeded */

out:
pr_info("[gadget/debug_usb.c] %s ret=%04x\n", __func__,ret);
	return ret;
}


static int debugusb_set_pid(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	unsigned long tmp;
//debug log
pr_info("[gadget/debug_usb.c] %s\n", __func__);

	ret = strict_strtoul(val, 16, &tmp);
	if (ret) {
		goto out;
	}

	if ((debugusb_set_nv(tmp, kp)) != 1) {
		pr_info("[gadget/debug_usb.c]%s:invalid combination %lx\n",__func__,tmp);
		ret = -EINVAL;
		goto out;
	}
	
pr_info("[gadget/debug_usb.c] %s tmp=0x%lx\n", __func__,tmp);
	android_debugusb_set_port(tmp);

out:
pr_info("[gadget/debug_usb.c] %s ret=0x%x\n", __func__,ret);
	return ret;
}

static int debugusb_get_pid(char *buffer, struct kernel_param *kp)
{
	int ret = 0;
//debug log
pr_info("[gadget/debug_usb.c] %s\n", __func__);
	product_id = android_get_current_pid();
	ret = sprintf(buffer, "%x", product_id);

pr_info("[gadget/debug_usb.c] %s ret=%04x\n", __func__,ret);
	return ret;
}

static void debugusb_hash_password(const char *kmessage)
{
	unsigned int hash[5], workspace[SHA_WORKSPACE_WORDS];
	char local_buf[MAX_STRING_LEN] = {0};
	int i =0;

	strlcpy(local_buf, kmessage, MAX_STRING_LEN);
	local_buf[64] = '\0';
	sha_init(hash);
	sha_transform(hash, (unsigned char *)local_buf, workspace);
	for (i = 0; i < 5; i++) {
		output_password[i]= hash[i];
	}
}

static int debugusb_set_password(const char *kmessage, struct kernel_param *kp)
{
	int len = 0;
	int i =0;
//debug log
pr_info("[gadget/debug_usb.c] %s\n", __func__);
	if(kmessage) {
		len = strlen(kmessage);
	}
	else {
		for(i=0;i<5;i++) {
			output_password[i] = 0;
		}
		return 0;
	}
	
	if (len > 64) {
		return -ENOSPC;
	}

	debugusb_hash_password(kmessage);

	return 0;
}
static int debugusb_get_password(char *buffer, struct kernel_param *kp)
{
	int ret = 0;
	int i =0;
	unsigned int *sv;

	sv = (unsigned int *)buffer;
//debug log
pr_info("[gadget/debug_usb.c] %s\n", __func__);
	for(i = 0 ; i < 5 ; i++) {
		sv[i] = output_password[i];
		output_password[i] = 0;
	}
	return ret;
}

static int __init debugusb_init(void)
{
	return 0;
}
module_init(debugusb_init);

static void __exit debugusb_deinit(void)
{
	return;
}
module_exit(debugusb_deinit);
/* FUJITSU:2011-05-26 USB end */
