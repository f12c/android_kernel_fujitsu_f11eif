/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/mfd/pmic8058.h>
#include <linux/leds-pmic8058.h>
/* FUJITSU:2011-04-14 LED start */
#include <linux/gpio.h>
#include <asm/mach-types.h>
/* FUJITSU:2011-04-14 LED end */
/* FUJITSU:2011-04-28 SUSPEND LED start */
#include <linux/ctype.h>
#include "../arch/arm/mach-msm/smd_private.h"
/* FUJITSU:2011-04-28 SUSPEND LED end */
#define SSBI_REG_ADDR_DRV_KEYPAD	0x48
#define PM8058_DRV_KEYPAD_BL_MASK	0xf0
#define PM8058_DRV_KEYPAD_BL_SHIFT	0x04

#define SSBI_REG_ADDR_FLASH_DRV0        0x49
#define PM8058_DRV_FLASH_MASK           0xf0
#define PM8058_DRV_FLASH_SHIFT          0x04

#define SSBI_REG_ADDR_FLASH_DRV1        0xFB

#define SSBI_REG_ADDR_LED_CTRL_BASE	0x131
#define SSBI_REG_ADDR_LED_CTRL(n)	(SSBI_REG_ADDR_LED_CTRL_BASE + (n))
#define PM8058_DRV_LED_CTRL_MASK	0xf8
#define PM8058_DRV_LED_CTRL_SHIFT	0x03

#define MAX_FLASH_CURRENT	300
#define MAX_KEYPAD_CURRENT 300
#define MAX_KEYPAD_BL_LEVEL	(1 << 4)
#define MAX_LED_DRV_LEVEL	20 /* 2 * 20 mA */

#define PMIC8058_LED_OFFSET(id) ((id) - PMIC8058_ID_LED_0)
/* FUJITSU:2011-04-14 LED start */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PMIC_GPIO_KEYLIGHT	12  /* PMIC GPIO Number 12 */
#define LED_BRIGHTNESS_ON_MAX	(10)
/* FUJITSU:2011-04-14 LED end */
/* FUJITSU:2011-05-31 LED start */
#define thrtshold_light_lux 80
/* FUJITSU:2011-05-31 LED end */

/* FUJITSU:2011-04-28 SUSPEND LED start */
static unsigned char led_notify;
static unsigned char led_charge;

struct smem_led_color_type
   	{
          uint8_t led_alpha;
          uint8_t led_red;
          uint8_t led_green;
          uint8_t led_blue;
	}; 

struct smem_led_color_type smem_led_color;

/* FUJITSU:2011-04-28 SUSPEND LED end */

struct pmic8058_led_data {
	struct led_classdev	cdev;
	int			id;
	enum led_brightness	brightness;
	u8			flags;
	struct pm8058_chip	*pm_chip;
	struct work_struct	work;
	struct mutex		lock;
	spinlock_t		value_lock;
	u8			reg_kp;
	u8			reg_led_ctrl[3];
	u8			reg_flash_led0;
	u8			reg_flash_led1;
};

/* FUJITSU:2011-05-27 LED start */
#define PM8058_MAX_LEDS		12
/* FUJITSU:2011-05-27 LED end */
/* FUJITSU:2011-05-31 LED start */
extern int g_light_lux;
/* FUJITSU:2011-05-31 LED end */

static struct pmic8058_led_data led_data[PM8058_MAX_LEDS];
/* FUJITSU:2011-06-07 LED start */
extern void timer_trig_activate(struct led_classdev *led_cdev);
/* FUJITSU:2011-06-07 LED end */
static void kp_bl_set(struct pmic8058_led_data *led, enum led_brightness value)
{
	int rc;
	u8 level;
	unsigned long flags;

	spin_lock_irqsave(&led->value_lock, flags);
	level = (value << PM8058_DRV_KEYPAD_BL_SHIFT) &
				 PM8058_DRV_KEYPAD_BL_MASK;

	led->reg_kp &= ~PM8058_DRV_KEYPAD_BL_MASK;
	led->reg_kp |= level;
	spin_unlock_irqrestore(&led->value_lock, flags);

	rc = pm8058_write(led->pm_chip, SSBI_REG_ADDR_DRV_KEYPAD,
				 &led->reg_kp, 1);
	if (rc)
		pr_err("%s: can't set keypad backlight level\n", __func__);
}

static enum led_brightness kp_bl_get(struct pmic8058_led_data *led)
{
	if ((led->reg_kp & PM8058_DRV_KEYPAD_BL_MASK) >>
			 PM8058_DRV_KEYPAD_BL_SHIFT)
		return LED_FULL;
	else
		return LED_OFF;
}

static void led_lc_set(struct pmic8058_led_data *led, enum led_brightness value)
{
	unsigned long flags;
	int rc, offset;
	u8 level, tmp;
/* FUJITSU:2011-05-06 MAPPING LED start */
	enum led_brightness brightness;
/* FUJITSU:2011-05-06 MAPPING LED end */

	spin_lock_irqsave(&led->value_lock, flags);

/* FUJITSU:2011-05-06 MAPPING LED start */
	if ( value == 0 ){
		brightness = 0;
	} else if ( value <= 255 ) {
		brightness = ( value / 26 ) + 1;
	} else {
		brightness = LED_BRIGHTNESS_ON_MAX;
	}
	level = (brightness << PM8058_DRV_LED_CTRL_SHIFT) &
		PM8058_DRV_LED_CTRL_MASK;
/* FUJITSU:2011-05-06 MAPPING LED end */

/* FUJITSU:2011-04-25 LED start */
	if(system_rev <= 0x0b) {
		offset = PMIC8058_LED_OFFSET(led->id);
	} else {
		switch (led->id) {
		case PMIC8058_ID_LED_0:
			offset = PMIC8058_LED_OFFSET(PMIC8058_ID_LED_2);
			break;
		case PMIC8058_ID_LED_1:
			offset = PMIC8058_LED_OFFSET(PMIC8058_ID_LED_1);
			break;
		case PMIC8058_ID_LED_2:
			offset = PMIC8058_LED_OFFSET(PMIC8058_ID_LED_0);
			break;
		default:
			offset = PMIC8058_LED_OFFSET(led->id);
			break;
		}
	}
/* FUJITSU:2011-04-25 LED end */

	tmp = led->reg_led_ctrl[offset];

	tmp &= ~PM8058_DRV_LED_CTRL_MASK;
	tmp |= level;
	spin_unlock_irqrestore(&led->value_lock, flags);

	rc = pm8058_write(led->pm_chip,	SSBI_REG_ADDR_LED_CTRL(offset),
			&tmp, 1);
	if (rc) {
		dev_err(led->cdev.dev, "can't set (%d) led value\n",
				led->id);
		return;
	}

	spin_lock_irqsave(&led->value_lock, flags);
	led->reg_led_ctrl[offset] = tmp;
	spin_unlock_irqrestore(&led->value_lock, flags);
}

static enum led_brightness led_lc_get(struct pmic8058_led_data *led)
{
/* FUJITSU:2011-06-20 LED start */
#if 0
	int offset;
	u8 value;

	offset = PMIC8058_LED_OFFSET(led->id);
	value = led->reg_led_ctrl[offset];

	if ((value & PM8058_DRV_LED_CTRL_MASK) >>
			PM8058_DRV_LED_CTRL_SHIFT)
		return LED_FULL;
	else
		return LED_OFF;
#endif
	return led->brightness;
/* FUJITSU:2011-06-20 LED end */
}

static void
led_flash_set(struct pmic8058_led_data *led, enum led_brightness value)
{
	int rc;
	u8 level;
	unsigned long flags;
	u8 reg_flash_led;
	u16 reg_addr;

	spin_lock_irqsave(&led->value_lock, flags);
	level = (value << PM8058_DRV_FLASH_SHIFT) &
				 PM8058_DRV_FLASH_MASK;

	if (led->id == PMIC8058_ID_FLASH_LED_0) {
		led->reg_flash_led0 &= ~PM8058_DRV_FLASH_MASK;
		led->reg_flash_led0 |= level;
		reg_flash_led	    = led->reg_flash_led0;
		reg_addr	    = SSBI_REG_ADDR_FLASH_DRV0;
	} else {
		led->reg_flash_led1 &= ~PM8058_DRV_FLASH_MASK;
		led->reg_flash_led1 |= level;
		reg_flash_led	    = led->reg_flash_led1;
		reg_addr	    = SSBI_REG_ADDR_FLASH_DRV1;
	}
	spin_unlock_irqrestore(&led->value_lock, flags);

	rc = pm8058_write(led->pm_chip, reg_addr, &reg_flash_led, 1);
	if (rc)
		pr_err("%s: can't set flash led%d level %d\n", __func__,
			led->id, rc);
}

/* FUJITSU:2011-05-27 LED start */
static void bt_bl_set(struct pmic8058_led_data *led, enum led_brightness value)
{
/* FUJITSU:2011-05-31 LED start */
	if(g_light_lux >= thrtshold_light_lux)value = LED_OFF;
/* FUJITSU:2011-05-31 LED end */
	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_KEYLIGHT -1), value);
}

static enum led_brightness bt_bl_get(struct pmic8058_led_data *led)
{
	return (unsigned int)
	gpio_get_value_cansleep(
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_KEYLIGHT - 1));
}
/* FUJITSU:2011-05-27 LED end */

int pm8058_set_flash_led_current(enum pmic8058_leds id, unsigned mA)
{
	struct pmic8058_led_data *led;

	if ((id < PMIC8058_ID_FLASH_LED_0) || (id > PMIC8058_ID_FLASH_LED_1)) {
		pr_err("%s: invalid LED ID (%d) specified\n", __func__, id);
		return -EINVAL;
	}

	led = &led_data[id];
	if (!led) {
		pr_err("%s: flash led not available\n", __func__);
		return -EINVAL;
	}

	if (mA > MAX_FLASH_CURRENT)
		return -EINVAL;

	led_flash_set(led, mA / 20);

	return 0;
}
EXPORT_SYMBOL(pm8058_set_flash_led_current);

int pm8058_set_led_current(enum pmic8058_leds id, unsigned mA)
{
	struct pmic8058_led_data *led;
	int brightness = 0;
/* FUJITSU:2011-05-27 LED start */
	if ((id < PMIC8058_ID_LED_KB_LIGHT) || (id > PMIC8058_ID_LED_BT_LIGHT)) {
/* FUJITSU:2011-05-27 LED end */
		pr_err("%s: invalid LED ID (%d) specified\n", __func__, id);
		return -EINVAL;
	}

	led = &led_data[id];
	if (!led) {
		pr_err("%s: flash led not available\n", __func__);
		return -EINVAL;
	}

	switch (id) {
	case PMIC8058_ID_LED_0:
	case PMIC8058_ID_LED_1:
	case PMIC8058_ID_LED_2:
/* FUJITSU:2011-06-20 LED start */
		brightness = mA;
/* FUJITSU:2011-06-20 LED end */
		if (brightness  > led->cdev.max_brightness)
			return -EINVAL;
		led_lc_set(led, brightness);
		break;

	case PMIC8058_ID_LED_KB_LIGHT:
	case PMIC8058_ID_FLASH_LED_0:
	case PMIC8058_ID_FLASH_LED_1:
		brightness = mA / 20;
		if (brightness  > led->cdev.max_brightness)
			return -EINVAL;
		if (id == PMIC8058_ID_LED_KB_LIGHT)
			kp_bl_set(led, brightness);
		else
			led_flash_set(led, brightness);
		break;
/* FUJITSU:2011-05-27 LED start */
	case PMIC8058_ID_LED_BT_LIGHT:
		bt_bl_set(led, led->brightness);
		break;
	default:
		break;
/* FUJITSU:2011-05-27 LED end */
	}

	return 0;
}
EXPORT_SYMBOL(pm8058_set_led_current);

static void pmic8058_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct pmic8058_led_data *led;
	unsigned long flags;

	led = container_of(led_cdev, struct pmic8058_led_data, cdev);

	spin_lock_irqsave(&led->value_lock, flags);
	led->brightness = value;
	schedule_work(&led->work);
	spin_unlock_irqrestore(&led->value_lock, flags);
}

static void pmic8058_led_work(struct work_struct *work)
{
	struct pmic8058_led_data *led = container_of(work,
					 struct pmic8058_led_data, work);

	mutex_lock(&led->lock);

	switch (led->id) {
	case PMIC8058_ID_LED_KB_LIGHT:
		kp_bl_set(led, led->brightness);
		break;
/* FUJITSU:2011-06-20 LED start */
	case PMIC8058_ID_LED_0:
                pm8058_set_led_current(PMIC8058_ID_LED_0,((led->brightness >> 16) & 0xFF));
		break;
	case PMIC8058_ID_LED_1:
                pm8058_set_led_current(PMIC8058_ID_LED_1,((led->brightness >> 8) & 0xFF));
		break;
	case PMIC8058_ID_LED_2:
                pm8058_set_led_current(PMIC8058_ID_LED_2,(led->brightness & 0xFF));
		break;
	case PMIC8058_ID_LED_01:
                pm8058_set_led_current(PMIC8058_ID_LED_0,((led->brightness >> 16) & 0xFF));
                pm8058_set_led_current(PMIC8058_ID_LED_1,((led->brightness >> 8) & 0xFF));
		break;
	case PMIC8058_ID_LED_02:
                pm8058_set_led_current(PMIC8058_ID_LED_0,((led->brightness >> 16) & 0xFF));
                pm8058_set_led_current(PMIC8058_ID_LED_2,(led->brightness & 0xFF));
		break;
	case PMIC8058_ID_LED_12:
                pm8058_set_led_current(PMIC8058_ID_LED_1,((led->brightness >> 8) & 0xFF));
                pm8058_set_led_current(PMIC8058_ID_LED_2,(led->brightness & 0xFF));
		break;
	case PMIC8058_ID_LED_012:
                pm8058_set_led_current(PMIC8058_ID_LED_0,((led->brightness >> 16) & 0xFF));
                pm8058_set_led_current(PMIC8058_ID_LED_1,((led->brightness >> 8) & 0xFF));
                pm8058_set_led_current(PMIC8058_ID_LED_2,(led->brightness & 0xFF));
		break;
/* FUJITSU:2011-06-20 LED end */
	case PMIC8058_ID_FLASH_LED_0:
	case PMIC8058_ID_FLASH_LED_1:
		led_flash_set(led, led->brightness);
		break;
/* FUJITSU:2011-05-27 LED start */
	case PMIC8058_ID_LED_BT_LIGHT:
		bt_bl_set(led, led->brightness);
		break;
/* FUJITSU:2011-05-27 LED end */
	}

	mutex_unlock(&led->lock);
}

static enum led_brightness pmic8058_led_get(struct led_classdev *led_cdev)
{
	struct pmic8058_led_data *led;

	led = container_of(led_cdev, struct pmic8058_led_data, cdev);

	switch (led->id) {
	case PMIC8058_ID_LED_KB_LIGHT:
		return kp_bl_get(led);
	case PMIC8058_ID_LED_0:
	case PMIC8058_ID_LED_1:
	case PMIC8058_ID_LED_2:
/* FUJITSU:2011-06-20 LED start */
	case PMIC8058_ID_LED_01:
	case PMIC8058_ID_LED_02:
	case PMIC8058_ID_LED_12:
	case PMIC8058_ID_LED_012:
/* FUJITSU:2011-06-20 LED end */
		return led_lc_get(led);
/* FUJITSU:2011-05-27 LED start */
	case PMIC8058_ID_LED_BT_LIGHT:
		return bt_bl_get(led);
/* FUJITSU:2011-05-27 LED end */
	}
	return LED_OFF;
}

/* FUJITSU:2011-04-28 SUSPEND LED start */
ssize_t
led_notify_store (struct device * dev, struct device_attribute * attr,
		  const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul (buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
		led_notify = (unsigned char)(state & 0x000000ff);
		led_charge = (unsigned char)((state & 0x0000ff00) >> 8);
		led_charge = led_charge & 0x07;
	}

	return ret;
}

ssize_t
led_notify_show (struct device * dev, struct device_attribute * attr,
		 char *buf)
{

	unsigned char state = led_charge;
	state = (unsigned char)(state << 8);
	state = (unsigned char)(state + led_notify); 

	return sprintf (buf, "%u\n", state);
}


ssize_t
led_color_store (struct device * dev, struct device_attribute * attr,
                  const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	char *after;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	
	unsigned long state = simple_strtoul (buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		if (!strcmp (led_cdev->name,"red"))
			smem_led_color.led_red = state & 0xFF;
		if (!strcmp (led_cdev->name,"green"))
			smem_led_color.led_green = state & 0xFF;
		if (!strcmp (led_cdev->name,"blue"))
			smem_led_color.led_blue = state & 0xFF;
	}

	return ret;
}

ssize_t
led_color_show (struct device * dev, struct device_attribute * attr, char *buf)
{
	char state = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	
	if (!strcmp (led_cdev->name,"red"))
		state = smem_led_color.led_red;
	if (!strcmp (led_cdev->name,"green"))
		state = smem_led_color.led_green;
	if (!strcmp (led_cdev->name,"blue"))
		state = smem_led_color.led_blue;

	return sprintf (buf, "%d\n", state);
}

static int
pmic8058_led_suspend (struct platform_device *dev, pm_message_t state)
{

	static unsigned char *led_ctrl_notify = NULL;
	static unsigned char *led_ctrl_charge = NULL;
	static struct smem_led_color_type *led_ctrl_color = NULL;

	if (led_ctrl_notify == NULL) {
		led_ctrl_notify = (unsigned char *)smem_alloc_vendor1(SMEM_OEM_002);
	}

	if (led_ctrl_charge == NULL) {
		led_ctrl_charge = (unsigned char *)smem_alloc_vendor1(SMEM_OEM_003);
	}

	if (led_ctrl_color == NULL) {
		led_ctrl_color = (struct smem_led_color_type *)smem_alloc_vendor1(SMEM_OEM_010);
	}

	if (led_ctrl_notify != NULL) {
		*led_ctrl_notify = led_notify;
	}

	if (led_ctrl_charge != NULL) {
		*led_ctrl_charge = led_charge;
	}

	if (led_ctrl_color != NULL) {
		*led_ctrl_color = smem_led_color;
	}

	return 0;
}
/* FUJITSU:2011-04-28 SUSPEND LED end */

static int pmic8058_led_probe(struct platform_device *pdev)
{
	struct pmic8058_leds_platform_data *pdata = pdev->dev.platform_data;
	struct pmic8058_led_data *led_dat;
	struct pmic8058_led *curr_led;
	int rc, i = 0;
	struct pm8058_chip	*pm_chip;
	u8			reg_kp;
	u8			reg_led_ctrl[3];
	u8			reg_flash_led0;
	u8			reg_flash_led1;

	pm_chip = platform_get_drvdata(pdev);
	if (pm_chip == NULL) {
		dev_err(&pdev->dev, "no parent data passed in\n");
		return -EFAULT;
	}

	if (pdata == NULL) {
		dev_err(&pdev->dev, "platform data not supplied\n");
		return -EINVAL;
	}

	rc = pm8058_read(pm_chip, SSBI_REG_ADDR_DRV_KEYPAD, &reg_kp,
				1);
	if (rc) {
		dev_err(&pdev->dev, "can't get keypad backlight level\n");
		goto err_reg_read;
	}

	rc = pm8058_read(pm_chip, SSBI_REG_ADDR_LED_CTRL_BASE,
			reg_led_ctrl, 3);
	if (rc) {
		dev_err(&pdev->dev, "can't get led levels\n");
		goto err_reg_read;
	}

	rc = pm8058_read(pm_chip, SSBI_REG_ADDR_FLASH_DRV0,
			&reg_flash_led0, 1);
	if (rc) {
		dev_err(&pdev->dev, "can't read flash led0\n");
		goto err_reg_read;
	}

	rc = pm8058_read(pm_chip, SSBI_REG_ADDR_FLASH_DRV1,
			&reg_flash_led1, 1);
	if (rc) {
		dev_err(&pdev->dev, "can't get flash led1\n");
		goto err_reg_read;
	}

	for (i = 0; i < pdata->num_leds; i++) {
		curr_led	= &pdata->leds[i];
		led_dat		= &led_data[curr_led->id];

		led_dat->cdev.name		= curr_led->name;
		led_dat->cdev.default_trigger   = curr_led->default_trigger;
		led_dat->cdev.brightness_set    = pmic8058_led_set;
		led_dat->cdev.brightness_get    = pmic8058_led_get;
		led_dat->cdev.brightness	= LED_OFF;
		led_dat->cdev.max_brightness	= curr_led->max_brightness;
		led_dat->cdev.flags		= LED_CORE_SUSPENDRESUME;

		led_dat->id		        = curr_led->id;
		led_dat->reg_kp			= reg_kp;
		memcpy(led_data->reg_led_ctrl, reg_led_ctrl,
					 sizeof(reg_led_ctrl));
		led_dat->reg_flash_led0		= reg_flash_led0;
		led_dat->reg_flash_led1		= reg_flash_led1;

		if (!((led_dat->id >= PMIC8058_ID_LED_KB_LIGHT) &&
/* FUJITSU:2011-05-27 LED start */
				(led_dat->id <= PMIC8058_ID_LED_BT_LIGHT))) {
/* FUJITSU:2011-05-27 LED end */
			dev_err(&pdev->dev, "invalid LED ID (%d) specified\n",
						 led_dat->id);
			rc = -EINVAL;
			goto fail_id_check;
		}

		led_dat->pm_chip		= pm_chip;

		mutex_init(&led_dat->lock);
		spin_lock_init(&led_dat->value_lock);
		INIT_WORK(&led_dat->work, pmic8058_led_work);

		rc = led_classdev_register(&pdev->dev, &led_dat->cdev);
		if (rc) {
			dev_err(&pdev->dev, "unable to register led %d\n",
						 led_dat->id);
			goto fail_id_check;
		}
/* FUJITSU:2011-06-07 LED start */
		timer_trig_activate(&led_dat->cdev);
/* FUJITSU:2011-06-07 LED end */
	}

	platform_set_drvdata(pdev, led_data);

	return 0;

err_reg_read:
fail_id_check:
	if (i > 0) {
		for (i = i - 1; i >= 0; i--)
			led_classdev_unregister(&led_data[i].cdev);
	}
	return rc;
}

static int __devexit pmic8058_led_remove(struct platform_device *pdev)
{
	int i;
	struct pmic8058_leds_platform_data *pdata = pdev->dev.platform_data;
	struct pmic8058_led_data *led = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_leds; i++) {
		led_classdev_unregister(&led[led->id].cdev);
		cancel_work_sync(&led[led->id].work);
	}

	return 0;
}

static struct platform_driver pmic8058_led_driver = {
	.probe		= pmic8058_led_probe,
	.remove		= __devexit_p(pmic8058_led_remove),
/* FUJITSU:2011-04-28 SUSPEND LED start */
	.suspend = pmic8058_led_suspend,
/* FUJITSU:2011-04-28 SUSPEND LED end */
	.driver		= {
		.name	= "pm8058-led",
		.owner	= THIS_MODULE,
	},
};

static int __init pmic8058_led_init(void)
{
	return platform_driver_register(&pmic8058_led_driver);
}
module_init(pmic8058_led_init);

static void __exit pmic8058_led_exit(void)
{
	platform_driver_unregister(&pmic8058_led_driver);
}
module_exit(pmic8058_led_exit);

MODULE_DESCRIPTION("PMIC8058 LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pmic8058-led");
