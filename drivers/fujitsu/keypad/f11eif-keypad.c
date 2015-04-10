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
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/interrupt.h>
#include <asm/mach-types.h>
#ifdef CONFIG_MACH_F11EIF
#include <mach/irqs-7x30.h>
#endif //CONFIG_MACH_F11EIF

#ifdef CONFIG_MACH_F11EIF
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#endif //CONFIG_MACH_F11EIF

static struct gpio_event_direct_entry keypad_nav_map[] = {
#ifdef CONFIG_MACH_F11K06
    { 166,   KEY_CAMERA    },
    { 165,   KEY_ZOOM  },
    { 164,   KEY_VOLUMEDOWN },
    { 163,   KEY_VOLUMEUP   },
    { 112,   KEY_BACK    },
    { 113,   KEY_HOME    },
    { 114,   KEY_MENU    },
#else
#ifdef CONFIG_MACH_F11EIF
    {115,	KEY_HOME},
    {114,	KEY_MENU},
    {113,	KEY_BACK},
    { 67,	KEY_VOLUMEUP},
    { 69,	KEY_VOLUMEDOWN},
#else
#error code not present
#endif //CONFIG_MACH_F11EIF
#endif //CONFIG_MACH_F11K06
};

static int keypad_gpio_event_nav_func(
    struct gpio_event_input_devs  *input_dev,
    struct gpio_event_info *info,
    void **data, int func);

static struct gpio_event_input_info keypad_nav_info = {
        .info.func = keypad_gpio_event_nav_func,
        .flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE,
        .type = EV_KEY,
        .keymap = keypad_nav_map,
        .debounce_time.tv.nsec = 20 * NSEC_PER_MSEC,
        .keymap_size = ARRAY_SIZE(keypad_nav_map)
};

static struct gpio_event_info *keypad_info[] = {
    &keypad_nav_info.info
};

static struct gpio_event_platform_data keypad_data = {
    .name       = "keypad",
    .info       = keypad_info,
    .info_count = ARRAY_SIZE(keypad_info)
};

#ifdef CONFIG_MACH_F11K06
struct platform_device keypad_device_f11k06 = {
#else
#ifdef CONFIG_MACH_F11EIF
struct platform_device keypad_device_f11eif = {
#else
#error code not present
#endif //CONFIG_MACH_F11EIF
#endif //CONFIG_MACH_F11K06
    .name   = GPIO_EVENT_DEV_NAME,
    .id = -1,
    .dev    = {
        .platform_data  = &keypad_data,
    },
};

struct gpio_event_input_devs *keypad_dev;

static int keypad_gpio_event_nav_func(
    struct gpio_event_input_devs  *input_dev,
    struct gpio_event_info *info,
    void **data, int func)
{
    int err;

#ifdef CONFIG_MACH_F11EIF
    if(system_rev >= 0x0b) {
        int i;
        for(i=0;i<keypad_nav_info.keymap_size;i++) {
            keypad_nav_map[i].gpio = PM8058_GPIO_PM_TO_SYS(i);
            //printk("%s:re-set to old gpio[%d:%d].\n",__func__,i,keypad_nav_map[i].gpio);
        }
    }
#endif //CONFIG_MACH_F11EIF

    err = gpio_event_input_func(input_dev, info, data, func);

    if (func == GPIO_EVENT_FUNC_INIT && !err) {
        keypad_dev = input_dev;
    } else if (func == GPIO_EVENT_FUNC_UNINIT) {
        keypad_dev = NULL;
    }

    return err;
}

struct gpio_event_input_devs *msm_keypad_get_input_dev(void)
{
    return keypad_dev;
}
MODULE_LICENSE("GPL");
