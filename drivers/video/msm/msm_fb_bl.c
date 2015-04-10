/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/backlight.h>

#include "msm_fb.h"


/* FUJITSU:2011-06-02 change bl_curve start */
static u8 custom_curve[FB_BACKLIGHT_LEVELS] = {
0,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
2,
2,
2,
2,
3,
3,
3,
3,
4,
4,
4,
4,
5,
5,
5,
5,
6,
6,
6,
6,
7,
7,
7,
7,
8,
8,
8,
8,
9,
9,
9,
9,
10,
10,
10,
10,
11,
11,
11,
11,
12,
12,
12,
12,
13,
13,
13,
13,
14,
14,
14,
14,
15,
15,
15,
15,
16,
16,
16,
16,
17,
17,
17,
17,
18,
18,
18,
18,
19,
19,
19,
19,
20,
20,
20,
20,
21,
21,
22,
22,
23,
23,
24,
24,
25,
25,
26,
26,
27,
27,
28,
28,
29,
29,
30,
30,
31,
31,
32,
32,
33,
33,
34,
34,
35,
35,
36,
36,
37,
37,
38,
38,
39,
39,
40,
40,
41,
41,
42,
42,
43,
43,
44,
44,
45,
45,
46,
46,
47,
47,
48,
48,
49,
49,
50,
50,
51,
51,
52,
52,
53,
53,
54,
54,
55,
55,
56,
56,
57,
57,
58,
58,
59,
59,
60,
60,
61,
61,
62,
62,
63,
63,
64,
64,
65,
65,
66,
66,
67,
67,
68,
68,
69,
69,
70,
70,
71,
72,
73,
74,
75,
76,
77,
78,
79,
80,
81,
82,
83,
84,
85,
86,
87,
88,
89,
90,
91,
92,
93,
94,
95,
96,
97,
98,
99,
100,
101,
102,
103,
104,
105,
106,
107,
108,
109,
110,
111,
112,
113,
114,
115,
116,
117,
118,
119,
120,
};
/* FUJITSU:2011-06-02 change bl_curve end */

static int msm_fb_bl_get_brightness(struct backlight_device *pbd)
{
	return pbd->props.brightness;
}

static int msm_fb_bl_update_status(struct backlight_device *pbd)
{
	struct msm_fb_data_type *mfd = bl_get_data(pbd);
	__u32 bl_lvl;

	bl_lvl = pbd->props.brightness;
	bl_lvl = mfd->fbi->bl_curve[bl_lvl];
	msm_fb_set_backlight(mfd, bl_lvl);
	return 0;
}

static struct backlight_ops msm_fb_bl_ops = {
	.get_brightness = msm_fb_bl_get_brightness,
	.update_status = msm_fb_bl_update_status,
};

void msm_fb_config_backlight(struct msm_fb_data_type *mfd)
{
	struct msm_fb_panel_data *pdata;
	struct backlight_device *pbd;
	struct fb_info *fbi;
	char name[16];
	struct backlight_properties props;

	int i = 0;/* FUJITSU:2011-06-02 change bl_curve */

	fbi = mfd->fbi;
	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

	if ((pdata) && (pdata->set_backlight)) {
		snprintf(name, sizeof(name), "msmfb_bl%d", mfd->index);
		props.max_brightness = FB_BACKLIGHT_LEVELS - 1;
		props.brightness = FB_BACKLIGHT_LEVELS - 1;
		pbd =
		    backlight_device_register(name, fbi->dev, mfd,
					      &msm_fb_bl_ops, &props);
		if (!IS_ERR(pbd)) {
			fbi->bl_dev = pbd;
			fb_bl_default_curve(fbi,
					    0,
					    mfd->panel_info.bl_min,
					    mfd->panel_info.bl_max);

/* FUJITSU:2011-06-02 change bl_curve start */
			for (i=0;i<FB_BACKLIGHT_LEVELS;i++) {
				fbi->bl_curve[i] = custom_curve[i];
			}
/* FUJITSU:2011-06-02 change bl_curve end */

		} else {
			fbi->bl_dev = NULL;
			printk(KERN_ERR "msm_fb: backlight_device_register failed!\n");
		}
	}
}
