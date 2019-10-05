/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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
 */
#ifndef LGE_COLOR_MANAGER_H
#define LGE_COLOR_MANAGER_H

#include "dsi_panel.h"
#include "../lge_dsi_panel.h"

#define BC_DIM_TIME msecs_to_jiffies(550)
#define BC_CTRL_REG             0xB8
#define BC_CTRL_REG_NUM         0x18
#define BC_DIM_FRAMES_NORMAL    0x02
#define BC_DIM_FRAMES_VE        0x10
#define BC_DIM_FRAMES_THERM     0x10
#define BC_DIM_BRIGHTNESS_THERM 0x0100
#define BC_DIM_MIN_FRAMES       0x02
#define BC_DIM_MAX_FRAMES       0x7E
#define BC_DIM_ON               0x01
#define BC_DIM_OFF              0x00

#define HDR_OFF  	0
#define CABC_REG0	0
#define CABC_REG1	1
#define CABC_ON 	0x26
#define CABC_OFF	0x02
#define IE_ON		0x81
#define IE_OFF		0x00

#define RGB_DEFAULT_PRESET	2
#define RGB_DEFAULT_RED		0
#define RGB_DEFAULT_BLUE	0
#define RGB_DEFAULT_GREEN	0

#define TRUE_VIEW_MAX_STEP	30

enum {
	RED      = 0,
	GREEN    = 1,
	BLUE     = 2,
	RGB_ALL  = 3
};

enum {
	PRESET_SETP0_INDEX = 0,
	PRESET_SETP1_INDEX = 6,
	PRESET_SETP2_INDEX = 12,
};

enum {
	screen_mode_auto = 0,
	screen_mode_cinema,
	screen_mode_photos,
	screen_mode_web,
	screen_mode_sports,
	screen_mode_game,
	screen_mode_expert = 10,
};

int lge_color_manager_create_sysfs(struct dsi_panel *panel, struct device *panel_sysfs_dev);
#endif /* LGE_COLOR_MANAGER_H */
