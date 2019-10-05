/*
 * Copyright (c) 2018, LG Eletronics. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _LGE_PRM_H_
#define _LGE_PRM_H_

#include <linux/mutex.h>
#include <linux/device.h>

#define PRM_TAG "prm_log: "

enum {
	LGE_PRM_INIT_DONE   = 0,
	LGE_PRM_INIT_VFPS   = 1,
	LGE_PRM_INIT_SBEN   = 2,
	LGE_PRM_INIT_FBCN   = 4,
	LGE_PRM_INIT_TRITON = 8,
	LGE_PRM_INIT_DD     = 16,
	LGE_PRM_INIT_MAX  =
		LGE_PRM_INIT_VFPS|
		LGE_PRM_INIT_SBEN|
		LGE_PRM_INIT_FBCN|
		LGE_PRM_INIT_TRITON|
		LGE_PRM_INIT_DD,
};

enum {
	LGE_PRM_INFO_DISPLAY_MAIN_STATE   = 0,
	LGE_PRM_INFO_DISPLAY_DD_STATE     = 1,
	LGE_PRM_INFO_VFPS_ENABLED         = 2,
	LGE_PRM_INFO_SBEN_ENABLED         = 3,
	LGE_PRM_INFO_FBCN_ENABLED         = 4,
	LGE_PRM_INFO_TRITON_ENABLED       = 5,
	LGE_PRM_INFO_DD_ENABLED           = 6,
	LGE_PRM_INFO_MAX,
};

typedef enum display_event {
	LGE_PRM_DISPLAY_EVENT_MAIN_STATE  = 0,
	LGE_PRM_DISPLAY_EVENT_DD_STATE    = 1,
	LGE_PRM_DISPLAY_EVENT_MAX,
} display_event_t;

enum {
	LGE_PRM_DISPLAY_OFF    = 0,
	LGE_PRM_DISPLAY_ON     = 1,
	LGE_PRM_DISPLAY_MAX,
};

struct lge_prm {
	struct class  *cnode;
	int           enable_mask;
	int           init_mask;
	/* need to atomic operation, using prm_lock */
	struct mutex  prm_lock;
	int           main_display_mode;
	int           dd_display_mode;
};

int lge_prm_get_info(int info);
int lge_prm_get_class_node(struct class **cdev);

void lge_prm_display_set_event(enum display_event event, int value);
#endif // _LGE_PRM_H_
