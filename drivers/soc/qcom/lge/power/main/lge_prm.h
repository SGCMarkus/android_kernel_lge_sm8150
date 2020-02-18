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
	LGE_PRM_INFO_CONNECTION_DS2_STATE = 7,
	LGE_PRM_INFO_HALLIC_STATE         = 8,
	LGE_PRM_INFO_DS_STATE             = 9,
	LGE_PRM_INFO_MAX,
};

typedef enum display_event {
	LGE_PRM_DISPLAY_EVENT_MAIN_STATE   = 0,
	LGE_PRM_DISPLAY_EVENT_DD_STATE     = 1,
	LGE_PRM_DISPLAY_EVENT_DD2_STATE    = 2,
	LGE_PRM_DISPLAY_EVENT_HALLIC_STATE = 3,
	LGE_PRM_DISPLAY_EVENT_DS_STATE     = 4,
	LGE_PRM_DISPLAY_EVENT_MAX,
} display_event_t;

enum {
	LGE_PRM_DISPLAY_OFF    = 0,
	LGE_PRM_DISPLAY_ON     = 1,
	LGE_PRM_DISPLAY_MAX,
};

enum {
	LGE_PRM_DS2_DISCONNECTION = 0,
	LGE_PRM_DS2_CONNECTION    = 1,
	LGE_PRM_DS2_CONNECTION_MAX,
};

enum {
	LGE_PRM_HALLIC_OFF    = 0,
	LGE_PRM_HALLIC_ON     = 1,
	LGE_PRM_HALLIC_MAX,
};

enum {
	LGE_PRM_DS_CONNECTION_OFF    = 0,
	LGE_PRM_DS1_CONNECTION_ON    = 1,
	LGE_PRM_DS2_CONNECTION_ON    = 2,
	LGE_PRM_HALLIC_CONNECTION_ON = 3,
	LGE_PRM_DS_STATE_MAX,
};

struct lge_prm {
	bool init_done_flag;
	struct class  *cnode;
	int           enable_mask;
	int           init_mask;
	/* need to atomic operation, using prm_lock */
	struct mutex  prm_lock;
	int           main_display_mode;
	int           ds1_display_mode;
	int           ds2_connect_mode;
	int           hallic_mode;
	int           ds_state;
};

#ifdef CONFIG_LGE_PM_PRM
int lge_prm_get_info(int info);
int lge_prm_get_class_node(struct class **cdev);

void lge_prm_display_set_event(enum display_event event, int value);
#else
int lge_prm_get_info(int info) {return 0};
int lge_prm_get_class_node(struct class **cdev) {return 0};

void lge_prm_display_set_event(enum display_event event, int value) {return 0};
#endif

#endif // _LGE_PRM_H_
