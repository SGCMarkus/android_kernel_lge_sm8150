/*
 *   copyright (c) 2018, The Linux Foundation. All rights reserved.
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

#ifndef __LINUX_LGE_PANEL_NOTIFY_H
#define __LINUX_LGE_PANEL_NOTIFY_H

#include <linux/notifier.h>

struct lge_panel_notifier {
	int display_id;
	int state;
};

enum {
	LGE_PANEL_EVENT_BLANK,
	LGE_PANEL_EVENT_RESET,
	LGE_PANEL_EVENT_POWER,
	LGE_PANEL_EVENT_RECOVERY,
	LGE_PANEL_EVENT_DUAL_DISPLAY,
}; /* LGE_PANEL_EVENT_TYPE */

enum {
	LGE_PANEL_STATE_UNBLANK,
	LGE_PANEL_STATE_LP1,
	LGE_PANEL_STATE_LP2,
	LGE_PANEL_STATE_BLANK,
}; /* LGE_PANEL_EVENT_BLANK */

enum {
	LGE_PANEL_RESET_LOW,
	LGE_PANEL_RESET_HIGH,
}; /* LGE_PANEL_EVENT_RESET */

enum {
	LGE_PANEL_POWER_VDDIO_ON,
	LGE_PANEL_POWER_VDDIO_OFF,
	LGE_PANEL_POWER_DSV_ON,
	LGE_PANEL_POWER_DSV_OFF,
}; /* LGE_PANEL_EVENT_POWER */

enum {
	LGE_PANEL_RECOVERY_DEAD,
	LGE_PANEL_RECOVERY_ALIVE,
}; /* LGE_PANEL_EVENT_RECOVERY */

enum {
	LGE_PANEL_DD_NOT_CONNECTED,
	LGE_PANEL_DD_CONNECTED,
}; /* LGE_PANEL_EVENT_DUAL_DISPLAY */

/**
 * lge_panel_notifier_register_client - register a client notifier
 * @nb: notifier block to callback on events
 *
 * This function registers a notifier callback function
 * to msm_drm_notifier_list, which would be called when
 * received unblank/power down event.
 */
int lge_panel_notifier_register_client(struct notifier_block *nb);

/**
 * lge_panel_notofier_unregister_client - unregister a client notifier
 * @nb: notifier block to callback on events
 *
 * This function unregisters the callback function from
 * msm_drm_notifier_list.
 */
int lge_panel_notifier_unregister_client(struct notifier_block *nb);

/**
 * lge_panel_notifier_call_chain - notify clients of lge_panel_events
 * @val: event UNBLANK, LP1, LP2, BLANK, RESET
 * @display_id: display_id
 * @state: event state
 */
int lge_panel_notifier_call_chain(unsigned long val, int display_id, int state);
#endif
