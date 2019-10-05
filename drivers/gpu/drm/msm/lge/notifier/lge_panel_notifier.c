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

#define pr_fmt(fmt)		"[Display][lge-panel-notifier:%s:%d: " fmt, __func__, __LINE__

#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/lge_panel_notify.h>
#include "dsi_display.h"
#include "sde_connector.h"

static BLOCKING_NOTIFIER_HEAD(lge_panel_notifier_list);

/**
 * lge_panel_notifier_register_client - register a client notifier
 * @nb: notifier block to callback on events
 *
 * This function registers a notifier callback function
 * to msm_drm_notifier_list, which would be called when
 * received unblank/power down event.
 */
int lge_panel_notifier_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&lge_panel_notifier_list, nb);
}

/**
 * lge_panel_notofier_unregister_client - unregister a client notifier
 * @nb: notifier block to callback on events
 *
 * This function unregisters the callback function from
 * msm_drm_notifier_list.
 */
int lge_panel_notifier_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&lge_panel_notifier_list, nb);
}

/**
 * lge_panel_notifier_call_chain - notify clients of lge_panel_events
 * @val: event UNBLANK, LP1, LP2, BLANK, RESET
 * @display_id: display_id
 * @state: event state
 */
int lge_panel_notifier_call_chain(unsigned long val, int display_id, int state)
{
	struct lge_panel_notifier notifier_data;
	notifier_data.display_id = display_id;
	notifier_data.state = state;

	return blocking_notifier_call_chain(&lge_panel_notifier_list, val, &notifier_data);
}
