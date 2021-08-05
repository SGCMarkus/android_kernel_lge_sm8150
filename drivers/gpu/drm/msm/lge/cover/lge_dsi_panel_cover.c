/*
 * Copyright(c) 2018, LG Electronics. All rights reserved.
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

#define pr_fmt(fmt)	"[Display][lge-dsi-panel_cover:%s:%d] " fmt, __func__, __LINE__

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <video/mipi_display.h>

#include "dsi_panel.h"
#include "dsi_ctrl_hw.h"
#include "lge_dsi_panel_cover.h"
#include "lge_cover_ctrl.h"
#include "../dp/lge_dp_def.h"
#if defined(CONFIG_LGE_COVER_DISPLAY)
#include "lge_cover_color_manager.h"
#endif

extern struct lge_dp_display *get_lge_dp(void);

void lge_cover_create_sysfs(struct dsi_panel *panel)
{
	static struct class *class_panel_cover = NULL;
#if defined(CONFIG_LGE_COVER_DISPLAY)
	static struct device *panel_cover_img_tune_sysfs_dev = NULL;
#endif
	static struct device *panel_cover_ctrl_sysfs_dev = NULL;
	struct lge_dp_display *lge_dp;

	if (!class_panel_cover) {
		class_panel_cover = class_create(THIS_MODULE, "panel_cover");
		if (IS_ERR(class_panel_cover)) {
			pr_err("Failed to create panel class\n");
			return;
		}
	}

#if defined(CONFIG_LGE_COVER_DISPLAY)
	if (!panel_cover_img_tune_sysfs_dev) {
		panel_cover_img_tune_sysfs_dev = device_create(class_panel_cover, NULL, 0, panel, "img_tune");
		if (IS_ERR(panel_cover_img_tune_sysfs_dev)) {
			pr_err("Failed to create dev(panel_cover_img_tune_sysfs_dev)!\n");
		} else {
			if (panel->lge.use_color_manager)
				lge_cover_color_manager_create_sysfs(panel_cover_img_tune_sysfs_dev);
		}
	}
#endif

	lge_dp = get_lge_dp();

	if (!panel_cover_ctrl_sysfs_dev) {
		panel_cover_ctrl_sysfs_dev = device_create(class_panel_cover, NULL, 0, lge_dp, "cover_ctrl");
		if (IS_ERR(panel_cover_ctrl_sysfs_dev)) {
			pr_err("Failed to create dev(cover_ctrl_sysfs_dev)!\n");
		} else {
			lge_cover_ctrl_create_sysfs(panel_cover_ctrl_sysfs_dev);
		}
	}

}
