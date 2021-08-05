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

#define pr_fmt(fmt)	"[Display][lge-backlight-cover:%s:%d] " fmt, __func__, __LINE__
#include "msm_drv.h"
#include "sde_dbg.h"

#include "sde_kms.h"
#include "sde_connector.h"
#include "sde_encoder.h"
#include <linux/backlight.h>
#include "dsi_drm.h"
#include "dsi_display.h"
#include "sde_crtc.h"

#include "../brightness/lge_brightness_def.h"
#include "../lge_dsi_panel.h"
#include "lge_cover_ctrl_ops.h"
#include "lge_backlight_cover.h"

#define BL_NODE_NAME_SIZE 32

extern struct ice40 *global_ice40;
extern int ice40_mcu_reg_write(struct ice40 *ice40, uint addr, uint val);

int br_to_offset_br(struct dsi_panel *panel, int br, int max_lvl, enum cover_br_type type)
{
	int offset, infp;
	int weight = 2;
	int min_br = 10;
	int max_br = ((max_lvl == 0) ? 255 : max_lvl);
	int offset_br = br;
	enum cover_br_type update_type = BR_XD;

	if (!panel) {
		pr_err("null ptr\n");
		return -EINVAL;
	}

	if (br < min_br) {
		pr_err("min br level\n");
		goto exit;
	}

	if (type == BR_DD && br > max_br) {
		pr_err("max br level\n");
		offset_br = max_br;
		goto exit;
	}

	if (panel->lge.br_offset_bypass) {
		pr_info("offset bypass (type=%d)\n", type);
		if (type == BR_MD)
			offset_br = br;
		else if (type == BR_DD) {
			panel->lge.br_offset_bypass = false;
			offset_br = br + (panel->lge.br_offset * weight);
		}

		goto exit;
	}

	if (panel->lge.br_offset_update) {
		offset_br = br + (panel->lge.br_offset * weight);
		goto exit;
	}

	offset = panel->lge.br_offset;
	if (offset > 0) {
		update_type = BR_MD;
	} else if (offset < 0) {
		update_type = BR_DD;
		offset = abs(offset);
	} else {
		goto exit;
	}

	offset *= weight;
	infp = min_br + (offset << 1);

	if (type == update_type) {
		if (br > infp)
			offset_br = br - offset;
		else {
			int tmp = infp - offset - min_br;
			tmp *= (br - min_br);
			tmp /= (infp - min_br);
			offset_br = min_br + tmp;
		}
	}

	if (offset_br < min_br) {
		pr_info("lower than min. br : set %d to %d\n", offset_br, min_br);
		offset_br = min_br;
	}

exit:
	pr_info("[%s] %d -> %d\n", ((type == BR_MD) ? "md" : "dd"), br, offset_br);
	return offset_br;
}

int dsi_display_set_backlight_cover(struct dsi_display *dsi_display, u32 bl_lvl)
{
	struct dsi_panel *panel;
	int rc = 0;

	if (dsi_display == NULL || dsi_display->panel == NULL)
		return -EINVAL;

	panel = dsi_display->panel;

	mutex_lock(&panel->panel_lock);

	if (bl_lvl > 255) {
		pr_err("Can't not set hbm mode for cover, set max brightness 255\n");
		bl_lvl = 255;
	}

	if (ice40_mcu_reg_write(global_ice40, ICE40_BL_REG, bl_lvl) < 0)
		pr_err("unable to set backlight\n");

	mutex_unlock(&panel->panel_lock);
	return rc;
}

int lge_update_backlight_cover(struct dsi_panel *panel)
{
	struct dsi_display *display;
	struct backlight_device *cover_bd;
	int rc = 0;

	if (panel == NULL || panel->host == NULL)
		return -EINVAL;

	cover_bd = panel->lge.bl_cover_device;
	if (cover_bd == NULL)
		return -EINVAL;

	mutex_lock(&cover_bd->ops_lock);

	if (panel->lge.bl_cover_lvl_unset < 0) {
		rc = 0;
		goto exit;
	}

	display = container_of(panel->host, struct dsi_display, host);

	rc = dsi_display_set_backlight_cover(display, panel->lge.bl_cover_lvl_unset);
	if (!rc) {
		pr_info("<--%pS unset=%d\n", __builtin_return_address(0), panel->lge.bl_cover_lvl_unset);
	}

	panel->lge.bl_cover_lvl_unset = -1;

exit:
	mutex_unlock(&cover_bd->ops_lock);
	return rc;
}

int lge_backlight_cover_device_update_status(struct backlight_device *bd)
{
	int brightness;
	struct dsi_panel *panel;
	struct dsi_display *display;
	struct sde_connector *c_conn;

	brightness = bd->props.brightness;

	c_conn = bl_get_data(bd);
	display = (struct dsi_display *) c_conn->display;
	panel = display->panel;

	if (panel->lge.br_offset != 0) {
		brightness = br_to_offset_br(panel, brightness, 0, BR_DD);
	}

	mutex_lock(&display->display_lock);
	if (panel->lge.lp_state == LGE_PANEL_NOLP && panel->lge.allow_bl_update) {
		panel->lge.bl_cover_lvl_unset = -1;
		dsi_display_set_backlight_cover(display, brightness);
		pr_info("BR:%d bl-cover\n", brightness);
	} else if (!panel->lge.allow_bl_update) {
		pr_info("brightness = %d -> not allowed, but considering in mcu side\n", brightness);
		panel->lge.bl_cover_lvl_unset = -1;
		dsi_display_set_backlight_cover(display, brightness);
	} else {
		panel->lge.bl_cover_lvl_unset = brightness;
		pr_info("brightness = %d -> differed bl-cover\n", brightness);
	}
	mutex_unlock(&display->display_lock);

	return 0;
}

static int lge_backlight_cover_device_get_brightness(struct backlight_device *bd)
{
	return 0;
}

static const struct backlight_ops lge_backlight_cover_device_ops = {
	.update_status = lge_backlight_cover_device_update_status,
	.get_brightness = lge_backlight_cover_device_get_brightness,
};

int lge_backlight_cover_setup(struct sde_connector *c_conn,
					struct drm_device *dev)
{
	struct backlight_properties props;
	struct dsi_panel *panel;
	struct dsi_display *display;
	struct dsi_backlight_config *bl_config;
	static int display_count;
	char bl_node_name[BL_NODE_NAME_SIZE];

	if (!c_conn || !dev || !dev->dev) {
		pr_err("invalid param\n");
		return -EINVAL;
	} else if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		return 0;
	}

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.power = FB_BLANK_UNBLANK;

	display = (struct dsi_display *) c_conn->display;
	panel = display->panel;
	bl_config = &panel->bl_config;
	props.max_brightness = bl_config->brightness_max_level;
	props.brightness = 0;
	snprintf(bl_node_name, BL_NODE_NAME_SIZE, "panel%u-backlight-cover",
							display_count);
	panel->lge.bl_cover_lvl_unset = -1;
	panel->lge.bl_cover_device = backlight_device_register(bl_node_name, dev->dev,
			c_conn, &lge_backlight_cover_device_ops, &props);
	if (IS_ERR_OR_NULL(panel->lge.bl_cover_device)) {
		SDE_ERROR("Failed to register backlight-cover: %ld\n",
				    PTR_ERR(panel->lge.bl_cover_device));
		panel->lge.bl_cover_device = NULL;
		return -ENODEV;
	}
	display_count++;

	return 0;
}

void lge_backlight_cover_destroy(struct sde_connector *c_conn)
{
	struct dsi_display *display = (struct dsi_display *) c_conn->display;
	if (display->panel->lge.bl_cover_device)
		backlight_device_unregister(display->panel->lge.bl_cover_device);
}
