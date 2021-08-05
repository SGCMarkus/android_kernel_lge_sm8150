/*
 * Copyright(c) 2017, LG Electronics. All rights reserved.
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

#define pr_fmt(fmt)     "[Display][lge-cm:%s:%d] " fmt, __func__, __LINE__

#include <linux/kallsyms.h>
#include "lge_color_manager.h"
#include "lge_dsi_panel_def.h"
#include "lge_dsi_panel.h"
#include "dsi_display.h"
#include <linux/delay.h>

static ssize_t daylight_mode_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 1 );
}

static ssize_t daylight_mode_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;
	sscanf(buf, "%d", &input);
	pr_info("%s input data is %d\n", __func__, input);
	return ret;
}

static DEVICE_ATTR(daylight_mode, S_IRUGO | S_IWUSR | S_IWGRP,
		daylight_mode_get, daylight_mode_set);

static ssize_t hdr_mode_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	panel = dev_get_drvdata(dev);

	if ((!panel) || (!panel->host)) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	if(!dsi_panel_initialized(panel)) {
		pr_err("panel not yet initialized\n");
		return -EINVAL;
	}

	pr_info("get hdr mode : %d\n", panel->lge.hdr_mode);

	return sprintf(buf, "%d\n", panel->lge.hdr_mode);
}

static ssize_t hdr_mode_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input;
	panel = dev_get_drvdata(dev);

	if ((!panel) || (!panel->host)) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	if(!dsi_panel_initialized(panel)) {
		pr_err("panel not yet initialized\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);
	panel->lge.hdr_mode = input;
	pr_info("hdr mode : %d \n", panel->lge.hdr_mode);

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->hdr_mode_set)
		panel->lge.ddic_ops->hdr_mode_set(panel, panel->lge.hdr_mode);

	return ret;
}
static DEVICE_ATTR(hdr_mode, S_IRUGO | S_IWUSR | S_IWGRP,
		hdr_mode_get, hdr_mode_set);

static ssize_t sharpness_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;

	panel = dev_get_drvdata(dev);
	if (panel == NULL) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", panel->lge.sharpness);
}

static ssize_t sharpness_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input;

	panel = dev_get_drvdata(dev);
	if (panel == NULL) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	if (panel->lge.ddic_ops == NULL) {
		pr_err("panel ops is NULL\n");
		return -EINVAL;
	}

	if(!dsi_panel_initialized(panel)) {
		pr_err("panel not yet initialized\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);
	panel->lge.sharpness = input;
	pr_info("ctrl->sharpness (%d)\n", panel->lge.sharpness);

	if (panel->lge.ddic_ops->sharpness_set)
		panel->lge.ddic_ops->sharpness_set(panel, input);
	else
		pr_err("Can not find sharpness_set\n");
	return ret;
}

static DEVICE_ATTR(sharpness, S_IRUGO | S_IWUSR | S_IWGRP,
		sharpness_get, sharpness_set);

static ssize_t screen_mode_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int len = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return len;
	}

	return sprintf(buf, "%d\n", panel->lge.screen_mode);
}

static ssize_t screen_mode_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input;

	panel = dev_get_drvdata(dev);
	if (panel == NULL) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	if (!dsi_panel_initialized(panel)) {
		pr_err("Panel off state. Ignore screen_mode set cmd\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);
	panel->lge.screen_mode = input;

	pr_info("ctrl->screen_mode (%d)\n", panel->lge.screen_mode);

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_set_screen_mode)
		panel->lge.ddic_ops->lge_set_screen_mode(panel, true);
	else
		pr_err("Can not find lge_set_screen_mode\n");

	return ret;
}
static DEVICE_ATTR(screen_mode, S_IRUGO | S_IWUSR | S_IWGRP,
					screen_mode_get, screen_mode_set);

static ssize_t screen_tune_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d %d %d %d \n", panel->lge.sc_sat_step,
					panel->lge.sc_hue_step,
					panel->lge.sc_sha_step,
					panel->lge.color_filter);
}

static ssize_t screen_tune_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input_param[4];

	panel = dev_get_drvdata(dev);
	if (panel == NULL) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	if (!dsi_panel_initialized(panel)) {
		pr_err("Panel off state. Ignore screen_mode set cmd\n");
		return -EINVAL;
	}

	sscanf(buf, "%d %d %d %d", &input_param[0], &input_param[1], &input_param[2], &input_param[3]);

	panel->lge.sc_sat_step		= abs(input_param[0]);
	panel->lge.sc_hue_step		= abs(input_param[1]);
	panel->lge.sc_sha_step		= abs(input_param[2]);
	panel->lge.color_filter		= abs(input_param[3]);

	pr_info("sat : %d , hue = %d , sha = %d , CF = %d \n",
			panel->lge.sc_sat_step, panel->lge.sc_hue_step,
			panel->lge.sc_sha_step, panel->lge.color_filter);

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_set_screen_tune)
		panel->lge.ddic_ops->lge_set_screen_tune(panel);

	panel->lge.sharpness_status = 0x01;

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_display_control_store)
		panel->lge.ddic_ops->lge_display_control_store(panel, true);

	return ret;
}
static DEVICE_ATTR(screen_tune, S_IRUGO | S_IWUSR | S_IWGRP,
					screen_tune_get, screen_tune_set);

static ssize_t rgb_tune_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d %d %d %d \n", panel->lge.cm_preset_step,
					panel->lge.cm_red_step,
					panel->lge.cm_green_step,
					panel->lge.cm_blue_step);
}

static ssize_t rgb_tune_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input_param[4];

	panel = dev_get_drvdata(dev);
	if (panel == NULL) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	if (!dsi_panel_initialized(panel)) {
		pr_err("Panel off state. Ignore rgb_tune_set set cmd\n");
		return -EINVAL;
	}

	sscanf(buf, "%d %d %d %d", &input_param[0], &input_param[1], &input_param[2], &input_param[3]);

	panel->lge.cm_preset_step = input_param[0];
	panel->lge.cm_red_step    = abs(input_param[1]);
	panel->lge.cm_green_step  = abs(input_param[2]);
	panel->lge.cm_blue_step   = abs(input_param[3]);

	if(panel->lge.cm_preset_step > 4)
		panel->lge.cm_preset_step = 4;

	pr_info("preset : %d , red = %d , green = %d , blue = %d \n",
			panel->lge.cm_preset_step, panel->lge.cm_red_step,
			panel->lge.cm_green_step, panel->lge.cm_blue_step);

	mutex_lock(&panel->panel_lock);
	if (!(panel->lge.screen_mode == screen_mode_auto ||
			panel->lge.screen_mode == screen_mode_expert)) {
		pr_info("skip rgb set with screen mode on\n");
		mutex_unlock(&panel->panel_lock);
		return ret;
	}
	mutex_unlock(&panel->panel_lock);

	if (panel->lge.color_manager_default_status) {
		panel->lge.color_manager_mode = panel->lge.color_manager_table[0].color_manager_mode;
		panel->lge.color_manager_status = panel->lge.color_manager_table[0].color_manager_status;

		if (panel->lge.cm_preset_step == 2 &&
			 !(panel->lge.cm_red_step | panel->lge.cm_green_step | panel->lge.cm_blue_step)) {
			if (panel->lge.dgc_absent && panel->lge.ddic_ops && panel->lge.ddic_ops->lge_set_custom_rgb)
				panel->lge.ddic_ops->lge_set_custom_rgb(panel, true);
			panel->lge.dgc_status = 0x00;
		} else {
			if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_set_custom_rgb)
				panel->lge.ddic_ops->lge_set_custom_rgb(panel, true);
			panel->lge.dgc_status = 0x01;
		}
	} else {
		if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_set_rgb_tune)
			panel->lge.ddic_ops->lge_set_rgb_tune(panel, true);
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_display_control_store)
		panel->lge.ddic_ops->lge_display_control_store(panel, true);

	return ret;
}
static DEVICE_ATTR(rgb_tune, S_IRUGO | S_IWUSR | S_IWGRP,
					rgb_tune_get, rgb_tune_set);

static ssize_t color_manager_status_get(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int len = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return len;
	}
	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", panel->lge.color_manager_status);
}

static ssize_t color_manager_status_set(struct device *dev,
						struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input;

	panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	if (!dsi_panel_initialized(panel)) {
		pr_err("Panel off state. Ignore screen_mode set cmd\n");
		return -EINVAL;
	}

	if (!panel->lge.color_manager_default_status) {
		pr_info("Color manager is disabled as default. Ignore color manager status control.\n");
		return ret;
	}

	sscanf(buf, "%d", &input);

	panel->lge.color_manager_status = input & 0x01;

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_display_control_store)
		panel->lge.ddic_ops->lge_display_control_store(panel, true);

	if (panel->lge.color_manager_status && panel->lge.ddic_ops && panel->lge.ddic_ops->lge_send_screen_mode_cmd)
		panel->lge.ddic_ops->lge_send_screen_mode_cmd(panel, panel->lge.color_manager_mode);

	pr_info("color_manager_status %d \n", panel->lge.color_manager_status);
	return ret;
}
static DEVICE_ATTR(color_manager_status,  S_IRUGO | S_IWUSR | S_IWGRP,
					color_manager_status_get, color_manager_status_set);

static ssize_t color_manager_mode_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int len = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return len;
	}
	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", panel->lge.color_manager_mode);
}

static ssize_t color_manager_mode_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input;

	panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	panel->lge.color_manager_mode = input & 0x03;
	panel->lge.color_manager_status = 0x01;
	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_display_control_store)
		panel->lge.ddic_ops->lge_display_control_store(panel, true);

	pr_info("color_manager_mode %d\n",panel->lge.color_manager_mode);
	return ret;
}
static DEVICE_ATTR(color_manager_mode, S_IWUSR|S_IRUGO,
					color_manager_mode_get, color_manager_mode_set);

static ssize_t hdr_hbm_lut_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int len = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return len;
	}
	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", panel->lge.hdr_hbm_lut);
}

static ssize_t hdr_hbm_lut_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input;

	panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	panel->lge.hdr_hbm_lut = input;

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_set_hdr_hbm_lut)
		panel->lge.ddic_ops->lge_set_hdr_hbm_lut(panel, panel->lge.hdr_hbm_lut);

	return ret;
}
static DEVICE_ATTR(hdr_hbm_lut, S_IWUSR|S_IRUGO,
					hdr_hbm_lut_get, hdr_hbm_lut_set);

static ssize_t acl_mode_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int len = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return len;
	}
	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", panel->lge.acl_mode);
}

static ssize_t acl_mode_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input;

	panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	panel->lge.acl_mode = input;

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_set_acl_mode)
		panel->lge.ddic_ops->lge_set_acl_mode(panel, panel->lge.acl_mode);

	return ret;
}
static DEVICE_ATTR(acl_mode, S_IWUSR|S_IRUGO, acl_mode_get, acl_mode_set);

static ssize_t video_enhancement_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int len = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return len;
	}
	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", panel->lge.video_enhancement);
}

static ssize_t video_enhancement_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input;

	panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	panel->lge.video_enhancement = input;

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_set_video_enhancement)
		panel->lge.ddic_ops->lge_set_video_enhancement(panel, panel->lge.video_enhancement);

	return ret;
}
static DEVICE_ATTR(video_enhancement, S_IRUGO | S_IWUSR | S_IWGRP,
					video_enhancement_get, video_enhancement_set);

void lge_mdss_dsi_bc_dim_work(struct work_struct *work)
{
	struct lge_dsi_panel *lge_panel = NULL;
	struct dsi_panel *panel = NULL;

	lge_panel = container_of(to_delayed_work(work), struct lge_dsi_panel, bc_dim_work);
	if (!lge_panel) {
		pr_err("fail to get lge_dsi_panel object\n");
		return;
	}

	panel = container_of(lge_panel, struct dsi_panel, lge);
	if (!panel) {
		pr_err("fail to get dsi_panel object\n");
		return;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_bc_dim_set) {
		panel->lge.ddic_ops->lge_bc_dim_set(panel, BC_DIM_ON, BC_DIM_FRAMES_NORMAL);
		pr_info("Set BC_DIM_FRAMES_NORMAL\n");
	} else
		pr_err("no lge_bc_dim_set function\n");

	return;
}

void lge_bc_dim_work_init(struct dsi_panel *panel)
{
	pr_info("Init bc dim work\n");
	INIT_DELAYED_WORK(&panel->lge.bc_dim_work, lge_mdss_dsi_bc_dim_work);
}

static ssize_t therm_dim_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input;

	panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_set_therm_dim)
		panel->lge.ddic_ops->lge_set_therm_dim(panel, input);
	else
		pr_err("Can not find lge_set_therm_dim\n");

	return ret;
}

static DEVICE_ATTR(therm_dim, S_IWUSR | S_IWGRP,
				          NULL, therm_dim_set);

static ssize_t brightness_dim_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int len = 0;
	int bc_dim_f_cnt = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return len;
	}
	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_get_brightness_dim)
		bc_dim_f_cnt = panel->lge.ddic_ops->lge_get_brightness_dim(panel);

	return sprintf(buf, "%d\n", bc_dim_f_cnt);
}

static ssize_t brightness_dim_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input;

	panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_set_brightness_dim)
		panel->lge.ddic_ops->lge_set_brightness_dim(panel, input);

	return ret;
}
static DEVICE_ATTR(brightness_dim, S_IRUGO | S_IWUSR | S_IWGRP,
					brightness_dim_get, brightness_dim_set);

static ssize_t vr_lp_get (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int len = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return len;
	}
	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}
	return sprintf(buf, "%d\n", panel->lge.vr_lp_mode);
}

static ssize_t vr_lp_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int enable;

	panel = dev_get_drvdata(dev);
	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &enable);

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_vr_lp_mode_set)
		panel->lge.ddic_ops->lge_vr_lp_mode_set(panel, enable);
	else
		pr_info("Not support\n");

	return ret;
}
static DEVICE_ATTR(vr_low_persist,  S_IRUGO | S_IWUSR | S_IWGRP,
					vr_lp_get, vr_lp_set);

static ssize_t true_view_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int len = 0;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return len;
	}
	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", panel->lge.true_view_mode);
}

static ssize_t true_view_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int input_param[2];

	panel = dev_get_drvdata(dev);

	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d %d", &input_param[0], &input_param[1]);

	if (panel->lge.true_view_mode != input_param[0]) {
		panel->lge.true_view_mode = input_param[0];
		pr_info("set mode=%d\n", panel->lge.true_view_mode);
	} else {
		pr_info("skip: same request idx=%d\n", panel->lge.true_view_mode);
		return ret;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_set_true_view_mode)
		panel->lge.ddic_ops->lge_set_true_view_mode(panel, true);
	else
		pr_err("can not find lge_true_view_mode\n");

	return ret;
}
static DEVICE_ATTR(true_view, S_IRUGO | S_IWUSR | S_IWGRP, true_view_get, true_view_set);

static ssize_t damping_mode_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;

	panel = dev_get_drvdata(dev);
	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", panel->lge.damping_hdr);
}
static ssize_t damping_mode_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;
	int enable;

	panel = dev_get_drvdata(dev);
	if (panel == NULL) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &enable);

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_damping_mode_set)
		panel->lge.ddic_ops->lge_damping_mode_set(panel, enable);
	else
		pr_info("Not support\n");

	return ret;
}
static DEVICE_ATTR(damping_mode, S_IRUGO | S_IWUSR | S_IWGRP, damping_mode_get, damping_mode_set);

int lge_color_manager_create_sysfs(struct dsi_panel *panel, struct device *panel_sysfs_dev)
{
	int rc = 0;
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_daylight_mode)) < 0)
		pr_err("add daylight_mode set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_hdr_mode)) < 0)
		pr_err("add hdr_mode node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_sharpness)) < 0)
		pr_err("add sharpness set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_screen_mode)) < 0)
		pr_err("add screen_mode set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_screen_tune)) < 0)
		pr_err("add screen_tune set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_rgb_tune)) < 0)
		pr_err("add rgb_tune set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_color_manager_status)) < 0)
		pr_err("add color_manager_status set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_color_manager_mode)) < 0)
		pr_err("add color_manager_mode set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_hdr_hbm_lut)) < 0)
		pr_err("add hdr_hbm_lut set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_acl_mode)) < 0)
		pr_err("add acl_mode set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_video_enhancement)) < 0)
		pr_err("add video enhancement_mode set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_therm_dim)) < 0)
		pr_err("add therm_dim set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_brightness_dim)) < 0)
		pr_err("add brightness dim node fail!\n");

	if (panel->lge.use_vr_lp_mode) {
		if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_vr_low_persist)) < 0)
			pr_err("add vr lp_mode node fail!\n");
	}

	if (panel->lge.true_view_supported) {
		if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_true_view)) < 0)
			pr_err("add true_view node fail!\n");
	}

	if (panel->lge.use_damping_mode) {
		if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_damping_mode)) < 0)
			pr_err("add damping_mode node fail!\n");
	}

	return rc;
}

int mdss_dsi_parse_color_manager_modes(struct device_node *np,
		struct lge_dsi_color_manager_mode_entry color_manager_table[NUM_COLOR_MODES],
		u32 *color_manager_modes_num,
		const char *name)
{
	int num = 0;
	int i, j;
	int rc;
	struct property *data;
	u32 tmp[NUM_COLOR_MODES];
	*color_manager_modes_num = 0;
	data = of_find_property(np, name, &num);
	pr_err("num =%d\n",num);
	num /= sizeof(u32);
	if (!data || !num || num > NUM_COLOR_MODES || num % 2) {
		pr_err("error reading %s, length found = %d\n", name, num);
	} else {
		rc = of_property_read_u32_array(np, name, tmp, num);
		if (rc)
			pr_err("error reading %s, rc = %d\n", name, rc);
		else {
			for (i = 0, j = 0; i < num/2; i++, j++) {
				color_manager_table[i].color_manager_mode = tmp[j];
				color_manager_table[i].color_manager_status = tmp[++j];
				pr_info("index = %d, color_manager_mode = %d, color_manager_status = %d\n", i, color_manager_table[i].color_manager_mode, color_manager_table[i].color_manager_status);
			}
			*color_manager_modes_num = num/2;
		}
	}
	return 0;
}
