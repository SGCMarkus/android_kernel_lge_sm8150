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

#define pr_fmt(fmt)     "[Display][lge-cover-cm:%s:%d] " fmt, __func__, __LINE__

#include <linux/kallsyms.h>
#include "lge_cover_color_manager.h"
#include "lge_dsi_panel_def.h"
#include "lge_dsi_panel.h"
#include "dsi_display.h"
#include <linux/delay.h>

extern struct ice40 *global_ice40;
extern int ice40_mcu_reg_write(struct ice40 *ice40, uint addr, uint val);
extern int ice40_mcu_reg_write_multi(struct ice40 *ice40, uint addr, u8 *val);
extern int global_luke_status;
#define ICE40_SCREEN_MODE 0x0008
#define ICE40_SCREEN_TUNE 0x0009
#define ICE40_RGB_TUNE 0x000A

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
	int input;

	if (!global_luke_status) {
		pr_err("DD is not connected\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);
	pr_info("DD screen_mode (%d)\n", input);

	if (ice40_mcu_reg_write(global_ice40, ICE40_SCREEN_MODE, input) < 0) {
		pr_err("unable to set DD screen mode\n");
		return -EINVAL;
	}

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
	int input[4] = {0,};
	u8 tx_buf[4] = {0,};

	if (!global_luke_status) {
		pr_err("DD is not connected\n");
		return -EINVAL;
	}

	sscanf(buf, "%d %d %d %d", &input[0], &input[1], &input[2], &input[3]);

	tx_buf[0] = (u8)input[0];
	tx_buf[1] = (u8)input[1];
	tx_buf[2] = (u8)input[2];
	tx_buf[3] = (u8)input[3];

	pr_info("DD SAT: %x, HUE: %x, SHA: %x, CF: %x\n",
			tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);

	if (ice40_mcu_reg_write_multi(global_ice40, ICE40_SCREEN_TUNE, tx_buf) < 0) {
		pr_err("unable to set DD rgb tune\n");
		return -EINVAL;
	}

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
	int input[4] = {0,};
	u8 tx_buf[4] = {0,};

	if (!global_luke_status) {
		pr_err("DD is not connected\n");
		return -EINVAL;
	}

	sscanf(buf, "%d %d %d %d", &input[0], &input[1], &input[2], &input[3]);

	tx_buf[0] = (u8)input[0];
	tx_buf[1] = (u8)input[1];
	tx_buf[2] = (u8)input[2];
	tx_buf[3] = (u8)input[3];

	pr_info("DD preset: %d, red: %d, green: %d, blue: %d\n",
			tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);

	if (ice40_mcu_reg_write_multi(global_ice40, ICE40_RGB_TUNE, tx_buf) < 0) {
		pr_err("unable to set DD rgb tune\n");
		return -EINVAL;
	}

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

int lge_cover_color_manager_create_sysfs(struct device *panel_sysfs_dev)
{
	int rc = 0;
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
	return rc;
}

