#define pr_fmt(fmt)	"[Display][lge-ambient:%s:%d] " fmt, __func__, __LINE__

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <video/mipi_display.h>

#include "dsi_panel.h"
#include "lge_factory.h"

static ssize_t check_vert_black_line(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_check_vert_black_line)
		panel->lge.ddic_ops->lge_check_vert_black_line(panel);
	else
		pr_err("Can not find lge_check_vert_white_line");

	return ret;
}
static DEVICE_ATTR(check_black_line, S_IWUSR | S_IWGRP, NULL, check_vert_black_line);

static ssize_t check_vert_white_line(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_check_vert_white_line)
		panel->lge.ddic_ops->lge_check_vert_white_line(panel);
	else
		pr_err("Can not find lge_check_vert_white_line");

	return ret;
}
static DEVICE_ATTR(check_white_line, S_IWUSR | S_IWGRP, NULL, check_vert_white_line);

static ssize_t check_vert_line_restore(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	struct dsi_panel *panel;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->lge_check_vert_line_restore)
		panel->lge.ddic_ops->lge_check_vert_line_restore(panel);
	else
		pr_err("Can not find lge_check_vert_white_line");

	return ret;
}
static DEVICE_ATTR(check_line_restore, S_IWUSR | S_IWGRP, NULL, check_vert_line_restore);

void lge_panel_line_detect_create_sysfs(struct device *panel_sysfs_dev)
{
	int rc = 0;

	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_check_black_line)) < 0)
		pr_err("add check_black_line node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_check_white_line)) < 0)
		pr_err("add check_white_line node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_check_line_restore)) < 0)
		pr_err("add check_line_restore node fail!");
}
