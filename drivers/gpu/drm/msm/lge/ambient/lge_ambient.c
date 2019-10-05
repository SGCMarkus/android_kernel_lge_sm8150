#define pr_fmt(fmt)	"[Display][lge-ambient:%s:%d] " fmt, __func__, __LINE__

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <video/mipi_display.h>

#include "dsi_panel.h"
#include "dsi_ctrl_hw.h"

#define SANITIZE(x) do { (x) = ((x)<0)?0:(x); } while(0)

enum {
	PA_NOT_DEFINED = 0,
	PA_PARTIAL= 1, /* string : partial */
	PA_FULL = 2, /* string : full-size */
	PA_SEMI_PARTIAL = 4, /* string : semi-partial */
	PA_MAX,
};

static void set_interface_data(struct dsi_panel *panel, int *data)
{
	panel->lge.aod_interface = ((*(data) << 8) | (*(data + 1) << 4) | *(data + 2));
	pr_info("mode0=%d, mode1=%d, mode2=%d, aod_interface=0x%03x\n",
					*data, *(data + 1), *(data + 2),
					panel->lge.aod_interface);
}

int lge_ambient_set_interface_data(struct dsi_panel *panel)
{
	int i;
	int interface_data[3] = {PA_FULL, PA_FULL, PA_FULL};

	if (!panel) {
		pr_err("panel is null\n");
		return -EINVAL;
	}

	for (i = 0; i < 3; i++) {
		if (!panel->lge.aod_interface_type[i]) {
			pr_err("not defined from idx=%d\n", i);
			break;
		}

		if(!strncmp(panel->lge.aod_interface_type[i], "partial", 4)) {
			interface_data[i] = PA_PARTIAL;
		} else if(!strncmp(panel->lge.aod_interface_type[i], "semi", 4)) {
			interface_data[i] = PA_SEMI_PARTIAL;
		} else {
			interface_data[i] = PA_FULL;
		}
	}

	set_interface_data(panel, &(interface_data[0]));
	return 0;
}
EXPORT_SYMBOL(lge_ambient_set_interface_data);

static ssize_t area_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel = NULL;
	struct lge_rect rect = {0,};

	panel = dev_get_drvdata(dev);
	if (panel) {
		rect = panel->lge.aod_area;
	}

	return sprintf(buf, "%d %d %d %d\n", rect.x, rect.y, rect.w, rect.h);
}

static ssize_t area_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int tmp;
	struct dsi_panel *panel = NULL;
	struct lge_rect rect;

	panel = dev_get_drvdata(dev);
	if (panel == NULL)
		return ret;

	sscanf(buf, "%d %d %d %d %d", &tmp,
		&rect.x,
		&rect.y,
		&rect.w,
		&rect.h);
	SANITIZE(rect.x);
	SANITIZE(rect.y);
	SANITIZE(rect.w);
	SANITIZE(rect.h);

	pr_info("lpmode:%d, x:%d, y:%d, w:%d, h:%d\n", tmp,
		rect.x,
		rect.y,
		rect.w,
		rect.h);

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->store_aod_area) {
		mutex_lock(&panel->lge.pa_changed_lock);
		panel->lge.ddic_ops->store_aod_area(panel, rect);
		if (panel->lge.wait_pa_changed) {
			complete(&panel->lge.pa_changed_done);
			panel->lge.wait_pa_changed = false;
		}
		mutex_unlock(&panel->lge.pa_changed_lock);
	}

	return ret;
}
static DEVICE_ATTR(area, S_IRUGO | S_IWUSR | S_IWGRP,
		area_get, area_set);

static ssize_t power_mode_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel = NULL;
	int power_mode = -1;

	panel = dev_get_drvdata(dev);
	if (panel) {
		power_mode = panel->lge.aod_power_mode;
	}

	return sprintf(buf, "%d\n", power_mode);
}

static ssize_t power_mode_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int power_mode;
	struct dsi_panel *panel = NULL;

	panel = dev_get_drvdata(dev);
	if (panel == NULL)
		return ret;

	sscanf(buf, "%d", &power_mode);
	panel->lge.aod_power_mode = power_mode;

	pr_info("power_mode=%d\n", panel->lge.aod_power_mode);

	return ret;
}
static DEVICE_ATTR(power_mode, S_IRUGO | S_IWUSR | S_IWGRP,
		power_mode_get, power_mode_set);

static ssize_t aod_interface_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel = NULL;
	u32 data = 0;

	panel = dev_get_drvdata(dev);
	if (panel) {
		data = panel->lge.aod_interface;
		pr_info("aod_interface data = %d\n", data);
	}

	return sprintf(buf, "%d\n", data);
}
static DEVICE_ATTR(aod_interface, S_IRUGO,
		aod_interface_get, NULL);

void lge_ambient_create_sysfs(struct dsi_panel *panel, struct class *class_panel)
{
	static struct device *aod_sysfs_dev = NULL;

	if(!aod_sysfs_dev){
		aod_sysfs_dev = device_create(class_panel, NULL, 0, panel, "aod");
		if (IS_ERR(aod_sysfs_dev)) {
			pr_err("Failed to create dev(aod_sysfs_dev)!");
		} else {
			if (device_create_file(aod_sysfs_dev, &dev_attr_area) < 0)
				pr_err("add aod area node fail!");
			if (device_create_file(aod_sysfs_dev, &dev_attr_power_mode) < 0)
				pr_err("add aod power mode node fail!");
			if (device_create_file(aod_sysfs_dev, &dev_attr_aod_interface) < 0)
				pr_err("add aod interface node fail!");
		}
	}
}
