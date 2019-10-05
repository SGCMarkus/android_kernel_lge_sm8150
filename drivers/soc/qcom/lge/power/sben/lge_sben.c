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

#include <linux/device.h>
#include <linux/mutex.h>
#include <uapi/drm/sde_drm.h>

#include "lge_sben.h"

static struct device *g_sysfs_dev;

void lge_sben_notify(void)
{
	if (g_sysfs_dev)
		sysfs_notify(&g_sysfs_dev->kobj, NULL, "sben_event");
}

static ssize_t lge_sben_event_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;

	int power_mode = lge_prm_get_info(
			LGE_PRM_INFO_DISPLAY_MAIN_STATE);

	pr_debug(PRM_TAG "Power_mode = %d\n", power_mode);

	ret = scnprintf(buf, PAGE_SIZE, "panel_power_on = %d\n",
			(power_mode != SDE_MODE_DPMS_ON) ?
			LGE_PRM_DISPLAY_OFF : LGE_PRM_DISPLAY_ON);

	return ret;
}
static DEVICE_ATTR(sben_event, S_IRUGO, lge_sben_event_show, NULL);

static struct attribute *sben_fs_attrs[] = {
	&dev_attr_sben_event.attr,
	NULL,
};

static struct attribute_group sben_fs_attrs_group = {
	.attrs = sben_fs_attrs,
};

int lge_sben_sysfs_create(void) {
	int ret = 0;
	struct class *cn = NULL;
	static struct device *sysfs_dev = NULL;

	ret = lge_prm_get_class_node(&cn);
	if (ret)
		return -EPROBE_DEFER;

	if (!sysfs_dev) {
		sysfs_dev = device_create(cn, NULL, 0, NULL, "sben");
		if (IS_ERR_OR_NULL(sysfs_dev)) {
			pr_err(PRM_TAG "Failed to create sben sysfs_dev\n");
			return -ENODEV;
		}

		ret = sysfs_create_group(&sysfs_dev->kobj, &sben_fs_attrs_group);
		if (ret) {
			pr_err(PRM_TAG "Failed to create sben sub_sysfs\n");
			return -ENODEV;
		}
	} else {
		pr_err(PRM_TAG "sben sysfs_dev already exist\n");
	}

	g_sysfs_dev = sysfs_dev;
	return 0;
}

int lge_sben_init(void)
{
	int ret = 0;

	ret = lge_sben_sysfs_create();
	if (ret) {
		pr_err(PRM_TAG "Error sben sysfs creation, %d\n", ret);
		return ret;
	}

	pr_err(PRM_TAG "LGE SBEN probe done\n");
	return 0;
}

