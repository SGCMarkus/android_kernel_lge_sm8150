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
#include <linux/power_supply.h>
#include "lge_dd.h"

static struct device *g_sysfs_dev;

void lge_dd_connect_notify(void)
{
	if (g_sysfs_dev)
		sysfs_notify(&g_sysfs_dev->kobj, NULL, "dd_connect");
}

void lge_dd_state_notify(void)
{
	int ds1_connect = lge_prm_get_info(
			LGE_PRM_INFO_DISPLAY_DD_STATE);
	int ds2_connect = lge_prm_get_info(
			LGE_PRM_INFO_CONNECTION_DS2_STATE);
	int hallic_connect = lge_prm_get_info(
			LGE_PRM_INFO_HALLIC_STATE);
	int ds_state = lge_prm_get_info(
			LGE_PRM_INFO_DS_STATE);
	int now_ds_state = LGE_PRM_DS_CONNECTION_OFF;
	struct power_supply*	psy;

	if (ds1_connect)
		now_ds_state = LGE_PRM_DS1_CONNECTION_ON;
	else if (ds2_connect)
		now_ds_state = LGE_PRM_DS2_CONNECTION_ON;
	else if (hallic_connect)
		now_ds_state = LGE_PRM_HALLIC_CONNECTION_ON;

	if (now_ds_state != ds_state &&
			(ds_state != LGE_PRM_DS1_CONNECTION_ON
			|| now_ds_state != LGE_PRM_HALLIC_CONNECTION_ON)) {
		pr_err(PRM_TAG "ds state is changed from %d to %d.(ds1(%d), ds2(%d), hallic(%d))\n",
			ds_state, now_ds_state, ds1_connect, ds2_connect, hallic_connect);
		lge_prm_display_set_event(LGE_PRM_DISPLAY_EVENT_DS_STATE, now_ds_state);

		psy = power_supply_get_by_name("usb");
		if (psy) {
			power_supply_changed(psy);
			power_supply_put(psy);
		}
		if (g_sysfs_dev)
			sysfs_notify(&g_sysfs_dev->kobj, NULL, "dd_phy_connect");
	}
}

static ssize_t lge_dd_connect_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;

	int dd_connect = lge_prm_get_info(
			LGE_PRM_INFO_DISPLAY_DD_STATE);

	pr_err(PRM_TAG "DD connect = %d\n", dd_connect);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", !!dd_connect);

	return ret;
}
static DEVICE_ATTR(dd_connect, S_IRUGO, lge_dd_connect_show, NULL);

static ssize_t lge_dd_phy_connect_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int dd_phy_connect = lge_prm_get_info(
			LGE_PRM_INFO_DS_STATE);

	if (dd_phy_connect == LGE_PRM_HALLIC_CONNECTION_ON)
		dd_phy_connect = LGE_PRM_DS1_CONNECTION_ON;

	pr_err(PRM_TAG "DD phy connect = %d\n", dd_phy_connect);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", dd_phy_connect);

	return ret;

}
static DEVICE_ATTR(dd_phy_connect, S_IRUGO, lge_dd_phy_connect_show, NULL);

static struct attribute *dd_fs_attrs[] = {
	&dev_attr_dd_connect.attr,
	&dev_attr_dd_phy_connect.attr,
	NULL,
};

static struct attribute_group dd_fs_attrs_group = {
	.attrs = dd_fs_attrs,
};

int lge_dd_sysfs_create(void) {
	int ret = 0;
	struct class *cn = NULL;
	static struct device *sysfs_dev = NULL;

	ret = lge_prm_get_class_node(&cn);
	if (ret)
		return -EPROBE_DEFER;

	if (!sysfs_dev) {
		sysfs_dev = device_create(cn, NULL, 0, NULL, "dd");
		if (IS_ERR_OR_NULL(sysfs_dev)) {
			pr_err(PRM_TAG "Failed to create dd sysfs_dev\n");
			return -ENODEV;
		}

		ret = sysfs_create_group(&sysfs_dev->kobj, &dd_fs_attrs_group);
		if (ret) {
			pr_err(PRM_TAG "Failed to create dd sub_sysfs\n");
			return -ENODEV;
		}
	} else {
		pr_err(PRM_TAG "dd sysfs_dev already exist\n");
	}

	g_sysfs_dev = sysfs_dev;
	return 0;
}

int lge_dd_init(void)
{
	int ret = 0;

	ret = lge_dd_sysfs_create();
	if (ret) {
		pr_err(PRM_TAG "Error dd sysfs creation, %d\n", ret);
		return ret;
	}

	pr_err(PRM_TAG "LGE DD probe done\n");
	return 0;
}

