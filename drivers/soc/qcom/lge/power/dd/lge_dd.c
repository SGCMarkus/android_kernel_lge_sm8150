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
#ifdef CONFIG_LGE_COVER_DISPLAY
#include <linux/lge_panel_notify.h>
#include <soc/qcom/lge/board_lge.h>
#endif

#include "lge_dd.h"

static struct device *g_sysfs_dev;
#ifdef CONFIG_LGE_COVER_DISPLAY
static int dd_phy_connect;
#endif

void lge_dd_connect_notify(void)
{
	if (g_sysfs_dev)
		sysfs_notify(&g_sysfs_dev->kobj, NULL, "dd_connect");
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

#ifdef CONFIG_LGE_COVER_DISPLAY
void lge_dd_phy_connect_notify(void)
{
	if (g_sysfs_dev)
		sysfs_notify(&g_sysfs_dev->kobj, NULL, "dd_phy_connect");
}

static ssize_t lge_dd_phy_connect_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", !!dd_phy_connect);

	return ret;
}
static DEVICE_ATTR(dd_phy_connect, S_IRUGO, lge_dd_phy_connect_show, NULL);
#endif

static struct attribute *dd_fs_attrs[] = {
	&dev_attr_dd_connect.attr,
#ifdef CONFIG_LGE_COVER_DISPLAY
	&dev_attr_dd_phy_connect.attr,
#endif
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

#ifdef CONFIG_LGE_COVER_DISPLAY
static int lge_prm_dd_state_callback(struct notifier_block *this,
			   unsigned long event, void *data)
{
	struct lge_panel_notifier *evdata = (struct lge_panel_notifier *)data;

	if (event != LGE_PANEL_EVENT_DUAL_DISPLAY)
		return 0;

	if (evdata && event == LGE_PANEL_EVENT_DUAL_DISPLAY) {
		switch (evdata->state) {
			case LGE_PANEL_DD_CONNECTED:
				dd_phy_connect = 1;
				break;
			case LGE_PANEL_DD_NOT_CONNECTED:
				dd_phy_connect = 0;
				break;
			default:
				break;
		}

		pr_err(PRM_TAG "DD phy connect = %d\n", dd_phy_connect);
		lge_dd_phy_connect_notify();
	}
	return NOTIFY_OK;
}

static struct notifier_block lge_prm_dd_noti_block = {
	.notifier_call = lge_prm_dd_state_callback,
};

int lge_dd_notifer_create(void) {
	if (lge_panel_notifier_register_client(&lge_prm_dd_noti_block)) {
		pr_err(PRM_TAG "Unable to register lge_prm_dd_noti_block notifier\n");
		return -ENODEV;
	}

	return 0;
}
#endif

int lge_dd_init(void)
{
	int ret = 0;

	ret = lge_dd_sysfs_create();
	if (ret) {
		pr_err(PRM_TAG "Error dd sysfs creation, %d\n", ret);
		return ret;
	}
#ifdef CONFIG_LGE_COVER_DISPLAY
	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		ret = lge_dd_notifer_create();
		if (ret) {
			pr_err(PRM_TAG "Error dd notifier register, %d\n", ret);
			return ret;
		}
	}
#endif
	pr_err(PRM_TAG "LGE DD probe done\n");
	return 0;
}

