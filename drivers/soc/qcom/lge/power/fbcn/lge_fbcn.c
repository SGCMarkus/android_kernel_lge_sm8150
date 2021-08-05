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

#include <linux/kallsyms.h>
#include <linux/sysfs.h>
#include <linux/device.h>

#include "lge_fbcn.h"
#include "lge_intm.h"

static struct fbcn_intv_stats fstats;
static struct device *g_sysfs_dev;

static void fbcn_calc_stats(u32 intv)
{
	if (intv == BOOST_INTV)
		fstats.intv = MIN_INTV;
	else
		fstats.intv = (u64)intv;
}

static void lge_fbcn_intv_notify(u32 intv)
{
	mutex_lock(&fstats.fbcn_lock);

	fbcn_calc_stats(intv);
	if (fstats.i_upper != ULONG_MAX
	    &&(fstats.intv > fstats.i_upper
	    || fstats.intv <= fstats.i_lower)) {
		mutex_unlock(&fstats.fbcn_lock);
		if (g_sysfs_dev)
			sysfs_notify(&g_sysfs_dev->kobj, NULL, "fbcn_intv");
		else
			pr_err(PRM_TAG "Fb sysfs device is null\n");
	} else {
		mutex_unlock(&fstats.fbcn_lock);
	}

	return;
}

static int lge_fbcn_callback_handler(struct notifier_block *nb,
				     unsigned long event, void *data)
{
	int ret = 0;
	u32 *val = data;

	switch (event) {
	case INTV_EVENT_NOTIFY:
		lge_fbcn_intv_notify(*val);
		break;
	case INTV_EVENT_BLOCK:
		break;
	default:
		break;
	};

	return ret;
}

ssize_t lge_fbcn_en_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	int enable;

	if (sscanf(buf, "%d", &enable) < 1) {
		pr_err(PRM_TAG "Failed to store enable\n");
		return -EINVAL;
	}

	mutex_lock(&fstats.fbcn_lock);
	if (enable != fstats.enable) {
		if (enable) {
			fstats.intv = MIN_INTV;
			fstats.i_upper = ULONG_MAX;
			fstats.i_lower = 0;
			fstats.enable = 1;
		} else {
			fstats.enable = 0;
		}
		mutex_unlock(&fstats.fbcn_lock);
		lge_intv_enable(fstats.enable);
		lge_intv_input_notify();
	} else {
		mutex_unlock(&fstats.fbcn_lock);
	}

	return count;
}

ssize_t lge_fbcn_en_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%u\n", fstats.enable);
	return ret;
}

ssize_t lge_fbcn_i_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	u64 i_upper, i_lower;

	if (sscanf(buf, "%llu %llu", &i_upper, &i_lower) < 1) {
		pr_err(PRM_TAG "Failed to store fbcn intv threshold\n");
		return -EINVAL;
	}
	mutex_lock(&fstats.fbcn_lock);
	fstats.i_upper = i_upper;
	fstats.i_lower = i_lower;
	mutex_unlock(&fstats.fbcn_lock);
	return count;
}

ssize_t lge_fbcn_i_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%llu %llu\n", fstats.i_upper, fstats.i_lower);
	return ret;
}

ssize_t lge_fbcn_intv_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%llu\n", fstats.intv);
	return ret;
}

static DEVICE_ATTR(fbcn_en, S_IRUGO | S_IWUSR, lge_fbcn_en_show, lge_fbcn_en_store);
static DEVICE_ATTR(fbcn_i, S_IRUGO | S_IWUSR, lge_fbcn_i_show, lge_fbcn_i_store);
static DEVICE_ATTR(fbcn_intv, S_IRUGO | S_IWUSR, lge_fbcn_intv_show, NULL);

static struct attribute *fbcn_fs_attrs[] = {
	&dev_attr_fbcn_en.attr,
	&dev_attr_fbcn_i.attr,
	&dev_attr_fbcn_intv.attr,
	NULL,
};

static struct attribute_group fbcn_fs_attrs_group = {
	.attrs = fbcn_fs_attrs,
};

int lge_fbcn_sysfs_create(void) {
	int ret = 0;
	struct class *cn = NULL;
	static struct device *sysfs_dev = NULL;

	ret = lge_prm_get_class_node(&cn);
	if (ret)
		return -EPROBE_DEFER;

	if (!sysfs_dev) {
		sysfs_dev = device_create(cn, NULL, 0, NULL, "fbcn");
		if (IS_ERR_OR_NULL(sysfs_dev)) {
			pr_err(PRM_TAG "Failed to create fbcn sysfs_dev\n");
			return -ENODEV;
		}

		ret = sysfs_create_group(&sysfs_dev->kobj, &fbcn_fs_attrs_group);
		if (ret) {
			pr_err(PRM_TAG "Failed to create fbcn sub_sysfs\n");
			return -ENODEV;
		}
	} else {
		pr_err(PRM_TAG "fbcn sysfs_dev already exist\n");
	}

	g_sysfs_dev = sysfs_dev;
	return 0;
}

int lge_fbcn_init(void)
{
	int ret = 0;

	ret = lge_fbcn_sysfs_create();
	if (ret) {
		pr_err(PRM_TAG "Error fbcn sysfs creation, %d\n", ret);
		return ret;
	}

	mutex_init(&fstats.fbcn_lock);

	ret = lge_intv_init();
	if (ret < 0) {
		pr_err(PRM_TAG "Failed to initialize INTM\n");
		return ret;

	}

	fstats.nb.notifier_call = lge_fbcn_callback_handler;
	lge_intv_notifier_register(&fstats.nb);

	pr_err(PRM_TAG "LGE FBCN probe done\n");
	return 0;
}
