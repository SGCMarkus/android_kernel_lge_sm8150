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

#include "lge_vfps.h"

static struct {
	char         enable;
	ulong        value;
	ulong        weight;
	ulong        bucket;
	ulong        count;
	int          fps_cnt_before;
	int          ratio;
	bool         first;
	unsigned int intv_min_fps;
	struct drm_encoder *encoder;
} vfps;

static struct device *g_sysfs_dev;

bool lge_vfps_check_internal(void)
{
	if (vfps.enable) {
		vfps.bucket += vfps.weight;
		if (vfps.first == false) {
			vfps.first = true;
			return false;
		} else {
			if (vfps.value <= vfps.bucket) {
				vfps.bucket -= vfps.value;
				return false;
			} else {
				vfps.count++;
				return true;
			}
		}
	} else {
		return false;
	}
}

void lge_vfps_set_encoder(struct drm_encoder *encoder)
{
	pr_err(PRM_TAG "display set encoder to vfps module\n");
	if (!vfps.encoder)
		vfps.encoder = encoder;
}

int lge_vfps_get_encoder(struct drm_encoder **encoder)
{
	if (IS_ERR_OR_NULL(vfps.encoder)) {
		return -ENODEV;
	} else {
		*encoder = vfps.encoder;
		return 0;
	}
}

static ssize_t vfps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int len = 0;

	len = snprintf(buf, PAGE_SIZE, "enable=%d\n"
			"weight=%lu\nvalue=%lu\n"
			"bucket=%lu\ncount=%lu\n",
			vfps.enable,
			vfps.weight,
			vfps.value,
			vfps.bucket,
			vfps.count);
	return len;
}

static ssize_t vfps_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ulong fps;

	if (!size)
		return -EINVAL;

	fps = simple_strtoul(buf, NULL, 10);

	if (fps == 0 || fps >= 60) {
		vfps.enable = 0;
		vfps.value = 0;
		vfps.weight = 0;
		vfps.bucket = 0;
		vfps.count = 0;
		vfps.ratio = 60;
		vfps.first = false;
		pr_debug(PRM_TAG "Disable VFPS\n");
	} else {
		vfps.enable = 1;
		vfps.value = (60 << 16) / fps;
		vfps.weight = (1 << 16);
		vfps.bucket = 0;
		vfps.ratio = fps;
		vfps.first = false;
		pr_debug(PRM_TAG "Enable VFPS %lu\n", fps);
	}

	return size;
}

static ssize_t vfps_ratio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r = 0;

	r = snprintf(buf, PAGE_SIZE, "%d 60\n", vfps.ratio);
	return r;
}

static ssize_t vfps_fcnt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sde_crtc *crtc = NULL;
	u64 count = 0;

	if (vfps.encoder) {
		if (vfps.encoder->crtc) {
			crtc = to_sde_crtc(vfps.encoder->crtc);
			count = crtc->play_count - vfps.fps_cnt_before;
			vfps.fps_cnt_before = crtc->play_count;
		} else {
			goto exit;
		}
	} else {
		goto exit;
	}

	return sprintf(buf, "%llu\n", count);

exit:
	vfps.fps_cnt_before = 0;
	return sprintf(buf, "%llu\n", (u64)0);
}

static DEVICE_ATTR(vfps, 0644, vfps_show, vfps_store);
static DEVICE_ATTR(vfps_ratio, 0444, vfps_ratio_show, NULL);
static DEVICE_ATTR(vfps_fcnt, 0444, vfps_fcnt_show, NULL);

static struct attribute *vfps_fs_attrs[] = {
	&dev_attr_vfps.attr,
	&dev_attr_vfps_ratio.attr,
	&dev_attr_vfps_fcnt.attr,
	NULL,
};

static struct attribute_group vfps_fs_attrs_group = {
	.attrs = vfps_fs_attrs,
};

int lge_vfps_sysfs_create(void) {
	int ret = 0;
	struct class *cn = NULL;
	static struct device *sysfs_dev = NULL;

	ret = lge_prm_get_class_node(&cn);
	if (ret)
		return -EPROBE_DEFER;

	if (!sysfs_dev) {
		sysfs_dev = device_create(cn, NULL, 0, NULL, "vfps");
		if (IS_ERR_OR_NULL(sysfs_dev)) {
			pr_err(PRM_TAG "Failed to create vfps sysfs_dev\n");
			return -ENODEV;
		}

		ret = sysfs_create_group(&sysfs_dev->kobj, &vfps_fs_attrs_group);
		if (ret) {
			pr_err(PRM_TAG "Failed to create vfps sub_sysfs\n");
			return -ENODEV;
		}
	} else {
		pr_err(PRM_TAG "vfps sysfs_dev already exist\n");
	}

	g_sysfs_dev = sysfs_dev;
	return 0;
}

int lge_vfps_init(void)
{
	struct drm_encoder *encoder = NULL;
	int ret = 0;

	ret = lge_vfps_get_encoder(&encoder);
	if (ret) {
		pr_err(PRM_TAG "Get display encoder fail, %d\n", ret);
		return -EPROBE_DEFER;
	}

	pr_err(PRM_TAG "Encoder name = %s\n", encoder->name);

	ret = lge_vfps_sysfs_create();
	if (ret) {
		pr_err(PRM_TAG "Error vfps sysfs creation, %d\n", ret);
		return ret;
	}

	vfps.encoder = encoder;

	pr_err(PRM_TAG "LGE VFPS probe done\n");
	return 0;
}

