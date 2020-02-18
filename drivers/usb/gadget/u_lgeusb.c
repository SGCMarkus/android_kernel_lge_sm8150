/* linux/drivers/usb/gadget/u_lgeusb.c
 *
 * Copyright (C) 2011,2012 LG Electronics Inc.
 * Author : Hyeon H. Park <hyunhui.park@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include "u_lgeusb.h"

#ifdef CONFIG_MACH_LGE
#include <soc/qcom/lge/board_lge.h>
#endif

#ifdef CONFIG_LGE_USB_GADGET_AUTORUN
static char model_string[32];
static char swver_string[32];
static char subver_string[32];
static char phoneid_string[32];
#endif

#define LGEUSB_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
	       char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE, "%s", buffer);			\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	if (size >= sizeof(buffer))					\
	return -EINVAL;							\
	if (sscanf(buf, "%31s", buffer) == 1) {				\
		return size;						\
	}								\
	return -ENODEV;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#ifdef CONFIG_LGE_USB_GADGET_AUTORUN
LGEUSB_STRING_ATTR(model_name, model_string)
LGEUSB_STRING_ATTR(sw_version, swver_string)
LGEUSB_STRING_ATTR(sub_version, subver_string)
LGEUSB_STRING_ATTR(phone_id, phoneid_string)
#endif

static struct device_attribute *lgeusb_attributes[] = {
#ifdef CONFIG_LGE_USB_GADGET_AUTORUN
	&dev_attr_model_name,
	&dev_attr_sw_version,
	&dev_attr_sub_version,
	&dev_attr_phone_id,
#endif
	NULL
};

#ifdef CONFIG_LGE_USB_GADGET_AUTORUN
int lgeusb_get_model_name(char *model, size_t size)
{
	if (!model)
		return -EINVAL;

	strlcpy(model, model_string, size);
	pr_info("lgeusb: model name %s\n", model);
	return 0;
}

int lgeusb_get_phone_id(char *phoneid, size_t size)
{
	if (!phoneid)
		return -EINVAL;

	strlcpy(phoneid, phoneid_string, size);
	pr_info("lgeusb: phoneid %s\n", phoneid);
	return 0;
}

int lgeusb_get_sw_ver(char *sw_ver, size_t size)
{
	if (!sw_ver)
		return -EINVAL;

	strlcpy(sw_ver, swver_string, size);
	pr_info("lgeusb: sw version %s\n", sw_ver);
	return 0;
}

int lgeusb_get_sub_ver(char *sub_ver, size_t size)
{
	if (!sub_ver)
		return -EINVAL;

	strlcpy(sub_ver, subver_string, size);
	pr_info("lgeusb: sw sub version %s\n", sub_ver);
	return 0;
}
#endif

static int lgeusb_probe(struct platform_device *pdev)
{
	struct device_attribute **attrs = lgeusb_attributes;
	struct device_attribute *attr;
	int ret;

	while ((attr = *attrs++)) {
		ret = device_create_file(&pdev->dev, attr);
		if (ret) {
			pr_err("lgeusb: error on creating device file %s\n",
			       attr->attr.name);
			return ret;
		}
	}
	return 0;
}

static struct platform_driver lgeusb_driver = {
	.probe          = lgeusb_probe,
	.driver         = {
		.name = "lge_android_usb",
		.owner  = THIS_MODULE,
	},
};

static struct platform_device lgeusb_device = {
	.name = "lge_android_usb",
	.id = -1,
};

static int __init lgeusb_init(void)
{
	int rc;

	rc = platform_device_register(&lgeusb_device);
	if (rc) {
		pr_err("%s: platform_device_register fail\n", __func__);
		return rc;
	}

	rc = platform_driver_register(&lgeusb_driver);
	if (rc) {
		pr_err("%s: platform_driver_register fail\n", __func__);
		platform_device_unregister(&lgeusb_device);
		return rc;
	}

	return 0;
}
module_init(lgeusb_init);
