#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/leds.h>

#include <soc/qcom/lge/board_lge.h>
#include "leds.h"
#include "led-pattern.h"

#define LED_PATTERN_CLASS	"lg_rgb_led"
#define LED_PATTERN_DEVICE	"use_patterns"
#define TUNING_LUT_SCALE_DEFAULT    255
#define TUNING_LUT_SCALE_HIGH       230

int qpnp_brightness_scale = TUNING_LUT_SCALE_DEFAULT;

static struct class*            led_pattern_class      = NULL;
static struct led_pattern_ops*  led_pattern_operations = NULL;

/*
static void qpnp_set_scale(void)
{
	int revid, sub_revid;

	revid = lge_get_board_rev_no();
	sub_revid = lge_get_board_subrev_no();

	if (revid >= 12) {
		if (sub_revid == HW_SUB_REV_1)
			qpnp_brightness_scale = TUNING_LUT_SCALE_HIGH;
		else
			qpnp_brightness_scale = TUNING_LUT_SCALE_DEFAULT;
	} else
		qpnp_brightness_scale = TUNING_LUT_SCALE_HIGH;

	pr_err("%s: H/W rev[%d:%d] set LUT scale to %d\n",
			__func__, revid, sub_revid, qpnp_brightness_scale);
}*/

static ssize_t led_pattern_select(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if( lge_get_boot_mode() <= LGE_BOOT_MODE_CHARGERLOGO )
		if( led_pattern_operations && led_pattern_operations->select )
			led_pattern_operations->select(buf, size);
	return size;
}
#ifdef CONFIG_MACH_SM8150_FLASH
extern int led_completed;
static ssize_t led_pattern_select_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", !!led_completed);
	return ret;
}
static DEVICE_ATTR(setting, 0200, led_pattern_select_show, led_pattern_select);
#else
static DEVICE_ATTR(setting, 0200, NULL, led_pattern_select);
#endif

static ssize_t led_pattern_input(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if( led_pattern_operations && led_pattern_operations->input )
		led_pattern_operations->input(buf, size);
	return size;
}
static DEVICE_ATTR(input_patterns, 0200, NULL, led_pattern_input);

static ssize_t led_pattern_blink(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if( led_pattern_operations && led_pattern_operations->blink )
		led_pattern_operations->blink(buf, size);
	return size;
}
static DEVICE_ATTR(blink_patterns, 0200, NULL, led_pattern_blink);

static ssize_t led_pattern_onoff(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if( led_pattern_operations && led_pattern_operations->onoff )
		led_pattern_operations->onoff(buf, size);
	return size;
}
static DEVICE_ATTR(onoff_patterns, 0200, NULL, led_pattern_onoff);

static ssize_t led_pattern_scale(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if( led_pattern_operations && led_pattern_operations->scale )
		led_pattern_operations->scale(buf, size);
	return size;
}
static DEVICE_ATTR(scale, 0200, NULL, led_pattern_scale);

void led_pattern_register(struct led_pattern_ops *led_platform_operations)
{
	led_pattern_operations = led_platform_operations;
}

static int __init leds_pattern_init(void)
{
	struct device*	led_pattern_device;

	led_pattern_class = class_create(THIS_MODULE, LED_PATTERN_CLASS);
	if (IS_ERR(led_pattern_class)) {
		printk("Failed to create class(lg_rgb_led)!\n");
	}

	led_pattern_device = device_create(led_pattern_class, NULL, 0, NULL, LED_PATTERN_DEVICE);
	if (IS_ERR(led_pattern_device))
		return PTR_ERR(led_pattern_device);

	if (device_create_file(led_pattern_device, &dev_attr_setting) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_setting.attr.name);

	if (device_create_file(led_pattern_device, &dev_attr_blink_patterns) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_blink_patterns.attr.name);

	if (device_create_file(led_pattern_device, &dev_attr_input_patterns) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_input_patterns.attr.name);

	if (device_create_file(led_pattern_device, &dev_attr_onoff_patterns) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_onoff_patterns.attr.name);

	if (device_create_file(led_pattern_device, &dev_attr_scale) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_scale.attr.name);
//	qpnp_set_scale();

	return 0;
}

static void __exit leds_pattern_exit(void)
{
	class_destroy(led_pattern_class);
}


subsys_initcall(leds_pattern_init);
module_exit(leds_pattern_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LG LED-Pattern Class Interface");
