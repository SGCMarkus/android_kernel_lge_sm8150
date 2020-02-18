/*
 * Copyright(c) 2017, LG Electronics. All rights reserved.
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

#define pr_fmt(fmt)      "[Display][lge-err-detect:%s:%d] " fmt, __func__, __LINE__

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "lge_err_detect.h"
#include "../../../../../../kernel/irq/internals.h"

void lge_panel_err_detect_remove(struct dsi_panel *panel)
{
	int irq_gpio, irq;

	if (panel == NULL) {
		pr_err("panel is NULL\n");
		return;
	}

	irq_gpio = panel->lge.err_detect_gpio;
	irq = gpio_to_irq(irq_gpio);

	if (panel->lge.err_detect_irq_enabled)
		disable_irq(irq);
	if (irq > 0)
		free_irq(irq, panel);
	if (panel->lge.err_detect_int_workq)
		destroy_workqueue(panel->lge.err_detect_int_workq);
	if (irq_gpio > 0 && gpio_is_valid(irq_gpio))
		gpio_free(irq_gpio);
}

#define istate core_internal_state__do_not_mess_with_it
void lge_panel_err_detect_irq_control(struct dsi_panel *panel, bool enable)
{
	int irq_gpio, irq;
	struct irq_desc *desc = NULL;

	if (panel == NULL) {
		pr_err("panel is NULL\n");
		return;
	}

	irq_gpio = panel->lge.err_detect_gpio;
	irq = gpio_to_irq(irq_gpio);
	desc = irq_to_desc(irq);

	if (enable) {
		if (desc) {
			if (desc->istate & IRQS_PENDING) {
				pr_info("Remove pending irq(%d)\n", irq);
				desc->istate &= ~(IRQS_PENDING);
			}
		}
		panel->lge.err_detect_result = 0;
		enable_irq(irq);
	}
	else {
		disable_irq(irq);
	}
	panel->lge.err_detect_irq_enabled = enable;
	pr_info("enable = %d\n", enable);
}

void lge_panel_err_detect_parse_dt(struct dsi_panel *panel, struct device_node *of_node)
{
	panel->lge.err_detect_gpio = of_get_named_gpio(of_node,	"lge,err-detect-gpio", 0);
}

static ssize_t set_err_mask(struct device *dev,
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

	mutex_lock(&panel->panel_lock);
	if (!dsi_panel_initialized(panel)) {
		pr_err("Panel off state. Ignore screen_mode set cmd\n");
		mutex_unlock(&panel->panel_lock);
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);
	panel->lge.err_detect_mask = input;

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->set_err_detect_mask) {
		if (panel->lge.ddic_ops->set_err_detect_mask(panel))
			pr_err("err detect mask %d set failed\n", panel->lge.err_detect_mask);
	}
	mutex_unlock(&panel->panel_lock);

	pr_info("err detect mask %d\n", panel->lge.err_detect_mask);
	return ret;
}

static ssize_t get_err_mask(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int len = 0;

	panel = dev_get_drvdata(dev);

	if (!panel) {
		pr_err("panel is NULL\n");
		return len;
	}

	pr_info("%d\n", panel->lge.err_detect_mask);
	return sprintf(buf, "%d\n", panel->lge.err_detect_mask);
}

static ssize_t set_err_crash(struct device *dev,
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

	sscanf(buf, "%d", &input);
	panel->lge.err_detect_crash_enabled = input;

	pr_info("set err detect crash enable =  %d\n", panel->lge.err_detect_crash_enabled);
	return ret;
}

static ssize_t get_err_crash(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int len = 0;

	panel = dev_get_drvdata(dev);

	if (!panel) {
		pr_err("panel is NULL\n");
		return len;
	}

	pr_info("%d\n", panel->lge.err_detect_crash_enabled);
	return sprintf(buf, "%d\n", panel->lge.err_detect_crash_enabled);
}

static ssize_t get_mem_test(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;
	int len = 0;

	panel = dev_get_drvdata(dev);

	if (!panel) {
		pr_err("panel is NULL\n");
		return len;
	}

	pr_info("0x%x\n", panel->lge.err_detect_result);
	return sprintf(buf, "%d\n", panel->lge.err_detect_result);
}

static DEVICE_ATTR(err_mask, S_IRUGO | S_IWUSR | S_IWGRP,
					get_err_mask, set_err_mask);
static DEVICE_ATTR(err_crash, S_IRUGO | S_IWUSR | S_IWGRP,
					get_err_crash, set_err_crash);
static DEVICE_ATTR(mem_test, S_IRUGO, get_mem_test, NULL);

int lge_panel_err_detect_create_sysfs(struct dsi_panel *panel, struct class *class_panel)
{
	int rc = 0;
	static struct device *error_detect_sysfs_dev = NULL;

	if (!panel && !class_panel) {
		pr_err("Invalid input\n");
		return -EINVAL;;
	}

	if (!error_detect_sysfs_dev) {
		error_detect_sysfs_dev = device_create(class_panel, NULL, 0, panel, "error_detect");
		if (IS_ERR(error_detect_sysfs_dev)) {
			pr_err("Failed to create dev(aod_sysfs_dev)!");
		} else {
			if ((rc = device_create_file(error_detect_sysfs_dev, &dev_attr_err_mask)) < 0)
				pr_err("add error mask node fail!");
			if ((rc = device_create_file(error_detect_sysfs_dev, &dev_attr_mem_test)) < 0)
				pr_err("add mem test node fail!");
			if ((rc = device_create_file(error_detect_sysfs_dev, &dev_attr_err_crash)) < 0)
				pr_err("add error crash node fail!");
		}
	}
	return rc;
}

void lge_panel_err_detect_init(struct dsi_panel *panel)
{
	int ret = 0;
	int irq = -ENXIO;
	int irq_gpio = 0;

	if (panel == NULL) {
		pr_err("invalid input\n");
		return;
	}

	irq_gpio = panel->lge.err_detect_gpio;

	if (irq_gpio > 0 && gpio_is_valid(irq_gpio)) {
		ret = gpio_request(irq_gpio, "ddic_err_irq_gpio");
		if (ret) {
			pr_err("unable to request gpio [%d] ret=%d\n", irq_gpio, ret);
			goto err_none;
		}
		ret = gpio_direction_input(irq_gpio);
		if (ret) {
			pr_err("unable to set dir for gpio[%d]\n", irq_gpio);
			goto err_irq_gpio;
		}
	} else {
		pr_err("irq gpio not provided\n");
		goto err_none;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->err_detect_work) {
		panel->lge.err_detect_int_workq = create_workqueue("lge_panel_err_detect_int");
		if (!panel->lge.err_detect_int_workq) {
			pr_warn(" Warning creating workqueue\n");
			goto err_none;
		} else {
			pr_info("register err_detect_work\n");
			INIT_DELAYED_WORK(&panel->lge.err_detect_int_work, panel->lge.ddic_ops->err_detect_work);
		}
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->err_detect_irq_handler) {
		irq = gpio_to_irq(irq_gpio);
		ret = request_threaded_irq(irq, NULL, panel->lge.ddic_ops->err_detect_irq_handler,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   "panel-err-detect-irq", panel);
		if (ret) {
			pr_err("Failed to request irq %d: %d\n", irq, ret);
			goto err_irq_gpio;
		}
	}

	lge_panel_err_detect_irq_control(panel, false);
	cancel_delayed_work(&panel->lge.err_detect_int_work);

	panel->lge.err_detect_result = 0;
	panel->lge.err_detect_mask = -1;
	panel->lge.err_detect_crash_enabled = 0;
	panel->lge.is_first_err_mask = true;
	return;

err_irq_gpio:
	if (panel->lge.err_detect_int_workq)
		destroy_workqueue(panel->lge.err_detect_int_workq);
	if (irq > 0) {
		disable_irq(irq);
		free_irq(irq, panel);
	}
	if (irq_gpio > 0 && gpio_is_valid(irq_gpio))
		gpio_free(irq_gpio);
err_none:
	return;
}
