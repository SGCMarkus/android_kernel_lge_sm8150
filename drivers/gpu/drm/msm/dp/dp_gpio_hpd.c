/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
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
#define pr_fmt(fmt)	"[drm-dp] %s: " fmt, __func__

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/sde_io_util.h>
#include <linux/of_gpio.h>
#include "dp_gpio_hpd.h"
#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
#include <asm/atomic.h>
#include <soc/qcom/lge/board_lge.h>
#include "../lge/cover/lge_cover_ctrl_ops.h"
#include <linux/lge_cover_display.h>

extern bool lge_get_mfts_mode(void);
#endif
struct dp_gpio_hpd_private {
	struct device *dev;
	struct dp_hpd base;
	struct dss_gpio gpio_cfg;
#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
	struct gpio_desc *gpiod;
	unsigned int debounce;
#endif
	struct delayed_work work;
	struct dp_hpd_cb *cb;
	int irq;
	bool hpd;
};

#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
#include "../lge/dp/lge_dp_def.h"
extern void request_cover_recovery(int inuput);
extern void hallic_handle_5v_boost_gpios(int input);
extern void cover_connection_enable(bool enable);
extern struct lge_dp_display *get_lge_dp(void);
extern bool is_dd_powermode(void);
#endif

static int dp_gpio_hpd_connect(struct dp_gpio_hpd_private *gpio_hpd, bool hpd)
{
	int rc = 0;

	if (!gpio_hpd) {
		pr_err("invalid input\n");
		rc = -EINVAL;
		goto error;
	}

	gpio_hpd->base.hpd_high = hpd;
	gpio_hpd->base.alt_mode_cfg_done = hpd;
	gpio_hpd->base.hpd_irq = false;

	if (!gpio_hpd->cb ||
		!gpio_hpd->cb->configure ||
		!gpio_hpd->cb->disconnect) {
		pr_err("invalid cb\n");
		rc = -EINVAL;
		goto error;
	}

	if (hpd)
		rc = gpio_hpd->cb->configure(gpio_hpd->dev);
	else
		rc = gpio_hpd->cb->disconnect(gpio_hpd->dev);

error:
	return rc;
}

static int dp_gpio_hpd_attention(struct dp_gpio_hpd_private *gpio_hpd)
{
	int rc = 0;

	if (!gpio_hpd) {
		pr_err("invalid input\n");
		rc = -EINVAL;
		goto error;
	}

	gpio_hpd->base.hpd_irq = true;

	if (gpio_hpd->cb && gpio_hpd->cb->attention)
		rc = gpio_hpd->cb->attention(gpio_hpd->dev);

error:
	return rc;
}

static irqreturn_t dp_gpio_isr(int unused, void *data)
{
	struct dp_gpio_hpd_private *gpio_hpd = data;
	u32 const disconnect_timeout_retry = 50;
	bool hpd;
	int i;
#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
	struct lge_dp_display *lge_dp = get_lge_dp();
#endif

	if (!gpio_hpd)
		return IRQ_NONE;

	hpd = gpio_get_value_cansleep(gpio_hpd->gpio_cfg.gpio);

#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
	pr_info("DD Before Debounce Time : %s",	hpd ? "CONNECT" : "DISCONECT");
	if (lge_dp->led_status) {
		pr_info("skip DD HPD low operation for cover_led\n");
		return IRQ_HANDLED;
	}

	if (!hpd) {
		int state = atomic_read(&lge_dp->dd_5v_power_state);
		pr_info("hpd=low, 5v_state=%d, powere_mode=%d\n", state, is_dd_powermode());
		if (state && is_dd_powermode()) {
			set_cover_display_state(COVER_DISPLAY_STATE_CONNECTED_POWERDROP);
		}
	} else if (COVER_DISPLAY_STATE_CONNECTED_POWERDROP == get_cover_display_state()) {
		set_cover_display_state(COVER_DISPLAY_STATE_CONNECTED_ON);

		if (gpio_hpd->base.last_recovery_time == 0) {
			gpio_hpd->base.recovery_count = 0;
			gpio_hpd->base.last_recovery_time = ktime_get();
		} else {
			unsigned int delta = 0;
			ktime_t current_time = ktime_get();
			delta = ktime_ms_delta(current_time, gpio_hpd->base.last_recovery_time);

			if (delta > CONSECUTIVE_RECOVERY_TIME) {
				gpio_hpd->base.recovery_count = 0;
			}
			gpio_hpd->base.last_recovery_time = current_time;
		}

		if (gpio_hpd->base.recovery_count++ < MAX_RECOVERY_COUNT) {
			pr_err("try to recovery (%d/%d)\n", gpio_hpd->base.recovery_count, MAX_RECOVERY_COUNT);
			LGE_COVER_OPS_CALL(set_recovery_state, RECOVERY_POWERDROP_BEGIN);
			request_cover_recovery(1);
		} else {
			pr_err("Power off due to recovery failure\n");
			gpio_hpd->base.recovery_count = 0;
			gpio_hpd->hpd = false;
			set_dd_skip_uevent(0);
			hallic_handle_5v_boost_gpios(0);
			queue_delayed_work(system_wq, &gpio_hpd->work, 0);
			return IRQ_HANDLED;
		}
	}
#endif
	if (!gpio_hpd->hpd && hpd) {
#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
		int connect_timeout_retry = 5;

		/* In DP 1.2 spec, 100msec is recommended for the detection
		 * of HPD connect event. But here we'll poll HPD status for
		 * 5x2ms = 10ms and if HPD is always high, we know DP is
		 * connected. If HPD is low, HPD_IRQ will be handled
		 */
		for (i = 0; i < connect_timeout_retry; i++) {
			if (!hpd) {
				return IRQ_HANDLED;
			}
			usleep_range(2000, 2100);
			hpd = gpio_get_value_cansleep(gpio_hpd->gpio_cfg.gpio);
		}
#endif

		gpio_hpd->hpd = true;
#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
		queue_delayed_work(system_wq, &gpio_hpd->work, msecs_to_jiffies(gpio_hpd->debounce));
#else
		queue_delayed_work(system_wq, &gpio_hpd->work, 0);
#endif
		return IRQ_HANDLED;
	}

#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
	if (gpio_hpd->hpd && hpd) {
		pr_info("Skip duplicated hpd event during debounce!\n");
		return IRQ_HANDLED;
	}
#endif

	if (!gpio_hpd->hpd)
		return IRQ_HANDLED;

	/* In DP 1.2 spec, 100msec is recommended for the detection
	 * of HPD connect event. Here we'll poll HPD status for
	 * 50x2ms = 100ms and if HPD is always low, we know DP is
	 * disconnected. If HPD is high, HPD_IRQ will be handled
	 */
	for (i = 0; i < disconnect_timeout_retry; i++) {
		if (hpd) {
			dp_gpio_hpd_attention(gpio_hpd);
			return IRQ_HANDLED;
		}
		usleep_range(2000, 2100);
		hpd = gpio_get_value_cansleep(gpio_hpd->gpio_cfg.gpio);
	}

	gpio_hpd->hpd = false;
	queue_delayed_work(system_wq, &gpio_hpd->work, 0);
	return IRQ_HANDLED;
}

#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
#include <linux/usb/phy.h>
#endif
static void dp_gpio_hpd_work(struct work_struct *work)
{
	struct delayed_work *dw = to_delayed_work(work);
	struct dp_gpio_hpd_private *gpio_hpd = container_of(dw,
		struct dp_gpio_hpd_private, work);
	int ret;

#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
	struct usb_phy		 *ss_phy;
	pr_info("%s : DD GPIO_HPD : %s", __func__,
		gpio_hpd->hpd ? "CONNECT" : "DISCONECT");
#endif

	if (gpio_hpd->hpd) {
		devm_free_irq(gpio_hpd->dev,
			gpio_hpd->irq, gpio_hpd);
		ret = devm_request_threaded_irq(gpio_hpd->dev,
			gpio_hpd->irq, NULL,
			dp_gpio_isr,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"dp-gpio-intp", gpio_hpd);
		dp_gpio_hpd_connect(gpio_hpd, true);
	} else {
		devm_free_irq(gpio_hpd->dev,
				gpio_hpd->irq, gpio_hpd);
		ret = devm_request_threaded_irq(gpio_hpd->dev,
			gpio_hpd->irq, NULL,
			dp_gpio_isr,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"dp-gpio-intp", gpio_hpd);
		dp_gpio_hpd_connect(gpio_hpd, false);
	}

	if (ret < 0)
		pr_err("Cannot claim IRQ dp-gpio-intp\n");

#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
	if (!gpio_hpd->hpd) {
		// find the USB SSPHY
		ss_phy = devm_usb_get_phy_by_phandle(gpio_hpd->dev,
				"usb-phy", 1);
		if (IS_ERR(ss_phy)) {
			pr_err("unable to get ssphy device on DD\n");
			ret = PTR_ERR(ss_phy);
			if (ret) {
				pr_err("setting pm-qos-latency to zero on DD.\n");
			}
		} else {
			usb_phy_set_suspend(ss_phy, 1);
		}
	}
#endif

}

static int dp_gpio_hpd_simulate_connect(struct dp_hpd *dp_hpd, bool hpd)
{
	int rc = 0;
	struct dp_gpio_hpd_private *gpio_hpd;

	if (!dp_hpd) {
		pr_err("invalid input\n");
		rc = -EINVAL;
		goto error;
	}

	gpio_hpd = container_of(dp_hpd, struct dp_gpio_hpd_private, base);

#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
	if (!hpd) {
		struct lge_dp_display *lge_dp = get_lge_dp();
		int state = atomic_read(&lge_dp->dd_5v_power_state);
		pr_info("hpd=low, 5v_state=%d, powere_mode=%d\n", state, is_dd_powermode());
		if (state && is_dd_powermode()) {
			set_cover_display_state(COVER_DISPLAY_STATE_CONNECTED_POWERDROP);
		}
	}
#endif

	dp_gpio_hpd_connect(gpio_hpd, hpd);

#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
	if (!hpd) {
		// find the USB SSPHY
		int ret = 0;
		struct usb_phy *ss_phy = devm_usb_get_phy_by_phandle(gpio_hpd->dev,
				"usb-phy", 1);
		if (IS_ERR(ss_phy)) {
			pr_err("unable to get ssphy device on DD\n");
			ret = PTR_ERR(ss_phy);
			if (ret) {
				pr_err("setting pm-qos-latency to zero on DD.\n");
			}
		} else {
			usb_phy_set_suspend(ss_phy, 1);
		}
	}
#endif
error:
	return rc;
}

static int dp_gpio_hpd_simulate_attention(struct dp_hpd *dp_hpd, int vdo)
{
	int rc = 0;
	struct dp_gpio_hpd_private *gpio_hpd;

	if (!dp_hpd) {
		pr_err("invalid input\n");
		rc = -EINVAL;
		goto error;
	}

	gpio_hpd = container_of(dp_hpd, struct dp_gpio_hpd_private, base);

	dp_gpio_hpd_attention(gpio_hpd);
error:
	return rc;
}

int dp_gpio_hpd_register(struct dp_hpd *dp_hpd)
{
	struct dp_gpio_hpd_private *gpio_hpd;
	int edge;
	int rc = 0;

	if (!dp_hpd)
		return -EINVAL;

	gpio_hpd = container_of(dp_hpd, struct dp_gpio_hpd_private, base);

	gpio_hpd->hpd = gpio_get_value_cansleep(gpio_hpd->gpio_cfg.gpio);

	edge = gpio_hpd->hpd ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	rc = devm_request_threaded_irq(gpio_hpd->dev, gpio_hpd->irq, NULL,
		dp_gpio_isr,
		edge | IRQF_ONESHOT,
		"dp-gpio-intp", gpio_hpd);
	if (rc) {
		pr_err("Failed to request INTP threaded IRQ: %d\n", rc);
		return rc;
	}

	if (gpio_hpd->hpd)
		queue_delayed_work(system_wq, &gpio_hpd->work, 0);

	return rc;
}

struct dp_hpd *dp_gpio_hpd_get(struct device *dev,
	struct dp_hpd_cb *cb)
{
	int rc = 0;
	const char *hpd_gpio_name = "qcom,dp-hpd-gpio";
	struct dp_gpio_hpd_private *gpio_hpd;
	struct dp_pinctrl pinctrl = {0};

	if (!dev || !cb) {
		pr_err("invalid device\n");
		rc = -EINVAL;
		goto error;
	}

	gpio_hpd = devm_kzalloc(dev, sizeof(*gpio_hpd), GFP_KERNEL);
	if (!gpio_hpd) {
		rc = -ENOMEM;
		goto error;
	}

	pinctrl.pin = devm_pinctrl_get(dev);
	if (!IS_ERR_OR_NULL(pinctrl.pin)) {
		pinctrl.state_hpd_active = pinctrl_lookup_state(pinctrl.pin,
						"mdss_dp_hpd_active");
		if (!IS_ERR_OR_NULL(pinctrl.state_hpd_active)) {
			rc = pinctrl_select_state(pinctrl.pin,
					pinctrl.state_hpd_active);
			if (rc) {
				pr_err("failed to set hpd active state\n");
				goto gpio_error;
			}
		}
	}

	gpio_hpd->gpio_cfg.gpio = of_get_named_gpio(dev->of_node,
		hpd_gpio_name, 0);
	if (!gpio_is_valid(gpio_hpd->gpio_cfg.gpio)) {
		pr_err("%s gpio not specified\n", hpd_gpio_name);
		rc = -EINVAL;
		goto gpio_error;
	}

	strlcpy(gpio_hpd->gpio_cfg.gpio_name, hpd_gpio_name,
		sizeof(gpio_hpd->gpio_cfg.gpio_name));
	gpio_hpd->gpio_cfg.value = 0;

	rc = gpio_request(gpio_hpd->gpio_cfg.gpio,
		gpio_hpd->gpio_cfg.gpio_name);
	if (rc) {
		pr_err("%s: failed to request gpio\n", hpd_gpio_name);
		goto gpio_error;
	}
	gpio_direction_input(gpio_hpd->gpio_cfg.gpio);
#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
	rc = of_property_read_u32(dev->of_node, "lge,dp-gpio-debounce-interval", &gpio_hpd->debounce);
	pr_info("%s: gpio_hpd debounce time = %d\n", __func__, gpio_hpd->debounce);

	if (gpio_hpd->debounce) {
		cover_connection_enable(false);
	} else {
		if (lge_get_factory_boot() || lge_get_mfts_mode()) {
			pr_info("%s : disable hallic connection code for factory mode\n", __func__);
			cover_connection_enable(false);
		} else
			cover_connection_enable(true);
	}
#endif
	gpio_hpd->dev = dev;
	gpio_hpd->cb = cb;
	gpio_hpd->irq = gpio_to_irq(gpio_hpd->gpio_cfg.gpio);
	INIT_DELAYED_WORK(&gpio_hpd->work, dp_gpio_hpd_work);

	gpio_hpd->base.simulate_connect = dp_gpio_hpd_simulate_connect;
	gpio_hpd->base.simulate_attention = dp_gpio_hpd_simulate_attention;
	gpio_hpd->base.register_hpd = dp_gpio_hpd_register;

	return &gpio_hpd->base;

gpio_error:
	devm_kfree(dev, gpio_hpd);
error:
	return ERR_PTR(rc);
}

void dp_gpio_hpd_put(struct dp_hpd *dp_hpd)
{
	struct dp_gpio_hpd_private *gpio_hpd;

	if (!dp_hpd)
		return;

	gpio_hpd = container_of(dp_hpd, struct dp_gpio_hpd_private, base);

	gpio_free(gpio_hpd->gpio_cfg.gpio);
	devm_kfree(gpio_hpd->dev, gpio_hpd);
}
