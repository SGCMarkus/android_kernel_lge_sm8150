#define pr_fmt(fmt)	"[Display][lge-cover-ctrl:%s:%d] " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/extcon.h>
#include <asm/atomic.h>
#include "lge_cover_ctrl.h"
#include "lge_cover_ctrl_ops.h"

extern void hallic_handle_5v_boost_gpios(int state);
extern struct ice40 *global_ice40;
extern int ice40_mcu_reg_write(struct ice40 *ice40, uint addr, uint val);
extern int ice40_mcu_reg_write_multi(struct ice40 *ice40, uint addr, u8 *val);
extern int ice40_master_reg_write(struct ice40 *ice40, uint addr, uint val);
extern int ice40_master_reg_read(struct ice40 *ice40, uint addr, uint *val);
extern int ice40_enable(struct ice40 *ice40);
extern int ice40_disable(struct ice40 *ice40);
extern struct lge_dp_display* get_lge_dp(void);
//extern void set_skip_uevent(int skip_uevent);
extern int cover_led_status;
extern bool is_dp_connected(void);
extern void call_disconnect_uevent(void);
extern bool is_dd_connected(void);

struct cover_ctrl_data {
	bool initialized;
	bool stream_preoff_state;
	enum recovery_state recovery_state;
	int recovery_count[5];
	ktime_t last_recovery_time[5];
};
static struct cover_ctrl_data cdisplay_data;

static const unsigned int dd_extcon_cable[] = {
	EXTCON_DISP_DP,
	EXTCON_DISP_DD,
	EXTCON_NONE,
};

struct dp_hpd *dd_hpd_get(struct device *dev, struct dp_hpd_cb *cb)
{
	struct dp_hpd *dd_hpd;

	dd_hpd = dp_gpio_hpd_get(dev, cb);
	if (!dd_hpd) {
		pr_err("failed to get gpio hpd\n");
		goto out;
	}
	dd_hpd->type = DP_HPD_GPIO;
out:
	return dd_hpd;
}

void dd_hpd_put(struct dp_hpd *dd_hpd)
{
	if (!dd_hpd)
		return;

	dp_gpio_hpd_put(dd_hpd);
}

void dd_set_force_disconnection(bool val)
{
	struct lge_dp_display *lge_dp = get_lge_dp();
	pr_err("=%d\n", val);
	lge_dp->force_disconnection = val;
	return;
}
EXPORT_SYMBOL(dd_set_force_disconnection);

bool dd_get_force_disconnection(void)
{
	struct lge_dp_display *lge_dp = get_lge_dp();
	pr_err("=%d\n", lge_dp->force_disconnection);
	return lge_dp->force_disconnection;
}
EXPORT_SYMBOL(dd_get_force_disconnection);

int is_dd_need_to_off(void)
{
	struct lge_dp_display *lge_dp;

	lge_dp = get_lge_dp();
	return atomic_read(&lge_dp->dd_uevent_switch);
}
EXPORT_SYMBOL(is_dd_need_to_off);

void dd_set_skip_uevent(int input)
{
	struct lge_dp_display *lge_dp = get_lge_dp();

	lge_dp->skip_uevent = input;
}
EXPORT_SYMBOL(dd_set_skip_uevent);

int lge_cover_extcon_register(struct platform_device *pdev, struct dp_display *display, int id)
{
	int ret = 0;
	struct platform_device *sdev = pdev;

	if(!sdev) {
		pr_err("%s : DD SDEV NULL register failed\n", __func__);
		return -EINVAL;
	}

	display->dd_extcon_sdev[id] = devm_extcon_dev_allocate (&sdev->dev, dd_extcon_cable);

	if (IS_ERR(display->dd_extcon_sdev[id]))
	    return PTR_ERR(display->dd_extcon_sdev[id]);

	ret = devm_extcon_dev_register(&sdev->dev, display->dd_extcon_sdev[id]);
	if (ret) {
		pr_err("%s : DD extcon register failed\n", __func__);
		return ret;
	}
	pr_info ("%s : done\n", __func__);
	return ret;
}

void dd_gpio_selection(int dd_hpd, int flip)
{
	int ret;
	uint data;

	ret = gpio_get_value(67);
	pr_info("[before enable] gpio 67 value %d\n", ret);

	ice40_enable(global_ice40);
	ret = ice40_master_reg_read(global_ice40, 0x00, &data);

	if (dd_hpd) {
		pr_info("Set to connect DD\n");
		ice40_master_reg_write(global_ice40, 0x00, (data&0xFE));
		gpio_set_value(35, 1);
		gpio_direction_output(67, 1);
	} else {
		pr_info("Set to connect USB_DP, flip:%d\n", flip);
		if (flip)
			ice40_master_reg_write(global_ice40, 0x00, (data&0xFD) | 0x01);
		else
			ice40_master_reg_write(global_ice40, 0x00, data|0x03);
		gpio_set_value(35, 0);
		gpio_direction_output(67, 0);
	}

	ret = gpio_get_value(67);
	pr_info("[after enable] gpio 67 value %d\n", ret);

	return;
}

void dd_lattice_disable()
{
	ice40_disable(global_ice40);
}

bool is_dd_button_enabled(void)
{
	struct lge_dp_display *lge_dp = get_lge_dp();

	if (!lge_dp)
		return false;

	return ((lge_dp->dd_button_state == 0) ? false : true);
}
EXPORT_SYMBOL(is_dd_button_enabled);

static void change_led_status(struct lge_dp_display *lge_dp, int status)
{
	pr_info("status=%d\n", status);

	if (status) {
		lge_dp->led_status = status;

		cover_led_status = status;
		hallic_handle_5v_boost_gpios(!!status);

		atomic_set(&lge_dp->pending, 0);
		lge_dp->pending_status = -1;

		gpio_set_value(73, 0);
		pr_info("gpio 73(PD_N_EN) value %d\n", gpio_get_value(73));
	} else {
		cover_led_status = status;
		hallic_handle_5v_boost_gpios(!!status);

		atomic_set(&lge_dp->pending, 0);
		lge_dp->pending_status = -1;

		lge_dp->led_status = status;
	}

	return;
}

static int update_led_status(struct lge_dp_display *lge_dp, int status)
{
	int button = 0;

	if (!lge_dp) {
		pr_err("null_ptr\n");
		return -EINVAL;
	}

	mutex_lock(&lge_dp->cover_lock);
	button = lge_dp->dd_button_state;

	if (button) {
		atomic_set(&lge_dp->pending, 1);
		lge_dp->pending_status = status;
	} else {
		int new_status = status;
		int retry_cnt = 500;
		if (atomic_read(&lge_dp->pending) > 0) {
			pr_info("change to pending status : %d->%d\n",
					new_status, lge_dp->pending_status);
			new_status = lge_dp->pending_status;

			do {
				usleep_range(2000, 2100);
				if (--retry_cnt == 0) {
					pr_err("waiting timed out, 1sec\n");
					break;
				}
			} while(!is_dd_connected());
		}

		if (new_status < 0)
			goto exit;

		change_led_status(lge_dp, new_status);
	}

exit:
	mutex_unlock(&lge_dp->cover_lock);
	return 0;
}

static ssize_t cover_button_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lge_dp_display *lge_dp;

	lge_dp = dev_get_drvdata(dev);

	return sprintf(buf, "%d %d\n", lge_dp->dd_button_state, lge_dp->skip_uevent);
}

static ssize_t cover_led_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int led_status;
	struct lge_dp_display *lge_dp;
	ssize_t ret = strnlen(buf, PAGE_SIZE);

	lge_dp = dev_get_drvdata(dev);

	sscanf(buf, "%d", &led_status);
	pr_err("%s : %d\n", __func__, led_status);

	if (update_led_status(lge_dp, led_status) < 0) {
		pr_err("failed to update status\n");
	}

	return ret;
}
static DEVICE_ATTR(cover_led, S_IWUSR | S_IWGRP, NULL, cover_led_set);

static ssize_t cover_button_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int onoff, skip_uevent;
	struct lge_dp_display *lge_dp;
	ssize_t ret = strnlen(buf, PAGE_SIZE);

	lge_dp = dev_get_drvdata(dev);

	sscanf(buf, "%d %d", &onoff, &skip_uevent);
	pr_err("%s : %d %d\n", __func__, onoff, skip_uevent);

	if (is_dp_connected()) {
		pr_info("%s : USBDP connecetd skip button set\n", __func__);
		return ret;
	}


	if ((is_dd_need_to_off()) && (!lge_dp->dd_button_state) && (!onoff) && skip_uevent) {
		pr_info("%s : Framework request to remove display contents \n", __func__);
		call_disconnect_uevent();
	}

	if (onoff && lge_dp->led_status) {
		update_led_status(lge_dp, 0);
		usleep_range(2000, 2100);
	}

	lge_dp->dd_button_state = onoff;
	lge_dp->skip_uevent = !skip_uevent;
	hallic_handle_5v_boost_gpios(!!onoff);

	if (!onoff) {
		if (update_led_status(lge_dp, -1) < 0) {
			pr_err("failed to update status\n");
		}
	}

	return ret;
}
static DEVICE_ATTR(cover_button, S_IRUGO | S_IWUSR | S_IWGRP, cover_button_get, cover_button_set);

static ssize_t cover_max_duty_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;

	if (!is_dd_connected()) {
		pr_err("DD is not connected\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	if (!input)
		pr_info("DD Panel Recovery\n");
	else
		pr_info("DD set max duty\n");

	if (ice40_mcu_reg_write(global_ice40, ICE40_DUTY_MAX, input) < 0) {
		pr_err("unable to set DD duty max\n");
		return -EINVAL;
	}

	return ret;
}
static DEVICE_ATTR(cover_max_duty, S_IWUSR | S_IWGRP, NULL, cover_max_duty_set);

static ssize_t cover_panel_onoff_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;

	if (!is_dd_connected()) {
		pr_err("DD is not connected\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	if (!input)
		pr_info("DD Panel Off\n");
	else
		pr_info("DD Panel On\n");

	if (ice40_mcu_reg_write(global_ice40, ICE40_PANEL_ONOFF, input) < 0) {
		pr_err("unable to set DD panel onoff\n");
		return -EINVAL;
	}

	return ret;
}
static DEVICE_ATTR(cover_panel_onoff, S_IWUSR | S_IWGRP, NULL, cover_panel_onoff_set);

static ssize_t cover_brightness_debug_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int input;
	u8 tx_buf[4] = {0,};

	if (!is_dd_connected()) {
		pr_err("DD is not connected\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);

	tx_buf[0] = (u8)(input >> 8);
	tx_buf[1] = (u8)(input & 0xFF);
	pr_info("DD debug brightness: %d, 0x%x%x\n", input, tx_buf[0], tx_buf[1]);

	if (ice40_mcu_reg_write_multi(global_ice40, ICE40_BRIGHTNESS_DEBUG, tx_buf) < 0) {
		pr_err("unable to set DD brightness debug\n");
		return -EINVAL;
	}

	return ret;
}
static DEVICE_ATTR(cover_brightness_debug, S_IWUSR | S_IWGRP, NULL, cover_brightness_debug_set);

int lge_cover_ctrl_create_sysfs(struct device *panel_sysfs_dev)
{
	int rc = 0;
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_cover_button)) < 0)
		pr_err("add cover_button set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_cover_led)) < 0)
		pr_err("add cover_button set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_cover_max_duty)) < 0)
		pr_err("add cover_max_duty set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_cover_panel_onoff)) < 0)
		pr_err("add cover_panel_onoff set node fail!");
	if ((rc = device_create_file(panel_sysfs_dev, &dev_attr_cover_brightness_debug)) < 0)
		pr_err("add cover_brightness_debug set node fail!");
	return rc;
}

/******* cover control ******/
static struct cover_ctrl_data *get_and_validate_data(void)
{
	struct cover_ctrl_data *c_data = &cdisplay_data;

	if (!c_data) {
		pr_err("null ptr\n");
		return NULL;
	}

	if (!c_data->initialized) {
		pr_err("not initialized\n");
		return NULL;
	}

	return c_data;
}

static bool is_recovery_consecutive(enum recovery_state state, struct cover_ctrl_data *c_data)
{
	bool ret = false;
	unsigned int delta = 0;
	int idx = state / 10;
	ktime_t current_time = ktime_get();
	ktime_t prev_time = c_data->last_recovery_time[idx];

	if (prev_time == 0) {
		goto exit;
	}

	delta = ktime_ms_delta(current_time, prev_time);

	if (delta <= CONSECUTIVE_RECOVERY_TIME) {
		c_data->recovery_count[idx] = 0;
		ret = true;
	}
	c_data->recovery_count[idx]++;
exit:
	c_data->last_recovery_time[idx] = current_time;

	return ret;
}

static int lge_cover_ctrl_set_recovery_state(enum recovery_state state)
{
	struct lge_dp_display *lge_dp = get_lge_dp();
	struct cover_ctrl_data *c_data = get_and_validate_data();
	enum recovery_state prev_state = RECOVERY_MAX;
	static int skip_uevent;

	if (!lge_dp || !c_data)
		return -EINVAL;

	prev_state = c_data->recovery_state;

	switch (state) {
	case RECOVERY_NONE:
	case RECOVERY_POWERDROP_BEGIN:
		c_data->recovery_state = state;
		break;
	case RECOVERY_LTFAIL_DETECTED:
	case RECOVERY_MCU_DETECTED:
		if (is_recovery_consecutive(state, c_data)) {
			c_data->recovery_state = --state;
		} else {
			c_data->recovery_state = ++state;
		}
		break;
	case RECOVERY_LTFAIL_BEGIN:
	case RECOVERY_MCU_BEGIN:
		skip_uevent = lge_dp->skip_uevent;
		dd_set_skip_uevent(0);
		hallic_handle_5v_boost_gpios(0);
		c_data->recovery_state = ++state;
		break;
	case RECOVERY_LTFAIL_DONE:
	case RECOVERY_MCU_DONE:
		dd_set_skip_uevent(skip_uevent);
		hallic_handle_5v_boost_gpios(1);
		c_data->recovery_state = RECOVERY_NONE;
		break;
	case RECOVERY_POWERDROP_DONE:
		c_data->recovery_state = RECOVERY_NONE;
		break;
	default:
		break;
	}

	pr_info("changed: %d => %d\n", prev_state, c_data->recovery_state);

	return 0;
};

static enum recovery_state lge_cover_ctrl_get_recovery_state(void)
{
	struct cover_ctrl_data *c_data = get_and_validate_data();

	if (!c_data)
		return RECOVERY_NONE;

	return c_data->recovery_state;
};

struct lge_cover_ops *get_lge_cover_ops(void)
{
	struct lge_dp_display *lge_dp = get_lge_dp();
	struct cover_ctrl_data *c_data = get_and_validate_data();

	if (!lge_dp || !c_data) {
		pr_err("null ptr\n");
		return NULL;
	}

	return lge_dp->cover_ops;
}

static void lge_cover_ctrl_set_stream_preoff_state(bool preoff)
{
	struct cover_ctrl_data *c_data = get_and_validate_data();

	if (!c_data)
		return;

	c_data->stream_preoff_state = preoff;
	return;
};

static bool lge_cover_ctrl_get_stream_preoff_state(void)
{
	struct cover_ctrl_data *c_data = get_and_validate_data();

	if (!c_data)
		return false;

	return c_data->stream_preoff_state;
};

struct lge_cover_ops cover_ops = {
	.set_recovery_state = lge_cover_ctrl_set_recovery_state,
	.get_recovery_state = lge_cover_ctrl_get_recovery_state,
	.set_stream_preoff_state = lge_cover_ctrl_set_stream_preoff_state,
	.get_stream_preoff_state = lge_cover_ctrl_get_stream_preoff_state,
};

int init_cover_ctrl_ops(void)
{
	struct lge_dp_display *lge_dp = get_lge_dp();
	struct cover_ctrl_data *c_data = &cdisplay_data;

	pr_info("initialized\n");

	lge_dp->cover_ops = &cover_ops;
	memset(c_data, 0, sizeof(struct cover_ctrl_data));
	c_data->initialized = true;

	return 0;
}

