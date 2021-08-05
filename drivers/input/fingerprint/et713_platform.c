/*
 * Simple synchronous userspace interface to PD devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

/*
 *
 *  et713_platform.c
 *  Date: 2016/03/16
 *  Version: 0.9.0.1
 *  Revise Date:  2017/4/17
 *  Copyright (C) 2007-2016 Egis Technology Inc.
 *
 */

#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif

#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/list.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <soc/qcom/scm.h>

#include <linux/pm_wakeup.h>

#include "et713.h"
//#include "navi_input.h"
#define EGIS_PANEL_SUPPORT
#define EDGE_TRIGGER_FALLING 0x0
#define EDGE_TRIGGER_RAISING 0x1
#define LEVEL_TRIGGER_LOW 0x2
#define LEVEL_TRIGGER_HIGH 0x3
#define EGIS_NAVI_INPUT 0 /* 1:open ; 0:close */

#define pinctrl_action_error -1

#ifdef EGIS_PANEL_SUPPORT
extern bool is_ddic_name(char *ddic_name);
#endif

static struct pinctrl *egistec_pinctrl;
static struct pinctrl_state *egistec_pin_state;

struct wakeup_source wakeup_source_fp;

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

static const struct vreg_config const vreg_conf[] = {
	{ "vcc_spi", 3300000UL, 3300000UL, 10, },
};

/*
 * FPS interrupt table
 */

struct interrupt_desc fps_ints = {0, 0, "BUT0", 0};
unsigned int bufsiz = 4096;
int gpio_irq;
int request_irq_done;
/* int t_mode = 255; */

#define EDGE_TRIGGER_FALLING 0x0
#define EDGE_TRIGGER_RISING 0x1
#define LEVEL_TRIGGER_LOW 0x2
#define LEVEL_TRIGGER_HIGH 0x3

struct ioctl_cmd {
	int int_mode;
	int detect_period;
	int detect_threshold;
};

static int vreg_setup(struct egis_data *egis, const char *name, bool enable)
{
	int i, rc;
	int found = 0;
	struct regulator *vreg;
	struct device *dev = &(egis->pd->dev);

	DEBUG_PRINT("[egis] %s\n", __func__);
	for (i = 0; i < 1; i++) {
		const char *n = vreg_conf[i].name;

		DEBUG_PRINT("[egis] vreg_conf[%d].name = %s\n", i, name);

		if (!strncmp(n, name, strlen(n))) {
			DEBUG_PRINT("[egis] Regulator %s is found\n", name);
			found = 1;
			break;
		}
	}

	if (found == 0) {
		DEBUG_PRINT("[egis] Regulator %s is not found\n", name);
		return -EINVAL;
	}

	vreg = egis->vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = regulator_get(dev, name);
			if (IS_ERR(vreg)) {
				DEBUG_PRINT("[egis] Unable to get %s\n", name);
				return PTR_ERR(vreg);
			}
		}

		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin, vreg_conf[i].vmax);
			if (rc)
				DEBUG_PRINT("[egis] Unable to set voltage on %s, %d\n", name, rc);
		}

		rc = regulator_set_load(vreg, vreg_conf[i].ua_load);
		if (rc < 0)
			DEBUG_PRINT("[egis] Unable to set current on %s, %d\n", name, rc);
		rc = regulator_enable(vreg);
		if (rc) {
			DEBUG_PRINT("[egis] error enabling %s: %d\n", name, rc);
			regulator_put(vreg);
			vreg = NULL;
		}
		egis->vreg[i] = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
//				regulator_force_disable(vreg);

				DEBUG_PRINT("[egis] disabled  regulator_force_disable  %s\n", name);
			}
			regulator_put(vreg);
			egis->vreg[i] = NULL;

/*			rc = regulator_set_voltage(vreg,0, 0);
			if (rc)
				DEBUG_PRINT("[egis] Unable to set voltage on %s, %d\n", name, rc);
	
			rc = regulator_enable(vreg);
			if (rc) {
				DEBUG_PRINT("[egis] error enabling %s: %d\n", name, rc);
				regulator_put(vreg);
				vreg = NULL;
			}
			egis->vreg[i] = vreg;
*/
		}
		rc = 0;
	}
	return rc;
}

static struct egis_data *g_data;

DECLARE_BITMAP(minors, N_PD_MINORS);
LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/* ------------------------------ Interrupt -----------------------------*/
/*
 * Interrupt description
 */

#define FP_INT_DETECTION_PERIOD 10
#define FP_DETECTION_THRESHOLD 10
/* struct interrupt_desc fps_ints; */
static DECLARE_WAIT_QUEUE_HEAD(interrupt_waitq);
/*
 *	FUNCTION NAME.
 *		interrupt_timer_routine
 *
 *	FUNCTIONAL DESCRIPTION.
 *		basic interrupt timer inital routine
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return
 */

void interrupt_timer_routine(unsigned long _data)
{
	struct interrupt_desc *bdata = (struct interrupt_desc *)_data;

	DEBUG_PRINT("[egis] FPS interrupt count = %d detect_threshold = %d\n", bdata->int_count, bdata->detect_threshold);
	if (bdata->int_count >= bdata->detect_threshold) {
		bdata->finger_on = 1;
		DEBUG_PRINT("[egis] FPS triggered!\n");
	} else
		DEBUG_PRINT("[egis] FPS not triggered!\n");
	bdata->int_count = 0;
	wake_up_interruptible(&interrupt_waitq);
}

static irqreturn_t fp_eint_func(int irq, void *dev_id)
{
	if (!fps_ints.int_count)
		mod_timer(&fps_ints.timer, jiffies + msecs_to_jiffies(fps_ints.detect_period));

	fps_ints.int_count++;
	__pm_wakeup_event(&wakeup_source_fp, msecs_to_jiffies(1500));
	return IRQ_HANDLED;
}

static irqreturn_t fp_eint_func_ll(int irq, void *dev_id)
{
	disable_irq_nosync(gpio_irq);
	fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;
	DEBUG_PRINT("[egis] %s\n", __func__);
	fps_ints.finger_on = 1;
	wake_up_interruptible(&interrupt_waitq);
	__pm_wakeup_event(&wakeup_source_fp, msecs_to_jiffies(1500));
	return IRQ_RETVAL(IRQ_HANDLED);
}

/*
 *	FUNCTION NAME.
 *		Interrupt_Init
 *
 *	FUNCTIONAL DESCRIPTION.
 *		button initial routine
 *
 *	ENTRY PARAMETERS.
 *		int_mode - determine trigger mode
 *			EDGE_TRIGGER_FALLING    0x0
 *			EDGE_TRIGGER_RAISING    0x1
 *			LEVEL_TRIGGER_LOW        0x2
 *			LEVEL_TRIGGER_HIGH       0x3
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

int Interrupt_Init(struct egis_data *egis, int int_mode, int detect_period, int detect_threshold)
{
	int err = 0;
	int status = 0;
	DEBUG_PRINT("[egis] %s: mode = %d period = %d threshold = %d\n", __func__, int_mode, detect_period, detect_threshold);
	DEBUG_PRINT("[egis] %s: request_irq_done = %d gpio_irq = %d  pin = %d\n", __func__, request_irq_done, gpio_irq, egis->irqPin);
	fps_ints.detect_period = detect_period;
	fps_ints.detect_threshold = detect_threshold;
	fps_ints.int_count = 0;
	fps_ints.finger_on = 0;
	if (request_irq_done == 0) {
		gpio_irq = gpio_to_irq(egis->irqPin);
		if (gpio_irq < 0) {
			DEBUG_PRINT("[egis] %s gpio_to_irq failed\n", __func__);
			status = gpio_irq;
			goto done;
		}

		DEBUG_PRINT("[egis] %s: flag current: %d disable: %d enable: %d\n",  __func__, fps_ints.drdy_irq_flag, DRDY_IRQ_DISABLE, DRDY_IRQ_ENABLE);
		if (int_mode == EDGE_TRIGGER_RISING) {
			DEBUG_PRINT("[egis] %s EDGE_TRIGGER_RISING\n", __func__);
			err = request_irq(gpio_irq, fp_eint_func, IRQ_TYPE_EDGE_RISING, "fp_detect-eint", egis);
			if (err)
				DEBUG_PRINT("[egis] request_irq failed==========%s, %d\n", __func__, __LINE__);
		} else if (int_mode == EDGE_TRIGGER_FALLING) {
			DEBUG_PRINT("[egis] %s EDGE_TRIGGER_FALLING\n", __func__);
			err = request_irq(gpio_irq, fp_eint_func, IRQ_TYPE_EDGE_FALLING, "fp_detect-eint", egis);
			if (err)
				DEBUG_PRINT("[egis] request_irq failed==========%s, %d\n", __func__, __LINE__);
		} else if (int_mode == LEVEL_TRIGGER_LOW) {
			DEBUG_PRINT("[egis] %s LEVEL_TRIGGER_LOW\n", __func__);
			err = request_irq(gpio_irq, fp_eint_func_ll, IRQ_TYPE_LEVEL_LOW, "fp_detect-eint", egis);
			if (err)
				DEBUG_PRINT("[egis] request_irq failed==========%s, %d\n", __func__, __LINE__);
		} else if (int_mode == LEVEL_TRIGGER_HIGH) {
			DEBUG_PRINT("[egis] %s LEVEL_TRIGGER_HIGH\n", __func__);
			err = request_irq(gpio_irq, fp_eint_func_ll, IRQ_TYPE_LEVEL_HIGH, "fp_detect-eint", egis);
			if (err)
				DEBUG_PRINT("[egis] request_irq failed==========%s, %d\n", __func__, __LINE__);
		}
		DEBUG_PRINT("[egis] %s: gpio_to_irq return: %d\n", __func__, gpio_irq);
		DEBUG_PRINT("[egis] %s: request_irq return: %d\n", __func__, err);
		fps_ints.drdy_irq_flag = DRDY_IRQ_ENABLE;
		enable_irq_wake(gpio_irq);
		request_irq_done = 1;
	}

	if (fps_ints.drdy_irq_flag == DRDY_IRQ_DISABLE) {
		fps_ints.drdy_irq_flag = DRDY_IRQ_ENABLE;
		enable_irq_wake(gpio_irq);
		enable_irq(gpio_irq);
	}
done:
	return 0;
}

/*
 *	FUNCTION NAME.
 *		Interrupt_Free
 *
 *	FUNCTIONAL DESCRIPTION.
 *		free all interrupt resource
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

int Interrupt_Free(struct egis_data *egis)
{
	DEBUG_PRINT("[egis] %s\n", __func__);
	fps_ints.finger_on = 0;

	if (fps_ints.drdy_irq_flag == DRDY_IRQ_ENABLE) {
		DEBUG_PRINT("[egis] %s (DISABLE IRQ)\n", __func__);
		disable_irq_nosync(gpio_irq);
		del_timer_sync(&fps_ints.timer);
		fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;
	}
	return 0;
}

/*
 *	FUNCTION NAME.
 *		fps_interrupt_re d
 *
 *	FUNCTIONAL DESCRIPTION.
 *		FPS interrupt read status
 *
 *	ENTRY PARAMETERS.
 *		wait poll table structure
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

unsigned int fps_interrupt_poll(
struct file *file,
struct poll_table_struct *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &interrupt_waitq, wait);
	if (fps_ints.finger_on)
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

void fps_interrupt_abort(void)
{
	DEBUG_PRINT("[egis] %s", __func__);
	fps_ints.finger_on = 0;
	wake_up_interruptible(&interrupt_waitq);
}

int pinctrl_action(struct platform_device *pd, const char *compat, const char *pin_name)
{
	DEBUG_PRINT("[egis] %s  compat = %s  pin_name = %s   start  ----\n", __func__, compat, pin_name);
	pd->dev.of_node = of_find_compatible_node(NULL, NULL, compat);

	if (pd->dev.of_node) {
		egistec_pinctrl = devm_pinctrl_get(&pd->dev);

		if (IS_ERR(egistec_pinctrl)) {
			DEBUG_PRINT("[egis] %s: cannot find egistec_pinctrl\n", __func__);
			return pinctrl_action_error;
		}
	} else {
		DEBUG_PRINT("[egis] %s: can't find compatible node   %s\n", __func__, compat);
		return pinctrl_action_error;
	}
	egistec_pin_state = pinctrl_lookup_state(egistec_pinctrl, pin_name);

	if (IS_ERR(egistec_pin_state)) {
		DEBUG_PRINT("[egis] %s: can't find  %s\n", __func__, pin_name);
		return pinctrl_action_error;
	}
	pinctrl_select_state(egistec_pinctrl, egistec_pin_state);
	DEBUG_PRINT("[egis] %s  compat = %s  pin_name = %s   done ----\n", __func__, compat, pin_name);
	return 0;
}

/*-------------------------------------------------------------------------*/

static void egis_reset(struct egis_data *egis)
{
	DEBUG_PRINT("[egis] %s\n", __func__);
	pinctrl_action(egis->pd, "egistec,et713", "et713_reset_reset");
	msleep(30);
	pinctrl_action(egis->pd, "egistec,et713", "et713_reset_active");
	msleep(20);
}

static void egis_reset_set(struct egis_data *egis, int reset_set)
{
	DEBUG_PRINT("[egis] %s\n", __func__);
	if (reset_set) {
	    pinctrl_action(egis->pd, "egistec,et713", "et713_reset_active");
	} else {
	    pinctrl_action(egis->pd, "egistec,et713", "et713_reset_reset");
	}

}
static void egis_power_onoff(struct egis_data *egis, int power_onoff)
{
	DEBUG_PRINT("[egis] %s   power_onoff = %d \n", __func__, power_onoff);
	if (power_onoff) {
	    vreg_setup(egis, "vcc_spi", 1);    //pinctrl_action(egis->pd, "egistec,et713", "et713_ldo_high");
	    usleep_range(2000,2000);
	    pinctrl_action(egis->pd, "egistec,et713", "et713_reset_active");
	} else {
	    vreg_setup(egis, "vcc_spi", 0);    //pinctrl_action(egis->pd, "egistec,et713", "et713_ldo_low");
	    pinctrl_action(egis->pd, "egistec,et713", "et713_reset_reset");
	    usleep_range(2000,2000);
	}
}

static void egis_power_onoff_ex(struct egis_data *egis, int power_onoff)
{
	DEBUG_PRINT("[egis] %s   power_onoff_ex = %d \n", __func__, power_onoff);
	if (power_onoff) {
	    vreg_setup(egis, "vcc_spi", 1);    //pinctrl_action(egis->pd, "egistec,et713", "et713_ldo_high");
	    msleep(2);
	    pinctrl_action(egis->pd, "egistec,et713", "et713_reset_active");
		msleep(2);
		pinctrl_action(egis->pd, "egistec,et713", "et713_reset_reset");
		msleep(13);
		pinctrl_action(egis->pd, "egistec,et713", "et713_reset_active");
		msleep(8);
	} else {
	    vreg_setup(egis, "vcc_spi", 0);    //pinctrl_action(egis->pd, "egistec,et713", "et713_ldo_low");
	    pinctrl_action(egis->pd, "egistec,et713", "et713_reset_reset");
	    msleep(50);
	}
}

static void egis_get_panel_info(u8 *panel_info)
{
	/* Get the panel info */
	if (is_ddic_name("rm692A9")) {
		*panel_info = 1; //visionox
	} else {
		*panel_info = 0; // default
	}
}

static ssize_t egis_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	/*if needed*/
	return 0;
}

static ssize_t egis_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	/*if needed*/
	return 0;
}

static long egis_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	struct egis_data *egis;
	struct ioctl_cmd data;
	u8 panel_type = 0;
	DEBUG_PRINT("[egis] %s", __func__);
	memset(&data, 0, sizeof(data));
	egis = filp->private_data;

	switch (cmd) {
	case INT_TRIGGER_INIT:
		if (copy_from_user(&data, (int __user *)arg, sizeof(data))) {
			retval = -EFAULT;
			break;
		}
		retval = Interrupt_Init(egis, data.int_mode, data.detect_period, data.detect_threshold);
		DEBUG_PRINT("[egis] %s: fp_ioctl trigger init = %x\n", __func__, retval);
		break;
	case FP_SENSOR_RESET:
		DEBUG_PRINT("[egis] %s: FP_SENSOR_RESET\n", __func__);
		egis_reset(egis);
		break;
	case FP_POWER_ONOFF:
		if (copy_from_user(&data, (int __user *)arg, sizeof(data))) {
			retval = -EFAULT;
			break;
		}
		egis_power_onoff(egis, data.int_mode);  // Use data.int_mode as power setting. 1 = on, 0 = off.
		DEBUG_PRINT("[egis] %s: egis_power_onoff = %d\n", __func__, data.int_mode);
		break;
	case FP_POWER_ONOFF_EX:
		if (copy_from_user(&data, (int __user *)arg, sizeof(data))) {
			retval = -EFAULT;
			break;
		}
		egis_power_onoff_ex(egis, data.int_mode);  // Use data.int_mode as power setting. 1 = on, 0 = off.
		DEBUG_PRINT("[egis] %s: egis_power_onoff_ex = %d\n", __func__, data.int_mode);
		break;
	case FP_RESET_SET:
		if (copy_from_user(&data, (int __user *)arg, sizeof(data))) {
			retval = -EFAULT;
			break;
		}
		egis_reset_set(egis, data.int_mode);  // Use data.int_mode as reset setting. 1 = high, 0 = low.
		DEBUG_PRINT("[egis] %s: egis_reset_set = %d\n", __func__, data.int_mode);
		break;
	case INT_TRIGGER_CLOSE:
		retval = Interrupt_Free(egis);
		DEBUG_PRINT("[egis] %s: INT_TRIGGER_CLOSE = %x\n", __func__, retval);
		break;
	case INT_TRIGGER_ABORT:
		DEBUG_PRINT("[egis] %s: INT_TRIGGER_ABORT\n", __func__);
		fps_interrupt_abort();
		break;
	case FP_GET_PANEL_INFO:
		DEBUG_PRINT("[egis] %s: FP_GET_PANEL_INFO\n", __func__);
		egis_get_panel_info(&panel_type);
	        copy_to_user((void __user *) arg, (void *) &panel_type, sizeof(u8));
		break;
	default:
		retval = -ENOTTY;
		break;
	}
	return retval;
}

#ifdef CONFIG_COMPAT
static long egis_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	DEBUG_PRINT("[egis] %s", __func__);
	return egis_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define egis_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int egis_open(struct inode *inode, struct file *filp)
{
	struct egis_data *egis;
	int status = -ENXIO;

	DEBUG_PRINT("[egis] %s\n", __func__);
	mutex_lock(&device_list_lock);

	list_for_each_entry(egis, &device_list, device_entry) {
		if (egis->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status == 0) {
		if (egis->buffer == NULL) {
			egis->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (egis->buffer == NULL)
				status = -ENOMEM;
		}
		if (status == 0) {
			egis->users++;
			filp->private_data = egis;
			nonseekable_open(inode, filp);
		}
	} else
		DEBUG_PRINT("[egis] %s nothing for minor %d\n", __func__, iminor(inode));
	mutex_unlock(&device_list_lock);
	return status;
}

static int egis_release(struct inode *inode, struct file *filp)
{
	struct egis_data *egis;

	DEBUG_PRINT("[egis] %s\n", __func__);
	mutex_lock(&device_list_lock);
	egis = filp->private_data;
	filp->private_data = NULL;

	egis->users--;
	if (egis->users == 0) {
		int dofree;

		kfree(egis->buffer);
		egis->buffer = NULL;

		/* after we unbound from the underlying device */
		spin_lock_irq(&egis->pd_lock);
		dofree = (egis->pd == NULL);
		spin_unlock_irq(&egis->pd_lock);

		if (dofree)
			kfree(egis);
	}
	mutex_unlock(&device_list_lock);
	return 0;
}

int egis_platformInit(struct egis_data *egis)
{
	int status = 0;

	DEBUG_PRINT("[egis] %s\n", __func__);

	if (egis != NULL) {
		/* Initial IRQ Pin*/
		status = gpio_request(egis->irqPin, "gpio_irq");
		if (status < 0) {
			DEBUG_PRINT("[egis] %s gpio_request egis_irq failed\n", __func__);
			goto egis_platformInit_irq_failed;
		}
		status = gpio_direction_input(egis->irqPin);
		if (status < 0) {
			DEBUG_PRINT("[egis] %s gpio_direction_input IRQ failed\n", __func__);
			goto egis_platformInit_gpio_init_failed;
		}
	}
	DEBUG_PRINT("[egis] %s: successful, status = %d\n", __func__, status);
	return status;

egis_platformInit_gpio_init_failed:
	gpio_free(egis->irqPin);
egis_platformInit_irq_failed:
	gpio_free(egis->rstPin);
	DEBUG_PRINT("[egis] %s is failed\n", __func__);
	return status;
}

static int egis_parse_dt(struct device *dev, struct egis_data *data)
{
	struct device_node *np = dev->of_node;
	int errorno = 0;
	int gpio;

	gpio = of_get_named_gpio(np, "egistec,gpio_rst", 0);
	if (gpio < 0) {
		errorno = gpio;
		goto dt_exit;
	} else {
		data->rstPin = gpio;
		DEBUG_PRINT("[egis] %s: sleepPin = %d\n", __func__, data->rstPin);
	}
	gpio = of_get_named_gpio(np, "egistec,gpio_irq", 0);
	if (gpio < 0) {
		errorno = gpio;
		goto dt_exit;
	} else {
		data->irqPin = gpio;
		DEBUG_PRINT("[egis] %s: drdyPin = %d\n", __func__, data->irqPin);
	}

	DEBUG_PRINT("[egis] %s is successful\n", __func__);
	return errorno;
dt_exit:
	DEBUG_PRINT("[egis] %s is failed\n", __func__);
	return errorno;
}

static const struct file_operations egis_fops = {
	.owner = THIS_MODULE,
	.write = egis_write,
	.read = egis_read,
	.unlocked_ioctl = egis_ioctl,
	.compat_ioctl = egis_compat_ioctl,
	.open = egis_open,
	.release = egis_release,
	.llseek = no_llseek,
	.poll = fps_interrupt_poll};

/*-------------------------------------------------------------------------*/

static struct class *egis_class;
static int egis_probe(struct platform_device *pdev);
static int egis_remove(struct platform_device *pdev);
static const struct of_device_id egis_match_table[] = {{
	.compatible = "egistec,et713",
	},
	{},
};
MODULE_DEVICE_TABLE(of, egis_match_table);

static struct platform_driver egis_driver = {
	.driver = {
		.name = "et713",
		.owner = THIS_MODULE,
		.of_match_table = egis_match_table,
	},
	.probe = egis_probe,
	.remove = egis_remove,
};

static int egis_remove(struct platform_device *pdev)
{
	DEBUG_PRINT("[egis] %s (#%d)\n", __func__, __LINE__);
	free_irq(gpio_irq, g_data);
	del_timer_sync(&fps_ints.timer);
	wakeup_source_trash(&wakeup_source_fp);
	request_irq_done = 0;
	return 0;
}

static int egis_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct egis_data *egis;
	int status = 0;
	int major_number = 0;
	unsigned long minor;
	struct device *fdev;

	DEBUG_PRINT("[egis] %s\n", __func__);
	BUILD_BUG_ON(N_PD_MINORS > 256);
	major_number = register_chrdev(major_number, "et713", &egis_fops);
	if (major_number < 0) {
		DEBUG_PRINT("[egis] %s: register_chrdev error.\n", __func__);
		return major_number;
	}
	else {
		DEBUG_PRINT("[egis] %s: register_chrdev major_number = %d.\n", __func__, major_number);
	}

	egis_class = class_create(THIS_MODULE, "et713");
	if (IS_ERR(egis_class)) {
		DEBUG_PRINT("[egis] %s: class_create error.\n", __func__);
		unregister_chrdev(major_number, egis_driver.driver.name);
		return PTR_ERR(egis_class);
	}
	/* Allocate driver data */
	egis = kzalloc(sizeof(*egis), GFP_KERNEL);
	if (egis == NULL) {
		DEBUG_PRINT("[egis] %s: Failed to kzalloc\n", __func__);
		return -ENOMEM;
	}
	/* Initialize the driver data */
	egis->pd = pdev;
	g_data = egis;

	spin_lock_init(&egis->pd_lock);
	mutex_init(&egis->buf_lock);
	mutex_init(&device_list_lock);

	INIT_LIST_HEAD(&egis->device_entry);


        status=egis_parse_dt(dev, egis);
	if (status != 0) {
		DEBUG_PRINT("[egis] %s: parse_dt failed\n", __func__);
		goto egis_probe_parse_dt_failed;
	}
	/* platform init */
	status = egis_platformInit(egis);
	if (status != 0) {
		DEBUG_PRINT("[egis] %s: platform init failed\n", __func__);
		goto egis_probe_platformInit_failed;
	}

	fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;
	/*
	 * If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_PD_MINORS);
	if (minor < N_PD_MINORS) {
		egis->devt = MKDEV(major_number, minor);
		fdev = device_create(egis_class, &pdev->dev, egis->devt, egis, "esfp0");
		status = IS_ERR(fdev) ? PTR_ERR(fdev) : 0;
	} else {
		DEBUG_PRINT("[egis] %s: no minor number available!\n", __func__);
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&egis->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		dev_set_drvdata(dev, egis);
	else
		goto egis_probe_failed;

	fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;

	/* the timer is for ET310 */
	setup_timer(&fps_ints.timer, interrupt_timer_routine, (unsigned long)&fps_ints);
	add_timer(&fps_ints.timer);

	wakeup_source_init(&wakeup_source_fp, "et713_wakeup");

	DEBUG_PRINT("[egis] %s: initialize success %d\n", __func__, status);
//	vreg_setup(egis, "vcc_spi", 1);
//	msleep(10);
//	egis_reset_set(egis, 1);
//	pinctrl_action(egis->pd, "egistec,et713", "et713_ldo_high");
//	pinctrl_action(egis->pd, "egistec,et713", "et713_irq_active");
//	egis_reset(egis);
	request_irq_done = 0;

#if EGIS_NAVI_INPUT
	sysfs_egis_init(egis);
	uinput_egis_init(egis);
#endif
	return status;

egis_probe_failed:
	device_destroy(egis_class, egis->devt);
	class_destroy(egis_class);

egis_probe_platformInit_failed:
egis_probe_parse_dt_failed:
	kfree(egis);
	DEBUG_PRINT("[egis] %s is failed\n", __func__);
#if EGIS_NAVI_INPUT
	uinput_egis_destroy(egis);
	sysfs_egis_destroy(egis);
#endif
	return status;
}

static int __init et713_init(void)
{
	int status = 0;

	DEBUG_PRINT("[egis] %s\n", __func__);
	status = platform_driver_register(&egis_driver);
	DEBUG_PRINT("[egis] %s done\n", __func__);
	return status;
}

static void __exit et713_exit(void)
{
	DEBUG_PRINT("   -------   [egis] %s platform_driver_unregister\n", __func__);
	platform_driver_unregister(&egis_driver);
}

module_init(et713_init);
module_exit(et713_exit);

MODULE_AUTHOR("Wang YuWei, <robert.wang@egistec.com>");
MODULE_DESCRIPTION("Platform Driver Interface for ET713");
MODULE_LICENSE("GPL");

