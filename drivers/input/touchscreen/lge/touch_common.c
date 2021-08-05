/*
 * touch_common.c
 *
 * Copyright (c) 2015 LGE.
 *
 * author : hoyeon.jang@lge.com
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/input/lge_touch_notify.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>
#include <touch_common.h>

void touch_msleep(unsigned int msecs)
{
	if (msecs >= 20)
		msleep(msecs);
	else
		usleep_range(msecs * 1000, msecs * 1000);
}

void touch_interrupt_control(struct device *dev, int on_off)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	if (on_off) {
		if (atomic_cmpxchg(&ts->state.irq_enable, 0, 1) == 0) {
			touch_enable_irq(ts->irq);

			if (ts->role.use_lpwg)
				touch_enable_irq_wake(ts->irq);
		}
	} else {
		if (atomic_cmpxchg(&ts->state.irq_enable, 1, 0) == 1) {
			if (ts->role.use_lpwg)
				touch_disable_irq_wake(ts->irq);

			touch_disable_irq(ts->irq);
		}
	}
}

int touch_snprintf(char *buf, int size, const char *fmt, ...)
{

	va_list args;
	int ret = 0;

	if(size <= 0) {
		TOUCH_E("size is negative!\n");
		return 0;
	}

	va_start(args, fmt);
	ret = vsnprintf(buf, (size_t)size, fmt, args);
	va_end(args);

	if (ret < 0) {
		TOUCH_E("snprintf error. change ret value(%d -> 0)\n", ret);
		ret = 0;
	}

	return ret;
}
