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
#include "lge_intm.h"

#define SET_BIT(data, idx)      ((data) |= (1 << (idx)))
#define CLR_BIT(data, idx)      ((data) &= ~(1 << (idx)))
#define CHECK_BIT(data, bit)    (!((bit) & ((data)^(bit))))

#define GO_ON_NOPANEL_BIT       ((BIT(INIT))|(BIT(ENABLE)))
#define GO_ON_BIT               ((GO_ON_NOPANEL_BIT)|BIT(PANEL))
#define IDLE_INTV           (33000)

enum status {
	INIT,
	ENABLE,
	PANEL,
	BLOCK,
};

static struct intm imon;
static struct task_struct *lge_intv_thread;
static BLOCKING_NOTIFIER_HEAD(nb_head);

void lge_intv_notifier_register(struct notifier_block *nb)
{
	blocking_notifier_chain_register(&nb_head, nb);
}

void lge_intv_notifier_unregister(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&nb_head, nb);
}

void lge_intv_notifier_call_chain(int event, void *v)
{
	blocking_notifier_call_chain(&nb_head, event, v);
}

static int lge_intv_add_idle_data(void)
{
	if (!CHECK_BIT(imon.status_bits, BIT(INIT)))
		return -EPERM;

	mutex_lock(&imon.intv_lock);
	if (!CHECK_BIT(imon.status_bits, GO_ON_BIT)) {
		pr_debug(PRM_TAG "intm is locked:%x\n", imon.status_bits);
		mutex_unlock(&imon.intv_lock);
		return -EPERM;
	}

	imon.cur_ts = ktime_get();
	imon.intv = ktime_to_us(ktime_sub(imon.cur_ts, imon.pre_ts));
	imon.pre_ts = imon.cur_ts;
	if (imon.intv < MIN_INTV)
		imon.intv = MIN_INTV;
	else if (imon.intv > MAX_INTV)
		imon.intv = MAX_INTV;

	atomic_inc(&imon.intv_pending);
	wake_up_all(&imon.intv_wait_q);
	mutex_unlock(&imon.intv_lock);
	return 0;
}

static int __lge_intv_thread(void *data)
{
	int ret;
	unsigned long timeout;

	timeout = usecs_to_jiffies(IDLE_INTV);
	while (1) {
		ret = wait_event_interruptible_timeout(imon.intv_wait_q,
			atomic_read(&imon.intv_pending) > 0 ||
			kthread_should_stop(), timeout);

		if (kthread_should_stop())
			break;

		if (!ret) {
			pr_debug(PRM_TAG "No frame update\n");
			lge_intv_add_idle_data();
			timeout = usecs_to_jiffies(MAX_INTV);
		} else {
			timeout = usecs_to_jiffies(IDLE_INTV);
		}
		lge_intv_notifier_call_chain(INTV_EVENT_NOTIFY,
						 &imon.intv);
		atomic_dec_if_positive(&imon.intv_pending);
		if (!CHECK_BIT(imon.status_bits, BIT(PANEL)))
			break;
	}
	atomic_set(&imon.intv_pending, 0);
	CLR_BIT(imon.status_bits, ENABLE);
	lge_intv_thread = NULL;

	return ret;
}

static int lge_intv_start_thread(void)
{
	int ret = 0;

	pr_debug(PRM_TAG "%pS: start lge_intv thread\n",
		 __builtin_return_address(0));
	lge_intv_thread = kthread_run(__lge_intv_thread,
					  NULL, "lge_intv");

	if (IS_ERR_OR_NULL(lge_intv_thread)) {
		pr_err(PRM_TAG "Unable to start lge_intv thread\n");
		ret = PTR_ERR(lge_intv_thread);
		lge_intv_thread = NULL;
	}

	return ret;
}

int lge_intv_notify(ktime_t cur_us)
{
	if (!CHECK_BIT(imon.status_bits, BIT(INIT)))
		return -EPERM;

	mutex_lock(&imon.intv_lock);
	if (!CHECK_BIT(imon.status_bits, GO_ON_BIT)) {
		pr_debug(PRM_TAG "intm is locked:%x\n", imon.status_bits);
		mutex_unlock(&imon.intv_lock);
		return -EPERM;
	}

	imon.cur_ts = cur_us;
	if (imon.input_boost > 0) {
		imon.intv = MIN_INTV;
		imon.input_boost = 0;
	} else {
		imon.intv = ktime_to_us(ktime_sub(imon.cur_ts, imon.pre_ts));
	}
	imon.pre_ts = imon.cur_ts;

	if (imon.intv < MIN_INTV)
		imon.intv = MIN_INTV;
	else if (imon.intv > MAX_INTV)
		imon.intv = MAX_INTV;

	atomic_inc(&imon.intv_pending);
	wake_up_all(&imon.intv_wait_q);
	mutex_unlock(&imon.intv_lock);

	return 0;
}

int lge_intv_input_notify(void)
{
	if (!CHECK_BIT(imon.status_bits, BIT(INIT)))
		return -EPERM;

	if (!CHECK_BIT(imon.status_bits, GO_ON_NOPANEL_BIT)) {
		pr_debug(PRM_TAG "INTM is locked:%x\n", imon.status_bits);
		return -EPERM;
	}

	imon.intv = BOOST_INTV;
	imon.pre_ts = ktime_get();
	imon.input_boost = 1;

	atomic_inc(&imon.intv_pending);
	wake_up_all(&imon.intv_wait_q);

	return 0;
}

int lge_intv_panel_power_notify(void)
{
	// UNBLANK - 1
	// ELSE    - 0
	int panel_on;
	int on = SDE_MODE_DPMS_OFF;

	if (!CHECK_BIT(imon.status_bits, BIT(INIT)))
		return -EPERM;

	on = lge_prm_get_info(
			LGE_PRM_INFO_DISPLAY_MAIN_STATE);
	on = (on != SDE_MODE_DPMS_ON) ?
		LGE_PRM_DISPLAY_OFF : LGE_PRM_DISPLAY_ON;

	mutex_lock(&imon.intv_lock);
	panel_on = CHECK_BIT(imon.status_bits, BIT(PANEL));
	if (on != panel_on) {
		pr_debug(PRM_TAG "Panel power state: %d->%d\n", panel_on, on);
		if (on)
			SET_BIT(imon.status_bits, PANEL);
		else
			CLR_BIT(imon.status_bits, PANEL);
	} else {
		pr_debug(PRM_TAG "Panel power state is already updated: %d\n", on);
		mutex_unlock(&imon.intv_lock);
		return -EPERM;
	}

	if (CHECK_BIT(imon.status_bits, BIT(ENABLE)) && on) {
		mutex_unlock(&imon.intv_lock);
		if (lge_intv_thread == NULL) {
			lge_intv_start_thread();
		}
		mutex_lock(&imon.intv_lock);
	}

	if (!CHECK_BIT(imon.status_bits, GO_ON_NOPANEL_BIT)) {
		pr_debug(PRM_TAG "INTM is locked:%x\n", imon.status_bits);
		mutex_unlock(&imon.intv_lock);
		return -EPERM;
	}

	imon.intv = BOOST_INTV;
	imon.pre_ts = ktime_get();

	atomic_inc(&imon.intv_pending);
	wake_up_all(&imon.intv_wait_q);
	mutex_unlock(&imon.intv_lock);

	return 0;
}

int lge_intv_enable(int enable)
{
	int ret = 0;
	int enabled;

	if (!CHECK_BIT(imon.status_bits, BIT(INIT)))
		return -EPERM;

	mutex_lock(&imon.intv_lock);
	enabled = CHECK_BIT(imon.status_bits, BIT(ENABLE));
	if (enable) {
		imon.req_cnt++;
	} else {
		if (imon.req_cnt > 0)
			imon.req_cnt--;
	}

	if (enable != enabled) {
		if (imon.req_cnt > 0) {
			SET_BIT(imon.status_bits, ENABLE);
			mutex_unlock(&imon.intv_lock);
			if (lge_intv_thread == NULL) {
				ret = lge_intv_start_thread();
				if (IS_ERR_VALUE(&ret))
					return ret;
			}
		} else {
			CLR_BIT(imon.status_bits, ENABLE);
			mutex_unlock(&imon.intv_lock);
		}
	} else {
		mutex_unlock(&imon.intv_lock);
	}

	return ret;
}

void lge_intv_block(int block)
{
	int blocked;

	if (!CHECK_BIT(imon.status_bits, BIT(INIT)))
		return;

	mutex_lock(&imon.intv_lock);
	blocked = CHECK_BIT(imon.status_bits, BIT(BLOCK));
	if (block)
		SET_BIT(imon.status_bits, BLOCK);
	else
		CLR_BIT(imon.status_bits, BLOCK);
	mutex_unlock(&imon.intv_lock);

	if (block != blocked) {
		if (block) {
			lge_intv_notifier_call_chain(INTV_EVENT_BLOCK,
							 &block);
		} else {
			lge_intv_notifier_call_chain(INTV_EVENT_BLOCK,
							 &block);
			lge_intv_input_notify();
		}
	}
}

int lge_intv_init(void)
{
	if (CHECK_BIT(imon.status_bits, BIT(INIT))) {
		pr_debug(PRM_TAG "INTM is already initialized\n");
		return 0;
	}

	mutex_init(&imon.intv_lock);
	init_waitqueue_head(&imon.intv_wait_q);
	imon.input_boost = 0;
	imon.req_cnt = 0;
	SET_BIT(imon.status_bits, PANEL);
	SET_BIT(imon.status_bits, INIT);

	pr_err(PRM_TAG "LGE INTM probe done\n");
	return 0;
}
