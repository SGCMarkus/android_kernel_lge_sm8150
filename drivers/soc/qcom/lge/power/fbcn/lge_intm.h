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
#ifndef LGE_INTM_H
#define LGE_INTM_H

#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/err.h>
#include <uapi/drm/sde_drm.h>

#include "../main/lge_prm.h"

#define MIN_INTV (USEC_PER_SEC/60)
#define MAX_INTV (USEC_PER_SEC/1)
#define BOOST_INTV (0)

enum {
	INTV_EVENT_NOTIFY,
	INTV_EVENT_BLOCK,
	INTV_EVENT_MAX,
};

struct intm {
	ktime_t pre_ts;
	ktime_t cur_ts;
	u32 intv;
	u16 status_bits;
	atomic_t intv_pending;
	wait_queue_head_t intv_wait_q;
	int input_boost;
	int req_cnt;
	struct mutex intv_lock;
};

struct display_info {
	int init;
};

int lge_intv_enable(int enable);
int lge_intv_input_notify(void);
void lge_intv_block(int block);
void lge_intv_notifier_register(struct notifier_block *nb);
void lge_intv_notifier_unregister(struct notifier_block *nb);
void lge_intv_notifier_call_chain(int event, void *v);

int lge_intv_notify(ktime_t cur_us);
int lge_intv_panel_power_notify(void);

int lge_intv_init(void);
#endif /* lge_intv_MONITOR_H */
