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
#ifndef _HEADER_TRITON_H_
#define _HEADER_TRITON_H_

#define TRITON_KERNEL_4P14

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/kernel_stat.h>

#ifndef TRITON_KERNEL_4P14
#include <asm/cputime.h>
#endif

#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/lib_lge_triton.h>

#include "main/lge_prm.h"

#define MAX_CPUS_DEFLT                    (8)

#define CPU_FREQ_TRANSITION               0
#define CPU_LOAD_TRANSITION               1
#define CPU_OWNER_TRANSITION              2
#define CPU_UPDATE_TARGET_GOVERNOR_START  3
#define CPU_UPDATE_TARGET_GOVERNOR_STOP   4

enum sysfs_notify_id {
	SYSFS_NOTIFY_CUR_POLICY,
	SYSFS_NOTIFY_AFREQ,
	SYSFS_NOTIFY_BFREQ,
	SYSFS_NOTIFY_ENABLE,
	SYSFS_NOTIFY_ENFORCE,
	SYSFS_NOTIFY_DEBUG
};

#ifdef FPS_BOOST
struct commit_sync
{
	u64 last_commit_ms;
	int drop_count;
	int sync_start;
	int drop_thres;
	int sync_duration;
};
#endif

struct io_dev {
	struct semaphore sem;
	struct cdev char_dev;
};

struct sysfs_notify_info{
	enum sysfs_notify_id notify_id;
	int cur_policy;
	int enable;
	int enforce;
	int aevents;
	int bevents;
	int debug;
};
struct governor_policy_info {
#if !defined(SCHED_BUSY_SUPPORT) && !defined(SCHED_BUSY_CPUS_SUPPORT)
	u64 prev_wall_time;
	u64 prev_idle_time;
#endif
	struct rw_semaphore sem;
	struct cpufreq_policy *policy;
};

struct ioctl_data {
	struct sys_cmd_freq_req freq_per_cluster[NUM_CLUSTER];
	struct sys_cmd_perf_level_req perf_param[BIT_MAX];
	struct sys_cmd_comm_req common_req;
	struct sys_cmd_tunables_req tunables_param;
	struct sys_cmd_tunables_bmc_req tunables_bmc_param[NUM_CLUSTER];
};

struct triton_platform_data {
	int major;
	enum sys_state_machine state;
	enum adjust_level level;
	spinlock_t hotplug_lock;
	spinlock_t frequency_change_lock;
	struct workqueue_struct *fwq;
	struct delayed_work frequency_changed_wq;
	struct workqueue_struct *ping_fwq;
	struct delayed_work ping_wq;
	struct work_struct sysfs_wq;
	struct class *class;
	struct kobject *kobject;
	struct io_dev *tio_dev;
	struct ioctl_data ioctl;
	struct sysfs_notify_info notify_info;
	bool target_governor_state;
	struct mutex target_governor_lock;
};

int triton_notify(unsigned int evt, unsigned int cpu, void *v);
void stack(int cpu, int freq);
struct sched_load *get_cpus_busy(int cpu);
int check_current_cpu_owner(int cpu);

struct cpufreq_policy *sugov_get_policy(int cpu);
void sugov_lock(int cpu, unsigned long lock);
void sugov_unlock(int cpu, unsigned long lock);
int sugov_get_stat(int cpu);

unsigned int sugov_restore_freq(unsigned long data);

int lge_triton_init(void);
#endif
