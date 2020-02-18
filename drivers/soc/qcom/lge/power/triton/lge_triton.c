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

#define CREATE_TRACE_POINTS

#include "lge_triton.h"

#ifndef TRITON_KERNEL_4P14
#include <trace/events/cpufreq_sched.h>
#endif

#ifdef TRITON_KERNEL_4P14
inline struct cpufreq_policy *sugov_get_policy(int cpu) { return NULL; }
inline void sugov_lock(int cpu, unsigned long lock) {}
inline void sugov_unlock(int cpu, unsigned long lock) {}
inline int sugov_get_stat(int cpu) { return 0; }
inline unsigned int sugov_restore_freq(unsigned long data) { return 0; }
#endif

#if (NUM_CLUSTER > 1)
static int lcluster_cores;
static int bcluster_cores;
static int lcluster_start;
static int bcluster_start;
#endif

static u64 ping_time, last_ping_time;
#define DEFAULT_PING_INTV (5000)
#define DEFAULT_PING_EXP (10000)

static int ping_expired = 0;
static struct device *g_sysfs_dev;

static struct triton_platform_data platform_data;
static DEFINE_PER_CPU(struct governor_policy_info, policy_info);

#if (NUM_CLUSTER > 1)
#define CPU_NUM_BY_CLUSTER(c, f, l) \
	*f = (c << (CORES_PER_CLUSTER >>1)); \
	*l = (CORES_PER_CLUSTER << c);

#define CLUSTER_BY_CPU(c) ((c >=0) ? ((c < lcluster_cores) ? 0 : 1) : -1)
#define CLUSTER_BY_POLICY(p) ((p > NT) ? ((p < SSTB) ? 0 : 1) : -1)
#else
#define CPU_NUM_BY_CLUSTER(c, f, l) \
	*f = 0; \
    *l = MAX_CORES;
#define CLUSTER_BY_CPU(c) (0)
#define CLUSTER_BY_POLICY(p) ((p > NT) ? 1 : 0)
#endif

spinlock_t load_lock;

/*---------------------------------+
 *      |      little  |    big    |
 * SSTB |     allow    |  restrict |
 * --------------------------------+
 *  NOM |    restrict  |  allow    |
 *  STB |              |           |
 *---------------------------------+
 * 1@return value : allow
 * 0@return value : not allow
 */
int check_current_cpu_owner(int cpu)
{
	int cluster = CLUSTER_BY_CPU(cpu);
	int policy = platform_data.notify_info.cur_policy;

	if (!policy || cluster < 0)
		return 1;

	switch(policy) {
#if (NUM_CLUSTER > 1)
	case SSTB:
		return !cluster;
#endif
	case STB:
	case NOM:
		return cluster;
	default:
		return 1;
    }
    return 1;
}

#ifndef SCHED_BUSY_CPUS_SUPPORT
static unsigned long get_busy(int cpu)
{
#ifdef SCHED_BUSY_SUPPORT
	asdfasdf
	return (u64)sched_get_busy(cpu);
#else
	u64 cur_wall_time, cur_idle_time;
	unsigned int delta_wall_time, delta_idle_time;
	unsigned int busy;
	struct governor_policy_info *pcpu = &per_cpu(policy_info, cpu);

	if (!pcpu)
		return 0;

	cur_idle_time = get_cpu_idle_time(cpu, &cur_wall_time, 0);
	delta_wall_time = (unsigned int)(cur_wall_time - pcpu->prev_wall_time);
	delta_idle_time = (unsigned int)(cur_idle_time - pcpu->prev_idle_time);

	if (!delta_wall_time || delta_wall_time  < delta_idle_time)
		return 0;

	busy = (((delta_wall_time - delta_idle_time) * 100) / delta_wall_time) * 200;
	pcpu->prev_wall_time = cur_wall_time;
	pcpu->prev_idle_time = cur_idle_time;
	return busy;
#endif
}
#endif

void update_cpufreq(int cpu, int freq)
{
	unsigned long flag = 0;
	int cluster = -1, fcpu, lcpu;
	struct sys_cmd_freq_req *pfreq;

	spin_lock_irqsave(&platform_data.frequency_change_lock, flag);
	cluster = CLUSTER_BY_CPU(cpu);
	CPU_NUM_BY_CLUSTER(cluster, &fcpu, &lcpu);

	if (cluster < 0) {
		pr_err(PRM_TAG "cluster is not valid..... -1\n");
		spin_unlock_irqrestore(&platform_data.frequency_change_lock, flag);
		return;
	}

	pfreq = &platform_data.ioctl.freq_per_cluster[cluster];
	pfreq->req_cluster = cluster;
	pfreq->req_cpu = cpu;
	pfreq->req_freq = freq;

	spin_unlock_irqrestore(&platform_data.frequency_change_lock, flag);

	if (IS_ERR_OR_NULL(platform_data.kobject)) {
		pr_err(PRM_TAG "triton kobj not exist\n");
		return;
	}

	switch (cluster) {
	case 0:
		sysfs_notify(platform_data.kobject, NULL, "aevents");
		break;
	case 1:
		sysfs_notify(platform_data.kobject, NULL, "bevents");
		break;
	default:
		break;
	}
}

void update_target_governor_state(bool enabled)
{
	if (lge_prm_get_info(LGE_PRM_INFO_TRITON_ENABLED)) {
		mutex_lock(&platform_data.target_governor_lock);
		platform_data.target_governor_state = enabled;
		mutex_unlock(&platform_data.target_governor_lock);
	}
}

int triton_notify(unsigned int evt, unsigned int cpu, void *v)
{
	int ret = 0;

	switch(evt) {
	case CPU_FREQ_TRANSITION:
		update_cpufreq(cpu, *(int *)v);
		break;
	case CPU_LOAD_TRANSITION:
		break;
	case CPU_OWNER_TRANSITION:
		/* important! */
		ret = check_current_cpu_owner(cpu);
		break;
	case CPU_UPDATE_TARGET_GOVERNOR_START:
		update_target_governor_state(true);
		break;
	case CPU_UPDATE_TARGET_GOVERNOR_STOP:
		update_target_governor_state(false);
		break;
	default:
		break;
	}

	return ret;
}

static int get_dst_cpu(int pol, int *cl)
{
	int cpu, fcpu, lcpu;
	struct device *dev;

	*cl = CLUSTER_BY_POLICY(pol);

	if (*cl < 0)
		return -1;

	CPU_NUM_BY_CLUSTER(*cl, &fcpu, &lcpu);

	for (cpu = fcpu; cpu < lcpu ; cpu++) {
		dev = get_cpu_device(cpu);
		if (dev && !cpu_is_offline(dev->id)) {
			return cpu;
		}
	}
	return -1;
}

static void sysfs_set_noti_data(enum sysfs_notify_id notify)
{
	platform_data.notify_info.notify_id = notify;
	schedule_work(&platform_data.sysfs_wq);
}

static int start_kpolicy(int kpolicy)
{
	unsigned long flag;
#ifdef SCHED_BUSY_CPUS_SUPPORT
	if (num_online_cpus() < MAX_CORES) {
		sysfs_set_noti_data(SYSFS_NOTIFY_CUR_POLICY);
		return 0;
	}
#endif
	spin_lock_irqsave(&platform_data.hotplug_lock, flag);

	platform_data.notify_info.cur_policy = kpolicy;

	if (platform_data.state == FREEZE && kpolicy > NT) {
		platform_data.state = RUNNING;
		platform_data.level = CEILING;
		queue_delayed_work(platform_data.fwq,
					&platform_data.frequency_changed_wq,
					usecs_to_jiffies(platform_data.ioctl.tunables_param.tunable_timer_rate_us));

		sysfs_set_noti_data(SYSFS_NOTIFY_CUR_POLICY);
	}

	spin_unlock_irqrestore(&platform_data.hotplug_lock, flag);

	return 0;
}

static int stop_kpolicy(void)
{
	int last_policy, cl = -1, dest_cpu = -1;
	struct device *dev;

	if (!platform_data.notify_info.cur_policy ||
			platform_data.state == FREEZE)
		return 0;

	last_policy = platform_data.notify_info.cur_policy;
	platform_data.notify_info.cur_policy = NT;
	platform_data.state = FREEZE;
	platform_data.level = CEILING;
	cancel_delayed_work(&platform_data.frequency_changed_wq);

	/* confirmed stop */
	sysfs_set_noti_data(SYSFS_NOTIFY_CUR_POLICY);

	/* restore frequency of last policy */
	dest_cpu = get_dst_cpu(last_policy, &cl);
	if (dest_cpu < 0)
		return -EFAULT;

	dev = get_cpu_device(dest_cpu);
	if (dev && !cpu_is_offline(dev->id)) {
		sugov_restore_freq(dest_cpu);
	}

	return 0;
}

static int set_dst_cpu(int freq, int cpu)
{
	struct device *dev = get_cpu_device(cpu);
	struct cpufreq_policy *policy = sugov_get_policy(cpu);
	struct governor_policy_info *pcpu = &per_cpu(policy_info, cpu);

	if (!dev || cpu_is_offline(dev->id))
		goto noti;

	if (!policy)
		goto noti;

	if (!policy->governor_data)
		goto noti;

	if (policy != pcpu->policy) {
		pcpu->policy = policy;
		goto noti;
	}

	if (!sugov_get_stat(cpu)) {
		if (policy->cur != freq) {
			pr_debug(PRM_TAG "[TT] cpu %d, freq : %d\n", cpu, freq);
			cpufreq_driver_target(policy, freq, CPUFREQ_RELATION_H);
		}
		return 0;
	}

noti:
	return -EINVAL;
}

#define ABS_DIFF(x,y) ((x > y)? (x-y) : (y-x))
static int get_opt_frequency(int cl, int lvl)
{
	int cpu, lcpu, fcpu, freq = 0, actual = 0;
	unsigned int load[CORES_PER_CLUSTER];
	unsigned int max = 0, min = UINT_MAX;
	unsigned int diff, /*coeff, */ratio, dyn_range;
	struct sys_cmd_tunables_bmc_req *bmc_param;
	u32 calcfreq;
#ifdef SCHED_BUSY_CPUS_SUPPORT
	struct sched_load schedl[CORES_PER_CLUSTER];
	struct cpufreq_policy *pol;
	int i = 0;
	unsigned long flags[2] = {0, 0};
#endif
	bmc_param = &platform_data.ioctl.tunables_bmc_param[cl];
	freq = platform_data.ioctl.freq_per_cluster[cl].req_freq;

	if (freq < bmc_param->minturbo)
		return bmc_param->minturbo;

	CPU_NUM_BY_CLUSTER(cl, &fcpu, &lcpu);
#ifdef SCHED_BUSY_CPUS_SUPPORT

	pol = sugov_get_policy(fcpu);

	if (!pol) {
		pr_err(PRM_TAG "policy is NULL skip this\n");
		return freq;
	}

	spin_lock_irqsave(&load_lock, flags[0]);

	sugov_lock(fcpu, flags[1]);
	sched_get_cpus_busy(&schedl[0], pol->cpus);
	sugov_unlock(fcpu, flags[1]);

	spin_unlock_irqrestore(&load_lock, flags[0]);

	for (cpu = fcpu; cpu < lcpu; cpu++) {
		if (!cpu_is_offline(cpu)) {
			if (schedl[i].prev_load > 20000)
				schedl[i].prev_load = 20000;

			ratio = (schedl[i].prev_load * 100) / 20000;
			load[i] = (unsigned int)ratio;

			if (load[i] < min)
				min = load[i];
			if (load[i] > max)
				max = load[i];
			i++;
		}
	}
#else
	for (cpu = fcpu; cpu < lcpu; cpu++) {
		unsigned long busy = 0;

		if (cpu_is_offline(cpu)){
			pr_debug("cpu%d offlined \n", cpu);
			return -1;
		}

		busy = get_busy(cpu);
		ratio = (busy * 100) / 20000;

		load[cpu-fcpu] = (unsigned int)ratio;

		if (load[cpu-fcpu] < min)
			min = load[cpu-fcpu];

		if (load[cpu-fcpu] > max)
			max = load[cpu-fcpu];
	}
#endif
	diff = ABS_DIFF(max, min);

	dyn_range = bmc_param->maxturbo - bmc_param->minturbo;

	calcfreq = (dyn_range * diff) /100;

	actual = (int)(freq - calcfreq);

	if (actual > bmc_param->maxturbo)
		actual = bmc_param->maxturbo;
#ifndef TRITON_KERNEL_4P14
	trace_sugov_triton_level(platform_data.notify_info.cur_policy,
				max, min, diff, freq, actual);
#endif
	return actual;
}

static int get_dst_freq(int dest_cpu, int lvl)
{
	int cl = -1;

	struct device *dev;

	cl = CLUSTER_BY_CPU(dest_cpu);
	if (cl < 0)
		return -1;

	dev = get_cpu_device(dest_cpu);
	if (!dev || cpu_is_offline(dev->id)) {
		pr_err(PRM_TAG "%s cpu%d is not valid\n", __func__, dest_cpu);
		return -1;
	}

	if (sugov_get_stat(dest_cpu)) {
		pr_err(PRM_TAG "%s gov %d is not valid\n", __func__, dest_cpu);
		return -1;
	}

	return get_opt_frequency(cl, lvl);
}

static void sysfs_noti_process(struct work_struct *sysfs_noti_work)
{
	struct triton_platform_data *platform_data = container_of(sysfs_noti_work,
							struct triton_platform_data,
							sysfs_wq);

	switch(platform_data->notify_info.notify_id) {
	case SYSFS_NOTIFY_CUR_POLICY:
		sysfs_notify(platform_data->kobject, NULL, "cur_policy");
		break;
	case SYSFS_NOTIFY_AFREQ:
		sysfs_notify(platform_data->kobject, NULL, "aevents");
		break;
	case SYSFS_NOTIFY_BFREQ:
		sysfs_notify(platform_data->kobject, NULL, "bevents");
		break;
	case SYSFS_NOTIFY_ENABLE:
		sysfs_notify(platform_data->kobject, NULL, "enable");
		break;
	case SYSFS_NOTIFY_ENFORCE:
		sysfs_notify(platform_data->kobject, NULL, "enforce");
		break;
	case SYSFS_NOTIFY_DEBUG:
		sysfs_notify(platform_data->kobject, NULL, "debug");
		break;
	}
}

static void ping_process(struct work_struct *work)
{
	int ping_exp = platform_data.ioctl.tunables_param.tunable_ping_exp;
	int ping_intv = platform_data.ioctl.tunables_param.tunable_ping_intv;

	if (ping_exp == 0)
		ping_exp = DEFAULT_PING_EXP;

	if (ping_intv == 0)
		ping_intv = DEFAULT_PING_INTV;

	last_ping_time = ktime_to_ms(ktime_get());
	if (last_ping_time - ping_time >= ping_exp) {
		ping_time = last_ping_time;
		ping_expired = 1;
		stop_kpolicy();
	}

	queue_delayed_work(platform_data.ping_fwq,
					&platform_data.ping_wq,
					msecs_to_jiffies(ping_intv));
}

static void frequency_process(struct work_struct *work)
{
	struct triton_platform_data *platform_data = container_of(work,
				struct triton_platform_data,
				frequency_changed_wq.work);
	int dst_cpu = -1, cl = -1;
	int opt_freq = 0;

	if (!platform_data)
		goto exit;

	mutex_lock(&(platform_data->target_governor_lock));

	if (!(platform_data->target_governor_state)) {
		pr_debug(PRM_TAG "target governor is not operating state(%s)\n",
				platform_data->target_governor_state ? "true" : "false");
		goto exit;
	} else {
		pr_debug(PRM_TAG "target governor is operating state(%s)\n",
				platform_data->target_governor_state ? "true" : "false");
	}

	if (!platform_data->notify_info.enable)
		goto exit;

	if (!(platform_data->state & RUNNING))
		goto exit;

	get_online_cpus();

	dst_cpu = get_dst_cpu(platform_data->notify_info.cur_policy, &cl);
	if (dst_cpu < 0) {
		pr_debug(PRM_TAG "dst cpu(%d) is not valid\n", dst_cpu);
		goto exit_cpu;
	}

	opt_freq = get_dst_freq(dst_cpu,
						platform_data->level);

	if (opt_freq < 0)
		goto exit_cpu;

	if (!set_dst_cpu(opt_freq, dst_cpu)){
		platform_data->level ^= 1;
		goto rearm;
	}
	goto exit_cpu;

rearm:
	put_online_cpus();
	mutex_unlock(&(platform_data->target_governor_lock));
	queue_delayed_work(platform_data->fwq,
			&platform_data->frequency_changed_wq,
			usecs_to_jiffies(platform_data->ioctl.tunables_param.tunable_timer_rate_us));

	return;

exit_cpu:
	stop_kpolicy();
	put_online_cpus();
	mutex_unlock(&(platform_data->target_governor_lock));
	return;

exit:
	stop_kpolicy();

	if (platform_data)
		mutex_unlock(&(platform_data->target_governor_lock));
	return;
}

static long validate(unsigned int *cmd, unsigned long *arg)
{
	long ret = 0, err_val = 0;

	if ((_IOC_TYPE(*cmd) != MAGIC_NUM)) {
		return -EFAULT;
	}

	if (_IOC_NR(*cmd) >= REQ_MAX) {
		return -EFAULT;
	}

	if (_IOC_DIR(*cmd) & _IOC_READ) {
		err_val = !access_ok(VERIFY_WRITE, (void __user *)*arg,
				_IOC_SIZE(*cmd));
	} else if (_IOC_DIR(*cmd) & _IOC_WRITE) {
		err_val = !access_ok(VERIFY_READ, (void __user *)*arg,
				_IOC_SIZE(*cmd));
	}

	if (err_val)
		return -EFAULT;

	return ret;
}


static int update_commons(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret = 0;
	int request, value;

	ret = copy_from_user(&platform_data.ioctl.common_req, argp,
			sizeof(struct sys_cmd_comm_req));
	if (ret)
		return -ENOTTY;

	request = platform_data.ioctl.common_req.req;
	value = platform_data.ioctl.common_req.val;

	switch(request) {
		/* policy */
	case COMM_POLICY:
		if (!value) {
			stop_kpolicy();
			break;
		}

		start_kpolicy(value);
		break;
	/* enable */
	case COMM_ENABLE:
		platform_data.notify_info.enable = value;
		break;
	/* enforce */
	case COMM_ENFORCE:
		platform_data.notify_info.enforce = value;
		break;
	default:
		break;
	}
	return ret;
}

static int update_sys_frequency
		(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret = 0;
	struct sys_cmd_freq_req freq;

	ret = copy_from_user(&freq,
				argp,
				sizeof(struct sys_cmd_freq_req));
	if (ret)
		return -ENOTTY;

	if ((freq.req_cluster < 0) || (freq.req_cluster >= NUM_CLUSTER))
		return -ENOTTY;

	ret = copy_to_user((void __user*)arg,
			&platform_data.ioctl.freq_per_cluster[freq.req_cluster],
		    sizeof(struct sys_cmd_freq_req));

	return ret;
}

static int update_sys_tunables(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret = 0;

	ret = copy_from_user(&platform_data.ioctl.tunables_param,
			             argp,
			             sizeof(struct sys_cmd_tunables_req));
	if (ret)
		return -ENOTTY;

	return ret;
}
static int update_sys_opt_tunables(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret = 0;

	ret = copy_from_user(&platform_data.ioctl.tunables_bmc_param,
			         argp,
			         sizeof(struct sys_cmd_tunables_bmc_req) * NUM_CLUSTER);
	if (ret)
		return -ENOTTY;

	return ret;

}
static int update_sys_perf_level(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret = 0;

	ret = copy_from_user(&platform_data.ioctl.perf_param,
			argp,
			sizeof(struct sys_cmd_perf_level_req) * BIT_MAX);
	if (ret)
		return -ENOTTY;

	return ret;
}

static long io_progress(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	long ret = 0;

	ret = validate(&cmd, &arg);

	if (ret) {
		return -EFAULT;
	}

	switch(cmd) {
	case IOCTL_AFREQ_REQ:
	case IOCTL_BFREQ_REQ:
		ret = update_sys_frequency(arg);
		break;
	case IOCTL_COMM_REQ:
		ret = update_commons(arg);
		break;
	case IOCTL_PERF_LEVEL_REQ:
		ret = update_sys_perf_level(arg);
		break;
	case IOCTL_BASIC_TUNABLE_REQ:
		ret = update_sys_tunables(arg);
		break;
	case IOCTL_TUNALBE_BMC_REQ:
		update_sys_opt_tunables(arg);
		break;
	case IOCTL_PING_REQ:
		ping_expired = 0;
		ping_time = ktime_to_ms(ktime_get());
		break;
	default:
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long compat_io_progress(struct file *filep,
		unsigned int cmd, unsigned long arg)
{
	arg = (unsigned long)compat_ptr(arg);
	return io_progress(filep, cmd, arg);
}
#endif

static int io_release(struct inode *node, struct file *filep)
{
	return 0;
}

static int io_open(struct inode *node, struct file *filep)
{
	int ret = 0;
	struct io_dev *dev;

	dev = container_of(node->i_cdev, struct io_dev,
			char_dev);
	filep->private_data = dev;

	return ret;
}

#ifndef TRITON_KERNEL_4P14
static int cpu_stat_callback(struct notifier_block *nfb,
					unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;
	struct device *dev;

	dev = get_cpu_device(cpu);
	if (dev) {
		switch (action & ~CPU_TASKS_FROZEN) {
		case CPU_DOWN_PREPARE:
		case CPU_ONLINE:
#ifndef CORE_CONTROL
			stop_kpolicy();
#endif
			break;
		default:
			break;
		}
	}
	return NOTIFY_OK;
}

static struct notifier_block __refdata cpu_stat_notifier = {
	.notifier_call = cpu_stat_callback,
};
#endif

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = io_open,
	.unlocked_ioctl = io_progress,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_io_progress,
#endif
	.release = io_release,
};

static int ioctl_init(void)
{
	int ret = 0;
	dev_t	io_dev_main;
	struct device *io_device;

	ret = alloc_chrdev_region(&io_dev_main, 0, 1, IOCTL_PATH);
	if (ret < 0) {
		pr_err(PRM_TAG "error in allocation device region\n");
		goto ioctl_init_exit;
	}

	platform_data.major = MAJOR(io_dev_main);
	platform_data.class = class_create(THIS_MODULE, "class_triton");

	if (IS_ERR(platform_data.class)) {
		pr_err(PRM_TAG "error in creating class triton\n");
		ret = PTR_ERR(platform_data.class);
		goto ioctl_class_fail;
	}

	io_device = device_create(platform_data.class,
				              NULL,
				              io_dev_main,
				              NULL,
				              IOCTL_PATH);
	if (IS_ERR(io_device)) {
		pr_err(PRM_TAG "error in creating triton device\n");
		ret = PTR_ERR(io_device);
		goto ioctl_dev_fail;
	}

	platform_data.tio_dev = kmalloc(sizeof(struct io_dev), GFP_KERNEL);
	if (!platform_data.tio_dev) {
		pr_err(PRM_TAG "error in allocation memory\n");
		ret = -ENOMEM;
		goto ioctl_clean_all;
	}

	memset(platform_data.tio_dev, 0, sizeof(struct io_dev));
	sema_init(&platform_data.tio_dev->sem, 1);
	cdev_init(&platform_data.tio_dev->char_dev, &fops);

	ret = cdev_add(&platform_data.tio_dev->char_dev, io_dev_main, 1);
	if (ret < 0) {
		pr_err(PRM_TAG "Error in adding character device\n");
		goto ioctl_clean_all;
	}

	return ret;

ioctl_clean_all:
	device_destroy(platform_data.class, io_dev_main);
ioctl_dev_fail:
	class_destroy(platform_data.class);
ioctl_class_fail:
	unregister_chrdev_region(io_dev_main, 1);
ioctl_init_exit:
	return ret;
}
/*
static void cleanup_device(void)
{
	dev_t dev = MKDEV(platform_data.major, 0);

	if (!platform_data.tio_dev)
		return;
	device_destroy(platform_data.class, dev);
	class_destroy(platform_data.class);
	unregister_chrdev_region(dev, 1);
	kfree(platform_data.tio_dev);
}
*/
ssize_t lge_triton_cur_policy_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%u\n", platform_data.notify_info.cur_policy);
	return ret;
}

ssize_t lge_triton_aevents_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%u\n", platform_data.notify_info.aevents);
	return ret;
}

ssize_t lge_triton_bevents_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%u\n", platform_data.notify_info.bevents);
	return ret;
}

ssize_t lge_triton_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%u\n", platform_data.notify_info.enable);
	return ret;
}

ssize_t lge_triton_enforce_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%u\n", platform_data.notify_info.enforce);
	return ret;
}

ssize_t lge_triton_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%u\n", platform_data.notify_info.debug);
	return ret;
}


ssize_t lge_triton_cur_policy_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	platform_data.notify_info.cur_policy = input;
	sysfs_set_noti_data(SYSFS_NOTIFY_CUR_POLICY);

	return count;
}

ssize_t lge_triton_aevents_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	return count;
}

ssize_t lge_triton_bevents_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	return count;
}

ssize_t lge_triton_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	platform_data.notify_info.enable = input;
	sysfs_set_noti_data(SYSFS_NOTIFY_ENABLE);

	return count;
}

ssize_t lge_triton_enforce_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	platform_data.notify_info.enforce = input;
	sysfs_set_noti_data(SYSFS_NOTIFY_ENFORCE);

	return count;
}

ssize_t lge_triton_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	platform_data.notify_info.debug = input;
	sysfs_set_noti_data(SYSFS_NOTIFY_DEBUG);

	return count;
}

static DEVICE_ATTR(cur_policy, S_IRUGO | S_IWUSR,
		lge_triton_cur_policy_show, lge_triton_cur_policy_store);
static DEVICE_ATTR(aevents, S_IRUGO | S_IWUSR,
		lge_triton_aevents_show, lge_triton_aevents_store);
static DEVICE_ATTR(bevents, S_IRUGO | S_IWUSR,
		lge_triton_bevents_show, lge_triton_bevents_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
		lge_triton_enable_show, lge_triton_enable_store);
static DEVICE_ATTR(enforce, S_IRUGO | S_IWUSR,
		lge_triton_enforce_show, lge_triton_enforce_store);
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR,
		lge_triton_debug_show, lge_triton_debug_store);

static struct attribute *triton_fs_attrs[] = {
	&dev_attr_cur_policy.attr,
	&dev_attr_aevents.attr,
	&dev_attr_bevents.attr,
	&dev_attr_enable.attr,
	&dev_attr_enforce.attr,
	&dev_attr_debug.attr,
	NULL
};

static struct attribute_group triton_fs_attrs_group = {
	.attrs = triton_fs_attrs,
};

int lge_triton_sysfs_create(void)
{
	int ret = 0;
	struct class *cn = NULL;
	static struct device *sysfs_dev = NULL;

	ret = lge_prm_get_class_node(&cn);
	if (ret)
		return -EPROBE_DEFER;

	if (!sysfs_dev) {
		sysfs_dev = device_create(cn, NULL, 0, NULL, "triton");
		if (IS_ERR_OR_NULL(sysfs_dev)) {
			pr_err(PRM_TAG "Failed to create triton sysfs_dev\n");
			return -ENODEV;
		}

		ret = sysfs_create_group(&sysfs_dev->kobj, &triton_fs_attrs_group);
		if (ret) {
			pr_err(PRM_TAG "Failed to create triton sub_sysfs\n");
			return -ENODEV;
		}
	} else {
		pr_err(PRM_TAG "triton sysfs_dev already exist\n");
	}

	platform_data.kobject = &sysfs_dev->kobj;
	g_sysfs_dev = sysfs_dev;
	return 0;
}

int lge_triton_init(void)
{
	int ret = 0;
	int i;
	struct governor_policy_info *pcpu;

#if (NUM_CLUSTER > 1)
	int csiblings[MAX_CPUS_DEFLT] = {-1,};

	for (i = 0; i < MAX_CORES; i++) {
		csiblings[i]= topology_physical_package_id(i);
	}

	for (i = 0; i < MAX_CORES; i++) {
		if (csiblings[i] == 0)
			lcluster_cores++;
		else if (csiblings[i] == 1)
			bcluster_cores++;
	}

	lcluster_start = 0;
	bcluster_start = lcluster_cores;
#endif



	ret = lge_triton_sysfs_create();
	if (ret) {
		pr_err(PRM_TAG "Error triton sysfs creation, %d\n", ret);
		return ret;
	}

	for_each_possible_cpu(i) {
		pcpu = &per_cpu(policy_info, i);
		init_rwsem(&pcpu->sem);
	}

	platform_data.fwq = alloc_workqueue("t:fwq", WQ_HIGHPRI, 0);
	platform_data.ping_fwq = alloc_workqueue("t:pfwq", WQ_HIGHPRI, 0);

	INIT_DELAYED_WORK(&platform_data.frequency_changed_wq, frequency_process);
	INIT_DELAYED_WORK(&platform_data.ping_wq, ping_process);
	INIT_WORK(&platform_data.sysfs_wq, sysfs_noti_process);
	spin_lock_init(&platform_data.hotplug_lock);
	spin_lock_init(&platform_data.frequency_change_lock);
	spin_lock_init(&load_lock);
	mutex_init(&platform_data.target_governor_lock);

	ioctl_init();

#ifndef TRITON_KERNEL_4P14
	register_hotcpu_notifier(&cpu_stat_notifier);
#endif

	platform_data.state = FREEZE;
	platform_data.level = 0;
	platform_data.notify_info.cur_policy = 0;

	queue_delayed_work(platform_data.ping_fwq,
				&platform_data.ping_wq,
				msecs_to_jiffies(DEFAULT_PING_INTV * 5));


	pr_err(PRM_TAG "LGE TRITON probe done\n");
	return ret;
}

