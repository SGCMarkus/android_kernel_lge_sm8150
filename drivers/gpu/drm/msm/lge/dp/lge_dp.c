#define pr_fmt(fmt)	"[Display][DP:%s:%d] " fmt, __func__, __LINE__

#ifdef CONFIG_LGE_DISPLAY_NOT_SUPPORT_DISPLAYPORT
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include "lge_dp.h"
#include <linux/err.h>

struct class *dp_noti_class;

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct dp_noti_dev *ndev = (struct dp_noti_dev *)
		dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ndev->state);
}

static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);

void dp_noti_set_state(struct dp_noti_dev *ndev, int state)
{
	char *envp[2];
	char name_buf[30];

	if (ndev->state != state) {
		ndev->state = state;
		snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s%c", ndev->name, ndev->state?'1':'0');
		envp[0] = name_buf;
		envp[1] = NULL;
		kobject_uevent_env(&ndev->dev->kobj, KOBJ_CHANGE, envp);
		pr_info("check_dp_notify,name_buf = %s state = %d,\n", name_buf, ndev->state);
	}
}

void lge_dp_set_id(unsigned int id) {
	pr_debug("not supported\n");
	return;
}

static int create_dp_noti_class(void)
{
	if (!dp_noti_class) {
		dp_noti_class = class_create(THIS_MODULE, "dp_notify");
		if (IS_ERR(dp_noti_class))
			return PTR_ERR(dp_noti_class);
	}

	return 0;
}

int dp_noti_register(struct dp_noti_dev *ndev)
{
	int ret;

	if (!ndev) {
		pr_err("%s : dp register failed\n", __func__);
		return -EINVAL;
	}

	ndev->dev = device_create(dp_noti_class, NULL,
			MKDEV(0, 1), NULL, ndev->name);
       if (IS_ERR(ndev->dev))
	       return PTR_ERR(ndev->dev);

       ret = device_create_file(ndev->dev, &dev_attr_state);
       if (ret < 0)
			goto err1;

       dev_set_drvdata(ndev->dev, ndev);
       ndev->state = 0;
       return 0;

err1:
       device_remove_file(ndev->dev, &dev_attr_state);
       return ret;
}

void dp_noti_unregister(struct dp_noti_dev *ndev)
{
	device_remove_file(ndev->dev, &dev_attr_state);
	dev_set_drvdata(ndev->dev, NULL);
	device_destroy(dp_noti_class, MKDEV(0, 1));
}

static int __init dp_noti_class_init(void)
{
	return create_dp_noti_class();
}

static void __exit dp_noti_class_exit(void)
{
	class_destroy(dp_noti_class);
}

module_init(dp_noti_class_init);
module_exit(dp_noti_class_exit);

#else // DP support

#include "lge_dp.h"
#include "lge_dp_def.h"
#include "dp_display.h"

extern struct lge_dp_display* get_lge_dp(void);

static ssize_t lge_dp_hpd_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dp_display *dp;

	dp = dev_get_drvdata(dev);

	if (!dp) {
		pr_err("dp is NULL\n");
		return -EINVAL;
	}
	return sprintf(buf, "%d\n", dp->lge_dp.hpd_state);
}
static DEVICE_ATTR(dp_hpd, S_IRUGO, lge_dp_hpd_show, NULL);

static ssize_t lge_dp_external_block(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct lge_dp_display *lge_dp = NULL;
	int rc = 0;
	int block = -1;

	lge_dp = get_lge_dp();
	if (!lge_dp) {
		pr_err("lge_dp is nullptr\n");
		return -EINVAL;
	}

	rc = kstrtoint(buf, 2, &block);
	if (rc) {
		pr_err("kstrtoint failed, ret=%d\n", rc);
		return rc;
	}
	block = !!block;

	if (lge_dp->block_state == block) {
		pr_warn("reqeust duplicated request\n");
		return -EINVAL;
	}

	rc = dp_display_external_block(lge_dp, block);
	if (rc < 0) {
		pr_err("invalid request\n");
		return -EINVAL;
	} else {
		pr_info("External Block [%s]\n", ((lge_dp->block_state == 1) ? "ON" : "OFF"));
	}

	return count;
}
static DEVICE_ATTR(external_block, S_IWUSR | S_IWGRP, NULL, lge_dp_external_block);

void lge_dp_set_id(unsigned int id)
{
	struct lge_dp_display *lge_dp = NULL;

	lge_dp = get_lge_dp();
	if (IS_ERR_OR_NULL(lge_dp)) {
		pr_err("lge_dp is nullptr\n");
		return;
	}

	lge_dp->vid_pid = id;
	pr_info("attached id = 0x%08x\n", lge_dp->vid_pid);

	if (dp_display_send_id_event(lge_dp) < 0) {
		pr_warn("failed to send id event\n");
	}

	return;
}

static void lge_dp_create_sysfs(struct dp_display *dp_display)
{
	static struct class *class_dp = NULL;
	static struct device *dp_sysfs_dev = NULL;

	if(!class_dp) {
		class_dp = class_create(THIS_MODULE, "display_port");
		if (IS_ERR(class_dp)) {
			pr_err("Failed to create DisplayPort class\n");
		} else {
			if(!dp_sysfs_dev) {
				dp_sysfs_dev = device_create(class_dp, NULL, 0, dp_display, "common");
				if (IS_ERR(dp_sysfs_dev)) {
					pr_err("Failed to create dev(dp_sysfs_dev)!\n");
				} else {
					if ((device_create_file(dp_sysfs_dev, &dev_attr_dp_hpd)) < 0)
						pr_err("add dp_hpd node failed!\n");
					if ((device_create_file(dp_sysfs_dev, &dev_attr_external_block)) < 0)
						pr_err("add external_block node failed!\n");
				}
			}
		}
	}

}

void lge_dp_drv_init(struct dp_display *dp_display)
{
	lge_dp_create_sysfs(dp_display);
}

void lge_set_dp_hpd(struct dp_display *dp_display, int value)
{
	dp_display->lge_dp.hpd_state = value;
}
#endif
