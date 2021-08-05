/*
 * driver/power/lge_pon_backup.c
 *
 * Copyright (C) 2019 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/fs_struct.h>
#include <linux/platform_device.h>
#define MODULE_NAME "lge_pon_backup"

struct lge_pon_backup_data {
	uint32_t smpl_boot;
};

#define PON_DELAY_MS 10000

#define FTM_BLOCK_SIZE		4096
#define FTM_PON_BACKUP_OFFSET 227
#define FTM_PON_BACKUP_SIZE	488

#define PON_BACKUP_MAX_COUNT 10
#define PON_BACKUP_MAX_PMIC 3
#define PON_BACKUP_MAX_REASON 2
#define FTM_PATH "/dev/block/bootdevice/by-name/ftm"

static const char * const qpnp_pon_reason_groups[] = {
	" PON=", " 0xC1=", " WR=", " 0xC3=", " ON=", " POFF=", " 0xC6=", " OFF="
};

static const char * const qpnp_pon_reasons[] = {
	"HR:", "SMPL:", "RTC:", "DC:", "USB:", "PON1:", "CBL:", "KPD:",
	"0:", "1:", "2:", "3:", "4:", "5:", "6:", "7:",
	"SOFT:", "PS_HOLD:", "PMIC_WD:", "GP1:", "GP2:", "KPD&RESIN:", "RESIN:", "KPD:",
	"0:", "1:", "2:", "3:", "4:", "5:", "6:", "7:",
	"0:", "1:", "2:", "3:", "4:", "5:", "WARM:", "PON:",
	"SOFT:", "PS_HOLD:", "PMIC_WD:", "GP1:", "GP2:", "KPD&RESIN:", "RESIN:", "KPD:",
	"0:", "1:", "2:", "3:", "4:", "5:", "6:", "7:",
	"0:", "1:", "RAW_XVDD:", "RAW_DVDD:", "IM_XVDD:", "S3_REST:", "FAULT:", "POFF:"
};

static const char * const qpnp_fault_reason_groups[] = {
	" FAULT1=", " FAULT2=", " S3=", " SOFT="
};

static const char * const qpnp_fault_reasons[] = {
	"GP_F0:", "GP_F1:", "GP_F2:", "GP_F3:", "MBG:", "OVLO:", "UVLO:", "AVDD:",
	"0:", "1:", "2:", "FAULT:", "PBS_WD:", "PBS_NACK:", "RESTART:", "OTST3:",
	"0:", "1:", "2:", "3:", "FAULT:", "PBS_WD:", "PBS_NACK:", "KPD&RESIN:",
	"SOFT:", "1:", "2:", "3:", "4:", "5:", "6:", "7:"
};

typedef struct {
  uint32_t count;
  uint64_t pon_reg[PON_BACKUP_MAX_COUNT][PON_BACKUP_MAX_PMIC][PON_BACKUP_MAX_REASON];
} pon_backup_type;

static int lge_get_pon_backup(pon_backup_type *now_pon)
{
	struct file *fp;
	struct path root;
	int pon_count = 0, cnt = 0;

	mm_segment_t old_fs=get_fs();
	pr_info("[%s %d] lge_pon_backup_func start\n", __func__, __LINE__);
	set_fs(KERNEL_DS);

	memset(&root, 0, sizeof(struct path));
	task_lock(&init_task);
	get_fs_root(init_task.fs, &root);
	task_unlock(&init_task);
	fp = file_open_root(root.dentry, root.mnt, FTM_PATH, O_RDWR, 0);
	path_put(&root);

	if(IS_ERR(fp)){
		pr_err("%s : Unable to open FTM (%d)\n", __func__, PTR_ERR(fp));
		pon_count = -1;
		goto err;
	}

	fp->f_pos = FTM_PON_BACKUP_OFFSET * FTM_BLOCK_SIZE; // LGFTM_PON_BACK_UP offset
	cnt = vfs_read(fp,(char*)now_pon, sizeof(*now_pon),&fp->f_pos);
	if(cnt != sizeof(*now_pon)){
		pr_err("%s : Unable to read FTM (%d)\n", __func__, cnt);
		pon_count = -1;
		goto err;
	}

	if (now_pon->count >= PON_BACKUP_MAX_COUNT)
		pon_count = PON_BACKUP_MAX_COUNT;
	else
		pon_count = now_pon->count;

	pr_info("[%s %d] FTM Size %d, pon count %d",
		__func__, __LINE__, sizeof(*now_pon), now_pon->count);
	now_pon->count = 0;
	fp->f_pos = FTM_PON_BACKUP_OFFSET * FTM_BLOCK_SIZE; // LGFTM_PON_BACK_UP offset
	cnt = vfs_write(fp, (char*)now_pon, sizeof(*now_pon), &fp->f_pos);
	if(cnt != sizeof(*now_pon)) {
		pr_err("%s : Unable to write FTM (%d)\n", __func__, cnt);
		goto err;
	}
err:
	if (!IS_ERR(fp))
		filp_close(fp, NULL);

	set_fs(old_fs);

	return pon_count;
}

#define PM_STATUS_MSG_LEN 300
static void lge_pon_backup_func(struct work_struct *w)
{
	pon_backup_type now_pon = {0, };
	int i, j, count, index = 0, pre_index = -8;
	char buf[PM_STATUS_MSG_LEN] = "";
	uint64_t pon_reg, fault_reg;

	count = lge_get_pon_backup(&now_pon);
	if (count < -1)
		return;

	for (i = 0; i < count; i++)
		for (j = 0; j < PON_BACKUP_MAX_PMIC; j++) {
			pon_reg = now_pon.pon_reg[i][j][0];
			fault_reg = now_pon.pon_reg[i][j][1];
			pr_debug("Pon reg boot[%d] pm%d=0x%llx:0x%llx\n", i, j, pon_reg, fault_reg);

			while (pon_reg != 0 && index < sizeof(qpnp_pon_reasons)/sizeof(char*) ) {
				if (pon_reg & 1) {
					if (index/8 != pre_index/8)
						strlcat(buf, qpnp_pon_reason_groups[index/8], PM_STATUS_MSG_LEN);

					strlcat(buf, qpnp_pon_reasons[index], PM_STATUS_MSG_LEN);
					pre_index = index;
				}
				pon_reg = pon_reg >> 1;
				index++;
			}

			index = 0;
			pre_index = -8;
			while (fault_reg != 0 && index < sizeof(qpnp_fault_reasons)/sizeof(char*)) {
				if (fault_reg & 1) {
					if (index/8 != pre_index/8)
						strlcat(buf, qpnp_fault_reason_groups[index/8], PM_STATUS_MSG_LEN);

					strlcat(buf, qpnp_fault_reasons[index], PM_STATUS_MSG_LEN);
					pre_index = index;
				}
				fault_reg = fault_reg >> 1;
				index++;
			}
			pr_info("power on/off reason boot[%d] pm%d=%s\n", i, j, buf);
			sprintf(buf, "");
			index = 0;
			pre_index = -8;
		}

	return;
};
static DECLARE_DELAYED_WORK(pon_backup_work, lge_pon_backup_func);

static int lge_pon_backup_probe(struct platform_device *pdev)
{
	pr_err("start Pon BackUp\n");
	schedule_delayed_work(&pon_backup_work, msecs_to_jiffies(PON_DELAY_MS));

	return 0;
}

static int lge_pon_backup_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver lge_pon_backup_driver = {
	.probe = lge_pon_backup_probe,
	.remove = lge_pon_backup_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static struct platform_device lge_pon_backup_device = {
	.name = MODULE_NAME,
	.dev = {
		.platform_data = NULL,
	}
};

static int __init lge_pon_backup_init(void)
{
	platform_device_register(&lge_pon_backup_device);

	return platform_driver_register(&lge_pon_backup_driver);
}

static void __exit lge_pon_backup_exit(void)
{
	platform_driver_unregister(&lge_pon_backup_driver);
}

module_init(lge_pon_backup_init);
module_exit(lge_pon_backup_exit);

MODULE_DESCRIPTION("LGE pon backup");
MODULE_LICENSE("GPL");