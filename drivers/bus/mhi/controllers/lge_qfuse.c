/*
 * * Copyright (C) 2018 LGE, Inc
 * *
 * * This software is licensed under the terms of the GNU General Public
 * * License version 2, as published by the Free Software Foundation, and
 * * may be copied, distributed, and modified under those terms.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * * GNU General Public License for more details.
 * */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/fs_struct.h>
#include <linux/path.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include "lge_qfuse.h"

bool g_cpFused = false;

static u32 qfprom_hw_key_read() {
	void __iomem *value_addr = NULL;
	u32 value = 0;

	value_addr = ioremap(QFPROM_HW_KEY_STATUS, sizeof(u32));

	if (value_addr != NULL) {
		value = (u32)readl(value_addr);
		iounmap(value_addr);
	}

	return value;
}

static int is_ap_fused(void)
{
	int ret = RET_ERR;

	if((qfprom_hw_key_read()&SEC_KEY_DERIVATION_BLOWN) == SEC_KEY_DERIVATION_BLOWN) {
		ret = RET_OK;
	}

	return ret;
}

static int is_cp_fused(void)
{
	struct file *fp;
	struct path root;
	int cnt = 0;
	u32 ret = RET_OK;
	char ftm_buf[32] = {0,};

	mm_segment_t old_fs=get_fs();

	set_fs(KERNEL_DS);

	memset(&root, 0, sizeof(struct path));
	task_lock(&init_task);
	get_fs_root(init_task.fs, &root);
	task_unlock(&init_task);
	fp = file_open_root(root.dentry, root.mnt, FTM_PATH, O_RDONLY, 0);
	path_put(&root);

	if(IS_ERR(fp)){
		printk(KERN_ERR "%s : Unable to open FTM (%d)\n", __func__, PTR_ERR(fp));
		ret = RET_ERR;
		goto err;
	}

	fp->f_pos = 0x99000; // LGFTM_FUSION_QFUSE_CHECK offset : 153 * 4096
	cnt = vfs_read(fp,(char*)ftm_buf, sizeof(ftm_buf),&fp->f_pos);
	if(cnt != sizeof(ftm_buf)){
		printk(KERN_ERR "%s : Unable to read FTM (%d)\n", __func__, cnt);
		ret = RET_ERR;
		goto err;
	}

	ret = strncmp(ftm_buf, qfuse_status_str[QFUSE_ALREADY_BLOWNED], strlen(qfuse_status_str[QFUSE_ALREADY_BLOWNED]))?1:0;

	printk(KERN_INFO "%s : return value (%d)\n", __func__, ret);
err:
	if (!IS_ERR(fp))
		filp_close(fp, NULL);

	set_fs(old_fs);
	return ret;
}

static int is_sbl_fuse_included(void)
{
	struct file *fp;
	int ret = RET_OK;

	mm_segment_t old_fs=get_fs();

	set_fs(KERNEL_DS);

	fp = filp_open(SBL_FUSE_PATH, O_RDONLY, 0);

	if (IS_ERR(fp)) {
		printk(KERN_ERR "%s: Unable to open file (%d)", __func__, PTR_ERR(fp));
		ret = RET_ERR;
		goto err;
	}

	filp_close(fp, NULL);
err:
	set_fs(old_fs);

	return ret;
}

int write_fuse_status(QFUSE_STATUS status) {
	struct file *fp;
	struct path root;
	int cnt = 0;
	u32 ret = RET_OK;
	mm_segment_t old_fs;

	if (status >= MAX_STATUS_COUNT){
		printk(KERN_ERR "%s : parameter must be less than MAX_STATUS_COUNT\n", __func__);
		ret = RET_ERR;
		return ret;
	}

	if (!is_cp_fused() || (status == QFUSE_ALREADY_BLOWNED && !g_cpFused))
		return ret;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	memset(&root, 0, sizeof(struct path));
	task_lock(&init_task);
	get_fs_root(init_task.fs, &root);
	task_unlock(&init_task);
	fp = file_open_root(root.dentry, root.mnt, FTM_PATH, O_RDWR, 0);
	path_put(&root);

	if(IS_ERR(fp)){
		printk(KERN_ERR "%s : Unable to open FTM (%d)\n", __func__, PTR_ERR(fp));
		ret = RET_ERR;
		goto err;
	}

	fp->f_pos = 0x99000; // LGFTM_FUSION_QFUSE_CHECK offset : 153 * 4096
	cnt = vfs_write(fp, qfuse_status_str[status] , strlen(qfuse_status_str[status]),&fp->f_pos);
	if(cnt != strlen(qfuse_status_str[status])){
		printk(KERN_ERR "%s : Unable to write FTM (%d)\n", __func__, cnt);
		ret = RET_ERR;
		goto err;
	}

err:
	if (!IS_ERR(fp))
		filp_close(fp, NULL);

	set_fs(old_fs);

	return ret;

}

void check_cp_fused(const char* buf) {
	if (NULL != buf && NULL != strstr(buf, QFUSE_CHECK_STR)) {
		g_cpFused = true;
		printk(KERN_INFO "%s: CP already has blowned fuse\n", __func__);
	}
}

bool check_if_sbl_fuse_is_loaded() {
	if (!is_sbl_fuse_included() && !is_ap_fused() && is_cp_fused()) {
		return true;
	}

	return false;
}
