/* production_test.c
 *
 * Copyright (C) 2015 LGE.
 *
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
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

#define MODULE_MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MODULE_MAX_LOG_FILE_COUNT	5
#define MODULE_FILE_STR_LEN		(128)

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

void module_write_file(struct device *dev, char *data, int write_time)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time = {0, };
	struct tm my_date = {0, };
	mm_segment_t old_fs = get_fs();
	int boot_mode = TOUCH_NORMAL_BOOT;
	struct touch_core_data *ts;

	TOUCH_TRACE();

	ts = (struct touch_core_data *) plist->touch_core_data;

	set_fs(KERNEL_DS);
	boot_mode = touch_check_boot_mode(ts->dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
		fname = "/data/vendor/touch/touch_module_self_test.txt";
		break;
	case TOUCH_MINIOS_AAT:
		fname = "/data/touch/touch_module_self_test.txt";
		break;
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_module_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}


	if (fname) {
		fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
		set_fs(old_fs);
		return;
	}

	if (fd >= 0) {
		if (write_time == TIME_INFO_WRITE) {
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec,
					sys_tz.tz_minuteswest * 60 * (-1),
					&my_date);
			snprintf(time_string, 64,
				"\n[%02d-%02d %02d:%02d:%02d.%03lu]\n",
				my_date.tm_mon + 1,
				my_date.tm_mday, my_date.tm_hour,
				my_date.tm_min, my_date.tm_sec,
				(unsigned long) my_time.tv_nsec / 1000000);
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	} else {
		TOUCH_E("%s : File open failed (fd: %d)\n", __func__, fd);
	}
	set_fs(old_fs);
}

void module_log_file_size_check(struct device *dev)
{
	char *fname = NULL;
	struct file *file = NULL;
	loff_t file_size = 0;
	int i = 0;
	char buf1[MODULE_FILE_STR_LEN] = {0};
	char buf2[MODULE_FILE_STR_LEN] = {0};
	int ret = 0;
	int boot_mode = TOUCH_NORMAL_BOOT;
	mm_segment_t old_fs = get_fs();

	TOUCH_TRACE();

	set_fs(KERNEL_DS);

	boot_mode = touch_check_boot_mode(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
		fname = "/data/vendor/touch/touch_module_self_test.txt";
		break;
	case TOUCH_MINIOS_AAT:
		fname = "/data/touch/touch_module_self_test.txt";
		break;
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_module_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}

	if (fname) {
		file = filp_open(fname, O_RDONLY, 0666);
		//sys_chmod(fname, 0666);
	} else {
		TOUCH_E("fname is NULL, can not open FILE\n");
		goto error;
	}

	if (IS_ERR(file)) {
		TOUCH_I("%s : ERR(%ld) Open file error [%s]\n",
				__func__, PTR_ERR(file), fname);
		goto error;
	}

	file_size = vfs_llseek(file, 0, SEEK_END);
	TOUCH_I("%s : [%s] file_size = %lld\n", __func__, fname, file_size);

	filp_close(file, 0);

	if (file_size > MODULE_MAX_LOG_FILE_SIZE) {
		TOUCH_I("%s : [%s] file_size(%lld) > MAX_LOG_FILE_SIZE(%d)\n",
				__func__, fname, file_size, MODULE_MAX_LOG_FILE_SIZE);

		for (i = MODULE_MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
			if (i == 0)
				snprintf(buf1, sizeof(buf1), "%s", fname);
			else
				snprintf(buf1, sizeof(buf1), "%s.%d", fname, i);

			ret = sys_access(buf1, 0);

			if (ret == 0) {
				TOUCH_I("%s : file [%s] exist\n", __func__, buf1);

				if (i == (MODULE_MAX_LOG_FILE_COUNT - 1)) {
					if (sys_unlink(buf1) < 0) {
						TOUCH_E("failed to remove file [%s]\n", buf1);
						goto error;
					}

					TOUCH_I("%s : remove file [%s]\n", __func__, buf1);
				} else {
					snprintf(buf2, sizeof(buf2), "%s.%d", fname, (i + 1));

					if (sys_rename(buf1, buf2) < 0) {
						TOUCH_E("failed to rename file [%s] -> [%s]\n",
								buf1, buf2);
						goto error;
					}

					TOUCH_I("%s : rename file [%s] -> [%s]\n",
							__func__, buf1, buf2);
				}
			} else {
				TOUCH_I("%s : file [%s] does not exist (ret = %d)\n",
						__func__, buf1, ret);
			}
		}
	}

error:
	set_fs(old_fs);
}


int module_touch_check_boot_mode(struct device *dev)
{
	int ret = TOUCH_NORMAL_BOOT;
#ifdef CONFIG_LGE_USB_FACTORY
	bool factory_boot = false;
	struct touch_core_data *ts = plist->touch_core_data;
#endif
	TOUCH_TRACE();

#ifdef CONFIG_LGE_USB_FACTORY
	factory_boot = lge_get_factory_boot();

	if (factory_boot) {
		switch (atomic_read(&ts->state.mfts)) {
		case MFTS_NONE:
			ret = TOUCH_MINIOS_AAT;
			break;
		case MFTS_FOLDER:
			ret = TOUCH_MINIOS_MFTS_FOLDER;
			break;
		case MFTS_FLAT:
			ret = TOUCH_MINIOS_MFTS_FLAT;
			break;
		case MFTS_CURVED:
			ret = TOUCH_MINIOS_MFTS_CURVED;
			break;
		default:
			ret = TOUCH_MINIOS_AAT;
			break;
		}
	}
#endif

	return ret;
}
