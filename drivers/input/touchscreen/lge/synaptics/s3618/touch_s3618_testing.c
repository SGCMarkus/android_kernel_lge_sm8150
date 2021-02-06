

/*
 * Synaptics TCM touchscreen driver
 *
 * Copyright (C) 2017-2018 Synaptics Incorporated. All rights reserved.
 *
 * Copyright (C) 2017-2018 Scott Lin <scott.lin@tw.synaptics.com>
 * Copyright (C) 2018-2019 Ian Su <ian.su@tw.synaptics.com>
 * Copyright (C) 2018-2019 Joey Zhou <joey.zhou@synaptics.com>
 * Copyright (C) 2018-2019 Yuehao Qiu <yuehao.qiu@synaptics.com>
 * Copyright (C) 2018-2019 Aaron Chen <aaron.chen@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/syscalls.h>
#include <linux/file.h>


#include <linux/gpio.h>
//#include "synaptics_tcm_core.h"
#include "touch_s3618_testing.h"

#include "touch_s3618.h"


#define SYSFS_DIR_NAME "testing"

#define REPORT_TIMEOUT_MS 5000


#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))


#define testing_sysfs_show(t_name) \
static ssize_t testing_sysfs_##t_name##_show(struct device *dev, char *buf) \
{ \
	int ret = 0; \
	struct touch_core_data *ts = testing_hcd->ts; \
	struct s3618_data *d = testing_hcd->d; \
\
	mutex_lock(&ts->lock); \
	mutex_lock(&d->extif_mutex); \
\
	ret = testing_##t_name(dev); \
	if (ret < 0) { \
		TOUCH_E("Failed to do "#t_name" test\n"); \
		goto exit; \
	} \
\
	ret = touch_snprintf(buf, PAGE_SIZE, w_buf); \
\
exit: \
	mutex_unlock(&d->extif_mutex); \
	mutex_unlock(&ts->lock); \
\
	return ret; \
}

enum test_code {
	TEST_NOT_IMPLEMENTED = 0,
	TEST_TRX_TRX_SHORTS_PT1 = 0x01,
	TEST_TRX_SENSOR_OPENS_PT2 = 0x02,
	TEST_TRX_GROUND_SHORTS_PT3 = 0x03,
	TEST_FULL_RAW_CAP_PT5 = 0x05,
	TEST_EE_SHORTS_PT6 = 0x06,
	TEST_DYNAMIC_RANGE_PT7 = 0x07,
	TEST_HIGH_RESISTANCE_PT8 = 0x08,
	TEST_NOISE_PT10 = 0x0A,
	TEST_OPEN_DETECTION_PT11 = 0x0B,
	TEST_PT12 = 0x0C,
	TEST_PT13 = 0x0D,
	TEST_DYNAMIC_RANGE_DOZE_PT14 = 0x0E,
	TEST_NOISE_DOZE_PT15 = 0x0F,
	TEST_SENSOR_SPEED_PT16 = 0x10,
	TEST_ADC_RANGE_PT17 = 0x11,
	TEST_HYBRID_ABS_RAW_PT18 = 0x12,
	TEST_HYBRID_ABS_RAW_WITH_CBC_PT21 = 0x15,
	TEST_DISCRETE_PT196 = 0xC4,
};

struct testing_hcd {
	bool result;
	unsigned char report_type;
	unsigned int report_index;
	unsigned int num_of_reports;
	struct kobject *sysfs_dir;
	struct s3618_buffer out;
	struct s3618_buffer resp;
	struct s3618_buffer report;
	struct s3618_buffer process;
	struct s3618_buffer output;

	struct touch_core_data *ts;
	struct s3618_data *d;
	bool need_write;

	int (*collect_reports)(enum report_type report_type,
			unsigned int num_of_reports);
	unsigned char *satic_cfg_buf;
	short tx_pins[MAX_PINS];
	short tx_assigned;
	short rx_pins[MAX_PINS];
	short rx_assigned;
	short guard_pins[MAX_PINS];
	short guard_assigned;
	short *noise_max;
};


DECLARE_COMPLETION(report_complete);

DECLARE_COMPLETION(testing_remove_complete);

static struct testing_hcd *testing_hcd;

static char w_buf[BUF_SIZE];

static int testing_device_id(struct device *dev);

static int testing_config_id(struct device *dev);

static int testing_noise(struct device *dev);

static int testing_reset_open(struct device *dev);

static int testing_pt1(struct device *dev);

static int testing_full_raw(struct device *dev);

static int testing_high_resistance(struct device *dev);

static int testing_hybrid_abs_raw(struct device *dev);

static int testing_trx_ground(struct device *dev);

static int testing_discrete_ex_trx_short(struct device *dev);

static ssize_t testing_sysfs_data_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static struct bin_attribute bin_attr = {
	.attr = {
		.name = "data",
		.mode = S_IRUGO,
	},
	.size = 0,
	.read = testing_sysfs_data_show,
};

testing_sysfs_show(device_id)

testing_sysfs_show(config_id)

testing_sysfs_show(noise)

testing_sysfs_show(reset_open)

testing_sysfs_show(pt1)

testing_sysfs_show(full_raw)

testing_sysfs_show(hybrid_abs_raw)

testing_sysfs_show(high_resistance)

testing_sysfs_show(trx_ground)

testing_sysfs_show(discrete_ex_trx_short)


static ssize_t testing_sysfs_size_show(struct device *dev,
		char *buf)
{
	int ret = 0;
	struct touch_core_data *ts = testing_hcd->ts;
	struct s3618_data *d = testing_hcd->d;

	mutex_lock(&ts->lock);
	mutex_lock(&d->extif_mutex);

	mutex_lock(&testing_hcd->output.buf_mutex);

	ret = touch_snprintf(buf, PAGE_SIZE,
			"%u\n",
			testing_hcd->output.data_length);

	mutex_unlock(&testing_hcd->output.buf_mutex);

	mutex_unlock(&d->extif_mutex);
	mutex_unlock(&ts->lock);

	return ret;
}
#if 0
void print_sd_log(char *buf)
{
	int i = 0;
	int index = 0;

	while (index < strlen(buf) && buf[index] != '\0' && i < LOG_BUF_SIZE - 1) {
		logbuf[i++] = buf[index];

		/* Final character is not '\n' */
		if ((index == strlen(buf) - 1 || i == LOG_BUF_SIZE - 2)
				&& logbuf[i - 1] != '\n')
			logbuf[i++] = '\n';

		if (logbuf[i - 1] == '\n') {
			logbuf[i - 1] = '\0';
			if (i - 1 != 0)
				TOUCH_I("%s\n", logbuf);

			i = 0;
		}
		index++;
	}
}
#endif
static void log_file_size_check(struct device *dev)
{
	char *fname = NULL;
	struct file *file = NULL;
	loff_t file_size = 0;
	int i = 0;
	char buf1[FILE_STR_LEN] = {0};
	char buf2[FILE_STR_LEN] = {0};
	int ret = 0;
	int boot_mode = TOUCH_NORMAL_BOOT;
	mm_segment_t old_fs = get_fs();

	TOUCH_TRACE();

	set_fs(KERNEL_DS);

	boot_mode = touch_check_boot_mode(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
		fname = "/data/vendor/touch/touch_self_test.txt";
		break;
	case TOUCH_MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}

	if (fname) {
		file = filp_open(fname, O_RDONLY, 0666);
		sys_chmod(fname, 0666);
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

	if (file_size > MAX_LOG_FILE_SIZE) {
		TOUCH_I("%s : [%s] file_size(%lld) > MAX_LOG_FILE_SIZE(%d)\n",
				__func__, fname, file_size, MAX_LOG_FILE_SIZE);

		for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
			if (i == 0)
				touch_snprintf(buf1, sizeof(buf1), "%s", fname);
			else
				touch_snprintf(buf1, sizeof(buf1), "%s.%d", fname, i);

			ret = sys_access(buf1, 0);

			if (ret == 0) {
				TOUCH_I("%s : file [%s] exist\n", __func__, buf1);

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					if (sys_unlink(buf1) < 0) {
						TOUCH_E("failed to remove file [%s]\n", buf1);
						goto error;
					}

					TOUCH_I("%s : remove file [%s]\n", __func__, buf1);
				} else {
					touch_snprintf(buf2, sizeof(buf2), "%s.%d", fname, (i + 1));

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

static void write_file(struct device *dev, char *data, int write_time)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time = {0, };
	struct tm my_date = {0, };
	mm_segment_t old_fs = get_fs();
	int boot_mode = TOUCH_NORMAL_BOOT;

	TOUCH_TRACE();

	set_fs(KERNEL_DS);
	boot_mode = touch_check_boot_mode(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
		fname = "/data/vendor/touch/touch_self_test.txt";
		break;
	case TOUCH_MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}


	if (fname) {
		fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("fname is NULL, can not open FILE\n");
		set_fs(old_fs);
		return;
	}

	if (fd >= 0) {
		if (write_time == TIME_INFO_WRITE) {
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec,
					sys_tz.tz_minuteswest * 60 * (-1),
					&my_date);
			touch_snprintf(time_string, 64,
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
		TOUCH_E("File open failed (fd: %d)\n", fd);
	}
	set_fs(old_fs);
}

static int firmware_version_log(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);

	int ret = 0;
	int ic_info_ret = 0;
	unsigned char buffer[LOG_BUF_SIZE] = {0,};
	int boot_mode = TOUCH_NORMAL_BOOT;
	char str[7] = {0};

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	boot_mode = touch_check_boot_mode(dev);

	switch (boot_mode) {
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
		ic_info_ret = s3618_ic_info(dev);
		if (ic_info_ret < 0) {
			TOUCH_E("failed to read ic info (ic_info_ret: %d)\n", ic_info_ret);
			return ic_info_ret;
		}
		break;
	default:
		break;
	}

	ret = touch_snprintf(buffer, LOG_BUF_SIZE, "======== Firmware Info ========\n");
	ret += touch_snprintf(buffer + ret, LOG_BUF_SIZE - ret,
					" Version: v%d.%02d, Build_id: %d\n",
					d->app_info.customer_config_id.release,
					d->app_info.customer_config_id.version,
					d->id_info.build_id);
	ret += touch_snprintf(buffer + ret, LOG_BUF_SIZE - ret,
					" Config ID: 0x%x%x%x%x%x%x%x%x\n",
					d->app_info.customer_config_id.maker,
					d->app_info.customer_config_id.key,
					d->app_info.customer_config_id.supplier,
					d->app_info.customer_config_id.inch0,
					d->app_info.customer_config_id.inch1,
					d->app_info.customer_config_id.panel,
					d->app_info.customer_config_id.release,
					d->app_info.customer_config_id.version);
	touch_snprintf(str, sizeof(str), "%s", d->prd_info.product_id);
	ret += touch_snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"\n=========== Production Info ===========\n");
	ret += touch_snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"Product_ID : %s\n", str);
	ret += touch_snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"Chip_ver: %d, FPC_ver: %d, Sensor_ver: %d\n",
			d->prd_info.chip_ver, d->prd_info.fpc_ver, d->prd_info.sensor_ver);
	ret += touch_snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"Inspector_channel : %d\n", d->prd_info.inspect_channel);
	ret += touch_snprintf(buffer + ret, LOG_BUF_SIZE - ret, "Time : 20%d/%d/%d - %dh %dm %ds\n\n",
			d->prd_info.inspect_date[0], d->prd_info.inspect_date[1], d->prd_info.inspect_date[2],
			d->prd_info.inspect_time[0], d->prd_info.inspect_time[1], d->prd_info.inspect_time[2]);

	if(testing_hcd->need_write)
		write_file(dev, buffer, TIME_INFO_SKIP);

	return 0;
}

static int s3618_sd(struct device *dev, char *buf)
{
	struct s3618_data *d = to_s3618_data(dev);

	int ret = 0;
	int noise_ret = 0;
	int full_raw_ret = 0;
	int high_resistance_ret = 0;
	int hybrid_abs_raw_ret = 0;
	int pt1_ret = 0;
	int prd_id_ret = 0;
	int trx_ground_ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	prd_id_ret = s3618_is_product(d, "PLG675", 6);

	mutex_lock(&d->extif_mutex);
	testing_full_raw(dev);
	full_raw_ret = testing_hcd->result;
	mutex_unlock(&d->extif_mutex);

	mutex_lock(&d->extif_mutex);
	testing_noise(dev);
	noise_ret = testing_hcd->result;
	mutex_unlock(&d->extif_mutex);

	mutex_lock(&d->extif_mutex);
	testing_high_resistance(dev);
	high_resistance_ret = testing_hcd->result;
	mutex_unlock(&d->extif_mutex);

	mutex_lock(&d->extif_mutex);
	testing_hybrid_abs_raw(dev);
	hybrid_abs_raw_ret = testing_hcd->result;
	mutex_unlock(&d->extif_mutex);

	mutex_lock(&d->extif_mutex);
	testing_pt1(dev);
	pt1_ret = testing_hcd->result;
	mutex_unlock(&d->extif_mutex);

	mutex_lock(&d->extif_mutex);
	testing_trx_ground(dev);
	trx_ground_ret = testing_hcd->result;
	mutex_unlock(&d->extif_mutex);

	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "\n========RESULT=======\n");
	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "Channel Status : %s",
				(high_resistance_ret && pt1_ret && trx_ground_ret)
				? "Pass\n" : "Fail ");

	if (!high_resistance_ret || !pt1_ret || !trx_ground_ret)
	{
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "(%s/%s/%s)\n",
					(high_resistance_ret != 1 ? "0" : "1"),
					(pt1_ret != 1 ? "0" : "1"),
					(trx_ground_ret != 1 ? "0" : "1"));
	}

	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "Raw Data : %s",
				(full_raw_ret && noise_ret && hybrid_abs_raw_ret && prd_id_ret)
				? "Pass\n" : "Fail ");

	if (!full_raw_ret || !noise_ret || !hybrid_abs_raw_ret || !prd_id_ret) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "(%s/%s/%s/%s)\n",
					(full_raw_ret != 1 ? "0" : "1"),
					(noise_ret != 1 ? "0" : "1"),
					(hybrid_abs_raw_ret != 1 ? "0" : "1"),
					(prd_id_ret != 1 ? "0" : "1"));
	}
	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "=====================\n");

	s3618_reset_ctrl(dev, HW_RESET);

	return ret;
}

static ssize_t show_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = testing_hcd->ts;
	struct s3618_data *d = testing_hcd->d;

	int ret = 0;
	int fw_ver_ret = 0;
//	int prd_info_ret = 0;

	TOUCH_I("%s\n", __func__);

	if (d->lcd_mode != LCD_MODE_U3) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD off!!!. Can not sd.\n");
		TOUCH_I("LCD off!!!. Can not sd.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
//	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	testing_hcd->need_write = true;
	/* file create , time log */
	if(testing_hcd->need_write) {
		write_file(dev, "\nShow_sd Test Start", TIME_INFO_SKIP);
		write_file(dev, "\n", TIME_INFO_WRITE);
	}
	TOUCH_I("Show_sd Test Start\n");

	fw_ver_ret = firmware_version_log(dev);
	if (fw_ver_ret < 0) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"\n========RESULT=======\n");
		TOUCH_I("========RESULT=======\n");
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"Raw Data : Fail (Check connector)\n");
		TOUCH_I("Raw Data : Fail (Check connector)\n");
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"Channel Status : Fail (Check connector)\n");
		TOUCH_I("Channel Status : Fail (Check connector)\n");
		goto exit;
	}

	ret = s3618_sd(dev, buf);

exit:
//	print_sd_log(buf);
	if(testing_hcd->need_write) {
		write_file(dev, buf, TIME_INFO_SKIP);
		write_file(dev, "\nShow_sd Test End\n\n", TIME_INFO_WRITE);
	}
	TOUCH_I("Show_sd Test End\n");
	log_file_size_check(dev);
//	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	testing_hcd->need_write = false;

	return ret;
}

static int s3618_lpwg_sd(struct device *dev, char *buf)
{
	struct s3618_data *d = to_s3618_data(dev);

	int ret = 0;
	int noise_ret = 0;
	int full_raw_ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	mutex_lock(&d->extif_mutex);
	testing_full_raw(dev);
	full_raw_ret = testing_hcd->result;
	mutex_unlock(&d->extif_mutex);

	mutex_lock(&d->extif_mutex);
	testing_noise(dev);
	noise_ret = testing_hcd->result;
	mutex_unlock(&d->extif_mutex);

	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "\n========RESULT=======\n");
	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "LPWG RawData : %s",
				(full_raw_ret && noise_ret)
				? "Pass\n" : "Fail ");

	if (!full_raw_ret || !noise_ret) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "(%s/%s)\n",
					(full_raw_ret != 1 ? "0" : "1"),
					(noise_ret != 1 ? "0" : "1"));
	}
	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "=====================\n");

	s3618_reset_ctrl(dev, HW_RESET);

	return ret;
}

static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = testing_hcd->d;

	int ret = 0;
	int fw_ver_ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	/* Deep sleep check */
	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		ret = touch_snprintf(buf + ret, PAGE_SIZE - ret,
			"LPWG Not Test. IC state is Deep Sleep.\n");
		TOUCH_I("LPWG Not Test. IC state is Deep Sleep.\n");
		return ret;
	}

	if (d->lcd_mode == LCD_MODE_U3) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD on!!!. Can not lpwg_sd.\n");
		TOUCH_I("LCD on!!!. Can not lpwg_sd.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	testing_hcd->need_write = true;

	/* file create , time log */
	if(testing_hcd->need_write) {
		write_file(dev, "\nShow_lpwg_sd Test Start", TIME_INFO_SKIP);
		write_file(dev, "\n", TIME_INFO_WRITE);
	}
	TOUCH_I("Show_lpwg_sd Test Start\n");

	fw_ver_ret = firmware_version_log(dev);
	if (fw_ver_ret < 0) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
				"\n========RESULT=======\n");
		TOUCH_I("========RESULT=======\n");
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
			"LPWG RawData : Fail (Check connector)\n");
		TOUCH_I("LPWG RawData : Fail (Check connector)\n");
		goto exit;
	}

	ret = s3618_lpwg_sd(dev, buf);

exit:
//	print_sd_log(buf);
	if(testing_hcd->need_write) {
		write_file(dev, buf, TIME_INFO_SKIP);
		write_file(dev, "\nShow_lpwg_sd Test End\n", TIME_INFO_WRITE);
	}
	TOUCH_I("Show_lpwg_sd Test End\n");
	log_file_size_check(dev);

	testing_hcd->need_write = false;
	mutex_unlock(&ts->lock);

	return ret;
}

static int enable_raw_delta_report(bool enable, char report_type)
{
	int retval = 0;
	unsigned char report_touch = REPORT_TOUCH;
	struct touch_core_data *ts = testing_hcd->ts;
	struct s3618_data *d = testing_hcd->d;
	unsigned char command = 0;

	if (enable) {
		retval = s3618_write_message(ts->dev,
						CMD_DISABLE_REPORT,
						&report_touch,
						1,
						d->response.buf,
						&d->response.data_length);
		if (retval < 0) {
			TOUCH_E("failed to disable touch report\n");
			return -EINVAL;
		}
		retval = s3618_set_dynamic_config(ts->dev, DC_NO_DOZE, 1);
		if (retval < 0) {
			TOUCH_E("failed to set %s Enable (ret: %d)\n", STR(DC_NO_DOZE), retval);
			return -EINVAL;
		}
	} else {
		retval = s3618_write_message(ts->dev,
						CMD_ENABLE_REPORT,
						&report_touch,
						1,
						d->response.buf,
						&d->response.data_length);
		if (retval < 0) {
			TOUCH_E("failed to disable touch report\n");
			return -EINVAL;
		}
		retval = s3618_set_dynamic_config(ts->dev, DC_NO_DOZE, 0);
		if (retval < 0) {
			TOUCH_E("failed to set %s Enable (ret: %d)\n", STR(DC_NO_DOZE), retval);
			return -EINVAL;
		}
	}


	if (enable)
		command = CMD_ENABLE_REPORT;
	else
		command = CMD_DISABLE_REPORT;

	retval = s3618_write_message(ts->dev,
					command,
					&report_type,
					1,
					d->response.buf,
					&d->response.data_length);
	if (retval < 0) {
		TOUCH_E("failed to write command %s\n", STR(command));
		return -EINVAL;
	}

	return 0;
}

#define DELTA_REPORT_TIMEOUT 300

static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = testing_hcd->ts;
	struct s3618_data *d = testing_hcd->d;

	int retval = 0;
	int report_size = 0;
	unsigned int rows = 0;
	unsigned int cols = 0;
	int* report_data = NULL;
	unsigned char timeout_count = 0;
	struct s3618_buffer buffer = {{0, }, };
	char log_buf[LOG_BUF_SIZE] = {0, };
	int size = 0;
	int log_size = 0;
	int col = 0, row = 0;
	int i = 0;
	struct s3618_app_info *app_info = NULL;
	short* p_data_16 = NULL;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);
	mutex_lock(&ts->lock);

	d->report_is_ready = false;

	memset(w_buf, 0x00, sizeof(w_buf));
	size = touch_snprintf(w_buf, sizeof(w_buf), "===== %sDelta Test =====\n",
			(d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");

	app_info = &d->app_info;
	rows = app_info->num_of_image_rows;
	cols = app_info->num_of_image_cols;
	report_size = rows * cols;

	report_data = kzalloc(report_size * sizeof(int), GFP_KERNEL);
	if (!report_data) {
		TOUCH_E("Failed to alloc memory for report_data");
		retval = -ENOMEM;
		goto exit;
	}

	if (d->id_info.mode != MODE_APPLICATION_FIRMWARE ||
		d->app_info.status != APP_STATUS_OK) {
		TOUCH_E("invalid app status (id_info.mode = 0x%x) (app_status = 0x%x)\n",
				d->id_info.mode, d->app_info.status);
		retval = -EINVAL;
		goto exit;
	}

	mutex_init(&buffer.buf_mutex);

	if (enable_raw_delta_report(true, REPORT_DELTA) < 0) {
		retval = -EINVAL;
		goto exit;
	}

	do {
		if (timeout_count == DELTA_REPORT_TIMEOUT) {
			TOUCH_E("timeout waiting for a report image");
			if (enable_raw_delta_report(false, REPORT_DELTA) < 0) {
				TOUCH_E("failed to disable the report");
			}
			retval = -EINVAL;
			goto exit;
		}

		touch_msleep(10);
		timeout_count++;
	} while (!d->report_is_ready);

	mutex_lock(&buffer.buf_mutex);

	memcpy(buffer.buf, d->report.buf, d->report.data_length);
	buffer.data_length = d->report.data_length;

	mutex_unlock(&buffer.buf_mutex);

	if (enable_raw_delta_report(false, REPORT_DELTA) < 0) {
		TOUCH_E("failed to disable the report");
		retval = -EINVAL;
		goto exit;
	}

	p_data_16 = (short *)buffer.buf;

	for (row = 0; row < rows; row++) {
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "[%2d] ", row);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "[%2d] ", row);

		for (col = 0; col < cols; col++) {
			i = row * cols + col;

			report_data[i] = (int)*p_data_16;
			p_data_16++;
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "%5d", report_data[i]);
			log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "%5d", report_data[i]);
		}
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");
		TOUCH_I("%s\n", log_buf);
		memset(log_buf, 0, sizeof(log_buf));
		log_size = 0;
	}
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");

	retval = touch_snprintf(buf, PAGE_SIZE, w_buf);

exit:
	kfree(report_data);
	mutex_unlock(&ts->lock);
	return retval;
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = testing_hcd->ts;
	struct s3618_data *d = testing_hcd->d;

	int retval = 0;
	int report_size = 0;
	unsigned int rows = 0;
	unsigned int cols = 0;
	int* report_data = NULL;
	unsigned char timeout_count = 0;
	struct s3618_buffer buffer = {{0, }, };
	char log_buf[LOG_BUF_SIZE] = {0, };
	int size = 0;
	int log_size = 0;
	int col = 0, row = 0;
	int i = 0;
	struct s3618_app_info *app_info = NULL;
	short* p_data_16 = NULL;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);
	mutex_lock(&ts->lock);

	d->report_is_ready = false;

	memset(w_buf, 0x00, sizeof(w_buf));
	size = touch_snprintf(w_buf, sizeof(w_buf), "===== %sRAWDATA Test =====\n",
			(d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");

	app_info = &d->app_info;
	rows = app_info->num_of_image_rows;
	cols = app_info->num_of_image_cols;
	report_size = rows * cols;

	report_data = kzalloc(report_size * sizeof(int), GFP_KERNEL);
	if (!report_data) {
		TOUCH_E("Failed to alloc memory for report_data");
		retval = -ENOMEM;
		goto exit;
	}

	if (d->id_info.mode != MODE_APPLICATION_FIRMWARE ||
		d->app_info.status != APP_STATUS_OK) {
		TOUCH_E("invalid app status (id_info.mode = 0x%x) (app_status = 0x%x)\n",
				d->id_info.mode, d->app_info.status);
		retval = -EINVAL;
		goto exit;
	}

	mutex_init(&buffer.buf_mutex);

	if (enable_raw_delta_report(true, REPORT_RAW) < 0) {
		retval = -EINVAL;
		goto exit;
	}

	do {
		if (timeout_count == DELTA_REPORT_TIMEOUT) {
			TOUCH_E("timeout waiting for a report image");
			if (enable_raw_delta_report(false, REPORT_RAW) < 0) {
				TOUCH_E("failed to disable the report");
			}
			retval = -EINVAL;
			goto exit;
		}

		touch_msleep(10);
		timeout_count++;
	} while (!d->report_is_ready);

	mutex_lock(&buffer.buf_mutex);

	memcpy(buffer.buf, d->report.buf, d->report.data_length);
	buffer.data_length = d->report.data_length;

	mutex_unlock(&buffer.buf_mutex);

	if (enable_raw_delta_report(false, REPORT_RAW) < 0) {
		TOUCH_E("failed to disable the report");
		retval = -EINVAL;
		goto exit;
	}

	p_data_16 = (short *)buffer.buf;

	for (row = 0; row < rows; row++) {
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "[%2d] ", row);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "[%2d] ", row);

		for (col = 0; col < cols; col++) {
			i = row * cols + col;

			report_data[i] = (int)*p_data_16;
			p_data_16++;
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "%5d", report_data[i]);
			log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "%5d", report_data[i]);
		}
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");
		TOUCH_I("%s\n", log_buf);
		memset(log_buf, 0, sizeof(log_buf));
		log_size = 0;
	}
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");


	retval = touch_snprintf(buf, PAGE_SIZE, w_buf);

exit:
	kfree(report_data);
	mutex_unlock(&ts->lock);
	return retval;
}


static TOUCH_ATTR(device_id,
		testing_sysfs_device_id_show, NULL);
static TOUCH_ATTR(config_id,
		testing_sysfs_config_id_show, NULL);
static TOUCH_ATTR(delta,
		show_delta, NULL);
static TOUCH_ATTR(jitter,
		testing_sysfs_noise_show, NULL);
static TOUCH_ATTR(size,
		testing_sysfs_size_show, NULL);
static TOUCH_ATTR(reset_open,
		testing_sysfs_reset_open_show, NULL);
static TOUCH_ATTR(pt1,
		testing_sysfs_pt1_show, NULL);
static TOUCH_ATTR(pre_rawdata,
		testing_sysfs_full_raw_show, NULL);
static TOUCH_ATTR(rawdata,
		show_rawdata, NULL);
static TOUCH_ATTR(hybrid_abs_raw,
		testing_sysfs_hybrid_abs_raw_show, NULL);
static TOUCH_ATTR(high_resistance,
		testing_sysfs_high_resistance_show, NULL);
static TOUCH_ATTR(trx_ground,
		testing_sysfs_trx_ground_show, NULL);
static TOUCH_ATTR(discrete_ex_trx_short,
		testing_sysfs_discrete_ex_trx_short_show, NULL);
static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);


static struct attribute *prd_attribute_list[] = {
	&touch_attr_device_id.attr,
	&touch_attr_config_id.attr,
	&touch_attr_delta.attr,
	&touch_attr_jitter.attr,
	&touch_attr_reset_open.attr,
	&touch_attr_size.attr,
	&touch_attr_pt1.attr,
	&touch_attr_pre_rawdata.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_hybrid_abs_raw.attr,
	&touch_attr_high_resistance.attr,
	&touch_attr_trx_ground.attr,
	&touch_attr_discrete_ex_trx_short.attr,
	&touch_attr_sd.attr,
	&touch_attr_lpwg_sd.attr,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
};


static ssize_t testing_sysfs_data_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int ret = 0;
	unsigned int readlen = 0;
	struct touch_core_data *ts = testing_hcd->ts;
	struct s3618_data *d = testing_hcd->d;

	mutex_lock(&ts->lock);
	mutex_lock(&d->extif_mutex);

	mutex_lock(&testing_hcd->output.buf_mutex);

	readlen = MIN(count, testing_hcd->output.data_length - pos);

	memcpy(buf,
		&testing_hcd->output.buf[pos],
		readlen);

	ret = readlen;

	mutex_unlock(&testing_hcd->output.buf_mutex);

	mutex_unlock(&d->extif_mutex);
	mutex_unlock(&ts->lock);

	return ret;
}

static int testing_run_prod_test_item(enum test_code test_code)
{
	int ret = 0;
	struct touch_core_data *ts = testing_hcd->ts;
	struct s3618_data *d = testing_hcd->d;
/*
	if (tcm_hcd->features.dual_firmware &&
			tcm_hcd->id_info.mode != MODE_PRODUCTIONTEST_FIRMWARE) {
		retval = tcm_hcd->switch_mode(tcm_hcd, FW_MODE_PRODUCTION_TEST);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to run production test firmware\n");
			return retval;
		}
	} else if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode) ||
			d->app_info.status != APP_STATUS_OK) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Identifying mode = 0x%02x\n",
				tcm_hcd->id_info.mode);
		return -ENODEV;
	}
*/

	if (d->id_info.mode != MODE_APPLICATION_FIRMWARE ||
			d->app_info.status != APP_STATUS_OK) {
		TOUCH_E("Identifying mode = 0x%02x\n", d->id_info.mode);
		return -ENODEV;
	}

	mutex_lock(&testing_hcd->out.buf_mutex);

	memset(testing_hcd->out.buf, 0x00, sizeof(testing_hcd->out.buf));
	testing_hcd->out.data_length = 0;

	testing_hcd->out.buf[0] = test_code;

	mutex_lock(&testing_hcd->resp.buf_mutex);

	ret = s3618_write_message(ts->dev,
			CMD_PRODUCTION_TEST,
			testing_hcd->out.buf,
			1,
			testing_hcd->resp.buf,
			&testing_hcd->resp.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n", STR(CMD_PRODUCTION_TEST));
		goto exit;
	}

	ret = 0;
exit:
	mutex_unlock(&testing_hcd->resp.buf_mutex);
	mutex_unlock(&testing_hcd->out.buf_mutex);

	return ret;
}

static int testing_collect_reports(enum report_type report_type,
		unsigned int num_of_reports)
{
	int ret = 0;
	bool completed = 0;
	unsigned int timeout = 0;
	struct touch_core_data *ts = testing_hcd->ts;

	testing_hcd->report_index = 0;
	testing_hcd->report_type = report_type;
	testing_hcd->num_of_reports = num_of_reports;

	reinit_completion(&report_complete);

	mutex_lock(&testing_hcd->out.buf_mutex);

	memset(testing_hcd->out.buf, 0x00, sizeof(testing_hcd->out.buf));
	testing_hcd->out.data_length = 0;


	testing_hcd->out.buf[0] = testing_hcd->report_type;

	mutex_lock(&testing_hcd->resp.buf_mutex);

	ret = s3618_write_message(ts->dev,
			CMD_ENABLE_REPORT,
			testing_hcd->out.buf,
			1,
			testing_hcd->resp.buf,
			&testing_hcd->resp.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n", STR(CMD_ENABLE_REPORT));
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		mutex_unlock(&testing_hcd->out.buf_mutex);
		goto exit;
	}

	mutex_unlock(&testing_hcd->resp.buf_mutex);
	mutex_unlock(&testing_hcd->out.buf_mutex);

	completed = false;
	timeout = REPORT_TIMEOUT_MS * num_of_reports;

	ret = wait_for_completion_timeout(&report_complete,
			msecs_to_jiffies(timeout));
	if (ret == 0) {
		TOUCH_E("Timed out waiting for report collection\n");
	} else {
		completed = true;
	}

	mutex_lock(&testing_hcd->out.buf_mutex);

	testing_hcd->out.buf[0] = testing_hcd->report_type;

	mutex_lock(&testing_hcd->resp.buf_mutex);

	ret = s3618_write_message(ts->dev,
			CMD_DISABLE_REPORT,
			testing_hcd->out.buf,
			1,
			testing_hcd->resp.buf,
			&testing_hcd->resp.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n", STR(CMD_DISABLE_REPORT));
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		mutex_unlock(&testing_hcd->out.buf_mutex);
		goto exit;
	}

	mutex_unlock(&testing_hcd->resp.buf_mutex);
	mutex_unlock(&testing_hcd->out.buf_mutex);

	if (completed)
		ret = 0;
	else
		ret = -EIO;

exit:
	testing_hcd->report_type = 0;

	return ret;
}

static void testing_get_frame_size_words(unsigned int *size, bool image_only)
{
	unsigned int rows = 0;
	unsigned int cols = 0;
	unsigned int hybrid = 0;
	unsigned int buttons = 0;
	struct s3618_app_info *app_info = NULL;
	struct s3618_data *d = testing_hcd->d;

	app_info = &d->app_info;

	rows = app_info->num_of_image_rows;
	cols = app_info->num_of_image_cols;
	hybrid = app_info->has_hybrid_data;
	buttons = app_info->num_of_buttons;

	*size = rows * cols;

	if (!image_only) {
		if (hybrid)
			*size += rows + cols;
		*size += buttons;
	}

	return;
}

static void testing_standard_frame_output(bool image_only)
{
	unsigned int data_size = 0;
	unsigned int header_size = 0;
	unsigned int output_size = 0;
	struct s3618_app_info *app_info = NULL;
	struct s3618_data *d = testing_hcd->d;

	app_info = &d->app_info;

	testing_get_frame_size_words(&data_size, image_only);

	header_size = sizeof(app_info->num_of_buttons) +
			sizeof(app_info->num_of_image_rows) +
			sizeof(app_info->num_of_image_cols) +
			sizeof(app_info->has_hybrid_data);

	mutex_lock(&testing_hcd->output.buf_mutex);

	memset(testing_hcd->output.buf, 0x00, sizeof(testing_hcd->output.buf));
	testing_hcd->output.data_length = 0;

	memcpy(testing_hcd->output.buf,
		&app_info->num_of_buttons,
		header_size);

	output_size = header_size;

	mutex_lock(&testing_hcd->resp.buf_mutex);

	memcpy(testing_hcd->output.buf + header_size,
		testing_hcd->resp.buf,
		testing_hcd->resp.data_length);

	output_size += testing_hcd->resp.data_length;

	mutex_unlock(&testing_hcd->resp.buf_mutex);

	testing_hcd->output.data_length = output_size;

	mutex_unlock(&testing_hcd->output.buf_mutex);

	return;
}

static int testing_device_id(struct device *dev)
{
	struct s3618_data *d = testing_hcd->d;
	struct s3618_id_info *id_info = NULL;
	char *strptr = NULL;

	id_info = &d->id_info;

	TOUCH_I("Start Device ID testing\n");

	testing_hcd->result = true;

	strptr = strnstr(id_info->part_number,
					device_id_limit,
					sizeof(id_info->part_number));
	if (strptr == NULL) {
		TOUCH_I("Device ID is mismatching, FW: %s (%s)\n",
				id_info->part_number, device_id_limit);
		testing_hcd->result = false;
	}

	TOUCH_I("Result = %s\n", (testing_hcd->result)?"pass":"fail");

	return 0;
}

static int testing_config_id(struct device *dev)
{
	struct s3618_data *d = testing_hcd->d;
	struct s3618_app_info *app_info = NULL;
	int i;

	app_info = &d->app_info;

	TOUCH_I("Start Config ID testing\n");

	testing_hcd->result = true;

	memset(w_buf, 0x00, sizeof(w_buf));

	for (i = 0; i < sizeof(config_id_limit); i++) {
//		if (config_id_limit[i] !=
//				d->app_info.customer_config_id[i]) {
//			TOUCH_E("Config ID is mismatching at byte %d\n", i);
//			testing_hcd->result = false;
//		}
	}

	TOUCH_I("Result = %s\n", (testing_hcd->result)?"pass":"fail");

	return 0;
}

static int testing_noise(struct device *dev) {
	int ret = 0;
	unsigned char *buf = NULL;
	unsigned int i = 0;
	unsigned int idx = 0;
	unsigned int row = 0;
	unsigned int col = 0;
	unsigned int rows = 0;
	unsigned int cols = 0;
	unsigned int limits_rows = 0;
	unsigned int limits_cols = 0;
	unsigned int frame_size_words = 0;
	struct s3618_app_info *app_info = NULL;
	struct s3618_data *d = testing_hcd->d;
	struct touch_core_data *ts = testing_hcd->ts;
	short *tmp_buf = NULL;
	char log_buf[LOG_BUF_SIZE] = {0, };
	int size = 0;
	int log_size = 0;
	int test_cnt = 0;

	TOUCH_I("Start Noise testing\n");
	testing_hcd->result = false;

	memset(w_buf, 0x00, sizeof(w_buf));

	if (testing_hcd->need_write ) {
		size = touch_snprintf(w_buf, sizeof(w_buf), "[%sNoise Test]\n",
				(d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");
	} else {
		size = touch_snprintf(w_buf, sizeof(w_buf), "===== %sNoise Test =====\n",
				(d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");
	}

	app_info = &d->app_info;

	rows = app_info->num_of_image_rows;
	cols = app_info->num_of_image_cols;

	testing_hcd->result = true;

	testing_get_frame_size_words(&frame_size_words, true);

	testing_hcd->noise_max = kzalloc(sizeof(unsigned short )*rows*cols, GFP_KERNEL);
	tmp_buf = kzalloc(sizeof(unsigned short )*rows*cols, GFP_KERNEL);

	mutex_unlock(&d->extif_mutex);
	for (test_cnt = 0; test_cnt < NOISE_TEST_CNT; test_cnt++) {
		mutex_lock(&d->extif_mutex);

		ret = testing_run_prod_test_item(TEST_NOISE_PT10);
		if (ret < 0) {
			TOUCH_E("Failed to run test\n");
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
					"Failed to run test\n");
			mutex_unlock(&d->extif_mutex);
			break;
		}

		mutex_lock(&testing_hcd->resp.buf_mutex);

		if (frame_size_words != testing_hcd->resp.data_length / 2) {
			TOUCH_E("Frame size mismatch\n");
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
					"Frame size mismatch\n");
			mutex_unlock(&testing_hcd->resp.buf_mutex);
			mutex_unlock(&d->extif_mutex);
			break;
		}

		idx = 0;
		buf = testing_hcd->resp.buf;

		for (row = 0; row < rows; row++) {
			for (col = 0; col < cols; col++) {
				i = row * cols + col;
				tmp_buf[i] = (short)le2_to_uint(&buf[idx * 2]);;
				idx++;

				if (ABS(tmp_buf[i]) > testing_hcd->noise_max[i]) {
					testing_hcd->noise_max[i] = ABS(tmp_buf[i]);
				}
			}
		}
		touch_msleep(100);
		mutex_unlock(&testing_hcd->resp.buf_mutex);

		mutex_unlock(&d->extif_mutex);
	}
	mutex_lock(&d->extif_mutex);

	kfree(tmp_buf);

	limits_rows = sizeof(noise_limits) / sizeof(noise_limits[0]);
	limits_cols = sizeof(noise_limits[0]) / sizeof(noise_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		TOUCH_E("Mismatching limits data\n");
		ret = -EINVAL;
		goto exit;
	}

	if (testing_hcd->need_write) {
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "  :  ");
		log_size = touch_snprintf(log_buf, LOG_BUF_SIZE, "  :  ");
		for (col = 0; col < cols; col++) {
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, " [%2d]", col);
			log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, " [%2d]", col);
		}
		TOUCH_I("%s\n", log_buf);
		memset(log_buf, 0, sizeof(log_buf));
		log_size = 0;
	}

	for (row = 0; row < rows; row++) {
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n[%2d] ", row);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "[%2d] ", row);

		for (col = 0; col < cols; col++) {
			i = row * cols + col;

			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "%5d", testing_hcd->noise_max[i]);
			log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "%5d", testing_hcd->noise_max[i]);
		}
		TOUCH_I("%s\n", log_buf);
		memset(log_buf, 0, sizeof(log_buf));
		log_size = 0;
	}
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");

	if (!testing_hcd->need_write) {
			//mutex_unlock(&testing_hcd->resp.buf_mutex);
			testing_standard_frame_output(false);
			ret = 0;
			goto exit;
	}

	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			i = row * cols + col;

			if (testing_hcd->noise_max[i] > noise_limits[row][col]) {
				size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
						"fail at (%d, %d) data = %d, limit = %d\n",
						row, col, testing_hcd->noise_max[i], noise_limits[row][col]);
				TOUCH_I("fail at (%d, %d) data = %d, limit = %d\n",
						row, col, testing_hcd->noise_max[i], noise_limits[row][col]);

				if (size > (BUF_SIZE / 2)) {
					write_file(dev, w_buf, TIME_INFO_SKIP);
					memset(w_buf, 0, BUF_SIZE);
					size = 0;
				}

				testing_hcd->result = false;
			}
		}
	}

	//mutex_unlock(&testing_hcd->resp.buf_mutex);

	testing_standard_frame_output(false);

	ret = 0;

		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
			"\nResult = %s\n\n", (testing_hcd->result)?"pass":"fail");
	if(testing_hcd->need_write) {
		write_file(dev, w_buf, TIME_INFO_SKIP);
	}
	TOUCH_I("Result = %s\n", (testing_hcd->result)?"pass":"fail");


exit:
	kfree(testing_hcd->noise_max);

	s3618_reset_ctrl(ts->dev, SW_RESET_NO_INIT);

	return ret;
}

static int testing_full_raw(struct device *dev)
{
	int ret = 0;
	unsigned char *buf = NULL;
	unsigned int i = 0;
	unsigned int idx = 0;
	unsigned int row = 0;
	unsigned int col = 0;
	unsigned int rows = 0;
	unsigned int cols = 0;
	unsigned int limits_rows = 0;
	unsigned int limits_cols = 0;
	unsigned int frame_size = 0;
	struct s3618_app_info *app_info = NULL;
	struct s3618_data *d = testing_hcd->d;
	struct touch_core_data *ts = testing_hcd->ts;
	short *tmp_buf = NULL;
	char log_buf[LOG_BUF_SIZE] = {0, };
	int size = 0;
	int log_size = 0;

	TOUCH_I("Start Full Raw testing\n");
	testing_hcd->result = false;

	memset(w_buf, 0x00, sizeof(w_buf));
	size = touch_snprintf(w_buf, sizeof(w_buf), "[%sFull Raw Test]\n",
			(d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");

	app_info = &d->app_info;

	rows = app_info->num_of_image_rows;
	cols = app_info->num_of_image_cols;

	frame_size = rows * cols * 2;


	ret = testing_run_prod_test_item(TEST_FULL_RAW_CAP_PT5);
	if (ret < 0) {
		TOUCH_E("Failed to run test\n");
		goto exit;
	}

	mutex_lock(&testing_hcd->resp.buf_mutex);

	if (frame_size != testing_hcd->resp.data_length) {
		TOUCH_E("Frame size is mismatching\n");
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		ret = -EINVAL;

		goto exit;
	}

	limits_rows = sizeof(pt5_full_raw_hi_limits) /
				sizeof(pt5_full_raw_hi_limits[0]);
	limits_cols = sizeof(pt5_full_raw_hi_limits[0]) /
				sizeof(pt5_full_raw_hi_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		TOUCH_E("Mismatching limits data\n");
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		ret = -EINVAL;
		goto exit;
	}

	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	tmp_buf = kzalloc(sizeof(unsigned short )*rows*cols, GFP_KERNEL);

	idx = 0;
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			i = row * cols + col;
			tmp_buf[i] = (unsigned short)(buf[idx] &0xff) |
							(unsigned short)(buf[idx+1] << 8);
			idx+=2;
		}
	}

	if (testing_hcd->need_write) {
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "  :  ");
		log_size = touch_snprintf(log_buf, LOG_BUF_SIZE, "  :  ");
		for (col = 0; col < cols; col++) {
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, " [%2d]", col);
			log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, " [%2d]", col);
		}
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");
		TOUCH_I("%s\n", log_buf);
		memset(log_buf, 0, sizeof(log_buf));
		log_size = 0;
	}

	for (row = 0; row < rows; row++) {
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "[%2d] ", row);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "[%2d] ", row);
		for (col = 0; col < cols; col++) {
			i = row * cols + col;

			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "%5d", tmp_buf[i]);
			log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "%5d", tmp_buf[i]);
		}
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");
		TOUCH_I("%s\n", log_buf);
		memset(log_buf, 0, sizeof(log_buf));
		log_size = 0;
	}
	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");

	if (!testing_hcd->need_write) {
		kfree(tmp_buf);
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		ret = 0;
		goto exit;
	}

	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			i = row * cols + col;

			if (tmp_buf[i]  > pt5_full_raw_hi_limits[row][col] ||
					tmp_buf[i]  < pt5_full_raw_lo_limits[row][col]) {
				testing_hcd->result = false;

				TOUCH_I("fail at (%-2d, %-2d) data = %d, limit = (%d - %d)\n",
						row, col, tmp_buf[i],
						pt5_full_raw_lo_limits[row][col],
						pt5_full_raw_hi_limits[row][col]);
				size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
					"fail at (%-2d, %-2d) data = %d, limit = (%d - %d)\n",
					row, col, tmp_buf[i],
					pt5_full_raw_lo_limits[row][col],
					pt5_full_raw_hi_limits[row][col]);

				if (size > (BUF_SIZE / 2)) {
					write_file(dev, w_buf, TIME_INFO_SKIP);
					memset(w_buf, 0, BUF_SIZE);
					size = 0;
				}
			}
		}
	}

	kfree(tmp_buf);
	mutex_unlock(&testing_hcd->resp.buf_mutex);

	ret = 0;

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
			"Result = %s\n\n", (testing_hcd->result)?"pass":"fail");
	if(testing_hcd->need_write)
		write_file(dev, w_buf, TIME_INFO_SKIP);
	TOUCH_I("Result = %s\n", (testing_hcd->result)?"pass":"fail");

exit:
	s3618_reset_ctrl(ts->dev, SW_RESET_NO_INIT);

	return ret;
}


static int testing_hybrid_abs_raw(struct device *dev)
{
	int ret = 0;
	unsigned char *buf = NULL;
	unsigned int idx = 0;
	unsigned int row = 0;
	unsigned int col = 0;
	unsigned int rows = 0;
	unsigned int cols = 0;
	unsigned int limits_rows = 0;
	unsigned int limits_cols = 0;
	unsigned int frame_size = 0;
	struct s3618_app_info *app_info = NULL;
	struct s3618_data *d = testing_hcd->d;
	struct touch_core_data *ts = testing_hcd->ts;
	int *tmp_buf = NULL;
	char log_buf[LOG_BUF_SIZE] = {0, };
	int size = 0;
	int log_size = 0;

	TOUCH_I("Start Hybrid Abs Raw testing\n");
	testing_hcd->result = false;

	memset(w_buf, 0x00, sizeof(w_buf));
	size = touch_snprintf(w_buf, sizeof(w_buf), "==Hybrid abs raw Test==\n");

	app_info = &d->app_info;

	rows = app_info->num_of_image_rows;
	cols = app_info->num_of_image_cols;

	frame_size = (rows + cols) * 4;

	ret = testing_run_prod_test_item(TEST_HYBRID_ABS_RAW_PT18);
	if (ret < 0) {
		TOUCH_E("Failed to run test\n");
		goto exit;
	}

	mutex_lock(&testing_hcd->resp.buf_mutex);

	if (frame_size != testing_hcd->resp.data_length) {
		TOUCH_E("Frame size is mismatching\n");
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		ret = -EINVAL;
		goto exit;
	}


	limits_rows = sizeof(hybrid_abs_tx_lo_limits) /
				sizeof(hybrid_abs_tx_lo_limits[0]);
	limits_cols = sizeof(hybrid_abs_rx_lo_limits) /
				sizeof(hybrid_abs_rx_lo_limits[0]);

	if (rows > limits_rows || cols > limits_cols) {
		TOUCH_E("Mismatching limits data\n");
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		ret = -EINVAL;
		goto exit;
	}

	idx = 0;
	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "[Hybrid Abs Test for Rx]\n");
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "  : ");
	log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "[Hybrid Abs Test for Rx]\n");
	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;

	for (col = 0; col < cols; col++) {
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "  [%2d]", col);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "  [%2d]", col);
	}
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");
	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;

	tmp_buf = kzalloc(frame_size, GFP_KERNEL);
	idx = 0;
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "[ 0]");
	for (col = 0; col < cols; col++) {
		tmp_buf[col] = (unsigned int)(buf[idx] & 0xff) |
						(unsigned int)(buf[idx+1] << 8) |
						(unsigned int)(buf[idx+2] << 16) |
						(unsigned int)(buf[idx+3] << 24);

		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "%6d", tmp_buf[col]);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "%6d", tmp_buf[col]);

		idx+=4;
	}
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");
	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "[Hybrid Abs Test for Tx]");
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "  : ");
	log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "[Hybrid Abs Test for Tx]\n");
	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;

	for (row = 0; row < rows; row++) {
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "  [%2d]", row);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "  [%2d]", row);
	}
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");
	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "[ 0]");

	for (row = 0; row < rows; row++) {
		tmp_buf[cols + row] = (unsigned int)(buf[idx] & 0xff) |
						(unsigned int)(buf[idx+1] << 8) |
						(unsigned int)(buf[idx+2] << 16) |
						(unsigned int)(buf[idx+3] << 24);

		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "%6d", tmp_buf[cols + row]);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "%6d", tmp_buf[cols + row]);

		idx+=4;
	}
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");
	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;

	if (!testing_hcd->need_write) {
		ret = 0;
		kfree(tmp_buf);
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		goto exit;
	}

	for (col = 0; col < cols; col++) {
		if (tmp_buf[col] < hybrid_abs_rx_lo_limits[col] ||
			tmp_buf[col] > hybrid_abs_rx_hi_limits[col]) {
			testing_hcd->result = false;

			TOUCH_I("fail at rx=%-2d. data = %d, limit = (%d~%d)\n",
					col, tmp_buf[col], hybrid_abs_rx_lo_limits[col],
					hybrid_abs_rx_hi_limits[col]);
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
					"fail at rx=%-2d. data = %d, limit = (%d~%d)\n",
					col, tmp_buf[col], hybrid_abs_rx_lo_limits[col],
					hybrid_abs_rx_hi_limits[col]);

			if (size > (BUF_SIZE / 2)) {
				write_file(dev, w_buf, TIME_INFO_SKIP);
				memset(w_buf, 0, BUF_SIZE);
				size = 0;
			}
		}
	}

	for (row = 0; row < rows; row++) {
		if (tmp_buf[cols + row] < hybrid_abs_tx_lo_limits[row] ||
			tmp_buf[cols + row] > hybrid_abs_tx_hi_limits[row]) {
			testing_hcd->result = false;

			TOUCH_I("fail at tx=%-2d. data = %d, limit = (%d~%d)\n",
					row, tmp_buf[cols + row],
					hybrid_abs_tx_lo_limits[row],
					hybrid_abs_tx_hi_limits[row]);
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
					"fail at tx=%-2d. data = %d, limit = (%d~%d)\n",
					row, tmp_buf[cols + row],
					hybrid_abs_tx_lo_limits[row],
					hybrid_abs_tx_hi_limits[row]);

			if (size > (BUF_SIZE / 2)) {
				write_file(dev, w_buf, TIME_INFO_SKIP);
				memset(w_buf, 0, BUF_SIZE);
				size = 0;
			}
		}
	}

	ret = 0;

	kfree(tmp_buf);

	mutex_unlock(&testing_hcd->resp.buf_mutex);

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
			"Result = %s\n\n", (testing_hcd->result)?"pass":"fail");
	if(testing_hcd->need_write)
		write_file(dev, w_buf, TIME_INFO_SKIP);
	TOUCH_I("Result = %s\n", (testing_hcd->result)?"pass":"fail");

exit:
	s3618_reset_ctrl(ts->dev, SW_RESET_NO_INIT);

	return ret;
}

static int testing_high_resistance(struct device *dev)
{
	int ret = 0;
	unsigned char *buf = NULL;
	unsigned int rows = 0;
	unsigned int cols = 0;
	unsigned int frame_size = 0;
	struct s3618_app_info *app_info = NULL;
	struct s3618_data *d = testing_hcd->d;
	struct touch_core_data *ts = testing_hcd->ts;
	int size = 0;
	unsigned int idx = 0, col = 0, row = 0;
	short data = 0;
	char log_buf[LOG_BUF_SIZE] = {0, };
	int log_size = 0;

	TOUCH_I("Start High Resistance testing\n");
	testing_hcd->result = false;

	memset(w_buf, 0x00, sizeof(w_buf));
	size = touch_snprintf(w_buf, sizeof(w_buf), "==High Resistance Test==\n");

	app_info = &d->app_info;

	rows = app_info->num_of_image_rows;
	cols = app_info->num_of_image_cols;

	frame_size = (rows * cols) * 2 + rows + cols + 6;

	ret = testing_run_prod_test_item(TEST_HIGH_RESISTANCE_PT8);
	if (ret < 0) {
		TOUCH_E("Failed to run test\n");
		goto exit;
	}

	mutex_lock(&testing_hcd->resp.buf_mutex);

	if (frame_size >= testing_hcd->resp.data_length) {
		TOUCH_E("Frame size is incorrect\n");
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		ret = -EINVAL;
		goto exit;
	}

	buf = testing_hcd->resp.buf;

	idx = 3;

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "[RxROE]\n");
	TOUCH_I("[RxROE]");
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "  :  ");

	for (col = 0; col < cols; col++) {
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "  [%2d]", col);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "  [%2d]", col);
	}
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");
	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "[ 0]");

	for (col = 0; col < cols; col++) {
		data = (int)le2_to_uint(&buf[idx * 2]);
		idx++;
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "%6d", data);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "%6d", data);
	}
	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "[TxROE]\n");
	TOUCH_I("[TxROE]");
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "  :  ");

	for (row = 0; row < rows; row++) {
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "  [%2d]", row);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "  [%2d]", row);
	}
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");
	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "[ 0]");

	for (row = 0; row < rows; row++) {
		data = (int)le2_to_uint(&buf[idx * 2]);
		idx++;
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "%6d", data);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "%6d", data);
	}
	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "[Tixel]\n");
	TOUCH_I("[Tixel]\n");

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "  :  ");
	log_size = touch_snprintf(log_buf, LOG_BUF_SIZE, "  :  ");
	for (col = 0; col < cols; col++) {
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, " [%2d]", col);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, " [%2d]", col);
	}
	TOUCH_I("%s\n", log_buf);
	memset(log_buf, 0, sizeof(log_buf));
	log_size = 0;

	for (row = 0; row < rows; row++) {
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n[%2d] ", row);
		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "[%2d] ", row);

		for (col = 0; col < cols; col++) {
			data = (int)le2_to_uint(&buf[idx * 2]);
			idx++;

			if (row == 0 && col == 8) {
				data = 0;
			}
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "%5d", data);
			log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "%5d", data);
		}
		TOUCH_I("%s\n", log_buf);
		memset(log_buf, 0, sizeof(log_buf));
		log_size = 0;
	}
	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");

	if (!testing_hcd->need_write) {
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		ret = 0;
		goto exit;
	}

	testing_hcd->result = true;

	idx = 3;

	for (col = 0; col < cols; col++) {
		data = (int)le2_to_uint(&buf[idx * 2]);
		idx++;

		if (data > pt8_high_resistance_rxroe_limit) {
			testing_hcd->result = false;
			TOUCH_I("fail at rxroe=%-2d. data = %d, limit = %d\n",
					col, data, pt8_high_resistance_rxroe_limit);
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
					"fail at rxroe=%-2d. data = %d, limit = %d\n",
					col, data, pt8_high_resistance_rxroe_limit);

			if (size > (BUF_SIZE / 2)) {
				write_file(dev, w_buf, TIME_INFO_SKIP);
				memset(w_buf, 0, BUF_SIZE);
				size = 0;
			}
		}
	}

	for (row = 0; row < rows; row++) {
		data = (int)le2_to_uint(&buf[idx * 2]);
		idx++;

		if (data > pt8_high_resistance_txroe_limit) {
			testing_hcd->result = false;
			TOUCH_I("fail at txroe=%-2d. data = %d, limit = %d\n",
					row, data, pt8_high_resistance_txroe_limit);
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
					"fail at txroe=%-2d. data = %d, limit = %d\n",
					row, data, pt8_high_resistance_txroe_limit);

			if (size > (BUF_SIZE / 2)) {
				write_file(dev, w_buf, TIME_INFO_SKIP);
				memset(w_buf, 0, BUF_SIZE);
				size = 0;
			}
		}
	}

	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (int)le2_to_uint(&buf[idx * 2]);
			idx++;

			if (row == 0 && col == 8) {
				TOUCH_I("(row = 0 / col = 8) is notch. skip to compare with the spec.");
				continue;
			}

			if (data < pt8_high_resistance_tixwl_limit) {
				testing_hcd->result = false;
				TOUCH_I("fail at tixel=(%-2d,%-2d). data = %d, limit = %d\n",
						row, col, data, pt8_high_resistance_tixwl_limit);
				size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
						"fail at tixel=(%-2d,%-2d). data = %d, limit = %d\n",
						row, col, data, pt8_high_resistance_tixwl_limit);

				if (size > (BUF_SIZE / 2)) {
					write_file(dev, w_buf, TIME_INFO_SKIP);
					memset(w_buf, 0, BUF_SIZE);
					size = 0;
				}
			}
		}
	}

	mutex_unlock(&testing_hcd->resp.buf_mutex);

	ret = 0;

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
			"Result = %s\n\n", (testing_hcd->result)?"pass":"fail");
	if(testing_hcd->need_write)
		write_file(dev, w_buf, TIME_INFO_SKIP);
	TOUCH_I("Result = %s\n", (testing_hcd->result)?"pass":"fail");

exit:
	s3618_reset_ctrl(ts->dev, SW_RESET_NO_INIT);

	return ret;
}

static bool testing_get_pins_assigned(unsigned short pin,
		short *tx_pins, short tx_assigned, short *rx_pins, short rx_assigned,
		short *guard_pins, short guard_assigned)
{
	int i = 0;

	for (i = 0; i < tx_assigned; i++) {
		if (pin == tx_pins[i]) {
			return true;
		}
	}
	for (i = 0; i < rx_assigned; i++) {
		if (pin == rx_pins[i]) {
			return true;
		}
	}
	for (i = 0; i < guard_assigned; i++) {
		if (pin == guard_pins[i]) {
			return true;
		}
	}

	return false;
}


static int testing_pins_mapping(unsigned char *cfg_data, unsigned int cfg_data_len)
{
	int i = 0, j = 0;
	int idx = 0;
	int offset_rx_pin = CFG_IMAGE_RXES_OFFSET/8;
	int length_rx_pin = CFG_IMAGE_RXES_LENGTH/8;
	int offset_tx_pin = CFG_IMAGE_TXES_OFFSET/8;
	int length_tx_pin = CFG_IMAGE_TXES_LENGTH/8;
	int num_rx_guard = 0;
	int offset_num_rx_guard= CFG_NUM_RXGUARD_OFFSET/8;
	int length_num_rx_guard= CFG_NUM_RXGUARD_LENGTH/8;
	int offset_rx_guard= CFG_RX_GUARD_PINS_OFFSET/8;
	int length_rx_guard= CFG_RX_GUARD_PINS_LENGTH/8;
	int num_tx_guard = 0;
	int offset_num_tx_guard= CFG_NUM_TXGUARD_OFFSET/8;
	int length_num_tx_guard= CFG_NUM_TXGUARD_LENGTH/8;
	int offset_tx_guard= CFG_TX_GUARD_PINS_OFFSET/8;
	int length_tx_guard= CFG_TX_GUARD_PINS_LENGTH/8;

	if (!cfg_data) {
		TOUCH_E("invalid parameter\n");
		return -EINVAL;
	}

	testing_hcd->tx_assigned = 0;
	testing_hcd->rx_assigned = 0;
	testing_hcd->guard_assigned = 0;

	/* get tx pins mapping */
	if (cfg_data_len > offset_tx_pin + length_tx_pin) {

		testing_hcd->tx_assigned = (length_tx_pin/2);

		idx = 0;
		for (i = 0; i < (length_tx_pin/2); i++) {
			testing_hcd->tx_pins[i] =
				(short)cfg_data[offset_tx_pin + idx] |
				(short)(cfg_data[offset_tx_pin + idx + 1] << 8);
			idx += 2;

			TOUCH_I("tx[%d] = %2d\n", i, testing_hcd->tx_pins[i]);
		}
	}

	/* get rx pins mapping */
	if (cfg_data_len > offset_rx_pin + length_rx_pin) {

		testing_hcd->rx_assigned = (length_rx_pin/2);

		idx = 0;
		for (i = 0; i < (length_rx_pin/2); i++) {
			testing_hcd->rx_pins[i] =
				(short)cfg_data[offset_rx_pin + idx] |
				(short)(cfg_data[offset_rx_pin + idx + 1] << 8);
			idx += 2;

			TOUCH_I("rx[%d] = %2d\n", i, testing_hcd->rx_pins[i]);
		}
	}

	/* get number of tx guards */
	if (cfg_data_len > offset_num_tx_guard + length_num_tx_guard) {

		num_tx_guard = (short)cfg_data[offset_num_tx_guard] |
						(short)(cfg_data[offset_num_tx_guard + 1] << 8);

		testing_hcd->guard_assigned += num_tx_guard;
	}

	/* get number of rx guards */
	if (cfg_data_len > offset_num_rx_guard + length_num_rx_guard) {

		num_rx_guard = (short)cfg_data[offset_num_rx_guard] |
						(short)(cfg_data[offset_num_rx_guard + 1] << 8);

		testing_hcd->guard_assigned += num_rx_guard;
	}

	if (testing_hcd->guard_assigned > 0) {

		TOUCH_I("num of guards = %2d (tx: %d, rx: %d)\n",
				testing_hcd->guard_assigned, num_tx_guard, num_rx_guard);

		j = 0;
	}

	/* get tx guards mapping */
	if ((num_tx_guard > 0) &&
		(cfg_data_len > offset_tx_guard + length_tx_guard)) {
		idx = 0;
		for (i = 0; i < num_tx_guard; i++) {
			testing_hcd->guard_pins[j] =
				(short)cfg_data[offset_tx_guard + idx] |
				(short)(cfg_data[offset_tx_guard + idx + 1] << 8);

			TOUCH_I("guard_pins[%d] = %2d\n", i, testing_hcd->guard_pins[j]);
			idx += 2;
			j += 1;

		}
	}

	/* get rx guards mapping */
	if ((num_rx_guard > 0) &&
		(cfg_data_len > offset_rx_guard + length_rx_guard)) {
		for (i = 0; i < num_rx_guard; i++) {
			testing_hcd->guard_pins[j] =
 				(short)cfg_data[offset_rx_guard + idx] |
				(short)(cfg_data[offset_rx_guard + idx + 1] << 8);

			TOUCH_I("guard_pins[%d] = %2d\n", i, testing_hcd->guard_pins[j]);
			idx += 2;
			j += 1;
		}
	}

	return 0;
}

static int testing_get_static_config(unsigned char *buf, unsigned int buf_len)
{
	int ret = 0;
	struct touch_core_data *ts = testing_hcd->ts;

	if (!buf) {
		TOUCH_E("invalid parameter\n");
		return -EINVAL;
	}

	mutex_lock(&testing_hcd->out.buf_mutex);
	mutex_lock(&testing_hcd->resp.buf_mutex);

	ret = s3618_write_message(ts->dev,
					CMD_GET_STATIC_CONFIG,
					NULL,
					0,
					testing_hcd->resp.buf,
					&testing_hcd->resp.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n",
				STR(CMD_GET_STATIC_CONFIG));
		goto exit;
	}

	if (testing_hcd->resp.data_length != buf_len) {
		TOUCH_E("Cfg size mismatch\n");
		ret = -EINVAL;
		goto exit;
	}

	memcpy(buf,
		testing_hcd->resp.buf,
		buf_len);

exit:
	mutex_unlock(&testing_hcd->out.buf_mutex);
	mutex_unlock(&testing_hcd->resp.buf_mutex);

	return ret;
}

static int testing_set_static_config(unsigned char *buf, unsigned int buf_len)
{
	int ret = 0;
	struct touch_core_data *ts = testing_hcd->ts;

	if (!buf) {
		TOUCH_E("invalid parameter\n");
		return -EINVAL;
	}


	mutex_lock(&testing_hcd->out.buf_mutex);

	memset(testing_hcd->out.buf, 0x00, buf_len);

	memcpy(testing_hcd->out.buf, buf, buf_len);

	mutex_lock(&testing_hcd->resp.buf_mutex);

	ret = s3618_write_message(ts->dev,
					CMD_SET_STATIC_CONFIG,
					testing_hcd->out.buf,
					buf_len,
					testing_hcd->resp.buf,
					&testing_hcd->resp.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n",
				STR(CMD_SET_STATIC_CONFIG));
	}

	mutex_unlock(&testing_hcd->resp.buf_mutex);
	mutex_unlock(&testing_hcd->out.buf_mutex);

	return ret;
}


static int testing_pt1(struct device *dev)
{
	int ret = 0;
	int i = 0, j = 0, ii = 0;
 	int phy_pin = 0;
	bool do_pin_test = false;
	bool is_rx = false;
	unsigned char *buf = NULL;
	unsigned char *pt1_data = NULL;
	unsigned int pt1_data_size = 8;
	unsigned char *satic_cfg_buf = NULL;
	unsigned int satic_cfg_length = 0;
	short *tx_pins = testing_hcd->tx_pins;
	short tx_assigned = 0;
	short *rx_pins = testing_hcd->rx_pins;
	short rx_assigned = 0;
	short *guard_pins = testing_hcd->guard_pins;
	short guard_assigned = 0;
	struct s3618_app_info *app_info = NULL;
	struct s3618_data *d = testing_hcd->d;
	struct touch_core_data *ts = testing_hcd->ts;
	int failure_cnt_pt1 = 0;
	int size = 0;

	TOUCH_I("Start pt1 testing\n");
	testing_hcd->result = false;

	memset(w_buf, 0x00, sizeof(w_buf));
	size = touch_snprintf(w_buf, sizeof(w_buf), "[PT1 Test]\n");
	//size += touch_snprintf(w_buf + size, PAGE_SIZE - size, " : ");
	//log_size = touch_snprintf(log_buf, LOG_BUF_SIZE, " : ");

	app_info = &d->app_info;

	if (!testing_hcd->satic_cfg_buf) {
		satic_cfg_length = app_info->static_config_size;

		satic_cfg_buf = kzalloc(satic_cfg_length, GFP_KERNEL);
		if (!satic_cfg_buf) {
			TOUCH_E("Failed on memory allocation for satic_cfg_buf\n");
			goto exit;
		}

		ret = testing_get_static_config(satic_cfg_buf,
										satic_cfg_length);
		if (ret < 0) {
			TOUCH_E("Failed to get static config\n");
			goto exit;
		}

		testing_hcd->satic_cfg_buf = satic_cfg_buf;
	}

	// get pins mapping
	if ( (testing_hcd->tx_assigned <= 0) ||
		 (testing_hcd->rx_assigned <= 0) ||
		 (testing_hcd->guard_assigned <= 0) ){

		if (satic_cfg_buf) {
			ret = testing_pins_mapping(satic_cfg_buf, satic_cfg_length);
			if (ret < 0) {
				TOUCH_E("Failed to get pins mapping\n");
				goto exit;
			}

			TOUCH_I("tx_assigned = %d, rx_assigned = %d, guard_assigned = %d",
					testing_hcd->tx_assigned, testing_hcd->rx_assigned,
					testing_hcd->guard_assigned);
		}
	}

	tx_assigned = testing_hcd->tx_assigned;
	rx_assigned = testing_hcd->rx_assigned;
	guard_assigned = testing_hcd->guard_assigned;

	ret = testing_run_prod_test_item(TEST_TRX_TRX_SHORTS_PT1);
	if (ret < 0) {
		TOUCH_E("Failed to run test\n");
		goto exit;
	}

	mutex_lock(&testing_hcd->resp.buf_mutex);

	if (pt1_data_size != testing_hcd->resp.data_length) {
		TOUCH_E("pt1 frame size is mismatching \n");
		ret = -EINVAL;
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		goto exit;
	}

	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	pt1_data = kzalloc(sizeof(unsigned char)*pt1_data_size, GFP_KERNEL);
	if (!pt1_data) {
		TOUCH_E("Failed to allocate mem to pt1_data\n");
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		goto exit;
	}

	memcpy(pt1_data, testing_hcd->resp.buf, pt1_data_size);

	for (i = 0; i < testing_hcd->resp.data_length; i++) {

		TOUCH_I("[%d]: %2d\n",i , pt1_data[i]);
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
			"[%d]: %2d\n",i , pt1_data[i]);

		for (j = 0; j < 8; j++) {
			phy_pin = (i*8 + j);
			do_pin_test = testing_get_pins_assigned(phy_pin,
				 					tx_pins, tx_assigned,
				 					rx_pins, rx_assigned,
				 					guard_pins, guard_assigned);

			if (do_pin_test) {

				if (CHECK_BIT(pt1_data[i], j) == 0) {
					TOUCH_I("pin-%2d : pass\n", phy_pin);
					size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
							"pin-%2d : pass\n", phy_pin);
				}
				else {

					// check pin-0, 1, 32, 33
					if (( 0 == phy_pin) || ( 1 == phy_pin) ||
						(32 == phy_pin)|| (33 == phy_pin)) {

						for (ii = 0; ii < rx_assigned; ii++) {
							is_rx = false;
							if (phy_pin == rx_pins[ii]) {
								is_rx = true;
								break;
							}
						}

						if (is_rx) {
							TOUCH_I("pin-%2d : n/a (is rx)\n", phy_pin);
							size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
									"pin-%2d : n/a (is rx)\n", phy_pin);
						} else {
							TOUCH_I("pin-%2d : fail (byte %d)\n", phy_pin, i);
							size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
									"pin-%2d : fail (byte %d)\n", phy_pin, i);

							if (size > (BUF_SIZE / 2)) {
								write_file(dev, w_buf, TIME_INFO_SKIP);
								memset(w_buf, 0, BUF_SIZE);
								size = 0;
							}

							failure_cnt_pt1 += 1;
						}
					} else {
						TOUCH_I("pin-%2d : fail (byte %d)\n", phy_pin, i);
						size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
									"pin-%2d : fail (byte %d)\n", phy_pin, i);

						if (size > (BUF_SIZE / 2)) {
							write_file(dev, w_buf, TIME_INFO_SKIP);
							memset(w_buf, 0, BUF_SIZE);
							size = 0;
						}

						failure_cnt_pt1 += 1;
					}
				}
			}// end if(do_pin_test)
		}
	}

	mutex_unlock(&testing_hcd->resp.buf_mutex);

	testing_hcd->result = (failure_cnt_pt1 == 0);

	ret = failure_cnt_pt1;

	if (!testing_hcd->need_write) {
		ret = 0;
		goto exit;
	}

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
			"Result = %s\n\n", (testing_hcd->result)?"pass":"fail");
	if(testing_hcd->need_write)
		write_file(dev, w_buf, TIME_INFO_SKIP);
	TOUCH_I("Result = %s\n", (testing_hcd->result)?"pass":"fail");

exit:
	if (pt1_data)
		kfree(pt1_data);
	if (satic_cfg_buf)
		kfree(satic_cfg_buf);

	s3618_reset_ctrl(ts->dev, SW_RESET_NO_INIT);

	return ret;
}

static int testing_trx_ground(struct device *dev)
{
	int retval = 0;
	int i = 0, j = 0;
	int phy_pin = 0;
	struct s3618_data *d = testing_hcd->d;
	struct touch_core_data *ts = testing_hcd->ts;
	unsigned int limits = 0;
	unsigned char data = 0;
	int size = 0;
	unsigned char *satic_cfg_buf = NULL;
	unsigned int satic_cfg_length = 0;
	short *tx_pins = testing_hcd->tx_pins;
	short tx_assigned = 0;
	short *rx_pins = testing_hcd->rx_pins;
	short rx_assigned = 0;
	short *guard_pins = testing_hcd->guard_pins;
	short guard_assigned = 0;
	struct s3618_app_info *app_info = NULL;
	bool do_pin_test = false;

	TOUCH_I("Start TRX Ground testing\n");

	memset(w_buf, 0x00, sizeof(w_buf));
	size = touch_snprintf(w_buf, sizeof(w_buf), "[TRX Ground Test]\n");

		app_info = &d->app_info;

	if (!testing_hcd->satic_cfg_buf) {
		satic_cfg_length = app_info->static_config_size;

		satic_cfg_buf = kzalloc(satic_cfg_length, GFP_KERNEL);
		if (!satic_cfg_buf) {
			TOUCH_E("Failed on memory allocation for satic_cfg_buf\n");
			goto exit;
		}

		retval = testing_get_static_config(satic_cfg_buf,
										satic_cfg_length);
		if (retval < 0) {
			TOUCH_E("Failed to get static config\n");
			goto exit;
		}

		testing_hcd->satic_cfg_buf = satic_cfg_buf;
	}

	// get pins mapping
	if ( (testing_hcd->tx_assigned <= 0) ||
		 (testing_hcd->rx_assigned <= 0) ||
		 (testing_hcd->guard_assigned <= 0) ){

		if (satic_cfg_buf) {
			retval = testing_pins_mapping(satic_cfg_buf, satic_cfg_length);
			if (retval < 0) {
				TOUCH_E("Failed to get pins mapping\n");
				goto exit;
			}

			TOUCH_I("tx_assigned = %d, rx_assigned = %d, guard_assigned = %d",
					testing_hcd->tx_assigned, testing_hcd->rx_assigned,
					testing_hcd->guard_assigned);
		}
	}

	tx_assigned = testing_hcd->tx_assigned;
	rx_assigned = testing_hcd->rx_assigned;
	guard_assigned = testing_hcd->guard_assigned;

	retval = testing_run_prod_test_item(TEST_TRX_GROUND_SHORTS_PT3);
	if (retval < 0) {
		TOUCH_E("Failed to run test\n");
		goto exit;
	}

	mutex_lock(&testing_hcd->resp.buf_mutex);

	limits = sizeof(trx_ground_limits) / sizeof(trx_ground_limits[0]);

	if (limits < testing_hcd->resp.data_length) {
		TOUCH_E("Mismatching limits data\n");
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		retval = -EINVAL;
		goto exit;
	}

	testing_hcd->result = true;

	for (i = 0; i < testing_hcd->resp.data_length; i++) {
		data = testing_hcd->resp.buf[i];
		TOUCH_I("[%d]: 0x%02x\n", i , data);
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
			"[%d]: 0x%02x\n", i , data);

		for (j = 0; j < 8; j++) {
			phy_pin = (i*8 + j);
			do_pin_test = testing_get_pins_assigned(phy_pin,
									tx_pins, tx_assigned,
									rx_pins, rx_assigned,
									guard_pins, guard_assigned);

			if (do_pin_test) {
				if (CHECK_BIT(data, j) != CHECK_BIT(trx_ground_limits[i], j)) {
					TOUCH_E("pin-%2d : fail\n", phy_pin);
					size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
							"pin-%2d : fail\n", phy_pin);

					if (size > (BUF_SIZE / 2)) {
						write_file(dev, w_buf, TIME_INFO_SKIP);
						memset(w_buf, 0, BUF_SIZE);
						size = 0;
					}

					testing_hcd->result = false;
				} else {
					TOUCH_I("pin-%2d : pass\n", phy_pin);
					size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
							"pin-%2d : pass\n", phy_pin);
				}
			}
		}
	}

	mutex_unlock(&testing_hcd->resp.buf_mutex);

	if (!testing_hcd->need_write) {
		retval = 0;
		goto exit;
	}

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
			"Result = %s\n\n", (testing_hcd->result)?"pass":"fail");
	if(testing_hcd->need_write)
		write_file(dev, w_buf, TIME_INFO_SKIP);
	TOUCH_I("Result = %s\n", (testing_hcd->result)?"pass":"fail");

exit:
	if (satic_cfg_buf)
		kfree(satic_cfg_buf);

	s3618_reset_ctrl(ts->dev, SW_RESET_NO_INIT);

	return retval;

}


static int testing_discrete_ex_trx_short(struct device *dev)
{
	int ret = 0;
	int i = 0, j = 0, k = 0, pin = 0;
	unsigned char *buf = NULL;
	unsigned int rows = 0;
	unsigned int cols = 0;
	unsigned char *satic_cfg_buf = NULL;
	unsigned int satic_cfg_length = 0;
	struct s3618_app_info *app_info = NULL;
	struct s3618_data *d = testing_hcd->d;
	struct touch_core_data *ts = testing_hcd->ts;
//	int result_pt1 = 0;
	int result_pt196 = 0;
	unsigned int pt196_size = 0;
	unsigned short *pt196_base = 0;
	unsigned short *pt196_delta = 0;
	unsigned short *min_rx = NULL;
	unsigned short *max_rx = NULL;
	unsigned short tmp_data = 0;
	unsigned short extend[4] = {0, 1, 32, 33};
	bool do_pin_test = 0;
	unsigned short logical_pin = 0;
	int size = 0;
	char log_buf[LOG_BUF_SIZE] = {0, };
	int log_size = 0;

	TOUCH_I("Start Discrete ex trx short testing\n");
	testing_hcd->result = false;

	memset(w_buf, 0x00, sizeof(w_buf));
	size = touch_snprintf(w_buf, sizeof(w_buf), "[Discrete ex trx short Test]\n");

	app_info = &d->app_info;

	rows = app_info->num_of_image_rows;
	cols = app_info->num_of_image_cols;

	pt196_size = rows * cols * 2;

	satic_cfg_length = app_info->static_config_size;

	satic_cfg_buf = kzalloc(satic_cfg_length, GFP_KERNEL);
	if (!satic_cfg_buf) {
		TOUCH_E("Failed on memory allocation for satic_cfg_buf\n");
		goto exit;
	}

	ret = testing_get_static_config(satic_cfg_buf, satic_cfg_length);
	if (ret < 0) {
		TOUCH_E("Failed to get static config\n");
		goto exit;
	}

	testing_hcd->satic_cfg_buf = satic_cfg_buf;

	// get pins mapping
	if ( (testing_hcd->tx_assigned <= 0) ||
		 (testing_hcd->rx_assigned <= 0) ||
		 (testing_hcd->guard_assigned <= 0) ){

		if (satic_cfg_buf) {
			ret = testing_pins_mapping(satic_cfg_buf, satic_cfg_length);
			if (ret < 0) {
				TOUCH_E("Failed to get pins mapping\n");
				goto exit;
			}

			TOUCH_I("tx_assigned = %d, rx_assigned = %d, guard_assigned = %d",
					testing_hcd->tx_assigned, testing_hcd->rx_assigned,
					testing_hcd->guard_assigned);
		}
	}

	// do pt1 testing
//	ret = testing_pt1(dev);
//	if (ret < 0) {
//		TOUCH_E("Failed to run pt1 test\n");
//		goto exit;
//	}
//	result_pt1 = ret;  // copy the result of pt1

	// do sw reset
//	s3618_reset_ctrl(ts->dev, SW_RESET);

	// change analog settings
	for (i = 0; i < (CFG_IMAGE_CBCS_LENGTH/8); i++) {
		satic_cfg_buf[(CFG_IMAGE_CBCS_OFFSET/8) + i] = 0x00;
	}
	satic_cfg_buf[(CFG_REF_LO_TRANS_CAP_OFFSET/8)] = 0x06;
	satic_cfg_buf[(CFG_REF_LO_XMTR_PL_OFFSET/8)] = 0x01;
	satic_cfg_buf[(CFG_REF_HI_TRANS_CAP_OFFSET/8)] = 0x06;
	satic_cfg_buf[(CFG_REF_HI_XMTR_PL_OFFSET/8)] = 0x00;
	satic_cfg_buf[(CFG_REF_GAIN_CTRL_OFFSET/8)] = 0x00;

	ret = testing_set_static_config(satic_cfg_buf,
									satic_cfg_length);
	if (ret < 0) {
		TOUCH_E("Failed to set static config\n");
		goto exit;
	}

	// get pt196 as baseline
	ret = testing_run_prod_test_item(TEST_DISCRETE_PT196);
	if (ret < 0) {
		TOUCH_E("Failed to get pt196 base image\n");
		goto exit;
	}

	mutex_lock(&testing_hcd->resp.buf_mutex);

	if (pt196_size != testing_hcd->resp.data_length) {
		TOUCH_E("Pt196 size is mismatching\n");
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		ret = -EINVAL;
		goto exit;
	}

	buf = testing_hcd->resp.buf;
	pt196_base = kzalloc(sizeof(unsigned short)*rows*cols, GFP_KERNEL);
	if (!pt196_base) {
		TOUCH_E("Failed on memory allocation for pt196_base\n");
		mutex_unlock(&testing_hcd->resp.buf_mutex);
		ret = -EINVAL;
		goto exit;
	}

	k = 0;
	for (i = 0; i < rows; i++) {
		for (j = 0; j < cols; j++) {
			pt196_base[i * cols + j] =
					(unsigned short)(buf[k] &0xff) |
					(unsigned short)(buf[k+1] << 8);
			k += 2;
		}
	}

	mutex_unlock(&testing_hcd->resp.buf_mutex);

	pt196_delta = kzalloc(sizeof(unsigned short)*rows*cols, GFP_KERNEL);
	if (!pt196_delta) {
		TOUCH_E("Failed on memory allocation for pt196_delta\n");
		ret = -EINVAL;
		goto exit;
	}
	min_rx= kzalloc(sizeof(unsigned short)*(testing_hcd->rx_assigned),
					GFP_KERNEL);
	if (!min_rx) {
		TOUCH_E("Failed on memory allocation for min_rx\n");
		ret = -EINVAL;
		goto exit;
	}
	max_rx = kzalloc(sizeof(unsigned short)*(testing_hcd->rx_assigned),
					 GFP_KERNEL);
	if (!max_rx) {
		TOUCH_E("Failed on memory allocation for max_rx\n");
		ret = -EINVAL;
		goto exit;
	}

	// walk through all extend pins
	for (pin = 0; pin < 4; pin++) {

		do_pin_test = testing_get_pins_assigned(extend[pin],
				 		testing_hcd->tx_pins, testing_hcd->tx_assigned,
				 		testing_hcd->rx_pins, testing_hcd->rx_assigned,
				 		testing_hcd->guard_pins, testing_hcd->guard_assigned);
		if (!do_pin_test)
			continue;  // skip if pin is not assigned

		for (i = 0; i < testing_hcd->rx_assigned; i++) {
			do_pin_test = false;
			if (extend[pin] == testing_hcd->rx_pins[i]) {
				do_pin_test = true;
				logical_pin = i;
				break;
			}
		}

		if (!do_pin_test)
			continue;  // skip if pin is not rx


		for (i = 0; i < testing_hcd->rx_assigned; i++) {
			min_rx[i] = 5000;
			max_rx[i] = 0;
		}

		TOUCH_I("pin = %d, logical pin = %d\n", extend[pin], logical_pin);

		// adjust cbc for the target logical pin
		for (i = 0; i < (CFG_IMAGE_CBCS_LENGTH/8); i++) {
			if (i == logical_pin*2)
				satic_cfg_buf[(CFG_IMAGE_CBCS_OFFSET/8) + i] = 0x0f;
			else
				satic_cfg_buf[(CFG_IMAGE_CBCS_OFFSET/8) + i] = 0x00;
		}
		ret = testing_set_static_config(satic_cfg_buf,
									satic_cfg_length);
		if (ret < 0) {
			TOUCH_E("Failed to set static config for logical pin %d\n",
					logical_pin);
			goto exit;
		}

		// get pt196 again
		// and do calculation to get the delta
		ret = testing_run_prod_test_item(TEST_DISCRETE_PT196);
		if (ret < 0) {
			TOUCH_E("Failed to get pt196 base image\n");
			goto exit;
		}

		mutex_lock(&testing_hcd->resp.buf_mutex);

		if (pt196_size != testing_hcd->resp.data_length) {
			TOUCH_E("pt196_size mismatch\n");
			mutex_unlock(&testing_hcd->resp.buf_mutex);
			ret = -EINVAL;
			goto exit;
		}

		buf = testing_hcd->resp.buf;
		k = 0;
		for (i = 0; i < rows; i++) {
			for (j = 0; j < cols; j++) {

				tmp_data = (unsigned short)(buf[k] &0xff) |
						(unsigned short)(buf[k+1] << 8);
				pt196_delta[i * cols + j] =
						abs(tmp_data - pt196_base[i * cols + j]);

				if (testing_hcd->rx_assigned == cols) {
					min_rx[j] = MIN(min_rx[j], pt196_delta[i * cols + j]);
					max_rx[j] = MAX(max_rx[j], pt196_delta[i * cols + j]);
				}
				else if (testing_hcd->rx_assigned == rows) {
					min_rx[i] = MIN(min_rx[i], pt196_delta[i * cols + j]);
					max_rx[i] = MAX(max_rx[i], pt196_delta[i * cols + j]);
				}

				k += 2;
			}
		}

		mutex_unlock(&testing_hcd->resp.buf_mutex);

		if (testing_hcd->need_write) {
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
					"pin = %d, logical pin = %d\n", extend[pin], logical_pin);
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "  :  ");
			log_size = touch_snprintf(log_buf, LOG_BUF_SIZE, "  :  ");
			for (i = 0; i < rows; i++) {
				size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "  [%2d]", i);
				log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "  [%2d]", i);
			}
		}

		log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "\n");

		for (j = 0; j < cols; j++) {
			size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n [%2d]", j);
			log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, " [%2d]", j);
			for (i = 0; i < rows; i++) {
				size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "%6d", pt196_delta[i * cols + j]);
				log_size += touch_snprintf(log_buf + log_size, LOG_BUF_SIZE - log_size, "%6d", pt196_delta[i * cols + j]);
			}
			TOUCH_I("%s\n", log_buf);
			memset(log_buf, 0, sizeof(log_buf));
			log_size = 0;
		}
		// data verification
		for (i = 0; i < testing_hcd->rx_assigned; i++) {

			if (i == logical_pin) {
				// the delta should be higher than limit
				if (min_rx[i] < discrete_ex_trx_short_upper_limit) {
					size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
							"fail at pin %d (logical: %d), data = (%d, %d)\n",
							extend[pin], logical_pin, min_rx[i], max_rx[i]);
					TOUCH_I("fail at pin %d (logical: %d), data = (%d, %d)\n",
							extend[pin], logical_pin, min_rx[i], max_rx[i]);

					if (size > (BUF_SIZE / 2)) {
						write_file(dev, w_buf, TIME_INFO_SKIP);
						memset(w_buf, 0, BUF_SIZE);
						size = 0;
					}

					result_pt196 += 1;
				}
			} else {
				// if it is not the extended pin
				// the delta should be less than limit
				if (max_rx[i] >= discrete_ex_trx_short_lower_limit) {
					size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
							"fail at logical pin %d, data = (%d, %d)\n",
							i, min_rx[i], max_rx[i]);
					TOUCH_I("fail at logical pin %d, data = (%d, %d)\n",
							i, min_rx[i], max_rx[i]);

					if (size > (BUF_SIZE / 2)) {
						write_file(dev, w_buf, TIME_INFO_SKIP);
						memset(w_buf, 0, BUF_SIZE);
						size = 0;
					}

					result_pt196 += 1;
				}
			}
		}
		size += touch_snprintf(w_buf + size, sizeof(w_buf) - size, "\n");

		if(testing_hcd->need_write)
			write_file(dev, w_buf, TIME_INFO_SKIP);
		memset(w_buf, 0x00, sizeof(w_buf));
		size = 0;
	}


	ret = 0;
	//testing_hcd->result = (result_pt1 == 0) & (result_pt196 == 0);
	testing_hcd->result = (result_pt196 == 0);

	if (!testing_hcd->need_write) {
		ret = 0;
		goto exit;
	}

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
			"Result = %s\n\n", (testing_hcd->result)?"pass":"fail");
	if(testing_hcd->need_write)
		write_file(dev, w_buf, TIME_INFO_SKIP);

	TOUCH_I("Result = %s\n", (testing_hcd->result)?"pass":"fail");

exit:
	if (satic_cfg_buf)
		kfree(satic_cfg_buf);
	if (pt196_base)
		kfree(pt196_base);
	if (pt196_delta)
		kfree(pt196_delta);
	if (min_rx)
		kfree(min_rx);
	if (max_rx)
		kfree(max_rx);

	s3618_reset_ctrl(ts->dev, SW_RESET_NO_INIT);

	return ret;
}

static int testing_reset_open(struct device *dev)
{
	int ret = 0;
	struct s3618_data *d = testing_hcd->d;
	struct touch_core_data *ts = testing_hcd->ts;
	int size = 0;

	TOUCH_I("Start Reset Open testing\n");
	testing_hcd->result = false;

	memset(w_buf, 0x00, sizeof(w_buf));
	size = touch_snprintf(w_buf, sizeof(w_buf), "[Reset Open Test]\n");

	if (ts->reset_pin < 0) {
		TOUCH_E("Hardware reset unavailable\n");
		return -EINVAL;
	}

	mutex_lock(&d->reset_mutex);

	gpio_set_value(ts->reset_pin, 0);
	touch_msleep(20);
	gpio_set_value(ts->reset_pin, 1);
	touch_msleep(ts->caps.hw_reset_delay);

	mutex_unlock(&d->reset_mutex);

	if (d->id_info.mode == MODE_APPLICATION_FIRMWARE) {
		ret = s3618_switch_mode(ts->dev, MODE_BOOTLOADER);
		if (ret < 0) {
			TOUCH_E("Failed to enter bootloader mode\n");
			return ret;
		}
	} else {
		ret = s3618_ic_info(ts->dev);
		if (ret < 0) {
			TOUCH_E("Failed to get ic_info\n");
			return ret;
		}
	}

	if (d->boot_info.last_reset_reason == reset_open_limit)
		testing_hcd->result = true;
	else
		testing_hcd->result = false;

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
			"Last reset reason = %d\n", d->boot_info.last_reset_reason);

	ret = 0;

//run_app_firmware:
	ret = s3618_switch_mode(ts->dev, MODE_APPLICATION_FIRMWARE);
	if (ret < 0) {
		TOUCH_E("Failed to run application firmware\n");
		return ret;
	}

	size += touch_snprintf(w_buf + size, sizeof(w_buf) - size,
			"Result = %s\n\n", (testing_hcd->result)?"pass":"fail");

	if (testing_hcd->need_write)
		write_file(dev, w_buf, TIME_INFO_SKIP);


	TOUCH_I("Result = %s\n", (testing_hcd->result)?"pass":"fail");

	s3618_reset_ctrl(ts->dev, SW_RESET_NO_INIT);

	return ret;
}


static void testing_report(void)
{
	unsigned int offset = 0;
	unsigned int report_size = 0;
	struct s3618_data *d = testing_hcd->d;

	report_size = d->report.data_length;

	mutex_lock(&testing_hcd->report.buf_mutex);

	if (testing_hcd->report_index == 0) {
		memset(&testing_hcd->report,
			0x00,
			report_size * testing_hcd->num_of_reports);
	}

	if (testing_hcd->report_index < testing_hcd->num_of_reports) {
		offset = report_size * testing_hcd->report_index;

		memcpy(testing_hcd->report.buf + offset,
			d->report.buf,
			d->report.data_length);

		testing_hcd->report_index++;
		testing_hcd->report.data_length += report_size;
	}

	mutex_unlock(&testing_hcd->report.buf_mutex);

	if (testing_hcd->report_index == testing_hcd->num_of_reports)
		complete(&report_complete);

	return;
}


static int testing_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;

	if (testing_hcd) {
		TOUCH_I("%s: Handle already exists\n", __func__);
		return 0;
	}

	testing_hcd = kzalloc(sizeof(*testing_hcd), GFP_KERNEL);
	if (!testing_hcd) {
		TOUCH_E("Failed to allocate memory for testing_hcd\n");
		return -ENOMEM;
	}

	testing_hcd->ts = ts;
	testing_hcd->d = d;

	testing_hcd->collect_reports = testing_collect_reports;

	mutex_init(&testing_hcd->out.buf_mutex);
	mutex_init(&testing_hcd->resp.buf_mutex);
	mutex_init(&testing_hcd->report.buf_mutex);
	mutex_init(&testing_hcd->process.buf_mutex);
	mutex_init(&testing_hcd->output.buf_mutex);

	testing_hcd->sysfs_dir = kobject_create_and_add(SYSFS_DIR_NAME,
			&ts->input->dev.kobj);
	if (!testing_hcd->sysfs_dir) {
		TOUCH_E("Failed to create sysfs directory\n");
		ret = -EINVAL;
		goto err_sysfs_create_dir;
	}

	ret = sysfs_create_group(&ts->kobj, &prd_attribute_group);
	if (ret < 0) {
			TOUCH_E("filed to create prd sysfs group");
	}

	ret = sysfs_create_bin_file(testing_hcd->sysfs_dir, &bin_attr);
	if (ret < 0) {
		TOUCH_E("Failed to create sysfs bin file\n");
		goto err_sysfs_create_bin_file;
	}

	return 0;

err_sysfs_create_bin_file:

	kobject_put(testing_hcd->sysfs_dir);

err_sysfs_create_dir:

	kfree(testing_hcd);
	testing_hcd = NULL;

	return ret;
}

static void testing_remove(struct device *dev)
{
	if (!testing_hcd)
		goto exit;

	sysfs_remove_bin_file(testing_hcd->sysfs_dir, &bin_attr);

	kobject_put(testing_hcd->sysfs_dir);

	kfree(testing_hcd);
	testing_hcd = NULL;

exit:
	complete(&testing_remove_complete);

	return;
}

static void testing_reinit(struct device *dev)
{
	if (!testing_hcd) {
		testing_init(dev);
	}

	return;
}

static void testing_syncbox(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);

	if (!testing_hcd)
		return;

	if (d->status_report_code == testing_hcd->report_type)
		testing_report();

	return;
}

static struct s3618_exp_fn testing_module = {
	.init = testing_init,
	.remove = testing_remove,
	.syncbox = testing_syncbox,
#ifdef REPORT_NOTIFIER
	.asyncbox = NULL,
#endif
	.reinit = testing_reinit,
	.suspend = NULL,
	.resume = NULL,
	.early_suspend = NULL,
};

static int __init testing_module_init(void)
{
	//return syna_tcm_add_module(&testing_module, true);
	s3618_testing_function(&testing_module, true);

	return 0;
}

static void __exit testing_module_exit(void)
{
	//syna_tcm_add_module(&testing_module, false);
	s3618_testing_function(&testing_module, false);

	wait_for_completion(&testing_remove_complete);

	return;
}

module_init(testing_module_init);
module_exit(testing_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TCM Testing Module");
MODULE_LICENSE("GPL v2");
