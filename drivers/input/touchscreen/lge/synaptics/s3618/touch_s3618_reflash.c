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
#include <linux/crc32.h>
#include <linux/firmware.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_s3618.h"

#define FORCE_REFLASH false

#define ENABLE_SYS_REFLASH true

#define SYSFS_DIR_NAME "reflash"

#define CUSTOM_DIR_NAME "custom"

#define FW_IMAGE_NAME "synaptics/reflash_firmware.img"

#define FW_IMAGE_NAME_MANUAL "synaptics/reflash_firmware_manual.img"

#define BOOT_CONFIG_ID "BOOT_CONFIG"

#define APP_CODE_ID "APP_CODE"

#define PROD_TEST_ID "APP_PROD_TEST"

#define APP_CONFIG_ID "APP_CONFIG"

#define DISP_CONFIG_ID "DISPLAY"

#define FB_READY_COUNT 2

#define FB_READY_WAIT_MS 100

#define FB_READY_TIMEOUT_S 30

#define IMAGE_FILE_MAGIC_VALUE 0x4818472b

#define FLASH_AREA_MAGIC_VALUE 0x7c05e516

#define BOOT_CONFIG_SIZE 8

#define BOOT_CONFIG_SLOTS 16

#define IMAGE_BUF_SIZE (512 * 1024)

#define ERASE_FLASH_DELAY_MS 500

#define WRITE_FLASH_DELAY_MS 20

#define REFLASH (1 << 0)

#define FORCE_UPDATE (1 << 1)

#define APP_CFG_UPDATE (1 << 2)

#define DISP_CFG_UPDATE (1 << 3)

#define BOOT_CFG_UPDATE (1 << 4)

#define BOOT_CFG_LOCKDOWN (1 << 5)

#define reflash_show_data() \
{ \
	mutex_lock(&reflash_hcd->read.buf_mutex); \
	readlen = MIN(count, reflash_hcd->read.data_length - pos); \
	memcpy(buf, &reflash_hcd->read.buf[pos], readlen); \
	mutex_unlock(&reflash_hcd->read.buf_mutex); \
}

enum update_area {
	NONE = 0,
	FIRMWARE_CONFIG,
	CONFIG_ONLY,
};

struct app_config_header {
	unsigned short magic_value[4];
	u32 checksum;
	u16 length;
	u32 build_id;
	struct s3618_customer_config_id customer_config_id;
	//unsigned char customer_config_id[16];
} __packed;

struct area_descriptor {
	unsigned char magic_value[4];
	unsigned char id_string[16];
	unsigned char flags[4];
	unsigned char flash_addr_words[4];
	unsigned char length[4];
	unsigned char checksum[4];
};

struct block_data {
	const unsigned char *data;
	unsigned int size;
	unsigned int flash_addr;
};

struct image_info {
	struct block_data boot_config;
	struct block_data app_firmware;
	struct block_data prod_test_firmware;
	struct block_data app_config;
	struct block_data disp_config;
};

struct image_header {
	unsigned char magic_value[4];
	unsigned char num_of_areas[4];
};

struct boot_config {
	union {
		unsigned char i2c_address;
		struct {
			unsigned char cpha:1;
			unsigned char cpol:1;
			unsigned char word0_b2__7:6;
		} __packed;
	};
	unsigned char attn_polarity:1;
	unsigned char attn_drive:2;
	unsigned char attn_pullup:1;
	unsigned char word0_b12__14:3;
	unsigned char used:1;
	unsigned short customer_part_id;
	unsigned short boot_timeout;
	unsigned short continue_on_reset:1;
	unsigned short word3_b1__15:15;
} __packed;

struct reflash_hcd {
	bool force_update;
	bool disp_cfg_update;
	bool reflash_by_manual;
	const unsigned char *image;
	unsigned char *image_buf;
	unsigned int image_size;
	unsigned int page_size;
	unsigned int write_block_size;
	unsigned int max_write_payload_size;
	const struct firmware *fw_entry;
	struct mutex reflash_mutex;
	struct kobject *sysfs_dir;
	struct kobject *custom_dir;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct image_info image_info;
	//struct syna_tcm_buffer out;
	//struct syna_tcm_buffer resp;
	//struct syna_tcm_buffer read;
	//struct syna_tcm_hcd *tcm_hcd;

	struct touch_core_data *ts;
	struct s3618_data *d;
	struct s3618_buffer out;
	struct s3618_buffer response;
	struct s3618_buffer read;
};

DECLARE_COMPLETION(reflash_remove_complete);

static struct reflash_hcd *reflash_hcd;

static int reflash_get_fw_image(void);

static int reflash_read_data(enum flash_area area, bool run_app_firmware);

static int reflash_update_custom_otp(const unsigned char *data,
		unsigned int offset, unsigned int datalen);

static int reflash_update_custom_lcm(const unsigned char *data,
		unsigned int offset, unsigned int datalen);

static int reflash_update_custom_oem(const unsigned char *data,
		unsigned int offset, unsigned int datalen);

static int reflash_update_boot_config(bool lock);

static int reflash_update_app_config(bool reset);

static int reflash_update_disp_config(bool reset);

static int reflash_do_reflash(void);

STORE_PROTOTYPE(reflash, reflash)

static struct device_attribute *attrs[] = {
	ATTRIFY(reflash),
};

static ssize_t reflash_sysfs_image_store(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t reflash_sysfs_lockdown_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t reflash_sysfs_lockdown_store(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t reflash_sysfs_lcm_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t reflash_sysfs_lcm_store(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t reflash_sysfs_oem_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t reflash_sysfs_oem_store(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t reflash_sysfs_cs_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static struct bin_attribute bin_attrs[] = {
	{
		.attr = {
			.name = "image",
			.mode = (S_IWUSR | S_IWGRP),
		},
		.size = 0,
		.write = reflash_sysfs_image_store,
	},
	{
		.attr = {
			.name = "lockdown",
			.mode = (S_IRUGO | S_IWUSR | S_IWGRP),
		},
		.size = 0,
		.read = reflash_sysfs_lockdown_show,
		.write = reflash_sysfs_lockdown_store,
	},
	{
		.attr = {
			.name = "lcm",
			.mode = (S_IRUGO | S_IWUSR | S_IWGRP),
		},
		.size = 0,
		.read = reflash_sysfs_lcm_show,
		.write = reflash_sysfs_lcm_store,
	},
	{
		.attr = {
			.name = "oem",
			.mode = (S_IRUGO | S_IWUSR | S_IWGRP),
		},
		.size = 0,
		.read = reflash_sysfs_oem_show,
		.write = reflash_sysfs_oem_store,
	},
	{
		.attr = {
			.name = "customer_serialization",
			.mode = (S_IRUGO | S_IRUSR | S_IRGRP),
		},
		.size = 0,
		.read = reflash_sysfs_cs_show,
	},
};

static ssize_t reflash_sysfs_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int input = 0;
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	mutex_lock(&d->extif_mutex);

	pm_stay_awake(ts->dev);

	mutex_lock(&reflash_hcd->reflash_mutex);

	if (reflash_hcd->image_size != 0)
		reflash_hcd->image = reflash_hcd->image_buf;

	reflash_hcd->force_update = input & FORCE_UPDATE ? true : false;

	reflash_hcd->reflash_by_manual = true;

	if (input & REFLASH || input & FORCE_UPDATE) {
		ret = reflash_do_reflash();
		if (ret < 0) {
			TOUCH_E("Failed to do reflash\n");
			goto exit;
		}
	}

	if ((input & ~(REFLASH | FORCE_UPDATE)) == 0) {
		ret = count;
		goto exit;
	}

	ret = reflash_get_fw_image();
	if (ret < 0) {
		TOUCH_E("Failed to get firmware image\n");
		goto exit;
	}

	if (input & BOOT_CFG_LOCKDOWN) {
		ret = reflash_update_boot_config(true);
		if (ret < 0) {
			TOUCH_E("Failed to lockdown boot config\n");
			goto exit;
		}
	} else if (input & BOOT_CFG_UPDATE) {
		ret = reflash_update_boot_config(false);
		if (ret < 0) {
			TOUCH_E("Failed to update boot config\n");
			goto exit;
		}
	}

	if (input & REFLASH || input & FORCE_UPDATE) {
		ret = count;
		goto exit;
	}

	if (input & DISP_CFG_UPDATE) {
		if (input & APP_CFG_UPDATE)
			ret = reflash_update_disp_config(false);
		else
			ret = reflash_update_disp_config(true);
		if (ret < 0) {
			TOUCH_E("Failed to reflash display config\n");
			goto exit;
		}
	}

	if (input & APP_CFG_UPDATE) {
		ret = reflash_update_app_config(true);
		if (ret < 0) {
			TOUCH_E("Failed to reflash application config\n");
			goto exit;
		}
	}

	ret = count;

exit:
	if (reflash_hcd->fw_entry) {
		release_firmware(reflash_hcd->fw_entry);
		reflash_hcd->fw_entry = NULL;
	}

	reflash_hcd->reflash_by_manual = false;
	reflash_hcd->image = NULL;
	reflash_hcd->image_size = 0;
	reflash_hcd->force_update = FORCE_REFLASH;

	mutex_unlock(&reflash_hcd->reflash_mutex);

	pm_relax(ts->dev);

	mutex_unlock(&d->extif_mutex);

	return ret;
}

static ssize_t reflash_sysfs_image_store(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int ret = 0;
	struct s3618_data *d = reflash_hcd->d;

	mutex_lock(&d->extif_mutex);

	memcpy(&reflash_hcd->image_buf[pos], buf, count);

	reflash_hcd->image_size = pos + count;

	ret = count;

	mutex_unlock(&d->extif_mutex);

	return ret;
}

static ssize_t reflash_sysfs_lockdown_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int ret = 0;
	unsigned int readlen = 0;
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;

	mutex_lock(&ts->lock);
	mutex_lock(&d->extif_mutex);

	mutex_lock(&reflash_hcd->reflash_mutex);

	ret = reflash_read_data(CUSTOM_OTP, true);
	if (ret < 0) {
		TOUCH_E("Failed to read lockdown data\n");
		goto exit;
	}

	reflash_show_data();

exit:
	mutex_unlock(&reflash_hcd->reflash_mutex);

	mutex_unlock(&d->extif_mutex);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t reflash_sysfs_lockdown_store(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int ret = 0;
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;

	mutex_lock(&d->extif_mutex);

	pm_stay_awake(ts->dev);

	mutex_lock(&reflash_hcd->reflash_mutex);

	ret = reflash_update_custom_otp(buf, pos, count);
	if (ret < 0) {
		TOUCH_E("Failed to update custom OTP data\n");
		goto exit;
	}

	ret = count;

exit:
	mutex_unlock(&reflash_hcd->reflash_mutex);

	pm_relax(ts->dev);

	mutex_unlock(&d->extif_mutex);

	return ret;
}

static ssize_t reflash_sysfs_lcm_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int ret = 0;
	unsigned int readlen = 0;
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;

	mutex_lock(&ts->lock);
	mutex_lock(&d->extif_mutex);

	mutex_lock(&reflash_hcd->reflash_mutex);

	ret = reflash_read_data(CUSTOM_LCM, true);
	if (ret < 0) {
		TOUCH_E("Failed to read LCM data\n");
		goto exit;
	}

	reflash_show_data();

exit:
	mutex_unlock(&reflash_hcd->reflash_mutex);

	mutex_unlock(&d->extif_mutex);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t reflash_sysfs_lcm_store(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int ret = 0;
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;

	mutex_lock(&d->extif_mutex);

	pm_stay_awake(ts->dev);

	mutex_lock(&reflash_hcd->reflash_mutex);

	ret = reflash_update_custom_lcm(buf, pos, count);
	if (ret < 0) {
		TOUCH_E("Failed to update custom LCM data\n");
		goto exit;
	}

	ret = count;

exit:
	mutex_unlock(&reflash_hcd->reflash_mutex);

	pm_relax(ts->dev);

	mutex_unlock(&d->extif_mutex);

	return ret;
}

static ssize_t reflash_sysfs_oem_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int ret = 0;
	unsigned int readlen = 0;
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;

	mutex_lock(&ts->lock);
	mutex_lock(&d->extif_mutex);

	mutex_lock(&reflash_hcd->reflash_mutex);

	ret = reflash_read_data(CUSTOM_OEM, true);
	if (ret < 0) {
		TOUCH_E("Failed to read OEM data\n");
		goto exit;
	}

	reflash_show_data();

exit:
	mutex_unlock(&reflash_hcd->reflash_mutex);

	mutex_unlock(&d->extif_mutex);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t reflash_sysfs_oem_store(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int ret = 0;
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;

	mutex_lock(&d->extif_mutex);

	pm_stay_awake(ts->dev);

	mutex_lock(&reflash_hcd->reflash_mutex);

	ret = reflash_update_custom_oem(buf, pos, count);
	if (ret < 0) {
		TOUCH_E("Failed to update custom OEM data\n");
		goto exit;
	}

	ret = count;

exit:
	mutex_unlock(&reflash_hcd->reflash_mutex);

	pm_relax(ts->dev);

	mutex_unlock(&d->extif_mutex);

	return ret;
}

static ssize_t reflash_sysfs_cs_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int ret = 0;
	unsigned int readlen = 0;
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;

	mutex_lock(&ts->lock);
	mutex_lock(&d->extif_mutex);

	mutex_lock(&reflash_hcd->reflash_mutex);

	ret = reflash_read_data(BOOT_CONFIG, true);
	if (ret < 0) {
		TOUCH_E("Failed to read OEM data\n");
		goto exit;
	}

	reflash_show_data();

exit:
	mutex_unlock(&reflash_hcd->reflash_mutex);

	mutex_unlock(&d->extif_mutex);
	mutex_unlock(&ts->lock);

	return ret;
}

#define PRD_BLOCK_MAX	(8)
#define PRD_SIZE_PER_BLOCK	(16)

int reflash_read_product_info(void)
{
	struct s3618_data *d = reflash_hcd->d;
	int ret = 0;
	int i = 0;
	int write_count = 0;

	TOUCH_I("%s", __func__);
	mutex_lock(&d->extif_mutex);

	mutex_lock(&reflash_hcd->reflash_mutex);

	ret = reflash_read_data(BOOT_CONFIG, true);
	if (ret < 0) {
		TOUCH_E("Failed to read OEM data\n");
		goto exit;
	}


	mutex_lock(&reflash_hcd->read.buf_mutex);

	for (i = 0; i < PRD_BLOCK_MAX; i++) {
		if (!reflash_hcd->read.buf[PRD_SIZE_PER_BLOCK * i]) {
			break;
		} else {
			memcpy(&d->prd_info, &reflash_hcd->read.buf[PRD_SIZE_PER_BLOCK * i],
				sizeof(d->prd_info));
			write_count++;
		}
	}

	if (!write_count) {
		TOUCH_E("Product Info is empty\n");
		ret = -1;
	}

	mutex_unlock(&reflash_hcd->read.buf_mutex);

exit:
	mutex_unlock(&reflash_hcd->reflash_mutex);

	mutex_unlock(&d->extif_mutex);

	return ret;
}
EXPORT_SYMBOL(reflash_read_product_info);

static int reflash_set_up_flash_access(void)
{
	int ret = 0;
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;

	TOUCH_TRACE();

	ret = s3618_identify(ts->dev);
	if (ret < 0) {
		TOUCH_E("Failed to do identification\n");
		return ret;
	}

	ret = s3618_ic_info(ts->dev);
	if (ts->force_fwup) {
		TOUCH_I("But ts->force_fwup is triggered!\n");
		goto pass;
	} else if (ret < 0) {
		TOUCH_E("Failed to get ic_info\n");
		return ret;
	}

pass:
	if (d->id_info.mode == MODE_APPLICATION_FIRMWARE) {
		ret = s3618_switch_mode(ts->dev, MODE_BOOTLOADER);
		if (ret < 0) {
			TOUCH_E("Failed to enter bootloader mode\n");
			return ret;
		}
	}

	reflash_hcd->write_block_size = d->boot_info.write_block_size_words * 2;
	reflash_hcd->page_size = d->boot_info.erase_page_size_words * 2;
	reflash_hcd->max_write_payload_size = d->boot_info.max_write_payload_size;

	TOUCH_I("Write block size = %d\n", reflash_hcd->write_block_size);
	TOUCH_I("Page size = %d\n", reflash_hcd->page_size);
	TOUCH_I("Max write payload size = %d\n", reflash_hcd->max_write_payload_size);

	if (reflash_hcd->write_block_size > (d->chunk_size - 5)) {
		TOUCH_E("Write block size greater than available chunk space\n");
		return -EINVAL;
	}

	return 0;
}

static int reflash_parse_fw_image(void)
{
	unsigned int idx = 0;
	unsigned int addr = 0;
	unsigned int offset = 0;
	unsigned int length = 0;
	unsigned int checksum = 0;
	unsigned int flash_addr = 0;
	unsigned int magic_value = 0;
	unsigned int num_of_areas = 0;
	struct image_header *header;
	struct image_info *image_info;
	struct area_descriptor *descriptor;
	const unsigned char *image;
	const unsigned char *content;

	TOUCH_TRACE();

	image = reflash_hcd->image;
	image_info = &reflash_hcd->image_info;
	header = (struct image_header *)image;

	reflash_hcd->disp_cfg_update = false;

	magic_value = le4_to_uint(header->magic_value);
	if (magic_value != IMAGE_FILE_MAGIC_VALUE) {
		TOUCH_E("Invalid image file magic value\n");
		return -EINVAL;
	}

	memset(image_info, 0x00, sizeof(*image_info));

	offset = sizeof(*header);
	num_of_areas = le4_to_uint(header->num_of_areas);

	for (idx = 0; idx < num_of_areas; idx++) {
		addr = le4_to_uint(image + offset);
		descriptor = (struct area_descriptor *)(image + addr);
		offset += 4;

		magic_value = le4_to_uint(descriptor->magic_value);
		if (magic_value != FLASH_AREA_MAGIC_VALUE)
			continue;

		length = le4_to_uint(descriptor->length);
		content = (unsigned char *)descriptor + sizeof(*descriptor);
		flash_addr = le4_to_uint(descriptor->flash_addr_words) * 2;
		checksum = le4_to_uint(descriptor->checksum);

		if (0 == strncmp((char *)descriptor->id_string,
				BOOT_CONFIG_ID,
				strlen(BOOT_CONFIG_ID))) {
			if (checksum != (crc32(~0, content, length) ^ ~0)) {
				TOUCH_E(	"Boot config checksum error\n");
				return -EINVAL;
			}
			image_info->boot_config.size = length;
			image_info->boot_config.data = content;
			image_info->boot_config.flash_addr = flash_addr;
			TOUCH_I("Boot config size = %d\n", length);
			TOUCH_I("Boot config flash address = 0x%08x\n",
					flash_addr);
		} else if (0 == strncmp((char *)descriptor->id_string,
				APP_CODE_ID,
				strlen(APP_CODE_ID))) {
			if (checksum != (crc32(~0, content, length) ^ ~0)) {
				TOUCH_E(	"Application firmware checksum error\n");
				return -EINVAL;
			}
			image_info->app_firmware.size = length;
			image_info->app_firmware.data = content;
			image_info->app_firmware.flash_addr = flash_addr;
			TOUCH_I("Application firmware size = %d\n",
					length);
			TOUCH_I("Application firmware flash address = 0x%08x\n",
					flash_addr);
		} else if (0 == strncmp((char *)descriptor->id_string,
				PROD_TEST_ID,
				strlen(PROD_TEST_ID))) {
			if (checksum != (crc32(~0, content, length) ^ ~0)) {
				TOUCH_E(	"Production test firmware checksum error\n");
				return -EINVAL;
			}
			image_info->prod_test_firmware.size = length;
			image_info->prod_test_firmware.data = content;
			image_info->prod_test_firmware.flash_addr = flash_addr;
			TOUCH_I("Production test firmware size = %d\n",
					length);
			TOUCH_I("Production test firmware flash address = 0x%08x\n",
					flash_addr);
		} else if (0 == strncmp((char *)descriptor->id_string,
				APP_CONFIG_ID,
				strlen(APP_CONFIG_ID))) {
			if (checksum != (crc32(~0, content, length) ^ ~0)) {
				TOUCH_E(	"Application config checksum error\n");
				return -EINVAL;
			}
			image_info->app_config.size = length;
			image_info->app_config.data = content;
			image_info->app_config.flash_addr = flash_addr;
			TOUCH_I("Application config size = %d\n",
					length);
			TOUCH_I("Application config flash address = 0x%08x\n",
					flash_addr);
		} else if (0 == strncmp((char *)descriptor->id_string,
				DISP_CONFIG_ID,
				strlen(DISP_CONFIG_ID))) {
			if (checksum != (crc32(~0, content, length) ^ ~0)) {
				TOUCH_E(	"Display config checksum error\n");
				return -EINVAL;
			}
			reflash_hcd->disp_cfg_update = true;
			image_info->disp_config.size = length;
			image_info->disp_config.data = content;
			image_info->disp_config.flash_addr = flash_addr;
			TOUCH_I("Display config size = %d\n",
					length);
			TOUCH_I("Display config flash address = 0x%08x\n",
					flash_addr);
		}
	}

	return 0;
}

static int reflash_get_fw_image(void)
{
	int ret = 0;
	struct touch_core_data *ts = reflash_hcd->ts;

	TOUCH_TRACE();

	if (reflash_hcd->image == NULL) {
		if (reflash_hcd->reflash_by_manual == false) {

			ret = request_firmware(&reflash_hcd->fw_entry,
				FW_IMAGE_NAME, ts->dev);
			if (ret < 0) {
				TOUCH_E(	"Failed to request %s\n",
						FW_IMAGE_NAME);
				return ret;
			}

		} else {
			ret = request_firmware(&reflash_hcd->fw_entry,
						FW_IMAGE_NAME_MANUAL,
						ts->dev);
			if (ret < 0) {
				TOUCH_E(	"Failed to request %s\n",
						FW_IMAGE_NAME_MANUAL);
				return ret;
			}
		}

		reflash_hcd->image = reflash_hcd->fw_entry->data;
		reflash_hcd->image_size = reflash_hcd->fw_entry->size;

		TOUCH_I("Firmware image size = %d\n",
				reflash_hcd->image_size);
	}

	ret = reflash_parse_fw_image();
	if (ret < 0) {
		TOUCH_E("Failed to parse firmware image\n");
		return ret;
	}

	return 0;
}

static enum update_area reflash_compare_id_info(void)
{
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;
	enum update_area update_area;
	u32 image_build_id = 0;
	u32 device_build_id = 0;
	u8 image_major = 0;
	u8 image_minor = 0;
	u8 device_major = 0;
	u8 device_minor = 0;
	struct app_config_header *app_config_header;
	const unsigned char *app_config_data;

	TOUCH_TRACE();

	update_area = NONE;

	if (reflash_hcd->image_info.app_config.size < sizeof(*app_config_header)) {
		TOUCH_E("Invalid application config in image file\n");
		goto exit;
	}

	app_config_data = reflash_hcd->image_info.app_config.data;
	app_config_header = (struct app_config_header *)app_config_data;

	if (ts->force_fwup) {
		TOUCH_I("Force upgrade triggered\n");
		update_area = FIRMWARE_CONFIG;
		goto exit;
	}

	if (reflash_hcd->force_update) {
		TOUCH_I("Force upgrade triggered by synaptics reflash_hcd->force_update\n");
		update_area = FIRMWARE_CONFIG;
		goto exit;
	}

	if (d->id_info.mode != MODE_APPLICATION_FIRMWARE &&
			d->id_info.mode != MODE_HOSTDOWNLOAD_FIRMWARE ) {
		TOUCH_I("Bootloader upgrade triggered\n");
		update_area = FIRMWARE_CONFIG;
		goto exit;
	}

	image_build_id = app_config_header->build_id;
	device_build_id = d->id_info.build_id;

	if (image_build_id != device_build_id) {
		update_area = FIRMWARE_CONFIG;
		goto exit;
	} else {
		update_area = NONE;
		goto exit;
	}

	image_major = app_config_header->customer_config_id.release;
	image_minor = app_config_header->customer_config_id.version;
	device_major = d->app_info.customer_config_id.release;
	device_minor = d->app_info.customer_config_id.version;

	if (image_major != device_major) {
		update_area = CONFIG_ONLY;
		goto exit;
	} else if (image_minor != device_minor) {
		update_area = CONFIG_ONLY;
		goto exit;
	} else {
		update_area = NONE;
		goto exit;
	}

	update_area = NONE;

exit:

	if (update_area == NONE) {
		TOUCH_I("%s: No need to do reflash\n", __func__);
	} else {
		TOUCH_I("%s: Updating %s\n", __func__,
				update_area == FIRMWARE_CONFIG ?
				"firmware and config" :
				"config only");
	}

	TOUCH_I("%s: image[%d.%02d](bulid:%d) device[%d.%02d](build:%d) -> update_area: %d, force: %d\n", __func__,
			image_major, image_minor, image_build_id,
			device_major, device_minor, device_build_id,
			update_area, ts->force_fwup);

	return update_area;
}

static int reflash_read_flash(unsigned int address, unsigned char *data,
		unsigned int datalen)
{
	struct touch_core_data *ts = reflash_hcd->ts;
	int ret = 0;
	unsigned int length_words = 0;
	unsigned int flash_addr_words = 0;

	mutex_lock(&reflash_hcd->out.buf_mutex);
	memset(reflash_hcd->out.buf, 0x00, sizeof(reflash_hcd->out.buf));
	reflash_hcd->out.data_length = 0;

	length_words = datalen / 2;
	flash_addr_words = address / 2;

	reflash_hcd->out.buf[0] = (unsigned char)flash_addr_words;
	reflash_hcd->out.buf[1] = (unsigned char)(flash_addr_words >> 8);
	reflash_hcd->out.buf[2] = (unsigned char)(flash_addr_words >> 16);
	reflash_hcd->out.buf[3] = (unsigned char)(flash_addr_words >> 24);
	reflash_hcd->out.buf[4] = (unsigned char)length_words;
	reflash_hcd->out.buf[5] = (unsigned char)(length_words >> 8);
	reflash_hcd->out.data_length = 6;

	mutex_lock(&reflash_hcd->response.buf_mutex);

	ret = s3618_write_message(ts->dev,
			CMD_READ_FLASH,
			reflash_hcd->out.buf,
			reflash_hcd->out.data_length,
			reflash_hcd->response.buf,
			&reflash_hcd->response.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n", STR(CMD_READ_FLASH));
		mutex_unlock(&reflash_hcd->response.buf_mutex);
		mutex_unlock(&reflash_hcd->out.buf_mutex);
		return ret;
	}

	mutex_unlock(&reflash_hcd->out.buf_mutex);

	if (reflash_hcd->response.data_length != datalen) {
		TOUCH_E("Failed to read requested length\n");
		mutex_unlock(&reflash_hcd->response.buf_mutex);
		return -EIO;
	}

	memcpy(data, reflash_hcd->response.buf, datalen);
	mutex_unlock(&reflash_hcd->response.buf_mutex);

	return 0;
}

static int reflash_get_data_location(enum flash_area area, unsigned int *addr, unsigned int *length)
{
	struct touch_core_data *ts = reflash_hcd->ts;
	int ret = 0;

	mutex_lock(&reflash_hcd->out.buf_mutex);

	memset(reflash_hcd->out.buf, 0x00, sizeof(reflash_hcd->out.buf));
	reflash_hcd->out.data_length = 0;

	switch (area) {
		case CUSTOM_LCM:
			reflash_hcd->out.buf[0] = LCM_DATA;
			break;
		case CUSTOM_OEM:
			reflash_hcd->out.buf[0] = OEM_DATA;
			break;
		case PPDT:
			reflash_hcd->out.buf[0] = PPDT_DATA;
			break;
		default:
			TOUCH_E("Invalid flash area\n");
			return -EINVAL;
	}
	reflash_hcd->out.data_length = 0;

	mutex_lock(&reflash_hcd->response.buf_mutex);
	ret = s3618_write_message(ts->dev,
			CMD_GET_DATA_LOCATION,
			reflash_hcd->out.buf,
			reflash_hcd->out.data_length,
			reflash_hcd->response.buf,
			&reflash_hcd->response.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n", STR(CMD_GET_DATA_LOCATION));
		mutex_unlock(&reflash_hcd->out.buf_mutex);
		mutex_unlock(&reflash_hcd->response.buf_mutex);
		goto exit;
	}

	if (reflash_hcd->response.data_length != 4) {
		TOUCH_E("Invalid data length\n");
		mutex_unlock(&reflash_hcd->out.buf_mutex);
		mutex_unlock(&reflash_hcd->response.buf_mutex);
		ret = -EINVAL;
		goto exit;
	}

	mutex_unlock(&reflash_hcd->out.buf_mutex);
	mutex_unlock(&reflash_hcd->response.buf_mutex);

	*addr = le2_to_uint(&reflash_hcd->response.buf[0]);
	*length = le2_to_uint(&reflash_hcd->response.buf[2]);

	ret = 0;

exit:
	return ret;
}

static int reflash_read_data(enum flash_area area, bool run_app_firmware)
{
	int ret = 0;
	unsigned int temp = 0;
	unsigned int addr = 0;
	unsigned int length = 0;
	struct s3618_app_info *app_info;
	struct s3618_boot_info *boot_info;
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;

	switch (area) {
	case CUSTOM_LCM:
	case CUSTOM_OEM:
	case PPDT:
		ret = reflash_get_data_location(area, &addr, &length);
		if (ret < 0) {
			TOUCH_E("Failed to get data location\n");
			return ret;
		}
		break;
	default:
		break;
	}

	ret = reflash_set_up_flash_access();
	if (ret < 0) {
		TOUCH_E("Failed to set up flash access\n");
		return ret;
	}

	app_info = &d->app_info;
	boot_info = &d->boot_info;

	switch (area) {
	case BOOT_CONFIG:
		temp = boot_info->boot_config_start_block;
		addr = temp * reflash_hcd->write_block_size;
		length = BOOT_CONFIG_SIZE * BOOT_CONFIG_SLOTS;
		break;
	case APP_CONFIG:
		temp = app_info->app_config_start_write_block;
		addr = temp * reflash_hcd->write_block_size;
		length = app_info->app_config_size;
		break;
	case DISP_CONFIG:
		temp = boot_info->display_config_start_block;
		addr = temp * reflash_hcd->write_block_size;
		temp = boot_info->display_config_length_blocks;
		length = temp * reflash_hcd->write_block_size;
		break;
	case CUSTOM_OTP:
		temp = boot_info->custom_otp_start_block;
		addr = temp * reflash_hcd->write_block_size;
		temp = boot_info->custom_otp_length_blocks;
		length = temp * reflash_hcd->write_block_size;
		break;
	case CUSTOM_LCM:
	case CUSTOM_OEM:
	case PPDT:
		addr *= reflash_hcd->write_block_size;
		length *= reflash_hcd->write_block_size;
		break;
	default:
		TOUCH_E("Invalid data area\n");
		ret = -EINVAL;
		goto run_app_firmware;
	}

	if (addr == 0 || length == 0) {
		TOUCH_E("Data area unavailable\n");
		ret = -EINVAL;
		goto run_app_firmware;
	}

	mutex_lock(&reflash_hcd->read.buf_mutex);

	memset(reflash_hcd->read.buf, 0x00, sizeof(reflash_hcd->out.buf));
	reflash_hcd->read.data_length = 0;

	ret = reflash_read_flash(addr, reflash_hcd->read.buf, length);
	if (ret < 0) {
		TOUCH_E("Failed to read from flash\n");
		mutex_unlock(&reflash_hcd->read.buf_mutex);
		goto run_app_firmware;
	}

	reflash_hcd->read.data_length = length;

	mutex_unlock(&reflash_hcd->read.buf_mutex);

	ret = 0;

run_app_firmware:
	if (!run_app_firmware)
		goto exit;

	if (s3618_switch_mode(ts->dev, MODE_APPLICATION_FIRMWARE) < 0) {
		TOUCH_E("Failed to run application firmware\n");
	}

exit:
	return ret;
}

static int reflash_check_boot_config(void)
{
	unsigned int temp = 0;
	unsigned int image_addr = 0;
	unsigned int device_addr = 0;
	struct s3618_data *d = reflash_hcd->d;

	if (reflash_hcd->image_info.boot_config.size < BOOT_CONFIG_SIZE) {
		TOUCH_E("No valid boot config in image file\n");
		return -EINVAL;
	}

	image_addr = reflash_hcd->image_info.boot_config.flash_addr;

	temp = d->boot_info.boot_config_start_block;
	device_addr = temp * reflash_hcd->write_block_size;

	if (image_addr != device_addr) {
		TOUCH_E("Flash address mismatch\n");
		return -EINVAL;
	}

	return 0;
}

static int reflash_check_app_config(void)
{
	unsigned int temp = 0;
	unsigned int image_addr = 0;
	unsigned int image_size = 0;
	unsigned int device_addr = 0;
	unsigned int device_size = 0;
	struct s3618_data *d = reflash_hcd->d;

	if (reflash_hcd->image_info.app_config.size == 0) {
		TOUCH_E("No application config in image file\n");
		return -EINVAL;
	}

	image_addr = reflash_hcd->image_info.app_config.flash_addr;
	image_size = reflash_hcd->image_info.app_config.size;

	temp = d->app_info.app_config_start_write_block;
	device_addr = temp * reflash_hcd->write_block_size;
	device_size = d->app_info.app_config_size;

	if (device_addr == 0 && device_size == 0)
		return 0;

	if (image_addr != device_addr) {
		TOUCH_E("Flash address mismatch\n");
		return -EINVAL;
	}

	if (image_size != device_size) {
		TOUCH_E("Config size mismatch\n");
		return -EINVAL;
	}

	return 0;
}

static int reflash_check_disp_config(void)
{
	unsigned int temp = 0;
	unsigned int image_addr = 0;
	unsigned int image_size = 0;
	unsigned int device_addr = 0;
	unsigned int device_size = 0;
	struct s3618_data *d = reflash_hcd->d;

	if (reflash_hcd->image_info.disp_config.size == 0) {
		TOUCH_E("No display config in image file\n");
		return -EINVAL;
	}

	image_addr = reflash_hcd->image_info.disp_config.flash_addr;
	image_size = reflash_hcd->image_info.disp_config.size;

	temp = d->boot_info.display_config_start_block;
	device_addr = temp * reflash_hcd->write_block_size;

	temp = d->boot_info.display_config_length_blocks;
	device_size = temp * reflash_hcd->write_block_size;

	if (image_addr != device_addr) {
		TOUCH_E("Flash address mismatch\n");
		return -EINVAL;
	}

	if (image_size != device_size) {
		TOUCH_E("Config size mismatch\n");
		return -EINVAL;
	}

	return 0;
}

static int reflash_check_prod_test_firmware(void)
{
	if (reflash_hcd->image_info.prod_test_firmware.size == 0) {
		TOUCH_E("No production test firmware in image file\n");
		return -EINVAL;
	}

	return 0;
}

static int reflash_check_app_firmware(void)
{
	TOUCH_TRACE();

	if (reflash_hcd->image_info.app_firmware.size == 0) {
		TOUCH_E("No application firmware in image file\n");
		return -EINVAL;
	}

	return 0;
}

static int reflash_write_flash(unsigned int address, const unsigned char *data,
		unsigned int datalen)
{
	int ret = 0;
	unsigned int offset = 0;
	unsigned int w_length = 0;
	unsigned int xfer_length = 0;
	unsigned int remaining_length = 0;
	unsigned int flash_address = 0;
	unsigned int block_address = 0;
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;

	TOUCH_TRACE();

	w_length = d->chunk_size - 5;

	w_length = w_length - (w_length % reflash_hcd->write_block_size);

	w_length = MIN(w_length, reflash_hcd->max_write_payload_size);

	offset = 0;

	remaining_length = datalen;

	mutex_lock(&reflash_hcd->out.buf_mutex);
	mutex_lock(&reflash_hcd->response.buf_mutex);

	while (remaining_length) {
		if (remaining_length > w_length)
			xfer_length = w_length;
		else
			xfer_length = remaining_length;

		memset(reflash_hcd->out.buf, 0x00, sizeof(reflash_hcd->out.buf));
		reflash_hcd->out.data_length = 0;

		flash_address = address + offset;
		block_address = flash_address / reflash_hcd->write_block_size;
		reflash_hcd->out.buf[0] = (unsigned char)block_address;
		reflash_hcd->out.buf[1] = (unsigned char)(block_address >> 8);

		memcpy(&reflash_hcd->out.buf[2], &data[offset], xfer_length);

		ret = s3618_write_message(ts->dev,
				CMD_WRITE_FLASH,
				reflash_hcd->out.buf,
				xfer_length + 2,
				reflash_hcd->response.buf,
				&reflash_hcd->response.data_length);
		if (ret < 0) {
			TOUCH_E("Failed to write command %s\n", STR(CMD_WRITE_FLASH));
			TOUCH_E("Flash address = 0x%08x\n", flash_address);
			TOUCH_E("Data length = %d\n", xfer_length);
			mutex_unlock(&reflash_hcd->out.buf_mutex);
			mutex_unlock(&reflash_hcd->response.buf_mutex);
			return ret;
		}

		offset += xfer_length;
		remaining_length -= xfer_length;

		TOUCH_I("Remaining data to write: (%d/%d byte)\n",
				remaining_length, datalen);
	}

	mutex_unlock(&reflash_hcd->out.buf_mutex);
	mutex_unlock(&reflash_hcd->response.buf_mutex);

	return 0;
}

static int reflash_write_app_config(void)
{
	int ret = 0;
	unsigned int size = 0;
	unsigned int flash_addr = 0;
	const unsigned char *data;

	data = reflash_hcd->image_info.app_config.data;
	size = reflash_hcd->image_info.app_config.size;
	flash_addr = reflash_hcd->image_info.app_config.flash_addr;

	ret = reflash_write_flash(flash_addr, data, size);
	if (ret < 0) {
		TOUCH_E("Failed to write app_config to flash\n");
		return ret;
	}

	return 0;
}

static int reflash_write_disp_config(void)
{
	int ret = 0;
	unsigned int size = 0;
	unsigned int flash_addr = 0;
	const unsigned char *data;

	data = reflash_hcd->image_info.disp_config.data;
	size = reflash_hcd->image_info.disp_config.size;
	flash_addr = reflash_hcd->image_info.disp_config.flash_addr;

	ret = reflash_write_flash(flash_addr, data, size);
	if (ret < 0) {
		TOUCH_E("Failed to write disp_config to flash\n");
		return ret;
	}

	return 0;
}

static int reflash_write_prod_test_firmware(void)
{
	int ret = 0;
	unsigned int size = 0;
	unsigned int flash_addr = 0;
	const unsigned char *data;

	data = reflash_hcd->image_info.prod_test_firmware.data;
	size = reflash_hcd->image_info.prod_test_firmware.size;
	flash_addr = reflash_hcd->image_info.prod_test_firmware.flash_addr;

	ret = reflash_write_flash(flash_addr, data, size);
	if (ret < 0) {
		TOUCH_E("Failed to write prod_test_firmware to flash\n");
		return ret;
	}

	return 0;
}

static int reflash_write_app_firmware(void)
{
	int ret = 0;
	unsigned int size = 0;
	unsigned int flash_addr = 0;
	const unsigned char *data;

	TOUCH_TRACE();

	data = reflash_hcd->image_info.app_firmware.data;
	size = reflash_hcd->image_info.app_firmware.size;
	flash_addr = reflash_hcd->image_info.app_firmware.flash_addr;

	ret = reflash_write_flash(flash_addr, data, size);
	if (ret < 0) {
		TOUCH_E("Failed to write app_firmware to flash\n");
		return ret;
	}

	return 0;
}


static int reflash_erase_flash(unsigned int page_start, unsigned int page_count)
{
	int ret = 0;
	unsigned char out_buf[4] = {0};
	int size_erase_cmd = 0;
	struct touch_core_data *ts = reflash_hcd->ts;

	if ((page_start > 0xff) || (page_count > 0xff))  {
		size_erase_cmd = 4;

		out_buf[0] = (unsigned char)(page_start & 0xff);
		out_buf[1] = (unsigned char)((page_start >> 8) & 0xff);
		out_buf[2] = (unsigned char)(page_count & 0xff);
		out_buf[3] = (unsigned char)((page_count >> 8) & 0xff);
	} else {
		size_erase_cmd = 2;

		out_buf[0] = (unsigned char)(page_start & 0xff);
		out_buf[1] = (unsigned char)(page_count & 0xff);
	}

	mutex_lock(&reflash_hcd->response.buf_mutex);

	ret = s3618_write_message(ts->dev,
			CMD_ERASE_FLASH,
			out_buf,
			size_erase_cmd,
			reflash_hcd->response.buf,
			&reflash_hcd->response.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n", STR(CMD_ERASE_FLASH));
		mutex_unlock(&reflash_hcd->response.buf_mutex);
		return ret;
	}

	mutex_unlock(&reflash_hcd->response.buf_mutex);

	return 0;
}


static int reflash_erase(unsigned int flash_addr, unsigned int size)
{
	int ret = 0;
	unsigned int page_start = 0;
	unsigned int page_count = 0;

	TOUCH_TRACE();

	page_start = flash_addr / reflash_hcd->page_size;

	page_count = ceil_div(size, reflash_hcd->page_size);

	TOUCH_I("Page start = %d (0x%04x)\n",
			page_start, page_start);

	TOUCH_I("Page count = %d (0x%04x)\n",
			page_count, page_count);

	ret = reflash_erase_flash(page_start, page_count);
	if (ret < 0) {
		TOUCH_E("Failed to erase pages, addr = 0x%04x, count = %d\n",
				page_start, page_count);
		return ret;
	}

	return 0;
}

static int reflash_erase_app_config(void)
{
	int ret = 0;
	unsigned int size = 0;
	unsigned int flash_addr = 0;

	flash_addr = reflash_hcd->image_info.app_config.flash_addr;

	size = reflash_hcd->image_info.app_config.size;

	ret = reflash_erase(flash_addr, size);
	if (ret < 0) {
		TOUCH_E("Failed to erase app_config\n");
		return ret;
	}

	return 0;
}

static int reflash_erase_disp_config(void)
{
	int ret = 0;
	unsigned int size = 0;
	unsigned int flash_addr = 0;

	flash_addr = reflash_hcd->image_info.disp_config.flash_addr;

	size = reflash_hcd->image_info.disp_config.size;

	ret = reflash_erase(flash_addr, size);
	if (ret < 0) {
		TOUCH_E("Failed to erase disp_config\n");
		return ret;
	}

	return 0;
}

static int reflash_erase_prod_test_firmware(void)
{
	int ret = 0;
	unsigned int size = 0;
	unsigned int flash_addr = 0;

	flash_addr = reflash_hcd->image_info.prod_test_firmware.flash_addr;

	size = reflash_hcd->image_info.prod_test_firmware.size;

	ret = reflash_erase(flash_addr, size);
	if (ret < 0) {
		TOUCH_E("Failed to erase prod_test_firmware\n");
		return ret;
	}

	return 0;
}

static int reflash_erase_app_firmware(void)
{
	int ret = 0;
	unsigned int size = 0;
	unsigned int flash_addr = 0;

	TOUCH_TRACE();

	flash_addr = reflash_hcd->image_info.app_firmware.flash_addr;

	size = reflash_hcd->image_info.app_firmware.size;

	ret = reflash_erase(flash_addr, size);
	if (ret < 0) {
		TOUCH_E("Failed to erase app_firmware\n");
		return ret;
	}

	return 0;
}


static int reflash_update_custom_otp(const unsigned char *data,
		unsigned int offset, unsigned int datalen)
{
	int ret = 0;
	unsigned int temp = 0;
	unsigned int addr = 0;
	unsigned int length = 0;
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;

	ret = reflash_set_up_flash_access();
	if (ret < 0) {
		TOUCH_E("Failed to set up flash access\n");
		return ret;
	}

	temp = d->boot_info.custom_otp_start_block;
	addr = temp * reflash_hcd->write_block_size;

	temp = d->boot_info.custom_otp_length_blocks;
	length = temp * reflash_hcd->write_block_size;

	if (addr == 0 || length == 0) {
		TOUCH_E("Data area unavailable\n");
		ret = -EINVAL;
		goto run_app_firmware;
	}

	if (datalen + offset > length) {
		TOUCH_E("Invalid data length\n");
		ret = -EINVAL;
		goto run_app_firmware;
	}

	ret = reflash_write_flash(addr + offset,
			data,
			datalen);
	if (ret < 0) {
		TOUCH_E("Failed to write to flash\n");
		goto run_app_firmware;
	}

	ret = 0;

run_app_firmware:
	if (s3618_switch_mode(ts->dev, MODE_APPLICATION_FIRMWARE) < 0) {
		TOUCH_E("Failed to run application firmware\n");
	}

	return ret;
}

static int reflash_update_custom_lcm(const unsigned char *data,
		unsigned int offset, unsigned int datalen)
{
	struct touch_core_data *ts = reflash_hcd->ts;
	int ret = 0;
	unsigned int addr = 0;
	unsigned int length = 0;
	unsigned int page_start = 0;
	unsigned int page_count = 0;

	ret = reflash_get_data_location(CUSTOM_LCM, &addr, &length);
	if (ret < 0) {
		TOUCH_E("Failed to get data location\n");
		return ret;
	}

	ret = reflash_set_up_flash_access();
	if (ret < 0) {
		TOUCH_E("Failed to set up flash access\n");
		return ret;
	}

	addr *= reflash_hcd->write_block_size;
	length *= reflash_hcd->write_block_size;

	if (addr == 0 || length == 0) {
		TOUCH_E("Data area unavailable\n");
		ret = -EINVAL;
		goto run_app_firmware;
	}

	if (datalen + offset > length) {
		TOUCH_E("Invalid data length\n");
		ret = -EINVAL;
		goto run_app_firmware;
	}

	if (offset == 0) {
		page_start = addr / reflash_hcd->page_size;

		page_count = ceil_div(length, reflash_hcd->page_size);

		ret = reflash_erase_flash(page_start, page_count);
		if (ret < 0) {
			TOUCH_E("Failed to erase flash pages\n");
			goto run_app_firmware;
		}
	}

	ret = reflash_write_flash(addr + offset,
			data,
			datalen);
	if (ret < 0) {
		TOUCH_E("Failed to write to flash\n");
		goto run_app_firmware;
	}

	ret = 0;

run_app_firmware:
	if (s3618_switch_mode(ts->dev, MODE_APPLICATION_FIRMWARE) < 0) {
		TOUCH_E("Failed to run application firmware\n");
	}

	return ret;
}

static int reflash_update_custom_oem(const unsigned char *data,
		unsigned int offset, unsigned int datalen)
{
	struct touch_core_data *ts = reflash_hcd->ts;
	int ret = 0;
	unsigned int addr = 0;
	unsigned int length = 0;
	unsigned int page_start = 0;
	unsigned int page_count = 0;

	ret = reflash_get_data_location(CUSTOM_OEM, &addr, &length);
	if (ret < 0) {
		TOUCH_E("Failed to get data location\n");
		return ret;
	}

	ret = reflash_set_up_flash_access();
	if (ret < 0) {
		TOUCH_E("Failed to set up flash access\n");
		return ret;
	}

	addr *= reflash_hcd->write_block_size;
	length *= reflash_hcd->write_block_size;

	if (addr == 0 || length == 0) {
		TOUCH_E("Data area unavailable\n");
		ret = -EINVAL;
		goto run_app_firmware;
	}

	if (datalen + offset > length) {
		TOUCH_E("Invalid data length\n");
		ret = -EINVAL;
		goto run_app_firmware;
	}

	if (offset == 0) {
		page_start = addr / reflash_hcd->page_size;

		page_count = ceil_div(length, reflash_hcd->page_size);

		ret = reflash_erase_flash(page_start, page_count);
		if (ret < 0) {
			TOUCH_E("Failed to erase flash pages\n");
			goto run_app_firmware;
		}
	}

	ret = reflash_write_flash(addr + offset,
			data,
			datalen);
	if (ret < 0) {
		TOUCH_E("Failed to write to flash\n");
		goto run_app_firmware;
	}

	ret = 0;

run_app_firmware:
	if (s3618_switch_mode(ts->dev, MODE_APPLICATION_FIRMWARE) < 0) {
		TOUCH_E("Failed to run application firmware\n");
	}

	return ret;
}

static int reflash_update_boot_config(bool lock)
{
	int ret = 0;
	unsigned char slot_used = 0;
	unsigned int idx = 0;
	unsigned int addr = 0;
	struct boot_config *data;
	struct boot_config *last_slot;
	struct touch_core_data *ts = reflash_hcd->ts;
	struct s3618_data *d = reflash_hcd->d;

	ret = reflash_set_up_flash_access();
	if (ret < 0) {
		TOUCH_E("Failed to set up flash access\n");
		return ret;
	}

	ret = reflash_check_boot_config();
	if (ret < 0) {
		TOUCH_E("Failed boot_config partition check\n");
		goto reset;
	}

	ret = reflash_read_data(BOOT_CONFIG, false);
	if (ret < 0) {
		TOUCH_E("Failed to read boot config\n");
		goto reset;
	}

	mutex_lock(&reflash_hcd->read.buf_mutex);

	data = (struct boot_config *)reflash_hcd->read.buf;
	last_slot = data + (BOOT_CONFIG_SLOTS - 1);
	slot_used = d->id_info.mode == MODE_TDDI_BOOTLOADER ? 0 : 1;

	if (last_slot->used == slot_used) {
		TOUCH_E("Boot config already locked down\n");
		mutex_unlock(&reflash_hcd->read.buf_mutex);
		goto reset;
	}

	if (lock) {
		idx = BOOT_CONFIG_SLOTS - 1;
	} else {
		for (idx = 0; idx < BOOT_CONFIG_SLOTS; idx++) {
			if (data->used == slot_used) {
				data++;
				continue;
			} else {
				break;
			}
		}
	}

	mutex_unlock(&reflash_hcd->read.buf_mutex);

	if (idx == BOOT_CONFIG_SLOTS) {
		TOUCH_E("No free boot config slot available\n");
		goto reset;
	}

	addr += idx * BOOT_CONFIG_SIZE;

	ret = reflash_write_flash(addr,
			reflash_hcd->image_info.boot_config.data,
			BOOT_CONFIG_SIZE);
	if (ret < 0) {
		TOUCH_E("Failed to write to flash\n");
		goto reset;
	}

	TOUCH_I("Slot %d updated with new boot config\n", idx);

	ret = 0;

reset:
	s3618_reset_ctrl(ts->dev, SW_RESET_NO_INIT);

	return ret;
}

static int reflash_update_app_config(bool do_reset)
{
	struct touch_core_data *ts = reflash_hcd->ts;
	int ret = 0;

	ret = reflash_set_up_flash_access();
	if (ret < 0) {
		TOUCH_E("Failed to set up flash access\n");
		return ret;
	}

	ret = reflash_check_app_config();
	if (ret < 0) {
		TOUCH_E("Failed to check app_config partition\n");
		do_reset = true;
		goto reset;
	}

	ret = reflash_erase_app_config();
	if (ret < 0) {
		TOUCH_E("Failed to erase app_config partition\n");
		do_reset = true;
		goto reset;
	}

	TOUCH_I("app_config partition erased\n");

	ret = reflash_write_app_config();
	if (ret < 0) {
		TOUCH_E("Failed to write app_config partition\n");
		do_reset = true;
		goto reset;
	}

	TOUCH_I("app_config partition written\n");

	ret = 0;

reset:
	if (!do_reset)
		goto exit;

	s3618_reset_ctrl(ts->dev, SW_RESET_NO_INIT);

exit:
	return ret;
}

static int reflash_update_disp_config(bool do_reset)
{
	int ret = 0;
	struct touch_core_data *ts = reflash_hcd->ts;

	ret = reflash_set_up_flash_access();
	if (ret < 0) {
		TOUCH_E("Failed to set up flash access\n");
		return ret;
	}

	ret = reflash_check_disp_config();
	if (ret < 0) {
		TOUCH_E("Failed to check disp_config partition\n");
		do_reset = true;
		goto reset;
	}

	ret = reflash_erase_disp_config();
	if (ret < 0) {
		TOUCH_E("Failed to erase disp_config partition\n");
		do_reset = true;
		goto reset;
	}

	TOUCH_I("disp_config partition erased\n");

	ret = reflash_write_disp_config();
	if (ret < 0) {
		TOUCH_E("Failed to write disp_config partition\n");
		do_reset = true;
		goto reset;
	}

	TOUCH_I("disp_config partition written\n");

	ret = 0;

reset:
	if (!do_reset)
		goto exit;

	s3618_reset_ctrl(ts->dev, SW_RESET_NO_INIT);

exit:
	return ret;
}

static int reflash_update_prod_test_firmware(void)
{
	int ret = 0;
	struct touch_core_data *ts = reflash_hcd->ts;

	ret = reflash_set_up_flash_access();
	if (ret < 0) {
		TOUCH_E("Failed to set up flash access\n");
		return ret;
	}

	ret = reflash_check_prod_test_firmware();
	if (ret < 0) {
		TOUCH_E("Failed to check prod_test_firmware partition\n");
		goto reset;
	}

	ret = reflash_erase_prod_test_firmware();
	if (ret < 0) {
		TOUCH_E("Failed to erase prod_test_firmware partition\n");
		goto reset;
	}

	TOUCH_I("prod_test_firmware partition erased\n");

	ret = reflash_write_prod_test_firmware();
	if (ret < 0) {
		TOUCH_E("Failed to write prod_test_firmware partition\n");
		goto reset;
	}

	TOUCH_I("prod_test_firmware partition written\n");

	ret = 0;

reset:
	s3618_reset_ctrl(ts->dev, SW_RESET_NO_INIT);

	return ret;
}

static int reflash_update_app_firmware(void)
{
	int ret = 0;
	struct touch_core_data *ts = reflash_hcd->ts;

	TOUCH_TRACE();

	ret = reflash_set_up_flash_access();
	if (ret < 0) {
		TOUCH_E("Failed to set up flash access\n");
		return ret;
	}

	ret = reflash_check_app_firmware();
	if (ret < 0) {
		TOUCH_E("Failed to check app_firmware partition\n");
		goto reset;
	}

	ret = reflash_erase_app_firmware();
	if (ret < 0) {
		TOUCH_E("Failed to erase app_firmware partition\n");
		goto reset;
	}

	TOUCH_I("app_firmware partition erased\n");

	ret = reflash_write_app_firmware();
	if (ret < 0) {
		TOUCH_E("Failed to write app_firmware partition\n");
		goto reset;
	}

	TOUCH_I("app_firmware partition written\n");

	ret = 0;

reset:
	s3618_reset_ctrl(ts->dev, SW_RESET_NO_INIT);

	return ret;
}

static int reflash_do_reflash()
{
	struct s3618_data *d = reflash_hcd->d;
	int ret = 0;
	enum update_area update_area;

	TOUCH_TRACE();

	ret = reflash_get_fw_image();
	if (ret < 0) {
		TOUCH_E("Failed to get firmware image\n");
		goto exit;
	}

	TOUCH_I("Start of reflash\n");

	//atomic_set(&tcm_hcd->firmware_flashing, 1);

	update_area = reflash_compare_id_info();

	switch (update_area) {
	case FIRMWARE_CONFIG:
		ret = reflash_update_app_firmware();
		if (ret < 0) {
			TOUCH_E("Failed to reflash application firmware\n");
			goto exit;
		}
		memset(&d->app_info, 0x00, sizeof(d->app_info));
		//if (tcm_hcd->features.dual_firmware) {
		if (0) {
			ret = reflash_update_prod_test_firmware();
			if (ret < 0) {
				TOUCH_E("Failed to reflash production test firmware\n");
				goto exit;
			}
		}
		if (reflash_hcd->disp_cfg_update) {
			ret = reflash_update_disp_config(false);
			if (ret < 0) {
				TOUCH_E("Failed to reflash display config\n");
				goto exit;
			}
		}
		ret = reflash_update_app_config(true);
		if (ret < 0) {
			TOUCH_E("Failed to reflash application config\n");
			goto exit;
		}
		break;
	case CONFIG_ONLY:
		if (reflash_hcd->disp_cfg_update) {
			ret = reflash_update_disp_config(false);
			if (ret < 0) {
				TOUCH_E("Failed to reflash display config\n");
				goto exit;
			}
		}
		ret = reflash_update_app_config(true);
		if (ret < 0) {
			TOUCH_E("Failed to reflash application config\n");
			goto exit;
		}
		break;
	case NONE:
		break;
	default:
		break;
	}

	TOUCH_I("End of reflash\n");

	ret = 0;

exit:
	if (reflash_hcd->fw_entry) {
		release_firmware(reflash_hcd->fw_entry);
		reflash_hcd->fw_entry = NULL;
		reflash_hcd->image = NULL;
		reflash_hcd->image_size = 0;
	}

	//atomic_set(&tcm_hcd->firmware_flashing, 0);
	//wake_up_interruptible(&tcm_hcd->reflash_wq);
	return ret;
}

int s3618_fw_updater(const unsigned char *fw_data)
{
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s : fw->data[%p]\n", __func__, fw_data);

	if (!reflash_hcd) {
		TOUCH_E("There is no reflash_hcd\n");
		return -ENODEV;
	}

#if 0
	if (fwu->in_ub_mode) {
		fwu->image = NULL;
		ret = fwu_start_recovery();
		if (ret < 0)
			return ret;
	}
#endif

	reflash_hcd->image = fw_data;

	ret = reflash_do_reflash();
	if (ret < 0) {
		TOUCH_E("Failed to firmware upgrade\n\n");
		goto error;
	}

	reflash_hcd->image = NULL;

	TOUCH_I("%s : fw_update end\n", __func__);

error:
	return ret;
}
EXPORT_SYMBOL(s3618_fw_updater);

int s3618_get_bin_fw_version(const unsigned char *fw_data)
{
	struct s3618_data *d = NULL;
	int ret = 0;
	const unsigned char *app_config_data;
	struct app_config_header *app_config_header;

	TOUCH_TRACE();

	if (!reflash_hcd || (reflash_hcd->d == NULL)) {
		TOUCH_E("There is no reflash_hcd\n");
		return -ENODEV;
	}

	d = reflash_hcd->d;
	reflash_hcd->image = fw_data;

	ret = reflash_get_fw_image();
	if (ret < 0) {
		TOUCH_E("Failed to get firmware image\n");
		goto error;
	}

	app_config_data = reflash_hcd->image_info.app_config.data;
	app_config_header = (struct app_config_header *)app_config_data;

	d->bin_fw_version_info.build_id = app_config_header->build_id;
	d->bin_fw_version_info.release = app_config_header->customer_config_id.release;
	d->bin_fw_version_info.version = app_config_header->customer_config_id.version;

	reflash_hcd->image = NULL;
error:
	return ret;
}
EXPORT_SYMBOL(s3618_get_bin_fw_version);

static int reflash_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;
	int idx = 0;

	if (reflash_hcd) {
		TOUCH_I("%s: Handle is already exists\n", __func__);
		return 0;
	}

	/*if (tcm_hcd->in_hdl_mode)
		return 0;*/

	reflash_hcd = kzalloc(sizeof(*reflash_hcd), GFP_KERNEL);
	if (!reflash_hcd) {
		TOUCH_E("Failed to allocate memory for reflash_hcd\n");
		return -ENOMEM;
	}

	reflash_hcd->image_buf = kzalloc(IMAGE_BUF_SIZE, GFP_KERNEL);
	if (!reflash_hcd->image_buf) {
		TOUCH_E("Failed to allocate memory for reflash_hcd->image_buf\n");
		goto err_allocate_memory;
	}

	reflash_hcd->ts = ts;
	reflash_hcd->d = d;

	reflash_hcd->force_update = FORCE_REFLASH;

	mutex_init(&reflash_hcd->reflash_mutex);
	mutex_init(&reflash_hcd->read.buf_mutex);
	mutex_init(&reflash_hcd->response.buf_mutex);
	mutex_init(&reflash_hcd->out.buf_mutex);

	if (ENABLE_SYS_REFLASH == false)
		goto init_finished;

	reflash_hcd->sysfs_dir = kobject_create_and_add(SYSFS_DIR_NAME,
			&ts->input->dev.kobj);
	if (!reflash_hcd->sysfs_dir) {
		TOUCH_E("Failed to create sysfs directory\n");
		ret = -EINVAL;
		goto err_sysfs_create_dir;
	}

	for (idx = 0; idx < ARRAY_SIZE(attrs); idx++) {
		ret = sysfs_create_file(reflash_hcd->sysfs_dir,
				&(*attrs[idx]).attr);
		if (ret < 0) {
			TOUCH_E("Failed to create sysfs file\n");
			goto err_sysfs_create_file;
		}
	}

	ret = sysfs_create_bin_file(reflash_hcd->sysfs_dir, &bin_attrs[0]);
	if (ret < 0) {
		TOUCH_E("Failed to create sysfs bin file\n");
		goto err_sysfs_create_bin_file;
	}

	reflash_hcd->custom_dir = kobject_create_and_add(CUSTOM_DIR_NAME,
			reflash_hcd->sysfs_dir);
	if (!reflash_hcd->custom_dir) {
		TOUCH_E("Failed to create custom sysfs directory\n");
		ret = -EINVAL;
		goto err_custom_sysfs_create_dir;
	}

	for (idx = 1; idx < ARRAY_SIZE(bin_attrs); idx++) {
		ret = sysfs_create_bin_file(reflash_hcd->custom_dir,
				&bin_attrs[idx]);
		if (ret < 0) {
			TOUCH_E("Failed to create sysfs bin file\n");
			goto err_custom_sysfs_create_bin_file;
		}
	}

init_finished:
	//tcm_hcd->read_flash_data = reflash_read_data;

	return 0;

err_custom_sysfs_create_bin_file:
	for (idx--; idx > 0; idx--)
		sysfs_remove_bin_file(reflash_hcd->custom_dir, &bin_attrs[idx]);

	kobject_put(reflash_hcd->custom_dir);

	idx = ARRAY_SIZE(attrs);

err_custom_sysfs_create_dir:
	sysfs_remove_bin_file(reflash_hcd->sysfs_dir, &bin_attrs[0]);

err_sysfs_create_bin_file:
err_sysfs_create_file:
	for (idx--; idx >= 0; idx--)
		sysfs_remove_file(reflash_hcd->sysfs_dir, &(*attrs[idx]).attr);

	kobject_put(reflash_hcd->sysfs_dir);

err_sysfs_create_dir:
err_allocate_memory:
	kfree(reflash_hcd->image_buf);

	kfree(reflash_hcd);
	reflash_hcd = NULL;

	return ret;
}

static void reflash_remove(struct device *dev)
{
	int idx = 0;

	if (!reflash_hcd)
		goto exit;

	//tcm_hcd->read_flash_data = NULL;

	if (ENABLE_SYS_REFLASH == true) {
		for (idx = 1; idx < ARRAY_SIZE(bin_attrs); idx++) {
			sysfs_remove_bin_file(reflash_hcd->custom_dir,
					&bin_attrs[idx]);
		}

		kobject_put(reflash_hcd->custom_dir);

		sysfs_remove_bin_file(reflash_hcd->sysfs_dir, &bin_attrs[0]);

		for (idx = 0; idx < ARRAY_SIZE(attrs); idx++) {
			sysfs_remove_file(reflash_hcd->sysfs_dir,
					&(*attrs[idx]).attr);
		}

		kobject_put(reflash_hcd->sysfs_dir);
	}

	kfree(reflash_hcd->image_buf);

	kfree(reflash_hcd);
	reflash_hcd = NULL;

exit:
	complete(&reflash_remove_complete);

	return;
}

//static struct syna_tcm_module_cb reflash_module = {
static struct s3618_exp_fn reflash_module = {
	//.type = TCM_REFLASH,
	.init = reflash_init,
	.remove = reflash_remove,
	//.syncbox = NULL,
	//#ifdef REPORT_NOTIFIER
	//.asyncbox = NULL,
	//#endif
	.reset = NULL,
	.reinit = NULL,
	.early_suspend = NULL,
	.suspend = NULL,
	.resume = NULL,
	.late_resume = NULL,
	.attn = NULL,
};

static int __init reflash_module_init(void)
{
	s3618_reflash_function(&reflash_module, true);
	//return syna_tcm_add_module(&reflash_module, true);

	return 0;
}

static void __exit reflash_module_exit(void)
{
	s3618_reflash_function(&reflash_module, false);

	wait_for_completion(&reflash_remove_complete);
	//syna_tcm_add_module(&reflash_module, false);

	//wait_for_completion(&reflash_remove_complete);

	return;
}

module_init(reflash_module_init);
module_exit(reflash_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TCM Reflash Module");
MODULE_LICENSE("GPL v2");
