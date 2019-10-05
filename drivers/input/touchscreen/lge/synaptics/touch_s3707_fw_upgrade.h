/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012-2016 Synaptics Incorporated. All rights reserved.
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
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

#ifndef _SYNAPTICS_DSX_RMI4_H_
#define _SYNAPTICS_DSX_RMI4_H_

#define SYNAPTICS_DS4 (1 << 0)
#define SYNAPTICS_DS5 (1 << 1)
#define SYNAPTICS_DSX_DRIVER_PRODUCT (SYNAPTICS_DS4 | SYNAPTICS_DS5)
#define SYNAPTICS_DSX_DRIVER_VERSION 0x2070

#include <linux/version.h>
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38))
#define KERNEL_ABOVE_2_6_38
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
#define KERNEL_ABOVE_3_6
#endif

#ifdef KERNEL_ABOVE_2_6_38
#define sstrtoul(...) kstrtoul(__VA_ARGS__)
#else
#define sstrtoul(...) strict_strtoul(__VA_ARGS__)
#endif
/*
#define F51_DISCRETE_FORCE
#ifdef F51_DISCRETE_FORCE
#define FORCE_LEVEL_ADDR 0x0419
#define FORCE_LEVEL_MAX 255
#define CAL_DATA_SIZE 144
#endif
#define SYNA_TDDI
*/

#define FW_IHEX_NAME "synaptics/startup_fw_update.bin"
#define FW_IMAGE_NAME "synaptics/startup_fw_update.img"
/*
#define DO_STARTUP_FW_UPDATE
*/
/*
#ifdef DO_STARTUP_FW_UPDATE
#ifdef CONFIG_FB
#define WAIT_FOR_FB_READY
#define FB_READY_WAIT_MS 100
#define FB_READY_TIMEOUT_S 30
#endif
#endif
*/
#define MAX_WRITE_SIZE 4096

#define ENABLE_SYS_REFLASH true
#define FORCE_UPDATE false
#define DO_LOCKDOWN false

#define MAX_IMAGE_NAME_LEN	256
#define MAX_FIRMWARE_ID_LEN	10

#define IMAGE_HEADER_VERSION_05	0x05
#define IMAGE_HEADER_VERSION_06	0x06
#define IMAGE_HEADER_VERSION_10	0x10

#define IMAGE_AREA_OFFSET	0x100
#define LOCKDOWN_SIZE	0x50

#define MAX_UTILITY_PARAMS	20

#define V5V6_BOOTLOADER_ID_OFFSET	0
#define V5V6_CONFIG_ID_SIZE	4

#define V5_PROPERTIES_OFFSET	2
#define V5_BLOCK_SIZE_OFFSET	3
#define V5_BLOCK_COUNT_OFFSET	5
#define V5_BLOCK_NUMBER_OFFSET	0
#define V5_BLOCK_DATA_OFFSET	2

#define V6_PROPERTIES_OFFSET	1
#define V6_BLOCK_SIZE_OFFSET	2
#define V6_BLOCK_COUNT_OFFSET	3
#define V6_PROPERTIES_2_OFFSET	4
#define V6_GUEST_CODE_BLOCK_COUNT_OFFSET	5
#define V6_BLOCK_NUMBER_OFFSET	0
#define V6_BLOCK_DATA_OFFSET	1
#define V6_FLASH_COMMAND_OFFSET	2
#define V6_FLASH_STATUS_OFFSET	3

#define V7_CONFIG_ID_SIZE	32

#define V7_FLASH_STATUS_OFFSET	0
#define V7_PARTITION_ID_OFFSET	1
#define V7_BLOCK_NUMBER_OFFSET	2
#define V7_TRANSFER_LENGTH_OFFSET	3
#define V7_COMMAND_OFFSET	4
#define V7_PAYLOAD_OFFSET	5

#define V7_PARTITION_SUPPORT_BYTES	4

#define SLEEP_MODE_NORMAL	(0x00)
#define SLEEP_MODE_SENSOR_SLEEP	(0x01)
#define SLEEP_MODE_RESERVED0	(0x02)
#define SLEEP_MODE_RESERVED1	(0x03)

#define ENABLE_WAIT_MS	(1 * 1000)
#define WRITE_WAIT_MS	(5 * 10)
#define ERASE_WAIT_MS	(4 * 1000)

#define MIN_SLEEP_TIME_US	50
#define MAX_SLEEP_TIME_US	100

#define INT_DISABLE_WAIT_MS	20
#define ENTER_FLASH_PROG_WAIT_MS	20
#define READ_CONFIG_WAIT_MS	20

#define SYNAPTICS_RMI4_F01	(0x01)
#define SYNAPTICS_RMI4_F11	(0x11)
#define SYNAPTICS_RMI4_F12	(0x12)
#define SYNAPTICS_RMI4_F1A	(0x1A)
#define SYNAPTICS_RMI4_F21	(0x21)
#define SYNAPTICS_RMI4_F34	(0x34)
#define SYNAPTICS_RMI4_F35	(0x35)
#define SYNAPTICS_RMI4_F38	(0x38)
#define SYNAPTICS_RMI4_F51	(0x51)
#define SYNAPTICS_RMI4_F54	(0x54)
#define SYNAPTICS_RMI4_F55	(0x55)
#define SYNAPTICS_RMI4_FDB	(0xDB)

#define PRODUCT_INFO_SIZE	2
#define PRODUCT_ID_SIZE	10
#define BUILD_ID_SIZE	3

#define F12_GESTURE_DETECTION_LEN	5

#define MAX_NUMBER_OF_BUTTONS	4
#define MAX_INTR_REGISTERS	4

enum exp_fn {
	RMI_DEV = 0,
	RMI_FW_UPDATER,
	RMI_TEST_REPORTING,
	RMI_PROXIMITY,
	RMI_ACTIVE_PEN,
	RMI_GESTURE,
	RMI_VIDEO,
	RMI_DEBUG,
	RMI_LAST,
};

enum f34_version {
	F34_V0 = 0,
	F34_V1,
	F34_V2,
};

enum bl_version {
	BL_V5 = 5,
	BL_V6 = 6,
	BL_V7 = 7,
	BL_V8 = 8,
};

enum flash_area {
	NONE = 0,
	UI_FIRMWARE,
	UI_CONFIG,
};

enum update_mode {
	NORMAL = 1,
	FORCE = 2,
	LOCKDOWN = 8,
};

enum config_area {
	UI_CONFIG_AREA = 0,
	PM_CONFIG_AREA,
	BL_CONFIG_AREA,
	DP_CONFIG_AREA,
	FLASH_CONFIG_AREA,
#ifdef SYNA_TDDI
	TDDI_FORCE_CONFIG_AREA,
	TDDI_LCM_DATA_AREA,
	TDDI_OEM_DATA_AREA,
#endif
	UPP_AREA,
};

enum v7_status {
	SUCCESS = 0x00,
	DEVICE_NOT_IN_BOOTLOADER_MODE,
	INVALID_PARTITION,
	INVALID_COMMAND,
	INVALID_BLOCK_OFFSET,
	INVALID_TRANSFER,
	NOT_ERASED,
	FLASH_PROGRAMMING_KEY_INCORRECT,
	BAD_PARTITION_TABLE,
	CHECKSUM_FAILED,
	FLASH_HARDWARE_FAILURE = 0x1f,
};

enum v7_partition_id {
	BOOTLOADER_PARTITION = 0x01,
	DEVICE_CONFIG_PARTITION,
	FLASH_CONFIG_PARTITION,
	MANUFACTURING_BLOCK_PARTITION,
	GUEST_SERIALIZATION_PARTITION,
	GLOBAL_PARAMETERS_PARTITION,
	CORE_CODE_PARTITION,
	CORE_CONFIG_PARTITION,
	GUEST_CODE_PARTITION,
	DISPLAY_CONFIG_PARTITION,
	EXTERNAL_TOUCH_AFE_CONFIG_PARTITION,
	UTILITY_PARAMETER_PARTITION,
};

enum v7_flash_command {
	CMD_V7_IDLE = 0x00,
	CMD_V7_ENTER_BL,
	CMD_V7_READ,
	CMD_V7_WRITE,
	CMD_V7_ERASE,
	CMD_V7_ERASE_AP,
	CMD_V7_SENSOR_ID,
};

enum v5v6_flash_command {
	CMD_V5V6_IDLE = 0x0,
	CMD_V5V6_WRITE_FW = 0x2,
	CMD_V5V6_ERASE_ALL = 0x3,
	CMD_V5V6_WRITE_LOCKDOWN = 0x4,
	CMD_V5V6_READ_CONFIG = 0x5,
	CMD_V5V6_WRITE_CONFIG = 0x6,
	CMD_V5V6_ERASE_UI_CONFIG = 0x7,
	CMD_V5V6_ERASE_BL_CONFIG = 0x9,
	CMD_V5V6_ERASE_DISP_CONFIG = 0xa,
	CMD_V5V6_ERASE_GUEST_CODE = 0xb,
	CMD_V5V6_WRITE_GUEST_CODE = 0xc,
	CMD_V5V6_ERASE_CHIP = 0x0d,
	CMD_V5V6_ENABLE_FLASH_PROG = 0xf,
#ifdef SYNA_TDDI
	CMD_V5V6_ERASE_FORCE_CONFIG = 0x11,
	CMD_V5V6_READ_FORCE_CONFIG = 0x12,
	CMD_V5V6_WRITE_FORCE_CONFIG = 0x13,
	CMD_V5V6_ERASE_LOCKDOWN_DATA = 0x1a,
	CMD_V5V6_READ_LOCKDOWN_DATA = 0x1b,
	CMD_V5V6_WRITE_LOCKDOWN_DATA = 0x1c,
	CMD_V5V6_ERASE_LCM_DATA = 0x1d,
	CMD_V5V6_ERASE_OEM_DATA = 0x1e,
#endif
};

enum flash_command {
	CMD_IDLE = 0,
	CMD_WRITE_FW,
	CMD_WRITE_CONFIG,
	CMD_WRITE_LOCKDOWN,
	CMD_WRITE_GUEST_CODE,
	CMD_WRITE_BOOTLOADER,
	CMD_WRITE_UTILITY_PARAM,
	CMD_READ_CONFIG,
	CMD_ERASE_ALL,
	CMD_ERASE_UI_FIRMWARE,
	CMD_ERASE_UI_CONFIG,
	CMD_ERASE_BL_CONFIG,
	CMD_ERASE_DISP_CONFIG,
	CMD_ERASE_FLASH_CONFIG,
	CMD_ERASE_GUEST_CODE,
	CMD_ERASE_BOOTLOADER,
	CMD_ERASE_UTILITY_PARAMETER,
	CMD_ENABLE_FLASH_PROG,
#ifdef SYNA_TDDI
	CMD_ERASE_CHIP,
	CMD_ERASE_FORCE_CONFIG,
	CMD_READ_FORCE_CONFIG,
	CMD_WRITE_FORCE_CONFIG,
	CMD_ERASE_LOCKDOWN_DATA,
	CMD_READ_LOCKDOWN_DATA,
	CMD_WRITE_LOCKDOWN_DATA,
	CMD_ERASE_LCM_DATA,
	CMD_READ_LCM_DATA,
	CMD_WRITE_LCM_DATA,
	CMD_ERASE_OEM_DATA,
	CMD_READ_OEM_DATA,
	CMD_WRITE_OEM_DATA,
#endif
};

enum f35_flash_command {
	CMD_F35_IDLE = 0x0,
	CMD_F35_RESERVED = 0x1,
	CMD_F35_WRITE_CHUNK = 0x2,
	CMD_F35_ERASE_ALL = 0x3,
	CMD_F35_RESET = 0x10,
};

enum container_id {
	TOP_LEVEL_CONTAINER = 0,
	UI_CONTAINER,
	UI_CONFIG_CONTAINER,
	BL_CONTAINER,
	BL_IMAGE_CONTAINER,
	BL_CONFIG_CONTAINER,
	BL_LOCKDOWN_INFO_CONTAINER,
	PERMANENT_CONFIG_CONTAINER,
	GUEST_CODE_CONTAINER,
	BL_PROTOCOL_DESCRIPTOR_CONTAINER,
	UI_PROTOCOL_DESCRIPTOR_CONTAINER,
	RMI_SELF_DISCOVERY_CONTAINER,
	RMI_PAGE_CONTENT_CONTAINER,
	GENERAL_INFORMATION_CONTAINER,
	DEVICE_CONFIG_CONTAINER,
	FLASH_CONFIG_CONTAINER,
	GUEST_SERIALIZATION_CONTAINER,
	GLOBAL_PARAMETERS_CONTAINER,
	CORE_CODE_CONTAINER,
	CORE_CONFIG_CONTAINER,
	DISPLAY_CONFIG_CONTAINER,
	EXTERNAL_TOUCH_AFE_CONFIG_CONTAINER,
	UTILITY_CONTAINER,
	UTILITY_PARAMETER_CONTAINER,
};

enum utility_parameter_id {
	UNUSED = 0,
	FORCE_PARAMETER,
	ANTI_BENDING_PARAMETER,
};

/*
 * struct synaptics_rmi4_fn_desc - function descriptor fields in PDT entry
 * @query_base_addr: base address for query registers
 * @cmd_base_addr: base address for command registers
 * @ctrl_base_addr: base address for control registers
 * @data_base_addr: base address for data registers
 * @intr_src_count: number of interrupt sources
 * @fn_version: version of function
 * @fn_number: function number
 */
struct synaptics_rmi4_fn_desc {
	union {
		struct {
			unsigned char query_base_addr;
			unsigned char cmd_base_addr;
			unsigned char ctrl_base_addr;
			unsigned char data_base_addr;
			unsigned char intr_src_count:3;
			unsigned char reserved_1:2;
			unsigned char fn_version:2;
			unsigned char reserved_2:1;
			unsigned char fn_number;
		} __packed;
		unsigned char data[6];
	};
};

struct pdt_properties {
	union {
		struct {
			unsigned char reserved_1:6;
			unsigned char has_bsr:1;
			unsigned char reserved_2:1;
		} __packed;
		unsigned char data[1];
	};
};

struct partition_table {
	unsigned char partition_id:5;
	unsigned char byte_0_reserved:3;
	unsigned char byte_1_reserved;
	unsigned char partition_length_7_0;
	unsigned char partition_length_15_8;
	unsigned char start_physical_address_7_0;
	unsigned char start_physical_address_15_8;
	unsigned char partition_properties_7_0;
	unsigned char partition_properties_15_8;
} __packed;

struct f01_device_control {
	union {
		struct {
			unsigned char sleep_mode:2;
			unsigned char nosleep:1;
			unsigned char reserved:2;
			unsigned char charger_connected:1;
			unsigned char report_rate:1;
			unsigned char configured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f34_v7_query_0 {
	union {
		struct {
			unsigned char subpacket_1_size:3;
			unsigned char has_config_id:1;
			unsigned char f34_query0_b4:1;
			unsigned char has_thqa:1;
			unsigned char f34_query0_b6__7:2;
		} __packed;
		unsigned char data[1];
	};
};

struct f34_v7_query_1_7 {
	union {
		struct {
			/* query 1 */
			unsigned char bl_minor_revision;
			unsigned char bl_major_revision;

			/* query 2 */
			unsigned char bl_fw_id_7_0;
			unsigned char bl_fw_id_15_8;
			unsigned char bl_fw_id_23_16;
			unsigned char bl_fw_id_31_24;

			/* query 3 */
			unsigned char minimum_write_size;
			unsigned char block_size_7_0;
			unsigned char block_size_15_8;
			unsigned char flash_page_size_7_0;
			unsigned char flash_page_size_15_8;

			/* query 4 */
			unsigned char adjustable_partition_area_size_7_0;
			unsigned char adjustable_partition_area_size_15_8;

			/* query 5 */
			unsigned char flash_config_length_7_0;
			unsigned char flash_config_length_15_8;

			/* query 6 */
			unsigned char payload_length_7_0;
			unsigned char payload_length_15_8;

			/* query 7 */
			unsigned char f34_query7_b0:1;
			unsigned char has_bootloader:1;
			unsigned char has_device_config:1;
			unsigned char has_flash_config:1;
			unsigned char has_manufacturing_block:1;
			unsigned char has_guest_serialization:1;
			unsigned char has_global_parameters:1;
			unsigned char has_core_code:1;
			unsigned char has_core_config:1;
			unsigned char has_guest_code:1;
			unsigned char has_display_config:1;
			unsigned char f34_query7_b11__15:5;
			unsigned char f34_query7_b16__23;
			unsigned char f34_query7_b24__31;
		} __packed;
		unsigned char data[21];
	};
};

struct f34_v7_data0 {
	union {
		struct {
			unsigned char operation_status:5;
			unsigned char device_cfg_status:2;
			unsigned char bl_mode:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f34_v7_data_1_5 {
	union {
		struct {
			unsigned char partition_id:5;
			unsigned char f34_data1_b5__7:3;
			unsigned char block_offset_7_0;
			unsigned char block_offset_15_8;
			unsigned char transfer_length_7_0;
			unsigned char transfer_length_15_8;
			unsigned char command;
			unsigned char payload_0;
			unsigned char payload_1;
		} __packed;
		unsigned char data[8];
	};
};

struct f34_v5v6_flash_properties {
	union {
		struct {
			unsigned char reg_map:1;
			unsigned char unlocked:1;
			unsigned char has_config_id:1;
			unsigned char has_pm_config:1;
			unsigned char has_bl_config:1;
			unsigned char has_disp_config:1;
			unsigned char has_ctrl1:1;
			unsigned char has_query4:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f34_v5v6_flash_properties_2 {
	union {
		struct {
			unsigned char has_guest_code:1;
			unsigned char f34_query4_b1:1;
			unsigned char has_gesture_config:1;
			unsigned char has_force_config:1;
			unsigned char has_lockdown_data:1;
			unsigned char has_lcm_data:1;
			unsigned char has_oem_data:1;
			unsigned char f34_query4_b7:1;
		} __packed;
		unsigned char data[1];
	};
};

struct register_offset {
	unsigned char properties;
	unsigned char properties_2;
	unsigned char block_size;
	unsigned char block_count;
	unsigned char gc_block_count;
	unsigned char flash_status;
	unsigned char partition_id;
	unsigned char block_number;
	unsigned char transfer_length;
	unsigned char flash_cmd;
	unsigned char payload;
};

struct block_count {
	unsigned short ui_firmware;
	unsigned short ui_config;
	unsigned short dp_config;
	unsigned short pm_config;
	unsigned short fl_config;
	unsigned short bl_image;
	unsigned short bl_config;
	unsigned short utility_param;
	unsigned short lockdown;
	unsigned short guest_code;
#ifdef SYNA_TDDI
	unsigned short tddi_force_config;
	unsigned short tddi_lockdown_data;
	unsigned short tddi_lcm_data;
	unsigned short tddi_oem_data;
#endif
	unsigned short total_count;
};

struct physical_address {
	unsigned short ui_firmware;
	unsigned short ui_config;
	unsigned short dp_config;
	unsigned short pm_config;
	unsigned short fl_config;
	unsigned short bl_image;
	unsigned short bl_config;
	unsigned short utility_param;
	unsigned short lockdown;
	unsigned short guest_code;
};

struct container_descriptor {
	unsigned char content_checksum[4];
	unsigned char container_id[2];
	unsigned char minor_version;
	unsigned char major_version;
	unsigned char reserved_08;
	unsigned char reserved_09;
	unsigned char reserved_0a;
	unsigned char reserved_0b;
	unsigned char container_option_flags[4];
	unsigned char content_options_length[4];
	unsigned char content_options_address[4];
	unsigned char content_length[4];
	unsigned char content_address[4];
};

struct image_header_10 {
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char minor_header_version;
	unsigned char major_header_version;
	unsigned char reserved_08;
	unsigned char reserved_09;
	unsigned char reserved_0a;
	unsigned char reserved_0b;
	unsigned char top_level_container_start_addr[4];
};

struct image_header_05_06 {
	/* 0x00 - 0x0f */
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char options_firmware_id:1;
	unsigned char options_bootloader:1;
	unsigned char options_guest_code:1;
	unsigned char options_tddi:1;
	unsigned char options_reserved:4;
	unsigned char header_version;
	unsigned char firmware_size[4];
	unsigned char config_size[4];
	/* 0x10 - 0x1f */
	unsigned char product_id[PRODUCT_ID_SIZE];
	unsigned char package_id[2];
	unsigned char package_id_revision[2];
	unsigned char product_info[PRODUCT_INFO_SIZE];
	/* 0x20 - 0x2f */
	unsigned char bootloader_addr[4];
	unsigned char bootloader_size[4];
	unsigned char ui_addr[4];
	unsigned char ui_size[4];
	/* 0x30 - 0x3f */
	unsigned char ds_id[16];
	/* 0x40 - 0x4f */
	union {
		struct {
			unsigned char cstmr_product_id[PRODUCT_ID_SIZE];
			unsigned char reserved_4a_4f[6];
		};
		struct {
			unsigned char dsp_cfg_addr[4];
			unsigned char dsp_cfg_size[4];
			unsigned char reserved_48_4f[8];
		};
	};
	/* 0x50 - 0x53 */
	unsigned char firmware_id[4];
};

struct block_data {
	unsigned int size;
	const unsigned char *data;
};

struct image_metadata {
	bool contains_firmware_id;
	bool contains_bootloader;
	bool contains_guest_code;
	bool contains_disp_config;
	bool contains_perm_config;
	bool contains_flash_config;
	bool contains_utility_param;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int bootloader_size;
	unsigned int disp_config_offset;
	unsigned char bl_version;
	unsigned char product_id[PRODUCT_ID_SIZE + 1];
	unsigned char cstmr_product_id[PRODUCT_ID_SIZE + 1];
	unsigned char utility_param_id[MAX_UTILITY_PARAMS];
	struct block_data bootloader;
	struct block_data utility;
	struct block_data ui_firmware;
	struct block_data ui_config;
	struct block_data dp_config;
	struct block_data pm_config;
	struct block_data fl_config;
	struct block_data bl_image;
	struct block_data bl_config;
	struct block_data utility_param[MAX_UTILITY_PARAMS];
	struct block_data lockdown;
	struct block_data guest_code;
	struct block_count blkcount;
	struct physical_address phyaddr;
};

struct synaptics_rmi4_fwu_handle {
	enum bl_version bl_version;
	bool initialized;
	bool in_bl_mode;
	bool in_ub_mode;
	bool bl_mode_device;
	bool force_update;
	bool do_lockdown;
	bool has_guest_code;
#ifdef SYNA_TDDI
	bool has_force_config;
	bool has_lockdown_data;
	bool has_lcm_data;
	bool has_oem_data;
#endif
	bool has_utility_param;
	bool new_partition_table;
	bool incompatible_partition_tables;
	bool write_bootloader;
	unsigned int data_pos;
	unsigned char *ext_data_source;
	unsigned char *read_config_buf;
	unsigned char intr_mask;
	unsigned char command;
	unsigned char bootloader_id[2];
	unsigned char config_id[32];
	unsigned char flash_status;
	unsigned char partitions;
#ifdef F51_DISCRETE_FORCE
	unsigned char *cal_data;
	unsigned short cal_data_off;
	unsigned short cal_data_size;
	unsigned short cal_data_buf_size;
	unsigned short cal_packet_data_size;
#endif
	unsigned short block_size;
	unsigned short config_size;
	unsigned short config_area;
	unsigned short config_block_count;
	unsigned short flash_config_length;
	unsigned short payload_length;
	unsigned short partition_table_bytes;
	unsigned short read_config_buf_size;
	const unsigned char *config_data;
	const unsigned char *image;
	unsigned char *image_name;
	unsigned int image_size;
	struct image_metadata img;
	struct register_offset off;
	struct block_count blkcount;
	struct physical_address phyaddr;
	struct f34_v5v6_flash_properties flash_properties;
	struct synaptics_rmi4_fn_desc f34_fd;
	struct synaptics_rmi4_fn_desc f35_fd;
	struct synaptics_rmi4_data *rmi4_data;
	struct workqueue_struct *fwu_workqueue;
	struct work_struct fwu_work;

	struct device *ts_dev;
	struct device dev;
	struct s3707_data *d;
};

/*
 * struct synaptics_rmi4_data - RMI4 device instance data
 * @pdev: pointer to platform device
 * @input_dev: pointer to associated input device
 * @rmi4_reset_mutex: mutex for software reset
 * @rmi4_exp_init_mutex: mutex for expansion function module initialization
 * @rmi4_irq_enable_mutex: mutex for enabling/disabling interrupt
 * @early_suspend: early suspend power management
 * @intr_mask: interrupt enable mask
 * @num_of_intr_regs: number of interrupt registers
 * @f01_query_base_addr: query base address for f$01
 * @f01_cmd_base_addr: command base address for f$01
 * @f01_ctrl_base_addr: control base address for f$01
 * @f01_data_base_addr: data base address for f$01
 * @f51_query_base_addr: query base address for f$51
 * @firmware_id: firmware build ID
 * @sensor_sleep: flag to indicate sleep state of sensor
 * @stay_awake: flag to indicate whether to stay awake during suspend
 * @fb_ready: flag to indicate whether display framebuffer in ready state
 */
struct synaptics_rmi4_data {
	struct platform_device *pdev;
	struct input_dev *input_dev;
	struct mutex rmi4_reset_mutex;
	struct mutex rmi4_exp_init_mutex;
	struct mutex rmi4_irq_enable_mutex;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	unsigned char intr_mask[MAX_INTR_REGISTERS];
	unsigned short num_of_intr_regs;
	unsigned short f01_query_base_addr;
	unsigned short f01_cmd_base_addr;
	unsigned short f01_ctrl_base_addr;
	unsigned short f01_data_base_addr;
#ifdef F51_DISCRETE_FORCE
	unsigned short f51_query_base_addr;
#endif
	unsigned int firmware_id;
	bool sensor_sleep;
	bool stay_awake;
	bool fb_ready;
};

int synaptics_fw_updater(const unsigned char *fw_data);

static inline ssize_t synaptics_rmi4_show_error(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	dev_warn(dev, "%s Attempted to read from write-only attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

static inline ssize_t synaptics_rmi4_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	dev_warn(dev, "%s Attempted to write to read-only attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

static inline int synaptics_rmi4_reg_read(struct device *dev,
		unsigned short addr, unsigned char *data, unsigned int len)
{
	return s3707_read(dev, addr, data, len);
}

static inline int synaptics_rmi4_reg_write(struct device *dev,
		unsigned short addr, unsigned char *data, unsigned int len)
{
	return s3707_write(dev, addr, data, len);
}

static inline int secure_memcpy(unsigned char *dest, unsigned int dest_size,
		const unsigned char *src, unsigned int src_size,
		unsigned int count)
{
	if (dest == NULL || src == NULL)
		return -EINVAL;

	if (count > dest_size || count > src_size)
		return -EINVAL;

	memcpy((void *)dest, (const void *)src, count);

	return 0;
}

static inline void batohs(unsigned short *dest, unsigned char *src)
{
	*dest = src[1] * 0x100 + src[0];
}

#endif
