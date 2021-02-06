/* touch_s3618.h
 *
 * Copyright (C) 2018 LGE.
 *
 * Author: BSP-TOUCH@lge.com
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

#ifndef LGE_TOUCH_s3618_H
#define LGE_TOUCH_s3618_H

#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/firmware.h>
#include <linux/input/lge_touch_notify.h>
#include <linux/spi/spi.h>
#include <linux/pm_qos.h>

#include <touch_hwif.h>
#include <touch_core.h>

#define USE_DEFAULT_TOUCH_REPORT_CONFIG

#define CHUNK_SIZE			1024 /* RW Limit in bytes, 0 = unlimited */
#define BUFFER_SIZE			2560

#define MESSAGE_MARKER			0xA5
#define MESSAGE_PADDING			0x5A
#define MESSAGE_HEADER_SIZE		4

#define TOUCH_REPORT_CONFIG_SIZE	128

#define WRITE_DELAY			1
#define READ_RETRY_TIMEOUT		100
#define RESPONSE_TIMEOUT		500
#define RESPONSE_TIMEOUT_LONG		3000
#define STATUS_POLL_TIMEOUT_MS		500
#define STATUS_POLL_MS			100

#define LPWG_SENSELESS_AREA_W		70	/* pixel */
#define LPWG_FAILREASON_TCI_NUM		9
#define LPWG_FAILREASON_SWIPE_NUM	12
#define LPWG_FAILREASON_MAX_BUFFER	10

#define PLATFORM_DRIVER_NAME "synaptics_tcm"

enum {
	LCD_MODE_U0 = 0,
	LCD_MODE_U2_UNBLANK,
	LCD_MODE_U2,
	LCD_MODE_U3,
	LCD_MODE_U3_PARTIAL,
	LCD_MODE_U3_QUICKCOVER,
	LCD_MODE_STOP,
};

enum {
	IC_INIT_NEED = 0,
	IC_INIT_DONE,
};

enum {
	SW_RESET = 0,
	HW_RESET,
	SW_RESET_NO_INIT,
};

enum {
	REPORT_DISABLE = 0,
	REPORT_ENABLE,
};

enum {
	S3618_SWIPE_U = 0,
	S3618_SWIPE_L = 1,
	S3618_SWIPE_R = 2,
	S3618_SWIPE_L2 = 3,
	S3618_SWIPE_R2 = 4,
	S3618_SWIPE_NUM = 5,
};

enum {
	SWIPE_DISABLE = 0,
	SWIPE_ENABLE,
};

enum {
	LPWG_ABS_DISABLE = 0,
	LPWG_ABS_ENABLE,
};

enum {
	AI_PICK_DISABLE = 0,
	AI_PICK_ENABLE,
};

enum {
	LPWG_LONGPRESS_DISABLE = 0,
	LPWG_LONGPRESS_ENABLE,
};

enum {
	LPWG_ONETAP_DISABLE = 0,
	LPWG_ONETAP_ENABLE,
};

/* Synaptics enum */

enum firmware_mode {
	MODE_APPLICATION_FIRMWARE = 0x01,
	MODE_HOSTDOWNLOAD_FIRMWARE = 0x02,
	MODE_ROMBOOTLOADER = 0x04,
	MODE_BOOTLOADER = 0x0b,
	MODE_TDDI_BOOTLOADER = 0x0c,
	MODE_TDDI_HOSTDOWNLOAD_BOOTLOADER = 0x0d,
	MODE_PRODUCTIONTEST_FIRMWARE = 0x0e,
};

enum command {
	CMD_NONE = 0x00,
	CMD_CONTINUE_WRITE = 0x01,
	CMD_IDENTIFY = 0x02,
	CMD_RESET = 0x04,
	CMD_ENABLE_REPORT = 0x05,
	CMD_DISABLE_REPORT = 0x06,
	CMD_GET_BOOT_INFO = 0x10,
	CMD_ERASE_FLASH = 0x11,
	CMD_WRITE_FLASH = 0x12,
	CMD_READ_FLASH = 0x13,
	CMD_RUN_APPLICATION_FIRMWARE = 0x14,
	CMD_SPI_MASTER_WRITE_THEN_READ = 0x15,
	CMD_REBOOT_TO_ROM_BOOTLOADER = 0x16,
	CMD_RUN_BOOTLOADER_FIRMWARE = 0x1f,
	CMD_GET_APPLICATION_INFO = 0x20,
	CMD_GET_STATIC_CONFIG = 0x21,
	CMD_SET_STATIC_CONFIG = 0x22,
	CMD_GET_DYNAMIC_CONFIG = 0x23,
	CMD_SET_DYNAMIC_CONFIG = 0x24,
	CMD_GET_TOUCH_REPORT_CONFIG = 0x25,
	CMD_SET_TOUCH_REPORT_CONFIG = 0x26,
	CMD_REZERO = 0x27,
	CMD_COMMIT_CONFIG = 0x28,
	CMD_DESCRIBE_DYNAMIC_CONFIG = 0x29,
	CMD_PRODUCTION_TEST = 0x2a,
	CMD_SET_CONFIG_ID = 0x2b,
	CMD_ENTER_DEEP_SLEEP = 0x2c,
	CMD_EXIT_DEEP_SLEEP = 0x2d,
	CMD_GET_TOUCH_INFO = 0x2e,
	CMD_GET_DATA_LOCATION = 0x2f,
	CMD_DOWNLOAD_CONFIG = 0x30,
	CMD_ENTER_PRODUCTION_TEST_MODE = 0x31,
	CMD_GET_FEATURES = 0x32,
	CMD_GET_ROMBOOT_INFO = 0x40,
	CMD_WRITE_PROGRAM_RAM = 0x41,
	CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE = 0x42,
	CMD_SPI_MASTER_WRITE_THEN_READ_EXTENDED = 0x43,
	CMD_ENTER_IO_BRIDGE_MODE = 0x44,
	CMD_ROMBOOT_DOWNLOAD = 0x45,
	CMD_SET_LGE_GESTURE_CONFIG = 0xc0,
	CMD_GET_LGE_GESTURE_FAILREASON = 0xc1,
};

enum status_code {
	STATUS_IDLE = 0x00,
	STATUS_OK = 0x01,
	STATUS_BUSY = 0x02,
	STATUS_CONTINUED_READ = 0x03,
	STATUS_NOT_EXECUTED_IN_DEEP_SLEEP = 0x0b,
	STATUS_RECEIVE_BUFFER_OVERFLOW = 0x0c,
	STATUS_PREVIOUS_COMMAND_PENDING = 0x0d,
	STATUS_NOT_IMPLEMENTED = 0x0e,
	STATUS_ERROR = 0x0f,
	STATUS_INVALID = 0xff,
};

enum report_type {
	REPORT_IDENTIFY = 0x10,
	REPORT_TOUCH = 0x11,
	REPORT_DELTA = 0x12,
	REPORT_RAW = 0x13,
	REPORT_STATUS = 0x1b,
	REPORT_PRINTF = 0x82,
	REPORT_HDL_ROMBOOT = 0xfd,
	REPORT_HDL_F35 = 0xfe,
};

enum app_status {
	APP_STATUS_OK = 0x00,
	APP_STATUS_BOOTING = 0x01,
	APP_STATUS_UPDATING = 0x02,
	APP_STATUS_BAD_APP_CONFIG = 0xff,
};

enum boot_status {
	BOOT_STATUS_OK = 0x00,
	BOOT_STATUS_BOOTING = 0x01,
	BOOT_STATUS_BOOTLOADER_REQUEST = 0x02,
	BOOT_STATUS_APP_BAD_DISPLAY_CRC = 0xfc,
	BOOT_STATUS_BAD_DISPLAY_CONFIG = 0xfd,
	BOOT_STATUS_BAD_APP_FIRMWARE = 0xfe,
	BOOT_STATUS_WARM_BOOT = 0xff,
};

enum romboot_status {
	ROMBOOT_STATUS_OK = 0x00,
};

enum dynamic_config_id {
	DC_UNKNOWN = 0x00,
	DC_NO_DOZE,
	DC_DISABLE_NOISE_MITIGATION,
	DC_INHIBIT_FREQUENCY_SHIFT,
	DC_REQUESTED_FREQUENCY,
	DC_DISABLE_HSYNC,
	DC_REZERO_ON_EXIT_DEEP_SLEEP,
	DC_CHARGER_CONNECTED,
	DC_NO_BASELINE_RELAXATION,
	DC_IN_WAKEUP_GESTURE_MODE,
	DC_STIMULUS_FINGERS,
	DC_GRIP_SUPPRESSION_ENABLE,
	DC_ENABLE_THICK_GLOVE,
	DC_ENABLE_GLOVE,
	DC_IME_MODE_ENABLE = 212,
	DC_SENSITIVE_MODE_ENABLE = 215,
	DC_WIRELESS_CHARGER_CONNECTED = 216,
	DC_DUALSCREEN_CONNECTED = 217,
};

enum report_classification {
	NO_OBJECT = 0,
	FINGER = 1,
	GLOVE = 2,
	STYLUS = 3,
	ERASER = 4,
	SMALL_OBJECT = 5,
	PALM = 6,
	UNKNOWN = 7,
	EDGE_TOUCH = 8,
	HOVERING_OBJECT = 9,
};

enum report_code {
	TOUCH_END = 0,
	TOUCH_FOREACH_ACTIVE_OBJECT,
	TOUCH_FOREACH_OBJECT,
	TOUCH_FOREACH_END,
	TOUCH_PAD_TO_NEXT_BYTE,
	TOUCH_TIMESTAMP,
	TOUCH_OBJECT_N_INDEX,				// 0x6
	TOUCH_OBJECT_N_CLASSIFICATION,
	TOUCH_OBJECT_N_X_POSITION,
	TOUCH_OBJECT_N_Y_POSITION,
	TOUCH_OBJECT_N_Z,				// 0xA
	TOUCH_OBJECT_N_X_WIDTH,
	TOUCH_OBJECT_N_Y_WIDTH,
	TOUCH_OBJECT_N_TX_POSITION_TIXELS,
	TOUCH_OBJECT_N_RX_POSITION_TIXELS,
	TOUCH_0D_BUTTONS_STATE,				// 0xF
	TOUCH_GESTURE_ID,				// 0x10
	TOUCH_FRAME_RATE,
	TOUCH_POWER_IM,					// 0x12
	TOUCH_CID_IM,
	TOUCH_RAIL_IM,
	TOUCH_CID_VARIANCE_IM,
	TOUCH_NSM_FREQUENCY,
	TOUCH_NSM_STATE,				// 0x17
	TOUCH_NUM_OF_ACTIVE_OBJECTS,			// 0x18
	TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME,
	TOUCH_FACE_DETECT,				// 0x1A
	TOUCH_GESTURE_DATA,
	TOUCH_OBJECT_N_FORCE,
	TOUCH_FINGERPRINT_AREA_MEET,
	TOUCH_TUNING_GAUSSIAN_WIDTHS = 0x80,
	TOUCH_TUNING_SMALL_OBJECT_PARAMS,
	TOUCH_TUNING_0D_BUTTONS_VARIANCE,
	// LGE Custom report code
	TOUCH_OBJECT_N_AREA = 0xC0,
	TOUCH_OBJECT_N_ANGLE,
	TOUCH_OBJECT_N_MAJOR,
	TOUCH_OBJECT_N_MINOR,
	TOUCH_BASELINE_ERR_LOG,				// 0xC4
	TOUCH_PALM_DETECTED,				// 0xC5
	TOUCH_CUSTOMER_GESTURE_DETECTED,
	TOUCH_CUSTOMER_GESTURE_INFO,
	TOUCH_CUSTOMER_GESTURE_INFO2,
};

enum command_status {
	CMD_IDLE = 0,
	CMD_BUSY = 1,
	CMD_ERROR = -1,
};

enum flash_area {
	BOOTLOADER = 0,
	BOOT_CONFIG,
	APP_FIRMWARE,
	APP_CONFIG,
	DISP_CONFIG,
	CUSTOM_OTP,
	CUSTOM_LCM,
	CUSTOM_OEM,
	PPDT,
};

enum flash_data {
	LCM_DATA = 1,
	OEM_DATA,
	PPDT_DATA,
};

enum gesture_detected_data {
	DETECT_NORMAL_TOUCH = 0x0,
	DETECT_KNOCK_ON = 0x1,
	DETECT_KNOCK_CODE = 0x2,
	DETECT_LONG_PRESS_DOWN = 0x4,
	DETECT_LONG_PRESS_UP = 0x8,
	DETECT_SWIPE = 0x10,
	DETECT_LONG_PRESS = 0x20,
	DETECT_ONE_TAP = 0x40,
};

enum gesture_config_code {
	CONFIG_CODE_KNOCK_ON = 0x0,
	CONFIG_CODE_KNOCK_CODE,
	CONFIG_CODE_SWIPE_UP,
	CONFIG_CODE_SWIPE_LEFT,
	CONFIG_CODE_SWIPE_RIGHT,
	CONFIG_CODE_SWIPE_LEFT2,
	CONFIG_CODE_SWIPE_RIGHT2,
	CONFIG_CODE_LONG_PRESS,
	CONFIG_CODE_ONE_TAP,
	CONFIG_CODE_LPWG_ABS,
	CONFIG_CODE_TCI_ACTIVE_AREA,
	CONFIG_CODE_MAX,
};

struct s3618_point {
	s16 x;
	s16 y;
} __packed;

struct s3618_coord_data {
	struct s3618_point point[MAX_LPWG_CODE];
} __packed;

struct s3618_swipe_coord_data {
	u16 index; // 0:UP, 1:LEFT 2:RIGHT ?:LEFT2 ?:RIGHT2
	u16 start_x;
	u16 start_y;
	u16 end_x;
	u16 end_y;
	u16 swipe_time;
} __packed;

struct s3618_lpwg_data {
	u8 buf[40]; // 160bit (GESTURE_INFO) + 160bit (GESTURE_INFO2)
	struct s3618_coord_data coord_data;
	struct s3618_swipe_coord_data swipe_coord_data;
};

struct s3618_touch_data {
	u8 index:4;
	u8 classification:4;
	u16 x_pos:12;
	u16 y_pos:12;
	u8 z;
	u8 x_width;
	u8 y_width;
	u16 tx_pos;
	u16 rx_pos;
	u8 area;
	s8 angle;
	u16 major;
	u16 minor;
} __packed;

struct s3618_touch_info {
	struct s3618_touch_data data[MAX_FINGER];
	u8 gesture_detected;
	struct s3618_lpwg_data lpwg_data;
	u16 palm_detected;
	u16 baseline_err_log;
	u32 timestamp;
	u16 buttons_state;
	u8 gesture_id;
	u8 frame_rate;
	u16 power_im;
	u32 cid_im;
	u32 rail_im;
	u32 cid_variance_im;
	u8 nsm_frequency:4;
	u8 nsm_state;
	u32 num_of_active_objects;
	u32 num_of_cpu_cycles;
	u8 face_detected:1;
	u16 force_measurement;
	u8 fingerprint_area_meet;
	u8 sensing_mode;
} __packed;

struct s3618_failreason_info {
	u8 knock_on[LPWG_FAILREASON_MAX_BUFFER];
	u8 swipe[LPWG_FAILREASON_MAX_BUFFER];
} __packed;

struct s3618_blank_status {
	int prev;
	int curr;
};

struct s3618_active_area {
	s16 x1;
	s16 y1;
	s16 x2;
	s16 y2;
} __packed;

// LPWG Write s3618 BUF struct
struct s3618_tci_data {
	u16 enable;
	u16 tap_count;
	u16 min_intertap;
	u16 max_intertap;
	u16 touch_slop;
	u16 tap_distance;
	u16 enable_delay;
	u16 intr_delay;
} __packed;

struct s3618_tci_buf {
	u8 gesture_config_code;
	struct s3618_tci_data data;
} __packed;

struct s3618_swipe_data {
	u16 enable;
	u8 distance;
	u8 ratio_thres;
	u16 min_time;
	u16 max_time;
	struct s3618_active_area area;
	struct s3618_active_area start_area;
	u16 wrong_dir_thres;
	u8 init_ratio_chk_dist;
	u8 init_ratio_thres;
} __packed;

struct s3618_swipe_buf {
	u8 gesture_config_code;
	struct s3618_swipe_data data;
} __packed;

struct s3618_lpwg_abs_buf {
	u8 gesture_config_code;
	u16 enable;
	struct s3618_active_area area;
} __packed;

struct s3618_lpwg_longpress_buf {
	u8 gesture_config_code;
	u8 enable;
	u8 touch_slop;
	u8 min_press_time;
	u8 min_contact_size;
	struct s3618_active_area area;
} __packed;

struct s3618_lpwg_onetap_buf {
	u8 gesture_config_code;
	u16 enable;
	struct s3618_active_area area;
	u8 touch_slop;
	u8 max_press_time;
	u16 interrupt_delay_time;
} __packed;

struct s3618_tci_active_area_buf {
	u8 gesture_config_code;
	struct s3618_active_area area;
} __packed;

// LPWG Ctrl struct
struct s3618_lpwg_abs_ctrl {
	u8 enable;
	struct s3618_active_area area;
	struct s3618_active_area border_area;
	s16 offset_y;
};

struct s3618_ai_pick_ctrl {
	u8 enable;
	struct s3618_active_area area;
	struct s3618_active_area border_area;
	struct s3618_active_area total_area;
};

struct s3618_lpwg_longpress_ctrl {
	u8 enable;
	int touch_slop;
	int min_press_time;
	int min_contact_size;
	struct s3618_active_area area;
#if defined(USE_LONGPRESS_AROUND)
	struct s3618_active_area around_area;
#endif
};

struct s3618_lpwg_onetap_ctrl {
	u8 enable;
	struct s3618_active_area area;
	int touch_slop;
	int max_press_time;
	int interrupt_delay_time;
};

struct s3618_state_info {
	atomic_t init;
	atomic_t config;
	atomic_t esd_recovery;
	atomic_t power;
};

/* Synaptics struct */
struct s3618_exp_fn {
	int (*init)(struct device *dev);
	void (*remove)(struct device *dev);
	void (*syncbox)(struct device *dev);
	void (*reset)(struct device *dev);
	void (*reinit)(struct device *dev);
	void (*early_suspend)(struct device *dev);
	void (*suspend)(struct device *dev);
	void (*resume)(struct device *dev);
	void (*late_resume)(struct device *dev);
	void (*attn)(struct device *dev, unsigned char intr_mask);
};

struct s3618_exp_fhandler {
	struct s3618_exp_fn *exp_fn;
	bool insert;
	bool initialized;
	bool remove;
};

struct s3618_id_info {
	u8 version;
	u8 mode;
	u8 part_number[16];
	u32 build_id;
	u16 max_write_size;
} __packed;

/* maker/4bit		0:ELK 1:Suntel 2:Tovis 3:Innotek 4:JDI 5:LGD 6:Tianma
 * Key/4bit		0: No key ~ 4: 4 keys
 * Supplier/4bit	0:Synaptics
 * Inch0/4bit		Decimal
 * Inch1/4bit		Decimal
 * Panel/4bit		Decimal
 * Release(Major)/1bit		0:Test 1:Official
 * Version(Minor)/7bit		Decimal
 * ex) 0x60064101 */
struct s3618_customer_config_id {
	u8 key:4;
	u8 maker:4;
	u8 inch0:4;
	u8 supplier:4;
	u8 panel:4;
	u8 inch1:4;
	u8 version:7;
	u8 release:1;
	u8 reserved[12];
} __packed;

struct s3618_app_info {
	u16 packet_version;
	u16 status;
	u16 static_config_size;
	u16 dynamic_config_size;
	u16 app_config_start_write_block;
	u16 app_config_size;
	u16 max_touch_report_config_size;
	u16 max_touch_report_payload_size;
	//u8 customer_config_id[16];
	struct s3618_customer_config_id customer_config_id;
	u16 max_x;
	u16 max_y;
	u16 max_objects;
	u16 num_of_buttons;
	u16 num_of_image_rows;
	u16 num_of_image_cols;
	u16 has_hybrid_data;
	u16 num_of_force_elecs;
} __packed;

struct s3618_bin_fw_version_info {
	u32 build_id;
	u8 version:7;
	u8 release:1;
} __packed;

struct s3618_boot_info {
	u8 packet_version;
	u8 status;
	u16 asic_id;
	u8 write_block_size_words;
	u16 erase_page_size_words;
	u16 max_write_payload_size;
	u8 last_reset_reason;
	u16 pc_at_time_of_last_reset;
	u16 boot_config_start_block;
	u16 boot_config_size_blocks;
	u32 display_config_start_block;
	u16 display_config_length_blocks;
	u32 backup_display_config_start_block;
	u16 backup_display_config_length_blocks;
	u16 custom_otp_start_block;
	u16 custom_otp_length_blocks;
} __packed;

struct s3618_romboot_info {
	u8 packet_version;
	u8 status;
	u16 asic_id;
	u8 write_block_size_words;
	u16 max_write_payload_size;
	u8 last_reset_reason;
	u16 pc_at_time_of_last_reset;
} __packed;

struct s3618_prd_info {
	u8 product_id[6];
	u8 chip_ver;
	u8 fpc_ver;
	u8 sensor_ver;
	u8 inspect_channel;
	u8 inspect_date[3];
	u8 inspect_time[3];
} __packed;

struct s3618_buffer {
	u8 buf[BUFFER_SIZE];
	u32 data_length;
	struct mutex buf_mutex;
};

struct s3618_message_header {
	u8 marker;
	u8 code;
	u16 length;
};

struct s3618_data {
	struct s3618_id_info id_info;
	struct s3618_app_info app_info;
	struct s3618_bin_fw_version_info bin_fw_version_info;
	struct s3618_boot_info boot_info;
	struct s3618_romboot_info romboot_info;
	struct s3618_touch_info touch_info;
	struct s3618_failreason_info failreason_info;
	struct s3618_state_info state;
	struct s3618_prd_info prd_info;
	struct s3618_lpwg_abs_ctrl lpwg_abs;
	struct s3618_ai_pick_ctrl ai_pick;
	struct s3618_lpwg_longpress_ctrl lpwg_longpress;
	struct s3618_lpwg_onetap_ctrl lpwg_onetap;

	struct s3618_buffer in;
	struct s3618_buffer out;
	struct s3618_buffer response;
	struct s3618_buffer report;
	struct s3618_buffer config;
	struct s3618_buffer temp;
	struct s3618_blank_status blank_status;

	struct mutex command_mutex;
	struct mutex rw_ctrl_mutex;
	struct mutex io_mutex;
	struct mutex reset_mutex;
	struct completion response_complete;
	struct pm_qos_request pm_qos_req;
	struct delayed_work fb_notify_work;

	atomic_t command_status;

	u8 command;
	u8 status_report_code;
	u8 response_code;
	u8 lcd_mode;
	u8 prev_lcd_mode;
	u32 payload_length;
	u32 chunk_size;
	u32 longpress_uevent_status;

	bool is_palm;
	bool report_is_ready;

	// touch_s3618_reflash.c
	struct mutex extif_mutex;

	// touch_s3618_device.c
	bool isDeviceOpen;
	int (*report_touch)(struct device *);
};

static inline struct s3618_data *to_s3618_data(struct device *dev)
{
	return (struct s3618_data *)touch_get_device(to_touch_core(dev));
}

static inline unsigned int le2_to_uint(const unsigned char *src)
{
	return (unsigned int)src[0] +
			(unsigned int)src[1] * 0x100;
}

static inline unsigned int le4_to_uint(const unsigned char *src)
{
	return (unsigned int)src[0] +
			(unsigned int)src[1] * 0x100 +
			(unsigned int)src[2] * 0x10000 +
			(unsigned int)src[3] * 0x1000000;
}

static inline unsigned int ceil_div(unsigned int dividend, unsigned divisor)
{
	return (dividend + divisor - 1) / divisor;
}

#define MAX(a, b) \
	({__typeof__(a) _a = (a); \
	 __typeof__(b) _b = (b); \
	 _a > _b ? _a : _b; })

#define MIN(a, b) \
	({__typeof__(a) _a = (a); \
	 __typeof__(b) _b = (b); \
	 _a < _b ? _a : _b; })

#define ABS(x) ((x < 0) ? (-x) : (x))

#define STR(x) #x

#define CONCAT(a, b) a##b

#define STORE_PROTOTYPE(m_name, a_name) \
	static ssize_t CONCAT(m_name##_sysfs, _##a_name##_store)(struct device *dev, \
			struct device_attribute *attr, const char *buf, size_t count); \
\
static struct device_attribute dev_attr_##a_name = \
__ATTR(a_name, (S_IWUSR | S_IWGRP), \
		NULL, \
		CONCAT(m_name##_sysfs, _##a_name##_store));

#define SHOW_STORE_PROTOTYPE(m_name, a_name) \
	static ssize_t CONCAT(m_name##_sysfs, _##a_name##_show)(struct device *dev, \
			struct device_attribute *attr, char *buf); \
\
static ssize_t CONCAT(m_name##_sysfs, _##a_name##_store)(struct device *dev, \
		struct device_attribute *attr, const char *buf, size_t count); \
\
static struct device_attribute dev_attr_##a_name = \
__ATTR(a_name, (S_IRUGO | S_IWUSR | S_IWGRP), \
		CONCAT(m_name##_sysfs, _##a_name##_show), \
		CONCAT(m_name##_sysfs, _##a_name##_store));

#define ATTRIFY(a_name) (&dev_attr_##a_name)

int s3618_read(struct device *dev, void *data, int size);
int s3618_write(struct device *dev, void *data, int size);
int s3618_read_message(struct device *dev);
int s3618_write_message(struct device *dev, u8 command, u8 *payload, u32 length,
		u8 *response_buffer, u32 *response_length);
int s3618_init(struct device *dev);
int s3618_identify(struct device *dev);
int s3618_ic_info(struct device *dev);
int s3618_switch_mode(struct device *dev, enum firmware_mode mode);
void s3618_reset_ctrl(struct device *dev, int ctrl);
void s3618_reflash_function(struct s3618_exp_fn *fn, bool insert);
void s3618_testing_function(struct s3618_exp_fn *fn, bool insert);
void s3618_device_function(struct s3618_exp_fn *fn, bool insert);
bool s3618_is_product(struct s3618_data *d, const char *product_id, size_t len);
int s3618_set_dynamic_config(struct device *dev, u8 dynamic_config_id, u16 value);

/* extern function */
extern int s3618_fw_updater(const unsigned char *fw_data);
extern int s3618_get_bin_fw_version(const unsigned char *fw_data);
extern bool is_ddic_name(char *ddic_name);

#endif /* LGE_TOUCH_s3618_H */
