/* touch_synaptics.h
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hoyeon.jang@lge.com
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

#ifndef LGE_TOUCH_SYNAPTICS_H
#define LGE_TOUCH_SYNAPTICS_H

#include "touch_s3706_fw_upgrade.h"
#include "touch_s3706_f54_test.h"
#include "touch_s3706_prd.h"
#include "touch_common_module.h"

#define LGE_MODULE_DRIVER_NAME "S3706_driver"
/* sync up to touch core */
#define LGE_TOUCH_DRIVER_NAME "lge_touch_driver"
#define LGE_MODULE_IRQ_NAME "lge_touch_module"

/* test */
#define MAX_BUF_SIZE (64 * 1024)
#define LOG_BUF_SIZE (256)

static struct i2c_driver touch_module_driver;

int module_get_dts_data(struct module_data *md);
void module_gpio_init(int pin, char *name);
void module_gpio_direction_output(int pin, int value);
void module_gpio_direction_input(int pin);
int module_power_init(struct module_data *md);
int module_bus_init(struct module_data *md, int buf_size);
int module_init_input(struct module_data *md);

irqreturn_t module_irq_handler(int irq, void *dev_id);
irqreturn_t module_irq_thread(int irq, void *dev_id);
void module_enable_irq(unsigned int irq);
void module_disable_irq(unsigned int irq);

int module_read(struct device *dev, u16 addr, void *data, int size);
int module_i2c_read(struct i2c_client *client, struct module_bus_msg *msg);
int module_write(struct device *dev, u16 addr, void *data, int size);
int module_i2c_write(struct i2c_client *client, struct module_bus_msg *msg);
int module_initialize(struct device *dev);
int module_ic_info(struct device *dev);
int touch_check_boot_mode(struct device *dev);

void module_msleep(unsigned int msecs);

#define TYPE_S3706 8

/* S3706 Functions */
#define S3706_FUNC_01		(u8)(1 << 0)
#define S3706_FUNC_12		(u8)(1 << 1)
#define S3706_FUNC_34		(u8)(1 << 2)
#define S3706_FUNC_35		(u8)(1 << 3)
#define S3706_FUNC_51		(u8)(1 << 4)
#define S3706_FUNC_54		(u8)(1 << 5)
#define S3706_FUNC_55		(u8)(1 << 6)
#define S3706_FUNC_DC		(u8)(1 << 7)

#define MAX_POINT_SIZE_FOR_LPWG	256
#define FW_VER_INFO_NUM			4
#define MAX_NUM_OF_FINGERS		10
#define PAGE_SELECT_REG			0xff
#define PAGES_TO_SERVICE		10
#define LPWG_MAX_BUFFER			10

#define MASK_16BIT	(0xFFFF)
#define MASK_8BIT	(0xFF)
#define MASK_7BIT	(0x7F)
#define MASK_6BIT	(0x3F)
#define MASK_5BIT	(0x1F)
#define MASK_4BIT	(0x0F)
#define MASK_3BIT	(0x07)
#define MASK_2BIT	(0x03)
#define MASK_1BIT	(0x01)

#define PAGE_SELECT_LEN		(2)
#define ADDRESS_WORD_LEN	(2)

#define PLATFORM_DRIVER_NAME "synaptics_dsx"

#define DEFAULT_PAGE			0x00
#define COMMON_PAGE			(d->f01.page)
#define FINGER_PAGE			(d->f12.page)
#define ANALOG_PAGE			(d->f54.page)
#define FLASH_PAGE			(d->f34.page)
#define LPWG_PAGE			(d->f51.page)

/* RMI_DEVICE_CONTROL */
#define DEVICE_CONTROL_REG		(d->f01.dsc.control_base + 0)
#define DEVICE_CONTROL_NORMAL_OP	0x00
#define DEVICE_CONTROL_SLEEP		0x01
#define DEVICE_CONTROL_NOSLEEP		0x04
#define DEVICE_CONTROL_CONFIGURED	0x80

#define INTERRUPT_ENABLE_REG		(d->f01.dsc.control_base + 1)
#define DOZE_INTERVAL_REG		(d->f01.dsc.control_base + 2)
#define DOZE_RECAL_INTERVAL_REG		(d->f01.dsc.control_base + 6)

#define DEVICE_STATUS_REG		(d->f01.dsc.data_base + 0)
#define INTERRUPT_STATUS_REG		(d->f01.dsc.data_base + 1)
#define INTERRUPT_MASK_FLASH		(1 << 0)
#define INTERRUPT_MASK_STATUS		(1 << 1)
#define INTERRUPT_MASK_ABS0			(1 << 2)
#define INTERRUPT_MASK_BUTTON		(1 << 4)
#define INTERRUPT_MASK_CUSTOM		(1 << 5)
#define INTERRUPT_MASK_LPWG		INTERRUPT_MASK_CUSTOM

#define STATUS_CODE_MASK                0x0F
#define NORMAL_MASK_STATUS              0x00
#define RESET_MASK_STATUS               0x01
#define INVALID_CONF_MASK_STATUS        0x02
#define DEVICE_FAILURE_MASK_STATUS      0x03
#define CONF_CRC_FAILURE_MASK_STATUS    0x04
#define FW_CRC_FAILURE_MASK_STATUS      0x05
#define CRC_PROGRESS_MASK_STATUS        0x06
#define GUEST_CRC_FAILURE_MASK_STATUS   0x07
#define EXT_AFE_FAILURE_MASK_STATUS     0x08
#define DISPLAY_FAILURE_MASK_STATUS     0x09
#define FLASH_PROG_MASK_STATUS          (1<<6)
#define DEVICE_UNCONF_MASK_STATUS       (1<<7)
#define DEVICE_CONTROL_CHARGER_BIT      (1<<5)

#define FINGER_DATA_REG			(d->f12_reg.data[1])
#define FINGER_ANGLE_DATA_REG		(d->f12_reg.data[10])
#define FINGER_WIDTH_DATA_REG		(d->f12_reg.data[30])
#define F12_NO_OBJECT_STATUS		(0x00)
#define F12_FINGER_STATUS		(0x01)
#define F12_ACTIVE_STYLUS_STATUS		(0x02)
#define F12_PALM_STATUS			(0x03)
#define F12_HOVERING_FINGER_STATUS	(0x05)
#define F12_GLOVED_FINGER_STATUS	(0x06)
#define F12_NARROW_OBJECT_STATUS	(0x07)
#define F12_HAND_EDGE_STATUS		(0x08)
#define F12_COVER_STATUS		(0x0A)
#define F12_STYLUS_STATUS		(0x0B)
#define F12_ERASER_STATUS		(0x0C)
#define F12_SMALL_OBJECT_STATUS		(0x0D)

#define F12_MAX_OBJECT				(0x06)
#define FINGER_REPORT_DATA			(d->f12_reg.data[15])
#define FINGER_REPORT_REG			(d->f12_reg.ctrl[20])
#define NOISE_FLOOR_REG			(d->f12_reg.ctrl[10])
#define OBJECT_REPORT_ENABLE_REG	(d->f12_reg.ctrl[23])
#define WAKEUP_GESTURE_ENABLE_REG	(d->f12_reg.ctrl[27])

#define DRUMMING_ACCELERATION_THRESHOLD		(15)
#define MIN_DRUMMING_DISTANCE			(10)
#define IME_DRUMMING_ACCELERATION_THRESHOLD	(5)
#define IME_MIN_DRUMMING_DISTANCE		(5)

#define LPWG_STATUS_REG				(d->f51.dsc.data_base)
#define LPWG_STATUS_DOUBLETAP		(1 << 0)
#define LPWG_STATUS_PASSWORD		(1 << 1)
#define LPWG_STATUS_SWIPE		(1 << 2)
#define LPWG_DATA_REG				(d->f51.dsc.data_base + 1)
#define LPWG_OVER_TAPCOUNT			(d->f51.dsc.data_base + 73)

#define LPWG_TAPCOUNT_REG		(d->f51.dsc.control_base)
#define LPWG_MIN_INTERTAP_REG		(d->f51.dsc.control_base + 1)
#define LPWG_MAX_INTERTAP_REG		(d->f51.dsc.control_base + 2)
#define LPWG_TOUCH_SLOP_REG		(d->f51.dsc.control_base + 3)
#define LPWG_TAP_DISTANCE_REG		(d->f51.dsc.control_base + 4)
#define LPWG_INTERRUPT_DELAY_REG	(d->f51.dsc.control_base + 6)
#define LPWG_BLKSIZ			7	/* 4-page */

#define LPWG_TAPCOUNT_REG2		(d->f51.dsc.control_base + 7)
#define LPWG_MIN_INTERTAP_REG2		(d->f51.dsc.control_base + 8)
#define LPWG_MAX_INTERTAP_REG2		(d->f51.dsc.control_base + 9)
#define LPWG_TOUCH_SLOP_REG2		(d->f51.dsc.control_base + 10)
#define LPWG_TAP_DISTANCE_REG2		(d->f51.dsc.control_base + 11)
#define LPWG_INTERRUPT_DELAY_REG2	(d->f51.dsc.control_base + 13)

#define LPWG_TCI1_FAIL_COUNT_REG	(d->f51.dsc.data_base + 49)
#define LPWG_TCI1_FAIL_INDEX_REG	(d->f51.dsc.data_base + 50)
#define LPWG_TCI1_FAIL_BUFFER_REG	(d->f51.dsc.data_base + 51)
#define LPWG_TCI2_FAIL_COUNT_REG	(d->f51.dsc.data_base + 61)
#define LPWG_TCI2_FAIL_INDEX_REG	(d->f51.dsc.data_base + 62)
#define LPWG_TCI2_FAIL_BUFFER_REG	(d->f51.dsc.data_base + 63)

/* Real-Time LPWG Fail Reason Ctrl */
#define NUM_OF_EACH_FINGER_DATA		8
#define MAX_NUM_OF_FAIL_REASON		2
/* F51_CUSTOM_DATA31 (Multitap Fail Reason Real-Time Interrupt) */
#define LPWG_FAIL_REASON_REALTIME_INT	(d->f51.dsc.data_base + 74)
/* F51_CUSTOM_CTRL06.00 (MultiTap Fail Real-Time Interrupt Enable) */
#define LPWG_FAIL_INT_ENABLE_REG	(d->f51.dsc.control_base + 15)

/* Swipe Data Reg */
#define SWIPE_START_X_REG		(d->f51.dsc.data_base + 75)
#define SWIPE_START_Y_REG		(d->f51.dsc.data_base + 77)
#define SWIPE_END_X_REG			(d->f51.dsc.data_base + 79)
#define SWIPE_END_Y_REG			(d->f51.dsc.data_base + 81)
#define SWIPE_TIME_REG			(d->f51.dsc.data_base + 83)
#define SWIPE_DIRECTION_REG		(d->f51.dsc.data_base + 85)
#define SWIPE_FAIL_COUNT_REG		(d->f51.dsc.data_base + 86)
#define SWIPE_FAIL_INDEX_REG		(d->f51.dsc.data_base + 87)
#define SWIPE_FAIL_BUFFER_REG		(d->f51.dsc.data_base + 88)
#define SWIPE_FAIL_BUFFER_SIZE		(10)

/* Swipe Control Reg */
#define SWIPE_ENABLE_REG		(d->f12_reg.ctrl[76])

/* LPWG ABS Control Reg */
#define LPWG_ABS_ENABLE_REG		(d->f51.dsc.control_base + 46)
#define LPWG_ABS_ACTIVE_X1_REG		(d->f51.dsc.control_base + 47)
#define LPWG_ABS_ACTIVE_Y1_REG		(d->f51.dsc.control_base + 49)
#define LPWG_ABS_ACTIVE_X2_REG		(d->f51.dsc.control_base + 51)
#define LPWG_ABS_ACTIVE_Y2_REG		(d->f51.dsc.control_base + 53)

/* Grip Suppression Control Reg */
#define GRIP_SUPPRESSION_CTRL_REG	(d->f51.dsc.control_base + 20)

#define REPORT_TYPE_REG		(d->f54.dsc.data_base)
#define FIFO_INDEX_LSB_REG		(d->f54.dsc.data_base + 1)
#define FIFO_INDEX_MSB_REG		(d->f54.dsc.data_base + 2)
#define REPORT_DATA_REG		(d->f54.dsc.data_base + 3)
#define INTERFERENCE_METRIC_LSB_REG	(d->f54.dsc.data_base + 4)
#define INTERFERENCE_METRIC_MSB_REG	(d->f54.dsc.data_base + 5)
#define CURRENT_NOISE_STATUS_REG	(d->f54.dsc.data_base + 8)
#define CID_IM_REG			(d->f54.dsc.data_base + 10)
#define FREQ_SCAN_IM_REG		(d->f54.dsc.data_base + 11)

#define DYNAMIC_SENSING_CTRL_REG	(d->f54.dsc.control_base + 33)
#define ANALOG_CONTROL2		(d->f54.dsc.control_base + 36)
#define WAVEFORM_DURATION_CTRL_REG	(d->f54.dsc.control_base + 41)
#define TDDI_CBC_ENABLE_CTRL_REG	(d->f54.dsc.control_base + 44)
#define CBC_TIMING_CONTROL		(d->f54.dsc.control_base + 46)

#define ANALOG_COMMAND_REG		(d->f54.dsc.command_base)

/* Flash Memory Management */
#define FLASH_CONFIG_ID_REG		(d->f34.dsc.control_base)
#define FLASH_STATUS_REG		(d->f34.dsc.data_base)
#define PARTITION_ID_REG		(d->f34.dsc.data_base + 1)
#define BLOCK_OFFSET_REG		(d->f34.dsc.data_base + 2)
#define TRANSFER_LENGTH_REG		(d->f34.dsc.data_base + 3)
#define PROGRAMING_CMD_REG		(d->f34.dsc.data_base + 4)
#define PAYLOAD_REG			(d->f34.dsc.data_base + 5)
#define FLASH_PROPERTY_REG		(d->f34.dsc.query_base + 3)

/* Manufacturer ID */
#define MANUFACTURER_ID_REG		(d->f01.dsc.query_base)
/* CUSTOMER_FAMILY QUERY */
#define CUSTOMER_FAMILY_REG		(d->f01.dsc.query_base + 2)
/* FW revision */
#define FW_REVISION_REG			(d->f01.dsc.query_base + 3)
/* Product ID */
#define PRODUCT_ID_REG			(d->f01.dsc.query_base + 11)
#define DEVICE_COMMAND_REG		(d->f01.dsc.command_base)

#define F35_ERROR_CODE_OFFSET		0
#define F35_FLASH_STATUS_OFFSET		5
#define F35_CHUNK_NUM_LSB_OFFSET	0
#define F35_CHUNK_NUM_MSB_OFFSET	1
#define F35_CHUNK_DATA_OFFSET		2
#define F35_CHUNK_COMMAND_OFFSET	18

#define F35_CHUNK_SIZE			16
#define F35_ERASE_ALL_WAIT_MS		8000
#define F35_RESET_WAIT_MS		250

#define PDT_PROPS			0x00EF
#define PDT_START			0x00E9
#define PDT_END				0x00D0
#define PDT_ENTRY_SIZE			0x0006
#define MCU_DISPLAY_ID_REG		0x000C

enum{
	NOISE_DISABLE = 0,
	NOISE_ENABLE,
};

enum {
	ABS_STOP = 0,
	ABS_SENSING,
	LPWG_STOP,
	LPWG_SENSING,
	PARTIAL_LPWG_SENSING,
};

enum {
	FAIL_DISTANCE_INTER_TAP = 1,
	FAIL_DISTANCE_TOUCHSLOP,
	FAIL_TIMEOUT_INTER_TAP,
	FAIL_MULTI_FINGER,
	FAIL_DELAY_TIME,
	FAIL_PALM_STATE,
	FAIL_ACTIVE_AREA,
	FAIL_TAP_COUNT,
};

enum {
	S3706_SWIPE_U = 0,
	S3706_SWIPE_D = 1,
	S3706_SWIPE_L = 2,
	S3706_SWIPE_R = 3,
	S3706_SWIPE_NUM = 4,
};

enum {
	DS_TIANMA_PATH = 0,
	DS_TOVIS_PATH = 1,
};

enum {
	OLD_TIANMA_ID = 1,
	NEW_TIANMA_ID = 2,
	TOVIS_ID = 3,
};

#if 0
/* The enum value used for touch_notify_connect() must be verified by the Power BSP or USB BSP. */
enum {
	CONNECT_INVALID = 0,
	CONNECT_SDP,
	CONNECT_DCP,
	CONNECT_CDP,
	CONNECT_PROPRIETARY,
	CONNECT_FLOATED,
	CONNECT_HUB, /* SHOULD NOT change the value */
};
#endif

/* Charger Connection State */
/* The enum value used for writing value in touch IC. */
enum{
	CONNECT_NONE = 0,
	CONNECT_USB,
	CONNECT_TA,
	CONNECT_OTG,
};

/* IC Initialization State */
enum {
	IC_INIT_NEED = 0,
	IC_INIT_DONE,
};

/* IC Configured State */
enum {
	IC_CONFIGURED_NEED = 0,
	IC_CONFIGURED_DONE,
};

/* ESD Initialization State */
enum {
	ESD_RECOVERY_NEED = 0,
	ESD_RECOVERY_DONE,
};

/* Stylus Mode State*/
enum {
	STYLUS_OFF = 0,
	STYLUS_ON,
};

/* Panel Reset State*/
enum {
	PANEL_RESET_NEED = 0,
	PANEL_RESET_MAX = 3,
};

/* Sensing Test Bit State */
enum {
	SENSING_BIT_CLEAR = 0,
	SENSING_BIT_SET,
};

enum {
	SW_RESET = 0,
	HW_RESET,
	SW_RESET_NO_INIT,
	HW_RESET_NO_INIT,
};

enum {
	TCI_REPORT_NOT_SET = -1,
	TCI_REPORT_DISABLE,
	TCI_REPORT_ENABLE,
};

enum {
	LCD_MODE_U0 = 0,
	LCD_MODE_U2_UNBLANK,
	LCD_MODE_U2,
	LCD_MODE_U3,
	LCD_MODE_U3_PARTIAL,
	LCD_MODE_U3_QUICKCOVER,
	LCD_MODE_STOP,
};

struct s3706_f12_reg {
	u8 *ctrl;
	u8 *data;
};

struct s3706_version {
	u8 build : 4;
	u8 major : 4;
	u8 minor;
};

struct s3706_touch_data {
	u8 type;
	u8 x_lsb;
	u8 x_msb;
	u8 y_lsb;
	u8 y_msb;
	u8 z;
	u8 wx;
	u8 wy;
} __packed;

struct s3706_touch_angle_data {
	u8 area;
	s8 angle;
} __packed;

struct s3706_touch_width_data {
	u16 major;
	u16 minor;
} __packed;

struct s3706_touch_info {
	u8 device_status;
	u8 irq_status;
	u8 touch_cnt:4;
	struct s3706_touch_data data[10];
	struct s3706_touch_angle_data angle_data[10];
	struct s3706_touch_width_data width_data[10];
} __packed;

struct s3706_ic_info {
	struct s3706_version version;
	u8 raws[4];
	u8 product_id[10];
	struct s3706_version img_version;
	u8 img_raws[4];
	u8 img_product_id[10];
	u8 revision;
	u8 family;
	u32 fw_ver_addr;
	u32 fw_pid_addr;
	u8 bootloader_type;
};

struct s3706_prd_info {
	u8 product_id[6];
	u8 chip_ver;
	u8 fpc_ver;
	u8 sensor_ver;
	u8 inspect_channel;
	u8 inspect_date[3];
	u8 inspect_time[3];
} __packed;

struct s3706_noise_ctrl {
	u8 noise_log;
	u8 check_noise;
	u8 cnt;
	u8 cns_avg;
	u8 im_avg;
	u8 cid_im_avg;
	u8 freq_scan_im_avg;
	unsigned long im_sum;
	unsigned long cns_sum;
	unsigned long cid_im_sum;
	unsigned long freq_scan_im_sum;
};

struct function_descriptor {
	u8 query_base;
	u8 command_base;
	u8 control_base;
	u8 data_base;
	u8 int_source_count;
	u8 fn_number;
};

struct s3706_function {
	struct function_descriptor dsc;
	u8 page;
};

struct s3706_rmidev_exp_fn {
	int (*init)(struct device *dev);
	void (*remove)(struct device *dev);
	void (*reset)(struct device *dev);
	void (*reinit)(struct device *dev);
	void (*early_suspend)(struct device *dev);
	void (*suspend)(struct device *dev);
	void (*resume)(struct device *dev);
	void (*late_resume)(struct device *dev);
	void (*attn)(struct device *dev,
			unsigned char intr_mask);
};

struct s3706_rmidev_exp_fhandler {
	struct s3706_rmidev_exp_fn *exp_fn;
	bool insert;
	bool initialized;
	bool remove;
};

struct s3706_fwu_exp_fn {
	int (*init)(struct device *dev);
	void (*remove)(struct device *dev);
	void (*reset)(struct device *dev);
	void (*reinit)(struct device *dev);
	void (*early_suspend)(struct device *dev);
	void (*suspend)(struct device *dev);
	void (*resume)(struct device *dev);
	void (*late_resume)(struct device *dev);
	void (*attn)(struct device *dev, unsigned char intr_mask);
};

struct s3706_fwu_exp_fhandler {
	struct s3706_fwu_exp_fn *exp_fn;
	bool insert;
	bool initialized;
	bool remove;
};


struct s3706_state_info {
	atomic_t charger;
	atomic_t init;
	atomic_t config;
	atomic_t scan_pdt;
	atomic_t esd_recovery;
	atomic_t power;
};

struct s3706_active_area {
	s16 x1;
	s16 y1;
	s16 x2;
	s16 y2;
} __packed;

struct s3706_lpwg_abs_ctrl {
	bool enable;
	struct s3706_active_area area;
	struct s3706_active_area border_area;
	s16 offset_y;
};

struct s3706_lpwg_abs_buf {
	u8 enable;
	struct s3706_active_area area;
} __packed;

struct synaptics_blank {
	int prev;
	int curr;
};

struct s3706_data {
	struct device *dev;

	struct s3706_function f01;
	struct s3706_function f12;
	struct s3706_function f34;
	struct s3706_function f35;
	struct s3706_function f51;
	struct s3706_function f54;
	struct s3706_function f55;
	struct s3706_function fdc;
	u8 s3706_function_bits;		/* Needs to changed from u8 to u16 if more then 8 functions*/
	struct s3706_f12_reg f12_reg;
	struct s3706_touch_info info;
	struct s3706_ic_info ic_info;
	struct s3706_prd_info prd_info;
	struct s3706_noise_ctrl noise;
	struct synaptics_blank blank_status;

	u8 curr_page;
	u8 object_report;
	u8 max_num_of_fingers;
	u8 lpwg_fail_reason;
	struct delayed_work fb_notify_work;
	struct pm_qos_request pm_qos_req;
	struct s3706_state_info state;

	u8 lcd_mode;
	u8 prev_lcd_mode;
	bool is_palm;

	struct s3706_lpwg_abs_ctrl lpwg_abs;
	struct s3706_lpwg_abs_buf lpwg_abs_buf;
};

bool s3706_is_product(struct s3706_data *d,
					const char *product_id, size_t len);
bool s3706_is_img_product(struct s3706_data *d,
					const char *product_id, size_t len);
int s3706_set_page(struct device *dev, u8 page);
//int s3706_init(struct device *dev); //module_initialize
//int s3706_ic_info(struct device *dev); //module_ic_info
int s3706_read(struct device *dev, u8 addr, void *data, int size);
int s3706_write(struct device *dev, u8 addr, void *data, int size);
int s3706_force_update(struct device *dev);
int s3706_force_calibration(struct device *dev);
void s3706_reset_ctrl(struct device *dev, int ctrl);
void s3706_rmidev_function(struct s3706_rmidev_exp_fn *exp_fn, bool insert);
void s3706_fwu_function(struct s3706_fwu_exp_fn *fwu_fn, bool insert);
int s3706_tci_report_enable(struct device *dev, bool enable);
irqreturn_t touch_irq_handler(int irq, void *dev_id);
irqreturn_t touch_irq_thread(int irq, void *dev_id);
void s3706_delayed_work(struct workqueue_struct *wq, struct delayed_work *dwork, unsigned long delay);

/* extern function */
extern int synaptics_fw_updater(const unsigned char *fw_data);
extern bool is_ddic_name(char *ddic_name);
extern bool lge_get_factory_boot(void);
extern int rmi4_fw_update_module_init(void);
extern void rmi4_fw_update_module_exit(void);
extern int rmidev_module_init(void);
extern void rmidev_module_exit(void);

static inline struct s3706_data *to_s3706_data(struct device *dev)
{
	return (struct s3706_data *)module_get_device(to_module(dev));
}

void module_interrupt_control(struct device *dev, int on_off);

#endif /* LGE_TOUCH_SYNAPTICS_H */
