/* touch_sw42000.h
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

#ifndef LGE_TOUCH_SW42000_H
#define LGE_TOUCH_SW42000_H

#include <linux/pm_qos.h>

#define __SUPPORT_CLK_CTRL
//#define __SUPPORT_NOTIFY_LCD_EVENT_REG
#define __SUPPORT_ABT
//#define __SUPPORT_NOTIFY_CALL
//#define __SUPPORT_LONGPRESS

/* report packet */
struct sw42000_touch_data {
	u32 track_id:5;
	u32 tool_type:3;
	u32 angle:8;
	u32 event:2;
	u32 x:14;
	u32 y:14;
	u32 pressure:8;
	u32 reserve1:10;
	u32 reserve2:4;
	u32 width_major:14;
	u32 width_minor:14;
} __packed;

struct sw42000_touch_info {
	u32 ic_status;
	u32 tc_status;
	u32 wakeup_type:8;
	u32 touch_cnt:5;
	u32 button_cnt:3;
	u32 self_recal:2;
	u32 mutual_recal:2;
	u32 abnormal1_recal:2;
	u32 abnormal2_recal:2;
	u32 current_mode:1;
	u32 rmdiff:7;
	struct sw42000_touch_data data[10];
	/* debug info */
} __packed;

#define TYPE_SW42000 9

#define PALM_ID				15

/* ic info offset*/
#define tc_version			(0)
//#define tc_product_code		(0x1)
#define tc_product_id1			(0x2)
//#define tc_product_id2		(0x3)
//#define tc_conf_version		(0x4)

#define pt_info_channel			(0x5)
#define pt_info_lot_um			(0x7)
#define pt_info_fpc_type		(0x8)
#define pt_info_pt_date			(0x9)
#define pt_info_pt_time			(0xa)

/* HW resister */
#define INFO_PTR_ADDR			(0x01B)

#define SERIAL_SPI_EN			(0xFE4)
#define SERIAL_I2C_EN			(0xFE5)
#define SPI_TATTN_OPT			(0xFF3)
#define SYS_GPIO_FUNC			(0x008)
#define SYS_LDO_CTL			(0x006)

#define SPR_CHIP_ID			(0)
#define TC_CMD				(0xC00)
//#define RSVD1				(0xC10)
#define TEST_CMD			(0xC20)
#define ABT_CMD				(0xF20)
//#define RSVD2				(0xC40)
#define CFG_S_SRAM_OFT			(0xCC0)
#define TC_IC_STATUS			(0x600)
#define TC_STATUS			(0x601)
#define ABT_REPORT			(0x602)
//#define RSVD3				(0)
#define CHIP_INFO			(0x642)
#define REG_INFO			(0x64A)
#define PT_INFO				(0x656)
#define TC_STS				(0x666)
#define ABT_STS				(0)
#define TUNE_CODE			(0x6A6)
#define DBG_BUF1_SRAM_OFT		(0xE39)
#define DBG_BUF2_SRAM_OFT		(0x10B4)
#define ABT_BUF_SRAM_OFT		(0x136E)
#define R_ADDR				(0x7DC)

#define DEBUG_INFO			(0x61F)

/* device control offset */
#define tc_device_ctl			(0x0)
#define tc_interrupt_ctl		(0x1)
#define tc_interrupt_status		(0x2)
#define tc_driving_ctl			(0x3)
//#define tc_flash_dn_ctl			(0x5)

#define command_start_addr		(0xC00)
#define command_end_addr		(0xEFF)

//#define tc_rtc_te_interval_cnt  (56)
//#define config_s_info1			(0x480)
//#define config_s_info2			(0x481)

/* F/W Upgrade */
#define FLASH_PAGE_OFFSET		(0x200)
#define FLASH_PAGE_SIZE			(2<<10)
#define FLASH_MAX_RW_SIZE		(1<<10)

#define GDMA_SADDR				0x056
#define GDMA_CTL				0x058
#define GDMA_CTL_READONLY_EN	BIT(17)
#define GDMA_CTL_GDMA_EN		BIT(26)
#define GDMA_START				0x059
#define GDMA_CRC_RESULT			0x05D
#define GDMA_CRC_PASS			0x05E

#define FC_CTRL					0x06B
#define FC_STAT					0x06C
#define FC_ADDR					0x06D
#define SPI_FLASH_STATUS		0xFE2

#define SYS_OSC_CTL				(0x005)
#define AVERAGE_TRIM_VAL		(0x3C)

#define __FC_CTRL_PAGE_ERASE	BIT(0)
#define __FC_CTRL_MASS_ERASE	BIT(1)
#define __FC_CTRL_WR_EN			BIT(2)

#define FC_CTRL_PAGE_ERASE		(__FC_CTRL_PAGE_ERASE | __FC_CTRL_WR_EN)
#define FC_CTRL_MASS_ERASE		(__FC_CTRL_MASS_ERASE | __FC_CTRL_WR_EN)
#define FC_CTRL_WR_EN			(__FC_CTRL_WR_EN)

#define FC_ERASE_WAIT_CNT		200
#define FC_ERASE_WAIT_TIME		5
/* F/W Upgrade END */

#define FAIL_REASON_MAX_CNT 6

/*
 * Engine control resister Offset
 * base:r_abt_cmd_addr(0xF20)
 */
/* TCI 1(Double Tap), 2(Knock Code) */
#define TCI_ENABLE					(0x10)
#define TCI_TOTAL_TAP_COUNT			(0x11)
#define TCI_INTER_TAP_TIME_MIN		(0x12)
#define TCI_INTER_TAP_TIME_MAX		(0x13)
#define TCI_INNER_TAP_DIST_MAX		(0x14)
#define TCI_INTER_TAP_DISP_MAX		(0x15)
#define TCI_INTERRUPT_DELAY_TIME	(0x16)
#define TCI_ACTIVE_AREA_X1			(0x17)
#define TCI_ACTIVE_AREA_Y1			(0x18)
#define TCI_ACTIVE_AREA_X2			(0x19)
#define TCI_ACTIVE_AREA_Y2			(0x1A)
#define ACT_SENSELESS_AREA_W		(0x57)	// 87 pixel == 5mm

/* Swipe */
#define SWIPE_ON			(0x1B)
#define SWIPE_DIST_THRESHOLD		(0x1C)
#define SWIPE_RATIO_THRESHOLD		(0x1D)
#define SWIPE_TIME_MIN_H		(0x1E)
#define SWIPE_TIME_MIN_V		(0x1F)
#define SWIPE_TIME_MAX_H		(0x20)
#define SWIPE_TIME_MAX_V		(0x21)
#define SWIPE_ACT_AREA_X1_H		(0x22)
#define SWIPE_ACT_AREA_Y1_H		(0x23)
#define SWIPE_ACT_AREA_X2_H		(0x24)
#define SWIPE_ACT_AREA_Y2_H		(0x25)
#define SWIPE_ACT_AREA_X1_V		(0x26)
#define SWIPE_ACT_AREA_Y1_V		(0x27)
#define SWIPE_ACT_AREA_X2_V		(0x28)
#define SWIPE_ACT_AREA_Y2_V		(0x29)
#define SWIPE_START_AREA_X1_H		(0x2A)
#define SWIPE_START_AREA_Y1_H		(0x2B)
#define SWIPE_START_AREA_X2_H		(0x2C)
#define SWIPE_START_AREA_Y2_H		(0x2D)
#define SWIPE_START_AREA_X1_V		(0x2E)
#define SWIPE_START_AREA_Y1_V		(0x2F)
#define SWIPE_START_AREA_X2_V		(0x30)
#define SWIPE_START_AREA_Y2_V		(0x31)
#define SWIPE_WRONG_DIRECTION_THD	(0x32)
#define SWIPE_INIT_RATIO_CHK_DIST	(0x33)
#define SWIPE_INIT_RATIO_THD		(0x34)

/* Swipe2 */
#define SWIPE2_ON			(0x60)
#define SWIPE2_DIST_THRESHOLD		(0x61)
#define SWIPE2_RATIO_THRESHOLD		(0x62)
#define SWIPE2_TIME_MIN_H		(0x63)
#define SWIPE2_TIME_MAX_H		(0x64)
#define SWIPE2_ACT_AREA_X1_H		(0x65)
#define SWIPE2_ACT_AREA_Y1_H		(0x66)
#define SWIPE2_ACT_AREA_X2_H		(0x67)
#define SWIPE2_ACT_AREA_Y2_H		(0x68)
#define SWIPE2_START_AREA_X1_H		(0x69)
#define SWIPE2_START_AREA_Y1_H		(0x6A)
#define SWIPE2_START_AREA_X2_H		(0x6B)
#define SWIPE2_START_AREA_Y2_H		(0x6C)
#define SWIPE2_WRONG_DIRECTION_THD	(0x6D)
#define SWIPE2_INIT_RATIO_CHK_DIST	(0x6E)
#define SWIPE2_INIT_RATIO_THD		(0x6F)

/* ABS */
#define LPWG_ABS_ENABLE			(0x35)
#define LPWG_ABS_ACTIE_AREA_START	(0x36)
#define LPWG_ABS_ACTIE_AREA_END		(0x37)

/* LongPress */
#define LONG_PRESS_ENABLE		(0x38)
#define LONG_PRESS_ACT_AREA_START	(0x39)
#define LONG_PRESS_ACT_AREA_END		(0x3A)
#define LONG_PRESS_SLOPE		(0x3B)
#define LONG_PRESS_TIME			(0x3C)

/* Fail Reason (TCI, Swipe) */
#define LPWG_FAILREASON_ON			(0x3D)
#define LPWG_FAILREASON_STS			(0x3E)
#define TCI0_FAILREASON_BUF			(0x3F)
#define TCI1_FAILREASON_BUF			(0x41)
#define SWIPE_FAILREASON_BUF		(0x43)
#define LONGPRESS_FAILREASON_BUF	(0x45)
/* QuickCover */
#define COVER_SENSITIVITY			(0x47)
#define COVER_ACTIVE_AREA_START_XY	(0x48)
#define COVER_ACTIVE_AREA_END_XY	(0x49)
/* SPECIAL Resister */
#define SPECIAL_CHARGER_INFO		(0x4A)
#define GLOVE_TOUCH_CTRL			(0x4B)
#define GRAB_TOUCH_CTRL				(0xF6C)

#define SPECIAL_IME_STATUS			(0x4D)
#define SPECIAL_CALL_INFO			(0x4E)
#define SPECIAL_SENSITIVE_INFO		(0x4F)



#if 1
#define R_HEADER_SIZE_SPI			(6)
#define W_HEADER_SIZE_SPI			(6)
#else
#define R_HEADER_SIZE_SPI			(2)
#define W_HEADER_SIZE_SPI			(4)
#endif

#define R_HEADER_SIZE_I2C			(0)
#define W_HEADER_SIZE_I2C			(2)

/* SPR control */
//#define SPI_RST_CTL			0xFE0
//#define SPI_CLK_CTL			0xFE1
#define SPI_OSC_CTL				(0xFE1) // 0xFE2


#define CONNECT_NONE		(0x00)
#define CONNECT_USB		(0x01)
#define CONNECT_WIRELESS	(0x10)

#define CFG_MAGIC_CODE		0xCACACACA
#define CFG_CHIP_ID			42000

#define CHIP_POW_C_CONF		10
#define	CHIP_POW_S_CONF		10

#define CHIP_NUM_C_CONF		0
#define CHIP_MIN_S_CONF		1
#define CHIP_MAX_S_CONF		10

#define CFG_C_SIZE		(CHIP_NUM_C_CONF<<CHIP_POW_C_CONF)
#define CFG_S_SIZE		(1<<10)

#define CRC_FIXED_VALUE			0x800D800D

/* Interrupt Status for check_status func */
/* (1<<1)|(1<<2) */
#define IC_STATUS_NORMAL_MASK		0x00000006

#define IC_STATUS_GLOBAL_RESET_BIT	0x00000000	/* NULL */
/* (1<<3) | (1<<5) */
#define IC_STATUS_HW_RESET_BIT		0x00000028
#define IC_STATUS_SW_RESET_BIT		0x00000000	/* NULL */
/* (1<<1) */
#define IC_STATUS_FW_UPGRADE_BIT	0x00000002
/* (1<<2) */
#define IC_STATUS_LOGGING_BIT		0x00000004

/* (1<<5)|(1<<6)|(1<<7)|(1<<15)|(1<<20)|(1<<22) */
#define TC_STATUS_NORMAL_MASK		0x005080E0

#define TC_STATUS_GLOBAL_RESET_BIT	0x00000000	/* NULL */
/* (1<<6)|(1<<7)|(1<<9)|(1<<10) */
#define TC_STATUS_HW_RESET_BIT		0x000006C0
#define TC_STATUS_SW_RESET_BIT		0x00000000	/* NULL */
#define TC_STATUS_FW_UPGRADE_BIT	0x00000000	/* NULL */
/* (1<<5)|(1<<13)|(1<<15)|(1<<20)|(1<<22)|(1<<28) */
#define TC_STATUS_LOGGING_BIT		0x1050A020
//#define TC_STATUS_MCU_FAULT		0x60000000	/* (1<<30)| (1<<29) */

#define STATUS_NORMAL_MASK			(((u64)IC_STATUS_NORMAL_MASK << 32) | (u64)TC_STATUS_NORMAL_MASK)
#define STATUS_GLOBAL_RESET_BIT		(((u64)IC_STATUS_GLOBAL_RESET_BIT << 32) | (u64)TC_STATUS_GLOBAL_RESET_BIT)
#define STATUS_HW_RESET_BIT			(((u64)IC_STATUS_HW_RESET_BIT << 32) | (u64)TC_STATUS_HW_RESET_BIT)
#define STATUS_SW_RESET_BIT			(((u64)IC_STATUS_SW_RESET_BIT << 32) | (u64)TC_STATUS_SW_RESET_BIT)
#define STATUS_FW_UPGRADE_BIT		(((u64)IC_STATUS_FW_UPGRADE_BIT << 32) | (u64)TC_STATUS_FW_UPGRADE_BIT)
#define STATUS_LOGGING_BIT			(((u64)IC_STATUS_LOGGING_BIT << 32) | (u64)TC_STATUS_LOGGING_BIT)

#define GRAB_TOUCH_CTRL_BIT		0x00000001
#define GRAB_NOTI_CTRL_BIT		0x00010000


/* ic bus test */
#define BUS_TEST_REG			(0x01C)
#define IC_TEST_ADDR_NOT_VALID 0x8000

/* interrupt type */
#define INTR_TYPE_BOOT_UP_DONE		0x01
#define INTR_TYPE_INIT_COMPLETE		0x02
#define INTR_TYPE_ABNORMAL_ERROR_REPORT	0x03
#define INTR_TYPE_DEBUG_REPORT		0x04
#define INTR_TYPE_REPORT_PACKET		0x05

enum{
	NOISE_DISABLE = 0,
	NOISE_ENABLE,
};

enum {
	FUNC_OFF = 0,
	FUNC_ON,
};

enum {
	NONE = 0,
	R_POWER_CTL,
	R_RESET_CTL,
};

enum {
	U3_MODE_2_ONLY = 0,
	U3_MODE_1_2_SWING,
	U3_MODE_2_ONLY_SYNC,
	U3_MODE_1_2_SWING_SYNC,
};

enum {
	SW_RESET = 0,
	HW_RESET_ASYNC,
	HW_RESET_SYNC,
	SW_RESET_CODE_DUMP,
	HW_RESET_POWER,
};

enum {
	TOUCHSTS_IDLE = 0,
	TOUCHSTS_DOWN,
	TOUCHSTS_MOVE,
	TOUCHSTS_UP,
};

enum {
	ABS_MODE	= 0,
	KNOCK_ON	= 1,
	KNOCK_CODE	= 2,
	SWIPE_LEFT	= 3,
	SWIPE_RIGHT	= 4,
	SWIPE_UP	= 5,
	SWIPE_DOWN	= 6,
	LONG_PRESS	= 7,
	SWIPE_LEFT2	= 8,
	SWIPE_RIGHT2	= 9,
	CUSTOM_DEBUG	= 200,
	KNOCK_OVERTAP	= 201,
};

enum {
	LCD_MODE_U0 = 0,
	LCD_MODE_U2_UNBLANK,
	LCD_MODE_U2,
	LCD_MODE_U3,
	LCD_MODE_U3_PARTIAL,
	LCD_MODE_U3_QUICKCOVER,
	LCD_MODE_STOP,
	LCD_MODE_NUM,
};

enum {
	SW42000_SWIPE_L = 0,
	SW42000_SWIPE_R = 1,
	SW42000_SWIPE_U = 2,
	SW42000_SWIPE_D = 3,
	SW42000_SWIPE_NUM = 4,
};

enum {
	SW42000_SWIPE2_L = 0,
	SW42000_SWIPE2_R = 1,
	SW42000_SWIPE2_NUM = 2,
};

enum {
	LPWG_FAILREASON_DISABLE = 0,
	LPWG_FAILREASON_ENABLE,
};

enum {
	LPWG_DEBUG_TCI_1 = 0,
	LPWG_DEBUG_TCI_2 = 1,
	LPWG_DEBUG_SWIPE = 2,
	LPWG_DEBUG_LONGPRESS = 3,
	LPWG_DEBUG_NUM = 4,
};

enum {
	IC_INIT_NEED = 0,
	IC_INIT_DONE,
};

enum {
	LOG_WRITE_DONE = 0,
	DO_WRITE_LOG,
};

#define FLASH_TPROG				0x65
#define serial_data_offset		(0x03C)
#define MAX_FLASH_SIZE			(137 * 1024)

#define BDMA_TRANS_SIZE			(0x1000)
#define DATASRAM_ADDR			(0x20000000)

typedef union {
	struct {
		unsigned    common_cfg_size : 16;
		unsigned    specific_cfg_size : 16;
	} b;
	uint32_t w;
} t_cfg_size;

typedef union {
	struct {
		unsigned    chip_rev : 8;
		unsigned    model_id : 8;
		unsigned    reserved : 16;
	} b;
	uint32_t w;
} t_cfg_specific_info1;

typedef union {
	struct {
		unsigned    lcm_id : 8;
		unsigned    fpcb_id : 8;
		unsigned    lot_id : 8;
		unsigned    reserved : 8;
	} b;
	uint32_t w;
} t_cfg_specific_info2;

typedef struct {
	uint32_t                        cfg_magic_code;
	uint32_t                        cfg_chip_id;
	uint32_t                        cfg_specific_index;
	t_cfg_size                      cfg_size;
	t_cfg_specific_info1            cfg_specific_info1;
	t_cfg_specific_info2            cfg_specific_info2;
	uint32_t                        cfg_header_reserved1;
	uint32_t                        cfg_header_reserved2;
} CFG_S_HEADER_CONTROL_TypeDef;

#define BDMA_SADDR						0x72
#define BDMA_DADDR						0x73
#define BDMA_CTL						0x74
#define BDMA_START						0x75
#define BDMA_STS						0x77

#define BDMA_CTL_BDMA_EN				(0x00010000)
#define BDMA_CTL_BDMA_BST_EN			(0x001E0000)
#define BDMA_STS_TR_BUSY				(0x00000040)

#define BDMA_SADDR						0x72
#define BDMA_DADDR						0x73
#define BDMA_CTL						0x74
#define BDMA_START						0x75
#define BDMA_STS						0x77

#define BDMA_CTL_BDMA_EN				(0x00010000)
#define BDMA_CTL_BDMA_BST_EN			(0x001E0000)
#define BDMA_STS_TR_BUSY				(0x00000040)





/* SPR control */

/* Firmware control */
#define spr_rst_ctl				(0x004)	//(0x006)
#define SYS_RST_CTL				spr_rst_ctl

#define spr_boot_ctl			(0x00F)
#define fw_boot_code_addr		(0x01A)	//(0x044)
#define spr_sram_ctl			(0x010)
#define spr_code_offset			(0x036)	//(0x07D)
#define spr_data_offset			(0x082)
#define rst_cnt_addr			(0xFF5)
#define tc_flash_dn_sts			(0)

#define code_access_addr		(0xFD8)	//(0xFD0)
#define data_access_addr		(0xFD1)

#define MAX_RW_SIZE				(60 * 1024)
#define FLASH_SIZE				(128 * 1024)
#define FLASH_CONF_SIZE			(1 * 1024)

#define FLASH_KEY_CODE_CMD		0xDFC1
#define FLASH_KEY_CONF_CMD		0xE87B
#define FLASH_BOOTCHK_VALUE		0x0A0A0000
#define FW_BOOT_LOADER_INIT		0x74696E69
#define FW_BOOT_LOADER_CODE		0x544F4F42
#define FLASH_CODE_DNCHK_VALUE	0x42
#define FLASH_CONF_DNCHK_VALUE	0x8C

enum {
	E_FW_CODE_SIZE_ERR = 1,
	E_FW_CODE_ONLY_VALID = 2,
	E_FW_CODE_AND_CFG_VALID = 3,
	E_FW_CODE_CFG_ERR = 4
};

union cfg_size {
	struct {
		u32 common_size:16;
		u32 specific_size:16;
	} b;
	u32 w;
};

union s_cfg_info_1 {
	struct {
		u32 chip_rev:8;
		u32 model_id:8;
		u32 lcm_id:8;
		u32 fpc_id:8;
	} b;
	u32 w;
};

union s_cfg_info_2 {
	struct {
		u32 lot_id:8;
		u32 rsvd:24;
	} b;
	u32 w;
};

struct s_cfg_head {
	union s_cfg_info_1	info_1;
	union s_cfg_info_2	info_2;
	u32 version;
	u32 model_name;
};

struct cfg_head {
	u32 magic_code;
	u32 chip_id;
	u32 s_index;
	union cfg_size c_size;
	struct s_cfg_head s_cfg_head;
	u32 rsvd1;
	u32 rsvd2;
	u32 rsvd3;
	u32 rsvd4;
};

struct sw42000_version_bin {
	u8 build : 4;
	u8 major : 4;
	u8 minor;
};

struct sw42000_version {
	u8 minor;
	u8 major:4;
	u8 build:4;
	u8 chip_id;
	u8 protocol_ver:4;
	u8 reverved:4;
};

struct sw42000_pt_info {
	u8 channel;
	u32 reserved_1:24;
	u8 chip_rev;
	u32 reserved_2:24;
	u8 sensor_ver;
	u32 reserved_3:24;
	u8 fpc_ver;
	u32 reserved_4:24;
	u8 pt_date_year;
	u8 pt_date_month;
	u8 pt_date_day;
	u8 reserved_5;
	u8 pt_time_hour;
	u8 pt_time_min;
	u8 pt_time_sec;
	u8 reserved_6;
} __packed;

struct sw42000_ic_info {
	struct sw42000_version version;
	u8 product_id[7];
	struct sw42000_pt_info pt_info;
};

struct sw42000_chip_info {
	u32 r_version;
	u32 r_product_code;
	u32 r_product_id1;
	u32 r_product_id2;
	u32 r_conf_dn_index;
	u32 r_conf_version;
};

struct project_param {
	u8 wa_trim_wr;
	u8 wa_suspend_reset_ctl;
	u8 dfc_resume_power_ctl;
	u8 sys_driving_ctrl;
	u8 sys_tc_stop_delay;
	u8 used_mode;
	u8 reg_debug;
	u8 dynamic_reg;
	u32 flash_fw_size;
	u8 dfc_bus_test;
	u8 esd_recovery_when_rw;
	u8 tcl_off_via_mipi;
	u8 touch_power_control_en;
	u8 tci_setting;
	u8 need_serial_control;
};

struct sw42000_touch_debug_info {
	u32 info[3];
	u32 type:24;
	u32 length:8;
} __packed;

struct sw42000_fb_blank {
	int prev;
	int curr;
};

struct sw42000_active_area {
	s16 x1;
	s16 y1;
	s16 x2;
	s16 y2;
} __packed;

struct lpwg_abs_buf {
	u32 enable;
	u16 start_x;
	u16 start_y;
	u16 end_x;
	u16 end_y;
} __packed;

struct sw42000_lpwg_abs_ctrl {
	bool enable;
	struct sw42000_active_area area;
	struct sw42000_active_area border;
	s16 offset_y;
};
struct sw42000_lpwg_longpress_ctrl {
	bool enable;
	struct sw42000_active_area area;
	struct sw42000_active_area border;
	int slop;
	int press_time;
};

struct sw42000_ai_pick_ctrl {
	bool enable;
	struct sw42000_active_area area;
	struct sw42000_active_area border_area;
	struct sw42000_active_area total_area;
};

struct sw42000_data {
	struct device *dev;
	struct kobject kobj;

	struct project_param p_param;

	struct sw42000_touch_info info;
	struct sw42000_touch_debug_info debug_info;
	struct sw42000_ic_info ic_info;
	struct sw42000_chip_info chip_info;
//	struct sw42000_reg_info reg_info;
	struct workqueue_struct *wq_log;
	struct sw42000_fb_blank fb_blank;

	struct sw42000_lpwg_abs_ctrl lpwg_abs;
	struct sw42000_ai_pick_ctrl ai_pick;
	struct sw42000_lpwg_longpress_ctrl lpwg_longpress;

	void *prd;

	u8 noise_log;
	u8 lcd_mode;
	u8 prev_lcd_mode;
	u8 driving_mode;
	u8 u3fake;

	struct mutex io_lock;
	struct delayed_work fb_notify_work;
	u32 charger;
	u32 earjack;
	u32 frame_cnt;
	u8 intr_type;
	u8 tci_debug_type;
	atomic_t block_watch_cfg;
	atomic_t init;
	struct pm_qos_request pm_qos_req;
	u32 q_sensitivity;
	char te_test_log[64];
	int te_ret;
	u8 te_write_log;
	u8 lpwg_failreason_ctrl;
	u32 lpwg_failreason_data;

	u8 err_cnt;
	u8 boot_err_cnt;
};

#define TCI_MAX_NUM				2
#define SWIPE_MAX_NUM				2
#define TCI_DEBUG_MAX_NUM			16
#define SWIPE_DEBUG_MAX_NUM			8
#define DISTANCE_INTER_TAP			(0x1 << 1) /* 2 */
#define DISTANCE_TOUCHSLOP			(0x1 << 2) /* 4 */
#define TIMEOUT_INTER_TAP_LONG			(0x1 << 3) /* 8 */
#define MULTI_FINGER				(0x1 << 4) /* 16 */
#define DELAY_TIME				(0x1 << 5) /* 32 */
#define TIMEOUT_INTER_TAP_SHORT			(0x1 << 6) /* 64 */
#define PALM_STATE				(0x1 << 7) /* 128 */
#define TAP_TIMEOVER				(0x1 << 8) /* 256 */
#define TCI_DEBUG_ALL (DISTANCE_INTER_TAP | DISTANCE_TOUCHSLOP |\
		TIMEOUT_INTER_TAP_LONG | MULTI_FINGER | DELAY_TIME |\
		TIMEOUT_INTER_TAP_SHORT | PALM_STATE | TAP_TIMEOVER)

static inline struct sw42000_data *to_sw42000_data(struct device *dev)
{
	return (struct sw42000_data *)touch_get_device(to_touch_core(dev));
}

static inline struct sw42000_data *to_sw42000_data_from_kobj(
		struct kobject *kobj)
{
	return (struct sw42000_data *)container_of(kobj,
			struct sw42000_data, kobj);
}

extern int sw42000_reg_read(struct device *dev, u16 addr, void *data, int size);
extern int sw42000_reg_write(struct device *dev, u16 addr, void *data, int size);
extern int sw42000_ic_info(struct device *dev);
extern int sw42000_te_info(struct device *dev, char *buf);
extern int sw42000_tc_driving(struct device *dev, int mode);
extern int sw42000_irq_abs(struct device *dev);
extern int sw42000_irq_abs_data(struct device *dev);
extern int sw42000_irq_lpwg(struct device *dev);
extern int sw42000_irq_handler(struct device *dev);
extern int sw42000_check_status(struct device *dev);
extern int sw42000_debug_info(struct device *dev);
extern int sw42000_sleep_ctrl(struct device *dev, int new_status);
/*
extern int sw42000_chip_info_load(struct device *dev);
*/
extern int sw42000_reset_ctrl(struct device *dev, int ctrl);
extern bool is_ddic_name(char *ddic_name);
#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY_ESD) || IS_ENABLED(CONFIG_LGE_TOUCH_PANEL_GLOBAL_RESET)
extern void lge_mdss_report_panel_dead(void);
#endif
#endif
#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
extern void mtkfb_esd_recovery(void);
#endif

void sw42000_irq_runtime_engine_debug(struct device *dev);

static inline int sw42000_read_value(struct device *dev,
		u16 addr, u32 *value)
{
	return sw42000_reg_read(dev, addr, value, sizeof(*value));
}

static inline int sw42000_write_value(struct device *dev,
		u16 addr, u32 value)
{
	return sw42000_reg_write(dev, addr, &value, sizeof(value));
}

#endif /* LGE_TOUCH_SW42000_H */

