/* touch_ftm4.h
 *
 * Copyright (C) 2017 LGE.
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

#ifndef LGE_TOUCH_FTM4_H
#define LGE_TOUCH_FTM4_H

#include <linux/pm_qos.h>

#define FTS_MAX_FW_PATH			256

#define FTS_TS_DRV_NAME			"fts_touch"

#define FTS_ID0				0x36
#define FTS_ID1				0x70

#define FTS_FIFO_ADDR			0x85

#define FTS_FIFO_MAX			32
#define FTS_EVENT_SIZE			8
#define FTS_FW_UPDATE_RETRY		3

#define FTS_LOCKDOWNCODE_SIZE		13

#define INT_ENABLE			0x48
#define INT_DISABLE			0x08

#define READ_STATUS			0x84
#define READ_ONE_EVENT			0x85
#define READ_ALL_EVENT			0x86

#define FTS_RETRY_COUNT			10

#define WRITE_CHUNK_SIZE		32
#define FLASH_CHUNK			(64 * 1024)
#define DMA_CHUNK			32

#define FW_HEADER_SIZE			64
#define FW_HEADER_FTB_SIGNATURE		0xAA55AA55
#define FW_FTB_VER			0x00000001
#define FW_BYTES_ALLIGN			4
#define FW_BIN_VER_OFFSET		16
#define FW_BIN_CONFIG_VER_OFFSET	20

/* Command for flash */
#define FLASH_CMD_UNLOCK		0xF7
#define FLASH_CMD_WRITE_64K		0xF8
#define FLASH_CMD_READ_REGISTER		0xF9
#define FLASH_CMD_WRITE_REGISTER	0xFA

/* Parameters for commands */
#define ADDR_WARM_BOOT			0x001E
#define WARM_BOOT_VALUE			0x38
#define FLASH_ADDR_CODE			0x00000000
#define FLASH_ADDR_CONFIG		0x0000FC00

#define FLASH_UNLOCK_CODE0		0x74
#define FLASH_UNLOCK_CODE1		0x45

#define FLASH_ERASE_UNLOCK_CODE0	0x72
#define FLASH_ERASE_UNLOCK_CODE1	0x03
#define FLASH_ERASE_UNLOCK_CODE2	0x02
#define FLASH_ERASE_CODE0		0x02
#define FLASH_ERASE_CODE1		0xC0
#define FLASH_DMA_CODE0			0x05
#define FLASH_DMA_CODE1			0xC0
#define FLASH_DMA_CONFIG		0x06

#define EVENTID_NO_EVENT			0x00
#define EVENTID_ENTER_POINTER			0x03
#define EVENTID_LEAVE_POINTER			0x04
#define EVENTID_MOTION_POINTER			0x05
#define EVENTID_HOVER_ENTER_POINTER		0x07
#define EVENTID_HOVER_LEAVE_POINTER		0x08
#define EVENTID_HOVER_MOTION_POINTER		0x09
#define EVENTID_PROXIMITY_IN			0x0B
#define EVENTID_PROXIMITY_OUT			0x0C
#define EVENTID_MSKEY				0x0E
#define EVENTID_ERROR				0x0F
#define EVENTID_CONTROLLER_READY		0x10
#define EVENTID_SLEEPOUT_CONTROLLER_READY	0x11
#define EVENTID_RESULT_READ_REGISTER		0x12
#define EVENTID_STATUS_REQUEST_COMP		0x13
#define EVENTID_INTERNAL_RELEASE_INFO		0x14
#define EVENTID_EXTERNAL_RELEASE_INFO		0x15
#define EVENTID_STATUS_EVENT			0x16
#define EVENTID_LOCKDOWN_CODE			0x1E
#define EVENTID_GESTURE				0x20
#define EVENTID_LPWG_EVENT			0x22
#define EVENTID_FROM_STRING			0x80
#define EVENTID_DEBUG_EVENT			0xDB

#define STATUS_EVENT_MUTUAL_AUTOTUNE_DONE		0x01
#define STATUS_EVENT_SELF_AUTOTUNE_DONE			0x02
#define STATUS_EVENT_FLASH_WRITE_CONFIG			0x03
#define STATUS_EVENT_FLASH_WRITE_CXTUNE_VALUE		0x04
#define STATUS_EVENT_TRIGGER_FORCE_CALIBRATION		0x05
#define STATUS_EVENT_FINISH_FORCE_CALIBRATION		0x06
#define STATUS_EVENT_RESERVED				0x07
#define STATUS_EVENT_LOCKDOWN_FOR_LGD			0x08
#define STATUS_EVENT_FRAME_DROP				0x09
#define STATUS_EVENT_WATER_MODE				0x0A
#define STATUS_EVENT_NOISE_MODE				0x0B
#define STATUS_EVENT_PURE_AUTOTUNE_SET			0x10
#define STATUS_EVENT_PURE_AUTOTUNE_CLEARED		0x11
#define STATUS_EVENT_BASIC_AUTOTUNE_PROTECTION		0xC1
#define STATUS_EVENT_FLASH_WRITE_AUTOTUNE_VALUE		0xC2
#define STATUS_EVENT_F_CAL_AFTER_AUTOTUNE_PROTECTION	0xC3
#define STATUS_EVENT_CHARGER_CONNECTED			0xCC
#define STATUS_EVENT_CHARGER_DISCONNECTED		0xCD
#define STATUS_EVENT_WIRELESS_CHARGER_ON		0xCE
#define STATUS_EVENT_WIRELESS_CHARGER_OFF		0xCF
#define STATUS_EVENT_DETECT_ESD_PATTERN			0xED

#define EVENTID_ERROR_M3			0x01
#define EVENTID_ERROR_AFE			0x02
#define EVENTID_ERROR_FLASH_CORRUPTION		0x03
#define EVENTID_ERROR_CONFIG_FLASH_CORRUPTION_1	0x01
#define EVENTID_ERROR_CONFIG_FLASH_CORRUPTION_2	0x02
#define EVENTID_ERROR_CX_FLASH_CORRUPTION	0x03
#define EVENTID_ERROR_ITO			0x05
#define EVENTID_ERROR_OSC_TRIM			0x06
#define EVENTID_ERROR_RTOS			0x07
#define EVENTID_ERROR_CX_TUNE			0x08
#define EVENTID_ERROR_LIB			0x09
#define EVENTID_ERROR_LOCKDOWN			0x0B
#define EVENTID_ERROR_INT_FIFO_CLEAR		0x0C

#define FTS_CMD_FAST_SCAN			0x01
#define FTS_CMD_SLOW_SCAN			0x02
#define FTS_CMD_USLOW_SCAN			0x03
#define SLEEPIN					0x90
#define SLEEPOUT				0x91
#define SENSEOFF				0x92
#define SENSEON					0x93
#define FTS_CMD_HOVER_OFF			0x94
#define FTS_CMD_HOVER_ON			0x95
#define FTS_CMD_MSKEY_AUTOTUNE			0x96
#define FTS_CMD_TRIM_LOW_POWER_OSCILLATOR	0x97
#define FTS_CMD_KEY_SENSE_OFF			0x9A
#define FTS_CMD_KEY_SENSE_ON			0x9B
#define FTS_CMD_SET_FAST_GLOVE_MODE		0x9D
#define FTS_CMD_MSHOVER_OFF			0x9E
#define FTS_CMD_MSHOVER_ON			0x9F
#define FTS_CMD_SET_NOR_GLOVE_MODE		0x9F
#define FLUSHBUFFER				0xA1
#define FORCECALIBRATION			0xA2
#define CX_TUNNING				0xA3
#define SELF_AUTO_TUNE				0xA4
#define HF_MAUTO_TUNE				0xA5
#define FTS_CMD_CHARGER_PLUGGED			0xA8
#define FTS_CMD_RELEASEINFO			0xAA
#define FTS_CMD_CHARGER_UNPLUGGED		0xAB
#define FTS_CMD_STYLUS_OFF			0xAB
#define FTS_CMD_STYLUS_ON			0xAC
#define FTS_CMD_LOWPOWER_MODE			0xAD
#define FTS_CMD_ENABLE_FEATURE			0xC1
#define FTS_CMD_DISABLE_FEATURE			0xC2
#define LOCKDOWN_READ				0xC4
#define FTS_CMD_WRITE_PRAM			0xF0
#define FTS_CMD_BURN_PROG_FLASH			0xF2
#define FTS_CMD_ERASE_PROG_FLASH		0xF3
#define FTS_CMD_READ_FLASH_STAT			0xF4
#define FTS_CMD_UNLOCK_FLASH			0xF7
#define FTS_CMD_SAVE_FWCONFIG			0xFB
#define FTS_CMD_SAVE_CX_TUNING			0xFC

#define FTS_TAP_ON		(1 << 0)
#define FTS_SWIPE_UP_ON		(1 << 1)
#define FTS_SWIPE_DOWN_ON	(1 << 2)
#define FTS_SWIPE_RIGHT_ON	(1 << 3)
#define FTS_SWIPE_LEFT_ON	(1 << 4)

#define FTS_TCI1_ON		(1 << 0)
#define FTS_TCI2_ON		(1 << 1)

#define FTS_LPWG_ABS_ON		(1 << 0)

#define TCI_FAIL_NUM		17
#define SWIPE_FAIL_NUM		17
#define FAIL_REASON_BUF_SIZE	50

#define LPWG_FAIL_EVENT		0x0F

#define TCI_DELAY_TIME_INT	5	/* It means Over Tap */

#define TCI_FAIL_REASON_INT_EN_CLEAR		(0)
#define TCI_DISTANCE_INTER_TAP_INT_EN		(1 << 0)
#define TCI_DISTANCE_TOUCHSLOP_INT_EN		(1 << 1)
#define TCI_TIMEOUT_INTER_TAP_INT_EN		(1 << 2)
#define TCI_MULTI_FINGER_INT_EN			(1 << 3)
#define TCI_DELAY_TIME_INT_EN			(1 << 4)	/* It means Over Tap */
#define TCI_PALM_STATE_INT_EN			(1 << 5)
#define TCI_FAIL_REASON_INT_EN_ALL		(TCI_DISTANCE_INTER_TAP_INT_EN \
							| TCI_DISTANCE_TOUCHSLOP_INT_EN \
							| TCI_TIMEOUT_INTER_TAP_INT_EN \
							| TCI_MULTI_FINGER_INT_EN \
							| TCI_DELAY_TIME_INT_EN \
							| TCI_PALM_STATE_INT_EN)

#define SWIPE_FAIL_REASON_INT_EN_CLEAR		(0)
#define SWIPE_1_FINGER_RELEASE_INT_EN		(1 << 0)
#define SWIPE_MULTI_FINGER_INT_EN		(1 << 1)
#define SWIPE_FAST_SWIPE_INT_EN			(1 << 2)
#define SWIPE_SLOW_SWIPE_INT_EN			(1 << 3)
#define SWIPE_INVALID_DIRECTION_INT_EN		(1 << 4)
#define SWIPE_RATIO_FAIL_INT_EN			(1 << 5)
#define SWIPE_OUT_OF_START_AREA_INT_EN		(1 << 6)
#define SWIPE_INITAL_RATIO_FAIL_INT_EN		(1 << 9)
#define SWIPE_WRONG_DIRECTION_INT_EN		(1 << 11)
#define SWIPE_FAIL_REASON_INT_EN_ALL		(SWIPE_1_FINGER_RELEASE_INT_EN \
							| SWIPE_MULTI_FINGER_INT_EN \
							| SWIPE_FAST_SWIPE_INT_EN \
							| SWIPE_SLOW_SWIPE_INT_EN \
							| SWIPE_INVALID_DIRECTION_INT_EN \
							| SWIPE_RATIO_FAIL_INT_EN \
							| SWIPE_OUT_OF_START_AREA_INT_EN \
							| SWIPE_INITAL_RATIO_FAIL_INT_EN \
							| SWIPE_WRONG_DIRECTION_INT_EN)
enum fts_tci_reg_address {
	FTS_TCI_SERVICE_EN			= 0x00,
	FTS_TCI_FRM_RATE			= 0x01,
	FTS_TCI_SENSITIVITY			= 0x02,
	FTS_TCI_EN				= 0x03,
	FTS_TCI_1_SLP_LSB			= 0x04,
	FTS_TCI_1_SLP_MSB			= 0x05,
	FTS_TCI_1_DIST_LSB			= 0x06,
	FTS_TCI_1_DIST_MSB			= 0x07,
	FTS_TCI_1_MIN_TIME			= 0x08,
	FTS_TCI_1_MAX_TIME			= 0x09,
	FTS_TCI_1_CNT				= 0x0A,
	FTS_TCI_1_DLY_TIME			= 0x0B,
	FTS_TCI_1_RESERVED			= 0x0C,
	FTS_TCI_2_SLP_LSB			= 0x0D,
	FTS_TCI_2_SLP_MSB			= 0x0E,
	FTS_TCI_2_DIST_LSB			= 0x0F,
	FTS_TCI_2_DIST_MSB			= 0x10,
	FTS_TCI_2_MIN_TIME			= 0x11,
	FTS_TCI_2_MAX_TIME			= 0x12,
	FTS_TCI_2_CNT				= 0x13,
	FTS_TCI_2_DLY_TIME			= 0x14,
	FTS_TCI_2_RESERVED			= 0x15,
	FTS_TCI_ACT_A1_Y_LSB			= 0x16,
	FTS_TCI_ACT_A1_Y_MSB			= 0x17,
	FTS_TCI_ACT_A1_X_LSB			= 0x18,
	FTS_TCI_ACT_A1_X_MSB			= 0x19,
	FTS_TCI_ACT_A2_Y_LSB			= 0x1A,
	FTS_TCI_ACT_A2_Y_MSB			= 0x1B,
	FTS_TCI_ACT_A2_X_LSB			= 0x1C,
	FTS_TCI_ACT_A2_X_MSB			= 0x1D,
	FTS_SWIPE_UP_DIST			= 0x1E,
	FTS_SWIPE_UP_RATIO			= 0x1F,
	FTS_SWIPE_UP_MIN_TIME			= 0x20,
	FTS_SWIPE_UP_MAX_TIME_LSB		= 0x21,
	FTS_SWIPE_UP_MAX_TIME_MSB		= 0x22,
	FTS_SWIPE_UP_START_A1_Y_LSB		= 0x23,
	FTS_SWIPE_UP_START_A1_Y_MSB		= 0x24,
	FTS_SWIPE_UP_START_A1_X_LSB		= 0x25,
	FTS_SWIPE_UP_START_A1_X_MSB		= 0x26,
	FTS_SWIPE_UP_START_A2_Y_LSB		= 0x27,
	FTS_SWIPE_UP_START_A2_Y_MSB		= 0x28,
	FTS_SWIPE_UP_START_A2_X_LSB		= 0x29,
	FTS_SWIPE_UP_START_A2_X_MSB		= 0x2A,
	FTS_SWIPE_UP_WR_DIR_THRES		= 0x2B,
	FTS_SWIPE_UP_INIT_RATIO_DIST		= 0x2C,
	FTS_SWIPE_UP_INIT_RATIO_THRES		= 0x2D,
	FTS_SWIPE_UP_ACT_A1_Y_LSB		= 0x2E,
	FTS_SWIPE_UP_ACT_A1_Y_MSB		= 0x2F,
	FTS_SWIPE_UP_ACT_A1_X_LSB		= 0x30,
	FTS_SWIPE_UP_ACT_A1_X_MSB		= 0x31,
	FTS_SWIPE_UP_ACT_A2_Y_LSB		= 0x32,
	FTS_SWIPE_UP_ACT_A2_Y_MSB		= 0x33,
	FTS_SWIPE_UP_ACT_A2_X_LSB		= 0x34,
	FTS_SWIPE_UP_ACT_A2_X_MSB		= 0x35,
	FTS_SWIPE_DOWN_DIST			= 0x36,
	FTS_SWIPE_DOWN_RATIO			= 0x37,
	FTS_SWIPE_DOWN_MIN_TIME			= 0x38,
	FTS_SWIPE_DOWN_MAX_TIME_LSB		= 0x39,
	FTS_SWIPE_DOWN_MAX_TIME_MSB		= 0x3A,
	FTS_SWIPE_DOWN_START_A1_Y_LSB		= 0x3B,
	FTS_SWIPE_DOWN_START_A1_Y_MSB		= 0x3C,
	FTS_SWIPE_DOWN_START_A1_X_LSB		= 0x3D,
	FTS_SWIPE_DOWN_START_A1_X_MSB		= 0x3E,
	FTS_SWIPE_DOWN_START_A2_Y_LSB		= 0x3F,
	FTS_SWIPE_DOWN_START_A2_Y_MSB		= 0x40,
	FTS_SWIPE_DOWN_START_A2_X_LSB		= 0x41,
	FTS_SWIPE_DOWN_START_A2_X_MSB		= 0x42,
	FTS_SWIPE_DOWN_WR_DIR_THRES		= 0x43,
	FTS_SWIPE_DOWN_INIT_RATIO_DIST		= 0x44,
	FTS_SWIPE_DOWN_INIT_RATIO_THRES		= 0x45,
	FTS_SWIPE_DOWN_ACT_A1_Y_LSB		= 0x46,
	FTS_SWIPE_DOWN_ACT_A1_Y_MSB		= 0x47,
	FTS_SWIPE_DOWN_ACT_A1_X_LSB		= 0x48,
	FTS_SWIPE_DOWN_ACT_A1_X_MSB		= 0x49,
	FTS_SWIPE_DOWN_ACT_A2_Y_LSB		= 0x4A,
	FTS_SWIPE_DOWN_ACT_A2_Y_MSB		= 0x4B,
	FTS_SWIPE_DOWN_ACT_A2_X_LSB		= 0x4C,
	FTS_SWIPE_DOWN_ACT_A2_X_MSB		= 0x4D,
	FTS_SWIPE_RIGHT_DIST			= 0x4E,
	FTS_SWIPE_RIGHT_RATIO			= 0x4F,
	FTS_SWIPE_RIGHT_MIN_TIME		= 0x50,
	FTS_SWIPE_RIGHT_MAX_TIME_LSB		= 0x51,
	FTS_SWIPE_RIGHT_MAX_TIME_MSB		= 0x52,
	FTS_SWIPE_RIGHT_START_A1_Y_LSB		= 0x53,
	FTS_SWIPE_RIGHT_START_A1_Y_MSB		= 0x54,
	FTS_SWIPE_RIGHT_START_A1_X_LSB		= 0x55,
	FTS_SWIPE_RIGHT_START_A1_X_MSB		= 0x56,
	FTS_SWIPE_RIGHT_START_A2_Y_LSB		= 0x57,
	FTS_SWIPE_RIGHT_START_A2_Y_MSB		= 0x58,
	FTS_SWIPE_RIGHT_START_A2_X_LSB		= 0x59,
	FTS_SWIPE_RIGHT_START_A2_X_MSB		= 0x5A,
	FTS_SWIPE_RIGHT_WR_DIR_THRES		= 0x5B,
	FTS_SWIPE_RIGHT_INIT_RATIO_DIST		= 0x5C,
	FTS_SWIPE_RIGHT_INIT_RATIO_THRES	= 0x5D,
	FTS_SWIPE_RIGHT_ACT_A1_Y_LSB		= 0x5E,
	FTS_SWIPE_RIGHT_ACT_A1_Y_MSB		= 0x5F,
	FTS_SWIPE_RIGHT_ACT_A1_X_LSB		= 0x60,
	FTS_SWIPE_RIGHT_ACT_A1_X_MSB		= 0x61,
	FTS_SWIPE_RIGHT_ACT_A2_Y_LSB		= 0x62,
	FTS_SWIPE_RIGHT_ACT_A2_Y_MSB		= 0x63,
	FTS_SWIPE_RIGHT_ACT_A2_X_LSB		= 0x64,
	FTS_SWIPE_RIGHT_ACT_A2_X_MSB		= 0x65,
	FTS_SWIPE_LEFT_DIST			= 0x66,
	FTS_SWIPE_LEFT_RATIO			= 0x67,
	FTS_SWIPE_LEFT_MIN_TIME			= 0x68,
	FTS_SWIPE_LEFT_MAX_TIME_LSB		= 0x69,
	FTS_SWIPE_LEFT_MAX_TIME_MSB		= 0x6A,
	FTS_SWIPE_LEFT_START_A1_Y_LSB		= 0x6B,
	FTS_SWIPE_LEFT_START_A1_Y_MSB		= 0x6C,
	FTS_SWIPE_LEFT_START_A1_X_LSB		= 0x6D,
	FTS_SWIPE_LEFT_START_A1_X_MSB		= 0x6E,
	FTS_SWIPE_LEFT_START_A2_Y_LSB		= 0x6F,
	FTS_SWIPE_LEFT_START_A2_Y_MSB		= 0x70,
	FTS_SWIPE_LEFT_START_A2_X_LSB		= 0x71,
	FTS_SWIPE_LEFT_START_A2_X_MSB		= 0x72,
	FTS_SWIPE_LEFT_WR_DIR_THRES		= 0x73,
	FTS_SWIPE_LEFT_INIT_RATIO_DIST		= 0x74,
	FTS_SWIPE_LEFT_INIT_RATIO_THRES		= 0x75,
	FTS_SWIPE_LEFT_ACT_A1_Y_LSB		= 0x76,
	FTS_SWIPE_LEFT_ACT_A1_Y_MSB		= 0x77,
	FTS_SWIPE_LEFT_ACT_A1_X_LSB		= 0x78,
	FTS_SWIPE_LEFT_ACT_A1_X_MSB		= 0x79,
	FTS_SWIPE_LEFT_ACT_A2_Y_LSB		= 0x7A,
	FTS_SWIPE_LEFT_ACT_A2_Y_MSB		= 0x7B,
	FTS_SWIPE_LEFT_ACT_A2_X_LSB		= 0x7C,
	FTS_SWIPE_LEFT_ACT_A2_X_MSB		= 0x7D,
	FTS_TAP_FAIL_REASON_INT_STATUS		= 0x7E,
	FTS_TAP_FAIL_REASON_INT_ENABLE_LSB	= 0x7F,
	FTS_TAP_FAIL_REASON_INT_ENABLE_MSB	= 0x80,
	FTS_SWIPE_UP_FAIL_REASON_INT_STATUS	= 0x81,
	FTS_SWIPE_UP_FAIL_REASON_ENABLE_LSB	= 0x82,
	FTS_SWIPE_UP_FAIL_REASON_ENABLE_MSB	= 0x83,
	FTS_SWIPE_DOWN_FAIL_REASON_INT_STATUS	= 0x84,
	FTS_SWIPE_DOWN_FAIL_REASON_ENABLE_LSB	= 0x85,
	FTS_SWIPE_DOWN_FAIL_REASON_ENABLE_MSB	= 0x86,
	FTS_SWIPE_RIGHT_FAIL_REASON_INT_STATUS	= 0x87,
	FTS_SWIPE_RIGHT_FAIL_REASON_ENABLE_LSB	= 0x88,
	FTS_SWIPE_RIGHT_FAIL_REASON_ENABLE_MSB	= 0x89,
	FTS_SWIPE_LEFT_FAIL_REASON_INT_STATUS	= 0x8A,
	FTS_SWIPE_LEFT_FAIL_REASON_ENABLE_LSB	= 0x8B,
	FTS_SWIPE_LEFT_FAIL_REASON_ENABLE_MSB	= 0x8C,

	FTS_TCI1_FAIL_REASON			= 0x105, /* 0x105(261) ~ 0x136(310) */
	FTS_TCI2_FAIL_REASON			= 0x137, /* 0x137(311) ~ 0x168(360) */
	FTS_SWIPE_UP_FAIL_REASON		= 0x169, /* 0x169(361) ~ 0x197(410) */
	FTS_SWIPE_DOWN_FAIL_REASON		= 0x198, /* 0x198(411) ~ 0x1CC(460) */
	FTS_SWIPE_RIGHT_FAIL_REASON		= 0x1CD, /* 0x1CD(461) ~ 0x1FE(510) */
	FTS_SWIPE_LEFT_FAIL_REASON		= 0x1FF, /* 0x1FF(511) ~ 0x230(560) */
	FTS_TCI1_FAIL_REASON_CNT		= 0x231,
	FTS_TCI2_FAIL_REASON_CNT		= 0x232,
	FTS_SWIPE_UP_FAIL_REASON_CNT		= 0x233,
	FTS_SWIPE_DOWN_FAIL_REASON_CNT		= 0x234,
	FTS_SWIPE_RIGHT_FAIL_REASON_CNT		= 0x235,
	FTS_SWIPE_LEFT_FAIL_REASON_CNT		= 0x236,
	FTS_LPWG_ABS_EN				= 0x237,
	FTS_LPWG_ABS_ACT_A1_Y_LSB		= 0x238,
	FTS_LPWG_ABS_ACT_A1_Y_MSB		= 0x239,
	FTS_LPWG_ABS_ACT_A1_X_LSB		= 0x23A,
	FTS_LPWG_ABS_ACT_A1_X_MSB		= 0x23B,
	FTS_LPWG_ABS_ACT_A2_Y_LSB		= 0x23C,
	FTS_LPWG_ABS_ACT_A2_Y_MSB		= 0x23D,
	FTS_LPWG_ABS_ACT_A2_X_LSB		= 0x23E,
	FTS_LPWG_ABS_ACT_A2_X_MSB		= 0x23F,
};

enum fts_error_return {
	FTS_NOT_ERROR = 0,
	FTS_ERROR_INVALID_CHIP_ID,
	FTS_ERROR_INVALID_CHIP_VERSION_ID,
	FTS_ERROR_INVALID_SW_VERSION,
	FTS_ERROR_EVENT_ID,
	FTS_ERROR_TIMEOUT,
	FTS_ERROR_FW_UPDATE_FAIL,
};

enum binfile_type {
	BIN_INVALID = 0,
	BIN_FTS128 = 1,
	BIN_FTS256 = 2,
	BIN_FTB = 3,
};

enum {
	IC_INIT_NEED = 0,
	IC_INIT_DONE = 1,
};

enum {
	SW_RESET = 0,
	HW_RESET = 1,
};

enum palm_status {
	PALM_RELEASED = 0,
	PALM_PRESSED = 1,
};

enum {
	LPWG_TAP_EVENT = 0,
	LPWG_SWIPE_UP_EVENT = 1,
	LPWG_SWIPE_DOWN_EVENT = 2,
	LPWG_SWIPE_RIGHT_EVENT = 3,
	LPWG_SWIPE_LEFT_EVENT = 4,
};

enum {
	TCI1_EVENT = 0,
	TCI2_EVENT = 1,
};

enum {
	FTM4_SWIPE_U = 0,
	FTM4_SWIPE_D = 1,
	FTM4_SWIPE_R = 2,
	FTM4_SWIPE_L = 3,
	FTM4_SWIPE_NUM = 4,
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

struct fw_ftb_header {
	u32 signature;
	u32 ftb_ver;
	u32 target;
	u32 fw_id;
	u32 fw_ver;
	u32 cfg_id;
	u32 cfg_ver;
	u32 reserved[2];
	u32 bl_fw_ver;
	u32 ext_ver;
	u32 sec0_size;
	u32 sec1_size;
	u32 sec2_size;
	u32 sec3_size;
	u32 hdr_crc;
};

struct ftm4_version {
	u16 internal_ver;
	u16 internal_config;
	u8 build : 4;
	u8 major : 4;
	u8 minor;
};

struct fts_flash_corruption_info {
	bool fw_broken;
	bool cfg_broken;
	bool cx_broken;
};

struct ftm4_prd_info {
	u8 product_id[3];
	u8 chip_rev;
	u8 fpc_rev;
	u8 panel_rev;
	u8 inspector_ch;
	u8 date[6];
};

struct afe_info {
	u8 old_ver;
	u8 ver;
	u8 final;
};

struct ftm4_active_area {
	s16 x1;
	s16 y1;
	s16 x2;
	s16 y2;
	s16 offset_y;
};

struct lpwg_abs_ctrl {
	bool enable;
	struct ftm4_active_area area;
	struct ftm4_active_area border_area;
};

struct voice_button_ctrl {
	bool enable;
	struct ftm4_active_area area;
	struct ftm4_active_area border_area;
	struct ftm4_active_area total_area;
};

struct ftm4_blank {
	int prev;
	int curr;
};

struct ftm4_data {
	struct device *dev;
	struct kobject kobj;
	struct mutex io_lock;
	struct pm_qos_request pm_qos_req;
	struct delayed_work lpwg_debug_work;
	struct delayed_work fb_notify_work;
	atomic_t power;
	atomic_t init;
	struct ftm4_version ic_fw_ver;
	struct ftm4_prd_info prd_info;
	u32 bin_product_id[2][3];
	struct afe_info afe;
	bool pure_autotune;
	u8 pure_autotune_info;
	int ic_info_ret;

	u8 lcd_mode;
	u8 prev_lcd_mode;

	int ta_detect_pin;
	struct ftm4_blank blank_status;
	int vr_status;
	int q_sensitivity;

	u8 data[FTS_EVENT_SIZE * FTS_FIFO_MAX];
	enum palm_status palm;

	u16 tci_base_addr;	/* Tap common interface */
	bool tci1_debug_enable;
	bool tci2_debug_enable;
	struct lpwg_abs_ctrl lpwg_abs;
	struct voice_button_ctrl voice_button;

	struct fts_flash_corruption_info flash_corruption_info;
};

static inline struct ftm4_data *to_ftm4_data(struct device *dev)
{
	return (struct ftm4_data *)touch_get_device(to_touch_core(dev));
}

static inline struct ftm4_data *to_ftm4_data_from_kobj(
						struct kobject *kobj)
{
	return (struct ftm4_data *)container_of(kobj,
			struct ftm4_data, kobj);
}

int ftm4_reg_read(struct device *dev, u8 *reg, int cnum, u8 *buf, int num);
int ftm4_reg_write(struct device *dev, u8 *reg, u16 num_com);
void ftm4_command(struct device *dev, u8 cmd);
int ftm4_system_reset(struct device *dev);
int ftm4_interrupt_set(struct device *dev, int enable);
int ftm4_wait_for_ready(struct device *dev);
int ftm4_read_chip_id(struct device *dev);
int ftm4_cmd_completion_check(struct device *dev, u8 event1, u8 event2, u8 event3);
int ftm4_ic_info(struct device *dev);
int ftm4_fw_wait_for_event(struct device *dev, u8 eid);
void ftm4_do_autotune(struct device *dev);
int ftm4_check_pure_autotune_key(void);
void ftm4_osc_trim_cmd(struct device *dev);
void ftm4_execute_force_autotune(struct device *dev);
int ftm4_init(struct device *dev);

#endif /* LGE_TOUCH_FTM4_H */
