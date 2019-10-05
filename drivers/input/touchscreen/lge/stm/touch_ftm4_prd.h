/* touch_ftm4_prd.h
 *
 * Copyright (C) 2017 LGE.
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

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_ftm4.h"

#ifndef LGE_TOUCH_FTM4_PRD_H
#define LGE_TOUCH_FTM4_PRD_H

#define FORCE_CH_SIZE		16
#define SENSE_CH_SIZE		32

#define LOG_BUF_SIZE		256
#define FILE_BUF_SIZE		256
#define BUF_SIZE                (PAGE_SIZE * 2)

#define MAX_LOG_FILE_SIZE       (10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT      (4)
#define FILE_STR_LEN		(128)
#define TIME_STR_LEN            (64)

#define READ_CHUNK_SIZE			128

#define FTS_WATER_SELF_RAW_ADDR		0x1E

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

enum {
	OPEN_TEST_FLAG = (1 << 0),
	SHORT_TEST_FLAG = (1 << 1),
};

enum sd_test_type {
	SD_TEST,
	LPWG_SD_TEST,
};

enum frame_data_type {
	TYPE_RAW_DATA = 0,
	TYPE_FILTERED_DATA = 2,
	TYPE_STRENGTH_DATA = 4,
	TYPE_BASELINE_DATA = 6,
};

enum ito_err_type {
	NO_ERROR = 0,
	ITO_FORCE_OPEN,
	ITO_SENSE_OPEN,
	ITO_FORCE_SHRT_GND,
	ITO_SENSE_SHRT_GND,
	ITO_FORCE_SHRT_VCM,
	ITO_SENSE_SHRT_VCM,
	ITO_FORCE_SHRT_FORCE,
	ITO_SENSE_SHRT_SENSE,
	ITO_F2E_SENSE,
	ITO_FPC_FORCE_OPEN,
	ITO_FPC_SENSE_OPEN,
	ITO_KEY_FORCE_OPEN,
	ITO_KEY_SENSE_OPEN,
	ITO_RESERVED0,
	ITO_RESERVED1,
	ITO_RESERVED2,
	ITO_MAX_ERR_REACHED = 0xFF,
};

enum fts_system_information_address {
	FTS_SI_FILTERED_RAW_ADDR		= 0x02,
	FTS_SI_STRENGTH_ADDR			= 0x04,
	FTS_SI_SELF_FILTERED_FORCE_RAW_ADDR	= 0x1E,
	FTS_SI_SELF_FILTERED_SENSE_RAW_ADDR	= 0x20,
	FTS_SI_NOISE_PARAM_ADDR			= 0x40,
	FTS_SI_PURE_AUTOTUNE_FLAG		= 0x4E,
	FTS_SI_COMPENSATION_OFFSET_ADDR		= 0x50,
	FTS_SI_PURE_AUTOTUNE_CONFIG		= 0x52,
	FTS_SI_FACTORY_RESULT_FLAG		= 0x56,
	FTS_SI_AUTOTUNE_CNT			= 0x58,
	FTS_SI_SENSE_CH_LENGTH			= 0x5A, /* 2 bytes */
	FTS_SI_FORCE_CH_LENGTH			= 0x5C, /* 2 bytes */
	FTS_SI_FINGER_THRESHOLD			= 0x60, /* 2 bytes */
	FTS_SI_AUTOTUNE_PROTECTION_CONFIG	= 0x62, /* 2 bytes */
	FTS_SI_REPORT_PRESSURE_RAW_DATA		= 0x64, /* 2 bytes */
	FTS_SI_SS_KEY_THRESHOLD			= 0x66, /* 2 bytes */
	FTS_SI_MS_TUNE_VERSION			= 0x68, /* 2 bytes */
	FTS_SI_CONFIG_CHECKSUM			= 0x6A, /* 4 bytes */
	FTS_SI_PRESSURE_FILTERED_RAW_ADDR	= 0x70,
	FTS_SI_PRESSURE_STRENGTH_ADDR		= 0x72,
	FTS_SI_PRESSURE_THRESHOLD		= 0x76,
};

extern void touch_msleep(unsigned int msecs);
int ftm4_prd_register_sysfs(struct device *dev);

#endif	/* LGE_TOUCH_FTM4_PRD_H */

