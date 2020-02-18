/* production_test.h
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

/* Include to touch core Header File */
#include "../../touch_core.h"

/* Include to Local Header File */
#include "touch_sw42000.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

/* production test */
#define TC_PT_TEST_CTL			(0xC04)
//(base addr) : r_tc_sts_spi_addr : 0x666
//PT resister Offset
#define PT_TEST_STS_OFT			(0x2A)
#define PT_TEST_PF_RESULT_OFT	(0x2B)
#define FLASH_INFO_CHECK_RESULT_OFT (0x3F)
#define tc_tsp_test_data_offset	(0x03C)
#define tune_code_addr			(0x6A6)	//(0x600)	//need check
#define LINE_FILTER_OPTION		(0x40000)

/* Engine debuging tool Reg */
#define ADDR_CMD_REG_SIC_IMAGECTRL_TYPE		(0xF0C)
#define ADDR_CMD_REG_SIC_GETTER_READYSTATUS	(0xF04)

#define TIME_STR_LEN		(64)

enum {
	PT_FRAME_1 = 0,
	PT_FRAME_2,
	PT_FRAME_3,
	PT_FRAME_4,
};

enum {
	DATA_IC_COMPARE = 0,
	DATA_DRIVER_COMPARE,
};

enum {
	DBG_MUTUAL = 0,
	DBG_SELF,
};

enum {
	VERTICAL_DATA = 0,
	HORIZONTAL_DATA,
};

/* Chanel settings */
#define MAX_CHANNEL		(40)	//<ROW>
#define COL_MAX_CHANNEL		(30)//(36)	//(36)
#define ROW_SIZE		(34)	//toe:(36)	//(32)
#define COL_SIZE		(15)	//toe:(18)	//(16)
#define M1_COL_SIZE		(2)
#define SIMPLE_SPEC_SIZE	(2)

#define LOG_BUF_SIZE		(512)
#define OCD_SIZE		104
#define DBG_BUF2_OFFSET		1280
#define BUF_SIZE (PAGE_SIZE * 2)
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)
#define DEBUG_ROW_SIZE		ROW_SIZE
#define DEBUG_COL_SIZE		COL_SIZE



/* tune code */
#define tc_tune_code_size		260
typedef union {
	struct {
		unsigned r_goft_tune_m1:	4;
		unsigned r_goft_tune_m1_sign:	1;
		unsigned r_goft_tune_m2:	4;
		unsigned r_goft_tune_m2_sign:	1;
		unsigned r_goft_tune_nd:	5;
		unsigned reserved:		17;
	} b;
	u32 w;
} t_goft_tune;

#if (0)
#define LOFT_CH_NUM	(MAX_CHANNEL/2)

struct tune_data_format {
	u32		r_tune_code_magic;

	t_goft_tune	r_goft_tune_u3_m1m2_left;
	u16		r_loft_tune_u3_m1_left[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_g1_left[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_g2_left[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_g3_left[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_nd_left[LOFT_CH_NUM];

	t_goft_tune	r_goft_tune_u3_m1m2_right;
	u16		r_loft_tune_u3_m1_right[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_g1_right[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_g2_right[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_g3_right[LOFT_CH_NUM];
	u16		r_loft_tune_u3_m2_nd_right[LOFT_CH_NUM];

	t_goft_tune	r_goft_tune_u0_m1m2_left;
	u16		r_loft_tune_u0_m1_left[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_g1_left[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_g2_left[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_g3_left[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_nd_left[LOFT_CH_NUM];

	t_goft_tune	r_goft_tune_u0_m1m2_right;
	u16		r_loft_tune_u0_m1_right[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_g1_right[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_g2_right[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_g3_right[LOFT_CH_NUM];
	u16		r_loft_tune_u0_m2_nd_right[LOFT_CH_NUM];
};
#else	//sw42000 alpha
#define LOFT_CH_NUM	(MAX_CHANNEL)

struct m1_tune {
	u8 m1_tune_sel_code[LOFT_CH_NUM];
};

struct m2_tune {
	u8 m2_tune_high_sel_code[LOFT_CH_NUM];
	u8 m2_tune_low_sel_code[LOFT_CH_NUM];
	u8 self1_tune_sel_code[LOFT_CH_NUM];
	u8 self2_tune_sel_code[LOFT_CH_NUM];
	u16 icr_tune_sel_code[LOFT_CH_NUM];
};

struct tune_data_sel {
	struct m1_tune	m1;
	struct m2_tune	m2;
};

struct tune_data_format {
	u32	r_tune_code_magic;

	struct tune_data_sel u3_tune;
	struct tune_data_sel u2_tune;
	struct tune_data_sel u0_tune;
};

#endif

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

/* AIT IT_IMAGE CMD */
enum {
	CMD_NONE = 0,
	CMD_RAWDATA,
	CMD_BASE_DATA,
	CMD_DELTADATA,
	CMD_LABELDATA,
	CMD_FILTERED_DELTA,
	CMD_RESERVED,
	CMD_DEBUGDATA,
	/* self cmd */
	CMD_RAWDATA_S,
	CMD_BASE_DATA_S,
	CMD_DELTADATA_S,
	CMD_LABELDATA_S,
	CMD_DEBUGDATA_S,

	DONT_USE_CMD = 0xEE,
	CMD_WAIT = 0xFF,
};

enum {
	U3_PT_TEST = 0,
	OPEN_NODE_TEST,
	SHORT_NODE_TEST,
	U3_M1_RAWDATA_TEST,
	U3_M1_JITTER_TEST,
	U3_M2_RAWDATA_TEST,
	U3_M2_JITTER_TEST,
	U0_M1_RAWDATA_TEST,
	U0_M1_JITTER_TEST,
	U0_M2_RAWDATA_TEST,
	U0_M2_JITTER_TEST,
	U3_M2_DELTA_TEST,
	U0_M2_DELTA_TEST,
	U3_BLU_JITTER_TEST,
	AVERAGE_JITTER_TEST,

	OPEN_RX_NODE_TEST,
	OPEN_TX_NODE_TEST,
	SHORT_RX_NODE_TEST,
	SHORT_TX_NODE_TEST,

	U3_M2_RAW_SELF_TEST = 19,
	U3_M2_JITTER_SELF_TEST,
	U0_M1_RAW_SELF_TEST,
	FLASH_INFO_CHECK,
	PT_INFO_CHECK,
};

enum {
	NORMAL_MODE = 0,
	PRODUCTION_MODE,
};

typedef enum {
	IT_NONE = 0,
	IT_ALGORITHM_RAW_IMAGE,
	IT_BASELINE_E_IMAGE,
	IT_BASELINE_O_IMAGE,
	IT_DELTA_IMAGE,
	IT_LABEL_IMAGE,
	IT_FILTERED_DELTA_IMAGE,
	IT_WAIT = 0xFF
} eImageType_t;

typedef enum {
	RS_READY    = 0xA0,
	RS_NONE     = 0x05,
	RS_LOG      = 0x77,
	RS_IMAGE	= 0xAA
} eProtocolReadyStatus_t;

struct frame_offset {
	u16 frame_1_offset;
	u16 frame_2_offset;
	u16 frame_3_offset;
	u16 frame_4_offset;
};

struct ait_tool_offset {
	u16 raw;	/* mutual data */
	u16 delta;
	u16 label;
	u16 base;
	u16 debug;
	u16 raw_s;	/* self data */
	u16 delta_s;
	u16 label_s;
	u16 base_s;
	u16 debug_s;
};

struct select_frame {
	u16 open_rx_frame;
	u16 open_tx_frame;
	u16 short_rx_frame;
	u16 short_tx_frame;
	u16 u3_m1_raw_frame;
	u16 u3_m1_jitter_frame;
	u16 u3_m2_raw_frame;
	u16 u3_m2_jitter_frame;
	u16 u0_m1_raw_frame;
	u16 u0_m1_jitter_frame;
	u16 u0_m2_raw_frame;
	u16 u0_m2_jitter_frame;
	u16 u3_m2_delta_frame;
	u16 u0_m2_delta_frame;
	u16 u3_blu_jitter_frame;
};

struct notch_cell {
	u8 start;
	u8 end;
};


struct prd_test_param {
	u32 sd_test_set;
	u32 lpwg_sd_test_set;
	u16 os_test_type;
	char *spec_file_path;
	char *mfts_spec_file_path;
	struct select_frame frame;
	struct frame_offset offset;
	struct ait_tool_offset ait_offset;
	struct notch_cell notch;
	u16 u3_m2_self;
};

/* [Start] for siw app */
enum {
	REPORT_END_RS_NG = 0x05,
	REPORT_END_RS_OK = 0xAA,
};

enum {
	PRD_CMD_TYPE_1 = 0,		/* new type: base only */
	PRD_CMD_TYPE_2 = 1,		/* old type: base_even, base_odd */
};


enum {
	APP_REPORT_OFF = 0,
	APP_REPORT_RAW,
	APP_REPORT_BASE,
	APP_REPORT_DELTA,
	APP_REPORT_LABEL,
	APP_REPORT_DEBUG_BUF,
	APP_REPORT_MAX,
};

enum {
	PRD_DATA_NAME_SZ	= 128,
	/* */
	PRD_ROW_SIZE		= ROW_SIZE,
	PRD_COL_SIZE		= COL_SIZE,
	PRD_COL_ADD			= 1,
	PRD_CH				= MAX_CHANNEL,
	PRD_M1_COL			= M1_COL_SIZE,
	PRD_CMD_TYPE		= PRD_CMD_TYPE_1,
	SECOND_SCR_BOUND_I	= 0,
	SECOND_SCR_BOUND_J	= 0,
	/* */
	PRD_SHOW_FLAG_DISABLE_PRT_RAW	= (1<<0),
	PRD_APP_INFO_SIZE	= 32,
};

enum {
	PRD_M2_ROW_COL_SIZE		= (PRD_ROW_SIZE * PRD_COL_SIZE),
	PRD_M2_ROW_COL_BUF_SIZE	= (PRD_ROW_SIZE * (PRD_COL_SIZE + PRD_COL_ADD)),
	/* */
	PRD_DELTA_SIZE			= ((PRD_ROW_SIZE+2)*(PRD_COL_SIZE+2)),
	/* */
	PRD_LABEL_TMP_SIZE		= ((PRD_ROW_SIZE+2)*(PRD_COL_SIZE+2)),
	PRD_DEBUG_BUF_SIZE		= (PRD_M2_ROW_COL_SIZE),
};

struct siw_hal_prd_data {
	struct device *dev;
	char name[PRD_DATA_NAME_SZ];
	/* */
	int prd_app_mode;
	/* */
	int16_t	m2_buf_rawdata[PRD_M2_ROW_COL_BUF_SIZE];
	/* */
	int16_t	buf_delta[PRD_DELTA_SIZE];
	int16_t	buf_debug[PRD_DEBUG_BUF_SIZE];
	u8	buf_label_tmp[PRD_LABEL_TMP_SIZE];
	u8	buf_label[PRD_M2_ROW_COL_SIZE];
	/* */
	struct prd_test_param prd_param;
};

enum {
	PRD_SYS_EN_IDX_SD = 0,
	PRD_SYS_EN_IDX_DELTA,
	PRD_SYS_EN_IDX_LABEL,
	PRD_SYS_EN_IDX_RAWDATA_PRD,
	PRD_SYS_EN_IDX_RAWDATA_TCM,
	PRD_SYS_EN_IDX_RAWDATA_AIT,
	PRD_SYS_EN_IDX_BASE,
	PRD_SYS_EN_IDX_DEBUG_BUF,
	PRD_SYS_EN_IDX_LPWG_SD,
	PRD_SYS_EN_IDX_FILE_TEST,
	PRD_SYS_EN_IDX_APP_RAW,
	PRD_SYS_EN_IDX_APP_BASE,
	PRD_SYS_EN_IDX_APP_LABEL,
	PRD_SYS_EN_IDX_APP_DELTA,
	PRD_SYS_EN_IDX_APP_DEBUG_BUF,
	PRD_SYS_EN_IDX_APP_END,
	PRD_SYS_EN_IDX_APP_INFO,
	PRD_SYS_ATTR_MAX,
};

enum {
	PRD_SYS_EN_SD				= (1<<PRD_SYS_EN_IDX_SD),
	PRD_SYS_EN_DELTA			= (1<<PRD_SYS_EN_IDX_DELTA),
	PRD_SYS_EN_LABEL			= (1<<PRD_SYS_EN_IDX_LABEL),
	PRD_SYS_EN_RAWDATA_PRD		= (1<<PRD_SYS_EN_IDX_RAWDATA_PRD),
	PRD_SYS_EN_RAWDATA_TCM		= (1<<PRD_SYS_EN_IDX_RAWDATA_TCM),
	PRD_SYS_EN_RAWDATA_AIT		= (1<<PRD_SYS_EN_IDX_RAWDATA_AIT),
	PRD_SYS_EN_BASE				= (1<<PRD_SYS_EN_IDX_BASE),
	PRD_SYS_EN_DEBUG_BUF		= (0<<PRD_SYS_EN_IDX_DEBUG_BUF),
	PRD_SYS_EN_LPWG_SD			= (1<<PRD_SYS_EN_IDX_LPWG_SD),
	PRD_SYS_EN_FILE_TEST		= (1<<PRD_SYS_EN_IDX_FILE_TEST),
	PRD_SYS_EN_APP_RAW			= (1<<PRD_SYS_EN_IDX_APP_RAW),
	PRD_SYS_EN_APP_BASE			= (1<<PRD_SYS_EN_IDX_APP_BASE),
	PRD_SYS_EN_APP_LABEL		= (1<<PRD_SYS_EN_IDX_APP_LABEL),
	PRD_SYS_EN_APP_DELTA		= (1<<PRD_SYS_EN_IDX_APP_DELTA),
	PRD_SYS_EN_APP_DEBUG_BUF	= (1<<PRD_SYS_EN_IDX_APP_DEBUG_BUF),
	PRD_SYS_EN_APP_END			= (1<<PRD_SYS_EN_IDX_APP_END),
	PRD_SYS_EN_APP_INFO			= (1<<PRD_SYS_EN_IDX_APP_INFO),
};

#define PRD_SYS_ATTR_EN_FLAG	(0		|\
		PRD_SYS_EN_SD			|\
		PRD_SYS_EN_DELTA		|\
		PRD_SYS_EN_LABEL		|\
		PRD_SYS_EN_RAWDATA_PRD		|\
		PRD_SYS_EN_RAWDATA_TCM		|\
		PRD_SYS_EN_RAWDATA_AIT		|\
		PRD_SYS_EN_BASE			|\
		PRD_SYS_EN_DEBUG_BUF		|\
		PRD_SYS_EN_LPWG_SD		|\
		PRD_SYS_EN_FILE_TEST		|\
		PRD_SYS_EN_APP_RAW		|\
		PRD_SYS_EN_APP_BASE		|\
		PRD_SYS_EN_APP_LABEL		|\
		PRD_SYS_EN_APP_DELTA		|\
		PRD_SYS_EN_APP_DEBUG_BUF	|\
		PRD_SYS_EN_APP_END		|\
		PRD_SYS_EN_APP_INFO		|\
		0)
/* [End] for siw app */

#define data_buf_read_opt(i, j, col, row, direction)	\
		({	u16 _r = 0;	\
			do {	\
				if (direction == HORIZONTAL_DATA) {	\
					_r = data_buf[i*col + j];	\
				} else {	\
					_r = data_buf[j*row + i];	\
				}	\
				break;	\
			} while (0);	\
			_r;	\
		})

extern void touch_msleep(unsigned int msecs);
int sw42000_prd_register_sysfs(struct device *dev);

/* For BLU test. We need to control backlight level. */
#if defined(CONFIG_TOUCHSCREEN_MTK)
extern unsigned int mt_get_bl_brightness(void);
extern int mt65xx_leds_brightness_set(int, int);
#elif defined(CONFIG_LGE_TOUCH_CORE_QCT)
extern int mdss_fb_get_bl_brightness_extern(void);
extern void mdss_fb_set_bl_brightness_extern(int);
#endif

#endif


