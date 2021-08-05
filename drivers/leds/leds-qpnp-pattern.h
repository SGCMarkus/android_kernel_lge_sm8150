#ifndef __LEDS_QPNP_PATTERN_H_INCLUDED
#define __LEDS_QPNP_PATTERN_H_INCLUDED

#include "leds.h"
#include "led-pattern.h"
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <soc/qcom/lge/board_lge.h>

/*  Pattern data structure
 *
 *  0 ............. 48                  [LUT TABLE]
 *
 *  START       LENGTH
 *  48          49                      [RED]
 *  50          51                      [GRN]
 *  52          53                      [BLU]
 *
 *  PAUSE_LO    PAUSE_HI    PAUSE_STEP  [R/G/B COMMON]
 *  54          55          56
 *  FLAGS
 *  57
 */

// Sizes of a pattern data structure
#define PATTERN_SIZE_ARRAY          58
#define PATTERN_SIZE_LUT            48

// Time parameters in MilliSec.
#define PATTERN_STEP_DEFAULT        100
#define PATTERN_STEP_CHARGING       140
#define PATTERN_STEP_POWERONOFF     130
#define PATTERN_STEP_NOTI           50
#define PATTERN_STEP_HI_NOTI        75
#define PATTERN_STEP_FULL_CHARGING  10000
#define PATTERN_STEP_SPRINT_POWERONOFF    120
#define PATTERN_STEP_SPRINT_INCALL        160
#define PATTERN_STEP_SPRINT_INUSE         80
#define PATTERN_STEP_5G_DATA              120
#define PATTERN_PAUSE_DISABLED      0
#define PATTERN_PAUSE_FULL_CHARGING 1
#define PATTERN_PAUSE_HI_NOTI          (12000 - (PATTERN_STEP_NOTI*7)) / PATTERN_STEP_NOTI
#define PATTERN_PAUSE_HI_MISSED_NOTI   (12000 - (PATTERN_STEP_NOTI*14)) / PATTERN_STEP_NOTI
#define PATTERN_PAUSE_HI_STEP_NOTI     (12000 - (PATTERN_STEP_HI_NOTI*7)) / PATTERN_STEP_HI_NOTI
#define PATTERN_PAUSE_HI_URGENT        (12000 - (PATTERN_STEP_NOTI*11)) / PATTERN_STEP_NOTI
#define PATTERN_PAUSE_HI_SECRETMODE    (12000 - (PATTERN_STEP_NOTI*23)) / PATTERN_STEP_NOTI
#define PATTERN_PAUSE_LO_SPRINT_INUSE  (4000 - (PATTERN_STEP_SPRINT_INUSE*20)) / PATTERN_STEP_SPRINT_INUSE
#define PATTERN_PAUSE_HI_SPRINT_INUSE  20
#define PATTERN_PAUSE_HI_CHARING       7

// Define RGB LED ID
#define QPNP_ID_RGB_RED		    0
#define QPNP_ID_RGB_GREEN	    1
#define QPNP_ID_RGB_BLUE	    2

// Flags for Look Up Table
#define PM_PWM_LUT_LOOP		    0x01
#define PM_PWM_LUT_RAMP_UP	    0x02
#define PM_PWM_LUT_REVERSE	    0x04
#define PM_PWM_LUT_PAUSE_HI_EN	    0x08
#define PM_PWM_LUT_PAUSE_LO_EN	    0x10
#define PM_PWM_LUT_NO_TABLE	    0x20
#define PM_PWM_LUT_USE_RAW_VALUE    0x40

// Pattern flags for the LPG
//#define PATTERN_FLAG_ONESHOT        /*0x42*/ (PM_PWM_LUT_USE_RAW_VALUE|PM_PWM_LUT_RAMP_UP)
//#define PATTERN_FLAG_SOLID          /*0x43*/ (PATTERN_FLAG_ONESHOT|PM_PWM_LUT_LOOP)
//#define PATTERN_FLAG_REPEAT         /*0x4B*/ (PATTERN_FLAG_ONESHOT|PM_PWM_LUT_PAUSE_HI_EN|PM_PWM_LUT_LOOP)
//#define PATTERN_FLAG_WAVEFORM       /*0x4F*/ (PATTERN_FLAG_REPEAT|PM_PWM_LUT_REVERSE)
//#define PATTERN_FLAG_BLINK          /*0x5B*/ (PATTERN_FLAG_REPEAT|PM_PWM_LUT_PAUSE_LO_EN)
#define PATTERN_FLAG_ONESHOT        /*0x2*/ PM_PWM_LUT_RAMP_UP
#define PATTERN_FLAG_SOLID          /*0x3*/ (PATTERN_FLAG_ONESHOT|PM_PWM_LUT_LOOP)
#define PATTERN_FLAG_REPEAT         /*0xB*/ (PATTERN_FLAG_ONESHOT|PM_PWM_LUT_PAUSE_HI_EN|PM_PWM_LUT_LOOP)
#define PATTERN_FLAG_WAVEFORM       /*0xF*/ (PATTERN_FLAG_REPEAT|PM_PWM_LUT_REVERSE)
#define PATTERN_FLAG_BLINK          /*0x19*/ (PM_PWM_LUT_LOOP|PM_PWM_LUT_PAUSE_HI_EN|PM_PWM_LUT_PAUSE_LO_EN)

// Index in a pattern data structure
#define PATTERN_INDEX_RED_START     PATTERN_SIZE_LUT
#define PATTERN_INDEX_RED_LENGTH    PATTERN_SIZE_LUT+1
#define PATTERN_INDEX_GREEN_START   PATTERN_SIZE_LUT+2
#define PATTERN_INDEX_GREEN_LENGTH  PATTERN_SIZE_LUT+3
#define PATTERN_INDEX_BLUE_START    PATTERN_SIZE_LUT+4
#define PATTERN_INDEX_BLUE_LENGTH   PATTERN_SIZE_LUT+5
#define PATTERN_INDEX_PAUSE_LO      PATTERN_SIZE_LUT+6
#define PATTERN_INDEX_PAUSE_HI      PATTERN_SIZE_LUT+7
#define PATTERN_INDEX_PAUSE_STEP    PATTERN_SIZE_LUT+8
#define PATTERN_INDEX_FLAGS         PATTERN_SIZE_LUT+9

// PATTERN_SCENARIO_DEFAULT_OFF (#0)
static int qpnp_pattern_default_off [] = {
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,

	0, 0,
	0, 0,
	0, 0,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_DEFAULT,
	PATTERN_FLAG_ONESHOT
};

// PATTERN_SCENARIO_POWER_ON (#1)
static int qpnp_pattern_power_on [] = {
	0,   0,   0,  0,   0,   0,   78  ,82,
	99,  86,  68, 47,  17,  0,   0,   0,
	0,   0,   41, 128, 179, 217, 242, 255,
	142, 122, 65, 45,  10,  0,   0,   0,
	10,  24,  45, 70,  49,  60,  0,   0,
	65,  56,  64, 44,  36,  20,  10,  3,

	32, 16,
	16, 16,
	0,  16,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_POWERONOFF,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_POWER_OFF (#6)
static int qpnp_pattern_power_off [] = {
	0,   0,   0,  0,   0,   0,   78  ,82,
	99,  86,  68, 47,  17,  0,   0,   0,
	0,   0,   41, 128, 179, 217, 242, 255,
	142, 122, 65, 45,  10,  0,   0,   0,
	10,  24,  45, 70,  49,  60,  0,   0,
	65,  56,  64, 44,  36,  20,  10,  3,

	32, 16,
	16, 16,
	0,  16,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_POWERONOFF,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_LCD_ON (#2)
static int qpnp_pattern_lcd_on[] = {
	/* GREEN/BLUE : 16 */
	0,   0,   0,   0,   0,   0,   0,  0,
	0,   0,   0,   0,   0,   0,   0,  0,
	255, 255, 255, 255, 255, 173, 58, 0,
	0,   0,   0,   0,   0,   0,   0,  0,
	255, 255, 255, 255, 255, 173, 58, 0,
	0,   0,   0,   0,   0,   0,   0,  0,

	0,  0,
	16, 10,
	16, 10,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_DEFAULT,
	PATTERN_FLAG_ONESHOT
};

// PATTERN_SCENARIO_CHARGING (#3)
static int qpnp_pattern_charging[] = {
	/* RED/GREEN/BLUE : 15 */
	0,   1,   1,   2,   3,   4,   6,   8,
	11,  14,  18,  21,  23,  25,  25,  25,
	0,   0,   1,   1,   2,   3,   4,   5,
	7,   9,   11,  13,  14,  15,  15,  15,
	3,   6,   15,  22,  33,  45,  62,  85,
	115, 145, 180, 213, 239, 255, 255, 255,

	33, 15,
	17, 15,
	1,  15,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_HI_CHARING, PATTERN_STEP_CHARGING,
	PATTERN_FLAG_WAVEFORM
};

// PATTERN_SCENARIO_CHARGING_FULL (#4)
static int qpnp_pattern_charging_full [] = {
	76, 76, 76, 76, 76, 76, 76, 76,
	76, 76, 76, 76, 76, 76, 76, 76,
	76, 76, 76, 76, 76, 76, 76, 76,
	76, 76, 76, 76, 76, 76, 76, 76,
	76, 76, 76, 76, 76, 76, 76, 76,
	76, 76, 76, 76, 76, 76, 76, 76,

	0,  0,
	0,  48,
	0,  0,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_FULL_CHARGING,
	PATTERN_FLAG_ONESHOT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_FAVORITE (#36)
static int qpnp_pattern_missed_noti_favorite [] = {
	/* This pattern is deprecated. */
	0,   0, 5, 33, 74, 148, 201, 242,
	255, 0, 0, 0,  0,  0,   0,   0,
	0,   0, 1, 3,  7,  15,  20,  24,
	25,  0, 0, 0,  0,  0,   0,   0,
	0,   0, 4, 23, 52, 104, 141, 170,
	179, 0, 0, 0,  0,  0,   0,   0,

	33, 8,
	17, 8,
	1,  8,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_DEFAULT,
	PATTERN_FLAG_WAVEFORM
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_URGENT (#37)
static int qpnp_pattern_missed_noti_urgent [] = {
	/* RED/GREEN/BLUE : 14 */
	0,   0,   15,  15,  14,  0,   0, 15,
	15,  14,  0,   0,   15,  14,  0, 0,
	0,   0,   25,  25,  24,  0,   0, 25,
	25,  24,  0,   0,   25,  24,  0, 0,
	0,   0,   255, 250, 240, 0,   0, 255,
	250, 240, 0,   0,   250, 240, 0, 0,

	34, 14,
	18, 14,
	2,  14,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_HI_MISSED_NOTI, PATTERN_STEP_NOTI,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_GREEN (#7)
static int qpnp_pattern_missed_noti_green [] = {
	/* G : 14 */
	0, 0,   255, 252, 200, 150, 0, 0,
	0, 255, 252, 200, 150, 0,   0, 0,
	0, 0,   0,   0,   0,   0,   0, 0,
	0, 0,   0,   0,   0,   0,   0, 0,
	0, 0,   0,   0,   0,   0,   0, 0,
	0, 0,   0,   0,   0,   0,   0, 0,

	0, 0,
	0, 16,
	0, 0,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_HI_MISSED_NOTI, PATTERN_STEP_NOTI,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_GREEN (#39)
static int qpnp_pattern_missed_noti_green_once [] = {
	/* G : 14 */
	0, 0,   255, 252, 200, 150, 0, 0,
	0, 255, 252, 200, 150, 0,   0, 0,
	0, 0,   0,   0,   0,   0,   0, 0,
	0, 0,   0,   0,   0,   0,   0, 0,
	0, 0,   0,   0,   0,   0,   0, 0,
	0, 0,   0,   0,   0,   0,   0, 0,

	0, 0,
	0, 16,
	0, 0,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_NOTI,
	PATTERN_FLAG_ONESHOT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_BLUE (#18)
static int qpnp_pattern_missed_noti_blue [] = {
	0,   0,   0, 0, 0, 0,   0,   0,
	0,   0,   0, 0, 0, 0,   0,   0,
	77,  74,  0, 0, 0, 77,  74,  0,
	0,   0,   0, 0, 0, 0,   0,   0,
	255, 250, 0, 0, 0, 255, 250, 0,
	0,   0,   0, 0, 0, 0,   0,   0,

	0,  0,
	16, 10,
	32, 10,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_HI_STEP_NOTI, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_BLUE (#41)
static int qpnp_pattern_missed_noti_blue_once [] = {
	0,   0,   0, 0, 0, 0,   0,   0,
	0,   0,   0, 0, 0, 0,   0,   0,
	77,  74,  0, 0, 0, 77,  74,  0,
	0,   0,   0, 0, 0, 0,   0,   0,
	255, 250, 0, 0, 0, 255, 250, 0,
	0,   0,   0, 0, 0, 0,   0,   0,

	0,  0,
	16, 10,
	32, 10,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_ONESHOT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PINK (#17)
static int qpnp_pattern_missed_noti_pink [] = {
	0, 65,  63,  0, 0, 0, 65,  63,
	0, 0,   0,   0, 0, 0, 0,   0,
	0, 65,  63,  0, 0, 0, 65,  63,
	0, 0,   0,   0, 0, 0, 0,   0,
	0, 255, 250, 0, 0, 0, 255, 250,
	0, 0,   0,   0, 0, 0, 0,   0,

	33, 10,
	17, 10,
	1,  10,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_HI_STEP_NOTI, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_PINK (#40)
static int qpnp_pattern_missed_noti_pink_once [] = {
	0, 65,  63,  0, 0, 0, 65,  63,
	0, 0,   0,   0, 0, 0, 0,   0,
	0, 65,  63,  0, 0, 0, 65,  63,
	0, 0,   0,   0, 0, 0, 0,   0,
	0, 255, 250, 0, 0, 0, 255, 250,
	0, 0,   0,   0, 0, 0, 0,   0,

	33, 10,
	17, 10,
	1,  10,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_ONESHOT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_YELLOW (#20)
static int qpnp_pattern_missed_noti_yellow [] = {
	0,   0,   0, 0, 0, 0,   0,   0,
	0,   0,   0, 0, 0, 0,   0,   0,
	230, 226, 0, 0, 0, 230, 226, 0,
	0,   0,   0, 0, 0, 0,   0,   0,
	255, 250, 0, 0, 0, 255, 250, 0,
	0,   0,   0, 0, 0, 0,   0,   0,

	32, 10,
	16, 10,
	0,  0,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_HI_STEP_NOTI, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_YELLOW (#43)
static int qpnp_pattern_missed_noti_yellow_once [] = {
	0,   0,   0, 0, 0, 0,   0,   0,
	0,   0,   0, 0, 0, 0,   0,   0,
	230, 226, 0, 0, 0, 230, 226, 0,
	0,   0,   0, 0, 0, 0,   0,   0,
	255, 250, 0, 0, 0, 255, 250, 0,
	0,   0,   0, 0, 0, 0,   0,   0,

	32, 10,
	16, 10,
	0,  0,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_ONESHOT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_ORANGE (#19)
static int qpnp_pattern_missed_noti_orange [] = {
	0,   0,   0, 0, 0, 0,   0,   0,
	0,   0,   0, 0, 0, 0,   0,   0,
	102, 99,  0, 0, 0, 102, 99,  0,
	0,   0,   0, 0, 0, 0,   0,   0,
	255, 250, 0, 0, 0, 255, 250, 0,
	0,   0,   0, 0, 0, 0,   0,   0,

	32, 10,
	16, 10,
	0,  0,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_HI_STEP_NOTI, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_ORANGE (#42)
static int qpnp_pattern_missed_noti_orange_once [] = {
	0,   0,   0, 0, 0, 0,   0,   0,
	0,   0,   0, 0, 0, 0,   0,   0,
	102, 99,  0, 0, 0, 102, 99,  0,
	0,   0,   0, 0, 0, 0,   0,   0,
	255, 250, 0, 0, 0, 255, 250, 0,
	0,   0,   0, 0, 0, 0,   0,   0,

	32, 10,
	16, 10,
	0,  0,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_ONESHOT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_TURQUOISE (#29)
static int qpnp_pattern_missed_noti_turquoise [] = {
	0,   0,   0, 0, 0, 0,   0,   0,
	0,   0,   0, 0, 0, 0,   0,   0,
	255, 250, 0, 0, 0, 255, 250, 0,
	0,   0,   0, 0, 0, 0,   0,   0,
	122, 119, 0, 0, 0, 122, 119, 0,
	0,   0,   0, 0, 0, 0,   0,   0,

	0,  0,
	16, 10,
	32, 10,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_HI_STEP_NOTI, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_TURQUOISE (#44)
static int qpnp_pattern_missed_noti_turquoise_once [] = {
	0,   0,   0, 0, 0, 0,   0,   0,
	0,   0,   0, 0, 0, 0,   0,   0,
	255, 250, 0, 0, 0, 255, 250, 0,
	0,   0,   0, 0, 0, 0,   0,   0,
	122, 119, 0, 0, 0, 122, 119, 0,
	0,   0,   0, 0, 0, 0,   0,   0,

	0,  0,
	16, 10,
	32, 10,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_ONESHOT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PURPLE (#30)
static int qpnp_pattern_missed_noti_purple [] = {
	0, 255, 250, 0, 0, 0, 255, 250,
	0, 0,   0,   0, 0, 0, 0,   0,
	0, 25,  25,  0, 0, 0, 25,  25,
	0, 0,   0,   0, 0, 0, 0,   0,
	0, 179, 177, 0, 0, 0, 179, 177,
	0, 0,   0,   0, 0, 0, 0,   0,

	33, 10,
	17, 10,
	1,  10,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_HI_STEP_NOTI, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_PURPLE (#45)
static int qpnp_pattern_missed_noti_purple_once [] = {
	0, 255, 250, 0, 0, 0, 255, 250,
	0, 0,   0,   0, 0, 0, 0,   0,
	0, 25,  25,  0, 0, 0, 25,  25,
	0, 0,   0,   0, 0, 0, 0,   0,
	0, 179, 177, 0, 0, 0, 179, 177,
	0, 0,   0,   0, 0, 0, 0,   0,

	33, 10,
	17, 10,
	1,  10,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_ONESHOT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_RED (#31)
static int qpnp_pattern_missed_noti_red [] = {
	0, 0,   15,  15,  12,  9,   0, 0,
	0, 15,  15,  12,  9,   0,   0, 0,
	0, 0,   25,  25,  20,  15,  0, 0,
	0, 25,  25,  20,  15,  0,   0, 0,
	0, 0,   255, 252, 200, 150, 0, 0,
	0, 255, 252, 200, 150, 0,   0, 0,

	34, 14,
	18, 14,
	2,  14,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_HI_MISSED_NOTI, PATTERN_STEP_NOTI,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_RED (#46)
static int qpnp_pattern_missed_noti_red_once [] = {
	0, 0,   15,  15,  12,  9,   0, 0,
	0, 15,  15,  12,  9,   0,   0, 0,
	0, 0,   25,  25,  20,  15,  0, 0,
	0, 25,  25,  20,  15,  0,   0, 0,
	0, 0,   255, 252, 200, 150, 0, 0,
	0, 255, 252, 200, 150, 0,   0, 0,

	34, 14,
	18, 14,
	2,  14,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_NOTI,
	PATTERN_FLAG_ONESHOT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_LIME (#32)
static int qpnp_pattern_missed_noti_lime [] = {
	0,   0,   0, 0, 0, 0,   0,   0,
	0,   0,   0, 0, 0, 0,   0,   0,
	255, 250, 0, 0, 0, 255, 250, 0,
	0,   0,   0, 0, 0, 0,   0,   0,
	135, 131, 0, 0, 0, 135, 131, 0,
	0,   0,   0, 0, 0, 0,   0,   0,

	32, 10,
	16, 10,
	0,  0,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_HI_STEP_NOTI, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_LIME (#47)
static int qpnp_pattern_missed_noti_lime_once [] = {
	0,   0,   0, 0, 0, 0,   0,   0,
	0,   0,   0, 0, 0, 0,   0,   0,
	255, 250, 0, 0, 0, 255, 250, 0,
	0,   0,   0, 0, 0, 0,   0,   0,
	135, 131, 0, 0, 0, 135, 131, 0,
	0,   0,   0, 0, 0, 0,   0,   0,

	32, 10,
	16, 10,
	0,  0,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_HI_NOTI,
	PATTERN_FLAG_ONESHOT
};

// PATTEN_SCENARIO_MISSED_NOTI_SECRETMODE_REPEAT_CYAN (#91)
static int qpnp_pattern_missed_noti_secretmode_cyan [] = {
	255, 255, 250, 228, 127, 30,  0,   0,
	0,   0,   0,   0,   0,   255, 255, 0,
	0,   0,   255, 255, 0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,

	 0,  0,
	 0, 24,
	 0, 24,

	 PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_HI_SECRETMODE, PATTERN_STEP_NOTI,
	 PATTERN_FLAG_REPEAT
};

// PATTEN_SCENARIO_MISSED_NOTI_SECRETMODE_ONESHOT_CYAN (#92)
static int qpnp_pattern_missed_noti_secretmode_cyan_once [] = {
	255, 255, 250, 228, 127, 30,  0,   0,
	0,   0,   0,   0,   0,   255, 255, 0,
	0,   0,   255, 255, 0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,

	 0,  0,
	 0, 24,
	 0, 24,

	 PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_HI_SECRETMODE, PATTERN_STEP_NOTI,
	 PATTERN_FLAG_ONESHOT
};

/* For TMUS operator*/
#define TMUS_PATTERN_LENGTH	8
#define TMUS_RAMP_STEP_MSEC	230	/* 60*1000/(TMUS_PATTERN_LENGTH-1+255) */
#define TMUS_HI_PAUSE_MSEC	(60*1000-(TMUS_RAMP_STEP_MSEC*(TMUS_PATTERN_LENGTH-1))/TMUS_RAMP_STEP_MSEC )

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_GREEN_TMUS (#207)
static int qpnp_pattern_missed_noti_green_tmus [] = {
	  0, 0, 0,   0, 0, 0, 0, 0,
	255, 0, 0, 255, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,

	0,                     0,
	TMUS_PATTERN_LENGTH,   TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH*2, 0,

	PATTERN_PAUSE_DISABLED, TMUS_HI_PAUSE_MSEC, TMUS_RAMP_STEP_MSEC,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PINK (#217)
static int qpnp_pattern_missed_noti_pink_tmus [] = {
	255, 0, 0, 255, 0, 0, 0, 0,
	 65, 0, 0,  65, 0, 0, 0, 0,
	 65, 0, 0,  65, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,

	0,                     TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH,   TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH*2, TMUS_PATTERN_LENGTH,

	PATTERN_PAUSE_DISABLED, TMUS_HI_PAUSE_MSEC, TMUS_RAMP_STEP_MSEC,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_BLUE (#218)
static int qpnp_pattern_missed_noti_blue_tmus [] = {
	  0, 0, 0,   0, 0, 0, 0, 0,
	 77, 0, 0,  77, 0, 0, 0, 0,
	255, 0, 0, 255, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,

	0,                     0,
	TMUS_PATTERN_LENGTH,   TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH*2, TMUS_PATTERN_LENGTH,

	PATTERN_PAUSE_DISABLED, TMUS_HI_PAUSE_MSEC, TMUS_RAMP_STEP_MSEC,
	PATTERN_FLAG_REPEAT
};
// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_ORANGE (#219)
static int qpnp_pattern_missed_noti_orange_tmus [] = {
	255, 0, 0, 255, 0, 0, 0, 0,
	102, 0, 0, 102, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,

	0,                     TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH,   TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH*2, 0,

	PATTERN_PAUSE_DISABLED, TMUS_HI_PAUSE_MSEC, TMUS_RAMP_STEP_MSEC,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_YELLOW (#220)
static int qpnp_pattern_missed_noti_yellow_tmus [] = {
	255, 0, 0, 255, 0, 0, 0, 0,
	230, 0, 0, 230, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,

	0,                     TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH,   TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH*2, 0,

	PATTERN_PAUSE_DISABLED, TMUS_HI_PAUSE_MSEC, TMUS_RAMP_STEP_MSEC,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_TURQUOISE (#229)
static int qpnp_pattern_missed_noti_turquoise_tmus [] = {
	  0, 0, 0,   0, 0, 0, 0, 0,
	255, 0, 0, 255, 0, 0, 0, 0,
	122, 0, 0, 122, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,

	0,                     0,
	TMUS_PATTERN_LENGTH,   TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH*2, TMUS_PATTERN_LENGTH,

	PATTERN_PAUSE_DISABLED, TMUS_HI_PAUSE_MSEC, TMUS_RAMP_STEP_MSEC,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PURPLE (#230)
static int qpnp_pattern_missed_noti_purple_tmus [] = {
	179, 0, 0, 179, 0, 0, 0, 0,
	 25, 0, 0,  25, 0, 0, 0, 0,
	255, 0, 0, 255, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,

	0,                     TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH,   TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH*2, TMUS_PATTERN_LENGTH,

	PATTERN_PAUSE_DISABLED, TMUS_HI_PAUSE_MSEC, TMUS_RAMP_STEP_MSEC,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_RED (#231)
static int qpnp_pattern_missed_noti_red_tmus [] = {
	255, 0, 0, 255, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,

	0,                     TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH,   0,
	TMUS_PATTERN_LENGTH*2, 0,

	PATTERN_PAUSE_DISABLED, TMUS_HI_PAUSE_MSEC, TMUS_RAMP_STEP_MSEC,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_LIME (#232)
static int qpnp_pattern_missed_noti_lime_tmus [] = {
	135, 0, 0, 135, 0, 0, 0, 0,
	255, 0, 0, 255, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,
	  0, 0, 0,   0, 0, 0, 0, 0,

	0,                     TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH,   TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH*2, 0,

	PATTERN_PAUSE_DISABLED, TMUS_HI_PAUSE_MSEC, TMUS_RAMP_STEP_MSEC,
	PATTERN_FLAG_REPEAT
};

// PATTERN_SCENARIO_MISSED_NOTI_REPEAT_URGENT (#237)
static int qpnp_pattern_missed_noti_urgent_tmus [] = {
	255, 0, 255, 0, 0, 0, 0, 0,
	  0, 0,   0, 0, 0, 0, 0, 0,
	  0, 0,   0, 0, 0, 0, 0, 0,
	  0, 0,   0, 0, 0, 0, 0, 0,
	  0, 0,   0, 0, 0, 0, 0, 0,
	  0, 0,   0, 0, 0, 0, 0, 0,

	0,                     TMUS_PATTERN_LENGTH,
	TMUS_PATTERN_LENGTH,   0,
	TMUS_PATTERN_LENGTH*2, 0,

	PATTERN_PAUSE_DISABLED, TMUS_HI_PAUSE_MSEC, TMUS_RAMP_STEP_MSEC,
	PATTERN_FLAG_REPEAT,
};

// PATTERN_SCENARIO_SPRINT_POWER_ON (#112)
static int qpnp_pattern_sprint_power_on [] = {
	2,   5,   10,  18,  27,  37,  47,  59,
	74,  92,  113, 133, 150, 162, 174, 185,
	195, 205, 205, 205, 0,   0,   0,   0,
	3,   6,   13,  22,  33,  46,  59,  74,
	92,  115, 140, 166, 186, 201, 217, 230,
	242, 255, 255, 255, 0,   0,   0,   0,

	1,  19,
	25, 19,
	0,  0,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_SPRINT_POWERONOFF,
	PATTERN_FLAG_WAVEFORM
};

// PATTERN_SCENARIO_SPRINT_POWER_OFF (#113)
static int qpnp_pattern_sprint_power_off [] = {
	2,   5,   10,  18,  27,  37,  47,  59,
	74,  92,  113, 133, 150, 162, 174, 185,
	195, 205, 205, 205, 0,   0,   0,   0,
	3,   6,   13,  22,  33,  46,  59,  74,
	92,  115, 140, 166, 186, 201, 217, 230,
	242, 255, 255, 255, 0,   0,   0,   0,

	1,  19,
	25, 19,
	0,  0,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_SPRINT_POWERONOFF,
	PATTERN_FLAG_WAVEFORM
};

// PATTERN_SCENARIO_SPRINT_IN_USE (#114)
static int qpnp_pattern_sprint_in_use [] = {
	0,   0,   2,   7,   12,  20,  28,  36,
	43,  58,  74,  94,  115, 135, 153, 171,
	192, 207, 219, 229, 238, 245, 250, 255,
	0,   0,   2,   6,   10,  16,  23,  29,
	35,  47,  60,  76,  93,  108, 123, 137,
	154, 166, 176, 184, 191, 197, 201, 205,

	26, 22,
	2,  22,
	0,  0,

	PATTERN_PAUSE_LO_SPRINT_INUSE, PATTERN_PAUSE_HI_SPRINT_INUSE, PATTERN_STEP_SPRINT_INUSE,
	PATTERN_FLAG_WAVEFORM
};

// PATTERN_SCENARIO_SPRINT_IN_CALL (#115)
static int qpnp_pattern_sprint_in_call [] = {
	0,   1,   2,   3,   4,   4,   4,   3,
	3,   2,   2,   1,   0,   0,   0,   0,
	17,  54,  100, 145, 205, 206, 205, 170,
	150, 119, 77,  44,  17,  9,   0,   0,
	12,  37,  68,  99,  140, 140, 140, 116,
	102, 81,  52,  30,  12,  6,   0,   0,

	32, 16,
	16, 16,
	0,  16,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_SPRINT_INCALL,
	PATTERN_FLAG_WAVEFORM
};

// PATTERN_SCENARIO_5G_DATA (#49)
static int qpnp_pattern_5g_data [] = {
	0,   3,   7,   12,  15,  5,  15,  5,
	15,  13,  10,  6,   3,   1,  0,   0,
	0,   5,   12,  20,  25,  8,  25,  8,
	25,  21,  16,  10,  5,   2,  0,   0,
	0,   46,  122, 204, 255, 77, 255, 77,
	255, 217, 166, 102, 46,  18, 0,   0,

	33, 15,
	17, 15,
	1,  15,

	PATTERN_PAUSE_DISABLED, PATTERN_PAUSE_DISABLED, PATTERN_STEP_5G_DATA,
	PATTERN_FLAG_ONESHOT
};


/* In the hidden-menu,
 * the pattern numbers 0~15 of seek-bar are mapped to ...
 *
 *  	PATTERN_SCENARIO_DEFAULT_OFF                    = 0,
 *  	PATTERN_SCENARIO_POWER_ON                       = 1,
 *  	PATTERN_SCENARIO_LCD_ON                         = 2,
 *  	PATTERN_SCENARIO_CHARGING                       = 3,
 *  	PATTERN_SCENARIO_CHARGING_FULL                  = 4,
 *  	*PATTERN_SCENARIO_BLANK_5                       = 5,
 *  	PATTERN_SCENARIO_POWER_OFF                      = 6,
 *  	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_GREEN       = 7,
 *  	*PATTERN_SCENARIO_BLANK_8                       = 8,
 *  	*PATTERN_SCENARIO_BLANK_9                       = 9,
 *  	*PATTERN_SCENARIO_BLANK_10                      = 10,
 *  	*PATTERN_SCENARIO_BLANK_11                      = 11,
 *  	*PATTERN_SCENARIO_BLANK_12                      = 12,
 *  	*PATTERN_SCENARIO_BLANK_13                      = 13,
 *  	*PATTERN_SCENARIO_BLANK_14                      = 14,
 *  	*PATTERN_SCENARIO_BLANK_15                      = 15
 */
enum qpnp_pattern_scenario {
	PATTERN_SCENARIO_DEFAULT_OFF                    = 0,

	PATTERN_SCENARIO_POWER_ON                       = 1,
	PATTERN_SCENARIO_POWER_OFF                      = 6,
	PATTERN_SCENARIO_LCD_ON                         = 2,

	PATTERN_SCENARIO_CHARGING                       = 3,
	PATTERN_SCENARIO_CHARGING_FULL                  = 4,

	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_FAVORITE    = 36,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_URGENT      = 37,

	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_GREEN       = 7,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_BLUE        = 18,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PINK        = 17,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_YELLOW      = 20,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_ORANGE      = 19,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_TURQUOISE   = 29,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PURPLE      = 30,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_RED         = 31,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_LIME        = 32,
	PATTERN_SCENARIO_MISSED_NOTI_SECRETMODE_REPEAT_CYAN = 91,

	PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_GREEN      = 39,
	PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_BLUE       = 41,
	PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_PINK       = 40,
	PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_YELLOW     = 43,
	PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_ORANGE     = 42,
	PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_TURQUOISE  = 44,
	PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_PURPLE     = 45,
	PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_RED        = 46,
	PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_LIME       = 47,
	PATTERN_SCENARIO_MISSED_NOTI_SECRETMODE_ONESHOT_CYAN = 92,

	PATTERN_SCENARIO_HIDDEN_MENU_BLANK_5            = 5,
	PATTERN_SCENARIO_HIDDEN_MENU_BLANK_8            = 8,
	PATTERN_SCENARIO_HIDDEN_MENU_BLANK_9            = 9,
	PATTERN_SCENARIO_HIDDEN_MENU_BLANK_10           = 10,
	PATTERN_SCENARIO_HIDDEN_MENU_BLANK_11           = 11,
	PATTERN_SCENARIO_HIDDEN_MENU_BLANK_12           = 12,
	PATTERN_SCENARIO_HIDDEN_MENU_BLANK_13           = 13,
	PATTERN_SCENARIO_HIDDEN_MENU_BLANK_14           = 14,
	PATTERN_SCENARIO_HIDDEN_MENU_BLANK_15           = 15,

	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_GREEN_TMUS       = 207,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PINK_TMUS        = 217,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_BLUE_TMUS        = 218,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_ORANGE_TMUS      = 219,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_YELLOW_TMUS      = 220,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_TURQUOISE_TMUS   = 229,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PURPLE_TMUS      = 230,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_RED_TMUS         = 231,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_LIME_TMUS        = 232,
	PATTERN_SCENARIO_MISSED_NOTI_REPEAT_URGENT_TMUS      = 237,

	PATTERN_SCENARIO_SPRINT_POWER_ON                = 112,
	PATTERN_SCENARIO_SPRINT_POWER_OFF               = 113,
	PATTERN_SCENARIO_SPRINT_IN_USE                  = 114,
	PATTERN_SCENARIO_SPRINT_IN_CALL                 = 115,

	PATTERN_SCENARIO_5G_DATA                        = 49,
};

static int* qpnp_pattern_parameter [] = {
	qpnp_pattern_default_off,                //  0 <- PATTERN_SCENARIO_DEFAULT_OFF
	qpnp_pattern_power_on,                   //  1 <- PATTERN_SCENARIO_POWER_ON
	qpnp_pattern_power_off,                  //  2 <- PATTERN_SCENARIO_POWER_OFF
	qpnp_pattern_lcd_on,                     //  3 <- PATTERN_SCENARIO_LCD_ON
	qpnp_pattern_charging,                   //  4 <- PATTERN_SCENARIO_CHARGING
	qpnp_pattern_charging_full,              //  5 <- PATTERN_SCENARIO_CHARGING_FULL
	qpnp_pattern_missed_noti_favorite,       //  6 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_FAVORITE
	qpnp_pattern_missed_noti_urgent,         //  7 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_URGENT
	qpnp_pattern_missed_noti_green,          //  8 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_GREEN
	qpnp_pattern_missed_noti_blue,           //  9 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_BLUE
	qpnp_pattern_missed_noti_pink,           // 10 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PINK
	qpnp_pattern_missed_noti_yellow,         // 11 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_YELLOW
	qpnp_pattern_missed_noti_orange,         // 12 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_ORANGE
	qpnp_pattern_missed_noti_turquoise,      // 13 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_TURQUOISE
	qpnp_pattern_missed_noti_purple,         // 14 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PURPLE
	qpnp_pattern_missed_noti_red,            // 15 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_RED
	qpnp_pattern_missed_noti_lime,           // 16 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_LIME
	qpnp_pattern_missed_noti_green_once,     // 17 <- PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_GREEN
	qpnp_pattern_missed_noti_blue_once,      // 18 <- PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_BLUE
	qpnp_pattern_missed_noti_pink_once,      // 19 <- PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_PINK
	qpnp_pattern_missed_noti_yellow_once,    // 20 <- PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_YELLOW
	qpnp_pattern_missed_noti_orange_once,    // 21 <- PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_ORANGE
	qpnp_pattern_missed_noti_turquoise_once, // 22 <- PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_TURQUOISE
	qpnp_pattern_missed_noti_purple_once,    // 23 <- PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_PURPLE
	qpnp_pattern_missed_noti_red_once,       // 24 <- PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_RED
	qpnp_pattern_missed_noti_lime_once,      // 25 <- PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_LIME

	qpnp_pattern_missed_noti_secretmode_cyan,     // 26 <- PATTERN_SCENARIO_MISSED_NOTI_SECRETMODE_REPEAT_CYAN
	qpnp_pattern_missed_noti_secretmode_cyan_once, // 27 <- PATTERN_SCENARIO_MISSED_NOTI_SECRETMODE_ONESHOT_CYAN

	qpnp_pattern_missed_noti_green_tmus,          // 28 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_GREEN
	qpnp_pattern_missed_noti_pink_tmus,           // 29 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PINK
	qpnp_pattern_missed_noti_blue_tmus,           // 30 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_BLUE
	qpnp_pattern_missed_noti_orange_tmus,         // 31 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_ORANGE
	qpnp_pattern_missed_noti_yellow_tmus,         // 32 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_YELLOW
	qpnp_pattern_missed_noti_turquoise_tmus,      // 33 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_TURQUOISE
	qpnp_pattern_missed_noti_purple_tmus,         // 34 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PURPLE
	qpnp_pattern_missed_noti_red_tmus,            // 35 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_RED
	qpnp_pattern_missed_noti_lime_tmus,           // 36 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_LIME
	qpnp_pattern_missed_noti_urgent_tmus,	      // 37 <- PATTERN_SCENARIO_MISSED_NOTI_REPEAT_URGENT
	qpnp_pattern_sprint_power_on,                 // 38 <- PATTERN_SCENARIO_SPRINT_POWER_ON
	qpnp_pattern_sprint_power_off,                // 39 <- PATTERN_SCENARIO_SPRINT_POWER_OFF
	qpnp_pattern_sprint_in_use,                   // 40 <- PATTERN_SCENARIO_SPRINT_IN_USE
	qpnp_pattern_sprint_in_call,                  // 41 <- PATTERN_SCENARIO_SPRINT_IN_CALL
	qpnp_pattern_5g_data,                         // 42 <- PATTERN_SCENARIO_5G_DATA
};

static inline char* qpnp_pattern_scenario_name(enum qpnp_pattern_scenario scenario)
{
	switch (scenario)
	{
	case PATTERN_SCENARIO_DEFAULT_OFF                   :
		return "PATTERN_SCENARIO_DEFAULT_OFF";
	case PATTERN_SCENARIO_POWER_ON                      :
		return "PATTERN_SCENARIO_POWER_ON";
	case PATTERN_SCENARIO_POWER_OFF                     :
		return "PATTERN_SCENARIO_POWER_OFF";
	case PATTERN_SCENARIO_LCD_ON                        :
		return "PATTERN_SCENARIO_LCD_ON";
	case PATTERN_SCENARIO_CHARGING                      :
		return "PATTERN_SCENARIO_CHARGING";
	case PATTERN_SCENARIO_CHARGING_FULL                 :
		return "PATTERN_SCENARIO_CHARGING_FULL";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_FAVORITE   :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_FAVORITE";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_URGENT     :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_URGENT";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_GREEN      :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_GREEN";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_BLUE       :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_BLUE";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PINK       :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PINK";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_YELLOW     :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_YELLOW";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_ORANGE     :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_ORANGE";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_TURQUOISE  :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_TURQUOISE";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PURPLE     :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PURPLE";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_RED        :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_RED";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_LIME       :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_LIME";
	case PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_GREEN     :
		return "PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_GREEN";
	case PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_BLUE      :
		return "PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_BLUE";
	case PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_PINK      :
		return "PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_PINK";
	case PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_YELLOW    :
		return "PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_YELLOW";
	case PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_ORANGE    :
		return "PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_ORANGE";
	case PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_TURQUOISE :
		return "PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_TURQUOISE";
	case PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_PURPLE    :
		return "PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_PURPLE";
	case PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_RED       :
		return "PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_RED";
	case PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_LIME      :
		return "PATTERN_SCENARIO_MISSED_NOTI_ONESHOT_LIME";
	case PATTERN_SCENARIO_HIDDEN_MENU_BLANK_5           :
		return "PATTERN_SCENARIO_HIDDEN_MENU_BLANK_5";
	case PATTERN_SCENARIO_HIDDEN_MENU_BLANK_8           :
		return "PATTERN_SCENARIO_HIDDEN_MENU_BLANK_8";
	case PATTERN_SCENARIO_HIDDEN_MENU_BLANK_9           :
		return "PATTERN_SCENARIO_HIDDEN_MENU_BLANK_9";
	case PATTERN_SCENARIO_HIDDEN_MENU_BLANK_10          :
		return "PATTERN_SCENARIO_HIDDEN_MENU_BLANK_10";
	case PATTERN_SCENARIO_HIDDEN_MENU_BLANK_11          :
		return "PATTERN_SCENARIO_HIDDEN_MENU_BLANK_11";
	case PATTERN_SCENARIO_HIDDEN_MENU_BLANK_12          :
		return "PATTERN_SCENARIO_HIDDEN_MENU_BLANK_12";
	case PATTERN_SCENARIO_HIDDEN_MENU_BLANK_13          :
		return "PATTERN_SCENARIO_HIDDEN_MENU_BLANK_13";
	case PATTERN_SCENARIO_HIDDEN_MENU_BLANK_14          :
		return "PATTERN_SCENARIO_HIDDEN_MENU_BLANK_14";
	case PATTERN_SCENARIO_HIDDEN_MENU_BLANK_15          :
		return "PATTERN_SCENARIO_HIDDEN_MENU_BLANK_15";
	case PATTERN_SCENARIO_MISSED_NOTI_SECRETMODE_REPEAT_CYAN :
		return "PATTERN_SCENARIO_MISSED_NOTI_SECRETMODE_REPEAT_CYAN";
	case PATTERN_SCENARIO_MISSED_NOTI_SECRETMODE_ONESHOT_CYAN :
		return "PATTERN_SCENARIO_MISSED_NOTI_SECRETMODE_ONESHOT_CYAN";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_GREEN_TMUS  :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_GREEN_TMUS";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PINK_TMUS   :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PINK_TMUS";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_BLUE_TMUS   :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_BLUE_TMUS";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_ORANGE_TMUS   :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_ORANGE_TMUS";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_YELLOW_TMUS   :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_YELLOW_TMUS";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_TURQUOISE_TMUS   :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_TURQUOISE_TMUS";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PURPLE_TMUS   :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_PURPLE_TMUS";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_RED_TMUS   :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_RED_TMUS";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_LIME_TMUS   :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_LIME_TMUS";
	case PATTERN_SCENARIO_MISSED_NOTI_REPEAT_URGENT_TMUS   :
		return "PATTERN_SCENARIO_MISSED_NOTI_REPEAT_URGENT_TMUS";
	case PATTERN_SCENARIO_SPRINT_POWER_ON  :
		return "PATTERN_SCENARIO_SPRINT_POWER_ON";
	case PATTERN_SCENARIO_SPRINT_POWER_OFF  :
		return "PATTERN_SCENARIO_SPRINT_POWER_OFF";
	case PATTERN_SCENARIO_SPRINT_IN_USE  :
		return "PATTERN_SCENARIO_SPRINT_IN_USE";
	case PATTERN_SCENARIO_SPRINT_IN_CALL  :
		return "PATTERN_SCENARIO_SPRINT_IN_CALL";
	case PATTERN_SCENARIO_5G_DATA  :
		return "PATTERN_SCENARIO_5G_DATA";
	default :
		break;
	}

	return "PATTERN_SCENARIO_NOT_DEFINED";
}

//static ssize_t qpnp_pattern_select(const char* string_format, size_t string_size);
//static ssize_t qpnp_pattern_input(const char* string_format, size_t string_size);
//static ssize_t qpnp_pattern_blink(const char* string_format, size_t string_size);
//static ssize_t qpnp_pattern_onoff(const char* string_format, size_t string_size);
//static ssize_t qpnp_pattern_scale(const char* string_format, size_t string_size);
void qpnp_pattern_config(void);
extern struct qpnp_lpg_channel *qpnp_get_lpg_channel(struct pwm_chip *pwm_chip,
		struct pwm_device *pwm);
extern int qpnp_tri_led_set(struct qpnp_led_dev *led);
extern void qpnp_init_lut_pattern(struct qpnp_lpg_channel *lpg);

#endif /* __LEDS_QPNP_PATTERN_H_INCLUDED */
