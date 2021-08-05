/* sma6101.c -- sma6101 ALSA SoC Audio driver
 *
 * r020, 2019.02.18	- initial version  sma6101
 *
 * Copyright 2018 Silicon Mitus Corporation / Iron Device Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/version.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <asm/div64.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/thermal.h>
#include <linux/power_supply.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#if 0
#include <linux/qpnp/qpnp-adc.h>
#endif

#ifdef CONFIG_MACH_LGE
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include "../msm-cdc-pinctrl.h"
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <dsp/q6afe-v2.h>
#include <sound/soc-dapm.h>
#include "../../../../../drivers/extcon/extcon.h"
#endif

#include "sma6101.h"

//sar_backoff for piezo amp
#ifdef CONFIG_MACH_LGE
static struct extcon_dev* edev_get_sar_backoff = NULL;
static bool piezo_receiver=0;  // 0:disable 1:enable
#endif /* CONFIG_SND_SOC_TFA9872_STEREO */


#define CHECK_COMP_PERIOD_TIME 10 /* sec per HZ */
#define CHECK_FAULT_PERIOD_TIME 5 /* sec per HZ */

#define FIFO_BUFFER_SIZE 10
#define VBAT_TABLE_NUM 4


#define PLL_MATCH(_input_clk_name, _output_clk_name, _input_clk,\
		_post_n, _n, _f1, _f2, _f3_p_cp)\
{\
	.input_clk_name		= _input_clk_name,\
	.output_clk_name	= _output_clk_name,\
	.input_clk		= _input_clk,\
	.post_n			= _post_n,\
	.n			= _n,\
	.f1			= _f1,\
	.f2			= _f2,\
	.f3_p_cp		= _f3_p_cp,\
}

#define FDPEC_GAIN_MATCH(_fdpec_gain_name, _gain,\
		_ext_gain, _ext_gain2)\
{\
	.fdpec_gain_name	= _fdpec_gain_name,\
	.gain			= _gain,\
	.ext_gain		= _ext_gain,\
	.ext_gain2		= _ext_gain2,\
}

#define TEMP_GAIN_MATCH(_thermal_deg_name, _thermal_limit, _comp_gain,\
		_ocp_count, _hit_count, _activate)\
{\
	.thermal_deg_name	= _thermal_deg_name,\
	.thermal_limit		= _thermal_limit,\
	.comp_gain		= _comp_gain,\
	.ocp_count		= _ocp_count,\
	.hit_count		= _hit_count,\
	.activate		= _activate,\
}

#define VBAT_GAIN_MATCH(_vbat_level_name, _vbat_level,\
		_comp_gain)\
{\
	.lvl_name		= _vbat_level_name,\
	.vbat_level		= _vbat_level,\
	.comp_gain		= _comp_gain,\
}

enum sma6101_type {
	SMA6101,
};

/* PLL clock setting Table */
struct sma6101_pll_match {
	char *input_clk_name;
	char *output_clk_name;
	unsigned int input_clk;
	unsigned int post_n;
	unsigned int n;
	unsigned int f1;
	unsigned int f2;
	unsigned int f3_p_cp;
};

struct outside_status {
	unsigned int id;
	int thermal_deg;
	int batt_voltage_mV;
	int interval;
};

struct sma6101_temperature_match {
	char *thermal_deg_name;
	int thermal_limit;
	int comp_gain;
	unsigned int ocp_count;
	unsigned int hit_count;
	bool activate;
};

struct sma6101_vbat_gain_match {
	char *lvl_name;
	int vbat_level;
	int comp_gain;
};

#if defined(CONFIG_MACH_LGE)
#define Q6AFE_LPASS_OSR_CLK_24_P576_MHZ		0x1770000 // LG Define

//SEC MI2S MCLK SETTING
/*
SDM845 LPASS HW Revioson : LPASS_HW_VER_4_1_0
MCLK_1 / MCLK_2 / MCLK_3 are supported.
refer to g_generic_clk_map_table_v3 variable in clock_manager.cpp
*/
static struct afe_clk_set lpass_mclk = {
	Q6AFE_LPASS_CLK_CONFIG_API_VERSION,
	Q6AFE_LPASS_CLK_ID_MCLK_2,
	Q6AFE_LPASS_OSR_CLK_24_P576_MHZ,
	Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO,
	Q6AFE_LPASS_CLK_ROOT_DEFAULT,
	0,
};

enum pinctrl_pin_state {
	STATE_SEC_MI2S_MCLK_SLEEP = 0,
	STATE_SEC_MI2S_MCLK_ACTIVE,
};

struct msm_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *sec_mi2s_mclk_sleep;
	struct pinctrl_state *sec_mi2s_mclk_active;
	enum pinctrl_pin_state curr_state;
};

static const char *const pin_states[] = {"sec_mi2s_mclk_sleep","sec_mi2s_mclk_active"};
#endif

/* FDPEC gain setting Table */
struct sma6101_fdpec_gain_match {
	char *fdpec_gain_name;
	unsigned int gain;
	unsigned int ext_gain;
	unsigned int ext_gain2;
};
struct sma6101_priv {
	enum sma6101_type devtype;
	struct attribute_group *attr_grp;
	struct kobject *kobj;
	struct regmap *regmap;
	struct sma6101_pll_match *pll_matches;
	int num_of_pll_matches;
	struct sma6101_fdpec_gain_match *fdpec_gain_matches;
	int num_of_fdpec_gain_matches;
	struct sma6101_temperature_match *temp_match;
	int num_of_temperature_matches;
	unsigned int mclk_in;
	unsigned int sys_clk_id;
	unsigned int init_vol;
	bool amp_power_status;
	bool force_amp_power_down;
	bool stereo_two_chip;
	bool src_bypass;
	unsigned int fdpec_gain_control;
	unsigned int voice_music_class_h_mode;
	const uint32_t *eq_reg_array;
	const uint32_t *bo_reg_array;
	uint32_t eq_reg_array_len;
	uint32_t bo_reg_array_len;
	unsigned int format;
	struct device *dev;
	struct delayed_work check_thermal_vbat_work;
	struct delayed_work check_thermal_fault_work;
	int irq;
	int gpio_int;
	unsigned int rev_num;
	atomic_t irq_enabled;
	unsigned int ocp_count;
	struct thermal_zone_device *tz_sense;
	struct power_supply *batt_psy;
	struct kfifo data_fifo;
	int fifo_count;
	struct mutex lock;
	uint32_t threshold_level;
	long check_thermal_vbat_period;
	long check_thermal_vbat_enable;
	long check_thermal_fault_period;
	long check_thermal_fault_enable;
	long temp_table_number;
	long temp_limit;
	long temp_comp_gain;
	long temp_ocp_count;
	long temp_hit_count;
	long temp_activate;
	long enable_ocp_aging;
	long thermal_sense_opt;
	int lowbattery_status;
#ifdef CONFIG_MACH_LGE
	int trm_vbst1;
	int reset_gpio;
	struct msm_pinctrl_info pinctrl_info;
	struct clk *ln_bb_clk3;
	unsigned int voice_music_class_h_mode_for_volume_boost;
#endif
};

#if defined(CONFIG_MACH_LGE)
#define SMA6101_SPKMODE_MONO 	1
#define SMA6101_SPKMODE_STEREO 	4
#endif

static struct sma6101_pll_match sma6101_pll_matches[] = {
/* in_clk_name, out_clk_name, input_clk, post_n, n, f1, f2, f3_p_cp */
PLL_MATCH("1.536MHz",  "24.576MHz", 1536000,  0x07, 0xE0, 0x00, 0x00, 0x03),
PLL_MATCH("3.072MHz",  "24.576MHz", 3072000,  0x07, 0x70, 0x00, 0x00, 0x03),
PLL_MATCH("6.144MHz",  "24.576MHz", 6144000,  0x07, 0x70, 0x00, 0x00, 0x07),
PLL_MATCH("12.288MHz", "24.576MHz", 12288000, 0x07, 0x70, 0x00, 0x00, 0x0B),
PLL_MATCH("19.2MHz",   "24.343MHz", 19200000, 0x07, 0x71, 0x00, 0x00, 0x0A),
PLL_MATCH("24.576MHz", "24.576MHz", 24576000, 0x07, 0x70, 0x00, 0x00, 0x0F),
};

static struct sma6101_fdpec_gain_match sma6101_fdpec_gain_matches[] = {
/* fdpec gain, gain(0x13[4]), ext_gain(0x13[1]), ext_gain2(0x13[0]) */
FDPEC_GAIN_MATCH("x2.6", FDPEC_GAIN_6, FDPEC_EXT_GAIN_6, FDPEC_EXT_GAIN2_2P6),
FDPEC_GAIN_MATCH("x4",   FDPEC_GAIN_6, FDPEC_EXT_GAIN_4, FDPEC_EXT_GAIN2_6),
FDPEC_GAIN_MATCH("x6",   FDPEC_GAIN_6, FDPEC_EXT_GAIN_6, FDPEC_EXT_GAIN2_6),
FDPEC_GAIN_MATCH("x8",   FDPEC_GAIN_8, FDPEC_EXT_GAIN_6, FDPEC_EXT_GAIN2_6),
};

static const struct sma6101_vbat_gain_match sma6101_vbat_gain_matches[] = {
/* level name,level, comp gain*/
VBAT_GAIN_MATCH("Normal LVL", 3, 0x00),
VBAT_GAIN_MATCH("LVL 2", 2, 0x02),
VBAT_GAIN_MATCH("LVL 1", 1, 0x04),
VBAT_GAIN_MATCH("LVL 0", 0, 0x06),
};

#ifndef CONFIG_MACH_LGE
static struct sma6101_temperature_match sma6101_temperature_gain_matches[] = {
/* degree name, temp limit, comp gain, ocp count, hit count, activate */
TEMP_GAIN_MATCH("35", 350, 0x00, 0, 0, 1), /* normal */
TEMP_GAIN_MATCH("40", 400, 0x01, 0, 0, 1),
TEMP_GAIN_MATCH("45", 450, 0x02, 0, 0, 1),
TEMP_GAIN_MATCH("50", 500, 0x03, 0, 0, 1),
TEMP_GAIN_MATCH("55", 550, 0x04, 0, 0, 1),
TEMP_GAIN_MATCH("60", 600, 0x05, 0, 0, 1),
TEMP_GAIN_MATCH("65", 650, 0x06, 0, 0, 1),
TEMP_GAIN_MATCH("70", 700, 0x07, 0, 0, 1),
TEMP_GAIN_MATCH("75", 750, 0x08, 0, 0, 1),
TEMP_GAIN_MATCH("80", 800, 0x09, 0, 0, 1),
TEMP_GAIN_MATCH("85", 850, 0x0a, 0, 0, 1),
TEMP_GAIN_MATCH("90", 900, 0x0b, 0, 0, 1),
TEMP_GAIN_MATCH("95", 950, 0x0c, 0, 0, 1),
TEMP_GAIN_MATCH("100", 1000, 0xd, 0, 0, 1), /* max */
};
#else

static struct sma6101_temperature_match sma6101_temperature_gain_matches[] = {
/* degree name, temp limit, comp gain, ocp count, hit count, activate */
TEMP_GAIN_MATCH("42.5", 425, 0x00, 0, 0, 0), /* normal */
TEMP_GAIN_MATCH("48.8", 488, 0x01, 0, 0, 0),
TEMP_GAIN_MATCH("55.0", 550, 0x02, 0, 0, 1),
TEMP_GAIN_MATCH("61.3", 613, 0x03, 0, 0, 1),
TEMP_GAIN_MATCH("67.5", 675, 0x04, 0, 0, 1),
TEMP_GAIN_MATCH("73.8", 738, 0x05, 0, 0, 1),
TEMP_GAIN_MATCH("80.0", 800, 0x06, 0, 0, 1),
TEMP_GAIN_MATCH("86.3", 863, 0x07, 0, 0, 1),
TEMP_GAIN_MATCH("92.5", 925, 0x08, 0, 0, 1),
TEMP_GAIN_MATCH("98.8", 988, 0x09, 0, 0, 1),
TEMP_GAIN_MATCH("100.0", 1000, 0xd, 0, 0, 1), /* max */
};
#endif

static int sma6101_setup_fdpec_gain(struct snd_soc_codec *, unsigned int);
static int sma6101_startup(struct snd_soc_codec *);
static int sma6101_shutdown(struct snd_soc_codec *);
static int sma6101_thermal_compensation(struct sma6101_priv *sma6101,
					bool ocp_status);

/* Initial register value - {register, value}
 * EQ Band : 1 to 5 / 0x40 to 0x8A (15EA register for each EQ Band)
 * Currently all EQ Bands are flat frequency response
 */
static const struct reg_default sma6101_reg_def[] = {
	{ 0x00, 0x80 }, /* 0x00 SystemCTRL  */
	{ 0x01, 0x00 }, /* 0x01 InputCTRL1  */
	{ 0x02, 0x00 }, /* 0x02 InputCTRL2  */
	{ 0x03, 0x01 }, /* 0x03 InputCTRL3  */
	{ 0x04, 0x17 }, /* 0x04 InputCTRL4  */
	{ 0x05, 0xBA }, /* 0x05 BrownOut Set1  */
	{ 0x06, 0x7A }, /* 0x06 BrownOut Set2  */
	{ 0x07, 0x3A }, /* 0x07 BrownOut Set3  */
	{ 0x08, 0x2A }, /* 0x08 BrownOut Set4  */
	{ 0x09, 0x00 }, /* 0x09 OutputCTRL  */
	{ 0x0A, 0x58 }, /* 0x0A SPK_VOL  */
	{ 0x0B, 0x1A }, /* 0x0B BrownOut Set5  */
	{ 0x0C, 0x0A }, /* 0x0C BrownOut Set6  */
	{ 0x0D, 0xC2 }, /* 0x0D Class-H Control Level3  */
	{ 0x0E, 0xAF }, /* 0x0E MUTE_VOL_CTRL  */
	{ 0x0F, 0xA2 }, /* 0x0F Class-H Control Level4  */
	{ 0x10, 0x00 }, /* 0x10 SystemCTRL1  */
	{ 0x11, 0x00 }, /* 0x11 SystemCTRL2  */
	{ 0x12, 0x00 }, /* 0x12 SystemCTRL3  */
	{ 0x13, 0x04 }, /* 0x13 FDPEC Control1  */
	{ 0x14, 0x60 }, /* 0x14 Modulator  */
	{ 0x15, 0x01 }, /* 0x15 BassSpk1  */
	{ 0x16, 0x0F }, /* 0x16 BassSpk2  */
	{ 0x17, 0x0F }, /* 0x17 BassSpk3  */
	{ 0x18, 0x0F }, /* 0x18 BassSpk4  */
	{ 0x19, 0x00 }, /* 0x19 BassSpk5  */
	{ 0x1A, 0x00 }, /* 0x1A BassSpk6  */
	{ 0x1B, 0x00 }, /* 0x1B BassSpk7  */
	{ 0x1C, 0xC0 }, /* 0x1C BrownOut Protection16  */
	{ 0x1D, 0xB3 }, /* 0x1D BrownOut Protection17  */
	{ 0x1E, 0xA6 }, /* 0x1E BrownOut Protection18  */
	{ 0x1F, 0x99 }, /* 0x1F BrownOut Protection19  */
	{ 0x20, 0x00 }, /* 0x20 BrownOut Protection20  */
	{ 0x21, 0x01 }, /* 0x21 TDM RX  */
	{ 0x23, 0x19 }, /* 0x23 CompLim1  */
	{ 0x24, 0x00 }, /* 0x24 CompLim2  */
	{ 0x25, 0x00 }, /* 0x25 CompLim3  */
	{ 0x26, 0x04 }, /* 0x26 CompLim4  */
	{ 0x2B, 0x07 }, /* 0x2B EqMode  */
	{ 0x33, 0x00 }, /* 0x33 SDM_CTRL  */
	{ 0x36, 0x92 }, /* 0x36 Protection  */
	{ 0x37, 0x3F }, /* 0x37 SlopeCTRL  */
	{ 0x38, 0x00 }, /* 0x38 DIS_CLASSH_LVL12  */
	{ 0x39, 0x88 }, /* 0x39 DIS_CLASSH_LVL34  */
	{ 0x3A, 0x8C }, /* 0x3A DIS_CLASSH_LVL56  */
	{ 0x3B, 0x00 }, /* 0x3B Test1  */
	{ 0x3C, 0x00 }, /* 0x3C Test2  */
	{ 0x3D, 0x00 }, /* 0x3D Test3  */
	{ 0x3E, 0x03 }, /* 0x3E ATest1  */
	{ 0x3F, 0x00 }, /* 0x3F ATest2  */
	{ 0x40, 0x00 }, /* 0x40 EQCTRL1 : EQ BAND1 */
	{ 0x41, 0x00 }, /* 0x41 EQCTRL2  */
	{ 0x42, 0x00 }, /* 0x42 EQCTRL3  */
	{ 0x43, 0x00 }, /* 0x43 EQCTRL4  */
	{ 0x44, 0x00 }, /* 0x44 EQCTRL5  */
	{ 0x45, 0x00 }, /* 0x45 EQCTRL6  */
	{ 0x46, 0x20 }, /* 0x46 EQCTRL7  */
	{ 0x47, 0x00 }, /* 0x47 EQCTRL8  */
	{ 0x48, 0x00 }, /* 0x48 EQCTRL9  */
	{ 0x49, 0x00 }, /* 0x49 EQCTRL10  */
	{ 0x4A, 0x00 }, /* 0x4A EQCTRL11  */
	{ 0x4B, 0x00 }, /* 0x4B EQCTRL12  */
	{ 0x4C, 0x00 }, /* 0x4C EQCTRL13  */
	{ 0x4D, 0x00 }, /* 0x4D EQCTRL14  */
	{ 0x4E, 0x00 }, /* 0x4E EQCTRL15  */
	{ 0x4F, 0x00 }, /* 0x4F EQCTRL16 : EQ BAND2 */
	{ 0x50, 0x00 }, /* 0x50 EQCTRL17  */
	{ 0x51, 0x00 }, /* 0x51 EQCTRL18  */
	{ 0x52, 0x00 }, /* 0x52 EQCTRL19  */
	{ 0x53, 0x00 }, /* 0x53 EQCTRL20  */
	{ 0x54, 0x00 }, /* 0x54 EQCTRL21  */
	{ 0x55, 0x20 }, /* 0x55 EQCTRL22  */
	{ 0x56, 0x00 }, /* 0x56 EQCTRL23  */
	{ 0x57, 0x00 }, /* 0x57 EQCTRL24  */
	{ 0x58, 0x00 }, /* 0x58 EQCTRL25  */
	{ 0x59, 0x00 }, /* 0x59 EQCTRL26  */
	{ 0x5A, 0x00 }, /* 0x5A EQCTRL27  */
	{ 0x5B, 0x00 }, /* 0x5B EQCTRL28  */
	{ 0x5C, 0x00 }, /* 0x5C EQCTRL29  */
	{ 0x5D, 0x00 }, /* 0x5D EQCTRL30  */
	{ 0x5E, 0x00 }, /* 0x5E EQCTRL31 : EQ BAND3 */
	{ 0x5F, 0x00 }, /* 0x5F EQCTRL32  */
	{ 0x60, 0x00 }, /* 0x60 EQCTRL33  */
	{ 0x61, 0x00 }, /* 0x61 EQCTRL34  */
	{ 0x62, 0x00 }, /* 0x62 EQCTRL35  */
	{ 0x63, 0x00 }, /* 0x63 EQCTRL36  */
	{ 0x64, 0x20 }, /* 0x64 EQCTRL37  */
	{ 0x65, 0x00 }, /* 0x65 EQCTRL38  */
	{ 0x66, 0x00 }, /* 0x66 EQCTRL39  */
	{ 0x67, 0x00 }, /* 0x67 EQCTRL40  */
	{ 0x68, 0x00 }, /* 0x68 EQCTRL41  */
	{ 0x69, 0x00 }, /* 0x69 EQCTRL42  */
	{ 0x6A, 0x00 }, /* 0x6A EQCTRL43  */
	{ 0x6B, 0x00 }, /* 0x6B EQCTRL44  */
	{ 0x6C, 0x00 }, /* 0x6C EQCTRL45  */
	{ 0x6D, 0x00 }, /* 0x6D EQCTRL46 : EQ BAND4 */
	{ 0x6E, 0x00 }, /* 0x6E EQCTRL47  */
	{ 0x6F, 0x00 }, /* 0x6F EQCTRL48  */
	{ 0x70, 0x00 }, /* 0x70 EQCTRL49  */
	{ 0x71, 0x00 }, /* 0x71 EQCTRL50  */
	{ 0x72, 0x00 }, /* 0x72 EQCTRL51  */
	{ 0x73, 0x20 }, /* 0x73 EQCTRL52  */
	{ 0x74, 0x00 }, /* 0x74 EQCTRL53  */
	{ 0x75, 0x00 }, /* 0x75 EQCTRL54  */
	{ 0x76, 0x00 }, /* 0x76 EQCTRL55  */
	{ 0x77, 0x00 }, /* 0x77 EQCTRL56  */
	{ 0x78, 0x00 }, /* 0x78 EQCTRL57  */
	{ 0x79, 0x00 }, /* 0x79 EQCTRL58  */
	{ 0x7A, 0x00 }, /* 0x7A EQCTRL59  */
	{ 0x7B, 0x00 }, /* 0x7B EQCTRL60  */
	{ 0x7C, 0x00 }, /* 0x7C EQCTRL61 : EQ BAND5 */
	{ 0x7D, 0x00 }, /* 0x7D EQCTRL62  */
	{ 0x7E, 0x00 }, /* 0x7E EQCTRL63  */
	{ 0x7F, 0x00 }, /* 0x7F EQCTRL64  */
	{ 0x80, 0x00 }, /* 0x80 EQCTRL65  */
	{ 0x81, 0x00 }, /* 0x81 EQCTRL66  */
	{ 0x82, 0x20 }, /* 0x82 EQCTRL67  */
	{ 0x83, 0x00 }, /* 0x83 EQCTRL68  */
	{ 0x84, 0x00 }, /* 0x84 EQCTRL69  */
	{ 0x85, 0x00 }, /* 0x85 EQCTRL70  */
	{ 0x86, 0x00 }, /* 0x86 EQCTRL71  */
	{ 0x87, 0x00 }, /* 0x87 EQCTRL72  */
	{ 0x88, 0x00 }, /* 0x88 EQCTRL73  */
	{ 0x89, 0x00 }, /* 0x89 EQCTRL74  */
	{ 0x8A, 0x00 }, /* 0x8A EQCTRL75  */
	{ 0x8B, 0x07 }, /* 0x8B PLL_POST_N  */
	{ 0x8C, 0x70 }, /* 0x8C PLL_N  */
	{ 0x8D, 0x00 }, /* 0x8D PLL_F1  */
	{ 0x8E, 0x00 }, /* 0x8E PLL_F2  */
	{ 0x8F, 0x00 }, /* 0x8F PLL_F3,P,CP  */
	{ 0x90, 0xC2 }, /* 0x90 Class-H Control Level1 */
	{ 0x91, 0x82 }, /* 0x91 Class-H Control Level2 */
	{ 0x92, 0x33 }, /* 0x92 FDPEC Control2  */
	{ 0x93, 0x8E }, /* 0x93 Boost Control0  */
	{ 0x94, 0x9B }, /* 0x94 Boost Control1  */
	{ 0x95, 0x25 }, /* 0x95 Boost Control2  */
	{ 0x96, 0x3E }, /* 0x96 Boost Control3  */
	{ 0x97, 0xE8 }, /* 0x97 Boost Control4  */
	{ 0xA2, 0x69 }, /* 0xA2 TOP_MAN1  */
	{ 0xA3, 0x28 }, /* 0xA3 TOP_MAN2  */
	{ 0xA4, 0x46 }, /* 0xA4 SDO OUTPUT FORMAT  */
	{ 0xA5, 0x28 }, /* 0xA5 TDM TX1  */
	{ 0xA6, 0x01 }, /* 0xA6 TDM TX2  */
	{ 0xA7, 0x00 }, /* 0xA7 TOP_MAN3  */
	{ 0xA8, 0x21 }, /* 0xA8 TONE GENERATOR  */
	{ 0xA9, 0x67 }, /* 0xA9 TONE / FINE VOLUME  */
	{ 0xAA, 0x8B }, /* 0xAA PLL_A_Setting  */
	{ 0xAB, 0x01 }, /* 0xAB PLL_D_Setting  */
	{ 0xAC, 0x2F }, /* 0xAC PLL_Mode_CTRL  */
	{ 0xAD, 0x09 }, /* 0xAD SPK_OCP_LVL  */
	{ 0xAE, 0x02 }, /* 0xAE TOP_MAN4  */
	{ 0xAF, 0xC0 }, /* 0xAF VBAT_Sensing  */
	{ 0xB0, 0x08 }, /* 0xB0 Brown Out Protection0  */
	{ 0xB1, 0xAA }, /* 0xB1 Brown Out Protection1  */
	{ 0xB2, 0x99 }, /* 0xB2 Brown Out Protection2   */
	{ 0xB3, 0x8C }, /* 0xB3 Brown Out Protection3  */
	{ 0xB4, 0x1C }, /* 0xB4 Brown Out Protection4  */
	{ 0xB5, 0x1B }, /* 0xB5 Brown Out Protection5  */
	{ 0xB6, 0xE6 }, /* 0xB6 Brown Out Protection6  */
	{ 0xB7, 0xD9 }, /* 0xB7 Brown Out Protection7  */
	{ 0xB8, 0x7F }, /* 0xB8 Brown Out Protection8  */
	{ 0xB9, 0x76 }, /* 0xB9 Brown Out Protection9  */
	{ 0xBA, 0x6E }, /* 0xBA Brown Out Protection10  */
	{ 0xBB, 0x6A }, /* 0xBB Brown Out Protection11  */
	{ 0xBC, 0x18 }, /* 0xBC Brown Out Protection12  */
	{ 0xBD, 0x76 }, /* 0xBD Brown Out Protection13  */
	{ 0xBE, 0x94 }, /* 0xBE Brown Out Protection14  */
	{ 0xBF, 0xB3 }, /* 0xBF Brown Out Protection15  */
	{ 0xFA, 0xC0 }, /* 0xFA Status1  */
	{ 0xFB, 0x00 }, /* 0xFB Status2  */
	{ 0xFC, 0x00 }, /* 0xFC Status3  */
	{ 0xFD, 0x00 }, /* 0xFD Status4  */
	{ 0xFE, 0x00 }, /* 0xFE Status5  */
	{ 0xFF, 0x0B }, /* 0xFF Version  */
};

static bool sma6101_readable_register(struct device *dev, unsigned int reg)
{
	if (reg > SMA6101_FF_VERSION)
		return false;

	switch (reg) {
	case SMA6101_00_SYSTEM_CTRL ... SMA6101_21_TDM_RX:
	case SMA6101_23_COMP_LIM1 ... SMA6101_26_COMP_LIM4:
	case SMA6101_2B_EQ_MODE:
	case SMA6101_33_SDM_CTRL:
	case SMA6101_36_PROTECTION ... SMA6101_97_BOOST_CTRL4:
	case SMA6101_A2_TOP_MAN1 ... SMA6101_BF_BROWN_OUT_P15:
	case SMA6101_FA_STATUS1 ... SMA6101_FF_VERSION:
		return true;
	default:
		return false;
	}
}

static bool sma6101_writeable_register(struct device *dev, unsigned int reg)
{
	if (reg > SMA6101_FF_VERSION)
		return false;

	switch (reg) {
	case SMA6101_00_SYSTEM_CTRL ... SMA6101_21_TDM_RX:
	case SMA6101_23_COMP_LIM1 ... SMA6101_26_COMP_LIM4:
	case SMA6101_2B_EQ_MODE:
	case SMA6101_33_SDM_CTRL:
	case SMA6101_36_PROTECTION ... SMA6101_97_BOOST_CTRL4:
	case SMA6101_A2_TOP_MAN1 ... SMA6101_BF_BROWN_OUT_P15:
		return true;
	default:
		return false;
	}
}

static bool sma6101_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SMA6101_FA_STATUS1 ... SMA6101_FF_VERSION:
		return true;
	default:
		return false;
	}
}

/* DB scale conversion of speaker volume(mute:-60dB) */
static const DECLARE_TLV_DB_SCALE(sma6101_spk_tlv, -6000, 50, 0);

/* common bytes ext functions */
static int bytes_ext_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol, int reg)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	struct soc_bytes_ext *params = (void *)kcontrol->private_value;
	unsigned int i, reg_val;
	u8 *val;

	val = (u8 *)ucontrol->value.bytes.data;
	for (i = 0; i < params->max; i++) {
		regmap_read(sma6101->regmap, reg + i, &reg_val);
		if (sizeof(reg_val) > 2)
			reg_val = cpu_to_le32(reg_val);
		else
			reg_val = cpu_to_le16(reg_val);
		memcpy(val + i, &reg_val, sizeof(u8));
	}

	return 0;
}

static int bytes_ext_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol, int reg)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	struct soc_bytes_ext *params = (void *)kcontrol->private_value;
	void *data;
	u8 *val;
	int i, ret;

	data = kmemdup(ucontrol->value.bytes.data,
			params->max, GFP_KERNEL | GFP_DMA);
	if (!data)
		return -ENOMEM;

	val = (u8 *)data;
	for (i = 0; i < params->max; i++) {
		ret = regmap_write(sma6101->regmap, reg + i, *(val + i));
		if (ret) {
			dev_err(codec->dev,
				"configuration fail, register: %x ret: %d\n",
				reg + i, ret);
			kfree(data);
			return ret;
		}
	}
	kfree(data);

	return 0;
}

static int power_up_down_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = sma6101->amp_power_status;

	return 0;
}

static int power_up_down_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 1))
		return -EINVAL;

	if (sel && !(sma6101->force_amp_power_down))
		sma6101_startup(codec);
	else
		sma6101_shutdown(codec);

	return 0;
}

static int power_down_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = sma6101->force_amp_power_down;

	return 0;
}

static int power_down_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	sma6101->force_amp_power_down = ucontrol->value.integer.value[0];

	if (sma6101->force_amp_power_down) {
		dev_info(codec->dev, "%s\n", "Force AMP power down mode");
		sma6101_shutdown(codec);
	} else
		dev_info(codec->dev, "%s\n",
				"Force AMP power down out of mode");

	return 0;
}

/* Clock System Set */
static const char * const sma6101_clk_system_text[] = {
	"Reserved", "Reserved", "Reserved", "External clock 19.2MHz",
	"External clock 24.576MHz", "Reserved", "Reserved", "Reserved"};

static const struct soc_enum sma6101_clk_system_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_clk_system_text),
		sma6101_clk_system_text);

static int sma6101_clk_system_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_00_SYSTEM_CTRL, &val);
	ucontrol->value.integer.value[0] = ((val & 0xE0) >> 5);

	return 0;
}

static int sma6101_clk_system_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 7))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_00_SYSTEM_CTRL,
			0xE0, (sel << 5));

	return 0;
}

/* InputCTRL1 Set */
static const char * const sma6101_input_format_text[] = {
	"Philips standard I2S", "Left justified", "Not used",
	"Not used", "Right justified 16bits", "Right justified 18bits",
	"Right justified 20bits", "Right justified 24bits"};

static const struct soc_enum sma6101_input_format_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_input_format_text),
		sma6101_input_format_text);

static int sma6101_input_format_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_01_INPUT1_CTRL1, &val);
	ucontrol->value.integer.value[0] = ((val & 0x70) >> 4);

	return 0;
}

static int sma6101_input_format_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 7))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_01_INPUT1_CTRL1,
			0x70, (sel << 4));

	return 0;
}

/* InputCTRL2 Set */
static const char * const sma6101_in_audio_mode_text[] = {
	"I2S mode", "PCM/IOM2 short sync", "PCM/IOM2 long sync", "Reserved"};

static const struct soc_enum sma6101_in_audio_mode_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_in_audio_mode_text),
		sma6101_in_audio_mode_text);

static int sma6101_in_audio_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_02_INPUT1_CTRL2, &val);
	ucontrol->value.integer.value[0] = ((val & 0xC0) >> 6);

	return 0;
}

static int sma6101_in_audio_mode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap,
		SMA6101_02_INPUT1_CTRL2, 0xC0, (sel << 6));

	return 0;
}

/* InputCTRL3 Set */
static const char * const sma6101_pcm_n_slot_text[] = {
	"Slot_1", "Slot_2", "Slot_3", "Slot_4", "Slot_5", "Slot_6",
	"Slot_7", "Slot_8", "Slot_9", "Slot_10", "Slot_11", "Slot_12",
	"Slot_13", "Slot_14", "Slot_15", "Slot_16"};

static const struct soc_enum sma6101_pcm_n_slot_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_pcm_n_slot_text),
		sma6101_pcm_n_slot_text);

static int sma6101_pcm_n_slot_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_03_INPUT1_CTRL3, &val);
	ucontrol->value.integer.value[0] = (val & 0x0F);

	return 0;
}

static int sma6101_pcm_n_slot_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_03_INPUT1_CTRL3, 0x0F, sel);

	return 0;
}

/* InputCTRL4 Set */
static const char * const sma6101_pcm1_slot_text[] = {
	"Slot_1", "Slot_2", "Slot_3", "Slot_4", "Slot_5", "Slot_6",
	"Slot_7", "Slot_8", "Slot_9", "Slot_10", "Slot_11", "Slot_12",
	"Slot_13", "Slot_14", "Slot_15", "Slot_16"};

static const struct soc_enum sma6101_pcm1_slot_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_pcm1_slot_text), sma6101_pcm1_slot_text);

static int sma6101_pcm1_slot_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_04_INPUT1_CTRL4, &val);
	ucontrol->value.integer.value[0] = ((val & 0xF0) >> 4);

	return 0;
}

static int sma6101_pcm1_slot_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_04_INPUT1_CTRL4,
			0xF0, (sel << 4));

	return 0;
}

static const char * const sma6101_pcm2_slot_text[] = {
	"Slot_1", "Slot_2", "Slot_3", "Slot_4", "Slot_5", "Slot_6",
	"Slot_7", "Slot_8", "Slot_9", "Slot_10", "Slot_11", "Slot_12",
	"Slot_13", "Slot_14", "Slot_15", "Slot_16"};

static const struct soc_enum sma6101_pcm2_slot_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_pcm2_slot_text), sma6101_pcm2_slot_text);

static int sma6101_pcm2_slot_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_04_INPUT1_CTRL4, &val);
	ucontrol->value.integer.value[0] = (val & 0x0F);

	return 0;
}

static int sma6101_pcm2_slot_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_04_INPUT1_CTRL4, 0x0F, sel);

	return 0;
}

/* BrownOut Set 1 to 4 */
static int brown_out_set1_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_05_BROWNOUT_SET1);
}

static int brown_out_set1_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_05_BROWNOUT_SET1);
}

/* Input / output port config */
static const char * const sma6101_port_config_text[] = {
	"Input port only", "Reserved", "Output port enable", "Reserved"};

static const struct soc_enum sma6101_port_config_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_port_config_text),
	sma6101_port_config_text);

static int sma6101_port_config_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_09_OUTPUT_CTRL, &val);
	ucontrol->value.integer.value[0] = ((val & 0x60) >> 5);

	return 0;
}

static int sma6101_port_config_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap,
		SMA6101_09_OUTPUT_CTRL, 0x60, (sel << 5));

	return 0;
}

/* Output format select */
static const char * const sma6101_port_out_format_text[] = {
	"I2S 32 SCK", "I2S 64 SCK", "PCM short sync 128fs", "Reserved"};

static const struct soc_enum sma6101_port_out_format_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_port_out_format_text),
	sma6101_port_out_format_text);

static int sma6101_port_out_format_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_09_OUTPUT_CTRL, &val);
	ucontrol->value.integer.value[0] = ((val & 0x18) >> 3);

	return 0;
}

static int sma6101_port_out_format_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap,
		SMA6101_09_OUTPUT_CTRL, 0x18, (sel << 3));

	return 0;
}

/* Output source */
static const char * const sma6101_port_out_sel_text[] = {
	"Disable", "Format Converter", "Mixer output",
	"SPK path, EQ, Bass, Vol, DRC",
	"Modulator input/tone generator output for test",
	"Reserved", "Reserved", "Reserved"};

static const struct soc_enum sma6101_port_out_sel_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_port_out_sel_text),
	sma6101_port_out_sel_text);

static int sma6101_port_out_sel_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_09_OUTPUT_CTRL, &val);
	ucontrol->value.integer.value[0] = (val & 0x07);

	return 0;
}

static int sma6101_port_out_sel_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 7))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_09_OUTPUT_CTRL, 0x07, sel);

	return 0;
}

/* BrownOut Set 5 to 6 */
static int brown_out_set2_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_0B_BROWNOUT_SET5);
}

static int brown_out_set2_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_0B_BROWNOUT_SET5);
}

/* Class-H Control Level3 Set */
static const char * const sma6101_attack_lvl_3_text[] = {
	"Disabled and BOOST LV3 on", "LVL_0.03125FS", "LVL_0.0625FS",
	"LVL_0.09375FS", "LVL_0.125FS", "LVL_0.15625FS", "LVL_0.1875FS",
	"LVL_0.21875FS", "LVL_0.25FS", "LVL_0.28125FS", "LVL_0.3125FS",
	"LVL_0.34375FS", "LVL_0.375FS", "LVL_0.40625FS", "LVL_0.4375FS",
	"Disabled and BOOST LV3 off"};

static const struct soc_enum sma6101_attack_lvl_3_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_attack_lvl_3_text),
		sma6101_attack_lvl_3_text);

static int sma6101_attack_lvl_3_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_0D_CLASS_H_CTRL_LVL3, &val);
	ucontrol->value.integer.value[0] = ((val & 0xF0) >> 4);

	return 0;
}

static int sma6101_attack_lvl_3_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_0D_CLASS_H_CTRL_LVL3,
			0xF0, (sel << 4));

	return 0;
}

static const char * const sma6101_release_time_3_text[] = {
	"Time_48kHz-00ms,96kHz-00ms", "Time_48kHz-20ms,96kHz-10ms",
	"Time_48kHz-40ms,96kHz-20ms", "Time_48kHz-60ms,96kHz-30ms",
	"Time_48kHz-80ms,96kHz-40ms", "Time_48kHz-100ms,96kHz-50ms",
	"Time_48kHz-120ms,96kHz-60ms", "Time_48kHz-140ms,96kHz-70ms",
	"Time_48kHz-160ms,96kHz-80ms", "Time_48kHz-180ms,96kHz-90ms",
	"Time_48kHz-200ms,96kHz-100ms", "Time_48kHz-220ms,96kHz-110ms",
	"Time_48kHz-240ms,96kHz-120ms", "Time_48kHz-260ms,96kHz-130ms",
	"Time_48kHz-280ms,96kHz-140ms", "Time_48kHz-300ms,96kHz-150ms"};

static const struct soc_enum sma6101_release_time_3_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_release_time_3_text),
		sma6101_release_time_3_text);

static int sma6101_release_time_3_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_0D_CLASS_H_CTRL_LVL3, &val);
	ucontrol->value.integer.value[0] = (val & 0x0F);

	return 0;
}

static int sma6101_release_time_3_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_0D_CLASS_H_CTRL_LVL3,
			0x0F, sel);

	return 0;
}

/* Volume slope */
static const char * const sma6101_vol_slope_text[] = {
	"Off", "Slow(about 1sec)", "Medium(about 0.5sec)",
	"Fast(about 0.1sec)"};

static const struct soc_enum sma6101_vol_slope_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_vol_slope_text),
	sma6101_vol_slope_text);

static int sma6101_vol_slope_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_0E_MUTE_VOL_CTRL, &val);
	ucontrol->value.integer.value[0] = ((val & 0xC0) >> 6);

	return 0;
}

static int sma6101_vol_slope_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap,
		SMA6101_0E_MUTE_VOL_CTRL, 0xC0, (sel << 6));

	return 0;
}

/* Mute slope */
static const char * const sma6101_mute_slope_text[] = {
	"Off", "Slow(about 200ms)", "Medium(about 50ms)",
	"Fast(about 10ms)"};

static const struct soc_enum sma6101_mute_slope_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_mute_slope_text),
	sma6101_mute_slope_text);

static int sma6101_mute_slope_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_0E_MUTE_VOL_CTRL, &val);
	ucontrol->value.integer.value[0] = ((val & 0x30) >> 4);

	return 0;
}

static int sma6101_mute_slope_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap,
		SMA6101_0E_MUTE_VOL_CTRL, 0x30, (sel << 4));

	return 0;
}

/* Class-H Control Level4 Set */
static const char * const sma6101_attack_lvl_4_text[] = {
	"Disabled and BOOST LV4 on", "LVL_0.0625FS", "LVL_0.125FS",
	"LVL_0.1875FS", "LVL_0.25FS", "LVL_0.3125FS", "LVL_0.375FS",
	"LVL_0.4375FS", "LVL_0.5FS", "LVL_0.5625FS", "LVL_0.625FS",
	"LVL_0.6875FS", "LVL_0.75FS", "LVL_0.8125FS", "LVL_0.875FS",
	"Disabled and BOOST LV4 off"};

static const struct soc_enum sma6101_attack_lvl_4_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_attack_lvl_4_text),
		sma6101_attack_lvl_4_text);

static int sma6101_attack_lvl_4_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_0F_CLASS_H_CTRL_LVL4, &val);
	ucontrol->value.integer.value[0] = ((val & 0xF0) >> 4);

	return 0;
}

static int sma6101_attack_lvl_4_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_0F_CLASS_H_CTRL_LVL4,
			0xF0, (sel << 4));

	return 0;
}

static const char * const sma6101_release_time_4_text[] = {
	"Time_48kHz-00ms,96kHz-00ms", "Time_48kHz-20ms,96kHz-10ms",
	"Time_48kHz-40ms,96kHz-20ms", "Time_48kHz-60ms,96kHz-30ms",
	"Time_48kHz-80ms,96kHz-40ms", "Time_48kHz-100ms,96kHz-50ms",
	"Time_48kHz-120ms,96kHz-60ms", "Time_48kHz-140ms,96kHz-70ms",
	"Time_48kHz-160ms,96kHz-80ms", "Time_48kHz-180ms,96kHz-90ms",
	"Time_48kHz-200ms,96kHz-100ms", "Time_48kHz-220ms,96kHz-110ms",
	"Time_48kHz-240ms,96kHz-120ms", "Time_48kHz-260ms,96kHz-130ms",
	"Time_48kHz-280ms,96kHz-140ms", "Time_48kHz-300ms,96kHz-150ms"};

static const struct soc_enum sma6101_release_time_4_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_release_time_4_text),
		sma6101_release_time_4_text);

static int sma6101_release_time_4_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_0F_CLASS_H_CTRL_LVL4, &val);
	ucontrol->value.integer.value[0] = (val & 0x0F);

	return 0;
}

static int sma6101_release_time_4_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_0F_CLASS_H_CTRL_LVL4,
			0x0F, sel);

	return 0;
}

/* Speaker mode */
static const char * const sma6101_spkmode_text[] = {
	"Off", "Mono for one chip solution", "Reserved", "Reserved",
	"Stereo for two chip solution", "Reserved", "Reserved", "Reserved"};

static const struct soc_enum sma6101_spkmode_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_spkmode_text),
	sma6101_spkmode_text);

static int sma6101_spkmode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_10_SYSTEM_CTRL1, &val);
	ucontrol->value.integer.value[0] = ((val & 0x1C) >> 2);

	return 0;
}

static int sma6101_spkmode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 7))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap,
		SMA6101_10_SYSTEM_CTRL1, 0x1C, (sel << 2));

	if (sel == (SPK_MONO >> 2))
		sma6101->stereo_two_chip = false;
	else if (sel == (SPK_STEREO >> 2))
		sma6101->stereo_two_chip = true;

	return 0;
}

/* SystemCTRL3 Set */
static const char * const sma6101_input_gain_text[] = {
	"Gain_0dB", "Gain_-6dB", "Gain_-12dB", "Gain_-Infinity"};

static const struct soc_enum sma6101_input_gain_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_input_gain_text),
		sma6101_input_gain_text);

static int sma6101_input_gain_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_12_SYSTEM_CTRL3, &val);
	ucontrol->value.integer.value[0] = ((val & 0xC0) >> 6);

	return 0;
}

static int sma6101_input_gain_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_12_SYSTEM_CTRL3,
			0xC0, (sel << 6));

	return 0;
}

static const char * const sma6101_input_r_gain_text[] = {
	"Gain_0dB", "Gain_-6dB", "Gain_-12dB", "Gain_-Infinity"};

static const struct soc_enum sma6101_input_r_gain_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_input_r_gain_text),
		sma6101_input_r_gain_text);

static int sma6101_input_r_gain_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_12_SYSTEM_CTRL3, &val);
	ucontrol->value.integer.value[0] = ((val & 0x30) >> 4);

	return 0;
}

static int sma6101_input_r_gain_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_12_SYSTEM_CTRL3,
			0x30, (sel << 4));

	return 0;
}

/* FDPEC Control1 Set */
static const char * const sma6101_fdpec_i_text[] = {
	"I_40uA", "I_80uA", "I_120uA", "I_160uA"};

static const struct soc_enum sma6101_fdpec_i_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_fdpec_i_text), sma6101_fdpec_i_text);

static int sma6101_fdpec_i_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_13_FDPEC_CTRL1, &val);
	ucontrol->value.integer.value[0] = ((val & 0x0C) >> 2);

	return 0;
}

static int sma6101_fdpec_i_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_13_FDPEC_CTRL1,
			0x0C, (sel << 2));

	return 0;
}

static const char * const fdpec_gain_control_text[] = {
	"Gain 2.6", "Gain 4", "Gain 6", "Gain 8"};

static const struct soc_enum fdpec_gain_control_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(fdpec_gain_control_text),
	fdpec_gain_control_text);

static int fdpec_gain_control_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = sma6101->fdpec_gain_control;

	return 0;
}

static int fdpec_gain_control_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	sma6101_setup_fdpec_gain(codec, sel);

	return 0;
}


/* Modulator Set */
static const char * const sma6101_spk_hysfb_text[] = {
	"f_625kHz", "f_414kHz", "f_297kHz", "f_226kHz"};

static const struct soc_enum sma6101_spk_hysfb_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_spk_hysfb_text), sma6101_spk_hysfb_text);

static int sma6101_spk_hysfb_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_14_MODULATOR, &val);
	ucontrol->value.integer.value[0] = ((val & 0xC0) >> 6);

	return 0;
}

static int sma6101_spk_hysfb_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_14_MODULATOR,
			0xC0, (sel << 6));

	return 0;
}

static int spk_bdelay_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_14_MODULATOR);
}

static int spk_bdelay_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_14_MODULATOR);
}

/* bass boost speaker coeff */
static int bass_spk_coeff_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_15_BASS_SPK1);
}

static int bass_spk_coeff_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_15_BASS_SPK1);
}

/* Brown Out Protection 16-20 Set */
static int brown_out_pt2_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_1C_BROWN_OUT_P16);
}

static int brown_out_pt2_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_1C_BROWN_OUT_P16);
}

/* TDM RX Set */
static const char * const sma6101_tdm_l_slot_text[] = {
	"Slot_0", "Slot_1", "Slot_2", "Slot_3",
	"Slot_4", "Slot_5", "Slot_6", "Slot_7"};

static const struct soc_enum sma6101_tdm_l_slot_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_tdm_l_slot_text),
		sma6101_tdm_l_slot_text);

static int sma6101_tdm_l_slot_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_21_TDM_RX, &val);
	ucontrol->value.integer.value[0] = ((val & 0x38) >> 3);

	return 0;
}

static int sma6101_tdm_l_slot_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 7))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_21_TDM_RX,
			0x38, (sel << 3));

	return 0;
}

static const char * const sma6101_tdm_r_slot_text[] = {
	"Slot_0", "Slot_1", "Slot_2", "Slot_3",
	"Slot_4", "Slot_5", "Slot_6", "Slot_7"};

static const struct soc_enum sma6101_tdm_r_slot_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_tdm_r_slot_text),
		sma6101_tdm_r_slot_text);

static int sma6101_tdm_r_slot_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_21_TDM_RX, &val);
	ucontrol->value.integer.value[0] = (val & 0x07);

	return 0;
}

static int sma6101_tdm_r_slot_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 7))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_21_TDM_RX,
			0x07, sel);

	return 0;
}

/* DRC speaker coeff */
static int comp_lim_spk_coeff_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_23_COMP_LIM1);
}

static int comp_lim_spk_coeff_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_23_COMP_LIM1);
}

/* OTP MODE Set */
static const char * const sma6101_otp_mode_text[] = {
	"Disable", "Ignore threshold1, shutdown threshold2",
	"Reduced threshold1, shutdown threshold2",
	"Shutdown threshold1, shutdown threshold2"};

static const struct soc_enum sma6101_otp_mode_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_otp_mode_text), sma6101_otp_mode_text);

static int sma6101_otp_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_36_PROTECTION, &val);
	ucontrol->value.integer.value[0] = (val & 0x03);

	return 0;
}

static int sma6101_otp_mode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 7))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_36_PROTECTION, 0x03, sel);

	return 0;
}

/* Slope CTRL */
static int slope_ctrl_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_37_SLOPE_CTRL);
}

static int slope_ctrl_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_37_SLOPE_CTRL);
}

/* Disable class-H set */
static int dis_class_h_lvl_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_38_DIS_CLASSH_LVL12);
}

static int dis_class_h_lvl_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_38_DIS_CLASSH_LVL12);
}

/* Test 1~3, ATEST 1~2 */
static int test_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_3B_TEST1);
}

static int test_mode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_3B_TEST1);
}

/* PEQ Band1 */
static int eq_ctrl_band1_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_40_EQ_CTRL1);
}

static int eq_ctrl_band1_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_40_EQ_CTRL1);
}

/* PEQ Band2 */
static int eq_ctrl_band2_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_4F_EQ_CTRL16);
}

static int eq_ctrl_band2_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_4F_EQ_CTRL16);
}

/* PEQ Band3 */
static int eq_ctrl_band3_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_5E_EQ_CTRL31);
}

static int eq_ctrl_band3_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_5E_EQ_CTRL31);
}

/* PEQ Band4 */
static int eq_ctrl_band4_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_6D_EQ_CTRL46);
}

static int eq_ctrl_band4_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_6D_EQ_CTRL46);
}

/* PEQ Band5 */
static int eq_ctrl_band5_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_7C_EQ_CTRL61);
}

static int eq_ctrl_band5_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_7C_EQ_CTRL61);
}

/* PLL setting */
static int pll_setting_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_8B_PLL_POST_N);
}

static int pll_setting_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_8B_PLL_POST_N);
}

/* Class-H control Level Set */
static const char * const sma6101_attack_lvl_1_text[] = {
	"Disabled and BOOST LV1 on", "LVL_0.007813FS", "LVL_0.015626FS",
	"LVL_0.023439FS", "LVL_0.03125FS", "LVL_0.039063FS", "LVL_0.046876FS",
	"LVL_0.054689FS", "LVL_0.0625FS", "LVL_0.070313FS", "LVL_0.078126FS",
	"LVL_0.085939FS", "LVL_0.09375FS", "LVL_0.101563FS", "LVL_0.109376FS",
	"Disabled and BOOST LV1 off"};

static const struct soc_enum sma6101_attack_lvl_1_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_attack_lvl_1_text),
		sma6101_attack_lvl_1_text);

static int sma6101_attack_lvl_1_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_90_CLASS_H_CTRL_LVL1, &val);
	ucontrol->value.integer.value[0] = ((val & 0xF0) >> 4);

	return 0;
}

static int sma6101_attack_lvl_1_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_90_CLASS_H_CTRL_LVL1,
			0xF0, (sel << 4));

	return 0;
}

static const char * const sma6101_release_time_1_text[] = {
	"Time_48kHz-00ms,96kHz-00ms", "Time_48kHz-20ms,96kHz-10ms",
	"Time_48kHz-40ms,96kHz-20ms", "Time_48kHz-60ms,96kHz-30ms",
	"Time_48kHz-80ms,96kHz-40ms", "Time_48kHz-100ms,96kHz-50ms",
	"Time_48kHz-120ms,96kHz-60ms", "Time_48kHz-140ms,96kHz-70ms",
	"Time_48kHz-160ms,96kHz-80ms", "Time_48kHz-180ms,96kHz-90ms",
	"Time_48kHz-200ms,96kHz-100ms", "Time_48kHz-220ms,96kHz-110ms",
	"Time_48kHz-240ms,96kHz-120ms", "Time_48kHz-260ms,96kHz-130ms",
	"Time_48kHz-280ms,96kHz-140ms", "Time_48kHz-300ms,96kHz-150ms"};

static const struct soc_enum sma6101_release_time_1_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_release_time_1_text),
		sma6101_release_time_1_text);

static int sma6101_release_time_1_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_90_CLASS_H_CTRL_LVL1, &val);
	ucontrol->value.integer.value[0] = (val & 0x0F);

	return 0;
}

static int sma6101_release_time_1_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_90_CLASS_H_CTRL_LVL1,
			0x0F, sel);

	return 0;
}

static const char * const sma6101_attack_lvl_2_text[] = {
	"Disabled and BOOST LV2 on", "LVL_0.03125FS", "LVL_0.0625FS",
	"LVL_0.09375FS", "LVL_0.125FS", "LVL_0.15625FS", "LVL_0.1875FS",
	"LVL_0.21875FS", "LVL_0.25FS", "LVL_0.28125FS", "LVL_0.3125FS",
	"LVL_0.34375FS", "LVL_0.375FS", "LVL_0.40625FS", "LVL_0.4375FS",
	"Disabled and BOOST LV2 off"};

static const struct soc_enum sma6101_attack_lvl_2_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_attack_lvl_2_text),
		sma6101_attack_lvl_2_text);

static int sma6101_attack_lvl_2_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_91_CLASS_H_CTRL_LVL2, &val);
	ucontrol->value.integer.value[0] = ((val & 0xF0) >> 4);

	return 0;
}

static int sma6101_attack_lvl_2_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_91_CLASS_H_CTRL_LVL2,
			0xF0, (sel << 4));

	return 0;
}

static const char * const sma6101_release_time_2_text[] = {
	"Time_48kHz-00ms,96kHz-00ms", "Time_48kHz-20ms,96kHz-10ms",
	"Time_48kHz-40ms,96kHz-20ms", "Time_48kHz-60ms,96kHz-30ms",
	"Time_48kHz-80ms,96kHz-40ms", "Time_48kHz-100ms,96kHz-50ms",
	"Time_48kHz-120ms,96kHz-60ms", "Time_48kHz-140ms,96kHz-70ms",
	"Time_48kHz-160ms,96kHz-80ms", "Time_48kHz-180ms,96kHz-90ms",
	"Time_48kHz-200ms,96kHz-100ms", "Time_48kHz-220ms,96kHz-110ms",
	"Time_48kHz-240ms,96kHz-120ms", "Time_48kHz-260ms,96kHz-130ms",
	"Time_48kHz-280ms,96kHz-140ms", "Time_48kHz-300ms,96kHz-150ms"};

static const struct soc_enum sma6101_release_time_2_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_release_time_2_text),
		sma6101_release_time_2_text);

static int sma6101_release_time_2_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_91_CLASS_H_CTRL_LVL2, &val);
	ucontrol->value.integer.value[0] = (val & 0x0F);

	return 0;
}

static int sma6101_release_time_2_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_91_CLASS_H_CTRL_LVL2,
			0x0F, sel);

	return 0;
}

/* FDPEC Control2 Set */
static const char * const sma6101_fdpec_gain_trm_text[] = {
	"No trimming", "7% increase", "10% increase", "26% increase"};

static const struct soc_enum sma6101_fdpec_gain_trm_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_fdpec_gain_trm_text),
		sma6101_fdpec_gain_trm_text);

static int sma6101_fdpec_gain_trm_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_92_FDPEC_CTRL2, &val);
	ucontrol->value.integer.value[0] = ((val & 0xC0) >> 6);

	return 0;
}

static int sma6101_fdpec_gain_trm_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_92_FDPEC_CTRL2,
			0xC0, (sel << 6));

	return 0;
}

/* Boost control0 Set*/
static const char * const sma6101_trm_vref_text[] = {
	"REF_1.3V", "REF_1.2875V", "REF_1.275V", "REF_1.2625V", "REF_1.25V",
	"REF_1.2375V", "REF_1.225V", "REF_1.2125V", "REF_1.2V", "REF_1.1875V",
	"REF_1.175V", "REF_1.1625V", "REF_1.15V", "REF_1.1375V", "REF_1.125V",
	"REF_1.1125V"};

static const struct soc_enum sma6101_trm_vref_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_trm_vref_text), sma6101_trm_vref_text);

static int sma6101_trm_vref_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_93_BOOST_CTRL0, &val);
	ucontrol->value.integer.value[0] = ((val & 0xF0) >> 4);

	return 0;
}

static int sma6101_trm_vref_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	if (sma6101->rev_num < REV_NUM_REV3) {
		regmap_update_bits(sma6101->regmap, SMA6101_93_BOOST_CTRL0,
			0xF0, (sel << 4));
	} else  {
		dev_info(codec->dev, "Trimming of VBG reference does not change on REV3 and above\n");
	}

	return 0;
}

static const char * const sma6101_trm_vbst1_text[] = {
	"BST_6V", "BST_7V", "BST_8V", "BST_9V", "BST_10V", "BST_11V",
	"BST_12V", "BST_13V", "BST_14V", "BST_15V", "BST_16V", "BST_17V",
	"BST_18V", "BST_19V", "BST_20V", "BST_21V"};

static const struct soc_enum sma6101_trm_vbst1_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_trm_vbst1_text), sma6101_trm_vbst1_text);

static int sma6101_trm_vbst1_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_93_BOOST_CTRL0, &val);
	ucontrol->value.integer.value[0] = (val & 0x0F);

	return 0;
}

static int sma6101_trm_vbst1_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	if (sma6101->rev_num < REV_NUM_REV3) {
		dev_info(codec->dev, "%s : Trimming of boost output voltage %dV\n",
					__func__, (sel + 6));
#if defined(CONFIG_MACH_LGE)
		sma6101->trm_vbst1 = sel;
		if (sma6101->rev_num != REV_NUM_REV0) {
			regmap_update_bits(sma6101->regmap, SMA6101_93_BOOST_CTRL0,0x0F, sel);	
		}
#else
		regmap_update_bits(sma6101->regmap, SMA6101_93_BOOST_CTRL0,
				0x0F, sel);
#endif
	} else {
		dev_info(codec->dev, "Trimming of boost output voltage does not change on REV3 and above\n");
	}

	return 0;
}

/* Boost control1 Set*/
static const char * const sma6101_trm_comp2_text[] = {
	"C_10pF", "C_30pF", "C_50pF", "C_70pF"};

static const struct soc_enum sma6101_trm_comp2_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_trm_comp2_text), sma6101_trm_comp2_text);

static int sma6101_trm_comp2_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_94_BOOST_CTRL1, &val);
	ucontrol->value.integer.value[0] = ((val & 0xC0) >> 6);

	return 0;
}

static int sma6101_trm_comp2_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_94_BOOST_CTRL1,
			0xC0, (sel << 6));

	return 0;
}

static const char * const sma6101_trm_osc_text[] = {
	"f_1.37MHz", "f_1.54MHz", "f_1.76MHz", "f_2.05MHz",
	"f_2.23MHz", "f_2.46MHz", "f_3.07MHz", "f_3.51MHz"};

static const struct soc_enum sma6101_trm_osc_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_trm_osc_text), sma6101_trm_osc_text);

static int sma6101_trm_osc_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_94_BOOST_CTRL1, &val);
	ucontrol->value.integer.value[0] = ((val & 0x38) >> 3);

	return 0;
}

static int sma6101_trm_osc_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 7))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_94_BOOST_CTRL1,
			0x38, (sel << 3));

	return 0;
}

static const char * const sma6101_trm_rmp_text[] = {
	"RMP_4.75A/us", "RMP_5.64A/us", "RMP_6.43A/us", "RMP_7.37A/us",
	"RMP_8.29A/us", "RMP_9.22A/us", "RMP_10.12A/us", "RMP_11.00A/us"};

static const struct soc_enum sma6101_trm_rmp_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_trm_rmp_text), sma6101_trm_rmp_text);

static int sma6101_trm_rmp_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_94_BOOST_CTRL1, &val);
	ucontrol->value.integer.value[0] = (val & 0x07);

	return 0;
}

static int sma6101_trm_rmp_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 7))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_94_BOOST_CTRL1,
			0x07, sel);

	return 0;
}

/* Boost control2 Set*/
static const char * const sma6101_trm_ocl_text[] = {
	"I_1.2A", "I_1.6A", "I_2.1A", "I_2.6A",
	"I_3.1A", "I_3.5A", "I_3.9A", "I_4.2A"};

static const struct soc_enum sma6101_trm_ocl_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_trm_ocl_text), sma6101_trm_ocl_text);

static int sma6101_trm_ocl_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_95_BOOST_CTRL2, &val);
	ucontrol->value.integer.value[0] = ((val & 0x70) >> 4);

	return 0;
}

static int sma6101_trm_ocl_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 7))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_95_BOOST_CTRL2,
			0x70, (sel << 4));

	return 0;
}

static const char * const sma6101_trm_comp_text[] = {
	"COMP_4.5Mohm/0.7pF", "COMP_4.0Mohm/0.7pF", "COMP_3.5Mohm/0.7pF",
	"COMP_3.0Mohm/0.7pF", "COMP_2.5Mohm/0.7pF", "COMP_2.0Mohm/0.7pF",
	"COMP_1.5Mohm/0.7pF", "COMP_1.0Mohm/0.7pF", "COMP_4.5Mohm/2.0pF",
	"COMP_4.0Mohm/2.0pF", "COMP_3.5Mohm/2.0pF", "COMP_3.0Mohm/2.0pF",
	"COMP_2.5Mohm/2.0pF", "COMP_2.0Mohm/2.0pF", "COMP_1.5Mohm/2.0pF",
	"COMP_1.0Mohm/2.0pF"};

static const struct soc_enum sma6101_trm_comp_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_trm_comp_text), sma6101_trm_comp_text);

static int sma6101_trm_comp_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_95_BOOST_CTRL2, &val);
	ucontrol->value.integer.value[0] = (val & 0x0F);

	return 0;
}

static int sma6101_trm_comp_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_95_BOOST_CTRL2,
			0x0F, sel);

	return 0;
}

/* Boost control3 Set*/
static const char * const sma6101_trm_dt_text[] = {
	"Time_24.0ns", "Time_18.0ns", "Time_12.1ns", "Time_10.4ns",
	"Time_7.99ns", "Time_7.26ns", "Time_6.14ns", "Time_5.72ns",
	"Time_4.00ns", "Time_3.83ns", "Time_3.54ns", "Time_3.42ns",
	"Time_1.97ns", "Time_1.95ns", "Time_1.90ns", "Time_1.88ns"};

static const struct soc_enum sma6101_trm_dt_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_trm_dt_text), sma6101_trm_dt_text);

static int sma6101_trm_dt_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_96_BOOST_CTRL3, &val);
	ucontrol->value.integer.value[0] = ((val & 0xF0) >> 4);

	return 0;
}

static int sma6101_trm_dt_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_96_BOOST_CTRL3,
			0xF0, (sel << 4));

	return 0;
}

static const char * const sma6101_trm_slw_text[] = {
	"Time_6ns", "Time_4ns", "Time_3ns", "Time_2ns"};

static const struct soc_enum sma6101_trm_slw_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_trm_slw_text), sma6101_trm_slw_text);

static int sma6101_trm_slw_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_96_BOOST_CTRL3, &val);
	ucontrol->value.integer.value[0] = (val & 0x03);

	return 0;
}

static int sma6101_trm_slw_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_96_BOOST_CTRL3,
			0x03, sel);

	return 0;
}

/* Boost control4 Set*/
static const char * const sma6101_trm_vbst2_text[] = {
	"BST_0.60/0.40/0.28", "BST_0.60/0.40/0.30", "BST_0.60/0.40/0.32",
	"BST_0.60/0.40/0.35", "BST_0.60/0.45/0.28", "BST_0.60/0.45/0.30",
	"BST_0.60/0.45/0.32", "BST_0.60/0.45/0.35", "BST_0.60/0.50/0.28",
	"BST_0.60/0.50/0.30", "BST_0.60/0.50/0.32", "BST_0.60/0.50/0.35",
	"BST_0.60/0.55/0.28", "BST_0.60/0.55/0.30", "BST_0.60/0.55/0.32",
	"BST_0.60/0.55/0.35", "BST_0.65/0.40/0.28", "BST_0.65/0.40/0.30",
	"BST_0.65/0.40/0.32", "BST_0.65/0.40/0.35", "BST_0.65/0.45/0.28",
	"BST_0.65/0.45/0.30", "BST_0.65/0.45/0.32", "BST_0.65/0.45/0.35",
	"BST_0.65/0.50/0.28", "BST_0.65/0.50/0.30", "BST_0.65/0.50/0.32",
	"BST_0.65/0.50/0.35", "BST_0.65/0.55/0.28", "BST_0.65/0.55/0.30",
	"BST_0.65/0.55/0.32", "BST_0.65/0.55/0.35", "BST_0.70/0.40/0.28",
	"BST_0.70/0.40/0.30", "BST_0.70/0.40/0.32", "BST_0.70/0.40/0.35",
	"BST_0.70/0.45/0.28", "BST_0.70/0.45/0.30", "BST_0.70/0.45/0.32",
	"BST_0.70/0.45/0.35", "BST_0.70/0.50/0.28", "BST_0.70/0.50/0.30",
	"BST_0.70/0.50/0.32", "BST_0.70/0.50/0.35", "BST_0.70/0.55/0.28",
	"BST_0.70/0.55/0.30", "BST_0.70/0.55/0.32", "BST_0.70/0.55/0.35",
	"BST_0.75/0.40/0.28", "BST_0.75/0.40/0.30", "BST_0.75/0.40/0.32",
	"BST_0.75/0.40/0.35", "BST_0.75/0.45/0.28", "BST_0.75/0.45/0.30",
	"BST_0.75/0.45/0.32", "BST_0.75/0.45/0.35", "BST_0.75/0.50/0.28",
	"BST_0.75/0.50/0.30", "BST_0.75/0.50/0.32", "BST_0.75/0.50/0.35",
	"BST_0.75/0.55/0.28", "BST_0.75/0.55/0.30", "BST_0.75/0.55/0.32",
	"BST_0.75/0.55/0.35"};

static const struct soc_enum sma6101_trm_vbst2_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_trm_vbst2_text), sma6101_trm_vbst2_text);

static int sma6101_trm_vbst2_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_97_BOOST_CTRL4, &val);
	ucontrol->value.integer.value[0] = ((val & 0xFC) >> 2);

	return 0;
}

static int sma6101_trm_vbst2_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 63))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_97_BOOST_CTRL4,
			0xFC, (sel << 2));

	return 0;
}

static const char * const sma6101_trm_tmin_text[] = {
	"Time_59ns", "Time_68ns", "Time_77ns", "Time_86ns"};

static const struct soc_enum sma6101_trm_tmin_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_trm_tmin_text), sma6101_trm_tmin_text);

static int sma6101_trm_tmin_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_97_BOOST_CTRL4, &val);
	ucontrol->value.integer.value[0] = (val & 0x03);

	return 0;
}

static int sma6101_trm_tmin_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_97_BOOST_CTRL4,
			0x03, sel);

	return 0;
}

/* SDO OUTPUT FORMAT Set */
static const char * const sma6101_o_format_text[] = {
	"Reserved", "LJ", "I2S", "Reserved", "TDM",
	"Reserved", "Reserved", "Reserved"};

static const struct soc_enum sma6101_o_format_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_o_format_text), sma6101_o_format_text);

static int sma6101_o_format_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_A4_SDO_OUT_FMT, &val);
	ucontrol->value.integer.value[0] = ((val & 0xE0) >> 5);

	return 0;
}

static int sma6101_o_format_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 7))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_A4_SDO_OUT_FMT,
			0xE0, (sel << 5));

	return 0;
}

static const char * const sma6101_sck_rate_text[] = {
	"fs_64", "fs_64", "fs_32", "fs_32"};

static const struct soc_enum sma6101_sck_rate_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_sck_rate_text), sma6101_sck_rate_text);

static int sma6101_sck_rate_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_A4_SDO_OUT_FMT, &val);
	ucontrol->value.integer.value[0] = ((val & 0x18) >> 3);

	return 0;
}

static int sma6101_sck_rate_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_A4_SDO_OUT_FMT,
			0x18, (sel << 3));

	return 0;
}

static const char * const sma6101_wd_length_text[] = {
	"WD_24bit", "WD_20bit", "WD_16bit", "WD_16bit"};

static const struct soc_enum sma6101_wd_length_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_wd_length_text), sma6101_wd_length_text);

static int sma6101_wd_length_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_A4_SDO_OUT_FMT, &val);
	ucontrol->value.integer.value[0] = ((val & 0x06) >> 1);

	return 0;
}

static int sma6101_wd_length_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_A4_SDO_OUT_FMT,
			0x06, (sel << 1));

	return 0;
}

/* TDM TX Set */
static const char * const sma6101_tdm_n_slot_text[] = {
	"Slot_2", "Slot_4", "Slot_8", "Slot_16"};

static const struct soc_enum sma6101_tdm_n_slot_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_tdm_n_slot_text),
		sma6101_tdm_n_slot_text);

static int sma6101_tdm_n_slot_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_A5_TDM_TX1, &val);
	ucontrol->value.integer.value[0] = ((val & 0x0C) >> 2);

	return 0;
}

static int sma6101_tdm_n_slot_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_A5_TDM_TX1,
			0x0C, (sel << 2));

	return 0;
}

static const char * const sma6101_tdm_slot1_text[] = {
	"Slot_1", "Slot_2", "Slot_3", "Slot_4", "Slot_5", "Slot_6",
	"Slot_7", "Slot_8", "Slot_9", "Slot_10", "Slot_11", "Slot_12",
	"Slot_13", "Slot_14", "Slot_15", "Slot_16"};

static const struct soc_enum sma6101_tdm_slot1_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_tdm_slot1_text), sma6101_tdm_slot1_text);

static int sma6101_tdm_slot1_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_A6_TDM_TX2, &val);
	ucontrol->value.integer.value[0] = ((val & 0xF0) >> 4);

	return 0;
}

static int sma6101_tdm_slot1_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_A6_TDM_TX2,
			0xF0, (sel << 4));

	return 0;
}

static const char * const sma6101_tdm_slot2_text[] = {
	"Slot_1", "Slot_2", "Slot_3", "Slot_4", "Slot_5", "Slot_6",
	"Slot_7", "Slot_8", "Slot_9", "Slot_10", "Slot_11", "Slot_12",
	"Slot_13", "Slot_14", "Slot_15", "Slot_16"};

static const struct soc_enum sma6101_tdm_slot2_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_tdm_slot2_text), sma6101_tdm_slot2_text);

static int sma6101_tdm_slot2_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_A6_TDM_TX2, &val);
	ucontrol->value.integer.value[0] = (val & 0x0F);

	return 0;
}

static int sma6101_tdm_slot2_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_A6_TDM_TX2, 0x0F, sel);

	return 0;
}

/* TOP_MAN3 Set */
static const char * const sma6101_test_clock_mon_time_sel_text[] = {
	"Time_80us", "Time_40us", "Time_20us", "Time_10us"};

static const struct soc_enum sma6101_test_clock_mon_time_sel_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_test_clock_mon_time_sel_text),
		sma6101_test_clock_mon_time_sel_text);

static int sma6101_test_clock_mon_time_sel_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_A7_TOP_MAN3, &val);
	ucontrol->value.integer.value[0] = ((val & 0xC0) >> 6);

	return 0;
}

static int sma6101_test_clock_mon_time_sel_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_A7_TOP_MAN3,
			0xC0, (sel << 6));

	return 0;
}

/* TONE_GENERATOR Set */
static const char * const sma6101_tone_freq_text[] = {
	"f_50Hz", "f_60Hz", "f_140Hz", "f_150Hz", "f_175Hz", "f_180Hz",
	"f_200Hz", "f_375Hz", "f_750Hz", "f_1.5kHz", "f_3kHz", "f_6kHz",
	"f_8kHz", "f_12kHz", "f_1kHz", "f_1kHz"};

static const struct soc_enum sma6101_tone_freq_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_tone_freq_text), sma6101_tone_freq_text);

static int sma6101_tone_freq_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_A8_TONE_GENERATOR, &val);
	ucontrol->value.integer.value[0] = ((val & 0x1E) >> 1);

	return 0;
}

static int sma6101_tone_freq_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 15))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_A8_TONE_GENERATOR,
			0x1E, (sel << 1));

	return 0;
}

/* TONE_FINE VOLUME Set */
static int tone_fine_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_A9_TONE_FINE_VOL);
}

static int tone_fine_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_A9_TONE_FINE_VOL);
}

/* PLL_A_D_Setting Set */
static int pll_a_d_setting_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_AA_PLL_A_SETTING);
}

static int pll_a_d_setting_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_AA_PLL_A_SETTING);
}

/* PLL LDO Control Set */
static int pll_ldo_ctrl_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_AC_PLL_MODE_CTRL);
}

static int pll_ldo_ctrl_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_AC_PLL_MODE_CTRL);
}

/* SPK_OCP LVL Set */
static const char * const sma6101_sync_delay_text[] = {
	"Time_10ns", "Time_20ns", "Time_30ns", "Time_40ns"};

static const struct soc_enum sma6101_sync_delay_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_sync_delay_text),
		sma6101_sync_delay_text);

static int sma6101_sync_delay_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_AD_SPK_OCP_LVL, &val);
	ucontrol->value.integer.value[0] = ((val & 0x30) >> 4);

	return 0;
}

static int sma6101_sync_delay_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_AD_SPK_OCP_LVL,
			0x30, (sel << 4));

	return 0;
}

static const char * const sma6101_ocp_filter_text[] = {
	"Filter_0(Slowest)", "Filter_1", "Filter_2", "Filter_3(Fastest)"};

static const struct soc_enum sma6101_ocp_filter_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_ocp_filter_text),
		sma6101_ocp_filter_text);

static int sma6101_ocp_filter_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_AD_SPK_OCP_LVL, &val);
	ucontrol->value.integer.value[0] = ((val & 0x0C) >> 2);

	return 0;
}

static int sma6101_ocp_filter_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_AD_SPK_OCP_LVL,
			0x0C, (sel << 2));

	return 0;
}

static const char * const sma6101_ocp_lvl_text[] = {
	"I_2.6A(Low)", "I_3.1A", "I_3.7A", "I_4.2A(High)"};

static const struct soc_enum sma6101_ocp_lvl_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma6101_ocp_lvl_text), sma6101_ocp_lvl_text);

static int sma6101_ocp_lvl_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma6101->regmap, SMA6101_AD_SPK_OCP_LVL, &val);
	ucontrol->value.integer.value[0] = (val & 0x03);

	return 0;
}

static int sma6101_ocp_lvl_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if ((sel < 0) || (sel > 3))
		return -EINVAL;

	regmap_update_bits(sma6101->regmap, SMA6101_AD_SPK_OCP_LVL, 0x03, sel);

	return 0;
}

/* VIN Sensing Set */
static int vin_sensing_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_AF_VIN_SENSING);
}

static int vin_sensing_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_AF_VIN_SENSING);
}

/* Brown Out Protection 0-15 Set */
static int brown_out_pt_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA6101_B0_BROWN_OUT_P0);
}

static int brown_out_pt_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA6101_B0_BROWN_OUT_P0);
}

#ifdef CONFIG_MACH_LGE
static int __sma6101_reset_gpio_set(struct sma6101_priv *sma6101, int value)
{
	int ret = 0;

	pr_info("%s : value %d\n",__func__,value);

	if(sma6101 == NULL){
		pr_err("%s : fail to set reset_gpio\n",__func__);
		return -EINVAL;
	}

	if(value > 0)
		gpio_set_value(sma6101->reset_gpio,1);
	else
		gpio_set_value(sma6101->reset_gpio,0);

	return ret;
}

static int __sma6101_reset_gpio_get(struct sma6101_priv *sma6101)
{
	int ret = 0;

	if(sma6101 == NULL){
		pr_err("%s : fail to get reset_gpio\n",__func__);
		return -EINVAL;
	}

	ret = gpio_get_value(sma6101->reset_gpio);
	pr_info("%s : value %d\n",__func__,ret);

	return ret;
}

static int sma6101_reset_gpio_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	int value = 0;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	value = (int)ucontrol->value.integer.value[0];
	ret = __sma6101_reset_gpio_set(sma6101,value);

	return ret;
}

static int sma6101_reset_gpio_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	ret = __sma6101_reset_gpio_get(sma6101);
	ucontrol->value.integer.value[0] = ret;

	return ret;
}

/*
notify low battery status
sma6101->lowbattery_status = 0 : 0% <= battery level <= 5%
sma6101->lowbattery_status = 1 : 6% <= battery level <= 10%
sma6101->lowbattery_status = 2 : 11% <= battery level <= 15%
sma6101->lowbattery_status = 3 : 16% <= battery level <= 100%
*/
static int sma6101_low_battery_status_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	int val = 0;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	val = (int)ucontrol->value.integer.value[0];

	if(( val >= 0 ) && (val <= 3))
		sma6101->lowbattery_status = val;

	return ret;
}

static int sma6101_low_battery_status_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = sma6101->lowbattery_status;

	return ret;
}

#endif

#if defined(CONFIG_MACH_LGE)
static int sma6101_msm_set_pinctrl(struct msm_pinctrl_info *pinctrl_info,
				enum pinctrl_pin_state new_state)
{
	int ret = 0;
	int curr_state = 0;

	if (pinctrl_info == NULL) {
		pr_err("%s: pinctrl_info is NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	if (pinctrl_info->pinctrl == NULL) {
		pr_err("%s: pinctrl_info->pinctrl is NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	curr_state = pinctrl_info->curr_state;
	pinctrl_info->curr_state = new_state;
	pr_debug("%s: curr_state = %s new_state = %s\n", __func__,
		 pin_states[curr_state], pin_states[pinctrl_info->curr_state]);
	pr_debug("%s: pinctrl_info->curr_state:%d\n", __func__, pinctrl_info->curr_state);

	if (curr_state == pinctrl_info->curr_state) {
		pr_info("%s: Already in same state\n", __func__);
		goto skip;
	}

	switch (pinctrl_info->curr_state) {
	case STATE_SEC_MI2S_MCLK_SLEEP:
		ret = pinctrl_select_state(pinctrl_info->pinctrl,
					pinctrl_info->sec_mi2s_mclk_sleep);
		if (ret) {
			pr_err("%s: STATE_SEC_MI2S_MCLK_SLEEP select failed with %d\n",
				__func__, ret);
			ret = -EIO;
			goto err;
		}
		lpass_mclk.enable = 0;
		ret = afe_set_lpass_clock_v2(AFE_PORT_ID_SECONDARY_MI2S_RX, &lpass_mclk);
		if (ret < 0) {
			pr_err("%s afe_set_digital_codec_core_clock failed\n",__func__);
			ret = -EIO;
			goto err;
		}
		break;
	case STATE_SEC_MI2S_MCLK_ACTIVE:
		lpass_mclk.enable = 1;
		ret = afe_set_lpass_clock_v2(AFE_PORT_ID_SECONDARY_MI2S_RX, &lpass_mclk);
		if (ret < 0) {
			pr_err("%s afe_set_digital_codec_core_clock failed\n",__func__);
			ret = -EIO;
			goto err;
		}

		ret = pinctrl_select_state(pinctrl_info->pinctrl,
					pinctrl_info->sec_mi2s_mclk_active);
		if (ret) {
			pr_err("%s: STATE_SEC_MI2S_MCLK_ACTIVE select failed with %d\n",
				__func__, ret);
			ret = -EIO;
			goto err;
		}
		break;
	default:
		pr_err("%s: TLMM pin state is invalid\n", __func__);
		return -EINVAL;
	}

err:
	if(ret < 0){
		pr_err("%s disable afe_set_digital_codec_core_clock\n",__func__);
		lpass_mclk.enable = 0;
		ret = afe_set_lpass_clock_v2(AFE_PORT_ID_SECONDARY_MI2S_RX, &lpass_mclk);
		if (ret < 0) {
			pr_err("%s afe_set_digital_codec_core_clock failed\n",__func__);
			ret = -EIO;
			goto err;
		}
	}
skip:
	return ret;
}

static void sma6101_msm_release_pinctrl(struct snd_soc_codec *codec)
{
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	struct msm_pinctrl_info *pinctrl_info = &sma6101->pinctrl_info;

	if (pinctrl_info->pinctrl) {
		devm_pinctrl_put(pinctrl_info->pinctrl);
		pinctrl_info->pinctrl = NULL;
	}
}

static int sma6101_msm_get_pinctrl(struct snd_soc_codec *codec)
{
	struct platform_device *pdev = to_platform_device(codec->dev);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	struct msm_pinctrl_info *pinctrl_info = NULL;
	struct pinctrl *pinctrl;
	int ret = 0;

	pinctrl_info = &sma6101->pinctrl_info;

	if (pinctrl_info == NULL) {
		pr_err("%s: pinctrl_info is NULL\n", __func__);
		return -EINVAL;
	}

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		pr_err("%s: Unable to get pinctrl handle\n", __func__);
		return -EINVAL;
	}
	pinctrl_info->pinctrl = pinctrl;

	pinctrl_info->sec_mi2s_mclk_sleep = pinctrl_lookup_state(pinctrl,
						"sec_mi2s_mclk_sleep");
	if (IS_ERR(pinctrl_info->sec_mi2s_mclk_sleep)) {
		pr_err("%s: could not get sec_mi2s_mclk_sleep pinstate\n", __func__);
		goto err;
	}

	pinctrl_info->sec_mi2s_mclk_active = pinctrl_lookup_state(pinctrl,
						"sec_mi2s_mclk_active");
	if (IS_ERR(pinctrl_info->sec_mi2s_mclk_active)) {
		pr_err("%s: could not get sec_mi2s_mclk_active pinstate\n", __func__);
		goto err;
	}

	/* Reset the TLMM pins to a default state */
	ret = pinctrl_select_state(pinctrl_info->pinctrl,
					pinctrl_info->sec_mi2s_mclk_sleep);
	if (ret != 0) {
		pr_err("%s: Disable TLMM pins failed with %d\n", __func__, ret);
		goto err;
	}

	pinctrl_info->curr_state = STATE_SEC_MI2S_MCLK_SLEEP;

	/* Reset the SEC MI2S MCLK disabled */
	lpass_mclk.enable = 0;
	ret = afe_set_lpass_clock_v2(AFE_PORT_ID_SECONDARY_MI2S_RX, &lpass_mclk);
	if (ret < 0) {
		pr_err("%s afe_set_digital_codec_core_clock failed\n",__func__);
		goto err;
	}

	return 0;

err:
	devm_pinctrl_put(pinctrl);
	pinctrl_info->pinctrl = NULL;
	return -EINVAL;
}

static int sma6101_clock_control(struct clk *clk,int enable)
{
	int ret = 0;
	
	if(enable)
		ret = clk_prepare_enable(clk);
	else
		clk_disable_unprepare(clk);
		
	pr_debug("%s(): clock enable = %d, ret %d\n", __func__, enable,ret);

	return ret;
}

static int set_voice_class_h_mode_for_volume_boost(struct snd_soc_codec *codec,bool enable) {
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	if(enable) {
		if (sma6101->rev_num < REV_NUM_REV3) {
			switch (sma6101->voice_music_class_h_mode_for_volume_boost) {
			case SMA6101_CLASS_H_VOICE_MODE:
				/* Class-H operation Level in voice scenario */
				dev_info(codec->dev, "%s : Class-H operation Level in voice scenario\n",
						__func__);
				regmap_write(sma6101->regmap,
					SMA6101_0D_CLASS_H_CTRL_LVL3, 0xAA);
				regmap_write(sma6101->regmap,
					SMA6101_0F_CLASS_H_CTRL_LVL4, 0x89);
				regmap_write(sma6101->regmap,
					SMA6101_90_CLASS_H_CTRL_LVL1, 0xAA);
				regmap_write(sma6101->regmap,
					SMA6101_91_CLASS_H_CTRL_LVL2, 0x6A);
				break;

			case SMA6101_CLASS_H_MUSIC_MODE:
				/* Class-H operation Level in music scenario */
				dev_info(codec->dev, "%s : Class-H operation Level in music scenario\n",
						__func__);
				regmap_write(sma6101->regmap,
					SMA6101_0D_CLASS_H_CTRL_LVL3, 0xFA);
				regmap_write(sma6101->regmap,
					SMA6101_0F_CLASS_H_CTRL_LVL4, 0x1F);
				regmap_write(sma6101->regmap,
					SMA6101_90_CLASS_H_CTRL_LVL1, 0xFA);
				regmap_write(sma6101->regmap,
					SMA6101_91_CLASS_H_CTRL_LVL2, 0xFA);
				break;

			default:
				dev_info(codec->dev, "%s : Class-H operation Level off\n",
						__func__);
				regmap_write(sma6101->regmap,
					SMA6101_0D_CLASS_H_CTRL_LVL3, 0xFA);
				regmap_write(sma6101->regmap,
					SMA6101_0F_CLASS_H_CTRL_LVL4, 0xF9);
				regmap_write(sma6101->regmap,
					SMA6101_90_CLASS_H_CTRL_LVL1, 0xFA);
				regmap_write(sma6101->regmap,
					SMA6101_91_CLASS_H_CTRL_LVL2, 0xFA);
				break;
			}
		} else if (sma6101->rev_num == REV_NUM_REV3) {
			switch (sma6101->voice_music_class_h_mode_for_volume_boost) {
			case SMA6101_CLASS_H_VOICE_MODE:
			/* FDPEC gain & Boost voltage in voice scenario */
			dev_info(codec->dev, "%s : FDPEC gain 6 & Boost 14V in voice scenario\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x6);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_14V);
			break;

			case SMA6101_CLASS_H_MUSIC_MODE:
			/* FDPEC gain & Boost voltage in music scenario */
			dev_info(codec->dev, "%s : FDPEC gain 8 & Boost 17V in music scenario\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x8);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_17V);
			break;

			default:
			dev_info(codec->dev, "%s : FDPEC gain 6 & Boost 14V default\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x6);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_14V);
			break;
			}
		} else if (sma6101->rev_num == REV_NUM_REV4) {
			switch (sma6101->voice_music_class_h_mode_for_volume_boost) {
			case SMA6101_CLASS_H_VOICE_MODE:
			/* FDPEC gain & Boost voltage in
			 * voice scenario
			 */
			dev_info(codec->dev, "%s : FDPEC gain 6 & Boost 15V in voice scenario\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x6);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_15V);
			break;

			case SMA6101_CLASS_H_MUSIC_MODE:
			/* FDPEC gain & Boost voltage in
			 * music scenario
			 */
			dev_info(codec->dev, "%s : FDPEC gain 8 & Boost 15V in music scenario\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x8);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_15V);
			break;

			case SMA6101_CLASS_H_LN_VOICE_MODE:
			/* FDPEC gain & Boost voltage in
			 * low noise voice scenario
			 */
			dev_info(codec->dev, "%s : FDPEC gain 4 & Boost 10V in Low Noise voice scenario\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x4);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_10V);
			break;

			case SMA6101_CLASS_H_ULN_VOICE_MODE:
			/* FDPEC gain & Boost voltage in
			 * ultra low noise voice scenario
			 */
			dev_info(codec->dev,
				"%s : FDPEC gain 2.6 & Boost 7V in Ultra Low Noise voice scenario\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x2P6);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_7V);
			break;

			default:
			dev_info(codec->dev, "%s : FDPEC gain 6 & Boost 15V default\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x6);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_15V);
			break;
			}
		}
	} else {
		if (sma6101->rev_num < REV_NUM_REV3) {
			switch (sma6101->voice_music_class_h_mode) {
			case SMA6101_CLASS_H_VOICE_MODE:
				/* Class-H operation Level in voice scenario */
				dev_info(codec->dev, "%s : Class-H operation Level in voice scenario\n",
						__func__);
				regmap_write(sma6101->regmap,
					SMA6101_0D_CLASS_H_CTRL_LVL3, 0xAA);
				regmap_write(sma6101->regmap,
					SMA6101_0F_CLASS_H_CTRL_LVL4, 0x89);
				regmap_write(sma6101->regmap,
					SMA6101_90_CLASS_H_CTRL_LVL1, 0xAA);
				regmap_write(sma6101->regmap,
					SMA6101_91_CLASS_H_CTRL_LVL2, 0x6A);
				break;

			case SMA6101_CLASS_H_MUSIC_MODE:
				/* Class-H operation Level in music scenario */
				dev_info(codec->dev, "%s : Class-H operation Level in music scenario\n",
						__func__);
				regmap_write(sma6101->regmap,
					SMA6101_0D_CLASS_H_CTRL_LVL3, 0xFA);
				regmap_write(sma6101->regmap,
					SMA6101_0F_CLASS_H_CTRL_LVL4, 0x1F);
				regmap_write(sma6101->regmap,
					SMA6101_90_CLASS_H_CTRL_LVL1, 0xFA);
				regmap_write(sma6101->regmap,
					SMA6101_91_CLASS_H_CTRL_LVL2, 0xFA);
				break;

			default:
				dev_info(codec->dev, "%s : Class-H operation Level off\n",
						__func__);
				regmap_write(sma6101->regmap,
					SMA6101_0D_CLASS_H_CTRL_LVL3, 0xFA);
				regmap_write(sma6101->regmap,
					SMA6101_0F_CLASS_H_CTRL_LVL4, 0xF9);
				regmap_write(sma6101->regmap,
					SMA6101_90_CLASS_H_CTRL_LVL1, 0xFA);
				regmap_write(sma6101->regmap,
					SMA6101_91_CLASS_H_CTRL_LVL2, 0xFA);
				break;
			}
		} else if (sma6101->rev_num == REV_NUM_REV3) {
			switch (sma6101->voice_music_class_h_mode) {
			case SMA6101_CLASS_H_VOICE_MODE:
			/* FDPEC gain & Boost voltage in voice scenario */
			dev_info(codec->dev, "%s : FDPEC gain 6 & Boost 14V in voice scenario\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x6);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_14V);
			break;

			case SMA6101_CLASS_H_MUSIC_MODE:
			/* FDPEC gain & Boost voltage in music scenario */
			dev_info(codec->dev, "%s : FDPEC gain 8 & Boost 17V in music scenario\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x8);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_17V);
			break;

			default:
			dev_info(codec->dev, "%s : FDPEC gain 6 & Boost 14V default\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x6);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_14V);
			break;
			}
		} else if (sma6101->rev_num == REV_NUM_REV4) {
			switch (sma6101->voice_music_class_h_mode) {
			case SMA6101_CLASS_H_VOICE_MODE:
			/* FDPEC gain & Boost voltage in
			 * voice scenario
			 */
			dev_info(codec->dev, "%s : FDPEC gain 6 & Boost 15V in voice scenario\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x6);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_15V);
			break;

			case SMA6101_CLASS_H_MUSIC_MODE:
			/* FDPEC gain & Boost voltage in
			 * music scenario
			 */
			dev_info(codec->dev, "%s : FDPEC gain 8 & Boost 15V in music scenario\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x8);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_15V);
			break;

			case SMA6101_CLASS_H_LN_VOICE_MODE:
			/* FDPEC gain & Boost voltage in
			 * low noise voice scenario
			 */
			dev_info(codec->dev, "%s : FDPEC gain 4 & Boost 10V in Low Noise voice scenario\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x4);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_10V);
			break;

			case SMA6101_CLASS_H_ULN_VOICE_MODE:
			/* FDPEC gain & Boost voltage in
			 * ultra low noise voice scenario
			 */
			dev_info(codec->dev,
				"%s : FDPEC gain 2.6 & Boost 7V in Ultra Low Noise voice scenario\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x2P6);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_7V);
			break;

			default:
			dev_info(codec->dev, "%s : FDPEC gain 6 & Boost 15V default\n",
					__func__);
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x6);

			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
					TRM_VBST1_MASK, TRM_VBST1_15V);
			break;
			}
		}
	}

	return 0;
}

static const char * const voice_music_class_h_mode_for_volume_boost_text[] = {
	"Voice", "Music", "LN Voice", "ULN Voice", "Off", "On"};

static const struct soc_enum voice_music_class_h_mode_for_volume_boost_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(voice_music_class_h_mode_for_volume_boost_text),
	voice_music_class_h_mode_for_volume_boost_text);

static int voice_music_class_h_mode_for_volume_boost_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = sma6101->voice_music_class_h_mode_for_volume_boost;

	return 0;
}

static int voice_music_class_h_mode_for_volume_boost_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	bool boost_enable = false;
	int temp = 0;

	temp = ucontrol->value.integer.value[0];
	if ((temp < 0) ||(temp > 5))
		return -EINVAL;

	if(temp <=3) { //set volume boost mode //set this variable only boot up by mixer default setting
		sma6101->voice_music_class_h_mode_for_volume_boost= temp;
		dev_info(codec->dev, "%s : set volume boost FDPEC Gain %d\n", __func__,sma6101->voice_music_class_h_mode_for_volume_boost);
		return 0;
	} else if (temp == 4) //set boost disable
		boost_enable = false;
	else if (temp == 5) //set boost enable
		boost_enable = true;

	if(sma6101->voice_music_class_h_mode_for_volume_boost == sma6101->voice_music_class_h_mode) {
		dev_info(codec->dev, "%s : don't need to change FDPEC Gain\n",__func__);
		return 0;
	}

	dev_info(codec->dev, "%s : boost_enable %d\n", 	__func__,boost_enable);

	set_voice_class_h_mode_for_volume_boost(codec,boost_enable);
	return 0;
}
#endif

static const char * const voice_music_class_h_mode_text[] = {
	"Voice", "Music", "LN Voice", "ULN Voice", "Off"};

static const struct soc_enum voice_music_class_h_mode_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(voice_music_class_h_mode_text),
	voice_music_class_h_mode_text);

static int voice_music_class_h_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = sma6101->voice_music_class_h_mode;

	return 0;
}

static int voice_music_class_h_mode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	sma6101->voice_music_class_h_mode = ucontrol->value.integer.value[0];

	if ((sma6101->voice_music_class_h_mode < 0) ||
	(sma6101->voice_music_class_h_mode > 4))
		return -EINVAL;

	if (sma6101->rev_num < REV_NUM_REV3) {
		switch (sma6101->voice_music_class_h_mode) {
		case SMA6101_CLASS_H_VOICE_MODE:
			/* Class-H operation Level in voice scenario */
			dev_info(codec->dev, "%s : Class-H operation Level in voice scenario\n",
					__func__);
			regmap_write(sma6101->regmap,
				SMA6101_0D_CLASS_H_CTRL_LVL3, 0xAA);
			regmap_write(sma6101->regmap,
				SMA6101_0F_CLASS_H_CTRL_LVL4, 0x89);
			regmap_write(sma6101->regmap,
				SMA6101_90_CLASS_H_CTRL_LVL1, 0xAA);
			regmap_write(sma6101->regmap,
				SMA6101_91_CLASS_H_CTRL_LVL2, 0x6A);
			break;

		case SMA6101_CLASS_H_MUSIC_MODE:
			/* Class-H operation Level in music scenario */
			dev_info(codec->dev, "%s : Class-H operation Level in music scenario\n",
					__func__);
			regmap_write(sma6101->regmap,
				SMA6101_0D_CLASS_H_CTRL_LVL3, 0xFA);
			regmap_write(sma6101->regmap,
				SMA6101_0F_CLASS_H_CTRL_LVL4, 0x1F);
			regmap_write(sma6101->regmap,
				SMA6101_90_CLASS_H_CTRL_LVL1, 0xFA);
			regmap_write(sma6101->regmap,
				SMA6101_91_CLASS_H_CTRL_LVL2, 0xFA);
			break;

		default:
			dev_info(codec->dev, "%s : Class-H operation Level off\n",
					__func__);
			regmap_write(sma6101->regmap,
				SMA6101_0D_CLASS_H_CTRL_LVL3, 0xFA);
			regmap_write(sma6101->regmap,
				SMA6101_0F_CLASS_H_CTRL_LVL4, 0xF9);
			regmap_write(sma6101->regmap,
				SMA6101_90_CLASS_H_CTRL_LVL1, 0xFA);
			regmap_write(sma6101->regmap,
				SMA6101_91_CLASS_H_CTRL_LVL2, 0xFA);
			break;
		}
	} else if (sma6101->rev_num == REV_NUM_REV3) {
		switch (sma6101->voice_music_class_h_mode) {
		case SMA6101_CLASS_H_VOICE_MODE:
		/* FDPEC gain & Boost voltage in voice scenario */
		dev_info(codec->dev, "%s : FDPEC gain 6 & Boost 14V in voice scenario\n",
				__func__);
		sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x6);

		regmap_update_bits(sma6101->regmap,
			SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_14V);
		break;

		case SMA6101_CLASS_H_MUSIC_MODE:
		/* FDPEC gain & Boost voltage in music scenario */
		dev_info(codec->dev, "%s : FDPEC gain 8 & Boost 17V in music scenario\n",
				__func__);
		sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x8);

		regmap_update_bits(sma6101->regmap,
			SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_17V);
		break;

		default:
		dev_info(codec->dev, "%s : FDPEC gain 6 & Boost 14V default\n",
				__func__);
		sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x6);

		regmap_update_bits(sma6101->regmap,
			SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_14V);
		break;
		}
	} else if (sma6101->rev_num == REV_NUM_REV4) {
		switch (sma6101->voice_music_class_h_mode) {
		case SMA6101_CLASS_H_VOICE_MODE:
		/* FDPEC gain & Boost voltage in
		 * voice scenario
		 */
		dev_info(codec->dev, "%s : FDPEC gain 6 & Boost 15V in voice scenario\n",
				__func__);
		sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x6);

		regmap_update_bits(sma6101->regmap,
			SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_15V);
		break;

		case SMA6101_CLASS_H_MUSIC_MODE:
		/* FDPEC gain & Boost voltage in
		 * music scenario
		 */
		dev_info(codec->dev, "%s : FDPEC gain 8 & Boost 15V in music scenario\n",
				__func__);
		sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x8);

		regmap_update_bits(sma6101->regmap,
			SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_15V);
		break;

		case SMA6101_CLASS_H_LN_VOICE_MODE:
		/* FDPEC gain & Boost voltage in
		 * low noise voice scenario
		 */
		dev_info(codec->dev, "%s : FDPEC gain 4 & Boost 10V in Low Noise voice scenario\n",
				__func__);
		sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x4);

		regmap_update_bits(sma6101->regmap,
			SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_10V);
		break;

		case SMA6101_CLASS_H_ULN_VOICE_MODE:
		/* FDPEC gain & Boost voltage in
		 * ultra low noise voice scenario
		 */
		dev_info(codec->dev,
			"%s : FDPEC gain 2.6 & Boost 7V in Ultra Low Noise voice scenario\n",
				__func__);
		sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x2P6);

		regmap_update_bits(sma6101->regmap,
			SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_7V);
		break;

		default:
		dev_info(codec->dev, "%s : FDPEC gain 6 & Boost 15V default\n",
				__func__);
		sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x6);

		regmap_update_bits(sma6101->regmap,
			SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_15V);
		break;
		}
	}
//sar_backoff for piezo Amp
#if defined(CONFIG_MACH_LGE)
	if( edev_get_sar_backoff == NULL ) {
		edev_get_sar_backoff = extcon_get_extcon_dev("sar_backoff");
		pr_info("%s got edev_get_sar_backoff 0x%p\n", __func__, edev_get_sar_backoff);
	}

	if( edev_get_sar_backoff != NULL ) {
		if( piezo_receiver == false && sma6101->voice_music_class_h_mode != SMA6101_CLASS_H_MUSIC_MODE) {	 // rcvcall open
			piezo_receiver = true;
			pr_debug("%s enable piezo_receiver %d, edev_get_sar_backoff->state = %d\n",
				__func__, (int)piezo_receiver, edev_get_sar_backoff->state);
			extcon_set_state_sync(edev_get_sar_backoff, EXTCON_MECHANICAL, piezo_receiver);
			pr_info("%s : enable SAR backoff through piezo_receiver\n", __func__);
		} else if( piezo_receiver == true && sma6101->voice_music_class_h_mode == SMA6101_CLASS_H_MUSIC_MODE) {    // rcvcall closed
			piezo_receiver = false;
			pr_debug("%s disable piezo_receiver %d, edev_get_sar_backoff->state = %d\n",
				__func__, (int)piezo_receiver, edev_get_sar_backoff->state);
			extcon_set_state_sync(edev_get_sar_backoff, EXTCON_MECHANICAL, piezo_receiver);
			pr_info("%s : disable SAR backoff through piezo_receiver\n", __func__);
		}
	}
#endif

	return 0;
}

static int sma6101_put_volsw(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int ret;
	unsigned int val;

	mutex_lock(&sma6101->lock);

	ret = snd_soc_put_volsw(kcontrol, ucontrol);
	if (ret < 0) {
		mutex_unlock(&sma6101->lock);
		return ret;
	}
	regmap_read(sma6101->regmap, reg, &val);

	if (val != sma6101->init_vol) {
		dev_dbg(codec->dev, "%s :  init vol[%d] updated to vol[%d]\n",
		__func__, sma6101->init_vol, val);

		sma6101->init_vol = val;
	}
	mutex_unlock(&sma6101->lock);

	return 0;
}

static const struct snd_kcontrol_new sma6101_snd_controls[] = {

SOC_SINGLE_EXT("Power Up(1:Up,0:Down)", SND_SOC_NOPM, 0, 1, 0,
	power_up_down_control_get, power_up_down_control_put),
SOC_SINGLE_EXT("Force AMP Power Down", SND_SOC_NOPM, 0, 1, 0,
	power_down_control_get, power_down_control_put),
SOC_ENUM_EXT("External Clock System", sma6101_clk_system_enum,
	sma6101_clk_system_get, sma6101_clk_system_put),

SOC_SINGLE("I2S/PCM Clock mode(1:Master,2:Slave)",
		SMA6101_01_INPUT1_CTRL1, 7, 1, 0),
SOC_ENUM_EXT("I2S input format(I2S,LJ,RJ)", sma6101_input_format_enum,
	sma6101_input_format_get, sma6101_input_format_put),
SOC_SINGLE("First-channel pol for I2S(1:High,0:Low)",
		SMA6101_01_INPUT1_CTRL1, 3, 1, 0),
SOC_SINGLE("Data written on SCK edge(1:rise,0:fall)",
		SMA6101_01_INPUT1_CTRL1, 2, 1, 0),

SOC_ENUM_EXT("Input audio mode", sma6101_in_audio_mode_enum,
	sma6101_in_audio_mode_get, sma6101_in_audio_mode_put),
SOC_SINGLE("Data inversion(1:right-first,0:left-first)",
		SMA6101_02_INPUT1_CTRL2, 5, 1, 0),
SOC_SINGLE("Decoding select(1:A-law,0:u-law)",
		SMA6101_02_INPUT1_CTRL2, 4, 1, 0),
SOC_SINGLE("Companding PCM data(1:companding,2:linear)",
		SMA6101_02_INPUT1_CTRL2, 3, 1, 0),
SOC_SINGLE("PCM sample freq(1:16kHZ,0:8kHz)", SMA6101_02_INPUT1_CTRL2, 2, 1, 0),
SOC_SINGLE("PCM stereo/mono sel(1:stereo,0:mono)",
		SMA6101_02_INPUT1_CTRL2, 1, 1, 0),
SOC_SINGLE("PCM data length(1:16-bit,0:8-bit)",
		SMA6101_02_INPUT1_CTRL2, 0, 1, 0),

SOC_SINGLE("SR converted bypass(1:bypass,0:normal)",
		SMA6101_03_INPUT1_CTRL3, 4, 1, 0),
SOC_ENUM_EXT("Number of slots per sampling period(PCM)",
		sma6101_pcm_n_slot_enum, sma6101_pcm_n_slot_get,
		sma6101_pcm_n_slot_put),

SOC_ENUM_EXT("Position of the first sample at 8,16kHz", sma6101_pcm1_slot_enum,
	sma6101_pcm1_slot_get, sma6101_pcm1_slot_put),
SOC_ENUM_EXT("Position of the second sample at 16kHz", sma6101_pcm2_slot_enum,
	sma6101_pcm2_slot_get, sma6101_pcm2_slot_put),

SND_SOC_BYTES_EXT("BrownOut Set 1~4", 4,
	brown_out_set1_get, brown_out_set1_put),

SOC_ENUM_EXT("Port In/Out port configuration", sma6101_port_config_enum,
	sma6101_port_config_get, sma6101_port_config_put),
SOC_ENUM_EXT("Port Output Format", sma6101_port_out_format_enum,
	sma6101_port_out_format_get, sma6101_port_out_format_put),
SOC_ENUM_EXT("Port Output Source", sma6101_port_out_sel_enum,
	sma6101_port_out_sel_get, sma6101_port_out_sel_put),

SOC_SINGLE_EXT_TLV("Speaker Volume", SMA6101_0A_SPK_VOL,
	0, 0xA8, 0, snd_soc_get_volsw, sma6101_put_volsw, sma6101_spk_tlv),

SND_SOC_BYTES_EXT("BrownOut Set 5~6", 2,
	brown_out_set2_get, brown_out_set2_put),

SOC_ENUM_EXT("Attack level control(0x0D)", sma6101_attack_lvl_3_enum,
	sma6101_attack_lvl_3_get, sma6101_attack_lvl_3_put),
SOC_ENUM_EXT("Release time control(0x0D)", sma6101_release_time_3_enum,
	sma6101_release_time_3_get, sma6101_release_time_3_put),

SOC_ENUM_EXT("Volume slope", sma6101_vol_slope_enum,
	sma6101_vol_slope_get, sma6101_vol_slope_put),
SOC_ENUM_EXT("Mute slope", sma6101_mute_slope_enum,
	sma6101_mute_slope_get, sma6101_mute_slope_put),
SOC_SINGLE("Speaker Mute Switch(1:muted,0:un-muted)",
		SMA6101_0E_MUTE_VOL_CTRL, 0, 1, 0),

SOC_ENUM_EXT("Attack level control(0x0F)", sma6101_attack_lvl_4_enum,
	sma6101_attack_lvl_4_get, sma6101_attack_lvl_4_put),
SOC_ENUM_EXT("Release time control(0x0F)", sma6101_release_time_4_enum,
	sma6101_release_time_4_get, sma6101_release_time_4_put),

SOC_ENUM_EXT("Speaker Mode", sma6101_spkmode_enum,
	sma6101_spkmode_get, sma6101_spkmode_put),

SOC_SINGLE("Speaker EQ(1:enable,0:bypass)", SMA6101_11_SYSTEM_CTRL2, 7, 1, 0),
SOC_SINGLE("Speaker Bass(1:enable,0:bypass)", SMA6101_11_SYSTEM_CTRL2, 6, 1, 0),
SOC_SINGLE("Speaker Comp/Limiter(1:enable,0:bypass)",
		SMA6101_11_SYSTEM_CTRL2, 5, 1, 0),
SOC_SINGLE("LR_DATA_SW(1:swap,0:normal)", SMA6101_11_SYSTEM_CTRL2, 4, 1, 0),
SOC_SINGLE("Mono Mix(1:enable,0:disable)", SMA6101_11_SYSTEM_CTRL2, 0, 1, 0),

SOC_ENUM_EXT("Input gain", sma6101_input_gain_enum,
	sma6101_input_gain_get, sma6101_input_gain_put),
SOC_ENUM_EXT("Input gain for right channel", sma6101_input_r_gain_enum,
	sma6101_input_r_gain_get, sma6101_input_r_gain_put),

SOC_SINGLE("ClassH2 Off(1:disable,0:enable)", SMA6101_13_FDPEC_CTRL1, 7, 1, 0),
SOC_SINGLE("ClassH1 Off(1:disable,0:enable)", SMA6101_13_FDPEC_CTRL1, 6, 1, 0),
SOC_SINGLE("SDM Sync(1:disable,0:normal)", SMA6101_13_FDPEC_CTRL1, 5, 1, 0),
SOC_ENUM_EXT("FDPEC Gain(3:8,2:6,1:4,0:2.6)",
	fdpec_gain_control_enum,
	fdpec_gain_control_get, fdpec_gain_control_put),
SOC_ENUM_EXT("HDC OPAMP Current", sma6101_fdpec_i_enum,
	sma6101_fdpec_i_get, sma6101_fdpec_i_put),
SOC_SINGLE("FDPEC Closed Loop(1:enable,0:disable)",
		SMA6101_13_FDPEC_CTRL1, 0, 1, 0),

SOC_ENUM_EXT("Speaker HYSFB", sma6101_spk_hysfb_enum,
	sma6101_spk_hysfb_get, sma6101_spk_hysfb_put),
SND_SOC_BYTES_EXT("Speaker BDELAY", 1, spk_bdelay_get, spk_bdelay_put),

SND_SOC_BYTES_EXT("Bass Boost SPK Coeff", 7,
	bass_spk_coeff_get, bass_spk_coeff_put),

SND_SOC_BYTES_EXT("Brown Out Protection 16~20", 5,
	brown_out_pt2_get, brown_out_pt2_put),

SOC_SINGLE("TDM mode(1:enable,0:disable)", SMA6101_21_TDM_RX, 6, 1, 0),
SOC_ENUM_EXT("Position of left channel at TDM mode", sma6101_tdm_l_slot_enum,
	sma6101_tdm_l_slot_get, sma6101_tdm_l_slot_put),
SOC_ENUM_EXT("Position of right channel at TDM mode", sma6101_tdm_r_slot_enum,
	sma6101_tdm_r_slot_get, sma6101_tdm_r_slot_put),

SND_SOC_BYTES_EXT("DRC SPK Coeff", 4,
	comp_lim_spk_coeff_get, comp_lim_spk_coeff_put),

SOC_SINGLE("SDM VLINK Disable(1:disable,0:enable)",
		SMA6101_33_SDM_CTRL, 3, 1, 0),
SOC_SINGLE("SDM Q Select(1:1/8,0:1/4)", SMA6101_33_SDM_CTRL, 2, 1, 0),

SOC_SINGLE("Edge displacement(1:disable,0:enable)",
		SMA6101_36_PROTECTION, 7, 1, 0),
SOC_SINGLE("SRC random jitter(1:disable,0:added)",
		SMA6101_36_PROTECTION, 4, 1, 0),
SOC_SINGLE("OCP SPK output(1:disable,0:enable)",
		SMA6101_36_PROTECTION, 3, 1, 0),
SOC_SINGLE("OCP mode(1:permanent SD,0:auto recover)",
		SMA6101_36_PROTECTION, 2, 1, 0),
SOC_ENUM_EXT("OTP MODE", sma6101_otp_mode_enum,
		sma6101_otp_mode_get, sma6101_otp_mode_put),

SND_SOC_BYTES_EXT("SlopeCTRL", 1, slope_ctrl_get, slope_ctrl_put),

SND_SOC_BYTES_EXT("Disable class-H Level 1~6", 3,
	dis_class_h_lvl_get, dis_class_h_lvl_put),

SND_SOC_BYTES_EXT("Test mode(Test 1~3, ATEST 1~2)", 5,
	test_mode_get, test_mode_put),

SND_SOC_BYTES_EXT("EQ Ctrl Band1", 15, eq_ctrl_band1_get, eq_ctrl_band1_put),
SND_SOC_BYTES_EXT("EQ Ctrl Band2", 15, eq_ctrl_band2_get, eq_ctrl_band2_put),
SND_SOC_BYTES_EXT("EQ Ctrl Band3", 15, eq_ctrl_band3_get, eq_ctrl_band3_put),
SND_SOC_BYTES_EXT("EQ Ctrl Band4", 15, eq_ctrl_band4_get, eq_ctrl_band4_put),
SND_SOC_BYTES_EXT("EQ Ctrl Band5", 15, eq_ctrl_band5_get, eq_ctrl_band5_put),

SND_SOC_BYTES_EXT("PLL Setting", 5, pll_setting_get, pll_setting_put),

SOC_ENUM_EXT("Attack level control(0x90)", sma6101_attack_lvl_1_enum,
	sma6101_attack_lvl_1_get, sma6101_attack_lvl_1_put),
SOC_ENUM_EXT("Release time control(0x90)", sma6101_release_time_1_enum,
	sma6101_release_time_1_get, sma6101_release_time_1_put),
SOC_ENUM_EXT("Attack level control(0x91)", sma6101_attack_lvl_2_enum,
	sma6101_attack_lvl_2_get, sma6101_attack_lvl_2_put),
SOC_ENUM_EXT("Release time control(0x91)", sma6101_release_time_2_enum,
	sma6101_release_time_2_get, sma6101_release_time_2_put),

SOC_ENUM_EXT("FDPEC Gain Trimming", sma6101_fdpec_gain_trm_enum,
	sma6101_fdpec_gain_trm_get, sma6101_fdpec_gain_trm_put),
SOC_SINGLE("REC CUR Mode(1:Normal,0:Enhanced)",
		SMA6101_92_FDPEC_CTRL2, 5, 1, 0),
SOC_SINGLE("REC CUR Control(1:disable,0:enable)",
		SMA6101_92_FDPEC_CTRL2, 4, 1, 0),
SOC_SINGLE("PWM frequency(1:740kHz,0:680kHz)",
		SMA6101_92_FDPEC_CTRL2, 3, 1, 0),
SOC_SINGLE("Diff OPAMP bias current(1:80uA,0:40uA)",
		SMA6101_92_FDPEC_CTRL2, 1, 1, 0),

SOC_ENUM_EXT("Trimming of VBG reference", sma6101_trm_vref_enum,
	sma6101_trm_vref_get, sma6101_trm_vref_put),
SOC_ENUM_EXT("Trimming of boost output voltage", sma6101_trm_vbst1_enum,
	sma6101_trm_vbst1_get, sma6101_trm_vbst1_put),

SOC_ENUM_EXT("Trimming I-gain of boost voltage loop", sma6101_trm_comp2_enum,
	sma6101_trm_comp2_get, sma6101_trm_comp2_put),
SOC_ENUM_EXT("Trimming of switching frequency", sma6101_trm_osc_enum,
	sma6101_trm_osc_get, sma6101_trm_osc_put),
SOC_ENUM_EXT("Trimming slope compensation", sma6101_trm_rmp_enum,
	sma6101_trm_rmp_get, sma6101_trm_rmp_put),

SOC_ENUM_EXT("Trimming of over current limit", sma6101_trm_ocl_enum,
	sma6101_trm_ocl_get, sma6101_trm_ocl_put),
SOC_ENUM_EXT("Trimming P-gain and type II I-gain", sma6101_trm_comp_enum,
	sma6101_trm_comp_get, sma6101_trm_comp_put),

SOC_ENUM_EXT("Trimming of driver deadtime", sma6101_trm_dt_enum,
	sma6101_trm_dt_get, sma6101_trm_dt_put),
SOC_SINGLE("Boost current limit(1:enable,0:disable)",
		SMA6101_96_BOOST_CTRL3, 3, 1, 0),
SOC_SINGLE("Boost OCP(1:enable,0:disable)",
		SMA6101_96_BOOST_CTRL3, 2, 1, 0),
SOC_ENUM_EXT("Trimming of switching slew", sma6101_trm_slw_enum,
	sma6101_trm_slw_get, sma6101_trm_slw_put),

SOC_ENUM_EXT("Trimming of boost reference", sma6101_trm_vbst2_enum,
	sma6101_trm_vbst2_get, sma6101_trm_vbst2_put),
SOC_ENUM_EXT("Trimming of minimum on-time", sma6101_trm_tmin_enum,
	sma6101_trm_tmin_get, sma6101_trm_tmin_put),

SOC_SINGLE("PLL Lock Skip Mode(1:disable,0:enable)",
		SMA6101_A2_TOP_MAN1, 7, 1, 0),
SOC_SINGLE("PLL Power Down(1:PLL PD en,0:PLL oper)",
		SMA6101_A2_TOP_MAN1, 6, 1, 0),
SOC_SINGLE("MCLK Selection(1:Ext CLK,0:PLL CLK)", SMA6101_A2_TOP_MAN1, 5, 1, 0),
SOC_SINGLE("PLL Reference Clock1(1:Int OSC,0:Ext CLK)",
		SMA6101_A2_TOP_MAN1, 4, 1, 0),
SOC_SINGLE("PLL Reference Clock2(1:SCK,0:PLL_REF_CLK1)",
		SMA6101_A2_TOP_MAN1, 3, 1, 0),
SOC_SINGLE("DAC Down Conversion(1:Down Con,0:Normal)",
		SMA6101_A2_TOP_MAN1, 2, 1, 0),
SOC_SINGLE("SDO Pad Output Control(1:LRCK L,0:LRCK H)",
		SMA6101_A2_TOP_MAN1, 1, 1, 0),
SOC_SINGLE("SDO data selection(1:DAC(48kHz only,0:DAC)",
		SMA6101_A2_TOP_MAN1, 0, 1, 0),

SOC_SINGLE("Monitoring at SDO(1:OSC,0:PLL)", SMA6101_A3_TOP_MAN2, 7, 1, 0),
SOC_SINGLE("Test clock output en(1:Clock out,0:Normal)",
		SMA6101_A3_TOP_MAN2, 6, 1, 0),
SOC_SINGLE("PLL SDM PD(1:SDM off,0:SDM on)", SMA6101_A3_TOP_MAN2, 5, 1, 0),
SOC_SINGLE("SDO output(1:High-Z,0:Normal output)",
		SMA6101_A3_TOP_MAN2, 3, 1, 0),
SOC_SINGLE("Clock Monitoring(1:Not,0:Monitoring)",
		SMA6101_A3_TOP_MAN2, 1, 1, 0),
SOC_SINGLE("OSC PD(1:Power down,0:Normal operation)",
		SMA6101_A3_TOP_MAN2, 0, 1, 0),

SOC_ENUM_EXT("SDO Output Format", sma6101_o_format_enum,
	sma6101_o_format_get, sma6101_o_format_put),
SOC_ENUM_EXT("SDO SCK rate", sma6101_sck_rate_enum,
	sma6101_sck_rate_get, sma6101_sck_rate_put),
SOC_ENUM_EXT("SDO WD Length", sma6101_wd_length_enum,
	sma6101_wd_length_get, sma6101_wd_length_put),

SOC_SINGLE("TDM format(1:Long sync,0:Short sync)", SMA6101_A5_TDM_TX1, 7, 1, 0),
SOC_SINGLE("TDM stereo/mono select(1:Stereo,0:Mono)",
		SMA6101_A5_TDM_TX1, 6, 1, 0),
SOC_SINGLE("TDM data length(1:16bit,0:8bit)", SMA6101_A5_TDM_TX1, 5, 1, 0),
SOC_SINGLE("TDM clock polarity(1:Falling,0:Rising)",
		SMA6101_A5_TDM_TX1, 4, 1, 0),
SOC_ENUM_EXT("TDM number of slot", sma6101_tdm_n_slot_enum,
	sma6101_tdm_n_slot_get, sma6101_tdm_n_slot_put),

SOC_ENUM_EXT("TDM slot1 position", sma6101_tdm_slot1_enum,
	sma6101_tdm_slot1_get, sma6101_tdm_slot1_put),
SOC_ENUM_EXT("TDM slot2 position", sma6101_tdm_slot2_enum,
	sma6101_tdm_slot2_get, sma6101_tdm_slot2_put),

SOC_ENUM_EXT("Clock monitoring time selection",
	sma6101_test_clock_mon_time_sel_enum,
	sma6101_test_clock_mon_time_sel_get,
	sma6101_test_clock_mon_time_sel_put),
SOC_SINGLE("CLK path select(1:External clk,0:SCK)",
		SMA6101_A7_TOP_MAN3, 5, 1, 0),
SOC_SINGLE("Test for limiter operating(1:En,0:Dis)",
		SMA6101_A7_TOP_MAN3, 3, 1, 0),
SOC_SINGLE("SDO IO control(1:Output only,0:normal)",
		SMA6101_A7_TOP_MAN3, 2, 1, 0),
SOC_SINGLE("Master mode enable PADs(1:Master,0:Slave)",
		SMA6101_A7_TOP_MAN3, 0, 1, 0),

SOC_SINGLE("Class H input(1:Audio,0:TONE+AUDIO)",
		SMA6101_A8_TONE_GENERATOR, 7, 1, 0),
SOC_SINGLE("Bypass tone Generator(1:Bypass,0:Normal)",
		SMA6101_A8_TONE_GENERATOR, 6, 1, 0),
SOC_SINGLE("Tone audio mixing(1:enable,0:disable)",
		SMA6101_A8_TONE_GENERATOR, 5, 1, 0),
SOC_ENUM_EXT("Tone generator frequency", sma6101_tone_freq_enum,
	sma6101_tone_freq_get, sma6101_tone_freq_put),
SOC_SINGLE("Tone generator switch(1:On,0:Off)",
		SMA6101_A8_TONE_GENERATOR, 0, 1, 0),

SND_SOC_BYTES_EXT("Tone/Fine Volume", 1,
	tone_fine_volume_get, tone_fine_volume_put),

SND_SOC_BYTES_EXT("PLL_A_D Setting", 2,
	pll_a_d_setting_get, pll_a_d_setting_put),
SND_SOC_BYTES_EXT("PLL LDO Control", 1, pll_ldo_ctrl_get, pll_ldo_ctrl_put),

SOC_ENUM_EXT("Speaker OCP Delay for sync circuit", sma6101_sync_delay_enum,
	sma6101_sync_delay_get, sma6101_sync_delay_put),
SOC_ENUM_EXT("Speaker OCP Filter time", sma6101_ocp_filter_enum,
	sma6101_ocp_filter_get, sma6101_ocp_filter_put),
SOC_ENUM_EXT("Speaker OCP Level", sma6101_ocp_lvl_enum,
	sma6101_ocp_lvl_get, sma6101_ocp_lvl_put),

SOC_SINGLE("IRQ disable(1:High-Z,0:Normal)", SMA6101_AE_TOP_MAN4, 6, 1, 0),

SND_SOC_BYTES_EXT("VIN Sensing", 1, vin_sensing_get, vin_sensing_put),

SND_SOC_BYTES_EXT("Brown Out Protection 0~15", 16,
	brown_out_pt_get, brown_out_pt_put),

SOC_ENUM_EXT("Class H mode(4:X,3:ULNV,2:LNV,1:M,0:V)",
	voice_music_class_h_mode_enum,
	voice_music_class_h_mode_get, voice_music_class_h_mode_put),
#ifdef CONFIG_MACH_LGE
    SOC_SINGLE_EXT("SMA6101 Reset Control", SND_SOC_NOPM, 0, 1, 0,
                    sma6101_reset_gpio_get,
                    sma6101_reset_gpio_put),
    SOC_SINGLE_EXT("SMA6101 Low Battery Status", SND_SOC_NOPM, 0, 3, 0,
                    sma6101_low_battery_status_get,
                    sma6101_low_battery_status_put),
	SOC_ENUM_EXT("Volume Boost Control",
		voice_music_class_h_mode_for_volume_boost_enum,
		voice_music_class_h_mode_for_volume_boost_get, voice_music_class_h_mode_for_volume_boost_put),
#endif
};

static int sma6101_startup(struct snd_soc_codec *codec)
{
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	if (sma6101->amp_power_status) {
		dev_info(codec->dev, "%s : %s\n",
			__func__, "Already AMP Power on");
		return 0;
	}

	dev_info(codec->dev, "%s\n", __func__);

	if (sma6101->rev_num == REV_NUM_REV3) {
		/* Trimming of VBG reference - 1.1125V,
		 * Trimming of boost output voltage - 6.0V
		 */
		regmap_write(sma6101->regmap, SMA6101_93_BOOST_CTRL0, 0xF0);
	}

	/* Add code to turn PWM off before global power on */
	regmap_update_bits(sma6101->regmap, SMA6101_0E_MUTE_VOL_CTRL,
			SPK_MUTE_MASK, SPK_MUTE);
	regmap_update_bits(sma6101->regmap, SMA6101_10_SYSTEM_CTRL1,
			SPK_MODE_MASK, SPK_OFF);

	regmap_update_bits(sma6101->regmap, SMA6101_00_SYSTEM_CTRL,
			POWER_MASK, POWER_ON);

	/* Improved boost OCP interrupt issue when turning on the amp */
	msleep(20);

	if (sma6101->rev_num == REV_NUM_REV0) {
	/* Temporary code(REV0) - Turn off the power during music playback,
	 * there is an issue that the NMOS will die as the overcurrent flows.
	 * To improve this, Lower the boost level before turning off the power.
	 */
		regmap_update_bits(sma6101->regmap, SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_6V);
		dev_info(codec->dev, "%s : Trimming of boost output voltage 6V\n",
				__func__);

		regmap_write(sma6101->regmap,
				SMA6101_0F_CLASS_H_CTRL_LVL4, 0x03);
		dev_info(codec->dev, "%s : ATTACK_LVL disabled and BOOST LV4 always on\n",
				__func__);
	} else if (sma6101->rev_num == REV_NUM_REV3) {
		/* Changed gain to reduce pop noise */
		regmap_update_bits(sma6101->regmap, SMA6101_13_FDPEC_CTRL1,
			FDPEC_GAIN_MASK, FDPEC_GAIN_6);

		if (sma6101->voice_music_class_h_mode ==
			SMA6101_CLASS_H_MUSIC_MODE) {
			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_17V);
		} else {
			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_14V);
		}

		usleep_range(1000, 1010);

		regmap_update_bits(sma6101->regmap, SMA6101_93_BOOST_CTRL0,
			TRM_VREF_MASK, TRM_VREF_0_IDX);
	} else if (sma6101->rev_num == REV_NUM_REV4) {
		regmap_update_bits(sma6101->regmap,
			SMA6101_90_CLASS_H_CTRL_LVL1,
			CLASS_H_RELEASE_TIME_MASK, CLASS_H_RELEASE_TIME_120);
		regmap_update_bits(sma6101->regmap,
			SMA6101_91_CLASS_H_CTRL_LVL2,
			CLASS_H_RELEASE_TIME_MASK, CLASS_H_RELEASE_TIME_100);
		regmap_update_bits(sma6101->regmap,
			SMA6101_0D_CLASS_H_CTRL_LVL3,
			CLASS_H_RELEASE_TIME_MASK, CLASS_H_RELEASE_TIME_80);
		regmap_update_bits(sma6101->regmap,
			SMA6101_0F_CLASS_H_CTRL_LVL4,
			CLASS_H_RELEASE_TIME_MASK, CLASS_H_RELEASE_TIME_60);

		/* Changed gain to reduce pop noise */
		if (sma6101->voice_music_class_h_mode ==
				SMA6101_CLASS_H_MUSIC_MODE) {
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x2P6);
		}

		if (sma6101->voice_music_class_h_mode ==
			SMA6101_CLASS_H_MUSIC_MODE) {
			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_15V);
		} else if (sma6101->voice_music_class_h_mode ==
			SMA6101_CLASS_H_LN_VOICE_MODE) {
			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_10V);
		} else if (sma6101->voice_music_class_h_mode ==
			SMA6101_CLASS_H_ULN_VOICE_MODE) {
			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_7V);
		} else {
			regmap_update_bits(sma6101->regmap,
				SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_15V);
		}

		usleep_range(1000, 1010);
	}

	/* Improved high frequency noise issue when voice call scenario */
	if (sma6101->voice_music_class_h_mode !=
			SMA6101_CLASS_H_MUSIC_MODE) {
		regmap_update_bits(sma6101->regmap, SMA6101_A8_TONE_GENERATOR,
			TONE_FREQ_MASK, TONE_FREQ_50);
		regmap_update_bits(sma6101->regmap, SMA6101_A9_TONE_FINE_VOL,
			TONE_VOL_MASK, TONE_VOL_M_30);
	}

	if (sma6101->stereo_two_chip == true) {
		/* SPK Mode (Stereo) */
		regmap_update_bits(sma6101->regmap, SMA6101_10_SYSTEM_CTRL1,
				SPK_MODE_MASK, SPK_STEREO);
	} else {
		/* SPK Mode (Mono) */
		regmap_update_bits(sma6101->regmap, SMA6101_10_SYSTEM_CTRL1,
				SPK_MODE_MASK, SPK_MONO);
	}

	/* Improved high frequency noise issue when voice call scenario */
	regmap_update_bits(sma6101->regmap, SMA6101_A8_TONE_GENERATOR,
			TONE_ON_MASK, TONE_ON);

	if (sma6101->voice_music_class_h_mode ==
			SMA6101_CLASS_H_MUSIC_MODE)
		sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x8);
	else if (sma6101->voice_music_class_h_mode ==
			SMA6101_CLASS_H_LN_VOICE_MODE)
		sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x4);
	else if (sma6101->voice_music_class_h_mode ==
			SMA6101_CLASS_H_ULN_VOICE_MODE)
		sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x2P6);
	else
		sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x6);

	if (sma6101->check_thermal_vbat_enable) {
		if ((sma6101->voice_music_class_h_mode ==
				SMA6101_CLASS_H_MUSIC_MODE)
			&& sma6101->check_thermal_vbat_period > 0) {
			queue_delayed_work(system_freezable_wq,
				&sma6101->check_thermal_vbat_work,
				msecs_to_jiffies(100));
		}
	}

	if (sma6101->check_thermal_fault_enable) {
		if (sma6101->check_thermal_fault_period > 0)
			queue_delayed_work(system_freezable_wq,
				&sma6101->check_thermal_fault_work,
				sma6101->check_thermal_fault_period * HZ);
		else
			queue_delayed_work(system_freezable_wq,
				&sma6101->check_thermal_fault_work,
					CHECK_FAULT_PERIOD_TIME * HZ);
	}

	sma6101->amp_power_status = true;

	regmap_update_bits(sma6101->regmap, SMA6101_0E_MUTE_VOL_CTRL,
				SPK_MUTE_MASK, SPK_UNMUTE);

	return 0;
}


static int sma6101_shutdown(struct snd_soc_codec *codec)
{
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	unsigned int cur_vol;

	if (!(sma6101->amp_power_status)) {
		dev_info(codec->dev, "%s : %s\n",
			__func__, "Already AMP Shutdown");
		return 0;
	}

	dev_info(codec->dev, "%s\n", __func__);

	regmap_update_bits(sma6101->regmap, SMA6101_0E_MUTE_VOL_CTRL,
				SPK_MUTE_MASK, SPK_MUTE);

	cancel_delayed_work_sync(&sma6101->check_thermal_vbat_work);
	cancel_delayed_work_sync(&sma6101->check_thermal_fault_work);

	/* Mute slope time(15ms) */
	usleep_range(15000, 15010);

	if (sma6101->rev_num == REV_NUM_REV4) {
		/* To improve the Boost OCP issue,
		 * set the release time to zero
		 */
		regmap_update_bits(sma6101->regmap,
			SMA6101_92_FDPEC_CTRL2,
			REC_CUR_MODE_MASK, REC_CUR_MODE_ENHANCED);
		regmap_update_bits(sma6101->regmap,
			SMA6101_90_CLASS_H_CTRL_LVL1,
			CLASS_H_RELEASE_TIME_MASK, CLASS_H_RELEASE_TIME_0);
		regmap_update_bits(sma6101->regmap,
			SMA6101_91_CLASS_H_CTRL_LVL2,
			CLASS_H_RELEASE_TIME_MASK, CLASS_H_RELEASE_TIME_0);
		regmap_update_bits(sma6101->regmap,
			SMA6101_0D_CLASS_H_CTRL_LVL3,
			CLASS_H_RELEASE_TIME_MASK, CLASS_H_RELEASE_TIME_0);
		regmap_update_bits(sma6101->regmap,
			SMA6101_0F_CLASS_H_CTRL_LVL4,
			CLASS_H_RELEASE_TIME_MASK, CLASS_H_RELEASE_TIME_0);
	}

	/* Changed gain to reduce pop noise */
	if (sma6101->voice_music_class_h_mode ==
				SMA6101_CLASS_H_MUSIC_MODE) {
		if (sma6101->rev_num == REV_NUM_REV3) {
			regmap_update_bits(sma6101->regmap,
				SMA6101_13_FDPEC_CTRL1,
				FDPEC_GAIN_MASK, FDPEC_GAIN_6);
		} else if (sma6101->rev_num == REV_NUM_REV4) {
			sma6101_setup_fdpec_gain(codec, FDPEC_GAIN_x2P6);
		}
	}

	regmap_update_bits(sma6101->regmap, SMA6101_10_SYSTEM_CTRL1,
			SPK_MODE_MASK, SPK_OFF);

	if (sma6101->rev_num == REV_NUM_REV0) {
	/* Temporary code(REV0) - Turn off the power during music playback,
	 * there is an issue that the NMOS will die as the overcurrent flows.
	 * To improve this, Lower the boost level before turning off the power.
	 */
		regmap_update_bits(sma6101->regmap, SMA6101_93_BOOST_CTRL0,
				TRM_VBST1_MASK, TRM_VBST1_6V);
		dev_info(codec->dev, "%s : Trimming of boost output voltage 6V\n",
			__func__);

		regmap_write(sma6101->regmap,
				SMA6101_0F_CLASS_H_CTRL_LVL4, 0xF3);
		dev_info(codec->dev, "%s : ATTACK_LVL disabled and BOOST LV4 off\n",
			__func__);
		msleep(20);
	}

	regmap_update_bits(sma6101->regmap, SMA6101_00_SYSTEM_CTRL,
			POWER_MASK, POWER_OFF);

	regmap_update_bits(sma6101->regmap, SMA6101_A9_TONE_FINE_VOL,
			TONE_VOL_MASK, TONE_VOL_OFF);
	regmap_update_bits(sma6101->regmap, SMA6101_A8_TONE_GENERATOR,
			TONE_ON_MASK, TONE_OFF);

	if (sma6101->check_thermal_vbat_enable) {
		if ((sma6101->voice_music_class_h_mode ==
				SMA6101_CLASS_H_MUSIC_MODE)
			&& sma6101->check_thermal_vbat_period > 0) {
			/* only compensation temp for Music Playback*/
			mutex_lock(&sma6101->lock);
			sma6101->threshold_level = 0;

			regmap_read(sma6101->regmap, SMA6101_0A_SPK_VOL,
						&cur_vol);

			if (cur_vol > sma6101->init_vol)
				dev_info(codec->dev, "%s : cur vol[%d]  new vol[%d]\n",
				__func__, cur_vol, sma6101->init_vol);
				regmap_write(sma6101->regmap,
					SMA6101_0A_SPK_VOL, sma6101->init_vol);
			mutex_unlock(&sma6101->lock);
		}
	}

	sma6101->amp_power_status = false;

	return 0;
}

static int sma6101_clk_supply_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
#if defined(CONFIG_MACH_LGE)
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
    struct msm_pinctrl_info *pinctrl_info = &sma6101->pinctrl_info;
    int ret_pinctrl = 0;
#endif

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		dev_info(codec->dev, "%s : PRE_PMU\n", __func__);
#if defined(CONFIG_MACH_LGE)
        if(sma6101->sys_clk_id == SMA6101_EXTERNAL_CLOCK_24_576) {
			ret_pinctrl = sma6101_msm_set_pinctrl(pinctrl_info, STATE_SEC_MI2S_MCLK_ACTIVE);
            if (ret_pinctrl)
            	pr_err("%s: STATE_SEC_MI2S_MCLK_ACTIVE pinctrl set failed with %d\n",__func__, ret_pinctrl);
        } else if(sma6101->sys_clk_id == SMA6101_PLL_CLKIN_MCLK) {
        	ret_pinctrl =  sma6101_clock_control(sma6101->ln_bb_clk3,1);
			if (ret_pinctrl < 0) {
				pr_err("%s(): ln_bb_clk3 enable fail %d\n", __func__,ret_pinctrl);
				return -EINVAL;	
			}
        }
#endif
	break;

	case SND_SOC_DAPM_POST_PMD:
		dev_info(codec->dev, "%s : POST_PMD\n", __func__);
#if defined(CONFIG_MACH_LGE)
        if(sma6101->sys_clk_id == SMA6101_EXTERNAL_CLOCK_24_576) {
			ret_pinctrl = sma6101_msm_set_pinctrl(pinctrl_info, STATE_SEC_MI2S_MCLK_SLEEP);
            if (ret_pinctrl)
            	pr_err("%s: STATE_SEC_MI2S_MCLK_SLEEP pinctrl set failed with %d\n",__func__, ret_pinctrl);
        } else if(sma6101->sys_clk_id == SMA6101_PLL_CLKIN_MCLK) {
        	ret_pinctrl =  sma6101_clock_control(sma6101->ln_bb_clk3,0);
			if (ret_pinctrl < 0) {
				pr_err("%s(): ln_bb_clk3 enable fail %d\n", __func__,ret_pinctrl);
				return -EINVAL;	
			}
        }
#endif
	break;
	}

	return 0;
}

static int sma6101_dac_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		dev_info(codec->dev, "%s : PRE_PMU\n", __func__);

		if (sma6101->force_amp_power_down == false)
			sma6101_startup(codec);
		break;

	case SND_SOC_DAPM_POST_PMU:
		dev_info(codec->dev, "%s : POST_PMU\n", __func__);

		if (sma6101->force_amp_power_down == false)
			regmap_update_bits(sma6101->regmap, SMA6101_AE_TOP_MAN4,
					DIS_IRQ_MASK, NORMAL_OPERATION_IRQ);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		dev_info(codec->dev, "%s : PRE_PMD\n", __func__);

		sma6101_shutdown(codec);

		if (atomic_read(&sma6101->irq_enabled)) {
			disable_irq((unsigned int)sma6101->irq);
			atomic_set(&sma6101->irq_enabled, false);
		}

		regmap_update_bits(sma6101->regmap, SMA6101_AE_TOP_MAN4,
				DIS_IRQ_MASK, HIGH_Z_IRQ);
		break;

	case SND_SOC_DAPM_POST_PMD:
		dev_info(codec->dev, "%s : POST_PMD\n", __func__);

		/* PLL LDO bypass disable */
		if (sma6101->sys_clk_id == SMA6101_PLL_CLKIN_MCLK
			|| sma6101->sys_clk_id == SMA6101_PLL_CLKIN_BCLK)
			regmap_update_bits(sma6101->regmap,
				SMA6101_AC_PLL_MODE_CTRL,
					PLL_LDO_BYP_MASK, PLL_LDO_BYP_DISABLE);
		break;
	}

	return 0;
}

static int sma6101_dac_feedback_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		dev_info(codec->dev, "%s : DAC feedback ON\n", __func__);
		regmap_update_bits(sma6101->regmap,
			SMA6101_09_OUTPUT_CTRL,
				PORT_CONFIG_MASK|PORT_OUT_SEL_MASK,
				OUTPUT_PORT_ENABLE|SPEAKER_PATH);
		regmap_update_bits(sma6101->regmap,
			SMA6101_A2_TOP_MAN1, SDO_DATA_MASK,
				SDO_DATA_DAC);
		/* even if Capture stream on, Mixer should turn on
		 * SDO output(1:High-Z,0:Normal output)
		 */
#if 0
		regmap_update_bits(sma6101->regmap,
			SMA6101_A3_TOP_MAN2, SDO_OUTPUT_MASK,
				NORMAL_OUT);
#endif
		regmap_update_bits(sma6101->regmap,
			SMA6101_AE_TOP_MAN4, SDO_LRCK_MASK,
				SDO_LRCK_LOW_VALID);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		dev_info(codec->dev, "%s : DAC feedback OFF\n", __func__);
		regmap_update_bits(sma6101->regmap,
			SMA6101_09_OUTPUT_CTRL, PORT_OUT_SEL_MASK,
				DISABLE);
		regmap_update_bits(sma6101->regmap,
			SMA6101_A3_TOP_MAN2, SDO_OUTPUT_MASK,
				HIGH_Z_OUT);
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget sma6101_dapm_widgets[] = {
SND_SOC_DAPM_SUPPLY("CLK_SUPPLY", SND_SOC_NOPM, 0, 0, sma6101_clk_supply_event,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_DAC_E("DAC", "Playback", SND_SOC_NOPM, 0, 0, sma6101_dac_event,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_ADC_E("DAC_FEEDBACK", "Capture", SND_SOC_NOPM, 0, 0,
				sma6101_dac_feedback_event,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_OUTPUT("SPK"),
SND_SOC_DAPM_INPUT("SDO"),
};

static const struct snd_soc_dapm_route sma6101_audio_map[] = {
/* sink, control, source */
{"DAC", NULL, "CLK_SUPPLY"},
{"SPK", NULL, "DAC"},
{"DAC_FEEDBACK", NULL, "SDO"},
};

static int sma6101_setup_pll(struct snd_soc_codec *codec,
		struct snd_pcm_hw_params *params)
{
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	int i = 0;
	int calc_to_bclk = params_rate(params) * params_physical_width(params)
					* params_channels(params);

	dev_info(codec->dev, "%s : rate = %d : bit size = %d : channel = %d\n",
		__func__, params_rate(params), params_physical_width(params),
			params_channels(params));

	/* PLL LDO bypass enable */
	regmap_update_bits(sma6101->regmap, SMA6101_AC_PLL_MODE_CTRL,
			PLL_LDO_BYP_MASK, PLL_LDO_BYP_ENABLE);

	if (sma6101->sys_clk_id == SMA6101_PLL_CLKIN_MCLK) {
		/* PLL operation, PLL Clock, External Clock,
		 * PLL reference PLL_REF_CLK1 clock
		 */
		regmap_update_bits(sma6101->regmap, SMA6101_A2_TOP_MAN1,
		PLL_PD_MASK|MCLK_SEL_MASK|PLL_REF_CLK1_MASK|PLL_REF_CLK2_MASK,
		PLL_OPERATION|PLL_CLK|REF_EXTERNAL_CLK|PLL_REF_CLK1);

		for (i = 0; i < sma6101->num_of_pll_matches; i++) {
			if (sma6101->pll_matches[i].input_clk ==
					sma6101->mclk_in)
				break;
		}
	} else if (sma6101->sys_clk_id == SMA6101_PLL_CLKIN_BCLK) {
		/* SCK clock monitoring mode */
		regmap_update_bits(sma6101->regmap, SMA6101_A7_TOP_MAN3,
				CLOCK_MON_SEL_MASK, CLOCK_MON_SCK);

		/* PLL operation, PLL Clock, External Clock,
		 * PLL reference SCK clock
		 */
		regmap_update_bits(sma6101->regmap, SMA6101_A2_TOP_MAN1,
		PLL_PD_MASK|MCLK_SEL_MASK|PLL_REF_CLK1_MASK|PLL_REF_CLK2_MASK,
		PLL_OPERATION|PLL_CLK|REF_EXTERNAL_CLK|PLL_SCK);

		for (i = 0; i < sma6101->num_of_pll_matches; i++) {
			if (sma6101->pll_matches[i].input_clk ==
					calc_to_bclk)
				break;
		}
	}

	regmap_write(sma6101->regmap, SMA6101_8B_PLL_POST_N,
			sma6101->pll_matches[i].post_n);
	regmap_write(sma6101->regmap, SMA6101_8C_PLL_N,
			sma6101->pll_matches[i].n);
	regmap_write(sma6101->regmap, SMA6101_8D_PLL_F1,
			sma6101->pll_matches[i].f1);
	regmap_write(sma6101->regmap, SMA6101_8E_PLL_F2,
			sma6101->pll_matches[i].f2);
	regmap_write(sma6101->regmap, SMA6101_8F_PLL_F3,
			sma6101->pll_matches[i].f3_p_cp);

	return 0;
}

static int sma6101_setup_fdpec_gain(struct snd_soc_codec *codec,
		unsigned int value)
{
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	regmap_update_bits(sma6101->regmap,
		SMA6101_13_FDPEC_CTRL1, FDPEC_GAIN_MASK,
		sma6101->fdpec_gain_matches[value].gain);
	regmap_update_bits(sma6101->regmap,
		SMA6101_13_FDPEC_CTRL1, FDPEC_EXT_GAIN_MASK,
		sma6101->fdpec_gain_matches[value].ext_gain);
	regmap_update_bits(sma6101->regmap,
		SMA6101_13_FDPEC_CTRL1, FDPEC_EXT_GAIN2_MASK,
		sma6101->fdpec_gain_matches[value].ext_gain2);

	sma6101->fdpec_gain_control = value;

	return 0;
}

static int sma6101_dai_hw_params_amp(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	unsigned int input_format = 0;

	dev_info(codec->dev, "%s : rate = %d : bit size = %d\n",
		__func__, params_rate(params), params_width(params));

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		/* The sigma delta modulation setting for
		 * using the fractional divider in the PLL clock
		 * if (params_format(params) == SNDRV_PCM_FORMAT_S24_LE ||
		 *		params_rate(params) == 44100) {
		 *	regmap_update_bits(sma6101->regmap,
		 *	SMA6101_A3_TOP_MAN2, PLL_SDM_PD_MASK, SDM_ON);
		 * } else {
		 *	regmap_update_bits(sma6101->regmap,
		 *	SMA6101_A3_TOP_MAN2, PLL_SDM_PD_MASK, SDM_OFF);
		 * }
		 */
		/* PLL clock setting according to sample rate and bit */
		if (sma6101->force_amp_power_down == false &&
			(sma6101->sys_clk_id == SMA6101_PLL_CLKIN_MCLK
			|| sma6101->sys_clk_id == SMA6101_PLL_CLKIN_BCLK)) {

			regmap_update_bits(sma6101->regmap,
				SMA6101_03_INPUT1_CTRL3,
				BP_SRC_MASK, BP_SRC_NORMAL);

			sma6101_shutdown(codec);
			sma6101_setup_pll(codec, params);
			sma6101_startup(codec);
		}

		if (sma6101->force_amp_power_down == false &&
			!atomic_read(&sma6101->irq_enabled)) {
			enable_irq((unsigned int)sma6101->irq);
			irq_set_irq_wake(sma6101->irq, 1);

			if (device_may_wakeup(sma6101->dev))
				enable_irq_wake(sma6101->irq);

			atomic_set(&sma6101->irq_enabled, true);
		}

		switch (params_rate(params)) {
		case 8000:
		case 12000:
		case 16000:
		case 24000:
		case 32000:
		case 44100:
		case 48000:
		case 96000:
		regmap_update_bits(sma6101->regmap, SMA6101_A2_TOP_MAN1,
				DAC_DN_CONV_MASK, DAC_DN_CONV_DISABLE);
		regmap_update_bits(sma6101->regmap, SMA6101_01_INPUT1_CTRL1,
				LEFTPOL_MASK, LOW_FIRST_CH);
		break;

		case 192000:
		regmap_update_bits(sma6101->regmap, SMA6101_A2_TOP_MAN1,
				DAC_DN_CONV_MASK, DAC_DN_CONV_ENABLE);
		regmap_update_bits(sma6101->regmap, SMA6101_01_INPUT1_CTRL1,
				LEFTPOL_MASK, HIGH_FIRST_CH);
		break;

		default:
			dev_err(codec->dev, "%s not support rate : %d\n",
				__func__, params_rate(params));

		return -EINVAL;
		}
	/* substream->stream is SNDRV_PCM_STREAM_CAPTURE */
	} else {

		switch (params_format(params)) {

		case SNDRV_PCM_FORMAT_S16_LE:
			dev_info(codec->dev,
				"%s set format SNDRV_PCM_FORMAT_S16_LE\n",
				__func__);
			regmap_update_bits(sma6101->regmap,
				SMA6101_A4_SDO_OUT_FMT, WD_LENGTH_MASK,
					WL_16BIT);
			break;

		case SNDRV_PCM_FORMAT_S24_LE:
			dev_info(codec->dev,
				"%s set format SNDRV_PCM_FORMAT_S24_LE\n",
				__func__);
			regmap_update_bits(sma6101->regmap,
				SMA6101_A4_SDO_OUT_FMT, WD_LENGTH_MASK,
					WL_24BIT);
			break;

		default:
			dev_err(codec->dev,
				"%s not support data bit : %d\n", __func__,
						params_format(params));
			return -EINVAL;
		}
	}

	switch (params_width(params)) {
	case 16:
		switch (sma6101->format) {
		case SND_SOC_DAIFMT_I2S:
			input_format |= STANDARD_I2S;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			input_format |= LJ;
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
			input_format |= RJ_16BIT;
			break;
		}
		break;
	case 24:
		switch (sma6101->format) {
		case SND_SOC_DAIFMT_I2S:
			input_format |= STANDARD_I2S;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			input_format |= LJ;
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
			input_format |= RJ_24BIT;
			break;
		}
		break;

	default:
		dev_err(codec->dev,
			"%s not support data bit : %d\n", __func__,
					params_format(params));
		return -EINVAL;
	}

	regmap_update_bits(sma6101->regmap, SMA6101_01_INPUT1_CTRL1,
				I2S_MODE_MASK, input_format);

	return 0;
}

static int sma6101_dai_set_sysclk_amp(struct snd_soc_dai *dai,
			int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "%s\n", __func__);

	/* Requested clock frequency is already setup */
	if (freq == sma6101->mclk_in)
		return 0;

	switch (clk_id) {
	case SMA6101_EXTERNAL_CLOCK_19_2:
		regmap_update_bits(sma6101->regmap, SMA6101_00_SYSTEM_CTRL,
				CLKSYSTEM_MASK, EXT_19_2);
		break;

	case SMA6101_EXTERNAL_CLOCK_24_576:
		regmap_update_bits(sma6101->regmap, SMA6101_00_SYSTEM_CTRL,
				CLKSYSTEM_MASK, EXT_24_576);
		break;
	case SMA6101_PLL_CLKIN_MCLK:
		if (freq < 1536000 || freq > 24576000) {
			/* out of range PLL_CLKIN, fall back to use BCLK */
			dev_warn(codec->dev, "Out of range PLL_CLKIN: %u\n",
				freq);
			clk_id = SMA6101_PLL_CLKIN_BCLK;
			freq = 0;
		}
	case SMA6101_PLL_CLKIN_BCLK:
		break;
	default:
		dev_err(codec->dev, "Invalid clk id: %d\n", clk_id);
		return -EINVAL;
	}
	sma6101->sys_clk_id = clk_id;
	sma6101->mclk_in = freq;
	return 0;
}

static int sma6101_dai_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	if (!(sma6101->amp_power_status)) {
		dev_info(codec->dev, "%s : %s\n",
			__func__, "Already AMP Shutdown");
		return 0;
	}

	if (mute) {

		dev_info(codec->dev, "%s : %s\n", __func__, "MUTE");

		regmap_update_bits(sma6101->regmap, SMA6101_0E_MUTE_VOL_CTRL,
					SPK_MUTE_MASK, SPK_MUTE);

	} else {

		dev_info(codec->dev, "%s : %s\n", __func__, "UNMUTE");

		regmap_update_bits(sma6101->regmap, SMA6101_0E_MUTE_VOL_CTRL,
					SPK_MUTE_MASK, SPK_UNMUTE);
	}

	return 0;
}

static int sma6101_dai_set_fmt_amp(struct snd_soc_dai *codec_dai,
					unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {

	case SND_SOC_DAIFMT_CBS_CFS:
		dev_info(codec->dev, "%s : %s\n", __func__, "I2S slave mode");
		/* I2S/PCM clock mode - slave mode */
		regmap_update_bits(sma6101->regmap, SMA6101_01_INPUT1_CTRL1,
					MASTER_SLAVE_MASK, SLAVE_MODE);
		regmap_update_bits(sma6101->regmap, SMA6101_A7_TOP_MAN3,
					MAS_EN_MASK, MAS_EN_SLAVE);
		break;

	case SND_SOC_DAIFMT_CBM_CFM:
		dev_info(codec->dev, "%s : %s\n", __func__, "I2S master mode");
		/* I2S/PCM clock mode - master mode */
		regmap_update_bits(sma6101->regmap, SMA6101_01_INPUT1_CTRL1,
					MASTER_SLAVE_MASK, MASTER_MODE);
		regmap_update_bits(sma6101->regmap, SMA6101_A7_TOP_MAN3,
					MAS_EN_MASK, MAS_EN_MASTER);
		break;

	default:
		dev_err(codec->dev, "Unsupported MASTER/SLAVE : 0x%x\n", fmt);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {

	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
		sma6101->format = fmt & SND_SOC_DAIFMT_FORMAT_MASK;
		break;

	default:
		dev_err(codec->dev, "Unsupported I2S FORMAT : 0x%x\n", fmt);
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops sma6101_dai_ops_amp = {
	.set_sysclk = sma6101_dai_set_sysclk_amp,
	.set_fmt = sma6101_dai_set_fmt_amp,
	.hw_params = sma6101_dai_hw_params_amp,
	.digital_mute = sma6101_dai_digital_mute,
};

#define SMA6101_RATES SNDRV_PCM_RATE_8000_192000
#define SMA6101_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
			SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver sma6101_dai[] = {
{
	.name = "sma6101-piezo",
	.id = 0,
	.playback = {
	.stream_name = "Playback",
	.channels_min = 1,
	.channels_max = 2,
	.rates = SMA6101_RATES,
	.formats = SMA6101_FORMATS,
	},
	.capture = {
	.stream_name = "Capture",
	.channels_min = 1,
	.channels_max = 2,
	.rates = SMA6101_RATES,
	.formats = SMA6101_FORMATS,
	},
	.ops = &sma6101_dai_ops_amp,
}
};

static int sma6101_set_bias_level(struct snd_soc_codec *codec,
			enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:

		dev_info(codec->dev, "%s\n", "SND_SOC_BIAS_ON");
		sma6101_startup(codec);

		break;

	case SND_SOC_BIAS_PREPARE:

		dev_info(codec->dev, "%s\n", "SND_SOC_BIAS_PREPARE");

		break;

	case SND_SOC_BIAS_STANDBY:

		dev_info(codec->dev, "%s\n", "SND_SOC_BIAS_STANDBY");

		break;

	case SND_SOC_BIAS_OFF:

		dev_info(codec->dev, "%s\n", "SND_SOC_BIAS_OFF");
		sma6101_shutdown(codec);

		break;
	}

	/* Don't use codec->dapm.bias_level,
	 * use snd_soc_component_get_dapm() if it is needed
	 */

	return 0;
}

static irqreturn_t sma6101_isr(int irq, void *data)
{
	struct sma6101_priv *sma6101 = (struct sma6101_priv *) data;
	int ret;
	unsigned int over_temp, ocp_val, sar_adc, bop_state;

	ret = regmap_read(sma6101->regmap, SMA6101_FA_STATUS1, &over_temp);
	if (ret != 0) {
		dev_err(sma6101->dev,
			"failed to read SMA6101_FA_STATUS1 : %d\n", ret);
	}

	ret = regmap_read(sma6101->regmap, SMA6101_FB_STATUS2, &ocp_val);
	if (ret != 0) {
		dev_err(sma6101->dev,
			"failed to read SMA6101_FB_STATUS2 : %d\n", ret);
	}

	ret = regmap_read(sma6101->regmap, SMA6101_FC_STATUS3, &sar_adc);
	if (ret != 0) {
		dev_err(sma6101->dev,
			"failed to read SMA6101_FC_STATUS3 : %d\n", ret);
	}

	ret = regmap_read(sma6101->regmap, SMA6101_FE_STATUS5, &bop_state);
	if (ret != 0) {
		dev_err(sma6101->dev,
			"failed to read SMA6101_FE_STATUS5 : %d\n", ret);
		return IRQ_HANDLED;
	}

	dev_crit(sma6101->dev,
			"%s : SAR_ADC : %x\n", __func__, sar_adc);
	dev_crit(sma6101->dev,
			"%s : BOP_STATE : %d\n", __func__, bop_state);

	if (~over_temp & OT2_OK_STATUS)
		dev_crit(sma6101->dev,
			"%s : OT2(Over Temperature Level 2)\n", __func__);
	if (ocp_val & OCP_SPK_STATUS) {
		dev_crit(sma6101->dev,
			"%s : OCP_SPK(Over Current Protect SPK)\n", __func__);
		if (sma6101->enable_ocp_aging) {
			mutex_lock(&sma6101->lock);
			sma6101_thermal_compensation(sma6101, true);
			mutex_unlock(&sma6101->lock);
		}
		sma6101->ocp_count++;
#if defined(CONFIG_MACH_LGE)
		if (sma6101->rev_num == REV_NUM_REV4)
			panic("AUDIO_BSP] OCP_SPK : Please contact to ALPHA-BSP-AUDIO@lge.com");
#endif
	}
	if (ocp_val & OCP_BST_STATUS) {
		dev_crit(sma6101->dev,
			"%s : OCP_BST(Over Current Protect Boost)\n", __func__);
		sma6101->ocp_count++;
#if defined(CONFIG_MACH_LGE)
		if (sma6101->rev_num == REV_NUM_REV4)
			panic("AUDIO_BSP] OCP_BST : Please contact to ALPHA-BSP-AUDIO@lge.com");
#endif
	}
	if (ocp_val & UVLO_BST_STATUS)
		dev_crit(sma6101->dev,
			"%s : UVLO(Under Voltage Lock Out)\n", __func__);
	if (ocp_val & CLOCK_MON_STATUS)
		dev_crit(sma6101->dev,
			"%s : CLK_FAULT(No clock input)\n", __func__);
	if ((ocp_val & OCP_SPK_STATUS) || (ocp_val & OCP_BST_STATUS))
		dev_crit(sma6101->dev, "%s : OCP has occurred < %d > times\n",
				__func__, sma6101->ocp_count);

	return IRQ_HANDLED;
}

static void sma6101_check_thermal_fault_worker(struct work_struct *work)
{
	struct sma6101_priv *sma6101 =
		container_of(work, struct sma6101_priv,
				check_thermal_fault_work.work);
	int ret;
	unsigned int over_temp, sar_adc, bop_state;
	unsigned int bop_threshold = 143;

	ret = regmap_read(sma6101->regmap, SMA6101_FA_STATUS1, &over_temp);
	if (ret != 0) {
		dev_err(sma6101->dev,
			"failed to read SMA6101_FA_STATUS1 : %d\n", ret);
		return;
	}
	ret = regmap_read(sma6101->regmap, SMA6101_FC_STATUS3, &sar_adc);
	if (ret != 0) {
		dev_err(sma6101->dev,
			"failed to read SMA6101_FC_STATUS3 : %d\n", ret);
	}

	ret = regmap_read(sma6101->regmap, SMA6101_FE_STATUS5, &bop_state);
	if (ret != 0) {
		dev_err(sma6101->dev,
			"failed to read SMA6101_FE_STATUS5 : %d\n", ret);
	}

	if (bop_state != 0 || sar_adc <= bop_threshold) {
		/* Expected brown out operation */
		dev_info(sma6101->dev,
			"%s : SAR_ADC : %x, BOP_STATE : %d\n",
				__func__, sar_adc, bop_state);
	}

	if (~over_temp & OT1_OK_STATUS) {
		dev_info(sma6101->dev,
			"%s : OT1(Over Temperature Level 1)\n", __func__);
	}

	if (sma6101->check_thermal_fault_enable) {
		if (sma6101->check_thermal_fault_period > 0)
			queue_delayed_work(system_freezable_wq,
				&sma6101->check_thermal_fault_work,
				sma6101->check_thermal_fault_period * HZ);
		else
			queue_delayed_work(system_freezable_wq,
				&sma6101->check_thermal_fault_work,
					CHECK_FAULT_PERIOD_TIME * HZ);
	}
}

static void sma6101_check_thermal_vbat_worker(struct work_struct *work)
{
	struct sma6101_priv *sma6101 =
		container_of(work, struct sma6101_priv,
			check_thermal_vbat_work.work);
#ifdef CONFIG_SMA6101_BATTERY_READING
	union power_supply_propval prop = {0, };
	int ret = 0;
#endif
	struct outside_status fifo_buf_in = {0, };

	mutex_lock(&sma6101->lock);

	if (sma6101->thermal_sense_opt == -1) {

/*base*/
#ifndef CONFIG_MACH_LGE
		sma6101->tz_sense =
			thermal_zone_get_zone_by_name("quiet_therm");
#else
		/* Function for checking thermal
		 * quiet_therm : skin-therm,
		 * piezo_therm : wp-therm, vts : vts-virt-therm
		 */
		sma6101->tz_sense =
			thermal_zone_get_zone_by_name("skin-therm");
#endif
	} else {

#ifndef CONFIG_MACH_LGE
		sma6101->tz_sense =
			thermal_zone_get_zone_by_name("quiet_therm");
#else
		/* Function for checking thermal
		 * quiet_therm : skin-therm,
		 * piezo_therm : wp-therm, vts : vts-virt-therm
		 */
		switch (sma6101->thermal_sense_opt) {
		case 1:
			sma6101->tz_sense =
				thermal_zone_get_zone_by_name("skin-therm");
			break;
		case 2:
			sma6101->tz_sense =
				thermal_zone_get_zone_by_name("wp-therm");
			break;
		default:
			sma6101->tz_sense =
				thermal_zone_get_zone_by_name("skin-therm");
			break;
		}
#endif

	}

	if (IS_ERR(sma6101->tz_sense))
		dev_info(sma6101->dev, "%s : need to check thermal zone name:%p\n",
			__func__, sma6101->tz_sense);
	else
		thermal_zone_get_temp(sma6101->tz_sense,
			&fifo_buf_in.thermal_deg);


#ifdef CONFIG_MACH_LGE
	/* Converting xxxxx mC to xx.x C */
	fifo_buf_in.thermal_deg = fifo_buf_in.thermal_deg/100;
#else
	fifo_buf_in.thermal_deg = fifo_buf_in.thermal_deg*10;
#endif

/* Currently not checked Battery level */
#ifdef CONFIG_SMA6101_BATTERY_READING
	sma6101->batt_psy = power_supply_get_by_name("battery");

	if (!sma6101->batt_psy) {
		pr_err("failed get batt_psy\n");
		goto exit_vbat_worker;
	}
	ret = power_supply_get_property(sma6101->batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	if (ret < 0) {
		pr_err("Error in getting battery voltage, ret=%d\n", ret);
		fifo_buf_in.batt_voltage_mV = 4450;
	} else
		fifo_buf_in.batt_voltage_mV = prop.intval;
#endif

	fifo_buf_in.id = sma6101->fifo_count;

	if (!kfifo_is_full(&sma6101->data_fifo)) {
		kfifo_in(&sma6101->data_fifo,
			(unsigned char *)&fifo_buf_in,
			sizeof(fifo_buf_in));

		sma6101->fifo_count++;
		pr_debug("%s :queue in", __func__);
	}


#ifdef CONFIG_SMA6101_BATTERY_READING
	dev_dbg(sma6101->dev,
	"%s : id - [%d]  sense_temp - [%3d] deg bat_vol - [%d] mV\n",
	__func__, fifo_buf_in.id,
	fifo_buf_in.thermal_deg,
	fifo_buf_in.batt_voltage_mV/1000);
#else
	dev_dbg(sma6101->dev,
	"%s : id - [%d]  sense_temp - [%3d]\n",
	__func__, fifo_buf_in.id,
	fifo_buf_in.thermal_deg);
#endif


	sma6101_thermal_compensation(sma6101, false);

#ifdef CONFIG_SMA6101_BATTERY_READING
exit_vbat_worker:
#endif

	if (sma6101->check_thermal_vbat_enable) {
		if (sma6101->check_thermal_vbat_period > 0)
			queue_delayed_work(system_freezable_wq,
				&sma6101->check_thermal_vbat_work,
				sma6101->check_thermal_vbat_period * HZ);
		else
			queue_delayed_work(system_freezable_wq,
				&sma6101->check_thermal_vbat_work,
					CHECK_COMP_PERIOD_TIME * HZ);
	}
	mutex_unlock(&sma6101->lock);
}

static int sma6101_thermal_compensation(struct sma6101_priv *sma6101,
		bool ocp_status)
{
	unsigned int cur_vol;
	int ret, i = 0;
	struct outside_status fifo_buf_out = {0, };
	int vbat_gain = 0, vbat_status = -1;

	/* SPK OCP issued or monitoring function */
	if (ocp_status) {
		i = sma6101->threshold_level;
		sma6101->temp_match[i].ocp_count++;

		if (i == 0) {
			dev_info(sma6101->dev, "%s : OCP occured in normal temp\n",
				__func__);
		} else {
			if (sma6101->enable_ocp_aging) {
				/* Volume control (0dB/0x30) */
				regmap_read(sma6101->regmap, SMA6101_0A_SPK_VOL,
					&cur_vol);

				sma6101->temp_match[i].comp_gain++;
				cur_vol = sma6101->init_vol +
					sma6101->temp_match[i].comp_gain;
				regmap_write(sma6101->regmap,
					SMA6101_0A_SPK_VOL, cur_vol);
			}
		}
		/* Need to update compensation gain */
		dev_info(sma6101->dev,
			"%s :OCP occured in TEMP[%d] GAIN_C[%d] OCP_N[%d] HIT_N[%d] ACT[%d]\n",
			__func__, sma6101->temp_match[i].thermal_limit,
			sma6101->temp_match[i].comp_gain,
			sma6101->temp_match[i].ocp_count,
			sma6101->temp_match[i].hit_count,
			sma6101->temp_match[i].activate);

		return 0;
	}

	if (!kfifo_is_empty(&sma6101->data_fifo)) {
		ret = kfifo_out(&sma6101->data_fifo,
			(unsigned char *)&fifo_buf_out,
			sizeof(fifo_buf_out));

		dev_dbg(sma6101->dev, "%s :queue out\n", __func__);

		if (ret != sizeof(fifo_buf_out))
			return ret;
#ifdef CONFIG_SMA6101_BATTERY_READING
		dev_dbg(sma6101->dev,
		"%s : id - [%d]  sense_temp - [%3d]  deg bat_vol - %d mV\n",
		__func__, fifo_buf_out.id,
		fifo_buf_out.thermal_deg,
		fifo_buf_out.batt_voltage_mV/1000);
#else
		dev_dbg(sma6101->dev,
		"%s : id - [%d]  sense_temp - [%3d]  deg\n",
		__func__, fifo_buf_out.id,
		fifo_buf_out.thermal_deg);
#endif
	}

	for (i = 0; i < sma6101->num_of_temperature_matches; i++) {
		/* Check  matching temperature
		 * compare current temp & table
		 */
		if ((fifo_buf_out.thermal_deg <
			 sma6101->temp_match[i].thermal_limit)) {
			dev_dbg(sma6101->dev,
				"%s :Matched TEMP[%d] GAIN_C[%d] OCP_N[%d] HIT_N[%d] ACT[%d]\n",
				__func__,
				sma6101->temp_match[i].thermal_limit,
				sma6101->temp_match[i].comp_gain,
				sma6101->temp_match[i].ocp_count,
				sma6101->temp_match[i].hit_count,
				sma6101->temp_match[i].activate);
			break;
		}
	}

	if (vbat_status != -1 &&
		vbat_status < VBAT_TABLE_NUM) {
		vbat_gain =
			sma6101_vbat_gain_matches[vbat_status].comp_gain;
	}

	/* Updating the gain for battery level and temperature */
	if (i == 0 || (sma6101->temp_match[i].activate == 0)) {
		/* Matched normal temeperature in table */
		dev_dbg(sma6101->dev, "%s :temp[%d] matched in normal temperature\n",
		__func__, i);

		if (vbat_gain > 0) {
			/* Prefered battery level in normal temperature */
			cur_vol = sma6101->init_vol + vbat_gain;
			regmap_write(sma6101->regmap, SMA6101_0A_SPK_VOL,
				cur_vol);
			dev_info(sma6101->dev, "%s : low battery gain[%d] in normal temp\n",
			__func__, cur_vol);
		} else if (sma6101->threshold_level != i) {
			/* Normal gain */
			regmap_write(sma6101->regmap, SMA6101_0A_SPK_VOL,
				sma6101->init_vol);
		}
	} else if (i < sma6101->num_of_temperature_matches) {

		/* Matched temeperature in table */
		dev_dbg(sma6101->dev, "%s :temp[%d] matched", __func__, i);
		sma6101->temp_match[i].hit_count++;

		if (sma6101->threshold_level != i) {
			/* First step, only tracking temperature
			 * need to optimise for temp rising and falling slope
			 */
			if (vbat_gain > sma6101->temp_match[i].comp_gain) {
				/* Case Battery gain comp */
				cur_vol = sma6101->init_vol + vbat_gain;
				regmap_write(sma6101->regmap,
					SMA6101_0A_SPK_VOL, cur_vol);
			} else {
				/* Temp comp */
				cur_vol = sma6101->init_vol +
					sma6101->temp_match[i].comp_gain;
				regmap_write(sma6101->regmap,
					SMA6101_0A_SPK_VOL, cur_vol);
			}
			dev_info(sma6101->dev, "%s : cur temp[%d]  previous temp[%d] gain[%d]\n",
			__func__, i, sma6101->threshold_level, cur_vol);

		} else if (vbat_gain > sma6101->temp_match[i].comp_gain) {
			/* Temperature is not changed
			 * Only battery gain comp
			 */
			dev_info(sma6101->dev,
				"%s : cur temp[%d] - only vbat gain[%d] comp\n",
				__func__, i, vbat_gain);
			cur_vol = sma6101->init_vol + vbat_gain;
			regmap_write(sma6101->regmap,
				SMA6101_0A_SPK_VOL, cur_vol);
		}
	}
	/* Updating previous temperature */
	sma6101->threshold_level = i;

	return 0;
}

#ifdef CONFIG_PM
static int sma6101_suspend(struct snd_soc_codec *codec)
{

	dev_info(codec->dev, "%s\n", __func__);

	return 0;
}

static int sma6101_resume(struct snd_soc_codec *codec)
{

	dev_info(codec->dev, "%s\n", __func__);

	return 0;
}
#else
#define sma6101_suspend NULL
#define sma6101_resume NULL
#endif

static int sma6101_reset(struct snd_soc_codec *codec)
{
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	struct reg_default *reg_val;
	int cnt, ret;
	unsigned int status;
	int eq_len = sma6101->eq_reg_array_len / sizeof(uint32_t);
	int bo_len = sma6101->bo_reg_array_len / sizeof(uint32_t);

	dev_info(codec->dev, "%s\n", __func__);

	ret = regmap_read(sma6101->regmap, SMA6101_FA_STATUS1, &status);

	if (ret != 0)
		dev_err(sma6101->dev,
			"failed to read SMA6101_FA_STATUS1 : %d\n", ret);
	else
		sma6101->rev_num = status & REV_NUM_STATUS;

	dev_info(codec->dev, "SMA6101 chip revision ID - %d\n",
			((sma6101->rev_num) >> 3));

	/* Trimming of VBG reference - 1.1125V,
	 * Trimming of boost output voltage - 6.0V
	 */
	regmap_write(sma6101->regmap, SMA6101_93_BOOST_CTRL0, 0xF0);
	/* Attack level - BOOST LV4 off,
	 * Release time - 48kHz-60ms,96kHz-30ms
	 */
	regmap_write(sma6101->regmap, SMA6101_0F_CLASS_H_CTRL_LVL4, 0xF3);
	/* External clock 24.576MHz */
	regmap_write(sma6101->regmap, SMA6101_00_SYSTEM_CTRL, 0x80);
	/* Volume control (0dB/0x30) */
	regmap_write(sma6101->regmap, SMA6101_0A_SPK_VOL, sma6101->init_vol);
	/* Attack level - BOOST LV3 off, Release time - 48kHz-60ms,96kHz-30ms */
	regmap_write(sma6101->regmap, SMA6101_0D_CLASS_H_CTRL_LVL3, 0xF3);
	/* VOL_SLOPE - Fast Volume Slope,
	 * MUTE_SLOPE - Fast Mute Slope, SPK_MUTE - muted
	 */
	regmap_write(sma6101->regmap, SMA6101_0E_MUTE_VOL_CTRL,	0xFF);

	if (sma6101->stereo_two_chip == true) {
		/* MONO MIX Off */
		regmap_update_bits(sma6101->regmap,
		SMA6101_11_SYSTEM_CTRL2, MONOMIX_MASK, MONOMIX_OFF);
	} else {
		/* MONO MIX ON */
		regmap_update_bits(sma6101->regmap,
		SMA6101_11_SYSTEM_CTRL2, MONOMIX_MASK, MONOMIX_ON);
	}

	/* Delay control between OUTA and OUTB with main clock duty cycle */
	regmap_write(sma6101->regmap, SMA6101_14_MODULATOR, 0x11);
	/* PWM Slope control, PWM Dead time control */
	regmap_write(sma6101->regmap, SMA6101_37_SLOPE_CTRL, 0x10);

	if (sma6101->rev_num == REV_NUM_REV0) {
		/* Attack level - BOOST LV1 on,
		 * Release time - 48kHz-40ms,96kHz-20ms
		 */
		regmap_write(sma6101->regmap,
				SMA6101_90_CLASS_H_CTRL_LVL1, 0x02);
	} else {
		/* Attack level - BOOST LV1 off,
		 * Release time - 48kHz-40ms,96kHz-20ms
		 */
		regmap_write(sma6101->regmap,
				SMA6101_90_CLASS_H_CTRL_LVL1, 0xF2);
	}

	/* Attack level - BOOST LV2 off, Release time - 48kHz-60ms,96kHz-30ms */
	regmap_write(sma6101->regmap, SMA6101_91_CLASS_H_CTRL_LVL2, 0xF3);
	/* Feedback gain - 1/3, PWMLS OPAMP current - 160uA */
	regmap_write(sma6101->regmap, SMA6101_92_FDPEC_CTRL2, 0x03);
	/* Trimming of ramp compensation I-gain - 50pF,
	 * Trimming of switching frequency - 3.34MHz
	 * Trimming of ramp compensation - 7.37A / us
	 */
	regmap_write(sma6101->regmap, SMA6101_94_BOOST_CTRL1, 0x9B);
	/* Trimming of over current limit - 2.1A,
	 * Trimming of ramp compensation - P-gain:3.5Mohm,Type II I-gain:2.0pF
	 */
	regmap_write(sma6101->regmap, SMA6101_95_BOOST_CTRL2, 0x25);
	/* Trimming of driver deadtime - 10.4ns,
	 * Trimming of boost OCP - pMOS OCP enable, nMOS OCP enable,
	 * Trimming of switching slew - 3ns
	 */
	regmap_write(sma6101->regmap, SMA6101_96_BOOST_CTRL3, 0x3E);
	/* Trimming of boost level reference - 0.75,0.50,0.28,
	 * Trimming of minimum on-time - 59ns
	 */
	regmap_write(sma6101->regmap, SMA6101_97_BOOST_CTRL4, 0xE0);
	/* PLL Lock disable, External clock  operation */
	regmap_write(sma6101->regmap, SMA6101_A2_TOP_MAN1, 0xE9);
	/* External clock monitoring mode */
	regmap_write(sma6101->regmap, SMA6101_A7_TOP_MAN3, 0x20);
	/* High-Z for IRQ pin (IRQ skip mode) */
	regmap_write(sma6101->regmap, SMA6101_AE_TOP_MAN4, 0x42);
	/* VIN sensing normal operation, VIN cut off freq - 34kHz,
	 * SAR clock freq - 1.536MHz
	 */
	regmap_write(sma6101->regmap, SMA6101_AF_VIN_SENSING, 0x00);

	if (sma6101->rev_num < REV_NUM_REV3) {
		/* Bass/Boost/CompLimiter Off & EQ Enable
		 * MONO_MIX Off(TW) for SPK Signal Path
		 */
		regmap_write(sma6101->regmap, SMA6101_11_SYSTEM_CTRL2, 0x80);
		/* Enable test registers */
		regmap_write(sma6101->regmap, SMA6101_3B_TEST1, 0x5A);
		/* Temporary code(REV0,REV1,REV2) - Problem with closed loop,
		 * so use open loop
		 */
		regmap_update_bits(sma6101->regmap, SMA6101_3C_TEST2,
				SPK_HSDM_BP_MASK, SPK_HSDM_BYPASS);
		/* Stereo idle noise improvement : Improved idle noise by
		 * asynchronizing the PWM waveform of Left and Right
		 */
		regmap_update_bits(sma6101->regmap, SMA6101_3C_TEST2,
		DIS_SDM_SYNC_TEST_MASK, DIS_SDM_SYNC_TEST_DISABLE);
	} else if  (sma6101->rev_num == REV_NUM_REV3) {
		/* Bass Off & EQ/CompLimiter Enable
		 * MONO_MIX Off(TW) for SPK Signal Path
		 */
		regmap_write(sma6101->regmap, SMA6101_11_SYSTEM_CTRL2, 0xA0);
		/* Stereo idle noise improvement, FDPEC Gain - 6,
		 * HDC OPAMP Current - 160uA
		 */
		regmap_write(sma6101->regmap, SMA6101_13_FDPEC_CTRL1, 0x3C);
		/* Delay control between OUTA and
		 * OUTB with main clock duty cycle
		 */
		regmap_write(sma6101->regmap, SMA6101_14_MODULATOR, 0x59);

		regmap_write(sma6101->regmap, SMA6101_23_COMP_LIM1, 0x1F);
		regmap_write(sma6101->regmap, SMA6101_26_COMP_LIM4, 0xFF);
		/* PWM Slope control, PWM Dead time control */
		regmap_write(sma6101->regmap, SMA6101_37_SLOPE_CTRL, 0x05);
		/* Enable test registers */
		regmap_write(sma6101->regmap, SMA6101_3B_TEST1, 0x5A);
		/* Changed the TSD Level to compensate for the issue of
		 * lowering the TSD level while increasing the VREF
		 */
		regmap_update_bits(sma6101->regmap, SMA6101_3F_ATEST2,
				THERMAL_ADJUST_MASK, THERMAL_160_120);

		/* Class-H Initial Setting */
		regmap_write(sma6101->regmap,
			SMA6101_0D_CLASS_H_CTRL_LVL3, 0xC5);
		regmap_write(sma6101->regmap,
			SMA6101_0F_CLASS_H_CTRL_LVL4, 0x94);
		regmap_write(sma6101->regmap,
			SMA6101_90_CLASS_H_CTRL_LVL1, 0xE7);
		regmap_write(sma6101->regmap,
			SMA6101_91_CLASS_H_CTRL_LVL2, 0x76);

		/* Feedback gain trimming - 7% increase,
		 * Disable BST OVP function,
		 * OVP mode set - OVP voltage = PVDD set voltage * 1.18,
		 * PWM frequency - 740kHz,
		 * Differential OPAMP bias current - 40uA
		 */
		regmap_write(sma6101->regmap, SMA6101_92_FDPEC_CTRL2, 0x89);
		/* Trimming of over current limit - 3.1A,
		 * Trimming of ramp compensation - P-gain:3.5Mohm,
		 * Type II I-gain:0.7pF
		 */
		regmap_write(sma6101->regmap, SMA6101_95_BOOST_CTRL2, 0x44);
		/* Apply -2.375dB fine volume to prevent SPK OCP */
		regmap_write(sma6101->regmap, SMA6101_A9_TONE_FINE_VOL, 0xAF);
		/* Speaker OCP level - 3.7A */
		regmap_write(sma6101->regmap, SMA6101_AD_SPK_OCP_LVL, 0x0A);
		/* VIN sensing Power down, VIN cut off freq - 34kHz,
		 * SAR clock freq - 3.072MHz
		 */
		regmap_write(sma6101->regmap, SMA6101_AF_VIN_SENSING, 0x01);
	} else if (sma6101->rev_num == REV_NUM_REV4) {
		/* Bass Off & EQ/CompLimiter Enable
		 * MONO_MIX Off(TW) for SPK Signal Path
		 */
		regmap_write(sma6101->regmap, SMA6101_11_SYSTEM_CTRL2, 0xA0);
		/* Stereo idle noise improvement, FDPEC Gain - 6,
		 * HDC OPAMP Current - 160uA
		 */
		regmap_write(sma6101->regmap, SMA6101_13_FDPEC_CTRL1, 0x3C);
		/* Delay control between OUTA and
		 * OUTB with main clock duty cycle
		 */
		regmap_write(sma6101->regmap, SMA6101_14_MODULATOR, 0x61);

		regmap_write(sma6101->regmap, SMA6101_23_COMP_LIM1, 0x1F);
		regmap_write(sma6101->regmap, SMA6101_24_COMP_LIM2, 0x02);
		regmap_write(sma6101->regmap, SMA6101_25_COMP_LIM3, 0x09);
		regmap_write(sma6101->regmap, SMA6101_26_COMP_LIM4, 0xFF);
		/* PWM Slope control, PWM Dead time control */
		regmap_write(sma6101->regmap, SMA6101_37_SLOPE_CTRL, 0x05);

		/* Class-H Initial Setting */
		regmap_write(sma6101->regmap,
			SMA6101_0D_CLASS_H_CTRL_LVL3, 0xB4);
		regmap_write(sma6101->regmap,
			SMA6101_0F_CLASS_H_CTRL_LVL4, 0x83);
		regmap_write(sma6101->regmap,
			SMA6101_90_CLASS_H_CTRL_LVL1, 0xE6);
		regmap_write(sma6101->regmap,
			SMA6101_91_CLASS_H_CTRL_LVL2, 0x75);

		/* Feedback gain trimming - No trimming,
		 * Recovery Current Control Mode - Enhanced mode,
		 * Recovery Current Control Enable,
		 * PWM frequency - 740kHz,
		 * Differential OPAMP bias current - 40uA
		 */
		regmap_write(sma6101->regmap, SMA6101_92_FDPEC_CTRL2, 0x09);
		/* Trimming of VBG Reference - 1.2V */
		regmap_update_bits(sma6101->regmap, SMA6101_93_BOOST_CTRL0,
					TRM_VREF_MASK, TRM_VREF_8_IDX);
		/* Trimming of over current limit - 3.1A,
		 * Trimming of ramp compensation - P-gain:3.5Mohm,
		 * Type II I-gain:0.7pF
		 */
		regmap_write(sma6101->regmap, SMA6101_95_BOOST_CTRL2, 0x44);
		/* Trimming of boost level reference - 0.75,0.50,0.30,
		 * Trimming of minimum on-time - 59ns
		 */
		regmap_write(sma6101->regmap, SMA6101_97_BOOST_CTRL4, 0xE4);
		/* Turn off the tone generator by default */
		regmap_update_bits(sma6101->regmap, SMA6101_A9_TONE_FINE_VOL,
					TONE_VOL_MASK, TONE_VOL_OFF);
		regmap_update_bits(sma6101->regmap, SMA6101_A8_TONE_GENERATOR,
					TONE_ON_MASK, TONE_OFF);
		/* Apply -1.125dB fine volume to prevent SPK OCP */
		regmap_write(sma6101->regmap, SMA6101_A9_TONE_FINE_VOL, 0x8f);
		/* Speaker OCP level - 3.7A */
		regmap_write(sma6101->regmap, SMA6101_AD_SPK_OCP_LVL, 0x06);
		/* VIN sensing Power down, VIN cut off freq - 34kHz,
		 * SAR clock freq - 3.072MHz
		 */
		regmap_write(sma6101->regmap, SMA6101_AF_VIN_SENSING, 0x01);
	}

	if (sma6101->src_bypass == true) {
		dev_info(codec->dev, "If do not use SRC, mono mix does not work property\n");
		regmap_update_bits(sma6101->regmap, SMA6101_03_INPUT1_CTRL3,
			BP_SRC_MASK, BP_SRC_BYPASS);
		/* Fine volume adj(-0.125dB) for SRC block enabling */
		if (sma6101->rev_num < REV_NUM_REV3)
			regmap_write(sma6101->regmap,
				SMA6101_A9_TONE_FINE_VOL, 0x6F);
	} else {
		regmap_update_bits(sma6101->regmap, SMA6101_03_INPUT1_CTRL3,
			BP_SRC_MASK, BP_SRC_NORMAL);
	}

	if (sma6101->sys_clk_id == SMA6101_EXTERNAL_CLOCK_19_2
		|| sma6101->sys_clk_id == SMA6101_PLL_CLKIN_MCLK) {
		regmap_update_bits(sma6101->regmap, SMA6101_00_SYSTEM_CTRL,
			CLKSYSTEM_MASK, EXT_19_2);

		regmap_update_bits(sma6101->regmap, SMA6101_03_INPUT1_CTRL3,
			BP_SRC_MASK, BP_SRC_NORMAL);
	}

	dev_info(codec->dev,
		"%s init_vol is 0x%x\n", __func__, sma6101->init_vol);
	/* EQ register value writing
	 * if register value is available from DT
	 */
	if (sma6101->eq_reg_array != NULL) {
		for (cnt = 0; cnt < eq_len; cnt += 2) {
			reg_val = (struct reg_default *)
				&sma6101->eq_reg_array[cnt];
			dev_dbg(codec->dev, "%s reg_write [0x%02x, 0x%02x]",
					__func__, be32_to_cpu(reg_val->reg),
						be32_to_cpu(reg_val->def));
			regmap_write(sma6101->regmap, be32_to_cpu(reg_val->reg),
					be32_to_cpu(reg_val->def));
		}
	}
	/* BrownOut Protection register value writing
	 * if register value is available from DT
	 */
	if (sma6101->bo_reg_array != NULL) {
		for (cnt = 0; cnt < bo_len; cnt += 2) {
			reg_val = (struct reg_default *)
				&sma6101->bo_reg_array[cnt];
			dev_dbg(codec->dev, "%s reg_write [0x%02x, 0x%02x]",
					__func__, be32_to_cpu(reg_val->reg),
						be32_to_cpu(reg_val->def));
			regmap_write(sma6101->regmap, be32_to_cpu(reg_val->reg),
					be32_to_cpu(reg_val->def));
		}
	}

	/* Ready to start amp, if need, add amp on/off mix */
	sma6101->voice_music_class_h_mode = SMA6101_CLASS_H_MODE_OFF;
	sma6101->ocp_count = 0;

	return 0;
}

static ssize_t sma6101_check_thermal_vbat_period_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE,
			"%ld\n", sma6101->check_thermal_vbat_period);

	return (ssize_t)rc;
}

static ssize_t sma6101_check_thermal_vbat_period_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma6101->check_thermal_vbat_period);

	if (ret)
		return -EINVAL;

	return (ssize_t)count;
}

static DEVICE_ATTR(check_thermal_vbat_period, 0644,
	sma6101_check_thermal_vbat_period_show,
	sma6101_check_thermal_vbat_period_store);

static ssize_t sma6101_check_thermal_vbat_enable_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE,
			"%ld\n", sma6101->check_thermal_vbat_enable);

	return (ssize_t)rc;
}

static ssize_t sma6101_check_thermal_vbat_enable_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma6101->check_thermal_vbat_enable);

	if (ret)
		return -EINVAL;

	return (ssize_t)count;
}

static DEVICE_ATTR(check_thermal_vbat_enable, 0644,
	sma6101_check_thermal_vbat_enable_show,
	sma6101_check_thermal_vbat_enable_store);

static ssize_t sma6101_check_thermal_table_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int i, rc = 0;

	rc += (int)snprintf(buf + rc, PAGE_SIZE,
			"Piezo Thermal Table Summary\n");

	for (i = 0; i < sma6101->num_of_temperature_matches; i++) {
		rc += (int)snprintf(buf+rc,
		PAGE_SIZE, "TEMP[%d] GAIN_C[%d] OCP_N[%d] HIT_N[%d] ACT[%d]\n",
			sma6101->temp_match[i].thermal_limit,
			sma6101->temp_match[i].comp_gain,
			sma6101->temp_match[i].ocp_count,
			sma6101->temp_match[i].hit_count,
			sma6101->temp_match[i].activate);
	}

	return (ssize_t)rc;
}

static DEVICE_ATTR(check_thermal_table, 0644,
	sma6101_check_thermal_table_show, NULL);

static ssize_t sma6101_check_thermal_value_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int ret;
	int i, rc = 0;
	struct outside_status fifo_buf_out = {0, };

	for (i = 0 ; i < FIFO_BUFFER_SIZE ; i++) {
		ret = kfifo_out_peek(&sma6101->data_fifo,
			(unsigned char *)&fifo_buf_out,
			sizeof(fifo_buf_out));

		if (ret != sizeof(fifo_buf_out))
			return -EINVAL;

		rc += (int)snprintf(buf + rc, PAGE_SIZE,
				"%d\n", fifo_buf_out.thermal_deg);
	}

	return (ssize_t)rc;
}

static DEVICE_ATTR(check_thermal_value, 0644,
	sma6101_check_thermal_value_show, NULL);

static ssize_t sma6101_temp_table_number_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE,
			"%ld\n", sma6101->temp_table_number);

	return (ssize_t)rc;
}

static ssize_t sma6101_temp_table_number_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma6101->temp_table_number);

	if (ret || (sma6101->temp_table_number < 0) ||
			(sma6101->temp_table_number >
				ARRAY_SIZE(sma6101_temperature_gain_matches))) {
		sma6101->temp_table_number = 0;
		return -EINVAL;
	}

	return (ssize_t)count;
}

static DEVICE_ATTR(temp_table_number, 0644,
	sma6101_temp_table_number_show, sma6101_temp_table_number_store);

static ssize_t sma6101_temp_limit_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE, "%d\n",
	sma6101->temp_match[sma6101->temp_table_number].thermal_limit);

	return (ssize_t)rc;
}

static ssize_t sma6101_temp_limit_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma6101->temp_limit);

	sma6101->temp_match[sma6101->temp_table_number].thermal_limit =
		(int)sma6101->temp_limit;

	if (ret)
		return -EINVAL;

	return (ssize_t)count;
}

static DEVICE_ATTR(temp_limit, 0644,
	sma6101_temp_limit_show, sma6101_temp_limit_store);


static ssize_t sma6101_temp_comp_gain_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE, "%d\n",
	sma6101->temp_match[sma6101->temp_table_number].comp_gain);

	return (ssize_t)rc;
}

static ssize_t sma6101_temp_comp_gain_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma6101->temp_comp_gain);

	if (ret)
		return -EINVAL;

	sma6101->temp_match[sma6101->temp_table_number].comp_gain =
		(int)sma6101->temp_comp_gain;

	return (ssize_t)count;
}

static DEVICE_ATTR(temp_comp_gain, 0644,
	sma6101_temp_comp_gain_show, sma6101_temp_comp_gain_store);


static ssize_t sma6101_temp_ocp_count_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE, "%d\n",
	sma6101->temp_match[sma6101->temp_table_number].ocp_count);

	return (ssize_t)rc;
}

static DEVICE_ATTR(temp_ocp_count, 0644,
	sma6101_temp_ocp_count_show, NULL);

static ssize_t sma6101_temp_hit_count_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE, "%d\n",
	sma6101->temp_match[sma6101->temp_table_number].hit_count);

	return (ssize_t)rc;
}

static DEVICE_ATTR(temp_hit_count, 0644,
	sma6101_temp_hit_count_show, NULL);

static ssize_t sma6101_temp_activate_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE, "%d\n",
	sma6101->temp_match[sma6101->temp_table_number].activate);

	return (ssize_t)rc;
}

static ssize_t sma6101_temp_activate_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma6101->temp_activate);

	if (ret)
		return -EINVAL;

	sma6101->temp_match[sma6101->temp_table_number].activate =
		(unsigned int)sma6101->temp_activate;

	return (ssize_t)count;
}

static DEVICE_ATTR(temp_activate, 0644,
	sma6101_temp_activate_show, sma6101_temp_activate_store);

static ssize_t sma6101_enable_ocp_aging_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE,
			"%ld\n", sma6101->enable_ocp_aging);

	return (ssize_t)rc;
}

static ssize_t sma6101_enable_ocp_aging_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma6101->enable_ocp_aging);

	if (ret)
		return -EINVAL;

	return (ssize_t)count;
}

static DEVICE_ATTR(enable_ocp_aging, 0644,
	sma6101_enable_ocp_aging_show, sma6101_enable_ocp_aging_store);

static ssize_t sma6101_check_thermal_fault_period_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE,
			"%ld\n", sma6101->check_thermal_fault_period);

	return (ssize_t)rc;
}

static ssize_t sma6101_check_thermal_fault_period_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma6101->check_thermal_fault_period);

	if (ret)
		return -EINVAL;

	return (ssize_t)count;
}

static DEVICE_ATTR(check_thermal_fault_period, 0644,
	sma6101_check_thermal_fault_period_show,
	sma6101_check_thermal_fault_period_store);

static ssize_t sma6101_check_thermal_fault_enable_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE,
			"%ld\n", sma6101->check_thermal_fault_enable);

	return (ssize_t)rc;
}

static ssize_t sma6101_check_thermal_fault_enable_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma6101->check_thermal_fault_enable);

	if (ret)
		return -EINVAL;

	return (ssize_t)count;
}

static DEVICE_ATTR(check_thermal_fault_enable, 0644,
	sma6101_check_thermal_fault_enable_show,
	sma6101_check_thermal_fault_enable_store);

static ssize_t sma6101_check_thermal_sensor_opt_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int rc = 0;

	/*quiet_therm :skin-therm, piezo_therm: wp-therm, vts: vts-virt-therm*/
	if (sma6101->thermal_sense_opt == -1) {
		rc = (int)snprintf(buf, PAGE_SIZE,
				"default selected:  skin(1) piezo(2),\n");
	} else {
		switch (sma6101->thermal_sense_opt) {
		case 1:
			rc = (int)snprintf(buf, PAGE_SIZE,
							"quiet_therm(skin-therm) selected\n");
			break;
		case 2:
			rc = (int)snprintf(buf, PAGE_SIZE,
							"piezo_therm(wp-therm) selected\n");
			break;
		}
	}
	return (ssize_t)rc;
}

static ssize_t sma6101_check_thermal_sensor_opt_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma6101_priv *sma6101 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma6101->thermal_sense_opt);

	if (ret || (sma6101->thermal_sense_opt != 1
			&& sma6101->thermal_sense_opt != 2)){
		sma6101->thermal_sense_opt = 1;
		return -EINVAL;
	}
	return (ssize_t)count;
}

static DEVICE_ATTR(check_thermal_sensor_opt, 0644,
	sma6101_check_thermal_sensor_opt_show,
	sma6101_check_thermal_sensor_opt_store);

static struct attribute *sma6101_attr[] = {
	&dev_attr_check_thermal_vbat_period.attr,
	&dev_attr_check_thermal_vbat_enable.attr,
	&dev_attr_check_thermal_table.attr,
	&dev_attr_check_thermal_value.attr,
	&dev_attr_temp_table_number.attr,
	&dev_attr_temp_limit.attr,
	&dev_attr_temp_comp_gain.attr,
	&dev_attr_temp_ocp_count.attr,
	&dev_attr_temp_hit_count.attr,
	&dev_attr_temp_activate.attr,
	&dev_attr_enable_ocp_aging.attr,
	&dev_attr_check_thermal_fault_period.attr,
	&dev_attr_check_thermal_fault_enable.attr,
	&dev_attr_check_thermal_sensor_opt.attr,
	NULL,
};

static struct attribute_group sma6101_attr_group = {
	.attrs = sma6101_attr,
	.name = "thermal_comp",
};

static int sma6101_probe(struct snd_soc_codec *codec)
{
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	char *dapm_widget_str = NULL;
	int prefix_len, ret = 0;
	int str_max = 30;

	dev_info(codec->dev, "%s\n", __func__);

#if defined(CONFIG_MACH_LGE)
	if(sma6101->sys_clk_id == SMA6101_EXTERNAL_CLOCK_24_576) {
		/* Parse pinctrl info from devicetree */
		ret = sma6101_msm_get_pinctrl(codec);
		if (!ret) {
			pr_debug("%s: pinctrl parsing successful\n", __func__);
		} else {
			pr_err("%s: Parsing pinctrl failed. Cannot use Ports\n", __func__);
			ret = 0;
		}
	} else if(sma6101->sys_clk_id == SMA6101_PLL_CLKIN_MCLK) {
		sma6101->ln_bb_clk3 = devm_clk_get(codec->dev, "ln_bb_clk3");
		if (IS_ERR(sma6101->ln_bb_clk3)) {
			ret = PTR_ERR(sma6101->ln_bb_clk3);
			pr_err("%s(): ln_bb_clk3 clock get fail %d\n", __func__,ret);
			return -EINVAL;		
		}
	}
#endif
	if (codec->component.name_prefix != NULL) {
		dev_info(codec->dev, "%s : component name prefix - %s\n",
			__func__, codec->component.name_prefix);

		prefix_len = strlen(codec->component.name_prefix);
		dapm_widget_str = kzalloc(prefix_len + str_max, GFP_KERNEL);

		if (!dapm_widget_str)
			return -ENOMEM;

		strcpy(dapm_widget_str, codec->component.name_prefix);
		strcat(dapm_widget_str, " Playback");

		snd_soc_dapm_ignore_suspend(dapm, dapm_widget_str);

		memset(dapm_widget_str + prefix_len, 0, str_max);

		strcpy(dapm_widget_str, codec->component.name_prefix);
		strcat(dapm_widget_str, " SPK");

		snd_soc_dapm_ignore_suspend(dapm, dapm_widget_str);
	} else {
		snd_soc_dapm_ignore_suspend(dapm, "Playback");
		snd_soc_dapm_ignore_suspend(dapm, "SPK");
	}

	snd_soc_dapm_sync(dapm);

	if (dapm_widget_str != NULL)
		kfree(dapm_widget_str);

	sma6101_reset(codec);

	ret = kfifo_alloc(&sma6101->data_fifo,
		sizeof(struct outside_status) * FIFO_BUFFER_SIZE,
		GFP_KERNEL);
	if (ret)
		dev_err(codec->dev, "%s: fifo alloc failed\n", __func__);

	return ret;
}

static int sma6101_remove(struct snd_soc_codec *codec)
{
	struct sma6101_priv *sma6101 = snd_soc_codec_get_drvdata(codec);
#if defined(CONFIG_MACH_LGE)
	if(sma6101->sys_clk_id == SMA6101_EXTERNAL_CLOCK_24_576)
		sma6101_msm_release_pinctrl(codec);
	else if(sma6101->sys_clk_id == SMA6101_PLL_CLKIN_MCLK)	
		sma6101_clock_control(sma6101->ln_bb_clk3,0); //disable the clk
#endif
	dev_info(codec->dev, "%s\n", __func__);

	sma6101_set_bias_level(codec, SND_SOC_BIAS_OFF);
	devm_free_irq(sma6101->dev, sma6101->irq, sma6101);
	devm_kfree(sma6101->dev, sma6101);

	kfifo_free(&sma6101->data_fifo);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_sma6101 = {
	.probe = sma6101_probe,
	.remove = sma6101_remove,
	.suspend = sma6101_suspend,
	.resume = sma6101_resume,
	.component_driver = {
		.controls = sma6101_snd_controls,
		.num_controls = ARRAY_SIZE(sma6101_snd_controls),
		.dapm_widgets = sma6101_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(sma6101_dapm_widgets),
		.dapm_routes = sma6101_audio_map,
		.num_dapm_routes = ARRAY_SIZE(sma6101_audio_map),
	},
	.idle_bias_off = true,
};

const struct regmap_config sma_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = SMA6101_FF_VERSION,
	.readable_reg = sma6101_readable_register,
	.writeable_reg = sma6101_writeable_register,
	.volatile_reg = sma6101_volatile_register,

	.cache_type = REGCACHE_NONE,
	.reg_defaults = sma6101_reg_def,
	.num_reg_defaults = ARRAY_SIZE(sma6101_reg_def),
};

static int sma6101_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct sma6101_priv *sma6101;
	struct device_node *np = client->dev.of_node;
	int ret;
	u32 value, value_clk;
	unsigned int version_status;

	dev_info(&client->dev, "%s is here. Driver version REV020\n", __func__);

	sma6101 = devm_kzalloc(&client->dev, sizeof(struct sma6101_priv),
							GFP_KERNEL);

	if (!sma6101)
		return -ENOMEM;

	sma6101->regmap = devm_regmap_init_i2c(client, &sma_i2c_regmap);
	if (IS_ERR(sma6101->regmap)) {
		ret = PTR_ERR(sma6101->regmap);
		dev_err(&client->dev,
			"Failed to allocate register map: %d\n", ret);
		return ret;
	}

	if (np) {
		if (!of_property_read_u32(np, "init-vol", &value)) {
			dev_info(&client->dev,
				"init-vol is 0x%x from DT\n", value);
			sma6101->init_vol = value;
		} else {
			dev_info(&client->dev,
				"init-vol is set with 0x30(0dB)\n");
			sma6101->init_vol = 0x30;
		}
		if (of_property_read_bool(np, "stereo-two-chip")) {
			dev_info(&client->dev, "Stereo for two chip solution\n");
				sma6101->stereo_two_chip = true;
		} else {
			dev_info(&client->dev, "Mono for one chip solution\n");
				sma6101->stereo_two_chip = false;
		}
		if (!of_property_read_u32(np, "sys-clk-id", &value)) {
			switch (value) {
			case SMA6101_EXTERNAL_CLOCK_19_2:
				dev_info(&client->dev, "Use the external 19.2MHz clock\n");
				break;
			case SMA6101_EXTERNAL_CLOCK_24_576:
				dev_info(&client->dev, "Use the external 24.576MHz clock\n");
#ifdef CONFIG_MACH_LGE
				sma6101->mclk_in = Q6AFE_LPASS_OSR_CLK_24_P576_MHZ;
				lpass_mclk.clk_freq_in_hz = sma6101->mclk_in;
#endif
				break;
			case SMA6101_PLL_CLKIN_MCLK:
				if (!of_property_read_u32(np,
					"mclk-freq", &value_clk))
					sma6101->mclk_in = value_clk;
				else
					sma6101->mclk_in = 19200000;

				dev_info(&client->dev,
				"Take an external %dHz clock and covert it to an internal PLL for use\n",
					sma6101->mclk_in);
				break;
			case SMA6101_PLL_CLKIN_BCLK:
				dev_info(&client->dev,
				"Take an BCLK(SCK) and covert it to an internal PLL for use\n");
				break;
			default:
				dev_err(&client->dev,
					"Invalid sys-clk-id: %d\n", value);
				return -EINVAL;
			}
			sma6101->sys_clk_id = value;
		} else {
			dev_info(&client->dev, "Use the internal PLL clock by default\n");
			sma6101->sys_clk_id = SMA6101_PLL_CLKIN_BCLK;
		}
		if (of_property_read_bool(np, "SRC-bypass")) {
			dev_info(&client->dev,
					"Do not set the sample rate converter\n");
				sma6101->src_bypass = true;
		} else {
			dev_info(&client->dev, "Set the sample rate converter\n");
				sma6101->src_bypass = false;
		}


#ifdef CONFIG_MACH_LGE
		sma6101->reset_gpio = of_get_named_gpio(np,"sm,reset-gpio",0);
		if(sma6101->reset_gpio < 0)
			dev_err(&client->dev,"Looking up %s property in node %s failed %d\n","sm,reset-gpio",np->full_name,sma6101->reset_gpio);
		else{
			dev_info(&client->dev,"reset_gpio %d",sma6101->reset_gpio);
			if(gpio_request(sma6101->reset_gpio,"sm_reset") < 0)
				dev_err(&client->dev,"SM reset gpio_request failed\n");
			if(gpio_direction_output(sma6101->reset_gpio,1) < 0)
				dev_err(&client->dev,"SM reset gpio_direction_output failed\n");
			__sma6101_reset_gpio_set(sma6101,1);
		}

		if (!of_property_read_u32(np, "sys-clk-id", &value)) {
			dev_info(&client->dev, "sys-clk-id is %d from DT\n", value);
		} else{
			dev_info(&client->dev, "sys-clk-id is set with %d by default\n",SMA6101_PLL_CLKIN_BCLK);
			value = SMA6101_PLL_CLKIN_BCLK;
		}

		sma6101->trm_vbst1 = TRM_VBST1_6V;
#endif
		sma6101->eq_reg_array = of_get_property(np, "registers-of-eq",
			&sma6101->eq_reg_array_len);
		if (sma6101->eq_reg_array == NULL)
			dev_info(&client->dev,
				"There is no EQ registers from DT\n");

		sma6101->bo_reg_array = of_get_property(np, "registers-of-bo",
			&sma6101->bo_reg_array_len);
		if (sma6101->bo_reg_array == NULL)
			dev_info(&client->dev,
				"There is no BrownOut registers from DT\n");

		sma6101->gpio_int = of_get_named_gpio(np,
				"sma6101,gpio-int", 0);
		if (!gpio_is_valid(sma6101->gpio_int)) {
			dev_err(&client->dev,
			"Looking up %s property in node %s failed %d\n",
			"sma6101,gpio-int", client->dev.of_node->full_name,
			sma6101->gpio_int);
		}
	} else {
		dev_err(&client->dev,
			"device node initialization error\n");
		return -ENODEV;
	}

	ret = regmap_read(sma6101->regmap, SMA6101_FF_VERSION, &version_status);

	if ((ret != 0) || (version_status != CHIP_VERSION)) {
		dev_err(&client->dev, "device initialization error (%d 0x%02X)",
				ret, version_status);
		return -ENODEV;
	}
	dev_info(&client->dev, "chip version 0x%02X\n", version_status);

	INIT_DELAYED_WORK(&sma6101->check_thermal_fault_work,
		sma6101_check_thermal_fault_worker);
	INIT_DELAYED_WORK(&sma6101->check_thermal_vbat_work,
		sma6101_check_thermal_vbat_worker);

	mutex_init(&sma6101->lock);
	sma6101->check_thermal_vbat_period = CHECK_COMP_PERIOD_TIME;
	sma6101->check_thermal_fault_period = CHECK_FAULT_PERIOD_TIME;
	sma6101->threshold_level = 0;
	sma6101->enable_ocp_aging = 0;
	sma6101->temp_table_number = 0;

	sma6101->devtype = id->driver_data;
	sma6101->dev = &client->dev;
	sma6101->kobj = &client->dev.kobj;
	sma6101->irq = -1;
	sma6101->pll_matches = sma6101_pll_matches;
	sma6101->num_of_pll_matches = ARRAY_SIZE(sma6101_pll_matches);
	sma6101->fdpec_gain_matches = sma6101_fdpec_gain_matches;
	sma6101->num_of_fdpec_gain_matches =
		ARRAY_SIZE(sma6101_fdpec_gain_matches);
	sma6101->temp_match = sma6101_temperature_gain_matches;
	sma6101->num_of_temperature_matches =
		ARRAY_SIZE(sma6101_temperature_gain_matches);

	if (gpio_is_valid(sma6101->gpio_int)) {

		dev_info(&client->dev, "%s , i2c client name: %s\n",
			__func__, dev_name(sma6101->dev));

		ret = gpio_request(sma6101->gpio_int, "sma6101-irq");
		if (ret) {
			dev_info(&client->dev, "gpio_request failed\n");
			return ret;
		}

		sma6101->irq = gpio_to_irq(sma6101->gpio_int);

		/* Get SMA6101 IRQ */
		if (sma6101->irq < 0) {
			dev_warn(&client->dev, "interrupt disabled\n");
		} else {
		/* Request system IRQ for SMA6101 */
			ret = request_threaded_irq(sma6101->irq,
				NULL, sma6101_isr, IRQF_ONESHOT |
				IRQF_TRIGGER_FALLING,
				"sma6101", sma6101);
			if (ret < 0) {
				dev_err(&client->dev, "failed to request IRQ(%u) [%d]\n",
						sma6101->irq, ret);
				sma6101->irq = -1;
				i2c_set_clientdata(client, NULL);
				devm_free_irq(&client->dev,
						sma6101->irq, sma6101);
				devm_kfree(&client->dev, sma6101);
				return ret;
			}
			disable_irq((unsigned int)sma6101->irq);
		}
	} else {
		dev_err(&client->dev,
			"interrupt signal input pin is not found\n");
	}

	atomic_set(&sma6101->irq_enabled, false);
	i2c_set_clientdata(client, sma6101);

	sma6101->force_amp_power_down = false;
	sma6101->amp_power_status = false;
	sma6101->check_thermal_vbat_enable = true;
	sma6101->check_thermal_fault_enable = true;
	sma6101->lowbattery_status = -1;
	sma6101->thermal_sense_opt = -1;
#ifdef CONFIG_MACH_LGE
	sma6101->voice_music_class_h_mode_for_volume_boost = 0;
#endif

	ret = snd_soc_register_codec(&client->dev,
		&soc_codec_dev_sma6101, sma6101_dai, ARRAY_SIZE(sma6101_dai));

	/* Create sma6101 sysfs attributes */
	sma6101->attr_grp = &sma6101_attr_group;
	ret = sysfs_create_group(sma6101->kobj, sma6101->attr_grp);

	if (ret) {
		pr_err("failed to create attribute group [%d]\n", ret);
		sma6101->attr_grp = NULL;
	}

	return ret;
}

static int sma6101_i2c_remove(struct i2c_client *client)
{
	struct sma6101_priv *sma6101 =
		(struct sma6101_priv *) i2c_get_clientdata(client);

	dev_info(&client->dev, "%s\n", __func__);

	if (sma6101->irq < 0)
		devm_free_irq(&client->dev, sma6101->irq, sma6101);

	if (sma6101) {
		sysfs_remove_group(sma6101->kobj, sma6101->attr_grp);
		devm_kfree(&client->dev, sma6101);
	}

	snd_soc_unregister_codec(&client->dev);

	return 0;
}


int sma6101_module_dep(void)
{
	return 0;
}
EXPORT_SYMBOL(sma6101_module_dep);

static const struct i2c_device_id sma6101_i2c_id[] = {
	{"sma6101", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, sma6101_i2c_id);

static const struct of_device_id sma6101_of_match[] = {
	{ .compatible = "siliconmitus,sma6101", },
	{ }
};
MODULE_DEVICE_TABLE(of, sma6101_of_match);

static struct i2c_driver sma6101_i2c_driver = {
	.driver = {
		.name = "sma6101",
		.of_match_table = sma6101_of_match,
	},
	.probe = sma6101_i2c_probe,
	.remove = sma6101_i2c_remove,
	.id_table = sma6101_i2c_id,
};

static int __init sma6101_init(void)
{
	int ret;

	pr_info("%s : module init\n", __func__);
	ret = i2c_add_driver(&sma6101_i2c_driver);

	if (ret)
		pr_err("Failed to register sma6101 I2C driver: %d\n", ret);

	return ret;
}

static void __exit sma6101_exit(void)
{
	i2c_del_driver(&sma6101_i2c_driver);
}

module_init(sma6101_init);
module_exit(sma6101_exit);

MODULE_DESCRIPTION("ALSA SoC SMA6101 driver");
MODULE_AUTHOR("HK Lee, <hwankyu.lee@siliconmitus.com>");
MODULE_AUTHOR("GH Park, <gyuhwa.park@irondevice.com>");
MODULE_LICENSE("GPL v2");
