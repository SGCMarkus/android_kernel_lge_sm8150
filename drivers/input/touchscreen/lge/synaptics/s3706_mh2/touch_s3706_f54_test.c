/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright ?2012 Synaptics Incorporated. All rights reserved.
 *
 * The information in this file is confidential under the terms
 * of a non-disclosure agreement with Synaptics and is provided
 * AS IS.
 *
 * The information in this file shall remain the exclusive property
 * of Synaptics and may be the subject of Synaptics?patents, in
 * whole or part. Synaptics?intellectual property rights in the
 * information in this file are not expressly or implicitly licensed
 * or otherwise transferred to you as a result of such information
 *  being made available to you.
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

/* FullRawCapacitance Support 0D button */
#define TS_MODULE "[refcode_f54]"

/*
 * Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 * Include to Local Header File
 */
#include "touch_s3706.h"
#include "touch_s3706_prd.h"
#include "touch_s3706_f54_test.h"

const int DefaultTimeout = 30;	/* In counts */

static int pageNum;
static int scanMaxPageCount = 5;
static int MaxArrayLength;

static uint8_t ButtonCount;

static uint8_t mask;

static uint8_t F01ControlBase;
static uint8_t F01CommandBase;

static uint8_t F51DataBase;
static uint8_t F51ControlBase;

static uint8_t F54DataBase;
static uint8_t F54QueryBase;
static uint8_t F54ControlBase;
static uint8_t F54CommandBase;

static uint8_t F55QueryBase;
static uint8_t F55ControlBase;

uint8_t RxChannelCount;
uint8_t TxChannelCount;
static uint8_t TouchControllerFamily;
static uint8_t CurveCompensationMode;
static uint8_t NumOfSensingFreq;

static bool bHaveF01;
static bool bHaveF12;
static bool bHaveF54;
static bool bHaveF55;

static bool SignalClarityOn;
static bool bHavePixelTouchThresholdTuning;
static bool bHaveInterferenceMetric;
static bool bHaveRelaxationControl;
static bool bHaveSensorAssignment;
static bool bHaveSenseFrequencyControl;
static bool bHaveFirmwareNoiseMitigation;
static bool bHaveIIRFilter;
static bool bHaveCmnRemoval;
static bool bHaveCmnMaximum;
static bool bHaveTouchHysteresis;
static bool bHaveEdgeCompensation;
static bool bHavePerFrequencyNoiseControl;
static bool bHaveSignalClarity;
static bool bHaveMultiMetricStateMachine;
static bool bHaveVarianceMetric;
static bool bHave0DRelaxationControl;
static bool bHave0DAcquisitionControl;
static bool bHaveSlewMetric;
static bool bHaveHBlank;
static bool bHaveVBlank;
static bool bHaveLongHBlank;
static bool bHaveNoiseMitigation2;
static bool bHaveSlewOption;
static bool bHaveEnhancedStretch;
static bool bHaveStartupFastRelaxation;
static bool bHaveESDControl;
static bool bHaveEnergyRatioRelaxation;
static bool ButtonShared;
static bool bIncellDevice;

static bool bHaveCtrl11;
static bool bHaveCtrl86;
static bool bHaveCtrl87;
static bool bHaveCtrl88;
static bool bHaveCtrl89;
static bool bHaveCtrl90;
static bool bHaveCtrl91;
static bool bHaveCtrl92;
static bool bHaveCtrl93;
static bool bHaveCtrl94;
static bool bHaveCtrl95;
static bool bHaveCtrl96;
static bool bHaveCtrl97;
static bool bHaveCtrl98;
static bool bHaveCtrl99;
static bool bHaveCtrl100;
static bool bHaveCtrl101;
static bool bHaveCtrl102;

static bool bHaveF54Query13;
static bool bHaveF54Query15;
static bool bHaveF54Query16;
static bool bHaveF54Query17;
static bool bHaveF54Query18;
static bool bHaveF54Query19;
static bool bHaveF54Query20;
static bool bHaveF54Query21;
static bool bHaveF54Query22;
static bool bHaveF54Query23;
static bool bHaveF54Query24;
static bool bHaveF54Query25;
static bool bHaveF54Query26;
static bool bHaveF54Query27;
static bool bHaveF54Query28;
static bool bHaveF54Query29;
static bool bHaveF54Query30;
static bool bHaveF54Query31;
static bool bHaveF54Query32;
static bool bHaveF54Query33;
static bool bHaveF54Query34;
static bool bHaveF54Query35;
static bool bHaveF54Query36;
static bool bHaveF54Query37;
static bool bHaveF54Query38;
static bool bHaveF54Query39;
static bool bHaveF54Query40;
static bool bHaveF54Query41;
static bool bHaveF54Query42;
static bool bHaveF54Query43;
static bool bHaveF54Query44;
static bool bHaveF54Query45;
static bool bHaveF54Query46;
static bool bHaveF54Query47;
static bool bHaveF54Query48;
static bool bHaveF54Query49;
static bool bHaveF54Query50;
static bool bHaveF54Query51;
static bool bHaveF54Query52;
static bool bHaveF54Query53;
static bool bHaveF54Query54;
static bool bHaveF54Query55;
static bool bHaveF54Query56;
static bool bHaveF54Query57;
static bool bHaveF54Query58;
static bool bHaveF54Query59;
static bool bHaveF54Query60;
static bool bHaveF54Query61;
static bool bHaveF54Query62;
static bool bHaveF54Query63;
static bool bHaveF54Query64;
static bool bHaveF54Query65;
static bool bHaveF54Query66;
static bool bHaveF54Query67;
static bool bHaveF54Query68;
static bool bHaveF54Query69;
static bool bHaveF54Query70;

static bool bHaveF54Ctrl07;
static bool bHaveF54Ctrl57;
static bool bHaveF54Ctrl103;
static bool bHaveF54Ctrl104;
static bool bHaveF54Ctrl105;
static bool bHaveF54Ctrl106;
static bool bHaveF54Ctrl107;
static bool bHaveF54Ctrl108;
static bool bHaveF54Ctrl109;
static bool bHaveF54Ctrl110;
static bool bHaveF54Ctrl111;
static bool bHaveF54Ctrl112;
static bool bHaveF54Ctrl113;
static bool bHaveF54Ctrl114;
static bool bHaveF54Ctrl115;
static bool bHaveF54Ctrl116;
static bool bHaveF54Ctrl117;
static bool bHaveF54Ctrl118;
static bool bHaveF54Ctrl119;
static bool bHaveF54Ctrl120;
static bool bHaveF54Ctrl121;
static bool bHaveF54Ctrl122;
static bool bHaveF54Ctrl123;
static bool bHaveF54Ctrl124;
static bool bHaveF54Ctrl125;
static bool bHaveF54Ctrl126;
static bool bHaveF54Ctrl127;
static bool bHaveF54Ctrl128;
static bool bHaveF54Ctrl129;
static bool bHaveF54Ctrl130;
static bool bHaveF54Ctrl131;
static bool bHaveF54Ctrl132;
static bool bHaveF54Ctrl133;
static bool bHaveF54Ctrl134;
static bool bHaveF54Ctrl135;
static bool bHaveF54Ctrl136;
static bool bHaveF54Ctrl137;
static bool bHaveF54Ctrl138;
static bool bHaveF54Ctrl139;
static bool bHaveF54Ctrl140;
static bool bHaveF54Ctrl141;
static bool bHaveF54Ctrl142;
static bool bHaveF54Ctrl143;
static bool bHaveF54Ctrl144;
static bool bHaveF54Ctrl145;
static bool bHaveF54Ctrl146;
static bool bHaveF54Ctrl147;
static bool bHaveF54Ctrl148;
static bool bHaveF54Ctrl149;
static bool bHaveF54Ctrl150;
static bool bHaveF54Ctrl151;
static bool bHaveF54Ctrl152;
static bool bHaveF54Ctrl153;
static bool bHaveF54Ctrl154;
static bool bHaveF54Ctrl155;
static bool bHaveF54Ctrl156;
static bool bHaveF54Ctrl157;
static bool bHaveF54Ctrl158;
static bool bHaveF54Ctrl159;
static bool bHaveF54Ctrl160;
static bool bHaveF54Ctrl161;
static bool bHaveF54Ctrl162;
static bool bHaveF54Ctrl163;
static bool bHaveF54Ctrl164;
static bool bHaveF54Ctrl165;
static bool bHaveF54Ctrl166;
static bool bHaveF54Ctrl167;
static bool bHaveF54Ctrl168;
static bool bHaveF54Ctrl169;
static bool bHaveF54Ctrl170;
static bool bHaveF54Ctrl171;
static bool bHaveF54Ctrl172;
static bool bHaveF54Ctrl173;
static bool bHaveF54Ctrl174;
static bool bHaveF54Ctrl175;
static bool bHaveF54Ctrl176;
static bool bHaveF54Ctrl177;
static bool bHaveF54Ctrl178;
static bool bHaveF54Ctrl179;
static bool bHaveF54Ctrl180;
static bool bHaveF54Ctrl181;
static bool bHaveF54Ctrl182;
static bool bHaveF54Ctrl183;
static bool bHaveF54Ctrl184;
static bool bHaveF54Ctrl185;
static bool bHaveF54Ctrl186;
static bool bHaveF54Ctrl187;
static bool bHaveF54Ctrl188;
static bool bHaveF54Ctrl189;
static bool bHaveF54Ctrl190;
static bool bHaveF54Ctrl191;
static bool bHaveF54Ctrl192;
static bool bHaveF54Ctrl193;
static bool bHaveF54Ctrl194;
static bool bHaveF54Ctrl195;
static bool bHaveF54Ctrl196;
static bool bHaveF54Ctrl197;
static bool bHaveF54Ctrl198;
static bool bHaveF54Ctrl199;
static bool bHaveF54Ctrl200;
static bool bHaveF54Ctrl201;
static bool bHaveF54Ctrl202;
static bool bHaveF54Ctrl203;
static bool bHaveF54Ctrl204;
static bool bHaveF54Ctrl205;
static bool bHaveF54Ctrl206;
static bool bHaveF54Ctrl207;
static bool bHaveF54Ctrl208;
static bool bHaveF54Ctrl209;
static bool bHaveF54Ctrl210;
static bool bHaveF54Ctrl211;
static bool bHaveF54Ctrl212;
static bool bHaveF54Ctrl213;
static bool bHaveF54Ctrl214;
static bool bHaveF54Ctrl215;
static bool bHaveF54Ctrl216;
static bool bHaveF54Ctrl217;
static bool bHaveF54Ctrl218;
static bool bHaveF54Ctrl219;
static bool bHaveF54Ctrl220;
static bool bHaveF54Ctrl221;
static bool bHaveF54Ctrl222;
static bool bHaveF54Ctrl223;
static bool bHaveF54Ctrl224;
static bool bHaveF54Ctrl225;
static bool bHaveF54Ctrl226;
static bool bHaveF54Ctrl227;
static bool bHaveF54Ctrl228;
static bool bHaveF54Ctrl229;
static bool bHaveF54Ctrl230;
static bool bHaveF54Ctrl231;
static bool bHaveF54Ctrl232;
static bool bHaveF54Ctrl233;
static bool bHaveF54Ctrl234;
static bool bHaveF54Ctrl235;
static bool bHaveF54Ctrl236;
static bool bHaveF54Ctrl237;
static bool bHaveF54Ctrl238;
static bool bHaveF54Ctrl239;
static bool bHaveF54Ctrl240;
static bool bHaveF54Ctrl241;
static bool bHaveF54Ctrl242;
static bool bHaveF54Ctrl243;
static bool bHaveF54Ctrl244;
static bool bHaveF54Ctrl245;
static bool bHaveF54Ctrl246;

static uint8_t F12ControlBase;
static uint8_t F12QueryBase;
static uint8_t F12_2DTxCount;
static uint8_t F12_2DRxCount;
static uint8_t F12Support;

static uint8_t F54Ctrl07Offset;
static uint8_t F54Ctrl08Offset;
static uint8_t F54Ctrl20Offset;
static uint8_t F54Ctrl41Offset;
static uint8_t F54Ctrl57Offset;
static uint8_t F54Ctrl88Offset;
static uint8_t F54Ctrl89Offset;
static uint8_t F54Ctrl91Offset;
static uint8_t F54Ctrl95Offset;
static uint8_t F54Ctrl96Offset;
static uint8_t F54Ctrl97Offset;
static uint8_t F54Ctrl98Offset;
static uint8_t F54Ctrl99Offset;
static uint8_t F54Ctrl102Offset;
static uint8_t F54Ctrl132Offset;
static uint8_t F54Ctrl146Offset;
static uint8_t F54Ctrl147Offset;
static uint8_t F54Ctrl149Offset;
static uint8_t F54Ctrl182Offset;
static uint8_t F54Ctrl186Offset;
static uint8_t F54Ctrl188Offset;
static uint8_t F54Ctrl189Offset;
static uint8_t F54Ctrl215Offset;
static uint8_t F54Ctrl225Offset;
static uint8_t F54Ctrl246Offset;

/* Assuming Tx = 32 & Rx = 32 to accommodate any configuration */
static short Image1[TRX_MAX][TRX_MAX];
static short ImagepF[TRX_MAX][TRX_MAX];
static short baseline_image0[TRX_MAX][TRX_MAX];
static short baseline_image1[TRX_MAX][TRX_MAX];
static short delta[TRX_MAX][TRX_MAX];
static short noise_min[TRX_MAX][TRX_MAX];
static short noise_max[TRX_MAX][TRX_MAX];
static uint8_t err_array[TRX_BITMAP_LENGTH] = {0, };
static int HybridAbsData[TRX_MAX * TRX_MAX * 4];
static uint8_t Data[TRX_MAX * TRX_MAX * 4];
static uint8_t TRxPhysical[TRX_MAPPING_MAX];
static uint8_t TRxPhysical_bit[TRX_MAPPING_MAX];
static uint8_t TxPhysical[TRX_MAPPING_MAX];
static uint8_t RxPhysical[TRX_MAPPING_MAX];
static uint8_t ExtendRT26_pin[4] = {0, 1, 32, 33};
static int16_t TransRxShortData[TRX_MAX];

struct timeval t_interval[TIME_PROFILE_MAX];
static int f54len;
static char f54buf[BUF_SIZE] = {0};

/* Function to switch beteen register pages */
static bool switchPage(struct device *dev, int page)
{
	uint8_t values[1] = {0};
	uint8_t data = 0;
	unsigned int count = 0;

	pageNum = values[0] = page;

	do {
		Write8BitRegisters(dev, 0xFF, values, 1);
		touch_msleep(20);
		Read8BitRegisters(dev, 0xFF, &data, 1);
		count++;
	} while ((int)data != page && (count < DefaultTimeout));

	if (count >= DefaultTimeout) {
		TOUCH_E("Timeout -- Page switch fail !\n");
		return -EAGAIN;
	}

	return true;
}

static void Reset(struct device *dev)
{
	uint8_t data = 0;

	TOUCH_TRACE();

	switchPage(dev, DEFAULT);

	data = 0x01;
	Write8BitRegisters(dev, F01CommandBase, &data, 1);

	TOUCH_I("reset on production test\n");
	touch_msleep(100);
}

static int prepareTest(struct device *dev, u16 data)
{
	int ret = 0;
	u8 buf = 0;
	u8 buffer[2] = {0, 0};

	TOUCH_TRACE();

	/* Test Preparation - Full Raw Cap Test */
	if (data == eRT_FullRawCapacitance) {
		ret = Read8BitRegisters(dev, F54ControlBase + 21, &buf, 1);
		if (ret < 0) {
			TOUCH_E("failed to read (F54ControlBase + 21) - ret:%d\n", ret);
			goto out;
		}

		buf = (buf & 0xdf);
		ret = Write8BitRegisters(dev, F54ControlBase + 21, &buf, 1);
		if (ret < 0) {
			TOUCH_E("failed to write (F54ControlBase + 21) - ret:%d\n", ret);
			goto out;
		}

		buf = 0x01;
		ret = Write8BitRegisters(dev, F54ControlBase + 20, &buf, 1);
		if (ret < 0) {
			TOUCH_E("failed to write (F54ControlBase + 20) - ret:%d\n", ret);
			goto out;
		}

		ret = s3706_force_update(dev);
		if (ret < 0) {
			TOUCH_E("force update failed - ret:%d\n", ret);
			goto out;
		}

		ret = s3706_force_calibration(dev);
		if (ret < 0) {
			TOUCH_E("force calibration failed - ret:%d\n", ret);
			goto out;
		}

		ret = switchPage(dev, 0x01);
		if (ret < 0) {
			TOUCH_I("[%s] retry switchPage, fail\n", __func__);
			return ret;
		}
	}

	/* Reset Read Report Index */
	ret = Write8BitRegisters(dev, F54DataBase + 1, &buffer[0], sizeof(buffer));
	if (ret < 0)
		goto out;

	/* Assign report type for Test*/
	ret = Write8BitRegisters(dev, F54DataBase, &data, 1);
	if (ret < 0)
		goto out;

out:
	return ret;
}

static int EnterActiveMode(struct device *dev)
{
	struct s3706_data *d = to_s3706_data(dev);

	int ret = 0;
	uint8_t data = 0;

	TOUCH_TRACE();

	ret = switchPage(dev, DEFAULT);
	if (ret < 0) {
		TOUCH_I("[%s] retry switchPage, fail\n", __func__);
		return ret;
	}
	data = 0x04;
	ret = Write8BitRegisters(dev, DEVICE_CONTROL_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("failed to write DEVICE_CONTROL_REG - ret:%d\n", ret);
		return ret;
	}

	ret = switchPage(dev, COMMON);
	if (ret < 0) {
		TOUCH_I("[%s] retry switchPage, fail\n", __func__);
		return ret;
	}

	return ret;
}

static int GetLogicalPin(int p_pin)
{
	int i = 0;

	for (i = 0; i < RxChannelCount; i++) {
		if (RxPhysical[i] == p_pin)
			return i;
	}
	return 0xff;
}

static void setErr_Array(uint8_t trx, bool err)
{
	uint8_t byte = trx / 8;
	uint8_t bit = trx % 8;

	if (err)
		err_array[byte] = (err_array[byte] | (0x01 << bit));
	else
		err_array[byte] = (err_array[byte] - (0x01 << bit));
}

/* Compare Report type #20 data against test limits */
static int CompareImageReport(struct device *dev)
{
	struct s3706_data *d = to_s3706_data(dev);

	bool result = true;
	int i, j = 0;

	TOUCH_TRACE();

	/* Compare 2D area */
	for (j = 0; j < (int)F12_2DRxCount; j++) {
		for (i = 0; i < (int)F12_2DTxCount; i++) {
			if ((j == 0) && (i == 7 || i == 8))
				continue;

			if ((ImagepF[i][j] < rawcap_lower[i][j])
					|| (ImagepF[i][j] > rawcap_upper[i][j])) {
				result = false;
				f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
						"\nFail [%2d][%2d] = %5d",
						i, j, ImagepF[i][j]);
				if (f54len > (BUF_SIZE / 2)) {
					print_sd_log(f54buf);
					write_file(dev, f54buf, TIME_INFO_SKIP);
					memset(f54buf, 0, BUF_SIZE);
					f54len = 0;
				}
			}
		}
	}

	if (result == false) {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\n\n%sFull Raw Capacitance Image Test failed.\n\n",
				(d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");
	} else {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\n%sFull Raw Capacitance Image Test passed.\n\n",
				(d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");
	}

	return (result) ? 1 : 0;
}

/* Compare Report type #4 data against test limits */
static int CompareHighResistance(struct device *dev)
{
	bool result = true;
	int i, j = 0;
	int lower = 0;
	int upper = 0;

	TOUCH_TRACE();

	for (i = 0; i < TxChannelCount; i++) {
		for (j = 0; j < RxChannelCount; j++) {
			if (j == 0) {
				lower = high_resistance_notch_lower;
				upper = high_resistance_notch_upper;
			} else {
				lower = high_resistance_lower;
				upper = high_resistance_upper;
			}

			if ((j == 0) && (i == 7 || i == 8))
				continue;

			if (ImagepF[i][j] < lower
					|| ImagepF[i][j] > upper) {
				result = false;
				f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
						"\nFail [%2d][%2d] = %5d",
						i, j, ImagepF[i][j]);
				if (f54len > (BUF_SIZE / 2)) {
					print_sd_log(f54buf);
					write_file(dev, f54buf, TIME_INFO_SKIP);
					memset(f54buf, 0, BUF_SIZE);
					f54len = 0;
				}
			}
		}
	}

	if (result == false) {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\n\nHighResistance Test failed.\n\n");
	} else {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\nHighResistance Test passed.\n\n");
	}

	return (result) ? 1 : 0;
}

/* Compare Report type #63 data against test limits */
static int CompareHybridAbsRawReport(struct device *dev)
{
	bool result = true;
	int i, k = 0;

	TOUCH_TRACE();

	for (i = 0; i < (int)RxChannelCount; i++) {
		if (HybridAbsData[k] > hybrid_abs_rx_upper[i]
					|| HybridAbsData[k] < hybrid_abs_rx_lower[i]) {
			result = false;
			f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"\nFail [%2d] = %6d", i, HybridAbsData[k]);
			if (f54len > (BUF_SIZE / 2)) {
				print_sd_log(f54buf);
				write_file(dev, f54buf, TIME_INFO_SKIP);
				memset(f54buf, 0, BUF_SIZE);
				f54len = 0;
			}
		}
		k++;
	}

	for (i = 0; i < (int)TxChannelCount; i++) {
		if (HybridAbsData[k] > hybrid_abs_tx_upper[i]
				|| HybridAbsData[k] < hybrid_abs_tx_lower[i]) {
			result = false;
			f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"\nFail [%2d] = %6d", i, HybridAbsData[k]);
			if (f54len > (BUF_SIZE / 2)) {
				print_sd_log(f54buf);
				write_file(dev, f54buf, TIME_INFO_SKIP);
				memset(f54buf, 0, BUF_SIZE);
				f54len = 0;
			}
		}
		k++;
	}

	if (result == false) {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"\n\nHybrid Abs Test Failed.\n\n");
	} else {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"\nHybrid Abs Test Passed.\n\n");
	}

	return (result) ? 1 : 0;
}

static void setExtendRT26result(struct device *dev)
{
	uint8_t byte, bit, i, j = 0;
	bool result_rt26 = true;
	bool in_use = false;
	/* set ExtendRT26_pin to default value */
	ExtendRT26_pin[0] = 0;
	ExtendRT26_pin[1] = 1;
	ExtendRT26_pin[2] = 32;
	ExtendRT26_pin[3] = 33;

	for (i = 0; i < 4; i++) {
		in_use = false;
		for (j = 0; j < TRX_MAPPING_MAX; j++) {
			if (ExtendRT26_pin[i] == TRxPhysical[j]) {
				in_use = true;
				break;
			}
		}

		if (!in_use) {
			ExtendRT26_pin[i] = 0xFF;
			continue;
		}

		if (ExtendRT26_pin[i] != 0) {
			byte = ExtendRT26_pin[i] / 8;
			bit = ExtendRT26_pin[i] % 8;
		} else {
			byte = 0;
			bit = 0;
		}

		err_array[byte] |= (0x01 << bit);
	}

	for (i = 0 ; i < TRX_BITMAP_LENGTH; i++) {
		if (err_array[i] != 0)
			result_rt26 &= false;
	}
}

/* Compare Report type #26 data against test limits */
static int CompareTRexShortTestReport(struct device *dev)
{
	bool result = true;
	int i, j = 0;

	TOUCH_TRACE();

	for (i = 0; i < TRX_BITMAP_LENGTH; i++) {
		Data[i] &= TRxPhysical_bit[i];

		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"Data[%d] = 0x%02X, trx_short_limit[%d] = 0x%02X\n",
				i, Data[i], i, trx_short_limit[i]);

		if (Data[i] != trx_short_limit[i]) {
			result = false;
			err_array[i] = Data[i] ^ trx_short_limit[i];

			f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
						"Fail [%d] = %#x\n", i, Data[i]);
			if (f54len > (BUF_SIZE / 2)) {
				print_sd_log(f54buf);
				write_file(dev, f54buf, TIME_INFO_SKIP);
				memset(f54buf, 0, BUF_SIZE);
				f54len = 0;
			}
		}
	}

	for (i = 0; i < TRX_BITMAP_LENGTH; i++) {
		for (j = 0; j < 8; j++) {
			if (err_array[i] & (0x01 << j)) {
				result = false;
				f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
						"Fail, TRX[%d]\n", ((i * 8) + j));
				if (f54len > (BUF_SIZE / 2)) {
					print_sd_log(f54buf);
					write_file(dev, f54buf, TIME_INFO_SKIP);
					memset(f54buf, 0, BUF_SIZE);
					f54len = 0;
				}
			}
		}
	}

	if (result == false) {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\n\nTRex-TRex Short Test failed.\n\n");
	} else {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\nTRex-TRex Short Test passed.\n\n");
	}

	setExtendRT26result(dev);

	return (result) ? 1 : 0;
}

static void print_noise_report(struct device *dev, short (*data)[TRX_MAX])
{
	int i, j = 0;

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "     ");

	for (i = 0; i < (int)RxChannelCount; i++)
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				" [%2d] ", i);

	for (i = 0; i < (int)TxChannelCount; i++) {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\n[%2d] ", i);
		for (j = 0; j < (int)RxChannelCount; j++) {
			f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"%5d ", data[i][j]);
		}
	}

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n");

	print_sd_log(f54buf);
	write_file(dev, f54buf, TIME_INFO_SKIP);
	memset(f54buf, 0, BUF_SIZE);
	f54len = 0;

}

/* Compare Report type #2 data against test limits */
static int CompareNoiseReport(struct device *dev)
{
	struct s3706_data *d = to_s3706_data(dev);

	bool result = true;
	int i, j = 0;
	int min = 9999;
	int max = 0;
	int max_tx = 0;
	int max_rx = 0;
	int min_tx = 0;
	int min_rx = 0;

	TOUCH_TRACE();

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "===== %sNoise Test =====",
			(d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"\n[%sNoise Min]\n", (d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");
	print_noise_report(dev, noise_min);

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"\n[%sNoise Max]\n", (d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");
	print_noise_report(dev, noise_max);

	for (i = 0; i < (int)TxChannelCount; i++) {
		for (j = 0; j < (int)RxChannelCount; j++) {
			if (noise_min[i][j] < min) {
				min = noise_min[i][j];
				min_tx = i;
				min_rx = j;
			}

			if (noise_max[i][j] > max) {
				max = noise_max[i][j];
				max_tx = i;
				max_rx = j;
			}

			if (noise_min[i][j] < jitter_lower
					|| noise_max[i][j] > jitter_upper) {
				result = false;
				f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
						"\nFail [%2d][%2d] : Min[%5d], Max[%5d]",
						i, j, noise_min[i][j], noise_max[i][j]);

				if (f54len > (BUF_SIZE / 2)) {
					print_sd_log(f54buf);
					write_file(dev, f54buf, TIME_INFO_SKIP);
					memset(f54buf, 0, BUF_SIZE);
					f54len = 0;
				}
			}
		}
	}

	if (result == false)
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\n%sNoise min[%2d][%2d] : %d, max[%2d][%2d] : %d\n",
				((d->lcd_mode != LCD_MODE_U3) ? "LPWG " : ""),
				min_tx, min_rx, min, max_tx, max_rx, max);

	if (result == false) {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\n%sNoise Test failed.\n\n",
				(d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");
	} else {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\n%sNoise Test passed.\n\n",
				(d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");
	}

	print_sd_log(f54buf);
	write_file(dev, f54buf, TIME_INFO_SKIP);

	return (result) ? 1 : 0;
}

/* Compare Report type #133 data against test limits */
static int CompareTransRxShortReport(struct device *dev)
{
	bool result = true;
	int i = 0;

	TOUCH_TRACE();

	for (i = 0; i < RxChannelCount; i++) {
		if (TransRxShortData[i] >= trans_rx_short_limit[i]) {
			result = false;
			f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"\nFail [%2d] = %5d",
					i, TransRxShortData[i]);
			if (f54len > (BUF_SIZE / 2)) {
				print_sd_log(f54buf);
				write_file(dev, f54buf, TIME_INFO_SKIP);
				memset(f54buf, 0, BUF_SIZE);
				f54len = 0;
			}
		}
	}

	if (result == false) {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\n\nTransRxShort Test failed.\n\n");
	} else {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\nTransRxShort Test passed.\n\n");
	}

	return (result) ? 1 : 0;
}

/* Construct data with Report Type #20 data */
static int ReadImageReport(struct device *dev)
{
	struct s3706_data *d = to_s3706_data(dev);

	int ret = 0;
	int i, j, k = 0;
	int min = 9999;
	int max = 0;

	TOUCH_TRACE();

	ret = Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST), &Data[0], MaxArrayLength);
	if (ret < 0) {
		TOUCH_E("failed to read (F54DataBase + REPORT_DATA_OFFEST) - ret:%d\n", ret);
		return -EAGAIN;
	}

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"===== %sFull Raw Capacitance Test =====\n",
			(d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"Tx = %d, Rx = %d\n",
			(int)TxChannelCount, (int)RxChannelCount);

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"[%sFull Raw Capacitance]\n",
			(d->lcd_mode != LCD_MODE_U3) ? "LPWG " : "");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "     ");

	for (i = 0; i < (int)RxChannelCount; i++)
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				" [%2d] ", i);

	for (i = 0; i < (int)TxChannelCount; i++) {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\n[%2d] ", i);
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = ((short)Data[k] | (short)Data[k + 1] << 8);
			ImagepF[i][j] = Image1[i][j];
			f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
								"%5d ", ImagepF[i][j]);
			k = k + 2;
			if (ImagepF[i][j] != 0 && ImagepF[i][j] < min)
				min = ImagepF[i][j];

			if (ImagepF[i][j] > max)
				max = ImagepF[i][j];
		}
	}
	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\nRawdata min : %d, max : %d\n", min, max);

	ret = CompareImageReport(dev);

	print_sd_log(f54buf);
	write_file(dev, f54buf, TIME_INFO_SKIP);

	Reset(dev);

	return ret;
}

/* Get Rawdata for ExtendedTRXShortTest */
static int GetExtendedTRexImageReport(struct device *dev)
{
	int ret = 0;
	int i, j, k = 0;

	TOUCH_TRACE();

	ret = Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST), &Data[0], MaxArrayLength);
	if (ret < 0) {
		TOUCH_E("failed to read (F54DataBase + REPORT_DATA_OFFEST) - ret:%d\n", ret);
		return -EAGAIN;
	}

	for (i = 0; i < (int)TxChannelCount; i++) {
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = ((short)Data[k]
					| (short)Data[k + 1] << 8);
			ImagepF[i][j] = Image1[i][j];
			k = k + 2;
		}
	}

	return ret;
}

/* Print Rawdata or Delta */
static int GetImageReport(struct device *dev, int input, char *buf)
{
	int ret = 0;
	int i, j, k = 0;

	TOUCH_TRACE();

	ret = Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST), &Data[0], MaxArrayLength);
	if (ret < 0) {
		TOUCH_E("failed to read (F54DataBase + REPORT_DATA_OFFEST) - ret:%d\n", ret);
		return -EAGAIN;
	}

	ret = strlen(buf);

	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "Tx = %d, Rx = %d\n",
			(int)TxChannelCount, (int)RxChannelCount);

	for (i = 0; i < (int)TxChannelCount; i++) {
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = ((short)Data[k]
					| (short)Data[k + 1] << 8);
			ImagepF[i][j] = Image1[i][j];
			k = k + 2;
		}
	}

	for (i = 0; i < (int)RxChannelCount; i++) {
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		for (j = 0; j < (int)TxChannelCount; j++) {
			ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
					"%5d ", ImagepF[j][i]);
		}
		ret += touch_snprintf(buf + ret, PAGE_SIZE - ret, "\n");
	}

	if (input == eRT_FullRawCapacitance)
		Reset(dev);

	return ret;
}

/* Construct data with Report Type #2 data */
static int ReadNoiseReport(struct device *dev)
{
	u8 buf[2] = {0, 0};
	int ret = 0;
	int i, j, k = 0;

	TOUCH_TRACE();

	/* Reset Read Report Index */
	ret = Write8BitRegisters(dev, F54DataBase + 1, &buf[0], sizeof(buf));
	if (ret < 0) {
		TOUCH_E("failed to write reset index - ret:%d\n", ret);
		return -EAGAIN;
	}

	ret = Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST), &Data[0], MaxArrayLength);
	if (ret < 0) {
		TOUCH_E("failed to read (F54DataBase + REPORT_DATA_OFFEST) - ret:%d\n", ret);
		return -EAGAIN;
	}

	for (i = 0; i < (int)TxChannelCount; i++) {
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = (short)Data[k] | ((short)Data[k + 1] << 8);
			ImagepF[i][j] = Image1[i][j];

			if (ImagepF[i][j] < noise_min[i][j])
				noise_min[i][j] = ImagepF[i][j];

			if (ImagepF[i][j] > noise_max[i][j])
				noise_max[i][j] = ImagepF[i][j];

			k = k + 2;
		}
	}

	return ret;
}

/* Construct data with Report Type #4 data */
static int ReadHighResistanceReport(struct device *dev)
{
	int ret = 0;
	int i, j = 0;
	int k = 6;
	int notch_min = 9999;
	int notch_max = 0;
	int min = 9999;
	int max = 0;

	TOUCH_TRACE();

	ret = Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST), &Data[0],
			(2 * (3 + (TxChannelCount + 1) * (RxChannelCount + 1))));
	if (ret < 0) {
		TOUCH_E("failed to read (F54DataBase + REPORT_DATA_OFFEST) - ret:%d\n", ret);
		return -EAGAIN;
	}

	for (i = 0; i < (int)RxChannelCount; i++)
		k = k + 2;

	for (i = 0; i < (int)TxChannelCount; i++)
		k = k + 2;

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"===== High Resistance Test =====\n");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"Tx = %d, Rx = %d\n",
			(int)TxChannelCount, (int)RxChannelCount);

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"[High Resistance]\n");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "      ");

	for (i = 0; i < (int)RxChannelCount; i++)
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				" [%2d] ", i);

	for (i = 0; i < (int)TxChannelCount; i++) {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\n[%2d] ", i);
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = ((short)Data[k] | (short)Data[k + 1] << 8);
			ImagepF[i][j] = Image1[i][j];

			k = k + 2;

			//ignore a value where notch
			if ((j == 0) && (i == 7 || i == 8)) {
				f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
						"%5d ", 0);
			} else {
				f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
						"%5d ", ImagepF[i][j]);

				if (j == 0) {
					if (ImagepF[i][j] < notch_min)
						notch_min = ImagepF[i][j];
					if (ImagepF[i][j] > notch_max)
						notch_max = ImagepF[i][j];
				} else {
					if (ImagepF[i][j] < min)
						min = ImagepF[i][j];
					if (ImagepF[i][j] > max)
						max = ImagepF[i][j];
				}
			}
		}
	}

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n\n");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"\nHigh Resistance notch_min: %d, notch_max: %d, min: %d, max: %d\n",
				notch_min, notch_max, min, max);

	ret = CompareHighResistance(dev);

	print_sd_log(f54buf);
	write_file(dev, f54buf, TIME_INFO_SKIP);

	Reset(dev);

	return ret;
}

/* Construct data with Report Type #63 data */
static int ReadHybridAbsRawReport(struct device *dev)
{
	int i, k = 0;
	int *p32data = NULL;
	int ret = 0;

	TOUCH_TRACE();

	ret = Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST),
				&Data[0], 4 * (RxChannelCount + TxChannelCount));
	if (ret < 0) {
		TOUCH_E("failed to read (F54DataBase + REPORT_DATA_OFFEST) - ret:%d\n", ret);
		return -EAGAIN;
	}

	p32data = (int *)&Data[0];

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"===== Hybrid Abs Test =====\n");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
						"Rx = %d\n", (int)RxChannelCount);

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"[Hybrid Abs Test for Rx]\n");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "      ");

	for (i = 0; i < (int)RxChannelCount; i++)
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"  [%2d] ", i);

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n[%2d] ", 0);

	for (i = 0; i < (int)RxChannelCount; i++) {
		HybridAbsData[k] = (int)*p32data;
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				" %6d", HybridAbsData[k]);
		k++;
		p32data++;
	}
	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n\n");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
						"Tx = %d\n", (int)TxChannelCount);

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"[Hybrid Abs Test for Tx]\n");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "      ");

	for (i = 0; i < (int)TxChannelCount; i++)
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"  [%2d] ", i);

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n[%2d] ", 0);

	for (i = 0; i < (int)TxChannelCount; i++) {
		HybridAbsData[k] = (int)*p32data;
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				" %6d", HybridAbsData[k]);
		k++;
		p32data++;
	}
	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n");

	ret = CompareHybridAbsRawReport(dev);

	print_sd_log(f54buf);
	write_file(dev, f54buf, TIME_INFO_SKIP);

	Reset(dev);

	return ret;
}

/* Construct data with Report Type #26 data */
static int ReadTRexShortReport(struct device *dev)
{
	int ret = 0;

	TOUCH_TRACE();

	ret = EnterActiveMode(dev);
	if (ret < 0) {
		TOUCH_E("%s, EnterActiveMode failed\n", __func__);
		return -EAGAIN;
	}

	ret = Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST), &Data[0], TRX_BITMAP_LENGTH);
	if (ret < 0) {
		TOUCH_E("failed to read (F54DataBase + REPORT_DATA_OFFEST) - ret:%d\n", ret);
		return -EAGAIN;
	}

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"===== Extend TRx Short Test =====\n");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"[TRex Short Test]\n");

	ret = CompareTRexShortTestReport(dev);

	print_sd_log(f54buf);
	write_file(dev, f54buf, TIME_INFO_SKIP);

	Reset(dev);

	return ret;
}

/* Construct data with Report Type #133 data */
static int ReadTransRxShortReport(struct device *dev)
{
	int ret = 0;
	int i, k = 0;

	TOUCH_TRACE();

	ret = Read8BitRegisters(dev, (F54DataBase + REPORT_DATA_OFFEST), &Data[0], RxChannelCount * 2);
	if (ret < 0) {
		TOUCH_E("failed to read (F54DataBase + REPORT_DATA_OFFEST) - ret:%d\n", ret);
		return -EAGAIN;
	}

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"===== Trans Rx Short Test =====\n");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
						"Rx = %d\n", (int)RxChannelCount);

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				"[Trans Rx Short]\n");

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "      ");

	for (i = 0; i < (int)RxChannelCount; i++)
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"  [%2d] ", i);

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n[%2d] ", 0);

	for (i = 0; i < (int)RxChannelCount; i++) {
		TransRxShortData[i] = (int16_t)Data[k] | ((int16_t)Data[k + 1] << 8);
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
				" %6d", TransRxShortData[i]);
		k = k + 2;
	}

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n");

	ret = CompareTransRxShortReport(dev);

	print_sd_log(f54buf);
	write_file(dev, f54buf, TIME_INFO_SKIP);

	Reset(dev);

	return ret;
}

/* Function to handle report reads based on user input */
static int ReadReport(struct device *dev, u16 input, int mode, char *buf)
{
	int ret = 0;
	uint8_t data = 0;
	unsigned int count = 0;

	TOUCH_TRACE();

	/* Set the GetReport bit to run the AutoScan */
	data = 0x01;
	ret = Write8BitRegisters(dev, F54CommandBase, &data, 1);
	if (ret < 0) {
		TOUCH_E("failed to write F54CommandBase - ret:%d\n", ret);
		goto error;
	}

	do {
		ret = Read8BitRegisters(dev, F54CommandBase, &data, 1);
		if (ret < 0) {
			TOUCH_E("failed to read F54CommandBase - ret:%d\n", ret);
			goto error;
		}
		touch_msleep(10);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));

	if (count >= DefaultTimeout) {
		TOUCH_E("Timeout - Not supported Report Type in FW. (input: %d)\n", input);
		Reset(dev);
		goto error;
	}

	do_gettimeofday(&t_interval[ENDTIME]);

	TOUCH_I("Takes %lu ticks\n",
			get_time_interval(t_interval[ENDTIME].tv_sec,
				t_interval[STARTTIME].tv_sec));

	switch (input) {
	case eRT_RawImageRT100:
	case eRT_FullRawCapacitance:
		if (mode == 2)
			ret = GetExtendedTRexImageReport(dev);
		else if (mode == 1)
			ret = GetImageReport(dev, input, buf);
		else
			ret = ReadImageReport(dev);
		break;
	case eRT_TRexShort:
		ret = ReadTRexShortReport(dev);
		break;
	case eRT_HighResistance:
		ret = ReadHighResistanceReport(dev);
		break;
	case eRT_HybirdRawCap:
		ret = ReadHybridAbsRawReport(dev);
		break;
	case eRT_Normalized16BitImageReport:
		if (mode == 1)
			ret = GetImageReport(dev, input, buf);
		else
			ret = ReadNoiseReport(dev);
		break;
	case eRT_TransRxShortUsingCBCVariation:
		ret = ReadTransRxShortReport(dev);
		break;
	default:
		break;
	}

	return ret;

error:
	TOUCH_E("[%s] ReadReport fail\n", __func__);

	return -EAGAIN;
}

/* Examples of reading query registers.
 * Real applications often do not need to read query registers at all.
 */
static void RunQueries(struct device *dev)
{
	unsigned short cAddr = 0xEE;
	uint8_t cFunc = 0;
	int rxCount = 0;
	int txCount = 0;
	int offset = 0;
	int q_offset = 0;
	int i, j = 0;
	int tsvd_select = 0;
	int tsvd = 0;
	int tshd = 0;
	int tsstb = 0;
	int tsfrq = 0;
	int tsfst = 0;
	int exvcom_pin_type = 0;
	int exvcom1 = 0;
	int exvcom_sel = 0;
	int exvcom2 = 0;
	int enable_guard = 0;
	int guard_ring = 0;
	int enable_verf = 0;
	int verf = 0;
	uint8_t byte = 0;
	uint8_t bit = 0;
	bool HasCtrl102Sub1;
	bool HasCtrl102Sub2;
	bool HasCtrl102Sub4;
	bool HasCtrl102Sub5;
	bool HasCtrl102Sub9;
	bool HasCtrl102Sub10;
	bool HasCtrl102Sub11;
	bool HasCtrl102Sub12;

	/* Scan Page Description Table (PDT)
	 * to find all RMI functions presented by this device.
	 * The Table starts at $00EE. This and every sixth register
	 * (decrementing) is a function number
	 * except when this "function number" is $00, meaning end of PDT.
	 * In an actual use case this scan might be done only once
	 * on first run or before compile.
	 */
	TOUCH_TRACE();

	do {
		Read8BitRegisters(dev, cAddr, &cFunc, 1);
		if (cFunc == 0)
			break;

		switch (cFunc) {
		case 0x01:
			if (!bHaveF01) {
				Read8BitRegisters(dev, (cAddr - 3), &F01ControlBase, 1);
				Read8BitRegisters(dev, (cAddr - 4), &F01CommandBase, 1);
			}
			break;

		case 0x12:
			if (!bHaveF12) {
				Read8BitRegisters(dev, (cAddr - 3), &F12ControlBase, 1);
				Read8BitRegisters(dev, (cAddr - 5), &F12QueryBase, 1);
				Read8BitRegisters(dev, (F12QueryBase), &F12Support, 1);

				if ((F12Support | 0x00) == 0) {
					TOUCH_I("Device not support F12.\n");
					break;
				}
				Read8BitRegisters(dev, (F12QueryBase + 5), Data, 2);
				mask = 0x01;
				for (j = 0; j < 8; j++) {
					if ((Data[1] & mask) == 1)
						offset++;
					Data[1] >>= 1;
				}
				Read8BitRegisters(dev, (F12ControlBase + offset),
						Data, 14);
				F12_2DRxCount = Data[12];
				F12_2DTxCount = Data[13];

				if (F12_2DRxCount >= TRX_MAX)
					F12_2DRxCount = TRX_MAX;
				if (F12_2DTxCount >= TRX_MAX)
					F12_2DTxCount = 16;

				offset = 0;
			}
			break;

		case 0x51:
			Read8BitRegisters(dev, (cAddr - 2), &F51DataBase, 1);
			Read8BitRegisters(dev, (cAddr - 3), &F51ControlBase, 1);
			Read8BitRegisters(dev, F51DataBase, &Data[0], 4);
			break;

		case 0x54:
			if (!bHaveF54) {
				Read8BitRegisters(dev, (cAddr - 2), &F54DataBase, 1);
				Read8BitRegisters(dev, (cAddr - 3), &F54ControlBase, 1);
				Read8BitRegisters(dev, (cAddr - 4), &F54CommandBase, 1);
				Read8BitRegisters(dev, (cAddr - 5), &F54QueryBase, 1);
				Read8BitRegisters(dev, F54QueryBase, &RxChannelCount, 1);
				Read8BitRegisters(dev, (F54QueryBase + 1), &TxChannelCount, 1);

				if (RxChannelCount >= TRX_MAX)
					RxChannelCount = TRX_MAX;
				if (TxChannelCount >= TRX_MAX)
					TxChannelCount = TRX_MAX;

				MaxArrayLength = (int)RxChannelCount * (int)TxChannelCount * 2;

				Read8BitRegisters(dev, F54QueryBase, Data, 60);
				TouchControllerFamily = Data[5];
				offset++;	/* Ctrl 00 */

				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01)
					offset++;	/* Ctrl 01 */
				offset += 2;	/* Ctrl 02 */
				bHavePixelTouchThresholdTuning =
					((Data[6] & 0x01) == 0x01);

				if (bHavePixelTouchThresholdTuning)
					offset++; /* Ctrl 03 */

				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01)
					offset += 3;	/* Ctrl 04/05/06 */

				if (TouchControllerFamily == 0x01) {
					F54Ctrl07Offset = offset;
					offset++;	/* Ctrl 07 */
					bHaveF54Ctrl07 = true;
				}

				/* Ctrl 08 */
				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01) {
					F54Ctrl08Offset = offset;
					offset += 2;
				}
				/* Ctrl 09 */
				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01)
					offset++;
				bHaveInterferenceMetric = ((Data[7] & 0x02) == 0x02);
				/* Ctrl 10 */
				if (bHaveInterferenceMetric)
					offset++;
				bHaveCtrl11 = ((Data[7] & 0x10) == 0x10);
				/* Ctrl 11 */
				if (bHaveCtrl11)
					offset += 2;
				bHaveRelaxationControl = ((Data[7] & 0x80) == 0x80);
				/* Ctrl 12/13 */
				if (bHaveRelaxationControl)
					offset += 2;
				bHaveSensorAssignment = ((Data[7] & 0x01) == 0x01);
				/* Ctrl 14 */
				if (bHaveSensorAssignment)
					offset++;
				/* Ctrl 15 */
				if (bHaveSensorAssignment)
					offset += RxChannelCount;
				/* Ctrl 16 */
				if (bHaveSensorAssignment)
					offset += TxChannelCount;
				bHaveSenseFrequencyControl =
					((Data[7] & 0x04) == 0x04);
				if (bHaveSenseFrequencyControl)
					NumOfSensingFreq = (Data[13] & 0x0F);
				/* Ctrl 17/18/19 */
				if (bHaveSenseFrequencyControl)
					offset += (3 * (int)NumOfSensingFreq);
				F54Ctrl20Offset = offset;
				offset++;	/* Ctrl 20 */
				if (bHaveSenseFrequencyControl)
					offset += 2;	/* Ctrl 21 */
				bHaveFirmwareNoiseMitigation = ((Data[7] & 0x08) == 0x08);
				if (bHaveFirmwareNoiseMitigation)
					offset++;	/* Ctrl 22 */
				if (bHaveFirmwareNoiseMitigation)
					offset += 2;	/* Ctrl 23 */
				if (bHaveFirmwareNoiseMitigation)
					offset += 2;	/* Ctrl 24 */
				if (bHaveFirmwareNoiseMitigation)
					offset++;	/* Ctrl 25 */
				if (bHaveFirmwareNoiseMitigation)
					offset++;	/* Ctrl 26 */
				bHaveIIRFilter = ((Data[9] & 0x02) == 0x02);
				if (bHaveIIRFilter)
					offset++;	/* Ctrl 27 */
				if (bHaveFirmwareNoiseMitigation)
					offset += 2;	/* Ctrl 28 */
				bHaveCmnRemoval = ((Data[9] & 0x04) == 0x04);
				bHaveCmnMaximum = ((Data[9] & 0x08) == 0x08);
				if (bHaveCmnRemoval)
					offset++;	/* Ctrl 29 */
				if (bHaveCmnMaximum)
					offset++;	/* Ctrl 30 */
				bHaveTouchHysteresis = ((Data[9] & 0x10) == 0x10);
				if (bHaveTouchHysteresis)
					offset++;	/* Ctrl 31 */
				bHaveEdgeCompensation = ((Data[9] & 0x20) == 0x20);
				if (bHaveEdgeCompensation)
					offset += 2;	/* Ctrl 32 */
				if (bHaveEdgeCompensation)
					offset += 2;	/* Ctrl 33 */
				if (bHaveEdgeCompensation)
					offset += 2;	/* Ctrl 34 */
				if (bHaveEdgeCompensation)
					offset += 2;	/* Ctrl 35 */
				CurveCompensationMode = (Data[8] & 0x03);
				if (CurveCompensationMode == 0x02) {
					offset += (int)RxChannelCount;
				} else if (CurveCompensationMode == 0x01) {
					offset += ((int)RxChannelCount > (int)TxChannelCount) ?
						(int)RxChannelCount : (int)TxChannelCount;
				}	/* Ctrl 36 */

				if (CurveCompensationMode == 0x02) {
					/* Ctrl 37 */
					offset += (int)TxChannelCount;
				}

				bHavePerFrequencyNoiseControl = ((Data[9] & 0x40) == 0x40);

				/* Ctrl 38/39/40 */
				if (bHavePerFrequencyNoiseControl)
					offset += (3 * (int)NumOfSensingFreq);

				bHaveSignalClarity = ((Data[10] & 0x04) == 0x04);

				if (bHaveSignalClarity) {
					F54Ctrl41Offset = offset;
					offset++;	/* Ctrl 41 */
					/* bHaveF54Ctrl41 = bSignalClarityOn */
					SignalClarityOn = true;
				} else
					SignalClarityOn = false;

				bHaveMultiMetricStateMachine = ((Data[10] & 0x02) == 0x02);
				bHaveVarianceMetric = ((Data[10] & 0x08) == 0x08);
				if (bHaveVarianceMetric)
					offset += 2;	/* Ctrl 42 */
				if (bHaveMultiMetricStateMachine)
					offset += 2;	/* Ctrl 43 */
				/* Ctrl 44/45/46/47/48/49/50/51/52/53/54 */
				if (bHaveMultiMetricStateMachine)
					offset += 11;

				bHave0DRelaxationControl = ((Data[10] & 0x10) == 0x10);
				bHave0DAcquisitionControl = ((Data[10] & 0x20) == 0x20);
				if (bHave0DRelaxationControl)
					offset += 2;	/*Ctrl 55/56 */
				if (bHave0DAcquisitionControl) {
					F54Ctrl57Offset = offset;
					offset++;	/* Ctrl 57 */
					bHaveF54Ctrl57 = true;
				}
				if (bHave0DAcquisitionControl)
					offset += 1;	/* Ctrl 58 */

				bHaveSlewMetric = ((Data[10] & 0x80) == 0x80);
				bHaveHBlank = ((Data[11] & 0x01) == 0x01);
				bHaveVBlank = ((Data[11] & 0x02) == 0x02);
				bHaveLongHBlank = ((Data[11] & 0x04) == 0x04);
				bHaveNoiseMitigation2 = ((Data[11] & 0x20) == 0x20);
				bHaveSlewOption = ((Data[12] & 0x02) == 0x02);

				if (bHaveHBlank)
					offset += 1;	/* Ctrl 59 */

				if (bHaveHBlank || bHaveVBlank || bHaveLongHBlank)
					offset += 3;	/* Ctrl 60/61/62 */

				if (bHaveSlewMetric || bHaveHBlank
						|| bHaveVBlank
						|| bHaveLongHBlank
						|| bHaveNoiseMitigation2
						|| bHaveSlewOption)
					offset += 1;	/* Ctrl 63 */

				if (bHaveHBlank)
					offset += 28;	/* Ctrl 64/65/66/67 */
				else if (bHaveVBlank || bHaveLongHBlank)
					offset += 4;	/* Ctrl 64/65/66/67 */

				if (bHaveHBlank || bHaveVBlank || bHaveLongHBlank)
					offset += 8;	/* Ctrl 68/69/70/71/72/73 */

				if (bHaveSlewMetric)
					offset += 2;	/* Ctrl 74 */

				bHaveEnhancedStretch = ((Data[9] & 0x80) == 0x80);
				/* Ctrl 75 */
				if (bHaveEnhancedStretch)
					offset += (int)NumOfSensingFreq;

				bHaveStartupFastRelaxation = ((Data[11] & 0x08) == 0x08);
				if (bHaveStartupFastRelaxation)
					offset += 1;	/* Ctrl 76 */

				bHaveESDControl = ((Data[11] & 0x10) == 0x10);
				if (bHaveESDControl)
					offset += 2;	/* Ctrl 77/78 */

				if (bHaveNoiseMitigation2)
					offset += 5;	/* Ctrl 79/80/81/82/83 */

				bHaveEnergyRatioRelaxation = ((Data[11] & 0x80) == 0x80);
				if (bHaveEnergyRatioRelaxation)
					offset += 2;	/* Ctrl 84/85 */

				bHaveF54Query13 = ((Data[12] & 0x08) == 0x08);
				if (bHaveSenseFrequencyControl) {
					q_offset = 13;
					NumOfSensingFreq = (Data[13] & 0x0F);
				} else
					q_offset = 12;

				if (bHaveF54Query13)
					q_offset++;

				bHaveCtrl86 = (bHaveF54Query13 && ((Data[13] & 0x01) == 0x01));
				bHaveCtrl87 = (bHaveF54Query13 && ((Data[13] & 0x02) == 0x02));
				bHaveCtrl88 = ((Data[12] & 0x40) == 0x40);

				if (bHaveCtrl86)
					offset += 1;	/* Ctrl 86 */
				if (bHaveCtrl87)
					offset += 1;	/* Ctrl 87 */
				if (bHaveCtrl88) {
					F54Ctrl88Offset = offset;
					offset++;	/* Ctrl 88 */
				}
				bHaveCtrl89 = ((Data[q_offset]
							& 0x20) == 0x20);
				bHaveCtrl89 = (bHaveCtrl89 | ((Data[q_offset]
							& 0x40) == 0x40));
				bHaveCtrl89 = (bHaveCtrl89 | ((Data[q_offset]
							& 0x80) == 0x80));
				if (bHaveCtrl89) {
					F54Ctrl89Offset	= offset;
					offset++;
				}
				bHaveF54Query15 = ((Data[12] & 0x80) == 0x80);
				if (bHaveF54Query15)
					q_offset++;	/* query_offset = 14 */
				bHaveCtrl90 = (bHaveF54Query15 &&
								((Data[q_offset] & 0x01) == 0x01));
				if (bHaveCtrl90)
					offset++;	/* offset = 1b */
				bHaveF54Query16 = ((Data[q_offset] & 0x8) == 0x8);
				bHaveF54Query20 = ((Data[q_offset] & 0x10) == 0x10);
				bHaveF54Query21 = ((Data[q_offset] & 0x20) == 0x20);
				bHaveF54Query22 = ((Data[q_offset] & 0x40) == 0x40);
				bHaveF54Query25 = ((Data[q_offset] & 0x80) == 0x80);
				if (bHaveF54Query16)
					q_offset++;	/* query_offset = 15 */
				bHaveF54Query17 = ((Data[q_offset] & 0x1) == 0x1);
				bHaveCtrl92 = ((Data[q_offset] & 0x4) == 0x4);
				bHaveCtrl93 = ((Data[q_offset] & 0x8) == 0x8);
				bHaveCtrl94 = ((Data[q_offset] & 0x10) == 0x10);
				bHaveF54Query18 = bHaveCtrl94;
				bHaveCtrl95 = ((Data[q_offset] & 0x20) == 0x20);
				bHaveF54Query19 = bHaveCtrl95;
				bHaveCtrl99 = ((Data[q_offset] & 0x40) == 0x40);
				bHaveCtrl100 = ((Data[q_offset] & 0x80) == 0x80);
				if (bHaveF54Query17)
					q_offset++;	/* query_offset = 16 */
				if (bHaveF54Query18)
					q_offset++;	/* query_offset = 17 */
				if (bHaveF54Query19)
					q_offset++;	/* query_offset = 18 */
				if (bHaveF54Query20)
					q_offset++;	/* query_offset = 19 */
				if (bHaveF54Query21)
					q_offset++;	/* query_offset = 20 */
				bHaveCtrl91 = ((Data[q_offset] & 0x4) == 0x4);
				bHaveCtrl96  = ((Data[q_offset] & 0x8) == 0x8);
				bHaveCtrl97  = ((Data[q_offset] & 0x10) == 0x10);
				bHaveCtrl98  = ((Data[q_offset] & 0x20) == 0x20);
				bHaveF54Query24  = ((Data[q_offset] & 0x80) == 0x80);
				if (bHaveF54Query22)
					q_offset++;	/* query_offset = 21 */
				bHaveCtrl101 = ((Data[q_offset] & 0x2) == 0x2);
				bHaveF54Query23 = ((Data[q_offset] & 0x8) == 0x8);
				bHaveF54Query26 = ((Data[q_offset] & 0x10) == 0x10);
				bHaveF54Ctrl103 = ((Data[q_offset] & 0x10) == 0x10);
				bHaveF54Ctrl104 = ((Data[q_offset] & 0x20) == 0x20);
				bHaveF54Ctrl105 = ((Data[q_offset] & 0x40) == 0x40);
				bHaveF54Query28 = ((Data[q_offset] & 0x80) == 0x80);
				if (bHaveF54Query23) {
					q_offset++;	/* query_offset = 22 */
					bHaveCtrl102 = ((Data[q_offset] & 0x01) == 0x01);
				} else
					bHaveCtrl102 = false;
				if (bHaveCtrl91) {
					F54Ctrl91Offset = offset;
					offset++;
				}
				if (bHaveCtrl92)
					offset++;
				if (bHaveCtrl93)
					offset++;
				if (bHaveCtrl94)
					offset++;
				if (bHaveCtrl95) {
					F54Ctrl95Offset = offset;
					offset++;
				}
				if (bHaveCtrl96) {
					F54Ctrl96Offset = offset;
					offset++;
				}
				if (bHaveCtrl97) {
					F54Ctrl97Offset = offset;
					offset++;
				}
				if (bHaveCtrl98) {
					F54Ctrl98Offset = offset;
					offset++;
				}
				if (bHaveCtrl99) {
					F54Ctrl99Offset = offset;
					offset++;
				}
				if (bHaveCtrl100)
					offset++;
				if (bHaveCtrl101)
					offset++;
				if (bHaveCtrl102) {
					uint8_t addr;

					bIncellDevice = true;
					F54Ctrl102Offset = offset;
					HasCtrl102Sub1 = (Data[q_offset] & 0x02);
					HasCtrl102Sub2 = (Data[q_offset] & 0x04);
					HasCtrl102Sub4 = (Data[q_offset] & 0x08);
					HasCtrl102Sub5 = (Data[q_offset] & 0x010);
					HasCtrl102Sub9 = (Data[q_offset] & 0x020);
					HasCtrl102Sub10 = (Data[q_offset] & 0x40);
					HasCtrl102Sub11 = (Data[q_offset] & 0x80);
					HasCtrl102Sub12 = false;
					offset = 0;
					addr = F54ControlBase + F54Ctrl102Offset;
					Read8BitRegisters(dev, addr, &Data[0], 27);
					tsvd_select = Data[0] & 0x03;
					tsvd = Data[1 + tsvd_select];
					offset = offset + 4;
					tshd = Data[offset];
					if (HasCtrl102Sub1) {
						offset = offset + 2;
						tsstb = Data[offset];
					}
					if (HasCtrl102Sub2) {
						tsfrq = Data[offset + 2];
						tsfst = Data[offset + 3];
						offset = offset + 3;
					}
					/* Ctrl102Sub3 */
					/* 0 = GPIO, 1 = TRX */
					exvcom_pin_type = (Data[offset + 1] & 0x01);
					exvcom1 = Data[offset + 2];
					offset = offset + 2;
					if (HasCtrl102Sub4) {
						exvcom_sel = (Data[offset + 1] & 0x03);
						exvcom2 = Data[offset + 2];
						offset = offset + 4;
					}
					if (HasCtrl102Sub5) {
						enable_guard = (Data[offset + 1] & 0x01);
						guard_ring = Data[offset + 2];
						offset = offset + 2;
					}
					/* Ctrl102Sub6, 7, 8 */
					offset = offset + 5;
					if (HasCtrl102Sub9)
						offset++;
					if (HasCtrl102Sub10) {
						exvcom_sel = Data[offset + 2];
						offset = offset + 2;
					}
					if (HasCtrl102Sub11)
						offset++;
					if (bHaveF54Query25)
						HasCtrl102Sub12 = (Data[q_offset + 1] & 0x02);
					if (HasCtrl102Sub12) {
						enable_verf = ((Data[offset + 1]) & 0x01);
						verf = (Data[offset + 2]);
					}
				}
				if (bHaveF54Query24)
					q_offset++;
				if (bHaveF54Query25)
					q_offset++;	/* Query 25 */
				bHaveF54Ctrl106 =
					((Data[q_offset] & 0x01) == 0x01);
				bHaveF54Ctrl107 =
					((Data[q_offset] & 0x04) == 0x04);
				bHaveF54Ctrl108 =
					((Data[q_offset] & 0x08) == 0x08);
				bHaveF54Ctrl109 =
					((Data[q_offset] & 0x10) == 0x10);
				bHaveF54Query27 =
					((Data[q_offset] & 0x80) == 0x80);

				if (bHaveF54Query26)
					q_offset++;

				if (bHaveF54Query27) {
					q_offset++;
					bHaveF54Ctrl110 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl111 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl112 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl113 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl114 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query29 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query28)
					q_offset++;
				if (bHaveF54Query29) {
					q_offset++;
					bHaveF54Ctrl115 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl116 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl117 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query30 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query30) {
					q_offset++;
					bHaveF54Ctrl118 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl119 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl120 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl121 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl122 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Query31 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl123 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl124 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query32 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query31)
					q_offset++;
				if (bHaveF54Query32) {
					q_offset++;
					bHaveF54Ctrl125 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl126 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl127 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Query33 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Query34 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query35 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query33) {
					q_offset++;
					bHaveF54Ctrl128 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl129 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl130 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl131 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl132 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl133 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl134 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query36 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query34)
					q_offset++;
				if (bHaveF54Query35) {
					q_offset++;
					bHaveF54Ctrl135 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl136 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl137 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl138 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl139 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl140 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query36) {
					q_offset++;
					bHaveF54Ctrl141 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl142 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Query37 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl143 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl144 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl145 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl146 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query38 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query37)
					q_offset++;
				if (bHaveF54Query38) {
					q_offset++;
					bHaveF54Ctrl147 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl148 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl149 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl151 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl152 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl153 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query39 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query39) {
					q_offset++;
					bHaveF54Ctrl154 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl155 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl156 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl160 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl157 = bHaveF54Ctrl158 =
						((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl159 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl161 = bHaveF54Ctrl162 =
						((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query40 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query40) {
					q_offset++;
					bHaveF54Ctrl169 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl163 = bHaveF54Query41 =
						((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl164 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl165 = bHaveF54Query42 =
						((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl166 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl167 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl168 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Query43 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query41)
					q_offset++;
				if (bHaveF54Query42)
					q_offset++;
				if (bHaveF54Query43) {
					q_offset++;
					bHaveF54Ctrl170 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl171 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl172 = bHaveF54Query44 = bHaveF54Query45 =
						((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl173 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl174 = ((Data[q_offset] & 0x10) == 0x20);
					bHaveF54Ctrl175 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query46 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query44)
					q_offset++;
				if (bHaveF54Query45)
					q_offset++;
				if (bHaveF54Query46) {
					q_offset++;
					bHaveF54Ctrl176 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl177 = bHaveF54Ctrl178 =
						((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl179 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl180 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl171 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query47 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query47) {
					q_offset++;
					bHaveF54Ctrl182 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl183 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Query48 = bHaveF54Ctrl184 =
						((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl185 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl186 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl187 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query49 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query48)
					q_offset++;
				if (bHaveF54Query49) {
					q_offset++;
					bHaveF54Ctrl188 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl189 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl190 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query50 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query50) {
					q_offset++;
					bHaveF54Ctrl191 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl192 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl193 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Query52 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl194 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl195 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query51 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query51) {
					q_offset++;
					bHaveF54Ctrl196 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl197 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl198 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Query54 = bHaveF54Query53 =
						((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl199 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query55 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query52)
					q_offset++;
				if (bHaveF54Query53)
					q_offset++;
				if (bHaveF54Query54)
					q_offset++;
				if (bHaveF54Query55) {
					q_offset++;
					bHaveF54Query56 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl200 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl202 = bHaveF54Ctrl201 =
						((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl203 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl204 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query57 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query56)
					q_offset++;
				if (bHaveF54Query57) {
					q_offset++;
					bHaveF54Ctrl205 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl206 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl207 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl208 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl209 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl210 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query58 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query58) {
					q_offset++;
					bHaveF54Query59 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Query60 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl211 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl212 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl213 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query61 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query59)
					q_offset++;
				if (bHaveF54Query60)
					q_offset++;
				if (bHaveF54Query61) {
					q_offset++;
					bHaveF54Ctrl214 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl215 = bHaveF54Query62 = bHaveF54Query63 =
						((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl216 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl217 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl218 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl219 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query64 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query62)
					q_offset++;
				if (bHaveF54Query63)
					q_offset++;
				if (bHaveF54Query64) {
					q_offset++;
					bHaveF54Ctrl220 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl221 = ((Data[q_offset] & 0x04) == 0x04);
					bHaveF54Ctrl222 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl224 = bHaveF54Ctrl226 = bHaveF54Ctrl227 =
						((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query65 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query65) {
					q_offset++;
					bHaveF54Ctrl225 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl229 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl230 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl231 = bHaveF54Query66 =
						((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Ctrl232 = ((Data[q_offset] & 0x40) == 0x40);
					bHaveF54Query67 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query66)
					q_offset++;
				if (bHaveF54Query67) {
					q_offset++;
					bHaveF54Ctrl233 = ((Data[q_offset] & 0x08) == 0x08);
					bHaveF54Ctrl234 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Ctrl235 = bHaveF54Ctrl236 =
						((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Query68 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query68) {
					q_offset++;
					bHaveF54Ctrl237 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl238 = ((Data[q_offset] & 0x02) == 0x02);
					bHaveF54Ctrl239 = ((Data[q_offset] & 0x10) == 0x10);
					bHaveF54Query69 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query69) {
					q_offset++;
					bHaveF54Ctrl240 = ((Data[q_offset] & 0x01) == 0x01);
					bHaveF54Ctrl241 = ((Data[q_offset] & 0x20) == 0x20);
					bHaveF54Query70 = ((Data[q_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query70) {
					q_offset++;
					bHaveF54Ctrl242 = ((Data[q_offset] & 0x04) == 0x04);
				}

				/* from Ctrl 103 */
				if (bHaveF54Ctrl103)
					offset++;
				if (bHaveF54Ctrl104)
					offset++;
				if (bHaveF54Ctrl105)
					offset++;
				if (bHaveF54Ctrl106)
					offset++;
				if (bHaveF54Ctrl107)
					offset++;
				if (bHaveF54Ctrl108)
					offset++;
				if (bHaveF54Ctrl109)
					offset++;
				if (bHaveF54Ctrl110)
					offset++;
				if (bHaveF54Ctrl111)
					offset++;
				if (bHaveF54Ctrl112)
					offset++;
				if (bHaveF54Ctrl113)
					offset++;
				if (bHaveF54Ctrl114)
					offset++;
				if (bHaveF54Ctrl115)
					offset++;
				if (bHaveF54Ctrl116)
					offset++;
				if (bHaveF54Ctrl117)
					offset++;
				if (bHaveF54Ctrl118)
					offset++;
				if (bHaveF54Ctrl119)
					offset++;
				if (bHaveF54Ctrl120)
					offset++;
				if (bHaveF54Ctrl121)
					offset++;
				if (bHaveF54Ctrl122)
					offset++;
				if (bHaveF54Ctrl123)
					offset++;
				if (bHaveF54Ctrl124)
					offset++;
				if (bHaveF54Ctrl125)
					offset++;
				if (bHaveF54Ctrl126)
					offset++;
				if (bHaveF54Ctrl127)
					offset++;
				if (bHaveF54Ctrl128)
					offset++;
				if (bHaveF54Ctrl129)
					offset++;
				if (bHaveF54Ctrl130)
					offset++;
				if (bHaveF54Ctrl131)
					offset++;
				if (bHaveF54Ctrl132) {
					F54Ctrl132Offset = offset;
					offset++;
				}
				if (bHaveF54Ctrl133)
					offset++;
				if (bHaveF54Ctrl134)
					offset++;
				if (bHaveF54Ctrl135)
					offset++;
				if (bHaveF54Ctrl136)
					offset++;
				if (bHaveF54Ctrl137)
					offset++;
				if (bHaveF54Ctrl138)
					offset++;
				if (bHaveF54Ctrl139)
					offset++;
				if (bHaveF54Ctrl140)
					offset++;
				if (bHaveF54Ctrl141)
					offset++;
				if (bHaveF54Ctrl142)
					offset++;
				if (bHaveF54Ctrl143)
					offset++;
				if (bHaveF54Ctrl144)
					offset++;
				if (bHaveF54Ctrl145)
					offset++;
				if (bHaveF54Ctrl146) {
					F54Ctrl146Offset = offset;
					offset++;
				}
				if (bHaveF54Ctrl147) {
					F54Ctrl147Offset = offset;
					offset++;
				}
				if (bHaveF54Ctrl148)
					offset++;
				if (bHaveF54Ctrl149) {
					F54Ctrl149Offset = offset;
					offset++;
				}
				if (bHaveF54Ctrl150)
					offset++;
				if (bHaveF54Ctrl151)
					offset++;
				if (bHaveF54Ctrl112)
					offset++;
				if (bHaveF54Ctrl153)
					offset++;
				if (bHaveF54Ctrl154)
					offset++;
				if (bHaveF54Ctrl155)
					offset++;
				if (bHaveF54Ctrl156)
					offset++;
				if (bHaveF54Ctrl157)
					offset++;
				if (bHaveF54Ctrl158)
					offset++;
				if (bHaveF54Ctrl159)
					offset++;
				if (bHaveF54Ctrl150)
					offset++;
				if (bHaveF54Ctrl161)
					offset++;
				if (bHaveF54Ctrl162)
					offset++;
				if (bHaveF54Ctrl163)
					offset++;
				if (bHaveF54Ctrl164)
					offset++;
				if (bHaveF54Ctrl165)
					offset++;
				if (bHaveF54Ctrl166)
					offset++;
				if (bHaveF54Ctrl167)
					offset++;
				if (bHaveF54Ctrl168)
					offset++;
				if (bHaveF54Ctrl169)
					offset++;
				if (bHaveF54Ctrl170)
					offset++;
				if (bHaveF54Ctrl171)
					offset++;
				if (bHaveF54Ctrl172)
					offset++;
				if (bHaveF54Ctrl173)
					offset++;
				if (bHaveF54Ctrl174)
					offset++;
				if (bHaveF54Ctrl175)
					offset++;
				if (bHaveF54Ctrl176)
					offset++;
				if (bHaveF54Ctrl177)
					offset++;
				if (bHaveF54Ctrl178)
					offset++;
				if (bHaveF54Ctrl179)
					offset++;
				if (bHaveF54Ctrl180)
					offset++;
				if (bHaveF54Ctrl181)
					offset++;
				if (bHaveF54Ctrl182) {
					F54Ctrl182Offset = offset;
					offset++;
				}
				if (bHaveF54Ctrl183)
					offset++;
				if (bHaveF54Ctrl184)
					offset++;
				if (bHaveF54Ctrl185)
					offset++;
				if (bHaveF54Ctrl186) {
					F54Ctrl186Offset = offset;
					offset++;
				}
				if (bHaveF54Ctrl187)
					offset++;
				if (bHaveF54Ctrl188) {
					F54Ctrl188Offset = offset;
					offset++;
				}
				if (bHaveF54Ctrl189) {
					F54Ctrl189Offset = offset;
					offset++;
				}
				if (bHaveF54Ctrl190)
					offset++;
				if (bHaveF54Ctrl191)
					offset++;
				if (bHaveF54Ctrl192)
					offset++;
				if (bHaveF54Ctrl193)
					offset++;
				if (bHaveF54Ctrl194)
					offset++;
				if (bHaveF54Ctrl195)
					offset++;
				if (bHaveF54Ctrl196)
					offset++;
				if (bHaveF54Ctrl197)
					offset++;
				if (bHaveF54Ctrl198)
					offset++;
				if (bHaveF54Ctrl199)
					offset++;
				if (bHaveF54Ctrl200)
					offset++;
				if (bHaveF54Ctrl201)
					offset++;
				if (bHaveF54Ctrl202)
					offset++;
				if (bHaveF54Ctrl203)
					offset++;
				if (bHaveF54Ctrl204)
					offset++;
				if (bHaveF54Ctrl205)
					offset++;
				if (bHaveF54Ctrl206)
					offset++;
				if (bHaveF54Ctrl207)
					offset++;
				if (bHaveF54Ctrl208)
					offset++;
				if (bHaveF54Ctrl209)
					offset++;
				if (bHaveF54Ctrl210)
					offset++;
				if (bHaveF54Ctrl211)
					offset++;
				if (bHaveF54Ctrl212)
					offset++;
				if (bHaveF54Ctrl213)
					offset++;
				if (bHaveF54Ctrl214)
					offset++;
				if (bHaveF54Ctrl215) {
					F54Ctrl215Offset = offset;
					offset++;
				}
				if (bHaveF54Ctrl216)
					offset++;
				if (bHaveF54Ctrl217)
					offset++;
				if (bHaveF54Ctrl218)
					offset++;
				if (bHaveF54Ctrl219)
					offset++;
				if (bHaveF54Ctrl220)
					offset++;
				if (bHaveF54Ctrl221)
					offset++;
				if (bHaveF54Ctrl222)
					offset++;
				if (bHaveF54Ctrl223)
					offset++;
				if (bHaveF54Ctrl224)
					offset++;
				if (bHaveF54Ctrl225) {
					F54Ctrl225Offset = offset;
					offset++;
				}
				if (bHaveF54Ctrl226)
					offset++;
				if (bHaveF54Ctrl227)
					offset++;
				if (bHaveF54Ctrl228)
					offset++;
				if (bHaveF54Ctrl229)
					offset++;
				if (bHaveF54Ctrl230)
					offset++;
				if (bHaveF54Ctrl231)
					offset++;
				if (bHaveF54Ctrl232)
					offset++;
				if (bHaveF54Ctrl233)
					offset++;
				if (bHaveF54Ctrl234)
					offset++;
				if (bHaveF54Ctrl235)
					offset++;
				if (bHaveF54Ctrl236)
					offset++;
				if (bHaveF54Ctrl237)
					offset++;
				if (bHaveF54Ctrl238)
					offset++;
				if (bHaveF54Ctrl239)
					offset++;
				if (bHaveF54Ctrl240)
					offset++;
				if (bHaveF54Ctrl241)
					offset++;
				if (bHaveF54Ctrl242)
					offset++;
				if (bHaveF54Ctrl243)
					offset++;
				if (bHaveF54Ctrl244)
					offset++;
				if (bHaveF54Ctrl245)
					offset++;
				if (bHaveF54Ctrl246) {
					F54Ctrl246Offset = offset;
					offset++;
				}
			}
			break;

		case 0x55:
			if (!bHaveF55) {
				Read8BitRegisters(dev, (cAddr - 3), &F55ControlBase, 1);
				Read8BitRegisters(dev, (cAddr - 5), &F55QueryBase, 1);

				Read8BitRegisters(dev, F55QueryBase, &RxChannelCount, 1);
				Read8BitRegisters(dev, (F55QueryBase + 1), &TxChannelCount, 1);

				rxCount = 0;
				txCount = 0;

				/* Read Sensor Mapping */
				Read8BitRegisters(dev, (F55ControlBase + 1), Data,
						(int)RxChannelCount);

				for (i = 0; i < (int)RxChannelCount; i++) {
					if (Data[i] != 0xFF) {
						rxCount++;
						TRxPhysical[i] = Data[i];
						RxPhysical[i] = Data[i];
					} else
						break;
				}

				if (bHaveF54Ctrl189) {
					Read8BitRegisters(dev, (F55ControlBase + F54Ctrl189Offset),
							Data, (int)TxChannelCount + 2);
					for (i = 0; i < (int)TxChannelCount; i++)
						Data[i] = Data[i + 2];

				} else {
					Read8BitRegisters(dev, (F55ControlBase + 2),
							Data, (int)TxChannelCount);
				}

				for (i = 0; i < (int)TxChannelCount; i++) {
					if (Data[i] != 0xFF) {
						TRxPhysical[rxCount + i] = Data[i];
						TxPhysical[i] = Data[i];
						txCount++;
					} else
						break;
				}

				for (i = (rxCount + txCount); i < TRX_MAPPING_MAX; i++)
					TRxPhysical[i] = 0xFF;

				for (i = rxCount; i < TRX_MAPPING_MAX; i++)
					RxPhysical[i] = 0xFF;

				for (i = txCount; i < TRX_MAPPING_MAX; i++)
					TxPhysical[i] = 0xFF;

				RxChannelCount = rxCount;
				TxChannelCount = txCount;

				if (RxChannelCount >= TRX_MAX)
					RxChannelCount = TRX_MAX;
				if (TxChannelCount >= TRX_MAX)
					TxChannelCount = TRX_MAX;

				for (i = 0; i < TRX_MAPPING_MAX; i++) {
					if (TRxPhysical[i] == 0xFF)
						break;
					byte = TRxPhysical[i] / 8;
					bit = TRxPhysical[i] % 8;
					TRxPhysical_bit[byte] = (TRxPhysical_bit[byte] | (0x01 << bit));
				}
				MaxArrayLength = (int)RxChannelCount * (int)TxChannelCount * 2;
				if (((int)TxChannelCount - F12_2DTxCount == 0)
						&& ButtonCount > 0) {
					ButtonShared = true;
				}
			}
			break;
		default:	/* Any other function */
			break;
		}
		cAddr -= 6;
	} while (true);
}

/*
 * The following function illustrates the steps in getting
 * a full raw image report (report #20) by Function $54.
 */
static int ImageTest(struct device *dev, u16 input, int mode, char *buf)
{
	uint8_t data = 0;
	u16 report_type = 0;
	int ret = 0;

	TOUCH_TRACE();

	/* Assign report type for Full Raw Image */
	ret = EnterActiveMode(dev);
	if (ret < 0) {
		TOUCH_E("%s, EnterActiveMode failed\n", __func__);
		goto error;
	}

	data = input;	/* Raw Capacitance mode */
	ret = prepareTest(dev, data);
	if (ret < 0) {
		TOUCH_E("failed to write F54DataBase for report type(%d) - ret:%d\n", data, ret);
		goto error;
	}

	report_type = input;
	ret = ReadReport(dev, report_type, mode, buf);

error:
	return ret;
}

static int DeltaTest(struct device *dev, int mode, char *buf)
{
	uint8_t data = 0;
	u16 report_type = 0;
	int ret = 0;

	TOUCH_TRACE();

	/* Assign report type for Full Raw Image */
	data = eRT_Normalized16BitImageReport;	/* Delta mode */
	ret = prepareTest(dev, data);
	if (ret < 0) {
		TOUCH_E("failed to write F54DataBase for report type(%d) - ret:%d\n", data, ret);
		goto error;
	}

	report_type = eRT_Normalized16BitImageReport;
	ret = ReadReport(dev, report_type, mode, buf);

error:
	return ret;
}

static int NoiseTest(struct device *dev, int mode, char *buf)
{
	int ret = 0;
	int count = 0;

	TOUCH_TRACE();

	memset(noise_min, 0x00, sizeof(short) * (TRX_MAX) * (TRX_MAX));
	memset(noise_max, 0x00, sizeof(short) * (TRX_MAX) * (TRX_MAX));

	for (count = 0; count < NOISE_TEST_COUNT; count++) {
		ret = prepareTest(dev, eRT_Normalized16BitImageReport);
		if (ret < 0) {
			TOUCH_E("failed to write F54DataBase for report type(%d) - ret:%d\n", eRT_Normalized16BitImageReport, ret);
			goto error;
		}

		ret = ReadReport(dev, eRT_Normalized16BitImageReport, mode, buf);
		if (ret < 0) {
			TOUCH_E("%s, ReadReport Failed\n", __func__);
			goto error;
		}
	}

	ret = CompareNoiseReport(dev);

error:
	Reset(dev);

	return ret;
}

/* report type 63 */
static int HybridAbsRaw(struct device *dev, int mode, char *buf)
{
	uint8_t data = 0;
	u16 report_type = 0;
	int ret = 0;

	TOUCH_TRACE();

	ret = EnterActiveMode(dev);
	if (ret < 0) {
		TOUCH_E("%s, EnterActiveMode failed\n", __func__);
		goto error;
	}

	/* Assign report type for Abs Sensing Raw Capacitance report */
	data = eRT_HybirdRawCap;	/* Hybrid Abs Raw mode */
	ret = prepareTest(dev, data);
	if (ret < 0) {
		TOUCH_E("failed to write F54DataBase for report type(%d) - ret:%d\n", data, ret);
		goto error;
	}

	report_type = eRT_HybirdRawCap;
	ret = ReadReport(dev, report_type, mode, buf);

error:
	return ret;
}

static void GetMaxRx(short value[TRX_MAX][TRX_MAX], short max[TRX_MAX], short logical_pin)
{
	int i, j = 0;

	for (j = 0; j < RxChannelCount; j++) {
		if (j == logical_pin)
			continue;

		for (i = 0; i < TxChannelCount; i++) {
			if (max[j] < value[i][j])
				max[j] = value[i][j];
		}
	}
}

static int GetMinRx(short value[TRX_MAX][TRX_MAX], short logical_pin)
{
	int i = 0;
	int min = 9999;

	for (i = 0; i < TxChannelCount; i++) {
		if (min > value[i][logical_pin])
			min = value[i][logical_pin];
	}

	return min;
}

static void getExtShortDelta(struct device *dev, int trx,  uint8_t logical_pin)
{
	int i, j = 0;

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"Logical Rx[%d] = Extend pin[%d]\n", logical_pin, trx);

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
			"Tx = %d, Rx = %d\n", (int)TxChannelCount, (int)RxChannelCount);

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf),
			"[Extend TRx Short Test (%d, %d)]\n    ", logical_pin, trx);

	for (i = 0; i < (int)RxChannelCount; i++)
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "  [%2d]", i);

	for (i = 0; i < (int)TxChannelCount; i++) {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n[%2d] ", i);
		for (j = 0; j < (int)RxChannelCount; j++) {
			delta[i][j] = abs(baseline_image0[i][j] - baseline_image1[i][j]);
			f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"%5d ", delta[i][j]);
		}
	}

	f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len, "\n\n");
}

static int ExtendedTRXShortRT100Test(struct device *dev, int mode, char *buf)
{
	bool ext_short_ret = true;
	bool short_ret = true;
	bool result = true;
	int ret = 0;
	int i, j, ii, jj = 0;
	short minrx = 0;
	short maxRX[TRX_MAX] = {0,};
	uint8_t rxIndex = 0;
	uint8_t data = 0;
	uint8_t buffer[TRX_MAPPING_MAX] = {0,};
	uint8_t logical_pin = 0;

	TOUCH_TRACE();

	memset(baseline_image0, 0x00, sizeof(short) * (TRX_MAX) * (TRX_MAX));
	memset(baseline_image1, 0x00, sizeof(short) * (TRX_MAX) * (TRX_MAX));
	memset(delta, 0x00, sizeof(short) * (TRX_MAX) * (TRX_MAX));
	memset(buffer, 0x00, TRX_MAPPING_MAX);

	/* read report type 26 */
	data = eRT_TRexShort;
	ret = prepareTest(dev, data);
	short_ret = ReadReport(dev, data, mode, buf);

	ret += EnterActiveMode(dev);
	if (ret < 0) {
		TOUCH_E("%s, TRex Short Test failed\n", __func__);
		goto error;
	}

	/* Set NoScan bit. data will not refresh automatically. */
	data = 0;
	ret = Read8BitRegisters(dev, F54ControlBase, &data, 1);
	data = (data | 0x02);
	ret = Write8BitRegisters(dev, F54ControlBase, &data, 1);
	if (ret < 0) {
		TOUCH_E("failed to write F54DataBase - ret:%d\n", ret);
		goto error;
	}

	/* set all local CBC Rxs to 0 */
	ret = Write8BitRegisters(dev, F54ControlBase + F54Ctrl96Offset, &buffer[0], RxChannelCount);
	if (ret < 0) {
		TOUCH_E("F54 write failed! - ret:%d\n", ret);
		goto error;
	}

	ret = s3706_force_update(dev);
	if (ret < 0) {
		TOUCH_E("forceUpdate failed! - ret:%d\n", ret);
		goto error;
	}

	ret = EnterActiveMode(dev);
	if (ret < 0) {
		TOUCH_E("%s, EnterActiveMode failed\n", __func__);
		goto error;
	}

	f54len = 0;
	memset(f54buf, 0, BUF_SIZE);

	/* Assign report type for RawImageRT100 Test*/
	data = eRT_RawImageRT100;	/* RawImageRT100 */
	ret = prepareTest(dev, data);
	if (ret < 0) {
		TOUCH_E("failed to write F54DataBase for report type(%d) - ret:%d\n", data, ret);
		goto error;
	}
	ReadReport(dev, data, mode, buf);

	for (i = 0; i < (int)TxChannelCount; i++) {
		for (j = 0; j < (int)RxChannelCount; j++)
			baseline_image0[i][j] = Image1[i][j];
	}

	for (i = 0; i < 4; i++)	{
		if (ExtendRT26_pin[i] == 0xFF)
			continue;

		memset(maxRX, 0x00, TRX_MAX);
		memset(delta, 0x00, sizeof(short) * (TRX_MAX) * (TRX_MAX));
		logical_pin = GetLogicalPin(ExtendRT26_pin[i]);
		buffer[logical_pin] = 0x0f; /* EXTENDED_TRX_SHORT_CBC */
		ret = Write8BitRegisters(dev, F54ControlBase + F54Ctrl96Offset,
						&buffer[0], RxChannelCount);
		if (ret < 0) {
			TOUCH_E("F54 write failed! - ret:%d\n", ret);
			goto error;
		}
		buffer[logical_pin] = 0;

		ret = s3706_force_update(dev);
		if (ret < 0) {
			TOUCH_E("forceUpdate failed! - ret:%d\n", ret);
			goto error;
		}

		/* read report type 100 */
		touch_msleep(200);
		ret = EnterActiveMode(dev);
		if (ret < 0) {
			TOUCH_E("%s, EnterActiveMode failed\n", __func__);
			goto error;
		}

		/* Assign report type for RawImageRT100 Test*/
		data = eRT_RawImageRT100;	/* RawImageRT100 */
		ret = prepareTest(dev, data);
		if (ret < 0) {
			TOUCH_E("failed to write F54DataBase for report type(%d) - ret:%d\n", data, ret);
			goto error;
		}
		ReadReport(dev, data, mode, buf);

		for (ii = 0; ii < (int)TxChannelCount; ii++) {
			for (jj = 0; jj < (int)RxChannelCount; jj++)
				baseline_image1[ii][jj] = Image1[ii][jj];
		}

		getExtShortDelta(dev, i, logical_pin);
		GetMaxRx(delta, maxRX, logical_pin);

		print_sd_log(f54buf);
		write_file(dev, f54buf, TIME_INFO_SKIP);

		f54len = 0;
		memset(f54buf, 0, BUF_SIZE);

		/* Check data: TREX w/o CBC raised changes >= 200 or TREXn (TREX with CBC raised) changes < 2000
		 * Flag TRX0 as 1 as well if any other RX changes are >=200
		 */
		for (rxIndex = 0; rxIndex < RxChannelCount; rxIndex++) {
			minrx = 0;
			if (rxIndex == logical_pin) {
				minrx = GetMinRx(delta, logical_pin);
				if (minrx < ext_trx_short_limit[1]) {
					TOUCH_E("Fail, minRX[%d] = %d when test pin %d (RX Logical pin [%d])\n",
							rxIndex, minrx, ExtendRT26_pin[i], logical_pin);
					setErr_Array(RxPhysical[rxIndex], true);
					ext_short_ret = false;
				}
			} else {
				if (maxRX[rxIndex] >= ext_trx_short_limit[0]) {
					TOUCH_E("Fail, maxRX[%d] = %d when test pin %d (RX Logical pin [%d])\n",
							rxIndex, maxRX[rxIndex], ExtendRT26_pin[i], logical_pin);
					setErr_Array(RxPhysical[rxIndex], true);
					setErr_Array(ExtendRT26_pin[i], true);
					ext_short_ret = false;
				}
			}

		}
		if (ext_short_ret)
			setErr_Array(ExtendRT26_pin[i], false);
	}
	for (i = 0 ; i < TRX_BITMAP_LENGTH; i++) {
		for (j = 0; j < 8; j++) {
			if (err_array[i] & (0x01 << j)) {
				TOUCH_E("Fail, TRX_physical[%d]\n", ((i * 8) + j));
				ext_short_ret = false;
			}
		}
	}

	if (ext_short_ret) {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"Extend TRx Short Test passed.\n\n");
	} else {
		f54len += touch_snprintf(f54buf + f54len, sizeof(f54buf) - f54len,
					"Extend TRx Short Test failed.\n\n");
	}
	result = short_ret & ext_short_ret;

	print_sd_log(f54buf);
	write_file(dev, f54buf, TIME_INFO_SKIP);

	Reset(dev);

	return (result) ? 1 : 0;

error:
	return -EAGAIN;
}

/* This test is to retrieve the high resistance report, report type #4. */
static int HighResistanceTest(struct device *dev, int mode, char *buf)
{
	uint8_t data = 0;
	u16 report_type = 0;
	int ret = 0;

	TOUCH_TRACE();

	/* Assign report type for High Resistance report*/
	data = eRT_HighResistance;	/* High Resistance mode */
	ret = prepareTest(dev, data);
	if (ret < 0) {
		TOUCH_E("failed to write F54DataBase for report type(%d) - ret:%d\n", data, ret);
		return -EAGAIN;
	}

	report_type = eRT_HighResistance;
	ret = ReadReport(dev, report_type, mode, buf);

	return ret;
}

/* report type #133. */
static int TransRxShortTest(struct device *dev, int mode, char *buf)
{
	uint8_t data = 0;
	u16 report_type = 0;
	int ret = 0;

	TOUCH_TRACE();

	/* Assign report type for High Resistance report*/
	data = eRT_TransRxShortUsingCBCVariation;	/* Trans Rx Short mode */
	ret = prepareTest(dev, data);
	if (ret < 0) {
		TOUCH_E("failed to write F54DataBase for report type(%d) - ret:%d\n", data, ret);
		return -EAGAIN;
	}

	report_type = eRT_TransRxShortUsingCBCVariation;
	ret = ReadReport(dev, report_type, mode, buf);

	return ret;
}

void SCAN_PDT(struct device *dev)
{
	int i = 0;

	TOUCH_TRACE();

	for (i = 0; i < scanMaxPageCount; i++) {
		if (switchPage(dev, i))
			RunQueries(dev);
	}
}

/* Main entry point for the application */
int F54Test(struct device *dev, u16 input, int mode, char *buf)
{
	int ret = 0;
	int retry_cnt1 = 0;
	int retry_cnt2 = 0;
	int i = 0;

	TOUCH_TRACE();

retry:
	ret = switchPage(dev, COMMON);
	if (ret == -EAGAIN && ++retry_cnt1 <= 3) {
		TOUCH_I("retry switchPage, count = %d\n", retry_cnt1);
		goto retry;
	} else if (ret == 0) {
		return ret;
	}

	f54len = 0;
	memset(f54buf, 0, BUF_SIZE);

	switch (input) {
	case eRT_RawImageRT100:
	case eRT_FullRawCapacitance:
		ret = ImageTest(dev, input, mode, buf);
		break;
	case eRT_TRexShort:
		ret = prepareTest(dev, input);
		ret = ReadReport(dev, input, mode, buf);
		break;
	case eRT_ExtendedTRexShortRT100:
		ret = ExtendedTRXShortRT100Test(dev, mode, buf);
		break;
	case eRT_HighResistance:
		ret = HighResistanceTest(dev, mode, buf);
		break;
	case eRT_HybirdRawCap:
		ret = HybridAbsRaw(dev, mode, buf);
		break;
	case eRT_Normalized16BitImageReport:
		if (mode == 1)
			ret = DeltaTest(dev, mode, buf);
		else
			ret = NoiseTest(dev, mode, buf);
		break;
	case eRT_TransRxShortUsingCBCVariation:
		ret = TransRxShortTest(dev, mode, buf);
		break;
	default:
		return -EINVAL;
	}

	for (i = 0 ; i < TRX_BITMAP_LENGTH; i++)
		err_array[i] = 0;

	if (switchPage(dev, DEFAULT) != true) {
		TOUCH_I("switchPage failed\n");
		Reset(dev);
	}

	if (ret == -EAGAIN && ++retry_cnt2 <= 3) {
		TOUCH_I("retry Test, count = %d\n", retry_cnt2);
		goto retry;
	}

	return ret;
}
