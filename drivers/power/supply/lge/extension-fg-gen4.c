/*
 * CAUTION! :
 * 	This file will be included at the end of "qpnp-fg-gen3.c".
 * 	So "qpnp-fg-gen3.c" should be touched before you start to build.
 * 	If not, your work will not be applied to the built image
 * 	because the build system doesn't care the update time of this file.
 */

#include <linux/thermal.h>
#include <linux/kernel.h>
#include <linux/iio/consumer.h>
#include "veneer-primitives.h"

#define LGE_FG_INITVAL -1

struct _fake {
	int temperature;
	int capacity;
	int uvoltage;
};

struct _fginfo {
/* Capacity */
	int capacity_rescaled;
	int capacity_monotonic;
	int capacity_chargecnt;
	int capacity_learned;
/* v/i ADCs */
	int battery_inow;
	int battery_vnow;
	int battery_vpredict;
	int battery_ocv;
	int input_vusb;
	int input_iusb;
	int input_vwlc;
	int input_iwlc;
	int input_aicl;
/* Temperature */
	int temp_compensated;
	int temp_thermister;
	int temp_vts;
/* Impedance */
	int impedance_esr;
	int impedance_rslow;
/* Misc */
	int misc_cycle;
/* SoCs */
	int misc_battsoc;
	int misc_ccsocsw;
	int misc_ccsoc;

};

#define TCOMP_TABLE_MAX 3
#define TCOMP_COUNT 25
struct tcomp_param {
	bool load_done;
	int load_max;
	bool icoeff_load_done;
	struct tcomp_entry {
		int temp_cell;
		int temp_bias;
	} table[TCOMP_TABLE_MAX][TCOMP_COUNT];
	int icoeff;

	bool rise_cut;
	int rise_cut_trig;
	bool fall_cut;
	int fall_cut_trig;

	bool qnovo_charging;
	bool logging;
};

struct _rescale {
	bool lge_monotonic;
	int	criteria;	// 0 ~ 255
	int	rawsoc;		// 0 ~ 255
	int	result;		// 0 ~ 100
};

enum tcomp_chg_type {
	TCOMP_CHG_NONE = 0,
	TCOMP_CHG_USB,
	TCOMP_CHG_WLC_LCDOFF,
	TCOMP_CHG_WLC_LCDON
};

/* Gloval variable for extension-fg-gen4 */
static struct _fake fake = {
	.temperature = LGE_FG_INITVAL,
	.capacity = LGE_FG_INITVAL,
	.uvoltage = LGE_FG_INITVAL,
};

static struct _fginfo fginfo = {
/* Capacity  */ LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL,
/* v/i ADCs  */ LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL,
                LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL,
                LGE_FG_INITVAL,
/* Temp      */ LGE_FG_INITVAL, LGE_FG_INITVAL, -1000,
/* impedance */ LGE_FG_INITVAL, LGE_FG_INITVAL,
/* Misc      */ LGE_FG_INITVAL,
/* SoCs      */ LGE_FG_INITVAL, LGE_FG_INITVAL, LGE_FG_INITVAL,
};

static struct tcomp_param tcomp = {
	.load_done = false,
	.icoeff_load_done = false,
	.icoeff = 0,

	.rise_cut = false,
	.rise_cut_trig = -999,
	.fall_cut = false,
	.fall_cut_trig = -999,

	.qnovo_charging = false,
	.logging = false,
};

/* For SoC rescaling, .rawsoc(0~255) is updated ONLY ON
 * 'fg_delta_msoc_irq_handler' and it is rescaled to 0~100
 */
static struct _rescale rescale = {
	.lge_monotonic = false,
	.criteria = 247,
	.rawsoc = LGE_FG_INITVAL,
	.result = LGE_FG_INITVAL,
};

/* calculate_battery_temperature
 *     bias  : 1st compensation by predefined diffs
 *     icomp : 2nd compensation by (i^2 * k)
 */
#define FAKE_TEMP_THRESHOLD -300
static bool wa_skip_batt_temp_on_bootup_running = false;
static void wa_skip_batt_temp_on_bootup_trigger(int batt_temp) {
	if (batt_temp <= FAKE_TEMP_THRESHOLD) {
		pr_info("Enable skip battery temp(%d) \n", batt_temp);
		wa_skip_batt_temp_on_bootup_running = true;
	}
}

static bool wa_skip_batt_temp_on_bootup_check(int batt_temp, bool irq) {
	if (wa_skip_batt_temp_on_bootup_running && (irq || batt_temp > FAKE_TEMP_THRESHOLD)) {
		pr_info("Clear skip battery temp. irq: (%d), temp (%d)\n", irq, batt_temp);
		wa_skip_batt_temp_on_bootup_running = false;
	}

	return wa_skip_batt_temp_on_bootup_running;
}

static irqreturn_t extension_delta_batt_temp_irq_handler(int irq, void *data) {
	wa_skip_batt_temp_on_bootup_check(0, true);
	return fg_delta_batt_temp_irq_handler(irq, data);
}

static int get_charging_type(struct fg_dev *fg)
{
	union power_supply_propval val = { 0, };
	char slimit[20] = "";

	if (fg->batt_psy) {
		if (!power_supply_get_property(fg->batt_psy,
			POWER_SUPPLY_PROP_STATUS_RAW, &val))
			if (val.intval != POWER_SUPPLY_STATUS_CHARGING)
				return TCOMP_CHG_NONE;
	}

	if (fg->usb_psy) {
		if (!power_supply_get_property(fg->usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &val))
			if (val.intval)
				return TCOMP_CHG_USB;
	}

	if (fg->dc_psy) {
		if (!power_supply_get_property(fg->dc_psy,
			POWER_SUPPLY_PROP_PRESENT, &val)) {
			if (val.intval) {
				if (unified_nodes_show("status_lcd", slimit)) {
					if (slimit[0] == '0')
						return TCOMP_CHG_WLC_LCDOFF;
					return TCOMP_CHG_WLC_LCDON;
				} else
					return TCOMP_CHG_WLC_LCDOFF;
			}
		}
	}

	return TCOMP_CHG_NONE;
}

static int get_batt_temp_current(struct fg_dev *fg)
{
	static int ichg = 0;
	bool is_cc = false;
	bool is_fast_charging = false;
	union power_supply_propval val = { 0, };

	if (!fg || !fg->batt_psy || !fg->usb_psy)
		return 0;

	if (!power_supply_get_property(fg->batt_psy,
			POWER_SUPPLY_PROP_STATUS_RAW, &val)
				&& val.intval == POWER_SUPPLY_STATUS_CHARGING) {
		if (!power_supply_get_property(fg->usb_psy,
				POWER_SUPPLY_PROP_REAL_TYPE, &val)) {

			if (val.intval == POWER_SUPPLY_TYPE_USB_HVDCP ||
				val.intval == POWER_SUPPLY_TYPE_USB_HVDCP_3 ||
				val.intval == POWER_SUPPLY_TYPE_USB_PD)
				is_fast_charging = true;

			if (!power_supply_get_property(fg->batt_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &val) &&
				(val.intval == POWER_SUPPLY_CHARGE_TYPE_TRICKLE ||
				 val.intval == POWER_SUPPLY_CHARGE_TYPE_FAST))
				is_cc = true;

			/*  in case of fast charging, fcc is refered instead of
				real current for avoiding qni negative pulse effect */
			if (is_fast_charging && is_cc ) {
				ichg = !power_supply_get_property(fg->batt_psy,
							POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val) ?
								val.intval / -1000 : ichg;
				goto out;
			} else {
				/* if charging current is over -25mA,
					batt_therm compensation current keeps the previous value */
				if (!power_supply_get_property(fg->batt_psy,
						POWER_SUPPLY_PROP_CURRENT_NOW, &val) &&
							val.intval > 25000)
					ichg = val.intval / 1000;

				goto out;
			}
		}
	}

	ichg = 0;

out:
	return ichg;
}

static int filtered_batt_therm(bool changed, int comp, int batt)
{
	bool tbl_changed = changed;
	int battemp_cell = batt;
	int battemp_comp = comp;
	static int pre_battemp_cell = -9999;
	static int pre_battemp_comp = -9999;
	static bool is_filtering_rise = false;
	static bool is_filtering_fall = false;
	int battemp_cell_diff = 0;
	int battemp_comp_diff = 0;

	if (!((tbl_changed && tcomp.rise_cut
			&& (battemp_comp > tcomp.rise_cut_trig)) ||
		(tbl_changed && tcomp.fall_cut
			&& (battemp_comp < tcomp.fall_cut_trig))))
		tbl_changed = false;

	if ((tbl_changed || is_filtering_rise || is_filtering_fall)
		&& (pre_battemp_cell > -9999 && pre_battemp_comp > -9999)) {
		battemp_cell_diff = battemp_cell - pre_battemp_cell;
		battemp_comp_diff = battemp_comp - pre_battemp_comp;
		// rise
		if (tcomp.rise_cut && (battemp_comp >= pre_battemp_comp)) {
			if (is_filtering_fall)
				is_filtering_fall = false;

			if (battemp_comp_diff > battemp_cell_diff) {
				is_filtering_rise = true;
				if ( battemp_cell_diff > 0 )
					battemp_comp = pre_battemp_comp + battemp_cell_diff;
				else
					battemp_comp = pre_battemp_comp;
			}
			else {
				is_filtering_rise = false;
			}
		}
		// fall
		else if (tcomp.fall_cut) {
			if (is_filtering_rise)
				is_filtering_rise = false;

			if (battemp_cell_diff > battemp_comp_diff ) {
				is_filtering_fall = true;
				if (battemp_cell_diff < 0)
					battemp_comp = pre_battemp_comp + battemp_cell_diff;
				else
					battemp_comp = pre_battemp_comp;
			}
			else {
				is_filtering_fall = false;
			}
		}
		else if (tcomp.rise_cut) {
			if (is_filtering_rise)
				is_filtering_rise = false;
		}
	}

	pre_battemp_cell = battemp_cell;
	pre_battemp_comp = battemp_comp;
	return battemp_comp;
}

static int calculate_battery_temperature(/* @Nonnull */ struct fg_dev *fg)
{
	int battemp_bias, battemp_icomp, battemp_cell = 0;
	int i, temp, ichg = 0, tbl_pt = 0;
	union power_supply_propval val = { 0, };
	bool tbl_changed = false;
	static int pre_tbl_pt = -1;

	if (fg_gen4_get_battery_temp(fg, &battemp_cell)){
		pr_info("get real batt therm error\n");
		return LGE_FG_INITVAL;
	}

	if (wa_skip_batt_temp_on_bootup_check(battemp_cell, false))
		return 250;

	if (!tcomp.load_done) {
		pr_info("not ready tcomp table. raw temp=%d\n", battemp_cell);
		return battemp_cell;
	}

	if (!tcomp.icoeff_load_done) {
		pr_info("not ready icoeff. raw temp=%d\n", battemp_cell);
		return battemp_cell;
	}

	if (tcomp.load_max > 1) {
		switch (get_charging_type(fg)) {
			case TCOMP_CHG_WLC_LCDOFF: tbl_pt = 1; break;
			case TCOMP_CHG_WLC_LCDON:  tbl_pt = 2; break;
			default: tbl_pt = 0; break;
		}
	}
	else
		tbl_pt = 0;

	if (pre_tbl_pt >= 0 )
		if (pre_tbl_pt != tbl_pt)
			tbl_changed = true;
	pre_tbl_pt = tbl_pt;

	/* Compensating battemp_bias */
	for (i = 0; i < TCOMP_COUNT; i++) {
		if (battemp_cell < tcomp.table[tbl_pt][i].temp_cell)
			break;
	}

	if (i == 0)
		battemp_bias = tcomp.table[tbl_pt][0].temp_bias;
	else if (i == TCOMP_COUNT)
		battemp_bias = tcomp.table[tbl_pt][TCOMP_COUNT-1].temp_bias;
	else
		battemp_bias =
		(	(tcomp.table[tbl_pt][i].temp_bias -
				tcomp.table[tbl_pt][i-1].temp_bias)
			* (battemp_cell - tcomp.table[tbl_pt][i-1].temp_cell)
			/ (tcomp.table[tbl_pt][i].temp_cell -
				tcomp.table[tbl_pt][i-1].temp_cell)
		) + tcomp.table[tbl_pt][i-1].temp_bias;

	/* Compensating battemp_icomp */
	if (fg->batt_psy) {
		if (tcomp.qnovo_charging) {
			ichg = get_batt_temp_current(fg);
		}
		else {
			if (!power_supply_get_property(
				fg->batt_psy, POWER_SUPPLY_PROP_STATUS_RAW, &val)
				&& val.intval == POWER_SUPPLY_STATUS_CHARGING
				&& !power_supply_get_property(
					fg->batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val)
				&& val.intval > 0)
				ichg = ((val.intval) / 1000);
		}
	} else {
		pr_info("Battery is not available, %d(=%d+%d) as batt temp\n",
			battemp_cell + battemp_bias, battemp_cell, battemp_bias);
	}

	battemp_icomp = ichg * ichg * tcomp.icoeff / 10000000;
	temp = battemp_cell + battemp_bias - battemp_icomp;

	if (tcomp.logging)
		pr_info("Battery temperature : "
				"%d = (%d)(cell) + (%d)(bias) - %d(icomp), "
				"icoeff = %d, ichg = %d\n",
			temp, battemp_cell, battemp_bias, battemp_icomp,
			tcomp.icoeff, ichg);

	if (tcomp.rise_cut || tcomp.fall_cut)
		return filtered_batt_therm(tbl_changed, temp, battemp_cell);

	return temp;
}

#define LGE_FG_CHARGING         1
#define LGE_FG_DISCHARGING      0
static int lge_is_fg_charging(struct fg_dev *fg)
{
	union power_supply_propval val = { 0, };
	bool power_status = false;

	if (!fg || !fg->batt_psy || !fg->usb_psy)
		return -1;

	if (!power_supply_get_property(fg->batt_psy,
					POWER_SUPPLY_PROP_STATUS_RAW, &val))
		power_status = (val.intval == POWER_SUPPLY_STATUS_CHARGING) ? true : false;
	else
		return -1;

	if (!power_status)
		return LGE_FG_DISCHARGING;

	if (!power_supply_get_property(fg->batt_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW, &val)) {
		if (val.intval > 25000 )
			return LGE_FG_CHARGING;
		else
			return LGE_FG_DISCHARGING;
	}

	return -1;
}

#define LGE_SOC_SCALE_UNIT	10
#define LGE_SOC_ROUND_UNIT	5
static int lge_get_ui_soc(struct fg_dev *fg, int msoc_raw)
{
	int round_soc = (((msoc_raw * FULL_CAPACITY * LGE_SOC_SCALE_UNIT) \
			 / rescale.criteria) + LGE_SOC_ROUND_UNIT) / LGE_SOC_SCALE_UNIT;
	int new_result = min(FULL_CAPACITY, round_soc);

	rescale.rawsoc = msoc_raw;

	pr_debug("msoc_raw: %d, round_soc = %d\n", msoc_raw, round_soc);

	if (!rescale.lge_monotonic) {
		if ( msoc_raw != 0 && new_result == 0)
			new_result = 1;
		rescale.result = new_result;
		return 0;
	}

	if (rescale.result <= 0 ||
		max(rescale.result, new_result) - min(rescale.result, new_result) > 5 )
		rescale.result = new_result;

	switch (lge_is_fg_charging(fg)) {
		case LGE_FG_CHARGING:
			pr_info("fg_rescale: charging: %d = max(old=%d, new=%d)\n",
				max(rescale.result, new_result), rescale.result, new_result);
			rescale.result = max(rescale.result, new_result);
			break;
		case LGE_FG_DISCHARGING:
			pr_info("fg_rescale: discharging: %d = min(old=%d, new=%d)\n",
				min(rescale.result, new_result), rescale.result, new_result);
			rescale.result = min(rescale.result, new_result);
			break;
		default:
			pr_info("fg_rescale: error: old=%d, new=%d\n", rescale.result, new_result);
			rescale.result = new_result;
			break;
	}

	if ( msoc_raw != 0 && new_result == 0)
			new_result = 1;
	return 0;
}

static int backup_from_cycle_counter(
	struct fg_dev *fg, struct cycle_counter *ori, u16 *backup)
{
	int i = 0, total = 0;

	if (!ori) {
		fg_dbg(fg, FG_LGE, "origin value is not defined - error.\n");
		return -ENOMEM;
	}

	/* read cycle count data from sram */
	ori->restore_count(ori->data, ori->count, BUCKET_COUNT);
	/* backup cycle count*/
	mutex_lock(&ori->lock);
	for (i = 0; i < BUCKET_COUNT; i++) {
		fg_dbg(fg, FG_LGE, "backup_from: %d: %d -> %d\n",
			i, ori->count[i], backup[i]);
		backup[i] = ori->count[i];
		total += backup[i];
	}
	mutex_unlock(&ori->lock);

	if (total > (3000 * BUCKET_COUNT)) {
		fg_dbg(fg, FG_LGE,
			"total counter(%d) is over..clear backup_from\n", total / BUCKET_COUNT);
		for (i = 0; i < BUCKET_COUNT; i++) {
			backup[i] = 0;
		}
	}

	return 0;
}

static int backup_to_cycle_counter(
	struct fg_dev *fg, struct cycle_counter *ori, u16 *backup)
{
	int rc = 0, i;

	if (!ori) {
		fg_dbg(fg, FG_LGE, "origin value is not defined - error.\n");
		return -ENOMEM;
	}

	mutex_lock(&ori->lock);
	/* restore cycle count from backup*/
	for (i = 0; i < BUCKET_COUNT; i++) {
		fg_dbg(fg, FG_LGE, "backup_to: %d: %d <- %d\n",
			i, ori->count[i], backup[i]);
		ori->count[i] = backup[i];
	}
	/* write backuped cycle count to from sram */
	rc = ori->store_count(ori->data, ori->count, 0, BUCKET_COUNT*2);
	if (rc < 0)
		fg_dbg(fg, FG_LGE, "failed to write cycle counter rc=%d\n", rc);
	mutex_unlock(&ori->lock);

	return 0;
}

static void fginfo_snapshot_print(void)
{
	printk("PMINFO: [CSV] "
/* Capacity  */ "cSYS:%d, cMNT:%d, cCHG:%d, cLRN:%d, "
/* v/i ADCs  */ "iBAT:%d, vBAT:%d, vPRD:%d, vOCV:%d, vUSB:%d, iUSB:%d, vWLC:%d, iWCL:%d, AiCL:%d, "
/* Temp      */ "tSYS:%d, tORI:%d, tVTS:%d, "
/* Impedance */ "rTOT:%d, rESR:%d, rSLW:%d, "
/* Misc      */ "CYCLE:%d, "
/* SoCs      */ "sBATT:%d, sCCSW:%d, sCC:%d\n",

/* Capacity  */ fginfo.capacity_rescaled*10, fginfo.capacity_monotonic, fginfo.capacity_chargecnt, fginfo.capacity_learned,
/* Battery   */ fginfo.battery_inow, fginfo.battery_vnow, fginfo.battery_vpredict, fginfo.battery_ocv,
/* Input     */ fginfo.input_vusb, fginfo.input_iusb, fginfo.input_vwlc, fginfo.input_iwlc, fginfo.input_aicl,
/* Temp      */ fginfo.temp_compensated, fginfo.temp_thermister, fginfo.temp_vts,
/* Impedance */ fginfo.impedance_esr + fginfo.impedance_rslow, fginfo.impedance_esr, fginfo.impedance_rslow,
/* Misc      */ fginfo.misc_cycle,
/* SoCs      */ fginfo.misc_battsoc, fginfo.misc_ccsocsw, fginfo.misc_ccsoc);
}

static void
fginfo_snapshot_inputnow(int* vusb, int* iusb, int* vwlc, int* iwlc, int* aicl)
{
	struct power_supply* psy_main = power_supply_get_by_name("main");
	struct power_supply* psy_usb = power_supply_get_by_name("usb");
	struct power_supply* psy_dc = power_supply_get_by_name("dc");
	struct power_supply* psy_wlc = power_supply_get_by_name("wireless");
	union power_supply_propval val = { 0, };

	*aicl = (psy_main && !power_supply_get_property(
			psy_main, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED, &val))
				? val.intval/1000 : LGE_FG_INITVAL;
	*vusb = (psy_usb && !power_supply_get_property(
			psy_usb, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val))
				? val.intval/1000 : LGE_FG_INITVAL;
	*iusb = (psy_usb && !power_supply_get_property(
			psy_usb, POWER_SUPPLY_PROP_INPUT_CURRENT_NOW, &val))
				? val.intval/1000 : LGE_FG_INITVAL;
	*vwlc = (psy_dc && !power_supply_get_property(
			psy_dc, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val))
				? val.intval/1000 : LGE_FG_INITVAL;
	*iwlc = (psy_wlc && !power_supply_get_property(
			psy_wlc, POWER_SUPPLY_PROP_CURRENT_NOW, &val))
				? val.intval : LGE_FG_INITVAL;

	if (psy_main)
		power_supply_put(psy_main);
	if (psy_usb)
		power_supply_put(psy_usb);
	if (psy_dc)
		power_supply_put(psy_dc);
	if (psy_wlc)
		power_supply_put(psy_wlc);
}

static void fginfo_snapshot_update(struct power_supply *psy)
{
	int buf = 0;
	int64_t temp64 = 0;
	struct fg_dev* fg = power_supply_get_drvdata(psy);
	struct fg_gen4_chip *chip = container_of(fg, struct fg_gen4_chip, fg);
	struct thermal_zone_device*	tzd = thermal_zone_get_zone_by_name("vts-virt-therm");

	if (!tzd || !fg || !chip)
		return;

/* Capacity */
	fginfo.capacity_rescaled = rescale.result < 0
		? LGE_FG_INITVAL : rescale.result;
	fginfo.capacity_monotonic = !fg_get_msoc_raw(fg, &buf)
		? buf * 1000 / 255 : LGE_FG_INITVAL;
	fginfo.capacity_chargecnt = !fg_gen4_get_charge_counter(chip, &buf)
		? buf / 1000 : LGE_FG_INITVAL;
	fginfo.capacity_learned = !fg_gen4_get_learned_capacity(chip, &temp64)
		? (int)temp64 / 1000 : LGE_FG_INITVAL;
/* Battery */
	fginfo.battery_inow = !fg_get_battery_current(fg, &buf)
		? buf / 1000 : LGE_FG_INITVAL;
	fginfo.battery_vnow = !fg_get_battery_voltage(fg, &buf)
		? buf / 1000 : LGE_FG_INITVAL;
	fginfo.battery_vpredict = !fg_get_sram_prop(fg, FG_SRAM_VOLTAGE_PRED, &buf)
		? buf / 1000 : LGE_FG_INITVAL;
	fginfo.battery_ocv = !fg_get_sram_prop(fg, FG_SRAM_OCV, &buf)
		? buf / 1000 : LGE_FG_INITVAL;
/* Input */
	fginfo_snapshot_inputnow(&fginfo.input_vusb, &fginfo.input_iusb,
		&fginfo.input_vwlc, &fginfo.input_iwlc,
		&fginfo.input_aicl);
/* Temperature */
	fginfo.temp_compensated
		= calculate_battery_temperature(fg);
	fginfo.temp_thermister = !fg_gen4_get_battery_temp(fg, &buf)
		? buf : LGE_FG_INITVAL;
	fginfo.temp_vts = !thermal_zone_get_temp(tzd, &buf)
		? buf / 100 : -1000;
/* Impedance */
	fginfo.impedance_esr = !fg_get_sram_prop(fg, FG_SRAM_ESR_MDL, &buf)
		? buf : LGE_FG_INITVAL;
	fginfo.impedance_rslow = !fg_get_sram_prop(fg, FG_SRAM_RSLOW, &buf)
		? buf : LGE_FG_INITVAL;
/* Misc */
	fginfo.misc_cycle = !get_cycle_count(chip->counter, &buf)
		? buf : LGE_FG_INITVAL;
/* SoCs */
	fginfo.misc_battsoc = !fg_get_sram_prop(fg, FG_SRAM_BATT_SOC, &buf)
		? ((u32)buf >> 24)*1000/255 : LGE_FG_INITVAL;
	fginfo.misc_ccsocsw = !fg_get_sram_prop(fg, FG_SRAM_CC_SOC_SW, &buf)
		? div_s64((s64)buf*1000, CC_SOC_30BIT) : LGE_FG_INITVAL;
	fginfo.misc_ccsoc = !fg_get_sram_prop(fg, FG_SRAM_CC_SOC, &buf)
		? div_s64((s64)buf*1000, CC_SOC_30BIT) : LGE_FG_INITVAL;

	/* logging finally */
	fginfo_snapshot_print();
}

///////////////////////////////////////////////////////////////////////////////
#define PROPERTY_CONSUMED_WITH_SUCCESS	0
#define PROPERTY_CONSUMED_WITH_FAIL	EINVAL
#define PROPERTY_BYPASS_REASON_NOENTRY	ENOENT
#define PROPERTY_BYPASS_REASON_ONEMORE	EAGAIN

static int wa_backup_bms_property[POWER_SUPPLY_PROP_CYCLE_COUNTS + 1];

static enum power_supply_property extension_bms_appended [] = {
	POWER_SUPPLY_PROP_UPDATE_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN, /* cl.skew coulomb count for qnovo */
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX, /* cl.skew ration for qnovo */
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,     /* LG ui rawsoc */
};

static int
extension_bms_get_property_pre(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *val)
{
	int rc = PROPERTY_CONSUMED_WITH_SUCCESS;
	struct fg_dev* fg = power_supply_get_drvdata(psy);
	struct irq_desc* irq = irq_to_desc(fg->irqs[SOC_UPDATE_IRQ].irq);

	switch (prop) {
	case POWER_SUPPLY_PROP_RESISTANCE:
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
	case POWER_SUPPLY_PROP_CHARGE_NOW_RAW:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	case POWER_SUPPLY_PROP_CC_SOC:
	case POWER_SUPPLY_PROP_CURRENT_AVG:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_POWER_NOW:
	case POWER_SUPPLY_PROP_POWER_AVG:
		if (irq && !irq->depth) {
			val->intval = wa_backup_bms_property[prop];
			pr_info("[W/A] BBP) SOC_UPDATE_IRQ enabled! (==Reading blocked!) "
				"Skip to read property(%d) and reuse %d for faster access time\n",
				prop, val->intval);
		}
		else {
			pr_debug("[W/A] BBP) Not blocked! Bypass to get prp (%d) from bms psy\n",
				prop);
			rc = -PROPERTY_BYPASS_REASON_ONEMORE;
		}
		break;

	case POWER_SUPPLY_PROP_CAPACITY :
		// Battery fake setting has top priority
		if (fake.capacity != LGE_FG_INITVAL)
			val->intval = fake.capacity;
		else if (rescale.result != LGE_FG_INITVAL)
			val->intval = rescale.result;
		else
			rc = -PROPERTY_BYPASS_REASON_ONEMORE;
		break;

	case POWER_SUPPLY_PROP_TEMP :
		if (fake.temperature == LGE_FG_INITVAL) {
			val->intval = calculate_battery_temperature(fg); // Use compensated temperature
		} else
			val->intval = fake.temperature;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW :
		if (fake.uvoltage == LGE_FG_INITVAL)
			rc = -PROPERTY_BYPASS_REASON_ONEMORE;
		else
			val->intval = fake.uvoltage;
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL :
		val->intval = rescale.rawsoc;
		break;

	case POWER_SUPPLY_PROP_UPDATE_NOW :
		/* Do nothing and just consume getting */
		val->intval = -1;
		break;

	default:
		rc = -PROPERTY_BYPASS_REASON_NOENTRY;
		break;
	}

	return rc;
}

static int
extension_bms_get_property_post(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *val, int rc)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_RESISTANCE:
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
	case POWER_SUPPLY_PROP_CHARGE_NOW_RAW:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	case POWER_SUPPLY_PROP_CC_SOC:
	case POWER_SUPPLY_PROP_CURRENT_AVG:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_POWER_NOW:
	case POWER_SUPPLY_PROP_POWER_AVG:
		wa_backup_bms_property[prop] = val->intval;
		break;

	default:
		break;
	}

	return rc;
}

static int
extension_bms_set_property_pre(struct power_supply *psy,
	enum power_supply_property prop, const union power_supply_propval *val)
{
	int* fakeset = NULL;
	int  rc = PROPERTY_CONSUMED_WITH_SUCCESS;
	struct fg_dev* fg = power_supply_get_drvdata(psy);
	struct fg_gen4_chip *chip = container_of(fg, struct fg_gen4_chip, fg);

	switch (prop) {
	case POWER_SUPPLY_PROP_UPDATE_NOW :
		if (val->intval)
			fginfo_snapshot_update(psy);
		break;

	case POWER_SUPPLY_PROP_TEMP :
		fakeset = &fake.temperature;
		break;
	case POWER_SUPPLY_PROP_CAPACITY :
		fakeset = &fake.capacity;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW :
		fakeset = &fake.uvoltage;
		break;

	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN : /* cl.skew coulomb count for qnovo */
		chip->cl->dt.skew_cc = val->intval;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX : /* cl.skew ration for qnovo */
		chip->cl->dt.skew_decipct = val->intval;
		break;

	default:
		rc = -PROPERTY_BYPASS_REASON_NOENTRY;
	}

	if (fakeset && *fakeset != val->intval) {
		*fakeset = val->intval;
		power_supply_changed(fg->batt_psy);
	}

	return rc;
}

static int
extension_bms_set_property_post(struct power_supply *psy,
	enum power_supply_property prop, const union power_supply_propval *val, int rc)
{
	return rc;
}

///////////////////////////////////////////////////////////////////////////////
static enum power_supply_property* extension_bms_properties(void)
{
	static enum power_supply_property
		extended_properties[ARRAY_SIZE(fg_psy_props)
		+ ARRAY_SIZE(extension_bms_appended)];
	int size_original = ARRAY_SIZE(fg_psy_props);
	int size_appended = ARRAY_SIZE(extension_bms_appended);

	memcpy(extended_properties, fg_psy_props,
		size_original * sizeof(enum power_supply_property));
	memcpy(&extended_properties[size_original], extension_bms_appended,
		size_appended * sizeof(enum power_supply_property));

	veneer_extension_pspoverlap(fg_psy_props, size_original,
		extension_bms_appended, size_appended);

	return extended_properties;
}

static size_t extension_bms_num_properties(void)
{
	return ARRAY_SIZE(fg_psy_props) + ARRAY_SIZE(extension_bms_appended);
}

static int extension_bms_get_property(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *val)
{
	int rc = extension_bms_get_property_pre(psy, prop, val);
	if (rc == -PROPERTY_BYPASS_REASON_NOENTRY
		|| rc == -PROPERTY_BYPASS_REASON_ONEMORE)
		rc = fg_psy_get_property(psy, prop, val);
	rc = extension_bms_get_property_post(psy, prop, val, rc);

	return rc;
}

static int extension_bms_set_property(struct power_supply *psy,
	enum power_supply_property prop, const union power_supply_propval *val)
{
	int rc = extension_bms_set_property_pre(psy, prop, val);
	if (rc == -PROPERTY_BYPASS_REASON_NOENTRY
		|| rc == -PROPERTY_BYPASS_REASON_ONEMORE)
		rc = fg_psy_set_property(psy, prop, val);
	rc = extension_bms_set_property_post(psy, prop, val, rc);

	return rc;
}

static int extension_bms_property_is_writeable(
	struct power_supply *psy, enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_UPDATE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CAPACITY_RAW:
		rc = 1;
		break;
	default:
		rc = fg_property_is_writeable(psy, prop);
		break;
	}
	return rc;
}

#define BIAS_MVOLTAGE	1825
static int extension_get_batt_id(struct fg_dev *fg, int *batt_id)
{
	struct iio_channel* bat_id_chan = NULL;
	struct device_node* node = fg->dev->of_node;
	int rc = 0, batt_id_adc = 0;

	*batt_id = unified_bootmode_batteryid();
	if (*batt_id) {
		pr_debug("battid from shared memory=%d\n", *batt_id);
	}
	else {
		rc = of_property_match_string(node, "io-channel-names", "bat_id");
		if (rc < 0) {
			pr_err("Couldn't find node name, rc=%d\n", rc);
			return rc;
		}

		bat_id_chan = iio_channel_get(fg->dev, "bat_id");
		if (IS_ERR(bat_id_chan)) {
			rc = PTR_ERR(bat_id_chan);
			pr_err("Failed to get bat-id channel, rc=%d\n", rc);
			return rc;
		}
		else {
			rc = iio_read_channel_processed(bat_id_chan, &batt_id_adc);
			if (rc < 0) {
				pr_err("Failed to read Batt_ID over ADC, rc=%d\n", rc);
				goto out;
			}
			*batt_id = (100 * 1000 * 1000) /
				(BIAS_MVOLTAGE * 1000 / (batt_id_adc / 1000) - 1000);
			pr_info("batt_id_ohms=%d, batt_id_adc=%d\n", *batt_id, batt_id_adc);
		}
	}

out:
	iio_channel_release(bat_id_chan);
	return rc;
}

static int extension_fg_gen4_adjust_cutoff_ki_coeff(
	struct fg_gen4_chip *chip, int batt_temp)
{
	struct fg_dev *fg = &chip->fg;
	int rc = 0;
	static int cutoff_ki_coeff = -EINVAL;
	u8 val;

	if (!chip->cutoff_ki_coeff_en)
		return 0;

	if (batt_temp < 0 && fg->charge_status == POWER_SUPPLY_STATUS_DISCHARGING) {
		if (cutoff_ki_coeff == chip->dt.cutoff_lt_ki_coeff)
			return 0;
		cutoff_ki_coeff = chip->dt.cutoff_lt_ki_coeff;
	} else {
		if (cutoff_ki_coeff == chip->dt.cutoff_ki_coeff)
			return 0;
		cutoff_ki_coeff = chip->dt.cutoff_ki_coeff;
	}

	fg_encode(fg->sp, FG_SRAM_CUTOFF_KI_COEFF, cutoff_ki_coeff, &val);
	rc = fg_sram_write(fg,
		fg->sp[FG_SRAM_CUTOFF_KI_COEFF].addr_word,
		fg->sp[FG_SRAM_CUTOFF_KI_COEFF].addr_byte, &val,
		fg->sp[FG_SRAM_CUTOFF_KI_COEFF].len,
		FG_IMA_DEFAULT);
	if (rc < 0) {
		pr_err("Error in writing cutoff_ki_coeff, rc=%d\n",rc);
		return rc;
	}

	fg_dbg(fg, FG_LGE, "Wrote cutoff_ki_coeff = %d(0x%X), batt_temp = %d\n",
		cutoff_ki_coeff, val, batt_temp);

	return 0;
}

static int extension_fg_parse_cutoff_ki_coefficients(struct fg_dev *fg, struct device_node *node)
{
	struct fg_gen4_chip *chip = container_of(fg, struct fg_gen4_chip, fg);

	chip->dt.cutoff_ki_coeff = -EINVAL;
	of_property_read_u32(node, "qcom,cutoff-ki-coeff",
		&chip->dt.cutoff_ki_coeff);

	chip->dt.cutoff_lt_ki_coeff = -EINVAL;
	of_property_read_u32(node, "qcom,cutoff-lt-ki-coeff",
		&chip->dt.cutoff_lt_ki_coeff);

	chip->cutoff_ki_coeff_en = true;
	if (chip->dt.cutoff_ki_coeff == -EINVAL
		|| chip->dt.cutoff_lt_ki_coeff == -EINVAL)
	chip->cutoff_ki_coeff_en = false;

	pr_info("FG: battery cutoff ki-coeff: en=%d, %d, %d\n",
		chip->cutoff_ki_coeff_en,
		chip->dt.cutoff_ki_coeff,
		chip->dt.cutoff_lt_ki_coeff);
	return 0;
}

static struct device_node *extension_get_batt_profile(
	struct fg_dev *fg, struct device_node* container, int resistance_id)
{
    /* Search with resistance_id and
	 * Hold the result to an unified node(sysfs) for the fab process
	 */
	struct device_node* node;
	const char* name;
	char buffer [8] = { '\0', };
	int kohm = 0;
	struct device_node* profile =
			of_batterydata_get_best_profile(container, resistance_id, NULL);

	extension_fg_parse_cutoff_ki_coefficients(fg, profile);

	/* If no matching, set it as default */
	if (!profile) {
		node = of_find_node_by_name(NULL, "lge-battery-supplement");
		if (of_property_read_string(node, "default-battery-name", &name))
			name = NULL;
		kohm = of_property_read_u32(node, "default-battery-kohm", &kohm)
				? 0 : kohm;
		profile = of_batterydata_get_best_profile(container, kohm, name);
		pr_info("Getting default battery profile(%s): %s\n",
			name, profile ? "success" : "fail");
	}

	// At this time, 'battery_valid' may be true always for the embedded battery model
	snprintf(buffer, sizeof(buffer), "%d", !!profile);
	unified_nodes_store("battery_valid", buffer, sizeof(buffer));

	return profile;
}

static int extension_fg_load_dt_post(struct fg_dev *fg)
{
	struct device_node* post_dtroot;
	struct device_node* coeff_override;
	struct fg_gen4_chip *chip = container_of(fg, struct fg_gen4_chip, fg);
	int dt_icomp = 0;
	int dt_kcoeff = 0;
	int dt_fcc = 0;
	int rc = -1;
	u8 val = 0;

	if (tcomp.icoeff_load_done) {
		pr_info("icoeff had already been loaded.\n");
		return 0;
	}

	if (!fg->soc_reporting_ready) {
		pr_info("fG profile is not ready.\n");
		return LGE_FG_INITVAL;
	}

	post_dtroot = of_find_node_by_name(NULL, "lge-battery-supplement");
	if (!post_dtroot) {
		pr_info("failed to find lge-battery-supplement\n");
		return LGE_FG_INITVAL;
	}

	if (fg->bp.batt_type_str) {
		coeff_override = of_find_node_by_name(
				post_dtroot, fg->bp.batt_type_str);
		if (coeff_override) {
			if (of_property_read_u32(
					coeff_override, "tempcomp-icoeff", &dt_icomp) >= 0)
				pr_info("ICOEFF is overridden to %d for %s\n",
						dt_icomp, fg->bp.batt_type_str);


			chip->dt.ki_coeff_low_chg = -EINVAL;
			dt_kcoeff |= of_property_read_u32(coeff_override, "qcom,ki-coeff-low-chg",
				&chip->dt.ki_coeff_low_chg);

			chip->dt.ki_coeff_med_chg = -EINVAL;
			dt_kcoeff |= of_property_read_u32(coeff_override, "qcom,ki-coeff-med-chg",
				&chip->dt.ki_coeff_med_chg);

			chip->dt.ki_coeff_hi_chg = -EINVAL;
			dt_kcoeff |= of_property_read_u32(coeff_override, "qcom,ki-coeff-hi-chg",
				&chip->dt.ki_coeff_hi_chg);

			if (dt_kcoeff) {
				chip->dt.ki_coeff_low_chg = -EINVAL;
				chip->dt.ki_coeff_med_chg = -EINVAL;
				chip->dt.ki_coeff_hi_chg = -EINVAL;
			}

			if (chip->dt.ki_coeff_low_chg != -EINVAL) {
				fg_encode(fg->sp, FG_SRAM_KI_COEFF_LOW_CHG,
					chip->dt.ki_coeff_low_chg, &val);
				rc = fg_sram_write(fg,
					fg->sp[FG_SRAM_KI_COEFF_LOW_CHG].addr_word,
					fg->sp[FG_SRAM_KI_COEFF_LOW_CHG].addr_byte, &val,
					fg->sp[FG_SRAM_KI_COEFF_LOW_CHG].len,
					FG_IMA_DEFAULT);
				if (rc < 0) {
					pr_err("Error in writing ki_coeff_low_chg, rc=%d\n",
						rc);
					return rc;
				}
			}

			if (chip->dt.ki_coeff_med_chg != -EINVAL) {
				fg_encode(fg->sp, FG_SRAM_KI_COEFF_MED_CHG,
					chip->dt.ki_coeff_med_chg, &val);
				rc = fg_sram_write(fg,
					fg->sp[FG_SRAM_KI_COEFF_MED_CHG].addr_word,
					fg->sp[FG_SRAM_KI_COEFF_MED_CHG].addr_byte, &val,
					fg->sp[FG_SRAM_KI_COEFF_MED_CHG].len,
					FG_IMA_DEFAULT);
				if (rc < 0) {
					pr_err("Error in writing ki_coeff_med_chg, rc=%d\n",
						rc);
					return rc;
				}
			}

			if (chip->dt.ki_coeff_hi_chg != -EINVAL) {
				fg_encode(fg->sp, FG_SRAM_KI_COEFF_HI_CHG,
					chip->dt.ki_coeff_hi_chg, &val);
				rc = fg_sram_write(fg,
					fg->sp[FG_SRAM_KI_COEFF_HI_CHG].addr_word,
					fg->sp[FG_SRAM_KI_COEFF_HI_CHG].addr_byte, &val,
					fg->sp[FG_SRAM_KI_COEFF_HI_CHG].len,
					FG_IMA_DEFAULT);
				if (rc < 0) {
					pr_err("Error in writing ki_coeff_hi_chg, rc=%d\n", rc);
					return rc;
				}
			}

			pr_info("FG: battery ki-coeff-chg: %d, %d, %d\n",
				chip->dt.ki_coeff_low_chg,
				chip->dt.ki_coeff_med_chg,
				chip->dt.ki_coeff_hi_chg);
		}
	}

	if (of_property_read_u32(post_dtroot, "qcom,fastchg-current-ma", &dt_fcc) >= 0) {
		fg->bp.fastchg_curr_ma = dt_fcc;
		pr_info("FCC is overriden to %d\n", fg->bp.fastchg_curr_ma);
	}

	if (!dt_icomp) {
		if (of_property_read_u32(post_dtroot, "tempcomp-icoeff", &dt_icomp) >= 0) {
			pr_info("ICOEFF is set to %d by default\n", dt_icomp);
		} else {
			pr_info("ICOEFF isn't set. error.\n");
			return -1;
		}
	}

	tcomp.icoeff = dt_icomp;
	tcomp.icoeff_load_done = true;
	return 0;
}

static int extension_fg_load_dt(void)
{
	const char str_tempcomp[TCOMP_TABLE_MAX][30] = {
		"tempcomp-offset",
		"tempcomp-offset-wlc-lcdoff",
		"tempcomp-offset-wlc-lcdon"
	};

	struct device_node* tcomp_dtroot;
	int dtarray_count = TCOMP_COUNT * 2;
	u32 dtarray_data [TCOMP_COUNT * 2];
	int i = 0, j = 0;

	if (tcomp.load_done) {
		pr_info("tcomp table had already been loaded.\n");
		return 0;
	}

	tcomp_dtroot = of_find_node_by_name(NULL, "lge-battery-supplement");
	if (!tcomp_dtroot) {
		pr_info("failed to find lge-battery-supplement\n");
		return -1;
	}

	if (of_property_read_bool(tcomp_dtroot, "tempcomp-offset-wlc-enable"))
		tcomp.load_max = 3;
	else
		tcomp.load_max = 1;

	for (j = 0; j < tcomp.load_max; j++ ) {
		/* Finding tcomp_table and tcomp_icoeff */
		if (of_property_read_u32_array(tcomp_dtroot, str_tempcomp[j],
				dtarray_data, dtarray_count) >= 0) {
			for (i = 0; i < dtarray_count; i += 2) {
				tcomp.table[j][i/2].temp_cell = dtarray_data[i];
				tcomp.table[j][i/2].temp_bias = dtarray_data[i+1];
				pr_debug("Index = %02d : %4d - %4d\n",
					i/2,
					tcomp.table[j][i/2].temp_cell,
					tcomp.table[j][i/2].temp_bias);
			}
		} else {
			pr_info("%s is not found, error\n", str_tempcomp[j]);
			tcomp.table[j][0].temp_cell = INT_MAX;
			tcomp.table[j][0].temp_bias = 0;
			return -1;
		}
	}

	tcomp.rise_cut = of_property_read_bool(tcomp_dtroot,
		"tempcomp-offset-wlc-rise-filter-enable");
	if (tcomp.rise_cut)
		of_property_read_u32(tcomp_dtroot,
			"tempcomp-offset-wlc-rise-filter-trigger", &tcomp.rise_cut_trig);
	tcomp.fall_cut = of_property_read_bool(tcomp_dtroot,
		"tempcomp-offset-wlc-fall-filter-enable");
	if (tcomp.fall_cut)
		of_property_read_u32(tcomp_dtroot,
			"tempcomp-offset-wlc-fall-filter-trigger", &tcomp.fall_cut_trig);

	of_property_read_u32(tcomp_dtroot,
		"capacity-raw-full", &rescale.criteria);
	rescale.lge_monotonic = of_property_read_bool(tcomp_dtroot,
		"lg-monotonic-soc-enable");
	tcomp.logging = of_property_read_bool(tcomp_dtroot,
		"tempcomp-logging");
	tcomp.qnovo_charging = of_property_read_bool(tcomp_dtroot,
		"tempcomp-qnovo-charging");

	if (j == tcomp.load_max) {
		tcomp.load_done = true;

		pr_info("[tempcomp config] table count: %s (%d/%d), "
			"rise cut filter: %s (trigger = %d degree), "
			"fall cut filter: %s (trigger = %d degree)\n",
			(j == tcomp.load_max) ? "done" : "error", j, tcomp.load_max,
			tcomp.rise_cut ? "enabled" : "disabled", tcomp.rise_cut_trig,
			tcomp.fall_cut ? "enabled" : "disabled", tcomp.fall_cut_trig);
	}

	return 0;
}
