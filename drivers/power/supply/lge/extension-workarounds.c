#define pr_fmt(fmt) "WA: %s: " fmt, __func__
#define pr_wa(fmt, ...) pr_err(fmt, ##__VA_ARGS__)
#define pr_dbg_wa(fmt, ...) pr_debug(fmt, ##__VA_ARGS__)

#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/pmic-voter.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>

#include "../qcom/smb5-reg.h"
#include "../qcom/smb5-lib.h"
#include "veneer-primitives.h"
#ifdef CONFIG_LGE_USB_SBU_SWITCH
#include <linux/usb/lge_sbu_switch.h>
#endif

////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Helper functions
////////////////////////////////////////////////////////////////////////////
static struct smb_charger* wa_helper_chg(void) {
	// getting smb_charger from air
	struct power_supply*	psy
		= power_supply_get_by_name("battery");
	struct smb_charger*	chg
		= psy ? power_supply_get_drvdata(psy) : NULL;
	if (psy)
		power_supply_put(psy);

	return chg;
}

#define APSD_RERUN_DELAY_MS 4000
static DEFINE_MUTEX(wa_lock);
static bool wa_command_apsd(/*@Nonnull*/ struct smb_charger* chg) {
	bool	ret = false;
	int rc;

	mutex_lock(&wa_lock);
	rc = smblib_masked_write(chg, CMD_APSD_REG,
				APSD_RERUN_BIT, APSD_RERUN_BIT);
	if (rc < 0) {
		pr_wa("Couldn't re-run APSD rc=%d\n", rc);
		goto failed;
	}
	ret = true;

failed:
	mutex_unlock(&wa_lock);
	return ret;
}

#define USBIN_CMD_ICL_OVERRIDE_REG (USBIN_BASE + 0x42)
#define ICL_OVERRIDE_BIT BIT(0)
bool wa_command_icl_override(/*@Nonnull*/ struct smb_charger* chg) {
	if (smblib_masked_write(chg, USBIN_CMD_ICL_OVERRIDE_REG, ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT) < 0) {
		pr_wa("Couldn't icl override\n");
		return false;
	}

	return true;
}

static void wa_get_pmic_dump_func(struct work_struct *unused) {
	struct smb_charger* chg = wa_helper_chg();
	union power_supply_propval debug  = {-1, };

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	power_supply_set_property(chg->batt_psy,
			POWER_SUPPLY_PROP_DEBUG_BATTERY, &debug);
}
static DECLARE_WORK(wa_get_pmic_dump_work, wa_get_pmic_dump_func);

void wa_get_pmic_dump(void) {
	schedule_work(&wa_get_pmic_dump_work);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Avoiding MBG fault on SBU pin
////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_LGE_USB_SBU_SWITCH
// Rather than accessing pointer directly, Referring it as a singleton instance
static struct lge_sbu_switch_instance* wa_avoiding_mbg_fault_singleton(void) {
	static struct lge_sbu_switch_desc 	wa_amf_description = {
		.flags  = LGE_SBU_SWITCH_FLAG_SBU_AUX
			| LGE_SBU_SWITCH_FLAG_SBU_USBID
			| LGE_SBU_SWITCH_FLAG_SBU_FACTORY_ID,
	};
	static struct lge_sbu_switch_instance*	wa_amf_instance;
	static DEFINE_MUTEX(wa_amf_mutex);
	struct smb_charger* chg;

	if (IS_ERR_OR_NULL(wa_amf_instance)) {
		mutex_lock(&wa_amf_mutex);
		chg = wa_helper_chg();
		if (IS_ERR_OR_NULL(wa_amf_instance) && chg) {
			wa_amf_instance
				= devm_lge_sbu_switch_instance_register(chg->dev, &wa_amf_description);
			if (IS_ERR_OR_NULL(wa_amf_instance))
				devm_lge_sbu_switch_instance_unregister(chg->dev, wa_amf_instance);
		}
		mutex_unlock(&wa_amf_mutex);
	}

	return IS_ERR_OR_NULL(wa_amf_instance) ? NULL : wa_amf_instance;
}

bool wa_avoiding_mbg_fault_uart(bool enable) {
	// Preparing instance and checking validation of it.
	struct lge_sbu_switch_instance* instance
		= wa_avoiding_mbg_fault_singleton();
	if (!instance)
		return false;

	if (enable) {
		if (lge_sbu_switch_get_current_flag(instance) != LGE_SBU_SWITCH_FLAG_SBU_AUX)
			lge_sbu_switch_get(instance, LGE_SBU_SWITCH_FLAG_SBU_AUX);
	}
	else
		lge_sbu_switch_put(instance, LGE_SBU_SWITCH_FLAG_SBU_AUX);

	return true;
}

bool wa_avoiding_mbg_fault_usbid(bool enable) {
	// Preparing instance and checking validation of it.
	struct lge_sbu_switch_instance* instance
		= wa_avoiding_mbg_fault_singleton();
	if (!instance)
		return false;

	if (enable)
		lge_sbu_switch_get(instance, LGE_SBU_SWITCH_FLAG_SBU_FACTORY_ID);
	else
		lge_sbu_switch_put(instance, LGE_SBU_SWITCH_FLAG_SBU_FACTORY_ID);

	return true;
}
#else
bool wa_avoiding_mbg_fault_uart(bool enable) { return false; };
bool wa_avoiding_mbg_fault_usbid(bool enable) { return false; };
#endif


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Detection of Standard HVDCP2
////////////////////////////////////////////////////////////////////////////

#define DSH_VOLTAGE_THRESHOLD  7000
static bool wa_is_standard_hvdcp = false;
static bool wa_detect_standard_hvdcp_done = false;
static void wa_detect_standard_hvdcp_main(struct work_struct *unused) {
	struct smb_charger*  chg = wa_helper_chg();
	union power_supply_propval	val = {0, };
	int rc, usb_vnow;
	wa_detect_standard_hvdcp_done = true;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	rc = smblib_dp_dm(chg, POWER_SUPPLY_DP_DM_FORCE_9V);
	if (rc < 0) {
		pr_wa("Couldn't force 9V rc=%d\n", rc);
		return;
	}

	msleep(200);
	usb_vnow = !smblib_get_prop_usb_voltage_now(chg, &val) ? val.intval/1000 : -1;
	if ( usb_vnow >= DSH_VOLTAGE_THRESHOLD) {
		wa_is_standard_hvdcp = true;
		power_supply_changed(chg->batt_psy);
	}

	pr_wa("Check standard hvdcp. %d mV\n", usb_vnow);
	rc = smblib_dp_dm(chg, POWER_SUPPLY_DP_DM_FORCE_5V);
	if (rc < 0) {
		pr_wa("Couldn't force 5v rc=%d\n", rc);
		return;
	}

	return;
}
static DECLARE_DELAYED_WORK(wa_detect_standard_hvdcp_dwork, wa_detect_standard_hvdcp_main);

void wa_detect_standard_hvdcp_trigger(struct smb_charger* chg) {
	u8 stat;
	int rc;

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		pr_wa("Couldn't read APSD_STATUS_REG rc=%d\n", rc);
		return;
	}

	pr_dbg_wa("apsd_status 0x%x, type %d\n", stat, chg->real_charger_type);
	if ((stat & QC_AUTH_DONE_STATUS_BIT)
			&& chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP
			&& !delayed_work_pending(&wa_detect_standard_hvdcp_dwork)
			&& !wa_detect_standard_hvdcp_done) {
		schedule_delayed_work(&wa_detect_standard_hvdcp_dwork, msecs_to_jiffies(0));
	}
}

void wa_detect_standard_hvdcp_clear(void) {
	wa_is_standard_hvdcp = false;
	wa_detect_standard_hvdcp_done = false;
}

bool wa_detect_standard_hvdcp_check(void) {
	return wa_is_standard_hvdcp;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Rerun apsd for dcp charger
////////////////////////////////////////////////////////////////////////////

static bool wa_rerun_apsd_done = false;

static void wa_rerun_apsd_for_dcp_main(struct work_struct *unused) {
	struct smb_charger* chg = wa_helper_chg();

	if (!chg || chg->pd_active ||!wa_rerun_apsd_done) {
		pr_wa("stop apsd done. apsd(%d), pd(%d)\n", wa_rerun_apsd_done, chg ? chg->pd_active : -1);
		return;
	}

	pr_wa(" Rerun apsd\n");
	wa_command_apsd(chg);
}

static DECLARE_DELAYED_WORK(wa_rerun_apsd_for_dcp_dwork, wa_rerun_apsd_for_dcp_main);

void wa_rerun_apsd_for_dcp_triger(struct smb_charger *chg) {
	union power_supply_propval val = { 0, };
	bool usb_type_dcp = chg->real_charger_type == POWER_SUPPLY_TYPE_USB_DCP;
	bool usb_vbus_high
		= !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val) ? !!val.intval : false;
	u8 stat;
	int rc;

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		pr_wa("Couldn't read APSD_STATUS_REG rc=%d\n", rc);
		return;
	}

	pr_dbg_wa("legacy(%d), done(%d), TO(%d), DCP(%d), Vbus(%d)\n",
		chg->typec_legacy, wa_rerun_apsd_done, stat, usb_type_dcp, usb_vbus_high);

	if (chg->typec_legacy && !wa_rerun_apsd_done
			&& (stat & HVDCP_CHECK_TIMEOUT_BIT) && usb_type_dcp && usb_vbus_high) {
		wa_rerun_apsd_done = true;
		schedule_delayed_work(&wa_rerun_apsd_for_dcp_dwork,
			round_jiffies_relative(msecs_to_jiffies(APSD_RERUN_DELAY_MS)));
	}
}

void wa_rerun_apsd_for_dcp_clear(void) {
	wa_rerun_apsd_done = false;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Charging without CC
////////////////////////////////////////////////////////////////////////////

/* CWC has two works for APSD and HVDCP, and this implementation handles the
 * works independently with different delay.
 * but you can see that retrying HVDCP detection work is depends on rerunning
 * APSD. i.e, APSD work derives HVDCP work.
 */
#define CWC_DELAY_MS  1000
static bool wa_charging_without_cc_processed = false;
static bool wa_charging_without_cc_required(struct smb_charger *chg) {
	union power_supply_propval val = { 0, };
	bool pd_hard_reset, usb_vbus_high, typec_mode_none,	wa_required;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return false;
	}

	pd_hard_reset = chg->pd_hard_reset;
	usb_vbus_high = !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val)
		? !!val.intval : false;
	typec_mode_none = chg->typec_mode == POWER_SUPPLY_TYPEC_NONE;

	wa_required = !pd_hard_reset && usb_vbus_high && typec_mode_none;
	if (!wa_required)
		pr_dbg_wa("Don't need CWC (pd_hard_reset:%d, usb_vbus_high:%d, typec_mode_none:%d)\n",
			pd_hard_reset, usb_vbus_high, typec_mode_none);

	return wa_required;
}

static void wa_charging_without_cc_main(struct work_struct *unused) {
	struct smb_charger*  chg = wa_helper_chg();

	if (wa_charging_without_cc_required(chg)) {
		pr_wa("CC line is not recovered until now, Start W/A\n");
		wa_charging_without_cc_processed = true;
		chg->typec_legacy = true;
		smblib_hvdcp_detect_enable(chg, true);
		smblib_rerun_apsd_if_required(chg);
		wa_command_icl_override(chg);
	}

	return;
}
static DECLARE_DELAYED_WORK(wa_charging_without_cc_dwork, wa_charging_without_cc_main);

void wa_charging_without_cc_trigger(struct smb_charger* chg, bool vbus) {
	// This may be triggered in the IRQ context of the USBIN rising.
	// So main function to start 'charging without cc', is deferred via delayed_work of kernel.
	// Just check and register (if needed) the work in this call.
	union power_supply_propval pval;

	if (vbus && wa_charging_without_cc_required(chg)) {
		if (delayed_work_pending(&wa_charging_without_cc_dwork)) {
			pr_wa(" Cancel the pended trying apsd . . .\n");
			cancel_delayed_work(&wa_charging_without_cc_dwork);
		}

		schedule_delayed_work(&wa_charging_without_cc_dwork,
			msecs_to_jiffies(CWC_DELAY_MS));
	} else if (!vbus && wa_charging_without_cc_processed) {
		wa_charging_without_cc_processed = false;
		cancel_delayed_work(&wa_charging_without_cc_dwork);
		pr_wa("Call typec_removal by force\n");
		extension_typec_src_removal(chg);

		pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		smblib_set_prop_typec_power_role(chg, &pval);
	}
}

bool wa_charging_without_cc_is_running(void) {
	return wa_charging_without_cc_processed;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Rerun apsd for unknown charger
////////////////////////////////////////////////////////////////////////////
static void wa_charging_for_unknown_cable_main(struct work_struct *unused);
static DECLARE_DELAYED_WORK(wa_charging_for_unknown_cable_dwork, wa_charging_for_unknown_cable_main);

#define FLOAT_SETTING_DELAY_MS	1000
static void wa_charging_for_unknown_cable_main(struct work_struct *unused) {
	struct smb_charger*  chg = wa_helper_chg();
	struct power_supply* veneer = power_supply_get_by_name("veneer");
	union power_supply_propval floated
		= { .intval = POWER_SUPPLY_TYPE_USB_FLOAT, };
	union power_supply_propval val = { 0, };
	bool pd_hard_reset, usb_type_unknown, moisture_detected, usb_vbus_high,
			workaround_required, apsd_done, typec_mode_sink, ok_to_pd;
	bool vbus_valid = false;
	u8 stat;
	int rc;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		goto out_charging_for_unknown;
	}

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		pr_wa("Couldn't read APSD_STATUS_REG rc=%d\n", rc);
		goto out_charging_for_unknown;
	}
	apsd_done = (stat & APSD_DTC_STATUS_DONE_BIT);

	if (!(*chg->lpd_ux)) {
		if (chg->lpd_reason == LPD_MOISTURE_DETECTED) {
			if (*chg->lpd_dpdm_disable) {
				floated.intval = POWER_SUPPLY_TYPE_USB_DCP;
			} else {
				floated.intval = POWER_SUPPLY_TYPE_USB;
			}
		}
	}

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		pr_wa("Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n", rc);
		goto out_charging_for_unknown;
	}
	typec_mode_sink = (stat & SNK_SRC_MODE_BIT);

	pd_hard_reset = chg->pd_hard_reset;
	usb_type_unknown = chg->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN;
	moisture_detected
		= !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_MOISTURE_DETECTED, &val)
		? (val.intval == POWER_SUPPLY_MOISTURE_DETECTED): false;
	usb_vbus_high
		= !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val)
		? true : false;
	ok_to_pd = chg->ok_to_pd;

	workaround_required = !pd_hard_reset
				&& !typec_mode_sink
				&& usb_type_unknown
				&& usb_vbus_high
				&& !ok_to_pd;

	if (*chg->lpd_ux)
		workaround_required = workaround_required && !moisture_detected;

	if (!workaround_required) {
		pr_dbg_wa("check(!(pd_hard_reset:%d, MD:%d, typec_mode_sink:%d)"
			" usb_type_unknown:%d, usb_vbus_high:%d, ok_to_pd:%d,"
			" apsd_done:%d wa_charging_cc:%d, pending work:%d)\n",
			pd_hard_reset, moisture_detected, typec_mode_sink,
			usb_type_unknown, usb_vbus_high, ok_to_pd, apsd_done,
			wa_charging_without_cc_processed,
			delayed_work_pending(&wa_charging_without_cc_dwork));

		goto out_charging_for_unknown;
	}

	if (apsd_done && !delayed_work_pending(&wa_charging_without_cc_dwork)) {
		rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG,
				USBIN_ADAPTER_ALLOW_5V);
		if (rc < 0) {
			pr_wa("Couldn't write 0x%02x to"
					" USBIN_ADAPTER_ALLOW_CFG_REG rc=%d\n",
					USBIN_ADAPTER_ALLOW_CFG_REG, rc);
			goto out_charging_for_unknown;
		}
		vbus_valid = !power_supply_get_property(chg->usb_psy,
				POWER_SUPPLY_PROP_PRESENT, &val)
				? !!val.intval : false;
		if (vbus_valid) {
			pr_wa("Force setting cable as FLOAT if it is UNKNOWN after APSD\n");
			power_supply_set_property(veneer,
					POWER_SUPPLY_PROP_REAL_TYPE, &floated);
			power_supply_changed(veneer);
		}
		else {
			pr_wa("VBUS is not valid\n");
		}
	}
	else {
		schedule_delayed_work(&wa_charging_for_unknown_cable_dwork,
			round_jiffies_relative(msecs_to_jiffies(FLOAT_SETTING_DELAY_MS)));
	}

out_charging_for_unknown:
	power_supply_put(veneer);
}

void wa_charging_for_unknown_cable_trigger(struct smb_charger* chg) {
	union power_supply_propval val = { 0, };
	bool usb_vbus_high
		= !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val)
		? true : false;
	if (usb_vbus_high
		&& (wa_charging_without_cc_processed ||
			delayed_work_pending(&wa_charging_without_cc_dwork))) {
		schedule_delayed_work(&wa_charging_for_unknown_cable_dwork,
			round_jiffies_relative(msecs_to_jiffies(FLOAT_SETTING_DELAY_MS)));
	}
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Support for weak battery pack
////////////////////////////////////////////////////////////////////////////

#define WEAK_SUPPLY_VOTER "WEAK_SUPPLY_VOTER"
#define WEAK_DELAY_MS		500
#define WEAK_DETECTION_COUNT	3
#define DEFAULT_WEAK_ICL_MA 1000

#define POWER_PATH_MASK		GENMASK(2, 1)
#define POWER_PATH_BATTERY	BIT(1)
#define POWER_PATH_USB		BIT(2)

static int  wa_support_weak_supply_count = 0;
static bool wa_support_weak_supply_running = false;

static void wa_support_weak_supply_func(struct work_struct *unused) {
	struct smb_charger* chg = wa_helper_chg();
	u8 stat;

	if (!wa_support_weak_supply_running)
		return;

	if (chg && !smblib_read(chg, POWER_PATH_STATUS_REG, &stat)) {
		if ((stat & POWER_PATH_MASK) == POWER_PATH_USB) {
			wa_support_weak_supply_count++;
			pr_wa("wa_support_weak_supply_count = %d\n",
				wa_support_weak_supply_count);
			if (wa_support_weak_supply_count >= WEAK_DETECTION_COUNT) {
				pr_wa("Weak battery is detected, set ICL to 1A\n");
				vote(chg->usb_icl_votable, WEAK_SUPPLY_VOTER,
					true, DEFAULT_WEAK_ICL_MA*1000);
			}
		}
	}
	wa_support_weak_supply_running = false;
}

static DECLARE_DELAYED_WORK(wa_support_weak_supply_dwork, wa_support_weak_supply_func);

void wa_support_weak_supply_trigger(struct smb_charger* chg, u8 stat) {
	bool trigger = !!(stat & USE_USBIN_BIT);

	if ((stat & POWER_PATH_MASK) != POWER_PATH_BATTERY)
		return;

	if (trigger) {
		if (!delayed_work_pending(&wa_support_weak_supply_dwork))
			schedule_delayed_work(&wa_support_weak_supply_dwork,
				round_jiffies_relative(msecs_to_jiffies(WEAK_DELAY_MS)));
	}
	else if (!!wa_support_weak_supply_count) {
		pr_wa("Clear wa_support_weak_supply_count\n");
		wa_support_weak_supply_count = 0;
		vote(chg->usb_icl_votable, WEAK_SUPPLY_VOTER, false, 0);
	}
	else
		; /* Do nothing */
}

void wa_support_weak_supply_check(void) {
	if (delayed_work_pending(&wa_support_weak_supply_dwork))
		wa_support_weak_supply_running = true;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Resuming Suspended USBIN
////////////////////////////////////////////////////////////////////////////

#define AICL_FAIL_BIT				BIT(1)
#define AICL_RERUN_TIME_MASK			GENMASK(1, 0)
#define AICL_RERUN_DELAY_MS 3500
#define INITIAL_DELAY_MS 5000

static bool wa_resuming_suspended_usbin_required(void) {
	// Checking condition in prior to recover usbin suspending
	struct smb_charger*  chg = wa_helper_chg();
	u8 reg_status_aicl, reg_status_powerpath, reg_status_rt;

	if (chg && (get_effective_result_locked(chg->usb_icl_votable) != 0)
			&& (smblib_read(chg, AICL_STATUS_REG, &reg_status_aicl) >= 0)
			&& (smblib_read(chg, POWER_PATH_STATUS_REG, &reg_status_powerpath) >= 0)
			&& (smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &reg_status_rt) >= 0) ) {
		pr_dbg_wa("AICL_STATUS_REG(0x%04x):0x%02x,"
			" POWER_PATH_STATUS_REG(0x%04x):0x%02x"
			" USB_INT_RT_STS(0x%04x):0x%02x\n",
			AICL_STATUS_REG, reg_status_aicl,
			POWER_PATH_STATUS_REG, reg_status_powerpath,
			USBIN_BASE + INT_RT_STS_OFFSET, reg_status_rt);

		if (reg_status_rt & USBIN_PLUGIN_RT_STS_BIT) {
			if ((reg_status_aicl & AICL_FAIL_BIT)
				|| (reg_status_powerpath & USBIN_SUSPEND_STS_BIT)) {
				pr_wa("AICL_FAIL:%d, USBIN_SUSPEND:%d\n",
					!!(reg_status_aicl & AICL_FAIL_BIT),
					!!(reg_status_powerpath & USBIN_SUSPEND_STS_BIT));
				return true;
			}
		}
		else
			pr_dbg_wa("[W/A] RSU-?) Skip because USB is not present\n");
	}

	return false;
}

static void wa_resuming_suspended_usbin_func(struct work_struct *unused) {
// 0. Local variables
	// References for charger driver
	struct smb_charger* chg = wa_helper_chg();
	union power_supply_propval val = { 0, };
	bool vbus = false;
	bool usb_type_unknown = false;
	int 		    irq = (chg && chg->irq_info) ? chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq : 0;
	// Buffer to R/W PMI register
	int		    ret;
	u8		    buf;

	// Previous values to be restored
	int pre_usbin_collapse;
	int pre_aicl_rerun_time;

	if (!chg || !wa_resuming_suspended_usbin_required()) {
		pr_wa("Exiting recovery for USBIN-suspend. (%p)\n", chg);
		return;
	}
	else {
		pr_wa("Start resuming suspended usbin\n");
	}

// 1. W/A to prevent the IRQ 'usbin-icl-change' storm (CN#03165535) on SDM845
	// : Before recovery USBIN-suspend, be sure that IRQ 'usbin-icl-change' is enabled.
	//   If not, this recovery will not work well due to the disabled AICL notification.
	// : To prevent IRQ 'usbin-icl-change' storm, it might be disabled in its own ISR.
	//   Refer to the disabling IRQ condition in 'smblib_handle_icl_change()'
	ret = smblib_read(chg, POWER_PATH_STATUS_REG, &buf);
	if (irq && ret >= 0 && (buf & USBIN_SUSPEND_STS_BIT) && !chg->usb_icl_change_irq_enabled) {
		enable_irq(irq);
		chg->usb_icl_change_irq_enabled = true;
		pr_wa("USBIN_SUSPEND_STS_BIT = High, Enable ICL-CHANGE IRQ\n");
	}
	else {
		pr_wa("irq_number=%d, irq_enabled=%d, read_return=%d, read_register=%d\n",
			irq, chg->usb_icl_change_irq_enabled, ret, buf);
	}

// 2. Toggling USBIN_CMD_IL_REG
	pr_wa("Toggling USBIN_CMD_IL_REG(0x1340[0]) := 1\n");
	if (smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, USBIN_SUSPEND_BIT) < 0) {
		pr_wa("Couldn't write suspend to USBIN_SUSPEND_BIT\n");
		goto failed;
	}

	pr_wa("Toggling USBIN_CMD_IL_REG(0x1340[0]) := 0\n");
	if (smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 0) < 0) {
		pr_wa("Couldn't write resume to USBIN_SUSPEND_BIT\n");
		goto failed;
	}

// 3. Save origial AICL configurations
	if (smblib_read(chg, USBIN_AICL_OPTIONS_CFG_REG /*0x1380*/, &buf) >= 0) {
		pre_usbin_collapse = buf & SUSPEND_ON_COLLAPSE_USBIN_BIT;
		pr_wa("USBIN_AICL_OPTIONS_CFG_REG=0x%02x, SUSPEND_ON_COLLAPSE_USBIN_BIT=0x%02x\n",
			buf, pre_usbin_collapse);
	}
	else {
		pr_wa("Couldn't read USBIN_AICL_OPTIONS_CFG_REG\n");
		goto failed;
	}

	if (smblib_read(chg, AICL_RERUN_TIME_CFG_REG /*0x1661*/, &buf) >= 0) {
		pre_aicl_rerun_time = buf & AICL_RERUN_TIME_MASK;
		pr_wa("AICL_RERUN_TIME_CFG_REG=0x%02x, AICL_RERUN_TIME_MASK=0x%02x\n",
			buf, pre_aicl_rerun_time);
	}
	else {
		pr_wa("Couldn't read AICL_RERUN_TIME_CFG_REG\n");
		goto failed;
	}

// 4. Set 0s to AICL configurationss
	pr_wa("Setting USBIN_AICL_OPTIONS(0x1380[7]) := 0x00\n");
	if (smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG, SUSPEND_ON_COLLAPSE_USBIN_BIT, 0) < 0) {
		pr_wa("Couldn't write USBIN_AICL_OPTIONS_CFG_REG\n");
		goto failed;
	}

	pr_wa("Setting AICL_RERUN_TIME_CFG_REG(0x1661[1:0]) := 0x00\n");
	if (smblib_masked_write(chg, AICL_RERUN_TIME_CFG_REG, AICL_RERUN_TIME_MASK, 0) < 0) {
		pr_wa("Couldn't write AICL_RERUN_TIME_CFG_REG\n");
		goto failed;
	}

// 5. Marginal delaying for AICL rerun
	pr_wa("Waiting more 3 secs . . .\n");
	msleep(AICL_RERUN_DELAY_MS);

// 6. Restore AICL configurations
	pr_wa("Restoring USBIN_AICL_OPTIONS(0x1380[7]) := 0x%02x\n", pre_usbin_collapse);
	if (smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG, SUSPEND_ON_COLLAPSE_USBIN_BIT,
			pre_usbin_collapse) < 0) {
		pr_wa("Couldn't write USBIN_AICL_OPTIONS_CFG_REG\n");
		goto failed;
	}

	pr_wa("Restoring AICL_RERUN_TIME_CFG_REG(0x1661[1:0]) := 0x%02x\n", pre_aicl_rerun_time);
	if (smblib_masked_write(chg, AICL_RERUN_TIME_CFG_REG, AICL_RERUN_TIME_MASK,
			pre_aicl_rerun_time) < 0) {
		pr_wa("Couldn't write AICL_RERUN_TIME_CFG_REG\n");
		goto failed;
	}

// 7. If USBIN suspend is not resumed even with rerunning AICL, recover it from APSD.
	msleep(APSD_RERUN_DELAY_MS);
	vbus = !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val) ? !!val.intval : false;
	usb_type_unknown = chg->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN;

	if (wa_resuming_suspended_usbin_required() || (vbus && usb_type_unknown)) {
		pr_wa("Recover USBIN from APSD\n");
		wa_command_apsd(chg);
	}
	else
		pr_wa("Success resuming suspended usbin\n");

	return;
failed:
	pr_wa("Error on resuming suspended usbin\n");
}

static DECLARE_DELAYED_WORK(wa_resuming_suspended_usbin_dwork, wa_resuming_suspended_usbin_func);

void wa_resuming_suspended_usbin_trigger(struct smb_charger* chg) {

	if (!wa_resuming_suspended_usbin_required()) {
		pr_wa(" Exiting recovery for USBIN-suspend.\n");
		return;
	}

	// Considering burst aicl-fail IRQs, previous wa works will be removed,
	// to make this trigger routine handle the latest aicl-fail
	if (delayed_work_pending(&wa_resuming_suspended_usbin_dwork)) {
		pr_wa("Cancel the pending resuming work . . .\n");
		cancel_delayed_work(&wa_resuming_suspended_usbin_dwork);
	}

	schedule_delayed_work(&wa_resuming_suspended_usbin_dwork,
		round_jiffies_relative(msecs_to_jiffies(INITIAL_DELAY_MS)));
}

void wa_resuming_suspended_usbin_clear(void) {
	cancel_delayed_work(&wa_resuming_suspended_usbin_dwork);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Charging with Rd-open charger.
////////////////////////////////////////////////////////////////////////////

static bool wa_charging_with_rd_running = false;
void wa_charging_with_rd_trigger(struct smb_charger *chg) {
	union power_supply_propval val = { 0, };
	bool vbus = !power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val) ? !!val.intval : false;
	bool non_src = (chg->sink_src_mode != SRC_MODE);
	bool cc_sink = (chg->typec_mode == POWER_SUPPLY_TYPEC_SINK);

	if (vbus && non_src && cc_sink) {
		wa_charging_with_rd_running = true;
		wa_command_icl_override(chg);
		pr_wa("Set icl override for rd open charger.\n");
	}
}

void wa_charging_with_rd_clear(struct smb_charger* chg) {
	union power_supply_propval pval;

	wa_charging_with_rd_running = false;
	if (chg->typec_mode == POWER_SUPPLY_TYPEC_SINK ) {
		pr_wa("Set TYPEC_PR_DUAL for rd open charger \n");
		pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		// Clear cc status with Rd-open charger.
		smblib_set_prop_typec_power_role(chg, &pval);
	}
}

bool wa_charging_with_rd_is_running(void) {
	return wa_charging_with_rd_running;
}

////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Clear DC Reverse Voltage status
////////////////////////////////////////////////////////////////////////////

#define INT_ABNORMAL_OFFSET	0xe6
#define DC_IN_EN_OVERRIDE	BIT(1)
void wa_clear_dc_reverse_volt_trigger(bool enable) {
	struct smb_charger* chg = wa_helper_chg();
	u8 status;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	if (enable) {
		smblib_masked_write(chg, DCIN_CMD_IL_REG, DC_IN_EN_OVERRIDE, 0);
	} else {
		smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &status);
		if (status == INT_ABNORMAL_OFFSET) {
			pr_wa("wa_clear_dc_reverse_volt_trigger write!\n");
			smblib_masked_write(chg, DCIN_CMD_IL_REG,
				DC_IN_EN_OVERRIDE, DC_IN_EN_OVERRIDE);
		}
	}
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : DCIN AICL logic
////////////////////////////////////////////////////////////////////////////

#define DCIN_UV_RT_STS_BIT	BIT(2)
#define DC_AICL_VOTER		"DC_AICL_VOTER"
#define DC_ICL_GRINDER_UA	50000
#define DC_ICL_MIN_UA		100000
#define AICL_DELAY_MS		5000
#define STATUS_BOOT_MS		20000

static void wa_dcin_restore_aicl_func(struct work_struct *unused);
static DECLARE_DELAYED_WORK(wa_dcin_restore_aicl_dwork, wa_dcin_restore_aicl_func);

static void wa_dcin_restore_aicl_func(struct work_struct *unused) {
	struct smb_charger* chg = wa_helper_chg();
	int dc_icl = 0;
	u8 stat = 0;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	if (!chg->dc_icl_votable) {
		pr_wa("'dc_icl_votable' is not ready\n");
		return;
	}

	dc_icl = get_effective_result(chg->dc_icl_votable);
	if (!dc_icl) {
		pr_wa("'dc_icl' is not ready\n");
		return;
	}

	if (!smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &stat)
		&& !(stat & DCIN_PLUGIN_RT_STS_BIT)) {
		pr_wa("'DCIN_PLUGIN_RT_STS_BIT' is not set\n");
		return;
	}

	if (dc_icl >= get_client_vote(chg->dc_icl_votable, DC_AICL_VOTER)) {
		dc_icl += DC_ICL_GRINDER_UA;
		vote(chg->dc_icl_votable, DC_AICL_VOTER, true, dc_icl);
	}

	if (delayed_work_pending(&wa_dcin_restore_aicl_dwork))
		cancel_delayed_work(&wa_dcin_restore_aicl_dwork);

	if (!smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &stat)
		&& (stat & DCIN_PLUGIN_RT_STS_BIT))
		schedule_delayed_work(&wa_dcin_restore_aicl_dwork,
			round_jiffies_relative(msecs_to_jiffies(AICL_DELAY_MS)));
}

static void wa_dcin_start_aicl_func(struct work_struct *unused) {
	struct smb_charger* chg = wa_helper_chg();
	int i = 0, value = 0 , dc_icl = 0;
	u8 stat = 0;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	if (!chg->dc_icl_votable) {
		pr_wa("'dc_icl_votable' is not ready\n");
		return;
	}

	dc_icl = get_effective_result(chg->dc_icl_votable);
	if (!dc_icl) {
		pr_wa("'dc_icl' is not ready\n");
		return;
	}

	for (i = 1; i <= (dc_icl / DC_ICL_GRINDER_UA); i++) {
		value = i * DC_ICL_GRINDER_UA;
		smblib_set_charge_param(chg, &chg->param.dc_icl, value);

		usleep_range(1000, 2000);

		if (!smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &stat)) {
			if (!(stat & DCIN_PLUGIN_RT_STS_BIT)) {
				pr_wa("INT_RT = 0x%02x ==> NO DC PLUGIN\n", stat);
				smblib_set_charge_param(chg, &chg->param.dc_icl, dc_icl);
				return;
			}
			if (stat & DCIN_UV_RT_STS_BIT) {
				pr_wa("INT_RT = 0x%02x ==> End DC_ICL AICL\n", stat);
				break;
			}
		}
	}

	if (dc_icl != value)
		vote(chg->dc_icl_votable, DC_AICL_VOTER, true, value);

	if (delayed_work_pending(&wa_dcin_restore_aicl_dwork))
		cancel_delayed_work(&wa_dcin_restore_aicl_dwork);

	schedule_delayed_work(&wa_dcin_restore_aicl_dwork,
		round_jiffies_relative(msecs_to_jiffies(AICL_DELAY_MS)));
}
static DECLARE_WORK(wa_dcin_start_aicl_work, wa_dcin_start_aicl_func);

void wa_dcin_start_aicl_trigger(void) {
	//schedule_work(&wa_dcin_start_aicl_work);
}

static void wa_clear_status_boot_func(struct work_struct *unused) {
	char buf[2] = { 0, };
	int boot_completed = 0;

	if (unified_nodes_show("status_boot", buf)
		&& sscanf(buf, "%d", &boot_completed) && boot_completed == 0) {
		unified_nodes_store("status_boot", "1", 1);
	}
}
static DECLARE_DELAYED_WORK(wa_clear_status_boot_dwork, wa_clear_status_boot_func);

static void wa_dcin_uv_aicl_func(struct work_struct *unused) {
	struct smb_charger* chg = wa_helper_chg();
	char buf[2] = { 0, };
	int boot_completed = 0;
	int dc_icl = 0;

	if (unified_nodes_show("status_boot", buf)
		&& sscanf(buf, "%d", &boot_completed) && boot_completed == 0) {
		struct power_supply* wireless_psy = power_supply_get_by_name("wireless");
		union power_supply_propval value = { .intval = 1, };

		if (!wireless_psy) {
			pr_wa("'wireless_psy' is not ready\n");
		} else {
			power_supply_set_property(wireless_psy,
				POWER_SUPPLY_PROP_INPUT_SUSPEND, &value);
			power_supply_put(wireless_psy);

			schedule_delayed_work(&wa_clear_status_boot_dwork,
				round_jiffies_relative(msecs_to_jiffies(STATUS_BOOT_MS)));
		}
		return;
	}

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	if (!chg->dc_icl_votable) {
		pr_wa("'dc_icl_votable' is not ready\n");
		return;
	}

	dc_icl = get_effective_result(chg->dc_icl_votable);
	if (!dc_icl) {
		pr_wa("'dc_icl' is not ready\n");
		return;
	}

	if (dc_icl > DC_ICL_MIN_UA) {
		dc_icl -= DC_ICL_GRINDER_UA;
		vote(chg->dc_icl_votable, DC_AICL_VOTER, true, dc_icl);
	}

	schedule_delayed_work(&wa_dcin_restore_aicl_dwork,
		round_jiffies_relative(msecs_to_jiffies(AICL_DELAY_MS)));
}
static DECLARE_WORK(wa_dcin_uv_aicl_work, wa_dcin_uv_aicl_func);

void wa_dcin_uv_aicl_trigger(void) {
	if (delayed_work_pending(&wa_dcin_restore_aicl_dwork))
		cancel_delayed_work(&wa_dcin_restore_aicl_dwork);

	schedule_work(&wa_dcin_uv_aicl_work);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Recovery vashdn during wireless charging
////////////////////////////////////////////////////////////////////////////

#define VASHDN_DELAY_MS		3000
#define DCIN_VASHDN_RT_STS	BIT(1)
static void wa_recovery_vashdn_wireless_func(struct work_struct *unused) {
	struct smb_charger* chg = wa_helper_chg();
	struct power_supply* wireless_psy = power_supply_get_by_name("wireless");
	union power_supply_propval val = { .intval = 0, };
	u8 stat;

	if (!wireless_psy) {
		pr_wa("'wireless_psy' is not ready\n");
		return;
	}

	if (chg) {
		int dc_present = !smblib_get_prop_dc_present(chg, &val) ? val.intval : 0;
		int dc_online = !smblib_get_prop_dc_online(chg, &val) ? val.intval : 0;
		bool dc_vashdn = !smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &stat)
			? (stat & DCIN_VASHDN_RT_STS) : 0;
		bool dc_pause = !smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat)
			? (stat & PAUSE_CHARGE) : 0;

		if (dc_present && !dc_online && dc_vashdn && dc_pause) {
			pr_wa("detection Vashdn wireless charging stop!\n");
			val.intval = 2;
			power_supply_set_property(wireless_psy,
				POWER_SUPPLY_PROP_DEBUG_BATTERY, &val);
		}
	}
	power_supply_put(wireless_psy);
}
static DECLARE_DELAYED_WORK(wa_recovery_vashdn_wireless_dwork, wa_recovery_vashdn_wireless_func);

void wa_recovery_vashdn_wireless_trigger(void) {
	if (delayed_work_pending(&wa_recovery_vashdn_wireless_dwork))
		cancel_delayed_work(&wa_recovery_vashdn_wireless_dwork);

	schedule_delayed_work(&wa_recovery_vashdn_wireless_dwork,
		round_jiffies_relative(msecs_to_jiffies(VASHDN_DELAY_MS)));
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Retry to enable vconn on vconn-oc
////////////////////////////////////////////////////////////////////////////

#define VCONN_MAX_ATTEMPTS	3
#define MAX_OC_FALLING_TRIES 10
static int  wa_vconn_attempts = 0;

static void wa_retry_vconn_enable_on_vconn_oc_func(struct work_struct *unused) {
	struct smb_charger* chg = wa_helper_chg();
	static DEFINE_MUTEX(wa_vconn_oc_lock);
	int rc, i;
	u8 stat;

	pr_wa("over-current detected on VCONN\n");
	if (!chg || !chg->vconn_vreg || !chg->vconn_vreg->rdev)
		return;

	mutex_lock(&wa_vconn_oc_lock);
	if (wa_connected_dual_display(chg))
		rc = smblib_vconn_regulator_disable(chg->vconn_vreg->rdev);
	else
		rc = override_vconn_regulator_disable(chg->vconn_vreg->rdev);
	if (rc < 0) {
		pr_wa("Couldn't disable VCONN rc=%d\n", rc);
		goto unlock;
	}

	if (++wa_vconn_attempts > VCONN_MAX_ATTEMPTS) {
		pr_wa("VCONN failed to enable after %d attempts\n",
			   wa_vconn_attempts - 1);
		wa_vconn_attempts = 0;
		goto unlock;
	}

	/*
	 * The real time status should go low within 10ms. Poll every 1-2ms to
	 * minimize the delay when re-enabling OTG.
	 */
	for (i = 0; i < MAX_OC_FALLING_TRIES; ++i) {
		usleep_range(1000, 2000);
		rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
		if (rc >= 0 && !(stat & TYPEC_VCONN_OVERCURR_STATUS_BIT))
			break;
	}

	if (i >= MAX_OC_FALLING_TRIES) {
		pr_wa("VCONN OC did not fall after %dms\n",
						2 * MAX_OC_FALLING_TRIES);
		wa_vconn_attempts = 0;
		goto unlock;
	}
	pr_wa("VCONN OC fell after %dms\n", 2 * i + 1);

	rc = override_vconn_regulator_enable(chg->vconn_vreg->rdev);
	if (rc < 0) {
		pr_wa("Couldn't enable VCONN rc=%d\n", rc);
		goto unlock;
	}

unlock:
	mutex_unlock(&wa_vconn_oc_lock);
}
static DECLARE_WORK(wa_retry_vconn_enable_on_vconn_oc_work, wa_retry_vconn_enable_on_vconn_oc_func);

void wa_retry_vconn_enable_on_vconn_oc_trigger(struct smb_charger* chg) {
	u8 stat;
	int rc;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		pr_wa("Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n", rc);
		return;
	}

	if (stat & TYPEC_VCONN_OVERCURR_STATUS_BIT)
		schedule_work(&wa_retry_vconn_enable_on_vconn_oc_work);
}

void wa_retry_vconn_enable_on_vconn_oc_clear(struct smb_charger* chg) {
	int rc;

	if (!chg || !chg->vconn_vreg->rdev)
		return;

	if(smblib_vconn_regulator_is_enabled(chg->vconn_vreg->rdev)
			&& chg->typec_mode == POWER_SUPPLY_TYPEC_NONE
			&& wa_vconn_attempts != 0) {
		rc = override_vconn_regulator_disable(chg->vconn_vreg->rdev);
		if (rc < 0) {
			pr_wa("Couldn't disable VCONN rc=%d\n", rc);
			return;
		}
	}
	wa_vconn_attempts = 0;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Retry pd check
////////////////////////////////////////////////////////////////////////////

static bool wa_retry_ok_to_pd = false;
void wa_retry_ok_to_pd_trigger(struct smb_charger* chg) {
	bool is_usb = chg->real_charger_type == POWER_SUPPLY_TYPE_USB
		|| chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOAT;
	bool is_high = chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_HIGH;

	if (!wa_retry_ok_to_pd && is_usb && is_high) {
		chg->ok_to_pd = !(*chg->pd_disabled) && !chg->pd_not_supported;
		wa_retry_ok_to_pd = true;
		pr_wa("retry ok_to_pd = %d\n", chg->ok_to_pd);
		power_supply_changed(chg->usb_psy);
	}
}

void wa_retry_ok_to_pd_clear(struct smb_charger* chg) {
	wa_retry_ok_to_pd = false;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Supporting USB Compliance test
////////////////////////////////////////////////////////////////////////////

static bool wa_usb_compliance_mode = false;
#define MICRO_2P5A			2500000

void wa_set_usb_compliance_mode(bool mode) {
	struct smb_charger* chg = wa_helper_chg();
	int rc;

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	wa_usb_compliance_mode = mode;
	if (wa_usb_compliance_mode) {
		/* set OTG current limit */
		rc = smblib_set_charge_param(chg, &chg->param.otg_cl, MICRO_2P5A);
		if (rc < 0) {
			pr_err("Couldn't set otg current limit rc=%d on compliance mode\n", rc);
			return;
		}

	} else {
		/* set OTG current limit */
		rc = smblib_set_charge_param(chg, &chg->param.otg_cl, chg->otg_cl_ua);
		if (rc < 0) {
			pr_err("Couldn't set otg current limit rc=%d on non-compliance mode\n", rc);
			return;
		}
	}
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : avoid over voltage by abnormal charger
////////////////////////////////////////////////////////////////////////////
#define CHG_TERM_WA_ENTRY_DELAY_MS     300000    /* 5 min */
#define CHG_TERM_WA_EXIT_DELAY_MS      60000     /* 1 min */
#define INPUT_PRESENT_USB	BIT(1)
#define INPUT_PRESENT_DC	BIT(2)
int wa_protect_overcharging(struct smb_charger* chg, int input_present)
{
	union power_supply_propval val = { 0, };
	int batt_vol = -EINVAL, upper_border = -EINVAL, lower_border = -EINVAL;
	int delay = CHG_TERM_WA_ENTRY_DELAY_MS;
	u8 stat = 0;
	int rc = 0;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		pr_err("Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n", rc);
		return CHG_TERM_WA_ENTRY_DELAY_MS;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (!smblib_get_prop_from_bms(chg,
			POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &val)) {
		upper_border = val.intval + chg->ext_chg.prot_overchg_ent_dischg_off;
		lower_border = val.intval - chg->ext_chg.prot_overchg_rel_off;
		if (stat == FULLON_CHARGE || stat == TAPER_CHARGE)
			upper_border = val.intval + chg->ext_chg.prot_overchg_ent_chg_off;
	}

	if (!smblib_get_prop_from_bms(chg,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &val))
		batt_vol = val.intval;

	if (batt_vol < lower_border) {
		vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
		vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER, false, 0);
	} else if (batt_vol >= upper_border) {
		if (input_present & INPUT_PRESENT_USB)
			vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER,
					true, 0);
		if (input_present & INPUT_PRESENT_DC)
			vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER,
					true, 0);
	}

	if (is_client_vote_enabled(chg->usb_icl_votable, CHG_TERMINATION_VOTER)
		|| is_client_vote_enabled(chg->dc_suspend_votable, CHG_TERMINATION_VOTER))
		delay = CHG_TERM_WA_EXIT_DELAY_MS;
	else
		delay = CHG_TERM_WA_ENTRY_DELAY_MS;

	pr_wa("batt = %d (lower: %d, upper: %d), delay = %d min\n",
		batt_vol, lower_border, upper_border, delay/CHG_TERM_WA_EXIT_DELAY_MS);
	return delay;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Voltags Drop for EOC.
////////////////////////////////////////////////////////////////////////////

static bool wa_drop_vbus_on_eoc_running = false;
void wa_drop_vbus_on_eoc_trigger(struct smb_charger* chg) {
	union power_supply_propval val = { 0, };

	smblib_get_prop_batt_charge_done(chg, &val);
	if (val.intval) {
		if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP) {
			wa_drop_vbus_on_eoc_running = true;
			pr_wa("Vbus is set to 5V on HVDCP2\n");
			val.intval = POWER_SUPPLY_DP_DM_FORCE_5V;
			power_supply_set_property(chg->batt_psy,
				POWER_SUPPLY_PROP_DP_DM, &val);
		} else if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3
				&& chg->pulse_cnt) {
			wa_drop_vbus_on_eoc_running = true;
			pr_wa("Vbus is dropped on HVDCP3 (%d)\n", chg->pulse_cnt);
			val.intval = POWER_SUPPLY_DP_DM_DM_PULSE;
			power_supply_set_property(chg->batt_psy,
				POWER_SUPPLY_PROP_DP_DM, &val);
		} else {
			wa_drop_vbus_on_eoc_running = false;
		}
	} else {
		wa_drop_vbus_on_eoc_running = false;
	}
}

void wa_drop_vbus_on_eoc_required(struct smb_charger* chg) {
	union power_supply_propval val = { 0, };

	if (wa_drop_vbus_on_eoc_running
			&& chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3
			&& chg->pulse_cnt) {
		pr_wa("Vbus is dropped on HVDCP3 (%d)\n", chg->pulse_cnt);
		val.intval = POWER_SUPPLY_DP_DM_DM_PULSE;
		power_supply_set_property(chg->batt_psy,
			POWER_SUPPLY_PROP_DP_DM, &val);
	}
}

void wa_drop_vbus_on_eoc_clear(void) {
	wa_drop_vbus_on_eoc_running = false;
}

bool wa_drop_vbus_on_eoc_is_running(void) {
	return wa_drop_vbus_on_eoc_running;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Disable hvdcp on Factory cable.
////////////////////////////////////////////////////////////////////////////

static bool disabled_hvdcp = false;
static bool is_usb_configured = false;
void wa_update_usb_configured(bool configured) {
	is_usb_configured = configured;
}

void wa_disable_hvdcp_with_factory_trigger(struct smb_charger *chg) {
	if (unified_bootmode_usermode())
		return;

	if (is_usb_configured && !disabled_hvdcp
			&& chg->real_charger_type == POWER_SUPPLY_TYPE_USB_DCP
			&& chg->typec_mode == POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY) {
		disabled_hvdcp = true;
		smblib_hvdcp_detect_enable(chg, false);
		pr_wa("APSD rerun\n");
		wa_command_apsd(chg);
	}
}

void wa_disable_hvdcp_with_factory_clear(struct smb_charger *chg) {
	disabled_hvdcp = false;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Disable AICL on rp-rp cable.
////////////////////////////////////////////////////////////////////////////

#define TYPE_C_DEBUG_ACCESS_SNK_CFG_REG			(TYPEC_BASE + 0x4A)
#define DAM_DIS_AICL_BIT				BIT(3)
static bool wa_avoid_src_dbg_cable_running = false;
void wa_avoid_src_dbg_cable_trigger(struct smb_charger* chg) {
	if (chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_DEBUG_ACCESSORY_DEFAULT) {
		smblib_masked_write(chg, TYPE_C_DEBUG_ACCESS_SNK_CFG_REG, DAM_DIS_AICL_BIT, 0);
		pr_wa("Disable Aicl on DAM\n");
		wa_avoid_src_dbg_cable_running = true;
	}
}

void wa_avoid_src_dbg_cable_clear(struct smb_charger* chg) {
	if (wa_avoid_src_dbg_cable_running) {
		smblib_masked_write(chg, TYPE_C_DEBUG_ACCESS_SNK_CFG_REG,
			DAM_DIS_AICL_BIT, DAM_DIS_AICL_BIT);
		wa_avoid_src_dbg_cable_running = false;
	}

}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Recover & Fake CC status in factory mode.
////////////////////////////////////////////////////////////////////////////

#define RECOVER_CC_DELAY_MS 100
#define MAX_RECOVER_CC_ATTEMPTS 3
static int  wa_recover_cc_attempts = 0;

static void wa_recover_cc_status_func(struct work_struct *unused);
static DECLARE_DELAYED_WORK(wa_recover_cc_status_dwork, wa_recover_cc_status_func);

static void wa_recover_cc_status_func(struct work_struct *unused) {
	struct smb_charger* chg = wa_helper_chg();
	union power_supply_propval pval = {0, };

	if (!chg) {
		pr_wa("'chg' is not ready\n");
		return;
	}

	if (chg->typec_mode != POWER_SUPPLY_TYPEC_SINK) {
		pr_wa("Stop to recover cc status, because it isn't rd-open\n");
		wa_recover_cc_attempts = 0;
		return;
	}

	pr_wa("Recover CC status by factory cable's error\n");
	pval.intval = POWER_SUPPLY_TYPEC_PR_NONE;
	// Clear cc status with Rd-open charger.
	smblib_set_prop_typec_power_role(chg, &pval);
	msleep(10);

	pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
	// Clear cc status with Rd-open charger.
	smblib_set_prop_typec_power_role(chg, &pval);

	if (wa_recover_cc_attempts < MAX_RECOVER_CC_ATTEMPTS) {
		schedule_delayed_work(&wa_recover_cc_status_dwork, msecs_to_jiffies(RECOVER_CC_DELAY_MS));
		wa_recover_cc_attempts++;
	} else {
		wa_recover_cc_attempts = 0;
	}
}

void wa_recover_cc_status_trigger(struct smb_charger *chg) {
	if (!unified_bootmode_usermode() && !delayed_work_pending(&wa_recover_cc_status_dwork))
		schedule_delayed_work(&wa_recover_cc_status_dwork, msecs_to_jiffies(RECOVER_CC_DELAY_MS));
}

bool wa_fake_cc_status_is_runnging(struct smb_charger *chg) {
	bool ret = false;
	union power_supply_propval	val = {0, };

	if (unified_bootmode_usermode() || chg->typec_mode != POWER_SUPPLY_TYPEC_SINK)
		return ret;

	if (!smblib_get_prop_usb_present(chg, &val) && val.intval)
		ret = true;

	return ret;
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Support DS2 with Rd/Open.
////////////////////////////////////////////////////////////////////////////

static int ds_state = LGE_PRM_DS_CONNECTION_OFF;
bool wa_need_to_load_sw_on(struct smb_charger *chg) {
	int rc = 0;
	u8 stat = 0;

	if (!chg->load_sw_on_gpio)
		return false;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		pr_wa("Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n", rc);
		return rc;
	}

	if (((chg->typec_mode == POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE && (stat & CC_ORIENTATION_BIT))
			|| chg->typec_mode == POWER_SUPPLY_TYPEC_SINK
			|| chg->typec_mode == POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY)
			&& wa_connected_dual_display(chg))
		return true;
	return false;
}

bool wa_connected_dual_display(struct smb_charger *chg) {
	if (!chg->load_sw_on_gpio)
		return false;

	if (ds_state == LGE_PRM_DS2_CONNECTION_ON
			|| ds_state == LGE_PRM_HALLIC_CONNECTION_ON)
		return true;
	else
		return false;
}

void wa_support_dual_display_trigger(struct smb_charger *chg) {
	union power_supply_propval val = { 0, };
	int now_ds_state = lge_prm_get_info(LGE_PRM_INFO_DS_STATE);
	int rc;
	u8 stat;
	bool vbus, hvdcp_timeout, is_ocp;
	static bool recheck_sink = false;
	static bool recheck_source = false;
	static bool pre_pd_active = false;

	if (!chg->load_sw_on_gpio)
		return;

	if (now_ds_state != ds_state) {
		ds_state = now_ds_state;
#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
		if (chg->lpd_reason == LPD_MOISTURE_DETECTED)
			schedule_work(&chg->lpd_recheck_work);
#endif
	}

	if (wa_need_to_load_sw_on(chg)) {
		if (smblib_vconn_regulator_is_enabled(chg->vconn_vreg->rdev) &&
				!gpiod_get_value(chg->load_sw_on_gpio)) {
			pr_wa("re-enabling VCONN by external LDO.\n");
			override_vconn_regulator_enable(chg->vconn_vreg->rdev);
		}

		vbus = !smblib_get_prop_usb_present(chg, &val) ? !!val.intval : false;

		rc = smblib_read(chg, APSD_STATUS_REG, &stat);
		if (rc < 0) {
			pr_wa("Couldn't read APSD_STATUS_REG rc=%d\n", rc);
			return;
		}
		hvdcp_timeout = stat & HVDCP_CHECK_TIMEOUT_BIT;

		rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
		if (rc < 0) {
			pr_wa("Couldn't read APSD_RESULT_STATUS_REG rc=%d\n", rc);
			return;
		}
		is_ocp = stat & OCP_CHARGER_BIT;

		if ((chg->real_charger_type != POWER_SUPPLY_TYPE_USB_DCP
				|| hvdcp_timeout || chg->hvdcp_disable || is_ocp)
				&& vbus) {
			pr_dbg_wa("Override ICL for DS2.\n");
			wa_command_icl_override(chg);
		}

		if (!chg->pd_active && pre_pd_active
				&& chg->real_charger_type == POWER_SUPPLY_TYPE_USB_DCP) {
			pr_wa("Rerun apsd\n");
			wa_command_apsd(chg);
		}
	}

	if (!recheck_sink && chg->early_usb_attach
			&& chg->typec_mode == POWER_SUPPLY_TYPEC_SINK) {
		pr_wa("re-check cc pin of sink for DS2.\n");
		recheck_sink = true;
		val.intval = POWER_SUPPLY_TYPEC_PR_NONE;
		smblib_set_prop_typec_power_role(chg, &val);
		val.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		smblib_set_prop_typec_power_role(chg, &val);
	}

	if (!recheck_source && ds_state == LGE_PRM_HALLIC_CONNECTION_ON
			&& chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT) {
		pr_wa("re-check cc pin of source for DS2.\n");
		recheck_source = true;
		val.intval = POWER_SUPPLY_TYPEC_PR_SOURCE;
		smblib_set_prop_typec_power_role(chg, &val);
		val.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		smblib_set_prop_typec_power_role(chg, &val);
	} else if (!smblib_get_prop_usb_present(chg, &val) && !val.intval){
		recheck_source = false;
	}

	pre_pd_active = !!chg->pd_active;
}

int wa_dual_display_vconn_enable(struct smb_charger *chg) {
	int ret = 0;

	if (wa_need_to_load_sw_on(chg)) {
		pr_wa("enabling VCONN by external LDO with cover\n");
		gpiod_direction_output(chg->load_sw_on_gpio, 1);
		smblib_masked_write(chg, TYPE_C_VCONN_CONTROL_REG,
			VCONN_EN_ORIENTATION_BIT | VCONN_EN_VALUE_BIT | VCONN_EN_SRC_BIT,
			VCONN_EN_VALUE_BIT | VCONN_EN_SRC_BIT);
	} else {
		ret = smblib_vconn_regulator_enable(chg->vconn_vreg->rdev);
	}

	return ret;
}

void wa_dual_display_vconn_disable(struct smb_charger *chg) {
	if (chg->load_sw_on_gpio)
		gpiod_direction_output(chg->load_sw_on_gpio, 0);
}

void wa_support_dual_display_clear(struct smb_charger *chg) {
	if (!chg->load_sw_on_gpio)
		return;

	if (wa_connected_dual_display(chg)) {
		extension_typec_src_removal(chg);
		smblib_hvdcp_detect_enable(chg, true);
	}
}

////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Charging for mcdodo
////////////////////////////////////////////////////////////////////////////

#define WA_MCDODO_VOTER			"WA_MCDODO_VOTER"
void wa_charging_for_mcdodo_trigger(struct smb_charger *chg) {
	static bool pre_floating = false;
	bool is_floating = (chg->lpd_reason == LPD_FLOATING_CABLE);
	bool vbus;
	union power_supply_propval val = { 0, };

	if (pre_floating != is_floating) {
		vbus = !smblib_get_prop_usb_present(chg, &val) ? !!val.intval : false;
		if (is_floating && !vbus && chg->typec_mode == POWER_SUPPLY_TYPEC_NONE) {
			vote(chg->usb_icl_votable, WA_MCDODO_VOTER, true, 0);
		} else {
			wa_charging_for_mcdodo_clear(chg);
		}
		pre_floating = is_floating;
	}
}

void wa_charging_for_mcdodo_clear(struct smb_charger *chg) {
	vote(chg->usb_icl_votable, WA_MCDODO_VOTER, false, 0);
}


////////////////////////////////////////////////////////////////////////////
// LGE Workaround : Charging for port forwarding of KT .
////////////////////////////////////////////////////////////////////////////

static bool supported_pf = false;
static bool connected_pf = false;

bool wa_connected_port_forwarding(void) {
	return lge_prm_get_info(LGE_PRM_INFO_DS_STATE) == LGE_PRM_HALLIC_CONNECTION_ON
		&& supported_pf
		&& connected_pf;
}
EXPORT_SYMBOL(wa_connected_port_forwarding);

void wa_supported_port_forwarding(struct smb_charger *chg) {
	struct device_node *node = chg->dev->of_node;
	char property [32];
	int i;

	strncpy(property, "lge,supported-pf-", 32);
	strncat(property, unified_bootmode_operator(), 32);
	for (i = 0; i < strlen(property); i++)
		property[i] = tolower(property[i]);

	pr_dbg_wa("property = %s\n", property);
	pr_wa("property = %s\n", property);

	if (of_property_read_bool(node, property)) {
		supported_pf = true;
		pr_wa("Supported port forwarding.\n");
	}
}

void wa_update_port_forwarding(bool connected) {
	if (connected_pf != connected) {
		pr_wa("%s port forwarding.\n", connected ? "Connected" : "Disconnected");
		connected_pf = connected;
	}
}
EXPORT_SYMBOL(wa_update_port_forwarding);
