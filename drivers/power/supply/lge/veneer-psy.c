/*
 * Veneer PSY
 * Copyright (C) 2017 LG Electronics, Inc
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) "VENEER: %s: " fmt, __func__
#define pr_veneer(fmt, ...) pr_err(fmt, ##__VA_ARGS__)
#define pr_dbg_veneer(fmt, ...) pr_debug(fmt, ##__VA_ARGS__)

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/of_batterydata.h>
#include <linux/platform_device.h>

#include "veneer-primitives.h"

#include <linux/pm_qos.h>

#define VENEER_NAME		"veneer"
#define VENEER_COMPATIBLE	"lge,veneer-psy"
#define VENEER_DRIVER		"lge-veneer-psy"

#define VENEER_WAKELOCK 	VENEER_NAME": charging"
#define VENEER_NOTREADY		INT_MAX

static int debug_polling_time = 30000;
module_param_named(
	debug_polling_time, debug_polling_time, int, 0600
);

struct veneer {
/* module descripters */
	struct device*		veneer_dev;
	struct power_supply*	veneer_psy;
	struct wakeup_source	veneer_wakelock;
	enum charging_supplier	veneer_supplier;
	int			veneer_exception;
	// delayed works
	struct delayed_work	dwork_logger;	// for logging
	struct delayed_work	dwork_slowchg;	// for charging-verbosity
	// battery profiles
	int			profile_mincap;
	int			profile_fullraw;
	int			profile_mvfloat;
	// highspeed threshold
	int			threshold_wired;
	int			threshold_wireless;

/* shadow states used for cache */
	// charger states
	enum power_supply_type	usbin_realtype;
	bool			usbin_typefix;
	int			usbin_aicl;
	int			usbin_power;
	bool			presence_otg;
	bool			presence_usb;
	bool			presence_wireless;
	// battery states
	bool			battery_eoc;
	int			battery_capacity;
	int			battery_uvoltage;
	int			battery_temperature;
	// pseudo values
	int			pseudo_status;
	int			pseudo_health;

/* limited current/voltage values by LGE scenario (unit in millis). */
	int			limited_iusb;
	int			limited_ibat;
	int			limited_idc;
	int			limited_vfloat;
	int			limited_hvdcp;

/* power supplies */
	struct power_supply*	psy_battery;
	struct power_supply*	psy_usb;
	struct power_supply*	psy_wireless;

	bool pm_qos_flag;
	struct pm_qos_request pm_qos;
	struct mutex veneer_lock;
};

static void add_pm_qos_request(struct veneer* veneer_me)
{
	if (veneer_me && !veneer_me->pm_qos_flag) {
		mutex_lock(&veneer_me->veneer_lock);
		veneer_me->pm_qos.irq = -1;
		veneer_me->pm_qos.type = PM_QOS_REQ_AFFINE_CORES;

		cpumask_copy(&veneer_me->pm_qos.cpus_affine, cpu_present_mask);
		if (!pm_qos_request_active(&veneer_me->pm_qos))
			pm_qos_add_request(&veneer_me->pm_qos, PM_QOS_CPU_DMA_LATENCY, 0);

		veneer_me->pm_qos_flag = true;
		mutex_unlock(&veneer_me->veneer_lock);
	}
}

static void remove_pm_qos_request(struct veneer* veneer_me)
{
	if (veneer_me && veneer_me->pm_qos_flag) {
		mutex_lock(&veneer_me->veneer_lock);
		if (pm_qos_request_active(&veneer_me->pm_qos))
			pm_qos_remove_request(&veneer_me->pm_qos);

		veneer_me->pm_qos_flag = false;
		mutex_unlock(&veneer_me->veneer_lock);
	}
}

static struct power_supply* get_psy_battery(struct veneer* veneer_me)
{
	if (!veneer_me->psy_battery)
		veneer_me->psy_battery = power_supply_get_by_name("battery");
	return veneer_me->psy_battery;
}

static struct power_supply* get_psy_usb(struct veneer* veneer_me)
{
	if (!veneer_me->psy_usb)
		veneer_me->psy_usb = power_supply_get_by_name("usb");
	return veneer_me->psy_usb;
}

static struct power_supply* get_psy_wireless(struct veneer* veneer_me)
{
	if (!veneer_me->psy_wireless)
		veneer_me->psy_wireless = power_supply_get_by_name("wireless");
	return veneer_me->psy_wireless;
}

static bool supplier_online(struct veneer* veneer_me)
{
	bool online_usb = veneer_me->presence_usb
		&& veneer_voter_suspended(VOTER_TYPE_IUSB)
			!= CHARGING_SUSPENDED_WITH_FAKE_OFFLINE;
	bool online_wireless = veneer_me->presence_wireless
		&& veneer_voter_suspended(VOTER_TYPE_IDC)

			!= CHARGING_SUSPENDED_WITH_FAKE_OFFLINE;

	return online_usb || online_wireless;
}

static bool supplier_connected(struct veneer* veneer_me)
{
	return veneer_me->presence_usb
		|| veneer_me->presence_wireless;
}

static enum charging_supplier supplier_sdp(
	/* @Nonnull */ struct power_supply* usb,
	/* @Nonnull */ union power_supply_propval* buf
){
	enum charging_supplier ret = CHARGING_SUPPLY_USB_2P0;

	if (!power_supply_get_property(
			usb, POWER_SUPPLY_PROP_RESISTANCE_ID, buf)) {
		switch (buf->intval) {
		case CHARGER_USBID_56KOHM :
			ret = CHARGING_SUPPLY_FACTORY_56K;
			break;
		case CHARGER_USBID_130KOHM :
			ret = CHARGING_SUPPLY_FACTORY_130K;
			break;
		case CHARGER_USBID_910KOHM :
			ret = CHARGING_SUPPLY_FACTORY_910K;
			break;
		default :
			break;
		}
	}
	else
		pr_veneer("USB-ID: unable to get POWER_SUPPLY_PROP_RESISTANCE_ID\n");

	return ret;
}

static enum charging_supplier supplier_typec(
	/* @Nonnull */ struct power_supply* usb,
	/* @Nonnull */ union power_supply_propval* buf
){

	enum power_supply_type real = buf->intval;
	enum charging_supplier ret = CHARGING_SUPPLY_TYPE_UNKNOWN;

	if (!power_supply_get_property(usb, POWER_SUPPLY_PROP_TYPEC_MODE, buf)) {
		switch (buf->intval) {
		case POWER_SUPPLY_TYPEC_NONE :
			ret = (real == POWER_SUPPLY_TYPE_USB_FLOAT)
				? CHARGING_SUPPLY_TYPE_FLOAT
				: CHARGING_SUPPLY_DCP_DEFAULT;
			break;
		case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT :
			ret = CHARGING_SUPPLY_DCP_DEFAULT;
			break;
		case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM :
			ret = CHARGING_SUPPLY_DCP_22K;
			break;
		case POWER_SUPPLY_TYPEC_SOURCE_HIGH :
			ret = CHARGING_SUPPLY_DCP_10K;
			break;
		default :
			ret = CHARGING_SUPPLY_DCP_DEFAULT;
			break;
		}
	}
	else
		pr_veneer("Failed to get POWER_SUPPLY_PROP_TYPEC_MODE\n");

	return ret;
}

static enum charging_supplier supplier_wireless(
	struct veneer* veneer_me,
	struct power_supply* wireless,
	union power_supply_propval* buf
){
	if (wireless &&
		!power_supply_get_property(wireless, POWER_SUPPLY_PROP_POWER_NOW, buf))
	{
		pr_veneer("wireless POWER_SUPPLY_PROP_POWER_NOW = %d\n", buf->intval);
		buf->intval /= 1000;
		return (buf->intval >= veneer_me->threshold_wireless) ?
			CHARGING_SUPPLY_WIRELESS_9W : CHARGING_SUPPLY_WIRELESS_5W;
	}
	else
		return CHARGING_SUPPLY_TYPE_UNKNOWN;
}

static bool charging_wakelock_acquire(struct veneer* veneer_me)
{
	if (veneer_me && !veneer_me->veneer_wakelock.active) {
		pr_veneer("%s\n", VENEER_WAKELOCK);
		__pm_stay_awake(&veneer_me->veneer_wakelock);
		add_pm_qos_request(veneer_me);
		return true;
	}
	return false;
}

static bool charging_wakelock_release(struct veneer* veneer_me)
{
	if (veneer_me && veneer_me->veneer_wakelock.active) {
		pr_veneer("%s\n", VENEER_WAKELOCK);
		__pm_relax(&veneer_me->veneer_wakelock);
		pm_wakeup_event(veneer_me->veneer_dev, 1000);
		remove_pm_qos_request(veneer_me);
		return true;
	}
	return false;
}

static void update_veneer_wakelock(struct veneer* veneer_me)
{
	bool connected = supplier_connected(veneer_me);
	bool eoc = veneer_me->battery_eoc;

	if (connected && !eoc)
		charging_wakelock_acquire(veneer_me);
	else
		charging_wakelock_release(veneer_me);
}

static void update_veneer_supplier(struct veneer* veneer_me)
{
	enum charging_supplier		new = CHARGING_SUPPLY_TYPE_UNKNOWN;
	struct power_supply*		psy = NULL;
	union power_supply_propval	val;

	if (veneer_me->presence_usb) {
		psy = get_psy_usb(veneer_me);
		if (psy) {
			val.intval = veneer_me->usbin_realtype;
			pr_dbg_veneer("POWER_SUPPLY_PROP_REAL_TYPE = %d\n", val.intval);

			switch (val.intval) {
			case POWER_SUPPLY_TYPE_USB_FLOAT :
				new = CHARGING_SUPPLY_TYPE_FLOAT;
				break;
				// fall through, (Here is no break)
			case POWER_SUPPLY_TYPE_USB_DCP :
				// USB C type would be enumerated to DCP type
				new = supplier_typec(psy, &val);
				break;

			case POWER_SUPPLY_TYPE_USB :
				new = supplier_sdp(psy, &val);
				// If USB 3.x is detected already, preserve it.
				if (veneer_me->veneer_supplier == CHARGING_SUPPLY_USB_3PX
					&& new == CHARGING_SUPPLY_USB_2P0)
					new = CHARGING_SUPPLY_USB_3PX;
				// If float charger is detected, stay it.
				if (veneer_me->veneer_supplier == CHARGING_SUPPLY_TYPE_FLOAT)
					new = CHARGING_SUPPLY_TYPE_FLOAT;
				break;
			case POWER_SUPPLY_TYPE_USB_CDP :
				new = CHARGING_SUPPLY_USB_CDP;
				break;
			case POWER_SUPPLY_TYPE_USB_PD :
				new = CHARGING_SUPPLY_USB_PD;
				break;
			case POWER_SUPPLY_TYPE_USB_HVDCP :
				new = CHARGING_SUPPLY_DCP_QC2;
				break;
			case POWER_SUPPLY_TYPE_USB_HVDCP_3 :
				new = CHARGING_SUPPLY_DCP_QC3;
				break;
			default :
				break;
			}
		}
		/* Overriding on fake hvdcp */
		val.intval = 1;
		if (psy && new != CHARGING_SUPPLY_DCP_QC2
			&& !power_supply_set_property(psy, POWER_SUPPLY_PROP_USB_HC, &val)
			&& !power_supply_get_property(psy, POWER_SUPPLY_PROP_USB_HC, &val)
			&& !!val.intval) {
			new = CHARGING_SUPPLY_DCP_QC2;
			pr_veneer("Fake HVDCP is enabled, set supplier to QC2\n");
		}
	}
	else if (veneer_me->presence_wireless) {
		psy = get_psy_wireless(veneer_me);
		new = supplier_wireless(veneer_me, psy, &val);
	}
	else {
		/* 'new' may be 'NONE' at the initial time and it will be updated soon */
		new = CHARGING_SUPPLY_TYPE_NONE;
	}

	if (veneer_me->veneer_supplier != new) {
		/* Updating member of charger type here */
		veneer_me->veneer_supplier = new;
		pr_veneer("Charger is changed to %s\n", charger_name(new));
	}
}

static void update_veneer_uninodes(struct veneer* veneer_me)
{
	// Should be called
	// - on every effective 'external_power_changed' and
	// - after 'update_veneer_supplier'
	enum charging_supplier	type = veneer_me->veneer_supplier;
	enum charging_verbosity verbose = VERBOSE_CHARGER_NORMAL;
	const char*		name = charger_name(type);
	char			buff [2] = { 0, };
	int mw_highspeed = INT_MAX;
	int mw_now = 0, stored = 0;
	struct power_supply*	psy = NULL;

    // 'bootcmd: lge.charger_verbose=(%bool)'
	//		is adopted to branch vzw/att or not.
	if (unified_bootmode_chgverbose()) {
		switch (type) {
		case CHARGING_SUPPLY_DCP_DEFAULT: {
			// VERBOSE_CHARGER_SLOW
			//		will be set in set_property and a dedicated work
			if (unified_nodes_show("charger_verbose", buff)
				&& sscanf(buff, "%d", &stored)
				&& stored == VERBOSE_CHARGER_SLOW)
				verbose = VERBOSE_CHARGER_SLOW;
		}	break;

		case CHARGING_SUPPLY_TYPE_NONE:
		case CHARGING_SUPPLY_TYPE_UNKNOWN:
			verbose = VERBOSE_CHARGER_NONE;
			break;

		case CHARGING_SUPPLY_TYPE_FLOAT:
			verbose = VERBOSE_CHARGER_INCOMPATIBLE;
			break;

		default :
			break;
		}

		snprintf(buff, sizeof(buff), "%d", verbose);
		/* Updating verbosity here for vzw models */
		unified_nodes_store("charger_verbose", buff, strlen(buff));
	}
	else {
		/* Updating incompatible charger status here for non-vzw models */
		unified_nodes_store("charger_incompatible",
				type == CHARGING_SUPPLY_TYPE_FLOAT ? "1" : "0", 1);
	}

	/* Updating fast charger status here */
	if (!unified_nodes_show("charger_highspeed", buff)
		|| strncmp(buff, "1", 1)
		|| type == CHARGING_SUPPLY_TYPE_NONE
		|| type == CHARGING_SUPPLY_TYPE_UNKNOWN
		|| type == CHARGING_SUPPLY_TYPE_FLOAT
		|| type == CHARGING_SUPPLY_WIRELESS_5W
		|| veneer_me->usbin_typefix)
	{

		if (veneer_me->presence_usb) {
			mw_highspeed = veneer_me->threshold_wired;
			psy = get_psy_usb(veneer_me);
		}
		else if (veneer_me->presence_wireless) {
			mw_highspeed = veneer_me->threshold_wireless;
			psy = get_psy_wireless(veneer_me);
		}
		else
			psy = NULL; // No supply

		if (psy) {
			union power_supply_propval val = { .intval = 0, };
			power_supply_get_property(psy, POWER_SUPPLY_PROP_POWER_NOW, &val);
			mw_now = val.intval / 1000;
			pr_veneer("mw_now = %d\n", mw_now);
		}

		unified_nodes_store("charger_highspeed",
			(mw_now >= mw_highspeed
				&& type != CHARGING_SUPPLY_DCP_10K) ? "1" : "0", 1);
	}
	else
		pr_dbg_veneer("Skip to update highspeed "
			"if buff == 1 && !CHARGING_SUPPLY_TYPE_NONE\n");

	/* Finally, updating charger name here */
	unified_nodes_store("charger_name", name, sizeof(name));
}

static void update_veneer_status(struct veneer* veneer_me)
{
	bool online = supplier_online(veneer_me);
	int capacity = veneer_me->battery_capacity;
	int status;

	if (capacity == 100)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (online)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else
		status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (veneer_me->pseudo_status != status) {
		pr_veneer("pseudo_status is updated to %d\n", status);
		veneer_me->pseudo_status = status;
	}
}

static void notify_siblings(struct veneer* veneer_me)
{
	/* Capping IUSB/IBAT/IDC by charger */
	charging_ceiling_vote(veneer_me->veneer_supplier);
	/* Calculating remained charging time */
	charging_time_update(veneer_me->veneer_supplier, false);
	/* LGE OTP scenario */
	protection_battemp_monitor();
	/* To meet battery spec. */
	protection_batvolt_refresh(supplier_connected(veneer_me));
	/* Limiting SoC range in the store mode charging */
	protection_showcase_update();
	/* protection of usb io */
	protection_usbio_update(veneer_me->presence_usb);
}

static void notify_fabproc(struct veneer* veneer_me)
{
	struct power_supply* psy;

	// Enabling parallel charger here if it is required
	char buf [16] = { 0, };
	int fastparallel;
	union power_supply_propval enable = { .intval = 0, };
	psy = get_psy_battery(veneer_me);

	if (psy && unified_nodes_show("support_fastpl", buf)
		&& sscanf(buf, "%d", &fastparallel) && !!fastparallel) {
		// Protocol : Enabling parallel charging by force
		if (power_supply_set_property(
			psy, POWER_SUPPLY_PROP_PARALLEL_DISABLE, &enable))
			pr_veneer("psy should provide POWER_SUPPLY_PROP_PARALLEL_DISABLE"
				"for the factory fast parallel charging.\n");
	}
}

static void notify_touch(struct veneer* veneer_me)
{
#ifdef CONFIG_LGE_TOUCH_CORE
	extern void touch_notify_connect(u32 type);
	extern void touch_notify_wireless(u32 type);
	static bool charging_wired_prev, charging_wireless_prev;

	bool charging_wired_now = veneer_me->presence_usb;
	bool charging_wireless_now = veneer_me->presence_wireless;

	if (charging_wired_now && charging_wireless_now) {
		// Assertion failed!
		pr_veneer("Wired and wireless charging should"
			" not be online at the same time");
		return;
	}
	else if (charging_wired_prev != charging_wired_now) {
		touch_notify_connect((u32)charging_wired_now);
		charging_wired_prev = charging_wired_now;
	}
	else if (charging_wireless_prev != charging_wireless_now) {
		touch_notify_wireless((u32)charging_wireless_now);
		charging_wireless_prev = charging_wireless_now;
	}
	else
		; // Do nothing yet
#endif
}

static struct veneer* veneer_data_fromair(void)
{
	struct power_supply* veneer_psy = power_supply_get_by_name("veneer");
	struct veneer* veneer_me;

	if (veneer_psy) {
		veneer_me = power_supply_get_drvdata(veneer_psy);
		power_supply_put(veneer_psy);
		return veneer_me;
	}

	return NULL;
}

static void veneer_data_update(struct veneer* veneer_me)
{
	update_veneer_status(veneer_me);
	update_veneer_wakelock(veneer_me);
	update_veneer_supplier(veneer_me);
	update_veneer_uninodes(veneer_me);

	// After update veneer structures,
	// do update sibling data, factory process and touch device finally.
	notify_siblings(veneer_me);
	notify_fabproc(veneer_me);
	notify_touch(veneer_me);
}

static enum power_supply_property psy_property_list [] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
};

#define SLOW_CHARGING_TIMEOUT_MS	5000
#define SLOW_CHARGING_CURRENT_MA	450
static bool detect_slowchg_required(/* @Nonnull */ struct veneer* veneer_me)
{
	char buf [2] = { 0, };
	int  verbose;

	return unified_bootmode_chgverbose()
		&& veneer_me->veneer_supplier == CHARGING_SUPPLY_DCP_DEFAULT
		&& veneer_me->usbin_aicl < SLOW_CHARGING_CURRENT_MA
		&& unified_nodes_show("charger_verbose", buf)
		&& sscanf(buf, "%d", &verbose)
		&& verbose != VERBOSE_CHARGER_SLOW;
}

static void detect_slowchg_timer(struct work_struct* work)
{
	struct veneer* veneer_me =
		container_of(work, struct veneer, dwork_slowchg.work);
	struct power_supply* battery;
	char buf [2] = { 0, };

	if (!veneer_me)
		return;

	if (detect_slowchg_required(veneer_me)) {
		battery = get_psy_battery(veneer_me);
		if (battery) {
			/* Updating VERBOSE_CHARGER_SLOW here for vzw models */
			snprintf(buf, sizeof(buf), "%d", VERBOSE_CHARGER_SLOW);
			unified_nodes_store("charger_verbose", buf, strlen(buf));

			/* Toasting popup */
			power_supply_changed(battery);

			pr_veneer("SLOWCHG: Slow charger detected "
				"(AICL %d < Threshold %d)\n",
				veneer_me->usbin_aicl, SLOW_CHARGING_CURRENT_MA);
			return;
		}
	}

	pr_veneer("SLOWCHG: Charger %s, AICL %d, Threshold %d\n",
		charger_name(veneer_me->veneer_supplier),
		veneer_me->usbin_aicl,
		SLOW_CHARGING_CURRENT_MA);

	return;
}

static const char* psy_property_name(enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS :
		return "POWER_SUPPLY_PROP_STATUS";
	case POWER_SUPPLY_PROP_HEALTH :
		return "POWER_SUPPLY_PROP_HEALTH";
	case POWER_SUPPLY_PROP_REAL_TYPE :
		return "POWER_SUPPLY_PROP_REAL_TYPE";
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		return "POWER_SUPPLY_PROP_TIME_TO_FULL_NOW";
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		return "POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED";
	default:
		return "NOT_SUPPORTED_PROP";
	}
	return "NOT_SUPPORTED_PROP";
}

static void
psy_property_set_input_current_settled(
	struct veneer* veneer_me, int input_current)
{
	bool is_slowchg, running_slowchg;

	if (veneer_me->usbin_aicl != input_current/1000) {
		veneer_me->usbin_aicl = input_current/1000;
		is_slowchg = detect_slowchg_required(veneer_me);
		running_slowchg = delayed_work_pending(&veneer_me->dwork_slowchg);

		if (is_slowchg && !running_slowchg) {
			schedule_delayed_work(&veneer_me->dwork_slowchg,
				round_jiffies_relative(msecs_to_jiffies(
					SLOW_CHARGING_TIMEOUT_MS)));
			pr_veneer("%s: SLOWCHG: Start timer to check slow charger, "
				"Initaial AICL = %d\n",
				psy_property_name(POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED),
				veneer_me->usbin_aicl);
		} else if (!is_slowchg && running_slowchg) {
			cancel_delayed_work(&veneer_me->dwork_slowchg);
			pr_veneer("%s: SLOWCHG: Stop timer to check slow charger, "
				"AICL = %d\n",
				psy_property_name(POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED),
				veneer_me->usbin_aicl);
		}
	}
}

static int psy_property_get(
	struct power_supply *psy_me,
	enum power_supply_property prop, union power_supply_propval *val)
{
	struct veneer* veneer_me = power_supply_get_drvdata(psy_me);
	struct power_supply* battery;
	char buff [2] = { 0, };
	int rc = 0;

	if (!veneer_me) {
		pr_veneer("veneer is not ready yet\n");
		return -EINVAL;
	}
	battery = get_psy_battery(veneer_me);

	switch (prop) {
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW : {
		rc = -EINVAL;

		if (battery && !power_supply_get_property(battery,
			POWER_SUPPLY_PROP_CAPACITY_RAW, val)) {
			val->intval = charging_time_remains(val->intval);
			rc = 0;
		}
	}	break;

	case POWER_SUPPLY_PROP_SDP_CURRENT_MAX :
		if (unified_nodes_show("fake_sdpmax", buff) && !strncmp(buff, "1", 1)
				&& veneer_me->usbin_realtype == POWER_SUPPLY_TYPE_USB) {
			val->intval = veneer_me->limited_iusb * 1000;
		} else if (veneer_me->veneer_supplier == CHARGING_SUPPLY_FACTORY_56K
				|| veneer_me->veneer_supplier == CHARGING_SUPPLY_FACTORY_130K
				|| veneer_me->veneer_supplier == CHARGING_SUPPLY_FACTORY_910K) {
			val->intval = veneer_me->limited_iusb * 1000;
		} else {
			val->intval = VOTE_TOTALLY_RELEASED;
		}
		break;

	case POWER_SUPPLY_PROP_REAL_TYPE :
		val->intval = veneer_me->presence_wireless
			? POWER_SUPPLY_TYPE_WIRELESS : veneer_me->usbin_realtype;
		break;

	case POWER_SUPPLY_PROP_STATUS :
		val->intval = veneer_me->pseudo_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH :
		val->intval = veneer_me->pseudo_health;
		break;

	default:
		rc = -EINVAL;
	}

	return rc;
}

static int psy_property_set(
	struct power_supply *psy_me,
	enum power_supply_property prop, const union power_supply_propval *val)
{
	int rc = 0;
	struct veneer* veneer_me = power_supply_get_drvdata(psy_me);

	if (!veneer_me) {
		pr_veneer("veneer is not ready yet\n");
		return -EINVAL;
	}
/* Be sure that below settable properties are hidden props,
 * so do not export them in 'psy_property_list' or 'psy_property_get'
 */
	switch (prop) {

/* 'veneer_me->set_property(POWER_SUPPLY_PROP_REAL_TYPE, real_type)'
 *   is designed veneer to accept the real charger type enumerated
 *   from charger driver.
 */
	case POWER_SUPPLY_PROP_REAL_TYPE :
		switch (val->intval) {
		case POWER_SUPPLY_TYPE_USB_FLOAT :
			if (veneer_me->veneer_supplier != CHARGING_SUPPLY_TYPE_FLOAT)
				veneer_me->veneer_supplier = CHARGING_SUPPLY_TYPE_FLOAT;
			else
				goto out;
			break;
		case POWER_SUPPLY_TYPE_USB_DCP :
			if (veneer_me->veneer_supplier != CHARGING_SUPPLY_DCP_DEFAULT)
				veneer_me->veneer_supplier = CHARGING_SUPPLY_DCP_DEFAULT;
			else
				goto out;
			break;
		case POWER_SUPPLY_TYPE_USB_PD :
			if (veneer_me->veneer_supplier == CHARGING_SUPPLY_TYPE_FLOAT)
				veneer_me->veneer_supplier = CHARGING_SUPPLY_USB_PD;
			else
				goto out;
			break;
		case POWER_SUPPLY_TYPE_USB :
			if (veneer_me->veneer_supplier != CHARGING_SUPPLY_USB_3PX)
				veneer_me->veneer_supplier = CHARGING_SUPPLY_USB_3PX;
			else
				goto out;
			break;
		default :
			goto out;
		}

		veneer_me->usbin_realtype = val->intval;
		veneer_me->usbin_typefix = true;
		charging_time_clear();
		veneer_data_update(veneer_me);
		pr_veneer("%s: Setting charger to %s externally\n",
			psy_property_name(prop), charger_name(veneer_me->veneer_supplier));

out:		break;

/* 'veneer_me->set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED, aicl)'
 *   is designed veneer to receive AICL measured in ISR of charger driver.
 *   Reuse it to detect slow charger for VZW.
 */
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED :
		psy_property_set_input_current_settled(veneer_me, val->intval);
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW_ERROR :
		pr_veneer("veneer exception %04x detected\n", val->intval);
		veneer_me->veneer_exception |= val->intval;
		if (val->intval == EXCEPTION_WIRED_VCCOVER)
			protection_usbio_trigger();
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int psy_property_writeable(
	struct power_supply *psy, enum power_supply_property prop
){
	int rc = 0;

	switch (prop) {
	default:
		rc = 0;
	}

	return rc;
}

/* VENEER delegates logging to an external psy who can access all the states of battery.
 * Consider that accessing PMIC regs in the VENEER, for example, would be impractical.
 * And the external psy should be able to process logging command,
 * by providing a dedicated logging PROP, POWER_SUPPLY_PROP_DEBUG_BATTERY
 */
static void psy_external_logging(struct work_struct* work)
{
	struct veneer* veneer_me =
		container_of(work, struct veneer, dwork_logger.work);
	struct power_supply* psy_logger = get_psy_battery(veneer_me);
	union power_supply_propval prp_command = { .intval = 0 };

	if (psy_logger)
		power_supply_set_property(
			psy_logger, POWER_SUPPLY_PROP_DEBUG_BATTERY, &prp_command);

	schedule_delayed_work(
		to_delayed_work(work), msecs_to_jiffies(debug_polling_time));
}

static void psy_external_changed(struct power_supply* psy_me)
{
	struct veneer* veneer_me = power_supply_get_drvdata(psy_me);
	union  power_supply_propval buffer;
	char hit [1024] = { 0, };

	struct power_supply* psy_batt;
	struct power_supply* psy_usb;
	struct power_supply* psy_wireless;
	bool terminated;

	if (veneer_me) {
		psy_batt	= get_psy_battery(veneer_me);
		psy_usb		= get_psy_usb(veneer_me);
		psy_wireless	= get_psy_wireless(veneer_me);
	}
	else {
		pr_veneer("veneer is not ready yet\n");
		return;
	}

	if (psy_batt) {
		/* Update eoc */
		if (!power_supply_get_property(
				psy_batt, POWER_SUPPLY_PROP_CHARGE_DONE, &buffer))
		{
			/* 'terminated' is true only when
			 *    1. CHARGE_DONE and
			 *    2. NOT_OTP
			 */
			terminated = !!buffer.intval
				&& (veneer_me->limited_vfloat == VOTE_TOTALLY_RELEASED);
			if (veneer_me->battery_eoc != terminated) {
				veneer_me->battery_eoc = terminated;
				strcat(hit, "B:CHARGE_DONE ");
			}
		}
		/* Update capacity */
		if (!power_supply_get_property(
				psy_batt, POWER_SUPPLY_PROP_CAPACITY, &buffer)
			&& veneer_me->battery_capacity != buffer.intval)
		{
			veneer_me->battery_capacity = buffer.intval;
			strcat(hit, "B:CAPACITY ");
		}
		/* Update uvoltage : Just being used to trigger BTP */
		if (!power_supply_get_property(
				psy_batt, POWER_SUPPLY_PROP_VOLTAGE_NOW, &buffer)
			&& veneer_me->battery_uvoltage != buffer.intval)
		{
			veneer_me->battery_uvoltage = buffer.intval;
			strcat(hit, "B:VOLTAGE_NOW ");
		}
		/* Update temperature : Just being used to trigger BTP */
		if (!power_supply_get_property(
				psy_batt, POWER_SUPPLY_PROP_TEMP, &buffer)
			&& veneer_me->battery_temperature != buffer.intval)
		{
			veneer_me->battery_temperature = buffer.intval;
			strcat(hit, "B:TEMP ");
		}
	}

	if (psy_usb) {
		/* Update usb present */
		if (!power_supply_get_property(
				psy_usb, POWER_SUPPLY_PROP_PRESENT, &buffer)
			&& veneer_me->presence_usb != !!buffer.intval) {
			veneer_me->presence_usb = !!buffer.intval;
			if (!veneer_me->presence_usb) {
				// wired status is changed to UNKNOWN only on VBUS removal
				// and clear AICL variables here
				veneer_me->usbin_realtype = POWER_SUPPLY_TYPE_UNKNOWN;
				veneer_me->usbin_typefix = false;
				veneer_me->usbin_aicl = 0;
				power_supply_set_property(
					psy_usb, POWER_SUPPLY_PROP_RESISTANCE, &buffer);
				cancel_delayed_work(&veneer_me->dwork_slowchg);
			} else {
				schedule_delayed_work(&veneer_me->dwork_slowchg,
					round_jiffies_relative(msecs_to_jiffies(
						SLOW_CHARGING_TIMEOUT_MS)));
				pr_veneer("SLOWCHG: Start timer to check slow charger, "
					"PRESENT\n");
			}
			strcat(hit, "U:PRESENT ");
		}
		/* Update usb type */
		if (veneer_me->presence_usb
			&& !veneer_me->usbin_typefix
			&& !power_supply_get_property(
				psy_usb, POWER_SUPPLY_PROP_REAL_TYPE, &buffer)
			&& buffer.intval != veneer_me->usbin_realtype
			&& buffer.intval != POWER_SUPPLY_TYPE_UNKNOWN) {
			// Changing wired status to UNKNOWN is skipped
			veneer_me->usbin_realtype = buffer.intval;
			power_supply_set_property(
				psy_usb, POWER_SUPPLY_PROP_RESISTANCE, &buffer);
			strcat(hit, "U:REAL_TYPE ");
		}
		/* Update otg presence */
		if (!power_supply_get_property(
				psy_usb, POWER_SUPPLY_PROP_TYPEC_MODE, &buffer)) {
			bool presence_otg = (buffer.intval==POWER_SUPPLY_TYPEC_SINK)
				|| (buffer.intval==POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE);
			if (veneer_me->presence_otg != presence_otg) {
				veneer_me->presence_otg = presence_otg;
				strcat(hit, "U:OTG ");
			}
		}
		/* Update Power */
		if (!power_supply_get_property(
				psy_usb, POWER_SUPPLY_PROP_POWER_NOW, &buffer)
			&& veneer_me->usbin_power != buffer.intval)
		{
			veneer_me->usbin_power = buffer.intval;
			strcat(hit, "B:POWER ");
		}
	}
	if (psy_wireless) {
		char buf[2] = { 0, };

		/* LGE scenario : Disable wireless charging on wired */
		buffer.intval = !(veneer_me->presence_otg || veneer_me->presence_usb);
		if (veneer_me->presence_otg
			&& !veneer_me->presence_usb
			&& (lge_prm_get_info(LGE_PRM_INFO_DS_STATE) == LGE_PRM_DS2_CONNECTION_ON
				|| lge_prm_get_info(LGE_PRM_INFO_DS_STATE) == LGE_PRM_HALLIC_CONNECTION_ON
				|| (unified_nodes_show("status_boot", buf)
					&& !strncmp(buf, "0", 1)))) {
			buffer.intval = 1;
		}
		if (power_supply_set_property(psy_wireless,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &buffer)) {
			pr_veneer("Error to enable wireless : %d\n", buffer.intval);
		}
		/* Then update wireless present */
		if (!power_supply_get_property(
				psy_wireless, POWER_SUPPLY_PROP_PRESENT, &buffer)
			&& veneer_me->presence_wireless != !!buffer.intval) {
			veneer_me->presence_wireless = !!buffer.intval;
			strcat(hit, "W:PRESENT ");
		}
		/* Sending EPT or CS100 to TX pad */
		buffer.intval = veneer_me->battery_eoc;
		if (veneer_me->presence_wireless
			&& veneer_me->battery_eoc
			&& power_supply_set_property(psy_wireless,
			POWER_SUPPLY_PROP_CHARGE_DONE, &buffer)) {
			pr_veneer("Error to set full wireless : %d\n", buffer.intval);
		}
	}

	if (strlen(hit)) {
		pr_veneer("externally changed : %s\n", hit);
		veneer_data_update(veneer_me);
	}
}

static bool feed_charging_time(int* power, int* rawsoc, int* bsm_ttf)
{
	/* Power may be unstable at the initial time of detecting charger */
	struct veneer* veneer_me = veneer_data_fromair();
	struct power_supply* psy = NULL;
	union power_supply_propval val = { .intval = 0 };
	char buffer [16];

	if (veneer_me) {
		*power = 0;
		*rawsoc = 0;

		psy = NULL;
		if (veneer_me->presence_usb)
			psy = get_psy_usb(veneer_me);
		else if (veneer_me->presence_wireless)
			psy = get_psy_wireless(veneer_me);

		if (psy) {
			if (!power_supply_get_property(
					psy, POWER_SUPPLY_PROP_POWER_NOW, &val))
				*power = val.intval / 1000;
		}

		psy = get_psy_battery(veneer_me);
		if (psy) {
			if (!power_supply_get_property(
					psy, POWER_SUPPLY_PROP_CAPACITY_RAW, &val))
				*rawsoc = val.intval;
		}

		if (unified_nodes_show("bsm_timetofull", buffer))
			sscanf(buffer, "%d", bsm_ttf);
		else
			*bsm_ttf = 0;

		return true;
	}

	return false;
}

static void back_charging_time(int power)
{
	/* Simple signal for finishing of charging-time-table */
	struct veneer* veneer_me = veneer_data_fromair();
	struct power_supply* psy_batt = NULL;

	if (veneer_me) {
		psy_batt = get_psy_battery(veneer_me);
		if (psy_batt) {
			pr_veneer("Building charging time table for %dmW is done\n", power);
			power_supply_changed(psy_batt);
		}
	}
}

/* Be sure that the "get_property(POWER_SUPPLY_PROP_TEMP)"
 * should return valid value at the time of probing also.
 * If not, you may need to add a flag to determine
 * the end of measuring the initial temperature from the psy,
 * which provides temperature data.
 */
static bool feed_protection_battemp(
	bool* charging, int* temperature, int* mvoltage)
{
	struct veneer* veneer_me = veneer_data_fromair();
	struct power_supply* battery = NULL;
	union power_supply_propval val = { .intval = VENEER_NOTREADY };

	if (veneer_me) {
		battery = get_psy_battery(veneer_me);
		if (!battery)
			goto out;

		*charging = supplier_connected(veneer_me);

		*temperature = VENEER_NOTREADY;
		if (!power_supply_get_property(
				battery, POWER_SUPPLY_PROP_TEMP, &val))
			*temperature = val.intval;

		*mvoltage = VENEER_NOTREADY;
		if (!power_supply_get_property(
				battery, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val))
			 *mvoltage = val.intval / 1000;

		if (*temperature == VENEER_NOTREADY || *mvoltage == VENEER_NOTREADY)
			goto out;
		else
			return true;
	}

out:
	pr_veneer("battery status is invalid\n");
	*charging = false;
	*temperature = VENEER_NOTREADY;
	*mvoltage = VENEER_NOTREADY;
	return false;
}

#define UPDATE_TEMP_THRESHOLD 430
static void back_protection_battemp(int health, int mcurrent, int mvfloat)
{
	static int btp_mcurrent = INT_MAX;
	static int btp_mvfloat = INT_MAX;
	static int btp_temp = INT_MAX;
	bool update_battemp = false;
	struct veneer* veneer_me = veneer_data_fromair();
	struct power_supply* psy_batt = NULL;
	union  power_supply_propval buffer;

	if (veneer_me) {
		if (veneer_me->pseudo_health != health) {
			pr_veneer("BTP health %d -> %d\n", veneer_me->pseudo_health,
				health);
			veneer_me->pseudo_health = health;
			update_battemp = true;
		}

		if (btp_mcurrent != mcurrent) {
			pr_veneer("BTP mcurrent %d -> %d\n",
				min(veneer_me->profile_mincap, btp_mcurrent),
				min(veneer_me->profile_mincap, mcurrent));
			btp_mcurrent = mcurrent;
		}

		if (btp_mvfloat != mvfloat) {
			pr_veneer("BTP mvfloat %d -> %d\n",
				min(veneer_me->profile_mvfloat, btp_mvfloat),
				min(veneer_me->profile_mvfloat, mvfloat));
			btp_mvfloat = mvfloat;
		}

		psy_batt = get_psy_battery(veneer_me);
		if (psy_batt) {
			if (!power_supply_get_property(
					psy_batt, POWER_SUPPLY_PROP_TEMP, &buffer)
					&& buffer.intval >= UPDATE_TEMP_THRESHOLD
					&& buffer.intval/10 != btp_temp/10) {
				pr_veneer("BTP temp %d -> %d\n", btp_temp, buffer.intval);
				btp_temp = buffer.intval;
				update_battemp = true;
			}
		}

		if (psy_batt && update_battemp) {
			power_supply_changed(psy_batt);
		}
	}
	else
		pr_veneer("fail to getting veneer data\n");
}

/* vnow_mv : current vbat
 * icap_ma : current mitigated ibat
 */
static bool feed_protection_batvolt(int* vnow_mv, int* icap_ma, int* chg_type)
{
	const char* provider = "battery";
	union power_supply_propval	vnow = { .intval = -1000 };
	union power_supply_propval	icap = { .intval = -1000 };
	union power_supply_propval	tchg = { .intval = -1000 };

	struct power_supply* psy = power_supply_get_by_name(provider);

	bool ret = false;

	if (psy) {
		if (!power_supply_get_property(psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &vnow)
			&& !power_supply_get_property(psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &icap)
			&& !power_supply_get_property(psy,
				POWER_SUPPLY_PROP_CHARGE_TYPE, &tchg))
		{
			*vnow_mv = vnow.intval / 1000;
			*icap_ma = icap.intval / 1000;
			*chg_type = tchg.intval;

			ret = true;
		}
		power_supply_put(psy);
	}
	else {
		pr_veneer("psy %s is not ready\n", provider);
	}

	return ret;
}

static bool feed_protection_showcase(
	bool* enabled, bool* charging, int* capacity)
{
	struct veneer* veneer_me = veneer_data_fromair();
	char buffer [16];
	int scan;
	if (veneer_me && unified_nodes_show("charging_showcase", buffer)) {
		sscanf(buffer, "%d", &scan);

		*enabled = !!scan;
		*charging = supplier_connected(veneer_me);
		*capacity = veneer_me->battery_capacity;
		return true;
	}
	else {
		pr_veneer("veneer or sysfs are not ready\n");
		*enabled = false;
		*charging = false;
		*capacity = 0;
		return false;
	}
}

static void back_protection_showcase(const char* status)
{
	struct veneer* veneer_me = veneer_data_fromair();
	struct power_supply* psy_usb = NULL;

	if (veneer_me) {
		psy_usb = get_psy_usb(veneer_me);
		if (psy_usb) {
			/* For dis/enabling lightning as fake */
			power_supply_changed(psy_usb);
			pr_veneer("Showcase charging is in progress : %s\n", status);
		}
	}
}

static void back_veneer_voter(enum voter_type type, int limit)
{
	struct veneer* veneer_me = veneer_data_fromair();

	if (veneer_me) {
		struct power_supply* psy_batt = get_psy_battery(veneer_me);
		const union power_supply_propval psy_prop = vote_make(type, limit);

		switch (type) {
			case VOTER_TYPE_IUSB: veneer_me->limited_iusb = limit;
				break;
			case VOTER_TYPE_IBAT: veneer_me->limited_ibat = limit;
				break;
			case VOTER_TYPE_IDC: veneer_me->limited_idc = limit;
				break;
			case VOTER_TYPE_VFLOAT: veneer_me->limited_vfloat = limit;
				break;
			case VOTER_TYPE_HVDCP: veneer_me->limited_hvdcp = limit;
				break;
			default: pr_veneer("Error on parsing type\n");
				break;
		}

		// Update the battery status considering fake UI
		update_veneer_status(veneer_me);

/* At this time, "POWER_SUPPLY_PROP_RESTRICTED_CHARGING" is adopted to vote
 * restriction value to the battery psy.
 * So be sure that set_property(POWER_SUPPLY_PROP_RESTRICTED_CHARGING)
 * should be defined in the psy for the charger driver,
 *   which is supporting limited charging.
 */
		if (!psy_batt || power_supply_set_property(psy_batt,
			POWER_SUPPLY_PROP_RESTRICTED_CHARGING, &psy_prop))
			pr_veneer("Error on voting to real world\n");
	}
	else
		pr_veneer("veneer is NULL\n");
}

static bool probe_siblings(struct device* dev, int mincap, int fullraw)
{
	struct device_node* dnode = dev->of_node;
	bool ret = true;

	ret &= veneer_voter_create(back_veneer_voter);

	// veneer voter should be ready before builing siblings
	ret &= charging_ceiling_create(
		of_find_node_by_name(dnode, "charging-ceiling"));
	ret &= charging_time_create(
		of_find_node_by_name(dnode, "charging-time"),
		fullraw,
		feed_charging_time,
		back_charging_time);
	ret &= protection_battemp_create(
		of_find_node_by_name(dnode, "protection-battemp"),
		mincap,
		feed_protection_battemp,
		back_protection_battemp);
	ret &= protection_batvolt_create(
		of_find_node_by_name(dnode, "protection-batvolt"),
		mincap,
		feed_protection_batvolt);
	ret &= protection_showcase_create(
		of_find_node_by_name(dnode, "protection-showcase"),
		feed_protection_showcase,
		back_protection_showcase);
	ret &= protection_usbio_create(
		of_find_node_by_name(dnode, "protection-usbio"));
	ret &= unified_nodes_create(
		of_find_node_by_name(dnode, "unified-nodes"));
	ret &= unified_sysfs_create(
		of_find_node_by_name(dnode, "unified-sysfs"));

	return ret;
}

static bool probe_preset(struct device* veneer_dev, struct veneer* veneer_me)
{
	struct device_node* veneer_supp =
		of_find_node_by_name(NULL, "lge-battery-supplement");

	if (!veneer_dev || !veneer_me || !veneer_supp)
		return false;

	veneer_me->veneer_dev = veneer_dev;
	veneer_me->veneer_supplier = CHARGING_SUPPLY_TYPE_UNKNOWN;
	mutex_init(&veneer_me->veneer_lock);
	wakeup_source_init(&veneer_me->veneer_wakelock, VENEER_WAKELOCK);
	INIT_DELAYED_WORK(&veneer_me->dwork_logger, psy_external_logging);
	INIT_DELAYED_WORK(&veneer_me->dwork_slowchg, detect_slowchg_timer);
	if (of_property_read_u32(veneer_supp, "capacity-mah-min",
			&veneer_me->profile_mincap) < 0
		|| of_property_read_u32(veneer_supp, "capacity-raw-full",
			&veneer_me->profile_fullraw) < 0) {
		pr_err("Failed to get battery profile, Check DT\n");
		return false;
	}
	veneer_me->profile_mvfloat = 4400;	// Fixed value at now

	if (of_property_read_u32(veneer_dev->of_node,
			"highspeed-threshold-mv-wired",
			&veneer_me->threshold_wired) < 0
		|| of_property_read_u32(veneer_dev->of_node,
			"highspeed-threshold-mv-wireless",
			&veneer_me->threshold_wireless) < 0) {
		veneer_me->threshold_wired = 15000;
		veneer_me->threshold_wireless = 7200;
	}

/* Initialize veneer by default */
	// below shadows can be read before being initialized.
	veneer_me->usbin_realtype = POWER_SUPPLY_TYPE_UNKNOWN;
	veneer_me->usbin_typefix = false;
	veneer_me->usbin_aicl = 0;
	veneer_me->presence_otg = false;
	veneer_me->presence_usb = false;
	veneer_me->presence_wireless = false;

	veneer_me->battery_eoc = false;
	veneer_me->battery_capacity = VENEER_NOTREADY;
	veneer_me->battery_uvoltage = VENEER_NOTREADY;
	veneer_me->battery_temperature = VENEER_NOTREADY;
	veneer_me->pseudo_status = POWER_SUPPLY_STATUS_UNKNOWN;
	veneer_me->pseudo_health = POWER_SUPPLY_HEALTH_UNKNOWN;

	veneer_me->limited_iusb = VOTE_TOTALLY_RELEASED;
	veneer_me->limited_ibat = VOTE_TOTALLY_RELEASED;
	veneer_me->limited_idc = VOTE_TOTALLY_RELEASED;
	veneer_me->limited_vfloat = VOTE_TOTALLY_RELEASED;
	veneer_me->limited_hvdcp = VOTE_TOTALLY_RELEASED;

	return true;
}

static bool probe_psy(struct veneer* veneer)
{
	static struct power_supply_desc desc = {
		.name = VENEER_NAME,
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = psy_property_list,
		.num_properties = ARRAY_SIZE(psy_property_list),
		.get_property = psy_property_get,
		.set_property = psy_property_set,
		.property_is_writeable = psy_property_writeable,
		.external_power_changed = psy_external_changed,
	};
	struct power_supply_config cfg = {
		.drv_data = veneer,
		.of_node = veneer->veneer_dev->of_node,
	};

	veneer->veneer_psy =
		power_supply_register(veneer->veneer_dev, &desc, &cfg);

	if (!IS_ERR(veneer->veneer_psy)) {
		static char* from [] = { "battery", "usb", "wireless"};
		veneer->veneer_psy->supplied_from = from;
		veneer->veneer_psy->num_supplies = ARRAY_SIZE(from);
		return true;
	}
	else {
		pr_veneer("Couldn't register veneer power supply (%ld)\n",
			PTR_ERR(veneer->veneer_psy));
		return false;
	}
}

static void veneer_clear(struct veneer* veneer_me)
{
	pr_veneer("Clearing . . .\n");

	charging_ceiling_destroy();
	charging_time_destroy();
	protection_battemp_destroy();
	protection_batvolt_destroy();
	protection_showcase_destroy();
	unified_nodes_destroy();
	unified_sysfs_destroy();

	veneer_voter_destroy();
	if (veneer_me) {
		wakeup_source_trash(&veneer_me->veneer_wakelock);
		cancel_delayed_work(&veneer_me->dwork_logger);
		cancel_delayed_work(&veneer_me->dwork_slowchg);
		if(veneer_me->veneer_psy)
			power_supply_unregister(veneer_me->veneer_psy);

		kfree(veneer_me);
	}
}

static int veneer_probe(struct platform_device *pdev)
{
	struct device* veneer_dev = &pdev->dev;
	struct veneer* veneer_me = kzalloc(sizeof(struct veneer), GFP_KERNEL);

	/* Siblings are should be ready before receiving messages from other psy-s */
	if (!probe_preset(veneer_dev, veneer_me)) {
		pr_veneer("Failed on probe_preset\n");
		goto fail;
	}

	if (!probe_siblings(
		veneer_dev, veneer_me->profile_mincap, veneer_me->profile_fullraw))
	{
		pr_veneer("Failed on probe_siblings\n");
		goto fail;
	}

	if (!probe_psy(veneer_me)) {
		pr_veneer("Failed on probe_psy\n");
		goto fail;
	}

	veneer_me->pm_qos_flag = false;
	platform_set_drvdata(pdev, veneer_me);
	device_init_wakeup(veneer_me->veneer_dev, true);
	psy_external_logging(&veneer_me->dwork_logger.work);
	return 0;

fail:	//veneer_clear(veneer_me);
	kfree(veneer_me);
	pr_veneer("Retry to probe further\n");
	return -EPROBE_DEFER;
}

static int veneer_remove(struct platform_device *pdev)
{
	struct veneer *veneer_me = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);
	veneer_clear(veneer_me);
	return 0;
}

static int veneer_suspend(struct device *dev)
{
	struct veneer* veneer_me = dev_get_drvdata(dev);
	cancel_delayed_work(&veneer_me->dwork_logger);
	return 0;
}

static int veneer_resume(struct device *dev)
{
	struct veneer* veneer_me = dev_get_drvdata(dev);
	schedule_delayed_work(
		&veneer_me->dwork_logger, msecs_to_jiffies(debug_polling_time));
	return 0;
}

static const struct dev_pm_ops veneer_pm_ops = {
	.suspend	= veneer_suspend,
	.resume		= veneer_resume,
};

static const struct of_device_id veneer_match [] = {
	{ .compatible = VENEER_COMPATIBLE },
	{ },
};

static const struct platform_device_id veneer_id [] = {
	{ VENEER_DRIVER, 0 },
	{ },
};

static struct platform_driver veneer_driver = {
	.driver = {
		.name = VENEER_DRIVER,
		.owner = THIS_MODULE,
		.of_match_table = veneer_match,
		.pm 	= &veneer_pm_ops,
	},
	.probe = veneer_probe,
	.remove = veneer_remove,
	.id_table = veneer_id,
};

static int __init veneer_init(void) {
	pr_veneer("platform_driver_register : veneer_driver\n");
	return platform_driver_register(&veneer_driver);
}

static void __exit veneer_exit(void) {
	pr_veneer("platform_driver_unregister : veneer_driver\n");
	platform_driver_unregister(&veneer_driver);
}

module_init(veneer_init);
module_exit(veneer_exit);

MODULE_DESCRIPTION(VENEER_DRIVER);
MODULE_LICENSE("GPL v2");
