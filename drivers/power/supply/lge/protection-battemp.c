#define pr_fmt(fmt) "BTP: %s: " fmt, __func__
#define pr_battemp(reason, fmt, ...)			\
do {							\
	if (pr_debugmask & (reason))			\
		pr_info(fmt, ##__VA_ARGS__);		\
	else						\
		pr_debug(fmt, ##__VA_ARGS__);		\
} while (0)

static int pr_debugmask;

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>

#include "veneer-primitives.h"

#define BATTEMP_NOTREADY	INT_MAX
#define BATTEMP_WAKELOCK	"lge-btp-scenario"

#define VOTER_NAME_ICHARGE	"BTP"
#define VOTER_NAME_VFLOAT	"BTP"
#define VOTER_NAME_CHILLY	"BTP(CHILLY)"

static struct protection_battemp {
	struct delayed_work	battemp_dwork;
	struct wakeup_source	battemp_wakelock;

	// processed in external
	bool (*get_protection_battemp)(bool* charging, int* temperature, int* mvoltage);
	void (*set_protection_battemp)(int health, int micharge, int mvfloat);

	struct voter_entry voter_ichilly;
	struct voter_entry voter_icharge;
	struct voter_entry voter_vfloat;

	bool health_chilly;
	int  health_jeita;

// below fields are set in device tree
	int threshold_degc_upto_cool;	//  30 by default
	int threshold_degc_upto_good;	// 120 by default
	int threshold_degc_upto_warm;	// 450 by default
	int threshold_degc_upto_hot;	// 550 by default
	int threshold_degc_downto_warm;	// 520 by default
	int threshold_degc_downto_good;	// 430 by default
	int threshold_degc_downto_cool;	// 100 by default
	int threshold_degc_downto_cold;	//   0 by default

	int period_ms_emergency;	// 10000 by default
	int period_ms_warning;		// 30000 by default
	int period_ms_normal;		// 60000 by default
	const int period_ms_unknown;

	int cool_mv_alert;
	int cool_ma_alert;
	int cool_ma_normal;

	int warm_ma_charge;
	int warm_mv_float;

	// below fileds are for battery protection in chilly
	bool chilly_is_supported;
	int  chilly_degc_lowerbound;
	int  chilly_degc_upperbound;
	int  chilly_mv_alert;
	int  chilly_ma_alert;
	int  chilly_ma_normal;

} battemp_me = {
	.voter_ichilly = { .type = VOTER_TYPE_INVALID },
	.voter_icharge = { .type = VOTER_TYPE_INVALID },
	.voter_vfloat  = { .type = VOTER_TYPE_INVALID },

	.health_jeita  = POWER_SUPPLY_HEALTH_UNKNOWN,
	.health_chilly = false,

	.threshold_degc_upto_cool  = BATTEMP_NOTREADY,
	.threshold_degc_upto_good  = BATTEMP_NOTREADY,
	.threshold_degc_upto_warm  = BATTEMP_NOTREADY,
	.threshold_degc_upto_hot   = BATTEMP_NOTREADY,
	.threshold_degc_downto_warm = BATTEMP_NOTREADY,
	.threshold_degc_downto_good = BATTEMP_NOTREADY,
	.threshold_degc_downto_cool = BATTEMP_NOTREADY,
	.threshold_degc_downto_cold = BATTEMP_NOTREADY,

	.period_ms_emergency = BATTEMP_NOTREADY,
	.period_ms_warning   = BATTEMP_NOTREADY,
	.period_ms_normal    = BATTEMP_NOTREADY,
	.period_ms_unknown   = 1000,

	.cool_mv_alert	= BATTEMP_NOTREADY,
	.cool_ma_alert	= BATTEMP_NOTREADY,
	.cool_ma_normal	= BATTEMP_NOTREADY,
	.warm_ma_charge	= BATTEMP_NOTREADY,
	.warm_mv_float	= BATTEMP_NOTREADY,

	.chilly_is_supported    = false,
	.chilly_degc_upperbound = BATTEMP_NOTREADY,
	.chilly_degc_lowerbound = BATTEMP_NOTREADY,
	.chilly_mv_alert	= BATTEMP_NOTREADY,
	.chilly_ma_alert	= BATTEMP_NOTREADY,
	.chilly_ma_normal       = BATTEMP_NOTREADY,
};

static const char* health_to_string(bool chilly, int jhealth) {
	if (!chilly) {
		switch (jhealth) {
		case POWER_SUPPLY_HEALTH_UNKNOWN :
			return "HEALTH_UNKNOWN";
		case POWER_SUPPLY_HEALTH_COLD :
			return "HEALTH_COLD";
		case POWER_SUPPLY_HEALTH_COOL :
			return "HEALTH_COOL";
		case POWER_SUPPLY_HEALTH_GOOD :;
			return "HEALTH_GOOD";
		case POWER_SUPPLY_HEALTH_WARM :
			return "HEALTH_WARM";
		case POWER_SUPPLY_HEALTH_HOT :
			return "HEALTH_HOT";
		default :
			return "Undefined health";
		}
	}
	else
		return "HEALTH_CHILLY";
}

static int health_to_index(bool chilly, int jhealth) {
	if (!chilly) {
		switch (jhealth) {
		case POWER_SUPPLY_HEALTH_UNKNOWN :
			return 0;
		case POWER_SUPPLY_HEALTH_COLD :
			return 1;
		case POWER_SUPPLY_HEALTH_COOL :
			return 2;
		case POWER_SUPPLY_HEALTH_GOOD :;
			return 3 + battemp_me.chilly_is_supported;
		case POWER_SUPPLY_HEALTH_WARM :
			return 4 + battemp_me.chilly_is_supported;
		case POWER_SUPPLY_HEALTH_HOT :
			return 5 + battemp_me.chilly_is_supported;
		default :
			return -1;
		}
	}
	else
		return 3;
}

static long health_to_period(bool chilly, int jhealth) {
	int msecs = 0;

	if (!chilly) {
		switch (jhealth) {
		case POWER_SUPPLY_HEALTH_HOT :
		case POWER_SUPPLY_HEALTH_COLD :
			msecs = battemp_me.period_ms_emergency;
			break;
		case POWER_SUPPLY_HEALTH_WARM :
		case POWER_SUPPLY_HEALTH_COOL :
			msecs = battemp_me.period_ms_warning;
			break;
		case POWER_SUPPLY_HEALTH_GOOD :
			msecs = battemp_me.period_ms_normal;
			break;
		case POWER_SUPPLY_HEALTH_UNKNOWN :
			msecs = battemp_me.period_ms_unknown;
			break;
		default :
			pr_battemp(ERROR, "Check the 'health'\n");
			break;
		}
	}
	else
		msecs = battemp_me.period_ms_warning;

	return msecs_to_jiffies(msecs);
}

static int icharge_by_chilly(bool chilly, int batvol) {
	if (chilly)
		return battemp_me.chilly_mv_alert <= batvol ?
			battemp_me.chilly_ma_alert : battemp_me.chilly_ma_normal;
	else
		return VOTE_TOTALLY_RELEASED;
}

static int icharge_by_jhealth(int jhealth, int batvol) {
	switch (jhealth) {
	case POWER_SUPPLY_HEALTH_COOL :
		return (battemp_me.cool_mv_alert <= batvol)
			? battemp_me.cool_ma_alert : battemp_me.cool_ma_normal;
	case POWER_SUPPLY_HEALTH_WARM :
		return battemp_me.warm_ma_charge;

	case POWER_SUPPLY_HEALTH_COLD :
	case POWER_SUPPLY_HEALTH_HOT :
		return VOTE_TOTALLY_BLOCKED;

	case POWER_SUPPLY_HEALTH_GOOD :
	case POWER_SUPPLY_HEALTH_UNKNOWN :
		return VOTE_TOTALLY_RELEASED;

	default :
		return -EINVAL;
	}
}

static int vfloat_by_jhealth(int jhealth) {
	switch (jhealth) {
	case POWER_SUPPLY_HEALTH_GOOD :
	case POWER_SUPPLY_HEALTH_COOL :
	case POWER_SUPPLY_HEALTH_COLD :
	case POWER_SUPPLY_HEALTH_UNKNOWN :
		return VOTE_TOTALLY_RELEASED;

	case POWER_SUPPLY_HEALTH_HOT :
	case POWER_SUPPLY_HEALTH_WARM :
		return battemp_me.warm_mv_float;

	default :
		return -EINVAL;
	}
}

#define STAT_NOW (health_now)
#define TEMP_NOW (battemp_now)

#define TEMP_UPTO_COOL (battemp_me.threshold_degc_upto_cool)		//	 30 by default
#define TEMP_UPTO_GOOD (battemp_me.threshold_degc_upto_good)		//	120 by default
#define TEMP_UPTO_WARM (battemp_me.threshold_degc_upto_warm)		//	450 by default
#define TEMP_UPTO_HOT (battemp_me.threshold_degc_upto_hot)		//	550 by default
#define TEMP_DOWNTO_WARM (battemp_me.threshold_degc_downto_warm)	//	520 by default
#define TEMP_DOWNTO_GOOD (battemp_me.threshold_degc_downto_good)	//	430 by default
#define TEMP_DOWNTO_COOL (battemp_me.threshold_degc_downto_cool)	//	100 by default
#define TEMP_DOWNTO_COLD (battemp_me.threshold_degc_downto_cold)	//	  0 by default
static int polling_status_jeita(int health_now, int battemp_now) {
	int health_new;

	switch (STAT_NOW) {
	case POWER_SUPPLY_HEALTH_UNKNOWN :
		if (TEMP_NOW < TEMP_DOWNTO_COLD)
			health_new = POWER_SUPPLY_HEALTH_COLD;
		else if (TEMP_NOW < TEMP_DOWNTO_COOL)
			health_new = POWER_SUPPLY_HEALTH_COOL;
		else if (TEMP_NOW < TEMP_UPTO_WARM )
			health_new = POWER_SUPPLY_HEALTH_GOOD;
		else if (TEMP_NOW < TEMP_UPTO_HOT)
			health_new = POWER_SUPPLY_HEALTH_WARM;
		else
			health_new = POWER_SUPPLY_HEALTH_HOT;
		break;

	case POWER_SUPPLY_HEALTH_COLD : // on the cold
		if (TEMP_NOW < TEMP_UPTO_COOL)
			health_new = POWER_SUPPLY_HEALTH_COLD;
		else if (TEMP_NOW < TEMP_DOWNTO_COOL)
			health_new = POWER_SUPPLY_HEALTH_COOL;
		else if (TEMP_NOW < TEMP_UPTO_WARM )
			health_new = POWER_SUPPLY_HEALTH_GOOD;
		else if (TEMP_NOW < TEMP_UPTO_HOT)
			health_new = POWER_SUPPLY_HEALTH_WARM;
		else
			health_new = POWER_SUPPLY_HEALTH_HOT;
		break;

	case POWER_SUPPLY_HEALTH_COOL : // on the cool
		if (TEMP_NOW < TEMP_DOWNTO_COLD)
			health_new = POWER_SUPPLY_HEALTH_COLD;
		else if (TEMP_NOW < TEMP_UPTO_GOOD)
			health_new = POWER_SUPPLY_HEALTH_COOL;
		else if (TEMP_NOW < TEMP_UPTO_WARM )
			health_new = POWER_SUPPLY_HEALTH_GOOD;
		else if (TEMP_NOW < TEMP_UPTO_HOT)
			health_new = POWER_SUPPLY_HEALTH_WARM;
		else
			health_new = POWER_SUPPLY_HEALTH_HOT;
		break;

	case POWER_SUPPLY_HEALTH_GOOD : // on the normal
		if (TEMP_NOW < TEMP_DOWNTO_COLD)
			health_new = POWER_SUPPLY_HEALTH_COLD;
		else if (TEMP_NOW < TEMP_DOWNTO_COOL)
			health_new = POWER_SUPPLY_HEALTH_COOL;
		else if (TEMP_NOW < TEMP_UPTO_WARM )
			health_new = POWER_SUPPLY_HEALTH_GOOD;
		else if (TEMP_NOW < TEMP_UPTO_HOT)
			health_new = POWER_SUPPLY_HEALTH_WARM;
		else
			health_new = POWER_SUPPLY_HEALTH_HOT;
		break;

	case POWER_SUPPLY_HEALTH_WARM : // on the warm
		if (TEMP_NOW < TEMP_DOWNTO_COLD)
			health_new = POWER_SUPPLY_HEALTH_COLD;
		else if (TEMP_NOW < TEMP_DOWNTO_COOL)
			health_new = POWER_SUPPLY_HEALTH_COOL;
		else if (TEMP_NOW < TEMP_DOWNTO_GOOD )
			health_new = POWER_SUPPLY_HEALTH_GOOD;
		else if (TEMP_NOW < TEMP_UPTO_HOT)
			health_new = POWER_SUPPLY_HEALTH_WARM;
		else
			health_new = POWER_SUPPLY_HEALTH_HOT;
		break;

	case POWER_SUPPLY_HEALTH_HOT : // on the hot
		if (TEMP_NOW < TEMP_DOWNTO_COLD)
			health_new = POWER_SUPPLY_HEALTH_COLD;
		else if (TEMP_NOW < TEMP_DOWNTO_COOL)
			health_new = POWER_SUPPLY_HEALTH_COOL;
		else if (TEMP_NOW < TEMP_UPTO_WARM )
			health_new = POWER_SUPPLY_HEALTH_GOOD;
		else if (TEMP_NOW < TEMP_DOWNTO_WARM)
			health_new = POWER_SUPPLY_HEALTH_WARM;
		else
			health_new = POWER_SUPPLY_HEALTH_HOT;
		break;
	default :
		health_new = POWER_SUPPLY_HEALTH_UNKNOWN;
		break;
	}

	return health_new;
}

#define CHILLY_TEMP_LOWERBOUND (battemp_me.chilly_degc_lowerbound)
#define CHILLY_TEMP_UPPERBOUND (battemp_me.chilly_degc_upperbound)
static bool polling_status_chilly(int battemp_now){
	return battemp_me.chilly_is_supported
		&& CHILLY_TEMP_LOWERBOUND <= battemp_now && battemp_now <= CHILLY_TEMP_UPPERBOUND;
}

static void polling_status_work(struct work_struct* work) {
	bool charging;
	int  temperature, mvoltage;
	int health_jeita, updated_icharge, updated_vfloat, updated_ichilly;
	bool health_chilly, warning_at_charging, warning_wo_charging;

	if (battemp_me.get_protection_battemp(&charging, &temperature, &mvoltage)) {
		// Calculates icharge and vfloat from the jeita health
		health_jeita = polling_status_jeita(battemp_me.health_jeita, temperature);
		updated_icharge = charging ? icharge_by_jhealth(health_jeita, mvoltage) : VOTE_TOTALLY_RELEASED;
		updated_vfloat = charging ? vfloat_by_jhealth(health_jeita) : VOTE_TOTALLY_RELEASED;

		// And ichilly from the boolean 'chilly' status
		health_chilly = polling_status_chilly(temperature);
		updated_ichilly = charging ? icharge_by_chilly(health_chilly, mvoltage) : VOTE_TOTALLY_RELEASED;

		// configure wakelock
		warning_at_charging = charging && (health_jeita != POWER_SUPPLY_HEALTH_GOOD);
		warning_wo_charging = health_jeita == POWER_SUPPLY_HEALTH_HOT;

		if (warning_at_charging || warning_wo_charging) {
			if (!battemp_me.battemp_wakelock.active) {
				pr_battemp(UPDATE, "Acquiring wake lock\n");
				__pm_stay_awake(&battemp_me.battemp_wakelock);
			}
		}
		else {
			if (battemp_me.battemp_wakelock.active) {
				pr_battemp(UPDATE, "Releasing wake lock\n");
				__pm_relax(&battemp_me.battemp_wakelock);
			}
		}

		// logging for changes
		if (battemp_me.health_chilly != health_chilly || battemp_me.health_jeita != health_jeita)
			pr_battemp(UPDATE, "%s(%d) -> %s(%d), temperature=%d\n",
				health_to_string(battemp_me.health_chilly, battemp_me.health_jeita),
					health_to_index(battemp_me.health_chilly, battemp_me.health_jeita),
				health_to_string(health_chilly, health_jeita),
					health_to_index(health_chilly, health_jeita),
				temperature);

		// Voting for icharge and vfloat
		veneer_voter_set(&battemp_me.voter_ichilly, updated_ichilly);
		veneer_voter_set(&battemp_me.voter_icharge, updated_icharge);
		veneer_voter_set(&battemp_me.voter_vfloat, updated_vfloat);
		if (updated_vfloat != VOTE_TOTALLY_RELEASED)
			veneer_voter_rerun(&battemp_me.voter_vfloat);

		// update member status in 'battemp_me'
		battemp_me.health_chilly = health_chilly;
		battemp_me.health_jeita = health_jeita;

		// finallly, notify the psy-type health to client
		battemp_me.set_protection_battemp(battemp_me.health_jeita,
			min(updated_ichilly, updated_icharge), updated_vfloat);
	}
	else
		pr_battemp(UPDATE, "temperature is not valid.\n");

	schedule_delayed_work(to_delayed_work(work),
		health_to_period(battemp_me.health_chilly, battemp_me.health_jeita));
	return;
}

#define SCALE_UNIT_MA	50
static bool battemp_create_parsedt(struct device_node* dnode, int mincap) {
	int rc = 0;
	int cool_ma_pct = 0, warm_ma_pct = 0;
	int chilly_ma_pct = 0;

	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_upto_cool,
		"threshold-degc-upto-cool", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_upto_good,
		"threshold-degc-upto-good", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_upto_warm,
		"threshold-degc-upto-warm", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_upto_hot,
		"threshold-degc-upto-hot", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_downto_warm,
		"threshold-degc-downto-warm", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_downto_good,
		"threshold-degc-downto-good", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_downto_cool,
		"threshold-degc-downto-cool", rc);
	OF_PROP_READ_S32(dnode, battemp_me.threshold_degc_downto_cold,
		"threshold-degc-downto-cold", rc);

	OF_PROP_READ_S32(dnode, battemp_me.period_ms_emergency,
		"period-ms-emergency", rc);
	OF_PROP_READ_S32(dnode, battemp_me.period_ms_warning,
		"period-ms-warning", rc);
	OF_PROP_READ_S32(dnode, battemp_me.period_ms_normal,
		"period-ms-normal", rc);

	OF_PROP_READ_S32(dnode, battemp_me.cool_mv_alert,
		"cool-mv-alert", rc);
	OF_PROP_READ_S32(dnode, battemp_me.cool_ma_alert,
		"cool-ma-alert", rc);
	OF_PROP_READ_S32(dnode, cool_ma_pct, "cool-ma-pct", rc);
		battemp_me.cool_ma_normal = (mincap * cool_ma_pct / 100) / SCALE_UNIT_MA * SCALE_UNIT_MA;

	OF_PROP_READ_S32(dnode, battemp_me.warm_mv_float,
		"warm-mv-float", rc);
	OF_PROP_READ_S32(dnode, warm_ma_pct, "warm-ma-pct", rc);
		battemp_me.warm_ma_charge = (mincap * warm_ma_pct / 100) / SCALE_UNIT_MA * SCALE_UNIT_MA;

	battemp_me.chilly_is_supported = of_property_read_bool(dnode, "lge,chilly-status-support");
	if (battemp_me.chilly_is_supported) {
		OF_PROP_READ_S32(dnode, battemp_me.chilly_degc_lowerbound,
			"chilly-degc-lowerbound", rc);
		OF_PROP_READ_S32(dnode, battemp_me.chilly_degc_upperbound,
			"chilly-degc-upperbound", rc);
		OF_PROP_READ_S32(dnode, battemp_me.chilly_mv_alert,
			"chilly-mv-alert", rc);
		OF_PROP_READ_S32(dnode, battemp_me.chilly_ma_alert,
			"chilly-ma-alert", rc);
		OF_PROP_READ_S32(dnode, chilly_ma_pct, "chilly-ma-pct", rc);
			battemp_me.chilly_ma_normal =
				(mincap * chilly_ma_pct / 100) / SCALE_UNIT_MA * SCALE_UNIT_MA;
	}

	return !rc;
}

static bool battemp_create_voters(void) {
	return veneer_voter_register(&battemp_me.voter_ichilly, VOTER_NAME_CHILLY, VOTER_TYPE_IBAT, false)
		&& veneer_voter_register(&battemp_me.voter_icharge, VOTER_NAME_ICHARGE, VOTER_TYPE_IBAT, false)
		&& veneer_voter_register(&battemp_me.voter_vfloat, VOTER_NAME_VFLOAT, VOTER_TYPE_VFLOAT, false);
}

static bool battemp_create_preset(bool (*feed_protection_battemp)(bool* charging, int* temperature, int* mvoltage),
	void (*back_protection_battemp)(int health, int micharge, int mvfloat)) {

	if( feed_protection_battemp && back_protection_battemp ) {
		battemp_me.get_protection_battemp = feed_protection_battemp;
		battemp_me.set_protection_battemp = back_protection_battemp;
	}
	else {
		pr_battemp(ERROR, "feed/back func should not be null\n");
		return false;
	}

	wakeup_source_init(&battemp_me.battemp_wakelock,
		BATTEMP_WAKELOCK);

	INIT_DELAYED_WORK(&battemp_me.battemp_dwork,
		polling_status_work);

	return true;
}

void protection_battemp_monitor(void) {
	if (delayed_work_pending(&battemp_me.battemp_dwork))
		cancel_delayed_work(&battemp_me.battemp_dwork);
	schedule_delayed_work(&battemp_me.battemp_dwork, msecs_to_jiffies(0));
}

bool protection_battemp_create(struct device_node* dnode, int mincap,
	bool (*feed_protection_battemp)(bool* charging, int* temperature, int* mvoltage),
	void (*back_protection_battemp)(int health, int micharge, int mvfloat)) {
	pr_debugmask = ERROR | UPDATE;

	if (!battemp_create_preset(feed_protection_battemp, back_protection_battemp)) {
		pr_battemp(ERROR, "error on battemp_create_preset");
		goto destroy;
	}

	if (!battemp_create_parsedt(dnode, mincap)) {
		pr_battemp(ERROR, "error on battemp_create_devicetree");
		goto destroy;
	}

	if (!battemp_create_voters()) {
		pr_battemp(ERROR, "error on battemp_create_voters");
		goto destroy;
	}

	protection_battemp_monitor();
	pr_battemp(UPDATE, "Complete to create\n");
	return true;
destroy:
	protection_battemp_destroy();
	return false;
}

void protection_battemp_destroy(void) {
	wakeup_source_trash(&battemp_me.battemp_wakelock);
	cancel_delayed_work_sync(&battemp_me.battemp_dwork);

	veneer_voter_unregister(&battemp_me.voter_ichilly);
	veneer_voter_unregister(&battemp_me.voter_icharge);
	veneer_voter_unregister(&battemp_me.voter_vfloat);

	battemp_me.get_protection_battemp = NULL;
	battemp_me.set_protection_battemp = NULL;

	battemp_me.health_chilly = false;
	battemp_me.health_jeita = POWER_SUPPLY_HEALTH_UNKNOWN;

	battemp_me.threshold_degc_upto_cool   = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_upto_good   = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_upto_warm   = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_upto_hot    = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_downto_warm = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_downto_good = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_downto_cool = BATTEMP_NOTREADY;
	battemp_me.threshold_degc_downto_cold = BATTEMP_NOTREADY;

	battemp_me.period_ms_emergency = BATTEMP_NOTREADY;
	battemp_me.period_ms_warning   = BATTEMP_NOTREADY;
	battemp_me.period_ms_normal    = BATTEMP_NOTREADY;

	battemp_me.cool_mv_alert  = BATTEMP_NOTREADY,
	battemp_me.cool_ma_alert  = BATTEMP_NOTREADY,
	battemp_me.cool_ma_normal = BATTEMP_NOTREADY,
	battemp_me.warm_ma_charge = BATTEMP_NOTREADY;
	battemp_me.warm_mv_float  = BATTEMP_NOTREADY;

	battemp_me.chilly_is_supported    = false;
	battemp_me.chilly_degc_upperbound = BATTEMP_NOTREADY;
	battemp_me.chilly_degc_lowerbound = BATTEMP_NOTREADY;
	battemp_me.chilly_mv_alert	  = BATTEMP_NOTREADY;
	battemp_me.chilly_ma_alert	  = BATTEMP_NOTREADY;
	battemp_me.chilly_ma_normal       = BATTEMP_NOTREADY;
}


