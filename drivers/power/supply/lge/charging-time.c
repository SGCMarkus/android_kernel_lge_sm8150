#define pr_fmt(fmt) "CHGTIME: %s: " fmt, __func__
#define pr_chgtime(reason, fmt, ...)			\
do {							\
	if (pr_debugmask & (reason))			\
		pr_info(fmt, ##__VA_ARGS__);		\
	else						\
		pr_debug(fmt, ##__VA_ARGS__);		\
} while (0)

static int pr_debugmask;

#define EMPTY			-1
#define NOTYET			-1
#define PROFILE_SLOT_COUNT	256
#define SAMPLING_PERIOD_MS	1500

#include <linux/of.h>
#include <linux/slab.h>

#include "veneer-primitives.h"

struct profile_data {
	int power;
	int decipct;
	int dt_remaining[PROFILE_SLOT_COUNT];
};

#define BSM_VOTER "BSM"
static struct charging_time {
	/* structures for operating */
	struct device_node*	chgtime_dnode;
	struct delayed_work	chgtime_dwork;
	bool		      (*chgtime_get)(int* power, int* rawsoc, int* bsm_watt);
	void		      (*chgtime_notify)(int power);
	int			chgtime_fullraw;	// the raw soc for the rescaled 100%

	bool support_dd;	// support dual-display for flash
	bool is_wireless;
	/* overstatement_coefficient = weight / base */
	int	overstatement_enabled;
	int	overstatement_weight;
	int	overstatement_base;
	/* ceiling params for max charging (mw) */
	int	maxcharging_chargerlogo;
	int	maxcharging_normal;

	/* static properties on charger profiles */
	struct profile_data *charging_profile;
	int	charging_profile_size;

	int	profile_power;
	int	profile_consuming[PROFILE_SLOT_COUNT];
	int	profile_remaining[PROFILE_SLOT_COUNT];

	/* runtime properties */
	int	rawsoc_begin;
	int	rawsoc_now;
	int	rawsoc_profile;
	int	decipct;
	int	runtime_consumed[PROFILE_SLOT_COUNT];
	int	runtime_remained[PROFILE_SLOT_COUNT];
	int	runtime_reported[PROFILE_SLOT_COUNT];

	/* Timestamps */
	long	starttime_of_charging;
	long	starttime_of_rawsoc;

	/* BSM */
	int	bsm_ttf;
	struct voter_entry voter_ibat;
	int mincap;
	int min_remained[PROFILE_SLOT_COUNT];
} time_me;

extern int lge_get_dual_display_support(void);

static int chgtime_upperbound(void) {
	return (!unified_bootmode_chargerlogo() && time_me.maxcharging_normal)
		? time_me.maxcharging_normal : INT_MAX;
}

static bool chgtime_profiling(int power, int rawsoc, int bsm_ttf) {
	int upper_power = INT_MAX, lower_power = 0;
	int upper_time, lower_time;
	int upper_index, lower_index;
	int current_ttf = -1;
	int numerator, denominator;
	int i = 0;;

	upper_index = i;
	upper_power = time_me.charging_profile[upper_index].power;
	upper_time = time_me.charging_profile[upper_index].dt_remaining[rawsoc];
	for (i = 0; i < time_me.charging_profile_size; i++) {
		lower_index = i;
		lower_power = time_me.charging_profile[lower_index].power;
		lower_time = time_me.charging_profile[lower_index].dt_remaining[rawsoc];

		if (power < lower_power || current_ttf > lower_time) {
			upper_power = lower_power;
			upper_index = lower_index;
			upper_time = lower_time;
			/* Go through to next iteration */
		} else {
			if (current_ttf == -1) {
				numerator = upper_power - lower_power ? upper_power-power : 1;
				denominator = upper_power - lower_power ? upper_power - lower_power : 1;

				current_ttf = upper_time
					+ (lower_time - upper_time) * numerator / denominator;

				if (bsm_ttf <= 0 || current_ttf >= time_me.min_remained[rawsoc]) {
					pr_chgtime(UPDATE, "BSM don't working\n");
					break;
				}

				current_ttf += bsm_ttf;
				current_ttf = current_ttf >= time_me.min_remained[rawsoc] ? time_me.min_remained[rawsoc] : current_ttf;

				if (current_ttf > lower_time)
					continue;
			}

			numerator = upper_time - lower_time ? upper_time-current_ttf : 1;
			denominator = upper_time - lower_time ? upper_time - lower_time : 1;
			break;
		}
	}
	pr_chgtime(UPDATE, "TTF %d(%d) < %d(%d) < %d (%d), factor: %d/%d\n",
		upper_time, upper_power, current_ttf, power, lower_time, lower_power, numerator, denominator);

	if (time_me.support_dd) {
		if (time_me.is_wireless && !lge_get_dual_display_support())
			time_me.decipct = time_me.charging_profile[lower_index].decipct;
		else
			time_me.decipct = 1000;
	};

	memset(time_me.profile_remaining, 0, sizeof(int)*PROFILE_SLOT_COUNT);
	for (i = 0; i < PROFILE_SLOT_COUNT; i++) {
		/* Calculate 'profile_remaining' for each soc */
		lower_time = time_me.charging_profile[lower_index].dt_remaining[i];
		upper_time = time_me.charging_profile[upper_index].dt_remaining[i];

		/* start point set to fullraw-2 for displaying +0 min at UI 100% */
		time_me.profile_remaining[i] = upper_time + (lower_time - upper_time) * numerator /denominator;

		/* Print for debugging */
		pr_chgtime(VERBOSE, "SoC %2d : Profiled remains : %5d\n", i, time_me.profile_remaining[i]);
	}

	return true;
}

static void chgtime_update_bsm_fcc(int mincap, int rawsoc, int bsm_ttf) {
	int bsm_fcc;
	int time_per_soc;

	if (bsm_ttf && time_me.profile_remaining[rawsoc] <= time_me.min_remained[rawsoc]) {
		time_per_soc = rawsoc >= PROFILE_SLOT_COUNT - 1 ? 0:
			time_me.profile_remaining[rawsoc] -time_me.profile_remaining[rawsoc + 1];
		bsm_fcc = mincap * 60 *60 /(PROFILE_SLOT_COUNT * time_per_soc);

		pr_chgtime(UPDATE, "BSM FCC %d, time_per_soc %d\n", bsm_fcc, time_per_soc);
		bsm_fcc = bsm_fcc/50*50;
		veneer_voter_set(&time_me.voter_ibat, bsm_fcc);

		time_me.is_wireless = false;
	} else {
		veneer_voter_release(&time_me.voter_ibat);
	}
}

static void chgtime_dworkf(struct work_struct *work) {
	int power, rawsoc, bsm_ttf;

	if (time_me.chgtime_get(&power, &rawsoc, &bsm_ttf) ) {
		power =  min(power, chgtime_upperbound());
	} else {
		pr_chgtime(ERROR, "Error chgtime_get\n");
		return;
	}

	if (time_me.profile_power != power || time_me.bsm_ttf != bsm_ttf) {
		pr_chgtime(UPDATE, "One more sampling (power:%dmw, bsm:%d)\n",
			power, bsm_ttf);

		/* Input power is lower than 2500mW is not permitted to be updated */
		if (power >= 2500) {
			/* Update information */
			time_me.profile_power = power;
		}
		time_me.bsm_ttf = bsm_ttf;
		time_me.rawsoc_profile = rawsoc;

		schedule_delayed_work(&time_me.chgtime_dwork,
			msecs_to_jiffies(SAMPLING_PERIOD_MS));

		return;
	}

	chgtime_profiling(time_me.profile_power, time_me.rawsoc_profile, time_me.bsm_ttf);
	chgtime_update_bsm_fcc(time_me.mincap, time_me.rawsoc_profile, time_me.bsm_ttf);
	time_me.chgtime_notify(time_me.profile_power);
}

static void chgtime_evaluate(long eoc) {
	// Evaluation has meaning only on charging termination (== soc 100%)
	int i, begin_soc = time_me.rawsoc_begin;
	int really_remained[PROFILE_SLOT_COUNT + 1];

	if (begin_soc == time_me.chgtime_fullraw) {
		/* If charging is started on 100%,
		 * Skip to evaluate
		 */
		return;
	}

	memset(really_remained, 0, sizeof(really_remained));
	for (i = time_me.chgtime_fullraw; begin_soc <= i; i--)
		really_remained[i] = time_me.runtime_consumed[i] + really_remained[i+1];

	pr_chgtime(EVALUATE, "Evaluating... %d[mW] charging from %2d(%ld) to 100(%ld), (duration %ld)\n",
		time_me.profile_power, begin_soc, time_me.starttime_of_charging, eoc, eoc-time_me.starttime_of_charging);

	pr_chgtime(EVALUATE, ", soc, really consumed, really remained"
		", profiled remaining, profiled consuming\n");
	for (i = begin_soc; i < PROFILE_SLOT_COUNT; i++) {
		pr_chgtime(EVALUATE, ", %d, %d, %d, %d, %d\n",
			i, time_me.runtime_consumed[i], really_remained[i],
			time_me.profile_remaining[i], time_me.profile_consuming[i]
		);
	}
}

int charging_time_remains(int rawsoc) {
	long		now;
	struct timespec tspec;

	// Simple check
	if ( !(0 < time_me.profile_power && 0 <= rawsoc && rawsoc < PROFILE_SLOT_COUNT) ) {
		/* Invalid invokation */
		return NOTYET;
	}

	// This calling may NOT be bound with SoC changing
	if (time_me.rawsoc_now != rawsoc) {
		get_monotonic_boottime(&tspec);
		now = tspec.tv_sec;

		if (time_me.starttime_of_charging == EMPTY) {
			// New insertion
			time_me.rawsoc_begin = rawsoc;
			time_me.starttime_of_charging = now;
		}
		else {	// Soc rasing up
			time_me.runtime_consumed[rawsoc > 0 ? rawsoc-1 : 0]
				= now - time_me.starttime_of_rawsoc;
		}

		/* Update time_me */
		time_me.rawsoc_now = rawsoc;
		time_me.starttime_of_rawsoc = now;

		if (rawsoc == time_me.chgtime_fullraw) {
			/* Evaluate NOW! (at the rescaled 100% soc) :
			 * Evaluation has meaning only on full(100%) charged status
			 */
			chgtime_evaluate(now);
		}
		else {
			pr_chgtime(UPDATE, "rawsoc %d, elapsed %ds...\n",
				rawsoc, time_me.runtime_consumed[rawsoc > 0 ? rawsoc-1 : 0]);
		}
	}

	// Overstate if needed
	if (time_me.overstatement_enabled)
		time_me.runtime_reported[rawsoc]
			= (time_me.overstatement_base + time_me.overstatement_weight)
			* time_me.profile_remaining[rawsoc]
			/ time_me.overstatement_base;
	else
		time_me.runtime_reported[rawsoc]
			= time_me.profile_remaining[rawsoc];

	if (time_me.support_dd) {
		if (time_me.is_wireless && !lge_get_dual_display_support())
			return time_me.runtime_reported[rawsoc] * time_me.decipct / 1000;
	}

	return time_me.runtime_reported[rawsoc];
}

bool charging_time_update(enum charging_supplier charger, bool reloading) {
	static enum charging_supplier type = CHARGING_SUPPLY_TYPE_NONE;
	bool charging;

	if (reloading)
		charger = type;

	if (type != charger || reloading) {
		charging = charger != CHARGING_SUPPLY_TYPE_UNKNOWN
			&& charger != CHARGING_SUPPLY_TYPE_NONE;

		if (charging) {
			pr_chgtime(UPDATE, "Charging started, Start sampling\n");
			cancel_delayed_work_sync(&time_me.chgtime_dwork);
			schedule_delayed_work(&time_me.chgtime_dwork,
				msecs_to_jiffies(SAMPLING_PERIOD_MS));
			time_me.is_wireless = false;
			if (charger == CHARGING_SUPPLY_WIRELESS_5W ||
				charger == CHARGING_SUPPLY_WIRELESS_9W)
				time_me.is_wireless = true;
		}
		else {
			pr_chgtime(UPDATE, "Charging stopped\n");
			charging_time_clear();
		}

		type = charger;
		return true;
	}
	else {
		pr_chgtime(VERBOSE, "Skip to initiate\n");
		return false;
	}
}

void charging_time_clear(void) {
	int i;
	cancel_delayed_work_sync(&time_me.chgtime_dwork);

	// PRESERVE '_remains[256]' as '0'
	for (i = 0; i < PROFILE_SLOT_COUNT; i++) {
		time_me.profile_consuming[i] = EMPTY;
		time_me.profile_remaining[i] = EMPTY;

		time_me.runtime_consumed[i] = EMPTY;
		time_me.runtime_remained[i] = EMPTY;
	}

	/* For runtime values */
	time_me.rawsoc_begin = NOTYET;
	time_me.rawsoc_now = NOTYET;
	time_me.starttime_of_charging = NOTYET;
	time_me.starttime_of_rawsoc = NOTYET;

	/* For beginning status */
	time_me.profile_power = NOTYET;

	veneer_voter_release(&time_me.voter_ibat);
}

void charging_time_destroy(void) {
	// dwork should be canceled before setting pointers to null
	charging_time_clear();

	time_me.chgtime_dnode = NULL;
	time_me.chgtime_get = NULL;
	time_me.chgtime_notify = NULL;
	veneer_voter_unregister(&time_me.voter_ibat);
}

static void charging_time_get_bsm_mincharging(int mincap, int fullraw, int min_charging_ma) {
	int mintime_per_soc = mincap * 60 *60 /(PROFILE_SLOT_COUNT * min_charging_ma);
	int delta_per_soc = 0, i = 0;

	pr_chgtime(UPDATE, "min_charging_ma: %d mA, mintime_per_soc %d\n", min_charging_ma, mintime_per_soc);
	memset(time_me.min_remained, 0, sizeof(int) * PROFILE_SLOT_COUNT);
	for (i = fullraw - 2; i >=0 ; i--) {
		/* Calculate 'profile_remaining' for each soc */
		delta_per_soc = time_me.charging_profile[0].dt_remaining[i] -time_me.charging_profile[0].dt_remaining[i+1];

		/* start point set to fullraw-2 for displaying +0 min at UI 100% */
		time_me.min_remained[i] = delta_per_soc >= mintime_per_soc ? delta_per_soc : mintime_per_soc;
		time_me.min_remained[i] += time_me.min_remained[i+1];

		/* Print for debugging */
		pr_chgtime(VERBOSE, "SoC %2d : Profiled remains : %5d, %d\n", i, time_me.min_remained[i],  delta_per_soc);
	}
}

static int charging_time_dt(struct device_node* dnode, int fullraw, int mincap) {
	struct device_node* child_dnode; // uses as a iterator
	struct profile_data *sub_profile;
	u32 charger_profile [PROFILE_SLOT_COUNT];
	int i, rc = 0;
	int min_charging_ma;

	time_me.support_dd = of_property_read_bool(dnode, "lge,support-dd");
	rc |= of_property_read_u32(dnode, "lge,maxcharging-mw-chargerlogo", &time_me.maxcharging_chargerlogo);
	rc |= of_property_read_u32(dnode, "lge,maxcharging-mw-normal", &time_me.maxcharging_normal);
	rc |= of_property_read_u32(dnode, "lge,overstatement-enable", &time_me.overstatement_enabled);
	rc |= of_property_read_u32(dnode, "lge,bsm-mincharging-ma", &min_charging_ma);
	if (time_me.overstatement_enabled) {
		rc |= of_property_read_u32(dnode, "lge,overstatement-weight", &time_me.overstatement_weight);
		rc |= of_property_read_u32(dnode, "lge,overstatement-base", &time_me.overstatement_base);
	}

	/* battery charger regualtors */
	time_me.charging_profile_size = of_get_child_count(dnode);
	if (time_me.charging_profile_size) {
		sub_profile = kzalloc(sizeof(struct profile_data) * time_me.charging_profile_size, GFP_KERNEL);

		if (!sub_profile)
			return -ENOMEM;

		time_me.charging_profile = sub_profile;

		for_each_child_of_node(dnode, child_dnode) {
			if (of_property_read_u32(child_dnode, "charger-power", &sub_profile->power))
				return -EINVAL;

			if (time_me.support_dd) {
				if (!lge_get_dual_display_support()) {
					if (of_property_read_u32(child_dnode,
							"charger-decipct", &sub_profile->decipct))
						sub_profile->decipct = 1000;
				}
			}

			if (of_property_read_u32_array(child_dnode, "charger-profile", charger_profile, PROFILE_SLOT_COUNT))
				return -EINVAL;

			/* Calculate 'profile_remaining' for each soc */
			/* start point set to fullraw-2 for displaying +0 min at UI 100% */
			memset(sub_profile->dt_remaining, 0, sizeof(int) * PROFILE_SLOT_COUNT);
			for (i = fullraw - 2; 0 <= i; i--) {
				sub_profile->dt_remaining[i] = charger_profile[i] + sub_profile->dt_remaining[i+1];
			}

			pr_chgtime(MONITOR, "power %d\n", sub_profile->power);
			for (i = 0; i < PROFILE_SLOT_COUNT; i++)
				pr_chgtime(MONITOR, "charger-profile[%d] =%d \n", i, sub_profile->dt_remaining[i]);

			sub_profile++;
		}
	} else {
		rc |= -EINVAL;
	}
	charging_time_get_bsm_mincharging(mincap, fullraw, min_charging_ma);

	return rc;
}

bool charging_time_create(struct device_node* dnode, int fullraw, int mincap,
	bool (*feed_charging_time)(int* power, int* rawsoc, int* bsm_ttf),
	void (*back_charging_time)(int power)) {
	pr_debugmask = ERROR | UPDATE | EVALUATE;

	if (dnode && feed_charging_time && back_charging_time) {
		time_me.chgtime_dnode = dnode;
	}
	else
		goto fail;

	if (0 > fullraw || fullraw >= PROFILE_SLOT_COUNT) {
		pr_chgtime(ERROR, "'chgtime_fullraw'(%d) is out of range\n", fullraw);
		goto fail;
	}
	else
		time_me.chgtime_fullraw = fullraw;

	if (charging_time_dt(dnode, fullraw, mincap)) {
		pr_chgtime(ERROR, "'Error of parsing dt file.\n");
		goto fail;
	}
	if (!veneer_voter_register(&time_me.voter_ibat, BSM_VOTER, VOTER_TYPE_IBAT, false)) {
		pr_chgtime(ERROR, "Error regiter of voter.\n");
		goto fail;
	}

	time_me.chgtime_get = feed_charging_time;
	time_me.chgtime_notify = back_charging_time;
	INIT_DELAYED_WORK(&time_me.chgtime_dwork, chgtime_dworkf);

	/* Redundant initializing, but be sure to PRESERVE '_remains[PROFILE_SLOT_COUNT]' as '0' */
	time_me.profile_remaining[PROFILE_SLOT_COUNT-1] = 0;
	time_me.runtime_remained[PROFILE_SLOT_COUNT-1] = 0;
	time_me.mincap = mincap;

	return true;

fail:	pr_chgtime(ERROR, "Failed to create charging time\n");
	return false;
}

