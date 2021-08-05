#define pr_fmt(fmt) "BVP: %s: " fmt, __func__
#define pr_batvolt(reason, fmt, ...)			\
do {							\
	if (pr_debugmask & (reason))			\
		pr_err(fmt, ##__VA_ARGS__);		\
	else						\
		pr_debug(fmt, ##__VA_ARGS__);		\
} while (0)

static int pr_debugmask;

#include <linux/of.h>
#include <linux/workqueue.h>

#include "veneer-primitives.h"

#define BVP_NOTREADY	INT_MAX
#define BVP_VOTER	"BVP"

static void protection_batvolt_work(struct work_struct *work);

static struct protection_batvolt_struct {
	/* for protection behavoirs */
	bool		      (*bvp_get)(int* vnow_mv, int* icap_ma, int* chg_type);
	struct voter_entry	bvp_voter;
	struct delayed_work	bvp_dwork;
	/* thresholds */
	int			threshold_vbat_limit;
	int			threshold_vbat_clear;
	int			threshold_ibat_rated;
	int			threshold_cv_ibat_rated;
	/* dt contents */
	int			step_ibat_ma;
	int			step_poll_ms;
} bvp_me = {
	.bvp_get	= NULL,
	.bvp_voter	= { .type = VOTER_TYPE_INVALID },
	.bvp_dwork	= __DELAYED_WORK_INITIALIZER(bvp_me.bvp_dwork,
		protection_batvolt_work, 0),

	.threshold_vbat_limit		= BVP_NOTREADY,
	.threshold_vbat_clear		= BVP_NOTREADY,
	.threshold_ibat_rated		= BVP_NOTREADY,
	.threshold_cv_ibat_rated	= BVP_NOTREADY,

	.step_ibat_ma		= BVP_NOTREADY,
	.step_poll_ms		= BVP_NOTREADY,
};

static void protection_batvolt_work(struct work_struct *work) {
	// Filled from client
	int vbat_now, icap_now, chg_now;
	// Set in this work
	int icap_new;

	if (bvp_me.bvp_get(&vbat_now, &icap_now, &chg_now)) {
		if (vbat_now <= bvp_me.threshold_vbat_limit) {
			pr_batvolt(VERBOSE, "Under voltage (%d)\n", vbat_now);

			if (veneer_voter_enabled(&bvp_me.bvp_voter) && vbat_now <= bvp_me.threshold_vbat_clear) {
				pr_batvolt(UPDATE, "Clear batvolt protection\n");
				veneer_voter_release(&bvp_me.bvp_voter);
			}

			goto done;
		}

		if (chg_now != POWER_SUPPLY_CHARGE_TYPE_TAPER
				&& icap_now <= bvp_me.threshold_ibat_rated) {
			pr_batvolt(VERBOSE, "Under C-rate (%d)\n", icap_now);
			goto done;
		}

		if (chg_now == POWER_SUPPLY_CHARGE_TYPE_TAPER
				&& icap_now <= bvp_me.threshold_cv_ibat_rated) {
			pr_batvolt(VERBOSE, "Under C-rate (%d) on CV\n", icap_now);
			goto done;
		}

	}
	else {
		pr_batvolt(UPDATE, "Host is not ready\n");
		goto done;
	}

	icap_new = (icap_now - bvp_me.step_ibat_ma) / bvp_me.step_ibat_ma * bvp_me.step_ibat_ma;
	veneer_voter_set(&bvp_me.bvp_voter, icap_new);

	pr_batvolt(UPDATE, "Condition : %dmv, %dma, chg type %d => Reduce IBAT to %d\n",
		vbat_now, icap_now, chg_now, icap_new);
done:
	schedule_delayed_work(to_delayed_work(work), msecs_to_jiffies(bvp_me.step_poll_ms));
	return;
}

void protection_batvolt_refresh(bool is_charging) {
	static bool is_started = false;

	bool is_ready = bvp_me.threshold_vbat_limit != BVP_NOTREADY
		&& bvp_me.threshold_ibat_rated != BVP_NOTREADY
		&& is_charging;

	if (is_ready) {
		if (!is_started) {
			schedule_delayed_work(&bvp_me.bvp_dwork, 0);
			is_started = true;
		}
		else
			; // Skip to handle BVP
	}
	else {
		veneer_voter_release(&bvp_me.bvp_voter);
		cancel_delayed_work_sync(&bvp_me.bvp_dwork);
		is_started = false;
	}
}

bool protection_batvolt_create(struct device_node* dnode, int mincap,
	bool (*feed_protection_batvolt)(int* vnow_mv, int* icap_ma, int* chg_type)) {
	int ret = 0, threshold_ibat_pct = 0, threshold_cv_ibat_pct = 0;
	pr_debugmask = ERROR | UPDATE;

	/* Parse device tree */
	ret = of_property_read_s32(dnode, "lge,threshold-ibat-pct", &threshold_ibat_pct);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,threshold-ibat-pct' ret=%d\n", ret);
		goto destroy;
	}
	else
		bvp_me.threshold_ibat_rated = mincap * threshold_ibat_pct / 100;

	ret = of_property_read_s32(dnode, "lge,threshold-cv-ibat-pct", &threshold_cv_ibat_pct);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,threshold-cv-ibat-pct' ret=%d\n", ret);
		goto destroy;
	}
	else
		bvp_me.threshold_cv_ibat_rated = mincap * threshold_cv_ibat_pct / 100;

	ret = of_property_read_s32(dnode, "lge,threshold-vbat-limit", &bvp_me.threshold_vbat_limit);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,threshold-vbat-limit' ret=%d\n", ret);
		goto destroy;
	}
	ret = of_property_read_s32(dnode, "lge,threshold-vbat-clear", &bvp_me.threshold_vbat_clear);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,threshold-vbat-clear' ret=%d\n", ret);
		goto destroy;
	}
	ret = of_property_read_s32(dnode, "lge,step-ibat-ma", &bvp_me.step_ibat_ma);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,step-ibat-ma' ret=%d\n", ret);
		goto destroy;
	}
	ret = of_property_read_s32(dnode, "lge,step-poll-ms", &bvp_me.step_poll_ms);
	if (ret < 0) {
		pr_batvolt(ERROR, "Failed to read 'lge,step-poll-ms' ret=%d\n", ret);
		goto destroy;
	}

	/* Fill callback */
	if (!feed_protection_batvolt) {
		pr_batvolt(ERROR, "feed func should not be null\n");
		goto destroy;
	}
	else
		bvp_me.bvp_get = feed_protection_batvolt;

	/* Register voter */
	if (!veneer_voter_register(&bvp_me.bvp_voter, BVP_VOTER, VOTER_TYPE_IBAT, false)) {
		pr_batvolt(ERROR, "Failed to register the BVP voter\n");
		goto destroy;
	}

	pr_batvolt(UPDATE, "Complete to create, "
		"threshold_vbat_limit(%d), threshold_vbat_clear(%d), threshold_ibat_rated(%d), "
		"step_ibat_ma(%d), step_poll_ms(%d)\n",
		bvp_me.threshold_vbat_limit, bvp_me.threshold_vbat_clear, bvp_me.threshold_ibat_rated,
		bvp_me.step_ibat_ma, bvp_me.step_poll_ms);

	return true;

destroy:
	protection_batvolt_destroy();
	return false;
}

void protection_batvolt_destroy(void) {
	cancel_delayed_work_sync(&bvp_me.bvp_dwork);
	veneer_voter_unregister(&bvp_me.bvp_voter);
	bvp_me.bvp_get = NULL;

	bvp_me.threshold_vbat_limit	= BVP_NOTREADY;
	bvp_me.threshold_vbat_clear	= BVP_NOTREADY;
	bvp_me.threshold_ibat_rated	= BVP_NOTREADY;

	bvp_me.step_ibat_ma		= BVP_NOTREADY;
	bvp_me.step_poll_ms		= BVP_NOTREADY;
}
