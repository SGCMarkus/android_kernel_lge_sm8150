/**********************************************************
 *
 *  this source expands the qnovo driver function for LGE
 *
 **********************************************************/

#define QNA_UNSAFETY_VOTER          "QNA_UNSAFETY_VOTER"
#define CHG_PROBATION_VOTER         "QNI_PROBATION_VOTER"

#define DC_VOLTAGE_MV_BPP           5000000
#define NPULS_CURRENT_MA            1550
#define TAPER_CONFIRM_TIME          2000      // 2sec

/* QNI Probation Parameter */
#define QNI_RT_MONITOR_TIME         5000      // 5sec
#define QNI_STEP_MIN_FCC            800000    // 80mA
#define CHG_PROBATION_MIN_FV        4380000   // 4.38V
#define CHG_PROBATION_MAX_FCC       950000    // 950mA
#define CHG_PROBATION_ENTER_FCC     1000000   // 1A

#define QNA_UNSAFETY_FV             4300000   // 4.3V
#define QNA_UNSAFETY_FCC            1000000   // 1.0A

#define QNI_SKEW_COMP_PCT           100
#define QNI_PT_TIME_MAX             512

enum {
	QPS_NOT_READY = -1,     /* before reading from device tree              */
	QPS_DISABLED = 0,       /* qni probation is disabled by device tree     */
	QPS_ENABLED,            /* qni probation is enabled by device tree      */
	QPS_QNI_READY,          /* qni daemon ready                             */
	QPS_PRE_DEFINED_WLC,    /* pre defined qni probation enabled for WLC    */
	QPS_PRE_DEFINED_USB,    /* pre defined qni probation enabled for USB    */
	QPS_RUNTIME_READY,      /* run-time qni probation ready                 */
	QPS_IN_RUNTIME,         /* run-time qni probation enabled               */
	QPS_IN_CV_MODE,         /* enter cv mode                                */
	QPS_MAX
};

static char qps_str[QPS_MAX+1][20] = {
	"NOT_READY",
	"DISABLED",
	"ENABLED",
	"QNI_READY",
	"PRE_DEFINED_WLC",
	"PRE_DEFINED_USB",
	"RUNTIME_READY",
	"IN_RUNTIME",
	"IN_CV_MODE"
};

struct _extension_qnovo {
	struct qnovo *me;

	int health;
	int set_fcc;
	int locked_fv;
	int locked_fcc;
	bool is_dc;
	int dc_volt;

	int status;
	bool is_qnovo_ready;
	int qni_probation_status;

	struct __qna {
		int meter;
		int safety;
		int vt_warn;
		int pcap;
	} qna;

	struct power_supply	*bms_psy;
	struct power_supply	*main_psy;
	struct power_supply	*veneer_psy;
	struct power_supply	*wireless_psy;

	int taper_count;
	int fcc_comp_count;
	int fcc_comp_old_fcc;
	int fcc_comp_old_curr;
	int fcc_comp_count_max;
	int fcc_comp_ready_count;
	int pt_done_watchdog_count;
	int pt_start_watchdog_count;

	struct __pt_manager {
		/* input ready */
		int cnt;
		int best;
		int worst;

		/* diag stage */
		int diag_count;

		/* pt_start & pt_done */
		u8 pp_cnt;
		u8 pe_ctrl;
		int target;
		unsigned long start;

		/* skew of learned capacity */
		int skew_cc;
		int skew_ratio;
		int skew_count;
	} pt_manager;

	struct delayed_work	input_ready_work;
	struct delayed_work	taper_confirm_work;
	struct delayed_work	rt_monitor_fcc_work;
	struct delayed_work	pt_done_watchdog_work;
	struct delayed_work	pt_start_watchdog_work;
} ext_qnovo;

struct _extension_qnovo_dt_props {
	bool	enable_for_dc;
	bool	enable_qni_probation;
	int	qni_step_min_fcc;
	int	qni_probation_min_fv;
	int	qni_probation_max_fcc;
	int	qni_probation_enter_fcc;

	bool	enable_fcc_compensate;

	bool	enable_qna_unsafety_protection;
	int	qna_unsafety_fv;
	int	qna_unsafety_fcc;

	int	qni_skew_comp_pct;

	int qni_pt_time_max;
} ext_dt;

static bool is_fg_available(void)
{
	if (!ext_qnovo.bms_psy)
		ext_qnovo.bms_psy = power_supply_get_by_name("bms");

	if (!ext_qnovo.bms_psy)
		return false;

	return true;
}

static bool is_dc_available(struct qnovo *chip)
{
	if (!chip->dc_psy)
		chip->dc_psy = power_supply_get_by_name("dc");

	if (!chip->dc_psy)
		return false;

	return true;
}

static bool is_main_available(void)
{
	if (!ext_qnovo.main_psy)
		ext_qnovo.main_psy = power_supply_get_by_name("main");

	if (!ext_qnovo.main_psy)
		return false;

	return true;
}

static bool is_veneer_available(void)
{
	if (!ext_qnovo.veneer_psy)
		ext_qnovo.veneer_psy = power_supply_get_by_name("veneer");

	if (!ext_qnovo.veneer_psy)
		return false;

	return true;
}

static bool is_wireless_available(void)
{
	if (!ext_qnovo.wireless_psy)
		ext_qnovo.wireless_psy = power_supply_get_by_name("wireless");

	if (!ext_qnovo.wireless_psy)
		return false;

	return true;
}

static int set_fcc_compensation(int updown)
{
	int rc = 0;
	union power_supply_propval pval = {0, };

	if (!is_main_available())
		return -EINVAL;

	pval.intval = updown;
	rc = power_supply_set_property(ext_qnovo.main_psy,
			POWER_SUPPLY_PROP_FCC_DELTA, &pval);

	return rc;
}

/* set/get qni probation status(qps) */
static int set_qps(int status)
{
	int fix_status = status;
	switch (status) {
		case QPS_NOT_READY:
		case QPS_DISABLED:
		case QPS_ENABLED:
			break;

		case QPS_QNI_READY:
			if (ext_qnovo.is_qnovo_ready)
				fix_status = status;
			else if (status > QPS_DISABLED)
				fix_status = QPS_ENABLED;
			else
				fix_status = QPS_NOT_READY;
			break;

		case QPS_PRE_DEFINED_WLC:
		case QPS_PRE_DEFINED_USB:
		case QPS_RUNTIME_READY:
		case QPS_IN_RUNTIME:
		default:
			break;
	}

	if (ext_qnovo.qni_probation_status != fix_status)
		pr_info("[QNI-PROB] set_qps: %s\n", qps_str[fix_status+1]);

	ext_qnovo.qni_probation_status = fix_status;
	return fix_status;
}

static int get_qps(void)
{
	return ext_qnovo.qni_probation_status;
}

/**********************************************************
 *   Extenstion of QNI function - Runtime QNI Probation   *
 **********************************************************/
static void input_ready_work(struct work_struct *work)
{
	int rc = 0;
	int now_fcc = 0, now_curr = 0;
	union power_supply_propval pval = {0, };

	if (!is_fg_available() ||
		!is_wireless_available() ||
		!is_usb_available(ext_qnovo.me) ||
		!is_batt_available(ext_qnovo.me) )
		return;

	rc = power_supply_get_property(ext_qnovo.me->usb_psy,
			POWER_SUPPLY_PROP_REAL_TYPE, &pval);
	if (rc < 0 ||
		pval.intval == POWER_SUPPLY_TYPE_USB ||
		pval.intval == POWER_SUPPLY_TYPE_UNKNOWN ||
		pval.intval == POWER_SUPPLY_TYPE_USB_FLOAT )
		return;

	rc = power_supply_get_property(ext_qnovo.wireless_psy,
			POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &pval);
	if (rc < 0 || pval.intval == DC_VOLTAGE_MV_BPP)
		return;

	rc = power_supply_get_property(ext_qnovo.bms_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &pval);
	now_curr = !rc ? pval.intval : 0;

	rc = power_supply_get_property(ext_qnovo.me->batt_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &pval);
	now_fcc = !rc ? pval.intval : 0;

	ext_qnovo.pt_manager.cnt++;
	if (now_curr < 0 && ext_qnovo.pt_manager.best > now_curr)
		ext_qnovo.pt_manager.best = now_curr;
	if (now_curr < 0 && ext_qnovo.pt_manager.worst < now_curr)
		ext_qnovo.pt_manager.worst = now_curr;

	pr_info("qni:input_ready cnt=%d, fcc=%d, now=%d, best=%d, worst=%d\n",
		ext_qnovo.pt_manager.cnt,
		now_fcc/1000,
		now_curr/1000,
		ext_qnovo.pt_manager.best/1000,
		ext_qnovo.pt_manager.worst/1000
	);

	if (!get_effective_result(ext_qnovo.me->chg_ready_votable))
		schedule_delayed_work(
			&ext_qnovo.input_ready_work, msecs_to_jiffies(1000));
}

static void pt_start_watchdog_work(struct work_struct *work)
{
	u8 reg = 0;
	int pt_en = 0, pt_time = 0;
	union power_supply_propval debug  = {10, };

	pt_en = qnovo5_read(ext_qnovo.me, QNOVO_PE_CTRL, &reg, 1) >= 0 ? (reg>>7) : -1;
	if (!pt_en) {
		qnovo5_masked_write(ext_qnovo.me, QNOVO_PE_CTRL,
			QNOVO_PTRAIN_EN_BIT, QNOVO_PTRAIN_EN_BIT);
		schedule_delayed_work(
			&ext_qnovo.pt_start_watchdog_work, msecs_to_jiffies(2100));
		return;
	}

	pt_time = qnovo5_read(ext_qnovo.me, QNOVO_PTTIME_LSB, &reg, 1) >= 0	? reg : -1;
	if (pt_time <= 0) {
		ext_qnovo.pt_start_watchdog_count++;
		pr_info("qni:watchdog - pt start bite!! pt_t=%d, cnt=%d\n",
			pt_time, ext_qnovo.pt_start_watchdog_count);

		if (is_batt_available(ext_qnovo.me))
			power_supply_set_property(ext_qnovo.me->batt_psy,
				POWER_SUPPLY_PROP_DEBUG_BATTERY, &debug);

		qnovo5_masked_write(ext_qnovo.me, QNOVO_PE_CTRL,
			QNOVO_PTRAIN_EN_BIT, 0);
		msleep(1000);
		qnovo5_masked_write(ext_qnovo.me, QNOVO_PE_CTRL,
			QNOVO_PTRAIN_EN_BIT, QNOVO_PTRAIN_EN_BIT);

		schedule_delayed_work(
			&ext_qnovo.pt_start_watchdog_work, msecs_to_jiffies(2100));
	}
}

static void pt_done_watchdog_work(struct work_struct *work)
{
	union power_supply_propval debug  = {10, };

	ext_qnovo.pt_done_watchdog_count++;
	pr_info("qni:watchdog - pt done bite!! cnt=%d\n",
		ext_qnovo.pt_done_watchdog_count);

	if (is_batt_available(ext_qnovo.me))
		power_supply_set_property(ext_qnovo.me->batt_psy,
			POWER_SUPPLY_PROP_DEBUG_BATTERY, &debug);

	// fake interrupt
	done_pt_manager(ext_qnovo.me);
	override_qnovo5_update_status(ext_qnovo.me);
	vote(ext_qnovo.me->pt_dis_votable, QNI_PT_VOTER, true, 0);
	kobject_uevent(&ext_qnovo.me->dev->kobj, KOBJ_CHANGE);
}

// It is triggered when start of pules train
static bool set_pt_manager(struct qnovo *chip, unsigned long en)
{
	int rc = 0;
	ext_qnovo.pt_manager.target = 0;

	cancel_delayed_work(&ext_qnovo.pt_done_watchdog_work);
	cancel_delayed_work(&ext_qnovo.pt_start_watchdog_work);

	rc = qnovo5_read(chip, QNOVO_PE_CTRL,
		&ext_qnovo.pt_manager.pe_ctrl, 1);
	if (rc)
		return false;

	if ((ext_qnovo.pt_manager.pe_ctrl & 0x7) == 0x2)
		ext_qnovo.pt_manager.diag_count++;
	else
		ext_qnovo.pt_manager.diag_count = 0;

	if (ext_qnovo.pt_manager.pe_ctrl & BIT(0)) {
		rc = qnovo5_read(chip, QNOVO_PPCNT_MAX_CTRL,
			&ext_qnovo.pt_manager.pp_cnt, 1);
		if (rc)
			return false;

		/* if the difference of qnovo charging current and fcc voter is twice,
		 *  pulse engine operation time will be twice.*/
		ext_qnovo.pt_manager.target += ext_qnovo.pt_manager.pp_cnt*1000*2;
	}
	if (ext_qnovo.pt_manager.pe_ctrl & BIT(1)) {
		// Phase 2: 3sec + Phase 3: 1sec = 4sec
		ext_qnovo.pt_manager.target += 4000;
	}

	if (en) {
		ext_qnovo.pt_manager.start = jiffies;
		schedule_delayed_work(
			&ext_qnovo.pt_start_watchdog_work,
			msecs_to_jiffies(2100));
		schedule_delayed_work(
			&ext_qnovo.pt_done_watchdog_work,
			msecs_to_jiffies(ext_qnovo.pt_manager.target + 5000));
	}

	return true;
}

static int qnovo_skew_set()
{
	int rc = 0;
	union power_supply_propval pval = {0, };

	if (!is_fg_available())
		return rc;

	/* cl.skew coulomb count for qnovo */
	pval.intval = (ext_qnovo.pt_manager.skew_cc * ext_dt.qni_skew_comp_pct
		                                    / QNI_SKEW_COMP_PCT);
	power_supply_set_property(ext_qnovo.bms_psy,
		POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN, &pval);

	/* cl.skew ratio for qnovo */
	pval.intval = ((ext_qnovo.pt_manager.skew_ratio / ext_qnovo.pt_manager.skew_count)
			* ext_dt.qni_skew_comp_pct / QNI_SKEW_COMP_PCT);
	power_supply_set_property(ext_qnovo.bms_psy,
		POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX, &pval);

	return rc;
}

static int qnovo_skew_reset()
{
	int rc = 0;
	union power_supply_propval pval = {0, };

	ext_qnovo.pt_manager.skew_cc = 0;
	ext_qnovo.pt_manager.skew_ratio = 0;
	ext_qnovo.pt_manager.skew_count = 0;

	if (!is_fg_available())
		return rc;

	/* cl.skew coulomb count for qnovo */
	power_supply_set_property(ext_qnovo.bms_psy,
		POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN, &pval);
	/* cl.skew ration for qnovo */
	power_supply_set_property(ext_qnovo.bms_psy,
		POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX, &pval);

	return rc;
}

static int qnovo_skew_count(struct qnovo *chip)
{
	int rc = 0;
	u8 ppcnt, pe_ctrl, nrest1, npuls1, prest1;

	rc = qnovo5_read(chip, QNOVO_PE_CTRL, &pe_ctrl, 1);
	if (pe_ctrl & BIT(0)) {
		rc = qnovo5_read(chip, QNOVO_NREST1_CTRL, &nrest1, 1);
		rc = qnovo5_read(chip, QNOVO_NPULS1_CTRL, &npuls1, 1);
		rc = qnovo5_read(chip, QNOVO_PREST1_CTRL, &prest1, 1);
		rc = qnovo5_read(chip, QNOVO_PPCNT_MAX_CTRL, &ppcnt, 1);

		ext_qnovo.pt_manager.skew_count++;
		ext_qnovo.pt_manager.skew_ratio += (nrest1 + prest1);
		ext_qnovo.pt_manager.skew_cc += (NPULS_CURRENT_MA * npuls1 * ppcnt)/1000;
	}

	return rc;
}

// It is triggered when done of pules train(interrupt)
static bool done_pt_manager(struct qnovo *chip)
{
	bool is_good = true;
	int msecs = 0;
	int target = ext_qnovo.pt_manager.target;
	unsigned long start = ext_qnovo.pt_manager.start;

	cancel_delayed_work(&ext_qnovo.pt_done_watchdog_work);
	cancel_delayed_work(&ext_qnovo.pt_start_watchdog_work);

	msecs = jiffies_to_msecs(jiffies - start);

	if ((max(msecs, target) - min(msecs, target)) > 1500)
		is_good = false;

	if (!is_good) {
		if ((ext_qnovo.pt_manager.pe_ctrl & 0x7) == 0x7 ) // Phase 1+2+3
			target = ext_qnovo.pt_manager.pp_cnt * 1000 + 4000;
		if ((ext_qnovo.pt_manager.pe_ctrl & 0x7) == 0x1 ) // Phase 1
			target = ext_qnovo.pt_manager.pp_cnt * 1000;

		if ((max(msecs, target) - min(msecs, target)) <= 1500)
			is_good = true;
	}

	if (is_good)
		qnovo_skew_count(chip);

	pr_info("qni:handle_ptrain_done: PE health=%s, "
			"PE=0x%X, PPCNT=%d, target=%dmsec, elapse=%dmsec, diag count=%d, "
			"skew_count=%d, skew_cc=%d, skew_ratio=%d\n",
		is_good ? "Good":"Bad",
		ext_qnovo.pt_manager.pe_ctrl, ext_qnovo.pt_manager.pp_cnt,
		target, msecs,
		ext_qnovo.pt_manager.diag_count, ext_qnovo.pt_manager.skew_count,
		ext_qnovo.pt_manager.skew_cc, ext_qnovo.pt_manager.skew_ratio);

	if (ext_qnovo.pt_manager.diag_count > 20) {
		ext_qnovo.pt_manager.diag_count = 0;
		return false;
	}

	return true;
}

static int qnovo_fet_ctrl(struct qnovo *chip)
{
	int rc = 0;

	if (chip->pinctrl) {
		rc = pinctrl_select_state(chip->pinctrl,
				chip->pinctrl_state1);
		if (rc < 0)
			pr_err("Couldn't select state 1 rc=%d\n", rc);

		rc = pinctrl_select_state(chip->pinctrl,
				chip->pinctrl_state2);
		if (rc < 0)
			pr_err("Couldn't select state 2 rc=%d\n", rc);
	}

	return rc;
}

static int enter_qni_probation(struct qnovo *chip)
{
	if (!(get_qps() == QPS_PRE_DEFINED_WLC ||
	      get_qps() == QPS_PRE_DEFINED_USB || get_qps() == QPS_IN_RUNTIME)) {
		pr_info("[QNI-PROB] >>> enter qni probation ERROR: sts=%d, qps=%s\n",
			get_client_vote(chip->not_ok_to_qnovo_votable, CHG_PROBATION_VOTER),
			qps_str[get_qps()+1]);
		return -EINVAL;
	}

	cancel_delayed_work(&ext_qnovo.pt_done_watchdog_work);
	cancel_delayed_work(&ext_qnovo.pt_start_watchdog_work);

	vote(chip->not_ok_to_qnovo_votable, CHG_PROBATION_VOTER, true, 0);
	qnovo_fet_ctrl(chip);

	pr_info("[QNI-PROB] >>> enter qni probation OK!!\n");

	return 0;
}

static int release_qni_probation(struct qnovo *chip)
{
	if (!is_usb_available(chip) || !is_dc_available(chip))
		return -EINVAL;

	/* if both usb and dc isn't connected = discharging.
		it releases run-time rt probation ready flag */
	if (!chip->usb_present && !chip->dc_present) {
		if (get_qps() == QPS_QNI_READY &&
			!get_client_vote(chip->not_ok_to_qnovo_votable,
					CHG_PROBATION_VOTER))
			return 0;

		pr_info("[QNI-PROB] no input source, force set qni ready state.\n");
		set_qps(QPS_QNI_READY);
		goto release;
	} else if (get_qps() == QPS_IN_RUNTIME) {
		pr_info("[QNI-PROB] release run-time qni probation.\n");
		set_qps(QPS_RUNTIME_READY);
		goto release;
	} else if (get_qps() == QPS_IN_CV_MODE) {
		pr_info("[QNI-PROB] release qni probation by cv stage\n");
		set_qps(QPS_QNI_READY);
		goto release;
	}

	return 0;

release:
	vote(chip->not_ok_to_qnovo_votable, CHG_PROBATION_VOTER, false, 0);
	return 0;
}

#define FCC_MIN_CURR_STEP		50000   // pmi fcc resolution is 50mA.
#define FCC_COMP_ERROR_PCNT		12		// compensation error allowed.
static int compensate_fcc(int now, int fcc)
{
	int diff = fcc + now;
	const int step = FCC_MIN_CURR_STEP;

	if (!(now/1000)) {
		pr_info("[QNI-PROB] fcc compensation: drop! "
				"-> current isn't measured, count: %d, fcc: %d)\n",
			ext_qnovo.fcc_comp_count, fcc/1000);
		return 0;
	}

	if (ext_qnovo.fcc_comp_old_fcc != fcc ) {
		ext_qnovo.fcc_comp_count = 0;
		ext_qnovo.fcc_comp_ready_count = 0;
		pr_info("[QNI-PROB] fcc compensation:"
				" >> init << -> old fcc: %d, new fcc: %d\n",
			ext_qnovo.fcc_comp_old_fcc/1000, fcc/1000);
	}
	ext_qnovo.fcc_comp_ready_count++;

	if (ext_qnovo.fcc_comp_count == 0) {
		ext_qnovo.fcc_comp_old_fcc = fcc;
		ext_qnovo.fcc_comp_old_curr = now;
		ext_qnovo.fcc_comp_count_max = (fcc/10 + step/2)/step - 1;
	}
	else if (ext_qnovo.fcc_comp_count >= ext_qnovo.fcc_comp_count_max) {
		pr_info("[QNI-PROB] fcc compensation: drop! "
				"-> over comp count (fcc comp count: %d >= max: %d)\n",
			ext_qnovo.fcc_comp_count, ext_qnovo.fcc_comp_count_max);
		goto out;
	}
	else if (now > (ext_qnovo.fcc_comp_old_curr - (step/2))) {
		pr_info("[QNI-PROB] fcc compensation: drop! "
				"-> not rise (now: %d > old-%d: %d) -> fcc: %d\n",
			now/1000, (step/2)/1000,
			(ext_qnovo.fcc_comp_old_curr - (step/2))/1000, fcc/1000);
		goto out;
	}

	if (diff < (step*150/100)) {
		pr_info("[QNI-PROB] fcc compensation: drop! "
				"-> small diff (diff: %d < step*1.5: %d), fcc: %d, now: %d\n",
			diff/1000, (step*150/100)/1000, fcc/1000, now/1000);
		goto out;
	}

	if (diff > (fcc*FCC_COMP_ERROR_PCNT/100)) {
		pr_info("[QNI-PROB] fcc compensation: drop! "
				"-> big diff (diff: %d > fcc*%d: %d), fcc: %d, now: %d\n",
			diff/1000, FCC_COMP_ERROR_PCNT, (fcc/10)/1000, fcc/1000, now/1000);
		goto out;
	}

	// it means that fcc compensation starts after 15 sec when fcc is changed.
	//   -> polling interval of monitor fcc is 5sec.
	if (ext_qnovo.fcc_comp_ready_count > 3) {
		ext_qnovo.fcc_comp_count++;
		pr_info("[QNI-PROB] fcc compensation: Rise! [%d] -> fcc: %d, now: %d\n",
			ext_qnovo.fcc_comp_count, fcc/1000, now/1000);
	}
	else {
		pr_info("[QNI-PROB] fcc compensation: Hold! [%d, %d] -> fcc: %d, now: %d\n",
			ext_qnovo.fcc_comp_count, ext_qnovo.fcc_comp_ready_count, fcc/1000, now/1000);
	}

out:
	set_fcc_compensation(ext_qnovo.fcc_comp_count);
	return 0;
}

static int monitor_fcc(struct qnovo *chip)
{
	u8 phase = 0, pe_ctrl = 0;
	int rc = -1;
	int now_fcc = 0, now_curr = 0,  now_health = 0, now_dc_volt = 0;

	union power_supply_propval pval = {0, };
	char health_str[12][22] = {
		"unknown",                  "good",            "overheat",
		   "dead",           "overvoltage",      "unspec_failure",
		   "cold", "watchdog_timer_expire", "safety_timer_expire",
		   "warm",                  "cool",                 "hot"};

	if (!is_fg_available()
		|| !is_main_available()
		|| !is_veneer_available()
		|| !is_batt_available(chip)
		|| !is_wireless_available())
		return -EINVAL;

	/* get now current */
	rc = power_supply_get_property(ext_qnovo.bms_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &pval);
	now_curr = !rc ? pval.intval : 0;

	/* get fcc */
	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &pval);
	now_fcc = !rc ? pval.intval : 0;

	qnovo5_read(chip, QNOVO_PHASE, &phase, 1);
	qnovo5_read(chip, QNOVO_PE_CTRL, &pe_ctrl, 1);
	if (get_qps() == QPS_IN_RUNTIME) {
	}
	else {
		if (phase == 2 || phase == 3 ||
			(pe_ctrl & 0x07) == 0x02 || (pe_ctrl & 0x07) == 0x06) {
			pr_info("[QNI-PROB] skip run-time monitor_fcc "
				"due to qni diag stage.(phase=%d, fcc=%d, cur=%d)\n",
				phase, now_fcc/1000, now_curr/1000);
			return -EINVAL;
		}
	}

	if (ext_dt.enable_fcc_compensate)
		compensate_fcc(now_curr, now_fcc);

	/* get health */
	rc = power_supply_get_property(ext_qnovo.veneer_psy,
			POWER_SUPPLY_PROP_HEALTH, &pval);
	now_health = !rc ? pval.intval : 0;

	/* get dc voltage */
	if (ext_qnovo.is_dc) {
		rc = power_supply_get_property(ext_qnovo.wireless_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &pval);
		now_dc_volt = !rc ? pval.intval : 0;
	}

	/* check duplication */
	if (now_fcc == ext_qnovo.set_fcc
		&& now_health == ext_qnovo.health
		&& now_dc_volt == ext_qnovo.dc_volt)
		return -EINVAL;

	pr_info("[QNI-PROB] run-time monitor_fcc: "
			"curr=%d, now=%d, old=%d, locked=%d, dc_volt=%d, health: %s\n",
			now_curr/1000, now_fcc/1000, ext_qnovo.set_fcc/1000,
			ext_qnovo.locked_fcc/1000, now_dc_volt/1000, health_str[now_health]);

	ext_qnovo.set_fcc = now_fcc;
	ext_qnovo.health = now_health;
	ext_qnovo.dc_volt = now_dc_volt;

	return 0;
}

static int rt_qni_probation(struct qnovo *chip)
{
	int rc = -1;
	union power_supply_propval pval = {0};

	if (!is_main_available())
		return -EINVAL;
	if (!is_batt_available(chip))
		return -EINVAL;

	if ((ext_qnovo.set_fcc > ext_dt.qni_probation_enter_fcc) ||
			!(ext_qnovo.health == POWER_SUPPLY_HEALTH_GOOD)  ){
		if (!ext_qnovo.is_dc) {
			release_qni_probation(chip);
			return 0;
		}
		else if (ext_qnovo.dc_volt != DC_VOLTAGE_MV_BPP) {
			release_qni_probation(chip);
			return 0;
		}
	}

	/* current */
	pval.intval = min(ext_qnovo.set_fcc, ext_dt.qni_probation_max_fcc);
	if (pval.intval != ext_qnovo.set_fcc ) {
	/* this code will make a problem when parallel charging.
	 *
	 *	rc = power_supply_set_property(ext_qnovo.main_psy,
	 *			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	 *	if (rc < 0) {
	 *		pr_err("Couldn't set prop qnovo_fcc rc = %d\n", rc);
	 *		return -EINVAL;
	 *	}
	 */
	}
	ext_qnovo.locked_fcc = pval.intval;

	/* voltage */
	pval.intval = ext_dt.qni_probation_min_fv;
	pval.intval = min(chip->fv_uV_request, pval.intval);
	if (pval.intval > 0 && (ext_qnovo.locked_fv != pval.intval)) {
		rc = power_supply_set_property(ext_qnovo.me->batt_psy,
				POWER_SUPPLY_PROP_VOLTAGE_QNOVO, &pval);
		if (rc < 0) {
			pr_err("Couldn't set prop qnovo_fv rc = %d\n", rc);
			return -EINVAL;
		}
	}
	ext_qnovo.locked_fv = pval.intval;

	pr_info("[QNI-PROB] set run-time qni probation: "
			"[fcc] now:%d, max:%d, sel:%d "
			"[fv] min:%d, qnovo:%d, sel:%d "
			"[wlc] en:%d, volt:%d\n",
				ext_qnovo.set_fcc/1000,
				ext_dt.qni_probation_max_fcc/1000,
				ext_qnovo.locked_fcc/1000,
				ext_dt.qni_probation_min_fv/1000,
				chip->fv_uV_request/1000,
				ext_qnovo.locked_fv/1000,
				ext_qnovo.is_dc, ext_qnovo.dc_volt/1000
	);

	if (get_qps() == QPS_RUNTIME_READY &&
		(ext_qnovo.health == POWER_SUPPLY_HEALTH_GOOD)) {
		if (!ext_qnovo.is_dc) {
			set_qps(QPS_IN_RUNTIME);
			enter_qni_probation(ext_qnovo.me);
		}
		else if (ext_qnovo.dc_volt == DC_VOLTAGE_MV_BPP) {
			set_qps(QPS_IN_RUNTIME);
			enter_qni_probation(ext_qnovo.me);
		}
	}

	return 0;
}

static void taper_confirm_work(struct work_struct *work)
{
	int rc = 0;
	struct qnovo *chip = ext_qnovo.me;
	union power_supply_propval pval = {0};
	union power_supply_propval chg_type = {0};

	if (!is_batt_available(chip))
		return;

	rc = power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_CHARGE_TYPE, &chg_type);
	if (rc < 0) {
		pr_err("Couldn't get prop charge type rc = %d\n", rc);
		return;
	}

	if (chg_type.intval == POWER_SUPPLY_CHARGE_TYPE_TAPER) {
		if (get_qps() == QPS_IN_RUNTIME
			&& ext_qnovo.locked_fcc > ext_dt.qni_step_min_fcc) {

			pr_info("[QNI-PROB] cv stage: "
					"step down fcc by 100mA (%d->%dmA)\n",
					ext_qnovo.locked_fcc/1000,
					ext_qnovo.locked_fcc/1000-100);

			pval.intval = ext_qnovo.locked_fcc - 100000;
			rc = power_supply_set_property(chip->batt_psy,
					POWER_SUPPLY_PROP_CURRENT_QNOVO, &pval);
			if (rc < 0) {
				pr_err("Couldn't set prop qnovo_fcc rc = %d\n", rc);
				return;
			}
			ext_qnovo.locked_fcc = pval.intval;
		}
		else if (get_qps() > QPS_PRE_DEFINED_WLC) {
			set_qps(QPS_IN_CV_MODE);
			release_qni_probation(chip);
			qnovo_skew_set();
		}
	}
}

static void rt_monitor_fcc_work(struct work_struct *work)
{
	if (get_qps() == QPS_RUNTIME_READY || get_qps() == QPS_IN_RUNTIME) {
		if (!monitor_fcc(ext_qnovo.me))
			rt_qni_probation(ext_qnovo.me);

		schedule_delayed_work(
				&ext_qnovo.rt_monitor_fcc_work,
				msecs_to_jiffies(QNI_RT_MONITOR_TIME));
	}
	else {
		ext_qnovo.fcc_comp_count = 0;
		ext_qnovo.fcc_comp_old_fcc = 0;
		ext_qnovo.fcc_comp_old_curr = 0;
		ext_qnovo.fcc_comp_count_max = 0;
		ext_qnovo.fcc_comp_ready_count = 0;
		set_fcc_compensation(ext_qnovo.fcc_comp_count);
	}
}

static bool pred_usb_probation(struct qnovo *chip)
{
	union power_supply_propval pval = {0};
	int rc = 0;

	if(is_usb_available(chip)) {
		rc = power_supply_get_property(chip->usb_psy,
						POWER_SUPPLY_PROP_REAL_TYPE, &pval);
		if (rc < 0) {
			pr_err("Couldn't get usb prop rc=%d\n", rc);
			goto out;
		}

		if (pval.intval == POWER_SUPPLY_TYPE_USB ||
			pval.intval == POWER_SUPPLY_TYPE_UNKNOWN ||
			pval.intval == POWER_SUPPLY_TYPE_USB_FLOAT ) {
			pr_info("[QNI-PROB] enter pre-defined qni probation by "
					"SDP or FLOAT or Unknown.\n");
			return true;
		}
	}

out:
	return false;
}

static bool pred_dc_probation(struct qnovo *chip)
{
	union power_supply_propval pval = {0};
	int rc = 0;

	if (is_wireless_available()) {
		rc = power_supply_get_property(ext_qnovo.wireless_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &pval);
		if (rc < 0) {
			pr_err("Couldn't get dc prop rc=%d\n", rc);
			goto out;
		}

		if (pval.intval == DC_VOLTAGE_MV_BPP ) {
			pr_info("[QNI-PROB] enter pre-defined qni probation by "
					"wireless-bpp(5V).\n");
			return true;
		}
	}

out:
	return false;
}

/* qni probation main function */
static bool pred_qni_probation(struct qnovo *chip, bool is_dc)
{
	bool is_pred_probation = false;

	if (get_qps() < QPS_ENABLED)
		return false;

	/* When input is plugged, confirm qni probation */
	ext_qnovo.is_dc = is_dc;
	is_pred_probation = is_dc ?
			pred_dc_probation(chip) : pred_usb_probation(chip);

	if (is_pred_probation)
	{
		if (is_dc)
			set_qps(QPS_PRE_DEFINED_WLC);
		else
			set_qps(QPS_PRE_DEFINED_USB);

		enter_qni_probation(chip);
	}
	else {
		set_qps(QPS_RUNTIME_READY);
		schedule_delayed_work(&ext_qnovo.rt_monitor_fcc_work,
			msecs_to_jiffies(QNI_RT_MONITOR_TIME));
		pr_info("[QNI-PROB] ready for run-time qni probation.\n");
	}

	return true;
}

/**************************************************
 *   Override function of QNI original function   *
 **************************************************/
static int override_qnovo5_update_status(struct qnovo *chip)
{
	int rc = -1;
	union power_supply_propval chg_type = {0};
	union power_supply_propval status_raw = {0};

	if (!is_batt_available(chip))
		return -EINVAL;

	rc = power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_STATUS_RAW, &status_raw);
	if (rc < 0) {
		pr_err("Couldn't get prop status raw rc = %d\n", rc);
		return -EINVAL;
	}

	rc = power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_CHARGE_TYPE, &chg_type);
	if (rc < 0) {
		pr_err("Couldn't get prop charge type rc = %d\n", rc);
		return -EINVAL;
	}

	if (status_raw.intval != POWER_SUPPLY_STATUS_CHARGING) {
		cancel_delayed_work(&ext_qnovo.taper_confirm_work);
		if (get_qps() > QPS_ENABLED)
			release_qni_probation(chip);
	}
	else {
		if (get_qps() < QPS_PRE_DEFINED_WLC)
			goto out;

		if (chg_type.intval == POWER_SUPPLY_CHARGE_TYPE_TAPER) {
			schedule_delayed_work(&ext_qnovo.taper_confirm_work,
				msecs_to_jiffies(TAPER_CONFIRM_TIME));
		}
		else {
			cancel_delayed_work(&ext_qnovo.taper_confirm_work);

			if (status_raw.intval == ext_qnovo.status)
				goto out;

			if (get_qps() == QPS_RUNTIME_READY)
				schedule_delayed_work(&ext_qnovo.rt_monitor_fcc_work,
					msecs_to_jiffies(QNI_RT_MONITOR_TIME));
		}
	}

out:
	ext_qnovo.status = status_raw.intval;
	return 0;
}

static int override_qnovo_disable_cb(
	struct votable *votable, void *data, int disable, const char *client
){
	int rc = 0;
	struct qnovo *chip = data;

	vote(chip->pt_dis_votable, QNOVO_OVERALL_VOTER, disable, 0);

	/* When it is disabled by CHG_PROBATION_VOTER, keep qnovo voter
			because qni probation current is controlled by qnovo voter */
	if (disable &&
		(get_qps() == QPS_IN_RUNTIME))
		pr_info("[QNI-PROB] skip qnovo voter for run-time qni probation.\n");
	else
		rc = qnovo_batt_psy_update(chip, disable);

	return rc;
}

static void init_input_ready(void)
{
	ext_qnovo.pt_manager.cnt = 0;
	ext_qnovo.pt_manager.best = 0;
	ext_qnovo.pt_manager.worst = -10000000;
	ext_qnovo.pt_manager.diag_count = 0;
	schedule_delayed_work(
		&ext_qnovo.input_ready_work, msecs_to_jiffies(8000));
}

static void set_usb_debounce(struct qnovo *chip, bool en)
{
	if (!en) {
		/* removal or discharging*/
		chip->usb_charging = 0;
		cancel_delayed_work_sync(&chip->usb_debounce_work);
		cancel_delayed_work(&ext_qnovo.pt_done_watchdog_work);
		cancel_delayed_work(&ext_qnovo.pt_start_watchdog_work);
		vote(chip->awake_votable, USB_READY_VOTER, false, 0);
		vote(chip->chg_ready_votable, USB_READY_VOTER, false, 0);
	} else {
		/* insertion and charging */
		chip->usb_charging = 1;
		vote(chip->awake_votable, USB_READY_VOTER, true, 0);
		schedule_delayed_work(
			&chip->usb_debounce_work, msecs_to_jiffies(DEBOUNCE_MS));

		/* set lge usb */
		init_input_ready();
		reset_qnovo_config(chip);
	}
}

static void set_dc_debounce(struct qnovo *chip, bool en)
{
	if (!en) {
		/* removal or discharging*/
		chip->dc_charging = 0;
		cancel_delayed_work_sync(&chip->dc_debounce_work);
		cancel_delayed_work(&ext_qnovo.pt_done_watchdog_work);
		cancel_delayed_work(&ext_qnovo.pt_start_watchdog_work);
		vote(chip->awake_votable, DC_READY_VOTER, false, 0);
		vote(chip->chg_ready_votable, DC_READY_VOTER, false, 0);
	} else {
		/* insertion and charging */
		chip->dc_charging = 1;
		vote(chip->awake_votable, DC_READY_VOTER, true, 0);
		schedule_delayed_work(
			&chip->dc_debounce_work, msecs_to_jiffies(DEBOUNCE_MS));

		/* set lge dc */
		init_input_ready();
		reset_qnovo_config(chip);
	}
}

static void override_usb_debounce_work(struct work_struct *work)
{
	struct qnovo *chip = container_of(work,
				struct qnovo, usb_debounce_work.work);
	union power_supply_propval chg_type = {0};

	if (!is_batt_available(chip))
		return;

	power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CHARGE_TYPE, &chg_type);

	if ((chg_type.intval == POWER_SUPPLY_CHARGE_TYPE_TRICKLE
			|| chg_type.intval == POWER_SUPPLY_CHARGE_TYPE_FAST)
		&& (ext_qnovo.pt_manager.worst > -450000
			|| ext_qnovo.pt_manager.worst == -10000000))
		set_usb_debounce(chip, 1);
	else {
		vote(chip->chg_ready_votable, USB_READY_VOTER, true, 0);
		vote(chip->awake_votable, USB_READY_VOTER, false, 0);
		pred_qni_probation(chip, false);
	}
}

static void override_dc_debounce_work(struct work_struct *work)
{
	struct qnovo *chip = container_of(work,
				struct qnovo, dc_debounce_work.work);
	union power_supply_propval chg_type = {0};

	if (!is_batt_available(chip))
		return;

	power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CHARGE_TYPE, &chg_type);

	if ((chg_type.intval == POWER_SUPPLY_CHARGE_TYPE_TRICKLE
			|| chg_type.intval == POWER_SUPPLY_CHARGE_TYPE_FAST)
		&& (ext_qnovo.pt_manager.worst > -450000
			|| ext_qnovo.pt_manager.worst == -10000000))
		set_dc_debounce(chip, 1);
	else {
		vote(chip->chg_ready_votable, DC_READY_VOTER, true, 0);
		vote(chip->awake_votable, DC_READY_VOTER, false, 0);
		pred_qni_probation(chip, true);
	}
}

static void override_status_change_work(struct work_struct *work)
{
	struct qnovo *chip = container_of(work,
			struct qnovo, status_change_work);
	union power_supply_propval pval;
	bool usb_present = false, dc_present = false, charging = false;
	int msoc = 0, rc = -1;

	if (is_fg_available()) {
		rc = power_supply_get_property(ext_qnovo.bms_psy,
				POWER_SUPPLY_PROP_CAPACITY_RAW, &pval);
		msoc = (rc < 0) ? 0 : pval.intval;
	}

	if (is_batt_available(chip)) {
		rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_STATUS_RAW, &pval);
		// the reason to include POWER_SUPPLY_STATUS_FULL as charging is that
		// it prevents from releasing qnovo fv voter, when termination.
		// recharging is faster because qnovo fv voter is lower than batt profile fv voter.
		if ((msoc == 255 && pval.intval == POWER_SUPPLY_STATUS_FULL)
			|| pval.intval == POWER_SUPPLY_STATUS_CHARGING)
			charging = true;
		else
			charging = false;
	}

	if (is_usb_available(chip)) {
		rc = power_supply_get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_PRESENT, &pval);
		usb_present = (rc < 0) ? 0 : pval.intval;
	}

	if ((chip->usb_present != usb_present) || (chip->charging != charging)) {
		if (usb_present && charging) {
			if (!chip->usb_charging && msoc < 255) {
				set_usb_debounce(chip, 1);
				pr_info("qni:status_change_work: usb=1, charging=1...wait 15sec\n");
			}
		}
		else {
			if (chip->usb_charging) {
				set_usb_debounce(chip, 0);
				qnovo_fet_ctrl(chip);
				pr_info("qni:status_change_work: usb=%d, charging=%d...disabled\n",
					usb_present, charging);
			}
		}
	}

	if (is_dc_available(chip)) {
		rc = power_supply_get_property(chip->dc_psy,
				POWER_SUPPLY_PROP_PRESENT, &pval);
		dc_present = (rc < 0) ? 0 : pval.intval;
	}

	if (usb_present)
		dc_present = 0;

	if (!ext_dt.enable_for_dc)
		dc_present = 0;

	if ((chip->dc_present != dc_present) || (chip->charging != charging)) {
		if (dc_present && charging) {
			if (!chip->usb_charging && msoc < 255) {
				set_dc_debounce(chip, 1);
				pr_info("qni:status_change_work: dc=1, charging=1...wait 15sec\n");
			}
		}
		else {
			if (chip->dc_charging) {
				set_dc_debounce(chip, 0);
				qnovo_fet_ctrl(chip);
				pr_info("qni:status_change_work: dc=%d, charging=%d...disabled\n",
					dc_present, charging);
			}
		}
	}

	chip->usb_present = usb_present;
	chip->dc_present = dc_present;
	chip->charging = charging;

	override_qnovo5_update_status(chip);
}

static ssize_t qna_show(struct class *c, struct class_attribute *attr,
				char *ubuf)
{
	int i = 0, ret = -2;

	i = __find_attr_idx(&attr->attr);
	if (i < 0)
		return -EINVAL;

	if (ext_qnovo.is_qnovo_ready) {
		switch (i) {
			case QNA_METER:  ret = ext_qnovo.qna.meter; break;
			case QNA_SAFETY: ret = ext_qnovo.qna.safety; break;
			case QNA_VTWARN: ret = ext_qnovo.qna.vt_warn; break;
			case QNA_PCAP:   ret = ext_qnovo.qna.pcap; break;
			default: ret = -2; break;
		}
	}
	else
		ret = -2;

	return snprintf(ubuf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t qna_store(struct class *c, struct class_attribute *attr,
		       const char *ubuf, size_t count)
{
	struct qnovo *chip = container_of(c, struct qnovo, qnovo_class);
	union power_supply_propval pval = {0, };
	long val = -2;
	int i = 0, rc = -EINVAL;

	if (kstrtol(ubuf, 0, &val))
		return -EINVAL;

	i = __find_attr_idx(&attr->attr);
	if (i < 0)
		return -EINVAL;

	switch (i) {
		case QNA_METER:
			ext_qnovo.qna.meter = val;
			pr_info("[QNA Safety] health meter: %d\n", ext_qnovo.qna.meter);
		break;
		case QNA_SAFETY:
			ext_qnovo.qna.safety = val;
			pr_info("[QNA Safety] safety indicator: %d\n", ext_qnovo.qna.safety);

			if (ext_dt.enable_qna_unsafety_protection
				&& ext_qnovo.qna.safety == 1 ) {
				if (!is_batt_available(chip))
					return -EINVAL;

				pval.intval = ext_dt.qna_unsafety_fv;
				rc = power_supply_set_property(chip->batt_psy,
					POWER_SUPPLY_PROP_VOLTAGE_MAX, &pval);
				if (rc < 0) {
					pr_err("Couldn't set prop batt_profile_fv rc = %d\n", rc);
					return -EINVAL;
				}

				pval.intval = ext_dt.qna_unsafety_fcc;
				rc = power_supply_set_property(chip->batt_psy,
					POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
				if (rc < 0) {
					pr_err("Couldn't set prop batt_profile_fcc rc = %d\n", rc);
					return -EINVAL;
				}

				pr_info("[QNA Safety] unsafety: %d...set restriction: "
						"fv=%d, fcc=%d\n", ext_qnovo.qna.safety,
					ext_dt.qna_unsafety_fv/1000, ext_dt.qna_unsafety_fcc/1000);
			}
		break;
		case QNA_VTWARN:
			ext_qnovo.qna.vt_warn = val;
			pr_info("[QNA Safety] v-t warning: %d\n", ext_qnovo.qna.vt_warn);
		break;
		case QNA_PCAP:
			ext_qnovo.qna.pcap = val;
			pr_info("[QNA Safety] projected capacity: %d\n", ext_qnovo.qna.pcap);
		break;
	}

	return count;
}

static irqreturn_t override_handle_ptrain_done(int irq, void *data)
{
	struct qnovo *chip = data;

	if (!done_pt_manager(chip)) {
		pr_info("qni:handle_ptrain_done: diage stage is over-counted(>20). qnovo restart!!\n");
		if (chip->usb_charging) {
			set_usb_debounce(chip, 0);
			set_usb_debounce(chip, 1);
		}
		else if (chip->dc_charging) {
			set_dc_debounce(chip, 0);
			set_dc_debounce(chip, 1);
		}
		return IRQ_HANDLED;
	}
	override_qnovo5_update_status(chip);

	/*
	 * hw resets pt_en bit once ptrain_done triggers.
	 * vote on behalf of QNI to disable it such that
	 * once QNI enables it, the votable state changes
	 * and the callback that sets it is indeed invoked
	 */
	vote(chip->pt_dis_votable, QNI_PT_VOTER, true, 0);

	kobject_uevent(&chip->dev->kobj, KOBJ_CHANGE);
	return IRQ_HANDLED;
}

static void set_qnovo_config(struct qnovo *chip)
{
	if (!ext_qnovo.is_qnovo_ready) {
		ext_qnovo.is_qnovo_ready = true;

		if (get_qps() == QPS_ENABLED) {
			set_qps(QPS_QNI_READY);
			vote(chip->not_ok_to_qnovo_votable,
					CHG_PROBATION_VOTER, false, 0);
		}
		pr_info("[QNI-PROB] ready for qni daemon.\n");
	}
}

static void reset_qnovo_config(struct qnovo *chip)
{
	if (get_qps() > QPS_DISABLED) {
		ext_qnovo.dc_volt = 0;
		ext_qnovo.is_dc = false;

		ext_qnovo.set_fcc = 0;
		ext_qnovo.locked_fv = 0;
		ext_qnovo.locked_fcc = 0;
		ext_qnovo.fcc_comp_count = 0;
		ext_qnovo.fcc_comp_old_fcc = 0;
		ext_qnovo.fcc_comp_old_curr = 0;
		ext_qnovo.fcc_comp_count_max = 0;
		ext_qnovo.fcc_comp_ready_count = 0;

		qnovo_skew_reset();
		set_qps(QPS_QNI_READY);
		set_fcc_compensation(ext_qnovo.fcc_comp_count);
		vote(chip->not_ok_to_qnovo_votable, CHG_PROBATION_VOTER, false, 0);
		pr_info("[QNI-PROB] initialize qni probation.\n");
	}
}

static int extension_qnovo_parse_dt(struct qnovo *chip)
{
	struct device_node *node = chip->dev->of_node;

	ext_dt.enable_for_dc = of_property_read_bool(node,
			"lge,enable-for-dc");

	ext_dt.enable_qni_probation = of_property_read_bool(node,
			"lge,enable-qni-probation");
	ext_qnovo.qni_probation_status = ext_dt.enable_qni_probation ?
					QPS_ENABLED : QPS_DISABLED;

	ext_dt.qni_step_min_fcc = QNI_STEP_MIN_FCC;
	of_property_read_u32(node,
			"lge,qni-step-min-fcc",
			&ext_dt.qni_step_min_fcc);

	ext_dt.qni_probation_min_fv = CHG_PROBATION_MIN_FV;
	of_property_read_u32(node,
			"lge,qni-probation-min-fv",
			&ext_dt.qni_probation_min_fv);

	ext_dt.qni_probation_max_fcc = CHG_PROBATION_MAX_FCC;
	of_property_read_u32(node,
			"lge,qni-probation-max-fcc",
			&ext_dt.qni_probation_max_fcc);

	ext_dt.qni_probation_enter_fcc = CHG_PROBATION_ENTER_FCC;
	of_property_read_u32(node,
			"lge,qni-probation-enter-fcc",
			&ext_dt.qni_probation_enter_fcc);

	ext_dt.enable_fcc_compensate = of_property_read_bool(node,
			"lge,enable-compensate-qni-fcc");

	ext_dt.enable_qna_unsafety_protection = of_property_read_bool(node,
			"lge,enable-qna-unsafety-protection");

	ext_dt.qna_unsafety_fv = QNA_UNSAFETY_FV;
	of_property_read_u32(node,
			"lge,qna-unsafety-fv",
			&ext_dt.qna_unsafety_fv);

	ext_dt.qna_unsafety_fcc = QNA_UNSAFETY_FCC;
	of_property_read_u32(node,
			"lge,qna-unsafety-fcc",
			&ext_dt.qna_unsafety_fcc);

	ext_dt.qni_skew_comp_pct = QNI_SKEW_COMP_PCT;
	of_property_read_u32(node,
			"lge,qni-skew-comp-pct",
			&ext_dt.qni_skew_comp_pct);

	ext_dt.qni_pt_time_max = QNI_PT_TIME_MAX;
	of_property_read_u32(node,
			"lge,qni-pt-time-max-sec",
			&ext_dt.qni_pt_time_max);
	if (ext_dt.qni_pt_time_max <= 0 || ext_dt.qni_pt_time_max > 0x8000)
		ext_dt.qni_pt_time_max = QNI_PT_TIME_MAX;

	pr_info("[QNI-DTSI] enable_dc=%d, probation=%d, fcc_compensate=%d, qna_unsafety_protection=%d, "
			"step_min=%d, min_fv=%d, max_fcc=%d, enter_fcc=%d, "
			"unsafety_fv=%d, unsafety_fcc=%d, qni_skew_comp_pct=%d, pt_time_max=%d\n",
				ext_dt.enable_for_dc,
				ext_dt.enable_qni_probation,
				ext_dt.enable_fcc_compensate,
				ext_dt.enable_qna_unsafety_protection,
				ext_dt.qni_step_min_fcc/1000,
				ext_dt.qni_probation_min_fv/1000,
				ext_dt.qni_probation_max_fcc/1000,
				ext_dt.qni_probation_enter_fcc/1000,
				ext_dt.qna_unsafety_fv/1000,
				ext_dt.qna_unsafety_fcc/1000,
				ext_dt.qni_skew_comp_pct,
				ext_dt.qni_pt_time_max);

	return 0;
}

static int qnovo_udpate_pt_time_max(struct qnovo *chip)
{
	u8 buf[2] = {0, 0};
	u16 regval = 0;
	int rc = 0;
	int val;

	rc = qnovo5_read(chip,
		params[PTTIME_MAX].start_addr, buf, params[PTTIME_MAX].num_regs);
	if (rc < 0) {
		pr_err("qni: Couldn't read %s rc = %d\n", params[PTTIME_MAX].name, rc);
		return -EINVAL;
	}
	regval = buf[1] << 8 | buf[0];
	val = (int)regval;

	if (val == ext_dt.qni_pt_time_max) {
		pr_info("qni: qnovo pt time max(%d) is same with dtsi\n", val);
		return 0;
	}

	regval = (u16) ext_dt.qni_pt_time_max;
	buf[0] = regval & 0xFF;
	buf[1] = (regval >> 8) & 0xFF;

	rc = qnovo5_write(chip,
		params[PTTIME_MAX].start_addr, buf, params[PTTIME_MAX].num_regs);
	if (rc < 0) {
		pr_err("qni: Couldn't write %s rc = %d\n", params[PTTIME_MAX].name, rc);
		return -EINVAL;
	}

	pr_info("qni: qnovo pt time max is updated from %d to %d\n",
		val, ext_dt.qni_pt_time_max);
	return 0;
}

static int qnovo_init_psy(struct qnovo *chip)
{
	int rc = 0;

	if (!ext_qnovo.me)
		ext_qnovo.me = chip;

	ext_qnovo.qna.meter = -1;
	ext_qnovo.qna.safety = -1;
	ext_qnovo.qna.vt_warn = -1;
	ext_qnovo.qna.pcap = -1;
	ext_qnovo.status = -1;
	ext_qnovo.is_qnovo_ready = false;
	ext_qnovo.qni_probation_status = QPS_NOT_READY;

	extension_qnovo_parse_dt(chip);
	qnovo_udpate_pt_time_max(chip);
	INIT_DELAYED_WORK(&ext_qnovo.input_ready_work, input_ready_work);
	INIT_DELAYED_WORK(&ext_qnovo.taper_confirm_work, taper_confirm_work);
	INIT_DELAYED_WORK(&ext_qnovo.rt_monitor_fcc_work, rt_monitor_fcc_work);
	INIT_DELAYED_WORK(&ext_qnovo.pt_done_watchdog_work, pt_done_watchdog_work);
	INIT_DELAYED_WORK(&ext_qnovo.pt_start_watchdog_work, pt_start_watchdog_work);

	return rc;
}
