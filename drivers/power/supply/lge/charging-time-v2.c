#define pr_fmt(fmt) "CHGTIME: %s: " fmt, __func__
#define pr_chgtime(reason, fmt, ...)			\
do {							\
	if (pr_debugmask & (reason))			\
		pr_info(fmt, ##__VA_ARGS__);		\
	else						\
		pr_debug(fmt, ##__VA_ARGS__);		\
} while (0)

#include <linux/of.h>
#include <linux/slab.h>
#include "veneer-primitives.h"

#define LGTTF_TYPE_WIRED        0
#define LGTTF_TYPE_WIRELESS     1
#define LGTTF_TYPE_BCC          2
#define LGTTF_ID_CHARGERLOGO    0
#define LGTTF_ID_POWERON        1

#define LGTTF_SLOT_COUNT        256
#define SAMPLING_PERIOD_MS      1500

#define CURRENT_PROFILE_COL     2
#define TARGET_DATA_COL         6
#define CURRENT_PROFILE_SIZE    CURRENT_PROFILE_COL * 40
#define TARGET_DATA_SIZE        TARGET_DATA_COL * 10

#define BCC_VOTER               "Battery Care Charging(BCC)"

static int pr_debugmask = ERROR | UPDATE | EVALUATE;

struct lgttf_datum {
	int msoc;          /*    255 */
	int raw_soc;       /*    x10 */
	int ui_soc;        /*    x10 */
	int chg_curr;      /*     uA */
	int slot_time;     /*    sec */
	int ttf;           /* second */
};

struct lgttf_table {
	int id;
	int type;          /* 0-wired, 1-wireless, 2-bcc */
	int power;         /*    mW */
	int max_current;   /*    mA */
	int down_ratio;    /*    uA */
	int target;        /*   sec */
	int dd_decipct;    /* x1000 */
	struct lgttf_datum data[LGTTF_SLOT_COUNT];
};

struct lgttf_base_param {
	int current_size;  /* total current table size            */
	u32 dt_current[CURRENT_PROFILE_SIZE];
	int target_size;   /* total target table size             */
	u32 dt_target[TARGET_DATA_SIZE];
	int rsoc;		   /* rescale soc                         */
	int designed;      /* full capacity designed              */
	int fix_ui_soc;    /* slot time is fixed from this ui soc */
};

struct lgttf_rt_param {
	bool is_no_dd;     /* it is a special option for flash dd */
	bool is_bcc;
	bool is_wireless;
	bool is_chargerlogo;
	int power;
	int bcc_ttf;
	int selected;
};

static struct lgttf {
	struct lgttf_base_param base;
	struct lgttf_rt_param rt;
	struct lgttf_table *table;
	struct voter_entry ibat_voter;
	struct delayed_work	lgttf_work;

	bool (*get_resource)(int* power, int* rawsoc, int* bcc_ttf);
	void (*changed)(int power);
} lgttf = {0, };

extern int lge_get_dual_display_support(void);

#define MAX_ERROR_RATIO    5     /* decipct */
#define MAX_TRY_COMP       10
static int lgttf_set_each_data(struct lgttf_table *table, u32 *curr)
{
	int rc = 0, i = 0;
	int next = 0, t_soc = 0, t_curr = 0;
	int temp = 0;
	int total = 0, total_temp = 0;
	int exit_count = MAX_TRY_COMP, error = 0;

	/* make base table */
	for (i = 0; i < LGTTF_SLOT_COUNT; i++) {
		t_soc = curr[next * 2];
		t_curr = curr[next * 2 + 1];

		if (i > t_soc && i <= lgttf.base.rsoc) {
			next++;
			if (next >= lgttf.base.current_size)
				next = lgttf.base.current_size - 1;
			t_soc = curr[next * 2];
			t_curr = curr[next * 2 + 1];
		}

		table->data[i].msoc = i;
		table->data[i].raw_soc = (i * 100) / 255;
		table->data[i].ui_soc = (((i * 1000) / lgttf.base.rsoc) + 5) / 10;

		if (table->down_ratio > 0 && i > 0 &&
			table->data[i].raw_soc > 0 &&
			(t_curr * 1000) >= table->data[i-1].chg_curr)
			table->data[i].chg_curr =
				table->data[i-1].chg_curr - table->down_ratio;
		else
			table->data[i].chg_curr = min(table->max_current, t_curr) * 1000;

		temp = lgttf.base.designed * 60 * 60 / 255;
		table->data[i].slot_time = temp * 1000 / (table->data[i].chg_curr/1000);
		table->data[i].slot_time = (table->data[i].slot_time + 500 ) / 1000;

		/* fix cv slot_time */
		if (table->id > LGTTF_ID_POWERON &&
			table->data[i].ui_soc > lgttf.base.fix_ui_soc) {
			table->data[i].slot_time =
				max(table->data[i].slot_time,
					lgttf.table[LGTTF_ID_POWERON].data[i].slot_time);
		}

		if (table->data[i].ui_soc > 1 && i < lgttf.base.rsoc - 1)
			total += table->data[i].slot_time;

		pr_chgtime(MONITOR,
			"proto data: %d, %d, %d, %d, %d, %d, %d\n",
			table->data[i].msoc, table->data[i].raw_soc, table->data[i].ui_soc,
			table->data[i].chg_curr / 1000, table->data[i].slot_time, total, total / 60);
	}

	error = max(table->target, total) - min(table->target, total);
	pr_chgtime(UPDATE,
		"Before Compensation: target=%d(%d), total=%d(%d), diff=%d(%d)\n",
		table->target, table->target / 60,
		total, total / 60, error, error / 60);

	/* compensate ttf accuracy: allowance error is max 0.5%  */
	while ((--exit_count > 0) && (error > (table->target * MAX_ERROR_RATIO / 1000))) {
		total_temp = 0;
		for (i = 0; i < LGTTF_SLOT_COUNT; i++ ) {
			if (total > 0)
				table->data[i].slot_time =
					(((table->data[i].slot_time * 10) * table->target / total) + 5) / 10;
			else
				table->data[i].slot_time = 0;

			/* fix cv slot_time */
			if (table->id > LGTTF_ID_POWERON &&
				table->data[i].ui_soc > lgttf.base.fix_ui_soc) {
				table->data[i].slot_time =
					max(table->data[i].slot_time,
						lgttf.table[LGTTF_ID_POWERON].data[i].slot_time);
			}

			if (table->data[i].ui_soc > 1 && i < lgttf.base.rsoc - 1)
				total_temp += table->data[i].slot_time;
		}

		total = total_temp;
		error = max(table->target, total) - min(table->target, total);
	}

	/* make ttf data */
	for (i = (LGTTF_SLOT_COUNT-1); i >= 0 ; i--) {
		if (i > lgttf.base.rsoc - 2)
			table->data[i].ttf = 0;
		else
			table->data[i].ttf =
				table->data[i+1].ttf + table->data[i].slot_time;
	}

	pr_chgtime(UPDATE,
		"After Compensation(%d): "
		"target=%d(%d), total=%d(%d), diff=%d(%d), ttf[4]=%d(%d)\n",
			MAX_TRY_COMP - exit_count,
			table->target, table->target / 60, total, total / 60,
			error, error / 60, table->data[4].ttf, table->data[4].ttf / 60);

	/* print result for debug */
	for (i = 0; i < LGTTF_SLOT_COUNT; i++)
		pr_chgtime(MONITOR,
			"last data[%d]: %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
			MAX_TRY_COMP - exit_count,
			table->data[i].msoc, table->data[i].raw_soc, table->data[i].ui_soc,
			table->data[i].chg_curr / 1000, table->data[i].slot_time, table->data[i].ttf,
			table->data[i].ttf / 60, table->target, table->target / 60);

	return rc;
}

static int lgttf_set_each_table(
	struct lgttf_table *table, int id, u32 *target, u32 *curr)
{
	int rc = 0;
	int temp = 0;

	memset(table, 0, sizeof(struct lgttf_table));

	table->id = id;
	table->type = target[0];
	table->power = target[1];
	table->max_current = target[2];
	table->down_ratio = target[3];
	table->target = target[4] * 60;
	table->dd_decipct = target[5];

	if (table->down_ratio > 0 &&
		(table->down_ratio * 245 / 1000) > (table->max_current - 300)) {
		temp = (table->max_current - 300) * 1000 / 245;
		pr_chgtime(ERROR,
			"[ERROR id: %d] down_ratio(%d) is too big. "
			"I will rescale it to %d, but you have to check down_raio.\n",
			id, table->down_ratio, temp);
		table->down_ratio = temp;
	}

	pr_chgtime(UPDATE,
		"header: id=%d, type=%d, power=%d, max_current=%d, "
		"down_ratio=%d, target=%d(%d), dd_decipct=%d\n",
		table->id, table->type, table->power, table->max_current,
		table->down_ratio, table->target, table->target / 60, table->dd_decipct);

	lgttf_set_each_data(table, curr);

	return rc;
}

static int lgttf_upperbound(void)
{
	if (lgttf.rt.is_chargerlogo)
		return lgttf.table[LGTTF_ID_CHARGERLOGO].power;
	else
		return lgttf.table[LGTTF_ID_POWERON].power;
}

static int lgttf_find_table(int power)
{
	int i = 0, begin = LGTTF_ID_POWERON;
	int upper = -1, lower = -1;
	int type = LGTTF_TYPE_WIRED;
	u32 target[TARGET_DATA_COL] = {0, };
	int ratio = 0;
	int upper_time = 0, lower_time = 0, time_error = 0;

	if (lgttf.rt.is_bcc)
		type = LGTTF_TYPE_BCC;
	else if (lgttf.rt.is_wireless)
		type = LGTTF_TYPE_WIRELESS;

	if (lgttf.rt.is_chargerlogo)
		begin = 0;

	for (i = begin; i < lgttf.base.target_size; i++) {
		if (lgttf.table[i].type == type) {
			if (lgttf.table[i].power >= power) {
				if (upper > 0) {
					if (lgttf.table[upper].power > lgttf.table[i].power)
						upper = i;
				}
				else
					upper = i;
			}
			else {
				if (lower > 0) {
					if (lgttf.table[lower].power < lgttf.table[i].power)
						lower = i;
				}
				else
					lower = i;
			}
		}
	}

	if (upper < 0 && lower >= 0) {
		lgttf.rt.selected = lower;
	}
	else if (upper >= 0 && lower < 0) {
		lgttf.rt.selected = upper;
	}
	else if (upper >= 0 && lower >= 0) {
		lgttf.rt.selected = upper;
		if ( lgttf.table[upper].power != power && upper != lower) {

			target[0] = type;
			target[1] = power;
			target[2] = power / 5;
			target[3] = 0;
			target[5] = 1000;

			if (type == LGTTF_TYPE_WIRELESS)
			{
				ratio = (lgttf.table[upper].power - power) * 1000
					/ (lgttf.table[upper].power - lgttf.table[lower].power);
				target[4] = lgttf.table[upper].target + (
					(lgttf.table[lower].target - lgttf.table[upper].target)
					 * ratio / 1000 );
				target[4] /= 60;

				pr_chgtime(MONITOR,
					"upper=%d, lower=%d, ratio=%d, target=%d\n",
					lgttf.table[upper].target, lgttf.table[lower].target,
					ratio, target[4]);
			} else {
				upper_time = 60 * 60 * 1000 /
					(lgttf.table[upper].power * 1000 / 5 / lgttf.base.designed);
				lower_time = 60 * 60 * 1000 /
					(lgttf.table[lower].power * 1000 / 5 / lgttf.base.designed);
				time_error = (lgttf.table[upper].target - upper_time +
					lgttf.table[lower].target - lower_time) / 2;
				target[4] = 60 * 1000 / (power * 1000 / 5 / lgttf.base.designed)
					+ (time_error / 60);

				pr_chgtime(MONITOR,
					"type=%d, upper=%d, %d, %d, "
					"lower=%d, %d, %d, error=%d, target=%d\n",
					type, upper, lgttf.table[upper].power, upper_time,
					lower, lgttf.table[lower].power, lower_time,
					time_error, target[4]);
			}

			lgttf.rt.selected = lgttf.base.target_size;
				lgttf_set_each_table(&lgttf.table[lgttf.rt.selected],
					lgttf.rt.selected, target, lgttf.base.dt_current);
		}
	}
	else {
		lgttf.rt.selected = -1;
	}

	pr_chgtime(UPDATE, "found...power=%d, selected=%d, type=%d, current=%d\n",
		power, lgttf.rt.selected, lgttf.table[lgttf.rt.selected].type,
		lgttf.table[lgttf.rt.selected].max_current);

	return lgttf.rt.selected;
}

static void lgttf_work_func(struct work_struct *work)
{
	int power = 0, rawsoc = 0, bcc_ttf = 0;

	if (lgttf.get_resource(&power, &rawsoc, &bcc_ttf)) {
		if (bcc_ttf > 0) {
			lgttf.rt.is_bcc = true;
			power = 7500;
		}
		else {
			lgttf.rt.is_bcc = false;
			power = min(power, lgttf_upperbound());
		}
	}
	else {
		pr_chgtime(ERROR, "Error get_resource\n");
		return;
	}

	if (lgttf.rt.power != power || lgttf.rt.bcc_ttf != bcc_ttf) {
		pr_chgtime(UPDATE,
			"One more sampling (power: %dmW, bcc: %d)\n", power, bcc_ttf);

		lgttf.rt.bcc_ttf = bcc_ttf;
		if (power >= 2500)
			lgttf.rt.power = power;

		schedule_delayed_work(
			&lgttf.lgttf_work, msecs_to_jiffies(SAMPLING_PERIOD_MS));
	}
	else {
		if (lgttf_find_table(power) < 0) {
			pr_chgtime(ERROR, "Can't find the proper table.\n");
			return;
		}

		if (lgttf.rt.is_bcc)
			veneer_voter_set(&lgttf.ibat_voter,
				lgttf.table[lgttf.rt.selected].max_current);
		else
			veneer_voter_release(&lgttf.ibat_voter);

		lgttf.changed(lgttf.rt.power);
	}
}

int charging_time_remains(int rawsoc)
{
	int result_new = 0;
	static int result_old = 0;
	struct lgttf_table *table = NULL;

	if (!(lgttf.rt.selected > -1 && 0 <= rawsoc && rawsoc < LGTTF_SLOT_COUNT)) {
		return -1;
	}

	table = &lgttf.table[lgttf.rt.selected];
	result_new = table->data[rawsoc].ttf;
	if (lgttf.rt.is_no_dd)
		result_new = table->data[rawsoc].ttf * table->dd_decipct / 1000;

	if (result_new != result_old) {
		pr_chgtime(UPDATE,
			"report...soc=%d(%d, %d), ttf=%dsec(%dmin) "
			"(table=%d, type=%d, power=%d, dd=%d(%d))\n",
			rawsoc, table->data[rawsoc].ui_soc, table->data[rawsoc].raw_soc,
			result_new, result_new / 60, lgttf.rt.selected, table->type,
			lgttf.rt.power, !lgttf.rt.is_no_dd, table->dd_decipct);
	}

	result_old = result_new;
	return result_old;
}

bool charging_time_update(enum charging_supplier charger, bool reloading)
{
	static enum charging_supplier type = CHARGING_SUPPLY_TYPE_NONE;
	bool charging = false, ret = false;

	if (reloading)
		charger = type;

	if (type != charger || reloading) {
		charging = ( (charger != CHARGING_SUPPLY_TYPE_UNKNOWN) &&
                     (charger != CHARGING_SUPPLY_TYPE_NONE)        );

		if (charging) {
			pr_chgtime(UPDATE, "Charging started, Start sampling\n");
			cancel_delayed_work_sync(&lgttf.lgttf_work);
			schedule_delayed_work(
				&lgttf.lgttf_work, msecs_to_jiffies(SAMPLING_PERIOD_MS));

			lgttf.rt.is_wireless = false;
			if (charger == CHARGING_SUPPLY_WIRELESS_5W ||
				charger == CHARGING_SUPPLY_WIRELESS_9W) {
				lgttf.rt.is_wireless = true;
			}
		}
		else {
			pr_chgtime(UPDATE, "Charging stopped\n");
			charging_time_clear();
		}

		type = charger;
		ret = true;
	}

	return ret;
}

void lgttf_init_rt(void)
{
	lgttf.rt.is_no_dd = !lge_get_dual_display_support();
	lgttf.rt.is_bcc = false;
	lgttf.rt.is_wireless = false;
	lgttf.rt.is_chargerlogo = unified_bootmode_chargerlogo();
	lgttf.rt.power = -1;
	lgttf.rt.bcc_ttf = 0;
	lgttf.rt.selected = -1;
}

void lgttf_clear(void)
{
	cancel_delayed_work_sync(&lgttf.lgttf_work);
	lgttf_init_rt();
	veneer_voter_release(&lgttf.ibat_voter);
}

void charging_time_clear(void)
{
	lgttf_clear();
}

void lgttf_destroy(void) {
	veneer_voter_unregister(&lgttf.ibat_voter);
}

void charging_time_destroy(void)
{
	lgttf_destroy();
}

static int lgttf_parse_dt(struct device_node* dnode, int fullraw)
{
	int rc = 0, i = 0;

	lgttf.base.rsoc = fullraw;
	rc |= of_property_read_u32(dnode,
			"lge,full-capacity-design",
			&lgttf.base.designed);
	rc |= of_property_read_u32(dnode,
			"lge,fix-slot-time-ui-soc",
			&lgttf.base.fix_ui_soc);

	rc |= of_property_read_u32(dnode,
			"lge,charging-current-profile-raws",
			&lgttf.base.current_size);
	rc |= of_property_read_u32_array(dnode,
			"lge,charging-current-profile",
			lgttf.base.dt_current,
			lgttf.base.current_size * CURRENT_PROFILE_COL);

	rc |= of_property_read_u32(dnode,
			"lge,charging-target-data-raws",
			&lgttf.base.target_size);
	rc |= of_property_read_u32_array(dnode,
			"lge,charging-target-data",
			lgttf.base.dt_target,
			lgttf.base.target_size * TARGET_DATA_COL);

	lgttf.table = kzalloc(
		sizeof(struct lgttf_table) * (lgttf.base.target_size + 1), GFP_KERNEL);

	for (i = 0; i < lgttf.base.target_size; i++) {
		lgttf_set_each_table(lgttf.table + i, i,
			&lgttf.base.dt_target[TARGET_DATA_COL * i], lgttf.base.dt_current);
	}

	rc |= !veneer_voter_register(&lgttf.ibat_voter, BCC_VOTER, VOTER_TYPE_IBAT, false);

	return rc;
}

bool charging_time_create(
	struct device_node* dnode, int fullraw,
	bool (*feed_charging_time)(int* power, int* rawsoc, int* bcc_ttf),
	void (*back_charging_time)(int power))
{
	if (!dnode || !feed_charging_time || !back_charging_time) {
		pr_chgtime(ERROR, "null param error.\n");
		goto fail;
	}

	if (0 > fullraw || fullraw >= LGTTF_SLOT_COUNT) {
		pr_chgtime(ERROR, "fullraw(%d) is out of range\n", fullraw);
		goto fail;
	}

	if (lgttf_parse_dt(dnode, fullraw)) {
		pr_chgtime(ERROR, "'Error of parsing dt file.\n");
		goto fail;
	}

	lgttf.get_resource = feed_charging_time;
	lgttf.changed = back_charging_time;
	INIT_DELAYED_WORK(&lgttf.lgttf_work, lgttf_work_func);

	lgttf_init_rt();
	return true;

fail:
	pr_chgtime(ERROR, "Failed to create charging time\n");
	return false;
}
