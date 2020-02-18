#include <linux/kobject.h>

#define UEVENT_SENDER_MAX 10

static struct filter {
	char	property[64];
	int	tolerance;

	int	namelen; // to compare with a non-null-terminated string
} filters [] = {
	{ "POWER_SUPPLY_INPUT_CURRENT_NOW",	100000,		30 },
	{ "POWER_SUPPLY_RESISTANCE_NOW",	11,		27 }, // bms       / RESISTANCE(mohm)
	{ "POWER_SUPPLY_CAPACITY_RAW",		255,		25 }, // bms       / msoc (0~255)
	{ "POWER_SUPPLY_VOLTAGE_OCV",		50000,		24 }, // bms       / OCV(uv)
	{ "POWER_SUPPLY_VOLTAGE_NOW",		50000,		24 }, // batt, usb / batt, input volt(uv)
	{ "POWER_SUPPLY_CURRENT_NOW",		200000,		24 }, // batt, bms / current(ua)
	{ "POWER_SUPPLY_CAPACITY",		1,		21 }, // batt      / SOC(%)
	{ "POWER_SUPPLY_TEMP",			10,		17 }, // batt      / temp(.x)
};

static struct score {
	int	total;
	int	reported;
	int	skipped;
	int	similars;
} evaluation;

static struct update {
	const char*	sender;
	char		data[UEVENT_BUFFER_SIZE];
	int		buflen;
} updated [UEVENT_SENDER_MAX];

enum compare {
	COMPARE_SAME = 0,
	COMPARE_SIMILAR = 1,
	COMPARE_DIFFERENT = 2,
};

static bool debug = true;
void veneer_uevent_evaluation(void);

static long each_property_diffval(const char* string1, const char* string2) {
	long long1, long2;

	if ((kstrtol(string1, 10, &long1) || kstrtol(string2, 10, &long2))) {
		pr_err("UEVENT, Check the caller for %s\n", __func__);
		return INT_MAX;
	}
	else
		return abs(long1 - long2);
}

static struct filter* each_property_filter(const char* property, int namelen) {
	struct filter* filter;
	int i;

	for (i = 0; i < ARRAY_SIZE(filters); ++i) {
		filter = &filters[i];
		if (filter->namelen == namelen && !strncmp(filter->property, property, namelen))
			return filter;
	}

	return NULL;
}

static enum compare each_property_compare(const char* property_name, int property_length,
	const char* property_value1, const char* property_value2) {
	struct filter* prop_filter = each_property_filter(property_name, property_length);
	int prop_toler;
	long prop_diff;

	if (prop_filter) {
		prop_toler = prop_filter->tolerance;
		prop_diff = each_property_diffval(property_value1, property_value2);

		pr_debug("UEVENT Check %s(%ld) with tolerance %d, %s : %s\n",
			prop_filter->property, prop_diff, prop_toler, property_value1, property_value2);

		if (prop_diff == 0)
			return COMPARE_SAME;
		else if (prop_diff < prop_toler)
			return COMPARE_SIMILAR;
		else
			return COMPARE_DIFFERENT;
	}
	else {
		/* Simple string comparing for the non-filtering properties */
		return strcmp(property_value1, property_value2)
			? COMPARE_DIFFERENT : COMPARE_SAME;
	}
}

static enum compare veneer_uevent_compare(const char* sender,
	const char *source_buffer, int source_count,
	const char *target_buffer, int target_count) {

	char similar_buffer [1024] = { 0, };
	enum compare ret = (source_count > 0 && target_count > 0) ?
		COMPARE_SAME : COMPARE_DIFFERENT;

	while (source_count > 0 && target_count > 0) {
		const char* source_eop = strchr(source_buffer, '=');	// eop := end of property
		const char* target_eop = strchr(target_buffer, '=');
		int length_of_property = source_eop-source_buffer;

		if (length_of_property == (target_eop-target_buffer)
			&& !strncmp(source_buffer, target_buffer, length_of_property)) {

			enum compare each = each_property_compare(source_buffer, length_of_property,
				source_eop + 1, target_eop + 1);
			ret = max(each, ret);

			// The meaning for 'max(each, ret)'
			//
			// ret(old) | SAME SAME SAME | SIMI SIMI SIMI | DIFF DIFF DIFF
			// each     | SAME SIMI DIFF | SAME SIMI DIFF | SAME SIMI DIFF
			// ===========================================================
			// ret(new) | SAME SIMI DIFF | SIMI SIMI DIFF | DIFF DIFF DIFF
			//
			// => But if debug is false, ret(old)==DIFF could not be met here.

			pr_debug("\t\tcomapring %s vs %s\n", source_buffer, target_buffer);
			if (each == COMPARE_SIMILAR) {
				char token [64];
				snprintf(token, 64, "%s vs %s", source_buffer, target_eop + 1);

				if (0 < strlen(similar_buffer) && strlen(similar_buffer) < 1022)
					strcat(similar_buffer, ", ");
				if (strlen(similar_buffer) + strlen(token) < 1024)
					strcat(similar_buffer, token);
			}
		}
		else {
			pr_debug("differnet property, %s, %s\n", source_buffer, target_buffer);
			ret = COMPARE_DIFFERENT;
		}

		if (debug || ret != COMPARE_DIFFERENT)
		{	/* next iteration */
			int source_length = strlen(source_buffer) + 1;
			int target_length = strlen(target_buffer) + 1;

			source_buffer += source_length;
			source_count -= source_length;
			target_buffer += target_length;
			target_count -= target_length;
		}
		else
			break;
	}

	evaluation.total += 1;
	evaluation.reported += (ret==COMPARE_DIFFERENT);
	evaluation.skipped += (ret!=COMPARE_DIFFERENT);
	evaluation.similars += (ret==COMPARE_SIMILAR);

	if (ret == COMPARE_SIMILAR) {
		veneer_uevent_evaluation();

		pr_debug("UEVENT similar %s\n", similar_buffer);
	}

	return ret;
}

/* veneer_uevent_duplicated
 *	filtering function exported to external
 * params : @ sender : name of psy who are sending a uevent
 *	    @ data : ptr for uevent buffer
 *	    @ buflen : len for uevent buffer
 * return : - true : should be filtered out.
 *	    - false : should be updated
 */
bool veneer_uevent_duplicated(const char* sender, char* data, int buflen) {
	struct update* updating = NULL;
	int i;

	pr_debug("UEVENT(%s) : Calling veneer_uevent_duplicated\n", sender);
	for(i=0; i<UEVENT_SENDER_MAX; ++i) {
		if (updated[i].sender == NULL)
			updated[i].sender = sender;

		if (updated[i].sender == sender) {
			updating = &updated[i];
			break;
		}
	}
	if (updating == NULL) {
		pr_err("Comparing snapshots are overflowed for %s\n", sender);
		return false;
	}

       /* If this duplication check does not consider the tolerance of properties,
	* below 'if' statement simply can be replaced to
	*       'if (updating->buflen != buflen || memcmp(updating->data, data, buflen))'
	*/
	if (veneer_uevent_compare(sender, updating->data, updating->buflen,
		data, buflen) == COMPARE_DIFFERENT) {
		memcpy(updating->data, data, buflen);
		updating->buflen = buflen;
		updating->data[buflen] = '\0';

		return false;
	}
	else
		return true;
}

void veneer_uevent_evaluation(void) {
	pr_info("UEVENT total(%d)/reported(%d)/skipped(%d) : similars(%d)\n",
		evaluation.total, evaluation.reported, evaluation.skipped,
		evaluation.similars);
}
