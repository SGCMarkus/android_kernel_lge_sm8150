#define pr_fmt(fmt) "CEIL: %s: " fmt, __func__
#define pr_ceil(fmt, ...) pr_err(fmt, ##__VA_ARGS__)

#include <linux/of.h>
#include <linux/ctype.h>

#include "veneer-primitives.h"

#define VOTER		"CHARGER"
#define UNCEILING	VOTE_TOTALLY_RELEASED

struct ceiling_data {
	int iusb;
	int ibat;
	int idc;
};

static struct charging_ceiling_struct {
	// Voters
	struct voter_entry	voter_iusb;
	struct voter_entry	voter_ibat;
	struct voter_entry	voter_idc;
	// Index of ceiling_entry and the ceiling tables
	enum charging_supplier	charger_type;
	struct ceiling_data 	ceiling_value[CHARGING_SUPPLY_MAX];
	struct device_node*	charger_preset;
	bool			charger_sdpmax;
} ceiling_me = {
	.voter_iusb	= { .type = VOTER_TYPE_INVALID },
	.voter_ibat	= { .type = VOTER_TYPE_INVALID },
	.voter_idc	= { .type = VOTER_TYPE_INVALID },

	.charger_type	= CHARGING_SUPPLY_TYPE_UNKNOWN,
	.charger_preset	= NULL,
	.charger_sdpmax	= false,
};


bool charging_ceiling_vote(enum charging_supplier charger) {
	struct ceiling_data ceiling_value = ceiling_me.ceiling_value[charger];

	if (ceiling_me.charger_type != charger) {
		/* Voting for charger type */
		veneer_voter_set(&ceiling_me.voter_iusb, ceiling_value.iusb);
		veneer_voter_set(&ceiling_me.voter_ibat, ceiling_value.ibat);
		veneer_voter_set(&ceiling_me.voter_idc, ceiling_value.idc);

		/* Logging and updating driver */
		pr_ceil("Ceiling for [%s] : IUSB(%d)/IBAT(%d)/IDC(%d)\n", charger_name(charger),
			(ceiling_value.iusb == UNCEILING) ? -1 : ceiling_value.iusb,
			(ceiling_value.ibat == UNCEILING) ? -1 : ceiling_value.ibat,
			(ceiling_value.idc == UNCEILING) ? -1 : ceiling_value.idc);

		ceiling_me.charger_type = charger;
	}

	veneer_voter_passover(VOTER_TYPE_IUSB, 900, ceiling_me.charger_sdpmax &&
		ceiling_me.charger_type == CHARGING_SUPPLY_USB_2P0);

	return true;
}

bool charging_ceiling_sdpmax(bool enable) {
	if (ceiling_me.charger_sdpmax != enable) {
		ceiling_me.charger_sdpmax = enable;
		charging_ceiling_vote(ceiling_me.charger_type);
		return true;
	}

	return false;
}

static bool charging_ceiling_entry(struct device_node* dnode, enum charging_supplier charger, u32* out, size_t size) {
	const char*	name = charger_name(charger);
	char		property [32] = { 'l', 'g', 'e', ',', '\0', };
	char*		pointer = property + strlen(property);
	int i;

	for (i = 0; i < strlen(name); i++)
		*(pointer + i) = tolower(name[i]);

	if (of_property_read_u32_array(dnode, property, out, size)) {
		pr_ceil("Couldn't find %s\n", property);
		return false;
	}
	else
		return true;
}

static void charging_ceiling_dt(struct device_node* dnode) {
	u32 charger_type, entry[3];
	int i = 0;

	for (charger_type = CHARGING_SUPPLY_TYPE_UNKNOWN; charger_type < CHARGING_SUPPLY_MAX; charger_type++) {
		if (!charging_ceiling_entry(dnode, charger_type, entry, 3)) {
			pr_ceil("Failed to get entry for %s\n", charger_name(charger_type));
			return;
		}

		for ( i = 0; i < 3; i++) {
			if (entry[i] == (u32) 0)
				entry[i] = UNCEILING;
		}

		ceiling_me.ceiling_value[charger_type].iusb = entry[0];
		ceiling_me.ceiling_value[charger_type].ibat = entry[1];
		ceiling_me.ceiling_value[charger_type].idc = entry[2];
	}
	ceiling_me.charger_preset = dnode;
}

bool charging_ceiling_create(struct device_node* dnode) {
	bool success = (dnode != NULL)
		&& veneer_voter_register(&ceiling_me.voter_iusb, VOTER, VOTER_TYPE_IUSB, false)
		&& veneer_voter_register(&ceiling_me.voter_ibat, VOTER, VOTER_TYPE_IBAT, false)
		&& veneer_voter_register(&ceiling_me.voter_idc,  VOTER, VOTER_TYPE_IDC,  false);

	if (success)
		charging_ceiling_dt(dnode);
	else
		goto destroy;

	pr_ceil("Ceiling created\n");
	return true;

destroy:
	charging_ceiling_destroy();
	return false;
}

void charging_ceiling_destroy(void) {
	// Sweep away all voters : for abnormal repeated destory()
	veneer_voter_unregister(&ceiling_me.voter_iusb);
	veneer_voter_unregister(&ceiling_me.voter_ibat);
	veneer_voter_unregister(&ceiling_me.voter_idc);

	ceiling_me.voter_iusb.type = VOTER_TYPE_INVALID;
	ceiling_me.voter_ibat.type = VOTER_TYPE_INVALID;
	ceiling_me.voter_idc.type  = VOTER_TYPE_INVALID;

	// Clear misc data
	ceiling_me.charger_type   = CHARGING_SUPPLY_TYPE_UNKNOWN;
	ceiling_me.charger_preset = NULL;
	ceiling_me.charger_sdpmax = false;

	pr_ceil("Ceiling destroyed\n");
}
