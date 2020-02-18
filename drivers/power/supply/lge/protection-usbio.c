#define pr_fmt(fmt) "USBIO: %s: " fmt, __func__
#define pr_usbio(fmt, ...) pr_err(fmt, ##__VA_ARGS__)

#include "veneer-primitives.h"

static struct protection_usbio {
	struct voter_entry	hvdcp;
} usbio_me;

void protection_usbio_trigger(void) {
	pr_usbio("OV ON CC! HVDCP is disabled\n");
	veneer_voter_set(&usbio_me.hvdcp, VOTE_TOTALLY_BLOCKED);
}

void protection_usbio_update(bool usbin) {
	if (!usbin)
		veneer_voter_release(&usbio_me.hvdcp);
}

static bool usbio_create_parsedt(struct device_node* dnode) {
	int rc = 0;
	return !rc;
}

#define USBIO_VOTER "CCOV"
static bool usbio_create_voters(void) {
	return veneer_voter_register(&usbio_me.hvdcp, USBIO_VOTER, VOTER_TYPE_HVDCP, false);
}

void protection_usbio_destroy(void) {
	veneer_voter_unregister(&usbio_me.hvdcp);
	memset(&usbio_me, 0 ,sizeof(usbio_me));
}

bool protection_usbio_create(struct device_node* dnode) {
	if (!usbio_create_parsedt(dnode)) {
		pr_usbio("error on usbio_create_parsedt\n");
		goto destroy;
	}

	if (!usbio_create_voters()) {
		pr_usbio("error on usbio_create_voters\n");
		goto destroy;
	}

	pr_usbio("Complete to create\n");
	return true;

destroy:
	protection_usbio_destroy();
	return false;
}
