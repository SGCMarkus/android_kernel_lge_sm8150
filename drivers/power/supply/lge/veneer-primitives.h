#ifndef _VENEER_COMMON_H_
#define _VENEER_COMMON_H_

#include <linux/list.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include "../../../soc/qcom/lge/power/main/lge_prm.h"

#define OF_PROP_READ_S32(dnode, buf, prop, rc)						\
do {											\
	if (rc)										\
		break;									\
											\
	rc = of_property_read_s32(dnode, "lge," prop, &buf);				\
											\
	if (rc)										\
		pr_info("VENEER: Error reading " #prop " property rc = %d\n", rc);	\
	else										\
		pr_debug("VENEER: %s : %d\n", prop, buf);				\
} while (0)

#define OF_PROP_READ_STR(dnode, buf, prop, rc)						\
do {											\
	if (rc)										\
		break;									\
											\
	rc = of_property_read_string(dnode, "lge," prop, &buf);				\
											\
	if (rc)										\
		pr_info("VENEER: Error reading " #prop " property rc = %d\n", rc);	\
	else										\
		pr_debug("VENEER: %s : %s\n", prop, buf);				\
} while (0)

#define OF_PROP_READ_GPIO(dnode, buf, prop, rc)						\
do {											\
	if (rc) 									\
		break;									\
											\
	buf = of_get_named_gpio(dnode, "lge," prop, 0); 			\
											\
	if (buf < 0) { 									\
		pr_info("VENEER: Error reading " #prop " property rc = %d\n", buf);	\
		rc = buf;								\
	} else										\
		pr_debug("VENEER: %s : %d\n", prop, buf);				\
} while (0)

/* VOTE_VALUE_GRINDER is used to identify who (== one of IUSB/IBAT/IDC/VFLOAT) has voted.
 * So VOTE_TOTALLY_RELEASED and VOTE_TOTALLY_BLOCKED are designed to import the
 * GRINDER value
 */
#define VOTE_VALUE_GRINDER 	10
#define VOTE_TOTALLY_RELEASED	((INT_MAX/VOTE_VALUE_GRINDER)*VOTE_VALUE_GRINDER)
#define VOTE_TOTALLY_BLOCKED 	((      0/VOTE_VALUE_GRINDER)*VOTE_VALUE_GRINDER)
#define VOTER_NAME_LENGTH 30

enum voter_type {
	VOTER_TYPE_INVALID = 0,
	VOTER_TYPE_IUSB,
	VOTER_TYPE_IBAT,
	VOTER_TYPE_IDC,
	VOTER_TYPE_VFLOAT,
	VOTER_TYPE_HVDCP,
	/* add 'veneer_voter_type's here */
};

struct voter_entry {
/* static values being set with registering */
	char name[VOTER_NAME_LENGTH];
	enum voter_type		type;
	bool fakeui;	// Interfering UI when it votes 'suspend' (assigning '0' to limit)

/* runtime value with voting */
	int	limit;		// in mA
	bool effecting;	// in the veneer domain, not overall system.

/* private members controlled by voter lib. */
	int	id;
	struct list_head node;
};

static inline
union power_supply_propval vote_make(enum voter_type type, int limit)
{
	union power_supply_propval vote = {
		.intval = (limit / VOTE_VALUE_GRINDER) * VOTE_VALUE_GRINDER + type
	};
	return vote;
}

static inline
enum voter_type vote_type(const union power_supply_propval* vote)
{
	enum voter_type type = VOTER_TYPE_INVALID;

	switch (vote->intval % VOTE_VALUE_GRINDER) {
	case VOTER_TYPE_IUSB:
		type = VOTER_TYPE_IUSB;
		break;
	case VOTER_TYPE_IBAT:
		type = VOTER_TYPE_IBAT;
		break;
	case VOTER_TYPE_IDC:
		type = VOTER_TYPE_IDC;
		break;
	case VOTER_TYPE_VFLOAT:
		type = VOTER_TYPE_VFLOAT;
		break;
	case VOTER_TYPE_HVDCP:
		type = VOTER_TYPE_HVDCP;
		break;
	default:
		break;
	}

	return type;
}

static inline
const char* vote_title(enum voter_type type)
{
	switch (type) {
		case VOTER_TYPE_IUSB:	return "iusb";
		case VOTER_TYPE_IBAT:	return "ibat";
		case VOTER_TYPE_IDC:	return "idc";
		case VOTER_TYPE_VFLOAT:	return "vfloat";
		case VOTER_TYPE_HVDCP:	return "hvdcp";

		default: return NULL;
	}
}

static inline
enum voter_type vote_fromdt(const char* string)
{
	if (!strcmp(string, "iusb"))
		return VOTER_TYPE_IUSB;
	if (!strcmp(string, "ibat"))
		return VOTER_TYPE_IBAT;
	if (!strcmp(string, "idc"))
		return VOTER_TYPE_IDC;
	if (!strcmp(string, "vfloat"))
		return VOTER_TYPE_VFLOAT;
	if (!strcmp(string, "hvdcp"))
		return VOTER_TYPE_HVDCP;

	return VOTER_TYPE_INVALID;
}

static inline
int vote_limit(const union power_supply_propval* vote)
{
	return (vote->intval / VOTE_VALUE_GRINDER) * VOTE_VALUE_GRINDER;
}

enum charging_supplier { // Exclusive charging types
	CHARGING_SUPPLY_TYPE_UNKNOWN = 0,
	CHARGING_SUPPLY_TYPE_FLOAT,
	CHARGING_SUPPLY_TYPE_NONE,

	CHARGING_SUPPLY_DCP_DEFAULT,
	CHARGING_SUPPLY_DCP_10K,
	CHARGING_SUPPLY_DCP_22K,
	CHARGING_SUPPLY_DCP_QC2,
	CHARGING_SUPPLY_DCP_QC3,

	CHARGING_SUPPLY_USB_2P0,
	CHARGING_SUPPLY_USB_3PX,
	CHARGING_SUPPLY_USB_CDP,
	CHARGING_SUPPLY_USB_PD,

	CHARGING_SUPPLY_FACTORY_56K,
	CHARGING_SUPPLY_FACTORY_130K,
	CHARGING_SUPPLY_FACTORY_910K,

	CHARGING_SUPPLY_WIRELESS_5W,
	CHARGING_SUPPLY_WIRELESS_9W,
	CHARGING_SUPPLY_MAX,
};

enum charging_step {
	CHARGING_STEP_TRICKLE,
	CHARGING_STEP_CC,
	CHARGING_STEP_CV,
	CHARGING_STEP_TERMINATED,
	CHARGING_STEP_NOTCHARGING,
	CHARGING_STEP_DISCHARGING,
};

enum charging_verbosity {
	VERBOSE_CHARGER_NONE,
	VERBOSE_CHARGER_NORMAL,
	VERBOSE_CHARGER_INCOMPATIBLE,
	VERBOSE_CHARGER_SLOW,
};

enum charging_suspended {
	CHARGING_NOT_SUSPENDED,
	CHARGING_SUSPENDED_WITH_ONLINE,
	CHARGING_SUSPENDED_WITH_FAKE_OFFLINE,
};

enum charger_usbid {
	CHARGER_USBID_INVALID	= INT_MIN,	// No init
	CHARGER_USBID_UNKNOWN	=   -1000,	// Out of range
	CHARGER_USBID_56KOHM	=   56000,
	CHARGER_USBID_130KOHM	=  130000,
	CHARGER_USBID_910KOHM	=  910000,
	CHARGER_USBID_OPEN	= INT_MAX,
};

static inline
bool charger_fabcable(enum charging_supplier charger)
{
	switch(charger) {
	case CHARGING_SUPPLY_FACTORY_56K :
	case CHARGING_SUPPLY_FACTORY_130K :
	case CHARGING_SUPPLY_FACTORY_910K :
		return true;
	default :
		break;
	}

	return false;
}

static inline
bool charger_dcptype(enum charging_supplier charger)
{
	switch(charger) {
	case CHARGING_SUPPLY_DCP_DEFAULT :
	case CHARGING_SUPPLY_DCP_10K :
	case CHARGING_SUPPLY_DCP_22K :
		return true;
	default :
		break;
	}

	return false;
}

static inline
const char* charger_name(enum charging_supplier charger)
{
	switch(charger) {

	case CHARGING_SUPPLY_TYPE_UNKNOWN :	return "UNKNOWN";
	case CHARGING_SUPPLY_TYPE_FLOAT :	return "FLOAT";
	case CHARGING_SUPPLY_TYPE_NONE :	return "NONE";

	case CHARGING_SUPPLY_DCP_DEFAULT :	return "DCP";
	case CHARGING_SUPPLY_DCP_10K :	return "10K";
	case CHARGING_SUPPLY_DCP_22K :	return "22K";
	case CHARGING_SUPPLY_DCP_QC2 :	return "QC2";
	case CHARGING_SUPPLY_DCP_QC3 :	return "QC3";

	case CHARGING_SUPPLY_USB_2P0 :	return "USB2";
	case CHARGING_SUPPLY_USB_3PX :	return "USB3";
	case CHARGING_SUPPLY_USB_CDP :	return "CDP";
	case CHARGING_SUPPLY_USB_PD :	return "PD";

	case CHARGING_SUPPLY_FACTORY_56K :	return "56K";
	case CHARGING_SUPPLY_FACTORY_130K :	return "130K";
	case CHARGING_SUPPLY_FACTORY_910K :	return "910K";

	case CHARGING_SUPPLY_WIRELESS_5W :	return "W5W";
	case CHARGING_SUPPLY_WIRELESS_9W :	return "W9W";

	default :
		break;
	}

	return "ERROR";
}

static inline
bool veneer_extension_pspoverlap(
	enum power_supply_property* ori_arr, int ori_cnt,
	enum power_supply_property* ext_arr, int ext_cnt
){
	int i, j;

	for (i=0; i<ori_cnt; ++i) {
		for (j=0; j<ext_cnt; ++j) {
			if (ori_arr[i] == ext_arr[j]) {
				pr_err("VENEER: %d is overlapped in %s\n",
					ext_arr[j], __func__);

				return true; // Duplicated declaration in extended psy
			}
		}
	}

	return false; // means OK (not overlapped)
}

enum veneer_exception { // Can be accumulated to the 'int' storage
	// for battery			0x000?
	EXCEPTION_BATTERY_VBATLOW	= BIT(1),
	EXCEPTION_BATTERY_VBATOVER	= BIT(1),
	EXCEPTION_BATTERY_TEMPOVER	= BIT(2),
	EXCEPTION_BATTERY_MISSING	= BIT(3),
	// for wired charger		0x00?0
	EXCEPTION_WIRED_INCOMPATIBALE	= BIT(4),
	EXCEPTION_WIRED_VBUSOVER	= BIT(5),
	EXCEPTION_WIRED_IBUSOVER	= BIT(6),
	EXCEPTION_WIRED_VCCOVER		= BIT(7),
	// for wireless charger		0x0?00
	EXCEPTION_WLC_OVERCURRENT	= BIT(8),
	EXCEPTION_WLC_OVERVOLTAGE	= BIT(9),
	EXCEPTION_WLC_OVERTEMPERATURE	= BIT(10),
	// on during charging		0x?000
	EXCEPTION_CHARGING_TIMEOUT	= BIT(12),
	EXCEPTION_CHARGING_AICLFAIL	= BIT(13),
};

enum veneer_bootmode {
	BOOTMODE_ANDROID_NORMAL,
	BOOTMODE_ANDROID_56K,
	BOOTMODE_ANDROID_130K,
	BOOTMODE_ANDROID_910K,
	BOOTMODE_MINIOS_AAT,
	BOOTMODE_MINIOS_56K,
	BOOTMODE_MINIOS_130K,
	BOOTMODE_MINIOS_910K,
	BOOTMODE_LAF_NORMAL,
	BOOTMODE_LAF_910K,
	BOOTMODE_ETC_CHARGERLOGO,
	BOOTMODE_ETC_RECOVERY,
	BOOTMODE_ETC_UNKNOWN,
};

enum veneer_logmask {
	ERROR		= BIT(0),
	RETURN		= BIT(1),
	UPDATE		= BIT(2),
	ASSERT		= BIT(3),
	MONITOR		= BIT(4),
	EVALUATE	= BIT(5),
	INTERRUPT	= BIT(6),

	VERBOSE		= BIT(7),
};

struct voter_entry* veneer_voter_search(enum voter_type type, const char* name);
enum charging_suspended veneer_voter_suspended(enum voter_type type);
void veneer_voter_passover(enum voter_type type, int limit, bool enable);
bool veneer_voter_effecting(struct voter_entry* voter);
bool veneer_voter_enabled(struct voter_entry* voter);
void veneer_voter_set(struct voter_entry* voter, int limit);
void veneer_voter_rerun(struct voter_entry* voter);
void veneer_voter_release(struct voter_entry* voter);
void veneer_voter_unregister(struct voter_entry* entry);
bool veneer_voter_register(
	struct voter_entry* entry,
	const char* name, enum voter_type type, bool fakeui);
void veneer_voter_destroy(void);
bool veneer_voter_create(
	void (*back_unified_voter)(enum voter_type type, int limit));

bool charging_ceiling_vote(enum charging_supplier charger);
bool charging_ceiling_sdpmax(bool enable);
void charging_ceiling_destroy(void);
bool charging_ceiling_create(struct device_node* dnode);

int  charging_time_remains(int rawsoc);
void charging_time_clear(void);
void charging_time_destroy(void);
bool charging_time_update(enum charging_supplier charger, bool reloading);
bool charging_time_create(struct device_node* dnode, int fullraw,
	bool (*feed_charging_time)(int* power, int* rawsoc, int* bsm_ttf),
	void (*back_charging_time)(int power));

void protection_battemp_monitor(void);
void protection_battemp_destroy(void);
bool protection_battemp_create(struct device_node* dnode, int mincap,
	bool (*feed_protection_battemp)(bool* charging, int* temperature, int* mvoltage),
	void (*back_protection_battemp)(int health, int micharge, int mvfloat));

void protection_batvolt_refresh(bool charging);
void protection_batvolt_destroy(void);
bool protection_batvolt_create(struct device_node* dnode, int mincap,
	bool (*feed_protection_batvolt)(int* vnow_mv, int* icap_ma, int* chg_type));

void protection_showcase_update(void);
void protection_showcase_destroy(void);
bool protection_showcase_create(struct device_node* dnode,
	bool (*feed_protection_showcase)(bool* enabled, bool* charging, int* capacity),
	void (*back_protection_showcase)(const char* status));

void protection_usbio_trigger(void);
void protection_usbio_update(bool presence_usb);
void protection_usbio_destroy(void);
bool protection_usbio_create(struct device_node* dnode);

enum veneer_bootmode unified_bootmode_type(void);
enum charger_usbid unified_bootmode_usbid(void);
const char* unified_bootmode_operator(void);
const char* unified_bootmode_region(void);
const char* unified_bootmode_marker(void);
bool unified_bootmode_chargerlogo(void);
bool unified_bootmode_chgverbose(void);
bool unified_bootmode_fabproc(void);

int unified_bootmode_batteryid(void);

bool unified_nodes_show(const char* key, char* value);
bool unified_nodes_store(const char* key, const char* value, size_t size);
void unified_nodes_destroy(void);
bool unified_nodes_create(struct device_node* dnode);

void unified_sysfs_destroy(void);
bool unified_sysfs_create(struct device_node* dnode);

#endif
