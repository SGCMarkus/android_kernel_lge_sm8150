#define pr_fmt(fmt) "BOOTMOD: %s: " fmt, __func__
#define pr_bootmod(fmt, ...) pr_err(fmt, ##__VA_ARGS__)

#define LENGTH_STRING_BUFFER	128

#include "veneer-primitives.h"

//////////////////////////////////////////////////////////////////////
// for detecting boot mode
//////////////////////////////////////////////////////////////////////

static char bootmode_cable [LENGTH_STRING_BUFFER] = { 0, };
static char bootmode_android [LENGTH_STRING_BUFFER] = { 0, };
static enum veneer_bootmode bootmode_type = BOOTMODE_ETC_UNKNOWN;

static void unified_bootmode_setup(void) {
	bootmode_type =
		!strcmp(bootmode_android, "qem_56k")	 ?
			(!strcmp(bootmode_cable, "LT_56K")
				? BOOTMODE_MINIOS_56K : BOOTMODE_MINIOS_AAT) :
		!strcmp(bootmode_android, "miniOS")	 ? BOOTMODE_MINIOS_AAT :
		!strcmp(bootmode_android, "qem_130k")	 ? BOOTMODE_MINIOS_130K :
		!strcmp(bootmode_android, "pif_910k")	 ? BOOTMODE_LAF_910K :
		!strcmp(bootmode_android, "qem_910k")	 ? BOOTMODE_LAF_910K :
		!strcmp(bootmode_android, "pif_56k")	 ? BOOTMODE_ANDROID_56K :
		!strcmp(bootmode_android, "pif_130k")	 ? BOOTMODE_ANDROID_130K :
		!strcmp(bootmode_android, "chargerlogo") ? BOOTMODE_ETC_CHARGERLOGO
			: BOOTMODE_ANDROID_NORMAL;

	pr_bootmod("input=(%s, %s)->%s\n", bootmode_cable, bootmode_android,
		unified_bootmode_marker());
}

const char* unified_bootmode_marker(void) {
	switch (bootmode_type) {
	case BOOTMODE_ANDROID_NORMAL:	return "A_N";
	case BOOTMODE_ANDROID_56K:	return "A_5";
	case BOOTMODE_ANDROID_130K:	return "A_1";
	case BOOTMODE_ANDROID_910K:	return "A_9";
	case BOOTMODE_MINIOS_AAT:	return "M_A";
	case BOOTMODE_MINIOS_56K:	return "M_5";
	case BOOTMODE_MINIOS_130K:	return "M_1";
	case BOOTMODE_MINIOS_910K:	return "M_9";
	case BOOTMODE_LAF_NORMAL:	return "L_N";
	case BOOTMODE_LAF_910K:		return "L_9";
	case BOOTMODE_ETC_CHARGERLOGO:	return "E_C";
	case BOOTMODE_ETC_RECOVERY:	return "E_R";
	default :
		break;
	}
	return "???"; // BOOTMODE_ETC_UNKNOWN
}

enum charger_usbid unified_bootmode_usbid(void) {
	switch (bootmode_type) {
	case BOOTMODE_ANDROID_56K:
	case BOOTMODE_MINIOS_56K:	return CHARGER_USBID_56KOHM;

	case BOOTMODE_ANDROID_130K:
	case BOOTMODE_MINIOS_130K:	return CHARGER_USBID_130KOHM;

	case BOOTMODE_ANDROID_910K:
	case BOOTMODE_MINIOS_910K:
	case BOOTMODE_LAF_910K:		return CHARGER_USBID_910KOHM;

	default:
		break;
	}

	return CHARGER_USBID_OPEN;
}

bool unified_bootmode_fabproc(void) {
	switch (bootmode_type) {
	case BOOTMODE_ANDROID_56K:
	case BOOTMODE_ANDROID_130K:
	case BOOTMODE_MINIOS_56K:
	case BOOTMODE_MINIOS_130K:
	case BOOTMODE_LAF_910K:
		return true;
	default:
		break;
	}

	return false;
}

bool unified_bootmode_usermode(void) {
	switch (bootmode_type) {
	case BOOTMODE_ANDROID_NORMAL:
	case BOOTMODE_ETC_CHARGERLOGO:
	case BOOTMODE_ETC_RECOVERY:
		return true;
	default:
		break;
	}

	return false;
}

void unified_bootmode_cable(char* arg) {
	strncpy(bootmode_cable, arg, LENGTH_STRING_BUFFER-1);
	unified_bootmode_setup();
}

void unified_bootmode_android(char* arg) {
	strncpy(bootmode_android, arg, LENGTH_STRING_BUFFER-1);
	unified_bootmode_setup();
}

bool unified_bootmode_chargerlogo(void) {
	return bootmode_type == BOOTMODE_ETC_CHARGERLOGO;
}

enum veneer_bootmode unified_bootmode_type(void) {
	return bootmode_type;
}


//////////////////////////////////////////////////////////////////////
// for detecting running S/W upon one-binary
//////////////////////////////////////////////////////////////////////

static enum {
	OPERATOR_KT,
	OPERATOR_ATT,
	OPERATOR_LGU,
	OPERATOR_SKT,
	OPERATOR_SPR,
	OPERATOR_TMO,
	OPERATOR_VZW,
	OPERATOR_OPEN,

	OPERATOR_MISC,
} bootmode_operator = OPERATOR_MISC;

static enum {
	REGION_CAN,
	REGION_JPN,
	REGION_KOR,
	REGION_USA,

	REGION_COM,
} bootmode_region = REGION_COM;

int battery_id;

const char* unified_bootmode_operator(void) {
	switch (bootmode_operator) {
	case OPERATOR_KT   :	return "KT";
	case OPERATOR_ATT  :	return "ATT";
	case OPERATOR_LGU  :	return "LGU";
	case OPERATOR_SKT  :	return "SKT";
	case OPERATOR_SPR  :	return "SPR";
	case OPERATOR_TMO  :	return "TMO";
	case OPERATOR_VZW  :	return "VZW";
	case OPERATOR_OPEN :	return "OPEN";

	case OPERATOR_MISC : 	return "MISC";
	default :
		break;
	}

	return "INVALID";
}

const char* unified_bootmode_region(void) {
	switch (bootmode_region) {
	case REGION_CAN :	return "CAN";
	case REGION_JPN :	return "JPN";
	case REGION_KOR :	return "KOR";
	case REGION_USA :	return "USA";

	case REGION_COM :	return "COM";
	default :
		break;
	}

	return "INVALID";
}

bool unified_bootmode_chgverbose(void) {
	return bootmode_operator == OPERATOR_SPR
		|| bootmode_operator == OPERATOR_ATT
		|| bootmode_operator == OPERATOR_VZW;
}

bool unified_bootmode_region_usa(void) {
	return bootmode_region == REGION_USA;
}

static int __init setup_bootmode_plmn(char* s) {
	if (!strcmp(s, "KT"))
		bootmode_operator = OPERATOR_KT;
	else if (!strcmp(s, "ATT"))
		bootmode_operator = OPERATOR_ATT;
	else if (!strcmp(s, "LGU"))
		bootmode_operator = OPERATOR_LGU;
	else if (!strcmp(s, "SKT"))
		bootmode_operator = OPERATOR_SKT;
	else if (!strcmp(s, "SPR"))
		bootmode_operator = OPERATOR_SPR;
	else if (!strcmp(s, "TMUS"))
		bootmode_operator = OPERATOR_TMO;
	else if (!strcmp(s, "VZW_POSTPAID") || !strcmp(s, "VZW_PREPAID"))
		bootmode_operator = OPERATOR_VZW;
	else if (!strcmp(s, "OPEN_KR") || !strcmp(s, "OPEN_US") || !strcmp(s, "OPEN_CA") || !strcmp(s, "GLOBAL"))
		bootmode_operator = OPERATOR_OPEN;
	else
		bootmode_operator = OPERATOR_MISC;

	if (!strcmp(s, "ATT") || !strcmp(s, "SPR") || !strcmp(s, "USC") || !strcmp(s, "TRF") || !strcmp(s, "TMUS") || !strcmp(s, "MPCS")
		|| !strcmp(s, "VZW_POSTPAID") || !strcmp(s, "VZW_PREPAID") || !strcmp(s, "CCA_LRA_ACG") || !strcmp(s, "AMAZON")
		|| !strcmp(s, "GOOGLEFI") || !strcmp(s, "COMCAST_US") || !strcmp(s, "CHARTER"))
		bootmode_region = REGION_USA;
	else if (!strcmp(s, "KT") || !strcmp(s, "LGU") || !strcmp(s, "SKT") || !strcmp(s, "OPEN_KR"))
		bootmode_region = REGION_KOR;
	else if (!strcmp(s, "OPEN_CA"))
		bootmode_region = REGION_CAN;
	else if (!strcmp(s, "DCM") /* Need to update */)
		bootmode_region = REGION_JPN;
	else
		bootmode_region = REGION_COM;

	pr_bootmod("PLMN: input %s -> (operator %s, region %s)\n",
		s, unified_bootmode_operator(), unified_bootmode_region());
	return 1;
}
__setup("androidboot.vendor.lge.plmn=", setup_bootmode_plmn);

int unified_bootmode_batteryid(void) {
	return battery_id;
}

static int __init setup_bootmode_batteryid(char* s) {
	int ret;

	ret = kstrtoint(s, 10, &battery_id);
	if(!ret)
		pr_info("battery id: %04d\n", battery_id);
	else
		pr_info("Fail to get battery id, ret=%d\n", ret);
	return 1;
}
__setup("androidboot.vendor.lge.battid=", setup_bootmode_batteryid);
