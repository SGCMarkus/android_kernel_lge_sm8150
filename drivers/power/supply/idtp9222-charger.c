/*
 * IDTP9222 Wireless Power Receiver driver
 *
 * Copyright (C) 2016 LG Electronics, Inc
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

/***********************************************************
            [ Making 'Online' for IDTP9222 ]

 +====================================================
 | +------------------------------------|------------
 | |                                    |              IDT_GPIO_PWRDET
 |-+                                    |
 +======================================|=============
 |  +---+   +---+   +---+               |
 |  |   |   |   |   |   |<--- 5secs --->|              DC_PROP_PRESENT
 |--+   +---+   +---+   +---------------|------------
 +======================================|=============
 |   +-+     +-+     +-+                |
 |   | |     | |     | |                |              DC_PROP_ONLINE
 |---+ +-----+ +-----+ +----------------|------------
 +===|==================================|=============
 |   |<---          online          --->|<--- offline

***********************************************************/

#define pr_fmt(fmt) "IDTP9222: %s: " fmt, __func__

#define pr_idt(reason, fmt, ...)				\
do {								\
	if (idtp9222_debug & (reason))				\
		pr_err(fmt, ##__VA_ARGS__);			\
	else							\
		pr_debug(fmt, ##__VA_ARGS__);			\
} while (0)

#define pr_assert(exp)						\
do {								\
	if ((idtp9222_debug & IDT_ASSERT) && !(exp)) {		\
		pr_idt(IDT_ASSERT, "Assertion failed\n");	\
	}							\
} while (0)

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/pmic-voter.h>
#include <linux/power_supply.h>
#include "qcom/smb5-lib.h"

#ifdef CONFIG_LGE_PM_VENEER_PSY
#include "lge/veneer-primitives.h"
#endif

// Constants
#define IDTP9222_NAME_COMPATIBLE	"idt,p9222-charger"
#define IDTP9222_NAME_DRIVER		"idtp9222-charger"
#define IDTP9222_NAME_PSY		"wireless"

#define IDTP9222_VOLTAGE_MV_BPP		5000
#define IDTP9222_VOLTAGE_MV_EPP		9000
#define IDTP9222_CURRENT_MA_MAX		900
#define IDTP9222_CURRENT_MA_MIN		500

// Register addresses
#define REG_ADDR_FIRMWARE	0x06
#define REG_ADDR_INT		0x36
#define REG_ADDR_VOUT		0x3C
#define REG_ADDR_CHGSTAT	0x3E
#define REG_ADDR_EPT		0x3F
#define REG_ADDR_VRECT_L	0x40
#define REG_ADDR_VRECT_M	0x41
#define REG_ADDR_VADC_L		0x42
#define REG_ADDR_VADC_M		0x43
#define REG_ADDR_IADC_L		0x44
#define REG_ADDR_IADC_M		0x45
#define REG_ADDR_OPMODE		0x4C
#define REG_ADDR_COMMAND	0x4E
#define REG_ADDR_FODSTART	0x70
#define REG_ADDR_FODEND		0x7B
#define REG_ADDR_TXID		0xC2
#define REG_ADDR_GLOBALGAIN	0xCF

// For VOUT register
#define VOUT_V5P5		0x37
#define VOUT_V9P0		0x5A
// For EPT register
#define EPT_BY_EOC		1
#define EPT_BY_OVERTEMP		3
#define EPT_BY_NORESPONSE	8
// For Operation mode register
#define OPMODE_MIDPOWER		BIT(4)
#define OPMODE_WPC		BIT(5)
#define OPMODE_PMA		BIT(6)
// For command register
#define SEND_CHGSTAT		BIT(4)
#define SEND_EPT		BIT(3)

// For votables
#define DISABLE_BY_USB		"DISABLE_BY_USB"
#define DISABLE_BY_WA		"DISABLE_BY_WA"
#define DISABLE_BY_EOC		"DISABLE_BY_EOC"
#define WLC_CS100_VOTE		"WLC_CS100_VOTE"
#define WLC_THERMAL_VOTER	"WLC_THERMAL_VOTER"
#define WLC_WA_VOTER		"WLC_WA_VOTER"
#define DEFAULT_VOTER		"DEFAULT_VOTER"

enum idtp9222_print {
	IDT_ASSERT	= BIT(0),
	IDT_ERROR	= BIT(1),
	IDT_INTERRUPT	= BIT(2),
	IDT_MONITOR	= BIT(3),
	IDT_REGISTER	= BIT(4),
	IDT_RETURN	= BIT(5),
	IDT_UPDATE	= BIT(6),

	IDT_VERBOSE	= BIT(7),
};

enum idtp9222_opmode {
	UNKNOWN = 0, WPC, PMA,
};

struct idtp9222_struct {
	/* idtp9222 descripters */
	struct power_supply*	wlc_psy;
	struct i2c_client* 	wlc_client;
	struct votable*		wlc_disable;
	struct votable*		wlc_voltage;
	struct votable*		wlc_suspend;
	struct votable*		dc_icl_votable;
	struct device* 		wlc_device;
	struct wakeup_source	wlc_wakelock;
	/* idtp9222 delayed_works */
	struct delayed_work	timer_maxinput;
	struct delayed_work	timer_setoff;
	struct delayed_work	timer_overheat;
	struct delayed_work	polling_temp;
	struct delayed_work	polling_volt;
	struct delayed_work 	logger_gpios;
#ifdef CONFIG_LGE_COVER_DISPLAY
	struct delayed_work	timer_hallic;
#endif
	/* shadow status */
	bool			status_onpad;		// opposite to gpio_detached
	bool			status_dcin;		// presence of DCIN on PMIC side
	bool			status_full;		// it means EoC, not 100%
	bool			status_overheat;
	bool			status_done;
	int 			status_capacity;
	int			status_capacityraw;
	int			status_temperature;
	/* onpad flags */
	enum idtp9222_opmode	opmode_type;		// WPC or PMA
	bool			opmode_midpower;	// 9W or 5W
	/* for controling GPIOs */
	int			gpio_idtfault;		// MSM GPIO #52, DIR_IN(interrupt)
	int			gpio_detached;		// MSM GPIO #37, DIR_IN(interrupt)
	int			gpio_disabled;		// PMI GPIO #09, DIR_OUT(command)
	/* FOD parameters */
	char			fod_bpp [REG_ADDR_FODEND-REG_ADDR_FODSTART+1];
	char			fod_epp [REG_ADDR_FODEND-REG_ADDR_FODSTART+1];
	/* configuration from DT */
	int			configure_bppcurr;
	int			configure_eppcurr;
	int			configure_bppvolt;
	int			configure_eppvolt;
	int			configure_overheat;	// shutdown threshold for overheat
	int			configure_recharge;	// recharge soc from fg
	bool			configure_sysfs;	// making sysfs or not (for debug)
};

static int idtp9222_regs [] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x1C, 0x1D, 0x1E, 0x1F,
	0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E,
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5C, 0x5D, 0x5E, 0x5F,
	0x60, 0x61,
};

static inline const char* idtp9222_modename(enum idtp9222_opmode modetype) {
	switch (modetype) {
	case WPC :
		return "WPC";
	case PMA :
		return "PMA";

	case UNKNOWN :
	default :
		return "UNKNOWN";
	}
}

static int idtp9222_debug = IDT_ASSERT | IDT_ERROR | IDT_INTERRUPT | IDT_MONITOR | IDT_REGISTER | IDT_RETURN | IDT_UPDATE;

static bool idtp9222_is_onpad(struct idtp9222_struct* idtp9222);
static bool idtp9222_is_enabled(struct idtp9222_struct* idtp9222);
static bool idtp9222_is_full(struct idtp9222_struct* idtp9222);

static ssize_t sysfs_i2c_show(struct device* dev,
	struct device_attribute* attr, char* buffer);
static ssize_t sysfs_i2c_store(struct device* dev,
	struct device_attribute* attr, const char* buf, size_t size);

static DEVICE_ATTR(register, S_IWUSR|S_IRUGO, sysfs_i2c_show, sysfs_i2c_store);

static struct attribute* idtp9222_sysfs_attrs [] = {
	&dev_attr_register.attr,
	NULL
};

static const struct attribute_group idtp9222_sysfs_files = {
	.attrs  = idtp9222_sysfs_attrs,
};

#define	I2C_RETRY_COUNT	5
#define	I2C_RETRY_DELAY	10

static inline bool idtp9222_reg_read(struct i2c_client* client, u16 reg, u8* value) {
	u8 address [] = {
		reg >> 8,
		reg & 0xff
	};

	struct i2c_msg message [] = {
		{	.addr	= client->addr,
			.flags	= 0,
			.buf	= address,
			.len	= 2
		},
		{	.addr	= client->addr,
			.flags	= I2C_M_RD,
			.buf	= value,
			.len	= 1
		}
	};

	bool	success = false;
	int	retry = I2C_RETRY_COUNT;

	while (--retry && !success) {
		if (i2c_transfer(client->adapter, message, 2) == 2) {
			pr_idt(IDT_VERBOSE, "I2C read : %d tried\n", I2C_RETRY_COUNT-retry);
			success = true;
		}
		else {
			mdelay(I2C_RETRY_DELAY);
		}
	}

	if(!success)
		pr_idt(IDT_ERROR, "I2C failed to read 0x%02x\n", reg);

	return success;
}

static inline bool idtp9222_reg_write(struct i2c_client* client, u16 reg, u8 val) {
	u8 address [] = {
		reg >> 8,
		reg & 0xff,
		val
	};

	struct i2c_msg message = {
		.addr	= client->addr,
		.flags	= 0,
		.buf	= address,
		.len	= 3
	};

	bool	success = false;
	int	retry = I2C_RETRY_COUNT;

	while (--retry && !success) {
		if (i2c_transfer(client->adapter, &message, 1) == 1) {
			pr_idt(IDT_VERBOSE, "I2C write : %d tried\n", I2C_RETRY_COUNT-retry);
			success = true;
		}
		else {
			mdelay(I2C_RETRY_DELAY);
		}
	}

	if(!success)
		pr_idt(IDT_ERROR, "I2C failed to write 0x%02x:0x%02x\n", reg, val);

	return success;
}

static void idtp9222_reg_dump(struct idtp9222_struct* idtp9222) {
	if (idtp9222_is_onpad(idtp9222)) {
		u8 val;
		int i;

		for (i=0; i<sizeof(idtp9222_regs)/sizeof(idtp9222_regs[0]); i++) {
			val = -1;
			idtp9222_reg_read(idtp9222->wlc_client, idtp9222_regs[i], &val);
			pr_idt(IDT_REGISTER, "%02x : %02x\n", idtp9222_regs[i], val);
		}
	}
	else
		pr_idt(IDT_VERBOSE, "idtp9222 is off\n");
}

static unsigned int sysfs_i2c_register = -1;
static ssize_t sysfs_i2c_store(struct device* dev,
	struct device_attribute* attr, const char* buf, size_t size) {

	struct idtp9222_struct* idtp9222 = dev->platform_data;

	u8 value = -1;
	if (sscanf(buf, "0x%04x-0x%02x", &sysfs_i2c_register, (unsigned int*)&value) == 2) {
		if (idtp9222_reg_write(idtp9222->wlc_client, sysfs_i2c_register, value))
			pr_idt(IDT_ERROR, "I2C write fail for 0x%04x\n", sysfs_i2c_register);
	}
	else if (sscanf(buf, "0x%04x", &sysfs_i2c_register) == 1) {
		pr_idt(IDT_ERROR, "I2C address 0x%04x is stored\n", sysfs_i2c_register);
	}
	else {
		pr_idt(IDT_ERROR, "Usage : echo 0x%%04x-0x%%02x\n > register");
	}

	return size;
}
static ssize_t sysfs_i2c_show(struct device* dev,
	struct device_attribute* attr, char* buffer) {

	struct idtp9222_struct* idtp9222 = dev->platform_data;

	u8 value = -1;
	if (sysfs_i2c_register != -1) {
		if (idtp9222_reg_read(idtp9222->wlc_client, sysfs_i2c_register, &value))
			return snprintf(buffer, PAGE_SIZE, "0x%03x", value);
		else
			return snprintf(buffer, PAGE_SIZE, "I2C read fail for 0x%04x\n", sysfs_i2c_register);
	}
	else
		return snprintf(buffer, PAGE_SIZE, "Address should be set befor reading\n");
}

static bool idtp9222_wakelock_acquire(struct wakeup_source* wakelock) {
	if (!wakelock->active) {
		pr_idt(IDT_INTERRUPT, "Success!\n");
		__pm_stay_awake(wakelock);

		return true;
	}
	return false;
}

static bool idtp9222_wakelock_release(struct wakeup_source* wakelock) {
	if (wakelock->active) {
		pr_idt(IDT_INTERRUPT, "Success!\n");
		__pm_relax(wakelock);

		return true;
	}
	return false;
}

static int idtp9222_get_current(struct idtp9222_struct* idtp9222, const char *client_str) {
	if (!idtp9222->dc_icl_votable)
		idtp9222->dc_icl_votable = find_votable("DC_ICL");

	if (!idtp9222->dc_icl_votable)
		return -EINVAL;

	if (client_str == NULL) {
		return get_effective_result(idtp9222->dc_icl_votable);
	}
	else {
		return get_client_vote(idtp9222->dc_icl_votable, client_str);
	}
}

static bool idtp9222_set_current(struct idtp9222_struct* idtp9222,
	const char *client_str, bool enabled, int val) {
	if (!idtp9222->dc_icl_votable)
		idtp9222->dc_icl_votable = find_votable("DC_ICL");

	if (!idtp9222->dc_icl_votable)
		return false;

	vote(idtp9222->dc_icl_votable, client_str, enabled, val);

	// Returning for further purpose
	return true;
}

static bool idtp9222_set_fod(struct idtp9222_struct* idtp9222) {
	const char* parameters = idtp9222->opmode_midpower ? idtp9222->fod_epp : idtp9222->fod_bpp;
	u16 i;
	for (i=0; i<REG_ADDR_FODEND-REG_ADDR_FODSTART+1; ++i)
		idtp9222_reg_write(idtp9222->wlc_client,
			REG_ADDR_FODSTART + i, parameters[i]);

	// Returning for further purpose
	return true;
}

static bool idtp9222_set_full(struct idtp9222_struct* idtp9222, bool full) {
	if (idtp9222_is_full(idtp9222) == full) {
		pr_idt(IDT_VERBOSE, "status full is already set to %d\n", full);
		return false;
	}

	if (!idtp9222_is_onpad(idtp9222)) {
		pr_idt(IDT_VERBOSE, "idtp9222 is off now\n");
		return false;
	}

	if (full) {
		switch (idtp9222->opmode_type) {
		case WPC :
			/* CS100 is special signal for some TX pads */
			idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_CHGSTAT, 100);
			idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_COMMAND, SEND_CHGSTAT);
			pr_idt(IDT_UPDATE, "Sending CS100 to WPC pads for EoC\n");
			break;
		case PMA :
			idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_EPT, EPT_BY_EOC);
			idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_COMMAND, SEND_EPT);
			pr_idt(IDT_UPDATE, "Sending EPT to PMA pads for EoC\n");
			break;
		default :
			pr_idt(IDT_ERROR, "Is IDTP onpad really?\n");
			break;
		}
	}
	else
		; // Nothing to do for !full

	idtp9222->status_full = full;
	return true;
}

static bool idtp9222_set_capacity(struct idtp9222_struct* idtp9222) {
	bool is_full = (idtp9222->status_capacity == 100);

	if (idtp9222->status_capacity < 100) {
		if (idtp9222->opmode_type == WPC) {
			/* CS100 is special signal for some TX pads */
			idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_CHGSTAT,
				idtp9222->status_capacity);
			idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_COMMAND,
				SEND_CHGSTAT);
		}
	}

	idtp9222_set_full(idtp9222, is_full);

	// Returning for further purpose
	return true;
}

#ifdef CONFIG_LGE_COVER_DISPLAY
extern int global_luke_status;
static bool idtp9222_set_hallic(struct idtp9222_struct* idtp9222) {
	u8 value = -1;

	if (!gpio_get_value(idtp9222->gpio_detached) && global_luke_status) {
		idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_GLOBALGAIN, 0xFF);
		idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_GLOBALGAIN, &value);
		pr_idt(IDT_UPDATE, "Flash DD cover detected! 0x%02x\n", value);
		schedule_delayed_work(&idtp9222->timer_hallic,
			round_jiffies_relative(msecs_to_jiffies(5000)));
	}

	// Returning for further purpose
	return true;
}
#endif

static bool idtp9222_set_maxinput(/* @Nullable */ struct idtp9222_struct* idtp9222,
	bool enable) {
#ifdef CONFIG_LGE_PM_VENEER_PSY
	/* At this time, Releasing IBAT/IDC via VENEER system */
	veneer_voter_passover(VOTER_TYPE_IBAT, VOTE_TOTALLY_RELEASED, enable);
	veneer_voter_passover(VOTER_TYPE_IDC, VOTE_TOTALLY_RELEASED, enable);
#endif
	return true;
}

static bool idtp9222_set_overheat(struct idtp9222_struct* idtp9222) {
	#define TIMER_OVERHEAT_MS	3000
	/* On shutdown by overheat during wireless charging, send EPT by OVERHEAT */
	if (idtp9222->status_temperature >= idtp9222->configure_overheat) {
		if (!idtp9222_is_onpad(idtp9222) || idtp9222->status_overheat)
			return true;

		pr_idt(IDT_MONITOR, "The device is overheat, Send EPT_BY_OVERTEMP\n");

		if (idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_EPT, EPT_BY_OVERTEMP) &&
			idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_COMMAND, SEND_EPT)) {
			pr_idt(IDT_MONITOR, "Send EPT_BY_OVERTEMP!\n");
			idtp9222->status_overheat = true;
			schedule_delayed_work(&idtp9222->timer_overheat,
				round_jiffies_relative(msecs_to_jiffies(TIMER_OVERHEAT_MS)));
		}
		else {
			pr_idt(IDT_ERROR, "Failed to turning off by EPT_BY_OVERTEMP\n");
			return false;
		}
	}
	else {
		idtp9222->status_overheat = false;
	}

	return true;
}

static bool idtp9222_is_onpad(struct idtp9222_struct* idtp9222) {
       /* Refer to shadow here,
	* And be sure that real GPIO may indicate diffrent value of onpad.
	*/
	return idtp9222->status_onpad;
}

static bool idtp9222_is_enabled(struct idtp9222_struct* idtp9222) {
	bool status = !get_effective_result_locked(idtp9222->wlc_disable);

	pr_assert(!gpio_get_value(idtp9222->gpio_disabled)==status);
	return status;
}

static bool idtp9222_is_full(struct idtp9222_struct* idtp9222) {
	bool status = idtp9222->status_full;

	if (idtp9222_is_onpad(idtp9222)) {
		u8 val = !status;
		if (idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_EPT, &val))
			pr_assert(status==!!val);

		return status;
	}
	else {
		pr_idt(IDT_VERBOSE, "idtp9222 is off now\n");

		pr_assert(status==false); // The status should be false on offline
		return false;
	}
}

#define WA_MAX_COUNT   5
static bool psy_set_debug_battery(struct idtp9222_struct* idtp9222, int num) {
	static int debug_cnt = 0;

	if (num == 0) {
		idtp9222_set_current(idtp9222, WLC_WA_VOTER, false, 0);
		debug_cnt = 0;
	}
	else if (num == 1) { /* dcin-uv */
		if (!idtp9222->opmode_midpower && debug_cnt >= WA_MAX_COUNT) {
			vote(idtp9222->wlc_disable, DISABLE_BY_WA, true, 0);
			vote(idtp9222->wlc_disable, DISABLE_BY_WA, false, 0);
			pr_idt(IDT_UPDATE, "Exceed count! Restart IDT chip...\n");
			debug_cnt = 0;
		}
		else {
			int current_ua = idtp9222_get_current(idtp9222, NULL) - 50000;

			idtp9222_set_current(idtp9222, WLC_WA_VOTER, true,
				max(current_ua, IDTP9222_CURRENT_MA_MIN*1000));
			pr_idt(IDT_UPDATE, "Success to restart! cnt=%d\n", debug_cnt++);
		}
	}
	else if (num == 2) { /* vashdn */
		vote(idtp9222->wlc_disable, DISABLE_BY_WA, true, 0);
		vote(idtp9222->wlc_disable, DISABLE_BY_WA, false, 0);
		pr_idt(IDT_VERBOSE, "Restart IDT chip for Vashdn workaround!\n");
	}
	else
		pr_idt(IDT_VERBOSE, "Skip debug_battery due to abnormal command!\n");

	return true;
}

static bool psy_set_charge_done(struct idtp9222_struct* idtp9222, bool charge_done) {
	if (idtp9222->status_done == charge_done) {
		pr_idt(IDT_VERBOSE, "status_done is already set to %d\n", charge_done);
		return false;
	}

	pr_idt(IDT_INTERRUPT, "charge_done is %s charging!\n",
		charge_done ? "true, stop" : "false, start");
	vote(idtp9222->wlc_suspend, DISABLE_BY_EOC, charge_done, 0);

	if (charge_done) {
		if (delayed_work_pending(&idtp9222->timer_setoff))
			cancel_delayed_work(&idtp9222->timer_setoff);
		psy_set_debug_battery(idtp9222, 0);
	}
	idtp9222->status_done = charge_done;

	return true;
}

static bool psy_set_onpad(struct idtp9222_struct* idtp9222, bool onpad) {
	if (onpad) {
		u8 value = 0;
	       /* On-pad conditions - Only for "dc" online signal
		*/

		// Check firmware version (may be >= 0x12)
		idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_FIRMWARE, &value);
		pr_idt(IDT_REGISTER, "REG_ADDR_FIRMWARE : 0x%02x\n", value);

		// Update system's operating mode {WPC, PMA} and {MIDPOWER, or not}
		idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_OPMODE, &value);
		if (value & OPMODE_WPC)
			idtp9222->opmode_type = WPC;
		else if (value & OPMODE_PMA)
			idtp9222->opmode_type = PMA;
		else
			idtp9222->opmode_type = UNKNOWN;
		idtp9222->opmode_midpower = !!(value & OPMODE_MIDPOWER);

		idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_VOUT, &value);
		pr_idt(IDT_REGISTER, "REG_ADDR_VOUT(%s) : 0x%02x\n",
			idtp9222->opmode_midpower ? "9W" : "5W",  value);
		if (idtp9222->opmode_midpower && value != VOUT_V9P0)
			idtp9222->opmode_midpower = false;

		idtp9222_set_current(idtp9222, DEFAULT_VOTER,
			true, idtp9222->opmode_midpower ?
			idtp9222->configure_eppcurr : idtp9222->configure_bppcurr);
		vote(idtp9222->wlc_voltage, DEFAULT_VOTER,
			true, idtp9222->opmode_midpower ?
			IDTP9222_VOLTAGE_MV_EPP : idtp9222->configure_bppvolt);
		idtp9222_set_fod(idtp9222);

		idtp9222->status_overheat = false;
		schedule_delayed_work(&idtp9222->polling_temp, 0);

		idtp9222_wakelock_release(&idtp9222->wlc_wakelock);

		idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_TXID, &value);
		pr_idt(IDT_REGISTER, "Finally, TX id = 0x%02x = %s%s\n", value,
			idtp9222_modename(idtp9222->opmode_type),
			idtp9222->opmode_type == UNKNOWN ? "" :
			(idtp9222->opmode_midpower ? "(9W)" : "(5W)"));
	}
	else { /* Off pad conditions
		* 1: idtp9222->gpio_detached HIGH (means device is far from pad) or
		* 2: idtp9222->gpio_disabled HIGH (means USB inserted) or
		* 3: psy_dc->PRESENT '0' over 5 secs
		*/
		pr_assert(!idtp9222->status_dcin
			|| !!gpio_get_value(idtp9222->gpio_detached)
			|| !!gpio_get_value(idtp9222->gpio_disabled));

		idtp9222->opmode_type = UNKNOWN;
		idtp9222->opmode_midpower = false;
		idtp9222->status_full = false;
		psy_set_charge_done(idtp9222, false);
		psy_set_debug_battery(idtp9222, 0);

		// Stop polling event for overheat
		cancel_delayed_work(&idtp9222->polling_temp);
	}

	if (idtp9222->status_onpad != onpad) {
		idtp9222->status_onpad = onpad;
		power_supply_changed(idtp9222->wlc_psy);
		pr_idt(IDT_UPDATE, "%s onpad %d\n", IDTP9222_NAME_PSY,
			idtp9222_is_onpad(idtp9222));
		return true;
	}
	else
		return false;
}

static bool psy_set_dcin(struct idtp9222_struct* idtp9222, bool dcin) {
#define UNVOTING_TIMER_MS 5000
#define OFFLINE_TIMER_MS 5000

	if (idtp9222->status_dcin != dcin) {
		idtp9222->status_dcin = dcin;

		/* In the case of DCIN, release IBAT/IDC for 5 secs to establish wireless link */
		if (idtp9222->status_dcin) {
			idtp9222_set_overheat(idtp9222);
			idtp9222_set_capacity(idtp9222);

			if (idtp9222->opmode_midpower) {
				pr_idt(IDT_UPDATE, "Releasing IBAT/IDC for %d ms\n", UNVOTING_TIMER_MS);
				idtp9222_set_maxinput(idtp9222, true);
				schedule_delayed_work(&idtp9222->timer_maxinput,
					round_jiffies_relative(msecs_to_jiffies(UNVOTING_TIMER_MS)));
			}
			else {
				rerun_election(idtp9222->wlc_voltage);
			}
		}
		else {
			pr_idt(IDT_UPDATE, "Canceling maxinput timer\n");
			cancel_delayed_work(&idtp9222->timer_maxinput);
			idtp9222_set_maxinput(idtp9222, false);
		}

		/* In the case of !DCIN, start timer to check real offline */
		if (!idtp9222->status_dcin
			&& !idtp9222->status_overheat
			&& !idtp9222->status_done
			&& !delayed_work_pending(&idtp9222->timer_setoff)) {
			pr_idt(IDT_MONITOR, "Start checking presence after %d ms\n",
				OFFLINE_TIMER_MS);
			schedule_delayed_work(&idtp9222->timer_setoff,
				round_jiffies_relative(msecs_to_jiffies(OFFLINE_TIMER_MS)));
		}
		else if (idtp9222->status_dcin
			&& delayed_work_pending(&idtp9222->timer_setoff)) {
			pr_idt(IDT_MONITOR, "Cancel checking presence due to DCIN present\n");
			cancel_delayed_work(&idtp9222->timer_setoff);
		}
		else
			;

		return true;
	}
	else
		return false;

}

static bool psy_set_enable(struct idtp9222_struct* idtp9222, bool enable) {
	if (idtp9222->wlc_disable)
		vote(idtp9222->wlc_disable, DISABLE_BY_USB, !enable, 0);

	return true;
}

static bool psy_set_suspend(struct idtp9222_struct* idtp9222, bool suspend) {
	if (idtp9222->wlc_disable)
		vote(idtp9222->wlc_disable, DISABLE_BY_WA, suspend, 0);

	return true;
}

static bool psy_set_capacity(struct idtp9222_struct* idtp9222, int capacity) {
	if (idtp9222->status_capacity == capacity) {
		pr_idt(IDT_VERBOSE, "status_capacity is already set to %d\n", capacity);
		return false;
	}

	idtp9222->status_capacity = capacity;
	if (idtp9222_is_onpad(idtp9222))
		idtp9222_set_capacity(idtp9222);

	if (idtp9222->status_capacity == 100)
		idtp9222_set_current(idtp9222, WLC_CS100_VOTE, true,
			IDTP9222_CURRENT_MA_MIN*1000);
	else
		idtp9222_set_current(idtp9222, WLC_CS100_VOTE, false, 0);

	return true;
}

static bool psy_set_capacity_raw(struct idtp9222_struct* idtp9222, int capacity_raw) {
	if (idtp9222->status_capacityraw == capacity_raw) {
		pr_idt(IDT_VERBOSE, "status_capacityraw is already set to %d\n", capacity_raw);
		return false;
	}

	idtp9222->status_capacityraw = capacity_raw;
	if (idtp9222->status_done && capacity_raw <= idtp9222->configure_recharge) {
		psy_set_charge_done(idtp9222, false);

		if (!delayed_work_pending(&idtp9222->timer_setoff)) {
			pr_idt(IDT_MONITOR, "Start checking presence after %d ms\n",
				OFFLINE_TIMER_MS);
			schedule_delayed_work(&idtp9222->timer_setoff,
				round_jiffies_relative(msecs_to_jiffies(OFFLINE_TIMER_MS)));
		}
	}

	return true;
}

static bool psy_set_temperature(struct idtp9222_struct* idtp9222, int temperature) {
	if (idtp9222->status_temperature == temperature) {
		pr_idt(IDT_VERBOSE, "status_temperature is already set to %d\n", temperature);
		return false;
	}

	pr_idt(IDT_VERBOSE, "status_temperature %d -> %d\n",
		idtp9222->status_temperature, temperature);
	idtp9222->status_temperature = temperature;
	idtp9222_set_overheat(idtp9222);

	return true;
}

static int psy_get_power(struct idtp9222_struct* idtp9222) {
	int power = 0;

	if (idtp9222_is_onpad(idtp9222)) {
		int voltage_mv = idtp9222->opmode_midpower
			? idtp9222->configure_eppvolt : idtp9222->configure_bppvolt;
		int current_ma = (idtp9222->opmode_midpower
			? idtp9222->configure_eppcurr : idtp9222->configure_bppcurr) / 1000;

		power = voltage_mv * current_ma;
	}

	return power;
}

static int psy_get_current_max(struct idtp9222_struct* idtp9222) {
	if (idtp9222_is_onpad(idtp9222))
		return idtp9222->opmode_midpower
			? idtp9222->configure_eppcurr : idtp9222->configure_bppcurr;
	else
		return 0;
}

static int psy_get_current_now(struct idtp9222_struct* idtp9222) {
	u8 value = -1;
	int result = 0;

	if (idtp9222_is_onpad(idtp9222)) {
		idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_IADC_M, &value);
		result = value << 8;
		idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_IADC_L, &value);
		result |= value;
	}

	return result;
}

static int psy_get_voltage_max(struct idtp9222_struct* idtp9222) {
	if (idtp9222_is_onpad(idtp9222))
		return (idtp9222->opmode_midpower
			? IDTP9222_VOLTAGE_MV_EPP : IDTP9222_VOLTAGE_MV_BPP)
			* 1000 /* Convert to uV */;
	else
		return 0;
}

static int psy_get_voltage_max_design(struct idtp9222_struct* idtp9222) {
	u8 value = -1;

	if (idtp9222_is_onpad(idtp9222)) {
		idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_VOUT, &value);
		return (value == VOUT_V9P0
			? IDTP9222_VOLTAGE_MV_EPP : IDTP9222_VOLTAGE_MV_BPP)
			* 1000 /* Convert to uV */;
	}
	else
		return 0;
}

static int psy_get_voltage_now(struct idtp9222_struct* idtp9222) {
	u8 value = -1;
	int result = 0;

	if (idtp9222_is_onpad(idtp9222)) {
		idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_VADC_M, &value);
		result = value << 8;
		idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_VADC_L, &value);
		result |= value;
	}

	return result;
}

static enum power_supply_property psy_property_list [] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_DONE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_DEBUG_BATTERY,
};

static int psy_property_set(struct power_supply* psy,
	enum power_supply_property prop, const union power_supply_propval* val) {

	struct idtp9222_struct* idtp9222 = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_DONE:
		psy_set_charge_done(idtp9222, !!val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		psy_set_enable(idtp9222, !!val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		psy_set_suspend(idtp9222, !!val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (idtp9222->wlc_voltage) {
			if (val->intval == IDTP9222_VOLTAGE_MV_EPP) {
				idtp9222_set_current(idtp9222, WLC_THERMAL_VOTER, false, 0);
				vote(idtp9222->wlc_voltage, WLC_THERMAL_VOTER, false, 0);
			}
			else {
				idtp9222_set_current(idtp9222, WLC_THERMAL_VOTER, true,
					idtp9222->configure_bppcurr);
				vote(idtp9222->wlc_voltage, WLC_THERMAL_VOTER, true,
					max(val->intval, idtp9222->configure_bppvolt));
			}
		}
		break;
	case POWER_SUPPLY_PROP_DEBUG_BATTERY:
		psy_set_debug_battery(idtp9222, val->intval);
		break;
	default:
		break;
	}
	return 0;
}

static int psy_property_get(struct power_supply* psy,
	enum power_supply_property prop, union power_supply_propval* val) {

	struct idtp9222_struct* idtp9222 = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
#ifdef CONFIG_LGE_PM_VENEER_PSY
{      /* Basically, IDTP9222's ONLINE and PRESENT are same.
	* But in the some cases of LGE scenario, 'wireless' psy is required to pretend to 'OFFLINE' as fake.
	*/
	if (veneer_voter_suspended(VOTER_TYPE_IDC) == CHARGING_SUSPENDED_WITH_FAKE_OFFLINE) {
		pr_idt(IDT_RETURN, "Set Wireless UI as discharging");
		val->intval = false;
		break;
	}
}	// Go through to check PRESENT, Be sure no 'break;' here.
#endif
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = idtp9222_is_onpad(idtp9222);
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = psy_get_power(idtp9222);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = psy_get_current_max(idtp9222);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = psy_get_current_now(idtp9222);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = psy_get_voltage_max(idtp9222);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = psy_get_voltage_max_design(idtp9222);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = psy_get_voltage_now(idtp9222);
		break;
	case POWER_SUPPLY_PROP_CHARGE_DONE:
		val->intval = idtp9222_is_full(idtp9222);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = idtp9222_is_enabled(idtp9222);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
	case POWER_SUPPLY_PROP_DEBUG_BATTERY:
		/* Do nothing and just consume getting */
		val->intval = -1;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int psy_property_writeable(struct power_supply* psy,
	enum power_supply_property prop) {

	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_DONE:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}

	return rc;
}

static void psy_external_changed(struct power_supply* psy_me) {
	struct idtp9222_struct* idtp9222 = power_supply_get_drvdata(psy_me);
	union power_supply_propval value = { .intval = 0, };

	struct power_supply* psy_dc	 = power_supply_get_by_name("dc");
	struct power_supply* psy_battery = power_supply_get_by_name("battery");

	if (psy_dc) {
		static bool online_cached = false;

		if (!power_supply_get_property(psy_dc, POWER_SUPPLY_PROP_ONLINE, &value)) {
			bool online_now = !!value.intval;
			/* calling psy_set_onpad(true) only if online false -> true */
			if (online_cached != online_now) {
				online_cached = online_now;
				if (online_now)
					psy_set_onpad(idtp9222, true);
			}
		}
		if (!power_supply_get_property(psy_dc, POWER_SUPPLY_PROP_PRESENT, &value))
			psy_set_dcin(idtp9222, !!value.intval);

		power_supply_put(psy_dc);
	}
	if (psy_battery) {
		if (!power_supply_get_property(psy_battery, POWER_SUPPLY_PROP_CAPACITY, &value))
			psy_set_capacity(idtp9222, value.intval);

		if (!power_supply_get_property(psy_battery, POWER_SUPPLY_PROP_RECHARGE_SOC, &value))
			idtp9222->configure_recharge = value.intval * 10;

		if (!power_supply_get_property(psy_battery, POWER_SUPPLY_PROP_CAPACITY_RAW, &value))
			psy_set_capacity_raw(idtp9222, value.intval * 1000 / 255);

		power_supply_put(psy_battery);
	}
}

static void idtp9222_timer_maxinput(struct work_struct* work) {
	struct idtp9222_struct* idtp9222 = container_of(work, struct idtp9222_struct,
		timer_maxinput.work);

	pr_idt(IDT_UPDATE, "Releasing timer is expired!\n");
	idtp9222_set_maxinput(idtp9222, false);
	rerun_election(idtp9222->wlc_voltage);
}

static void idtp9222_timer_setoff(struct work_struct* work) {
	struct idtp9222_struct* idtp9222 = container_of(work, struct idtp9222_struct,
		timer_setoff.work);

	#define RECOVERY_TIMER_MS 1000
	if (idtp9222_is_onpad(idtp9222)) {
		struct power_supply* psy  = power_supply_get_by_name("dc");
		union power_supply_propval val = { .intval = 0, };

		if (psy) {
			if (!power_supply_get_property(psy,
				POWER_SUPPLY_PROP_PRESENT, &val) && !val.intval) {
				pr_idt(IDT_UPDATE, "FOD Detection!\n");
				psy_set_onpad(idtp9222, false);
			}
			power_supply_put(psy);
		}
	}
}

static void idtp9222_timer_overheat(struct work_struct* work) {
	struct idtp9222_struct* idtp9222 = container_of(work, struct idtp9222_struct,
		timer_overheat.work);
	struct power_supply* psy_dc = power_supply_get_by_name("dc");
	union power_supply_propval value = { .intval = 0, };

	if (psy_dc) {
		if (!power_supply_get_property(psy_dc, POWER_SUPPLY_PROP_PRESENT, &value)) {
			if (!!value.intval) {
				idtp9222->status_overheat = false;
				idtp9222_set_overheat(idtp9222);
			}
			else
				pr_idt(IDT_MONITOR, "Success to turn off Tx by OVERHEAT\n");
		}

		power_supply_put(psy_dc);
	}
}

#ifdef CONFIG_LGE_COVER_DISPLAY
static void idtp9222_timer_hallic(struct work_struct* work) {
	struct idtp9222_struct* idtp9222 = container_of(work, struct idtp9222_struct,
		timer_hallic.work);
	if (idtp9222_is_onpad(idtp9222)) {
		if (idtp9222->opmode_midpower)
			idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_GLOBALGAIN, 0xC0);
		else
			idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_GLOBALGAIN, 0x80);
	}
}
#endif

static void idtp9222_polling_temp(struct work_struct* work) {
	struct idtp9222_struct* idtp9222 = container_of(work, struct idtp9222_struct,
		polling_temp.work);
	struct power_supply* psy_battery = power_supply_get_by_name("battery");
	union power_supply_propval value = { .intval = 0, };

	#define POLLING_TEMP_MS 30000
	if (psy_battery) {
		if (!power_supply_get_property(psy_battery, POWER_SUPPLY_PROP_TEMP, &value))
			psy_set_temperature(idtp9222, value.intval);

		power_supply_put(psy_battery);
	}

	schedule_delayed_work(&idtp9222->polling_temp, round_jiffies_relative
		(msecs_to_jiffies(POLLING_TEMP_MS)));
}

#define ABNORMAL_VRECT_MV	2900
static void idtp9222_polling_volt(struct work_struct* work) {
	struct idtp9222_struct* idtp9222 = container_of(work, struct idtp9222_struct,
		polling_volt.work);
	bool success = false;
	u8 value = -1;
	int result = 0;

	success = idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_VRECT_M, &value);
	result = value << 8;
	success &= idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_VRECT_L, &value);
	result |= value;

	if (success && result < ABNORMAL_VRECT_MV) {
		pr_idt(IDT_UPDATE, "Vrect : %dmV\n", result);
		wa_clear_dc_reverse_volt_trigger(false);
	}

	if (!gpio_get_value(idtp9222->gpio_detached))
		schedule_delayed_work(&idtp9222->polling_volt,
			round_jiffies_relative(msecs_to_jiffies(1000)));
}

static void idtp9222_logger_gpios(struct work_struct* work) {
	struct idtp9222_struct* idtp9222 = container_of(work, struct idtp9222_struct,
		logger_gpios.work);

	// Monitor 3 GPIOs
	int idtfault = gpio_get_value(idtp9222->gpio_idtfault);
	int detached = gpio_get_value(idtp9222->gpio_detached);
	int disabled = get_effective_result_locked(idtp9222->wlc_disable);
	pr_err(	"MSM_GPIO%d(<-OD2_N, idtfault):%d, "
		"MSM_GPIO%d(<-PDT_N, detached):%d, "
		"PMI_GPIO%d(->OFF_P, disabled):%d\n",
		idtp9222->gpio_idtfault, idtfault,
		idtp9222->gpio_detached, detached,
		idtp9222->gpio_disabled, disabled);

	if (false) {
		/* for debugging */
		idtp9222_reg_dump(idtp9222);
	}

	schedule_delayed_work(&idtp9222->logger_gpios, round_jiffies_relative
		(msecs_to_jiffies(1000*30)));
}

static int idtp9222_disable_callback(struct votable *votable, void *data,
	int disabled, const char *client) {
	struct idtp9222_struct* idtp9222 = data;
	u8 txid = -1;

	if (idtp9222_is_onpad(idtp9222) && !!disabled
		&& idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_TXID, &txid)
		&& txid == 0x63) {
		pr_idt(IDT_MONITOR, "psy_set_enable EPT_BY_NORESPONSE\n");
		idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_EPT, EPT_BY_NORESPONSE);
		idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_COMMAND, SEND_EPT);
	}

	gpiod_set_value(gpio_to_desc(idtp9222->gpio_disabled), !!disabled);
	if (disabled && strcmp(client, DISABLE_BY_WA))
		psy_set_onpad(idtp9222, false);

	msleep(20);

	pr_assert(gpio_get_value(idtp9222->gpio_disabled)==disabled);
	pr_idt(IDT_INTERRUPT, "idtp9222_disable_callback is written %s : %d\n",
		(gpio_get_value(idtp9222->gpio_disabled) == disabled)
		? "success!" : "fail!", disabled);

	return 0;
}

static int idtp9222_voltage_callback(struct votable *votable, void *data,
	int mV, const char *client) {
	struct idtp9222_struct* idtp9222 = data;

	if (mV < 0)
		return 0;

	if (idtp9222_is_onpad(idtp9222)) {
		u8 value = -1;

		idtp9222_reg_write(idtp9222->wlc_client, REG_ADDR_VOUT, mV / 100);
		idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_VOUT, &value);
		pr_idt(IDT_REGISTER, "set voltage %dmV(0x%02x)\n", mV, value);
	}

	return 0;
}

static int idtp9222_suspend_callback(struct votable *votable, void *data,
	int suspend, const char *client) {
	struct power_supply* psy_dc = power_supply_get_by_name("dc");
	union power_supply_propval value = { .intval = suspend, };

	if (psy_dc) {
		power_supply_set_property(psy_dc,
			POWER_SUPPLY_PROP_INPUT_SUSPEND, &value);
		power_supply_put(psy_dc);
	}
	else
		pr_idt(IDT_VERBOSE, "Couldn't get psy_dc!\n");

	return 0;
}

static irqreturn_t idtp9222_isr_idtfault(int irq, void* data) {
       /* This ISR will be triggered on below unrecoverable exceptions :
	* Over temperature, Over current, or Over voltage detected by IDTP922X chip.
	* IDTP9222 turns off after notifying it to host, so there's nothing to handle
	* except logging here.
	*/
	struct idtp9222_struct* idtp9222 = data;
	u8 value = 0;

	if (idtp9222_is_onpad(idtp9222))
		idtp9222_reg_read(idtp9222->wlc_client, REG_ADDR_INT, &value);

	pr_idt(IDT_INTERRUPT, "idtp9222_isr_idtfault is triggered : %d(0x%02x)\n",
		gpio_get_value(idtp9222->gpio_idtfault), value);
	return IRQ_HANDLED;
}

static irqreturn_t idtp9222_isr_detached(int irq, void* data) {
	struct idtp9222_struct* idtp9222 = data;
	bool detached = !!gpio_get_value(idtp9222->gpio_detached);

	if (detached) {
		psy_set_onpad(idtp9222, false);
#ifdef CONFIG_LGE_COVER_DISPLAY
		if (delayed_work_pending(&idtp9222->timer_hallic))
			cancel_delayed_work(&idtp9222->timer_hallic);
#endif
		if (delayed_work_pending(&idtp9222->polling_volt))
			cancel_delayed_work(&idtp9222->polling_volt);
		idtp9222_wakelock_release(&idtp9222->wlc_wakelock);
	}
	else {
		idtp9222_wakelock_acquire(&idtp9222->wlc_wakelock);
#ifdef CONFIG_LGE_COVER_DISPLAY
		idtp9222_set_hallic(idtp9222);
#endif
		wa_clear_dc_reverse_volt_trigger(true);
		schedule_delayed_work(&idtp9222->polling_volt,
			round_jiffies_relative(msecs_to_jiffies(1000)));
	}

	pr_idt(IDT_INTERRUPT, "idtp9222_isr_detached is triggered : %d\n",
		detached);
	return IRQ_HANDLED;
}

static bool idtp9222_probe_devicetree(struct device_node* dnode,
	struct idtp9222_struct* idtp9222) {
	const char*	arr = NULL;
	int		buf = -1;

	if (!dnode) {
		pr_idt(IDT_ERROR, "dnode is null\n");
		return false;
	}

/* Parse GPIOs */
	idtp9222->gpio_idtfault = of_get_named_gpio(dnode, "idt,gpio-idtfault", 0);
	if (idtp9222->gpio_idtfault < 0) {
		pr_idt(IDT_ERROR, "Fail to get gpio-idtfault\n");
		return false;
	}

	idtp9222->gpio_detached = of_get_named_gpio(dnode, "idt,gpio-detached", 0);
	if (idtp9222->gpio_detached < 0) {
		pr_idt(IDT_ERROR, "Fail to get gpio-detached\n");
		return false;
	}

	idtp9222->gpio_disabled = of_get_named_gpio(dnode, "idt,gpio-disabled", 0);
	if (idtp9222->gpio_disabled < 0) {
		pr_idt(IDT_ERROR, "Fail to get gpio-disabled\n");
		return false;
	}

/* Parse FOD parameters */
	arr = of_get_property(dnode, "idt,fod-bpp", &buf);
	if (!arr || buf != ARRAY_SIZE(idtp9222->fod_bpp)) {
		pr_idt(IDT_ERROR, "Fail to get idt,fod-bpp\n");
		return false;
	}
	else
		memcpy(idtp9222->fod_bpp, arr, buf);

	arr = of_get_property(dnode, "idt,fod-epp", &buf);
	if (!arr || buf != ARRAY_SIZE(idtp9222->fod_epp)) {
		pr_idt(IDT_ERROR, "Fail to get idt,fod-epp\n");
		return false;
	}
	else
		memcpy(idtp9222->fod_epp, arr, buf);

/* Parse misc */
	if (of_property_read_u32(dnode, "idt,configure-bppcurr", &buf) < 0)
		idtp9222->configure_bppcurr = INT_MAX;
	else
		idtp9222->configure_bppcurr = buf;

	if (of_property_read_u32(dnode, "idt,configure-eppcurr", &buf) < 0)
		idtp9222->configure_eppcurr = INT_MAX;
	else
		idtp9222->configure_eppcurr = buf;

	if (of_property_read_u32(dnode, "idt,configure-bppvolt", &buf) < 0)
		idtp9222->configure_bppvolt = INT_MAX;
	else
		idtp9222->configure_bppvolt = buf;

	if (of_property_read_u32(dnode, "idt,configure-eppvolt", &buf) < 0)
		idtp9222->configure_eppvolt = INT_MAX;
	else
		idtp9222->configure_eppvolt = buf;

	if (of_property_read_u32(dnode, "idt,configure-overheat", &buf) < 0) {
		pr_idt(IDT_ERROR, "Fail to get configure-overheat\n");
		return false;
	}
	else
		idtp9222->configure_overheat = buf;

	idtp9222->configure_sysfs = of_property_read_bool(dnode, "idt,configure-sysfs");

	return true;
}

static bool idtp9222_probe_gpios(struct idtp9222_struct* idtp9222) {
	struct pinctrl* 	gpio_pinctrl;
	struct pinctrl_state*	gpio_state;
	int ret;

	// PINCTRL here
	gpio_pinctrl = devm_pinctrl_get(idtp9222->wlc_device);
	if (IS_ERR_OR_NULL(gpio_pinctrl)) {
		pr_idt(IDT_ERROR, "Failed to get pinctrl (%ld)\n", PTR_ERR(gpio_pinctrl));
		return false;
	}

	gpio_state = pinctrl_lookup_state(gpio_pinctrl, "wlc_pinctrl");
	if (IS_ERR_OR_NULL(gpio_state)) {
		pr_idt(IDT_ERROR, "pinstate not found, %ld\n", PTR_ERR(gpio_state));
		return false;
	}

	ret = pinctrl_select_state(gpio_pinctrl, gpio_state);
	if (ret < 0) {
		pr_idt(IDT_ERROR, "cannot set pins %d\n", ret);
		return false;
	}

	// Set direction...
	ret = gpio_request_one(idtp9222->gpio_idtfault, GPIOF_DIR_IN, "gpio_idtfault");
	if (ret < 0) {
		pr_idt(IDT_ERROR, "Fail to request gpio_idtfault %d\n", ret);
		return false;
	}

	ret = gpio_request_one(idtp9222->gpio_detached, GPIOF_DIR_IN, "gpio_detached");
	if (ret < 0) {
		pr_idt(IDT_ERROR, "Fail to request gpio_detached, %d\n", ret);
		return false;
	}

	ret = gpio_request_one(idtp9222->gpio_disabled, GPIOF_DIR_OUT, "gpio_disabled");
	if (ret < 0) {
		pr_idt(IDT_ERROR, "Fail to request gpio_disabled %d\n", ret);
		return false;
	}

	return true;
}

static bool idtp9222_probe_psy(/* @Nonnulll */ struct idtp9222_struct* idtp9222) {
	const static struct power_supply_desc desc = {
		.name = IDTP9222_NAME_PSY,
		.type = POWER_SUPPLY_TYPE_WIRELESS,
		.properties = psy_property_list,
		.num_properties = ARRAY_SIZE(psy_property_list),
		.get_property = psy_property_get,
		.set_property = psy_property_set,
		.property_is_writeable = psy_property_writeable,
		.external_power_changed = psy_external_changed,
	};
	const struct power_supply_config cfg = {
		.drv_data = idtp9222,
		.of_node = idtp9222->wlc_device->of_node,
	};

	idtp9222->wlc_psy = power_supply_register(idtp9222->wlc_device, &desc, &cfg);
	if (!IS_ERR(idtp9222->wlc_psy)) {
		static char* from [] = { "battery", "dc" };
		idtp9222->wlc_psy->supplied_from = from;
		idtp9222->wlc_psy->num_supplies = ARRAY_SIZE(from);
		return true;
	}
	else {
		pr_info("Couldn't register idtp9222 power supply (%ld)\n",
			PTR_ERR(idtp9222->wlc_psy));
		return false;
	}
}

static bool idtp9222_probe_irqs(struct idtp9222_struct* idtp9222) {
	int ret = 0;

	/* GPIO IDTFault */
	ret = request_threaded_irq(gpio_to_irq(idtp9222->gpio_idtfault),
		NULL, idtp9222_isr_idtfault, IRQF_ONESHOT|IRQF_TRIGGER_FALLING,
		"wlc-idtfault", idtp9222);
	if (ret) {
		pr_idt(IDT_ERROR, "Cannot request irq %d (%d)\n",
			gpio_to_irq(idtp9222->gpio_idtfault), ret);
		return false;
	}
	else
		enable_irq_wake(gpio_to_irq(idtp9222->gpio_idtfault));

	/* GPIO Detached */
	ret = request_threaded_irq(gpio_to_irq(idtp9222->gpio_detached),
		NULL, idtp9222_isr_detached, IRQF_ONESHOT|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
		"wlc-detached", idtp9222);
	if (ret) {
		pr_idt(IDT_ERROR, "Cannot request irq %d (%d)\n",
			gpio_to_irq(idtp9222->gpio_detached), ret);
		return false;
	}
	else
		enable_irq_wake(gpio_to_irq(idtp9222->gpio_detached));

	return true;
}

static int idtp9222_remove(struct i2c_client* client) {
	struct idtp9222_struct* idtp9222 = i2c_get_clientdata(client);
	pr_idt(IDT_VERBOSE, "idt9222 is about to be removed from system\n");

	if (idtp9222) {
	/* Clear descripters */
		if (delayed_work_pending(&idtp9222->timer_maxinput))
			cancel_delayed_work_sync(&idtp9222->timer_maxinput);
		if (delayed_work_pending(&idtp9222->timer_setoff))
			cancel_delayed_work_sync(&idtp9222->timer_setoff);
		if (delayed_work_pending(&idtp9222->timer_overheat))
			cancel_delayed_work_sync(&idtp9222->timer_overheat);
		if (delayed_work_pending(&idtp9222->polling_temp))
			cancel_delayed_work_sync(&idtp9222->polling_temp);
		if (delayed_work_pending(&idtp9222->polling_volt))
			cancel_delayed_work(&idtp9222->polling_volt);
		if (delayed_work_pending(&idtp9222->logger_gpios))
			cancel_delayed_work_sync(&idtp9222->logger_gpios);
		if (idtp9222->wlc_disable)
			destroy_votable(idtp9222->wlc_disable);

		if (idtp9222->wlc_psy)
			power_supply_unregister(idtp9222->wlc_psy);
	/* Clear gpios */
		if (idtp9222->gpio_idtfault)
			gpio_free(idtp9222->gpio_idtfault);
		if (idtp9222->gpio_detached)
			gpio_free(idtp9222->gpio_detached);
		if (idtp9222->gpio_disabled)
			gpio_free(idtp9222->gpio_disabled);
	/* Finally, make me free */
		kfree(idtp9222);
		return 0;
	}
	else
		return -EINVAL;
}

static int idtp9222_probe(struct i2c_client* client, const struct i2c_device_id* id) {
	struct idtp9222_struct* idtp9222 = kzalloc(sizeof(struct idtp9222_struct), GFP_KERNEL);

	pr_idt(IDT_VERBOSE, "Start\n");

	if (!idtp9222) {
		pr_idt(IDT_ERROR, "Failed to alloc memory\n");
		goto error;
	}
	else {	// Store the platform_data to drv_data
		i2c_set_clientdata(client, idtp9222);
	}

	idtp9222->dc_icl_votable = find_votable("DC_ICL");
	if (!idtp9222->dc_icl_votable) {
		pr_idt(IDT_ERROR, "unable to get DC_ICL votable\n");
		goto error;
	}

	// For client and device
	idtp9222->wlc_client = client;
	idtp9222->wlc_device = &client->dev;
	idtp9222->wlc_device->platform_data = idtp9222;

	// For remained preset
	if (!idtp9222_probe_devicetree(idtp9222->wlc_device->of_node, idtp9222)) {
		pr_idt(IDT_ERROR, "Fail to read parse_dt\n");
		goto error;
	}
	// For GPIOs
	if (!idtp9222_probe_gpios(idtp9222)) {
		pr_idt(IDT_ERROR, "Fail to request gpio at probe\n");
		goto error;
	}
	// For psy
	if (!idtp9222_probe_psy(idtp9222)) {
		pr_idt(IDT_ERROR, "Unable to register wlc_psy\n");
		goto error;
	}
	// Request irqs
	if (!idtp9222_probe_irqs(idtp9222)) {
		pr_idt(IDT_ERROR, "Fail to request irqs at probe\n");
		goto error;
	}
	// Create sysfs if it is configured
	if (idtp9222->configure_sysfs && sysfs_create_group(&idtp9222->wlc_device->kobj, &idtp9222_sysfs_files) < 0) {
		pr_idt(IDT_ERROR, "unable to create sysfs\n");
		goto error;
	}
	// For votables
	idtp9222->wlc_disable = create_votable("WLC_DISABLE",
		VOTE_SET_ANY,
		idtp9222_disable_callback,
		idtp9222);
	idtp9222->wlc_voltage = create_votable("WLC_VOLTAGE",
		VOTE_MIN,
		idtp9222_voltage_callback,
		idtp9222);
	idtp9222->wlc_suspend = create_votable("WLC_SUSPEND",
		VOTE_SET_ANY,
		idtp9222_suspend_callback,
		idtp9222);
	wakeup_source_init(&idtp9222->wlc_wakelock, "IDTP9222: wakelock");

	// For delayed works
	INIT_DELAYED_WORK(&idtp9222->timer_maxinput, idtp9222_timer_maxinput);
	INIT_DELAYED_WORK(&idtp9222->timer_setoff, idtp9222_timer_setoff);
	INIT_DELAYED_WORK(&idtp9222->timer_overheat, idtp9222_timer_overheat);
	INIT_DELAYED_WORK(&idtp9222->polling_temp, idtp9222_polling_temp);
	INIT_DELAYED_WORK(&idtp9222->polling_volt, idtp9222_polling_volt);
	INIT_DELAYED_WORK(&idtp9222->logger_gpios, idtp9222_logger_gpios);
#ifdef CONFIG_LGE_COVER_DISPLAY
	INIT_DELAYED_WORK(&idtp9222->timer_hallic, idtp9222_timer_hallic);
#endif
	schedule_delayed_work(&idtp9222->logger_gpios, 0);

	psy_external_changed(idtp9222->wlc_psy);
	pr_idt(IDT_VERBOSE, "Complete probing IDTP9222\n");
	return 0;

error:
	idtp9222_remove(client);
	return -EPROBE_DEFER;
}

//Compatible node must be matched to dts
static struct of_device_id idtp9222_match [] = {
	{ .compatible = IDTP9222_NAME_COMPATIBLE, },
	{ },
};

//I2C slave id supported by driver
static const struct i2c_device_id idtp9222_id [] = {
	{ IDTP9222_NAME_DRIVER, 0 },
	{ }
};

//I2C Driver Info
static struct i2c_driver idtp9222_driver = {
	.driver = {
		.name = IDTP9222_NAME_DRIVER,
		.owner = THIS_MODULE,
		.of_match_table = idtp9222_match,
	},
	.id_table = idtp9222_id,

	.probe = idtp9222_probe,
	.remove = idtp9222_remove,
};

//Easy wrapper to do driver init
module_i2c_driver(idtp9222_driver);

MODULE_DESCRIPTION(IDTP9222_NAME_DRIVER);
MODULE_LICENSE("GPL v2");
