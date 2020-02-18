/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _GPIO_KEYS_H
#define _GPIO_KEYS_H

#if defined (CONFIG_LGE_COVER_DISPLAY) || defined(CONFIG_LGE_DUAL_SCREEN)
#include <linux/usb/usbpd.h>
#include <linux/ice40/ice40-core.h>

enum dd_fw_update {
	DD_MCU_CANCEL			= 0,
	DD_MCU_FORCE_UPDATE	= 1,
	DD_MCU_UPDATE			= 2,
	DD_TOUCH_UPDATE		= 3,
	DD_TOUCH_MCU_UPDATE	= 4,
	DD_UPDATE_CLEAR		= 5,
	DD_MCU_RECOVERY		= 6
};

enum mcu_fw_update_status {
	MCU_FW_UPDATE_NOT_INITIALIZED      = -1,
	MCU_FW_UPDATE_READY_VERSION_CHECK  = 0,
	MCU_FW_UPDATE_FORCE                = 1,
	MCU_FW_UPDATE_NEED                 = 2,
	MCU_FW_UPDATE_NO_NEED              = 3,
	MCU_FW_UPDATE_READY_USER_CONFIRM   = 4,
	MCU_FW_UPDATE_USER_CONFIRM         = 5,
	MCU_FW_UPDATE_START                = 6,
};

enum coverfw_status {
	UPDATE_NO_NEED		= 0,
	UPDATE_NEED		= 1,
	UPDATE_DONE		= 2,
	RECOVERY_NEED		= 6,
};

enum mcu_status {
	MCU_NOT_INITIALIZED  = -1,
	MCU_5V_LOW           = 0,
	MCU_5V_HIGH          = 1,
	MCU_START_BOOTLOADER = 2,
	MCU_START_SYSTEM     = 3,
};

enum {
	OLD_TIANMA_ID = 1,
	NEW_TIANMA_ID = 2,
	TOVIS_ID = 3,
};
#endif

struct device;

/**
 * struct gpio_keys_button - configuration parameters
 * @code:		input event code (KEY_*, SW_*)
 * @gpio:		%-1 if this key does not support gpio
 * @active_low:		%true indicates that button is considered
 *			depressed when gpio is low
 * @desc:		label that will be attached to button's gpio
 * @type:		input event type (%EV_KEY, %EV_SW, %EV_ABS)
 * @wakeup:		configure the button as a wake-up source
 * @debounce_interval:	debounce ticks interval in msecs
 * @can_disable:	%true indicates that userspace is allowed to
 *			disable button via sysfs
 * @value:		axis value for %EV_ABS
 * @irq:		Irq number in case of interrupt keys
 */
struct gpio_keys_button {
	unsigned int code;
	int gpio;
	int active_low;
	const char *desc;
	unsigned int type;
	int wakeup;
	int debounce_interval;
	bool can_disable;
	int value;
	unsigned int irq;
};

/**
 * struct gpio_keys_platform_data - platform data for gpio_keys driver
 * @buttons:		pointer to array of &gpio_keys_button structures
 *			describing buttons attached to the device
 * @nbuttons:		number of elements in @buttons array
 * @poll_interval:	polling interval in msecs - for polling driver only
 * @rep:		enable input subsystem auto repeat
 * @enable:		platform hook for enabling the device
 * @disable:		platform hook for disabling the device
 * @name:		input device name
 */
struct gpio_keys_platform_data {
	const struct gpio_keys_button *buttons;
	int nbuttons;
	unsigned int poll_interval;
	unsigned int rep:1;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	const char *name;
};

#ifdef CONFIG_LGE_COVER_DISPLAY
int get_global_luke_status(void);
void hallic_handle_5v_boost_gpios(int state);
void hallic_handle_displayport_initialize(struct usbpd_svid_handler *handler);
extern struct ice40 *global_ice40;
extern int ice40_master_reg_write(struct ice40 *ice40, uint addr, uint val);
extern int ice40_master_reg_read(struct ice40 *ice40, uint addr, uint *val);
extern int ice40_enable(struct ice40 *ice40);
extern int ice40_mcu_reg_read(struct ice40 *ice40, uint addr, char *data, int len);
extern int ice40_mcu_reg_write(struct ice40 *ice40, uint addr, uint val);
extern int ice40_mcu_reg_write_norecovery(struct ice40 *ice40, uint addr, uint val);
extern int mcu_firmware_partialbulkwrite(struct ice40 *ice40, int select);
#endif

#endif
