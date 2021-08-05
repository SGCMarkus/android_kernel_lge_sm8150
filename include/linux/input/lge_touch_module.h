#ifndef __LINUX_TOUCH_MODULE_H
#define __LINUX_TOUCH_MODULE_H

#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>

#define MAX_FINGER		10

#define TOUCH_MODULE_IRQ_NONE		0
#define TOUCH_MODULE_IRQ_FINGER		(1 << 0)
#define TOUCH_MODULE_IRQ_KNOCK		(1 << 1)

#define EUPGRADE		140
#define EHWRESET_ASYNC		141
#define EHWRESET_SYNC		142
#define ESWRESET		143
#define EGLOBALRESET		144

enum {
	MODULE_POWER_OFF = 0,
	MODULE_POWER_SLEEP,
	MODULE_POWER_WAKE,
	MODULE_POWER_ON,
	MODULE_POWER_HW_RESET_ASYNC,
	MODULE_POWER_HW_RESET_SYNC,
	MODULE_POWER_SW_RESET,
	MODULE_POWER_DSV_TOGGLE,
	MODULE_POWER_DSV_ALWAYS_ON,
};

enum {
	MODULE_FIRMWARE_UPGRADE = 0,
	MODULE_ATTACHED_TYPE    = 1,
};

enum {
	DONE = 0,
	WAIT = 1,
};

enum {
	MODULE_NONE = 0,
	MODULE_PROBE,
	MODULE_UPGRADE,
	MODULE_NORMAL,
};

enum {
	FIRMWARE_UPGRADE_NOT_INITIALIZED = -1,
	FIRMWARE_UPGRADE_NO_NEED = 0,
	FIRMWARE_UPGRADE_NEED = 1,
	FIRMWARE_UPGRADE_DONE = 2,
	FIRMWARE_UPGRADE_FORCE = 3,
	FIRMWARE_UPGRADE_FAIL = 4,
};

enum {
	MODULE_INTERRUPT_DISABLE = 0,
	MODULE_INTERRUPT_ENABLE,
};

struct module_bus_msg {
	u8 *tx_buf;
	int tx_size;
	u8 *rx_buf;
	int rx_size;
};

 /*
  * struct for module device.
  * */
struct p_driver {
	int (*match)(struct device *dev);
	int (*probe)(struct device *dev);
	int (*remove)(struct device *dev);
	int (*func)(struct device *dev, int control, char *data);
	int (*notify)(int control, int data);
	int (*write)(struct device *dev, struct module_bus_msg *msg);
	int (*read)(struct device *dev, struct module_bus_msg *msg);
	int (*suspend)(struct device *dev);
	int (*resume)(struct device *dev);
	int (*init)(struct device *dev);
	int (*upgrade)(struct device *dev);
	int (*lpwg)(struct device *dev, u32 code, void *param);
	int (*ta_connect)(struct device *dev);
	int (*register_sysfs)(struct device *dev);
	void (*write_file)(struct device *dev, char *data, int write_time);
	void (*log_file_size_check)(struct device *dev);
	int (*touch_check_boot_mode)(struct device *dev);
};

struct dts_data {
	u32 reset_pin;
	u32 int_pin;
	u32 irqflags;
	u32 max_x;
	u32 max_y;
	u32 max_pressure;
	u32 max_width_major;
	u32 max_width_minor;
	u32 max_orientation;
	u32 max_id;
	u32 hw_reset_delay;
	u32 sw_reset_delay;
	u32 vcl_pin;
	u32 vdd_pin;

	u8 def_fwcnt;
	const char *def_fwpath[4];
	char test_fwpath[256];
	const char *panel_spec;
	const char *panel_spec_mfts;
	const char *dual_panel_spec[4];
	const char *dual_panel_spec_mfts[4];
};

struct module_pinctrl {
	struct pinctrl *ctrl;
	struct pinctrl_state *active;
	struct pinctrl_state *suspend;
};

struct touch_data_module {
	u16 id;
	u16 x;
	u16 y;
	u16 width_major;
	u16 width_minor;
	s16 orientation;
	u16 pressure;
	/* finger, palm, pen, glove, hover */
	u16 type;
};

struct m_point {
	int x;
	int y;
};

struct m_lpwg_info {
	u8 mode;
	u8 screen;
	u8 sensor;
	u8 qcover;
	u8 code_num;
	struct m_point area[2];
	struct m_point code[128];
};

/* data struct communicating with touch_core*/
struct module_data {
	u8 bus_type;
	int irq;

	struct platform_device *pdev;
	struct device *dev;			/* client device : i2c or spi */
	struct input_dev *input;
	struct p_driver m_driver;
	struct kobject kobj;

	struct dts_data dts;
	struct module_pinctrl pinctrl;
	struct mutex io_lock;
	struct mutex lock;

	struct pm_qos_request pm_qos_req;
	struct workqueue_struct *wq;
	struct delayed_work init_work;
	struct delayed_work upgrade_work;
	struct delayed_work debug_work;
	u8 *tx_buf;
	u8 *rx_buf;
	void *vcl;
	void *vdd;

	u32 intr_status;
	u16 new_mask;
	u16 old_mask;

	const char *panel_spec;
	const char *panel_spec_mfts;

	atomic_t core;
	atomic_t irq_enable;

	int tcount;
	struct touch_data_module tdata[MAX_FINGER];
	struct m_lpwg_info lpwg;
	void *module_device_data;
	int is_cancel;
	u8 force_fwup;
	int check_fwup;
	u8 display_id;
//	struct lpwg_info lpwg;
//	struct tci_ctrl tci;
//	struct swipe_ctrl swipe[4];	/*down, up, right, left */
};

struct pdev_list {
	struct device *sub_dev[10];
	void *touch_core_data;
	int attached_type;
	/* add?? */
};
extern struct pdev_list *plist;

#ifdef CONFIG_LGE_COVER_DISPLAY
extern void notify_touch_version_check(int state);
#endif

static inline void module_set_device(struct module_data *md, void *data)
{
	md->module_device_data = data;
}

static inline void *module_get_device(struct module_data *md)
{
	return md->module_device_data;
}

static inline struct module_data *to_module(struct device *dev)
{
	return (struct module_data *)dev_get_drvdata(dev);
}

#endif /* _LINUX_TOUCH_MODULE_H */
