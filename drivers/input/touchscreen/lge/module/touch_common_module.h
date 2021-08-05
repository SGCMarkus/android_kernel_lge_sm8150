#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/firmware.h>
#include <linux/buffer_head.h>
#include <linux/regulator/consumer.h>
#include <soc/qcom/lge/board_lge.h>
#include "../../../../../kernel/irq/internals.h"
#include <linux/input/lge_touch_module.h>

// ice40 struct to enable touch reset
extern struct ice40 *global_ice40;
extern int ice40_master_reg_write(struct ice40 *ice40, uint addr, uint val);
extern int ice40_master_reg_read(struct ice40 *ice40, uint addr, uint *val);
extern int ice40_mcu_reg_read(struct ice40 *ice40, uint addr, char *data, int len);
extern int link_tr_state;

#define PROPERTY_GPIO(np, string, target)				\
	(target = of_get_named_gpio_flags(np, string, 0, NULL))

#define PROPERTY_BOOL(np, string, target)				\
	do {								\
		u32 tmp_val = 0;					\
		if (of_property_read_u32(np, string, &tmp_val) < 0)	\
			target = 0;					\
		else							\
			target = (u8)tmp_val;				\
	} while (0)

#define PROPERTY_U32(np, string, target)				\
	do {								\
		u32 tmp_val = 0;					\
		if (of_property_read_u32(np, string, &tmp_val) < 0)	\
			target = -1;					\
		else							\
			target = tmp_val;				\
	} while (0)

#define PROPERTY_STRING_ARRAY(np, string, target, cnt)			\
	do {								\
		int i;						\
		cnt = of_property_count_strings(np, string);		\
		for (i = 0; i < cnt; i++) {				\
			of_property_read_string_index(np, string, i,	\
						      &target[i]);	\
		}							\
	} while (0)

#define PROPERTY_STRING(np, string, target)				\
	do {								\
		of_property_read_string(np, string, &target);		\
	} while (0)

enum { /* Command lists */
	CMD_VERSION,
	CMD_ATCMD_VERSION,
};

enum {
	TOUCH_NORMAL_BOOT = 0,
	TOUCH_MINIOS_AAT,
	TOUCH_MINIOS_MFTS_FOLDER,
	TOUCH_MINIOS_MFTS_FLAT,
	TOUCH_MINIOS_MFTS_CURVED,
	TOUCH_CHARGER_MODE,
	TOUCH_LAF_MODE,
	TOUCH_RECOVERY_MODE,
};

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

enum TOUCH_DEBUG {
	_NONE                      = 0,
	BASE_INFO                 = (1U << 0),    /* 1 */
	TRACE                     = (1U << 1),    /* 2 */
	GET_DATA                  = (1U << 2),    /* 4 */
	ABS                       = (1U << 3),    /* 8 */
	BUTTON                    = (1U << 4),    /* 16*/
	FW_UPGRADE                = (1U << 5),    /* 32 */
	GHOST                     = (1U << 6),    /* 64 */
	IRQ_HANDLE                = (1U << 7),    /* 128 */
	POWER                     = (1U << 8),    /* 256 */
	JITTER                    = (1U << 9),    /* 512 */
	ACCURACY                  = (1U << 10),   /* 1024 */
	BOUNCING                  = (1U << 11),   /* 2048 */
	GRIP                      = (1U << 12),   /* 4096 */
	FILTER_RESULT             = (1U << 13),   /* 8192 */
	QUICKCOVER                = (1U << 12),   /* 4096 */
	LPWG                      = (1U << 14),   /* 16384 */
	NOISE                     = (1U << 15),   /* 32768 */
	LPWG_COORDINATES          = (1U << 16),   /* 65536 */
};


#define TOUCH_I(fmt, args...)					\
	pr_info("[Touch_M] "					\
			fmt, ##args)

#define TOUCH_E(fmt, args...)					\
	pr_err("[Touch_M E] [%s %d] "				\
			fmt, __func__, __LINE__, ##args)

extern u32 touch_debug_mask;
#define TOUCH_D(condition, fmt, args...)			\
	do {							\
		if (unlikely(touch_debug_mask & (condition)))	\
			pr_info("[Touch_M] " fmt, ##args);	\
	} while (0)

#define TOUCH_DEBUG_SHOW_FILE
#ifdef TOUCH_DEBUG_SHOW_FILE
#define __SHORT_FILE__ (strrchr(__FILE__, '/') + 1)
#define TOUCH_TRACE()	TOUCH_D(TRACE, "- %s(%s) %d\n",		\
		__func__, __SHORT_FILE__, __LINE__)
#else
#define TOUCH_TRACE()	TOUCH_D(TRACE, "- %s %d", __func__, __LINE__)
#endif

struct touch_attribute {
	struct attribute attr;
	ssize_t (*show)(struct device *dev, char *buf);
	ssize_t (*store)(struct device *idev, const char *buf, size_t count);
};

#define TOUCH_ATTR(_name, _show, _store)		\
			struct touch_attribute touch_attr_##_name	\
			= __ATTR(_name, 0644, _show, _store)

static struct i2c_driver touch_module_driver;

static inline void write_file(struct device *dev, char *data, int write_time)
{
	struct module_data *md = to_module(dev);

	if (md->m_driver.write_file)
		md->m_driver.write_file(dev, data, write_time);
}

static inline void log_file_size_check(struct device *dev)
{
	struct module_data *md = to_module(dev);

	if (md->m_driver.log_file_size_check)
		md->m_driver.log_file_size_check(dev);
}

static inline int touch_check_boot_mode(struct device *dev)
{
	struct module_data *md = to_module(dev);
	int ret = TOUCH_NORMAL_BOOT;

	if(md->m_driver.touch_check_boot_mode)
		ret = md->m_driver.touch_check_boot_mode(dev);
	return ret;
}
