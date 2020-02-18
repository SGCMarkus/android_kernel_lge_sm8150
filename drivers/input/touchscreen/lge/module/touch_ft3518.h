#include "touch_common_module.h"

#define LGE_MODULE_DRIVER_NAME "FT3518_driver"
/* sync up to touch core */
#define LGE_TOUCH_MODULE_DRIVER_NAME "lge_touch_driver"
#define LGE_MODULE_IRQ_NAME "lge_touch_module"

/* test */
#define LOG_BUF_SIZE (100)

static struct i2c_driver touch_module_driver;

int module_get_dts_data(struct module_data *md);
void module_gpio_init(int pin, char *name);
void module_gpio_direction_output(int pin, int value);
void module_gpio_direction_input(int pin);
int module_power_init(struct module_data *md);
int module_bus_init(struct module_data *md, int buf_size);
int module_init_input(struct module_data *md);

irqreturn_t module_irq_handler(int irq, void *dev_id);
irqreturn_t module_irq_thread(int irq, void *dev_id);
void module_enable_irq(unsigned int irq);
void module_disable_irq(unsigned int irq);

int module_read(struct device *dev, u16 addr, void *data, int size);
int module_i2c_read(struct i2c_client *client, struct module_bus_msg *msg);
int module_write(struct device *dev, u16 addr, void *data, int size);
int module_i2c_write(struct i2c_client *client, struct module_bus_msg *msg);
static void touch_report_event(struct module_data *md);
void module_msleep(unsigned int msecs);

/*
 *         FT3518 DATA from here [Start]
 *
 *
 * */

#define FT8006M_ESD_SKIP_WHILE_TOUCH_ON
#define FTS_CTL_IIC
#define FTS_APK_DEBUG
#define FTS_SYSFS_DEBUG

/* Flexible Report */
#define REPORT_PACKET_SIZE		(12) // 12byte = sizeof(sw49106_touch_data)
#define REPORT_PACKET_BASE_COUNT	(2)
#define REPORT_PACKET_EXTRA_COUNT	(MAX_FINGER - REPORT_PACKET_BASE_COUNT)
#define REPORT_PACKET_BASE_DATA		(0x203)
#define REPORT_PACKET_EXTRA_DATA	(0x203 + ((REPORT_PACKET_SIZE/4) * REPORT_PACKET_BASE_COUNT))

struct ft3518_touch_debug {
	u32 padding[3];	/* packet structure 4+4+4+12*10+12+112 */
	u8 protocol_ver;
	u8 reserved_1;
	u32 frame_cnt;
	u8 rn_max_bfl;
	u8 rn_max_afl;
	u8 rn_min_bfl;
	u8 rn_min_afl;
	u8 rn_max_afl_x;
	u8 rn_max_afl_y;
	s8 lf_oft[18];
	u8 seg1_cnt:4;
	u8 seg2_cnt:4;
	u8 seg1_thr;
	u8 rebase[8];
	u8 rn_pos_cnt;
	u8 rn_neg_cnt;
	u8 rn_pos_sum;
	u8 rn_neg_sum;
	u8 rn_stable;
	u8 track_bit[10];
	u8 rn_max_tobj[12];
	u8 palm[8];
	u8 noise_detect[8];
	u8 reserved_2[21];
	u32 ic_debug[3];
	u32 ic_debug_info;
} __packed;

/* report packet */
struct ft3518_touch_data {
	u8 xh;
	u8 xl;
	u8 yh;
	u8 yl;
	u8 weight;
	u8 area;
} __packed;


struct ft3518_touch_info {
	u32 ic_status;
	u32 device_status;
	u32 wakeup_type:8;
	u32 touch_cnt:5;
	u32 button_cnt:3;
	u32 palm_bit:16;
	struct ft3518_touch_data data[10];
	/* debug info */
	struct ft3518_touch_debug debug;
} __packed;

/* Definitions for FTS */
#if 1    //  JASON_TF10J
#define FTS_PACKET_LENGTH	128
#else
#define FTS_PACKET_LENGTH	32
#endif

#define MAX_TAP_COUNT		12

#define FT5X06_ID		0x55
#define FT5X16_ID		0x0A
#define FT5X36_ID		0x14
#define FT6X06_ID		0x06
#define FT6X36_ID		0x36

#define FT5316_ID		0x0A
#define FT5306I_ID		0x55

#define LEN_FLASH_ECC_MAX	0xFFFE

#define FTS_MAX_POINTS		10

#define FTS_WORKQUEUE_NAME	"fts_wq"

#define FTS_DEBUG_DIR_NAME	"fts_debug"

#define FTS_INFO_MAX_LEN	512
#define FTS_FW_NAME_MAX_LEN	50

#define FTS_REG_ID			0xA3	// Chip Selecting (High)
#define FTS_REG_ID_LOW			0x9F	// Chip Selecting (Low)
#define FTS_REG_FW_VER			0xA6	// Firmware Version (Major)
#define FTS_REG_FW_VENDOR_ID		0xA8	// Focaltech's Panel ID
#define FTS_REG_LIB_VER_H		0xA1	// LIB Version Info (High)
#define FTS_REG_LIB_VER_L		0xA2	// LIB Version Info (Low)
#define FTS_REG_FW_VER_MINOR		0xB2	// Firmware Version (Minor)
#define FTS_REG_FW_VER_SUB_MINOR	0xB3	// Firmware Version (Sub-Minor)

#define FTS_REG_POINT_RATE		0x88

#define FTS_FACTORYMODE_VALUE		0x40
#define FTS_WORKMODE_VALUE		0x00

#define FTS_META_REGS			3
#define FTS_ONE_TCH_LEN			6
#define FTS_TCH_LEN(x)			(FTS_META_REGS + FTS_ONE_TCH_LEN * x)

#define FTS_PRESS			0x7F
#define FTS_MAX_ID			0x0F
#define FTS_TOUCH_P_NUM			2
#define FTS_TOUCH_X_H_POS		3
#define FTS_TOUCH_X_L_POS		4
#define FTS_TOUCH_Y_H_POS		5
#define FTS_TOUCH_Y_L_POS		6
#define FTS_TOUCH_PRE_POS		7
#define FTS_TOUCH_AREA_POS		8
#define FTS_TOUCH_POINT_NUM		2
#define FTS_TOUCH_EVENT_POS		3
#define FTS_TOUCH_ID_POS		5

#define FTS_TOUCH_DOWN			0
#define FTS_TOUCH_UP			1
#define FTS_TOUCH_CONTACT		2

#define POINT_READ_BUF			(3 + FTS_ONE_TCH_LEN * FTS_MAX_POINTS)

/* charger status */
#define SPR_CHARGER_STS			(0x8b)
#define SPR_QUICKCOVER_STS		(0xC1)

#define CONNECT_NONE			(0x00)
#define CONNECT_USB			(0x01)
#define CONNECT_TA			(0x02)
#define CONNECT_OTG			(0x03)
#define CONNECT_WIRELESS		(0x10)


enum {
	SW_RESET = 0,
	HW_RESET_ASYNC,
	HW_RESET_SYNC,
};

enum {
	TOUCHSTS_IDLE = 0,
	TOUCHSTS_DOWN,
	TOUCHSTS_MOVE,
	TOUCHSTS_UP,
};

enum {
	ABS_MODE = 0,
	KNOCK_1,
	KNOCK_2,
	SWIPE_UP,
	SWIPE_DOWN,
	SWIPE_RIGHT,
	SWIPE_LEFT,
	CUSTOM_DEBUG = 200,
	KNOCK_OVERTAP = 201,
};

enum {
	LCD_MODE_U0 = 0,
	LCD_MODE_U2_UNBLANK,
	LCD_MODE_U2,
	LCD_MODE_U3,
	LCD_MODE_U3_PARTIAL,
	LCD_MODE_U3_QUICKCOVER,
	LCD_MODE_STOP,
};

enum {
	SWIPE_R = 0,
	SWIPE_D,
	SWIPE_L,
	SWIPE_U
};

enum {
//	SWIPE_RIGHT_BIT	= 1,
//	SWIPE_DOWN_BIT = 1 << 8,
//	SWIPE_LEFT_BIT	= 1 << 16,
	SWIPE_UP_BIT = 1,
};

/* swipe */
enum {
	SWIPE_ENABLE_CTRL = 0,
	SWIPE_DISABLE_CTRL,
	SWIPE_DIST_CTRL,
	SWIPE_RATIO_THR_CTRL,
	SWIPE_RATIO_PERIOD_CTRL,
	SWIPE_RATIO_DIST_CTRL,
	SWIPE_TIME_MIN_CTRL,
	SWIPE_TIME_MAX_CTRL,
	SWIPE_AREA_CTRL,
};

enum {
	IC_INIT_NEED = 0,
	IC_INIT_DONE,
};

enum {
	REV_0 = 0,
	REV_6,     /* revision 6   */
	REV_8 = 8, /* revision 7/8 */
	REV_9,     /* revision 9   */
};

enum {
	TCI_CTRL_SET = 0,
	TCI_CTRL_CONFIG_COMMON,
	TCI_CTRL_CONFIG_TCI_1,
	TCI_CTRL_CONFIG_TCI_2,
};

enum {
	TC_STATE_ACTIVE = 0,
	TC_STATE_LPWG,
	TC_STATE_DEEP_SLEEP,
	TC_STATE_POWER_OFF,
};

/* test control */
struct ft3518_version {
	u8 major;
	u8 minor;
	u8 sub_minor;
};

struct ft3518_ic_info {
	struct ft3518_version version;
	u8 info_valid;
	u8 chip_id;
	u8 chip_id_low;
	u8 is_official;
	u8 fw_version;
	u8 fw_vendor_id;
	u8 lib_version_high;
	u8 lib_version_low;
	u8 is_official_bin;
	u8 fw_version_bin;
};

struct ft3518_data {
	struct device *dev;
	struct kobject kobj;
	struct ft3518_touch_info info;
	struct ft3518_ic_info ic_info;
//	struct ft3518_asc_info asc;	/* ASC */
	struct workqueue_struct *wq_fts;
	u8 lcd_mode;
	u8 prev_lcd_mode;
	u8 driving_mode;
	u8 u3fake;
//	struct watch_data watch;
//	struct swipe_ctrl swipe;
//	struct mutex spi_lock;
	struct mutex rw_lock;
	struct mutex fb_lock;
	struct delayed_work font_download_work;
	struct delayed_work fb_notify_work;
	struct delayed_work debug_work;
	u8 charger;
	u32 earjack;
	u32 frame_cnt;
	u8 tci_debug_type;
//	u8 swipe_debug_type;
	atomic_t block_watch_cfg;
	atomic_t init;
	u8 state;
	u8 chip_rev;
	u8 en_i2c_lpwg;
	struct notifier_block fb_notif;

	struct bin_attribute prd_delta_attr;
	u8 *prd_delta_data;
	struct bin_attribute prd_rawdata_attr;
	u8 *prd_rawdata_data;

	int debug_test_cnt;
};

#ifdef CONFIG_LGD_FT8006M_FHD_VIDEO_LCD_PANEL

extern int tianma_ft860x_hd_video_panel_external_api(int type, int enable);
#define LCD_RESET_H	tianma_ft860x_hd_video_panel_external_api(2, 1)
#define LCD_RESET_L	tianma_ft860x_hd_video_panel_external_api(2, 0)
#define LCD_DSV_ON	tianma_ft860x_hd_video_panel_external_api(1, 1)
#define LCD_DSV_OFF	tianma_ft860x_hd_video_panel_external_api(1, 0)
#define LCD_VDDI_ON	tianma_ft860x_hd_video_panel_external_api(0, 1)
#define LCD_VDDI_OFF	tianma_ft860x_hd_video_panel_external_api(0, 0)

#else
#define LCD_RESET_H
#define LCD_RESET_L
#define LCD_DSV_ON
#define LCD_DSV_OFF
#define LCD_VDDI_ON
#define LCD_VDDI_OFF
#endif

#define TCI_MAX_NUM		2
#define SWIPE_MAX_NUM		2
#define TCI_DEBUG_MAX_NUM	16
#define SWIPE_DEBUG_MAX_NUM	8
#define DISTANCE_INTER_TAP	(0x1 << 1) /* 2 */
#define DISTANCE_TOUCHSLOP	(0x1 << 2) /* 4 */
#define TIMEOUT_INTER_TAP_LONG	(0x1 << 3) /* 8 */
#define MULTI_FINGER		(0x1 << 4) /* 16 */
#define DELAY_TIME		(0x1 << 5) /* 32 */
#define PALM_STATE		(0x1 << 6) /* 64 */
#define OUTOF_AREA		(0x1 << 7) /* 128 */

#define TCI_DEBUG_ALL (DISTANCE_INTER_TAP | DISTANCE_TOUCHSLOP |\
		TIMEOUT_INTER_TAP_LONG | MULTI_FINGER | DELAY_TIME |\
		PALM_STATE | OUTOF_AREA)

int ft3518_reg_read(struct device *dev, u16 addr, void *data, int size);
int ft3518_reg_write(struct device *dev, u16 addr, void *data, int size);
int ft3518_ic_info(struct device *dev);
int ft3518_tc_driving(struct device *dev, int mode);
int ft3518_irq_abs(struct device *dev);
//int sw49106_irq_lpwg(struct device *dev);
int ft3518_irq_lpwg(struct device *dev, int tci);
int ft3518_irq_handler(struct device *dev);
int ft3518_check_status(struct device *dev);
//int sw49106_debug_info(struct device *dev);
int ft3518_debug_info(struct device *dev, int mode);
//int sw49106_reset_ctrl(struct device *dev, int ctrl);

int ft3518_report_tci_fr_buffer(struct device *dev);
int ft3518_chipid_info(void);

static inline int ft3518_read_value(struct device *dev,
					u16 addr, u32 *value)
{
	return ft3518_reg_read(dev, addr, value, sizeof(*value));
}

static inline int ft3518_write_value(struct device *dev,
					 u16 addr, u32 value)
{
	return ft3518_reg_write(dev, addr, &value, sizeof(value));
}
static inline struct ft3518_data *to_ft3518_data(struct device *dev)
{
	return (struct ft3518_data *)module_get_device(to_module(dev));
}
/*
 *         SW49106 DATA from here [END]
 *
*/

int module_initialize(struct device *dev);
void module_interrupt_control(struct device *dev, int on_off);
