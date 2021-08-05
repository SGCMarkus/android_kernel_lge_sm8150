#ifndef __LINUX_CLASS_DUAL_ROLE_H__
#define __LINUX_CLASS_DUAL_ROLE_H__

#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/types.h>

struct device;

enum dual_role_supported_modes {
	DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP = 0,
	DUAL_ROLE_SUPPORTED_MODES_DFP,
	DUAL_ROLE_SUPPORTED_MODES_UFP,
/*The following should be the last element*/
	DUAL_ROLE_PROP_SUPPORTED_MODES_TOTAL,
};

enum {
	DUAL_ROLE_PROP_MODE_UFP = 0,
	DUAL_ROLE_PROP_MODE_DFP,
#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
	DUAL_ROLE_PROP_MODE_FAULT,
	DUAL_ROLE_PROP_MODE_FAULT_NO_UX,
	DUAL_ROLE_PROP_MODE_FLOAT,
#endif
	DUAL_ROLE_PROP_MODE_NONE,
/*The following should be the last element*/
	DUAL_ROLE_PROP_MODE_TOTAL,
};

enum {
	DUAL_ROLE_PROP_PR_SRC = 0,
	DUAL_ROLE_PROP_PR_SNK,
#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
	DUAL_ROLE_PROP_PR_FAULT,
	DUAL_ROLE_PROP_PR_FAULT_NO_UX,
	DUAL_ROLE_PROP_PR_FLOAT,
#endif
	DUAL_ROLE_PROP_PR_NONE,
/*The following should be the last element*/
	DUAL_ROLE_PROP_PR_TOTAL,

};

enum {
	DUAL_ROLE_PROP_DR_HOST = 0,
	DUAL_ROLE_PROP_DR_DEVICE,
#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
	DUAL_ROLE_PROP_DR_FAULT,
	DUAL_ROLE_PROP_DR_FAULT_NO_UX,
	DUAL_ROLE_PROP_DR_FLOAT,
#endif
	DUAL_ROLE_PROP_DR_NONE,
/*The following should be the last element*/
	DUAL_ROLE_PROP_DR_TOTAL,
};

enum {
	DUAL_ROLE_PROP_VCONN_SUPPLY_NO = 0,
	DUAL_ROLE_PROP_VCONN_SUPPLY_YES,
/*The following should be the last element*/
	DUAL_ROLE_PROP_VCONN_SUPPLY_TOTAL,
};

#ifdef CONFIG_LGE_USB
enum {
	DUAL_ROLE_PROP_CONTROL_AUTO = 0,
	DUAL_ROLE_PROP_CONTROL_ON,
/*The following should be the last element*/
	DUAL_ROLE_PROP_CONTROL_TOTAL,
};

enum {
	DUAL_ROLE_PROP_CC_OPEN = 0,
	DUAL_ROLE_PROP_CC_RP_DEFAULT,
	DUAL_ROLE_PROP_CC_RP_POWER1P5,
	DUAL_ROLE_PROP_CC_RP_POWER3P0,
	DUAL_ROLE_PROP_CC_RD,
	DUAL_ROLE_PROP_CC_RA,
/*The following should be the last element*/
	DUAL_ROLE_PROP_CC_TOTAL,
};

enum {
	DUAL_ROLE_PROP_PD_REV_10,
	DUAL_ROLE_PROP_PD_REV_20,
	DUAL_ROLE_PROP_PD_REV_30,
	DUAL_ROLE_PROP_PD_REV_RESERVED,
	DUAL_ROLE_PROP_PD_REV_NOT_SUPPORTED,
	DUAL_ROLE_PROP_PD_REV_NONE,
/*The following should be the last element*/
	DUAL_ROLE_PROP_PD_REV_TOTAL,
};
#endif

#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
enum dual_role_prop_moisture_en {
	DUAL_ROLE_PROP_MOISTURE_EN_ENABLE = 0,
	DUAL_ROLE_PROP_MOISTURE_EN_DISABLE,
	DUAL_ROLE_PROP_MOISTURE_EN_TOTAL,
};

enum dual_role_prop_moisture_ux {
	DUAL_ROLE_PROP_MOISTURE_UX_ENABLE = 0,
	DUAL_ROLE_PROP_MOISTURE_UX_DISABLE,
	DUAL_ROLE_PROP_MOISTURE_UX_TOTAL,
};

enum dual_role_prop_moisture_usb{
	DUAL_ROLE_PROP_MOISTURE_USB_ENABLE = 0,
	DUAL_ROLE_PROP_MOISTURE_USB_DISABLE,
	DUAL_ROLE_PROP_MOISTURE_USB_TOTAL,
};
#endif

enum dual_role_property {
	DUAL_ROLE_PROP_SUPPORTED_MODES = 0,
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
	DUAL_ROLE_PROP_VCONN_SUPPLY,
#ifdef CONFIG_LGE_USB
	DUAL_ROLE_PROP_CONTROL,
	DUAL_ROLE_PROP_CC1,
	DUAL_ROLE_PROP_CC2,
	DUAL_ROLE_PROP_PD_REV,
	DUAL_ROLE_PROP_PDO1,
	DUAL_ROLE_PROP_PDO2,
	DUAL_ROLE_PROP_PDO3,
	DUAL_ROLE_PROP_PDO4,
	DUAL_ROLE_PROP_PDO5,
	DUAL_ROLE_PROP_PDO6,
	DUAL_ROLE_PROP_PDO7,
	DUAL_ROLE_PROP_RDO,
#endif
#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
	DUAL_ROLE_PROP_MOISTURE_EN,
	DUAL_ROLE_PROP_MOISTURE_UX,
	DUAL_ROLE_PROP_MOISTURE_USB,
#endif
};

#ifdef CONFIG_LGE_USB
/* PDO */
enum {
	DUAL_ROLE_PROP_PDO_TYPE_FIXED,
	DUAL_ROLE_PROP_PDO_TYPE_BATTERY,
	DUAL_ROLE_PROP_PDO_TYPE_VARIABLE,
	DUAL_ROLE_PROP_PDO_TYPE_AUGMENTED,
};

#define DUAL_ROLE_PROP_PDO_SET_TYPE(type)	(((type) & 0x3) << 30)

#define DUAL_ROLE_PROP_PDO_SET_FIXED_VOLT(mV)	((((mV) / 50U) & 0x3FF) << 10)
#define DUAL_ROLE_PROP_PDO_SET_FIXED_CURR(mA)	(((mA) / 10U) & 0x3FF)

#define DUAL_ROLE_PROP_PDO_SET_BATTERY_MAX_VOLT(mV)  ((((mV) / 50U) & 0x3FF) << 20)
#define DUAL_ROLE_PROP_PDO_SET_BATTERY_MIN_VOLT(mV)  ((((mV) / 50U) & 0x3FF) << 10)
#define DUAL_ROLE_PROP_PDO_SET_BATTERY_MAX_POWER(mW) (((mV) / 250U) & 0x3FF)

#define DUAL_ROLE_PROP_PDO_SET_VARIABLE_MAX_VOLT(mV) ((((mV) / 50U) & 0x3FF) << 20)
#define DUAL_ROLE_PROP_PDO_SET_VARIABLE_MIN_VOLT(mV) ((((mV) / 50U) & 0x3FF) << 10)
#define DUAL_ROLE_PROP_PDO_SET_VARIABLE_MAX_CURR(mW) (((mW) / 10U) & 0x3FF)

#define DUAL_ROLE_PROP_PDO_SET_AUGMENTED_MAX_VOLT(mV) ((((mV) / 100U) & 0xFF) << 17)
#define DUAL_ROLE_PROP_PDO_SET_AUGMENTED_MIN_VOLT(mV) ((((mV) / 100U) & 0xFF) << 8)
#define DUAL_ROLE_PROP_PDO_SET_AUGMENTED_MAX_CURR(mW) (((mW) / 50U) & 0x7F)

#define DUAL_ROLE_PROP_PDO_GET_TYPE(pdo)	(((pdo) >> 30) & 0x3)

#define DUAL_ROLE_PROP_PDO_GET_FIXED_VOLT(pdo)	((((pdo) >> 10) & 0x3FF) * 50U)
#define DUAL_ROLE_PROP_PDO_GET_FIXED_CURR(pdo)	(((pdo) & 0x3FF) * 10U)

#define DUAL_ROLE_PROP_PDO_GET_BATTERY_MAX_VOLT(pdo)  ((((pdo) >> 20) & 0x3FF) * 50U)
#define DUAL_ROLE_PROP_PDO_GET_BATTERY_MIN_VOLT(pdo)  ((((pdo) >> 10) & 0x3FF) * 50U)
#define DUAL_ROLE_PROP_PDO_GET_BATTERY_MAX_POWER(pdo) (((pdo) & 0x3FF) * 250U)

#define DUAL_ROLE_PROP_PDO_GET_VARIABLE_MAX_VOLT(pdo) ((((pdo) >> 20) & 0x3FF) * 50U)
#define DUAL_ROLE_PROP_PDO_GET_VARIABLE_MIN_VOLT(pdo) ((((pdo) >> 10) & 0x3FF) * 50U)
#define DUAL_ROLE_PROP_PDO_GET_VARIABLE_MAX_CURR(pdo) (((pdo) & 0x3FF) * 10U)

#define DUAL_ROLE_PROP_PDO_GET_AUGMENTED_MAX_VOLT(pdo)  ((((pdo) >> 17) & 0xFF) * 100U)
#define DUAL_ROLE_PROP_PDO_GET_AUGMENTED_MIN_VOLT(pdo)  ((((pdo) >> 8) & 0xFF) * 100U)
#define DUAL_ROLE_PROP_PDO_GET_AUGMENTED_MAX_CURR(pdo) (((pdo) & 0x7F) * 50U)

/* RDO */
#define DUAL_ROLE_PROP_RDO_SET_OBJ_POS(pos)	(((pos) & 0x7) << 28)

#define DUAL_ROLE_PROP_RDO_SET_FIXED_OP_CURR(mA)	((((mA) / 10U) & 0x3FF) << 10)
#define DUAL_ROLE_PROP_RDO_SET_FIXED_MINMAX_CURR(mA)	((((mA) / 10U) & 0x3FF)

#define DUAL_ROLE_PROP_RDO_SET_BATTERY_OP_POWER(mW)	((((mW) / 250U) & 0x3FF) << 10)
#define DUAL_ROLE_PROP_RDO_SET_BATTERY_MIN_POWER(mW)	((((mW) / 250U) & 0x3FF)

#define DUAL_ROLE_PROP_RDO_SET_VARIABLE_OP_CURR(mA)	((((mA) / 10U) & 0x3FF) << 10)
#define DUAL_ROLE_PROP_RDO_SET_VARIABLE_MINMAX_CURR(mA)	((((mA) / 10U) & 0x3FF)

#define DUAL_ROLE_PROP_RDO_SET_AUGMENTED_VOLT(mA)	((((mA) / 20U) & 0x7FF) << 9)
#define DUAL_ROLE_PROP_RDO_SET_AUGMENTED_CURR(mA)	((((mA) / 50U) & 0x7F)

#define DUAL_ROLE_PROP_RDO_GET_OBJ_POS(rdo)	(((rdo) >> 28) & 0x7)

#define DUAL_ROLE_PROP_RDO_GET_FIXED_OP_CURR(rdo)	((((rdo) >> 10) & 0x3FF) * 10U)
#define DUAL_ROLE_PROP_RDO_GET_FIXED_MINMAX_CURR(rdo)	(((rdo) & 0x3FF) * 10U)

#define DUAL_ROLE_PROP_RDO_GET_BATTERY_OP_POWER(rdo)	((((rdo) >> 10) & 0x3FF) * 250U)
#define DUAL_ROLE_PROP_RDO_GET_BATTERY_MIN_POWER(rdo)	(((rdo) & 0x3FF) * 250U)

#define DUAL_ROLE_PROP_RDO_GET_VARIABLE_OP_CURR(rdo)	((((rdo) >> 10) & 0x3FF) * 10U)
#define DUAL_ROLE_PROP_RDO_GET_VARIABLE_MINMAX_CURR(rdo)	(((rdo) & 0x3FF) * 10U)

#define DUAL_ROLE_PROP_RDO_GET_AUGMENTED_VOLT(rdo)	((((rdo) >> 9) & 0x7FF) * 20U)
#define DUAL_ROLE_PROP_RDO_GET_AUGMENTED_CURR(rdo)	(((rdo) & 0x7F) * 50U)
#endif

struct dual_role_phy_instance;

/* Description of typec port */
struct dual_role_phy_desc {
	/* /sys/class/dual_role_usb/<name>/ */
	const char *name;
	enum dual_role_supported_modes supported_modes;
	enum dual_role_property *properties;
	size_t num_properties;

	/* Callback for "cat /sys/class/dual_role_usb/<name>/<property>" */
	int (*get_property)(struct dual_role_phy_instance *dual_role,
			     enum dual_role_property prop,
			     unsigned int *val);
	/* Callback for "echo <value> >
	 *                      /sys/class/dual_role_usb/<name>/<property>" */
	int (*set_property)(struct dual_role_phy_instance *dual_role,
			     enum dual_role_property prop,
			     const unsigned int *val);
	/* Decides whether userspace can change a specific property */
	int (*property_is_writeable)(struct dual_role_phy_instance *dual_role,
				      enum dual_role_property prop);
};

struct dual_role_phy_instance {
	const struct dual_role_phy_desc *desc;

	/* Driver private data */
	void *drv_data;

	struct device dev;
	struct work_struct changed_work;
};

#if IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)
extern void dual_role_instance_changed(struct dual_role_phy_instance
				       *dual_role);
extern struct dual_role_phy_instance *__must_check
devm_dual_role_instance_register(struct device *parent,
				 const struct dual_role_phy_desc *desc);
extern void devm_dual_role_instance_unregister(struct device *dev,
					       struct dual_role_phy_instance
					       *dual_role);
extern int dual_role_get_property(struct dual_role_phy_instance *dual_role,
				  enum dual_role_property prop,
				  unsigned int *val);
extern int dual_role_set_property(struct dual_role_phy_instance *dual_role,
				  enum dual_role_property prop,
				  const unsigned int *val);
extern int dual_role_property_is_writeable(struct dual_role_phy_instance
					   *dual_role,
					   enum dual_role_property prop);
extern void *dual_role_get_drvdata(struct dual_role_phy_instance *dual_role);
#else /* CONFIG_DUAL_ROLE_USB_INTF */
static inline void dual_role_instance_changed(struct dual_role_phy_instance
				       *dual_role){}
static inline struct dual_role_phy_instance *__must_check
devm_dual_role_instance_register(struct device *parent,
				 const struct dual_role_phy_desc *desc)
{
	return ERR_PTR(-ENOSYS);
}
static inline void devm_dual_role_instance_unregister(struct device *dev,
					       struct dual_role_phy_instance
					       *dual_role){}
static inline void *dual_role_get_drvdata(struct dual_role_phy_instance
		*dual_role)
{
	return ERR_PTR(-ENOSYS);
}
#endif /* CONFIG_DUAL_ROLE_USB_INTF */
#endif /* __LINUX_CLASS_DUAL_ROLE_H__ */
