/*
 * LGE CC/SBU Protection Switch driver
 *
 * Copyright (C) 2018 LG Electronics, Inc.
 * Author: Hansun Lee <hansun.lee@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __LGE_SBU_SWITCH_H__
#define __LGE_SBU_SWITCH_H__

struct device;

/*
 * LGE_SBU_SWITCH modes & flags
 *
 * Must be declared in priority order. Small value have a high priority.
 * That is, 0 is the highest priority.
 */
enum lge_sbu_switch_modes {
	LGE_SBU_SWITCH_MODE_SBU_DISABLE = 0,
	LGE_SBU_SWITCH_MODE_SBU_MD,
	LGE_SBU_SWITCH_MODE_SBU_FACTORY_ID,
	LGE_SBU_SWITCH_MODE_SBU_MD_ING,
	LGE_SBU_SWITCH_MODE_EDGE_MD,
	LGE_SBU_SWITCH_MODE_SBU_AUX,
	LGE_SBU_SWITCH_MODE_SBU_UART,
	LGE_SBU_SWITCH_MODE_SBU_USBID,
	LGE_SBU_SWITCH_MODE_EDGE_MD_ING,
	LGE_SBU_SWITCH_MODE_SBU_IDLE,
	LGE_SBU_SWITCH_MODE_MAX,
};

#define LGE_SBU_SWITCH_FLAG_SBU_DISABLE		BIT(LGE_SBU_SWITCH_MODE_SBU_DISABLE)
#define LGE_SBU_SWITCH_FLAG_SBU_MD		BIT(LGE_SBU_SWITCH_MODE_SBU_MD)
#define LGE_SBU_SWITCH_FLAG_SBU_FACTORY_ID	BIT(LGE_SBU_SWITCH_MODE_SBU_FACTORY_ID)
#define LGE_SBU_SWITCH_FLAG_SBU_MD_ING		BIT(LGE_SBU_SWITCH_MODE_SBU_MD_ING)
#define LGE_SBU_SWITCH_FLAG_EDGE_MD		BIT(LGE_SBU_SWITCH_MODE_EDGE_MD)
#define LGE_SBU_SWITCH_FLAG_SBU_AUX		BIT(LGE_SBU_SWITCH_MODE_SBU_AUX)
#define LGE_SBU_SWITCH_FLAG_SBU_UART		BIT(LGE_SBU_SWITCH_MODE_SBU_UART)
#define LGE_SBU_SWITCH_FLAG_SBU_USBID		BIT(LGE_SBU_SWITCH_MODE_SBU_USBID)
#define LGE_SBU_SWITCH_FLAG_EDGE_MD_ING		BIT(LGE_SBU_SWITCH_MODE_EDGE_MD_ING)
#define LGE_SBU_SWITCH_FLAG_SBU_IDLE		BIT(LGE_SBU_SWITCH_MODE_SBU_IDLE)
#define LGE_SBU_SWITCH_FLAG_MAX			BIT(LGE_SBU_SWITCH_MODE_MAX)

struct lge_sbu_switch_instance;

struct lge_sbu_switch_desc {
	unsigned long flags;
	void (*ovp_callback)(struct lge_sbu_switch_instance *inst, int ovp);
};

struct lge_sbu_switch_instance {
	struct device			*dev;
	const struct lge_sbu_switch_desc	*desc;
	unsigned long			flags;
	struct list_head		list;

	/* Driver private data */
	void				*drv_data;
};

int lge_sbu_switch_get(struct lge_sbu_switch_instance *inst, unsigned long flag);
int lge_sbu_switch_put(struct lge_sbu_switch_instance *inst, unsigned long flag);
unsigned long lge_sbu_switch_get_current_flag(struct lge_sbu_switch_instance *inst);
int lge_sbu_switch_get_ovp_state(struct lge_sbu_switch_instance *inst);

struct lge_sbu_switch_instance *__must_check
devm_lge_sbu_switch_instance_register(struct device *dev,
			       const struct lge_sbu_switch_desc *desc);
void devm_lge_sbu_switch_instance_unregister(struct device *dev,
				      struct lge_sbu_switch_instance *inst);
#endif /* __LGE_SBU_SWITCH_H__ */
