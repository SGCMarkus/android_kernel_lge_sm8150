/*
 * Copyright(c) 2019, LG Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LGE_COVER_CTRL_OPS_H_
#define _LGE_COVER_CTRL_OPS_H_

#define CONSECUTIVE_RECOVERY_TIME 10000

#define ICE40_BL_REG           0x0002
#define ICE40_BL_HBM_REG       0x0007
#define ICE40_DUTY_MAX         0x0008
#define ICE40_PANEL_ONOFF      0x0009
#define ICE40_BRIGHTNESS_DEBUG 0x000A

enum recovery_state {
	RECOVERY_NONE = 0,
	RECOVERY_POWERDROP_TIMEDOUT = 9,
	RECOVERY_POWERDROP_DETECTED = 10,
	RECOVERY_POWERDROP_BEGIN,
	RECOVERY_POWERDROP_DONE,
	RECOVERY_LTFAIL_TIMEDOUT = 19,
	RECOVERY_LTFAIL_DETECTED = 20,
	RECOVERY_LTFAIL_BEGIN,
	RECOVERY_LTFAIL_DONE,
	RECOVERY_MCU_TIMEDOUT = 29,
	RECOVERY_MCU_DETECTED = 30,
	RECOVERY_MCU_BEGIN,
	RECOVERY_MCU_DONE,

	RECOVERY_MAX,
};

extern struct lge_cover_ops *get_lge_cover_ops(void);
extern int init_cover_ctrl_ops(void);

struct lge_cover_ops {
	int (*set_recovery_state)(enum recovery_state state);
	enum recovery_state (*get_recovery_state)(void);
	void (*set_stream_preoff_state)(bool);
	bool (*get_stream_preoff_state)(void);
};

#endif // _LGE_COVER_CTRL_OPS_H_

