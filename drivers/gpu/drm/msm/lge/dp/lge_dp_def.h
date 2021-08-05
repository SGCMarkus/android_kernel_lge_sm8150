#ifndef _H_LGE_DP_DEF_
#define _H_LGE_DP_DEF_
#include <linux/module.h>
#include <asm/atomic.h>
#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
#include "../cover/lge_cover_ctrl_ops.h"
#include <linux/mutex.h>
#endif

#if IS_ENABLED(CONFIG_LGE_DUAL_SCREEN)
#include <linux/extcon.h>
#define EXT_DD_MAX_COUNT 3
#endif

struct lge_dp_display {
	bool force_disconnection;
	int hpd_state;
	int skip_uevent;
	int dd_button_state;
	int led_status;
	atomic_t pending;
	int pending_status;
	atomic_t dd_5v_power_state;
	int block_state;
	unsigned int vid_pid;
#if defined(CONFIG_LGE_COVER_DISPLAY) || defined(CONFIG_LGE_DUAL_SCREEN)
	struct lge_cover_ops *cover_ops;
	struct mutex cover_lock;
#if defined(CONFIG_LGE_DUAL_SCREEN)
	struct mutex cd_state_lock;
	struct extcon_dev *dd_extcon_sdev[EXT_DD_MAX_COUNT];
#endif
#endif

};
#endif //_H_LGE_DP_DEF_
