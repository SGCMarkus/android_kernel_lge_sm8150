#ifndef _H_LGE_DP_DEF_
#define _H_LGE_DP_DEF_
#include <linux/module.h>
#include <asm/atomic.h>
#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
#include "../cover/lge_cover_ctrl_ops.h"
#include <linux/mutex.h>
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
	unsigned int vid_pid;
#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY)
	struct lge_cover_ops *cover_ops;
	struct mutex cover_lock;
#endif

};
#endif //_H_LGE_DP_DEF_
