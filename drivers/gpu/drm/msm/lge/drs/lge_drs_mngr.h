#ifndef _LGE_DRS_MNGR_H_
#define _LGE_DRS_MNGR_H_

#include <linux/delay.h>
#include <linux/mutex.h>
#include <video/mipi_display.h>

#define DRS_MAX_RETRY 5
#define DRS_TIMEOUT 1000

enum lge_drs_state {
	DRS_NONE = 0,
	DRS_IDLE,
	DRS_RUNNING,
};

enum lge_drs_result {
	DRS_SUCCESS = 1,
	DRS_FAIL,
};

enum lge_drs_request {
	DRS_NOT_READY = -1,
	DRS_UNFREEZE = 0,
	DRS_FREEZE,
	DRS_TIMEDOUT,
	DRS_REQUEST_MAX,
};

struct drs_res_info {
	char resolution[5];
	int data;
	u32 width;
	u32 height;
};

struct dsi_panel;
struct dsi_display;
struct lge_drs_mngr {
	struct dsi_display *main_display;
	enum lge_drs_state drs_state;
	u8 drs_requested;
	u8 keep_bist_on;
	bool is_updated;
	int num_res;
	u32 request_res;
	u32 current_res;
	int current_freeze_state;
	struct drs_res_info supported_res[3];
	struct mutex drs_lock;
	struct workqueue_struct *drs_workq;
	struct delayed_work drs_work;
	struct completion drs_done;
	struct completion drs_work_done; /* to avoid race condition btw drs and blank */
};

extern bool lge_drs_mngr_is_enabled(struct dsi_panel *panel);
extern int lge_drs_mngr_begin(struct dsi_panel *panel);
extern int lge_drs_mngr_finish(struct dsi_panel *panel);
extern int lge_drs_mngr_get_state(struct dsi_panel *panel);
extern int lge_drs_mngr_set_freeze_state(enum lge_drs_request state);
extern int lge_drs_mngr_get_freeze_state(void);
#endif // _LGE_DRS_MNGR_H_
