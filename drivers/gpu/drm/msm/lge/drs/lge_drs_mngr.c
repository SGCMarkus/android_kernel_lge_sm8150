
#define pr_fmt(fmt)	"[Display][lge-drs:%s:%d] " fmt, __func__, __LINE__
#include <linux/device.h>
#include <linux/mutex.h>
#include "lge_drs_mngr.h"
#include "dsi_panel.h"
#include "dsi_drm.h"
#include "drm_connector.h"
#include "../lge_ddic_ops_helper.h"

static struct lge_drs_mngr *gdrs_mngr = NULL;
extern int dsi_panel_tx_cmd_set(struct dsi_panel *panel, enum dsi_cmd_set_type type);

static struct lge_drs_mngr* lge_drs_mngr_get_and_validate(struct dsi_panel *panel)
{
	struct lge_drs_mngr *drs_mngr = NULL;

	if (!panel)
		return NULL;

	drs_mngr = &panel->lge.drs_mngr;
	if (!drs_mngr)
		pr_err("drs mngr is null\n");

	if (drs_mngr->drs_state == DRS_NONE)
		pr_err("drs mngr is not initialized\n");

	return drs_mngr;
}

static struct dsi_panel* lge_drs_mngr_dm_to_panel(struct lge_drs_mngr *dm)
{
	struct lge_dsi_panel *lge_panel = NULL;
	struct dsi_panel *panel = NULL;

	lge_panel = container_of(dm, struct lge_dsi_panel, drs_mngr);
	if (!lge_panel) {
		pr_err("lge panel is null\n");
		return panel;
	}

	panel = container_of(lge_panel, struct dsi_panel, lge);
	if (!panel)
		pr_err("panel is null\n");

	return panel;
}

static struct drm_connector* lge_drs_mngr_dm_to_conn(struct lge_drs_mngr *dm)
{
	struct drm_connector *conn = NULL;

	if (!dm || !dm->main_display)
		return NULL;

	conn = dm->main_display->drm_conn;
	if (!conn) {
		return NULL;
	}

	return conn;
}

static struct drm_device* lge_drs_mngr_dm_to_dev(struct lge_drs_mngr *dm)
{
	struct drm_device *dev = NULL;

	if (!dm || !dm->main_display)
		return NULL;

	dev = dm->main_display->drm_dev;
	if (!dev) {
		return NULL;
	}

	return dev;
}

static u32 lge_drs_mngr_convert_idx_to_res(struct lge_drs_mngr *drs_mngr, int idx)
{
	int i;

	for (i = 0; i < drs_mngr->num_res; i++) {
		if (drs_mngr->supported_res[i].data == idx) {
			return drs_mngr->supported_res[i].height;
		}
	}

	return 0;
}

static bool lge_drs_mngr_is_updated(struct dsi_panel *panel)
{
	int idx;
	struct lge_drs_mngr *drs_mngr = &panel->lge.drs_mngr;

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->get_current_res) {
		idx = panel->lge.ddic_ops->get_current_res(panel);
	} else {
		pr_debug("dic scaler is not supported\n");
		return true;
	}

	drs_mngr->current_res = lge_drs_mngr_convert_idx_to_res(drs_mngr, idx);

	drs_mngr->is_updated =
		((drs_mngr->request_res == drs_mngr->current_res) ? true : false);

	pr_info("result=%s, request_res=%d, current_res=%d\n",
				(drs_mngr->is_updated ? "success" : "failure"),
				drs_mngr->request_res,
				drs_mngr->current_res);

	return drs_mngr->is_updated;
}

static int lge_drs_mngr_notify_to_user(struct dsi_panel *panel, enum lge_drs_result result)
{
	char *envp[5];
	char name[30], name_buf[30];
	struct lge_drs_mngr *drs_mngr = lge_drs_mngr_get_and_validate(panel);
	struct drm_device *dev = lge_drs_mngr_dm_to_dev(drs_mngr);
	struct drm_connector *conn = lge_drs_mngr_dm_to_conn(drs_mngr);

	if (!drs_mngr || !dev || !conn)
		return -EINVAL;

	if (result > 0) {
		snprintf(name_buf, sizeof(name_buf), "RESOLUTION_SWITCH=%d", result);
		snprintf(name, sizeof(name), "name=%s", conn->name);
		envp[0] = name_buf;
		envp[1] = name;
		envp[2] = NULL;
		kobject_uevent_env(&dev->primary->kdev->kobj, KOBJ_CHANGE, envp);
	}

	return 0;
}

static void lge_drs_mngr_work(struct work_struct *work)
{
	int rc, retry_cnt = DRS_MAX_RETRY;
	struct lge_drs_mngr *drs_mngr = NULL;
	struct delayed_work *dw = to_delayed_work(work);
	struct dsi_panel *panel = NULL;

	drs_mngr = container_of(dw, struct lge_drs_mngr, drs_work);
	if (!drs_mngr) {
		pr_err("drs mngr is null\n");
		return;
	}

	rc = wait_for_completion_timeout(&drs_mngr->drs_done, DRS_TIMEOUT);
	if (rc <= 0) {
		pr_err("timeout\n");
	}

	panel = lge_drs_mngr_dm_to_panel(drs_mngr);

	if (!panel) {
		pr_err("panel is null\n");
		goto out;
	}

	do {
		if (lge_drs_mngr_is_updated(panel)) {
			pr_info("switching success\n");
			rc = lge_drs_mngr_notify_to_user(panel, DRS_SUCCESS);
			if (rc) {
				pr_warn("WARNING: fail to notify\n");
			}
			break;
		} else {
			mutex_lock(&panel->panel_lock);
			rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_TIMING_SWITCH);
			if (rc)
				pr_err("failed to send switch cmds, rc=%d\n", rc);
			mutex_unlock(&panel->panel_lock);
		}

		if (0 == --retry_cnt) {
			mdelay(17);
			pr_err("failed to switch scale info\n");
			rc = lge_drs_mngr_notify_to_user(panel, DRS_FAIL);
			if (rc)
				pr_warn("WARNING: fail to notify");
		}
	} while (retry_cnt);

	if ((is_bist_supported(panel, "drs")) && (panel->lge.ddic_ops && panel->lge.ddic_ops->bist_ctrl)) {
		mutex_lock(&panel->lge.bist_lock);
		if (panel->lge.ddic_ops->bist_ctrl(panel, false) < 0)
			pr_err("fail to control BIST\n");
		mutex_unlock(&panel->lge.bist_lock);
	}

	if (drs_mngr->drs_requested)
		drs_mngr->drs_requested = false;
out:
	mutex_lock(&drs_mngr->drs_lock);
	drs_mngr->drs_state = DRS_IDLE;
	mutex_unlock(&drs_mngr->drs_lock);
	complete(&drs_mngr->drs_work_done);
}

bool lge_drs_mngr_is_enabled(struct dsi_panel *panel)
{
	if (!panel)
		return -EINVAL;

	if (!panel->lge.use_drs_mngr) {
		pr_err("is not enabled\n");
		return false;
	}

	return true;
}

int lge_drs_mngr_get_state(struct dsi_panel *panel)
{
	struct lge_drs_mngr *drs_mngr = lge_drs_mngr_get_and_validate(panel);

	if (!drs_mngr) {
		pr_err("not validated\n");
		return -EINVAL;
	}

	return drs_mngr->drs_state;
}

int lge_drs_mngr_finish(struct dsi_panel *panel)
{
	int rc = 0;
	struct lge_drs_mngr *drs_mngr = lge_drs_mngr_get_and_validate(panel);

	if (!drs_mngr) {
		pr_err("not validated\n");
		return -EINVAL;
	}

	mutex_lock(&drs_mngr->drs_lock);
	if (drs_mngr->drs_state == DRS_RUNNING) {
		pr_info("switching done\n");
		complete(&drs_mngr->drs_done);
	}
	mutex_unlock(&drs_mngr->drs_lock);

	return rc;
}

int lge_drs_mngr_begin(struct dsi_panel *panel)
{
	int rc = 0;
	struct lge_drs_mngr *drs_mngr = lge_drs_mngr_get_and_validate(panel);

	if (!drs_mngr) {
		pr_err("is null\n");
		return -EINVAL;
	}

	mutex_lock(&drs_mngr->drs_lock);
	if (drs_mngr->drs_state > DRS_IDLE) {
		pr_info("duplicated\n");
		mutex_unlock(&drs_mngr->drs_lock);
		return rc;
	}

	drs_mngr->request_res = panel->cur_mode->timing.v_active;
	pr_info("request_res=%d\n", drs_mngr->request_res);

	if ((is_bist_supported(panel, "drs")) && (panel->lge.ddic_ops && panel->lge.ddic_ops->bist_ctrl)) {
		mutex_lock(&panel->lge.bist_lock);
		if (panel->lge.ddic_ops->bist_ctrl(panel, true) < 0)
			pr_err("fail to control BIST\n");
		mutex_unlock(&panel->lge.bist_lock);
	}

	init_completion(&drs_mngr->drs_done);
	init_completion(&drs_mngr->drs_work_done);
	drs_mngr->drs_state = DRS_RUNNING;
	mutex_unlock(&drs_mngr->drs_lock);
	/* turn off */
	queue_delayed_work(drs_mngr->drs_workq, &drs_mngr->drs_work, msecs_to_jiffies(17));
	return rc;
}

static int lge_drs_mngr_get_main_display(struct lge_drs_mngr *drs_mngr)
{
	struct dsi_display *display = NULL;

	if (!drs_mngr)
		return -EINVAL;

	display = primary_display;
	if (display) {
		drs_mngr->main_display = display;
	} else {
		pr_err("%s not found\n", "main_display");
		return -EAGAIN;
	}

	return 0;
}

int lge_drs_mngr_init(struct dsi_panel *panel)
{
	int i, rc = 0;
	struct lge_drs_mngr *drs_mngr = lge_drs_mngr_get_and_validate(panel);

	if (!drs_mngr) {
		pr_err("drs mngr is not validated\n");
		return -EINVAL;
	}

	mutex_init(&drs_mngr->drs_lock);

	if (!drs_mngr->main_display) {
		rc = lge_drs_mngr_get_main_display(drs_mngr);
		if (rc) {
			pr_err("fail to get main_display, rc=%d\n", rc);
			return -EINVAL;
		}
	}

	drs_mngr->current_freeze_state = DRS_NOT_READY;

	drs_mngr->drs_workq = create_workqueue("lge_drs_mngr");
	if (!drs_mngr->drs_workq) {
		pr_warn("warning: fail to create workqueue\n");
		drs_mngr->drs_state = DRS_NONE;
	} else {
		INIT_DELAYED_WORK(&drs_mngr->drs_work, lge_drs_mngr_work);
		drs_mngr->drs_state = DRS_IDLE;
	}

	if ((panel->num_timing_nodes > 0) && (panel->lge.ddic_ops && panel->lge.ddic_ops->get_support_res)) {
		for (i = 0; i < panel->num_timing_nodes; i++)
			panel->lge.ddic_ops->get_support_res(i, &drs_mngr->supported_res[i]);
		drs_mngr->num_res = panel->num_timing_nodes;
	}

	gdrs_mngr = drs_mngr;

	pr_info("drs mngr initialized\n");

	return rc;
}

static int lge_drs_mngr_freeze_timedout(struct dsi_panel *panel)
{
	struct lge_drs_mngr *drs_mngr = NULL;


	if (!panel)
		return -EINVAL;

	drs_mngr = &panel->lge.drs_mngr;
	if (!drs_mngr) {
		pr_err("drs_mngr is null\n");
		return -EINVAL;
	}

	drs_mngr->drs_requested = false;

	if (is_bist_supported(panel, "drs")) {
		drs_mngr->keep_bist_on = false;
		if ((panel->lge.bist_on > 0) &&
				(panel->lge.ddic_ops && panel->lge.ddic_ops->release_bist)) {
			pr_warn("WARNING: timeout: release bist\n");
			mutex_lock(&panel->lge.bist_lock);
			if (panel->lge.ddic_ops->release_bist(panel) < 0)
				pr_err("fail to release BIST\n");
			mutex_unlock(&panel->lge.bist_lock);
		}
	}

	return 0;
}

int lge_drs_mngr_freeze_state_hal(int req_type)
{
	struct dsi_panel *panel = NULL;
	struct drm_device *dev = NULL;
	struct lge_drs_mngr *drs_mngr = gdrs_mngr;
	char *envp1[3] = {"PANEL_FREEZE=1", "name=DSI-1", NULL};
	char *envp0[3] = {"PANEL_FREEZE=0", "name=DSI-1", NULL};
	int rc = 0;

	if (!drs_mngr) {
		pr_err("gdrs mngr is null\n");
		return -EINVAL;
	}

	if (drs_mngr->drs_state < DRS_IDLE) {
		pr_err("drs mngr is not ready\n");
		return -EINVAL;
	}

	if (drs_mngr->main_display->is_cont_splash_enabled) {
		pr_err("%s : skip freeze with cont_splash_enabled", __func__);
		return -EINVAL;
	}

	panel = lge_drs_mngr_dm_to_panel(drs_mngr);
	if (!panel) {
		pr_err("panel is null\n");
		return -EINVAL;
	}

	dev = lge_drs_mngr_dm_to_dev(drs_mngr);
	if (!dev) {
		pr_err("dev is null\n");
		return -EINVAL;
	}

	switch (req_type) {
	case DRS_UNFREEZE:
		kobject_uevent_env(&dev->primary->kdev->kobj, KOBJ_CHANGE, envp0);
		break;
	case DRS_FREEZE:
		drs_mngr->drs_requested = true;
		if (is_bist_supported(panel, "drs")) {
			drs_mngr->keep_bist_on = false;
		}
		kobject_uevent_env(&dev->primary->kdev->kobj, KOBJ_CHANGE, envp1);
		break;
	case DRS_TIMEDOUT:
		pr_info("freeze timed-out\n");
		rc = lge_drs_mngr_freeze_timedout(panel);
		if (rc < 0)
			pr_err("failed timedout func\n");
		break;
	default:
		pr_err("invalid request=%d\n", req_type);
		return -EINVAL;
		break;
	}

	return rc;
}

int lge_drs_mngr_set_freeze_state(enum lge_drs_request state)
{
	struct lge_drs_mngr *drs_mngr = gdrs_mngr;

	if (!drs_mngr) {
		pr_err("gdrs mngr is null\n");
		return -EINVAL;
	}

	pr_info("change state (%d)->(%d)\n", drs_mngr->current_freeze_state, state);
	drs_mngr->current_freeze_state = state;

	return 0;
}

int lge_drs_mngr_get_freeze_state(void)
{
	struct lge_drs_mngr *drs_mngr = gdrs_mngr;

	if (!drs_mngr) {
		pr_err("gdrs mngr is null\n");
		return -EINVAL;
	}

	return drs_mngr->current_freeze_state;
}

static ssize_t freeze_state_hal_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int freeze_state = lge_drs_mngr_get_freeze_state();
	return scnprintf(buf, PAGE_SIZE, "%d\n", freeze_state);
}

static ssize_t freeze_state_hal_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int rc, enable;

	sscanf(buf, "%d", &enable);

	rc = lge_drs_mngr_freeze_state_hal(enable);
	if (rc < 0) {
		pr_warn("WARNING: fail to freeze, rc=%d\n", rc);
	} else {
		if(enable > DRS_REQUEST_MAX)
			enable = DRS_REQUEST_MAX;
		lge_drs_mngr_set_freeze_state(enable);
	}

	return ret;
}
static DEVICE_ATTR(freeze_state_hal, S_IRUGO | S_IWUSR | S_IWGRP,
		freeze_state_hal_show, freeze_state_hal_store);

void lge_panel_drs_create_sysfs(struct dsi_panel *panel, struct class *class_panel)
{
	static struct device *panel_drs_dev = NULL;

	if (class_panel) {
		panel_drs_dev = device_create(class_panel, NULL, 0, panel, "drs");
		if (IS_ERR(panel_drs_dev)) {
			pr_err("Failed to create dev(panel_reg_dev)!\n");
		} else {
			if (device_create_file(panel_drs_dev, &dev_attr_freeze_state_hal) < 0)
				pr_err("Failed to create panel/drs/freeze_state_hal\n");
		}
	}
}
