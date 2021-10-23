#define pr_fmt(fmt)	"[Display][lge-dsi-panel:%s:%d] " fmt, __func__, __LINE__

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <video/mipi_display.h>
#include <linux/lge_panel_notify.h>
#include <soc/qcom/lge/board_lge.h>

#include "msm_drv.h"
#include "dsi_panel.h"
#include "dsi_ctrl_hw.h"
#include "dsi_display.h"
#include "sde_connector.h"
#include "sde_encoder.h"
#include <drm/drm_mode.h>
#include "dsi_drm.h"
#include "drs/lge_drs_mngr.h"
#include "lge_ddic_ops_helper.h"
#include "brightness/lge_brightness_def.h"
#include "cm/lge_color_manager.h"
#include "factory/lge_factory.h"
#include "err_detect/lge_err_detect.h"

extern int mdss_dsi_parse_color_manager_modes(struct device_node *np,
			struct lge_dsi_color_manager_mode_entry color_manager_table[NUM_COLOR_MODES],
			u32 *color_manager_modes_num,
			const char *name);
extern void lge_bc_dim_work_init(struct dsi_panel *panel);
int dsi_panel_reset(struct dsi_panel *panel);

extern int lge_get_mfts_mode(void);
extern int dsi_panel_set_pinctrl_state(struct dsi_panel *panel, bool enable);
extern int dsi_panel_tx_cmd_set(struct dsi_panel *panel, enum dsi_cmd_set_type type);
extern int dsi_panel_get_cmd_pkt_count(const char *data, u32 length, u32 *cnt);
extern int dsi_panel_create_cmd_packets(const char *data,
					u32 length,
					u32 count,
					struct dsi_cmd_desc *cmd);
extern int dsi_panel_alloc_cmd_packets(struct dsi_panel_cmd_set *cmd,
					u32 packet_count);

extern void lge_dsi_panel_blmap_free(struct dsi_panel *panel);
extern int lge_dsi_panel_parse_blmap(struct dsi_panel *panel, struct device_node *of_node);
extern int lge_dsi_panel_parse_brightness(struct dsi_panel *panel,	struct device_node *of_node);
extern void lge_panel_drs_create_sysfs(struct dsi_panel *panel, struct class *class_panel);
extern void lge_panel_reg_create_sysfs(struct dsi_panel *panel, struct class *class_panel);
extern void lge_ddic_ops_init(struct dsi_panel *panel);
extern void lge_ddic_feature_init(struct dsi_panel *panel);
extern int lge_ddic_dsi_panel_parse_cmd_sets(struct dsi_panel *panel, struct device_node *of_node);
extern int lge_update_backlight_ex(struct dsi_panel *panel);
extern void lge_ambient_create_sysfs(struct dsi_panel *panel, struct class *class_panel);
extern int lge_drs_mngr_init(struct dsi_panel *panel);
extern int lge_mdss_dsi_panel_cmd_read(struct dsi_panel *panel,
					u8 cmd, int cnt, char* ret_buf);
extern int lge_mdss_dsi_panel_cmds_backup(struct dsi_panel *panel, char *owner,
				enum dsi_cmd_set_type type, u8 reg, int nth_cmds);
static int lge_dsi_panel_pin_seq(struct lge_panel_pin_seq *seq);
extern int lge_ddic_dsi_panel_tx_cmd_set(struct dsi_panel *panel,
				enum lge_ddic_dsi_cmd_set_type type);
extern int lge_brightness_create_sysfs(struct dsi_panel *panel,
		struct class *class_panel);
#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY) | IS_ENABLED(CONFIG_LGE_DUAL_SCREEN)
extern void lge_cover_create_sysfs(struct dsi_panel *panel);
#endif /* CONFIG_LGE_COVER_DISPLAY */
extern int lge_ambient_set_interface_data(struct dsi_panel *panel);
extern int lge_ddic_dsi_panel_parse_cm_lut_cmd_sets(struct dsi_panel *panel,
				struct device_node *of_node);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DIMMING_BOOT_SUPPORT)
/*---------------------------------------------------------------------------*/
/* LCD off & dimming                                                         */
/*---------------------------------------------------------------------------*/
static bool fb_blank_called;

bool is_blank_called(void)
{
	return fb_blank_called;
}

int lge_get_bootreason_with_lcd_dimming(void)
{
	int ret = 0;

	if (lge_get_bootreason() == 0x23)
		ret = 1;
	else if (lge_get_bootreason() == 0x24)
		ret = 2;
	else if (lge_get_bootreason() == 0x25)
		ret = 3;
	return ret;
}

bool is_factory_cable(void)
{
	cable_boot_type cable_info = lge_get_boot_cable();

	if (cable_info == LT_CABLE_56K ||
		cable_info == LT_CABLE_130K ||
		cable_info == LT_CABLE_910K)
		return true;
	return false;
}

static void lge_set_blank_called(void)
{
	fb_blank_called = true;
}
#endif

static int lge_dsi_panel_mode_set(struct dsi_panel *panel)
{
	bool reg_backup_cond = false;
	int tc_perf_status = 0;

	if (!panel) {
		pr_err("invalid params\n");
		return -EINVAL;
	}
	reg_backup_cond = !(panel->lge.use_ddic_reg_backup^panel->lge.ddic_reg_backup_complete);

	pr_info("backup=%d\n", reg_backup_cond);
	if (reg_backup_cond && panel->lge.use_color_manager) {
		if (panel->lge.ddic_ops &&
				panel->lge.ddic_ops->lge_set_screen_mode)
			panel->lge.ddic_ops->lge_set_screen_mode(panel, true);

		if (panel->lge.ddic_ops &&
				panel->lge.ddic_ops->lge_bc_dim_set)
			panel->lge.ddic_ops->lge_bc_dim_set(panel, BC_DIM_ON, BC_DIM_FRAMES_NORMAL);
		panel->lge.is_sent_bc_dim_set = true;
	} else {
		pr_warn("skip ddic mode set on booting or not supported!\n");
	}

	if(panel->lge.use_tc_perf) {
		tc_perf_status = panel->lge.tc_perf;
		if (tc_perf_status && panel->lge.ddic_ops &&
				panel->lge.ddic_ops->lge_set_tc_perf) {
			panel->lge.ddic_ops->lge_set_tc_perf(panel, tc_perf_status);
		}
	}

	return 0;
}

static inline bool cmd_set_exists(struct dsi_panel *panel, enum lge_ddic_dsi_cmd_set_type type)
{
	if (!panel || !panel->cur_mode)
		return false;
	return (panel->lge.lge_cmd_sets[type].count != 0);
}

#define MAN_NAME_LEN    10
#define	DDIC_NAME_LEN	15
static char lge_man_name[MAN_NAME_LEN+1];
static char lge_ddic_name[DDIC_NAME_LEN+1];

char* get_ddic_name(void)
{
	return lge_ddic_name;
}

bool is_ddic_name(char *ddic_name)
{
	if (ddic_name == NULL) {
		pr_err("input parameter is NULL\n");
		return false;
	}

	if(!strcmp(lge_ddic_name, ddic_name)) {
		return true;
	}
	pr_err("input ddic_name = %s, lge_ddic = %s\n", ddic_name, lge_ddic_name);
	return false;
}
EXPORT_SYMBOL(is_ddic_name);

int dsi_panel_full_power_seq(struct dsi_panel *panel)
{
	int ret = 0;

	if (!panel->lge.is_incell || lge_get_mfts_mode() || panel->lge.panel_dead)
		ret = 1;

	if (panel->lge.mfts_auto_touch)
		ret = 0;

	return ret;
}

static const char *LPNAME[] = { "NOLP", "LP1", "LP2", "OFF", "MAX"};
static bool dsi_panel_need_mask(struct dsi_panel *panel)
{
	if (panel->lge.partial_area_vertical_changed)
		return true;

	if (panel->lge.partial_area_height_changed &&
			panel->lge.update_pps_in_lp)
		return true;

	return false;
}

static void set_aod_area(struct dsi_panel *panel)
{
	int rc = 0;

	if (!panel) {
		pr_err("invalid params\n");
		return;
	}

	if (panel->lge.ddic_ops && panel->lge.ddic_ops->prepare_aod_area) {
		panel->lge.ddic_ops->prepare_aod_area(panel,
				panel->lge.lge_cmd_sets[LGE_DDIC_DSI_AOD_AREA].cmds,
				panel->lge.lge_cmd_sets[LGE_DDIC_DSI_AOD_AREA].count);
	}

	rc = lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_AOD_AREA);
	if (rc) {
		pr_err("[%s] failed to send cmd, rc=%d\n",
		       panel->name, rc);
	}

	return;
}

static void set_lp(struct dsi_panel *panel, enum lge_panel_lp_state lp_state, enum lge_ddic_dsi_cmd_set_type cmd_set_type)
{
	int rc = 0;

	if (!panel) {
		pr_err("invalid params\n");
		return;
	}

	if (panel->lge.panel_state == lp_state) {
		pr_debug("already %s state\n", LPNAME[lp_state]);
		return;
	}

	if (!cmd_set_exists(panel, cmd_set_type)) {
		pr_err("No %s cmd\n", LPNAME[lp_state]);
		return;
	}

	rc = lge_ddic_dsi_panel_tx_cmd_set(panel, cmd_set_type);
	if (rc) {
		pr_err("[%s] failed to send %d cmd, rc=%d\n",
		       panel->name, cmd_set_type, rc);
	} else {
		panel->lge.panel_state = lp_state;
		pr_info("sent %s cmd\n", LPNAME[lp_state]);
	}

	return;
}

static int dsi_panel_get_current_power_mode(struct dsi_panel *panel)
{
	struct dsi_display *display = NULL;
	struct sde_connector *conn = NULL;
	int mode;

	if (!panel) {
		pr_err("invalid panel param\n");
		return -EINVAL;
	}

	display = container_of(panel->host, struct dsi_display, host);
	if (!display) {
		pr_err("invalid display param\n");
		return -EINVAL;
	}

	conn = to_sde_connector(display->drm_conn);
	if (!conn)
		return -EINVAL;

	switch (conn->dpms_mode) {
	case DRM_MODE_DPMS_ON:
		mode = conn->lp_mode;
		break;
	case DRM_MODE_DPMS_STANDBY:
		mode = SDE_MODE_DPMS_STANDBY;
		break;
	case DRM_MODE_DPMS_SUSPEND:
		mode = SDE_MODE_DPMS_SUSPEND;
		break;
	case DRM_MODE_DPMS_OFF:
		mode = SDE_MODE_DPMS_OFF;
		break;
	default:
		mode = conn->lp_mode;
		pr_err("unrecognized mode=%d\n", mode);
		break;
	}

	return mode;
}

static int dsi_panel_get_last_power_mode(struct dsi_panel *panel)
{
	struct dsi_display *display = NULL;
	struct sde_connector *conn = NULL;

	if (!panel) {
		pr_err("invalid panel param\n");
		return -EINVAL;
	}

	display = container_of(panel->host, struct dsi_display, host);
	if (!display) {
		pr_err("invalid display param\n");
		return -EINVAL;
	}

	conn = to_sde_connector(display->drm_conn);
	if (!conn)
		return -EINVAL;

	return conn->last_panel_power_mode;
}

static inline bool is_power_off(int pwr_mode)
{
	return (pwr_mode == SDE_MODE_DPMS_OFF);
}

static inline bool is_power_on_interactive(int pwr_mode)
{
	return (pwr_mode == SDE_MODE_DPMS_ON);
}

static inline bool is_power_on(int pwr_mode)
{
	return !is_power_off(pwr_mode);
}

static inline bool is_power_on_lp(int pwr_mode)
{
	return !is_power_off(pwr_mode) &&
		!is_power_on_interactive(pwr_mode);
}

static inline bool is_power_on_ulp(int pwr_mode)
{
	return (pwr_mode == SDE_MODE_DPMS_LP2);
}

bool lge_dsi_panel_is_power_off(struct dsi_panel *panel)
{
	int last_panel_power_mode = dsi_panel_get_current_power_mode(panel);
	return is_power_off(last_panel_power_mode);
}

bool lge_dsi_panel_is_power_on_interactive(struct dsi_panel *panel)
{
	int last_panel_power_mode = dsi_panel_get_current_power_mode(panel);
	return is_power_on_interactive(last_panel_power_mode);
}

bool lge_dsi_panel_is_power_on(struct dsi_panel *panel)
{
	int last_panel_power_mode = dsi_panel_get_current_power_mode(panel);
	return is_power_on(last_panel_power_mode);
}

bool lge_dsi_panel_is_power_on_lp(struct dsi_panel *panel)
{
	int last_panel_power_mode = dsi_panel_get_current_power_mode(panel);
	return is_power_on_lp(last_panel_power_mode);
}

bool lge_dsi_panel_is_power_on_ulp(struct dsi_panel *panel)
{
	int last_panel_power_mode = dsi_panel_get_current_power_mode(panel);
	return is_power_on_ulp(last_panel_power_mode);
}

static enum lge_ddic_dsi_cmd_set_type dsi_panel_select_cmd_type(struct dsi_panel *panel)
{
	int rc;
	int last_panel_power_mode;
	enum lge_ddic_dsi_cmd_set_type type = LGE_DDIC_DSI_CMD_SET_MAX;

	last_panel_power_mode = dsi_panel_get_last_power_mode(panel);
	if (last_panel_power_mode < 0) {
		pr_err("fail to get last_panel_pwr_mode\n");
		goto exit;
	}
	mutex_lock(&panel->lge.pa_changed_lock);
	if (!is_power_on_lp(last_panel_power_mode)) {
		if (is_power_off(last_panel_power_mode) &&
				panel->lge.use_cmd_wait_pa_changed &&
				((panel->lge.aod_area.w == 0) ||
				(panel->lge.aod_area.h == 0))) {
			init_completion(&panel->lge.pa_changed_done);
			panel->lge.wait_pa_changed = true;
			mutex_unlock(&panel->lge.pa_changed_lock);
			rc = wait_for_completion_timeout(&panel->lge.pa_changed_done, 2000);
			if (rc <= 0) {
				pr_warn("aod image will be displayed by default setting\n");
			}
			mutex_lock(&panel->lge.pa_changed_lock);
		}
		type = LGE_DDIC_DSI_SET_LP2;
	} else if (panel->lge.partial_area_vertical_changed ||
			panel->lge.partial_area_height_changed) {
		type = LGE_DDIC_DSI_AOD_AREA;
	} else {
		pr_debug("skip command\n");
	}
	mutex_unlock(&panel->lge.pa_changed_lock);

exit:
	pr_info("select_cmd=%d\n", type);
	return type;
}

static int dsi_panel_send_lp_cmds(struct dsi_panel *panel,
				enum lge_ddic_dsi_cmd_set_type cmd_type)
{
	int rc = 0;
	bool need_mask = true;
	bool need_prepare = true;
	char *bist_name;
	enum lge_panel_lp_state panel_state = LGE_PANEL_STATE_MAX;

	if (!panel)
		return -EINVAL;

	if ((panel->lge.use_fp_lhbm) && (panel->lge.ddic_ops) &&
			(panel->lge.ddic_ops->lge_set_fp_lhbm) &&
			((cmd_type == LGE_DDIC_DSI_SET_LP2) ||
			(cmd_type == LGE_DDIC_DSI_SET_NOLP)) &&
			((panel->lge.old_fp_lhbm_mode == LGE_FP_LHBM_SM_ON) ||
			(panel->lge.old_fp_lhbm_mode == LGE_FP_LHBM_ON) ||
			(panel->lge.old_fp_lhbm_mode == LGE_FP_LHBM_FORCED_ON) ||
			(panel->lge.old_panel_fp_mode == LGE_FP_LHBM_READY)))
		panel->lge.ddic_ops->lge_set_fp_lhbm(panel, LGE_FP_LHBM_FORCED_EXIT);

	switch (cmd_type) {
	case LGE_DDIC_DSI_SET_LP1:
		bist_name = "lp1";
		panel_state = LGE_PANEL_LP1;
		break;
	case LGE_DDIC_DSI_SET_LP2:
		bist_name = "lp2";
		panel_state = LGE_PANEL_LP2;
		break;
	case LGE_DDIC_DSI_SET_NOLP:
		bist_name = "nolp";
		panel_state = LGE_PANEL_NOLP;
		need_prepare = false;
		break;
	case LGE_DDIC_DSI_AOD_AREA:
		bist_name ="aod_area";
		break;
	default:
		bist_name = "none";
		break;
	};

	mutex_lock(&panel->lge.pa_changed_lock);

	if (cmd_type == LGE_DDIC_DSI_CMD_SET_MAX)
		goto exit;

	need_mask = dsi_panel_need_mask(panel);

	/* 1. masking */
	if (is_bist_supported(panel, bist_name) && need_mask &&
			(panel->lge.ddic_ops && panel->lge.ddic_ops->bist_ctrl)) {
		mutex_lock(&panel->lge.bist_lock);
		if (panel->lge.ddic_ops->bist_ctrl(panel, true) < 0)
			pr_err("fail to control BIST\n");
		mutex_unlock(&panel->lge.bist_lock);
	}

	/* 2. send lp command */
	mutex_lock(&panel->panel_lock);

	/* UNLOCK REGISTER */
	if (panel->lge.use_ddic_reg_lock)
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_UNLOCK);

	if (panel->cur_mode && need_prepare) {
		set_aod_area(panel);

		if (cmd_type != LGE_DDIC_DSI_AOD_AREA) {
			if (panel->lge.ddic_ops && panel->lge.ddic_ops->prepare_aod_cmds) {
				panel->lge.ddic_ops->prepare_aod_cmds(panel,
						panel->lge.lge_cmd_sets[cmd_type].cmds,
						panel->lge.lge_cmd_sets[cmd_type].count);
			}
		}
	}

	if(cmd_type != LGE_DDIC_DSI_AOD_AREA)
		set_lp(panel, panel_state, cmd_type);

	/* LOCK REGISTER */
	if (panel->lge.use_ddic_reg_lock)
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_LOCK);

	mutex_unlock(&panel->panel_lock);
	/* 3. update pps */
	if (panel->lge.update_pps_in_lp) {
		if (panel->lge.ddic_ops && panel->lge.ddic_ops->set_pps_cmds) {
			rc = panel->lge.ddic_ops->set_pps_cmds(panel, cmd_type);
			if (rc) {
				pr_warn("WARNING: fail to update pps info\n");
			}
		}

		rc = dsi_panel_update_pps(panel);
		if (rc)
			pr_err("fail to send pps\n");

		if (panel->lge.ddic_ops && panel->lge.ddic_ops->unset_pps_cmds) {
			rc = panel->lge.ddic_ops->unset_pps_cmds(panel, cmd_type);
			if (rc) {
				pr_warn("WARNING: fail to unset pps info\n");
			}
		}
	}

	/* 4. un-masking */
	if ((is_bist_supported(panel, bist_name)) && need_mask &&
			(panel->lge.ddic_ops && panel->lge.ddic_ops->bist_ctrl)) {
		mutex_lock(&panel->lge.bist_lock);
		if (panel->lge.ddic_ops->bist_ctrl(panel, false) < 0)
			pr_err("fail to control BIST\n");
		mutex_unlock(&panel->lge.bist_lock);
	}

	panel->lge.partial_area_vertical_changed = false;
	panel->lge.partial_area_height_changed = false;
exit:
	mutex_unlock(&panel->lge.pa_changed_lock);

	return rc;
}

static int dsi_panel_update_lp_state(struct dsi_panel *panel, enum lge_panel_lp_state new)
{
	if (!panel)
		return -EINVAL;

	panel->lge.lp_state = new;
	return 0;
}

/* @Override */
int dsi_panel_set_lp2(struct dsi_panel *panel)
{
	int rc;
	enum lge_ddic_dsi_cmd_set_type cmd_type;

	if (!panel)
		return -EINVAL;

	cmd_type = dsi_panel_select_cmd_type(panel);

	rc = dsi_panel_send_lp_cmds(panel, cmd_type);
	if (rc < 0) {
		pr_err("fail to send lp command\n");
	}

	if ((cmd_type == LGE_DDIC_DSI_CMD_SET_MAX) || (cmd_type == LGE_DDIC_DSI_AOD_AREA))
		panel->lge.panel_state = LGE_PANEL_LP2;
	rc = dsi_panel_update_lp_state(panel, LGE_PANEL_LP2);
	if (rc < 0) {
		pr_err("fail to update lp state\n");
	} else {
		lge_panel_notifier_call_chain(LGE_PANEL_EVENT_BLANK,
				0, LGE_PANEL_STATE_LP2); /* U2_UNBLANK; DOZE */
	}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DIMMING_BOOT_SUPPORT)
	lge_set_blank_called();
#endif
	return rc;
}

/* @Override */
int dsi_panel_set_lp1(struct dsi_panel *panel)
{
	int rc;
	enum lge_ddic_dsi_cmd_set_type cmd_type;

	if (!panel)
		return -EINVAL;

	/**
	 * Consider LP1->LP2->LP1.
	 * If the panel is already in LP mode, do not need to
	 * set the regulator.
	 * IBB and AB power mode woulc be set at the same time
	 * in PMIC driver, so we only call ibb setting, that
	 * is enough.
	 */
	if (dsi_panel_is_type_oled(panel) &&
	    panel->power_mode != SDE_MODE_DPMS_LP2)
		dsi_pwr_panel_regulator_mode_set(&panel->power_info,
			"ibb", REGULATOR_MODE_IDLE);

	cmd_type = dsi_panel_select_cmd_type(panel);

	rc = dsi_panel_send_lp_cmds(panel, cmd_type);
	if (rc < 0) {
		pr_err("fail to send lp command\n");
	}

	if ((cmd_type == LGE_DDIC_DSI_CMD_SET_MAX) || (cmd_type == LGE_DDIC_DSI_AOD_AREA))
		panel->lge.panel_state = LGE_PANEL_LP1;
	rc = dsi_panel_update_lp_state(panel, LGE_PANEL_LP1);
	if (rc < 0) {
		pr_err("update lp state\n");
	} else {
		lge_panel_notifier_call_chain(LGE_PANEL_EVENT_BLANK,
				0, LGE_PANEL_STATE_LP1); /* U2_BLANK; DOZE_SUSPEND */
	}
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DIMMING_BOOT_SUPPORT)
	lge_set_blank_called();
#endif
	return rc;
}

/* @Override */
int dsi_panel_set_nolp(struct dsi_panel *panel)
{
	int rc;
	struct dsi_display *display = NULL;
	struct sde_connector *conn = NULL;
	int lp_mode, last_panel_power_mode;

	if (!panel)
		return -EINVAL;

	/**
	 * Consider about LP1->LP2->NOLP.
	 */
	if (dsi_panel_is_type_oled(panel) &&
	    (panel->power_mode == SDE_MODE_DPMS_LP1 ||
		panel->power_mode == SDE_MODE_DPMS_LP2))
		dsi_pwr_panel_regulator_mode_set(&panel->power_info,
			"ibb", REGULATOR_MODE_NORMAL);

	display = container_of(panel->host, struct dsi_display, host);
	conn = to_sde_connector(display->drm_conn);
	lp_mode = conn->lp_mode;
	last_panel_power_mode = conn->last_panel_power_mode;

	if (lp_mode == SDE_MODE_DPMS_ON && last_panel_power_mode == SDE_MODE_DPMS_OFF) {
		panel->lge.panel_state = LGE_PANEL_NOLP;
		rc = dsi_panel_update_lp_state(panel, LGE_PANEL_NOLP);
		pr_info("going to on from off\n");
		goto mode_set;
	} else if (lp_mode == SDE_MODE_DPMS_OFF) {
		panel->lge.panel_state = LGE_PANEL_OFF;
		rc = dsi_panel_update_lp_state(panel, LGE_PANEL_OFF);
		pr_info("going to off\n");
		return 0;
	}

	rc = dsi_panel_send_lp_cmds(panel, LGE_DDIC_DSI_SET_NOLP);
	if (rc < 0) {
		pr_err("fail to send lp command\n");
	}

	rc = dsi_panel_update_lp_state(panel, LGE_PANEL_NOLP);
	if (rc < 0) {
		pr_err("fail to update lp state\n");
	}

	lge_panel_notifier_call_chain(LGE_PANEL_EVENT_BLANK, 0, LGE_PANEL_STATE_UNBLANK); // U3, UNBLANK

mode_set:
	lge_dsi_panel_mode_set(panel);

	return rc;
}

/* @Override */
int dsi_panel_power_on(struct dsi_panel *panel)
{
	int rc = 0;

	rc = dsi_panel_set_pinctrl_state(panel, true);
	if (rc) {
		pr_err("[%s] failed to set pinctrl, rc=%d\n", panel->name, rc);
		goto exit;
	}

	if (dsi_panel_full_power_seq(panel)) {
		if(panel->lge.pins) {
			rc = lge_dsi_panel_pin_seq(panel->lge.panel_on_seq);
			lge_panel_notifier_call_chain(LGE_PANEL_EVENT_POWER, 0, LGE_PANEL_POWER_VDDIO_ON); // PANEL VDDIO ON
			if (rc){
				pr_err("[%s] failed to set lge panel pin, rc=%d\n", panel->name, rc);
				goto error_pinctrl_false;
			}
		}
	} else {
		if (panel->lge.use_panel_reset_low_before_lp11) {
			pr_info("[Display] reset low before LP11");
			if (gpio_is_valid(panel->reset_config.reset_gpio)) {
				lge_panel_notifier_call_chain(LGE_PANEL_EVENT_RESET, 0, LGE_PANEL_RESET_LOW); // PANEL RESET LOW
				gpio_set_value(panel->reset_config.reset_gpio, 0);
				usleep_range(5000, 5000);
			}
		} else {
			pr_info("[Display] Do not control display powers for Incell display");
		}
	}

	usleep_range(5000, 5000);

	if (panel->lge.use_labibb && panel->lge.reset_after_ddvd) {
		rc = dsi_pwr_enable_regulator(&panel->power_info, true);
		if (rc) {
			pr_err("[%s] failed to enable LABIBB, rc=%d\n", panel->name, rc);
			goto error_disable_gpio_before_ddvd;
		}
		pr_info("Turn on LABIBB\n");
		lge_panel_notifier_call_chain(LGE_PANEL_EVENT_POWER, 0, LGE_PANEL_POWER_DSV_ON); // PANEL DSV ON
		usleep_range(5000, 5000);
	}

	goto exit;

error_disable_gpio_before_ddvd:
	if (panel->lge.reset_after_ddvd)
		dsi_pwr_enable_regulator(&panel->power_info, false);

	if (dsi_panel_full_power_seq(panel) || panel->lge.use_panel_reset_low_before_lp11) {
		if(panel->lge.pins) {
			lge_dsi_panel_pin_seq(panel->lge.panel_off_seq);
		} else {
			dsi_pwr_enable_regulator(&panel->power_info, false);
		}
	}

error_pinctrl_false:
		(void)dsi_panel_set_pinctrl_state(panel, false);

exit:
	return rc;
}

static int lge_dsi_panel_pin_seq(struct lge_panel_pin_seq *seq)
{
	int rc = 0;

	if (!seq) {
		return -EINVAL;
	}

	while (gpio_is_valid(seq->gpio)) {
		if (seq->level) {
			rc = gpio_direction_output(seq->gpio, 1);
			if (rc) {
				pr_err("unable to set dir for gpio %d, rc=%d\n", seq->gpio, rc);
				break;
			} else {
				pr_info("gpio %d -> %d\n", seq->gpio, 1);
			}
		} else {
			gpio_set_value(seq->gpio, 0);
			pr_info("gpio %d -> %d\n", seq->gpio, 0);
		}
		usleep_range(seq->sleep_ms*1000, seq->sleep_ms*1000);
		seq++;
	}

	return rc;
}

/* @Override */
int dsi_panel_power_off(struct dsi_panel *panel)
{
	int rc = 0;

	if (gpio_is_valid(panel->reset_config.disp_en_gpio))
		gpio_set_value(panel->reset_config.disp_en_gpio, 0);

	if (gpio_is_valid(panel->reset_config.lcd_mode_sel_gpio))
		gpio_set_value(panel->reset_config.lcd_mode_sel_gpio, 0);

	if (panel->lge.use_labibb && panel->lge.reset_after_ddvd) {
		if (panel->lge.is_incell) {
			if (dsi_panel_full_power_seq(panel)) {
				dsi_pwr_set_regulator(&panel->power_info, REGULATOR_MODE_TTW_OFF);
			} else {
				dsi_pwr_set_regulator(&panel->power_info, REGULATOR_MODE_TTW_ON);
			}
		}
		rc = dsi_pwr_enable_regulator(&panel->power_info, false);
		if (rc)
			pr_err("[%s] failed to disable LABIBB, rc=%d\n", panel->name, rc);
		else {
			pr_info("Turn off LABIBB\n");
			lge_panel_notifier_call_chain(LGE_PANEL_EVENT_POWER, 0, LGE_PANEL_POWER_DSV_OFF); // PANEL DSV OFF
		}
		usleep_range(5000, 5000);
	}

	if (dsi_panel_full_power_seq(panel)) {
		lge_dsi_panel_pin_seq(panel->lge.panel_off_seq); // load
		lge_panel_notifier_call_chain(LGE_PANEL_EVENT_POWER, 0, LGE_PANEL_POWER_VDDIO_OFF); // PANEL VDDIO LOW
	} else {
		pr_info("[Display] Do not control LCD powers for LPWG mode");
	}

	if (panel->lge.use_ddic_reg_backup && panel->lge.ddic_reg_backup_complete &&
			panel->lge.is_sent_bc_dim_set)
		panel->lge.is_sent_bc_dim_set = false;
	else if (!panel->lge.use_ddic_reg_backup && panel->lge.is_sent_bc_dim_set)
		panel->lge.is_sent_bc_dim_set = false;

	if (panel->lge.use_irc_ctrl || panel->lge.use_ace_ctrl) {
		panel->lge.ddic_ops->set_irc_default_state(panel);
		panel->lge.irc_pending = false;
	}

	lge_panel_notifier_call_chain(LGE_PANEL_EVENT_BLANK, 0, LGE_PANEL_STATE_BLANK); // U0, BLANK
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DIMMING_BOOT_SUPPORT)
	lge_set_blank_called();
#endif
	return rc;
}

static int dsi_panel_reset_off(struct dsi_panel *panel)
{
	int rc = 0;

	if (lge_get_factory_boot() || lge_get_mfts_mode()) {
		if (panel->lge.use_panel_err_detect && panel->lge.err_detect_irq_enabled)
			lge_panel_err_detect_irq_control(panel, false);
	}
	usleep_range(5000, 5000);

	if (panel->lge.use_labibb && !panel->lge.reset_after_ddvd) {
		if (panel->lge.is_incell) {
			if (dsi_panel_full_power_seq(panel)) {
				dsi_pwr_set_regulator(&panel->power_info, REGULATOR_MODE_TTW_OFF);
			} else {
				dsi_pwr_set_regulator(&panel->power_info, REGULATOR_MODE_TTW_ON);
			}
		}
		rc = dsi_pwr_enable_regulator(&panel->power_info, false);
		if (rc)
			pr_err("[%s] failed to disable LABIBB, rc=%d\n", panel->name, rc);
		else {
			pr_info("Turn off LABIBB\n");
			lge_panel_notifier_call_chain(LGE_PANEL_EVENT_POWER, 0, LGE_PANEL_POWER_DSV_OFF); // PANEL DSV OFF
		}
		usleep_range(5000, 5000);
	}

	if (dsi_panel_full_power_seq(panel)) {
		rc = dsi_panel_set_pinctrl_state(panel, false);
		if (rc) {
			pr_err("[%s] failed set pinctrl state, rc=%d\n", panel->name, rc);
		}

		if (gpio_is_valid(panel->reset_config.reset_gpio)) {
			pr_info("Set Reset GPIO to Low\n");
			lge_panel_notifier_call_chain(LGE_PANEL_EVENT_RESET, 0, LGE_PANEL_RESET_LOW); // PANEL RESET LOW
			gpio_set_value(panel->reset_config.reset_gpio, 0);
		}
		usleep_range(5000, 5000);
	}
	return rc;
}

int dsi_panel_reset(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_panel_reset_config *r_config = &panel->reset_config;
	int i;

	if (gpio_is_valid(panel->reset_config.disp_en_gpio)) {
		rc = gpio_direction_output(panel->reset_config.disp_en_gpio, 1);
		if (rc) {
			pr_err("unable to set dir for disp gpio rc=%d\n", rc);
			goto exit;
		}
	}

	if (r_config->count) {
		rc = gpio_direction_output(r_config->reset_gpio,
			r_config->sequence[0].level);
		if (rc) {
			pr_err("unable to set dir for rst gpio rc=%d\n", rc);
			goto exit;
		}
	}

	pr_info("Set Reset GPIO to HIGH\n");
	lge_panel_notifier_call_chain(LGE_PANEL_EVENT_RESET, 0, LGE_PANEL_RESET_HIGH); // PANEL RESET HIGH
	for (i = 0; i < r_config->count; i++) {
		gpio_set_value(r_config->reset_gpio,
			       r_config->sequence[i].level);

		if (r_config->sequence[i].sleep_ms)
			usleep_range(r_config->sequence[i].sleep_ms * 1000,
				     r_config->sequence[i].sleep_ms * 1000);
	}

	usleep_range(5000, 5000);

	if (panel->lge.use_labibb && !panel->lge.reset_after_ddvd) {
		rc = dsi_pwr_enable_regulator(&panel->power_info, true);
		if (rc) {
			pr_err("[%s] failed to enable LABIBB, rc=%d\n", panel->name, rc);
			goto error_disable_reset_pin;
		}
		pr_info("Turn on LABIBB\n");
		lge_panel_notifier_call_chain(LGE_PANEL_EVENT_POWER, 0, LGE_PANEL_POWER_DSV_ON); // PANEL DSV ON
	}

	if (gpio_is_valid(panel->bl_config.en_gpio)) {
		rc = gpio_direction_output(panel->bl_config.en_gpio, 1);
		if (rc) {
			pr_err("unable to set dir for bklt gpio rc=%d\n", rc);
			goto error_disable_regulator;
		}
	}

	if (gpio_is_valid(panel->reset_config.lcd_mode_sel_gpio)) {
		bool out = true;

		if ((panel->reset_config.mode_sel_state == MODE_SEL_DUAL_PORT)
				|| (panel->reset_config.mode_sel_state
					== MODE_GPIO_LOW))
			out = false;
		else if ((panel->reset_config.mode_sel_state
				== MODE_SEL_SINGLE_PORT) ||
				(panel->reset_config.mode_sel_state
				 == MODE_GPIO_HIGH))
			out = true;

		rc = gpio_direction_output(
			panel->reset_config.lcd_mode_sel_gpio, out);
		if (rc) {
			pr_err("unable to set dir for mode gpio rc=%d\n", rc);
			goto error_disable_gpio_and_regulator;
		}
	}

	goto exit;

error_disable_gpio_and_regulator:
	if (gpio_is_valid(panel->reset_config.disp_en_gpio))
		gpio_set_value(panel->reset_config.disp_en_gpio, 0);

	if (gpio_is_valid(panel->bl_config.en_gpio))
		gpio_set_value(panel->bl_config.en_gpio, 0);

error_disable_regulator:
	if (panel->lge.use_labibb && !panel->lge.reset_after_ddvd) {
		(void)dsi_pwr_enable_regulator(&panel->power_info, false);
		lge_panel_notifier_call_chain(LGE_PANEL_EVENT_POWER, 0, LGE_PANEL_POWER_DSV_OFF); // PANEL DSV OFF
	}

error_disable_reset_pin:
	if (gpio_is_valid(panel->reset_config.reset_gpio)) {
		lge_panel_notifier_call_chain(LGE_PANEL_EVENT_RESET, 0, LGE_PANEL_RESET_LOW); // PANEL RESET LOW
		gpio_set_value(panel->reset_config.reset_gpio, 0);
	}
exit:
	return rc;
}

/* @Override */
int dsi_panel_pre_prepare(struct dsi_panel *panel)
{
	int rc = 0;

	if (!panel) {
		pr_err("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	rc = dsi_panel_power_on(panel);
	if (rc) {
		pr_err("[%s] panel power on failed, rc=%d\n", panel->name, rc);
		goto error;
	}

	/* If LP11_INIT is set, panel will be powered up during prepare() */
	if (panel->lp11_init)
		goto error;

	rc = dsi_panel_reset(panel);
	if (rc) {
		pr_err("[%s] panel reset failed, rc=%d\n", panel->name, rc);
		goto error;
	}

error:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

/* @Override */
int dsi_panel_prepare(struct dsi_panel *panel)
{
	int rc = 0;

	if (!panel) {
		pr_err("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	if (panel->lp11_init) {
		rc = dsi_panel_reset(panel);
		if (rc) {
			pr_err("[%s] panel reset failed, rc=%d\n",
			       panel->name, rc);
			goto error;
		}
	}

	rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_PRE_ON);
	if (rc) {
		pr_err("[%s] failed to send DSI_CMD_SET_PRE_ON cmds, rc=%d\n",
		       panel->name, rc);
		goto error;
	}

error:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

/* @Override */
int dsi_panel_unprepare(struct dsi_panel *panel)
{
	int rc = 0;

	if (!panel) {
		pr_err("invalid params\n");
		return -EINVAL;
	}

	if (panel->lge.use_ddic_reg_backup) {
		if (atomic_read(&panel->lge.backup_state) > 0) {
			pr_warn("WARNING: backup work pending\n");
			flush_work(&panel->lge.backup_work);
			atomic_set(&panel->lge.backup_state, 0);
		}
	}

	if ((lge_drs_mngr_is_enabled(panel))
			&& (lge_drs_mngr_get_state(panel) > DRS_IDLE)) {
		rc = lge_drs_mngr_finish(panel);
		if (rc) {
			pr_warn("WARNING: drs is not finished correctly\n");
		} else {
			rc = wait_for_completion_timeout(&panel->lge.drs_mngr.drs_work_done,
							DRS_TIMEOUT);
			if (rc <= 0)
				pr_err("drs timeout\n");

			pr_info("drs state (%d)\n", lge_drs_mngr_get_state(panel));
		}
	}

	panel->lge.allow_bl_update_ex = false;

	mutex_lock(&panel->panel_lock);
	rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_POST_OFF);
	if (rc) {
		pr_err("[%s] failed to send DSI_CMD_SET_POST_OFF cmds, rc=%d\n",
		       panel->name, rc);
		goto error;
	}

	if (panel->lp11_init) {
		rc = dsi_panel_reset_off(panel);
		if (rc) {
			pr_err("[%s] panel reset_off failed, rc=%d\n", panel->name, rc);
			goto error;
		}
	}

error:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

/* @Override */
int dsi_panel_post_unprepare(struct dsi_panel *panel)
{
	int rc = 0;

	if (!panel) {
		pr_err("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	if (!panel->lp11_init) {
		rc = dsi_panel_reset_off(panel);
		if (rc) {
			pr_err("[%s] panel reset_off failed, rc=%d\n", panel->name, rc);
			goto error;
		}
	}

	rc = dsi_panel_power_off(panel);
	if (rc) {
		pr_err("[%s] panel power_Off failed, rc=%d\n", panel->name, rc);
		goto error;
	}

error:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

/* @Override */
int dsi_panel_post_enable(struct dsi_panel *panel)
{
	int rc = 0;

	if (!panel) {
		pr_err("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_POST_ON);
	if (rc) {
		pr_err("[%s] failed to send DSI_CMD_SET_POST_ON cmds, rc=%d\n",
		       panel->name, rc);
		goto error;
	}

	if (panel->lge.panel_dead) {
		panel->lge.panel_dead = false;
		lge_panel_notifier_call_chain(LGE_PANEL_EVENT_RECOVERY, 0, LGE_PANEL_RECOVERY_ALIVE);
		if (panel->lge.bl_lvl_recovery_unset == -1) {
			panel->bl_config.bl_level = panel->lge.bl_lvl_unset;
			dsi_panel_set_backlight(panel, panel->lge.bl_lvl_unset);
		} else {
			panel->bl_config.bl_level = panel->lge.bl_lvl_recovery_unset;
			dsi_panel_set_backlight(panel, panel->lge.bl_lvl_recovery_unset);
		}
		panel->lge.bl_lvl_recovery_unset = -1;
		panel->lge.bl_lvl_unset = -1;
		panel->lge.allow_bl_update = true;
	}

	if (lge_get_factory_boot() || lge_get_mfts_mode()) {
		if (panel->lge.use_panel_err_detect && !panel->lge.err_detect_irq_enabled) {
			if (panel->lge.ddic_ops && panel->lge.ddic_ops->set_err_detect_mask)
				panel->lge.ddic_ops->set_err_detect_mask(panel);
			lge_panel_err_detect_irq_control(panel, true);
		}
	}
error:
	mutex_unlock(&panel->panel_lock);

	pr_info("lp_state=%d, dsi_mode_flags=0x%X\n", panel->lge.lp_state, panel->cur_mode->dsi_mode_flags);
	if (panel->lge.lp_state == LGE_PANEL_NOLP && !(panel->cur_mode->dsi_mode_flags & DSI_MODE_FLAG_DMS))
		lge_panel_notifier_call_chain(LGE_PANEL_EVENT_BLANK, 0, LGE_PANEL_STATE_UNBLANK); // U3, UNBLANK

	return rc;
}

static void lge_mdss_panel_dead_notify(struct dsi_display *display)
{
	struct sde_connector *conn = NULL;
	struct drm_event event;

	if (!display) {
		pr_err("display is null.\n");
		return;
	}

	if (!display->panel) {
		pr_err("panel is null.\n");
		return;
	}

	conn = to_sde_connector(display->drm_conn);
	if (!conn) {
		pr_err("display->drm_conn is null\n");
		return;
	}

	mutex_lock(&display->panel->panel_lock);
	if (display->panel->lge.panel_dead) {
		pr_err("Already in recovery state\n");
		mutex_unlock(&display->panel->panel_lock);
		return;
	}

	pr_info("******** ESD detected!!!!LCD recovery function called!!!! ********\n");
	display->panel->lge.panel_dead = true;

	if (display->panel->lge.bl_lvl_unset == -1 && display->panel->lge.allow_bl_update == false)
		display->panel->lge.bl_lvl_recovery_unset = -1;
	else if (display->panel->lge.bl_lvl_unset != -1 && display->panel->lge.allow_bl_update == false)
		display->panel->lge.bl_lvl_recovery_unset = display->panel->lge.bl_lvl_unset;
	else
		display->panel->lge.bl_lvl_recovery_unset = display->panel->bl_config.bl_level;

	mutex_unlock(&display->panel->panel_lock);

	if (lge_get_factory_boot() || lge_get_mfts_mode()) {
		if (display->panel->lge.use_panel_err_detect && display->panel->lge.err_detect_irq_enabled)
			lge_panel_err_detect_irq_control(display->panel, false);
	}
	lge_panel_notifier_call_chain(LGE_PANEL_EVENT_RECOVERY, 0, LGE_PANEL_RECOVERY_DEAD);

	event.type = DRM_EVENT_PANEL_DEAD;
	event.length = sizeof(u32);
	msm_mode_object_event_notify(&conn->base.base,
		conn->base.dev, &event, (u8 *)&display->panel->lge.panel_dead);
	sde_encoder_display_failure_notification(conn->encoder, false);
}

void lge_mdss_panel_dead_work(struct work_struct *work)
{
	struct lge_dsi_panel *lge_panel = NULL;
	struct dsi_panel *panel = NULL;
	struct dsi_display *display = NULL;
	struct sde_connector *conn = NULL;

	lge_panel = container_of(to_delayed_work(work), struct lge_dsi_panel, panel_dead_work);
	if (!lge_panel) {
		pr_err("fail to get lge_dsi_panel object\n");
		return;
	}

	panel = container_of(lge_panel, struct dsi_panel, lge);
	if (!panel) {
		pr_err("fail to get dsi_panel object\n");
		return;
	}

	display = container_of(panel->host, struct dsi_display, host);
	if (!display) {
		pr_err("display is null.\n");
		return;
	}

	conn = to_sde_connector(display->drm_conn);
	if (!conn) {
		pr_err("display->drm_conn is null\n");
		return;
	}

	mutex_lock(&conn->lock);
	if (dsi_panel_get_current_power_mode(display->panel) != SDE_MODE_DPMS_ON) {
		mutex_unlock(&conn->lock);
		pr_info("lp_state is not nolp(U3)\n");
		mutex_lock(&display->panel->panel_lock);
		if (display->panel->lge.panel_dead_pending) {
			mutex_unlock(&display->panel->panel_lock);
			pr_info("re-trigger panel_dead after 5 secs\n");
			schedule_delayed_work(&display->panel->lge.panel_dead_work,
							msecs_to_jiffies(STATUS_CHECK_INTERVAL_MS));
			return;
		}
		mutex_unlock(&display->panel->panel_lock);
	}
	mutex_unlock(&conn->lock);

	mutex_lock(&display->panel->panel_lock);
	display->panel->lge.panel_dead_pending = false;
	mutex_unlock(&display->panel->panel_lock);

	lge_mdss_panel_dead_notify(display);
}

void lge_mdss_report_panel_dead(void)
{
	struct dsi_display *display = NULL;
	struct sde_connector *conn = NULL;

	display = primary_display;

	if (!display) {
		pr_err("display is null.\n");
		return;
	}
	if (!display->panel) {
		pr_err("panel is null.\n");
		return;
	}

	conn = to_sde_connector(display->drm_conn);
	if (!conn) {
		pr_err("display->drm_conn is null\n");
		return;
	}

	mutex_lock(&conn->lock);
	if (dsi_panel_get_current_power_mode(display->panel) != SDE_MODE_DPMS_ON) {
		pr_info("lp_state is not nolp(U3)\n");
		if (!display->panel->lge.panel_dead_pending) {
			pr_info("re-trigger panel_dead after 5 secs\n");
			display->panel->lge.panel_dead_pending = true;
			mutex_unlock(&conn->lock);
			schedule_delayed_work(&display->panel->lge.panel_dead_work,
							msecs_to_jiffies(STATUS_CHECK_INTERVAL_MS));
		} else {
			pr_info("already re-triggered panel_dead\n");
			mutex_unlock(&conn->lock);
		}
		return;
	} else if (display->panel->lge.panel_dead_pending) {
		pr_err("already panel dead work scheduled\n");
		mutex_unlock(&conn->lock);
		return;
	}

	mutex_unlock(&conn->lock);

	lge_mdss_panel_dead_notify(display);
}
EXPORT_SYMBOL(lge_mdss_report_panel_dead);

static ssize_t lge_mdss_force_report_panel_dead(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	lge_mdss_report_panel_dead();
	return 1;
}
static DEVICE_ATTR(report_panel_dead, S_IRUGO,
										lge_mdss_force_report_panel_dead, NULL);

static int lge_dsi_panel_gpio_request(struct dsi_panel *panel)
{
	int rc = 0, i, j;
	char name[10] = {0,};

	for (i = 0; i < panel->lge.pins_num; i++) {
		if (gpio_is_valid(panel->lge.pins[i])) {
			snprintf(name, sizeof(name), "panel_pin_%d_gpio", i);
			rc = gpio_request(panel->lge.pins[i], name);
			if (rc) {
				pr_err("request for %s failed, rc=%d\n", name, rc);
				break;
			}
		}
	}

	if (i < panel->lge.pins_num) {
		for (j = i; j >= 0; j--) {
			if (gpio_is_valid(panel->lge.pins[j]))
				gpio_free(panel->lge.pins[j]);
		}
	}

	return rc;
}

static int lge_dsi_panel_gpio_release(struct dsi_panel *panel)
{
	int rc = 0, i;
	for (i = 0; i < panel->lge.pins_num; i++) {
		if (gpio_is_valid(panel->lge.pins[i]))
			gpio_free(panel->lge.pins[i]);
	}
	return rc;
}

static ssize_t panel_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel = NULL;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	pr_err("%s-%s\n", lge_man_name, lge_ddic_name);

	/* The number of characters should not exceed 30 characters. */
	return sprintf(buf, "%s-%s\n", lge_man_name, lge_ddic_name);
}
static DEVICE_ATTR(panel_type, S_IRUGO,
		panel_type_show, NULL);

static ssize_t mfts_auto_touch_test_mode_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_panel *panel;

	panel = dev_get_drvdata(dev);

	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", panel->lge.mfts_auto_touch);
}

static ssize_t mfts_auto_touch_test_mode_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)

{
	struct dsi_panel *panel;
	int input;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	sscanf(buf, "%d", &input);
	mutex_lock(&panel->panel_lock);
	panel->lge.mfts_auto_touch = input;
	mutex_unlock(&panel->panel_lock);

	pr_info("auto touch test : %d\n", input);

	return size;
}
static DEVICE_ATTR(mfts_auto_touch_test_mode, S_IRUGO | S_IWUSR | S_IWGRP,
		mfts_auto_touch_test_mode_get, mfts_auto_touch_test_mode_set);

void lge_panel_factory_create_sysfs(struct dsi_panel *panel, struct class *class_panel)
{
	static struct device *panel_reg_dev = NULL;

	if (!panel_reg_dev) {
		panel_reg_dev = device_create(class_panel, NULL, 0, panel, "factory");
		if (IS_ERR(panel_reg_dev)) {
			pr_err("Failed to create dev(panel_reg_dev)!\n");
		} else {
			if ((device_create_file(panel_reg_dev, &dev_attr_panel_type)) < 0)
				pr_err("add panel_type node fail!\n");
			if ((device_create_file(panel_reg_dev, &dev_attr_mfts_auto_touch_test_mode)) < 0)
				pr_err("add mfts_auto_touch_test_mode node fail!\n");
			if (panel->lge.use_line_detect)
				lge_panel_line_detect_create_sysfs(panel_reg_dev);
		}
	}
}

static void lge_dsi_panel_create_sysfs(struct dsi_panel *panel)
{
	static struct class *class_panel = NULL;
	static struct device *panel_img_tune_sysfs_dev = NULL;
	static struct device *panel_test_sysfs_dev = NULL;
	int rc =0;

	if(!class_panel){
		class_panel = class_create(THIS_MODULE, "panel");
		if (IS_ERR(class_panel)) {
			pr_err("Failed to create panel class\n");
			return;
		}
	}

	if(!panel_img_tune_sysfs_dev){
		panel_img_tune_sysfs_dev = device_create(class_panel, NULL, 0, panel, "img_tune");
		if (IS_ERR(panel_img_tune_sysfs_dev)) {
			pr_err("Failed to create dev(panel_img_tune_sysfs_dev)!\n");
		} else {
			if (panel->lge.use_color_manager)
				lge_color_manager_create_sysfs(panel, panel_img_tune_sysfs_dev);
		}
	}
	if (panel->lge.use_irc_ctrl || panel->lge.use_ace_ctrl ||
			panel->lge.use_dynamic_brightness || panel->lge.use_fp_lhbm ||
			panel->lge.use_tc_perf)
		lge_brightness_create_sysfs(panel, class_panel);
	if (panel->lge.use_ambient)
		lge_ambient_create_sysfs(panel, class_panel);

	lge_panel_drs_create_sysfs(panel, class_panel);
	lge_panel_reg_create_sysfs(panel, class_panel);
	lge_panel_factory_create_sysfs(panel, class_panel);
	lge_panel_err_detect_create_sysfs(panel, class_panel);

	if(!panel_test_sysfs_dev){
		panel_test_sysfs_dev = device_create(class_panel, NULL, 0, panel, "test");
		if (IS_ERR(panel_test_sysfs_dev)) {
			pr_err("Failed to create dev(panel_test_sysfs_dev)!\n");
		} else {
			if ((rc = device_create_file(panel_test_sysfs_dev, &dev_attr_report_panel_dead)) < 0)
				pr_err("add report_panel_dead set node fail!");
		}
	}

#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY) | IS_ENABLED(CONFIG_LGE_DUAL_SCREEN)
	lge_cover_create_sysfs(panel);
#endif /* CONFIG_LGE_COVER_DISPLAY */
}

int lge_dsi_panel_drv_init(struct dsi_panel *panel)
{
	int rc = 0;

	rc = lge_dsi_panel_gpio_request(panel);
	if (rc)
		pr_err("failed to request panel pins, rc=%d\n", rc);

	lge_dsi_panel_create_sysfs(panel);

	return rc;
}

int lge_dsi_panel_drv_post_init(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_display *display = container_of(panel->host, struct dsi_display, host);

	pr_info("bl control variable init\n");
	if (display->is_cont_splash_enabled) {
		panel->lge.allow_bl_update = true;
		panel->lge.bl_lvl_unset = -1;
		panel->lge.allow_bl_update_ex = false;
		panel->lge.bl_ex_lvl_unset = -1;
		panel->lge.bl_lvl_recovery_unset = -1;
	} else {
		panel->lge.allow_bl_update = false;
		panel->lge.bl_lvl_unset = -1;
		panel->lge.allow_bl_update_ex = false;
		panel->lge.bl_ex_lvl_unset = -1;
		panel->lge.bl_lvl_recovery_unset = -1;
	}
	return rc;
}

int lge_dsi_panel_drv_deinit(struct dsi_panel *panel)
{
	int rc = 0;

	if (panel->lge.use_panel_err_detect)
		lge_panel_err_detect_remove(panel);
	rc = lge_dsi_panel_gpio_release(panel);
	if (rc)
		pr_err("failed to release panel pins, rc=%d\n", rc);

	return rc;
}

static int lge_dsi_panel_parse_pin_seq(struct dsi_panel *panel, struct device_node *of_node, const char* prop_name, struct lge_panel_pin_seq **seq_out)
{
	int rc = 0, i;
	u32 length = 0, count = 0;
	const u32 *prop;
	u32 *arr;
	struct lge_panel_pin_seq *seq;

	prop = of_get_property(of_node, prop_name, &length);
	if (!prop) {
		pr_err("%s not found\n", prop_name);
		rc = -EINVAL;
		goto error_no_free_arr;
	}

	arr = kzalloc(length, GFP_KERNEL);
	if (!arr) {
		rc = -ENOMEM;
		goto error_no_free_arr;
	}

	length /= sizeof(u32);
	rc = of_property_read_u32_array(of_node, prop_name, arr, length);
	if (rc) {
		pr_err("failed to read u32 array of %s, rc=%d\n", prop_name, rc);
		goto error_free_arr;
	}

	count = length/3;
	seq = kzalloc(sizeof(struct lge_panel_pin_seq)*(count+1), GFP_KERNEL);
	if (!seq) {
		rc = -ENOMEM;
		goto error_free_arr;
	}

	*seq_out = seq;
	for (i = 0; i < length; i += 3) {
		if (arr[i] >= panel->lge.pins_num || arr[i] < 0) {
			pr_err("failed to parse %s, pins_num=%d, arr[%d]=%d\n", prop_name, panel->lge.pins_num, i, arr[i]);
			rc = -EINVAL;
			break;
		}
		seq->gpio = panel->lge.pins[arr[i]];
		seq->level = arr[i+1];
		seq->sleep_ms = arr[i+2];
		seq++;
	}
	seq->gpio = -1;

	if (rc) {
		kfree(*seq_out);
		*seq_out = NULL;
	}

error_free_arr:
	kfree(arr);
error_no_free_arr:
	return rc;
}

#define LGE_PROPNAME_PANEL_ON_PIN_SEQ "lge,panel-on-pin-seq"
#define LGE_PROPNAME_PANEL_OFF_PIN_SEQ "lge,panel-off-pin-seq"
#define LGE_PROPNAME_PANEL_PINS "lge,panel-pins"
static int lge_dsi_panel_parse_gpios(struct dsi_panel *panel, struct device_node *of_node)
{
	int rc = 0, i;

	panel->lge.pins_num = of_gpio_named_count(of_node, LGE_PROPNAME_PANEL_PINS);

	if (panel->lge.pins_num <= 0) {
		pr_warn("no panel pin defined\n");
		return 0;
	}

	panel->lge.pins = kcalloc(panel->lge.pins_num, sizeof(int), GFP_KERNEL);
	if (!panel->lge.pins) {
		rc = -ENOMEM;
		goto error_alloc_gpio_array;
	}

	for (i = 0; i < panel->lge.pins_num; i++) {
		panel->lge.pins[i] = of_get_named_gpio(of_node, LGE_PROPNAME_PANEL_PINS, i);
	}

	rc = lge_dsi_panel_parse_pin_seq(panel, of_node, LGE_PROPNAME_PANEL_ON_PIN_SEQ, &panel->lge.panel_on_seq);
	if (rc) {
		goto error_parse_pins;
	}

	rc = lge_dsi_panel_parse_pin_seq(panel, of_node, LGE_PROPNAME_PANEL_OFF_PIN_SEQ, &panel->lge.panel_off_seq);
	if (rc) {
		goto error_parse_pins;
	}

	return rc;

error_parse_pins:
	kfree(panel->lge.pins);
	panel->lge.pins = NULL;
	panel->lge.pins_num = 0;

error_alloc_gpio_array:
	return rc;
}

static int lge_dsi_panel_parse_dt(struct dsi_panel *panel, struct device_node *of_node)
{
	int rc = 0;
	const char *ddic_name;
	const char *man_name;
	u32 tmp = 0;

	// TODO: change property name
	panel->lp11_init = of_property_read_bool(of_node, "qcom,mdss-dsi-lane-0-state");

	memset(lge_man_name, 0x0, MAN_NAME_LEN+1);
	man_name = of_get_property(of_node, "lge,man-name", NULL);
	if (man_name) {
		strncpy(lge_man_name, man_name, MAN_NAME_LEN);
		pr_info("lge_man_name=%s\n", lge_man_name);
	} else {
		strncpy(lge_man_name, "undefined", MAN_NAME_LEN);
		pr_info("manufacturer name is not set\n");
	}

	memset(lge_ddic_name, 0x0, DDIC_NAME_LEN+1);
	ddic_name = of_get_property(of_node, "lge,ddic-name", NULL);
	if (ddic_name) {
		strncpy(lge_ddic_name, ddic_name, DDIC_NAME_LEN);
		pr_info("lge_ddic_name=%s\n", lge_ddic_name);
	} else {
		strncpy(lge_ddic_name, "undefined", DDIC_NAME_LEN);
		pr_info("ddic name is not set\n");
	}

	// TODO: temporal use
	panel->lge.use_labibb = of_property_read_bool(of_node, "lge,use-labibb");
	pr_info("use_labibb=%d\n", panel->lge.use_labibb);

	panel->lge.reset_after_ddvd = of_property_read_bool(of_node, "lge,reset-after-ddvd");
	pr_info("reset_after_ddvd=%d\n", panel->lge.reset_after_ddvd);

	panel->lge.dcs_brightness_be = of_property_read_bool(of_node, "lge,dcs-brightness-bigendian");
	pr_info("dcs_brightness_be=%d\n", panel->lge.dcs_brightness_be);

	panel->lge.is_incell = of_property_read_bool(of_node, "lge,incell-panel");
	pr_info("is_incell=%d\n", panel->lge.is_incell);

	panel->lge.use_bist = of_property_read_bool(of_node, "lge,ddic-bist-enabled");
	pr_info("use bist pattern=%d\n", panel->lge.use_bist);
	if (panel->lge.use_bist) {
		int i = 0;
		rc = of_property_read_string_array(of_node, "lge,ddic-bist-usage-type",
						panel->lge.bist_usage_type,
						MAX_BIST_USAGE_TYPE);
		for (i = 0; i < MAX_BIST_USAGE_TYPE; i++) {
			pr_debug("bist type=%s\n", panel->lge.bist_usage_type[i]);
		}
	}

	panel->lge.update_pps_in_lp = of_property_read_bool(of_node, "lge,update-pps-in-lp-mode");
	pr_info("update_pps in lp state=%d\n", panel->lge.update_pps_in_lp);

	panel->lge.use_drs_mngr = of_property_read_bool(of_node, "lge,drs-mngr-enabled");
	pr_info("use drs manager=%d\n", panel->lge.use_drs_mngr);

	panel->lge.use_internal_pps_switch =
		of_property_read_bool(of_node, "lge,drs-mngr-internal-pps-switch-enabled");

	panel->lge.use_ddic_reg_backup = of_property_read_bool(of_node, "lge,ddic-register-backup");
	pr_info("use register backup=%d\n", panel->lge.use_ddic_reg_backup);

	panel->lge.bc_dim_en = of_property_read_bool(of_node, "lge,use-bc-dim");
	pr_info("use bc dim =%d\n", panel->lge.bc_dim_en);

	panel->lge.use_color_manager = of_property_read_bool(of_node, "lge,use-color-manager");
	pr_info("use color manager=%d\n", panel->lge.use_color_manager);

	rc = of_property_read_u32(of_node, "lge,hbm-mode", &tmp);
	if (rc) {
		panel->lge.hbm_mode = DEFAULT_HBM_MODE;
		pr_err("fail to parse lge.hbm_mode Set to Default %d\n", panel->lge.hbm_mode);
	} else {
		panel->lge.hbm_mode = tmp;
		pr_info("lge.hbm_mode %d\n", panel->lge.hbm_mode);
	}

	panel->lge.use_ambient = of_property_read_bool(of_node, "lge,use-ambient");
	pr_info("use ambient=%d\n", panel->lge.use_ambient);
	if (panel->lge.use_ambient) {
		rc = of_property_read_string_array(of_node, "lge,aod-interface-data",
						panel->lge.aod_interface_type, 3);
		lge_ambient_set_interface_data(panel);
	}

	panel->lge.use_cmd_wait_pa_changed = of_property_read_bool(of_node, "lge,cmd-wait-pa-changed");
	pr_info("use cmd_wait_pa_changed=%d\n", panel->lge.use_cmd_wait_pa_changed);

	panel->lge.use_line_detect = of_property_read_bool(of_node, "lge,use-line-detect");
	pr_info("use line detect=%d\n", panel->lge.use_line_detect);

	panel->lge.use_bc_dimming_work = of_property_read_bool(of_node, "lge,bc-dimming-work");
	pr_info("use bc dimming work=%d\n", panel->lge.use_bc_dimming_work);

	if (panel->lge.use_color_manager) {
		rc = of_property_read_u32(of_node, "lge,color-manager-default-status", &tmp);
		if (rc) {
			pr_err("fail to parse lge,color-manager-default-status\n");
			panel->lge.color_manager_default_status = false;
		} else {
			panel->lge.color_manager_default_status = (tmp > 0)? true : false;
			panel->lge.color_manager_status = 1;
			pr_info("color manager default status is %d\n", panel->lge.color_manager_default_status);
		}

		mdss_dsi_parse_color_manager_modes(of_node, panel->lge.color_manager_table,
					&(panel->lge.color_manager_table_len), "lge,mdss-dsi-color-manager-mode-table");

		panel->lge.dgc_absent = of_property_read_bool(of_node, "lge,digital-gamma-absent");
		pr_info("digital gamma absent = %d\n", panel->lge.dgc_absent);
	} else {
		panel->lge.color_manager_default_status = false;
	}

	panel->lge.use_panel_err_detect = of_property_read_bool(of_node, "lge,use-panel-err-detect");
	pr_info("use panel err detect = %d\n", panel->lge.use_panel_err_detect);

	if (panel->lge.use_panel_err_detect) {
		lge_panel_err_detect_parse_dt(panel, of_node);
	}

	panel->lge.use_panel_reset_low_before_lp11 = of_property_read_bool(of_node, "lge,use-panel-reset-low-before-lp11");
	pr_info("use panel reset low before lp11 = %d\n", panel->lge.use_panel_reset_low_before_lp11);

	panel->lge.use_extra_recovery_cmd = of_property_read_bool(of_node, "lge,use-extra-recovery-cmd");
	pr_info("use extra recovery command = %d\n", panel->lge.use_extra_recovery_cmd);

	panel->lge.use_dcs_brightness_short = of_property_read_bool(of_node, "lge,dcs-brightness-short-write");
	pr_info("use dcs_brightness_short=%d\n", panel->lge.use_dcs_brightness_short);

	panel->lge.use_ddic_reg_lock = of_property_read_bool(of_node, "lge,use-ddic-register-lock");
	pr_info("use ddic_register_lock=%d\n", panel->lge.use_ddic_reg_lock);

	panel->lge.use_irc_ctrl = of_property_read_bool(of_node, "lge,use-irc-ctrl");
	pr_info("use irc_ctrl=%d\n", panel->lge.use_irc_ctrl);

	panel->lge.use_ace_ctrl = of_property_read_bool(of_node, "lge,use-ace-ctrl");
	pr_info("use ace_ctrl=%d\n", panel->lge.use_ace_ctrl);

	if (panel->lge.use_ace_ctrl) {
		rc = of_property_read_u32(of_node, "lge,default-ace-mode", &tmp);
		if (rc) {
			pr_err("fail to get ace default, set %d\n", panel->lge.ace_mode);
		} else {
			panel->lge.ace_mode = tmp;
			pr_info("ace default mode=%d\n", panel->lge.ace_mode);
		}
	}

	panel->lge.true_view_supported = of_property_read_bool(of_node, "lge,true-view-supported");
	pr_info("use true_view supported=%d\n", panel->lge.true_view_supported);

	panel->lge.use_vr_lp_mode = of_property_read_bool(of_node, "lge,use-vr-lp-mode");
	pr_info("use vr_lp_mode=%d\n", panel->lge.use_vr_lp_mode);

	panel->lge.use_dim_ctrl = of_property_read_bool(of_node, "lge,use-dim-ctrl");
	pr_info("use_dim_ctrl=%d\n", panel->lge.use_dim_ctrl);

	panel->lge.use_br_ctrl_ext = of_property_read_bool(of_node, "lge,disp-br-ctrl-ext-supported");
	pr_info("use_br_ctrl_ext=%d\n", panel->lge.use_br_ctrl_ext);

	panel->lge.use_fp_lhbm = of_property_read_bool(of_node, "lge,use-fp-lhbm");
	if(panel->lge.use_fp_lhbm) {
		panel->lge.fp_lhbm_br_lvl = FP_LHBM_DEFAULT_BR_LVL;
		panel->lge.need_fp_lhbm_set = false;
		panel->lge.is_fp_hbm_mode = of_property_read_bool(of_node, "lge,is-fp-hbm-mode");
		if (panel->lge.is_fp_hbm_mode) {
			rc = of_property_read_u32(of_node, "lge,fp-hbm-mode-backlight-level", &tmp);
			if (rc) {
				panel->lge.fp_hbm_mode_backlight_lvl = 0xFFF;
				pr_err("fail to get fp_hbm_mode_backlight_level. set %d\n", panel->lge.fp_hbm_mode_backlight_lvl);
			} else {
				panel->lge.fp_hbm_mode_backlight_lvl = tmp;
				pr_info("fp_hbm_mode_backlight_level = %d\n", panel->lge.fp_hbm_mode_backlight_lvl);
			}
		}
	}
	pr_info("use_fp_lhbm=%d\n", panel->lge.use_fp_lhbm);

	panel->lge.use_damping_mode = of_property_read_bool(of_node, "lge,use-damping-mode");
	pr_info("use_damping_mode=%d\n", panel->lge.use_damping_mode);

	panel->lge.use_tc_perf = of_property_read_bool(of_node, "lge,use-tc-perf");
	pr_info("use_tc_perf=%d\n", panel->lge.use_tc_perf);

	panel->lge.use_cm_lut = of_property_read_bool(of_node, "lge,use-color-manager-lut");
	pr_info("use color mananger lut supported=%d\n", panel->lge.use_cm_lut);

	if (panel->lge.use_cm_lut) {
		panel->lge.cm_lut_cnt = of_property_count_strings(of_node, "lge,cm-lut-screen-mode-set-name");
		if (panel->lge.cm_lut_cnt) {
			panel->lge.cm_lut_name_list = kzalloc(panel->lge.cm_lut_cnt * 8, GFP_KERNEL);
			if (!panel->lge.cm_lut_name_list) {
				pr_err("Unable to cm_lut_name_list alloc fail\n");
			} else {
				of_property_read_string_array(of_node, "lge,cm-lut-screen-mode-set-name", panel->lge.cm_lut_name_list, panel->lge.cm_lut_cnt);
			}
		}
		lge_ddic_dsi_panel_parse_cm_lut_cmd_sets(panel, of_node);
	}
	return rc;
}

int lge_dsi_panel_get(struct dsi_panel *panel, struct device_node *of_node)
{
	int rc = 0;

	rc = lge_dsi_panel_parse_gpios(panel, of_node);
	if (rc)
		pr_err("failed to parse panel gpios, rc=%d\n", rc);

	rc = lge_dsi_panel_parse_blmap(panel, of_node);
	if (rc)
		pr_err("failed to parse blmap, rc=%d\n", rc);

	rc = lge_dsi_panel_parse_brightness(panel, of_node);
	if (rc)
		pr_err("failed to parse default brightness, rc=%d\n", rc);

	rc = lge_dsi_panel_parse_dt(panel, of_node);
	if (rc)
		pr_err("failed to parse dt, rc=%d\n", rc);

	rc = lge_ddic_dsi_panel_parse_cmd_sets(panel, of_node);
	if (rc)
		pr_err("failed to parse ddic cmds sets, rc=%d\n", rc);

	lge_ddic_ops_init(panel);

	lge_ddic_feature_init(panel);

	/* TO DO */
	if (lge_drs_mngr_is_enabled(panel)) {
		if (lge_drs_mngr_init(panel) < 0)
			pr_warn("failed to initialize drs mngr\n");
	}

	if (panel->lge.use_color_manager) {
		pr_info("default cm_preset_step 2\n");
		panel->lge.cm_preset_step = 2;

		if (panel->lge.use_bc_dimming_work)
			lge_bc_dim_work_init(panel);
	}

	INIT_DELAYED_WORK(&panel->lge.panel_dead_work, lge_mdss_panel_dead_work);

	return rc;
}

inline void lge_dsi_panel_pin_seq_deinit(struct lge_panel_pin_seq **pseq)
{
	struct lge_panel_pin_seq *seq = *pseq;
	if (seq) {
		*pseq = NULL;
		kfree(seq);
	}
}

static void lge_dsi_panel_pins_deinit(struct dsi_panel *panel)
{
	if (panel->lge.pins_num && panel->lge.pins) {
		panel->lge.pins_num = 0;
		kfree(panel->lge.pins);
		lge_dsi_panel_pin_seq_deinit(&panel->lge.panel_on_seq);
		lge_dsi_panel_pin_seq_deinit(&panel->lge.panel_off_seq);
	}
}

void lge_dsi_panel_put(struct dsi_panel *panel)
{
	lge_dsi_panel_blmap_free(panel);
	lge_dsi_panel_pins_deinit(panel);
}

int lge_dsi_panel_parse_cmd_state(struct device_node *of_node, const char *name, enum dsi_cmd_set_state *pstate)
{
	int rc = 0;
	const char *state;

	if (pstate == NULL)
		return -EINVAL;

	state = of_get_property(of_node, name, NULL);
	if (state) {
		if (!strcmp(state, "dsi_lp_mode")) {
			*pstate = DSI_CMD_SET_STATE_LP;
		} else if (!strcmp(state, "dsi_hs_mode")) {
			*pstate = DSI_CMD_SET_STATE_HS;
		}
	} else {
		pr_warn("%s is not set", name);
		rc = -EINVAL;
	}

	return rc;
}

int lge_dsi_panel_parse_cmd_sets_sub(struct dsi_panel_cmd_set *cmd,
					const char *data,
					u32 length)
{
	int rc = 0;
	u32 packet_count = 0;

	rc = dsi_panel_get_cmd_pkt_count(data, length, &packet_count);
	if (rc) {
		pr_err("commands failed, rc=%d\n", rc);
		goto error;
	}

	rc = dsi_panel_alloc_cmd_packets(cmd, packet_count);
	if (rc) {
		pr_err("failed to allocate cmd packets, rc=%d\n", rc);
		goto error;
	}

	rc = dsi_panel_create_cmd_packets(data, length, packet_count,
					  cmd->cmds);
	if (rc) {
		pr_err("failed to create cmd packets, rc=%d\n", rc);
		goto error_free_mem;
	}

	return rc;
error_free_mem:
	kfree(cmd->cmds);
	cmd->cmds = NULL;
error:
	return rc;
}

int lge_dsi_panel_tx_cmd_set(struct dsi_panel *panel,
				struct dsi_panel_cmd_set *cmd)
{
	int rc = 0, i = 0, j=0;
	ssize_t len;
	struct dsi_cmd_desc *cmds;

	u32 count;
	enum dsi_cmd_set_state state;
	const struct mipi_dsi_host_ops *ops = panel->host->ops;


	if (!panel || !panel->cur_mode)
		return -EINVAL;

	cmds = cmd->cmds;
	count = cmd->count;
	state = cmd->state;

	if (count == 0) {
		pr_debug("[%s] No commands to be sent\n",
			 panel->name);
		goto error;
	}

	for (i = 0; i < count; i++) {
		/* TODO:  handle last command */
		if (state == DSI_CMD_SET_STATE_LP)
			cmds->msg.flags |= MIPI_DSI_MSG_USE_LPM;

		if (cmds->last_command)
			cmds->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;

		len = ops->transfer(panel->host, &cmds->msg);
		if (len < 0) {
			rc = len;
			pr_err("failed to set cmds, rc=%d\n", rc);
			goto error;
		}
		if (cmds->post_wait_ms)
			usleep_range(cmds->post_wait_ms * 1000, ((cmds->post_wait_ms * 1000) + 10));
		for(j=0; j < cmd->cmds[i].msg.tx_len; j++)
		{
			pr_debug("0x%02x send\n", (*(u8 *)(cmd->cmds[i].msg.tx_buf+j)));
		}
		cmds++;
	}
error:
	return rc;
}

int lge_mdss_dsi_panel_cmd_read(struct dsi_panel *panel, u8 cmd, int cnt, char* ret_buf)
{
	u8 rx_buf[256] = {0x0};
	int i = 0, ret = 0, checksum = 0;
	const struct mipi_dsi_host_ops *ops;

	struct dsi_cmd_desc cmds = {
		.msg = {
			.channel = 0,
			.type = MIPI_DSI_DCS_READ,
			.tx_buf = &cmd,
			.tx_len = 1,
			.rx_buf = &rx_buf[0],
			.rx_len = cnt,
			.flags = MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_LASTCOMMAND | MIPI_DSI_MSG_REQ_ACK
		},
		.last_command = false,
		.post_wait_ms = 0,
	};

	/* TO DO : panel connection check */
	/* if (not_connected) return -EINVAL */

	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	ops = panel->host->ops;
	ret = ops->transfer(panel->host, &cmds.msg);

	for (i = 0; i < cnt; i++)
		checksum += rx_buf[i];

	pr_info("[Reg:0x%02x] checksum=%d\n", cmd, checksum);

	memcpy(ret_buf, rx_buf, cnt);

	return checksum;
}

int lge_mdss_dsi_panel_cmds_backup(struct dsi_panel *panel, char *owner,
				enum dsi_cmd_set_type type, u8 reg, int nth_cmds)
{
	struct dsi_cmd_desc *cmds = NULL;
	u32 count;
	char *payload;

	if (!strncmp(owner, "lge", 3)) {
		cmds = panel->lge.lge_cmd_sets[type].cmds;
		count = panel->lge.lge_cmd_sets[type].count;
	} else if (!strncmp(owner, "qct", 3)) {
		cmds = panel->cur_mode->priv_info->cmd_sets[type].cmds;
		count = panel->cur_mode->priv_info->cmd_sets[type].count;
	} else if (!strncmp(owner, "dummy", 5)) {
		mdelay(17);
		return 0;
	} else {
		pr_err("owner is not defined\n");
		return -EINVAL;
	}

	if (count == 0) {
		pr_warn("lge (%d) cmds is not founded\n", type);
		return -EINVAL;
	} else {
		int mipi_read_max = 5;
		int update_count;
		struct dsi_cmd_desc *update_cmd = NULL;

		mutex_lock(&panel->panel_lock);
		if (panel->lge.use_ddic_reg_lock)
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_UNLOCK);
		update_cmd = find_nth_cmd(cmds, count, reg, nth_cmds);
		if (update_cmd) {
			payload = (char *)update_cmd->msg.tx_buf;
			update_count = (int)update_cmd->msg.tx_len - 1;
			payload++;
			while (mipi_read_max) {
				if (lge_mdss_dsi_panel_cmd_read(panel, reg, update_count, payload) > 0) {
					break;
				} else {
					pr_warn("try_again\n");
					if (panel->lge.use_ddic_reg_lock)
						lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_UNLOCK);
					mipi_read_max--;
					if (mipi_read_max == 0) {
						mutex_unlock(&panel->panel_lock);
						panic("read fail - reboot");
						return -EINVAL;
					}
				}
				usleep_range(1000, 1100);
			}
		} else {
			pr_warn("cmd for addr 0x%02x not found\n", reg);
		}
		if (panel->lge.use_ddic_reg_lock)
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_LOCK);
		mutex_unlock(&panel->panel_lock);
	}

	return 0;
}
