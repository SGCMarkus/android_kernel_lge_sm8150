#define pr_fmt(fmt)	"[Display][r66456-ops:%s:%d] " fmt, __func__, __LINE__

#include "dsi_panel.h"
#include "lge_ddic_ops_helper.h"
#include "cm/lge_color_manager.h"
#include "brightness/lge_brightness_def.h"

#define DDIC_REG_DEBUG 0

#define CM_HUE_INDEX 6
#define CM_SATURATION_INDEX 24
#define SHARPNESS_OFF 0x32
#define PRESET_SETP0_INDEX 0
#define PRESET_SETP1_INDEX 6
#define PRESET_SETP2_INDEX 12
#define STEP_DG_PRESET 5
#define RGB_OFFSET 3

#define FP_LHBM_DDIC_ON				0x13
#define FP_LHBM_DDIC_OFF			0x11

#define FP_LHBM_AOD_IRC_OFF			0x0C
#define FP_LHBM_AOD_IRC_ON			0x0D

#define ADDR_PTLAR 0x30
#define WORDS_TO_BYTE_ARRAY(w1, w2, b) do {\
		b[0] = WORD_UPPER_BYTE(w1);\
		b[1] = WORD_LOWER_BYTE(w1);\
		b[2] = WORD_UPPER_BYTE(w2);\
		b[3] = WORD_LOWER_BYTE(w2);\
} while(0)

static int rgb_preset[STEP_DG_PRESET][RGB_ALL] = {
	{PRESET_SETP2_INDEX, PRESET_SETP0_INDEX, PRESET_SETP2_INDEX},
	{PRESET_SETP1_INDEX, PRESET_SETP0_INDEX, PRESET_SETP1_INDEX},
	{PRESET_SETP0_INDEX, PRESET_SETP0_INDEX, PRESET_SETP0_INDEX},
	{PRESET_SETP0_INDEX, PRESET_SETP1_INDEX, PRESET_SETP0_INDEX},
	{PRESET_SETP0_INDEX, PRESET_SETP2_INDEX, PRESET_SETP0_INDEX}
};

extern int lge_ddic_dsi_panel_tx_cmd_set(struct dsi_panel *panel,
				enum lge_ddic_dsi_cmd_set_type type);
extern int lge_backlight_device_update_status(struct backlight_device *bd);
extern char* get_payload_addr(struct dsi_panel *panel, enum lge_ddic_dsi_cmd_set_type type, int position);
extern int get_payload_cnt(struct dsi_panel *panel, enum lge_ddic_dsi_cmd_set_type type, int position);

struct r66456_iq_data {
	char mode[8];
};

static struct r66456_iq_data r66456_color_space_value[8] = {
	{"def"},
	{"cin"},
	{"pho"},
	{"web"},
	{"spo"},
	{"gam"},
	{"exp"},
	{"hdr"},
};

const struct drs_res_info r66456_res[1] = {
	{"fhd", 0, 1080, 2340},
};

static void dump_cm_dummy_reg_value(struct dsi_panel *panel)
{
	int i = 0;
	char *payload = NULL;
	int length = 0;

	length = get_payload_cnt(panel, LGE_DDIC_DSI_DISP_CM_SET, 0);
	pr_info("CM-DEBUG] =========================\n");
	for (i = 0; i < length; i++) {
		payload = get_payload_addr(panel, LGE_DDIC_DSI_DISP_CM_COMMAND_DUMMY, i);
		if (!payload) {
			pr_err("null ptr \n");
			return;
		}

		pr_info("[%d] 0x%02x : 0x%02x\n", i, *payload, *(payload + 1));
	}
	pr_info("CM-DEBUG] =========================\n");
}

static void adjust_roi(struct dsi_panel *panel, int *sr, int *er)
{
	u32 cur_res = panel->cur_mode->timing.v_active;
	int type, num = panel->num_timing_nodes;

	for (type = 0; type < num; type++) {
		if (cur_res == r66456_res[type].height)
			break;
	}
	if (type == num) {
		pr_err("invalid height\n");
		*sr = 0;
		*er = panel->cur_mode->timing.h_active - 1;
		return;
	}

	if ((panel->lge.aod_area.w == 0) || (panel->lge.aod_area.h == 0)) {
		pr_err("area (w=%d)(h=%d), please check with application\n",
				panel->lge.aod_area.w, panel->lge.aod_area.h);
		goto full_roi;
	}

	*sr = panel->lge.aod_area.y;
	*er = *sr + panel->lge.aod_area.h - 1;

	return;

full_roi:
	*sr = 0;
	*er = *sr + r66456_res[0].height - 1;
	return;
}

static void prepare_cmd(struct dsi_cmd_desc *cmds, int cmds_count, int addr, int param1, int param2)
{
	struct dsi_cmd_desc *cmd = NULL;
	char *payload = NULL;

	cmd = find_cmd(cmds, cmds_count, addr);
	if (cmd) {
		payload = (char *)cmd->msg.tx_buf;
		payload++;
		WORDS_TO_BYTE_ARRAY(param1, param2, payload);
	} else {
		pr_warn("cmd for addr 0x%02X not found\n", addr);
	}
}

static void prepare_power_optimize_cmds(struct dsi_panel *panel, struct dsi_cmd_desc *cmds, int cmds_count, bool optimize)
{
	/* To Do */
	/*
	struct dsi_cmd_desc *cmd = NULL;
	char *payload = NULL;
	*/
	pr_info("%s:%s\n", __func__, (optimize ? "set" : "unset"));
}

static void prepare_aod_area_r66456(struct dsi_panel *panel, struct dsi_cmd_desc *cmds, int cmds_count)
{
	int sr = 0, er = 0;

	if (panel == NULL || cmds == NULL || cmds_count == 0) {
		pr_err("null ptr \n");
		return;
	}

	adjust_roi(panel, &sr, &er);
	prepare_cmd(cmds, cmds_count, ADDR_PTLAR, sr, er);

	return;
}

static int prepare_aod_cmds_r66456(struct dsi_panel *panel, struct dsi_cmd_desc *cmds, int cmds_count)
{
	int rc = 0;

	if (panel == NULL || cmds == NULL || cmds_count == 0) {
		pr_err("null ptr \n");
		return -EINVAL;
	}

	if (panel->lge.aod_power_mode &&
			(panel->lge.aod_area.h != r66456_res[0].height)) {
		prepare_power_optimize_cmds(panel, cmds, cmds_count, true);
	} else {
		prepare_power_optimize_cmds(panel, cmds, cmds_count, false);
	}

	return rc;
}

static int update_color_table(struct dsi_panel *panel, int idx)
{
	char *dst = NULL;
	char *src = NULL;
	int cnt = 0;

	dst = get_payload_addr(panel, LGE_DDIC_DSI_DISP_CM_COMMAND_DUMMY, 1);
	src = get_payload_addr(panel, LGE_DDIC_DSI_DISP_CM_SET, idx);
	cnt = get_payload_cnt(panel, LGE_DDIC_DSI_DISP_CM_COMMAND_DUMMY, 1);
	if (src == NULL || dst == NULL) {
		pr_err("null ptr \n");
		return -EINVAL;
	}
	memcpy(dst, src, cnt);

	return 0;
}

static void color_space_mapper(struct dsi_panel *panel, char *mode)
{
	int i = 0;
	int max_mode = sizeof(r66456_color_space_value)/sizeof(struct r66456_iq_data);

	for (i = 0; i < max_mode; i++) {
		if(!strncmp(mode, r66456_color_space_value[i].mode, 3)) {
			if(update_color_table(panel, i) < 0) {
				pr_err("failed to update color mode table\n");
				return;
			}
		}
	}
}

static void custom_rgb_mapper(struct dsi_panel *panel)
{
	char *dst = NULL;
	char *src_r = NULL;
	char *src_g = NULL;
	char *src_b = NULL;
	int rs = 0;
	int gs = 0;
	int bs = 0;

	if (panel == NULL) {
		pr_err("null ptr \n");
		return;
	}
	rs = rgb_preset[panel->lge.cm_preset_step][RED] + panel->lge.cm_red_step;
	gs = rgb_preset[panel->lge.cm_preset_step][GREEN] + panel->lge.cm_green_step;
	bs = rgb_preset[panel->lge.cm_preset_step][BLUE] + panel->lge.cm_blue_step;

	dst = get_payload_addr(panel, LGE_DDIC_DSI_DISP_CM_COMMAND_DUMMY, 1);
	src_r = get_payload_addr(panel, LGE_DDIC_DSI_RGB_LUT, rs);
	src_g = get_payload_addr(panel, LGE_DDIC_DSI_RGB_LUT, gs);
	src_b = get_payload_addr(panel, LGE_DDIC_DSI_RGB_LUT, bs);

	if (dst == NULL || src_r == NULL || src_g == NULL || src_b == NULL) {
		pr_err("null ptr \n");
		return;
	}

	dst[RGB_OFFSET+RED]= src_r[RED];
	dst[RGB_OFFSET+GREEN]= src_g[GREEN];
	dst[RGB_OFFSET+BLUE]= src_b[BLUE];

	return;
}

static void screen_tune_mapper(struct dsi_panel *panel)
{
	char *dst_org = NULL;
	char *dst_hue = NULL;
	char *dst_sat = NULL;
	char *src_hue = NULL;
	char *src_sat = NULL;
	int cnt_hue = 0;
	int cnt_sat = 0;
	int hue_step = 0;
	int sat_step = 0;

	if (panel == NULL) {
		pr_err("null ptr\n");
		return;
	}

	hue_step = panel->lge.sc_hue_step;
	sat_step = panel->lge.sc_sat_step;

	dst_org = get_payload_addr(panel, LGE_DDIC_DSI_DISP_CM_COMMAND_DUMMY, 1);
	src_hue = get_payload_addr(panel, LGE_DDIC_DSI_DISP_RGB_HUE_LUT, hue_step);
	cnt_hue = get_payload_cnt(panel, LGE_DDIC_DSI_DISP_RGB_HUE_LUT, hue_step);
	src_sat = get_payload_addr(panel, LGE_DDIC_DSI_DISP_SAT_LUT, sat_step);
	cnt_sat = get_payload_cnt(panel, LGE_DDIC_DSI_DISP_SAT_LUT, sat_step);

	if (dst_org == NULL || src_hue == NULL || src_sat == NULL || cnt_hue == 0 || cnt_sat == 0) {
		pr_err("null ptr \n");
		return;
	}

	dst_hue = dst_org + CM_HUE_INDEX;
	dst_sat = dst_org + CM_SATURATION_INDEX;

	memcpy(dst_hue, src_hue, cnt_hue);
	memcpy(dst_sat, src_sat, cnt_sat);

	return;
}

static void screen_tune_sharpness(struct dsi_panel *panel, bool status)
{
	char *dst = NULL;
	char *src = NULL;
	int cnt = 0;
	int sharp_step = 0;

	if(panel == NULL) {
		pr_err("null ptr\n");
		return;
	}
	sharp_step = panel->lge.sc_sha_step;
	dst = get_payload_addr(panel, LGE_DDIC_DSI_SET_SHARPNESS, 1);
	src = get_payload_addr(panel, LGE_DDIC_DSI_DISP_SHA_LUT, sharp_step);
	cnt = get_payload_cnt(panel, LGE_DDIC_DSI_DISP_SHA_LUT, sharp_step);

	if (dst == NULL || src == NULL || cnt == 0) {
		pr_err("null ptr \n");
		return;
	}

	if (status == true) {
		memcpy(dst, src, cnt);
	} else {
		dst[1] = SHARPNESS_OFF;
	}

	return;
}

static void lge_display_control_store_r66456(struct dsi_panel *panel, bool send_cmd)
{
	if(!panel) {
		pr_err("panel not exist\n");
		return;
	}

	mutex_lock(&panel->panel_lock);

	if (panel->lge.hdr_mode) {
		color_space_mapper(panel, "hdr");
	} else {
		switch (panel->lge.screen_mode) {
		case screen_mode_sports:
			color_space_mapper(panel, "spo");
			break;
		case screen_mode_game:
			color_space_mapper(panel, "gam");
			break;
		case screen_mode_cinema:
			color_space_mapper(panel, "cin");
			break;
		case screen_mode_photos:
			color_space_mapper(panel, "pho");
			break;
		case screen_mode_web:
			color_space_mapper(panel, "web");
			break;
		case screen_mode_expert:
		case screen_mode_auto:
		default:
			if (panel->lge.screen_mode == screen_mode_expert) {
				color_space_mapper(panel, "exp");
				screen_tune_mapper(panel);
			} else {
				color_space_mapper(panel, "def");
			}
			custom_rgb_mapper(panel);
			break;
		}
	}

	if (panel->lge.screen_mode == screen_mode_expert){
		screen_tune_sharpness(panel, true);
	} else {
		screen_tune_sharpness(panel, false);
	}

	if (send_cmd) {
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_DISP_CM_COMMAND_DUMMY);
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_SHARPNESS);
	}

	mutex_unlock(&panel->panel_lock);

	if(DDIC_REG_DEBUG)
		dump_cm_dummy_reg_value(panel);

	return;
}

static void lge_update_ddic_hdr_status(struct dsi_panel *panel)
{
	char *tune_dst = NULL;
	char *tune_src = NULL;
	int tune_cnt = 0;
	char *ace_mode_dst = NULL;
	char *dd_mode_dst = NULL;
	int ace_idx = 0;
	u8 ace_mode = 0xD0;
	u8 dd_mode = 0x08;
	int rc = 0;

	if(!panel) {
		pr_err("null ptr\n");
		return;
	}

	panel->lge.ddic_hdr = !!(panel->lge.ve_hdr | panel->lge.ace_hdr | panel->lge.damping_hdr);

	if (panel->lge.ace_hdr) {
		ace_idx = 1;
	} else if (panel->lge.ve_hdr || panel->lge.damping_hdr){
		ace_idx = 0;
	} else {
		ace_mode = 0x00;
		dd_mode = 0x00;
	}

	tune_dst = get_payload_addr(panel, LGE_DDIC_DSI_ACE_TUNE, 1);
	tune_src = get_payload_addr(panel, LGE_DDIC_DSI_ACE_LUT, ace_idx);
	tune_cnt = get_payload_cnt(panel, LGE_DDIC_DSI_ACE_TUNE, 1);
	if (tune_src == NULL || tune_dst == NULL) {
		pr_err("null ptr \n");
		return;
	}
	memcpy(tune_dst, tune_src, tune_cnt);

	ace_mode_dst = get_payload_addr(panel, LGE_DDIC_DSI_ACE_TUNE, 3);
	if (ace_mode_dst == NULL) {
		pr_err("null ptr \n");
		return;
	}
	ace_mode_dst[1] = ace_mode;

	dd_mode_dst = get_payload_addr(panel, LGE_DDIC_DSI_ACE_TUNE, 4);
	if (dd_mode_dst == NULL) {
		pr_err("null ptr \n");
		return;
	}
	dd_mode_dst[1] = dd_mode;

	rc = lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_ACE_TUNE);
	if (rc)
		pr_err("failed to send LGE_DDIC_DSI_ACE_TUNE cmd, rc=%d\n", rc);

	pr_info("hdr %d ve_hdr %d ace_hdr %d damp_hdr %d \n",
			panel->lge.ddic_hdr, panel->lge.ve_hdr, panel->lge.ace_hdr, panel->lge.damping_hdr);
}

static void lge_set_video_enhancement_r66456(struct dsi_panel *panel, int input)
{
	mutex_lock(&panel->panel_lock);
	panel->lge.ve_hdr = !!input;
	lge_update_ddic_hdr_status(panel);
	mutex_unlock(&panel->panel_lock);

	pr_info("send cmds to %d video enhancer \n", panel->lge.ve_hdr );

	lge_backlight_device_update_status(panel->bl_config.raw_bd);
}

static void lge_vr_lp_mode_set_r66456(struct dsi_panel *panel, int input) {
	int rc = 0;
	bool enable = false;

	mutex_lock(&panel->panel_lock);

	panel->lge.vr_lp_mode = input;
	enable = !!panel->lge.vr_lp_mode;

	if (enable) {
		rc = lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_VR_MODE_ON);
	} else {
		rc = lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_VR_MODE_POST_OFF);
	}
	mutex_unlock(&panel->panel_lock);

	pr_info("send cmds to %s vr_lp_set \n",	(input == true) ? "enable" : "disable");
}

static void lge_update_irc_state(struct dsi_panel *panel, int pos, int value)
{
	int *cur_state = &(panel->lge.irc_current_state);

	if (value)
		*cur_state |= BIT(pos);
	else
		*cur_state &= ~BIT(pos);

	pr_info("success (%d), (%d)\n", *cur_state, panel->lge.irc_current_state);

	return;
}

int lge_set_irc_state_r66456(struct dsi_panel *panel, enum lge_irc_mode mode, enum lge_irc_ctrl enable)
{
	char *dst = NULL;
	int prev_state = !!panel->lge.irc_current_state;
	int new_state;

	pr_info("irc request=%s\n", ((enable == LGE_IRC_OFF) ? "off" : "on"));

	if(panel == NULL) {
		pr_err("null ptr\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	lge_update_irc_state(panel, mode, enable);
	new_state = !!panel->lge.irc_current_state;

	if(!panel->lge.use_irc_ctrl) {
		pr_info("go to ace set\n");
		goto ace_set;
	}

	if (prev_state == new_state) {
		pr_info("same state, skip=(%d,%d)\n", prev_state, new_state);
		goto ace_set;
	}

	dst = get_payload_addr(panel, LGE_DDIC_DSI_IRC_CTRL, 1);
	if (dst == NULL) {
		pr_err("get irc failed\n");
		mutex_unlock(&panel->panel_lock);
		return 0;
	}

	if (enable == LGE_IRC_OFF)
		dst[1] = 0x00; /* Global IRC OFF */
	else
		dst[1] = 0x01; /* Global IRC ON */

	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_IRC_CTRL);

ace_set:
	panel->lge.irc_pending = false;
	if ((mode == LGE_GLOBAL_IRC_HBM) && panel->lge.use_ace_ctrl) {
		if (enable == LGE_IRC_OFF) {
			panel->lge.ace_hdr = 1;
			lge_update_ddic_hdr_status(panel);
		} else {
			panel->lge.ace_hdr = 0;
			lge_update_ddic_hdr_status(panel);
		}
	}

	mutex_unlock(&panel->panel_lock);
	return 0;
}

int lge_get_irc_state_r66456(struct dsi_panel *panel)
{
	int ret;

	mutex_lock(&panel->panel_lock);
	pr_info("current_state=%d\n", panel->lge.irc_current_state);
	ret = !!panel->lge.irc_current_state;
	mutex_unlock(&panel->panel_lock);
	return ret;
}

int lge_set_irc_default_state_r66456(struct dsi_panel *panel)
{
	lge_update_irc_state(panel, LGE_GLOBAL_IRC_HBM, LGE_IRC_ON);
	lge_update_irc_state(panel, LGE_GLOBAL_IRC_HDR, LGE_IRC_ON);
	return 0;
}

int hdr_mode_set_r66456(struct dsi_panel *panel, int input)
{
	bool hdr_mode = ((input > 0) ? true : false);

	lge_display_control_store_r66456(panel, true);

	if (panel->lge.use_irc_ctrl) {
		if (hdr_mode) {
			lge_set_irc_state_r66456(panel, LGE_GLOBAL_IRC_HDR, LGE_IRC_OFF);
		} else {
			lge_set_irc_state_r66456(panel, LGE_GLOBAL_IRC_HDR, LGE_IRC_ON);
		}
	}

	lge_backlight_device_update_status(panel->bl_config.raw_bd);
	return 0;
}

static void lge_set_screen_mode_r66456(struct dsi_panel *panel, bool send_cmd)
{
	lge_display_control_store_r66456(panel, true);
    return;
}

static void lge_set_screen_tune_r66456(struct dsi_panel *panel)
{
	lge_display_control_store_r66456(panel, true);
	return;
}

static void lge_set_rgb_tune_r66456(struct dsi_panel *panel, bool send_cmd)
{
	lge_display_control_store_r66456(panel, true);
	return;
}

static void sharpness_set_r66456(struct dsi_panel *panel, int input)
{
	lge_display_control_store_r66456(panel, true);
}

static void lge_set_fp_lhbm_r66456(struct dsi_panel *panel, int input)
{
	char *payload = NULL;
	char *payload_aod = NULL;
	bool fp_aod_ctrl = false;
	u32 aod_wait_ms = 15;

	mutex_lock(&panel->panel_lock);
	fp_aod_ctrl = lge_dsi_panel_is_power_on_lp(panel);

	if (((input == panel->lge.old_fp_lhbm_mode) || (input == panel->lge.old_panel_fp_mode)) &&
			(LGE_FP_LHBM_SM_ON != panel->lge.old_fp_lhbm_mode) &&
			(LGE_FP_LHBM_FORCED_ON != panel->lge.old_fp_lhbm_mode)) {
		mutex_unlock(&panel->panel_lock);
		pr_info("requested same state=%d lhbm:%d panel:%d\n", input, panel->lge.old_fp_lhbm_mode, panel->lge.old_panel_fp_mode);
		return;
	}

	payload = get_payload_addr(panel, LGE_DDIC_DSI_FP_LHBM_COMMAND, 1);
	payload_aod = get_payload_addr(panel, LGE_DDIC_DSI_FP_LHBM_AOD_COMMAND, 1);

	if (!payload) {
		pr_err("cmd null\n");
		mutex_unlock(&panel->panel_lock);
		return;
	}

	switch (input) {
	case LGE_FP_LHBM_FORCED_ON:
		if (fp_aod_ctrl && payload_aod) {
			payload_aod[30] = FP_LHBM_AOD_IRC_ON;
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_AOD_COMMAND);
		}
		if(!fp_aod_ctrl)
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_IRC_SET);
		payload[1] = FP_LHBM_DDIC_ON;
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_COMMAND);
		if (fp_aod_ctrl)
			usleep_range(aod_wait_ms*1000, ((aod_wait_ms*1000)+10));
		panel->lge.old_fp_lhbm_mode = LGE_FP_LHBM_FORCED_ON;
		panel->lge.forced_lhbm = true;
		break;
	case LGE_FP_LHBM_FORCED_OFF:
		if(!fp_aod_ctrl)
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_NORM_IRC_SET);
		payload[1] = FP_LHBM_DDIC_OFF;
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_COMMAND);
		if (fp_aod_ctrl)
			usleep_range(aod_wait_ms*1000, ((aod_wait_ms*1000)+10));
		if (fp_aod_ctrl && payload_aod) {
			payload_aod[30] = FP_LHBM_AOD_IRC_OFF;
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_AOD_COMMAND);
		}
		panel->lge.old_fp_lhbm_mode = LGE_FP_LHBM_FORCED_OFF;
		panel->lge.forced_lhbm = false;
		break;
	case LGE_FP_LHBM_SM_ON:
		if (fp_aod_ctrl && payload_aod) {
			payload_aod[30] = FP_LHBM_AOD_IRC_ON;
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_AOD_COMMAND);
		}
		if(!fp_aod_ctrl)
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_IRC_SET);
		payload[1] = FP_LHBM_DDIC_ON;
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_COMMAND);
		if (fp_aod_ctrl)
			usleep_range(aod_wait_ms*1000, ((aod_wait_ms*1000)+10));
		panel->lge.old_fp_lhbm_mode = LGE_FP_LHBM_SM_ON;
		break;
	case LGE_FP_LHBM_SM_OFF:
		if(!fp_aod_ctrl)
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_NORM_IRC_SET);
		payload[1] = FP_LHBM_DDIC_OFF;
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_COMMAND);
		if (fp_aod_ctrl)
			usleep_range(aod_wait_ms*1000, ((aod_wait_ms*1000)+10));
		if (fp_aod_ctrl && payload_aod) {
			payload_aod[30] = FP_LHBM_AOD_IRC_OFF;
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_AOD_COMMAND);
		}
		panel->lge.old_fp_lhbm_mode = LGE_FP_LHBM_SM_OFF;
		panel->lge.forced_lhbm = false;
		break;
	case LGE_FP_LHBM_ON:
		if (panel->lge.old_panel_fp_mode != LGE_FP_LHBM_READY) {
			if (fp_aod_ctrl && payload_aod) {
				payload_aod[30] = FP_LHBM_AOD_IRC_ON;
				lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_AOD_COMMAND);
			}
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_READY);
			if (fp_aod_ctrl)
				usleep_range(aod_wait_ms*1000, ((aod_wait_ms*1000)+10));
			panel->lge.old_panel_fp_mode = LGE_FP_LHBM_READY;
		}
		payload[1] = FP_LHBM_DDIC_ON;
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_COMMAND);
		if (fp_aod_ctrl)
			usleep_range(aod_wait_ms*1000, ((aod_wait_ms*1000)+10));
		panel->lge.old_fp_lhbm_mode = LGE_FP_LHBM_ON;
		break;
	case LGE_FP_LHBM_READY:
		if (fp_aod_ctrl && payload_aod) {
			payload_aod[30] = FP_LHBM_AOD_IRC_ON;
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_AOD_COMMAND);
		}
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_READY);
		if (fp_aod_ctrl)
			usleep_range(aod_wait_ms*1000, ((aod_wait_ms*1000)+10));
		panel->lge.old_panel_fp_mode = LGE_FP_LHBM_READY;
		break;
	case LGE_FP_LHBM_OFF:
		payload[1] = FP_LHBM_DDIC_OFF;
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_COMMAND);
		if (fp_aod_ctrl)
			usleep_range(aod_wait_ms*1000, ((aod_wait_ms*1000)+10));
		panel->lge.old_fp_lhbm_mode = LGE_FP_LHBM_OFF;
		panel->lge.forced_lhbm = false;
		break;
	case LGE_FP_LHBM_EXIT:
		panel->lge.forced_lhbm = false;
	case LGE_FP_LHBM_FORCED_EXIT:
	default:
		if ((panel->lge.old_fp_lhbm_mode == LGE_FP_LHBM_ON) ||
			(panel->lge.old_fp_lhbm_mode == LGE_FP_LHBM_SM_ON) ||
			(panel->lge.old_fp_lhbm_mode == LGE_FP_LHBM_FORCED_ON)) {
			payload[1] = FP_LHBM_DDIC_OFF;
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_COMMAND);
			if (fp_aod_ctrl)
				usleep_range(aod_wait_ms*1000, ((aod_wait_ms*1000)+10));
			panel->lge.old_fp_lhbm_mode = LGE_FP_LHBM_OFF;
		}
		if (fp_aod_ctrl && payload_aod) {
			payload_aod[30] = FP_LHBM_AOD_IRC_OFF;
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_AOD_COMMAND);
		}
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_EXIT);
		if (fp_aod_ctrl)
			usleep_range(aod_wait_ms*1000, ((aod_wait_ms*1000)+10));
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_EXIT_POST);
		if (fp_aod_ctrl)
			usleep_range(aod_wait_ms*1000, ((aod_wait_ms*1000)+10));
		panel->lge.old_panel_fp_mode = LGE_FP_LHBM_EXIT;
		break;
	}
	mutex_unlock(&panel->panel_lock);
	pr_info("set lhbm mode : set to %d, %d\n", panel->lge.old_fp_lhbm_mode, panel->lge.old_panel_fp_mode);

	return;
}

static void lge_set_fp_lhbm_br_lvl_r66456(struct dsi_panel *panel, int input)
{
	int req_br_lvl = FP_LHBM_DEFAULT_BR_LVL;
	char *payload = NULL;

	if(input > FP_LHBM_DEFAULT_BR_LVL)
		input = FP_LHBM_DEFAULT_BR_LVL;

	req_br_lvl = input;

	mutex_lock(&panel->panel_lock);
	payload = get_payload_addr(panel, LGE_DDIC_DSI_FP_LHBM_BR_LVL_COMMAND, 1);

	if (!payload) {
		pr_err("cmd null\n");
		mutex_unlock(&panel->panel_lock);
		return;
	}

	payload[3] = req_br_lvl & 0xFF;

	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_BR_LVL_COMMAND);
	mutex_unlock(&panel->panel_lock);
	pr_info("set lhbm color lvl : %d set to 0x%x \n", req_br_lvl, payload[3]);

	return;
}

static void lge_set_tc_perf_r66456(struct dsi_panel *panel, int input)
{
	mutex_lock(&panel->panel_lock);
	if (input)
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_TC_PERF_COMMAND);
	mutex_unlock(&panel->panel_lock);
	pr_info("set tc perf %s\n", (input) ? "enable" : "disable");

	return;
}

static void lge_damping_mode_set_r66456(struct dsi_panel *panel, int input) {
	mutex_lock(&panel->panel_lock);
	panel->lge.damping_hdr = !!input;
	lge_update_ddic_hdr_status(panel);
	mutex_unlock(&panel->panel_lock);

	pr_info("send cmds to %d damping_set \n", panel->lge.damping_hdr);
}

struct lge_ddic_ops r66456_ops = {
	.store_aod_area = store_aod_area,
	.prepare_aod_cmds = prepare_aod_cmds_r66456,
	.prepare_aod_area = prepare_aod_area_r66456,
	/* err check not supported */
	.lge_check_vert_black_line = NULL,
	.lge_check_vert_white_line = NULL,
	.lge_check_vert_line_restore = NULL,
	/* bc dim not supported */
	.lge_bc_dim_set = NULL,
	.lge_set_therm_dim = NULL,
	.lge_get_brightness_dim = NULL,
	.lge_set_brightness_dim = NULL,
	/* To Do : Image Quality */
	.hdr_mode_set = hdr_mode_set_r66456,
	.lge_set_rgb_tune = lge_set_rgb_tune_r66456,
	.lge_display_control_store = lge_display_control_store_r66456,
	.lge_set_screen_tune = lge_set_screen_tune_r66456,
	.lge_set_screen_mode = lge_set_screen_mode_r66456,
	.sharpness_set = sharpness_set_r66456,
	/* trueview not supported */
	.lge_set_true_view_mode = NULL,
	.lge_set_video_enhancement = lge_set_video_enhancement_r66456,
	.lge_vr_lp_mode_set = lge_vr_lp_mode_set_r66456,
	.lge_set_fp_lhbm = lge_set_fp_lhbm_r66456,
	.lge_set_fp_lhbm_br_lvl = lge_set_fp_lhbm_br_lvl_r66456,
	.lge_set_tc_perf = lge_set_tc_perf_r66456,
	.lge_damping_mode_set = lge_damping_mode_set_r66456,
	/* drs not supported */
	.get_current_res = NULL,
	.get_support_res = NULL,
	/* bist not supported */
	.bist_ctrl = NULL,
	.release_bist = NULL,
	/* error detect not supported */
	.err_detect_work = NULL,
	.err_detect_irq_handler = NULL,
	.set_err_detect_mask = NULL,
	/* pps not used */
	.set_pps_cmds = NULL,
	.unset_pps_cmds = NULL,
	/* irc not supported */
	.set_irc_default_state = lge_set_irc_default_state_r66456,
	.set_irc_state = lge_set_irc_state_r66456,
	.get_irc_state = lge_get_irc_state_r66456,
};
