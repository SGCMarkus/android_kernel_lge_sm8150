#define pr_fmt(fmt)	"[Display][ddic-ops:%s:%d] " fmt, __func__, __LINE__

#include "dsi_display.h"
#include "dsi_panel.h"
#include "err_detect/lge_err_detect.h"
#include "cm/lge_color_manager.h"

extern struct lge_ddic_ops sw43408a_ops;
extern struct lge_ddic_ops sw43410_ops;
extern struct lge_ddic_ops rm69299_ops;
extern struct lge_ddic_ops rm69299c_ops;
extern struct lge_ddic_ops rm692A9_ops;
extern struct lge_ddic_ops r66456_ops;

struct lge_ddic_match {
	char compatible[15];
	struct lge_ddic_ops *ops;
};

static struct lge_ddic_match supported_ddic_list[] = {
	{"sw43408a", &sw43408a_ops},
	{"sw43410", &sw43410_ops},
	{"rm69299a", &rm69299_ops}, /* type A : rm69299+tianma */
	{"rm69299b", &rm69299_ops}, /* type B : rm69299+tovisDD */
	{"rm69299c", &rm69299c_ops}, /* type C : rm69299+tovisMD */
	{"rm692A9", &rm692A9_ops}, /* type C : rm692A9+tovisMD */
	{"r66456", &r66456_ops}, /* r566456+tianmaMD */
	{"r66456a", &r66456_ops}, /* r566456+tianmaMD, D-CUT(DV2) */
};

extern char* get_ddic_name(void);
extern bool is_ddic_name(char *ddic_name);
extern int lge_mdss_dsi_panel_cmds_backup(struct dsi_panel *panel, char *owner,
				enum dsi_cmd_set_type type, u8 reg, int nth_cmds);

static struct backup_info* lge_ddic_panel_reg_backup_prepare(struct dsi_panel *panel, int *cnt)
{
	struct backup_info *list = NULL;

	if (!panel) {
		pr_err("panel is null\n");
		return list;
	}

	if (!panel->lge.ddic_ops->get_reg_backup_list) {
		pr_err("backup ops is not connected\n");
		return list;
	} else {
		list = panel->lge.ddic_ops->get_reg_backup_list(cnt);
		if (!list) {
			pr_err("fail to get list\n");
		}
	}

	return list;
}

static void lge_ddic_register_backup_work(struct work_struct *w)
{
	int count, ret, i = 0;
	struct backup_info *list = NULL;
	struct dsi_panel *panel = NULL;
	struct lge_dsi_panel *lge_panel = container_of(w, struct lge_dsi_panel, backup_work);
	struct dsi_display *display = NULL;

	if (!lge_panel) {
		pr_err("lge_panel is null\n");
		return;
	}

	panel = container_of(lge_panel, struct dsi_panel, lge);
	if (!panel) {
		pr_err("panel is null\n");
		return;
	}

	display = container_of(panel->host, struct dsi_display, host);
	if (!display)
		return;

	list = lge_ddic_panel_reg_backup_prepare(panel, &count);
	if (!list) {
		pr_err("list is empty\n");
		return;
	}

	for (i = 0; i < count ; i++) {
		if (list->is_backup)
			goto backup_done;

		mutex_lock(&display->display_lock);

		if (!lge_dsi_panel_is_power_on_interactive(display->panel)) {
			pr_err("power mode is not allowed\n");
			mutex_unlock(&display->display_lock);
			goto read_fail;
		}

		ret = lge_mdss_dsi_panel_cmds_backup(panel, list->owner,
				list->type, list->reg, list->nth_cmds);
		if (ret < 0) {
			pr_err("[%s]: read fail\n", list->name);
			list->is_backup = false;
			mutex_unlock(&display->display_lock);
			goto read_fail;
		} else {
			list->is_backup = true;
		}
		mutex_unlock(&display->display_lock);
backup_done:
		pr_info("[%s]: success\n", list->name);
		list++;
	}

	mutex_lock(&display->display_lock);
	if (!panel->lge.is_sent_bc_dim_set) {
		if (panel->lge.ddic_ops &&
				panel->lge.ddic_ops->lge_set_screen_mode)
			panel->lge.ddic_ops->lge_set_screen_mode(panel, true);

		if (panel->lge.ddic_ops &&
				panel->lge.ddic_ops->lge_bc_dim_set)
			panel->lge.ddic_ops->lge_bc_dim_set(panel, BC_DIM_ON, BC_DIM_FRAMES_NORMAL);
		panel->lge.is_sent_bc_dim_set = true;
	}
	mutex_unlock(&display->display_lock);

	panel->lge.ddic_reg_backup_complete = true;
	atomic_set(&panel->lge.backup_state, 0);
	return;

read_fail:
	return;
}

int lge_ddic_dsi_panel_reg_backup(struct dsi_panel *panel)
{
	if (!panel) {
		pr_err("panel is null\n");
		return -EINVAL;
	}

	if (atomic_read(&panel->lge.backup_state) > 0) {
		pr_info("backup work is already running\n");
		return 0;
	}
pr_info("test read  %d \n", panel->lge.backup_state);
	atomic_set(&panel->lge.backup_state, 1);
	schedule_work(&panel->lge.backup_work);

	return 0;
}

int lge_ddic_dsi_panel_reg_backup_reinit(struct dsi_panel *panel)
{
	int count = 0, i = 0;
	struct backup_info *list = NULL;

	list = lge_ddic_panel_reg_backup_prepare(panel, &count);
	if (!list)
		return -EINVAL;

	for (i = 0 ; i < count ; i++) {
		if (!strncmp(list->owner, "qct", 3)) {
			list->is_backup = false;
		}
		list++;
	}

	panel->lge.ddic_reg_backup_complete = false;

	return 0;
}

bool is_need_register_backup(struct dsi_panel *panel)
{
	if (!panel) {
		pr_err("panel is null\n");
		return false;
	}

pr_info("test read  %d \n", panel->lge.backup_state);

	return ((panel->lge.use_ddic_reg_backup) &&
			(!panel->lge.ddic_reg_backup_complete) &&
			(!atomic_read(&panel->lge.backup_state)));
}

void lge_ddic_ops_init(struct dsi_panel *panel)
{
	int i;
	int count = sizeof(supported_ddic_list)/sizeof(supported_ddic_list[0]);

	for (i = 0; i < count; ++i) {
		if (is_ddic_name(supported_ddic_list[i].compatible)) {
			panel->lge.ddic_ops = supported_ddic_list[i].ops;
			break;
		}
	}

	if (panel->lge.ddic_ops == NULL)
		pr_warn("no matched ddic ops for %s\n", get_ddic_name());

	mutex_init(&panel->lge.pa_changed_lock);
}

void lge_ddic_feature_init(struct dsi_panel *panel)
{
	if (panel->lge.use_bist) {
		panel->lge.bist_on = 0;
		mutex_init(&panel->lge.bist_lock);
	}

	if (panel->lge.use_ddic_reg_backup) {
		INIT_WORK(&panel->lge.backup_work, lge_ddic_register_backup_work);
		atomic_set(&panel->lge.backup_state, 0);
		panel->lge.ddic_reg_backup_complete = false;
	}

	if (panel->lge.use_panel_err_detect) {
		lge_panel_err_detect_init(panel);
	}

	if ((panel->lge.use_irc_ctrl || panel->lge.use_ace_ctrl) &&
			panel->lge.ddic_ops && panel->lge.ddic_ops->set_irc_default_state) {
		panel->lge.irc_current_state = 0;
		panel->lge.ddic_ops->set_irc_default_state(panel);
	}
}

bool is_bist_supported(struct dsi_panel *panel, const char type[])
{
	int i = 0;

	if (!panel || !panel->lge.use_bist)
		return false;

	for (i = 0; i < MAX_BIST_USAGE_TYPE; i++) {
		if (!strncmp(panel->lge.bist_usage_type[i], type, 3)) {
			return true;
		}
	}

	return false;
}

int store_aod_area(struct dsi_panel *panel, struct lge_rect rect)
{
	if (panel->lge.aod_area.y != rect.y)
		panel->lge.partial_area_vertical_changed = true;
	if (panel->lge.aod_area.h != rect.h)
		panel->lge.partial_area_height_changed = true;
	panel->lge.aod_area = rect;
	return 0;
}

struct dsi_cmd_desc* find_nth_cmd(struct dsi_cmd_desc *cmds, int cmds_count, int addr, int nth)
{
	struct dsi_cmd_desc *ret = NULL;
	int i;
	char *payload;

	for (i = 0; i < cmds_count; ++i) {
		payload = (char*)cmds[i].msg.tx_buf;
		if (payload[0] == addr) {
			if (--nth == 0) {
				ret = &cmds[i];
				break;
			}
		}
	}

	return ret;
}

struct dsi_cmd_desc* find_cmd(struct dsi_cmd_desc *cmds, int cmds_count, int addr)
{
	struct dsi_cmd_desc *ret = NULL;
	int i;
	char *payload;

	for (i = 0; i < cmds_count; ++i) {
		payload = (char*)cmds[i].msg.tx_buf;
		if (payload[0] == addr) {
			ret = &cmds[i];
			break;
		}
	}

	return ret;
}

extern int dsi_panel_get_cmd_pkt_count(const char *data, u32 length, u32 *cnt);
extern int dsi_panel_create_cmd_packets(const char *data,
					u32 length,
					u32 count,
					struct dsi_cmd_desc *cmd);
extern int dsi_panel_alloc_cmd_packets(struct dsi_panel_cmd_set *cmd,
					u32 packet_count);

char *lge_ddic_cmd_set_prop_map[LGE_DDIC_DSI_CMD_SET_MAX] = {
	"lge,mdss-dsi-ie-command",
	"lge,mdss-dsi-bist-on-command",
	"lge,mdss-dsi-bist-off-command",
	"lge,mdss-dsi-wb-default-command",
	"lge,mdss-dsi-cm-dci-p3-command",
	"lge,mdss-dsi-cm-srgb-command",
	"lge,mdss-dsi-cm-adobe-command",
	"lge,mdss-dsi-cm-native-command",
	"lge,mdss-dsi-disp-ctrl-command-1",
	"lge,mdss-dsi-disp-ctrl-command-2",
	"lge,digital-gamma-cmds-dummy",
	"lge,mdss-dsi-bc-dim-command",
	"lge,mdss-dsi-bc-default-command",
	"lge,mdss-dsi-vr-lp-mode-on-command",
	"lge,mdss-dsi-vr-pre-lp-mode-off-command",
	"lge,mdss-dsi-vr-post-lp-mode-off-command",
	"lge,mdss-dsi-lp1-command",
	"lge,mdss-dsi-lp2-command",
	"lge,mdss-dsi-nolp-command",
	"lge,mdss-dsi-saturation-command",
	"lge,mdss-dsi-hue-command",
	"lge,mdss-dsi-sharpness-command",
	"lge,mdss-dsi-saturation-command",
	"lge,mdss-dsi-hue-command",
	"lge,mdss-dsi-sharpness-command",
	"lge,mdss-dsi-cm-sports",
	"lge,mdss-dsi-cm-game",
	"lge,detect-vert-line-restore-command",
	"lge,detect-black-vert-line-command",
	"lge,detect-white-vert-line-command",
	"lge,memory-error-detect-command",
	"lge,esd-detect-command",
	"lge,line-defect-detect-command",
	"lge,selective-color-cmds-dummy-command",
	"lge,ddic-register-lock",
	"lge,ddic-register-unlock",
	"lge,mdss-dsi-ve-on-command",
	"lge,mdss-dsi-ve-off-command",
	"lge,mdss-dsi-hdr-set",
	"lge,mdss-dsi-irc-command",
	"lge,mdss-dsi-ace-tune-command",
	"lge,mdss-dsi-ace-restore-command",
	"lge,digital-gamma-set",
	"lge,mdss-dsi-aod-area-command",
	"lge,color-mode-cmds-dummy",
	"lge,color-mode-set",
	"lge,custom-rgb-hue-lut",
	"lge,saturation-lut",
	"lge,sharpness-lut",
	"lge,trueview-lut",
	"lge,ddic-dsi-br-ctrl-ext-command",
	"lge,mdss-dsi-fp-lhbm-command",
	"lge,mdss-dsi-fp-lhbm-br-lvl-command",
	"lge,lhbm-lut",
	"lge,mdss-dsi-tc-perf-command",
	"lge,rgb-lut",
	"lge,ace-lut",
	"lge,mdss-dsi-fp-lhbm-ready-command",
	"lge,mdss-dsi-fp-lhbm-exit-command",
	"lge,mdss-dsi-fp-lhbm-aod-command",
	"lge,mdss-dsi-fp-lhbm-exit-post-command",
	"lge,mdss-dsi-fp-lhbm-irc-command",
	"lge,mdss-dsi-fp-norm-irc-command",
};

char *lge_ddic_cmd_set_state_map[LGE_DDIC_DSI_CMD_SET_MAX] = {
	"lge,mdss-dsi-ie-command-state",
	"lge,mdss-dsi-bist-control-command-state",
	"lge,mdss-dsi-bist-control-command-state",
	"lge,mdss-dsi-wb-default-command-state",
	"lge,mdss-dsi-cm-dci-p3-command-state",
	"lge,mdss-dsi-cm-srgb-command-state",
	"lge,mdss-dsi-cm-adobe-command-state",
	"lge,mdss-dsi-cm-native-command-state",
	"lge,mdss-dsi-disp-ctrl-command-1-state",
	"lge,mdss-dsi-disp-ctrl-command-2-state",
	"lge,digital-gamma-cmds-dummy-state",
	"lge,mdss-dsi-bc-dim-command-state",
	"lge,mdss-dsi-bc-default-command-state",
	"lge,mdss-dsi-vr-lp-command-state",
	"lge,mdss-dsi-vr-lp-command-state",
	"lge,mdss-dsi-vr-lp-command-state",
	"lge,mdss-dsi-lp1-command-state",
	"lge,mdss-dsi-lp2-command-state",
	"lge,mdss-dsi-nolp-command-state",
	"lge,mdss-dsi-saturation-command-state",
	"lge,mdss-dsi-hue-command-state",
	"lge,mdss-dsi-sharpness-command-state",
	"lge,mdss-dsi-saturation-command-state",
	"lge,mdss-dsi-hue-command-state",
	"lge,mdss-dsi-sharpness-command-state",
	"lge,mdss-dsi-cm-sports-state",
	"lge,mdss-dsi-cm-game-state",
	"lge,detect-vert-line-restore-command-state",
	"lge,detect-black-vert-line-command-state",
	"lge,detect-white-vert-line-command-state",
	"lge,memory-error-detect-command-state",
	"lge,esd-detect-command-state",
	"lge,line-defect-detect-command-state",
	"lge,selective-color-cmds-dummy-command-state",
	"lge,ddic-register-lock-unlock-state",
	"lge,ddic-register-lock-unlock-state",
	"lge,mdss-dsi-ve-on-command-state",
	"lge,mdss-dsi-ve-off-command-state",
	"lge,mdss-dsi-hdr-set-state",
	"lge,mdss-dsi-irc-command-state",
	"lge,mdss-dsi-ace-command-state",
	"lge,mdss-dsi-ace-command-state",
	"lge,digital-gamma-set-state",
	"lge,mdss-dsi-aod-area-command-state",
	"lge,color-mode-cmds-dummy-state",
	"lge,color-mode-set-state",
	"lge,custom-rgb-hue-lut-state",
	"lge,saturation-lut-state",
	"lge,sharpness-lut-state",
	"lge,trueview-lut-state",
	"lge,ddic-dsi-br-ctrl-ext-command-state",
	"lge,mdss-dsi-fp-lhbm-command-state",
	"lge,mdss-dsi-fp-lhbm-br-lvl-command-state",
	"lge,lhbm-lut-state",
	"lge,mdss-dsi-tc-perf-command-state",
	"lge,rgb-lut-state",
	"lge,ace-lut-state",
	"lge,mdss-dsi-fp-lhbm-ready-command-state",
	"lge,mdss-dsi-fp-lhbm-exit-command-state",
	"lge,mdss-dsi-fp-lhbm-aod-command-state",
	"lge,mdss-dsi-fp-lhbm-exit-post-command-state",
	"lge,mdss-dsi-fp-lhbm-irc-command-state",
	"lge,mdss-dsi-fp-norm-irc-command-state",
};

char *lge_ddic_cm_lut_cmd_set_prop_map[LGE_CM_LUT_TYPE_MAX] = {
	"lge,cm-lut-screen-mode-set",
	"lge,cm-lut-saturation",
	"lge,cm-lut-sharpness",
	"lge,cm-lut-rgb",
	"lge,cm-lut-ace",
	"lge,cm-lut-trueview",
	"lge,cm-lut-rgb-hue",
};

char *lge_ddic_cm_lut_cmd_set_count_map[LGE_CM_LUT_TYPE_MAX] = {
	"lge,cm-lut-screen-mode-set-cnt",
	"lge,cm-lut-saturation-cnt",
	"lge,cm-lut-sharpness-cnt",
	"lge,cm-lut-rgb-cnt",
	"lge,cm-lut-ace-cnt",
	"lge,cm-lut-trueview-cnt",
	"lge,cm-lut-rgb-hue-cnt",
};

/* lge_ddic_dsi_panel_tx_cmd_set for LGE DSI CMD SETS*/
int lge_ddic_dsi_panel_tx_cmd_set(struct dsi_panel *panel,
				enum lge_ddic_dsi_cmd_set_type type)
{
	int rc = 0, i = 0;
	ssize_t len;
	struct dsi_cmd_desc *cmds;
	u32 count;
	enum dsi_cmd_set_state state;
	const struct mipi_dsi_host_ops *ops = panel->host->ops;

	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	if(!dsi_panel_initialized(panel)) {
		pr_err("panel not yet initialized\n");
		return -EINVAL;
	}

	cmds = panel->lge.lge_cmd_sets[type].cmds;
	count = panel->lge.lge_cmd_sets[type].count;
	state = panel->lge.lge_cmd_sets[type].state;

	if (count == 0) {
		pr_debug("[%s] No commands to be sent for state(%d)\n",
			 panel->name, type);
		goto error;
	}
	for (i = 0; i < count; i++) {
		if (state == DSI_CMD_SET_STATE_LP)
			cmds->msg.flags |= MIPI_DSI_MSG_USE_LPM;

		if (cmds->last_command)
			cmds->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;

		len = ops->transfer(panel->host, &cmds->msg);
		if (len < 0) {
			rc = len;
			pr_err("failed to set cmds(%d), rc=%d\n", type, rc);
			goto error;
		}
		if (cmds->post_wait_ms)
			usleep_range(cmds->post_wait_ms*1000,
					((cmds->post_wait_ms*1000)+10));
		cmds++;
	}
error:
	return rc;
}


int lge_ddic_dsi_panel_alloc_cmd_packets(struct lge_ddic_dsi_panel_cmd_set *cmd,
					u32 packet_count)
{
	u32 size;

	size = packet_count * sizeof(*cmd->cmds);
	cmd->cmds = kzalloc(size, GFP_KERNEL);
	if (!cmd->cmds)
		return -ENOMEM;

	cmd->count = packet_count;
	return 0;
}


int lge_ddic_dsi_panel_parse_cmd_sets_sub(struct lge_ddic_dsi_panel_cmd_set *cmd,
					enum lge_ddic_dsi_cmd_set_type type,
					struct device_node *of_node)
{
	int rc = 0;
	u32 length = 0;
	const char *data;
	const char *state;
	u32 packet_count = 0;

	data = of_get_property(of_node, lge_ddic_cmd_set_prop_map[type], &length);
	if (!data) {
		pr_debug("%s commands not defined\n", lge_ddic_cmd_set_prop_map[type]);
		rc = -ENOTSUPP;
		goto error;
	}

	rc = dsi_panel_get_cmd_pkt_count(data, length, &packet_count);
	if (rc) {
		pr_err("commands failed, rc=%d\n", rc);
		goto error;
	}
	pr_debug("[%s] packet-count=%d, %d\n", lge_ddic_cmd_set_prop_map[type],
		packet_count, length);

	rc = lge_ddic_dsi_panel_alloc_cmd_packets(cmd, packet_count);
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

	state = of_get_property(of_node, lge_ddic_cmd_set_state_map[type], NULL);
	if (!state || !strcmp(state, "dsi_lp_mode")) {
		cmd->state = DSI_CMD_SET_STATE_LP;
	} else if (!strcmp(state, "dsi_hs_mode")) {
		cmd->state = DSI_CMD_SET_STATE_HS;
	} else {
		pr_err("[%s] command state unrecognized-%s\n",
		       lge_ddic_cmd_set_state_map[type], state);
		goto error_free_mem;
	}

	return rc;
error_free_mem:
	kfree(cmd->cmds);
	cmd->cmds = NULL;
error:
	return rc;

}

int lge_ddic_dsi_panel_parse_cmd_sets(struct dsi_panel *panel,
	struct device_node *of_node)
{
	int rc = 0;
	struct lge_ddic_dsi_panel_cmd_set *set;
	u32 i;

	for(i = 0; i < LGE_DDIC_DSI_CMD_SET_MAX; i++) {
		set = &panel->lge.lge_cmd_sets[i];
		set->type = i;
		set->count = 0;
		rc = lge_ddic_dsi_panel_parse_cmd_sets_sub(set, i, of_node);
		if(rc)
			pr_err("parse set %d is failed or not defined\n", i);
	}
	rc = 0;
	return rc;
}

char* get_payload_addr(struct dsi_panel *panel, enum lge_ddic_dsi_cmd_set_type type, int position)
{
	struct lge_ddic_dsi_panel_cmd_set *cmd_set = NULL;
	struct dsi_cmd_desc *cmd = NULL;
	char *payload = NULL;

	if (type >= LGE_DDIC_DSI_CMD_SET_MAX) {
		pr_err("out of range\n");
		goto exit;
	}

	cmd_set = &(panel->lge.lge_cmd_sets[type]);
	if (cmd_set->count == 0) {
		pr_err("cmd set is not defined\n");
		goto exit;
	}

	cmd = &(panel->lge.lge_cmd_sets[type].cmds[position]);
	if (!cmd) {
		pr_err("empty cmd\n");
		goto exit;
	}

	payload = (char *)cmd->msg.tx_buf;
	if (!payload) {
		pr_err("empty payload\n");
		goto exit;
	}

	pr_debug("find payload\n");

exit:
	return payload;
}

int get_payload_cnt(struct dsi_panel *panel, enum lge_ddic_dsi_cmd_set_type type, int position)
{
	struct lge_ddic_dsi_panel_cmd_set *cmd_set = NULL;
	struct dsi_cmd_desc *cmd = NULL;
	int payload_count = 0;

	if (type >= LGE_DDIC_DSI_CMD_SET_MAX) {
		pr_err("out of range\n");
		goto exit;
	}

	cmd_set = &(panel->lge.lge_cmd_sets[type]);
	if (cmd_set->count == 0) {
		pr_debug("cmd set is not defined\n");
		goto exit;
	}

	cmd = &(panel->lge.lge_cmd_sets[type].cmds[position]);
	if (!cmd) {
		pr_err("empty cmd\n");
		goto exit;
	}

	payload_count = (int)cmd->msg.tx_len;

	pr_debug("find payload\n");

exit:
	return payload_count;
}

int lge_ddic_dsi_panel_parse_cm_lut_cmd_sets_sub(struct lge_cm_lut_list_set *cmd,
					enum lge_cm_lut_type type,
					struct device_node *of_node)
{
	int rc = 0;
	u32 length = 0;
	u32 packet_size = 0;
	const char *data = NULL;
	u8 *payload = NULL;
	int i = 0, j = 0;

	rc = of_property_read_u32(of_node, lge_ddic_cm_lut_cmd_set_count_map[type], &packet_size);

	if (rc) {
		pr_err("Unable to read packet_size of %s, rc:%d\n", lge_ddic_cm_lut_cmd_set_count_map[type], rc);
		return rc;
	}

	data = of_get_property(of_node, lge_ddic_cm_lut_cmd_set_prop_map[type], &length);

	if (!data) {
		pr_err("Unable to get property: %s\n", lge_ddic_cm_lut_cmd_set_prop_map[type]);
		rc = -EINVAL;
		return rc;
	}

	cmd->count = length/packet_size;

	cmd->cmds = kzalloc(cmd->count * sizeof(struct lge_lut_command), GFP_KERNEL);
	if (!cmd->cmds) {
		pr_err("Unable to allocate lut_command set\n");
		rc = -EINVAL;
		return rc;
	}

	for (i = 0; i < cmd->count; i++) {
		payload = kzalloc(packet_size, GFP_KERNEL);

		if (!payload) {
			pr_err("Unable to allocate palyload packet_size:%d\n", packet_size);
			rc = -EINVAL;
			goto error_free_payloads;
		}

		for (j = 0; j < packet_size; j++) {
			payload[j] = data[packet_size*i + j];
		}

		cmd->cmds[i].buf = payload;
		cmd->cmds[i].size = packet_size;
    }

	return rc;

error_free_payloads:
	for (i = i-1; i >= 0; i--) {
		if (cmd->cmds[i].buf) {
			kfree(cmd->cmds[i].buf);
		}
	}
	kfree(cmd->cmds);
	return rc;
}

int lge_ddic_dsi_panel_parse_cm_lut_cmd_sets(struct dsi_panel *panel,
	struct device_node *of_node)
{
	int rc = 0;
	struct lge_cm_lut_list_set *set;
	u32 i;

	for(i = 0; i < LGE_CM_LUT_TYPE_MAX; i++) {
		set = &panel->lge.cm_lut_sets[i];
		set->type = i;

		rc = lge_ddic_dsi_panel_parse_cm_lut_cmd_sets_sub(set, i, of_node);
		if(rc)
			pr_err("parse set %d is failed or not defined\n", i);
	}
	return rc;
}

char* get_cm_lut_payload_addr(struct dsi_panel *panel, enum lge_cm_lut_type type, int position)
{
	struct lge_cm_lut_list_set *cmd_set = NULL;
	struct lge_lut_command *cmd = NULL;
	char *payload = NULL;

	if (type >= LGE_CM_LUT_TYPE_MAX) {
		pr_err("out of range\n");
		goto exit;
	}

	cmd_set = &(panel->lge.cm_lut_sets[type]);
	if (cmd_set->count == 0) {
		pr_err("cmd set is not defined\n");
		goto exit;
	}

	cmd = &(panel->lge.cm_lut_sets[type].cmds[position]);
	if (!cmd) {
		pr_err("empty cmd\n");
		goto exit;
	}

	payload = (char *)cmd->buf;

	if (!payload) {
		pr_err("empty payload\n");
		goto exit;
	}

	pr_debug("find payload\n");

exit:
	return payload;
}

int get_cm_lut_payload_cnt(struct dsi_panel *panel, enum lge_cm_lut_type type, int position)
{
	struct lge_cm_lut_list_set *cmd_set = NULL;
	struct lge_lut_command *cmd = NULL;
	int payload_count = 0;

	if (type >= LGE_CM_LUT_TYPE_MAX) {
		pr_err("out of range\n");
		goto exit;
	}

	cmd_set = &(panel->lge.cm_lut_sets[type]);
	if (cmd_set->count == 0) {
		pr_info("cmd set is not defined\n");
		goto exit;
	}

	cmd = &(panel->lge.cm_lut_sets[type].cmds[position]);
	if (!cmd) {
		pr_err("empty cmd\n");
		goto exit;
	}

	payload_count = cmd->size;

	pr_debug("find payload count:%d\n", payload_count);

exit:
	return payload_count;
}

