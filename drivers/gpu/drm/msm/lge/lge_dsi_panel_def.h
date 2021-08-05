#ifndef _H_LGE_DSI_PANEL_DEF_
#define _H_LGE_DSI_PANEL_DEF_
#include "drs/lge_drs_mngr.h"

#define NUM_COLOR_MODES  10
#define MAX_BIST_USAGE_TYPE 5

/**
 * enum lge_ddic_dsi_cmd_set_type = LGE DSI command set type
 * @
 */
enum lge_ddic_dsi_cmd_set_type {
	LGE_DDIC_DSI_SET_IMAGE_ENHANCEMENT = 0,
	LGE_DDIC_DSI_SET_BIST_ON,
	LGE_DDIC_DSI_SET_BIST_OFF,
	LGE_DDIC_DSI_SET_WB_DEFAULT,
	LGE_DDIC_DSI_SET_CM_DCI_P3,
	LGE_DDIC_DSI_SET_CM_SRGB,
	LGE_DDIC_DSI_SET_CM_ADOBE,
	LGE_DDIC_DSI_SET_CM_NATIVE,
	LGE_DDIC_DSI_DISP_CTRL_COMMAND_1,
	LGE_DDIC_DSI_DISP_CTRL_COMMAND_2,
	LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY,
	LGE_DDIC_DSI_BC_DIMMING,
	LGE_DDIC_DSI_BC_DEFAULT_DIMMING,
	LGE_DDIC_DSI_SET_VR_MODE_ON,
	LGE_DDIC_DSI_SET_VR_MODE_PRE_OFF,
	LGE_DDIC_DSI_SET_VR_MODE_POST_OFF,
	LGE_DDIC_DSI_SET_LP1,
	LGE_DDIC_DSI_SET_LP2,
	LGE_DDIC_DSI_SET_NOLP,
	LGE_DDIC_DSI_SET_SATURATION,
	LGE_DDIC_DSI_SET_HUE,
	LGE_DDIC_DSI_SET_SHARPNESS,
	LGE_DDIC_DSI_SET_SATURATION_DEFAULT,
	LGE_DDIC_DSI_SET_HUE_DEFAULT,
	LGE_DDIC_DSI_SET_SHARPNESS_DEFAULT,
	LGE_DDIC_DSI_CM_SPORTS,
	LGE_DDIC_DSI_CM_GAME,
	LGE_DDIC_DSI_DETECT_VERT_LINE_RESTORE,
	LGE_DDIC_DSI_DETECT_BLACK_VERT_LINE,
	LGE_DDIC_DSI_DETECT_WHITE_VERT_LINE,
	LGE_DDIC_DSI_MEM_ERR_DETECT,
	LGE_DDIC_DSI_ESD_DETECT,
	LGE_DDIC_DSI_LINE_DEFECT_DETECT,
	LGE_DDIC_DSI_DISP_SC_COMMAND_DUMMY,
	LGE_DDIC_DSI_REGISTER_LOCK,
	LGE_DDIC_DSI_REGISTER_UNLOCK,
	LGE_DDIC_DSI_VIDEO_ENHANCEMENT_ON,
	LGE_DDIC_DSI_VIDEO_ENHANCEMENT_OFF,
	LGE_DDIC_DSI_HDR_SET_CTRL,
	LGE_DDIC_DSI_IRC_CTRL,
	LGE_DDIC_DSI_ACE_TUNE,
	LGE_DDIC_DSI_ACE_RESTORE,
	LGE_DDIC_DSI_DIGITAL_GAMMA_SET,
	LGE_DDIC_DSI_AOD_AREA,
	LGE_DDIC_DSI_DISP_CM_COMMAND_DUMMY,
	LGE_DDIC_DSI_DISP_CM_SET,
	LGE_DDIC_DSI_DISP_RGB_HUE_LUT,
	LGE_DDIC_DSI_DISP_SAT_LUT,
	LGE_DDIC_DSI_DISP_SHA_LUT,
	LGE_DDIC_DSI_DISP_TRUEVIEW_LUT,
	LGE_DDIC_DSI_BRIGHTNESS_CTRL_EXT_COMMAND,
	LGE_DDIC_DSI_FP_LHBM_COMMAND,
	LGE_DDIC_DSI_FP_LHBM_BR_LVL_COMMAND,
	LGE_DDIC_DSI_FP_LHBM_LUT,
	LGE_DDIC_DSI_TC_PERF_COMMAND,
	LGE_DDIC_DSI_RGB_LUT,
	LGE_DDIC_DSI_ACE_LUT,
	LGE_DDIC_DSI_FP_LHBM_READY,
	LGE_DDIC_DSI_FP_LHBM_EXIT,
	LGE_DDIC_DSI_FP_LHBM_AOD_COMMAND,
	LGE_DDIC_DSI_FP_LHBM_EXIT_POST,
	LGE_DDIC_DSI_FP_LHBM_IRC_SET,
	LGE_DDIC_DSI_FP_NORM_IRC_SET,
	LGE_DDIC_DSI_CMD_SET_MAX
};

struct lge_ddic_dsi_panel_cmd_set {
	enum lge_ddic_dsi_cmd_set_type type;
	enum dsi_cmd_set_state state;
	u32 count;
	u32 ctrl_idx;
	struct dsi_cmd_desc *cmds;
};

enum lge_panel_lp_state {
	LGE_PANEL_NOLP = 0,
	LGE_PANEL_LP1,
	LGE_PANEL_LP2,
	LGE_PANEL_OFF,
	LGE_PANEL_STATE_MAX,
};

struct lge_panel_pin_seq {
	int gpio;
	u32 level;
	u32 sleep_ms;
};

struct lge_blmap {
	int size;
	int *map;
};

struct lge_rect {
	int x;
	int y;
	int w;
	int h;
};

struct backup_info {
	char owner[32];
	enum dsi_cmd_set_type type;
	char name[32];
	u8 reg;
	int nth_cmds; /* #N command, if duplicated */
	bool is_backup;
};

enum lge_irc_ctrl {
	LGE_IRC_ON = 0,
	LGE_IRC_OFF,
	LGE_IRC_MAX,
};

enum lge_irc_mode {
	LGE_GLOBAL_IRC_HBM = 0,
	LGE_GLOBAL_IRC_HDR,
	LGE_GLOBAL_IRC_MAX,
};

struct dsi_panel;

struct lge_ddic_ops {
	/* For DISPLAY_AMBIENT */
	int (*store_aod_area)(struct dsi_panel *panel, struct lge_rect rect);
	int (*prepare_aod_cmds)(struct dsi_panel *panel, struct dsi_cmd_desc *cmds, int cmds_count);
	void (*prepare_aod_area)(struct dsi_panel *panel, struct dsi_cmd_desc *cmds, int cmds_count);

	/* For DISPLAY_COLOR_MANAGER */
	void (*lge_display_control_store)(struct dsi_panel *panel, bool send_cmd);
	void (*lge_bc_dim_set)(struct dsi_panel *panel, u8 bc_dim_en, u8 bc_dim_f_cnt);
	int (*lge_set_therm_dim)(struct dsi_panel *panel, int input);
	int (*lge_get_brightness_dim)(struct dsi_panel *panel);
	void (*lge_set_brightness_dim)(struct dsi_panel *panel, int input);
	void (*lge_set_custom_rgb)(struct dsi_panel *panel, bool send_cmd);
	void (*lge_set_rgb_tune)(struct dsi_panel *panel, bool send_cmd);
	void (*lge_set_screen_tune)(struct dsi_panel *panel);
	void (*lge_set_screen_mode)(struct dsi_panel *panel, bool send_cmd);
	void (*lge_send_screen_mode_cmd)(struct dsi_panel *panel, int index);
	void (*lge_set_hdr_hbm_lut)(struct dsi_panel *panel, int input);
	void (*lge_set_hdr_mode)(struct dsi_panel *panel, int input);
	void (*lge_set_acl_mode)(struct dsi_panel *panel, int input);
	void (*lge_set_video_enhancement)(struct dsi_panel *panel, int input);
	void (*lge_vr_lp_mode_set)(struct dsi_panel *panel, int input);
	void (*sharpness_set)(struct dsi_panel *panel, int mode);
	void (*lge_set_true_view_mode)(struct dsi_panel *panel, bool send_cmd);
	int (*hdr_mode_set)(struct dsi_panel *panel, int input);
	int (*get_irc_state)(struct dsi_panel *panel);
	int (*set_irc_state)(struct dsi_panel *panel, enum lge_irc_mode mode, enum lge_irc_ctrl enable);
	int (*set_irc_default_state)(struct dsi_panel *panel);
	void (*set_dim_ctrl)(struct dsi_panel *panel, bool status);
	void (*lge_set_fp_lhbm)(struct dsi_panel *panel, int input);
	void (*lge_set_fp_lhbm_br_lvl)(struct dsi_panel *panel, int input);
	void (*lge_set_tc_perf)(struct dsi_panel *panel, int input);
	void (*lge_damping_mode_set)(struct dsi_panel *panel, int input);

	/* For Display DRS */
	int (*bist_ctrl)(struct dsi_panel *panel, bool enable);
	int (*release_bist)(struct dsi_panel *panel);
	int (*get_current_res)(struct dsi_panel *panel);
	void (*get_support_res)(int idx, void* input);
	struct backup_info* (*get_reg_backup_list)(int *cnt);
	int (*set_pps_cmds)(struct dsi_panel *panel, enum lge_ddic_dsi_cmd_set_type type);
	int (*unset_pps_cmds)(struct dsi_panel *panel, enum lge_ddic_dsi_cmd_set_type type);

	/* For DISPLAY_FACTORY */
	void (*lge_check_vert_black_line)(struct dsi_panel *panel);
	void (*lge_check_vert_white_line)(struct dsi_panel *panel);
	void (*lge_check_vert_line_restore)(struct dsi_panel *panel);

	/* For DISPLAY_ERR_DETECT */
	void (*err_detect_work)(struct work_struct *work);
	irqreturn_t (*err_detect_irq_handler)(int irq, void *data);
	int (*set_err_detect_mask)(struct dsi_panel *panel);
};

struct lge_dsi_color_manager_mode_entry {
	u32 color_manager_mode;
	u32 color_manager_status;
};

struct lge_lut_command {
    u32 size;
    const u8 *buf;
};

enum lge_cm_lut_type {
	LGE_CM_LUT_COLOR_MODE_SET = 0,
	LGE_CM_LUT_SATURATION,
	LGE_CM_LUT_SHARPNESS,
	LGE_CM_LUT_RGB,
	LGE_CM_LUT_ACE,
	LGE_CM_LUT_TRUEVIEW,
	LGE_CM_LUT_RGB_HUE,
	LGE_CM_LUT_TYPE_MAX,
};

struct lge_cm_lut_list_set {
	enum lge_cm_lut_type type;
	u32 count;
	struct lge_lut_command *cmds;
};

struct lge_dsi_panel {
	int *pins;
	int pins_num;

	struct lge_panel_pin_seq *panel_on_seq;
	struct lge_panel_pin_seq *panel_off_seq;

	bool use_labibb;
	bool reset_after_ddvd;
	bool is_incell;
	bool use_panel_reset_low_before_lp11;
	bool use_extra_recovery_cmd;

	enum lge_panel_lp_state lp_state;
	enum lge_panel_lp_state panel_state;

	struct lge_blmap *blmap_list;
	int blmap_list_size;
	int default_brightness;
	bool dcs_brightness_be;
	bool use_bist;
	bool update_pps_in_lp;
	bool panel_dead;
	bool panel_dead_pending;
	struct delayed_work panel_dead_work;
	int pps_orig;
	int bist_on;
	const char *bist_usage_type[MAX_BIST_USAGE_TYPE];
	struct mutex bist_lock;
	bool use_drs_mngr;
	bool use_internal_pps_switch;
	struct lge_drs_mngr drs_mngr;
	bool use_ddic_reg_backup;
	bool ddic_reg_backup_complete;
	bool is_sent_bc_dim_set;
	int hdr_mode;
	int mfts_auto_touch;

	/* For DISPLAY_FACTORY */
	int use_line_detect;

//Display Bringup need to check is it need
	int screen_tune_status;
	int sc_sat_step;
	int sc_hue_step;
	int sc_sha_step;
	int color_filter;
	bool sharpness_control;

	/* For DISPLAY_COLOR_MANAGER */
	bool use_color_manager;
	bool dgc_absent;
	u8 move_wp;
	u8 dgc_status;
	u8 sharpness_status;
	int color_manager_status;
	int color_manager_mode;
	bool is_backup;
	int screen_mode;
	struct dsi_panel_cmd_set d65_default_cmds;
	int cm_preset_step;
	int cm_red_step;
	int cm_green_step;
	int cm_blue_step;
	struct lge_dsi_color_manager_mode_entry color_manager_table[NUM_COLOR_MODES];
	u32 color_manager_table_len;
	bool color_manager_default_status;
	struct dsi_panel_cmd_set dg_preset_cmds;
	int sharpness;
	int video_enhancement;
	int hdr_hbm_lut;
	int ddic_hdr;
	int ve_hdr;
	int ace_hdr;
	int hbm_mode;
	int acl_mode;
	bool is_cm_reg_backup;
	u8 bc_dim_en;
	u8 bc_dim_f_cnt;
	struct delayed_work bc_dim_work;
	bool use_bc_dimming_work;
	bool use_vr_lp_mode;
	int vr_lp_mode;
	bool use_dim_ctrl;
	bool use_fp_lhbm;
	bool is_fp_hbm_mode;
	u32 fp_hbm_mode_backlight_lvl;
	bool need_fp_lhbm_set;
	int fp_lhbm_mode;
	int old_fp_lhbm_mode;
	int fp_lhbm_br_lvl;
	int old_panel_fp_mode;
	int forced_lhbm;
	bool use_tc_perf;
	int tc_perf;

	bool use_damping_mode;
	int damping_hdr;

	bool true_view_supported;
	u32 true_view_mode;

	bool use_irc_ctrl;
	bool irc_pending;
	int irc_request_state;
	int irc_current_state;
	bool use_ace_ctrl;
	int ace_mode;

	bool use_dynamic_brightness;
	int brightness_table;

	atomic_t backup_state;
	struct work_struct backup_work;
	struct lge_ddic_ops *ddic_ops;

	/* FOR DISPLAY BACKLIGT CONTROL */
	bool allow_bl_update;
	int bl_lvl_unset;
	int bl_lvl_recovery_unset;
	bool use_dcs_brightness_short;

	/* For DISPLAY_AMBIENT */
	bool use_ambient;
	bool use_cmd_wait_pa_changed;
	bool wait_pa_changed;
	struct completion pa_changed_done;
	bool allow_bl_update_ex;
	int bl_ex_lvl_unset;
	struct backlight_device *bl_ex_device;
	struct lge_rect aod_area;
	struct mutex pa_changed_lock;
	bool partial_area_vertical_changed;
	bool partial_area_horizontal_changed;
	bool partial_area_height_changed;
	bool aod_power_mode;
	bool use_br_ctrl_ext;

	u32 aod_interface;
	const char *aod_interface_type[3];

#if IS_ENABLED(CONFIG_LGE_COVER_DISPLAY) || IS_ENABLED(CONFIG_LGE_DUAL_SCREEN)
	/*For Cover Display */
	struct backlight_device *bl_cover_device;
	int bl_cover_lvl_unset;
	int br_offset;
	bool br_offset_update;
	bool br_offset_bypass;
	// For Cover Color Mode
	int cover_screen_mode;
	int cover_preset;
	int cover_red;
	int cover_green;
	int cover_blue;
	int cover_saturation;
	int cover_hue;
	int cover_sharpness;
#endif /* CONFIG_LGE_COVER_DISPLAY */

	/* For DISPLAY_ERR_DETECT */
	bool use_panel_err_detect;
	bool err_detect_crash_enabled;
	bool err_detect_irq_enabled;
	bool is_first_err_mask;
	int err_detect_gpio;
	int err_detect_result;
	int err_detect_mask;
	struct workqueue_struct *err_detect_int_workq;
	struct delayed_work err_detect_int_work;

	bool use_ddic_reg_lock;
	struct lge_ddic_dsi_panel_cmd_set lge_cmd_sets[LGE_DDIC_DSI_CMD_SET_MAX];

	struct lge_cm_lut_list_set cm_lut_sets[LGE_CM_LUT_TYPE_MAX];
	bool use_cm_lut;
	const char **cm_lut_name_list;
	int cm_lut_cnt;
};

#endif //_H_LGE_DSI_PANEL_DEF_
