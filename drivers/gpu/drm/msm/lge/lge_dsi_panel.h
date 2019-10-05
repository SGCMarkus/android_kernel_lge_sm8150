#ifndef _H_LGE_DSI_PANEL_
#define _H_LGE_DSI_PANEL_

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DIMMING_BOOT_SUPPORT)
bool is_blank_called(void);
int lge_get_bootreason_with_lcd_dimming(void);
bool is_factory_cable(void);
#endif

extern int lge_dsi_panel_drv_init(struct dsi_panel *panel);
extern int lge_dsi_panel_drv_deinit(struct dsi_panel *panel);
extern int lge_dsi_panel_get(struct dsi_panel *panel, struct device_node *of_node);
extern void lge_dsi_panel_put(struct dsi_panel *panel);
extern int lge_dsi_panel_parse_cmd_state(struct device_node *of_node, const char *name, enum dsi_cmd_set_state *pstate);
extern bool lge_dsi_panel_is_power_off(struct dsi_panel *panel);
extern bool lge_dsi_panel_is_power_on_interactive(struct dsi_panel *panel);
extern bool lge_dsi_panel_is_power_on(struct dsi_panel *panel);
extern bool lge_dsi_panel_is_power_on_lp(struct dsi_panel *panel);
extern bool lge_dsi_panel_is_power_on_ulp(struct dsi_panel *panel);
#endif //_H_LGE_DSI_PANEL_
