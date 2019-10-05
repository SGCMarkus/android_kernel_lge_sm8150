#define pr_fmt(fmt)	"[Display][lge-backlight-ex:%s:%d] " fmt, __func__, __LINE__
#include "msm_drv.h"
#include "sde_dbg.h"

#include "sde_kms.h"
#include "sde_connector.h"
#include "sde_encoder.h"
#include <linux/backlight.h>
#include "dsi_drm.h"
#include "dsi_display.h"
#include "sde_crtc.h"

#include "../brightness/lge_brightness_def.h"
#include "../lge_dsi_panel.h"

#define BL_NODE_NAME_SIZE 32

extern struct lge_blmap *lge_get_blmap(struct dsi_panel *panel, enum lge_blmap_type type);

int dsi_display_set_backlight_ex(struct dsi_display *dsi_display, u32 bl_lvl)
{
	struct dsi_panel *panel;
	int rc = 0;

	if (dsi_display == NULL || dsi_display->panel == NULL)
		return -EINVAL;

	panel = dsi_display->panel;

	mutex_lock(&panel->panel_lock);
	if (!dsi_panel_initialized(panel)) {
		rc = -EINVAL;
		goto error;
	}

	rc = dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_ON);
	if (rc) {
		pr_err("[%s] failed to enable DSI core clocks, rc=%d\n",
		       dsi_display->name, rc);
		goto error;
	}

	rc = dsi_panel_set_backlight(panel, (u32)bl_lvl);
	if (rc)
		pr_err("unable to set backlight\n");

	rc = dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_OFF);
	if (rc) {
		pr_err("[%s] failed to disable DSI core clocks, rc=%d\n",
		       dsi_display->name, rc);
		goto error;
	}

error:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

int lge_update_backlight_ex(struct dsi_panel *panel)
{
	struct dsi_display *display;
	struct backlight_device *ex_bd;
	int rc = 0;

	if (panel == NULL || panel->host == NULL)
		return -EINVAL;

	ex_bd = panel->lge.bl_ex_device;
	if (ex_bd == NULL)
		return -EINVAL;

	if (panel->lge.allow_bl_update_ex || !lge_dsi_panel_is_power_on_lp(panel))
		return 0;

	mutex_lock(&ex_bd->ops_lock);
	if (panel->lge.bl_ex_lvl_unset < 0) {
		rc = 0;
		goto exit;
	}

	display = container_of(panel->host, struct dsi_display, host);

	rc = dsi_display_set_backlight_ex(display, panel->lge.bl_ex_lvl_unset);
	if (!rc) {
		pr_info("<--%pS unset=%d\n", __builtin_return_address(0), panel->lge.bl_ex_lvl_unset);
	}
	panel->lge.allow_bl_update_ex = true;
	panel->lge.bl_ex_lvl_unset = -1;

exit:
	mutex_unlock(&ex_bd->ops_lock);
	return rc;
}

static int lge_backlight_ex_device_update_status(struct backlight_device *bd)
{
	int brightness;
	struct lge_blmap *blmap;
	struct dsi_panel *panel;
	struct dsi_display *display;
	struct sde_connector *c_conn;
	enum lge_blmap_type bl_type;
	int bl_lvl;

	brightness = bd->props.brightness;

	c_conn = bl_get_data(bd);
	display = (struct dsi_display *) c_conn->display;
	panel = display->panel;
	bl_type = LGE_BLMAP_EX;
	blmap = lge_get_blmap(panel, bl_type);

	if (blmap) {
		// DUMMY panel doesn't have blmap, so this code is mandatory
		if(blmap->size == 0)	return -EINVAL;
		if (brightness >= blmap->size) {
			pr_warn("brightness=%d is bigger than blmap size (%d)\n", brightness, blmap->size);
			brightness = blmap->size-1;
		}
		bl_lvl = blmap->map[brightness];
	} else {
		if (brightness > display->panel->bl_config.bl_max_level)
			brightness = display->panel->bl_config.bl_max_level;

		/* map UI brightness into driver backlight level with rounding */
		bl_lvl = mult_frac(brightness, display->panel->bl_config.bl_max_level,
				display->panel->bl_config.brightness_max_level);

		if (!bl_lvl && brightness)
			bl_lvl = 1;
	}

	mutex_lock(&display->display_lock);
	if (((panel->lge.lp_state == LGE_PANEL_LP2) || (panel->lge.lp_state == LGE_PANEL_LP1))
			&& panel->lge.allow_bl_update_ex) {
		panel->lge.bl_ex_lvl_unset = -1;
		dsi_display_set_backlight_ex(display, bl_lvl);
		pr_info("BR:%d BL:%d %s\n", brightness, bl_lvl, lge_get_blmapname(bl_type));
	} else if (!panel->lge.allow_bl_update_ex) {
		panel->lge.bl_ex_lvl_unset = bl_lvl;
		pr_info("brightness=%d, bl_lvl=%d -> differed (not allow)\n", brightness, bl_lvl);
	} else {
		panel->lge.bl_ex_lvl_unset = bl_lvl;
		pr_info("brightness=%d, bl_lvl=%d -> differed\n", brightness, bl_lvl);
	}
	mutex_unlock(&display->display_lock);

	return 0;
}

static int lge_backlight_ex_device_get_brightness(struct backlight_device *bd)
{
	return 0;
}

static const struct backlight_ops lge_backlight_ex_device_ops = {
	.update_status = lge_backlight_ex_device_update_status,
	.get_brightness = lge_backlight_ex_device_get_brightness,
};

int lge_backlight_ex_setup(struct sde_connector *c_conn,
					struct drm_device *dev)
{
	struct backlight_properties props;
	struct dsi_panel *panel;
	struct dsi_display *display;
	struct dsi_backlight_config *bl_config;
	static int display_count;
	char bl_node_name[BL_NODE_NAME_SIZE];

	if (!c_conn || !dev || !dev->dev) {
		pr_err("invalid param\n");
		return -EINVAL;
	} else if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI) {
		return 0;
	}

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.power = FB_BLANK_UNBLANK;

	display = (struct dsi_display *) c_conn->display;
	panel = display->panel;
	bl_config = &panel->bl_config;
	props.max_brightness = bl_config->brightness_max_level;
	props.brightness = 0;
	snprintf(bl_node_name, BL_NODE_NAME_SIZE, "panel%u-backlight-ex",
							display_count);
	panel->lge.bl_ex_lvl_unset = -1;
	panel->lge.allow_bl_update_ex = false;
	panel->lge.bl_ex_device = backlight_device_register(bl_node_name, dev->dev,
			c_conn, &lge_backlight_ex_device_ops, &props);
	if (IS_ERR_OR_NULL(panel->lge.bl_ex_device)) {
		SDE_ERROR("Failed to register backlight-ex: %ld\n",
				    PTR_ERR(panel->lge.bl_ex_device));
		panel->lge.bl_ex_device = NULL;
		return -ENODEV;
	}
	display_count++;

	return 0;
}

void lge_backlight_ex_destroy(struct sde_connector *c_conn)
{
	struct dsi_display *display = (struct dsi_display *) c_conn->display;
	if (display->panel->lge.bl_ex_device)
		backlight_device_unregister(display->panel->lge.bl_ex_device);
}
