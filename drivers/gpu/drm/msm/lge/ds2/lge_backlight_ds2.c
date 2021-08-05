/*
 * Copyright(c) 2018, LG Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt)	"[Display][lge-backlight-ds2:%s:%d] " fmt, __func__, __LINE__
#include "msm_drv.h"
#include "sde_dbg.h"

#include "sde_kms.h"
#include "sde_connector.h"
#include "sde_encoder.h"
#include <linux/backlight.h>
#include "dsi_drm.h"
#include "dsi_display.h"
#include "sde_crtc.h"

#include "../lge_dsi_panel.h"
#include "lge_backlight_ds2.h"

#define BL_NODE_NAME_SIZE 32

int br_to_offset_br_ds2(struct dsi_panel *panel, int br, int max_lvl, enum cover_br_type type)
{
	int offset, infp;
	int weight = 2;
	int min_br = 10;
	int max_br = ((max_lvl == 0) ? 255 : max_lvl);
	int offset_br = br;
	enum cover_br_type update_type = BR_XD;

	if (!panel) {
		pr_err("null ptr\n");
		return -EINVAL;
	}

	if (br < min_br) {
		pr_err("min br level\n");
		goto exit;
	}

	if (type == BR_DD && br > max_br) {
		pr_err("max br level\n");
		offset_br = max_br;
		goto exit;
	}

	if (panel->lge.br_offset_bypass) {
		pr_info("offset bypass (type=%d)\n", type);
		if (type == BR_MD)
			offset_br = br;
		else if (type == BR_DD) {
			panel->lge.br_offset_bypass = false;
			offset_br = br + (panel->lge.br_offset * weight);
		}

		goto exit;
	}

	if (panel->lge.br_offset_update) {
		offset_br = br + (panel->lge.br_offset * weight);
		goto exit;
	}

	offset = panel->lge.br_offset;
	if (offset > 0) {
		update_type = BR_MD;
	} else if (offset < 0) {
		update_type = BR_DD;
		offset = abs(offset);
	} else {
		goto exit;
	}

	offset *= weight;
	infp = min_br + (offset << 1);

	if (type == update_type) {
		if (br > infp)
			offset_br = br - offset;
		else {
			int tmp = infp - offset - min_br;
			tmp *= (br - min_br);
			tmp /= (infp - min_br);
			offset_br = min_br + tmp;
		}
	}

	if (offset_br < min_br) {
		pr_info("lower than min. br : set %d to %d\n", offset_br, min_br);
		offset_br = min_br;
	}

exit:
	pr_info("[%s] %d -> %d\n", ((type == BR_MD) ? "md" : "dd"), br, offset_br);
	return offset_br;
}

