/*
 * Copyright (c) 2018, LG Eletronics. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _LGE_VFPS_H_
#define _LGE_VFPS_H_

#include <linux/err.h>
#include "main/lge_prm.h"
#include "sde/sde_crtc.h"

bool lge_vfps_check_internal(void);
void lge_vfps_set_encoder(struct drm_encoder *encoder);

int lge_vfps_init(void);
#endif // _LGE_VFPS_H_
