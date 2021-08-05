/*
 * Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

// Topology ID, Module ID, Parameter ID
#define LGE_CROSSTALK_ANC                  0x1000E910
#define LGE_CROSSTALK_ANC_HEADSET_MODE      0x1000E912

struct adm_crosstalk_param {
	int32_t              crosstalk_enable_mode;
} __packed;

int q6adm_set_crosstalk_parms(int port_id, int copp_idx, int mode);
