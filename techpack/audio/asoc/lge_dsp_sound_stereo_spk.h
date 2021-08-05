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


#define CAPI_V2_MODULE_ID_LGE_STEREO 		     	0x10002000
#define CAPI_V2_PARAM_LGE_STEREO_ENABLE      		0x10002001
#define CAPI_V2_PARAM_LGE_STEREO_CH_SWAP     		0x10002002
#define CAPI_V2_PARAM_LGE_STEREO_DOWNMIXING  		0x10002003

struct asm_lgestereo_param_value {
    int32_t                  value;
} __packed;

int q6asm_set_lgestereo_send_command(struct audio_client *ac, int module_id, int param_id, int value);
