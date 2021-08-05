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

enum {
    DTS_VIRT_ON = 0,
	DTS_VIRT_MODE,
	DTS_GEQ_ON,
	DTS_GEQ_BAND1,
	DTS_GEQ_BAND2,
	DTS_GEQ_BAND3,
	DTS_GEQ_BAND4,
	DTS_GEQ_BAND5,
	DTS_GEQ_BAND6,
	DTS_GEQ_BAND7,
	DTS_GEQ_BAND8,
	DTS_GEQ_BAND9,
	DTS_GEQ_BAND10,
	DTS_BASSBOOST_ON,
	DTS_VOCALBOOST_ON,
	DTS_LOUDNESS_LEVEL_ON,
	DTS_GEQ_PRESET,
	DTS_BASSBOOST_LEVEL,
	DTS_VOCALBOOST_LEVEL,
	DTS_VOLUME_LEVEL,
	DTS_TURN_ON,
	DTS_SPEAKER_ENABLE,
	DTS_FADE_ENABLE,
	LGE_DTS_PARAM_MAX
};

struct asm_lge_dts_param {
	uint32_t                  value;
} __packed;
#if 0
struct asm_lge_dts_param {
    struct apr_hdr  hdr;
    struct asm_stream_cmd_set_pp_params_v2 param;
    struct asm_stream_param_data_v2 data;
    int32_t                  value;
} __packed;
#endif
int q6asm_set_lge_dts_param(struct audio_client *ac, int param_id, int value);
