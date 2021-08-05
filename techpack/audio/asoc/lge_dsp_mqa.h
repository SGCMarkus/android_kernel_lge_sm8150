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



#define AUDPROC_MODULE_ID_LGMQA                             0x1000D040
#define AUDPROC_PARAM_ID_LGMQA_ENABLE                       0x1000D041
#define AUDPROC_PARAM_ID_LGMQA_POWERMODE                    0x1000D042
#define AUDPROC_PARAM_ID_LGMQA_MULTIPLERATE                 0x1000D043
#define AUDPROC_PARAM_ID_LGMQA_OUTPUTMODE                   0x1000D044
#define AUDPROC_PARAM_ID_LGMQA_PROPERTIES                   0x1000D045

struct asm_lgmqa_param_one {
    int32_t                  msg;
} __packed;

struct asm_lgmqa_param_all {
    int32_t                  powermode;
    int32_t                  multiplerate;
    int32_t                  outputmode;
} __packed;


