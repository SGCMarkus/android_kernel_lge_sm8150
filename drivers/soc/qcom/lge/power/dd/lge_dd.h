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
#ifndef _LGE_DD_H_
#define _LGE_DD_H_

#include <linux/err.h>

#include "main/lge_prm.h"

void lge_dd_connect_notify(void);

int lge_dd_init(void);
#endif // _LGE_DD_H_
