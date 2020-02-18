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
#ifndef LGE_FBCN_H
#define LGE_FBCN_H

#include "main/lge_prm.h"

struct fbcn_intv_stats {
	u64 intv;
	u64 i_upper;
	u64 i_lower;
	int enable;
	int init;
	struct mutex fbcn_lock;
	struct notifier_block nb;
};

int _lge_fbcn_sysfs_create(struct device *dev);

int lge_fbcn_init(void);
#endif /* LGE_FBCN_H */
