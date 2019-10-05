/* production_test.h
 *
 * Copyright (C) 2015 LGE.
 *
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

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

#define BUF_SIZE		(PAGE_SIZE * 2)
#define TIME_STR_LEN		(64)
#define PRD_BLOCK_MAX		(20)

int module_prd_register_sysfs(struct device *dev);

#endif


