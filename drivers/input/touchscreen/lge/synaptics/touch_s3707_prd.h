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

#include <touch_hwif.h>
#include <touch_core.h>

#include "touch_s3707.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

#define BUF_SIZE		(PAGE_SIZE * 2)
#define LOG_BUF_SIZE		256
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT	4
#define FILE_STR_LEN		(128)
#define TIME_STR_LEN		(64)
#define PRD_BLOCK_MAX		(20)

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

extern void touch_msleep(unsigned int msecs);
int s3707_prd_register_sysfs(struct device *dev);

#endif


