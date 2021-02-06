/*
 * touch_common.h
 *
 * Copyright (c) 2015 LGE.
 *
 * author : hoyeon.jang@lge.com
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

#ifndef LGE_TOUCH_COMMON_H
#define LGE_TOUCH_COMMON_H

void touch_msleep(unsigned int msecs);
void touch_interrupt_control(struct device *dev, int on_off);
int touch_snprintf(char *buf, int size, const char *fmt, ...);
#endif /* LGE_TOUCH_CORE_H */
