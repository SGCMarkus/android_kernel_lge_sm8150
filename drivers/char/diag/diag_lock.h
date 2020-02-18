/*
 * Diag Lock Driver for LGE
 *
 * Copyright (C) 2016 LG Electronics Inc. All rights reserved.
 * Author : Hansun Lee <hansun.lee@lge.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __DIAG_LOCK_H__
#define __DIAG_LOCK_H__

typedef enum {
	DIAG_LOCK_STATE_LOCK,
	DIAG_LOCK_STATE_UNLOCK,
} diag_lock_state_t;

bool diag_lock_is_allowed(void);
bool diag_lock_is_allowed_command(const unsigned char *buf);

#endif /* __DIAG_LOCK_H__ */
