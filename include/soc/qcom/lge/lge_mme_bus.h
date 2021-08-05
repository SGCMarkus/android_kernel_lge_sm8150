/* Copyright (c) 2018 LG Electronics, Inc.
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

#ifndef __MACH_LGE_MME_BUS_H
#define __MACH_LGE_MME_BUS_H

#ifdef CONFIG_LGE_MME_BUS
extern int lge_mme_bus_boost(int enable);
extern int lge_mme_bus_boost_long_term(int enable);
extern int lge_mme_bus_check_boost_app(char *appname);
#else
static inline int lge_mme_bus_boost(int enable) { return 0; }
static inline int lge_mme_bus_boost_long_term(int enable) { return 0; }
static inline int lge_mme_bus_check_boost_app(char *appname) { return 0; }
#endif

#endif

