/*
 * Copyright(c) 2018, LG Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LGE_COVER_CTRL_H_
#define _LGE_COVER_CTRL_H_
#include "../../dp/dp_gpio_hpd.h"
#include "../../dp/dp_hpd.h"
#include "../../dp/dp_display.h"

struct dp_hpd *dd_hpd_get(struct device *dev, struct dp_hpd_cb *cb);
void dd_hpd_put(struct dp_hpd *dd_hpd);
int lge_cover_extcon_register(struct platform_device *pdev, struct dp_display *display, int id);
void dd_gpio_selection(int dd_hpd, int flip);
void dd_lattice_disable(void);
int lge_cover_ctrl_create_sysfs(struct device *panel_sysfs_dev);

#endif // _LGE_COVER_CTRL_H_

