/*
 *  cxd22xx.h - cxd22xx NFC driver
 *
 * Copyright (C) 2013 Sony Corporation.
 * Copyright (C) 2012 Broadcom Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _CXD22XX_H
#define _CXD22XX_H

#define CXDNFC_MAGIC 'S'
/*
 * CXDNFC power control via ioctl
 * CXDNFC_POWER_CTL(1): power on
 * CXDNFC_POWER_CTL(0): power off
 * CXDNFC_WAKE_CTL(1): PON HIGH (normal power mode)
 * CXDNFC_WAKE_CTL(0): PON LOW (low power mode)
 * CXDNFC_WAKE_RST():  assert XRST
 */
#define CXDNFC_POWER_CTL _IO(CXDNFC_MAGIC, 0x01)
#define CXDNFC_WAKE_CTL _IO(CXDNFC_MAGIC, 0x02)
#define CXDNFC_RST_CTL _IO(CXDNFC_MAGIC, 0x03)
#define CXDNFC_GET_STAT _IO(CXDNFC_MAGIC, 0x04)
#define CXDNFC_EXCLRW_REQ _IO(CXDNFC_MAGIC, 0x05)

#define LGE_NFC_FIX
#ifdef LGE_NFC_FIX
#define CXDNFC_SERVICEMODE_CTL       _IO(CXDNFC_MAGIC, 0x10)
#define CXDNFC_BOOTMODE_CTL          _IO(CXDNFC_MAGIC, 0x11)
#endif

#define CXDNFC_RST_ACTIVE 1  /* ActiveHi = 1, ActiveLow = 0 */
#define CXDNFC_WAKE_ACTIVE 1 /* ActiveHi = 1, ActiveLow = 0 */
#define CXDNFC_POWER_ACTIVE 1 /* ActiveHi = 1, ActiveLow = 0 */

#define CXDNFC_STAT_USERS_SHIFT (8)
#define CXDNFC_STAT_USERS_MASK ((0xff) << CXDNFC_STAT_USERS_SHIFT)
#define CXDNFC_STAT_STATE_MASK (0xff)
#define CXDNFC_STAT_STATE_NONE (0x00)    /* probed users=0 */
#define CXDNFC_STAT_STATE_OPEND (0x01)   /* device opend, users>0 */
#define CXDNFC_STAT_STATE_NFCRST (0x02)  /* after CORE_RESET or xRST */
#define CXDNFC_STAT_STATE_NFCINIT (0x03) /* after CORE_INIT */
#define CXDNFC_STAT_STATE_CLOSED (0x04)  /* closed users=0 */

struct cxd22xx_dev;
long cxd22xx_wake(struct cxd22xx_dev *cxd22xx_dev, unsigned long enable_wake);
long cxd22xx_get_status(struct cxd22xx_dev *cxd22xx_dev, unsigned long entry);

#endif
