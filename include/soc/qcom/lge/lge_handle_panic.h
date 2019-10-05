/*
 * include/soc/qcom/lge/lge_handle_panic.h
 *
 * Copyright (C) 2016 LG Electronics, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MACH_LGE_HANDLE_PANIC_H
#define __MACH_LGE_HANDLE_PANIC_H

/* sync with boot_images/QcomPkg/SDM845Pkg/Library/XBLRamDumpLib/boot_lge_crash_handler.h */
#define LGE_RB_MAGIC              0x6D630000

#define LGE_ERR_KERN              0x0100
#define LGE_ERR_RPM               0x0200
#define LGE_ERR_TZ                0x0300
#define LGE_ERR_HYP               0x0400
#define LGE_ERR_LAF               0x0500
#define LGE_ERR_UEFI              0x0600

#define LGE_ERR_KERN_KEY          0x0001
#define LGE_ERR_KERN_RPMH_TIMEOUT 0x0002

#define LGE_ERR_TZ_NON_SEC_WDT    0x0001
#define LGE_ERR_TZ_WDT_BARK       0x003A

#define LGE_SUB_ADSP              0x1000
#define LGE_SUB_MBA               0x2000
#define LGE_SUB_MODEM             0x3000
#define LGE_SUB_WCNSS             0x4000
#define LGE_SUB_SLPI              0x5000
#define LGE_SUB_VENUS             0x6000
#define LGE_SUB_CDSP              0x7000

#define LGE_ERR_SUB_SD            0x0001
#define LGE_ERR_SUB_RST           0x0002
#define LGE_ERR_SUB_UNK           0x0003
#define LGE_ERR_SUB_PWR           0x0004
#define LGE_ERR_SUB_TOW           0x0005
#define LGE_ERR_SUB_CDS           0x0006
#define LGE_ERR_SUB_CLO           0x0007

void lge_set_subsys_crash_reason(const char *name, int type);
void lge_set_ram_console_addr(unsigned int addr, unsigned int size);
void lge_set_panic_reason(void);
void lge_set_fb_addr(unsigned int addr);
void lge_set_restart_reason(unsigned int);
int  lge_get_restart_reason(void);
void lge_disable_watchdog(void);
void lge_enable_watchdog(void);
void lge_pet_watchdog(void);
void lge_panic_handler_fb_cleanup(void);

#ifdef CONFIG_LGE_HANDLE_PANIC_RPMH_TIMEOUT
void lge_set_rphm_timeout_panic(const char* str);
#endif

struct panic_handler_data {
	unsigned long	fb_addr;
	unsigned long	fb_size;
};

void lge_gen_key_panic(int key, int status);
#ifdef CONFIG_QCOM_DLOAD_MODE
extern int lge_get_download_mode(void);
extern int lge_set_download_mode(int val);
#else
static int lge_get_download_mode(void) {
	return 0;
}
#endif
extern int qct_wcnss_crash;
#endif
