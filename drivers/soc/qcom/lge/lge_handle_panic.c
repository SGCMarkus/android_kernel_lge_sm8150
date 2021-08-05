/*
 * drivers/soc/qcom/lge/lge_handle_panic.c
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/memblock.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/workqueue.h>
#include <asm/setup.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/subsystem_restart.h>

#include <soc/qcom/lge/lge_handle_panic.h>

#include <linux/input.h>

#define PANIC_HANDLER_NAME        "panic-handler"

#define RESTART_REASON_ADDR       0x65c

#define CRASH_HANDLER_MAGIC_NUM   0x4c474500
#define CRASH_HANDLER_MAGIC_ADDR  0x54
#define RAM_CONSOLE_ADDR_ADDR     0x58
#define RAM_CONSOLE_SIZE_ADDR     0x5C
#define FB_ADDR_ADDR              0x60
#ifdef CONFIG_LGE_HANDLE_PANIC_RPMH_TIMEOUT
#define RPMH_TIMEOUT_STRING_ADDR  0x6C
/* caution : do NOT exceed struct size to SHARED_IMEM_BOOT_SIZE
 * struct boot_shared_imem_cookie_type
 * boot_images/QcomPkg/XBLLoader/boot_shared_imem_cookie.h */
#define RPMH_TIMEOUT_STRING_SIZE  20
#endif


#define RESTART_REASON      (msm_imem_base + RESTART_REASON_ADDR)
#define CRASH_HANDLER_MAGIC (msm_imem_base + CRASH_HANDLER_MAGIC_ADDR)
#define RAM_CONSOLE_ADDR    (msm_imem_base + RAM_CONSOLE_ADDR_ADDR)
#define RAM_CONSOLE_SIZE    (msm_imem_base + RAM_CONSOLE_SIZE_ADDR)
#define FB_ADDR             (msm_imem_base + FB_ADDR_ADDR)
#ifdef CONFIG_LGE_HANDLE_PANIC_RPMH_TIMEOUT
#define RPMH_TIMEOUT_ADDR   (msm_imem_base + RPMH_TIMEOUT_STRING_ADDR)
#endif
static void *msm_imem_base;
static int dummy_arg;
static int subsys_crash_magic;

static struct panic_handler_data *panic_handler;

#define KEY_CRASH_TIMEOUT 3000
static int gen_key_panic = 0;
static int key_crash_cnt = 0;
static unsigned long key_crash_last_time = 0;


#ifdef CONFIG_LGE_HANDLE_PANIC_RPMH_TIMEOUT
static int rphm_timeout_panic = 0;

void lge_set_rphm_timeout_panic(const char* str)
{
	int i;
	size_t len;
	char buf[RPMH_TIMEOUT_STRING_SIZE] = {0, };

	if (str == NULL) return;

	rphm_timeout_panic = 1;

	len = strlen(str);
	if (len > RPMH_TIMEOUT_STRING_SIZE - 1) {
		char* ptr = (char*)str;
		ptr += (len - RPMH_TIMEOUT_STRING_SIZE + 1);
		strncpy(buf, ptr, RPMH_TIMEOUT_STRING_SIZE - 1);
	} else {
		strncpy(buf, str, RPMH_TIMEOUT_STRING_SIZE -1);
	}

	for(i=0; i<RPMH_TIMEOUT_STRING_SIZE; i++) {
		writeb_relaxed(buf[i], RPMH_TIMEOUT_ADDR+i);
	}
}
#endif

void lge_set_subsys_crash_reason(const char *name, int type)
{
	const char *subsys_name[] =
#if defined (CONFIG_MACH_SM8150_FLASH) || defined (CONFIG_MACH_SM8150_MH2LM_5G)
		{ "adsp", "modem", "wcnss", "slpi", "venus", "cdsp", "esoc0"};
#else
		{ "adsp", "modem", "wcnss", "slpi", "venus", "cdsp"};
#endif
	int i = 0;

	if (!name)
		return;

	for (i = 0; i < ARRAY_SIZE(subsys_name); i++) {
		if (!strncmp(subsys_name[i], name, 40)) {
			subsys_crash_magic = LGE_RB_MAGIC | ((i+1) << 12)
					| type;
			break;
		}
	}
	return;
}

void lge_set_ram_console_addr(unsigned int addr, unsigned int size)
{
	writel_relaxed(addr, RAM_CONSOLE_ADDR);
	writel_relaxed(size, RAM_CONSOLE_SIZE);
}

void lge_set_fb_addr(unsigned int addr)
{
	writel_relaxed(addr, FB_ADDR);
}

void lge_set_restart_reason(unsigned int reason)
{
	writel_relaxed(reason, RESTART_REASON);
}

void lge_set_panic_reason(void)
{
	if (lge_get_download_mode() && gen_key_panic) {
		lge_set_restart_reason(LGE_RB_MAGIC | LGE_ERR_KERN | LGE_ERR_KERN_KEY);
		return;
	}

	if (lge_get_download_mode() && rphm_timeout_panic) {
		lge_set_restart_reason(LGE_RB_MAGIC | LGE_ERR_KERN | LGE_ERR_KERN_RPMH_TIMEOUT);
		return;
	}

	if (subsys_crash_magic == 0)
		lge_set_restart_reason(LGE_RB_MAGIC | LGE_ERR_KERN);
	else
		lge_set_restart_reason(subsys_crash_magic);
}

int lge_get_restart_reason(void)
{
	if (msm_imem_base)
		return readl_relaxed(RESTART_REASON);
	else
		return 0;
}

inline static void lge_set_key_crash_cnt(int key, int* clear)
{
	unsigned long cur_time = 0;
	unsigned long key_crash_gap = 0;

	cur_time = jiffies_to_msecs(jiffies);
	key_crash_gap = cur_time - key_crash_last_time;

	if ((key_crash_cnt != 0) && (key_crash_gap > KEY_CRASH_TIMEOUT)) {
		pr_debug("%s: Ready to panic %d : over time %ld!\n", __func__, key, key_crash_gap);
		return;
	}

	*clear = 0;
	key_crash_cnt++;
	key_crash_last_time = cur_time;

	pr_info("%s: Ready to panic %d : count %d, time gap %ld!\n", __func__, key, key_crash_cnt, key_crash_gap);
}

#ifdef CONFIG_LGE_USB_DEBUGGER
#define VOLUP_KEY_PRESSED 0x1
#define POW_KEY_PRESSED 0x2
int uart_key_press_status=0;
extern int debug_accessory_status;
#else
int debug_accessory_status = 0;
#endif
extern bool unified_nodes_show(const char* key, char* value);
void lge_gen_key_panic(int key, int status)
{
	int clear = 1;
	int order = key_crash_cnt % 3;
	static int valid = 0;

	int ret = -1;
	bool power_supply_present = false;
	char buff [16] = { 0, };

	if (debug_accessory_status) {
		ret = unified_nodes_show("charger_name", buff);
		if (ret == 1 && strcmp(buff, "NONE")) {
			power_supply_present = true;
		} else {
			power_supply_present = false;
		}
	}

	if(!lge_get_download_mode()
		&& !(debug_accessory_status && !power_supply_present))
		return;

#ifdef CONFIG_LGE_USB_DEBUGGER
	if(key == KEY_VOLUMEUP) {
		uart_key_press_status = status ? (uart_key_press_status | VOLUP_KEY_PRESSED) : 0;
	} else if(key == KEY_POWER) {
		uart_key_press_status = status ? (uart_key_press_status | POW_KEY_PRESSED) : 0;
	}
#endif

	if(((key == KEY_VOLUMEDOWN) && (order == 0))
		|| ((key == KEY_POWER) && (order == 1))
		|| ((key == KEY_VOLUMEUP) && (order == 2))) {
		if(status == 0) {
			if(valid == 1) {
				valid = 0;
				lge_set_key_crash_cnt(key, &clear);
			}
		} else {
			valid = 1;
			return;
		}
	}

	if (clear == 1) {
		if (valid == 1)
			valid = 0;

		if (key_crash_cnt > 0 )
			key_crash_cnt = 0;

		pr_debug("%s: Ready to panic %d : cleared!\n", __func__, key);
		return;
	}

	if (key_crash_cnt >= 7) {
		if (debug_accessory_status)
			lge_set_download_mode(1);
		gen_key_panic = 1;
		panic("%s: Generate panic by key!\n", __func__);
	}
}

static int gen_bug(const char *val, const struct kernel_param *kp)
{
	BUG();
	return 0;
}
module_param_call(gen_bug, gen_bug, param_get_bool, &dummy_arg,
		S_IWUSR | S_IRUGO | S_IWGRP);

static int gen_panic(const char *val, const struct kernel_param *kp)
{
	panic("generate test-panic");
	return 0;
}
module_param_call(gen_panic, gen_panic, param_get_bool, &dummy_arg,
		S_IWUSR | S_IRUGO | S_IWGRP);

static int gen_adsp_panic(const char *val, const struct kernel_param *kp)
{
	subsystem_restart("adsp");
	return 0;
}
module_param_call(gen_adsp_panic, gen_adsp_panic, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO | S_IWGRP);

/* remove because gen_mba_panic does not work */
/*
static int gen_mba_panic(const char *val, const struct kernel_param *kp)
{
	subsystem_restart("mba");
	return 0;
}
module_param_call(gen_mba_panic, gen_mba_panic, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO | S_IWGRP);
*/

static int gen_modem_panic(const char *val, const struct kernel_param *kp)
{
	subsystem_restart("modem");
	return 0;
}
module_param_call(gen_modem_panic, gen_modem_panic, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO | S_IWGRP);

#if defined (CONFIG_MACH_SM8150_FLASH) || defined (CONFIG_MACH_SM8150_MH2LM_5G)
static int gen_esoc0_panic(const char *val, const struct kernel_param *kp)
{
	subsystem_restart("esoc0");
	return 0;
}
module_param_call(gen_esoc0_panic, gen_esoc0_panic, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO | S_IWGRP);
#endif

static int gen_wcnss_panic(const char *val, const struct kernel_param *kp)
{
	subsystem_restart("wcnss");
	return 0;
}
module_param_call(gen_wcnss_panic, gen_wcnss_panic, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO | S_IWGRP);

static int gen_slpi_panic(const char *val, const struct kernel_param *kp)
{
	subsystem_restart("slpi");
	return 0;
}
module_param_call(gen_slpi_panic, gen_slpi_panic, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO | S_IWGRP);

static int gen_venus_panic(const char *val, const struct kernel_param *kp)
{
	subsystem_restart("venus");
	return 0;
}
module_param_call(gen_venus_panic, gen_venus_panic, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO | S_IWGRP);

static int gen_cdsp_panic(const char *val, const struct kernel_param *kp)
{
	subsystem_restart("cdsp");
	return 0;
}
module_param_call(gen_cdsp_panic, gen_cdsp_panic, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO | S_IWGRP);

#define WDT0_RST        0x04
#define WDT0_EN         0x08
#define WDT0_BARK_TIME  0x10
#define WDT0_BITE_TIME  0x14

extern void __iomem *wdt_timer_get_timer0_base(void);

static int gen_wdt_bark(const char *val, const struct kernel_param *kp)
{
	void __iomem *msm_tmr0_base;
	msm_tmr0_base = wdt_timer_get_timer0_base();

	pr_info("%s\n", __func__);
	writel_relaxed(0, msm_tmr0_base + WDT0_EN);
	writel_relaxed(1, msm_tmr0_base + WDT0_RST);
	writel_relaxed(0x31F3, msm_tmr0_base + WDT0_BARK_TIME);
	writel_relaxed(5 * 0x31F3, msm_tmr0_base + WDT0_BITE_TIME);
	writel_relaxed(1, msm_tmr0_base + WDT0_EN);
	mb();
	mdelay(5000);

	pr_err("%s failed\n", __func__);

	return -1;
}
module_param_call(gen_wdt_bark, gen_wdt_bark, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO | S_IWGRP);

static int gen_wdt_bite(const char *val, const struct kernel_param *kp)
{
	void __iomem *msm_tmr0_base;
	msm_tmr0_base = wdt_timer_get_timer0_base();

	pr_info("%s\n", __func__);
	writel_relaxed(0, msm_tmr0_base + WDT0_EN);
	writel_relaxed(1, msm_tmr0_base + WDT0_RST);
	writel_relaxed(5 * 0x31F3, msm_tmr0_base + WDT0_BARK_TIME);
	writel_relaxed(0x31F3, msm_tmr0_base + WDT0_BITE_TIME);
	writel_relaxed(1, msm_tmr0_base + WDT0_EN);
	mb();
	mdelay(5000);

	pr_err("%s failed\n", __func__);

	return -1;
}
module_param_call(gen_wdt_bite, gen_wdt_bite, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO | S_IWGRP);

/* forced sec wdt bite can cause unexpected bus hang */
/* remove because gen_sec_wdt_bite does not work */
/*
#define REG_MPM2_WDOG_BASE             0xFC4AA000
#define REG_OFFSET_MPM2_WDOG_RESET     0x0
#define REG_OFFSET_MPM2_WDOG_BITE_VAL  0x10
#define REG_VAL_WDOG_RESET_DO_RESET    0x1
#define REG_VAL_WDOG_BITE_VAL          0x400

static int gen_sec_wdt_bite(const char *val, const struct kernel_param *kp)
{
	void *sec_wdog_virt;
	sec_wdog_virt = ioremap(REG_MPM2_WDOG_BASE, SZ_4K);

	if (!sec_wdog_virt) {
		pr_err("unable to map sec wdog page\n");
		return -ENOMEM;
	}

	pr_info("%s\n", __func__);
	writel_relaxed(REG_VAL_WDOG_RESET_DO_RESET,
		sec_wdog_virt + REG_OFFSET_MPM2_WDOG_RESET);
	writel_relaxed(REG_VAL_WDOG_BITE_VAL,
		sec_wdog_virt + REG_OFFSET_MPM2_WDOG_BITE_VAL);
	mb();
	mdelay(5000);

	pr_err("%s failed\n", __func__);
	iounmap(sec_wdog_virt);

	return -1;
}
module_param_call(gen_sec_wdt_bite, gen_sec_wdt_bite, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO | S_IWGRP);
*/


/* remove because gen_sec_wdt_scm does not work & lockup happen on qfusing device*/
/*
#define SCM_SVC_SEC_WDOG_TRIG  0x08

static int gen_sec_wdt_scm(const char *val, const struct kernel_param *kp)
{
	struct scm_desc desc;
	desc.args[0] = 0;
	desc.arginfo = SCM_ARGS(1);

	lge_disable_watchdog();

	pr_info("%s\n", __func__);
	scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_BOOT,
			SCM_SVC_SEC_WDOG_TRIG), &desc);

	pr_err("%s failed\n", __func__);

	return -1;
}
module_param_call(gen_sec_wdt_scm, gen_sec_wdt_scm, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO | S_IWGRP);
*/

void lge_disable_watchdog(void)
{
	static int once = 1;
	void __iomem *msm_tmr0_base;

	if (once > 1)
		return;

	msm_tmr0_base = wdt_timer_get_timer0_base();
	if (!msm_tmr0_base)
		return;

	writel_relaxed(0, msm_tmr0_base + WDT0_EN);
	mb();
	once++;

	pr_info("%s\n", __func__);
}

void lge_enable_watchdog(void)
{
	static int once = 1;
	void __iomem *msm_tmr0_base;

	if (once > 1)
		return;

	msm_tmr0_base = wdt_timer_get_timer0_base();
	if (!msm_tmr0_base)
		return;

	writel_relaxed(1, msm_tmr0_base + WDT0_EN);
	mb();
	once++;

	pr_info("%s\n", __func__);
}

void lge_pet_watchdog(void)
{
	void __iomem *msm_tmr0_base;
	msm_tmr0_base = wdt_timer_get_timer0_base();

	if (!msm_tmr0_base)
		return;

	writel_relaxed(1, msm_tmr0_base + WDT0_RST);
	mb();

	pr_info("%s\n", __func__);
}

void lge_panic_handler_fb_free_page(unsigned long mem_addr, unsigned long size)
{
	unsigned long pfn_start, pfn_end, pfn_idx;

	pfn_start = mem_addr >> PAGE_SHIFT;
	pfn_end = (mem_addr + size) >> PAGE_SHIFT;
	for (pfn_idx = pfn_start; pfn_idx < pfn_end; pfn_idx++)
		free_reserved_page(pfn_to_page(pfn_idx));
}

void lge_panic_handler_fb_cleanup(void)
{
	static int free = 1;

	if (!panic_handler || free > 1)
		return;

	if (panic_handler->fb_addr && panic_handler->fb_size) {
		memblock_free(panic_handler->fb_addr, panic_handler->fb_size);
		lge_panic_handler_fb_free_page(
				panic_handler->fb_addr, panic_handler->fb_size);
		free++;

		pr_info("%s: free[@0x%lx+@0x%lx)\n", PANIC_HANDLER_NAME,
				panic_handler->fb_addr, panic_handler->fb_size);
	}
}

static int __init lge_panic_handler_early_init(void)
{
	struct device_node *np;

	panic_handler = kzalloc(sizeof(*panic_handler), GFP_KERNEL);
	if (!panic_handler) {
		pr_err("could not allocate memory for panic_handler\n");
		return -ENOMEM;
	}

	np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem");
	if (!np) {
		pr_err("unable to find DT imem node\n");
		return -ENODEV;
	}

	msm_imem_base = of_iomap(np, 0);
	if (!msm_imem_base) {
		pr_err("unable to map imem\n");
		return -ENOMEM;
	}

	/* check struct boot_shared_imem_cookie_type is matched */
	if (readl_relaxed(CRASH_HANDLER_MAGIC) != CRASH_HANDLER_MAGIC_NUM) {
		pr_err("Check sbl's struct boot_shared_imem_cookie_type.\n"
			"Need to update lge_handle_panic's imem offset.\n");
	}

	/* Set default restart_reason to Unknown reset. */
	lge_set_restart_reason(LGE_RB_MAGIC | LGE_ERR_TZ);

#if 0 //not use crash_fb
	np = of_find_compatible_node(NULL, NULL, "crash_fb");
	if (!np) {
		pr_err("unable to find crash_fb node\n");
		return -ENODEV;
	}

	of_property_read_u32(np, "mem-addr", (u32*)&panic_handler->fb_addr);
	of_property_read_u32(np, "mem-size", (u32*)&panic_handler->fb_size);

	pr_info("%s: reserved[@0x%lx+@0x%lx)\n", PANIC_HANDLER_NAME,
			panic_handler->fb_addr, panic_handler->fb_size);

	lge_set_fb_addr(panic_handler->fb_addr);
#endif

	return 0;
}
early_initcall(lge_panic_handler_early_init);

static int __init lge_panic_handler_probe(struct platform_device *pdev)
{
	int ret = 0;

	return ret;
}

static int lge_panic_handler_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver panic_handler_driver __refdata = {
	.probe = lge_panic_handler_probe,
	.remove = lge_panic_handler_remove,
	.driver = {
		.name = PANIC_HANDLER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init lge_panic_handler_init(void)
{
	return platform_driver_register(&panic_handler_driver);
}

static void __exit lge_panic_handler_exit(void)
{
	platform_driver_unregister(&panic_handler_driver);
}

module_init(lge_panic_handler_init);
module_exit(lge_panic_handler_exit);

MODULE_DESCRIPTION("LGE panic handler driver");
MODULE_AUTHOR("SungEun Kim <cleaneye.kim@lge.com>");
MODULE_LICENSE("GPL");
