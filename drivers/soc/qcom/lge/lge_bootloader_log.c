/*
 * drivers/soc/qcom/lge/lge_bootloader_log.c
 *
 * Copyright (C) 2012 LGE, Inc
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

#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/io.h>
#include <asm/setup.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/platform_device.h>

#define LOGBUF_SIZE   0x20000
#define LOGBUF_SIG    0x6c6f6762

struct log_buffer {
	uint32_t    sig;
	uint32_t    size;
	uint32_t    end;
	uint32_t    start;
};

struct log_buffer *bootlog_buf;

struct bootlog_platform_data {
	unsigned long paddr;
	unsigned long size;
};

#ifdef CONFIG_OF
static int bootlog_parse_dt(struct device *dev, struct device_node *node)
{
	struct bootlog_platform_data *pdata;
	unsigned int mem_addr, mem_size, console_size;

	if (!node) {
		dev_err(dev, "no platform data\n");
		return -EINVAL;
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	/* calculate bootlog addr from ramoops dt
	 * base addr : ramoops,mem-address
	 * offset    : mem-size + console-size
	 */
	of_property_read_u32(node, "mem-address", &mem_addr);
	of_property_read_u32(node, "mem-size", &mem_size);
	of_property_read_u32(node, "console-size", &console_size);
	pdata->paddr = mem_addr + mem_size + console_size;
	pr_info("LGE:%s: bootlog addr %#lx\n", __func__, pdata->paddr);

	dev->platform_data = pdata;

	return 0;
}
#else
static int bootlog_parse_dt(struct device *dev, struct device_node *node)
{
	return -EINVAL;
}
#endif

#ifdef CONFIG_OF
static struct of_device_id bootlog_of_match[] = {
	{.compatible = "bootlog", },
	{ },
};
#endif

static int bootlog_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bootlog_platform_data *pdata = pdev->dev.platform_data;
	struct device_node *node;

	unsigned long paddr, size;
	char *buffer, *token;
	int err;

	node = of_find_compatible_node(NULL, NULL, "ramoops");
	if (node == NULL) {
		pr_err("%s: of_find_node_by_path failed\n", __func__);
		return -EINVAL;
	}

	if (!pdata) {
		err = bootlog_parse_dt(dev, node);
		if (err < 0)
			return err;

		pdata = pdev->dev.platform_data;
	}

	if (!pdata->paddr) {
		pr_err("%s: pdata->paddr is NULL\n", __func__);
		return -EINVAL;
	}

	paddr = pdata->paddr;
	size = pdata->size ? pdata->size : LOGBUF_SIZE;

	if (!request_mem_region(paddr, size, "persistent_ram")) {
		pr_err("request mem region (0x%llx@0x%llx) failed\n",
				(unsigned long long)size,
				(unsigned long long)paddr);
		return -EINVAL;
	}

	bootlog_buf = (struct log_buffer *)ioremap(paddr, size);
	if (bootlog_buf == NULL) {
		pr_info("%s: failed to map memory\n", __func__);
		return 0;
	}

	if (bootlog_buf->sig != LOGBUF_SIG) {
		pr_info("bootlog_buf->sig is not valid (%x)\n",
				bootlog_buf->sig);
		return -EINVAL;
	}

	pr_info("%s: addr(0x%lx) start(0x%x) end(0x%x) size(0x%x)\n", __func__, paddr, bootlog_buf->start, bootlog_buf->end, bootlog_buf->size);
	pr_info("-------------------------------------------------\n");
	pr_info("below logs are got from bootloader\n");
	pr_info("-------------------------------------------------\n");

	buffer = (char *)&bootlog_buf->start;
	buffer[size - sizeof(struct log_buffer) - 1] = '\0';

	while (1) {
		token = strsep(&buffer, "\n");
		if (!token) {
			pr_info("%s: token %p\n", __func__, token);
			break;
		}
		pr_info("%s\n", token);
	}

	pr_info("-------------------------------------------------\n");

	iounmap(bootlog_buf);
	release_region(paddr, size);

	return 0;
}

static struct platform_driver bootlog_driver = {
	.probe		= bootlog_probe,
	.remove		= __exit_p(bootlog_remove),
	.driver		= {
		.name	= "bootlog",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(bootlog_of_match),
#endif
	},
};

static int __init lge_bootlog_init(void)
{
	return platform_driver_register(&bootlog_driver);
}

static void __exit lge_bootlog_exit(void)
{
	platform_driver_unregister(&bootlog_driver);
}

module_init(lge_bootlog_init);
module_exit(lge_bootlog_exit);

MODULE_DESCRIPTION("LGE bootloader log driver");
MODULE_AUTHOR("SungEun Kim <cleaneye.kim@lge.com>");
MODULE_LICENSE("GPL");
