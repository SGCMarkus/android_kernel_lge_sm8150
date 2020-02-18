/*
 * touch_hwif.h
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

#ifndef LGE_TOUCH_HWIF_H
#define LGE_TOUCH_HWIF_H

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#include <linux/regulator/consumer.h>
#include <soc/qcom/lge/board_lge.h>
#include "../../../../kernel/irq/internals.h"
#endif
#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
#include <mt_gpio.h>
#include <mach/gpio_const.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/irqchip/mt-eic.h>
#include <upmu_common.h>
#include <linux/of_irq.h>
#include <soc/mediatek/lge/lge_boot_mode.h>
#include <mt_boot_common.h>
#endif

extern int touch_gpio_init(int pin, const char *name);
extern int touch_gpio_direction_input(int pin);
extern int touch_gpio_direction_output(int pin, int value);

extern int touch_power_init(struct device *dev);
extern void touch_power_3_3_vcl(struct device *dev, int value);
extern void touch_power_1_8_vdd(struct device *dev, int value);

enum {
	HWIF_I2C = 0,
	HWIF_SPI,
};

struct touch_hwif {
	u8 bus_type;
	const char *name;
	struct module *owner;
	const struct of_device_id *of_match_table;
	u32 max_freq;
	u8 bits_per_word;
	u8 spi_mode;
	void *info;
};

struct touch_bus_msg {
	u8 *tx_buf;
	int tx_size;
	u8 *rx_buf;
	int rx_size;
};

#define MAX_BUF_SIZE		(64 * 1024)
#define MAX_XFER_BUF_SIZE	(1024)
#define MAX_XFER_COUNT		(15)

struct touch_xfer_data_t {
	u16 addr;
	u16 size;
	u8 *buf;
	u8 data[MAX_XFER_BUF_SIZE];
};

struct touch_xfer_data {
	struct touch_xfer_data_t tx;
	struct touch_xfer_data_t rx;
};

struct touch_xfer_msg {
	struct touch_xfer_data data[MAX_XFER_COUNT];
	u8 msg_count;
};

extern void touch_enable_irq_wake(unsigned int irq);
extern void touch_disable_irq_wake(unsigned int irq);
extern void touch_enable_irq(unsigned int irq);
extern void touch_disable_irq(unsigned int irq);
extern int touch_request_irq(unsigned int irq, irq_handler_t handler,
		     irq_handler_t thread_fn,
		     unsigned long flags, const char *name, void *dev);
extern void touch_resend_irq(unsigned int irq);
extern void touch_set_irq_pending(unsigned int irq);
extern int touch_check_boot_mode(struct device *dev);
extern int touch_bus_init(struct device *dev, int buf_size);
extern int touch_bus_read(struct device *dev, struct touch_bus_msg *msg);
extern int touch_bus_write(struct device *dev, struct touch_bus_msg *msg);
extern int touch_bus_xfer(struct device *dev, struct touch_xfer_msg *xfer);
extern int touch_bus_device_init(struct touch_hwif *hwif, void *driver);
extern void touch_bus_device_exit(struct touch_hwif *hwif);

#endif /* LGE_TOUCH_HWIF_H */
