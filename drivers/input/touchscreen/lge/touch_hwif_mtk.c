/*
 * touch_hwif_mtk.c
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

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_i2c.h>
#include <touch_spi.h>
#include <touch_hwif.h>

/* -- gpio -- */
int touch_gpio_init(int pin, const char *name)
{
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s - pin:%d, name:%s\n", __func__, pin, name);

	if (gpio_is_valid(pin))
		ret = gpio_request(pin, name);

	return ret;
}

int touch_gpio_direction_input(int pin)
{
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s - pin:%d\n", __func__, pin);

	if (gpio_is_valid(pin))
		ret = gpio_direction_input(pin);

	return ret;
}

int touch_gpio_direction_output(int pin, int value)
{
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s - pin:%d, value:%d\n", __func__, pin, value);

	if (gpio_is_valid(pin))
		ret = gpio_direction_output(pin, value);

	return ret;
}

/* -- power -- */
int touch_power_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	if (gpio_is_valid(ts->vcl_pin)) {
		ret = gpio_request(ts->vcl_pin, "touch-vcl");
	} else {
		TOUCH_I("%s vcl - id: %d, vol: %d\n", __func__,
				ts->vcl_id, ts->vcl_vol);
	}

	if (gpio_is_valid(ts->vdd_pin)) {
		ret = gpio_request(ts->vdd_pin, "touch-vdd");
	} else {
		TOUCH_I("%s vdd - id: %d, vol: %d\n", __func__,
				ts->vdd_id, ts->vdd_vol);
	}

	return ret;
}

void touch_power_3_3_vcl(struct device *dev, int value)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	if (gpio_is_valid(ts->vcl_pin)) {
		touch_gpio_direction_output(ts->vcl_pin, value);
	} else if (ts->vcl_id >= 0) {
		TOUCH_I("%s vcl - id: %d, vol: %d\n", __func__,
				ts->vcl_id, ts->vcl_vol);
		if (value)
			hwPowerOn(ts->vcl_id, ts->vcl_vol, "TP");
		else
			hwPowerDown(ts->vcl_id, "TP");
	}
}

void touch_power_1_8_vdd(struct device *dev, int value)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	if (gpio_is_valid(ts->vdd_pin)) {
		touch_gpio_direction_output(ts->vdd_pin, value);
	} else if (ts->vdd_id >= 0) {
		TOUCH_I("%s vdd - id: %d, vol: %d\n", __func__,
				ts->vdd_id, ts->vdd_vol);
		if (value)
			hwPowerOn(ts->vdd_id, ts->vdd_vol, "TP");
		else
			hwPowerDown(ts->vdd_id, "TP");
	}

}

int touch_bus_init(struct device *dev, int buf_size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	if (buf_size) {
		ts->tx_buf = (u8 *)dma_alloc_coherent(NULL,
				buf_size, (dma_addr_t *)(long)&ts->tx_pa, GFP_KERNEL);

		if (!ts->tx_buf)
			TOUCH_E("fail to allocate tx_buf\n");

		ts->rx_buf = (u8 *)dma_alloc_coherent(NULL,
				buf_size, (dma_addr_t *)(long)&ts->rx_pa, GFP_KERNEL);
		if (!ts->rx_buf)
			TOUCH_E("fail to allocate rx_buf\n");

#ifdef CONFIG_PHYS_ADDR_T_64BIT
		TOUCH_I("tx_buf:%p, dma[pa:%08llx]\n",
				ts->tx_buf, ts->tx_pa);
		TOUCH_I("rx_buf:%p, dma[pa:%08llx]\n",
				ts->rx_buf, ts->rx_pa);
#else
		TOUCH_I("tx_buf:%p, dma[pa:%08x]\n",
				ts->tx_buf, ts->tx_pa);
		TOUCH_I("rx_buf:%p, dma[pa:%08x]\n",
				ts->rx_buf, ts->rx_pa);
#endif
	}

	ts->pinctrl.ctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(ts->pinctrl.ctrl)) {
		if (PTR_ERR(ts->pinctrl.ctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		TOUCH_I("Target does not use pinctrl\n");
		ts->pinctrl.ctrl = NULL;
	} else {
		ts->pinctrl.active = pinctrl_lookup_state(ts->pinctrl.ctrl,
				"touch_pin_active");

		if (IS_ERR_OR_NULL(ts->pinctrl.active))
			TOUCH_E("cannot get pinctrl.active\n");

		ts->pinctrl.suspend = pinctrl_lookup_state(ts->pinctrl.ctrl,
				"touch_pin_sleep");

		if (IS_ERR_OR_NULL(ts->pinctrl.suspend))
			TOUCH_E("cannot get pinctrl.suspend\n");

		if (!IS_ERR_OR_NULL(ts->pinctrl.active)) {
			ret = pinctrl_select_state(ts->pinctrl.ctrl,
					ts->pinctrl.active);
			if (ret)
				TOUCH_I("cannot set pinctrl.active\n");
			else
				TOUCH_I("pinctrl set active\n");
		}
	}
	if (ts->bus_type == HWIF_SPI) {
		ret = spi_setup(to_spi_device(dev));

		if (ret < 0) {
			TOUCH_E("Failed to perform SPI setup\n");
			return -ENODEV;
		}
	}

	return ret;
}

int touch_bus_read(struct device *dev, struct touch_bus_msg *msg)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if (ts->bus_type == HWIF_I2C)
		ret = touch_i2c_read(to_i2c_client(dev), msg);
	else if (ts->bus_type == HWIF_SPI) {
		if (atomic_read(&ts->state.pm) >= DEV_PM_SUSPEND) {
			TOUCH_E("bus_read when pm_suspend");
			return -EDEADLK;
		}
		ret = touch_spi_read(to_spi_device(dev), msg);
	}

	return ret;
}

int touch_bus_write(struct device *dev, struct touch_bus_msg *msg)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if (ts->bus_type == HWIF_I2C)
		ret = touch_i2c_write(to_i2c_client(dev), msg);
	else if (ts->bus_type == HWIF_SPI) {
		if (atomic_read(&ts->state.pm) >= DEV_PM_SUSPEND) {
			TOUCH_E("bus_write when pm_suspend");
			return -EDEADLK;
		}
		ret = touch_spi_write(to_spi_device(dev), msg);
	}

	return ret;
}

struct touch_core_data *touch_ts;

void touch_enable_irq_wake(unsigned int irq)
{
}

void touch_disable_irq_wake(unsigned int irq)
{
}

#define istate core_internal_state__do_not_mess_with_it
void touch_enable_irq(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	TOUCH_TRACE();

	if (desc) {
		if (desc->irq_data.chip->irq_unmask)
			desc->irq_data.chip->irq_unmask(&desc->irq_data);

		if (desc->istate & 0x00000200 /*IRQS_PENDING*/)
			TOUCH_D(BASE_INFO, "Remove pending irq(%d)\n", irq);
		desc->istate &= ~(0x00000200);
	}
	enable_irq(irq);
}

void touch_disable_irq(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	TOUCH_TRACE();
	disable_irq_nosync(irq);

	if (desc) {
		if (desc->irq_data.chip->irq_mask)
			desc->irq_data.chip->irq_mask(&desc->irq_data);
	}
}

int touch_request_irq(unsigned int irq, irq_handler_t handler,
		     irq_handler_t thread_fn,
		     unsigned long flags, const char *name, void *dev)
{
	TOUCH_TRACE();
	return request_threaded_irq(irq, handler, thread_fn, flags, name, dev);
}

void touch_set_irq_pending(unsigned int irq)
{
	TOUCH_D(BASE_INFO, "%s is not supported!\n", __func__);
}

void touch_resend_irq(unsigned int irq)
{
	TOUCH_D(BASE_INFO, "%s is not supported!\n", __func__);
}

int touch_check_boot_mode(struct device *dev)
{
	int ret = TOUCH_NORMAL_BOOT;
	unsigned int boot_mode = 0;
	bool factory_boot = false;
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	/* 15 == CHARGERLOGO_BOOT, 110 == LAF_BOOT, defined only in LK   */
	boot_mode = get_boot_mode();
	if (boot_mode == 110) {
		ret = TOUCH_LAF_MODE;
		return ret;
	} else if (boot_mode == RECOVERY_BOOT) {
		ret = TOUCH_RECOVERY_MODE;
		return ret;
#if defined(CONFIG_LGE_PM_CHARGERLOGO)
	} else if (boot_mode == CHARGER_BOOT) {
		ret = TOUCH_CHARGER_MODE;
		return ret;
#endif
	}

	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		ret = TOUCH_CHARGER_MODE;
		return ret;
	}

	factory_boot = lge_get_factory_boot();

	if (factory_boot) {
		switch (atomic_read(&ts->state.mfts)) {
		case MFTS_NONE:
			ret = TOUCH_MINIOS_AAT;
			break;
		case MFTS_FOLDER:
			ret = TOUCH_MINIOS_MFTS_FOLDER;
			break;
		case MFTS_FLAT:
			ret = TOUCH_MINIOS_MFTS_FLAT;
			break;
		case MFTS_CURVED:
			ret = TOUCH_MINIOS_MFTS_CURVED;
			break;
		default:
			ret = TOUCH_MINIOS_AAT;
			break;
		}
	}

	return ret;
}

int touch_bus_xfer(struct device *dev, struct touch_xfer_msg *xfer)
{
	TOUCH_D(BASE_INFO, "%s is not supported!\n", __func__);
	return 0;
}

static struct i2c_board_info touch_i2c_board_info __initdata;
static struct spi_board_info touch_spi_board_info __initdata;

int __init touch_bus_device_init(struct touch_hwif *hwif, void *driver)
{
	TOUCH_TRACE();

	if (hwif->bus_type == HWIF_I2C) {
		struct i2c_board_info board_info = {
			I2C_BOARD_INFO(LGE_TOUCH_NAME, 0x28)
		};

		touch_i2c_board_info = board_info;
		i2c_register_board_info(0 /*touch_i2c_bus_num*/,
				&touch_i2c_board_info, 1);
		return touch_i2c_device_init(hwif, driver);
	} else if (hwif->bus_type == HWIF_SPI) {
		struct spi_board_info board_info = {
			.modalias = LGE_TOUCH_NAME,
			/*
			 * TODO
			.bus_num = touch_spi_bus_num,
			*/
		};

		touch_spi_board_info = board_info;
		spi_register_board_info(&touch_spi_board_info, 1);
		return touch_spi_device_init(hwif, driver);
	}

	TOUCH_E("Unknown touch interface : %d\n", hwif->bus_type);

	return -ENODEV;
}

void touch_bus_device_exit(struct touch_hwif *hwif)
{
	TOUCH_TRACE();

	if (hwif->bus_type == HWIF_I2C)
		touch_i2c_device_exit(hwif);
	else if (hwif->bus_type == HWIF_SPI)
		touch_spi_device_exit(hwif);
}
