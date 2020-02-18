/*
 * FPGA Manager Driver for Lattice iCE40.
 *
 *  Copyright (c) 2016 Joel Holdsworth
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This driver adds support to the FPGA manager for configuring the SRAM of
 * Lattice iCE40 FPGAs through slave SPI.
 */

#include <linux/fpga/fpga-mgr.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/stringify.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <soc/qcom/lge/board_lge.h>
#include <linux/ice40/ice40-core.h>
#include <linux/fpga/ice40-spi.h>

#define TAG "[ice40] "
#define LOGI(fmt, args...) pr_info(TAG fmt, ##args)
#define LOGE(fmt, args...) pr_err(TAG "[%s %d] " fmt, __func__, __LINE__, ##args)

#define LATTICE_OFFSET		2
#define BUILDDATE_LENGTH	23

#define RESET_GPIO 42
#define LRESET_GPIO 74
#define FLLDO_GPIO 130
#define DDLDO_GPIO 82

#define TEST_FW_PATH "lg_singlewire_m48_impl_2.img" //push it to /etc/firmware with same name.

#define ICE40_SPI_MAX_SPEED 25000000 /* Hz */
#define ICE40_SPI_MIN_SPEED 1000000 /* Hz */

#define ICE40_SPI_RESET_DELAY 1 /* us (>200ns) */
#define ICE40_SPI_HOUSEKEEPING_DELAY 1200 /* us */

#define ICE40_SPI_NUM_ACTIVATION_BYTES DIV_ROUND_UP(100, 8)

enum {
	POWER_OFF = 0,
	POWER_ON,
};

struct ice40_fpga_priv {
	struct spi_device *dev;
	struct gpio_desc *reset;
#ifdef ICE40_USE_CDONE
	struct gpio_desc *cdone;
#endif
	struct gpio_desc *lreset;
	char builddate[BUILDDATE_LENGTH+1];
	struct regulator *vdda33;
	struct gpio_desc *flldo;
	struct gpio_desc *ddldo;
	const char *fw;
};

static struct ice40_fpga_priv *g_ice40_fpga_priv = NULL;
extern struct ice40 *global_ice40;

static void ice40_power(struct device *dev, int ctrl)
{
	struct fpga_manager *mgr = dev_get_drvdata(dev);
	struct ice40_fpga_priv *priv = mgr->priv;
	int ret;

	switch(ctrl)
	{
		case POWER_OFF:
			if( priv->vdda33 ) {
				ret = regulator_disable(priv->vdda33);
				if(ret < 0)
					LOGE("Failed to disable vdd33 supply %d\n", ret);
			}

			if( priv->ddldo ) {
				ret = gpiod_direction_output(priv->ddldo, 0);
				if(ret < 0)
					LOGE("Failed to set ddldo %d\n", ret);
			}

			if( priv->flldo ) {
				ret = gpiod_direction_output(priv->flldo, 0);
				if(ret < 0)
					LOGE("Failed to set flldo %d\n", ret);
			}
			break;
		case POWER_ON:
			if( priv->vdda33 ) {
				ret = regulator_enable(priv->vdda33);
				if(ret < 0)
					LOGE("Failed to enable vdd33 supply %d\n", ret);
			}

			if( priv->ddldo ) {
				ret = gpiod_direction_output(priv->ddldo, 1);
				if(ret < 0)
					LOGE("Failed to set ddldo %d\n", ret);
			}

			if( priv->flldo ) {
				ret = gpiod_direction_output(priv->flldo, 1);
				if(ret < 0)
					LOGE("Failed to set flldo %d\n", ret);
			}
			break;
	}
}

static int ice40_write_firmware(struct device *dev, const char *img)
{
	int ret = 0;
	struct fpga_manager *mgr;
	struct fpga_image_info info;
	info.flags = 0;

	mgr = fpga_mgr_get(dev);
	ret = fpga_mgr_firmware_load(mgr, &info, img);
	if (ret)
		LOGE("Failed to load fpga image: %d\n", ret);

	fpga_mgr_put(mgr);

	return ret;
}

void ice40_set_lreset(int value)
{
    if(g_ice40_fpga_priv)
        gpiod_set_value(g_ice40_fpga_priv->lreset, value);
    else
        LOGE("g_ice40_fpga_priv is null\n");
}
EXPORT_SYMBOL(ice40_set_lreset);

int ice40_get_lreset(void)
{
    int ret = 0;

    if(g_ice40_fpga_priv)
        ret = gpiod_get_value(g_ice40_fpga_priv->lreset);
    else
        LOGE("g_ice40_fpga_priv is null\n");

    return ret;
}
EXPORT_SYMBOL(ice40_get_lreset);

int ice40_reset(void)
{
	int ret = 0;

	ice40_power(&g_ice40_fpga_priv->dev->dev, POWER_OFF);
	ice40_power(&g_ice40_fpga_priv->dev->dev, POWER_ON);
	ret = ice40_write_firmware(&g_ice40_fpga_priv->dev->dev, g_ice40_fpga_priv->fw);
	if (ret)
		LOGE("Failed to ice40_write_firmware: %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(ice40_reset);

static ssize_t show_sanity(struct device *dev, struct device_attribute *attr,
		              char *buf)
{
	int ret = 0;
	int power, download, i2c_ap, i2c_mcu;
	uint udata = 0;
	char data[80] = {0,};
	struct fpga_manager *mgr = dev_get_drvdata(dev);
	struct ice40_fpga_priv *priv = mgr->priv;

	power = download = i2c_ap = i2c_mcu = 1;

	//check power
	if( priv->vdda33 ) {
		if(regulator_is_enabled(priv->vdda33) != 1) power = 0;
	}
	if( priv->flldo ) {
		if(gpiod_get_value(priv->flldo) != 1) power = 0;
	}
	if( priv->ddldo ) {
		if(gpiod_get_value(priv->ddldo) != 1) power = 0;
	}
	ret += snprintf(buf + ret, PAGE_SIZE, "POWER : %s\n", power ? "OK" : "FAIL");

	//check download
	if( mgr->state != FPGA_MGR_STATE_OPERATING ) download = 0;
	ret += snprintf(buf + ret, PAGE_SIZE, "\nFW DOWNLOAD : %s\n", download ? "OK" : "FAIL");

	if(global_ice40->ice40_on) {
		//check i2c to AP lattice
		if( ice40_master_reg_read(global_ice40, 0x00, &udata) < 0 ) i2c_ap = 0;
		if( ice40_master_reg_write(global_ice40, 0x00, udata) < 0 ) i2c_ap = 0;
		ret += snprintf(buf + ret, PAGE_SIZE, "\nI2C TO AP LATTICE : %s\n", i2c_ap ? "OK" : "FAIL");

		//check i2c to MCU
		if( udata & 0x40 ) {
			if( ice40_mcu_reg_read(global_ice40, 0x0003, data, sizeof(data)) < 0 ) i2c_mcu = 0;
			if( ice40_mcu_reg_write(global_ice40, 0x0002, 0x55) < 0 ) i2c_mcu = 0;
			ret += snprintf(buf + ret, PAGE_SIZE, "\nI2C TO MCU: %s\n", i2c_mcu ? "OK" : "FAIL");
		} else {
			ret += snprintf(buf + ret, PAGE_SIZE, "\nDD IS NOT CONNECTED.\nIF DD IS CONNECTED, I2C COMMUNICATION WILL BE CHECKED.\n");
		}
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE, "\nDD IS NOT CONNECTED.\nIF DD IS CONNECTED, I2C COMMUNICATION WILL BE CHECKED.\n");
	}

	return ret;
}

static ssize_t show_fw(struct device *dev, struct device_attribute *attr,
		              char *buf)
{
	int ret = 0;
	struct fpga_manager *mgr = dev_get_drvdata(dev);
	struct ice40_fpga_priv *priv = mgr->priv;

	ret += snprintf(buf + ret, PAGE_SIZE, "%s\n", priv->builddate);

	return ret;
}

static ssize_t store_fw(struct device *dev, struct device_attribute *attr,
		             const char *buf, size_t count)
{
	struct fpga_manager *mgr = dev_get_drvdata(dev);
	struct ice40_fpga_priv *priv = mgr->priv;

	ice40_power(dev, POWER_OFF);
	ice40_power(dev, POWER_ON);
	ice40_write_firmware(dev, priv->fw);

	return count;
}

static ssize_t store_fwadb(struct device *dev, struct device_attribute *attr,
		             const char *buf, size_t count)
{
	ice40_write_firmware(dev, TEST_FW_PATH);

	return count;
}

static ssize_t show_gpio(struct device *dev, struct device_attribute *attr,
		              char *buf)
{
	int ret = 0;
	struct fpga_manager *mgr = dev_get_drvdata(dev);
	struct ice40_fpga_priv *priv = mgr->priv;

	ret += snprintf(buf + ret, PAGE_SIZE, "RESET = %d\n", gpiod_get_value(priv->reset));
	ret += snprintf(buf + ret, PAGE_SIZE, "LRESET = %d\n", gpiod_get_value(priv->lreset));
	if( priv->vdda33 )
		ret += snprintf(buf + ret, PAGE_SIZE, "vdd33 = %d\n", regulator_is_enabled(priv->vdda33));
	if( priv->ddldo )
		ret += snprintf(buf + ret, PAGE_SIZE, "DDLDO = %d\n", gpiod_get_value(priv->ddldo));
	if( priv->flldo )
		ret += snprintf(buf + ret, PAGE_SIZE, "FLLDO = %d\n", gpiod_get_value(priv->flldo));
#ifdef ICE40_USE_CDONE
	ret += snprintf(buf + ret, PAGE_SIZE, "CDONE = %d\n", gpiod_get_value(priv->cdone));
#endif
	return ret;
}

static ssize_t store_gpio(struct device *dev, struct device_attribute *attr,
		             const char *buf, size_t count)
{
	struct fpga_manager *mgr = dev_get_drvdata(dev);
	struct ice40_fpga_priv *priv = mgr->priv;
	int gpio = 0;
	int value = 0;
	int ret = 0;

	ret = sscanf(buf, "%d %d", &gpio, &value);
	if(ret != 2)
		return -EINVAL;

	if ((value > 1) || (value < 0)) {
        LOGE("invalid value(%d)\n", value);
		return count;
	}

	switch(gpio)
	{
		case RESET_GPIO:
			gpiod_set_value(priv->reset, value);
			value = gpiod_get_value(priv->reset);
			LOGI("%s: RESET = %d\n", __func__, value);
			break;
		case LRESET_GPIO:
			gpiod_set_value(priv->lreset, value);
			value = gpiod_get_value(priv->lreset);
			LOGI("%s: LRESET = %d\n", __func__, value);
			break;
		case FLLDO_GPIO:
			if(priv->flldo) {
				gpiod_direction_output(priv->flldo, value);
				value = gpiod_get_value(priv->flldo);
				LOGI("%s: FLLDO = %d\n", __func__, value);
			} else {
				LOGI("%s: There is no FLLDO\n", __func__);
			}
			if( value == 1) {
				if(priv->vdda33)
					regulator_enable(priv->vdda33);
			} else {
				if(priv->vdda33)
					regulator_disable(priv->vdda33);
			}
			if(priv->vdda33)
				LOGI("%s : vdda33 = %d\n", __func__, regulator_is_enabled(priv->vdda33));
			break;
		case DDLDO_GPIO:
			if(priv->ddldo) {
				gpiod_direction_output(priv->ddldo, value);
				value = gpiod_get_value(priv->ddldo);
				LOGI("%s: DDLDO = %d\n", __func__, value);
			} else {
				LOGI("%s: There is no DDLDO\n", __func__);
			}
			break;
	}

	return count;
}

static DEVICE_ATTR(fw, S_IRUGO | S_IWUSR, show_fw, store_fw);
static DEVICE_ATTR(fwadb, S_IRUGO | S_IWUSR, NULL, store_fwadb);
static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR, show_gpio, store_gpio);
static DEVICE_ATTR(sanity, S_IRUGO | S_IWUSR, show_sanity, NULL);

static struct attribute *dev_attrs[] = {
	    &dev_attr_fw.attr,
	    &dev_attr_fwadb.attr,
	    &dev_attr_gpio.attr,
	    &dev_attr_sanity.attr,
		    NULL,
};

static struct attribute_group dev_attr_grp = {
	    .attrs = dev_attrs,
};

static enum fpga_mgr_states ice40_fpga_ops_state(struct fpga_manager *mgr)
{
#ifdef ICE40_USE_CDONE
	struct ice40_fpga_priv *priv = mgr->priv;

	return gpiod_get_value(priv->cdone) ? FPGA_MGR_STATE_OPERATING :
		FPGA_MGR_STATE_UNKNOWN;
#else
	return FPGA_MGR_STATE_OPERATING;
#endif
}

static int ice40_fpga_ops_write_init(struct fpga_manager *mgr,
				     struct fpga_image_info *info,
				     const char *buf, size_t count)
{
	struct ice40_fpga_priv *priv = mgr->priv;
	struct spi_device *dev = priv->dev;
	struct spi_message *message;
	struct spi_transfer assert_cs_then_reset_delay = {
		.cs_change   = 1,
		.delay_usecs = ICE40_SPI_RESET_DELAY
	};
	struct spi_transfer housekeeping_delay_then_release_cs = {
		.delay_usecs = ICE40_SPI_HOUSEKEEPING_DELAY
	};
	int ret;

	LOGI("%s\n", __func__);
	if ((info->flags & FPGA_MGR_PARTIAL_RECONFIG)) {
		LOGE("Partial reconfiguration is not supported\n");
		return -ENOTSUPP;
	}

	message = kmalloc(sizeof(struct spi_message), GFP_KERNEL);
	if(!message) {
		LOGE("Failed to allocate message\n");
		return -ENOMEM;
	}

	gpiod_set_value(priv->lreset, 0);
	/* Lock the bus, assert CRESET_B and SS_B and delay >200ns */
	spi_bus_lock(dev->master);

	gpiod_set_value(priv->reset, 0);

	spi_message_init(message);
	spi_message_add_tail(&assert_cs_then_reset_delay, message);
	ret = spi_sync_locked(dev, message);

	/* Come out of reset */
	gpiod_set_value(priv->reset, 1);

	/* Abort if the chip-select failed */
	if (ret) {
		LOGE("Abort if the chip-select failed: %d\n", ret);
		goto fail;
	}

	/* Check CDONE is de-asserted i.e. the FPGA is reset */
#ifdef ICE40_USE_CDONE
	if (gpiod_get_value(priv->cdone)) {
		dev_err(&dev->dev, "Device reset failed, CDONE is asserted\n");
		ret = -EIO;
		goto fail;
	}
#endif

	/* Wait for the housekeeping to complete, and release SS_B */
	spi_message_init(message);
	spi_message_add_tail(&housekeeping_delay_then_release_cs, message);
	ret = spi_sync_locked(dev, message);

fail:
	spi_bus_unlock(dev->master);
	kfree(message);

	return ret;
}

//strstr seems to stop searching if needle meets a null character.
static char* my_strstr(const char *buf, const char *search)
{
	char c, sc;
	int len, cnt = 0;

	c = *search;
	len = strlen(search);
	do {
		do {
			sc = *buf++;
			cnt++;
		} while (sc != c && cnt < 0x100);
	} while (strncmp((buf-1), search, len) != 0 && cnt < 0x100);

	if(cnt >= 0x100) {
		LOGI("Fail to search %s\n", search);
		return NULL;
	}
	else
		return (char *)--buf;
}

static int ice40_fpga_ops_write(struct fpga_manager *mgr,
				const char *buf, size_t count)
{
	struct ice40_fpga_priv *priv = mgr->priv;
	char *p = my_strstr(buf, "Date");

	if(p)
		strncpy(priv->builddate, p, BUILDDATE_LENGTH);

	LOGI("%s %s %zu %s\n",(buf + LATTICE_OFFSET), priv->builddate, count, __func__);

	return spi_write(priv->dev, buf, count);
}

static int ice40_fpga_ops_write_complete(struct fpga_manager *mgr,
					 struct fpga_image_info *info)
{
	int ret;
	struct ice40_fpga_priv *priv = mgr->priv;
	struct spi_device *dev = priv->dev;
	u8 *padding;

	LOGI("%s\n", __func__);
	/* Check CDONE is asserted */
#ifdef ICE40_USE_CDONE
	if (!gpiod_get_value(priv->cdone)) {
		dev_err(&dev->dev,
			"CDONE was not asserted after firmware transfer\n");
		return -EIO;
	}
#endif

	padding = kzalloc(sizeof(u8)*ICE40_SPI_NUM_ACTIVATION_BYTES, GFP_KERNEL);
	if(!padding) {
		LOGE("Failed to allocate padding\n");
		return -ENOMEM;
	}

	/* Send of zero-padding to activate the firmware */
	ret = spi_write(dev, padding, sizeof(padding));
#ifdef ICE40_USE_CDONE
	if (gpiod_get_value(priv->cdone))
		LOGI("CDONE High\n");
	else
		LOGI("CDONE Low\n");
#endif
	gpiod_set_value(priv->lreset, 1);
	mdelay(50);
	gpiod_set_value(priv->lreset, 0);

	kfree(padding);

	return ret;
}

static const struct fpga_manager_ops ice40_fpga_ops = {
	.state = ice40_fpga_ops_state,
	.write_init = ice40_fpga_ops_write_init,
	.write = ice40_fpga_ops_write,
	.write_complete = ice40_fpga_ops_write_complete,
};

static int ice40_fpga_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ice40_fpga_priv *priv;
	int ret;
	struct device_node *np = dev->of_node;

	LOGI("%s\n", __func__);
	spi_setup(spi);
	priv = devm_kzalloc(&spi->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		LOGE("priv devm_kzalloc failed\n");
		return -ENOMEM;
	}
	g_ice40_fpga_priv = priv;
	priv->dev = spi;

	/* Check board setup data. */
	if (spi->max_speed_hz > ICE40_SPI_MAX_SPEED) {
		LOGE("SPI speed is too high, maximum speed is "
			__stringify(ICE40_SPI_MAX_SPEED) "\n");
		return -EINVAL;
	}

	if (spi->max_speed_hz < ICE40_SPI_MIN_SPEED) {
		LOGE("SPI speed is too low, minimum speed is "
			__stringify(ICE40_SPI_MIN_SPEED) "\n");
		return -EINVAL;
	}

	if (spi->mode & SPI_CPHA) {
		LOGE("Bad SPI mode, CPHA not supported\n");
		return -EINVAL;
	}

	/* Set up the GPIOs */
#ifdef ICE40_USE_CDONE
	priv->cdone = devm_gpiod_get(dev, "cdone", GPIOD_IN);
	if (IS_ERR(priv->cdone)) {
		ret = PTR_ERR(priv->cdone);
		dev_err(dev, "Failed to get CDONE GPIO: %d\n", ret);
		return ret;
	}
#endif

	priv->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset)) {
		ret = PTR_ERR(priv->reset);
		LOGE("Failed to get CRESET_B GPIO: %d\n", ret);
		return ret;
	}

	priv->lreset = devm_gpiod_get(dev, "lreset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->lreset)) {
		ret = PTR_ERR(priv->lreset);
		LOGE("Failed to get LATTICE_RESET GPIO: %d\n", ret);
		return ret;
	}

	// Though vdda33-supply is not exist, dev_regulator_get returns dummy regulator.
	// priv->vdda33 is not NULL, check vdda33-supply property.
	// kernel message : spi1.2 supply vdda33 not found, using dummy regulator
	if(of_get_property(np, "vdda33-supply", NULL)) {
		priv->vdda33 = devm_regulator_get(dev, "vdda33");
		if(IS_ERR(priv->vdda33)) {
			ret = PTR_ERR(priv->vdda33);
			LOGI("Failed to get vdda33 supply\n", ret);
		}
	}

	if(of_get_property(np, "flldo-gpios", NULL)) {
		priv->flldo = devm_gpiod_get(dev, "flldo", GPIOD_OUT_LOW);
		if (IS_ERR(priv->flldo)) {
			ret = PTR_ERR(priv->flldo);
			LOGI("Failed to get FL_LDO_EN GPIO: %d\n", ret);
		}
	}

	if(of_get_property(np, "ddldo-gpios", NULL)) {
		priv->ddldo = devm_gpiod_get(dev, "ddldo", GPIOD_OUT_LOW);
		if (IS_ERR(priv->ddldo)) {
			ret = PTR_ERR(priv->ddldo);
			LOGI("Failed to get DD_LDO_EN GPIO: %d\n", ret);
		}
	}

	ret = of_property_read_string(np, "fw", &priv->fw);
	if(ret < 0) {
		LOGE("Failed to get firmware image path: %d\n", ret);
		return ret;
	}

	/* Register with the FPGA manager */
	if((ret = fpga_mgr_register(dev, "Lattice iCE40 FPGA Manager", &ice40_fpga_ops, priv)) != 0) {
		return ret;
	}

	ret = sysfs_create_group(&spi->dev.kobj, &dev_attr_grp);
	if (ret) {
		 LOGE("failed to create dev. attrs : %d\n", ret);
		 return ret;
	}

	ice40_power(dev, POWER_ON);

	ret = ice40_write_firmware(dev, priv->fw);

	return ret;
}

static int ice40_fpga_remove(struct spi_device *spi)
{
	fpga_mgr_unregister(&spi->dev);
	return 0;
}

#ifdef CONFIG_PM
static int ice40_spi_runtime_suspend(struct device *dev)
{
	//struct fpga_manager *mgr = dev_get_drvdata(dev);
	//struct ice40_fpga_priv *priv = mgr->priv;

	//gpiod_set_value(priv->lreset, 0);

	return 0;
}

static int ice40_spi_runtime_resume(struct device *dev)
{
	//struct fpga_manager *mgr = dev_get_drvdata(dev);
	//struct ice40_fpga_priv *priv = mgr->priv;

	//gpiod_set_value(priv->lreset, 1);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ice40_spi_suspend(struct device *dev)
{
	//struct fpga_manager *mgr = dev_get_drvdata(dev);
	//struct ice40_fpga_priv *priv = mgr->priv;

	//gpiod_set_value(priv->lreset, 0);

	return 0;
}

static int ice40_spi_resume(struct device *dev)
{

	//struct fpga_manager *mgr = dev_get_drvdata(dev);
	//struct ice40_fpga_priv *priv = mgr->priv;

	//gpiod_set_value(priv->lreset, 1);

	return 0;
}
#endif


static const struct dev_pm_ops ice40_spi_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(ice40_spi_suspend, ice40_spi_resume)
    SET_RUNTIME_PM_OPS(ice40_spi_runtime_suspend, ice40_spi_runtime_resume, NULL)
};
#define ICE40_SPI_PM_OPS    (&ice40_spi_pm_ops)
#else
#define ICE40_SPI_PM_OPS    NULL
#endif

static const struct of_device_id ice40_fpga_of_match[] = {
	{ .compatible = "lattice,ice40-fpga-mgr", },
	{},
};
MODULE_DEVICE_TABLE(of, ice40_fpga_of_match);

static struct spi_driver ice40_fpga_driver = {
	.probe = ice40_fpga_probe,
	.remove = ice40_fpga_remove,
	.driver = {
		.name = "ice40spi",
		.pm = ICE40_SPI_PM_OPS,
		.of_match_table = of_match_ptr(ice40_fpga_of_match),
	},
};

module_spi_driver(ice40_fpga_driver);

MODULE_AUTHOR("Joel Holdsworth <joel@airwebreathe.org.uk>");
MODULE_DESCRIPTION("Lattice iCE40 FPGA Manager");
MODULE_LICENSE("GPL v2");
