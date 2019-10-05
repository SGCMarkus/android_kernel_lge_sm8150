#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if 0
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#endif
#include <linux/platform_device.h>

#if 1
int gf_parse_dts(struct gf_dev* gf_dev)
{
	int rc = GF_NO_ERROR;

	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"goodix,gpio_reset",0);
	if(!gpio_is_valid(gf_dev->reset_gpio)) {
		gf_dbg("RESET GPIO is invalid.\n");
		return -GF_PERM_ERROR;
	}
	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if(rc) {
		dev_err(&gf_dev->spi->dev, "Failed to request RESET GPIO. rc = %d\n", rc);
		return -GF_PERM_ERROR;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);


	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"goodix,gpio_irq",0);
	gf_dbg("gf:irq_gpio:%d\n", gf_dev->irq_gpio);
	if(!gpio_is_valid(gf_dev->irq_gpio)) {
		gf_dbg("IRQ GPIO is invalid.\n");
		return -GF_PERM_ERROR;
	}

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if(rc) {
		dev_err(&gf_dev->spi->dev, "Failed to request IRQ GPIO. rc = %d\n", rc);
		return -GF_PERM_ERROR;
	}
	gpio_direction_input(gf_dev->irq_gpio);

/******************************************************************************************
	gf_dev->cs_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"goodix,gpio_cs",0);
	if(!gpio_is_valid(gf_dev->cs_gpio)) {
		gf_dbg("CS GPIO is invalid.\n");
		return -GF_PERM_ERROR;
	}
	rc = gpio_request(gf_dev->cs_gpio, "goodix_cs");
	if(rc) {
		dev_err(&gf_dev->spi->dev, "Failed to request CS GPIO. rc = %d\n", rc);
		return -GF_PERM_ERROR;
	}
	gpio_direction_output(gf_dev->cs_gpio, 0);
*********************** remove the code for TZ5.0 by mono *********************************/
	gf_dev->vddio_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"goodix,gpio_vddio",0);
	gf_dbg("gf:vddio_gpio:%d\n", gf_dev->vddio_gpio);
	if(!gpio_is_valid(gf_dev->vddio_gpio)) {
		gf_dbg("VDDIO GPIO is invalid.\n");
		//return -GF_PERM_ERROR;
	} else {
		gf_dbg("VDDIO GPIO is %d.\n", gf_dev->vddio_gpio);
		rc = gpio_request(gf_dev->vddio_gpio, "goodix_vddio");
		if(rc) {
			dev_err(&gf_dev->spi->dev, "Failed to request VDDIO GPIO. rc = %d\n", rc);
			//return -GF_PERM_ERROR;
		}
		gpio_direction_output(gf_dev->vddio_gpio, 1);
	}

	if((rc = of_property_read_u32(gf_dev->spi->dev.of_node, "gf,vddio-uV", &gf_dev->vddio_uV)) < 0)
	{
		dev_err(&gf_dev->spi->dev,"Error getting vddio_uV\n");
		//return -GF_PERM_ERROR;
	}
	gf_dbg("gf: vddio_uV: %d",gf_dev->vddio_uV);

	return GF_NO_ERROR;
}
#endif

void gf_cleanup(struct gf_dev	* gf_dev)
{
	gf_dbg("[info] %s\n",__func__);
	if (gpio_is_valid(gf_dev->irq_gpio))
	{
		gpio_free(gf_dev->irq_gpio);
		gf_dbg("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio))
	{
		gpio_free(gf_dev->reset_gpio);
		gf_dbg("remove reset_gpio success\n");
	}
/*******************************************************************************************
	if (gpio_is_valid(gf_dev->cs_gpio))
	{
		gpio_free(gf_dev->cs_gpio);
		gf_dbg("remove cs_gpio success\n");
	}
*********************** remove the code for TZ5.0 by mono *********************************/
}


int gf_power_on(struct gf_dev* gf_dev)
{
	int rc = 0;

	msleep(10);
	gf_dbg("---- power on ok ----\n");

	return rc;
}

int gf_power_off(struct gf_dev* gf_dev)
{
	int rc = 0;

	gf_dbg("---- power off ----\n");
	return rc;
}

/********************************************************************
 *CPU output low level in RST pin to reset GF. This is the MUST action for GF.
 *Take care of this function. IO Pin driver strength / glitch and so on.
 ********************************************************************/
int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if(gf_dev == NULL) {
		gf_dbg("Input buff is NULL.\n");
		return -1;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	gf_dbg("RST pin status: %d",gpio_get_value(gf_dev->reset_gpio));
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if(gf_dev == NULL) {
		gf_dbg("Input buff is NULL.\n");
		return -1;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

