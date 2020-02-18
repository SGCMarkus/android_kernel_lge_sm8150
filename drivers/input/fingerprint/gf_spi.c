/*Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *     Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
//#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
//#include <linux/wakelock.h>
#include <linux/pm_wakeup.h>
#include "gf_spi.h"
#include "gf_common.h"

//#include <soc/qcom/lge/board_lge.h>

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

#include <linux/async.h>

#define GF_SPIDEV_NAME     "goodix,fingerprint"
/*device name after register in charater*/
#define GF_DEV_NAME            "goodix_fp"
#define	GF_INPUT_NAME	    "goodix_fp_key"	/*"goodix_fp" */

#define	CHRD_DRIVER_NAME	"goodix_fp_spi"
#define	CLASS_NAME		    "goodix_fp"
#define SPIDEV_MAJOR		225
#define N_SPI_MINORS		32


/*GF regs*/
#define GF_CHIP_ID_LO				0x0000
#define GF_CHIP_ID_HI				0x0002
#define GF_IRQ_CTRL1 				0x0120 // LGE_CHANGES - for pin check test
#define GF_IRQ_CTRL2				0x0124
#define GF_IRQ_CTRL3				0x0126
#define GF_IRQ_CTRL4 				0x0128 // LGE_CHANGES - for pin check test

/**************************debug******************************/

#define GF_3208_CHIP_ID (0x00002202)
#define GF_3208_RST_DATA (0x0100)
#define GF_3228_CHIP_ID (0x0000220A)
#define GF_3228_RST_DATA (0x0800)  //3258/3228 same
#define GF_3258_CHIP_ID (0x0000220D)

#define MAX_RETRY_HW_CHECK_COUNT (5)

#define MAX_REST_RETRY_COUNT 3
#define MAX_IRQ_PIN_CHECK_RETRY_COUNT 10

/*Global variables*/
static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static struct gf_dev gf;
static unsigned int bufsiz = 22180;//132*112*1.5+4
//static unsigned char g_frame_buf[22180]= {0};
//static unsigned int g_chipID = 0;
//static struct wake_lock fp_wakelock;
//static unsigned int gf_spi_speed[GF_SPI_KEEP_SPEED] = {4800000, 4800000};
static struct class *gf_class;

struct gf_key_input_map key_input_map[] = {
	{ EV_KEY, GF_KEY_INPUT_PROGRAM },
#if 0 // LGE use KEY_PROGRAM as default
	{ EV_KEY, GF_KEY_INPUT_HOME },
	{ EV_KEY, GF_KEY_INPUT_MENU },
	{ EV_KEY, GF_KEY_INPUT_BACK },
	{ EV_KEY, GF_KEY_INPUT_POWER },
#endif
#if defined(SUPPORT_NAV_EVENT)
	{ EV_KEY, GF_NAV_INPUT_UP },
	{ EV_KEY, GF_NAV_INPUT_DOWN },
	{ EV_KEY, GF_NAV_INPUT_RIGHT },
	{ EV_KEY, GF_NAV_INPUT_LEFT },
	{ EV_KEY, GF_NAV_INPUT_CLICK },
	{ EV_KEY, GF_NAV_INPUT_DOUBLE_CLICK },
	{ EV_KEY, GF_NAV_INPUT_LONG_PRESS },
	{ EV_KEY, GF_NAV_INPUT_HEAVY },
#endif
};

#if defined(USE_SPI_BUS)
static struct spi_driver gf_driver;
#elif defined(USE_PLATFORM_BUS)
static struct platform_driver gf_driver;
#endif
static int gf_probe_fail = 0;

/******************* CLK Related Start ****************/
#if 0
static long spi_clk_max_rate(struct clk *clk, unsigned long rate)
{
	long lowest_available, nearest_low, step_size, cur;
	long step_direction = -1;
	long guess = rate;
	int max_steps = 10;

	FUNC_ENTRY();
	cur = clk_round_rate(clk, rate);
	if (cur == rate)
		return rate;

	/* if we got here then: cur > rate */
	lowest_available = clk_round_rate(clk, 0);
	if (lowest_available > rate)
		return -EINVAL;

	step_size = (rate - lowest_available) >> 1;
	nearest_low = lowest_available;

	while (max_steps-- && step_size) {
		guess += step_size * step_direction;
		cur = clk_round_rate(clk, guess);

		if ((cur < rate) && (cur > nearest_low))
			nearest_low = cur;
		/*
		 * if we stepped too far, then start stepping in the other
		 * direction with half the step size
		 */
		if (((cur > rate) && (step_direction > 0))
				|| ((cur < rate) && (step_direction < 0))) {
			step_direction = -step_direction;
			step_size >>= 1;
		}
	}
	return nearest_low;
}

static void spi_clock_set(struct gf_dev *gf_dev, int speed)
{
	long rate;
	int rc;
	FUNC_ENTRY();

	rate = spi_clk_max_rate(gf_dev->core_clk, speed);
	if (rate < 0) {
		gf_dbg("%s: no match found for requested clock frequency:%d",
				__func__, speed);
		return;
	}

	rc = clk_set_rate(gf_dev->core_clk, rate);
}

static int gfspi_ioctl_clk_init(struct spi_device *spi, struct gf_dev *gf_dev)
{
	gf_dbg("%s: enter\n", __func__);

	FUNC_ENTRY();
	gf_dev->clk_enabled = GF_SPI_CLK_DISABLED;
	gf_dev->core_clk = clk_get(&spi->dev, "core_clk");
	if (IS_ERR_OR_NULL(gf_dev->core_clk)) {
		gf_dbg("%s: fail to get core_clk\n", __func__);
		return -GF_PERM_ERROR;
	}
	gf_dev->iface_clk = clk_get(&spi->dev, "iface_clk");
	if (IS_ERR_OR_NULL(gf_dev->iface_clk)) {
		gf_dbg("%s: fail to get iface_clk\n", __func__);
		clk_put(gf_dev->core_clk);
		gf_dev->core_clk = NULL;
		return -GF_PERM_ERROR; /// -2; ????
	}
	return GF_NO_ERROR;
}

static int gfspi_ioctl_clk_enable(struct gf_dev *gf_dev)
{
	int err;

	gf_dbg("%s: enter\n", __func__);
	FUNC_ENTRY();

	if (gf_dev->clk_enabled == GF_SPI_CLK_ENABLED)
		return 0;

	err = clk_prepare_enable(gf_dev->core_clk);
	if (err) {
		gf_dbg("%s: fail to enable core_clk\n", __func__);
		return -1;
	}

	err = clk_prepare_enable(gf_dev->iface_clk);
	if (err) {
		gf_dbg("%s: fail to enable iface_clk\n", __func__);
		clk_disable_unprepare(gf_dev->core_clk);
		return -2;
	}

	gf_dev->clk_enabled = GF_SPI_CLK_ENABLED;

	return 0;
}

static int gfspi_ioctl_clk_disable(struct gf_dev *gf_dev)
{
	FUNC_ENTRY();

	if (gf_dev->clk_enabled == GF_SPI_CLK_DISABLED)
		return 0;

	clk_disable_unprepare(gf_dev->core_clk);
	clk_disable_unprepare(gf_dev->iface_clk);
	gf_dev->clk_enabled = GF_SPI_CLK_DISABLED;

	return 0;
}

static int gfspi_ioctl_clk_uninit(struct gf_dev *gf_dev)
{
	gf_dbg("%s: enter\n", __func__);

	FUNC_ENTRY();
	if (gf_dev->clk_enabled == GF_SPI_CLK_ENABLED)
		gfspi_ioctl_clk_disable(gf_dev);

	if (!IS_ERR_OR_NULL(gf_dev->core_clk)) {
		clk_put(gf_dev->core_clk);
		gf_dev->core_clk = NULL;
	}

	if (!IS_ERR_OR_NULL(gf_dev->iface_clk)) {
		clk_put(gf_dev->iface_clk);
		gf_dev->iface_clk = NULL;
	}

	return 0;
}
#endif

/******************* CLK Related End ****************/

/******************* Enable/Disable IRQ Start ****************/
static void gf_enable_irq(struct gf_dev *gf_dev)
{
#if 0 // LGE_CHANGES - to reduce debugging log
	FUNC_ENTRY();
#endif
	if (gf_dev->irq_enabled) {
		gf_dbg("IRQ has been enabled.\n");
	} else {
		enable_irq(gf_dev->irq);
		gf_dev->irq_enabled = 1;
	}
#if 0 // LGE_CHANGES - to reduce debugging log
	FUNC_EXIT();
#endif
}

static void gf_disable_irq(struct gf_dev *gf_dev)
{
#if 0 // LGE_CHANGES - to reduce debugging log
	FUNC_ENTRY();
#endif
	if (gf_dev->irq_enabled) {
		gf_dev->irq_enabled = 0;
		disable_irq(gf_dev->irq);
	} else {
		gf_dbg("IRQ has been disabled.\n");
	}
#if 0 // LGE_CHANGES - to reduce debugging log
	FUNC_EXIT();
#endif
}
/******************* Enable/Disable IRQ End ****************/


#if 0
static int gf_get_hw_id(struct gf_dev *gf_dev)
{
	int i;
	unsigned short chip_id_1,chip_id_2;

	gf_hw_reset(gf_dev,5);
	//gf_spi_write_word(gf_dev,GF_IRQ_CTRL2,0xFFFF); /*0x0124,0x0400 clean reset INT*/
	for(i = 0;i < MAX_RETRY_HW_CHECK_COUNT;i++)
	{
		gf_spi_read_word(gf_dev,GF_CHIP_ID_LO,&chip_id_1);
		gf_spi_read_word(gf_dev,GF_CHIP_ID_HI,&chip_id_2);
		g_chipID = ((chip_id_2<<16)|(chip_id_1)) >> 8;
		gf_dbg("%s: chip_id_high = 0x%x,chip_id_low = 0x%x, g_chipID is 0x%08x \n",__func__,chip_id_2,chip_id_1,g_chipID);
		if(g_chipID == GF_3208_CHIP_ID || g_chipID == GF_3228_CHIP_ID || g_chipID == GF_3258_CHIP_ID)
		{
			gf_dbg("succeed to read chip id in probe.\n");
			gf_dev->device_available = GF_DEVICE_AVAILABLE;
			return GF_NO_ERROR;
		}
		gf_hw_reset(gf_dev,5);
	}
	gf_dev->device_available = GF_DEVICE_NOT_AVAILABLE;
	gf_dbg("fail to read chip id in probe.\n");

	return -GF_PERM_ERROR;
}

static int gf_check_rst(struct gf_dev *gf_dev)
{
	int i;

	unsigned short reset_data;
	gf_spi_write_word(gf_dev,GF_IRQ_CTRL2,0x01FF); /*0x0124,0x0400 clean reset INT*/
	for(i = 0;i < MAX_RETRY_HW_CHECK_COUNT;i++)	{
		gf_hw_reset(gf_dev,5);
		if(!gpio_get_value(gf_dev->reset_gpio))	{
			gf_dbg("rst pin check fail.\n");
			continue;
		}

		gf_dbg("rst pin is high.\n");
		gf_spi_read_word(gf_dev,GF_IRQ_CTRL3,&reset_data);

		gf_dbg("%s: reset_irq_data = 0x%x \n",__func__,reset_data);
		if((reset_data != GF_3208_RST_DATA) && (reset_data != GF_3228_RST_DATA)) {
			gf_get_hw_id(gf_dev);
			gf_dbg("chip reset check fail.\n");
			msleep(20);
			continue;
		} else {
			gf_dbg("chip reset check success.\n");
			gf_dev->device_available = 1;
			gf_spi_write_word(gf_dev,GF_IRQ_CTRL2,reset_data); /*0x0124,0x0400 clean reset INT*/
			return GF_NO_ERROR;
		}
	}
	gf_dev->device_available = 0;

	return -GF_PERM_ERROR;
}
#endif

#if 0
static int gf_check_irq(struct gf_dev *gf_dev)
{
	int i, j;
	int gpio_status_prev = 0;
	int gpio_status = 0;
	int result = -GF_PERM_ERROR;
	for(i = 0 ; i < MAX_REST_RETRY_COUNT ; i++) {
		gpio_status_prev = gpio_get_value(gf_dev->irq_gpio);
		gf_dbg("irq_gpio_prev status [%d]", gpio_status_prev);

		gf_spi_write_word(gf_dev,GF_IRQ_CTRL1,0xFFFF); // GF_IRQ_CTRL1 set irq high time
		gf_spi_write_word(gf_dev,GF_IRQ_CTRL4,0x0002);
		gpio_status = gpio_get_value(gf_dev->irq_gpio);

		if(gpio_status_prev == 0 && gpio_status == 0) {
			gf_dbg("start irq pin check.... irq_gpio status [%d]", gpio_status);
			for(j = 0 ; j < MAX_IRQ_PIN_CHECK_RETRY_COUNT ; j++) {
				mdelay(10);
				gpio_status = gpio_get_value(gf_dev->irq_gpio);
				if(gpio_status == 1) {
					gf_dbg("irq_gpio status high..... [%d] %d round", gpio_status, j);
					break;
				}
			}
			if(gpio_status_prev == 0 && gpio_status == 1) {
				gf_dbg("[pass] finish irq_gpio check");
				result = GF_NO_ERROR;
				break;
			} else {
				gf_dbg("reset and retry irq_gpio check");
				gf_hw_reset(gf_dev,5);
				continue;
			}
		} else if(gpio_status_prev == 0 && gpio_status == 1) {
			gf_dbg("irq_gpio status high..... [%d] w/o delay\n", gpio_status);
			gf_dbg("[pass] finish irq_gpio check");
			result = GF_NO_ERROR;
			break;
		} else {
			gf_dbg("irq_gpio_prev status was high..... irq_gpio_ status [%d]", gpio_status);
			gf_hw_reset(gf_dev,5);
			continue;
		}
	}

	gf_spi_write_word(gf_dev,GF_IRQ_CTRL2,0xFFFF); // GF_IRQ_CTRL2 set irq clear
	gpio_status = gpio_get_value(gf_dev->irq_gpio);
	mdelay(10);
	if(gpio_status == 0) {
		gf_dbg("irq_gpio clear success\n");
	} else {
		gf_dbg("irq_gpio clear fail\n");
		result = -GF_PERM_ERROR;
	}

	return result;
}

void gf_spi_setup(struct gf_dev *gf_dev, enum gf_spi_transfer_speed speed)
{

	int ret = 0;

	gf_dbg("####### %s %d \n", __func__, __LINE__);
	if (speed == GF_SPI_KEEP_SPEED)
		return;
	gf_dev->spi->mode = SPI_MODE_0; //CPOL=CPHA=0
	gf_dev->spi->max_speed_hz = gf_spi_speed[speed];
	gf_dev->spi->bits_per_word = 8;
	ret = spi_setup(gf_dev->spi);
	gf_dbg("%s spi_setup ret = %d", __func__, ret);
}
#endif

#if 0
static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct gf_dev *gf_dev = filp->private_data;
	int status = 0;
	int len = 0;

	if(buf == NULL || count > bufsiz)
	{
		gf_dbg("%s input parameters invalid. bufsiz = %d,count = %d \n",
				__func__,bufsiz,(int)count);
		return -EMSGSIZE;
	}

	len = gf_spi_read_data(gf_dev,0xAAAA,count,g_frame_buf);
	status = copy_to_user(buf, g_frame_buf, count);
	if(status != 0) {
		gf_dbg("%s copy_to_user failed. status = %d \n",__func__,status);
		return -EFAULT;
	}
	return 0;
}
#endif

#if defined(SUPPORT_NAV_EVENT)
static void nav_event_input(struct gf_dev *gf_dev, gf_nav_code_t gf_nav_code)
{
	uint32_t nav_input = 0;

	switch (gf_nav_code) {
	case GF_NAV_FINGER_DOWN:
		gf_dbg("%s nav finger down\n", __func__);
		nav_input = GF_KEY_INPUT_PROGRAM;
		//input_report_key(gf_dev->input, nav_input, GF_KEY_STATUS_DOWN);
		//input_sync(gf_dev->input);
		break;

	case GF_NAV_FINGER_UP:
		gf_dbg("%s nav finger up\n", __func__);
		nav_input = GF_KEY_INPUT_PROGRAM;
		//input_report_key(gf_dev->input, nav_input, GF_KEY_STATUS_UP);
		//input_sync(gf_dev->input);
		break;

	case GF_NAV_DOWN:
		nav_input = GF_NAV_INPUT_DOWN;
		gf_dbg("%s nav down\n", __func__);
		break;

	case GF_NAV_UP:
		nav_input = GF_NAV_INPUT_UP;
		gf_dbg("%s nav up\n", __func__);
		break;

	case GF_NAV_LEFT:
		nav_input = GF_NAV_INPUT_LEFT;
		gf_dbg("%s nav left\n", __func__);
		break;

	case GF_NAV_RIGHT:
		nav_input = GF_NAV_INPUT_RIGHT;
		gf_dbg("%s nav right\n", __func__);
		break;

	case GF_NAV_CLICK:
		nav_input = GF_NAV_INPUT_CLICK;
		gf_dbg("%s nav click\n", __func__);
		break;

	case GF_NAV_HEAVY:
		nav_input = GF_NAV_INPUT_HEAVY;
		gf_dbg("%s nav heavy\n", __func__);
		break;

	case GF_NAV_LONG_PRESS:
		nav_input = GF_NAV_INPUT_LONG_PRESS;
		gf_dbg("%s nav long press\n", __func__);
		break;

	case GF_NAV_DOUBLE_CLICK:
		nav_input = GF_NAV_INPUT_DOUBLE_CLICK;
		gf_dbg("%s nav double click\n", __func__);
		break;

	default:
		gf_dbg("%s unknown nav event: %d\n", __func__, gf_nav_code);
		break;
	}

	if ((gf_nav_code != GF_NAV_FINGER_DOWN) && (gf_nav_code != GF_NAV_FINGER_UP) && (gf_nav_code != GF_NAV_HEAVY) && (gf_nav_code != GF_NAV_CLICK) && (gf_nav_code != GF_NAV_DOUBLE_CLICK)) {     //don't report key status
		input_report_key(gf_dev->input, nav_input, GF_KEY_STATUS_DOWN);
		input_sync(gf_dev->input);
		input_report_key(gf_dev->input, nav_input, GF_KEY_STATUS_UP);
		input_sync(gf_dev->input);
	}
}
#endif
static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = &gf;
	gf_key_event_t gf_key_event = { 0 };
#if defined(SUPPORT_NAV_EVENT)
	gf_nav_code_t gf_nav_code = GF_NAV_NONE;
#endif
	int retval = 0;
	//int i;
#if 1 // new_code_porting - added
	u8 netlink_route = NETLINK_TEST;
	struct gf_ioc_chip_info info;
#endif
#ifdef AP_CONTROL_CLK
	unsigned int speed = 0;
#endif
#if 0 // LGE_CHANGES - to reduce debugging log
	FUNC_ENTRY();
#endif
	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENODEV;

	if (_IOC_DIR(cmd) & _IOC_READ)
		retval =
			!access_ok(VERIFY_WRITE, (void __user *)arg,
					_IOC_SIZE(cmd));
	if ((retval == 0) && (_IOC_DIR(cmd) & _IOC_WRITE))
		retval =
			!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (retval)
		return -EFAULT;

	if(gf_dev->device_available == GF_DEVICE_NOT_AVAILABLE)
	{
		if((cmd == GF_IOC_POWER_ON) || (cmd == GF_IOC_POWER_OFF))
		{
			gf_dbg("power cmd\n");
		}
		else
		{
			gf_dbg("Sensor is power off currently. \n");
			return -ENODEV;
		}
	}

	switch (cmd) {
#if 1 // new_code_porting - added
		case GF_IOC_INIT:
			pr_info("%s GF_IOC_INIT .\n", __func__);
			if (copy_to_user((void __user *) arg, (void *) &netlink_route,
						sizeof(u8))) {
				retval = -EFAULT;
				break;
			}
			break;
		case GF_IOC_EXIT:
			pr_info("%s GF_IOC_EXIT .\n", __func__);
			break;
#endif
		case GF_IOC_DISABLE_IRQ:
			gf_dbg("GF_IOC_DISABLE_IRQ");
			gf_disable_irq(gf_dev);
			break;
		case GF_IOC_ENABLE_IRQ:
			gf_dbg("GF_IOC_ENABLE_IRQ");
			gf_enable_irq(gf_dev);
			break;
#if 0 // new_code_porting
		case GF_IOC_SETSPEED:
#if 0 // LGE_CHANGES - to reduce debugging log
			gf_dbg("GF_IOC_SETSPEED");
#ifdef AP_CONTROL_CLK
			retval = __get_user(speed, (u32 __user *) arg);
			if (retval == 0) {
				if (speed > 12 * 1000 * 1000) {
					gf_dbg("Set speed:%d is larger than 8Mbps.\n",	speed);
				} else {
					spi_clock_set(gf_dev, speed);
				}
			} else {
				gf_dbg("Failed to get speed from user. retval = %d\n",	retval);
			}
#else
			gf_dbg("This kernel doesn't support control clk in AP\n");
#endif
#endif
#endif
			break;
		case GF_IOC_RESET:
			gf_dbg("GF_IOC_RESET");
			gf_hw_reset(gf_dev, 1);
			break;
#if 0 // new_code_porting
		case GF_IOC_COOLBOOT:
			gf_dbg("GF_IOC_COOLBOOT");
			gf_power_off(gf_dev);
			mdelay(5);
			gf_power_on(gf_dev);
			break;
#endif
		case GF_IOC_SENDKEY:
			gf_dbg("GF_IOC_SENDKEY");
			if (copy_from_user(&gf_key_event, (gf_key_event_t *)arg, sizeof(gf_key_event_t))) {
				gf_dbg("Failed to copy data from user space.\n");
				retval = -EFAULT;
				break;
			}
			gf_key_event.key = GF_KEY_INPUT_PROGRAM;
			gf_dbg("KEY=%d, gf_key.value = %d", gf_key_event.key, gf_key_event.status);
			input_report_key(gf_dev->input, gf_key_event.key, gf_key_event.status);
			input_sync(gf_dev->input);
			break;

#if defined(SUPPORT_NAV_EVENT)
		case GF_IOC_NAV_EVENT:
		    gf_dbg("%s GF_IOC_NAV_EVENT\n", __func__);
		    if (copy_from_user(&gf_nav_code, (gf_nav_code_t *)arg, sizeof(gf_nav_code_t))) {
			    gf_dbg("Failed to copy nav event from user to kernel\n");
			    retval = -EFAULT;
			    break;
		    }
		    nav_event_input(gf_dev, gf_nav_code);
		    break;
#endif
		case GF_IOC_CLK_READY:
#if 0 // LGE_CHANGES - to reduce debugging log
			gf_dbg("GF_IOC_CLK_READY");
#ifdef AP_CONTROL_CLK
			//gfspi_ioctl_clk_enable(gf_dev);
#else
			gf_dbg("Doesn't support control clock.\n");
#endif
#endif
			break;
		case GF_IOC_CLK_UNREADY:
#if 0 // LGE_CHANGES - to reduce debugging log
			gf_dbg("GF_IOC_CLK_UNREADY");
#ifdef AP_CONTROL_CLK
			gfspi_ioctl_clk_disable(gf_dev);
#else
			gf_dbg("Doesn't support control clock.\n");
#endif
#endif
			break;
#if 0 // new_code_porting
		case GF_IOC_PM_FBCABCK:
			gf_dbg("GF_IOC_PM_FBCABCK");
			__put_user(gf_dev->fb_black, (u8 __user *) arg);
			break;
#endif
		case GF_IOC_POWER_ON:
			gf_dbg("GF_IOC_POWER_ON");
			if(gf_dev->device_available == GF_DEVICE_AVAILABLE)
				gf_dbg("Sensor has already powered-on.\n");
			else
				gf_power_on(gf_dev);
			gf_dev->device_available = GF_DEVICE_AVAILABLE;
			break;
		case GF_IOC_POWER_OFF:
			gf_dbg("GF_IOC_POWER_OFF");
			if(gf_dev->device_available == GF_DEVICE_NOT_AVAILABLE)
				gf_dbg("Sensor has already powered-off.\n");
			else
				gf_power_off(gf_dev);
			gf_dev->device_available = GF_DEVICE_NOT_AVAILABLE;
			break;
#if 1 // new_code_porting - added
		case GF_IOC_ENTER_SLEEP_MODE:
			pr_info("%s GF_IOC_ENTER_SLEEP_MODE. \n", __func__);
			break;
		case GF_IOC_GET_FW_INFO:
			pr_info("%s GF_IOC_GET_FW_INFO. \n", __func__);
			break;
		case GF_IOC_REMOVE:
			pr_info("%s GF_IOC_REMOVE. \n", __func__);
			break;
		case GF_IOC_CHIP_INFO:
			pr_info("%s GF_IOC_CHIP_INFO. \n", __func__);
			if (copy_from_user(&info, (struct gf_ioc_chip_info *) arg,
						sizeof(struct gf_ioc_chip_info))) {
				retval = -EFAULT;
				break;
			}
			pr_info(" vendor_id : 0x%x \n", info.vendor_id);
			pr_info(" mode : 0x%x \n", info.mode);
			pr_info(" operation: 0x%x \n", info.operation);
#endif
			break;
		default:
			gf_dbg("Unsupport cmd:0x%x\n", cmd);
			break;
	}
#if 0 // LGE_CHANGES - to reduce debugging log
	FUNC_EXIT();
#endif
	return retval;
}


#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif /*CONFIG_COMPAT*/

static irqreturn_t gf_irq(int irq, void *handle)
{
#if defined(GF_NETLINK_ENABLE)
	char temp[2] = {GF_NET_EVENT_IRQ, 0};
#if 0 //to reduce debugging log
	gf_dbg("gf_irq \n");
#endif
	//wake_lock_timeout(&fp_wakelock, msecs_to_jiffies(1000));
	struct gf_dev *gf_dev = &gf;
	pm_wakeup_event(&gf_dev->spi->dev,1000);
	sendnlmsg(temp);
#elif defined (GF_FASYNC)
	struct gf_dev *gf_dev = &gf;

	if (gf_dev->async)
		kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif

	return IRQ_HANDLED;
}


static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = -ENXIO;

	FUNC_ENTRY();

	mutex_lock(&device_list_lock);

	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devt == inode->i_rdev) {
			gf_dbg("Found\n");
			status = 0;
			break;
		}
	}

	if (status == 0) {
		if (status == 0) {
			gf_dev->users++;
			filp->private_data = gf_dev;
			nonseekable_open(inode, filp);
			gf_dbg("Succeed to open device. irq = %d\n",
					gf_dev->irq);
			if (gf_dev->users == 1)
				gf_enable_irq(gf_dev);

			gf_power_on(gf_dev);
			gf_hw_reset(gf_dev, 5);
			//add by jicai 20161019  disable_irq to solve  always interrupt
			gf_disable_irq(gf_dev);
			gf_dev->device_available = GF_DEVICE_AVAILABLE;
		}
	} else {
		gf_dbg("No device for minor %d\n", iminor(inode));
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
	struct gf_dev *gf_dev = filp->private_data;
	int ret;

	FUNC_ENTRY();
	ret = fasync_helper(fd, filp, mode, &gf_dev->async);
	FUNC_EXIT();
	gf_dbg("ret = %d\n", ret);
	return ret;
}
#endif

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = 0;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);
	gf_dev = filp->private_data;
	filp->private_data = NULL;

	/*last close?? */
	gf_dev->users--;
	if (!gf_dev->users) {

		gf_dbg("disble_irq. irq = %d\n", gf_dev->irq);
		gf_disable_irq(gf_dev);

		gf_dev->device_available = GF_DEVICE_NOT_AVAILABLE;
		gf_power_off(gf_dev);
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif /*CONFIG_COMPAT*/
	.open = gf_open,
	.release = gf_release,
#if 0
	.read = gf_read,
#endif
#ifdef GF_FASYNC
	.fasync = gf_fasync,
#endif
};

#ifdef GF_CUSTOM_NOTIFIER
static int goodix_fb_state_chg_callback(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct gf_dev *gf_dev;
	struct lge_panel_notifier *evdata = (struct lge_panel_notifier *) data;
	int blank;
#if defined(GF_NETLINK_ENABLE) // fixed build error - label 'temp' defined but not used [-Wunused-label]
	char temp[2] = {0};
#endif
	if (val != LGE_PANEL_EVENT_BLANK)
		return 0;

	gf_dev = container_of(nb, struct gf_dev, notifier);
	if (evdata && val == LGE_PANEL_EVENT_BLANK && gf_dev) {
		blank = evdata->state;
		gf_dbg("[info] %s go to the goodix_fb_state_chg_callback val=[%d], blank=[%d]\n", __func__, (int)val, blank);
		switch (blank) {
			case LGE_PANEL_STATE_LP1: // U2_UNBLANK @ AOD
			case LGE_PANEL_STATE_LP2: // U2_BLANK @ AOD
			case LGE_PANEL_STATE_BLANK: // U0, BLANK
				if (gf_dev->device_available == GF_DEVICE_AVAILABLE) {
					if (gf_dev->fb_black == 1) { break; } // it is already off status
					gf_dev->fb_black = 1;
#if defined(GF_NETLINK_ENABLE)
					temp[0] = GF_NET_EVENT_FB_BLACK;
					sendnlmsg(temp);
#elif defined (GF_FASYNC)
					if (gf_dev->async) {
						kill_fasync(&gf_dev->async, SIGIO,
								POLL_IN);
					}
#endif
					enable_irq_wake(gf_dev->irq);
					gf_dbg("[info] goodix_fb_state_chg_callback enable_irq_wake.\n");
				}
				break;
			case LGE_PANEL_STATE_UNBLANK: // U3, UNBLANK

				if (gf_dev->device_available == GF_DEVICE_AVAILABLE) {
					if (gf_dev->fb_black == 0) { break; }
					gf_dev->fb_black = 0;
#if defined(GF_NETLINK_ENABLE)
					temp[0] = GF_NET_EVENT_FB_UNBLACK;
					sendnlmsg(temp);
#elif defined (GF_FASYNC)
					if (gf_dev->async) {
						kill_fasync(&gf_dev->async, SIGIO,
								POLL_IN);
					}
#endif
					disable_irq_wake(gf_dev->irq);
					gf_dbg("[info] goodix_fb_state_chg_callback disable_irq_wake.\n");
				}
				break;
			default:
				gf_dbg("%s defalut\n", __func__);
				break;
		}
	}
	return NOTIFY_OK;
}
#else
static int goodix_fb_state_chg_callback(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct gf_dev *gf_dev;
	struct fb_event *evdata = data;
	unsigned int blank;
#if defined(GF_NETLINK_ENABLE) // fixed build error - label 'temp' defined but not used [-Wunused-label]
	char temp[2] = {0};
#endif
	if (val != FB_EARLY_EVENT_BLANK)
		return 0;
	gf_dbg("[info] %s go to the goodix_fb_state_chg_callback value = %d\n",
			__func__, (int)val);
	gf_dev = container_of(nb, struct gf_dev, notifier);
	if (evdata && evdata->data && val == FB_EARLY_EVENT_BLANK && gf_dev) {
		blank = *(int *)(evdata->data);
		switch (blank) {
			case FB_BLANK_POWERDOWN:
				if (gf_dev->device_available == GF_DEVICE_AVAILABLE) {
					gf_dev->fb_black = 1;
#if defined(GF_NETLINK_ENABLE)
					temp[0] = GF_NET_EVENT_FB_BLACK;
					sendnlmsg(temp);
#elif defined (GF_FASYNC)
					if (gf_dev->async) {
						kill_fasync(&gf_dev->async, SIGIO,
								POLL_IN);
					}
#endif
				}
				enable_irq_wake(gf_dev->irq);
				break;
			case FB_BLANK_UNBLANK:

				if (gf_dev->device_available == GF_DEVICE_AVAILABLE) {
					gf_dev->fb_black = 0;
#if defined(GF_NETLINK_ENABLE)
					temp[0] = GF_NET_EVENT_FB_UNBLACK;
					sendnlmsg(temp);
#elif defined (GF_FASYNC)
					if (gf_dev->async) {
						kill_fasync(&gf_dev->async, SIGIO,
								POLL_IN);
					}
#endif
				}
				disable_irq_wake(gf_dev->irq);
				break;
			default:
				gf_dbg("%s defalut\n", __func__);
				break;
		}
	}
	return NOTIFY_OK;
}
#endif

static struct notifier_block goodix_noti_block = {
	.notifier_call = goodix_fb_state_chg_callback,
};

static void gf_reg_key_kernel(struct gf_dev *gf_dev)
{
	int i;
	set_bit(EV_KEY, gf_dev->input->evbit);
	for (i = 0; i < ARRAY_SIZE(key_input_map); i++)
		set_bit(key_input_map[i].code, gf_dev->input->keybit);
}
#if 1
static int gf_regulator_init(struct gf_dev *gf_dev)
{
	int error = 0;
	struct regulator *vreg;

	gf_dev->vreg = NULL;
	vreg = regulator_get(&gf_dev->spi->dev, "goodix,vddio");
	if (IS_ERR(vreg)) {
		error = PTR_ERR(vreg);
		gf_dbg("Regulator get failed, error=%d", error);
		return error;
	}


	if (regulator_count_voltages(vreg) > 0) {
		error = regulator_set_voltage(vreg,
				gf_dev->vddio_uV, gf_dev->vddio_uV);
		if (error) {
			gf_dbg("regulator set_vtg failed error=%d", error);
			goto err;
		}
	}

#if 0
	if(regulator_count_voltages(vreg) > 0) {
		error = regulator_set_optimum_mode(vreg, 7000);
		if(error < 0) {
			gf_dbg("unable to set current");
			goto err;
		}
	}
#endif
	gf_dev->vreg = vreg;
	return error;
err:
	regulator_put(vreg);
	return error;
}
#endif

#if 1
static int gf_regulator_set(struct gf_dev *gf_dev, bool enable)
{
	int error = 0;

	gf_dbg("power %s!!", (enable) ? "on" : "off");
	gf_dbg(KERN_DEBUG ": [DEBUG] %s:%i %s gf_regulator_set(): power %s \n", __FILE__, __LINE__, __func__, (enable) ? "on" : "off");

	if(enable) {
		if(!gf_dev->power_on){
			error = regulator_enable(gf_dev->vreg);
			dev_info(&gf_dev->spi->dev, "[INFO] regulator_enable on \n");
		}
	} else {
		if(gf_dev->power_on){
			error = regulator_disable(gf_dev->vreg);
			dev_info(&gf_dev->spi->dev, "[INFO] regulator_enable off \n");
		}
	}

	if(error < 0)
		dev_err(&gf_dev->spi->dev, "can't set(%d) regulator, error(%d)", enable, error);
	else
		gf_dev->power_on = enable;


	dev_info(&gf_dev->spi->dev, "[INFO] regulator value : %d", regulator_get_voltage(gf_dev->vreg));
	return error;
}
#endif

static ssize_t gf_show_qup_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gf_dev *gf_dev = dev_get_drvdata(dev);
// just for debug, for sdm845
	gf_dev->qup_id =0;
	return sprintf(buf, "%d\n", gf_dev->qup_id);
}

static DEVICE_ATTR(qup_id, S_IRUGO,
		gf_show_qup_id, NULL);

/* -------------------------------------------------------------------- */
static ssize_t gf_store_spi_prepare_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	struct gf_dev *gf_dev = dev_get_drvdata(dev);
	int res = 0;
	bool to_tz;

	if(*buf == '1')
		to_tz = true;
	else if(*buf == '0')
		to_tz = false;
	else
		return -EINVAL;

#if 0	// moved to TZ
	res = spi_set_prepare(ix, to_tz);
#else
	/* set spi ownership flag */
	gf_dev->pipe_owner = to_tz;
#endif

	return res ? res : count;
}

static ssize_t gf_show_spi_prepare(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gf_dev *gf_dev = dev_get_drvdata(dev);

	if(gf_dev->pipe_owner)
		return sprintf(buf, "%d \n", TZBSP_TZ_ID);
	else
		return sprintf(buf, "%d \n", TZBSP_APSS_ID);
}

static DEVICE_ATTR(spi_prepare, S_IRUGO | S_IWUSR,
		gf_show_spi_prepare, gf_store_spi_prepare_set);


static struct attribute *gf_attributes[] = {
	&dev_attr_qup_id.attr,
	&dev_attr_spi_prepare.attr,
	NULL
};

static const struct attribute_group gf_attr_group = {
	.attrs = gf_attributes,
};

#if defined(USE_SPI_BUS)
static int gf_initialize_device_data(struct gf_dev* gf_dev, struct spi_device *spi) {
#elif defined(USE_PLATFORM_BUS)
static int gf_initialize_device_data(struct gf_dev* gf_dev, struct platform_device *pdev ) {
#endif
	/* Initialize the driver data */
	INIT_LIST_HEAD(&gf_dev->device_entry);
#if defined(USE_SPI_BUS)
    gf_dev->spi = spi;
#elif defined(USE_PLATFORM_BUS)
    gf_dev->spi = pdev;
#endif
	gf_dev->irq_gpio = -EINVAL;
	gf_dev->reset_gpio = -EINVAL;
	gf_dev->pwr_gpio = -EINVAL;
	gf_dev->device_available = GF_DEVICE_NOT_AVAILABLE;
	gf_dev->fb_black = 0;
	gf_dev->notifier = goodix_noti_block;

	mutex_init(&gf_dev->buf_lock);
	mutex_init(&gf_dev->frame_lock);
	spin_lock_init(&gf_dev->spi_lock);
//	wake_lock_init(&fp_wakelock,WAKE_LOCK_SUSPEND,"fp_wakelock");
	device_init_wakeup(&gf_dev->spi->dev, true);
#ifdef GF_CUSTOM_NOTIFIER
	if(lge_panel_notifier_register_client(&gf_dev->notifier)){
#else
	if(fb_register_client(&gf_dev->notifier)) {
#endif
		pr_err("%s: Unable to register goodix_noti_block notifier\n", __FUNCTION__);
		return -GF_PERM_ERROR;
	}

	return GF_NO_ERROR;

}

#if 0
static bool gf_check_chargerlogo_bootmode(struct gf_dev* gf_dev) {

	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		gf_dbg("CHARGERLOGO mode: skip probe");

		mdelay(1);
		gpio_set_value(gf_dev->reset_gpio, 0);
		gpio_set_value(gf_dev->cs_gpio, 0);
		mdelay(1);

		gf_cleanup(gf_dev);
#ifdef GF_CUSTOM_NOTIFIER
		lge_panel_notifier_unregister_client(&gf_dev->notifier);
#else
		fb_unregister_client(&gf_dev->notifier);
#endif
		return true;
	}
	return false;

}
#endif

#if 0
static int gf_setup_spi_context(struct gf_dev* gf_dev, struct spi_device *spi) {

	/* Enable spi clock */
	if (gfspi_ioctl_clk_init(spi, gf_dev))
		return -GF_PERM_ERROR;

	if (gfspi_ioctl_clk_enable(gf_dev))
		return -GF_PERM_ERROR;

	spi_clock_set(gf_dev, 9600000);

	gf_spi_setup(gf_dev, GF_SPI_HIGH_SPEED);

	spi_set_drvdata(spi, gf_dev);

	return GF_NO_ERROR;
}
#endif

#if 1
static int gf_setup_regulator(struct gf_dev* gf_dev) {
	int status = GF_NO_ERROR;

	if((status = gf_regulator_init(gf_dev)) < 0) {
		dev_err(&gf_dev->spi->dev,"gf_regulator_init err\n");
		return -GF_PERM_ERROR;
	}

	if((status = gf_regulator_set(gf_dev, true)))
	{
		dev_err(&gf_dev->spi->dev,"gf_regulator_set err\n");

		return -GF_PERM_ERROR;
	}
	return GF_NO_ERROR;
}
#endif

#if 0
static int gf_run_oem_test(struct gf_dev* gf_dev){
	int ret = GF_NO_ERROR;

	ret = gf_get_hw_id(gf_dev);
	if(ret)
	{
		gf_dbg("%s gf_get_hw_id failed,probe failed!\n",__func__);
		return -GF_PERM_ERROR;
	}

	ret = gf_check_rst(gf_dev);
	if(ret)
	{
		gf_dbg("%s gf_check_rst failed,probe failed!\n",__func__);
		return -GF_PERM_ERROR;
	}

	ret = gf_check_irq(gf_dev);
	if(ret)
	{
		gf_dbg("%s gf_check_irq failed,probe failed!\n",__func__);
		return -GF_PERM_ERROR;
	}

	return GF_NO_ERROR;
}
#endif

static int gf_setup_input_device(struct gf_dev* gf_dev){

	/*input device subsystem */
	gf_dev->input = input_allocate_device();
	if (gf_dev->input == NULL) {
		gf_dbg("Faile to allocate input device.\n");
		return -ENOMEM;
	}

	gf_dev->input->name = GF_INPUT_NAME;

	input_set_drvdata(gf_dev->input, gf_dev);

	if (input_register_device(gf_dev->input)) {
		gf_dbg("Failed to register GF as input device.\n");
		input_free_device(gf_dev->input);
		return -ENOMEM;
	}

	return GF_NO_ERROR;
}

static int gf_setup_irq_pin(struct gf_dev* gf_dev){
	int ret;
	gf_dev->irq = gf_irq_num(gf_dev);
	ret = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"gf", gf_dev);
	if (!ret) {
		gf_dbg("%s called enable_irq_wake.\n",__func__);
		disable_irq(gf_dev->irq);
		enable_irq(gf_dev->irq);

		gf_dev->irq_enabled = 1;
		enable_irq_wake(gf_dev->irq);
		gf_disable_irq(gf_dev);
	}
	return ret;

}

static int gf_setup_lge_input_device(struct gf_dev* gf_dev){
	int status = GF_NO_ERROR;

	/* register input device */
	gf_dev->lge_input = input_allocate_device();
	if(!gf_dev->lge_input) {
		gf_dbg("[ERROR] input_allocate_deivce failed.\n");
		return -ENOMEM;
	}

	gf_dev->lge_input->name = "fingerprint";
	gf_dev->lge_input->dev.init_name = "lge_fingerprint";

	input_set_drvdata(gf_dev->lge_input, gf_dev);
	status = input_register_device(gf_dev->lge_input);
	if(status) {
		gf_dbg("[ERROR] nput_register_device failed.\n");
		input_free_device(gf_dev->lge_input);
		return -ENOMEM;
	}
	if(sysfs_create_group(&gf_dev->lge_input->dev.kobj, &gf_attr_group)) {
		gf_dbg("[ERROR] sysfs_create_group failed.\n");
		input_unregister_device(gf_dev->lge_input);
		return -ENOMEM;
	}
	return status;
}

static int gf_make_device_node(struct gf_dev* gf_dev) {

	int status = GF_NO_ERROR;
	unsigned long minor;

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_class, &gf_dev->spi->dev, gf_dev->devt,
				gf_dev, GF_DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : GF_NO_ERROR;
	} else {
		dev_dbg(&gf_dev->spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}

	if (status == GF_NO_ERROR) {
		set_bit(minor, minors);
		list_add(&gf_dev->device_entry, &device_list);
	} else {
		gf_dev->devt = 0;
	}
	mutex_unlock(&device_list_lock);

	return status;

}

static int gf_get_alloc_gbuffer(struct gf_dev* gf_dev) {
	gf_dev->gBuffer = kzalloc(bufsiz + GF_RDATA_OFFSET, GFP_KERNEL);
	if(gf_dev->gBuffer == NULL) {
		return -ENOMEM;
	}
	return GF_NO_ERROR;
}

#if 0
static void gf_set_chip_gpio_status(struct gf_dev* gf_dev, int err) {

	gf_dbg("%s: [%d]\n", __FUNCTION__, err);
	if(err == GF_NO_ERROR) {
		gfspi_ioctl_clk_disable(gf_dev);
		mdelay(1);
		gpio_set_value(gf_dev->cs_gpio, 0);
		mdelay(1);
		return;
	}

	if(err == GF_PERM_ERROR) {
		mdelay(1);
		gpio_set_value(gf_dev->reset_gpio, 0);
		gpio_set_value(gf_dev->cs_gpio, 0);
		mdelay(1);
		return;
	}
}
#endif

#if defined(USE_SPI_BUS)
static int gf_probe(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_probe(struct platform_device *pdev)
#endif
{
	struct gf_dev *gf_dev = &gf;

	FUNC_ENTRY();

#if defined(USE_SPI_BUS)
	if(gf_initialize_device_data(gf_dev, spi)) {
#elif defined(USE_PLATFORM_BUS)
	if(gf_initialize_device_data(gf_dev, pdev)) {
#endif
		goto error;
	}

	if (gf_parse_dts(gf_dev))
	{
		gf_dbg("gf_parse_dts error!");
		goto error_parse_dts;
	}
#if 0
	if(gf_check_chargerlogo_bootmode(gf_dev)) {
		gf_dbg("gf_check_chargerlogo_bootmode skip probe!");
		return GF_NO_ERROR;
	}
#endif

#if 1
  if(gf_dev->vddio_uV <= 0)
  {
		gf_dbg("vddio_uV is not available!");
  }
  else
  {
    if(gf_setup_regulator(gf_dev)) {
      goto error_setup_regulator;
    }
  }
#endif
	gf_dbg("vddio_uV :%d", gf_dev->vddio_uV);

	if(gf_dev->vddio_gpio < 0) {
		gf_dbg("vddio_gpio is not available!");
	} else {
		gpio_set_value(gf_dev->vddio_gpio ,1);
	}

	gf_hw_reset(gf_dev, 0);

	if(gf_make_device_node(gf_dev)) {
		goto error_make_device_node;
	}

	if(gf_get_alloc_gbuffer(gf_dev)) {
		goto error_get_alloc_gbuffer;
	}

	if(gf_setup_input_device(gf_dev)) {
		goto error_setup_input_device;
	}

	if(gf_setup_lge_input_device(gf_dev)) {
		goto error_setup_lge_input_device;
	}

#if 0	
	if(gf_setup_spi_context(gf_dev, spi)) {
		goto error_setup_spi_context;
	}
#endif

	gf_reg_key_kernel(gf_dev);

	if(gf_setup_irq_pin(gf_dev)) {
		goto error_setup_irq_pin;
	}
	
#if 0
	if(gf_run_oem_test(gf_dev)) {
		goto error_run_oem_test;
	}

	gf_set_chip_gpio_status(gf_dev, GF_NO_ERROR);
	gf_spi_write_word(gf_dev,GF_IRQ_CTRL2,0xFFFF);
#endif 

	FUNC_EXIT();
	return GF_NO_ERROR;

#if 0
error_run_oem_test:
	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->irq)
		free_irq(gf_dev->irq, gf_dev);
#endif
error_setup_irq_pin:
#if 0
	gfspi_ioctl_clk_disable(gf_dev);
	gfspi_ioctl_clk_uninit(gf_dev);
#endif

#if 0
error_setup_spi_context:
	if (gf_dev->lge_input != NULL) {
		sysfs_remove_group(&gf_dev->lge_input->dev.kobj, &gf_attr_group);
		input_unregister_device(gf_dev->lge_input);
	}
#endif
error_setup_lge_input_device:
	if (gf_dev->input != NULL) {
		input_unregister_device(gf_dev->input);
	}
error_setup_input_device:
	if(gf_dev->gBuffer != NULL) {
		kfree(gf_dev->gBuffer);
	}
error_get_alloc_gbuffer:
	gf_dbg("device_destroy....");
	mutex_lock(&device_list_lock);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);
	mutex_unlock(&device_list_lock);
error_make_device_node:
#if 1
error_setup_regulator:
#endif
#if 0
	gf_set_chip_gpio_status(gf_dev, GF_PERM_ERROR);
#endif 
	gf_cleanup(gf_dev);
#if 1
error_parse_dts:
#ifdef GF_CUSTOM_NOTIFIER
	lge_panel_notifier_unregister_client(&gf_dev->notifier);
#else
	fb_unregister_client(&gf_dev->notifier);
#endif
#endif
error:
	gf_dev->device_available = GF_DEVICE_NOT_AVAILABLE; // ??
//	wake_lock_destroy(&fp_wakelock);
	gf_probe_fail = 1;
	FUNC_EXIT();
	return -GF_PERM_ERROR;
}

#if defined(USE_SPI_BUS)
static int gf_remove(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_remove(struct platform_device *pdev)
#endif
{
	struct gf_dev *gf_dev = &gf;
	FUNC_ENTRY();

//	wake_lock_destroy(&fp_wakelock);
	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->irq)
		free_irq(gf_dev->irq, gf_dev);

	if (gf_dev->input != NULL)
		input_unregister_device(gf_dev->input);
	input_free_device(gf_dev->input);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);
	if (gf_dev->users == 0)
		gf_cleanup(gf_dev);
	gf_dev->device_available = GF_DEVICE_NOT_AVAILABLE;
#ifdef GF_CUSTOM_NOTIFIER
	//lge_panel_notifier_unregister_client(&gf_dev->notifier);
#else
	//fb_unregister_client(&gf_dev->notifier);
#endif
#if 0
	gfspi_ioctl_clk_uninit(gf_dev);
#endif

	if (gf_dev->users == 0) {
		if(gf_dev->gBuffer)
			kfree(gf_dev->gBuffer);
	}
	else {
		gf_dbg("Not free_pages.\n");
	}
#ifdef GF_CUSTOM_NOTIFIER
	lge_panel_notifier_unregister_client(&gf_dev->notifier);
#else
	fb_unregister_client(&gf_dev->notifier);
#endif
	mutex_unlock(&device_list_lock);

	FUNC_EXIT();
	return 0;
}

#if 0
static int gf_suspend(struct spi_device *spi, pm_message_t mesg)
{
	gf_dbg("gf_suspend_test.\n");

	return 0;
}

static int gf_resume(struct spi_device *spi)
{
	gf_dbg("gf_resume_test.\n");

	return 0;
}

#endif
#if 1
static struct of_device_id gx_match_table[] = {
	{.compatible = GF_SPIDEV_NAME,},
	{},
};
#endif


#if defined(USE_SPI_BUS)
static struct spi_driver gf_driver = {
#elif defined(USE_PLATFORM_BUS)
static struct platform_driver gf_driver = {
#endif
	.driver = {
		.name = GF_DEV_NAME,
		.owner = THIS_MODULE,
#if 0
		.bus    = &spi_bus_type,
#endif
		.of_match_table = gx_match_table,
	},
	.probe = gf_probe,
	.remove = gf_remove,
#if 0
	.suspend = gf_suspend,
	.resume = gf_resume,
#endif
};

static void async_gf_init(void *data, async_cookie_t cookie)
{
	int status;
	FUNC_ENTRY();

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);

	if (status < 0) {
		gf_dbg("Failed to register char device!\n");
		FUNC_EXIT();
		return;
	}
	gf_class = class_create(THIS_MODULE, CLASS_NAME);

	if (IS_ERR(gf_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		gf_dbg("Failed to create class.\n");
		FUNC_EXIT();
		return;
	}

#if defined(USE_PLATFORM_BUS)
    status = platform_driver_register(&gf_driver);
#elif defined(USE_SPI_BUS)
    status = spi_register_driver(&gf_driver);
#endif
	if (status < 0) {
		class_destroy(gf_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		gf_dbg("Failed to register SPI driver.\n");
	}

	if (gf_probe_fail) {
		gf_dbg("%s: gf_probe_failed.... status = %d\n", __FUNCTION__, status);

		mutex_lock(&device_list_lock);
		class_destroy(gf_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		mutex_unlock(&device_list_lock);

		FUNC_EXIT();
		return;
	}

#ifdef GF_NETLINK_ENABLE
	netlink_init();
#endif

	gf_dbg(" status = 0x%x\n", status);
	FUNC_EXIT();
	return;
}

static int __init gf_init(void)
{
	async_schedule(async_gf_init, NULL);
	return 0;
}

module_init(gf_init);

static void __exit gf_exit(void)
{
	FUNC_ENTRY();
#ifdef GF_NETLINK_ENABLE
	netlink_exit();
#endif
#if defined(USE_PLATFORM_BUS)
    platform_driver_unregister(&gf_driver);
#elif defined(USE_SPI_BUS)
    spi_unregister_driver(&gf_driver);
#endif
	class_destroy(gf_class);
	unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
	FUNC_EXIT();
}

module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf-spi");
