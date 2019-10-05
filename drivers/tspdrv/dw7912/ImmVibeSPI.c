/*
** =============================================================================
**
** File: ImmVibeSPI.c
**
** Description:
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
**
** Copyright (c) 2018 Immersion Corporation. All Rights Reserved.
**
** This file contains Original Code and/or Modifications of Original Code
** as defined in and that are subject to the GNU Public License v2 -
** (the 'License'). You may not use this file except in compliance with the
** License. You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES,
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see
** the License for the specific language governing rights and limitations
** under the License.
**
** =============================================================================
*/

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#if defined(CONFIG_MACH_LGE)
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include <linux/uaccess.h>
#include <linux/pm_wakeup.h>
#include <linux/of.h>

#if 1
	#define gprintk(fmt, x... ) printk( "%s: " fmt, __FUNCTION__ , ## x)
#else
	#define gprintk(x...) do { } while (0)
#endif

#define IMMVIBESPI_DEVICE_GETSTATUS_SUPPORT

#define DW7912_INFO             0x00
#define DW7912_VERSION          0x00
#define DW7912_STATUS           0x01
#define DW7912_FIFOFULL         0x01
#define DW7912_MODE				0x03
#define DW7912_DATA             0x0A
#define DW7912_SWRESET          0x2F
#define DW7912_LDO              0x08
#define SW_RESET_COMMAND        0x01 /* Reset */
#define HW_RESET_ENABLE_COMMAND 0x00 /* Enable hardware */
#define RTP_MODE				0x00
#define MEM_MODE				0x01
#endif

/*
** i2c bus
*/
#if defined(CONFIG_MACH_LGE)
#define DEVICE_BUS  4
#else
#define DEVICE_BUS  3
#endif

/*
** i2c device address
*/
#if defined(CONFIG_MACH_LGE)
#define DEVICE_ADDR 0x59
#else
#define DEVICE_ADDR 0x5A
#endif

/*
** Number of actuators this SPI supports
*/
#define NUM_ACTUATORS 1

/*
** Name displayed in TouchSense API and tools
*/
#define DEVICE_NAME "LG Alpha"

/*
** Manage amplifier buffer using ImmVibeSPI_ForceOut_BufferFull
*/
#define IMMVIBESPI_USE_BUFFERFULL

/*
** Number of sample periods to prebuffer before sending
*/
#define PREBUFFER_SAMPLE_PERIODS 3

/*
** Higher values buffer more future samples.
*/
#define DW7912_FIFOFULL_TARGET (PREBUFFER_SAMPLE_PERIODS * VIBE_OUTPUT_SAMPLE_SIZE)

/*
** Upscale immvibed data because it supports up to 12kHz (and 24kHz is less audible)
*/
#define UPSCALE_12_TO_24khz

#define NAK_RESEND_ATTEMPT 3

VibeStatus I2CWrite(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer);
static void i2c_commands(struct i2c_client *i2c, struct i2c_msg *msgs, u8 *command, int size);
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer);

struct dw7912_dev {
#if defined(CONFIG_MACH_LGE)
    int use_en_pin; /* en gpio enable/disable */
    int enable_pin; /* if use_en_pin is 1, this pin used */
#endif
    unsigned char chipid;
    unsigned char i2c_addr;
    struct i2c_client *i2c;
    ktime_t time;
    int buffering;
    s64 position;
#ifdef UPSCALE_12_TO_24khz
    VibeInt8 buffer[1 + DW7912_FIFOFULL_TARGET*2];
#else
    VibeInt8 buffer[1 + DW7912_FIFOFULL_TARGET];
#endif
};

static struct dw7912_dev *dw7912;

/*
** I2C Transfer Functions
*/

static int i2c_recv_buf(struct i2c_client *i2c, unsigned char reg, unsigned char *buf, int count)
{
    struct i2c_msg msg[2];

    msg[0].addr = i2c->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &reg;

    msg[1].addr = i2c->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = count;
    msg[1].buf = buf;

    return i2c_transfer(i2c->adapter, msg, 2);
}

static int i2c_send_buf(struct i2c_client *i2c, unsigned char *buf, int count)
{
    struct i2c_msg msg;

    msg.addr = i2c->addr;
    msg.flags = 0;
    msg.len = count;
    msg.buf = buf;

    return i2c_transfer(i2c->adapter, &msg, 1);
}

#define DW7912_CHIPID                0x00
#define DW7912_STATUS                0x01
#define DW7912_INT_EN                0x02

#define DW7912_MODE                  0x03
#define DW7912_MODE_RTP              0x00
#define DW7912_MODE_MEMORY           0x01
#define DW7912_MODE_TRIG_EXT         0x00  /* INT/TRIG pin is an input */
#define DW7912_MODE_TRIG_INT         0x10  /* INT/TRIG pin is an output */
#define DW7912_MODE_TRIG_PULSE       0x00
#define DW7912_MODE_TRIG_EDGE        0x20

#define DW7912_PWM_FREQ              0x04
#define DW7912_PWM_FREQ_24kHz        0x00
#define DW7912_PWM_FREQ_48kHz        0x01
#define DW7912_PWM_FREQ_96kHz        0x02
#define DW7912_PWM_FREQ_12kHz        0x03

#define DW7912_BOOST_OUTPUT          0x05
#define DW7912_BOOST_OUTPUT_4_68     0x00
#define DW7912_BOOST_OUTPUT_6_36     0x4F
#define DW7912_BOOST_OUTPUT_7_00     0x57
#define DW7912_BOOST_OUTPUT_8_60     0x6B
#define DW7912_BOOST_OUTPUT_10_2     0x7F

#define DW7912_BOOST_MODE            0x06
#define DW7912_BOOST_MODE_VBST       0x00
#define DW7912_BOOST_MODE_BST_EN     0x01
#define DW7912_BOOST_MODE_BST_BYPASS 0x02
#define DW7912_BOOST_MODE_BST_LUMP   0x04
#define DW7912_BOOST_MODE_BST_ADAPT  0x08

#define DW7912_VMH                   0x07
#define DW7912_VD_CLAMP              0x08
#define DW7912_VD_CLAMP_mV(x)        ((x)/40)

#define DW7912_PLAYBACK              0x09
#define DW7912_PLAYBACK_GO           0x01
#define DW7912_PLAYBACK_STOP         0x00

#define DW7912_RTP_INPUT             0x0A
#define DW7912_MEM_GAIN              0x0B /* ignored in RTP mode */

#define DW7912_BOOST_OPTION                0x23
#define DW7912_BOOST_OPTION_BST_OFFSET_mV(x) ((x)/80)   /* 80 mV increments */
#define DW7912_BOOST_OPTION_BST_ILIMIT_30  0x00
#define DW7912_BOOST_OPTION_BST_ILIMIT_25  0x20
#define DW7912_BOOST_OPTION_BST_ILIMIT_20  0x40
#define DW7912_BOOST_OPTION_BST_ILIMIT_15  0x60
#define DW7912_BOOST_OPTION_BST_ILIMIT_35  0x80

#define DW7912_SWRST                 0x2F
#define DW7912_SWRST_SW_RESET        0x01

#define DW7912_DELAY_SAMPLES_US(us) ((us) * 60 / 5000)

#ifdef UPSCALE_12_TO_24khz
#define CHOSEN_SAMPLE_FREQUENCY DW7912_PWM_FREQ_24kHz
#else
#define CHOSEN_SAMPLE_FREQUENCY DW7912_PWM_FREQ_12kHz
#endif

#if !defined(CONFIG_MACH_LGE)
static void dw7912_dump_registers(struct dw7912_dev *dev)
{
    int i = 0;
    char buf = 0;
    char regs[] = {
        DW7912_CHIPID,
        DW7912_STATUS,
        DW7912_INT_EN,
        DW7912_MODE,
        DW7912_PWM_FREQ,
        DW7912_BOOST_OUTPUT,
        DW7912_BOOST_MODE,
        DW7912_VMH,
        DW7912_VD_CLAMP,
        DW7912_BOOST_OPTION
    };

    if (!dev || !dev->i2c) return;

    for (i = 0; i < sizeof(regs); i++) {
        i2c_recv_buf(dev->i2c, regs[i], &buf, sizeof(buf));
        DbgOut((DBL_INFO, "dw7912: reg[0x%02x]=0x%02x\n", i, buf));
    }
}
#endif

/*
** I2C Driver
*/

#if defined(CONFIG_MACH_LGE)

static void dw7912_poweron(int mode)
{	
    unsigned char data;
    msleep(5);

	if(!mode) {
		// Software reset
		data = 0x01;
		if (VIBE_S_SUCCESS != I2CWrite(DW7912_SWRESET, 1, &data))
		{
			//DbgOut((DBL_ERROR, "I2CWrite failed to send SW_RESET_COMMAND\n"));
		}
		msleep(1);	

	        // batch dw7912 configuration sequence
	        {
	            u8 command[] = {
	                DW7912_MODE, DW7912_MODE_RTP,
	                DW7912_VD_CLAMP, DW7912_VD_CLAMP_mV(8000),
	                DW7912_BOOST_OPTION, DW7912_BOOST_OPTION_BST_OFFSET_mV(400) | DW7912_BOOST_OPTION_BST_ILIMIT_35,
	                DW7912_PWM_FREQ, CHOSEN_SAMPLE_FREQUENCY,
	            };
	            struct i2c_msg msgs[sizeof(command)/2];
	            i2c_commands(dw7912->i2c, msgs, command, sizeof(command));
	        }
		
	}
	
	else {
		data = 0x01;
		if (VIBE_S_SUCCESS != I2CWrite(DW7912_SWRESET, 1, &data))
		{
			//DbgOut((DBL_ERROR, "I2CWrite failed to send SW_RESET_COMMAND\n"));
		}
		msleep(1);	
		
		data = MEM_MODE; // meomory mode set
		if (VIBE_S_SUCCESS != I2CWrite(DW7912_MODE, 1, &data))
		{
			//DbgOut((DBL_ERROR, "I2CWrite failed to send RTP_COMMAND\n"));
		}
		msleep(1);					
	}
    //dw7912_dump_registers(&dw7912);
}

/* 
 * sysfs device functions 
 * ---
 */
static ssize_t dw7912_show_regs(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *c = dw7912->i2c;
	int i, cnt = 0;

	for (i = 0; i < 0x20; i+=16) {
		cnt += sprintf(buf + cnt, "%02x: %02x", i, i2c_smbus_read_byte_data(c, i));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+1));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+2));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+3));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+4));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+5));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+6));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+7));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+8));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+9));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+10));
		cnt += sprintf(buf + cnt, " %02x", i2c_smbus_read_byte_data(c, i+11));
		cnt += sprintf(buf + cnt, " %02x",   i2c_smbus_read_byte_data(c, i+12));
		cnt += sprintf(buf + cnt, " %02x",   i2c_smbus_read_byte_data(c, i+13));
		cnt += sprintf(buf + cnt, " %02x",	 i2c_smbus_read_byte_data(c, i+14));
		cnt += sprintf(buf + cnt, " %02x\n", i2c_smbus_read_byte_data(c, i+15));
	}
	
	cnt += sprintf(buf + cnt, "23: %02x\n\n", i2c_smbus_read_byte_data(c, 0x23) ); // boost_option

	return cnt;
}


static ssize_t dw7912_store_regs(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t cnt)
{
	struct i2c_client *c = dw7912->i2c;
	unsigned int reg = 0, val = 0;
	char tmp[5];
	int i;

	memset(tmp,0x00,5);
	for(i = 0; i < cnt; i++) {
		if ( buf[i] != ' ' )
			tmp[i] = buf[i];
		else {
			buf += i+1;
			reg = simple_strtoul(tmp, NULL, 16);
			val = simple_strtoul(buf, NULL, 16);
			break;
		}
	}
	printk(KERN_INFO "writing: 0x%x: 0x%02X --> 0x%02X\n",
			reg, i2c_smbus_read_byte_data(c, reg), val);
	i2c_smbus_write_byte_data(c, reg, val);

	return cnt;
}

static DEVICE_ATTR(index_reg, 0660,
		dw7912_show_regs, dw7912_store_regs);

#ifdef CONFIG_OF
static int dw7912_i2c_parse_dt(struct i2c_client *i2c, struct dw7912_dev *p)
{
	struct device *dev = &i2c->dev;
	struct device_node *np = dev->of_node;
	int ret = -1;
	u32 value;

	if (!np)
		return -1;


    ret = of_property_read_u32(np, "use_en_pin", &value);
    if (ret < 0) {
        dev_err(&i2c->dev, "use_en_pin read error\n");
        p->use_en_pin = -1;
    }
    p->use_en_pin = value;
		
	/* If you want to use en gpio pin */
	if (p->use_en_pin ) {
		p->enable_pin = of_get_named_gpio(np, "dw7912,en-gpio", 0);
		if (p->enable_pin < 0) {
			printk(KERN_ERR "Looking up %s property in node %s failed %d\n",
					"dw7912,en-gpio", dev->of_node->full_name,
					p->enable_pin);
			p->enable_pin = -1;
        
			goto error;
		}
		
		if( !gpio_is_valid(p->enable_pin) ) {
			printk(KERN_ERR "dw7912 enable_pin pin(%u) is invalid\n", p->enable_pin);
			goto error;
		}
	}

	gprintk("p->use_en_pin = %d\n", p->use_en_pin);
	gprintk("p->enable_pin = %d\n", p->enable_pin);
	return 0;

error:
	gprintk("p->use_en_pin = %d\n", p->use_en_pin);
	gprintk("p->enable_pin = %d\n", p->enable_pin);
	return -1;	
}
#else
static int dw7912_i2c_parse_dt(struct i2c_client *i2c, struct dw7912_dev *p)
{
	return NULL;
}
#endif

#endif

static int dw7912_probe(struct i2c_client* client, const struct i2c_device_id* id);
static void dw7912_shutdown(struct i2c_client* client);
static int dw7912_remove(struct i2c_client* client);

#if defined(CONFIG_MACH_LGE)

#ifdef CONFIG_PM
static int dw7912_suspend(struct device *dev)
{
	return 0;
}

static int dw7912_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(dw7912_pm_ops,
			 dw7912_suspend, dw7912_resume);

#define DW7912_VIBRATOR_PM_OPS (&dw7912_pm_ops)
#else
#define DW7912_VIBRATOR_PM_OPS NULL
#endif

#ifdef CONFIG_OF
static struct of_device_id dw7912_i2c_dt_ids[] = {
	{ .compatible = "dwanatech,dw7912"},
	{ }
};
#endif
#endif

static const struct i2c_device_id dw7912_id[] = {
    { "dw7912", 0 },
    { }
};
#if !defined(CONFIG_MACH_LGE)
static struct i2c_board_info info = {
    I2C_BOARD_INFO("dw7912", DEVICE_ADDR),
};
#endif

static struct i2c_driver dw7912_driver = {
    .probe = dw7912_probe,
    .remove = dw7912_remove,
    .id_table = dw7912_id,
    .shutdown = dw7912_shutdown,
    .driver = {
        .name = "dw7912",
#if defined(CONFIG_MACH_LGE)
	.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(dw7912_i2c_dt_ids),
#endif
#ifdef CONFIG_PM
		.pm	= DW7912_VIBRATOR_PM_OPS,
#endif
#endif
    },
};

static int dw7912_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
    uint8_t status = 0;
    uint8_t result = 0;
#if defined(CONFIG_MACH_LGE)    
    int ret = -1;
#endif

    dw7912 = kzalloc(sizeof(struct dw7912_dev), GFP_KERNEL);
    if (!dw7912)
        return -ENOMEM;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        DbgOut((DBL_ERROR, "dw7912_probe: i2c_check_functionality failed.\n"));
        return -ENODEV;
    }
#if defined(CONFIG_MACH_LGE)
	if (client->dev.of_node) {
		gprintk("Read enable pin from device tree\n");
		ret = dw7912_i2c_parse_dt(client, dw7912);
		if( ret < 0 ) {
			dev_err(&client->dev, "%s: dt parse error, use_en_pin = %d,"\
			                      "enable_pin = %d\n", __func__, 
			                      dw7912->use_en_pin, dw7912->enable_pin);
		}
	} else {
		dw7912->use_en_pin = -1;
	}
	
	if (dw7912->use_en_pin == -1) {
		dev_err(&client->dev, "%s: use_en_pin error\n", __func__);
		ret = -EINVAL;
		goto error1;
	} 
	
	
	if (dw7912->use_en_pin > 0) { 
		ret = gpio_request(dw7912->enable_pin, "dw7912_en");
		if (ret < 0) {
			dev_err(&client->dev, "%s: dw7912_en gpio pin request error\n", __func__);
			ret = -EINVAL;
			goto error1;
		}
		
		ret = gpio_direction_output(dw7912->enable_pin, 1);
		if (ret < 0) {
			dev_err(&client->dev, "enable pin level set failed");
			ret = -EIO;
			goto error2;
		}
		gpio_set_value(dw7912->enable_pin,1);
	}
	i2c_set_clientdata(client, dw7912);
#endif
    dw7912->i2c = client;
#if	defined(CONFIG_MACH_LGE)
    // code here!!
    dw7912_poweron(RTP_MODE);	// rtp mode setting
    
    ret = device_create_file(&client->dev, &dev_attr_index_reg);
    if (ret < 0) {
    	dev_err(&client->dev, "create sysfs index_reg file --> failed");
    	ret = -EIO;
    	goto error2;
    }
#endif

    if (dw7912->i2c) {

        /* read any pending error codes */
        result = i2c_recv_buf(dw7912->i2c, DW7912_STATUS, &status, sizeof(status));
        if (result != 2) {
            DbgOut((DBL_ERROR, "dw7912_probe: i2c read status failure. result=%d\n", result));
        }

        /* get device id */
        if (2 != i2c_recv_buf(client, DW7912_CHIPID, &dw7912->chipid, 1)) {
            DbgOut((DBL_ERROR, "dw7912_probe: failed to read chipid.\n"));
            /*return -ENODEV;*/
        }

        /* diagnostic */
#if !defined(CONFIG_MACH_LGE)
        dw7912_dump_registers(dw7912);
#endif
    }

    dw7912->buffer[0] = DW7912_RTP_INPUT;

    DbgOut((DBL_VERBOSE, "dw7912_probe.\n"));

    return 0;

#if defined(CONFIG_MACH_LGE)
error1:
	kfree(dw7912);
	return ret;
	
error2:
	if (dw7912->use_en_pin > 0)
		gpio_free(dw7912->enable_pin);
	kfree(dw7912);
	return ret;
#endif
}

static void dw7912_shutdown(struct i2c_client* client)
{
    DbgOut((DBL_VERBOSE, "dw7912_shutdown.\n"));

    return;
}

static int dw7912_remove(struct i2c_client* client)
{
#if defined(CONFIG_MACH_LGE)
    struct dw7912_dev *p = i2c_get_clientdata(client);
#endif
    
    DbgOut((DBL_VERBOSE, "dw7912_remove.\n"));
	
#if defined(CONFIG_MACH_LGE)
    if (dw7912->use_en_pin > 0) {
		gpio_set_value(p->enable_pin,0);
		gpio_free(p->enable_pin);
	}
    device_remove_file(&client->dev, &dev_attr_index_reg);

    i2c_set_clientdata(client, NULL);
    kfree(p);
#endif

    return 0;
}
/*
** TouchSense SPI Functions
*/

IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
    DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_AmpDisable.\n"));

    if (dw7912->buffering) {
        dw7912->buffering = -1;
        ImmVibeSPI_ForceOut_SetSamples(nActuatorIndex, 8, VIBE_OUTPUT_SAMPLE_SIZE, 0);
    }
    return VIBE_S_SUCCESS;
}

static void i2c_commands(struct i2c_client *i2c, struct i2c_msg *msgs, u8 *command, int size)
{
    int i, c, count = size/2;

    for (i = 0, c = 0; c < size; c+=2, i++) {
        msgs[i].flags = 0;
        msgs[i].addr = i2c->addr;
        msgs[i].buf = command + c;
        msgs[i].len = 2;
    }
    if (count != i2c_transfer(i2c->adapter, msgs, count)) {
        DbgOut((DBL_ERROR, "failed to send dw7912 command sequence\n"));
    }
}

IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
    int result;
    uint8_t status;
#if defined(CONFIG_MACH_LGE)
    unsigned char data;
#endif

    DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_AmpEnable.\n"));

    dw7912->buffering = 1;
    dw7912->position = 1;

    if (dw7912->i2c) {

        /* read any pending error codes */
        result = i2c_recv_buf(dw7912->i2c, DW7912_STATUS, &status, sizeof(status));
        if (result != 2) {
            DbgOut((DBL_ERROR, "dw7912: i2c read status failure. result=%d\n", result));
            //return VIBE_E_FAIL;
        }
        if (status & 0x0F) {
            DbgOut((DBL_ERROR, "dw7912: status error! (0x%02x)\n", status));
            //return VIBE_E_FAIL;
        }
#if !defined(CONFIG_MACH_LGE)// move to dw7912_poweron function for default setting
        // batch dw7912 configuration sequence
        {
            u8 command[] = {
                DW7912_MODE, DW7912_MODE_RTP,
                DW7912_VD_CLAMP, DW7912_VD_CLAMP_mV(8000),
                DW7912_BOOST_OPTION, DW7912_BOOST_OPTION_BST_OFFSET_mV(400) | DW7912_BOOST_OPTION_BST_ILIMIT_35,
                DW7912_PWM_FREQ, CHOSEN_SAMPLE_FREQUENCY,
                DW7912_PLAYBACK, DW7912_PLAYBACK_GO
            };
            struct i2c_msg msgs[sizeof(command)/2];
            i2c_commands(dw7912->i2c, msgs, command, sizeof(command));
        }
#else
		data = DW7912_PLAYBACK_GO;
		if (VIBE_S_SUCCESS != I2CWrite(DW7912_PLAYBACK, 1, &data))
		{
			DbgOut((DBL_ERROR, "I2CWrite failed to send DW7912_PLAYBACK\n"));
		}	
#endif
    }
    //DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_AmpEnable. End\n"));
    return VIBE_S_SUCCESS;
}

IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
#if defined(CONFIG_MACH_LGE)
    DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_Initialize enter.\n"));
    return VIBE_S_SUCCESS;
#else
    struct i2c_adapter* adapter = 0;
    struct i2c_client* client = 0;
    int retVal = 0;

    adapter = i2c_get_adapter(DEVICE_BUS);
    if (!adapter) {
        DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_Initialize: I2C Adapter not found.\n"));
        return VIBE_E_FAIL;
    }

    client = i2c_new_device(adapter, &info);
    if (!client) {
        DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_Initialize: Cannot create new device.\n"));
        return VIBE_E_FAIL;
    }

    retVal = i2c_add_driver(&dw7912_driver);
    if (retVal) {
        DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_Initialize: Cannot add driver.\n"));
        i2c_unregister_device(client);
        return VIBE_E_FAIL;
    }

    return VIBE_S_SUCCESS;
#endif
}

IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
    DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_Terminate.\n"));

    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set the force
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
#if 0
    if (pForceOutputBuffer) {
        char buffer[VIBE_OUTPUT_SAMPLE_SIZE*3];
        int i, count = nBufferSizeInBytes / (nOutputSignalBitDepth / 8);
        for (i = 0; i < count; i++) {
            sprintf(buffer + (i * 3), " %02hhx", pForceOutputBuffer[i]);
        }
        DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_SetSamples:%s\n", buffer));
    }
#endif

#ifdef IMMVIBESPI_USE_BUFFERFULL
    /* write zeros to keep hardware buffer full and in sync with driver */
    if (nBufferSizeInBytes != VIBE_OUTPUT_SAMPLE_SIZE) {
        static VibeInt8 buffer[VIBE_OUTPUT_SAMPLE_SIZE] = {0,};
        pForceOutputBuffer = buffer;
        nBufferSizeInBytes = sizeof(buffer);
        nOutputSignalBitDepth = 8;
    }
#endif

    /* buffer initial samples from daemon to compensate for system delay */
    if (dw7912->buffering >= 0 && pForceOutputBuffer && nBufferSizeInBytes > 0 /*&& dw7912->position + nBufferSizeInBytes + 1 <= sizeof(dw7912->buffer)*/) {
        DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_SetSamples: buffering position=%d buffer=%d sizeof=%d\n", dw7912->position, nBufferSizeInBytes, sizeof(dw7912->buffer)));
#ifdef UPSCALE_12_TO_24khz
        {   s8 sample1;
            s8 *src = pForceOutputBuffer;
            s8 *dest = dw7912->buffer + dw7912->position;
            s8 *last = pForceOutputBuffer + (nBufferSizeInBytes - 1);
            while (src < last) {
                *dest = sample1 = *src; src++; dest++;
                *dest = sample1 + ((*src - sample1) >> 1); dest++;
            }
            /* copy last element, since there is no element after to average */
            dest[0] = src[0];
            dest[1] = src[0];
        }
        dw7912->position += nBufferSizeInBytes << 1;
#else
        memcpy(dw7912->buffer + dw7912->position, pForceOutputBuffer, nBufferSizeInBytes);
        dw7912->position += nBufferSizeInBytes;
#endif
        /* filling buffer */
        if (dw7912->buffering > 0 && dw7912->position < sizeof(dw7912->buffer))
            return VIBE_S_SUCCESS;
    }

    /* estimate dw7912 sample time */
    if (dw7912->buffering) {
        dw7912->buffering = 0;
        dw7912->time = ktime_get();
    } else {
        dw7912->time = ktime_add_ms(dw7912->time, g_nTimerPeriodMs);
    }

    if (dw7912->i2c) {

        /* batch multiple i2c transactions for faster transmission */
        if (dw7912->position > 1)
        {
            struct i2c_msg msgs[2];
            // send 'go' to wake up dw7912 fifo logic
            u8 command[2] = {DW7912_PLAYBACK, DW7912_PLAYBACK_GO};
            msgs[0].flags = 0;
            msgs[0].addr = dw7912->i2c->addr;
            msgs[0].buf = command;
            msgs[0].len = sizeof(command);
            msgs[1].flags = 0;
            msgs[1].addr = dw7912->i2c->addr;
            msgs[1].buf = dw7912->buffer;
            msgs[1].len = dw7912->position;
            if (2 != i2c_transfer(dw7912->i2c->adapter, msgs, 2)) {
                DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_SetSamples: i2c write failed\n"));
                //return VIBE_E_FAIL;
            }
        }
    }

    /* buffer is empty */
    dw7912->position = 1;

    //DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_SetSamples: done\n"));

    return VIBE_S_SUCCESS;
}

VibeStatus I2CWrite(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    unsigned char buf[VIBE_OUTPUT_SAMPLE_SIZE+1];

    if (!dw7912->i2c) {
        return VIBE_E_FAIL;
    }

    if (nBufferSizeInBytes > VIBE_OUTPUT_SAMPLE_SIZE) {
        return VIBE_E_FAIL;
    }

    buf[0] = address;
    memcpy(buf + 1, pForceOutputBuffer, nBufferSizeInBytes);
    if (1 != i2c_send_buf(dw7912->i2c, buf, nBufferSizeInBytes+1)) {
        return VIBE_E_FAIL;
    }

    return VIBE_S_SUCCESS;
}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
    return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
#if defined(CONFIG_MACH_LGE)
    return VIBE_S_SUCCESS;
#else
    char szRevision[24];
    if ((!szDevName) || (nSize < 1)) {
        return VIBE_E_FAIL;
    }

    /* Append revision number to the device name */
    sprintf(szRevision, DEVICE_NAME "-DW7912-%02x", dw7912->chipid);
    if (strlen(szRevision) + strlen(szDevName) < nSize - 1) {
        strcat(szDevName, szRevision);
    }

    /* Make sure the string is NULL terminated */
    szDevName[nSize - 1] = '\0';

    return VIBE_S_SUCCESS;
#endif
}

#ifdef IMMVIBESPI_DEVICE_GETSTATUS_SUPPORT
/*
** Called by the TouchSense Player Service/Daemon when an application calls
** ImmVibeGetDeviceCapabilityInt32 with VIBE_DEVCAPTYPE_DRIVER_STATUS.
** The TouchSense Player does not interpret the returned status.
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetStatus(VibeUInt8 nActuatorIndex)
{
    return VIBE_S_SUCCESS;
}
#endif

#ifdef IMMVIBESPI_USE_BUFFERFULL
/*
** Check if the amplifier sample buffer is full (not ready for more data).
*/
IMMVIBESPIAPI int ImmVibeSPI_ForceOut_BufferFull(void)
{
    return !dw7912->buffering && ktime_before(ktime_get(), dw7912->time);
}
#endif

#if defined(CONFIG_MACH_LGE)
static int __init dw7912_modinit(void)
{
	int ret;

	gprintk("\n");
	ret = i2c_add_driver(&dw7912_driver);
	//if (ret)
		//pr_err("Failed to register dw7912 I2C driver: %d\n", ret);

	return ret;
}

module_init(dw7912_modinit);

static void __exit dw7912_exit(void)
{
	i2c_del_driver(&dw7912_driver);
}

module_exit(dw7912_exit);
#endif
