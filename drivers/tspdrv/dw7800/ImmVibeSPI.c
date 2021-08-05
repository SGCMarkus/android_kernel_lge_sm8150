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
** Copyright (c) 2012-2017 Immersion Corporation. All Rights Reserved.
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
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>

#define DW7800_SYSFS // use this feature only for user debug, not release
#define DW7800_REG_00 0
#define DW7800_REG_01 1
#define DW7800_REG_02 2
#define DW7800_REG_03 3
#define DW7800_REG_04 4
#define DW7800_REG_05 5
#define DW7800_REG_06 6
#define DW7800_REG_07 7
#define DW7800_REG_08 8
#define DW7800_REG_09 9

/*
** This SPI supports only one actuator.
*/
#define DEVICE_BUS  6
#define DEVICE_ADDR 0x59
#define NUM_ACTUATORS       1

/*
** Called to disable amp (disable output force)
*/
#define DEVICE_NAME "LG JUDY"
/*
** Manage amplifier buffer using ImmVibeSPI_ForceOut_BufferFull
*/
#define IMMVIBESPI_USE_BUFFERFULL

/*
** Value between 1 and 86. Higher values buffer more future samples.
*/
#define DW7800_FIFOFULL_TARGET 80

/*
** Number of 5ms blocks of data to preload at effect start.
** Use a value of '0' when using IMMVIBESPI_USE_BUFFERFULL
** since a different mechanism manages the buffer.
*/
#define NUM_EXTRA_BUFFERS  0

/*
** Include optional ImmVibeSPI_Device_GetStatus in driver.
*/
#define IMMVIBESPI_DEVICE_GETSTATUS_SUPPORT

#define I2C_BUF_MAX 32

VibeStatus I2CWrite(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer);
VibeStatus I2CWriteData(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer);
VibeStatus I2CWriteWithResendOnError(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer);

unsigned char fifo = 0;
bool skip_fifo_check = false;
static bool is_immersion_haptic_on = false;
static bool is_i2c_probe_success = true;

static char const * const i2c_rsrcs[] = {"i2c_clk", "i2c_sda"};
struct qup_i2c_clk_path_vote {
	u32                         client_hdl;
	struct msm_bus_scale_pdata *pdata;
	bool                        reg_err;
};
struct qup_i2c_dev {
	struct device                *dev;
	void __iomem                 *base;		/* virtual */
	void __iomem                 *gsbi;		/* virtual */
	int                          in_irq;
	int                          out_irq;
	int                          err_irq;
	int                          num_irqs;
	struct clk                   *clk;
	struct clk                   *pclk;
	struct i2c_adapter           adapter;

	struct i2c_msg               *msg;
	int                          pos;
	int                          cnt;
	int                          err;
	int                          mode;
	int                          clk_ctl;
	int                          one_bit_t;
	int                          out_fifo_sz;
	int                          in_fifo_sz;
	int                          out_blk_sz;
	int                          in_blk_sz;
	int                          wr_sz;
	struct msm_i2c_platform_data *pdata;
	int                          suspended;
	int                          pwr_state;
	struct mutex                 mlock;
	void                         *complete;
	int                          i2c_gpios[ARRAY_SIZE(i2c_rsrcs)];
	struct qup_i2c_clk_path_vote clk_path_vote;
    // Begin Immersion Changes
    int                          sent_cnt;
    // End Immersion Changes
};
#define I2C_SENT_BYTES(client) \
    ((struct qup_i2c_dev *)i2c_get_adapdata((client)->adapter))->sent_cnt

struct dw7800_dev {
    unsigned int state      : 1;
    unsigned int htime      : 2;
    unsigned int freq       : 2;
    unsigned int id         : 4;
    unsigned int model      : 4;
    unsigned int design     : 4;
    unsigned int ldo        : 4;
    unsigned int man_num    : 4;
    unsigned int model_num  : 4;
    unsigned int packet_sz  : 6;
    unsigned char version;
    unsigned char i2c_addr;
    unsigned char buf[40 + 1];
    struct i2c_client *i2c;
};

static struct dw7800_dev dw7800 = {
    .htime = 2,         /* 10ms timeout */
    .freq = 0,          /* 8kHz */
#ifdef CONFIG_MACH_SM8150_BETA
    .ldo = 7,			/* 2.6V LDO */
#else
    .ldo = 8,           /* 2.8V LDO */
#endif
    .i2c_addr = DEVICE_ADDR,
};

/*
** I2C Driver
*/
static int dw7800_probe(struct i2c_client* client, const struct i2c_device_id* id);
static void dw7800_shutdown(struct i2c_client* client);
static int dw7800_remove(struct i2c_client* client);

static const struct i2c_device_id dw7800_id[] = {
    { "dw7800", 0 },
    { }
};

#if 0
static struct i2c_board_info info = {
    I2C_BOARD_INFO("dw7800", DEVICE_ADDR),
};
#endif

/* for dw7800 by yoseph.kim Start */
static struct of_device_id dw7800_match_table[] = {
    { .compatible = "dongwoon,dw7800",},
    { },
};

static struct i2c_driver dw7800_driver = {
    .probe = dw7800_probe,
    .remove = dw7800_remove,
    .id_table = dw7800_id,
    .shutdown = dw7800_shutdown,
    .driver = {
        .name = "dw7800",
        .of_match_table = dw7800_match_table,
    },
};

#ifdef DW7800_SYSFS
struct dw7800_regmap {
    const char *name;
    uint8_t reg;
    int writeable;
} dw7800_regs[] = {
    { "00_IC_INFO",          DW7800_REG_00, 0 },
    { "01_IC_VERSION",       DW7800_REG_01, 0 },
    { "02_STATUS",           DW7800_REG_02, 0 },
    { "03_FIFO_FULLNESS",    DW7800_REG_03, 0 },
    { "04_HAPTIC_DATA",      DW7800_REG_04, 1 },
    { "05_SOFTWARE_RESET",   DW7800_REG_05, 1 },
    { "06_TIMING_SET",       DW7800_REG_06, 1 },
    { "07_LDO_LEVEL",        DW7800_REG_07, 1 },
    { "08_HW_RESET_DISABLE", DW7800_REG_08, 1 },
    { "09_PACKET_SIZE",      DW7800_REG_09, 0 },
};

unsigned char moisture_data_buf[166] = {
    0x04,
    0x00, 0x0E, 0x1D, 0x2B, 0x39, 0x46, 0x52, 0x5D, 0x66, 0x6E, 0x75, 0x7A, 0x7D, 0x7E, 0x7E, 0x7C, 0x78, 0x73, 0x6C, 0x63,
    0x59, 0x4E, 0x42, 0x35, 0x27, 0x18, 0x09, 0xFC, 0xED, 0xDE, 0xD0,

    0x04,
    0xC2, 0xB6, 0xAA, 0xA0, 0x97, 0x8F, 0x89, 0x85, 0x82, 0x81, 0x82, 0x85, 0x89, 0x8F, 0x97, 0xA0, 0xAA, 0xB6, 0xC2, 0xD0,
    0xDE, 0xED, 0xFC, 0x09, 0x18, 0x27, 0x35, 0x42, 0x4E, 0x59, 0x63,

    0x04,
    0x6C, 0x73, 0x78, 0x7C, 0x7E, 0x7E, 0x7D, 0x7A, 0x75, 0x6E, 0x66, 0x5D, 0x52, 0x46, 0x39, 0x2B, 0x1D, 0x0E, 0x00, 0xF2,
    0xE3, 0xD5, 0xC7, 0xBA, 0xAE, 0xA3, 0x9A, 0x92, 0x8B, 0x86, 0x83,

    0x04,
    0x82, 0x82, 0x84, 0x88, 0x8D, 0x94, 0x9D, 0xA7, 0xB2, 0xBE, 0xCB, 0xD9, 0xE8, 0xF7, 0x04, 0x13, 0x22, 0x30, 0x3E, 0x4A,
    0x56, 0x60, 0x69, 0x71, 0x77, 0x7B, 0x7E, 0x7F, 0x7E, 0x7B, 0x77,

    0x04,
    0x71, 0x69, 0x60, 0x56, 0x4A, 0x3E, 0x30, 0x22, 0x13, 0x04, 0xF7, 0xE8, 0xD9, 0xCB, 0xBE, 0xB2, 0xA7, 0x9D, 0x94, 0x8D,
    0x88, 0x84, 0x82, 0x82, 0x83, 0x86, 0x8B, 0x92, 0x9A, 0xA3, 0xAE,

    0x04,
    0xBA, 0xC7, 0xD5, 0xE3, 0xF2
};

static ssize_t set_moisture_vib(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
    int i = 0;
    int j = 0;
    int res = 0;
    unsigned char reg = 0x3;
    unsigned char fifo = 0;
    struct i2c_msg send_msg;
    struct i2c_msg recv_msg[2];

    if(!dw7800.i2c)
        return VIBE_E_FAIL;

    send_msg.addr = dw7800.i2c->addr;
    send_msg.flags = 0;
    send_msg.len = 32;

    recv_msg[0].addr = dw7800.i2c->addr;
    recv_msg[0].flags = 0;
    recv_msg[0].len = 1;
    recv_msg[0].buf = &reg;

    recv_msg[1].addr = dw7800.i2c->addr;
    recv_msg[1].flags = I2C_M_RD;
    recv_msg[1].len = 1;
    recv_msg[1].buf = &fifo;

    for(j = 0; j < 5; j++) {
        for(i = 0; i < 6; i++) {
            send_msg.buf = moisture_data_buf + (32*i);
            res = i2c_transfer(dw7800.i2c->adapter, &send_msg, 1);

            while(1) {
                res = i2c_transfer(dw7800.i2c->adapter, recv_msg, 2);
                if(fifo < 80)
                    break;

                usleep_range(1000, 1100);
            }
        }
    }

    return res;
}

static DEVICE_ATTR(moisture_vib, S_IWUSR | S_IRUGO, NULL, set_moisture_vib);

static ssize_t dw7800_registers_show(struct device *dev,
                  struct device_attribute *attr, char *buf)
{
    unsigned i, n, reg_count;
    u8 read_buf;

    if (!dw7800.i2c)
        return VIBE_E_FAIL;

    reg_count = sizeof(dw7800_regs) / sizeof(dw7800_regs[0]);
    for (i = 0, n = 0; i < reg_count; i++) {
        read_buf = i2c_smbus_read_byte_data(dw7800.i2c, dw7800_regs[i].reg);
        n += scnprintf(buf + n, PAGE_SIZE - n,
                   "%-30s <#%02d>= 0x%02X\n",
                   dw7800_regs[i].name, dw7800_regs[i].reg,
                   read_buf);
    }

    return n;
}

static ssize_t dw7800_registers_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf, size_t count)
{
    unsigned i, reg_count, value;
    int error = 0;
    char name[45];

    if (!dw7800.i2c)
        return VIBE_E_FAIL;

    if (count >= 45) {
        pr_err("%s:input too long\n", __func__);
        return -1;
    }

    if (sscanf(buf, "%30s %x", name, &value) != 2) {
        pr_err("%s:unable to parse input\n", __func__);
        return -1;
    }

    reg_count = sizeof(dw7800_regs) / sizeof(dw7800_regs[0]);
    for (i = 0; i < reg_count; i++) {
        if (!strcmp(name, dw7800_regs[i].name)) {
            if (dw7800_regs[i].writeable) {
                error = i2c_smbus_write_byte_data(dw7800.i2c, dw7800_regs[i].reg, value);
                if (error) {
                    pr_err("%s:Failed to write %s\n", __func__, name);
                    return -1;
                }
            }
            else {
                pr_err("%s:Register %s is not writeable\n", __func__, name);
                return -1;
            }

            return count;
        }
    }

    pr_err("%s:no such register %s\n", __func__, name);
    return -1;

}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
        dw7800_registers_show, dw7800_registers_store);

static struct attribute *dw7800_attrs[] = {
    &dev_attr_registers.attr,
    &dev_attr_moisture_vib.attr,
    NULL
};

static const struct attribute_group dw7800_attr_group = {
    .attrs = dw7800_attrs,
};

#endif // End of #ifdef DW7800_SYSFS

/* for dw7800 by yoseph.kim start */
struct timed_vibrator_data {
    atomic_t gp1_clk_flag;
    int haptic_en_gpio;
    int motor_pwm_gpio;
    int vpwr_on;
    struct regulator *vreg_l21;
    int vibe_n_value;
    unsigned int clk_rate;
};

struct timed_vibrator_data vib;
static DEFINE_MUTEX(vib_lock);
/* for dw7800 by yoseph.kim End */
/*
** I2C Transfer Functions
*/

static int i2c_recv_buf(struct i2c_client *i2c, unsigned char reg, unsigned char *buf, int count)
{
    struct i2c_msg msg[2];
    int res = 0;

    if(!i2c)
    {
        return VIBE_E_FAIL;
    }


    msg[0].addr = i2c->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &reg;

    msg[1].addr = i2c->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = count;
    msg[1].buf = buf;

    res = i2c_transfer(i2c->adapter, msg, 2);
    DbgOut((DBL_INFO, "dw7800: i2c recv res=%d\n", res));

    return res;
}

static bool i2c_send_buf(struct i2c_client *i2c, unsigned char *buf, int count)
{
    struct i2c_msg msg;
    int res = 0;

    if(!i2c)
    {
        return VIBE_E_FAIL;
    }

    msg.addr = i2c->addr;
    msg.flags = 0;
    msg.len = count;
    msg.buf = buf;

    res = i2c_transfer(i2c->adapter, &msg, 1);
    DbgOut((DBL_INFO, "dw7800: i2c send res=%d\n", res));

    return res;
}

#define DW7800_INFO             0x00
#define DW7800_VERSION          0x01
#define DW7800_STATUS           0x02
#define DW7800_FIFOFULL         0x03
#define DW7800_DATA             0x04
#define DW7800_SWRESET          0x05
#define DW7800_TIMING           0x06
#define DW7800_LDO              0x07
#define DW7800_HWRESET          0x08
#define DW7800_PKTSIZE          0x09
#define SW_RESET_COMMAND        0x01 /* Reset */
#define HW_RESET_ENABLE_COMMAND 0x00 /* Enable hardware */

static void dw7800_dump_registers(struct dw7800_dev *dev)
{
    char buf[5] = {0,};

    i2c_recv_buf(dev->i2c, DW7800_INFO, buf, 1);
    dev->id = buf[0] >> 4;
    dev->model = buf[0] & 0x0F;
    i2c_recv_buf(dev->i2c, DW7800_INFO+1, buf, 1);
    dev->design = buf[1] & 0x0F;
    i2c_recv_buf(dev->i2c, DW7800_INFO+2, buf, 1);
    i2c_recv_buf(dev->i2c, DW7800_INFO+3, buf, 1);
    DbgOut((DBL_INFO, "dw7800: 0x%02x 0x%02x 0x%02x 0x%02x\n",
        buf[0], buf[1], buf[2], buf[3]));
    i2c_recv_buf(dev->i2c, DW7800_SWRESET, buf, 1);
    i2c_recv_buf(dev->i2c, DW7800_SWRESET+1, buf, 1);
    i2c_recv_buf(dev->i2c, DW7800_SWRESET+2, buf, 1);
    i2c_recv_buf(dev->i2c, DW7800_SWRESET+3, buf, 1);
    i2c_recv_buf(dev->i2c, DW7800_SWRESET+4, buf, 1);
    DbgOut((DBL_INFO, "dw7800: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
        buf[0], buf[1], buf[2], buf[3], buf[4]));
}

static void dw7800_poweron(struct dw7800_dev *dev)
{
    unsigned char data;
    msleep(5);

    // Software reset
    data = SW_RESET_COMMAND;
    if (VIBE_S_SUCCESS != I2CWrite(DW7800_SWRESET, 1, &data))
    {
        DbgOut((DBL_ERROR, "I2CWrite failed to send SW_RESET_COMMAND\n"));
    }
    msleep(1);

    // Send timing
    data = dev->htime << 4 | dev->freq;
    if (VIBE_S_SUCCESS != I2CWrite(DW7800_TIMING, 1, &data))
    {
        DbgOut((DBL_ERROR, "I2CWrite failed to send timing data\n"));
    }
    msleep(1);

    // Send LDO level
    data = dev->ldo;
    if (VIBE_S_SUCCESS != I2CWrite(DW7800_LDO, 1, &data))
    {
        DbgOut((DBL_ERROR, "I2CWrite failed to send ldo level data\n"));
    }
    msleep(1);

    // Enable
    data = HW_RESET_ENABLE_COMMAND;
    if (VIBE_S_SUCCESS != I2CWrite(DW7800_HWRESET, 1, &data))
    {
        DbgOut((DBL_ERROR, "I2CWrite failed to send HW_RESET_ENABLE_COMMAND\n"));
    }
    msleep(1);

    dw7800_dump_registers(dev);
}

/* for dw7800 by yoseph.kim Start */
static void dw7800_parse_dt(struct device *dev, struct timed_vibrator_data *vib_data)
{
    struct device_node *np = dev->of_node;
    of_property_read_u32(np, "dongwoon,vpwr-on", &vib_data->vpwr_on);
    vib_data->haptic_en_gpio = of_get_named_gpio_flags(np, "dongwoon,haptic-pwr-gpio", 0, NULL);
    vib_data->motor_pwm_gpio = of_get_named_gpio_flags(np, "dongwoon,motor-pwm-gpio", 0, NULL);
    of_property_read_u32(np, "dongwoon,n-value", &vib_data->vibe_n_value);
    of_property_read_u32(np, "dongwoon,clk-rate", &vib_data->clk_rate);
}
/* for dw7800 by yoseph.kim End */
static int dw7800_probe(struct i2c_client* client, const struct i2c_device_id* id) {
    //int rc = 0;
    int ret = 0;

    if(client->dev.of_node) {
        pr_info("dw7800_probe: dw7800_parse_dt.\n");
        dw7800_parse_dt(&client->dev, &vib);
    } else {
        pr_err("dw7800_probe: client is NULL.\n");
        return -EPROBE_DEFER;
    }

    if(vib.vpwr_on != 1) {
        if(!(vib.vreg_l21)) {
            //vib.vreg_l21 = regulator_get(&client->dev, "vib_vdd");
            pr_err("dw7800_probe: After regulator_get.\n");
            if(IS_ERR(vib.vreg_l21)) {
                vib.vreg_l21 = NULL;
                pr_err("dw7800_probe: vib.vreg_l21 is NULL.\n");
                return -EPROBE_DEFER;
            }
        } else {
            pr_err("dw7800_probe: regulator_get fail1.\n");
            return -EPROBE_DEFER;
        }
    } else {
        pr_err("dw7800_probe: regulator_get fail2.\n");
    }

    mutex_lock(&vib_lock);

    if (vib.vpwr_on != 1) {
        pr_info("dw7800_probe: regulator_enable.\n");
        //rc = regulator_enable(vib.vreg_l21);
    } else {
        pr_err("dw7800_probe: regulator_enable faile.\n");
        return -EPROBE_DEFER;
    }

    mutex_unlock(&vib_lock);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("dw7800_probe: i2c_check_functionality failed.\n");
        return -ENODEV;
    }

    dw7800.i2c = client;
    dw7800_poweron(&dw7800);

    /* get device id */
    if (2 != i2c_recv_buf(client, DW7800_VERSION, &dw7800.version, 1)) {
        pr_err("dw7800_probe: failed to read version from dw7800.\n");
		is_i2c_probe_success = false;
        /*return -ENODEV;*/
    }

    pr_info("dw7800_probe.\n");

#ifdef DW7800_SYSFS
    ret = sysfs_create_group(&client->dev.kobj, &dw7800_attr_group);
#endif

    return 0;
}

static void dw7800_shutdown(struct i2c_client* client)
{
    /* Remove TS5000 driver */
//    i2c_del_driver(&dw7800_driver);

    /* Reverse i2c_new_device */
//    i2c_unregister_device(dw7800.i2c);

    DbgOut((DBL_ERROR, "dw7800_shutdown.\n"));

    return;
}

static int dw7800_remove(struct i2c_client* client)
{
    DbgOut((DBL_ERROR, "dw7800_remove.\n"));

    return 0;
}

/* =====================================================================================
function : Remapped Function for A2V
descript :
version  : 1.0
release  : 2018.08.24
====================================================================================== */

int a2v_seq_write(u8* data, u32 size)
{
	/* call i2c bulk write func */
#if 0
	if( haptic_status == false ) {
		dw791x_seq_write(dw791x->rtp_input, 0x0, RAM_ADDR0, (u8*)data, size);
	}
#endif
    int ret;

    if(is_immersion_haptic_on)
        return -1;

	ret = I2CWrite(DW7800_DATA, size, data);

	return ret;
}
EXPORT_SYMBOL(a2v_seq_write);

unsigned char a2v_byte_read(u8 addr)
{
    unsigned char reg_val;

    if(is_immersion_haptic_on)
        return -1;

    i2c_recv_buf(dw7800.i2c, addr, &reg_val, 1);

	return reg_val;
}
EXPORT_SYMBOL(a2v_byte_read);

/*
** TouchSense SPI Functions
*/

#define NAK_RESEND_ATTEMPT 3
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
    DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_AmpDisable.\n"));

    /* Nothing to do. DW7800 enters standby when FIFO is empty. */
    skip_fifo_check = false;
    is_immersion_haptic_on = false;

    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
    DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_AmpEnable.\n"));

    /* Set duty cycle to 50% */
    /* To be implemented with appropriate hardware access macros */

    /* Enable amp */
    /* To be implemented with appropriate hardware access macros */

    is_immersion_haptic_on = true;

    return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
    int retVal = 0;

    retVal = i2c_add_driver(&dw7800_driver);
    if (retVal) {
		        DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_Initialize: Cannot add driver.\n"));
		        return VIBE_E_FAIL;
    }

    return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
    /* Remove driver */

    /* Set PWM frequency */
    /* To be implemented with appropriate hardware access macros */

    /* Set duty cycle to 50% */
    /* To be implemented with appropriate hardware access macros */

    DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_Terminate.\n"));
    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set force output, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
#ifdef IMMVIBESPI_USE_BUFFERFULL
    /* write zeros to keep hardware buffer full */
    if (nBufferSizeInBytes == 0) {
        static VibeInt8 buffer[40] = {0,};
        pForceOutputBuffer = buffer;
        nBufferSizeInBytes = sizeof(buffer);
        nOutputSignalBitDepth = 8;
    }
#endif

    if (VIBE_S_SUCCESS != I2CWriteData(DW7800_DATA, nBufferSizeInBytes, pForceOutputBuffer)) {
        DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_SetSamples: i2c write failed\n"));
        return VIBE_E_FAIL;
    }

    return VIBE_S_SUCCESS;
}

VibeStatus I2CWrite(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    unsigned char buf[VIBE_OUTPUT_SAMPLE_SIZE+1];

    if (!dw7800.i2c)
        return VIBE_E_FAIL;

    if (nBufferSizeInBytes > VIBE_OUTPUT_SAMPLE_SIZE)
        return VIBE_E_FAIL;

    buf[0] = address;
    memcpy(buf + 1, pForceOutputBuffer, nBufferSizeInBytes);
    if (1 != i2c_send_buf(dw7800.i2c, buf, nBufferSizeInBytes+1)) {
        return VIBE_E_FAIL;
    }

    return VIBE_S_SUCCESS;
}

VibeStatus I2CWriteData(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    unsigned char buf1[I2C_BUF_MAX];
    int buf_remain_size = nBufferSizeInBytes % I2C_BUF_MAX + 2;
    unsigned char buf2[buf_remain_size];

    if (!dw7800.i2c)
        return VIBE_E_FAIL;

    if (nBufferSizeInBytes > VIBE_OUTPUT_SAMPLE_SIZE)
        return VIBE_E_FAIL;

    buf1[0] = address;
    memcpy(buf1 + 1, pForceOutputBuffer, I2C_BUF_MAX - 1);
    if (1 != i2c_send_buf(dw7800.i2c, buf1, I2C_BUF_MAX)) {
        return VIBE_E_FAIL;
    }

    buf2[0] = address;
    memcpy(buf2 + 1, pForceOutputBuffer + I2C_BUF_MAX - 1, buf_remain_size - 1);
    if (1 != i2c_send_buf(dw7800.i2c, buf2, buf_remain_size)) {
        return VIBE_E_FAIL;
    }

    return VIBE_S_SUCCESS;
}

VibeStatus I2CWriteWithResendOnError(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    int nDataIndex = 0;
    int waitDuration = 0;
    int nResendAttempt = NAK_RESEND_ATTEMPT;
    char datacnt = 0;
    char pktsize = 0;
    unsigned char buf[VIBE_OUTPUT_SAMPLE_SIZE+1];

    if (!dw7800.i2c)
        return VIBE_E_FAIL;

    if (nBufferSizeInBytes > VIBE_OUTPUT_SAMPLE_SIZE)
        return VIBE_E_FAIL;

    while (nResendAttempt--) {

        if (!pktsize) {

            /* First byte is i2c destination register address */
            buf[0] = address;

            /* Copy remaining data */
            memcpy(buf + 1, pForceOutputBuffer + nDataIndex, nBufferSizeInBytes - nDataIndex);

            /* Send remaining bytes */
            /* The return value of this function is unreliable so ignoring it */
            i2c_send_buf(dw7800.i2c, buf, nBufferSizeInBytes - nDataIndex + 1);
        }

        /* Check how many bytes actually received by dw7800 */
        datacnt = 0;
        pktsize = (2 != i2c_recv_buf(dw7800.i2c, DW7800_PKTSIZE, &datacnt, 1));
        if (pktsize) {
            DbgOut((DBL_WARNING, "dw7800_pktsize could not be read\n"));
            udelay(250);
            continue;
        }

        /* Advance data pointer */
        nDataIndex += datacnt;

        if (nDataIndex < nBufferSizeInBytes)
        {
            /* wait small amount to avoid underrun */
            waitDuration = nDataIndex > 0 ? nDataIndex * 50 : 50;
            udelay(waitDuration);
        }
        else
        {
            return VIBE_S_SUCCESS;
        }
    }

    DbgOut((DBL_ERROR, "I2CWriteWithResendOnError failed\n"));

    return VIBE_E_FAIL;
}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(
        VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID,
        VibeUInt32 nFrequencyParameterValue) {
    return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
    char szRevision[24];
    if ((!szDevName) || (nSize < 1)) return VIBE_E_FAIL;

    DbgOut((DBL_VERBOSE, "ImmVibeSPI_Device_GetName.\n"));

    /* Append revision number to the device name */
    sprintf(szRevision, "%s-DW7800-%02x.%02x", DEVICE_NAME, (dw7800.id << 4) | dw7800.model, dw7800.design);
    if (strlen(szRevision) + strlen(szDevName) < nSize - 1)
        strcat(szDevName, szRevision);

    /* Make sure the string is NULL terminated */
    szDevName[nSize - 1] = '\0';

    dw7800_dump_registers(&dw7800);
    return VIBE_S_SUCCESS;
}

/*
** Called by the TouchSense Player Service/Daemon when an application calls
** ImmVibeGetDeviceCapabilityInt32 with VIBE_DEVCAPTYPE_DRIVER_STATUS.
** The TouchSense Player does not interpret the returned status.
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetStatus(VibeUInt8 nActuatorIndex)
{
    char ic_info = 0;

    if( 2 != i2c_recv_buf(dw7800.i2c, DW7800_INFO, &ic_info, 1) )
    {
        return VIBE_E_FAIL;
    }

    if( ic_info != 0xF8 )
    {
        return VIBE_E_FAIL;
    }

    return VIBE_S_SUCCESS;
}

#ifdef IMMVIBESPI_USE_BUFFERFULL
/*
** Check if the amplifier sample buffer is full (not ready for more data).
*/
IMMVIBESPIAPI int ImmVibeSPI_ForceOut_BufferFull(void)
{
    static int nResendAttempt = NAK_RESEND_ATTEMPT;

    /* if the hardware is too busy to respond, assume it is full */
    if (2 != i2c_recv_buf(dw7800.i2c, DW7800_FIFOFULL, &fifo, 1)) {

        /* failed too many consecutive times: stop managing the buffer */
        if (nResendAttempt < 0) {
            return -1;
        }

        /* try again later */
        DbgOut((DBL_WARNING, "dw7800_pktsize could not be read\n"));
        nResendAttempt -= 1;
        return 1;
    }

    nResendAttempt = NAK_RESEND_ATTEMPT;

    if (fifo < 60)
        skip_fifo_check = true;
    else
        skip_fifo_check = false;

    return fifo > DW7800_FIFOFULL_TARGET;
}

IMMVIBESPIAPI bool i2c_probe_success()
{
	return is_i2c_probe_success;
}
#endif
