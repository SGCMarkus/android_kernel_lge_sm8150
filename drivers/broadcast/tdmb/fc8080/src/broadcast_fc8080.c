#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
//#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/pm_wakeup.h>         /* wake_lock, unlock */
#include <linux/version.h>          /* check linux version */

#include "../../broadcast_tdmb_drv_ifdef.h"
#include "../inc/broadcast_fc8080.h"
#include "../inc/fci_types.h"
#include "../inc/bbm.h"

#include <linux/err.h>
#include <linux/of_gpio.h>

#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>

#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/async.h>

/* external function */
extern int broadcast_fc8080_drv_if_isr(void);
extern void tunerbb_drv_fc8080_isr_control(fci_u8 onoff);
extern int tunerbb_drv_fc8080_fic_cb(uint32 userdata, uint8 *data, int length);
extern int tunerbb_drv_fc8080_msc_cb(uint32 userdata, uint8 subChId, uint8 *data, int length);

/* proto type declare */
static irqreturn_t broadcast_tdmb_spi_event_handler(int irq, void *handle);
void tunerbb_drv_load_hw_configure(struct spi_device *spi);
static int broadcast_tdmb_fc8080_probe(struct spi_device *spi);
static int broadcast_tdmb_fc8080_remove(struct spi_device *spi);
static int broadcast_tdmb_fc8080_check_chip_id(void);
static int tdmb_pinctrl_init(void);
static int tdmb_set_gpio_config(void);
static int tdmb_free_gpio_config(void);

#define LGE_FC8080_DRV_VER  "1.02.02"

static struct of_device_id tdmb_spi_table[] = {
    {
        .compatible = "lge,tdmb",
    },
    {}
};

static struct spi_driver broadcast_tdmb_driver = {
    .probe = broadcast_tdmb_fc8080_probe,
    .remove    = __broadcast_dev_exit_p(broadcast_tdmb_fc8080_remove),
    .driver = {
        .name = "tdmb",
        .of_match_table = tdmb_spi_table,
        .bus    = &spi_bus_type,
        .owner = THIS_MODULE,
    },
};

/************************************************************************/
/* LINUX Driver Setting                                                 */
/************************************************************************/
static uint32 user_stop_flg = 0;
struct tdmb_fc8080_ctrl_blk
{
    boolean                                   pwr_state;
    struct spi_device*                   spi_ptr;
    struct mutex                           mutex;
    struct wakeup_source              wake_lock;    /* wake_lock,wake_unlock */
    boolean                                   spi_irq_status;
    spinlock_t                                spin_lock;
    struct clk                               *clk;
    struct platform_device            *pdev;

    struct pinctrl                          *tdmb_pinctrl;
    struct pinctrl_state                 *gpio_state_active;
    struct pinctrl_state                 *gpio_state_sleep;

    int32                                      dmb_en;

    int32                                      dmb_ant_sw;
    int32                                      dmb_ant_sw_active_value;

    int32                                      dmb_lna_gc;
    int32                                      dmb_lna_en;

    int32                                      dmb_en_lna_en_same_gpio;
    int32                                      dmb_ldo_en;

    int32                                      dmb_use_xtal;
    int32                                      dmb_xtal_freq;
    int32                                      dmb_interface_freq;

    int32                                      ctrl_dmb_ldo;
    struct regulator                         *dmb_ldo;
    int32                                      ctrl_lna_ldo;
    struct regulator                         *lna_ldo;
    int32                                      ctrl_ant_sw_ldo;
    struct regulator                         *ant_sw_ldo;
};

static struct tdmb_fc8080_ctrl_blk fc8080_ctrl_info;

static Device_drv device_fc8080 = {
    &broadcast_fc8080_drv_if_power_on,
    &broadcast_fc8080_drv_if_power_off,
    &broadcast_fc8080_drv_if_init,
    &broadcast_fc8080_drv_if_stop,
    &broadcast_fc8080_drv_if_set_channel,
    &broadcast_fc8080_drv_if_detect_sync,
    &broadcast_fc8080_drv_if_get_sig_info,
    &broadcast_fc8080_drv_if_get_fic,
    &broadcast_fc8080_drv_if_get_msc,
    &broadcast_fc8080_drv_if_reset_ch,
    &broadcast_fc8080_drv_if_user_stop,
    &broadcast_fc8080_drv_if_select_antenna,
    &broadcast_fc8080_drv_if_set_nation,
    &broadcast_fc8080_drv_if_is_on
};

#if 0
void tdmb_fc8080_spi_write_read_test(void)
{
    uint16 i;
    uint32 wdata = 0;
    uint32 ldata = 0;
    uint32 data = 0;
    uint32 temp = 0;

#define TEST_CNT    5
    tdmb_fc8080_power_on();

    printk("tdmb_fc8080_spi_write_read_test ++\n");
    for(i=0;i<TEST_CNT;i++)
    {
        bbm_com_write(NULL, 0xa4, i & 0xff);
        bbm_com_read(NULL, 0xa4, (fci_u8*)&data);
        printk("FC8080 byte test (0x%x,0x%x)\n", i & 0xff, data);
        if((i & 0xff) != data)
            printk("FC8080 byte test (0x%x,0x%x)\n", i & 0xff, data);
    }

    for(i=0;i<TEST_CNT;i++)
    {
        bbm_com_word_write(NULL, 0xa4, i & 0xffff);
        bbm_com_word_read(NULL, 0xa4, (fci_u16*)&wdata);
        printk("FC8080 word test (0x%x,0x%x)\n", i & 0xffff, wdata);
        if((i & 0xffff) != wdata)
            printk("FC8080 word test (0x%x,0x%x)\n", i & 0xffff, wdata);
    }

    for(i=0;i<TEST_CNT;i++)
    {
        bbm_com_long_write(NULL, 0xa4, i & 0xffffffff);
        bbm_com_long_read(NULL, 0xa4, (fci_u32*)&ldata);
        printk("FC8080 long test (0x%x,0x%x)\n", i & 0xffffffff, ldata);
        if((i & 0xffffffff) != ldata)
            printk("FC8080 long test (0x%x,0x%x)\n", i & 0xffffffff, ldata);
    }

    data = 0;

    for(i=0;i<TEST_CNT;i++)
    {
        temp = i&0xff;
        bbm_com_tuner_write(NULL, 0x58, 0x01, (fci_u8*)&temp, 0x01);
        bbm_com_tuner_read(NULL, 0x58, 0x01, (fci_u8*)&data, 0x01);
        printk("FC8080 tuner test (0x%x,0x%x)\n", i & 0xff, data);
        if((i & 0xff) != data)
            printk("FC8080 tuner test (0x%x,0x%x)\n", i & 0xff, data);
    }
    tdmb_fc8080_power_off();
    printk("tdmb_fc8080_spi_write_read_test --\n");
}
#endif

void tunerbb_drv_load_hw_configure(struct spi_device *spi)
{
    fc8080_ctrl_info.pwr_state = FALSE;
    fc8080_ctrl_info.spi_ptr = spi;
    fc8080_ctrl_info.spi_ptr->mode = SPI_MODE_0;
    fc8080_ctrl_info.spi_ptr->bits_per_word = 8;
    fc8080_ctrl_info.pdev = to_platform_device(&spi->dev);

    /*load gpio configuration */
    fc8080_ctrl_info.dmb_en = of_get_named_gpio(fc8080_ctrl_info.pdev->dev.of_node, "en-gpio", 0);
    if(fc8080_ctrl_info.dmb_en < 0) {
        printk("%s: not setting or failed get dmb_en = %d\n", __func__, fc8080_ctrl_info.dmb_en);
    } else {
        printk("%s: dmb_en = %d\n", __func__, fc8080_ctrl_info.dmb_en);
    }

    fc8080_ctrl_info.dmb_ant_sw = of_get_named_gpio(fc8080_ctrl_info.pdev->dev.of_node, "ant-sw-gpio", 0);
    if(fc8080_ctrl_info.dmb_ant_sw < 0) {
        printk("%s: not setting or failed get dmb_ant_sw = %d\n", __func__, fc8080_ctrl_info.dmb_ant_sw);
    } else {
        printk("%s: dmb_ant_sw = %d\n", __func__, fc8080_ctrl_info.dmb_ant_sw);
        of_property_read_u32(fc8080_ctrl_info.pdev->dev.of_node, "ant-sw-active-value", &fc8080_ctrl_info.dmb_ant_sw_active_value);
        printk("%s: dmb_ant_sw_active_value : %d\n", __func__, fc8080_ctrl_info.dmb_ant_sw_active_value);
    }

    of_property_read_u32(fc8080_ctrl_info.pdev->dev.of_node, "dmb-en-lna-en-same-gpio", &fc8080_ctrl_info.dmb_en_lna_en_same_gpio);
    printk("%s: dmb_en_lna_en_same_gpio : %d\n", __func__, fc8080_ctrl_info.dmb_en_lna_en_same_gpio);

    fc8080_ctrl_info.dmb_ldo_en = of_get_named_gpio(fc8080_ctrl_info.pdev->dev.of_node, "ldo-en-gpio", 0);
    if(fc8080_ctrl_info.dmb_ldo_en < 0) {
        printk("%s: not setting or failed get dmb_ldo_en = %d\n", __func__, fc8080_ctrl_info.dmb_ldo_en);
    } else {
        printk("%s: dmb_ldo_en = %d\n", __func__, fc8080_ctrl_info.dmb_ldo_en);
    }

    fc8080_ctrl_info.dmb_lna_gc = of_get_named_gpio(fc8080_ctrl_info.pdev->dev.of_node, "lna-gc-gpio",0);
    if(fc8080_ctrl_info.dmb_lna_gc < 0) {
        printk("%s: not setting or failed get dmb_lna_gc = %d\n", __func__, fc8080_ctrl_info.dmb_lna_gc);
    } else {
        printk("%s: dmb_lna_gc = %d\n", __func__, fc8080_ctrl_info.dmb_lna_gc);
    }

    fc8080_ctrl_info.dmb_lna_en = of_get_named_gpio(fc8080_ctrl_info.pdev->dev.of_node, "lna-en-gpio",0);
    if(fc8080_ctrl_info.dmb_lna_en < 0) {
        printk("%s: not setting or failed get dmb_lna_en = %d\n", __func__, fc8080_ctrl_info.dmb_lna_en);
    } else {
        printk("%s: dmb_lna_en = %d\n", __func__, fc8080_ctrl_info.dmb_lna_en);
    }

    /*load hw configruation value*/
    of_property_read_u32(fc8080_ctrl_info.pdev->dev.of_node, "use-xtal", &fc8080_ctrl_info.dmb_use_xtal);
    printk("%s: use_xtal : %d\n", __func__, fc8080_ctrl_info.dmb_use_xtal);

    of_property_read_u32(fc8080_ctrl_info.pdev->dev.of_node, "xtal-freq", &fc8080_ctrl_info.dmb_xtal_freq);
    printk("%s: dmb_xtal_freq : %d\n", __func__, fc8080_ctrl_info.dmb_xtal_freq);

    of_property_read_u32(fc8080_ctrl_info.pdev->dev.of_node, "interface-freq", &fc8080_ctrl_info.dmb_interface_freq);
    fc8080_ctrl_info.spi_ptr->max_speed_hz = (fc8080_ctrl_info.dmb_interface_freq*1000);
    printk("%s: dmb_interface_freq : %d\n", __func__, fc8080_ctrl_info.dmb_interface_freq);

    of_property_read_u32(fc8080_ctrl_info.pdev->dev.of_node, "ctrl-dmb-ldo", &fc8080_ctrl_info.ctrl_dmb_ldo);
    printk("%s: ctrl-dmb-ldo : %d\n", __func__, fc8080_ctrl_info.ctrl_dmb_ldo);

    if(fc8080_ctrl_info.ctrl_dmb_ldo == TRUE) {
        fc8080_ctrl_info.dmb_ldo = devm_regulator_get(&fc8080_ctrl_info.spi_ptr->dev, "dmb-ldo");
        if(IS_ERR(fc8080_ctrl_info.dmb_ldo)) {
            dev_err(&fc8080_ctrl_info.spi_ptr->dev, "regulator dmb_ldo failed\n");
        }
        else {
            printk("%s: dmb_ldo regulator OK\n", __func__);
        }
    }

    of_property_read_u32(fc8080_ctrl_info.pdev->dev.of_node, "ctrl-lna-ldo", &fc8080_ctrl_info.ctrl_lna_ldo);
    printk("%s: ctrl-lna-ldo : %d\n", __func__, fc8080_ctrl_info.ctrl_lna_ldo);

    if(fc8080_ctrl_info.ctrl_lna_ldo == TRUE) {
        fc8080_ctrl_info.lna_ldo= devm_regulator_get(&fc8080_ctrl_info.spi_ptr->dev, "lna-ldo");
        if(IS_ERR(fc8080_ctrl_info.lna_ldo)) {
            dev_err(&fc8080_ctrl_info.spi_ptr->dev, "regulator lna_ldo failed\n");
        }
        else {
            printk("%s: lna_ldo regulator OK\n", __func__);
        }
    }

    of_property_read_u32(fc8080_ctrl_info.pdev->dev.of_node, "ctrl-ant-sw-ldo", &fc8080_ctrl_info.ctrl_ant_sw_ldo);
    printk("%s: ctrl-ant-sw-ldo : %d\n", __func__, fc8080_ctrl_info.ctrl_ant_sw_ldo);

    if(fc8080_ctrl_info.ctrl_ant_sw_ldo == TRUE) {
        fc8080_ctrl_info.ant_sw_ldo= devm_regulator_get(&fc8080_ctrl_info.spi_ptr->dev, "ant-sw-ldo");
        if(IS_ERR(fc8080_ctrl_info.ant_sw_ldo)) {
            dev_err(&fc8080_ctrl_info.spi_ptr->dev, "regulator lna_ldo failed\n");
        }
        else {
            printk("%s: ant_sw_ldo regulator OK\n", __func__);
        }
    }
}
struct spi_device *tdmb_fc8080_get_spi_device(void)
{
    return fc8080_ctrl_info.spi_ptr;
}

void tdmb_fc8080_set_userstop(int mode)
{
    user_stop_flg = mode;
    printk("tdmb_fc8080_set_userstop, user_stop_flg = %d \n", user_stop_flg);
}

int tdmb_fc8080_mdelay(int32 ms)
{
    int32    wait_loop =0;
    int32    wait_ms = ms;
    int        rc = 1;  /* 0 : false, 1 : true */

    if(ms > 100)
    {
        wait_loop = (ms /100);   /* 100, 200, 300 more only , Otherwise this must be modified e.g (ms + 40)/50 */
        wait_ms = 100;
    }

    do
    {
        mdelay(wait_ms);

        if(user_stop_flg == 1)
        {
            printk("~~~~~~~~ Ustop flag is set so return false ms =(%d)~~~~~~~\n", ms);
            rc = 0;
            break;
        }
    }while((--wait_loop) > 0);

    return rc;
}

void tdmb_fc8080_Must_mdelay(int32 ms)
{
    mdelay(ms);
}

int tdmb_fc8080_tdmb_is_on(void)
{
    return (int)fc8080_ctrl_info.pwr_state;
}

unsigned int tdmb_fc8080_get_xtal_freq(void)
{
    return (unsigned int)fc8080_ctrl_info.dmb_xtal_freq;
}

#ifdef FEATURE_POWER_ON_RETRY
int tdmb_fc8080_power_on_retry(void)
{
    int res;
    int i;

    tdmb_fc8080_interrupt_lock();

    for(i = 0 ; i < 10; i++)
    {
        printk("[FC8080] tdmb_fc8080_power_on_retry :  %d\n", i);

        if(fc8080_ctrl_info.dmb_use_xtal == FALSE) {
            if(fc8080_ctrl_info.clk != NULL) {
                clk_disable_unprepare(fc8080_ctrl_info.clk);
                printk("[FC8080] retry clk_disable %d\n", i);
            }
            else
                printk("[FC8080] ERR fc8080_ctrl_info.clkdis is NULL\n");
        }

        if(fc8080_ctrl_info.dmb_en > 0) {
            gpio_set_value(fc8080_ctrl_info.dmb_en, 0);
            mdelay(150);

            gpio_set_value(fc8080_ctrl_info.dmb_en, 1);
            mdelay(5);
        }

        if(fc8080_ctrl_info.dmb_use_xtal == FALSE) {
            if(fc8080_ctrl_info.clk != NULL) {
                res = clk_prepare_enable(fc8080_ctrl_info.clk);
                if (res) {
                    printk("[FC8080] retry clk_prepare_enable fail %d\n", i);
                }
            }
            else
                printk("[FC8080] ERR fc8080_ctrl_info.clken is NULL\n");
        }
        mdelay(30);

        res = bbm_com_probe(NULL);

        if (!res)
            break;

    }

    tdmb_fc8080_interrupt_free();

    return res;
}
#endif

int tdmb_fc8080_power_on(void)
{
    int rc = OK;
    printk("%s\n", __func__);

    rc = tdmb_set_gpio_config();
    printk("%s: tdmb_set_gpio_config=%d\n", __func__, rc);
    mdelay(5);

    if (fc8080_ctrl_info.pwr_state == TRUE) {
        printk("%s: the power already turn on \n", __func__);
        return rc;
    }

    rc = tdmb_ldo_power_on();
    printk("%s: tdmb_ldo_power_on : %d\n", __func__, rc);

    __pm_stay_awake(&fc8080_ctrl_info.wake_lock);

    if(fc8080_ctrl_info.dmb_en > 0) {
        gpio_set_value(fc8080_ctrl_info.dmb_en, 0);
        mdelay(5);
        gpio_set_value(fc8080_ctrl_info.dmb_en, 1);
        mdelay(5);
    }

    //[TDMBDEV-2766] (issue) DMB_EN control that FC8080 LDO_EN & 1.8V regulator
    //So add 100ms delay for power sequence before clk enable
    if(fc8080_ctrl_info.dmb_en_lna_en_same_gpio == TRUE) {
        mdelay(100);
    }

    if(fc8080_ctrl_info.dmb_use_xtal == FALSE) {
        if(fc8080_ctrl_info.clk != NULL) {
            rc = clk_prepare_enable(fc8080_ctrl_info.clk);
            if (rc) {
                if(fc8080_ctrl_info.dmb_en > 0) {
                    gpio_set_value(fc8080_ctrl_info.dmb_en, 0);
                }
                dev_err(&fc8080_ctrl_info.spi_ptr->dev, "could not enable clock\n");
                return rc;
            }
        }
    }
    mdelay(30); /* Due to X-tal stablization Time */

    rc = tdmb_lna_power_on();
    printk("%s: tdmb_lna_power_on : %d\n", __func__,rc);
    rc = tdmb_ant_sw_power_on();
    printk("%s: tdmb_ant_sw_power_on : %d\n", __func__,rc);

    tdmb_fc8080_interrupt_free();
    fc8080_ctrl_info.pwr_state = TRUE;

    printk("%s: completed \n", __func__);
    rc = TRUE;

    return rc;
}

int tdmb_fc8080_power_off(void)
{
    int rc = FALSE;
    if ( fc8080_ctrl_info.pwr_state == TRUE ) {
        tdmb_fc8080_interrupt_lock();

        tdmb_lna_power_off();
        tdmb_ant_sw_power_off();

        if(fc8080_ctrl_info.dmb_use_xtal == FALSE) {
            if(fc8080_ctrl_info.clk != NULL) {
                clk_disable_unprepare(fc8080_ctrl_info.clk);
            }
        }

        fc8080_ctrl_info.pwr_state = FALSE;

        if(fc8080_ctrl_info.dmb_en > 0) {
            gpio_set_value(fc8080_ctrl_info.dmb_en, 0);
        }

        __pm_relax(&fc8080_ctrl_info.wake_lock);
        mdelay(20);

        tdmb_ldo_power_off();

    } else {
        printk("%s: tdmb_fc8080_power_on the power already turn off \n", __func__);
    }

    rc = tdmb_free_gpio_config();
    printk("%s: tdmb_free_gpio_config=%d\n", __func__, rc);

    printk("%s: completed\n", __func__);
    rc = TRUE;
    
    return rc;
}

int tdmb_ldo_power_on(void)
{
    int rc = TRUE;

    if(fc8080_ctrl_info.ctrl_dmb_ldo == TRUE) {
        rc = regulator_enable(fc8080_ctrl_info.dmb_ldo);
        if(rc) {
            dev_err(&fc8080_ctrl_info.spi_ptr->dev, "unable to enable dmb_ldo\n");
            return rc;
        } else {
            printk("%s: dmb_ldo enable\n", __func__);
        }
    }

    if(fc8080_ctrl_info.dmb_ldo_en > 0) {
        gpio_set_value(fc8080_ctrl_info.dmb_ldo_en, 1);
        printk("%s: set tdmb_ldo_en_gpio_value to 1\n", __func__);
        mdelay(10);
    }
    return rc;
}

void tdmb_ldo_power_off(void)
{
    if(fc8080_ctrl_info.dmb_ldo_en > 0) {
        gpio_set_value(fc8080_ctrl_info.dmb_ldo_en, 0);
        printk("%s: set tdmb_ldo_en_gpio_value to 0\n", __func__);
        mdelay(10);
    }

    if(fc8080_ctrl_info.ctrl_dmb_ldo == TRUE) {
        regulator_disable(fc8080_ctrl_info.dmb_ldo);
        printk("%s: dmb_ldo disable\n", __func__);
    }
}

int tdmb_lna_power_on(void)
{
    int rc = TRUE;
    if(fc8080_ctrl_info.ctrl_lna_ldo == TRUE) {
        rc = regulator_enable(fc8080_ctrl_info.lna_ldo);
        if(rc) {
            dev_err(&fc8080_ctrl_info.spi_ptr->dev, "unable to enable lna_ldo\n");
            return rc;
        } else {
            printk("%s: lna_ldo enable\n", __func__);
        }
    }

    if(fc8080_ctrl_info.dmb_lna_gc > 0) {
        gpio_set_value(fc8080_ctrl_info.dmb_lna_gc, 0);
    }

    if(fc8080_ctrl_info.dmb_lna_en > 0) {
        gpio_set_value(fc8080_ctrl_info.dmb_lna_en, 1);
    }

    return rc;
}

int tdmb_ant_sw_power_on(void)
{
    int rc = TRUE;
    if (fc8080_ctrl_info.ctrl_ant_sw_ldo == TRUE) {
        rc = regulator_enable(fc8080_ctrl_info.ant_sw_ldo);
        if(rc) {
            dev_err(&fc8080_ctrl_info.spi_ptr->dev, "unable to enable ant_sw_ldo\n");
            return rc;
        } else {
            printk("%s: ant_sw_ldo enable\n", __func__);
        }
    }

    if(fc8080_ctrl_info.dmb_ant_sw > 0) {
        gpio_set_value(fc8080_ctrl_info.dmb_ant_sw, fc8080_ctrl_info.dmb_ant_sw_active_value);
        printk("%s: tdmb_ant_sw_gpio_value : %d\n", __func__, fc8080_ctrl_info.dmb_ant_sw_active_value);
    }

    return rc;
}

void tdmb_lna_power_off(void)
{
    if(fc8080_ctrl_info.dmb_lna_gc > 0) {
        gpio_set_value(fc8080_ctrl_info.dmb_lna_gc, 0);
    }

    if(fc8080_ctrl_info.dmb_lna_en > 0) {
        gpio_set_value(fc8080_ctrl_info.dmb_lna_en, 0);
    }

    if(fc8080_ctrl_info.ctrl_lna_ldo == TRUE) {
        regulator_disable(fc8080_ctrl_info.lna_ldo);
        printk("%s: lna_ldo disable\n", __func__);
    }
}

void tdmb_ant_sw_power_off(void)
{
    if(fc8080_ctrl_info.dmb_ant_sw > 0) {
        gpio_set_value(fc8080_ctrl_info.dmb_ant_sw, !(fc8080_ctrl_info.dmb_ant_sw_active_value));
        printk("%s: tdmb_ant_sw_gpio_value : %d\n", __func__, !(fc8080_ctrl_info.dmb_ant_sw_active_value));
    }

    if(fc8080_ctrl_info.ctrl_ant_sw_ldo == TRUE) {
        regulator_disable(fc8080_ctrl_info.ant_sw_ldo);
        printk("%s: ant_sw_ldo disable\n", __func__);
    }
}

int tdmb_fc8080_select_antenna(unsigned int sel)
{
    return FALSE;
}

void tdmb_fc8080_interrupt_lock(void)
{
    if (fc8080_ctrl_info.spi_ptr == NULL)
    {
        printk("%s fail\n", __func__);
    }
    else
    {
        disable_irq(fc8080_ctrl_info.spi_ptr->irq);
    }
}

void tdmb_fc8080_interrupt_free(void)
{
    if (fc8080_ctrl_info.spi_ptr == NULL)
    {
        printk("tdmb_fc8080_interrupt_free fail\n");
    }
    else
    {
        enable_irq(fc8080_ctrl_info.spi_ptr->irq);
    }
}

int tdmb_fc8080_spi_write_read(uint8* tx_data, int tx_length, uint8 *rx_data, int rx_length)
{
    int rc;

    struct spi_transfer    t = {
            .tx_buf        = tx_data,
            .rx_buf        = rx_data,
            .len        = tx_length+rx_length,
        };

    struct spi_message    m;

    if (fc8080_ctrl_info.spi_ptr == NULL)
    {
        printk("%s: error txdata=%p, length=%d\n", __func__, (void *)tx_data, tx_length+rx_length);
        return FALSE;
    }

    mutex_lock(&fc8080_ctrl_info.mutex);

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    rc = spi_sync(fc8080_ctrl_info.spi_ptr, &m);

    if ( rc < 0 )
    {
        printk("%s: result(%d), actual_len=%d\n", __func__, rc, m.actual_length);
    }

    mutex_unlock(&fc8080_ctrl_info.mutex);

    return TRUE;
}

static irqreturn_t broadcast_tdmb_spi_event_handler(int irq, void *handle)
{
    struct tdmb_fc8080_ctrl_blk* fc8080_info_p;

    fc8080_info_p = (struct tdmb_fc8080_ctrl_blk *)handle;
    if ( fc8080_info_p && fc8080_info_p->pwr_state )
    {
        if (fc8080_info_p->spi_irq_status)
        {
            printk("######### spi read function is so late skip ignore #########\n");
            return IRQ_HANDLED;
        }

        tunerbb_drv_fc8080_isr_control(0);
        fc8080_info_p->spi_irq_status = TRUE;
        broadcast_fc8080_drv_if_isr();
        fc8080_info_p->spi_irq_status = FALSE;
        tunerbb_drv_fc8080_isr_control(1);
    }
    else
    {
        printk("broadcast_tdmb_spi_isr is called, but device is off state\n");
    }

    return IRQ_HANDLED;
}

static int broadcast_tdmb_fc8080_probe(struct spi_device *spi)
{
    int rc;

    if(spi == NULL)
    {
        printk("%s: spi is NULL, so spi can not be set\n", __func__);
        return -1;
    }

    tunerbb_drv_load_hw_configure(spi);

    // Once I have a spi_device structure I can do a transfer anytime
    rc = spi_setup(spi);
    printk("%s: spi_setup=%d\n", __func__, rc);
    bbm_com_hostif_select(NULL, BBM_SPI);

    //[Start] This code for kmalloc fic & msc buffer when device booting.
    // This code's original location : tunerbb_drv_fc8080_init() in tdmb_tunerbbdrv_fc8080.c
#if !defined(STREAM_TS_UPLOAD)
    bbm_com_fic_callback_register((UDynamic_32_64)NULL, tunerbb_drv_fc8080_fic_cb);
    bbm_com_msc_callback_register((UDynamic_32_64)NULL, tunerbb_drv_fc8080_msc_cb);
#endif
    //[End] This code for fic & msc buffer kmalloc when device booting.

    if(fc8080_ctrl_info.dmb_use_xtal == FALSE) {
        fc8080_ctrl_info.clk = clk_get(&fc8080_ctrl_info.spi_ptr->dev, "tdmb_xo");
        if (IS_ERR(fc8080_ctrl_info.clk)) {
            rc = PTR_ERR(fc8080_ctrl_info.clk);
            dev_err(&fc8080_ctrl_info.spi_ptr->dev, "could not get clock\n");
            return rc;
        }

        /* We enable/disable the clock only to assure it works */
        rc = clk_prepare_enable(fc8080_ctrl_info.clk);
        if (rc) {
            dev_err(&fc8080_ctrl_info.spi_ptr->dev, "could not enable clock\n");
            return rc;
        }
        clk_disable_unprepare(fc8080_ctrl_info.clk);
    }

    rc = tdmb_pinctrl_init();
    printk("%s: tdmb_pinctrl_init=%d\n", __func__, rc);

    rc = request_threaded_irq(spi->irq, NULL, broadcast_tdmb_spi_event_handler,
        IRQF_ONESHOT | IRQF_TRIGGER_FALLING, spi->dev.driver->name, &fc8080_ctrl_info);

    printk("%s: request_threaded_irq = %d\n", __func__, rc);

    tdmb_fc8080_interrupt_lock();

    mutex_init(&fc8080_ctrl_info.mutex);

    wakeup_source_init(&fc8080_ctrl_info.wake_lock, dev_name(&spi->dev));

    spin_lock_init(&fc8080_ctrl_info.spin_lock);

    rc = broadcast_tdmb_drv_start(&device_fc8080);

#if defined(CONFIG_ARCH_QCOM)
    printk("LGE_FC8080_DRV_VER (QCOM)  : %s\n", LGE_FC8080_DRV_VER);
#else
    printk("LGE_FC8080_DRV_VER (MTK or other)  : %s\n", LGE_FC8080_DRV_VER);
#endif

    printk("%s start %d \n", __func__, rc);

    if(broadcast_tdmb_fc8080_check_chip_id() != OK) {
        printk("%s: Chip ID read fail!!\n", __func__);
        rc = ERROR;
    }

    printk("%s: rc=%d, End\n", __func__, rc);

    return rc;
}

static int broadcast_tdmb_fc8080_remove(struct spi_device *spi)
{
    printk("%s \n", __func__);

    free_irq(spi->irq, NULL);

    mutex_destroy(&fc8080_ctrl_info.mutex);

    wakeup_source_trash(&fc8080_ctrl_info.wake_lock);

    memset((unsigned char*)&fc8080_ctrl_info, 0x0, sizeof(struct tdmb_fc8080_ctrl_blk));
    return 0;
}

static int broadcast_tdmb_fc8080_check_chip_id(void)
{
    int rc = ERROR;

    rc = tdmb_fc8080_power_on();

    if(rc == TRUE && !bbm_com_probe(NULL)) //rc == TRUE : power on success,  OK : 0
        rc = OK;
    else
        rc = ERROR;

    rc = tdmb_fc8080_power_off();
    if( rc == TRUE)
        rc = OK;
    else
        rc = ERROR;

    return rc;
}

static int tdmb_pinctrl_init(void)
{
    fc8080_ctrl_info.tdmb_pinctrl = devm_pinctrl_get(&(fc8080_ctrl_info.pdev->dev));

    if(IS_ERR_OR_NULL(fc8080_ctrl_info.tdmb_pinctrl)) {
        pr_err("%s: Getting pinctrl handle failed\n", __func__);
        return -EINVAL;
    } else {
        printk("%s: Getting pinctrl handle succed\n", __func__);
    }

    fc8080_ctrl_info.gpio_state_active
     = pinctrl_lookup_state(fc8080_ctrl_info.tdmb_pinctrl, "tdmb_pin_active");

     if(IS_ERR_OR_NULL(fc8080_ctrl_info.gpio_state_active)) {
         pr_err("%s: Failed to get the active state pinctrl handle\n", __func__);
         return -EINVAL;
    } else {
        printk("%s: Getting active state pinctrl handle succed\n", __func__);
    }

    fc8080_ctrl_info.gpio_state_sleep
     = pinctrl_lookup_state(fc8080_ctrl_info.tdmb_pinctrl, "tdmb_pin_sleep");

     if(IS_ERR_OR_NULL(fc8080_ctrl_info.gpio_state_sleep)) {
         pr_err("%s: Failed to get the sleep state pinctrl handle\n", __func__);
         return -EINVAL;
    } else {
        printk("%s: Getting sleep state pinctrl handle succed\n", __func__);
    }

    return 0;
}

static int tdmb_set_gpio_config(void)
{
    if(pinctrl_select_state(fc8080_ctrl_info.tdmb_pinctrl, fc8080_ctrl_info.gpio_state_active)) {
        pr_err("%s: error on pinctrl_select_state to active\n", __func__);
        return -EINVAL;
    }
    else {
        printk("%s: success to set pinctrl_select_state to active\n", __func__);
    }

    return 0;
}

static int tdmb_free_gpio_config(void)
{
    if(pinctrl_select_state(fc8080_ctrl_info.tdmb_pinctrl, fc8080_ctrl_info.gpio_state_sleep)) {
        pr_err("%s: error on pinctrl_select_state to sleep\n", __func__);
        return -EINVAL;
    }
    else {
        printk("%s: success to set pinctrl_select_state to sleep\n", __func__);
    }

    return 0;
}

static void async_fc8080_drv_init(void *data, async_cookie_t cookie)
{
    int rc;

    rc = spi_register_driver(&broadcast_tdmb_driver);
    printk("%s : spi_register_driver(%d)\n", __func__, rc);

    return;
}

int __broadcast_dev_init broadcast_tdmb_fc8080_drv_init(void)
{
    int rc;

    if(broadcast_tdmb_drv_check_module_init() != OK) {
        rc = ERROR;
        return rc;
    }

    async_schedule(async_fc8080_drv_init, NULL);

    return 0;
}

static void __exit broadcast_tdmb_fc8080_drv_exit(void)
{
    spi_unregister_driver(&broadcast_tdmb_driver);
    printk("%s\n", __func__);
}

module_init(broadcast_tdmb_fc8080_drv_init);
module_exit(broadcast_tdmb_fc8080_drv_exit);

/* optional part when we include driver code to build-on
it's just used when we make device driver to module(.ko)
so it doesn't work in build-on */
MODULE_DESCRIPTION("FC8080 tdmb device driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("FCI");
