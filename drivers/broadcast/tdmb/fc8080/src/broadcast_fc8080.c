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
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/async.h>
#include "../../broadcast_tdmb_drv_ifdef.h"
#include "../inc/broadcast_fc8080.h"
#include "../inc/fci_types.h"
#include "../inc/bbm.h"

struct tdmb_platform_data
{
    int                        powstate;
    struct spi_device*         spidev;
    struct mutex               mutex;
    struct wakeup_source       wakelock;    /* wake_lock,wake_unlock */
    boolean                    irqstate;
    //spinlock_t                 spinlock;
    struct clk                 *clk;
    struct platform_device     *pdev;

    struct pinctrl             *pinctrl;
    struct pinctrl_state       *pins_active;
    struct pinctrl_state       *pins_sleep;

    int                        en_gpio;

    int                        antsw_gpio;
    int                        dmb_ant_sw_active_value;

    int                        lna_gc_gpio;
    int                        lna_en_gpio;

    int                        dmb_en_lna_en_use_same_gpio;
    int                        ldo_en_gpio;

    int                        dmb_use_xtal;
    int                        dmb_xtal_freq;
    int                        dmb_interface_freq;

    int                        use_dmb_ldo;
    struct regulator           *dmb_ldo_reg;
    int                        use_lna_ldo;
    struct regulator           *lna_ldo_reg;
    int                        use_ant_sw_ldo;
    struct regulator           *ant_sw_ldo_reg;
};

/* external function */
extern int broadcast_fc8080_drv_if_isr(void);
extern void tunerbb_drv_fc8080_isr_control(fci_u8 onoff);
extern int tunerbb_drv_fc8080_fic_cb(uint32 userdata, uint8 *data, int length);
extern int tunerbb_drv_fc8080_msc_cb(uint32 userdata, uint8 subChId, uint8 *data, int length);

/* proto type declare */
static irqreturn_t broadcast_tdmb_spi_event_handler(int irq, void *handle);
static int broadcast_tdmb_fc8080_probe(struct spi_device *spi);
static int broadcast_tdmb_fc8080_remove(struct spi_device *spi);
static int broadcast_tdmb_fc8080_check_chip_id(void);


#define LGE_FC8080_DRV_VER  "1.03.00"

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
static struct tdmb_platform_data fc8080_pdata;

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

enum tdmb_dt_entry_type {
    DT_U32,
    DT_GPIO,
    DT_BOOL,
};

enum tdmb_dt_entry_status {
    DT_REQ,  /* Required:  fail if missing */
    DT_SGST, /* Suggested: warn if missing */
    DT_OPT,  /* Optional:  don't warn if missing */
};

struct tdmb_dt_to_pdata_map {
    const char                *dt_name;
    void                      *ptr_data;
    enum tdmb_dt_entry_status status;
    enum tdmb_dt_entry_type   type;
    int                       default_val;
};


static inline void* tdmb_get_platform_data(void)
{
    return &fc8080_pdata;
}

static int tdmb_dt_to_pdata_populate(struct platform_device *pdev,
                    struct tdmb_dt_to_pdata_map  *itr)
{
    int  ret, err = 0;
    struct device_node *node = pdev->dev.of_node;

    for (; itr->dt_name; ++itr) {
        switch (itr->type) {
        case DT_GPIO:
            ret = of_get_named_gpio(node, itr->dt_name, 0);
            if (ret >= 0) {
                *((int *) itr->ptr_data) = ret;
                ret = 0;
            }
            break;
        case DT_U32:
            ret = of_property_read_u32(node, itr->dt_name,
                    (u32 *) itr->ptr_data);
            break;
        case DT_BOOL:
            *((bool *) itr->ptr_data) =
                of_property_read_bool(node, itr->dt_name);
            ret = 0;
            break;
        default:
            printk("%d is an unknown DT entry type\n",itr->type);
            ret = -1;
        }

        /* If you like to see debug logs for node, de-block the blow printk */
        /*printk("[%s]DeviceTree entry ret:%d name:%s val:%d\n",
            __func__, ret, itr->dt_name, *((int *)itr->ptr_data)); */

        if (ret) {
            *((int *)itr->ptr_data) = itr->default_val;
            if (itr->status < DT_OPT) {
                printk("Missing '%s' DT entry\n", itr->dt_name);
                /* cont on err to dump all missing entries */
                if (itr->status == DT_REQ && !err)
                    err = ret;
            }
        }
    }

    return err;
}

static int tdmb_dt_to_platform_data(struct tdmb_platform_data* tpdata)
{
    if(tpdata == NULL) {
        printk("[%s]tpdata is null error\n", __func__);
        return -1;
    }
    // of_node is valid, enter to get node information of dts
    if(tpdata->pdev && tpdata->pdev->dev.of_node) {
        struct tdmb_dt_to_pdata_map tdmb_dts_map[] = {
            {"en-gpio",
                &tpdata->en_gpio, DT_OPT, DT_GPIO, -1},
            {"ant-sw-gpio",
                &tpdata->antsw_gpio, DT_OPT, DT_GPIO, -1},
            {"lna-gc-gpio",
                &tpdata->lna_gc_gpio, DT_OPT, DT_GPIO, -1},
            {"lna-en-gpio",
                &tpdata->lna_en_gpio, DT_OPT, DT_GPIO, -1},
            {"ldo-en-gpio",
                &tpdata->ldo_en_gpio, DT_OPT, DT_GPIO, -1},
            {"ant-sw-active-value",
                &tpdata->dmb_ant_sw_active_value, DT_OPT, DT_U32, 0},
            {"dmb-en-lna-en-same-gpio",
                &tpdata->dmb_en_lna_en_use_same_gpio, DT_OPT, DT_U32, 0},
            {"use-xtal",
                &tpdata->dmb_use_xtal, DT_OPT, DT_U32, 0},
            {"xtal-freq",
                &tpdata->dmb_xtal_freq, DT_OPT, DT_U32, 0},
            {"interface-freq",
                &tpdata->dmb_interface_freq, DT_OPT, DT_U32, 19200},
            {"ctrl-dmb-ldo",
                &tpdata->use_dmb_ldo, DT_OPT, DT_U32, 0},
            {"ctrl-lna-ldo",
                &tpdata->use_lna_ldo, DT_OPT, DT_U32, 0},
            {"ctrl-ant-sw-ldo",
                &tpdata->use_ant_sw_ldo, DT_OPT, DT_U32, 0},
            {NULL,  NULL,  0,  0,   0},
        };

        if (tdmb_dt_to_pdata_populate(tpdata->pdev, tdmb_dts_map)) {
            printk("[%s]dt_to_pdata error\n", __func__);
            return -1;
        }
    } else {
        printk("[%s]pdev or dev.of_node is not valid\n", __func__);
        return -1;
    }

    //printk("[%s]en_gpio(%d) and-sw-gpio(%d) ldo-en-gpio(%d) lna-en-gpio(%d)\n"
    //    ,__func__, tpdata->en_gpio, tpdata->antsw_gpio, tpdata->lna_en_gpio, tpdata->lna_gc_gpio);
    return 0;
}

static int tdmb_pinctrl_init(struct tdmb_platform_data* tpdata)
{
    tpdata->pinctrl = devm_pinctrl_get(&(tpdata->pdev->dev));
    if(IS_ERR_OR_NULL(tpdata->pinctrl)) {
        pr_err("%s: Getting pinctrl handle failed\n", __func__);
        return -EINVAL;
    }

    tpdata->pins_active = pinctrl_lookup_state(tpdata->pinctrl, "tdmb_pin_active");
    if(IS_ERR_OR_NULL(tpdata->pins_active)) {
         pr_err("%s: Failed to get the active state pinctrl handle\n", __func__);
         return -EINVAL;
    }

    tpdata->pins_sleep = pinctrl_lookup_state(tpdata->pinctrl, "tdmb_pin_sleep");
    if(IS_ERR_OR_NULL(tpdata->pins_sleep)) {
         pr_err("%s: Failed to get the sleep state pinctrl handle\n", __func__);
         return -EINVAL;
    }
    return 0;
}

static int tdmb_request_gpios(struct tdmb_platform_data* tpdata)
{
    if(pinctrl_select_state(tpdata->pinctrl, tpdata->pins_active)) {
        pr_err("%s: error on pinctrl_select_state to active\n", __func__);
        return -EINVAL;
    }
    return 0;
}

static int tdmb_free_gpios(struct tdmb_platform_data* tpdata)
{
    if(pinctrl_select_state(tpdata->pinctrl, tpdata->pins_sleep)) {
        pr_err("%s: error on pinctrl_select_state to sleep\n", __func__);
        return -EINVAL;
    }
    return 0;
}

/**
 * tdmb_initialize_powsource: tuner, lna, switch power source init
 */
static int tdmb_power_init(struct tdmb_platform_data* tpdata)
{
    if(tpdata == NULL) {
        printk("%s: tdmb_platform_data is null\n", __func__);
    }

    printk("%s: ctrl-dmb-ldo : %d\n", __func__, tpdata->use_dmb_ldo);
    if(tpdata->use_dmb_ldo == 1) {
        tpdata->dmb_ldo_reg = devm_regulator_get(&tpdata->spidev->dev, "dmb-ldo");
        if(IS_ERR(tpdata->dmb_ldo_reg)) {
            dev_err(&tpdata->spidev->dev, "regulator dmb_ldo_reg failed\n");
            return -1;
        }
    }

    printk("%s: ctrl-lna-ldo : %d\n", __func__, tpdata->use_lna_ldo);
    if(tpdata->use_lna_ldo == 1) {
        tpdata->lna_ldo_reg= devm_regulator_get(&tpdata->spidev->dev, "lna-ldo");
        if(IS_ERR(tpdata->lna_ldo_reg)) {
            dev_err(&tpdata->spidev->dev, "regulator lna_ldo_reg failed\n");
            return -1;
        }
    }

    printk("%s: ctrl-ant-sw-ldo : %d\n", __func__, tpdata->use_ant_sw_ldo);
    if(tpdata->use_ant_sw_ldo == 1) {
        tpdata->ant_sw_ldo_reg= devm_regulator_get(&tpdata->spidev->dev, "ant-sw-ldo");
        if(IS_ERR(tpdata->ant_sw_ldo_reg)) {
            dev_err(&tpdata->spidev->dev, "regulator ant_sw_ldo_reg failed\n");
            return -1;
        }
    }
    return 0;
}

static int tdmb_clk_init(struct tdmb_platform_data* tpdata)
{
    int rc = 0;
    if(tpdata->dmb_use_xtal == FALSE) {
        tpdata->clk = clk_get(&tpdata->spidev->dev, "tdmb_xo");
        if (IS_ERR(tpdata->clk)) {
            rc = PTR_ERR(tpdata->clk);
            dev_err(&tpdata->spidev->dev, "could not get clock\n");
            return rc;
        }

        /* We enable/disable the clock only to assure it works */
        rc = clk_prepare_enable(tpdata->clk);
        if (rc) {
            dev_err(&tpdata->spidev->dev, "could not enable clock\n");
            return rc;
        }
        clk_disable_unprepare(tpdata->clk);
    }
    return rc;
}

static int tdmb_ldo_power_on(struct tdmb_platform_data* tpdata)
{
    int rc = TRUE;

    if(tpdata->use_dmb_ldo == TRUE) {
        rc = regulator_enable(tpdata->dmb_ldo_reg);
        if(rc) {
            dev_err(&tpdata->spidev->dev, "unable to enable dmb_ldo_reg\n");
            return rc;
        } else {
            printk("%s: dmb_ldo_reg enable\n", __func__);
        }
    }

    if(tpdata->ldo_en_gpio > 0) {
        gpio_set_value(tpdata->ldo_en_gpio, 1);
        printk("%s: set ldo_en_gpio to 1\n", __func__);
        mdelay(10);
    }
    return rc;
}

static void tdmb_ldo_power_off(struct tdmb_platform_data* tpdata)
{
    if(tpdata->ldo_en_gpio > 0) {
        gpio_set_value(tpdata->ldo_en_gpio, 0);
        printk("%s: set tdmb_ldo_en_gpio_value to 0\n", __func__);
        mdelay(10);
    }

    if(tpdata->use_dmb_ldo == TRUE) {
        regulator_disable(tpdata->dmb_ldo_reg);
        printk("%s: dmb_ldo_reg disable\n", __func__);
    }
}

static int tdmb_lna_power_on(struct tdmb_platform_data* tpdata)
{
    int rc = TRUE;
    if(tpdata->use_lna_ldo == TRUE) {
        rc = regulator_enable(tpdata->lna_ldo_reg);
        if(rc) {
            dev_err(&tpdata->spidev->dev, "unable to enable lna_ldo_reg\n");
            return rc;
        } else {
            printk("%s: lna_ldo_reg enable\n", __func__);
        }
    }

    if(tpdata->lna_gc_gpio > 0) {
        gpio_set_value(tpdata->lna_gc_gpio, 0);
    }

    if(tpdata->lna_en_gpio > 0) {
        gpio_set_value(tpdata->lna_en_gpio, 1);
    }

    return rc;
}

static void tdmb_lna_power_off(struct tdmb_platform_data* tpdata)
{
    if(tpdata->lna_gc_gpio > 0) {
        if(tpdata->lna_en_gpio > 0) {
            gpio_set_value(tpdata->lna_gc_gpio, 0); // OFF mode
        } else {
            gpio_set_value(tpdata->lna_gc_gpio, 1); // Bypass mode
        }
    }

    if(tpdata->lna_en_gpio > 0) {
        gpio_set_value(tpdata->lna_en_gpio, 0);
    }

    if(tpdata->use_lna_ldo == TRUE) {
        regulator_disable(tpdata->lna_ldo_reg);
        printk("%s: lna_ldo_reg disable\n", __func__);
    }
}

static int tdmb_ant_sw_power_on(struct tdmb_platform_data* tpdata)
{
    int rc = TRUE;
    if (tpdata->use_ant_sw_ldo == TRUE) {
        rc = regulator_enable(tpdata->ant_sw_ldo_reg);
        if(rc) {
            dev_err(&tpdata->spidev->dev, "unable to enable ant_sw_ldo_reg\n");
            return rc;
        } else {
            printk("%s: ant_sw_ldo_reg enable\n", __func__);
        }
    }

    if(tpdata->antsw_gpio > 0) {
        gpio_set_value(tpdata->antsw_gpio, tpdata->dmb_ant_sw_active_value);
        printk("%s: tdmb_ant_sw_gpio_value : %d\n", __func__, tpdata->dmb_ant_sw_active_value);
    }

    return rc;
}

static void tdmb_ant_sw_power_off(struct tdmb_platform_data* tpdata)
{
    if(tpdata->antsw_gpio > 0) {
        gpio_set_value(tpdata->antsw_gpio, !(tpdata->dmb_ant_sw_active_value));
        printk("%s: tdmb_ant_sw_gpio_value : %d\n", __func__, !(tpdata->dmb_ant_sw_active_value));
    }

    if(tpdata->use_ant_sw_ldo == TRUE) {
        regulator_disable(tpdata->ant_sw_ldo_reg);
        printk("%s: ant_sw_ldo_reg disable\n", __func__);
    }
}

static void tdmb_interrupt_lock(struct tdmb_platform_data* tpdata)
{
    if (tpdata->spidev == NULL) {
        printk("%s fail\n", __func__);
    } else {
        disable_irq(tpdata->spidev->irq);
    }
}

static void tdmb_interrupt_free(struct tdmb_platform_data* tpdata)
{
    if (tpdata->spidev == NULL) {
        printk("tdmb_interrupt_free fail\n");
    } else {
        enable_irq(tpdata->spidev->irq);
    }
}

static inline void tdmb_print_driver_version(void)
{
#if defined(CONFIG_ARCH_MSM) || defined(CONFIG_ARCH_QCOM)
    printk("LGE_FC8080_DRV_VER (QCOM)  : %s\n", LGE_FC8080_DRV_VER);
#else
    printk("LGE_FC8080_DRV_VER (MTK or other)  : %s\n", LGE_FC8080_DRV_VER);
#endif
}

struct spi_device *tdmb_fc8080_get_spi_device(void)
{
    return fc8080_pdata.spidev;
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

    if(ms > 100) {
        /* 100, 200, 300 more only , Otherwise this must be modified e.g (ms + 40)/50 */
        wait_loop = (ms /100);
        wait_ms = 100;
    }

    do {
        mdelay(wait_ms);
        if(user_stop_flg == 1) {
            printk("** UserStop flag is set, Stop wait ms =(%d) **\n", ms);
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
    return (int)fc8080_pdata.powstate;
}

unsigned int tdmb_fc8080_get_xtal_freq(void)
{
    return (unsigned int)fc8080_pdata.dmb_xtal_freq;
}

#ifdef FEATURE_POWER_ON_RETRY
int tdmb_fc8080_power_on_retry(void)
{
    int res;
    int i;

    struct tdmb_platform_data* tpdata = tdmb_get_platform_data();
    if(tpdata == NULL) {
        printk("[FC8080] tdmb_fc8080_power_on_retry getdata error\n");
        return -1;
    }

    tdmb_interrupt_lock(tpdata);

    for(i = 0 ; i < 10; i++)
    {
        printk("[FC8080] tdmb_fc8080_power_on_retry :  %d\n", i);

        if(tpdata->dmb_use_xtal == FALSE) {
            if(tpdata->clk != NULL) {
                clk_disable_unprepare(tpdata->clk);
                printk("[FC8080] retry clk_disable %d\n", i);
            } else {
                printk("[FC8080] ERR fc8080 clk is NULL\n");
            }
        }

        if(tpdata->en_gpio > 0) {
            gpio_set_value(tpdata->en_gpio, 0);
            mdelay(150);

            gpio_set_value(tpdata->en_gpio, 1);
            mdelay(5);
        }

        if(tpdata->dmb_use_xtal == FALSE) {
            if(tpdata->clk != NULL) {
                res = clk_prepare_enable(tpdata->clk);
                if (res) {
                    printk("[FC8080] retry clk_prepare_enable fail %d\n", i);
                }
            } else {
                printk("[FC8080] ERR fc8080 clk is NULL\n");
            }
        }
        mdelay(30);

        res = bbm_com_probe(NULL);

        if (!res)
            break;

    }

    tdmb_interrupt_free(tpdata);

    return res;
}
#endif

int tdmb_fc8080_power_on(void)
{
    int rc = OK;
    struct tdmb_platform_data* tpdata = tdmb_get_platform_data();
    if(tpdata == NULL) {
        printk("[FC8080] tdmb_fc8080_power_on getdata error\n");
        return FALSE;
    }

    printk("%s\n", __func__);

    rc = tdmb_request_gpios(tpdata);
    printk("%s: tdmb_request_gpios=%d\n", __func__, rc);
    mdelay(5);

    if (tpdata->powstate == TRUE) {
        printk("%s: the power already turn on \n", __func__);
        return TRUE;
    }

    rc = tdmb_ldo_power_on(tpdata);
    printk("%s: tdmb_ldo_power_on : %d\n", __func__, rc);

    __pm_stay_awake(&tpdata->wakelock);

    if(tpdata->en_gpio > 0) {
        gpio_set_value(tpdata->en_gpio, 0);
        mdelay(5);
        gpio_set_value(tpdata->en_gpio, 1);
        mdelay(5);
    }

    //[TDMBDEV-2766] (issue) DMB_EN control that FC8080 LDO_EN & 1.8V regulator
    //So add 100ms delay for power sequence before clk enable
    if(tpdata->dmb_en_lna_en_use_same_gpio == TRUE) {
        mdelay(100);
    }

    if(tpdata->dmb_use_xtal == FALSE) {
        if(tpdata->clk != NULL) {
            rc = clk_prepare_enable(tpdata->clk);
            if (rc) {
                if(tpdata->en_gpio > 0) {
                    gpio_set_value(tpdata->en_gpio, 0);
                }
                dev_err(&tpdata->spidev->dev, "could not enable clock\n");
                return FALSE;
            }
        }
    }
    mdelay(30); /* Due to X-tal stablization Time */

    rc = tdmb_lna_power_on(tpdata);
    printk("%s: tdmb_lna_power_on : %d\n", __func__,rc);
    rc = tdmb_ant_sw_power_on(tpdata);
    printk("%s: tdmb_ant_sw_power_on : %d\n", __func__,rc);

    tdmb_interrupt_free(tpdata);
    tpdata->powstate = TRUE;

    printk("%s: completed \n", __func__);
    rc = TRUE;

    return rc;
}

int tdmb_fc8080_power_off(void)
{
    int rc = FALSE;
    struct tdmb_platform_data* tpdata = tdmb_get_platform_data();
    if(tpdata == NULL) {
        printk("[FC8080] tdmb_fc8080_power_on getdata error\n");
        return -1;
    }

    if (tpdata->powstate == TRUE ) {
        tdmb_interrupt_lock(tpdata);
        tdmb_lna_power_off(tpdata);
        tdmb_ant_sw_power_off(tpdata);

        if(tpdata->dmb_use_xtal == FALSE) {
            if(tpdata->clk != NULL) {
                clk_disable_unprepare(tpdata->clk);
            }
        }

        tpdata->powstate = FALSE;
        if(tpdata->en_gpio > 0) {
            gpio_set_value(tpdata->en_gpio, 0);
        }

        __pm_relax(&tpdata->wakelock);
        mdelay(20);

        tdmb_ldo_power_off(tpdata);

    } else {
        printk("%s: tdmb_fc8080_power_on the power already turn off \n", __func__);
    }

    rc = tdmb_free_gpios(tpdata);
    printk("%s: tdmb_free_gpios=%d\n", __func__, rc);

    printk("%s: completed\n", __func__);
    rc = TRUE;

    return rc;
}

int tdmb_fc8080_select_antenna(unsigned int sel)
{
    return FALSE;
}

int tdmb_fc8080_spi_write_read(uint8* tx_data, int tx_length, uint8 *rx_data, int rx_length)
{
    int rc;
    struct tdmb_platform_data* tpdata = tdmb_get_platform_data();
    struct spi_transfer    t;
    struct spi_message    m;

    memset(&t, 0, sizeof(struct spi_transfer));
    memset(&m, 0, sizeof(struct spi_message));

    t.tx_buf = tx_data;
    t.rx_buf = rx_data;
    t.len = tx_length+rx_length;
   
    if (tpdata == NULL || tpdata->spidev == NULL)
    {
        printk("%s: error txdata=%p, length=%d\n", __func__, (void *)tx_data, tx_length+rx_length);
        return FALSE;
    }

    mutex_lock(&tpdata->mutex);
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    rc = spi_sync(tpdata->spidev, &m);

    if ( rc < 0 )
    {
        printk("%s: result(%d), actual_len=%d\n", __func__, rc, m.actual_length);
    }

    mutex_unlock(&tpdata->mutex);

    return TRUE;
}

static irqreturn_t broadcast_tdmb_spi_event_handler(int irq, void *handle)
{
    struct tdmb_platform_data* tpdata = handle;;

    if ( tpdata && tpdata->powstate ) {
        if (tpdata->irqstate) {
            printk("[%s]spi read function is so late, ignore\n", __func__);
            return IRQ_HANDLED;
        }

        tunerbb_drv_fc8080_isr_control(0);
        tpdata->irqstate = TRUE;
        broadcast_fc8080_drv_if_isr();
        tpdata->irqstate = FALSE;
        tunerbb_drv_fc8080_isr_control(1);
    } else {
        printk("tdmb_spi_handler is called, but device is off state\n");
    }

    return IRQ_HANDLED;
}

static int broadcast_tdmb_fc8080_probe(struct spi_device *spi)
{
    int rc;
    struct tdmb_platform_data* tpdata = tdmb_get_platform_data();
    if(spi == NULL || tpdata == NULL){
        printk("%s: spi is NULL, so spi can not be set\n", __func__);
        return -1;
    }

    tpdata->powstate = FALSE;
    tpdata->pdev = to_platform_device(&spi->dev);

    /*
     * Read DeviceTree Info. about GPIO/U32/bool property
     */
    if(tdmb_dt_to_platform_data(tpdata) != OK) {
        printk("[%s]dt_to_platform_data error\n", __func__);
        return -1;
    }
    tpdata->spidev = spi;
    tpdata->spidev->mode = SPI_MODE_0;
    tpdata->spidev->bits_per_word = 8;
    tpdata->spidev->max_speed_hz = (tpdata->dmb_interface_freq*1000);

    /*
     * Initialize Power(LDO)
     */
    if(tdmb_power_init(tpdata)!= OK) {
        printk("[%s]tdmb_power_init error\n", __func__);
        return -1;
    }

    /*
     * Initialize Clock Source
     */
    if(tdmb_clk_init(tpdata) != OK) {
        printk("[%s]tdmb_clk_init error\n", __func__);
        return -1;
    }

    /*
     * Initialize pintctrl
     */
    if(tdmb_pinctrl_init(tpdata) != OK) {
        printk("[%s]tdmb_clk_init error\n", __func__);
        return -1;
    }

    // Once I have a spi_device structure I can do a transfer anytime
    rc = spi_setup(spi);
    printk("%s: spi_setup=%d, 2019.09.02.14:30\n", __func__, rc);

    rc = request_threaded_irq(spi->irq, NULL, broadcast_tdmb_spi_event_handler,
        IRQF_ONESHOT | IRQF_TRIGGER_FALLING, spi->dev.driver->name, tpdata);
    if (rc != OK) {
        printk("%s: request_threaded_irq failed = %d\n", __func__, rc);
    }

    tdmb_interrupt_lock(tpdata);

    mutex_init(&tpdata->mutex);
    wakeup_source_init(&tpdata->wakelock, dev_name(&spi->dev));
    //spin_lock_init(&tpdata->spinlock);

    rc = broadcast_tdmb_drv_start(&device_fc8080);
    tdmb_print_driver_version();

    printk("[%s] start %d \n", __func__, rc);
    /*
     * [Start] This code for kmalloc fic & msc buffer when device booting.
     * This code's original location : tunerbb_drv_fc8080_init() in tdmb_tunerbbdrv_fc8080.c
     */
    bbm_com_hostif_select(NULL, BBM_SPI);
#if !defined(STREAM_TS_UPLOAD)
    bbm_com_fic_callback_register((UDynamic_32_64)NULL, tunerbb_drv_fc8080_fic_cb);
    bbm_com_msc_callback_register((UDynamic_32_64)NULL, tunerbb_drv_fc8080_msc_cb);
#endif
    /*
     * [End] This code for fic & msc buffer kmalloc when device booting.
     */

    if(broadcast_tdmb_fc8080_check_chip_id() != OK) {
        printk("%s: Chip ID read fail!!\n", __func__);
        rc = ERROR;
    }
    printk("%s: rc=%d, End\n", __func__, rc);
    return rc;
}

static int broadcast_tdmb_fc8080_remove(struct spi_device *spi)
{
    struct tdmb_platform_data* tpdata = tdmb_get_platform_data();
    printk("%s \n", __func__);
    if(tpdata == NULL) {
        printk("[%s]tdmb_platform_data null\n", __func__);
        return ERROR;
    }

    free_irq(tpdata->spidev->irq, NULL);
    mutex_destroy(&tpdata->mutex);
    wakeup_source_trash(&tpdata->wakelock);

    memset((void*)tpdata, 0x0, sizeof(struct tdmb_platform_data));
    return OK;
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


static void async_fc8080_drv_init(void *data, async_cookie_t cookie)
{
    int rc;

    rc = spi_register_driver(&broadcast_tdmb_driver);
    printk("%s : spi_register_driver(%d)\n", __func__, rc);

    return;
}

static int __broadcast_dev_init broadcast_tdmb_fc8080_drv_init(void)
{

    struct tdmb_platform_data* tpdata = tdmb_get_platform_data();
    if(tpdata == NULL || broadcast_tdmb_drv_check_module_init() != OK) {
        return ERROR;
    }

    memset((void*)tpdata, 0x0, sizeof(struct tdmb_platform_data));
    async_schedule(async_fc8080_drv_init, NULL);

    return OK;
}

static void __broadcast_dev_exit broadcast_tdmb_fc8080_drv_exit(void)
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
