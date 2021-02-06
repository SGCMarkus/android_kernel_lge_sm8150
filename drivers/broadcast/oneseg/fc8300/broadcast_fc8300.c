#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/pm_wakeup.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/pm_qos.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

#include "broadcast_dmb_typedef.h"
#include "broadcast_dmb_drv_ifdef.h"
#include "broadcast_fc8300.h"
#include "fci_types.h"
#include "fci_oal.h"
#include "fc8300_drv_api.h"

struct isdbt_platform_data
{
    int                    powstate;
    struct wakeup_source   wakelock;
    struct spi_device*     spidev;
    struct i2c_client*     pclient;
    struct clk             *clk;
    struct platform_device *pdev;

    struct pinctrl         *pinctrl;
    struct pinctrl_state   *pins_active;
    struct pinctrl_state   *pins_sleep;

    int                    en_gpio;
    int                    reset_gpio;

    int                    lna_gc_gpio;
    int                    lna_en_gpio;

    int                    ldo_en_gpio;

    int                    dtv_use_xtal;
    int                    dtv_xtal_freq;
    int                    dtv_interface_freq;

    int                    use_isdbt_ldo;
    struct regulator       *isdbt_ldo_reg;
    int                    use_lna_ldo;
    struct regulator       *lna_ldo_reg;
};

/* proto type declare */
static int broadcast_isdbt_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int broadcast_isdbt_i2c_remove(struct i2c_client *client);

#define LGE_FC8300_DRV_VER  "1.03.00"

static const struct i2c_device_id isdbt_fc8300_id[] = {
    {"isdbt",    0},
    {},
};

static struct of_device_id isdbt_table[] = {
{ .compatible = "lge, isdbt",}, //Compatible node must match dts
{ },
};

static struct i2c_driver broadcast_isdbt_driver = {
    .driver = {
        .name = "isdbt",
        .owner = THIS_MODULE,
        .of_match_table = isdbt_table,
    },
    .probe = broadcast_isdbt_i2c_probe,
    .remove    = broadcast_isdbt_i2c_remove,
    .id_table = isdbt_fc8300_id,
};

static struct isdbt_platform_data fc8300_pdata;

struct i2c_client*    FCI_GET_I2C_DRIVER(void)
{
    return fc8300_pdata.pclient;
}

static Device_drv device_fc8300 = {
    &broadcast_fc8300_drv_if_power_on,
    &broadcast_fc8300_drv_if_power_off,
    &broadcast_fc8300_drv_if_open,
    &broadcast_fc8300_drv_if_close,
    &broadcast_fc8300_drv_if_set_channel,
    &broadcast_fc8300_drv_if_resync,
    &broadcast_fc8300_drv_if_detect_sync,
    &broadcast_fc8300_drv_if_get_sig_info,
    &broadcast_fc8300_drv_if_get_ch_info,
    &broadcast_fc8300_drv_if_get_dmb_data,
    &broadcast_fc8300_drv_if_reset_ch,
    &broadcast_fc8300_drv_if_user_stop,
    &broadcast_fc8300_drv_if_select_antenna,
    &broadcast_fc8300_drv_if_read_control,
    &broadcast_fc8300_drv_if_get_mode,
};

enum dt_entry_type {
    DT_U32,
    DT_GPIO,
    DT_BOOL,
};

enum dt_entry_status {
    DT_REQ,  /* Required:  fail if missing */
    DT_SGST, /* Suggested: warn if missing */
    DT_OPT,  /* Optional:  don't warn if missing */
};

struct dt_to_pdata_map {
    const char             *dt_name;
    void                   *ptr_data;
    enum dt_entry_status   status;
    enum dt_entry_type     type;
    int                    default_val;
};


static inline void* isdbt_get_platform_data(void)
{
    return &fc8300_pdata;
}

static int isdbt_dt_to_pdata_populate(struct platform_device *pdev,
                    struct dt_to_pdata_map  *itr)
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
            printk("%s: %d is an unknown DT entry type\n", __func__, itr->type);
            ret = -1;
        }

        /* If you like to see debug logs for node, de-block the blow printk */
        /*printk("[%s]DeviceTree entry ret:%d name:%s val:%d\n",
            __func__, ret, itr->dt_name, *((int *)itr->ptr_data)); */

        if (ret) {
            *((int *)itr->ptr_data) = itr->default_val;
            if (itr->status < DT_OPT) {
                printk("%s: Missing '%s' DT entry\n", __func__, itr->dt_name);
                /* cont on err to dump all missing entries */
                if (itr->status == DT_REQ && !err)
                    err = ret;
            }
        }
    }

    return err;
}

static int isdbt_dt_to_platform_data(struct isdbt_platform_data* tpdata)
{
    if(tpdata == NULL) {
        printk("%s: tpdata is null error\n", __func__);
        return -1;
    }
    // of_node is valid, enter to get node information of dts
    if(tpdata->pdev && tpdata->pdev->dev.of_node) {
        struct dt_to_pdata_map dts_map[] = {
            {"en-gpio",
                &tpdata->en_gpio, DT_OPT, DT_GPIO, -1},
            {"reset-gpio",
                &tpdata->reset_gpio, DT_OPT, DT_GPIO, -1},
            {"lna-gc-gpio",
                &tpdata->lna_gc_gpio, DT_OPT, DT_GPIO, -1},
            {"lna-en-gpio",
                &tpdata->lna_en_gpio, DT_OPT, DT_GPIO, -1},
            {"ldo-en-gpio",
                &tpdata->ldo_en_gpio, DT_OPT, DT_GPIO, -1},
            {"use-xtal",
                &tpdata->dtv_use_xtal, DT_OPT, DT_U32, 0},
            {"xtal-freq",
                &tpdata->dtv_xtal_freq, DT_OPT, DT_U32, 0},
            {"ctrl-isdbt-ldo",
                &tpdata->use_isdbt_ldo, DT_OPT, DT_U32, 0},
            {"ctrl-lna-ldo",
                &tpdata->use_lna_ldo, DT_OPT, DT_U32, 0},
            {NULL,  NULL,  0,  0,   0},
        };

        if (isdbt_dt_to_pdata_populate(tpdata->pdev, dts_map)) {
            printk("%s: dt_to_pdata error\n", __func__);
            return -1;
        }
    } else {
        printk("%s: pdev or dev.of_node is not valid\n", __func__);
        return -1;
    }

    //printk("[%s]en_gpio(%d) and-sw-gpio(%d) ldo-en-gpio(%d) lna-en-gpio(%d)\n"
    //    ,__func__, tpdata->en_gpio, tpdata->antsw_gpio, tpdata->lna_en_gpio, tpdata->lna_gc_gpio);
    return 0;
}

static int isdbt_pinctrl_init(struct isdbt_platform_data* tpdata)
{
    tpdata->pinctrl = devm_pinctrl_get(&(tpdata->pdev->dev));
    if(IS_ERR_OR_NULL(tpdata->pinctrl)) {
        pr_err("%s: Getting pinctrl handle failed\n", __func__);
        return -EINVAL;
    }

    tpdata->pins_active
    = pinctrl_lookup_state(tpdata->pinctrl, "isdbt_pin_active");

    if(IS_ERR_OR_NULL(tpdata->pins_active)) {
         pr_err("%s: Failed to get the active state pinctrl handle\n", __func__);
         return -EINVAL;
    }

    tpdata->pins_sleep
    = pinctrl_lookup_state(tpdata->pinctrl, "isdbt_pin_sleep");

    if(IS_ERR_OR_NULL(tpdata->pins_sleep)) {
         pr_err("%s: Failed to get the sleep state pinctrl handle\n", __func__);
         return -EINVAL;
    }
    return 0;
}

static int isdbt_request_gpios(struct isdbt_platform_data* tpdata)
{
    if(pinctrl_select_state(tpdata->pinctrl, tpdata->pins_active)) {
        pr_err("%s: error on pinctrl_select_state to active\n", __func__);
        return -EINVAL;
    }
    return 0;
}

static int isdbt_free_gpios(struct isdbt_platform_data* tpdata)
{
    if(pinctrl_select_state(tpdata->pinctrl, tpdata->pins_sleep)) {
        pr_err("%s: error on pinctrl_select_state to sleep\n", __func__);
        return -EINVAL;
    }
    return 0;
}

/**
 * isdbt_initialize_powsource: tuner, lna power source init
 */
static int isdbt_power_init(struct isdbt_platform_data* tpdata)
{
    if(tpdata == NULL) {
        printk("%s: isdbt_platform_data is null\n", __func__);
    }

    printk("%s: ctrl-dmb-ldo : %d\n", __func__, tpdata->use_isdbt_ldo);
    if(tpdata->use_isdbt_ldo == 1) {
        tpdata->isdbt_ldo_reg = devm_regulator_get(&tpdata->pclient->dev, "dmb-ldo");
        if(IS_ERR(tpdata->isdbt_ldo_reg)) {
            dev_err(&tpdata->pclient->dev, "regulator isdbt_ldo_reg failed\n");
            return -1;
        }
    }

    printk("%s: ctrl-lna-ldo : %d\n", __func__, tpdata->use_lna_ldo);
    if(tpdata->use_lna_ldo == 1) {
        tpdata->lna_ldo_reg= devm_regulator_get(&tpdata->pclient->dev, "lna-ldo");
        if(IS_ERR(tpdata->lna_ldo_reg)) {
            dev_err(&tpdata->pclient->dev, "regulator lna_ldo_reg failed\n");
            return -1;
        }
    }

    return 0;
}

static int isdbt_clk_init(struct isdbt_platform_data* tpdata)
{
    int rc = 0;
    if(tpdata->dtv_use_xtal == FALSE) {
        tpdata->clk = clk_get(&tpdata->pclient->dev, "isdbt_xo");
        if (IS_ERR(tpdata->clk)) {
            rc = PTR_ERR(tpdata->clk);
            dev_err(&tpdata->pclient->dev, "could not get clock\n");
            return rc;
        }

        /* We enable/disable the clock only to assure it works */
        rc = clk_prepare_enable(tpdata->clk);
        if (rc) {
            dev_err(&tpdata->pclient->dev, "could not enable clock\n");
            return rc;
        }
        clk_disable_unprepare(tpdata->clk);
    }
    return rc;
}

static int isdbt_ldo_power_on(struct isdbt_platform_data* tpdata)
{
    int rc = TRUE;

    if(tpdata->use_isdbt_ldo == TRUE) {
        rc = regulator_enable(tpdata->isdbt_ldo_reg);
        if(rc) {
            dev_err(&tpdata->pclient->dev, "unable to enable isdbt_ldo_reg\n");
            return rc;
        } else {
            printk("%s: isdbt_ldo_reg enable\n", __func__);
        }
    }

    if(tpdata->ldo_en_gpio > 0) {
        gpio_set_value(tpdata->ldo_en_gpio, 1);
        printk("%s: set ldo_en_gpio to 1\n", __func__);
        mdelay(10);
    }
    return rc;
}

static void isdbt_ldo_power_off(struct isdbt_platform_data* tpdata)
{
    if(tpdata->ldo_en_gpio > 0) {
        gpio_set_value(tpdata->ldo_en_gpio, 0);
        printk("%s: set isdbt_ldo_en_gpio_value to 0\n", __func__);
        mdelay(10);
    }

    if(tpdata->use_isdbt_ldo == TRUE) {
        regulator_disable(tpdata->isdbt_ldo_reg);
        printk("%s: isdbt_ldo_reg disable\n", __func__);
    }
}

static int isdbt_lna_power_on(struct isdbt_platform_data* tpdata)
{
    int rc = TRUE;
    if(tpdata->use_lna_ldo == TRUE) {
        rc = regulator_enable(tpdata->lna_ldo_reg);
        if(rc) {
            dev_err(&tpdata->pclient->dev, "unable to enable lna_ldo_reg\n");
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

static void isdbt_lna_power_off(struct isdbt_platform_data* tpdata)
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

static inline void isdbt_print_driver_version(void)
{
#if defined(CONFIG_ARCH_MSM) || defined(CONFIG_ARCH_QCOM)
    printk("LGE_FC8300_DRV_VER (QCOM)  : %s\n", LGE_FC8300_DRV_VER);
#else
    printk("LGE_FC8300_DRV_VER (MTK or other)  : %s\n", LGE_FC8300_DRV_VER);
#endif
}
int isdbt_fc8300_power_on(void)
{
    int rc = OK;
    struct isdbt_platform_data* tpdata = isdbt_get_platform_data();
    if(tpdata == NULL) {
        printk("%s: getdata error\n", __func__);
        return FALSE;
    }

    printk("%s\n", __func__);

    rc = isdbt_request_gpios(tpdata);
    printk("%s: isdbt_request_gpios=%d\n", __func__, rc);

    if (tpdata->powstate == TRUE) {
        printk("%s: the power already turn on \n", __func__);
        return TRUE;
    }

    __pm_stay_awake(&tpdata->wakelock);
    gpio_set_value(tpdata->reset_gpio, 0);

    rc = isdbt_ldo_power_on(tpdata);
    printk("%s: isdbt_ldo_power_on : %d\n", __func__, rc);

    gpio_set_value(tpdata->en_gpio, 1);

    if(tpdata->dtv_use_xtal == FALSE) {
        if(tpdata->clk!= NULL) {
            printk("%s: clk enable\n", __func__);
            rc = clk_prepare_enable(tpdata->clk);
        }

        mdelay(2);
    }

    mdelay(3);
    gpio_set_value(tpdata->reset_gpio, 1);
    mdelay(2);

    rc = isdbt_lna_power_on(tpdata);
    printk("%s: isdbt_lna_power_on : %d\n", __func__, rc);

    tpdata->powstate = TRUE;

    return rc;
}

int isdbt_fc8300_is_power_on()
{
    return (int)fc8300_pdata.powstate;
}

int isdbt_fc8300_power_off(void)
{
    int rc = OK;
    struct isdbt_platform_data* tpdata = isdbt_get_platform_data();
    if(tpdata == NULL) {
        printk("%s: getdata error\n", __func__);
        return -1;
    }

    if (tpdata->powstate == TRUE ) {
        isdbt_lna_power_off(tpdata);

        if(tpdata->dtv_use_xtal == FALSE) {
            if(tpdata->clk != NULL) {
                clk_disable_unprepare(tpdata->clk);
            }
        }

        tpdata->powstate = FALSE;
        if(tpdata->en_gpio > 0) {
            gpio_set_value(tpdata->en_gpio, 0);
        }
        mdelay(1);
        if(tpdata->reset_gpio > 0) {
            gpio_set_value(tpdata->reset_gpio, 0);
        }
        mdelay(5);

        isdbt_ldo_power_off(tpdata);
    } else {
        printk("%s: the power already turn off \n", __func__);
    }

    __pm_relax(&tpdata->wakelock);

    rc = isdbt_free_gpios(tpdata);
    printk("%s: tdmb_free_gpios=%d\n", __func__, rc);
    printk("%s: completed\n", __func__);

    return rc;
}

static int broadcast_isdbt_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc = 0;
    struct isdbt_platform_data* tpdata = isdbt_get_platform_data();
    if(client == NULL || tpdata == NULL){
        printk("%s: client is NULL, so spi can not be set\n", __func__);
        return -1;
    }

    if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk("%s: need I2C_FUNC_I2C\n", __func__);

        return -ENODEV;
    }
    pr_err("%s: i2c slave addr [%x] \n", client->addr); //slave Addr

    tpdata->powstate = FALSE;
    tpdata->pclient = client;
    tpdata->pdev = to_platform_device(&client->dev);

    /*
     * Read DeviceTree Info. about GPIO/U32/bool property
     */
    if(isdbt_dt_to_platform_data(tpdata) != OK) {
        printk("%s: isdbt_dt_to_platform_data error\n", __func__);
        return -1;
    }

    /*
     * Initialize Power(LDO)
     */
    if(isdbt_power_init(tpdata)!= OK) {
        printk("%s: isdbt_power_init error\n", __func__);
        return -1;
    }

    /*
     * Initialize Clock Source
     */
    if(isdbt_clk_init(tpdata) != OK) {
        printk("%s: isdbt_clk_init error\n", __func__);
        return -1;
    }

    /*
     * Initialize pintctrl
     */
    if(isdbt_pinctrl_init(tpdata) != OK) {
        printk("%s: isdbt_pinctrl_init error\n", __func__);
        return -1;
    }

    wakeup_source_init(&tpdata->wakelock, dev_name(&client->dev));

    isdbt_print_driver_version();
    rc = broadcast_dmb_drv_start(&device_fc8300);
    printk("%s: start %d \n", __func__, rc);

    isdbt_fc8300_power_on();
    if(tunerbb_drv_fc8300_read_chip_id() != OK) {
        printk("%s: Chip ID read fail!!\n", __func__);
        rc = ERROR;
    }
    printk("%s: rc=%d, End\n", __func__, rc);
    isdbt_fc8300_power_off();

    return rc;
}

static int broadcast_isdbt_i2c_remove(struct i2c_client *client)
{
    int rc = 0;

    printk("%s\n", __func__);
    wakeup_source_trash(&fc8300_pdata.wakelock);

    memset((unsigned char*)&fc8300_pdata, 0x0, sizeof(struct isdbt_platform_data));
    return rc;
}

int broadcast_isdbt_fc8300_drv_init(void)
{
    int rc;
    printk("%s\n", __func__);

    printk("%s: add i2c driver\n", __func__);
    rc = i2c_add_driver(&broadcast_isdbt_driver);
    printk("%s: broadcast_add_driver rc = (%d)\n", __func__, rc);
    return rc;
}

static void __broadcast_dev_exit broadcast_isdbt_fc8300_drv_exit(void)
{
    i2c_del_driver(&broadcast_isdbt_driver);
}

module_init(broadcast_isdbt_fc8300_drv_init);
module_exit(broadcast_isdbt_fc8300_drv_exit);
MODULE_DESCRIPTION("broadcast_isdbt_drv_init");
MODULE_DEVICE_TABLE(i2c, isdbt_fc8300_id);
MODULE_LICENSE("FCI");
