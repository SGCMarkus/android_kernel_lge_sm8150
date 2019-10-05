/* Copyright (c) 2017 LG Electronics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <soc/qcom/lge/lge_pkm_drv.h>
#include <soc/qcom/lge/board_lge.h>
#include "../../../misc/qseecom_kernel.h"

/* driver name */
#define PKM_NAME                     "lge_pkm"

#define SET_PKM_BASELINE               1
#define SET_MOD_BASELINE               2

#define SAVE_MODULE_BASELINE           13
#define ADD_MODULE_BASELINE            16
#define DEL_MODULE_BASELINE            17

#define SUCCESS                        0
#define FAILED                         (-1)

#define LGPKI_TZAPP_NAME               "lgpki64"
#define MAX_APP_NAME                   25
#define QSEECOM_SBUFF_SIZE             0x1000
#define TOTAL_DATA_SIZE                290
#define TOTAL_MOD_DATA_SIZE            72

struct lgpki_send_cmd {
    uint32_t cmd_id;
    uint8_t data[TOTAL_DATA_SIZE];
    uint8_t mod_data[TOTAL_MOD_DATA_SIZE];
};

struct lgpki_send_cmd_rsp {
    int32_t status;
    int32_t errno_type;
    uint8_t data[TOTAL_DATA_SIZE];
};

struct pkm_modules {
    char name[MODULE_NAME_LEN];
    void *base;
    unsigned int text_size;
    unsigned int ro_size;
    struct list_head list;
};

typedef enum {
    LGE_PKM_NONE = 0,
    LGE_PKM_RUNNING,
    LGE_PKM_COMPLETE
} pkm_status_t;

static struct qseecom_handle *l_handle = NULL;
static struct class *pkm_class;
static struct device *pkm_dev;
static int pkm_major;
static DEFINE_MUTEX(lgpki_mutex);
static int pkm_flag = 0;
static int is_pkm_enabled = -1;
static LIST_HEAD(pkm_module_list);
static pkm_status_t lge_pkm_status = LGE_PKM_NONE;

static int __init lge_check_pkm_bootcmd(char *str)
{
    int ret = 0;
    ret = kstrtoint(str, 10, &is_pkm_enabled);
    if (ret)
        is_pkm_enabled = 0;
    pr_debug("[LGE_PKM] is_pkm_enabled(%d)\n", is_pkm_enabled);
    return 1;
}
__setup("androidboot.vendor.lge.pkm=", lge_check_pkm_bootcmd);

pkm_status_t lge_get_pkm_status(void)
{
    return lge_pkm_status;
}

static void lge_set_pkm_status(pkm_status_t pkm_status)
{
    lge_pkm_status = pkm_status;
}

static int32_t lgpki_start(void)
{
    int32_t ret = 0;

    if(l_handle != NULL) {
        pr_err("[LGE_PKM] already opened qseecom\n");
        ret = FAILED;
        goto exit;
    }

    ret = qseecom_start_app(&l_handle, LGPKI_TZAPP_NAME, QSEECOM_SBUFF_SIZE);
    if(ret) {
        pr_err("[LGE_PKM] Failed to load LGPKI qsapp, ret(%d)\n", ret);
    }

exit:
    return ret;
}

static int32_t lgpki_shutdown(void)
{
    int32_t ret = 0;

    if(l_handle == NULL) {
        pr_err("[LGE_PKM] qseecom handle is NULL\n");
        ret = FAILED;
        goto exit;
    }

    ret = qseecom_shutdown_app(&l_handle);
    if(ret) {
        pr_err("[LGE_PKM] Failed to shutdown LGPKI qsapp, ret(%d)\n", ret);
    }

exit:
    return ret;
}

static int32_t get_rsp_buffers(struct qseecom_handle *q_handle, void **cmd, int *cmd_len, void **rsp, int *rsp_len)
{
    if ((q_handle == NULL) ||
            (cmd == NULL) || (cmd_len == NULL) ||
            (rsp == NULL) || (rsp_len == NULL)) {
        pr_err("[LGE_PKM] Bad parameters\n");
        return FAILED;
    }

    if (*cmd_len & QSEECOM_ALIGN_MASK)
        *cmd_len = QSEECOM_ALIGN(*cmd_len);

    if (*rsp_len & QSEECOM_ALIGN_MASK)
        *rsp_len = QSEECOM_ALIGN(*rsp_len);

    if ((*rsp_len + *cmd_len) > QSEECOM_SBUFF_SIZE) {
        pr_err("[LGE_PKM] Shared buffer too small to hold cmd=%d and rsp=%d\n", *cmd_len, *rsp_len);
        return FAILED;
    }

    *cmd = q_handle->sbuf;
    *rsp = q_handle->sbuf + *cmd_len;

    return SUCCESS;
}

static int32_t lgpki_send_req_command(int32_t cmd_id, struct pkm_modules *mod)
{
    int32_t ret = 0;
    int32_t req_len = 0;
    int32_t rsp_len = 0;
    struct lgpki_send_cmd *req;
    struct lgpki_send_cmd_rsp *rsp;

    if(l_handle == NULL) {
        pr_err("[LGE_PKM] already closed qseecom\n");
        ret = FAILED;
        goto exit;
    }

    if (cmd_id != SET_PKM_BASELINE && mod == NULL) {
        pr_err("[LGE_PKM] mod info is null\n");
        ret = FAILED;
        goto exit;
    }

    req_len = sizeof(struct lgpki_send_cmd);
    rsp_len = sizeof(struct lgpki_send_cmd_rsp);

    ret = get_rsp_buffers(l_handle, (void **)&req, &req_len, (void **)&rsp, &rsp_len);
    if (!ret) {
        req->cmd_id = cmd_id;
        if (req->cmd_id == SAVE_MODULE_BASELINE || req->cmd_id == ADD_MODULE_BASELINE || req->cmd_id == DEL_MODULE_BASELINE) {
            memcpy(req->mod_data, mod, TOTAL_MOD_DATA_SIZE);
        }
        ret = qseecom_send_command(l_handle, (void *)req, req_len, (void *)rsp, rsp_len);
        if(ret) {
            pr_err("[LGE_PKM] Failed to send cmd_id, ret(%d)\n", ret);
        }
    }
exit:
    return ret;
}

static int save_modules_baseline(void)
{
    int ret = FAILED;
    struct pkm_modules *pkm_mod;
    int cnt = 0;

    list_for_each_entry(pkm_mod, &pkm_module_list, list) {
        if (pkm_mod != NULL) {
            if (cnt == 0) {
                ret = lgpki_send_req_command(SAVE_MODULE_BASELINE, pkm_mod);
            } else {
                ret = lgpki_send_req_command(ADD_MODULE_BASELINE, pkm_mod);
            }
            if (ret) {
                pr_err("[LGE_PKM] lgpki_send_req_command %d\n", ret);
                return FAILED;
            }
            pkm_mod->base = NULL;
            pkm_mod->text_size = 0;
            pkm_mod->ro_size = 0;
            cnt++;
        }
    }
    return SUCCESS;
}

static int lgpki_call(int cmd, struct pkm_modules *pkm_mod)
{
    int ret = FAILED;
    int status = SUCCESS;

    mutex_lock(&lgpki_mutex);
    pr_debug("[LGE_PKM] lgpki_call::%d \n", cmd);
    ret = lgpki_start();
    if (ret) {
        pr_err("[LGE_PKM] lgpki_start failed %d\n", ret);
        status = FAILED;
        mutex_unlock(&lgpki_mutex);
        return status;
    }

    switch(cmd) {
        case SET_PKM_BASELINE :
            {
                ret = lgpki_send_req_command(SET_PKM_BASELINE, pkm_mod);
                if (ret) {
                    pr_err("[LGE_PKM] lgpki_send_req_command %d\n", ret);
                    status = FAILED;
                    break;
                }

                pkm_flag = 1;
            }
            break;

        case SET_MOD_BASELINE :
            {
                ret = save_modules_baseline();
                if (ret) {
                    pr_err("[LGE_PKM] send_mod_info %d\n", ret);
                    status = FAILED;
                    break;
                }

                lge_set_pkm_status(LGE_PKM_COMPLETE);
            }
            break;

        case ADD_MODULE_BASELINE :
        case DEL_MODULE_BASELINE :
            {
                ret = lgpki_send_req_command(cmd, pkm_mod);
                if (ret) {
                    pr_err("[LGE_PKM] lgpki_send_req_command %d\n", ret);
                    status = FAILED;
                    break;
                }
            }
            break;

        default:
            {
            pr_err("[LGE_PKM] Invalid CMD(%d) \n", cmd);
            status = FAILED;
            }
            break;
    }

    ret = lgpki_shutdown();
    if (ret) {
        pr_err("[LGE_PKM] lgpki_shutdown failed %d\n", ret);
        status = FAILED;
    }
    mutex_unlock(&lgpki_mutex);

    return status;
}

int lge_pkm_add_module(struct module *mod)
{
    struct pkm_modules *pkm_mod = NULL;
    int ret = FAILED;

    if (lge_get_pkm_status() != LGE_PKM_NONE && is_pkm_enabled == 1) {

        if (mod == NULL) {
            pr_err("[LGE_PKM] mod info is NULL\n");
            return ret;
        }

        pkm_mod = kmalloc( sizeof(struct pkm_modules), GFP_KERNEL );
        if (!pkm_mod) {
            pr_err("[LGE_PKM] Failed to allocate kernel memory\n");
            return ret;
        }

        strlcpy(pkm_mod->name, mod->name, MODULE_NAME_LEN);
        pkm_mod->base = mod->core_layout.base;
        pkm_mod->text_size = mod->core_layout.text_size;
        pkm_mod->ro_size = mod->core_layout.ro_size;

        pr_debug("[LGE_PKM] add_module: module_name= %s\n", pkm_mod->name);
        pr_debug("[LGE_PKM] add_module: module base= %p\n", pkm_mod->base);
        pr_debug("[LGE_PKM] add_module: module text_size= %u\n", pkm_mod->text_size);
        pr_debug("[LGE_PKM] add_module: module ro_size= %u\n", pkm_mod->ro_size);

        if (lge_get_pkm_status() == LGE_PKM_COMPLETE) {
            ret = lgpki_call(ADD_MODULE_BASELINE, pkm_mod);
            if (ret != SUCCESS) {
                pr_err("[LGE_PKM] lgpki_call failed(%d)\n", ret);
            }
            pkm_mod->base = NULL;
            pkm_mod->text_size = 0;
            pkm_mod->ro_size = 0;
        }
        list_add_tail(&pkm_mod->list, &pkm_module_list);
    }

    return SUCCESS;
}

int lge_pkm_delete_module(const char *name)
{
    struct pkm_modules *pkm_mod, *tmp;
    int ret = FAILED;

    if (lge_get_pkm_status() == LGE_PKM_COMPLETE && is_pkm_enabled == 1) {
        if (name == NULL) {
            pr_err("[LGE_PKM] name is NULL\n");
            return FAILED;
        }
        list_for_each_entry_safe(pkm_mod, tmp, &pkm_module_list, list) {
            if (pkm_mod != NULL) {
                if (strncmp(pkm_mod->name, name, strlen(name)) == 0) {
                    pr_debug("[LGE_PKM] del_module: module_name= %s\n", pkm_mod->name);
                    if (lge_get_pkm_status() == LGE_PKM_COMPLETE) {
                        ret = lgpki_call(DEL_MODULE_BASELINE, pkm_mod);
                        if (ret != SUCCESS) {
                            pr_err("[LGE_PKM] lgpki_call failed, ret(%d)\n", ret);
                            return FAILED;
                        }
                    }
                    list_del(&pkm_mod->list);
                    kfree(pkm_mod);
                }
            }
        }
    }
    return SUCCESS;
}

static ssize_t lge_show_pkm_command (struct device *dev,
        struct device_attribute *attr, char *buf)
{
    pr_debug("[LGE_PKM] pkm status(%d)\n", is_pkm_enabled);
    return sprintf(buf, "%d\n", is_pkm_enabled);
}

static ssize_t lge_store_pkm_command (struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int32_t cmd = 0;
    int32_t ret = 0;

    if (lge_get_boot_mode() != LGE_BOOT_MODE_NORMAL || lge_get_laf_mode() == LGE_LAF_MODE_LAF) {
        pr_err("[LGE_PKM] unsupported mode.\n");
        return count;
    }

    sscanf(buf, "%d", &cmd);
    pr_debug("[LGE_PKM] cmd(%d) \n", cmd);
    if (cmd == SET_PKM_BASELINE && pkm_flag == 0 && lge_get_pkm_status() != LGE_PKM_COMPLETE) {
        ret = lgpki_call(cmd, NULL);
        if (ret != SUCCESS) {
            pr_err("[LGE_PKM] lgpki_call(%d) failed(%d)\n", SET_PKM_BASELINE, ret);
            return count;
        }
        if (is_pkm_enabled != 1) {
            if (!device_remove_file_self(dev, attr)) {
                pr_err("[LGE_PKM] remove file failed");
            }
        }
    } else if (cmd == SET_MOD_BASELINE && is_pkm_enabled == 1 && lge_get_pkm_status() != LGE_PKM_COMPLETE) {
        ret = lgpki_call(cmd, NULL);
        if (ret != SUCCESS) {
            pr_err("[LGE_PKM] lgpki_call(%d) failed(%d)\n", SET_MOD_BASELINE, ret);
            return count;
        }
        if (is_pkm_enabled == 1) {
            if (!device_remove_file_self(dev, attr)) {
                pr_err("[LGE_PKM] remove file failed");
            }
        }
    }

    return count;
}

static DEVICE_ATTR(pkm_command, 0600, lge_show_pkm_command, lge_store_pkm_command);

static struct attribute *lge_pkm_attrs[] = {
    &dev_attr_pkm_command.attr,
    NULL
};

static const struct attribute_group lge_pkm_files = {
    .attrs  = lge_pkm_attrs,
};

static int __init lge_pkm_probe(struct platform_device *pdev)
{
    int ret = 0;

    pr_info("[LGE_PKM] lge_pkm_probe start\n");
    pkm_class = class_create(THIS_MODULE, PKM_NAME);
    if (IS_ERR(pkm_class)) {
        pr_err("[LGE_PKM] class_create() failed\n");
        ret = FAILED;
        goto exit;
    }

    pkm_dev = device_create(pkm_class, NULL, MKDEV(pkm_major, 0), NULL, "pkm_ctrl");
    if (IS_ERR(pkm_dev)) {
        pr_err("[LGE_PKM] device_create() failed\n");
        ret = PTR_ERR(pkm_dev);
        goto exit;
    }

    ret = device_create_file(pkm_dev, &dev_attr_pkm_command);
    if (ret < 0) {
        pr_err("[LGE_PKM] device create file fail\n");
        goto exit;
    }

    pr_info("[LGE_PKM] lge_pkm_probe done\n");
    return SUCCESS;
exit:
    pr_err("[LGE_PKM] probe fail - %d\n", ret);
    return ret;
}

static int lge_pkm_remove(struct platform_device *pdev)
{
    pr_info("[LGE_PKM] lge_pkm_remove \n");
    return SUCCESS;
}

static struct of_device_id pkm_match_table[] = {
    { .compatible = "lge-pkm-drv",},
    {},
};

static struct platform_driver lge_pkm_driver __refdata = {
    .probe = lge_pkm_probe,
    .remove = lge_pkm_remove,
    .driver = {
        .name = "lge_pkm_drv",
        .owner = THIS_MODULE,
        .of_match_table = pkm_match_table,
    },
};

static int __init lge_pkm_init(void)
{
    int ret = 0;

    pr_info("[LGE_PKM] lge_pkm_init \n");
    ret = platform_driver_register(&lge_pkm_driver);
    if (ret < 0)
        pr_err("[LGE_PKM] platform_driver_register() err=%d\n", ret);
    else
        lge_set_pkm_status(LGE_PKM_RUNNING);
    return ret;
}

static void __exit lge_pkm_exit(void)
{
    pr_info("[LGE_PKM] lge_pkm_exit \n");
    platform_driver_unregister(&lge_pkm_driver);
    lge_set_pkm_status(LGE_PKM_NONE);
}

module_init(lge_pkm_init);
module_exit(lge_pkm_exit);

MODULE_DESCRIPTION("LGE PKM kernel driver for Kernel Monitoring");
MODULE_AUTHOR("lge-b2b-dev <lge-b2b-dev@lge.com>");
MODULE_LICENSE("GPL");
