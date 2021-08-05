/*
 *  cxd22xx-i2c.c - cxd22xx NFC i2c driver
 *
 * Copyright (C) 2013 Sony Corporation.
 * Copyright (C) 2012 Broadcom Corporation.
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#define LGE_NFC_FIX

#ifdef LGE_NFC_FIX
#include <soc/qcom/lge/board_lge.h>
#endif

#ifdef LGE_NFC_FIX
#include <linux/nfc/cxd22xx.h>
#else
#include "cxd22xx.h"
#endif

/* do not change below */
#define MAX_BUFFER_SIZE 780

/* Read data */
#define PACKET_HEADER_SIZE_NCI (3)
#define MAX_PACKET_SIZE (PACKET_HEADER_SIZE_NCI + 255)

/* RESET */
#define RESET_ASSERT_MS (1)
/* Maximum open count */
#ifndef CXDNFC_MAX_OPEN_DEVICE
#define CXDNFC_MAX_OPEN_DEVICE 7
#endif
#define MAX_WAKECTRL_DEVICE (CXDNFC_MAX_OPEN_DEVICE + 1)

/* get status */
enum {
  CXDNFC_STAT_USERS_STATE=0,
};

struct cxd22xx_i2c_phy {
    unsigned int nfc_int;  /* interrupt */
    unsigned int nfc_rst;  /* reset */
    unsigned int nfc_wake; /* wake */
    unsigned int nfc_en;   /* voltage enable */
#ifdef LGE_NFC_FIX
    unsigned int nfc_hvdd;
#ifdef CONFIG_NFC_CXD22XX_SERVICE_MODE
    unsigned int nfc_tst1;
#endif
#endif
};

struct cxd22xx_platform_data {
    unsigned int irq_gpio;
    unsigned int en_gpio;
    unsigned int wake_gpio;
    unsigned int rst_gpio;
#ifdef LGE_NFC_FIX
    unsigned int hvdd_gpio;
#ifdef CONFIG_NFC_CXD22XX_SERVICE_MODE
    unsigned int tst1_gpio;
#endif
#endif
};

struct cxd22xx_dev {
    wait_queue_head_t read_wq;
    struct mutex rw_mutex;
    struct i2c_client *client;
    struct miscdevice cxd22xx_device;
    struct cxd22xx_i2c_phy *phy;
    bool irq_enabled;
    struct mutex lock;
    spinlock_t irq_enabled_lock;
    unsigned int users;
    unsigned int count_irq;
    /* Driver message queue */
    struct workqueue_struct *wqueue;
    struct work_struct qmsg;
    /* read buffer */
    size_t kbuflen;
    void *kbuf;
    /* wake state ctrl */
    int8_t wake_state[MAX_WAKECTRL_DEVICE];
    /* NFC state */
    int state;
    /* exclusive read/write by file device */
    int excl_fd;
};

struct cxd22xx_fp {
    struct cxd22xx_dev *cxd22xx_dev;
    int fd;
};

#if defined(CONFIG_NFC_CXD22XX_RST)
static void cxd22xx_workqueue(struct work_struct *work)
{
    struct cxd22xx_dev *cxd22xx_dev =
        container_of(work, struct cxd22xx_dev, qmsg);
    unsigned long flags;

    dev_info(&cxd22xx_dev->client->dev, "%s, xrst assert\n", __func__);
    spin_lock_irqsave(&cxd22xx_dev->irq_enabled_lock, flags);
    gpio_set_value(cxd22xx_dev->phy->nfc_rst, CXDNFC_RST_ACTIVE);
    cxd22xx_dev->count_irq = 0; /* clear irq */
    spin_unlock_irqrestore(&cxd22xx_dev->irq_enabled_lock, flags);
    cxd22xx_dev->state = CXDNFC_STAT_STATE_NFCRST;
    msleep(RESET_ASSERT_MS);
    dev_info(&cxd22xx_dev->client->dev, "%s, xrst deassert\n", __func__);
    gpio_set_value(cxd22xx_dev->phy->nfc_rst, ~CXDNFC_RST_ACTIVE & 0x1);
}

static int __init init_wqueue(struct cxd22xx_dev *cxd22xx_dev)
{
    INIT_WORK(&cxd22xx_dev->qmsg, cxd22xx_workqueue);
    cxd22xx_dev->wqueue = create_workqueue("cxd22xx-i2c_wrokq");
    if (cxd22xx_dev->wqueue == NULL)
        return -EBUSY;
    return 0;
}
#endif /* CONFIG_NFC_CXD22XX_RST */

static void cxd22xx_init_stat(struct cxd22xx_dev *cxd22xx_dev)
{
    cxd22xx_dev->count_irq = 0;
}

static void cxd22xx_disable_irq(struct cxd22xx_dev *cxd22xx_dev)
{
    unsigned long flags;
    spin_lock_irqsave(&cxd22xx_dev->irq_enabled_lock, flags);
    if (cxd22xx_dev->irq_enabled) {
        disable_irq_nosync(cxd22xx_dev->client->irq);
        cxd22xx_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore(&cxd22xx_dev->irq_enabled_lock, flags);
}

static void cxd22xx_enable_irq(struct cxd22xx_dev *cxd22xx_dev)
{
    unsigned long flags;
    spin_lock_irqsave(&cxd22xx_dev->irq_enabled_lock, flags);
    if (!cxd22xx_dev->irq_enabled) {
        cxd22xx_dev->irq_enabled = true;
        enable_irq(cxd22xx_dev->client->irq);
    }
    spin_unlock_irqrestore(&cxd22xx_dev->irq_enabled_lock, flags);
}

static long cxd22xx_power_gpio(struct cxd22xx_dev *cxd22xx_dev,
                   unsigned long state)
{
    long r = -EINVAL;
#if defined(CONFIG_NFC_CXD22XX_VEN)
    if (state <= 1) {
        gpio_set_value(cxd22xx_dev->en_gpio,
                   state ? CXDNFC_POWER_ACTIVE
                     : ~CXDNFC_POWER_ACTIVE & 0x1);
        r = 0;
    }
#endif
    return r;
}

static long cxd22xx_wake_gpio(struct cxd22xx_dev *cxd22xx_dev, int fd,
                  unsigned long state)
{
    int i;
    int en = 0;
    if (fd > MAX_WAKECTRL_DEVICE || !cxd22xx_dev) {
        return -ENODEV;
    }
    if (fd < 0 || state > 1) {
        return -EINVAL;
    }
    mutex_lock(&cxd22xx_dev->rw_mutex);
    cxd22xx_dev->wake_state[fd] = state ? 1 : 0;
    for (i = 0; i < MAX_WAKECTRL_DEVICE; i++) {
        en += cxd22xx_dev->wake_state[i] > 0 ? 1 : 0;
    }
    if (en) {
        gpio_set_value(cxd22xx_dev->phy->nfc_wake, CXDNFC_WAKE_ACTIVE);
    } else {
        gpio_set_value(cxd22xx_dev->phy->nfc_wake,
                   ~CXDNFC_WAKE_ACTIVE & 0x1);
    }
    mutex_unlock(&cxd22xx_dev->rw_mutex);
    dev_dbg(&cxd22xx_dev->client->dev, "%s en=%d\n", __func__, en);
    return 0;
}

long cxd22xx_wake(struct cxd22xx_dev *cxd22xx_dev, unsigned long enable_wake)
{
    return cxd22xx_wake_gpio(cxd22xx_dev, 0, enable_wake);
}
EXPORT_SYMBOL_GPL(cxd22xx_wake);

long cxd22xx_get_status(struct cxd22xx_dev *cxd22xx_dev, unsigned long entry)
{
  long r = -EINVAL;
    if (!cxd22xx_dev)
        return -ENODEV;
  switch(entry) {
  case CXDNFC_STAT_USERS_STATE:
    r = (cxd22xx_dev->state
         | (cxd22xx_dev->users << CXDNFC_STAT_USERS_SHIFT));
    break;
  default:
    ;
  }
  return r;
}
EXPORT_SYMBOL_GPL(cxd22xx_get_status);

/* exclusive read/write request */
int cxd22xx_excl_rw_req(struct cxd22xx_dev *cxd22xx_dev, int fd,
            unsigned long state)
{
    int r = 0;

    mutex_lock(&cxd22xx_dev->lock);
    if (state == 1) {
        if (cxd22xx_dev->excl_fd == 0) {
            /* requested fd */
            cxd22xx_dev->excl_fd = fd;
        } else {
            r = -EBUSY;
        }
    } else if (state == 0 && cxd22xx_dev->excl_fd == fd) {
        /* release */
        cxd22xx_dev->excl_fd = 0;
    } else {
        r = -EINVAL;
    }
    mutex_unlock(&cxd22xx_dev->lock);
    return r;
}

static irqreturn_t cxd22xx_dev_irq_handler(int irq, void *dev_id)
{
    struct cxd22xx_dev *cxd22xx_dev = dev_id;
    unsigned long flags;

    spin_lock_irqsave(&cxd22xx_dev->irq_enabled_lock, flags);
    cxd22xx_dev->count_irq++;
    spin_unlock_irqrestore(&cxd22xx_dev->irq_enabled_lock, flags);
    wake_up(&cxd22xx_dev->read_wq);

    return IRQ_HANDLED;
}

static unsigned int cxd22xx_dev_poll(struct file *filp, poll_table *wait)
{
    struct cxd22xx_fp *cxd22xx_fp = filp->private_data;
    struct cxd22xx_dev *cxd22xx_dev = cxd22xx_fp->cxd22xx_dev;

    unsigned int mask = 0;
    unsigned long flags;

    if (cxd22xx_dev->excl_fd && cxd22xx_dev->excl_fd != cxd22xx_fp->fd) {
        msleep(10); /* during exclusive enabled */
        return 0;
    }

    poll_wait(filp, &cxd22xx_dev->read_wq, wait);

    spin_lock_irqsave(&cxd22xx_dev->irq_enabled_lock, flags);
    if (cxd22xx_dev->excl_fd && cxd22xx_dev->excl_fd != cxd22xx_fp->fd) {
        ; /* do not update mask */
    } else if (cxd22xx_dev->count_irq > 0) {
        cxd22xx_dev->count_irq--;
        mask |= POLLIN | POLLRDNORM;
    }
    spin_unlock_irqrestore(&cxd22xx_dev->irq_enabled_lock, flags);

    return mask;
}

static ssize_t cxd22xx_dev_read(struct file *filp, char __user *buf,
                size_t count, loff_t *offset)
{
    struct cxd22xx_fp *cxd22xx_fp = filp->private_data;
    struct cxd22xx_dev *cxd22xx_dev = cxd22xx_fp->cxd22xx_dev;

    unsigned char *tmp = NULL;
    int total, len, ret;

    total = 0;
    len = 0;

    if (!cxd22xx_dev) {
        return -ENODEV;
    }

    if (count > cxd22xx_dev->kbuflen)
        count = cxd22xx_dev->kbuflen;

    if (cxd22xx_dev->excl_fd && cxd22xx_dev->excl_fd != cxd22xx_fp->fd) {
        /* during exclusive enable, reject read */
        return 0;
    }
    mutex_lock(&cxd22xx_dev->rw_mutex);

    tmp = (unsigned char *)cxd22xx_dev->kbuf;
    memset(tmp, 0x00, count);
    ret = i2c_master_recv(cxd22xx_dev->client, tmp, 3);
    if (ret == 3 && (tmp[0] != 0xff)) {
        total = ret;

        len = tmp[PACKET_HEADER_SIZE_NCI - 1];

        /** make sure full packet fits in the buffer
         **/
        if (len > 0 && (len + total) <= count) {
            /** read the remainder of the packet.
             **/
            ret = i2c_master_recv(cxd22xx_dev->client, tmp + total,
                          len);
            if (ret == len)
                total += len;
        }
        if (tmp[0] == 0x40 && tmp[1] == 0x00) {
            /* CORE_RESET_RSP */
            cxd22xx_dev->state = CXDNFC_STAT_STATE_NFCRST;
        } else if (tmp[0] == 0x40 && tmp[1] == 0x01 && tmp[2] > 1) {
            /* CORE_INIT_RSP */
            cxd22xx_dev->state = CXDNFC_STAT_STATE_NFCINIT;
        }
        dev_dbg(&cxd22xx_dev->client->dev, "%02x:%02x:%02x state=%d",
            tmp[0], tmp[1], tmp[2], cxd22xx_dev->state);
    }

    mutex_unlock(&cxd22xx_dev->rw_mutex);

    if (total > count || copy_to_user(buf, tmp, total)) {
        dev_err(&cxd22xx_dev->client->dev,
            "failed to copy to user space, total = %d\n", total);
        total = -EFAULT;
    }
    dev_dbg(&cxd22xx_dev->client->dev, "%s, (%d)\n", __func__, (int)total);
    return total;
}

static ssize_t cxd22xx_dev_write(struct file *filp, const char __user *buf,
                 size_t count, loff_t *offset)
{
    struct cxd22xx_fp *cxd22xx_fp = filp->private_data;
    struct cxd22xx_dev *cxd22xx_dev = cxd22xx_fp->cxd22xx_dev;

    char *tmp = NULL;
    int r = 0;

    dev_dbg(&cxd22xx_dev->client->dev, "%s, (%d)\n", __func__, (int)count);

    if (!cxd22xx_dev) {
        return -ENODEV;
    }

    if (count > MAX_BUFFER_SIZE) {
        dev_err(&cxd22xx_dev->client->dev, "out of memory\n");
        return -ENOMEM;
    }
    if (cxd22xx_dev->excl_fd && cxd22xx_dev->excl_fd != cxd22xx_fp->fd) {
        /* during exclusive enable, reject write */
        return 0;
    }
    tmp = memdup_user(buf, count);
    if (IS_ERR(tmp)) {
        dev_err(&cxd22xx_dev->client->dev,
            "failed to copy from user space\n");
        return PTR_ERR(tmp);
    }

    mutex_lock(&cxd22xx_dev->rw_mutex);
    /* Write data */
    r = i2c_master_send(cxd22xx_dev->client, tmp, count);
    if (r != count) {
        dev_err(&cxd22xx_dev->client->dev, "failed to write %d\n", r);
        r = -EIO;
    }
    mutex_unlock(&cxd22xx_dev->rw_mutex);
    kfree(tmp);
    return (ssize_t)r;
}

static int cxd22xx_dev_open(struct inode *inode, struct file *filp)
{
    int ret = 0;
    int call_enable = 0;
    int i = -1;
    struct cxd22xx_dev *cxd22xx_dev = container_of(
        filp->private_data, struct cxd22xx_dev, cxd22xx_device);
    struct cxd22xx_fp *cxd22xx_fp;

    mutex_lock(&cxd22xx_dev->lock);
    if (!cxd22xx_dev->users) {
        cxd22xx_init_stat(cxd22xx_dev);
        call_enable = 1;
    }
    cxd22xx_dev->users++;
    if (cxd22xx_dev->users >= MAX_WAKECTRL_DEVICE) {
        dev_err(&cxd22xx_dev->client->dev,
            "too many open files  %d > MAX\n", cxd22xx_dev->users);
        ret = -EMFILE;
        cxd22xx_dev->users--;
    } else {
        cxd22xx_fp = devm_kzalloc(&cxd22xx_dev->client->dev,
                      MAX_BUFFER_SIZE, GFP_KERNEL);
        for (i = 1; i < MAX_WAKECTRL_DEVICE; i++) {
            /* search unused entry */
            if (cxd22xx_fp && cxd22xx_dev->wake_state[i] < 0) {
                cxd22xx_dev->wake_state[i] = 0; /*disable*/
                break;
            }
        }
        if (i < MAX_WAKECTRL_DEVICE) {
            /* found unused entry */
            cxd22xx_fp->fd = i;
            cxd22xx_fp->cxd22xx_dev = cxd22xx_dev;
            filp->private_data = cxd22xx_fp;
        } else if (cxd22xx_fp) {
            devm_kfree(&cxd22xx_dev->client->dev, cxd22xx_fp);
            ret = -EMFILE;
            cxd22xx_dev->users--;
        }
    }
    if (call_enable)
        cxd22xx_dev->state = CXDNFC_STAT_STATE_OPEND;
    mutex_unlock(&cxd22xx_dev->lock);
    if (call_enable)
        cxd22xx_enable_irq(cxd22xx_dev);

    if (ret) {
        return ret;
    }
    dev_info(&cxd22xx_dev->client->dev, "open(%d) %d,%d users=%d\n", i,
         imajor(inode), iminor(inode), cxd22xx_dev->users);

    return ret;
}

static int cxd22xx_dev_release(struct inode *inode, struct file *filp)
{
    int ret = 0;
    int call_disable = 0;
    int fd = -1;
    struct cxd22xx_fp *cxd22xx_fp = filp->private_data;
    struct cxd22xx_dev *cxd22xx_dev = cxd22xx_fp->cxd22xx_dev;

    mutex_lock(&cxd22xx_dev->lock);
    cxd22xx_dev->users--;
    if (!cxd22xx_dev->users) {
        call_disable = 1;
        /* disable */
        cxd22xx_dev->excl_fd = 0;
    }
    if (cxd22xx_fp && cxd22xx_fp->fd > 0
        && cxd22xx_fp->fd < MAX_WAKECTRL_DEVICE) {
        cxd22xx_wake_gpio(cxd22xx_dev, cxd22xx_fp->fd, 0);
        cxd22xx_dev->wake_state[cxd22xx_fp->fd] = -1;
        if (cxd22xx_fp->fd == cxd22xx_dev->excl_fd) {
            /* disable */
            cxd22xx_dev->excl_fd = 0;
        }
        fd = cxd22xx_fp->fd;
        devm_kfree(&cxd22xx_dev->client->dev, cxd22xx_fp);
    }
    if (call_disable)
        cxd22xx_dev->state = CXDNFC_STAT_STATE_CLOSED;
    mutex_unlock(&cxd22xx_dev->lock);
    if (call_disable)
        cxd22xx_disable_irq(cxd22xx_dev);

    dev_info(&cxd22xx_dev->client->dev, "release(%d) %d,%d users=%d\n", fd,
         imajor(inode), iminor(inode), cxd22xx_dev->users);

    return ret;
}

static long cxd22xx_dev_unlocked_ioctl(struct file *filp, unsigned int cmd,
                       unsigned long arg)
{
    struct cxd22xx_fp *cxd22xx_fp = filp->private_data;
    struct cxd22xx_dev *cxd22xx_dev = cxd22xx_fp->cxd22xx_dev;
    long r = -EINVAL;

    switch (cmd) {
    case CXDNFC_RST_CTL:
#if defined(CONFIG_NFC_CXD22XX_RST)
        dev_info(&cxd22xx_dev->client->dev, "%s, rst arg=%d\n",
             __func__, (int)arg);
        r = queue_work(cxd22xx_dev->wqueue, &cxd22xx_dev->qmsg) ? 0 : 1;
#endif
        break;
    case CXDNFC_POWER_CTL:
        r = cxd22xx_power_gpio(cxd22xx_dev, arg);
        break;
    case CXDNFC_WAKE_CTL:
        r = cxd22xx_wake_gpio(cxd22xx_dev, cxd22xx_fp->fd, arg);
        break;
    case CXDNFC_GET_STAT:
        r = cxd22xx_get_status(cxd22xx_dev, arg);
        break;
    case CXDNFC_EXCLRW_REQ:
        dev_info(&cxd22xx_dev->client->dev, "%s, excl_rw arg=%d\n",
             __func__, (int)arg);
        r = (long)cxd22xx_excl_rw_req(cxd22xx_dev, cxd22xx_fp->fd, arg);
        break;
#ifdef LGE_NFC_FIX
    case CXDNFC_BOOTMODE_CTL:
        return lge_get_boot_mode();
#ifdef CONFIG_NFC_CXD22XX_SERVICE_MODE
    case CXDNFC_SERVICEMODE_CTL:
        dev_info(&cxd22xx_dev->client->dev, "%s, CXDNFC_SERVICEMODE_CTL\n", __func__);
        gpio_set_value(cxd22xx_dev->phy->nfc_tst1, 1);
        msleep(10);
        gpio_set_value(cxd22xx_dev->phy->nfc_wake, 1);
        gpio_set_value(cxd22xx_dev->phy->nfc_rst, 1);
        msleep(1);
        gpio_set_value(cxd22xx_dev->phy->nfc_rst, 0);
        msleep(10);
        gpio_set_value(cxd22xx_dev->phy->nfc_tst1, 0);
        msleep(4000);
        r = 0;
        break;
#endif
#endif
    default:
        dev_err(&cxd22xx_dev->client->dev,
            "%s, unknown cmd (%x, %lx)\n", __func__, cmd, arg);
    }

    return r;
}

static const struct file_operations cxd22xx_dev_fops = {
    .owner = THIS_MODULE,
    .llseek = no_llseek,
    .poll = cxd22xx_dev_poll,
    .read = cxd22xx_dev_read,
    .write = cxd22xx_dev_write,
    .open = cxd22xx_dev_open,
    .release = cxd22xx_dev_release,
    .unlocked_ioctl = cxd22xx_dev_unlocked_ioctl,
    .compat_ioctl = cxd22xx_dev_unlocked_ioctl,
};

static int cxd22xx_i2c_of_request_resources(struct i2c_client *client,
                        struct cxd22xx_i2c_phy *phy)
{
#if defined(CONFIG_OF)
    struct device_node *pp;
    int gpio;
    int r;
    int en_gpio_ok = 0;
    int rst_gpio_ok = 0;
    int wake_gpio_ok = 0;

#ifdef LGE_NFC_FIX
    int irq_gpio_ok = 0;
    int hvdd_gpio_ok = 0;
    int hvdd_gpio_value = 0;
#ifdef CONFIG_NFC_CXD22XX_SERVICE_MODE
    int tst1_gpio_ok = 0;
#endif
#endif
    pp = client->dev.of_node;
    if (!pp)
        return -ENODEV;

#if defined(CONFIG_NFC_CXD22XX_VEN)
    /* Get Nfc_en from device tree */
    gpio = of_get_named_gpio(pp, "sony,nfc_en", 0);
    if (gpio < 0) {
        dev_err(&client->dev,
            "Failed to retrieve sony,nfc_en from device tree\n");
        r = gpio;
        goto err_exit;
    }

    /* GPIO request and configuration */
    r = devm_gpio_request_one(&client->dev, gpio, GPIOF_OUT_INIT_LOW,
                  "nfc_en");
    if (r) {
        dev_err(&client->dev, "Failed to request enable pin\n");
        r = -ENODEV;
        goto err_exit;
    }
    phy->nfc_en = gpio;
    en_gpio_ok = 1;
#endif

#if defined(CONFIG_NFC_CXD22XX_RST)
    /* Get Nfc_rst from device tree */
#ifdef LGE_NFC_FIX
    gpio = of_get_named_gpio(pp, "sony,gpio_xrst", 0);
#else
    gpio = of_get_named_gpio(pp, "sony,nfc_rst", 0);
#endif
    if (gpio < 0) {
        dev_err(&client->dev,
            "Failed to retrieve sony,nfc_rst from device tree\n");
        r = gpio;
        goto err_exit;
    }

    /* GPIO request and configuration */
    r = devm_gpio_request_one(&client->dev, gpio, ~CXDNFC_RST_ACTIVE & 0x1,
                  "nfc_reset");
    if (r) {
        dev_err(&client->dev, "Failed to request reset pin\n");
        r = -ENODEV;
    }
    phy->nfc_rst = gpio;
    rst_gpio_ok = 1;
#endif

    /* Get Nfc_wake from device tree */
#ifdef LGE_NFC_FIX
    gpio = of_get_named_gpio(pp, "sony,gpio_mode", 0);
#else
    gpio = of_get_named_gpio(pp, "sony,nfc_wake", 0);
#endif
    if (gpio < 0) {
        dev_err(&client->dev,
            "Failed to retrieve sony,nfc_wake from device tree\n");
        r = gpio;
        goto err_exit;
    }
    phy->nfc_wake = gpio;

    /* GPIO request and configuration */
    r = devm_gpio_request_one(&client->dev, gpio, GPIOF_OUT_INIT_LOW,
                  "nfc_wake");
    if (r) {
        dev_err(&client->dev, "Failed to request wake pin\n");
        r = -ENODEV;
        goto err_exit;
    }
    wake_gpio_ok = 1;

#ifdef LGE_NFC_FIX
    gpio = of_get_named_gpio(pp, "sony,gpio_irq", 0);
    if (gpio < 0) {
        dev_err(&client->dev,
            "Failed to retrieve sony,nfc_irq from device tree\n");
        r = gpio;
        goto err_exit;
    }

    r = devm_gpio_request_one(&client->dev, gpio, GPIOF_IN,
                  "nfc_int");
    if (r) {
        dev_err(&client->dev, "Failed to request int pin\n");
        r = -ENODEV;
    }
    phy->nfc_int = gpio;
    irq_gpio_ok = 1;

    gpio = of_get_named_gpio(pp, "sony,gpio_hvdd", 0);
    if (gpio < 0) {
        dev_err(&client->dev,
            "Failed to retrieve sony,nfc_hvdd from device tree\n");
        r = gpio;
        goto err_exit;
    }

    if(lge_get_boot_mode() != LGE_BOOT_MODE_CHARGERLOGO ) {
        hvdd_gpio_value = GPIOF_OUT_INIT_HIGH;
    } else {
        hvdd_gpio_value = GPIOF_OUT_INIT_LOW;
    }

    r = devm_gpio_request_one(&client->dev, gpio, hvdd_gpio_value,
                  "nfc_hvdd");
    if (r) {
        dev_err(&client->dev, "Failed to request hvdd pin\n");
        r = -ENODEV;
    }
    phy->nfc_hvdd = gpio;
    hvdd_gpio_ok = 1;
#ifdef CONFIG_NFC_CXD22XX_SERVICE_MODE
    gpio = of_get_named_gpio(pp, "sony,gpio_tst1", 0);
    if (gpio < 0) {
        dev_err(&client->dev,
            "Failed to retrieve sony,gpio_tst1 from device tree\n");
        r = gpio;
        goto err_exit;
    }

    /* GPIO request and configuration */
    r = devm_gpio_request_one(&client->dev, gpio, GPIOF_OUT_INIT_LOW,
                  "nfc_tst1");
    if (r) {
        dev_err(&client->dev, "Failed to request tst1 pin\n");
        r = -ENODEV;
        goto err_exit;
    }
    phy->nfc_tst1 = gpio;
    tst1_gpio_ok = 1;
#endif
#endif
    /* IRQ */
    r = irq_of_parse_and_map(pp, 0);
    if (r < 0) {
        dev_err(&client->dev, "Unable to get irq, error: %d\n", r);
        goto err_exit;
    }

    client->irq = r;
#ifdef LGE_NFC_FIX
    dev_info(&client->dev, "wake_gpio = %d\n",  phy->nfc_wake);
    dev_info(&client->dev, "wake_gpio_value = %d\n",  gpio_get_value(phy->nfc_wake));

    dev_info(&client->dev, "hvdd_gpio = %d\n",  phy->nfc_hvdd);
    dev_info(&client->dev, "hvdd_gpio_value = %d\n",  gpio_get_value(phy->nfc_hvdd));

    dev_info(&client->dev, "rst_gpio = %d\n",  phy->nfc_rst);
    dev_info(&client->dev, "xrst_gpio_value = %d\n",  gpio_get_value(phy->nfc_rst));

    dev_info(&client->dev, "irq_gpio = %d\n",  phy->nfc_int);
#ifdef CONFIG_NFC_CXD22XX_SERVICE_MODE
    dev_info(&client->dev, "tst1_gpio = %d\n",  phy->nfc_tst1);
    dev_info(&client->dev, "tst1_gpio_value = %d\n",  gpio_get_value(phy->nfc_tst1));
#endif
#endif
    return 0;

err_exit:
    if (en_gpio_ok)
        devm_gpio_free(&client->dev, phy->nfc_en);
    if (rst_gpio_ok)
        devm_gpio_free(&client->dev, phy->nfc_rst);
    if (wake_gpio_ok)
        devm_gpio_free(&client->dev, phy->nfc_wake);
#ifdef LGE_NFC_FIX
    if (irq_gpio_ok)
        devm_gpio_free(&client->dev, phy->nfc_int);
    if (hvdd_gpio_ok)
        devm_gpio_free(&client->dev, phy->nfc_hvdd);
#ifdef CONFIG_NFC_CXD22XX_SERVICE_MODE
    if (tst1_gpio_ok)
        devm_gpio_free(&client->dev, phy->nfc_tst1);
#endif
#endif
#else
    r = -ENODEV;
#endif
    return r;
}

static int cxd22xx_i2c_request_resources(struct i2c_client *client,
                     struct cxd22xx_i2c_phy *phy)
{
    struct cxd22xx_platform_data *pdata;
    int r;
    int irq;

    pdata = client->dev.platform_data;
    if (pdata == NULL) {
        dev_err(&client->dev, "No platform data\n");
        return -EINVAL;
    }

    /* store for later use */
    phy->nfc_rst = pdata->rst_gpio;
    phy->nfc_wake = pdata->wake_gpio;

    r = devm_gpio_request_one(&client->dev, phy->nfc_rst,
                  ~CXDNFC_RST_ACTIVE & 0x1, "nfc_reset");
    if (r) {
        dev_err(&client->dev, "%s : reset gpio_request failed\n",
            __FILE__);
        return -ENODEV;
    }

    r = devm_gpio_request_one(&client->dev, phy->nfc_wake,
                  GPIOF_OUT_INIT_LOW, "nfc_wake");
    if (r) {
        dev_err(&client->dev, "%s : wake gpio_request failed\n",
            __FILE__);
        return -ENODEV;
    }

    /* IRQ */
    irq = gpio_to_irq(pdata->irq_gpio);
    if (irq < 0) {
        dev_err(&client->dev,
            "Unable to get irq number for GPIO %d error %d\n",
            phy->nfc_int, irq);
        return -ENODEV;
    }
    client->irq = irq;

    return 0;
}

static int cxd22xx_probe(struct i2c_client *client,
             const struct i2c_device_id *id)
{
    struct cxd22xx_i2c_phy *phy;
    struct cxd22xx_platform_data *pdata = NULL;
    struct cxd22xx_dev *cxd22xx_dev;
    int r;
    int i;

    dev_dbg(&client->dev, "%s\n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "need I2C_FUNC_I2C\n");
        return -ENODEV;
    }

    phy = devm_kzalloc(&client->dev, sizeof(struct cxd22xx_i2c_phy),
               GFP_KERNEL);
    if (!phy) {
        dev_err(&client->dev,
            "Cannot allocate memory for cxd22xx i2c phy.\n");
        return -ENOMEM;
    }

    pdata = client->dev.platform_data;
    if (!pdata && client->dev.of_node) {
        r = cxd22xx_i2c_of_request_resources(client, phy);
        if (r) {
            dev_err(&client->dev, "No platform data\n");
            return r;
        }
    } else if (pdata) {
        r = cxd22xx_i2c_request_resources(client, phy);
        if (r) {
            dev_err(&client->dev,
                "Cannot get platform resources\n");
            return r;
        }
    } else {
        dev_err(&client->dev,
            "cxd22xx platform resources not available\n");
        return -ENODEV;
    }

    dev_dbg(&client->dev, "%s, nfc_rst(%d)\n", __func__, phy->nfc_rst);
    dev_dbg(&client->dev, "%s, nfc_en(%d)\n", __func__, phy->nfc_en);
    dev_dbg(&client->dev, "%s, nfc_wake(%d)\n", __func__, phy->nfc_wake);

    cxd22xx_dev =
        devm_kzalloc(&client->dev, sizeof(*cxd22xx_dev), GFP_KERNEL);
    if (!cxd22xx_dev) {
        dev_err(&client->dev,
            "failed to allocate memory for module data\n");
        r = -ENOMEM;
        goto err_exit;
    }
    cxd22xx_dev->kbuflen = MAX_BUFFER_SIZE;
    cxd22xx_dev->kbuf =
        devm_kzalloc(&client->dev, MAX_BUFFER_SIZE, GFP_KERNEL);
    if (!cxd22xx_dev->kbuf) {
        dev_err(&client->dev,
            "failed to allocate memory for read data\n");
        r = -ENOMEM;
        goto err_exit;
    }
    cxd22xx_dev->client = client;
    cxd22xx_dev->phy = phy;
    cxd22xx_dev->users = 0;
    i2c_set_clientdata(client, cxd22xx_dev);

    /* init mutex and queues */
    init_waitqueue_head(&cxd22xx_dev->read_wq);
    mutex_init(&cxd22xx_dev->rw_mutex);
    mutex_init(&cxd22xx_dev->lock);
    spin_lock_init(&cxd22xx_dev->irq_enabled_lock);
    for (i = 0; i < MAX_WAKECTRL_DEVICE; i++) {
        cxd22xx_dev->wake_state[i] = -1;
    }
#if defined(CONFIG_NFC_CXD22XX_RST)
    if (init_wqueue(cxd22xx_dev) != 0) {
        dev_err(&client->dev, "init workqueue failed\n");
        goto err_misc_register;
    }
#endif
    cxd22xx_dev->cxd22xx_device.minor = MISC_DYNAMIC_MINOR;
    cxd22xx_dev->cxd22xx_device.name = "cxd22xx-i2c";
    cxd22xx_dev->cxd22xx_device.fops = &cxd22xx_dev_fops;

    r = misc_register(&cxd22xx_dev->cxd22xx_device);
    if (r) {
        dev_err(&client->dev, "misc_register failed\n");
        goto err_misc_register;
    }

    /* request irq.  the irq is set whenever the chip has data available
     * for reading.  it is cleared when all data has been read.
     */
    dev_info(&client->dev, "requesting IRQ %d\n", client->irq);
    cxd22xx_dev->irq_enabled = true;
    r = devm_request_irq(&client->dev, client->irq, cxd22xx_dev_irq_handler,
                 IRQF_TRIGGER_FALLING, client->name, cxd22xx_dev);
    if (r) {
        dev_err(&client->dev, "request_irq failed\n");
        goto err_request_irq_failed;
    }
    cxd22xx_disable_irq(cxd22xx_dev);
    cxd22xx_dev->state = CXDNFC_STAT_STATE_NONE;

    dev_info(&client->dev,
         "%s, probing cxd22xx driver exited successfully\n", __func__);
    return 0;

err_request_irq_failed:
    misc_deregister(&cxd22xx_dev->cxd22xx_device);
err_misc_register:
    mutex_destroy(&cxd22xx_dev->rw_mutex);
    mutex_destroy(&cxd22xx_dev->lock);

err_exit:

#if defined(CONFIG_NFC_CXD22XX_VEN)
    devm_gpio_free(&client->dev, phy->nfc_en);
#endif
#if defined(CONFIG_NFC_CXD22XX_RST)
    devm_gpio_free(&client->dev, phy->nfc_rst);
#endif
    devm_gpio_free(&client->dev, phy->nfc_wake);

    return r;
}

static int cxd22xx_remove(struct i2c_client *client)
{
    struct cxd22xx_dev *cxd22xx_dev = i2c_get_clientdata(client);
    struct cxd22xx_i2c_phy *phy = cxd22xx_dev->phy;

    misc_deregister(&cxd22xx_dev->cxd22xx_device);
    mutex_destroy(&cxd22xx_dev->rw_mutex);
    mutex_destroy(&cxd22xx_dev->lock);
    if (phy) {
        devm_gpio_free(&client->dev, phy->nfc_wake);

#if defined(CONFIG_NFC_CXD22XX_VEN)
        devm_gpio_free(&client->dev, phy->nfc_en);
#endif
#if defined(CONFIG_NFC_CXD22XX_RST)
        devm_gpio_free(&client->dev, phy->nfc_rst);
#endif
    }
    return 0;
}

#ifdef CONFIG_NFC_CXD22XX_PM
static int cxd22xx_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cxd22xx_dev *cxd22xx_dev = i2c_get_clientdata(client);

    if (cxd22xx_dev && cxd22xx_dev->users > 0
        && device_may_wakeup(&client->dev)) {
        enable_irq_wake(client->irq);
    }
    return 0;
}

static int cxd22xx_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);

    if (device_may_wakeup(&client->dev)) {
        disable_irq_wake(client->irq);
    }
    return 0;
}

static const struct dev_pm_ops cxd22xx_pm_ops = {
    .suspend = cxd22xx_suspend,
    .resume = cxd22xx_resume,
};
#endif

static const struct i2c_device_id cxd22xx_id[] = {{"cxd22xx-i2c", 0}, {}};
MODULE_DEVICE_TABLE(i2c, cxd22xx_id);

static struct i2c_driver cxd22xx_driver = {
    .id_table = cxd22xx_id,
    .probe = cxd22xx_probe,
    .remove = cxd22xx_remove,
    .driver =
        {
            .owner = THIS_MODULE,
            .name = "cxd22xx-i2c",
#ifdef CONFIG_NFC_CXD22XX_PM
            .pm = &cxd22xx_pm_ops,
#endif
        },
};

/*
 * module load/unload record keeping
 */
static int __init cxd22xx_dev_init(void)
{
    return i2c_add_driver(&cxd22xx_driver);
}
module_init(cxd22xx_dev_init);

static void __exit cxd22xx_dev_exit(void)
{
    i2c_del_driver(&cxd22xx_driver);
}
module_exit(cxd22xx_dev_exit);

MODULE_AUTHOR("Sony");
MODULE_DESCRIPTION("NFC cxd22xx driver");
MODULE_LICENSE("GPL");
