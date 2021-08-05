/*
 * Gadget Driver for Android LAF
 *
 * Copyright (C) 2012 LG Electronics, Inc.
 * Author: DH, kang <deunghyung.kang@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/usb.h>
#include <linux/usb_usual.h>
#include <linux/usb/ch9.h>
#include <linux/configfs.h>
#include <linux/usb/composite.h>
#include <linux/kref.h>

#include "configfs.h"

#define LAF_RX_BUFFER_INIT_SIZE    (524288)
#define LAF_BULK_BUFFER_SIZE       (524288)
#define MAX_INST_NAME_LEN          40

/* number of tx requests to allocate */
#define LAF_TX_REQ_MAX 4
#define RX_REQ_MAX 4

#define DRIVER_NAME "laf"

unsigned int laf_rx_req_len = LAF_RX_BUFFER_INIT_SIZE;
module_param(laf_rx_req_len, uint, S_IRUGO | S_IWUSR);

unsigned int laf_tx_req_len = LAF_BULK_BUFFER_SIZE;
module_param(laf_tx_req_len, uint, S_IRUGO | S_IWUSR);

unsigned int laf_tx_reqs = LAF_TX_REQ_MAX;
module_param(laf_tx_reqs, uint, S_IRUGO | S_IWUSR);

static const char laf_shortname[] = DRIVER_NAME;

struct laf_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	atomic_t online;
	atomic_t error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;

	struct kref kref;

	struct list_head tx_idle;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	struct usb_request *rx_req[RX_REQ_MAX];
	int rx_done;

};

static struct usb_interface_descriptor laf_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass     = USB_SUBCLASS_VENDOR_SPEC,
	.bInterfaceProtocol     = 0xFF,
};

static struct usb_endpoint_descriptor laf_superspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor laf_superspeed_in_comp_desc = {
	.bLength =		sizeof(laf_superspeed_in_comp_desc),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor laf_superspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor laf_superspeed_out_comp_desc = {
	.bLength =		sizeof(laf_superspeed_out_comp_desc),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor laf_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
	.bInterval		= 0,
};

static struct usb_endpoint_descriptor laf_highspeed_out_desc = {
	.bLength          =	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes     =	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   =	__constant_cpu_to_le16(512),
	.bInterval        =	0,
};

static struct usb_endpoint_descriptor laf_fullspeed_in_desc = {
	.bLength          =	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes     =	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   =	__constant_cpu_to_le16(64),
	.bInterval        =	0,
};

static struct usb_endpoint_descriptor laf_fullspeed_out_desc = {
	.bLength          =	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes     =	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   =	__constant_cpu_to_le16(64),
	.bInterval        =	0,
};

static struct usb_descriptor_header *fs_laf_descs[] = {
	(struct usb_descriptor_header *) &laf_interface_desc,
	(struct usb_descriptor_header *) &laf_fullspeed_in_desc,
	(struct usb_descriptor_header *) &laf_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_laf_descs[] = {
	(struct usb_descriptor_header *) &laf_interface_desc,
	(struct usb_descriptor_header *) &laf_highspeed_in_desc,
	(struct usb_descriptor_header *) &laf_highspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *ss_laf_descs[] = {
	(struct usb_descriptor_header *) &laf_interface_desc,
	(struct usb_descriptor_header *) &laf_superspeed_in_desc,
	(struct usb_descriptor_header *) &laf_superspeed_in_comp_desc,
	(struct usb_descriptor_header *) &laf_superspeed_out_desc,
	(struct usb_descriptor_header *) &laf_superspeed_out_comp_desc,
	NULL,
};

struct laf_instance {
	struct usb_function_instance func_inst;
	const char *name;
	struct laf_dev *dev;
};

/* temporary variable used between laf_open() and laf_gadget_bind() */
static struct laf_dev *_laf_dev;

static inline struct laf_dev *func_to_laf(struct usb_function *f)
{
	return container_of(f, struct laf_dev, function);
}

static void laf_dev_free(struct kref *kref)
{
	struct laf_dev *dev = container_of(kref, struct laf_dev, kref);

	kfree(dev);
	_laf_dev = NULL;
}

static struct usb_request *laf_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void laf_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int laf_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void laf_unlock(atomic_t *excl)
{
	if (atomic_dec_return(excl) < 0)
		atomic_inc(excl);
}

/* add a request to the tail of a list */
void laf_req_put(struct laf_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request
*laf_req_get(struct laf_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static void laf_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct laf_dev *dev = _laf_dev;

	if (req->status != 0)
		atomic_set(&dev->error, 1);

	laf_req_put(dev, &dev->tx_idle, req);

	wake_up(&dev->write_wq);
}

static void laf_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct laf_dev *dev = _laf_dev;

	dev->rx_done = 1;
	if (unlikely(req->status != 0 && req->status != -ECONNRESET))
		atomic_set(&dev->error, 1);

	wake_up(&dev->read_wq);
}

static int laf_create_bulk_endpoints(struct laf_dev *dev,
		struct usb_endpoint_descriptor *in_desc,
		struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	pr_debug("laf_create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		pr_debug("usb_ep_autoconfig for laf ep_in failed\n");
		return -ENODEV;
	}
	pr_debug("usb_ep_autoconfig for laf ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		pr_debug("usb_ep_autoconfig for laf ep_out failed\n");
		return -ENODEV;
	}
	pr_debug("usb_ep_autoconfig for laf ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

retry_tx_alloc:
	if (laf_tx_req_len > LAF_BULK_BUFFER_SIZE)
		laf_tx_reqs = 4;

	/* now allocate requests for our endpoints */
	for (i = 0; i < laf_tx_reqs; i++) {
		req = laf_request_new(dev->ep_in, laf_tx_req_len);
		if (!req) {
			if (laf_tx_req_len <= LAF_BULK_BUFFER_SIZE)
				goto fail;
			while ((req = laf_req_get(dev, &dev->tx_idle)))
				laf_request_free(req, dev->ep_in);
			laf_tx_req_len = LAF_BULK_BUFFER_SIZE;
			laf_tx_reqs = LAF_TX_REQ_MAX;
			goto retry_tx_alloc;
		}
		req->complete = laf_complete_in;
		laf_req_put(dev, &dev->tx_idle, req);
	}

	/*
	 * The RX buffer should be aligned to EP max packet for
	 * some controllers.  At bind time, we don't know the
	 * operational speed.  Hence assuming super speed max
	 * packet size.
	 */
	if (laf_rx_req_len % 1024)
		laf_rx_req_len = LAF_BULK_BUFFER_SIZE;
retry_rx_alloc:
	for (i = 0; i < RX_REQ_MAX; i++) {
		req = laf_request_new(dev->ep_out, laf_rx_req_len);
		if (!req) {
			if (laf_rx_req_len <= LAF_BULK_BUFFER_SIZE)
				goto fail;
			for (--i; i >= 0; i--)
				laf_request_free(dev->rx_req[i], dev->ep_out);
			laf_rx_req_len = LAF_BULK_BUFFER_SIZE;
			goto retry_rx_alloc;
		}
		req->complete = laf_complete_out;
		dev->rx_req[i] = req;
	}

	return 0;

fail:
	pr_err("laf_bind() could not allocate requests\n");
	return -1;
}

static ssize_t laf_read(struct file *fp, char __user *buf,
		size_t count, loff_t *pos)
{
	struct laf_dev *dev = fp->private_data;
	struct usb_request *req;
	ssize_t r = count, xfer;
	int ret = 0;

	pr_debug("laf_read(%zu)\n", count);

	if (unlikely(!_laf_dev))
		return -ENODEV;

	if (unlikely(count > LAF_BULK_BUFFER_SIZE))
		return -EINVAL;

	if (unlikely(laf_lock(&dev->read_excl)))
		return -EBUSY;

	/* we will block until we're online */
	while (unlikely(!(atomic_read(&dev->online)) ||
			unlikely(atomic_read(&dev->error)))) {
		pr_debug("laf_read: waiting for online state\n");
		ret = wait_event_interruptible(dev->read_wq,
					       (atomic_read(&dev->online) ||
						atomic_read(&dev->error)));
		if (unlikely(ret < 0)) {
			laf_unlock(&dev->read_excl);
			return ret;
		}
	}
	if (unlikely(atomic_read(&dev->error))) {
		r = -EIO;
		goto done;
	}

requeue_req:
	/* queue a request */
	req = dev->rx_req[0];
	req->length = count;
	dev->rx_done = 0;
	ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
	if (unlikely(ret < 0)) {
		pr_debug("laf_read: failed to queue req %p (%d)\n", req, ret);
		r = -EIO;
		atomic_set(&dev->error, 1);
		goto done;
	} else {
		pr_debug("rx %p queue\n", req);
	}

	/* wait for a request to complete */
	ret = wait_event_interruptible(dev->read_wq, dev->rx_done);
	if (unlikely(ret < 0)) {
		if (ret != -ERESTARTSYS)
			atomic_set(&dev->error, 1);
		r = ret;
		usb_ep_dequeue(dev->ep_out, req);
		goto done;
	}
	if (likely(!atomic_read(&dev->error))) {
		/* If we got a 0-len packet, throw it back and try again. */
		if (unlikely(req->actual == 0))
			goto requeue_req;

		pr_debug("rx %p %d\n", req, req->actual);
		xfer = (req->actual < count) ? req->actual : count;
		r = xfer;
		if (unlikely(copy_to_user(buf, req->buf, xfer)))
			r = -EFAULT;
	} else
		r = -EIO;

done:
	laf_unlock(&dev->read_excl);
	pr_debug("laf_read returning %zd\n", r);
	return r;
}

static ssize_t laf_write(struct file *fp, const char __user *buf,
		size_t count, loff_t *pos)
{
	struct laf_dev *dev = fp->private_data;
	struct usb_request *req = 0;
	ssize_t r = count;
	unsigned xfer;
	int ret;

	pr_debug("%s\n", __func__);
	if (!_laf_dev)
		return -ENODEV;
	pr_debug("laf_write(%zu)\n", count);

	if (laf_lock(&dev->write_excl))
		return -EBUSY;

	while (count > 0) {
		if (atomic_read(&dev->error)) {
			pr_debug("laf_write dev->error\n");
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(dev->write_wq,
			((req = laf_req_get(dev, &dev->tx_idle)) ||
			 atomic_read(&dev->error)));

		if (ret < 0) {
			r = ret;
			break;
		}

		if (req != 0) {
			if (count > LAF_BULK_BUFFER_SIZE)
				xfer = LAF_BULK_BUFFER_SIZE;
			else
				xfer = count;
			if (copy_from_user(req->buf, buf, xfer)) {
				r = -EFAULT;
				break;
			}

			req->length = xfer;
			ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
			if (ret < 0) {
				pr_debug("laf_write: xfer error %d\n", ret);
				atomic_set(&dev->error, 1);
				r = -EIO;
				break;
			}

			buf += xfer;
			count -= xfer;

			/* zero this so we don't try to free it on error exit */
			req = 0;
		}
	}

	if (req)
		laf_req_put(dev, &dev->tx_idle, req);

	laf_unlock(&dev->write_excl);
	pr_debug("laf_write returning %zd\n", r);
	return r;
}

static int laf_open(struct inode *ip, struct file *fp)
{
	struct laf_dev *dev = _laf_dev;

	pr_debug("laf_open\n");
	if (!dev)
		return -ENODEV;

	if (laf_lock(&dev->open_excl))
		return -EBUSY;

	fp->private_data = dev;

	/* clear the error latch */
	atomic_set(&_laf_dev->error, 0);

	kref_get(&dev->kref);

	return 0;
}

static int laf_release(struct inode *ip, struct file *fp)
{
	struct laf_dev *dev = _laf_dev;

	printk(KERN_INFO "laf_release\n");

	laf_unlock(&dev->open_excl);

	kref_put(&dev->kref, laf_dev_free);

	return 0;
}

/* file operations for LAF device /dev/android_laf */
static const struct file_operations laf_fops = {
	.owner = THIS_MODULE,
	.read = laf_read,
	.write = laf_write,
	.open = laf_open,
	.release = laf_release,
};

static struct miscdevice laf_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = laf_shortname,
	.fops = &laf_fops,
};

static int
laf_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct laf_dev	*dev = func_to_laf(f);
	int			id;
	int			ret;

	dev->cdev = cdev;
	pr_debug("laf_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	laf_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = laf_create_bulk_endpoints(dev, &laf_fullspeed_in_desc,
			&laf_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		laf_highspeed_in_desc.bEndpointAddress =
			laf_fullspeed_in_desc.bEndpointAddress;
		laf_highspeed_out_desc.bEndpointAddress =
			laf_fullspeed_out_desc.bEndpointAddress;
	}
	/* support super speed hardware */
	if (gadget_is_superspeed(c->cdev->gadget)) {
		laf_superspeed_in_desc.bEndpointAddress =
			laf_fullspeed_in_desc.bEndpointAddress;
		laf_superspeed_out_desc.bEndpointAddress =
			laf_fullspeed_out_desc.bEndpointAddress;
	}

	pr_debug("%s speed %s: IN/%s, OUT/%s\n",
		 gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
		 f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}

static void
laf_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct laf_dev *dev = func_to_laf(f);
	struct usb_request *req;
	int i;

	atomic_set(&dev->online, 0);
	atomic_set(&dev->error, 1);

	wake_up(&dev->read_wq);

	while ((req = laf_req_get(dev, &dev->tx_idle)))
		laf_request_free(req, dev->ep_in);
	for (i = 0; i < RX_REQ_MAX; i++)
		laf_request_free(dev->rx_req[i], dev->ep_out);
}

static int laf_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct laf_dev *dev = func_to_laf(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	pr_debug("laf_function_set_alt intf: %d alt: %d\n", intf, alt);

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_in);
	if (ret)
		return ret;

	ret = usb_ep_enable(dev->ep_in);
	if (ret)
		return ret;

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_out);
	if (ret) {
		usb_ep_disable(dev->ep_in);
		return ret;
	}

	ret = usb_ep_enable(dev->ep_out);
	if (ret) {
		usb_ep_disable(dev->ep_in);
		return ret;
	}

	atomic_set(&dev->online, 1);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	return 0;
}

static void laf_function_disable(struct usb_function *f)
{
	struct laf_dev *dev = func_to_laf(f);

	pr_debug("laf_function_disable\n");
	atomic_set(&dev->online, 0);
	atomic_set(&dev->error, 1);
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);

#ifdef VERBOSE_DEBUG
	pr_debug("%s disabled\n", dev->function.name);
#endif
}

static int __laf_setup(struct laf_instance *fi_laf)
{
	struct laf_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);

	if (fi_laf != NULL)
		fi_laf->dev = dev;

	if (!dev)
		return -ENOMEM;

	kref_init(&dev->kref);
	spin_lock_init(&dev->lock);
	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);
	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);
	INIT_LIST_HEAD(&dev->tx_idle);

	_laf_dev = dev;

	ret = misc_register(&laf_device);
	if (ret)
		goto err;

	return 0;

err:
	_laf_dev = NULL;
	kfree(dev);
	printk(KERN_ERR "laf gadget driver failed to initialize\n");
	return ret;
}

static int laf_setup_configfs(struct laf_instance *fi_laf)
{
	return __laf_setup(fi_laf);
}

static void laf_cleanup(void)
{
	struct laf_dev *dev = _laf_dev;

	if (!dev)
		return;

	misc_deregister(&laf_device);

	kref_put(&dev->kref, laf_dev_free);
}

static struct laf_instance *to_laf_instance(struct config_item *item)
{
	return container_of(to_config_group(item), struct laf_instance,
		func_inst.group);
}

static void laf_attr_release(struct config_item *item)
{
	struct laf_instance *fi_laf = to_laf_instance(item);
	usb_put_function_instance(&fi_laf->func_inst);
}

static struct configfs_item_operations laf_item_ops = {
	.release        = laf_attr_release,
};

static struct config_item_type laf_func_type = {
	.ct_item_ops    = &laf_item_ops,
	.ct_owner       = THIS_MODULE,
};

static struct laf_instance *to_fi_laf(struct usb_function_instance *fi)
{
	return container_of(fi, struct laf_instance, func_inst);
}

static int laf_set_inst_name(struct usb_function_instance *fi, const char *name)
{
	struct laf_instance *fi_laf;
	char *ptr;
	int name_len;

	name_len = strlen(name) + 1;
	if (name_len > MAX_INST_NAME_LEN)
		return -ENAMETOOLONG;

	ptr = kstrndup(name, name_len, GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	fi_laf = to_fi_laf(fi);
	fi_laf->name = ptr;

	return 0;
}

static void laf_free_inst(struct usb_function_instance *fi)
{
	struct laf_instance *fi_laf;

	fi_laf = to_fi_laf(fi);
	kfree(fi_laf->name);
	laf_cleanup();
	kfree(fi_laf);
}

static struct usb_function_instance *laf_alloc_inst(void)
{
	struct laf_instance *fi_laf;
	int ret = 0;

	fi_laf = kzalloc(sizeof(*fi_laf), GFP_KERNEL);
	if (!fi_laf)
		return ERR_PTR(-ENOMEM);
	fi_laf->func_inst.set_inst_name = laf_set_inst_name;
	fi_laf->func_inst.free_func_inst = laf_free_inst;

	ret = laf_setup_configfs(fi_laf);
	if (ret) {
		kfree(fi_laf);
		pr_err("Error setting LAF\n");
		return ERR_PTR(ret);
	}

	config_group_init_type_name(&fi_laf->func_inst.group,
					"", &laf_func_type);

	return  &fi_laf->func_inst;
}

static void laf_free(struct usb_function *f)
{
	/*NO-OP: no function specific resource allocation in laf_alloc*/
}

static struct usb_function *laf_alloc(struct usb_function_instance *fi)
{
	struct laf_instance *fi_laf = to_fi_laf(fi);
	struct laf_dev *dev;

	dev = fi_laf->dev;
	dev->function.name = DRIVER_NAME;
	dev->function.fs_descriptors = fs_laf_descs;
	dev->function.hs_descriptors = hs_laf_descs;
	dev->function.ss_descriptors = ss_laf_descs;
	dev->function.bind = laf_function_bind;
	dev->function.unbind = laf_function_unbind;
	dev->function.set_alt = laf_function_set_alt;
	dev->function.disable = laf_function_disable;
	dev->function.free_func = laf_free;

	return &dev->function;
}

DECLARE_USB_FUNCTION_INIT(laf, laf_alloc_inst, laf_alloc);
MODULE_LICENSE("GPL");
