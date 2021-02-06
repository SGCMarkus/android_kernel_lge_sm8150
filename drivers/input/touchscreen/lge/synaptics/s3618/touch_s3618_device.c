/*
 * Synaptics TCM touchscreen driver
 *
 * Copyright (C) 2017-2018 Synaptics Incorporated. All rights reserved.
 *
 * Copyright (C) 2017-2018 Scott Lin <scott.lin@tw.synaptics.com>
 * Copyright (C) 2018-2019 Ian Su <ian.su@tw.synaptics.com>
 * Copyright (C) 2018-2019 Joey Zhou <joey.zhou@synaptics.com>
 * Copyright (C) 2018-2019 Yuehao Qiu <yuehao.qiu@synaptics.com>
 * Copyright (C) 2018-2019 Aaron Chen <aaron.chen@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include "touch_s3618.h"

#define CHAR_DEVICE_NAME "tcm"

#define CONCURRENT true

#define DEVICE_IOC_MAGIC 's'
#define DEVICE_IOC_RESET _IO(DEVICE_IOC_MAGIC, 0) /* 0x00007300 */
#define DEVICE_IOC_IRQ _IOW(DEVICE_IOC_MAGIC, 1, int) /* 0x40047301 */
#define DEVICE_IOC_RAW _IOW(DEVICE_IOC_MAGIC, 2, int) /* 0x40047302 */
#define DEVICE_IOC_CONCURRENT _IOW(DEVICE_IOC_MAGIC, 3, int) /* 0x40047303 */

struct device_hcd {
	dev_t dev_num;
	bool raw_mode;
	bool concurrent;
	unsigned int ref_count;
	struct cdev char_dev;
	struct class *class;
	struct device *device;
	struct s3618_buffer out;
	struct s3618_buffer resp;
	struct s3618_buffer report;
	struct touch_core_data *ts;
	struct s3618_data *d;
};

DECLARE_COMPLETION(device_remove_complete);

static struct device_hcd *device_hcd;

static int rmidev_major_num;

static int device_raw_read(unsigned char *in_buf, unsigned int length)
{
	int retval = 0;
	unsigned char code = 0;
	unsigned int idx = 0;
	unsigned int offset = 0;
	unsigned int chunks = 0;
	unsigned int chunk_space = 0;
	unsigned int xfer_length = 0;
	unsigned int remaining_length = 0;
	struct touch_core_data *ts = device_hcd->ts;
	struct s3618_data*d = device_hcd->d;

	if (length < 2) {
		TOUCH_E("Invalid length information\n");
		return -EINVAL;
	}

	remaining_length = length - 2;

	if (d->chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = d->chunk_size - 2;

	chunks = ceil_div(remaining_length, chunk_space);
	chunks = (chunks == 0) ? 1 : chunks;
	offset = 0;

	mutex_lock(&d->temp.buf_mutex);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		if (xfer_length == 1) {
			in_buf[offset] = MESSAGE_PADDING;
			offset += xfer_length;
			remaining_length -= xfer_length;
			continue;
		}


		memset(d->temp.buf, 0x00, sizeof(d->temp.buf));
		d->temp.data_length = 0;

		retval = s3618_read(ts->dev, d->temp.buf, xfer_length + 2);
		if (retval < 0) {
			TOUCH_E("Failed to read from device\n");
			mutex_unlock(&d->temp.buf_mutex);
			return retval;
		}

		code = d->temp.buf[1];

		if (idx == 0) {
			memcpy(&in_buf[0], &d->temp.buf[0], xfer_length + 2);
		} else {
			if (code != STATUS_CONTINUED_READ) {
				TOUCH_E("Incorrect header code (0x%02x)\n", code);
				mutex_unlock(&d->temp.buf_mutex);
				return -EIO;
			}

			memcpy(&in_buf[offset], &d->temp.buf[2], xfer_length);
		}

		if (idx == 0)
			offset += (xfer_length + 2);
		else
			offset += xfer_length;

		remaining_length -= xfer_length;
	}

	mutex_unlock(&d->temp.buf_mutex);

	return 0;
}

static int device_raw_write(unsigned char command,
				unsigned char *data, unsigned int length)
{
	int retval = 0;
	unsigned int idx = 0;
	unsigned int chunks = 0;
	unsigned int chunk_space = 0;
	unsigned int xfer_length = 0;
	unsigned int remaining_length = 0;
	struct touch_core_data *ts = device_hcd->ts;
	struct s3618_data*d = device_hcd->d;

	remaining_length = length;

	if (d->chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = d->chunk_size - 1;

	chunks = ceil_div(remaining_length, chunk_space);
	chunks = chunks == 0 ? 1 : chunks;

	mutex_lock(&d->out.buf_mutex);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		memset(d->out.buf, 0x00, sizeof(d->out.buf));
		d->out.data_length = 0;

		if (idx == 0)
			d->out.buf[0] = command;
		else
			d->out.buf[0] = CMD_CONTINUE_WRITE;

		if (xfer_length) {
			memcpy(&d->out.buf[1], &data[idx * chunk_space], xfer_length);
		}

		retval = s3618_write(ts->dev, d->out.buf, xfer_length + 1);
		if (retval < 0) {
			TOUCH_E("Failed to write to device\n");
			mutex_unlock(&d->out.buf_mutex);
			return retval;
		}

		remaining_length -= xfer_length;
	}

	mutex_unlock(&d->out.buf_mutex);

	return 0;
}


static void device_capture_touch_report(unsigned int count)
{
	unsigned char id = 0;
	unsigned int idx = 0;
	unsigned int size = 0;
	unsigned char *data = 0;
	struct s3618_data*d = device_hcd->d;
	static bool report = false;
	static unsigned int offset = 0;
	static unsigned int remaining_size = 0;
	struct touch_core_data *ts = device_hcd->ts;

	if (count < 2)
		return;

	data = &device_hcd->resp.buf[0];

	if (data[0] != MESSAGE_MARKER)
		return;

	id = data[1];

	size = 0;

	mutex_lock(&device_hcd->report.buf_mutex);

	switch (id) {
	case REPORT_TOUCH:
		if (count >= 4) {
			remaining_size = le2_to_uint(&data[2]);
		} else {
			report = false;
			goto exit;
		}
		memset(&device_hcd->report.buf, 0x0, remaining_size);
		device_hcd->report.data_length = 0;
		idx = 4;
		size = count - idx;
		offset = 0;
		report = true;
		break;
	case STATUS_CONTINUED_READ:
		if (report == false)
			goto exit;
		if (count >= 2) {
			idx = 2;
			size = count - idx;
		}
		break;
	default:
		goto exit;
	}

	if (size) {
		size = MIN(size, remaining_size);
		memcpy(&device_hcd->report.buf[offset],
				&data[idx], size);
		offset += size;
		remaining_size -= size;
		device_hcd->report.data_length += size;
	}

	if (remaining_size)
		goto exit;

	mutex_lock(&d->report.buf_mutex);


//	d->report.buf = device_hcd->report.buf;
	memcpy(d->report.buf, device_hcd->report.buf, BUFFER_SIZE);
	d->report.data_length = device_hcd->report.data_length;

	d->report_touch(ts->dev);

	mutex_unlock(&d->report.buf_mutex);

	report = false;

exit:
	mutex_unlock(&device_hcd->report.buf_mutex);

	return;
}

static int device_capture_touch_report_config(unsigned int count)
{
	unsigned int size = 0;
	unsigned int buf_size = 0;
	u8 *data = NULL;
	struct s3618_data*d = device_hcd->d;

	if (device_hcd->raw_mode) {
		if (count < 3) {
			TOUCH_E("Invalid write data\n");
			return -EINVAL;
		}

		size = le2_to_uint(&device_hcd->out.buf[1]);

		if (count - 3 < size) {
			TOUCH_E("Incomplete write data\n");
			return -EINVAL;
		}

		if (!size)
			return 0;

		data = &device_hcd->out.buf[3];
		buf_size = device_hcd->out.data_length - 3;
	} else {
		size = count - 1;

		if (!size)
			return 0;

		data = &device_hcd->out.buf[1];
		buf_size = device_hcd->out.data_length - 1;
	}

	mutex_lock(&d->config.buf_mutex);

	memset(d->config.buf, 0x0, sizeof(d->config.buf));
	d->config.data_length = 0;

	memcpy(d->config.buf, data, size);

	d->config.data_length = size;

	mutex_unlock(&d->config.buf_mutex);

	return 0;
}

#ifdef HAVE_UNLOCKED_IOCTL
static long device_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int device_ioctl(struct inode *inp, struct file *filp, unsigned int cmd,
		unsigned long arg)
#endif
{
	int retval = 0;
	struct s3618_data*d = device_hcd->d;
	struct touch_core_data *ts = device_hcd->ts;

	TOUCH_TRACE();

	mutex_lock(&d->extif_mutex);

	retval = 0;

	switch (cmd) {
	case DEVICE_IOC_RESET:
//		s3618_reset_ctrl(ts->dev, SW_RESET);
		break;
	case DEVICE_IOC_IRQ:
		if (arg == 0)
			touch_interrupt_control(ts->dev, 0);
		else if (arg == 1)
			touch_interrupt_control(ts->dev, 1);
		break;
	case DEVICE_IOC_RAW:
		if (arg == 0) {
			device_hcd->raw_mode = false;
		} else if (arg == 1) {
			device_hcd->raw_mode = true;
		}
		break;
	case DEVICE_IOC_CONCURRENT:
		if (arg == 0)
			device_hcd->concurrent = false;
		else if (arg == 1)
			device_hcd->concurrent = true;
		break;
	default:
		retval = -ENOTTY;
		break;
	}

	mutex_unlock(&d->extif_mutex);

	return retval;
}

static loff_t device_llseek(struct file *filp, loff_t off, int whence)
{
	return -EINVAL;
}

static ssize_t device_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	int retval = 0;
	struct s3618_data*d = device_hcd->d;

	TOUCH_TRACE();

	if (count == 0)
		return 0;

	mutex_lock(&d->extif_mutex);

	mutex_lock(&device_hcd->resp.buf_mutex);

	if (device_hcd->raw_mode) {
		memset(device_hcd->resp.buf, 0x0, count);
		device_hcd->resp.data_length = 0;

		retval = device_raw_read(device_hcd->resp.buf, count);
		if (retval < 0) {
			TOUCH_E("Failed to read message\n");
			mutex_unlock(&device_hcd->resp.buf_mutex);
			goto exit;
		}
	} else {
		if (count != device_hcd->resp.data_length) {
			TOUCH_E("Invalid length information\n");
			mutex_unlock(&device_hcd->resp.buf_mutex);
			retval = -EINVAL;
			goto exit;
		}
	}

	if (copy_to_user(buf, device_hcd->resp.buf, count)) {
		TOUCH_E("Failed to copy data to user space\n");
		mutex_unlock(&device_hcd->resp.buf_mutex);
		retval = -EINVAL;
		goto exit;
	}

	if (!device_hcd->concurrent)
		goto skip_concurrent;

	if (d->report_touch == NULL) {
		TOUCH_E("Unable to report touch\n");
		device_hcd->concurrent = false;
	}

	if (device_hcd->raw_mode)
		device_capture_touch_report(count);

skip_concurrent:
	mutex_unlock(&device_hcd->resp.buf_mutex);

	retval = count;

exit:
	mutex_unlock(&d->extif_mutex);

	return retval;
}

static ssize_t device_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	int retval = 0;
	struct s3618_data *d = device_hcd->d;
	struct touch_core_data *ts = device_hcd->ts;

	TOUCH_TRACE();

	if (count == 0)
		return 0;

	mutex_lock(&d->extif_mutex);

	mutex_lock(&device_hcd->out.buf_mutex);

	memset(device_hcd->out.buf, 0x0, count == 1 ? count + 1 : count);

	if (copy_from_user(device_hcd->out.buf, buf, count)) {
		TOUCH_E("Failed to copy data from user space\n");
		mutex_unlock(&device_hcd->out.buf_mutex);
		retval = -EINVAL;
		goto exit;
	}

	mutex_lock(&device_hcd->resp.buf_mutex);

	if (device_hcd->raw_mode) {
		retval = device_raw_write(device_hcd->out.buf[0],
					&device_hcd->out.buf[1],
					count - 1);
	} else {
		mutex_lock(&d->reset_mutex);
		retval = s3618_write_message(ts->dev,
						device_hcd->out.buf[0],
						&device_hcd->out.buf[1],
						count - 1,
						device_hcd->resp.buf,
						&device_hcd->resp.data_length);
		mutex_unlock(&d->reset_mutex);
	}
	if (retval < 0) {
		TOUCH_E("Failed to write command 0x%02x\n",
				device_hcd->out.buf[0]);
		mutex_unlock(&device_hcd->resp.buf_mutex);
		mutex_unlock(&device_hcd->out.buf_mutex);
		goto exit;
	}

	if (count && device_hcd->out.buf[0] == CMD_SET_TOUCH_REPORT_CONFIG) {
		retval = device_capture_touch_report_config(count);
		if (retval < 0) {
			TOUCH_E("Failed to capture touch report config\n");
		}
	}

	mutex_unlock(&device_hcd->out.buf_mutex);

	if (device_hcd->raw_mode)
		retval = count;
	else
		retval = device_hcd->resp.data_length;

	mutex_unlock(&device_hcd->resp.buf_mutex);

exit:
	mutex_unlock(&d->extif_mutex);

	return retval;
}

static int device_open(struct inode *inp, struct file *filp)
{
	int retval = 0;
	struct s3618_data *d = device_hcd->d;

	TOUCH_TRACE();

	mutex_lock(&d->extif_mutex);

	if (device_hcd->ref_count < 1) {
		device_hcd->ref_count++;
		d->isDeviceOpen = true;
		retval = 0;
	} else {
		retval = -EACCES;
	}

	mutex_unlock(&d->extif_mutex);

	return retval;
}

static int device_release(struct inode *inp, struct file *filp)
{
	struct s3618_data *d = device_hcd->d;

	TOUCH_TRACE();

	mutex_lock(&d->extif_mutex);

	if (device_hcd->ref_count) {
		device_hcd->ref_count--;
		d->isDeviceOpen = false;
	}

	mutex_unlock(&d->extif_mutex);

	return 0;
}

static char *device_devnode(struct device *dev, umode_t *mode)
{
	if (!mode)
		return NULL;

	TOUCH_TRACE();

	*mode = (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);

	return kasprintf(GFP_KERNEL, "%s/%s", PLATFORM_DRIVER_NAME,
			dev_name(dev));
}

static int device_create_class(void)
{
	if (device_hcd->class != NULL)
		return 0;

	TOUCH_TRACE();

	device_hcd->class = class_create(THIS_MODULE, PLATFORM_DRIVER_NAME);

	if (IS_ERR(device_hcd->class)) {
		TOUCH_E("Failed to create class\n");
		return -ENODEV;
	}

	device_hcd->class->devnode = device_devnode;

	return 0;
}

static const struct file_operations device_fops = {
	.owner = THIS_MODULE,
#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = device_ioctl,
#ifdef HAVE_COMPAT_IOCTL
	.compat_ioctl = device_ioctl,
#endif
#else
	.ioctl = device_ioctl,
#endif
	.llseek = device_llseek,
	.read = device_read,
	.write = device_write,
	.open = device_open,
	.release = device_release,
};

static int device_init(struct device *dev)
{
	int retval = 0;
	dev_t dev_num = 0;
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);

	TOUCH_TRACE();

	if (device_hcd) {
		TOUCH_I("%s: Handle already exists\n", __func__);
		return 0;
	}

	device_hcd = kzalloc(sizeof(*device_hcd), GFP_KERNEL);
	if (!device_hcd) {
		TOUCH_E("Failed to allocate memory for device_hcd\n");
		return -ENOMEM;
	}

	device_hcd->ts = ts;
	device_hcd->d = d;

	device_hcd->concurrent = CONCURRENT;

	mutex_init(&device_hcd->out.buf_mutex);
	mutex_init(&device_hcd->resp.buf_mutex);
	mutex_init(&device_hcd->report.buf_mutex);

	if (rmidev_major_num) {
		dev_num = MKDEV(rmidev_major_num, 0);
		retval = register_chrdev_region(dev_num, 1,
				PLATFORM_DRIVER_NAME);
		if (retval < 0) {
			TOUCH_E("Failed to register char device\n");
			goto err_register_chrdev_region;
		}
	} else {
		retval = alloc_chrdev_region(&dev_num, 0, 1,
				PLATFORM_DRIVER_NAME);
		if (retval < 0) {
			TOUCH_E("Failed to allocate char device\n");
			goto err_alloc_chrdev_region;
		}

		rmidev_major_num = MAJOR(dev_num);
	}

	device_hcd->dev_num = dev_num;

	cdev_init(&device_hcd->char_dev, &device_fops);

	retval = cdev_add(&device_hcd->char_dev, dev_num, 1);
	if (retval < 0) {
		TOUCH_E("Failed to add char device\n");
		goto err_add_chardev;
	}

	retval = device_create_class();
	if (retval < 0) {
		TOUCH_E("Failed to create class\n");
		goto err_create_class;
	}

	device_hcd->device = device_create(device_hcd->class, NULL,
			device_hcd->dev_num, NULL, CHAR_DEVICE_NAME"%d",
			MINOR(device_hcd->dev_num));
	if (IS_ERR(device_hcd->device)) {
		TOUCH_E("Failed to create device\n");
		retval = -ENODEV;
		goto err_create_device;
	}

	if (ts->int_pin >= 0) {
		retval = gpio_export(ts->int_pin, false);
		if (retval < 0) {
			TOUCH_E("Failed to export GPIO\n");
		} else {
			retval = gpio_export_link(ts->dev,
					"attn", ts->int_pin);
			if (retval < 0) {
				TOUCH_E("Failed to export GPIO link\n");
			}
		}
	}

	return 0;

err_create_device:
	class_destroy(device_hcd->class);

err_create_class:
	cdev_del(&device_hcd->char_dev);

err_add_chardev:
	unregister_chrdev_region(dev_num, 1);

err_alloc_chrdev_region:
err_register_chrdev_region:

	kfree(device_hcd);
	device_hcd = NULL;

	return retval;
}

static void device_remove(struct device *dev)
{
	TOUCH_TRACE();

	if (!device_hcd)
		goto exit;

	device_destroy(device_hcd->class, device_hcd->dev_num);

	class_destroy(device_hcd->class);

	cdev_del(&device_hcd->char_dev);

	unregister_chrdev_region(device_hcd->dev_num, 1);

	kfree(device_hcd);
	device_hcd = NULL;

exit:
	complete(&device_remove_complete);

	return;
}

static void device_reinit(struct device *dev)
{
	int retval = 0;

	TOUCH_TRACE();

	if (!device_hcd) {
		retval = device_init(dev);
		return;
	}

	return;
}

static struct s3618_exp_fn device_module = {
//	.type = TCM_DEVICE,
	.init = device_init,
	.remove = device_remove,
	.syncbox = NULL,
#ifdef REPORT_NOTIFIER
	.asyncbox = NULL,
#endif
	.reinit = device_reinit,
	.suspend = NULL,
	.resume = NULL,
	.early_suspend = NULL,
};

static int __init device_module_init(void)
{
	s3618_device_function(&device_module, true);

	return 0;
}

static void __exit device_module_exit(void)
{
	s3618_device_function(&device_module, false);

	wait_for_completion(&device_remove_complete);

	return;
}

module_init(device_module_init);
module_exit(device_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TCM Device Module");
MODULE_LICENSE("GPL v2");
