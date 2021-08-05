/* touch_s3618.c
 *
 * Copyright (C) 2018 LGE.
 *
 * Author: BSP-TOUCH@lge.com
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_s3618.h"

extern int lge_get_mfts_mode(void);

const char *s3618_lpwg_failreason_tci_str[LPWG_FAILREASON_TCI_NUM] = {
	"NONE",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_INTER_TAP",
	"MULTI_FINGER",
	"DELAY_TIME",
	"PALM_STATE",
	"ACTIVE_AREA",
	"TAP_COUNT",
};

const char *s3618_lpwg_failreason_swipe_str[LPWG_FAILREASON_SWIPE_NUM] = {
	"NONE",
	"FAST_RELEASE",
	"MULTI_FINGER",
	"FAST_SWIPE",
	"SLOW_SWIPE",
	"WRONG_DIRECTION",
	"RATIO_FAIL",
	"OUT_OF_START_AREA",
	"OUT_OF_ACTIVE_AREA",
	"PALM_STATE",
	"OTHERS_INITRATIO",
	"OTHERS",
};

const char *s3618_gesture_config_code_str[CONFIG_CODE_MAX] = {
	"CONFIG_CODE_KNOCK_ON",
	"CONFIG_CODE_KNOCK_CODE",
	"CONFIG_CODE_SWIPE_UP",
	"CONFIG_CODE_SWIPE_LEFT",
	"CONFIG_CODE_SWIPE_RIGHT",
	"CONFIG_CODE_SWIPE_LEFT2",
	"CONFIG_CODE_SWIPE_RIGHT2",
	"CONFIG_CODE_LONG_PRESS",
	"CONFIG_CODE_ONE_TAP",
	"CONFIG_CODE_LPWG_ABS",
	"CONFIG_CODE_TCI_ACTIVE_AREA",
};

//static struct s3618_exp_fhandler rmidev_fhandler;
static struct s3618_exp_fhandler reflash_fhandler;
static struct s3618_exp_fhandler testing_fhandler;
static struct s3618_exp_fhandler device_fhandler;


static int s3618_lpwg_mode(struct device *dev);
//static int s3618_report_control(struct device *dev, int enable, int report_type);
static int s3618_tci_active_area(struct device *dev, u16 x1, u16 y1, u16 x2, u16 y2);
extern int reflash_read_product_info(void);

bool s3618_is_product(struct s3618_data *d,
				const char *product_id, size_t len)
{
	return strncmp(d->prd_info.product_id, product_id, len)
			? false : true;
}

int s3618_read(struct device *dev, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = size,
			.buf = data,
		},
	};

	int ret = 0;
	// DEBUG
	int i = 0;
	u8 *tmp = (u8*)data;

	mutex_lock(&d->io_mutex);

#if defined(CONFIG_SECURE_TOUCH)
	if (atomic_read(&ts->st_enabled)) {
		TOUCH_E("cannot use i2c, ownership changed!\n");
		ret = -EINVAL;
		goto exit;
	}
#endif

	if (size > CHUNK_SIZE) {
		TOUCH_I("s3618 read size overflow!!!");
		ret = -ENOMEM;
		goto exit;
	}

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		goto exit;
	}

	// DEBUG
	for (i = 0; i < size; i++) {
		TOUCH_D(GET_DATA, "%s - data[%d]=0x%x\n", __func__, i, tmp[i]);
	}

exit:
	mutex_unlock(&d->io_mutex);

	return ret;
}

static int s3618_get_report_data(struct device *dev, u32 offset, u32 bits, u32 *data)
{
	struct s3618_data *d = to_s3618_data(dev);
	u8 mask = 0, byte_data = 0;
	u32 output_data = 0, bit_offset = 0, byte_offset = 0, data_bits = 0, available_bits = 0;
	int remaining_bits = 0;

	if (bits == 0 || bits > 32) {
		TOUCH_E("Invalid number of bits\n");
		return -EINVAL;
	}

	if (offset + bits > d->report.data_length * 8) {
		TOUCH_E("Overflow d->report.data_length*8, offset+bits:%d+%d\n", offset, bits);
		*data = 0;
		return 0;
	}

	output_data = 0;
	remaining_bits = bits;

	bit_offset = offset % 8;
	byte_offset = offset / 8;

	while (remaining_bits > 0) {
		if (byte_offset >= BUFFER_SIZE) {
			TOUCH_E("Overflow d->report.buf, byte_offset:%d\n", byte_offset);
			break;
		}
		byte_data = d->report.buf[byte_offset];
		byte_data >>= bit_offset;

		available_bits = 8 - bit_offset;
		data_bits = MIN(available_bits, remaining_bits);
		mask = 0xff >> (8 - data_bits);

		byte_data &= mask;

		output_data |= byte_data << (bits - remaining_bits);

		bit_offset = 0;
		byte_offset += 1;
		remaining_bits -= data_bits;
		if (remaining_bits < 0) {
			TOUCH_E("remaining_bits is negative value:%d\n", remaining_bits);
			break;
		}
	}

	*data = output_data;

	TOUCH_D(IRQ_HANDLE, "Packet: report_data=0x%x\n", output_data);

	return 0;

}

static int s3618_parse_data(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0, bits = 0;
	u8 *config_data = NULL;
	u8 config_size = 0, code = 0;
	u32 config_idx = 0, offset = 0, obj = 0, objects = 0, next = 0, data = 0, buf_idx = 0;
	static u32 end_of_foreach = 0;
	bool active_only = false, num_of_active_objects = false;

	TOUCH_TRACE();

	if (d->config.data_length <= 0) {
		TOUCH_E("d->config.buf is not ready, data_length: %d\n",
				d->config.data_length);
		return -EINVAL;
	}

	config_data = d->config.buf;
	config_size = d->config.data_length;
	config_idx = 0;
	offset = 0;
	objects = 0;

	memset(&d->touch_info, 0, sizeof(d->touch_info));

	while (config_idx < config_size) {
		TOUCH_D(IRQ_HANDLE, "Packet: config_data[%d]=0x%x, [%d]=%dbits\n",
				config_idx, config_data[config_idx],
				config_idx+1, config_data[config_idx+1]);

		code = config_data[config_idx];
		config_idx++;
		switch (code) {
		case TOUCH_END:
			goto exit;
			break;
		case TOUCH_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			next = config_idx;
			active_only = true;
			break;
		case TOUCH_FOREACH_OBJECT:
			obj = 0;
			next = config_idx;
			active_only = false;
			break;
		case TOUCH_FOREACH_END:
			end_of_foreach = config_idx;
			if (active_only) {
				if (num_of_active_objects) {
					objects++;
					if (objects < d->touch_info.num_of_active_objects)
						config_idx = next;
				} else if (offset < d->report.data_length * 8) {
					config_idx = next;
				}
			} else {
				obj++;
				if (obj < MAX_FINGER)
					config_idx = next;
			}
			break;
		case TOUCH_PAD_TO_NEXT_BYTE:
			offset = ceil_div(offset, 8) * 8;
			break;
		case TOUCH_CUSTOMER_GESTURE_DETECTED:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get customer gesture Detected\n");
				return ret;
			}
			d->touch_info.gesture_detected = data;
			offset += bits;
			break;
		case TOUCH_CUSTOMER_GESTURE_INFO:
			bits = config_data[config_idx];
			config_idx++;
			buf_idx = 0;
			while (bits > 0) {
				if (buf_idx >= 20)
					break;
				s3618_get_report_data(dev, offset, 8, &data);
				if (ret < 0) {
					TOUCH_E("Failed to get lpwg_data %dth byte\n", buf_idx);
					return ret;
				}
				d->touch_info.lpwg_data.buf[buf_idx] = data;
				offset += 8;
				bits -= 8;
				buf_idx++;
			}
			break;
		case TOUCH_CUSTOMER_GESTURE_INFO2:
			bits = config_data[config_idx];
			config_idx++;
			buf_idx = 20;
			while (bits > 0) {
				if (buf_idx >= 40)
					break;
				s3618_get_report_data(dev, offset, 8, &data);
				if (ret < 0) {
					TOUCH_E("Failed to get lpwg_data %dth byte\n", buf_idx);
					return ret;
				}
				d->touch_info.lpwg_data.buf[buf_idx] = data;
				offset += 8;
				bits -= 8;
				buf_idx++;
			}
			break;
		case TOUCH_PALM_DETECTED:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get palm detect\n");
				return ret;
			}
			d->touch_info.palm_detected = data;
			offset += bits;
			break;
		case TOUCH_BASELINE_ERR_LOG:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get baseline err log\n");
				return ret;
			}
			d->touch_info.baseline_err_log = data;
			offset += bits;
			break;
		case TOUCH_POWER_IM:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get power IM\n");
				return ret;
			}
			d->touch_info.power_im = data;
			offset += bits;
			break;
		case TOUCH_NSM_STATE:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get NSM state\n");
				return ret;
			}
			d->touch_info.nsm_state = data;
			offset += bits;
			break;
		case TOUCH_NUM_OF_ACTIVE_OBJECTS:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get number of active objects\n");
				return ret;
			}
			num_of_active_objects = true;

			if (data < 0) {
				TOUCH_E("Invalid num_of_active_objects = %d -> 0\n", data);
				d->touch_info.num_of_active_objects = 0;
			} else if (data > MAX_FINGER) {
				TOUCH_E("Invalid num_of_active_objects = %d -> MAX_FINGER\n", data);
				d->touch_info.num_of_active_objects = MAX_FINGER;
			} else {
				d->touch_info.num_of_active_objects = data;
			}

			offset += bits;
			if (d->touch_info.num_of_active_objects == 0) {
				if (end_of_foreach != 0) {
					config_idx = end_of_foreach;
				} else {
					TOUCH_I("num_of_active_objects & end_of_foreach are 0\n");
					goto exit;
				}
			}
			break;
		case TOUCH_OBJECT_N_INDEX:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object index\n");
				return ret;
			}

			if (data < 0) {
				TOUCH_E("Invalid obj = %d -> 0\n", data);
				obj = 0;
			} else if (data >= MAX_FINGER) {
				TOUCH_E("Invalid obj = %d -> MAX_FINGER\n", data);
				obj = MAX_FINGER - 1;
			} else {
				obj = data;
			}

			d->touch_info.data[obj].index = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_CLASSIFICATION:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object classification\n");
				return ret;
			}
			d->touch_info.data[obj].classification = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_POSITION:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object x position\n");
				return ret;
			}
			d->touch_info.data[obj].x_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_POSITION:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object y position\n");
				return ret;
			}
			d->touch_info.data[obj].y_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Z:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object z\n");
				return ret;
			}
			d->touch_info.data[obj].z = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_ANGLE:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object angle\n");
				return ret;
			}
			d->touch_info.data[obj].angle = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_MAJOR:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object major\n");
				return ret;
			}
			d->touch_info.data[obj].major = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_MINOR:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object minor\n");
				return ret;
			}
			d->touch_info.data[obj].minor = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_WIDTH:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object x width\n");
				return ret;
			}
			d->touch_info.data[obj].x_width = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_WIDTH:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object y width\n");
				return ret;
			}
			d->touch_info.data[obj].y_width = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_TX_POSITION_TIXELS:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object tx position\n");
				return ret;
			}
			d->touch_info.data[obj].tx_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_RX_POSITION_TIXELS:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object rx position\n");
				return ret;
			}
			d->touch_info.data[obj].rx_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_AREA:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object area\n");
				return ret;
			}
			d->touch_info.data[obj].area = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_FORCE:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object force\n");
				return ret;
			}
			d->touch_info.force_measurement = data;
			offset += bits;
			break;
		case TOUCH_TIMESTAMP:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get timestamp\n");
				return ret;
			}
			d->touch_info.timestamp = data;
			offset += bits;
			break;
		case TOUCH_FINGERPRINT_AREA_MEET:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get object force\n");
				return ret;
			}
			d->touch_info.fingerprint_area_meet = data;
			TOUCH_I("fingerprint_area_meet = %x\n", d->touch_info.fingerprint_area_meet);
			offset += bits;
			break;
		case TOUCH_0D_BUTTONS_STATE:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get 0D buttons state\n");
				return ret;
			}
			d->touch_info.buttons_state = data;
			offset += bits;
			break;
		case TOUCH_GESTURE_ID:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get gesture double tap\n");
				return ret;
			}
			d->touch_info.gesture_id = data;
			offset += bits;
			break;
		case TOUCH_FRAME_RATE:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get frame rate\n");
				return ret;
			}
			d->touch_info.frame_rate = data;
			offset += bits;
			break;
		case TOUCH_CID_IM:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get CID IM\n");
				return ret;
			}
			d->touch_info.cid_im = data;
			offset += bits;
			break;
		case TOUCH_RAIL_IM:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get rail IM\n");
				return ret;
			}
			d->touch_info.rail_im = data;
			offset += bits;
			break;
		case TOUCH_CID_VARIANCE_IM:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get CID variance IM\n");
				return ret;
			}
			d->touch_info.cid_variance_im = data;
			offset += bits;
			break;
		case TOUCH_NSM_FREQUENCY:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get NSM frequency\n");
				return ret;
			}
			d->touch_info.nsm_frequency = data;
			offset += bits;
			break;
		case TOUCH_GESTURE_DATA:
			bits = config_data[config_idx];
			config_idx++;
			offset += bits;
			break;
		case TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to get number of CPU cycles used since last frame\n");
				return ret;
			}
			d->touch_info.num_of_cpu_cycles = data;
			offset += bits;
			break;
		case TOUCH_FACE_DETECT:
			bits = config_data[config_idx];
			config_idx++;
			ret = s3618_get_report_data(dev, offset, bits, &data);
			if (ret < 0) {
				TOUCH_E("Failed to detect face\n");
				return ret;
			}
			d->touch_info.face_detected = data;
			offset += bits;
			break;
		case TOUCH_TUNING_GAUSSIAN_WIDTHS:
			bits = config_data[config_idx];
			config_idx++;
			offset += bits;
			break;
		case TOUCH_TUNING_SMALL_OBJECT_PARAMS:
			bits = config_data[config_idx];
			config_idx++;
			offset += bits;
			break;
		case TOUCH_TUNING_0D_BUTTONS_VARIANCE:
			bits = config_data[config_idx];
			config_idx++;
			offset += bits;
			break;
		default:
			bits = config_data[config_idx];
			config_idx++;
			offset += bits;
			break;
		}
	}

exit:
	return 0;
}

static void s3618_lpwg_abs_filter(struct device *dev, u8 touch_id)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);

	u16 old_y = ts->tdata[touch_id].y;
	u16 new_y = old_y - d->lpwg_abs.offset_y;
	u16 old_mask = ts->old_mask;
	u16 new_mask = ts->new_mask;
	u16 change_mask = old_mask ^ new_mask;
	u16 press_mask = new_mask & change_mask;
	bool hide_lockscreen_coord =
		((atomic_read(&ts->state.lockscreen) == LOCKSCREEN_LOCK) &&
		 (ts->role.hide_coordinate));

	TOUCH_TRACE();

	if ((new_y > ts->caps.max_y) || (new_y < 0)) {
		TOUCH_D(ABS, "%s: invalid new_y(%d)\n", __func__, new_y);
		new_y = 0;
	}

	if (press_mask & (1 << touch_id)) {
		if (hide_lockscreen_coord) {
			TOUCH_I("%s: <id:%d> shift Y value(xxxx->xxxx)\n",
					__func__, touch_id);
		} else {
			TOUCH_I("%s: <id:%d> shift Y value(%d->%d)\n",
					__func__, touch_id, old_y, new_y);
		}
	}

	ts->tdata[touch_id].y = new_y;
}

static int s3618_report_abs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	struct touch_data *tdata = NULL;

	u8 finger_index = 0;
	int ret = 0;
	int i = 0;

	TOUCH_TRACE();

	ts->new_mask = 0;

	if (d->touch_info.palm_detected && !d->is_palm) {
		TOUCH_I("Palm Detected\n");
		d->is_palm = true;
		ts->is_cancel = 1;
		ts->tcount = 0;
		ts->intr_status = TOUCH_IRQ_FINGER;
		return ret;
	} else if (!d->touch_info.palm_detected && d->is_palm) {
		TOUCH_I("Palm Released\n");
		d->is_palm = false;
		ts->is_cancel = 1;
		ts->tcount = 0;
		ts->intr_status = TOUCH_IRQ_FINGER;
		return ret;
	}

	for (i = 0; i < MAX_FINGER; i++) {
		// TODO DEBUG
		if (d->touch_info.data[i].classification == NO_OBJECT)
			continue;

		if (d->touch_info.data[i].classification == FINGER) {
			ts->new_mask |= (1 << i);
			tdata = ts->tdata + i;

			tdata->id = d->touch_info.data[i].index;
			tdata->type = d->touch_info.data[i].classification;
			tdata->x = d->touch_info.data[i].x_pos;
			tdata->y = d->touch_info.data[i].y_pos;
			tdata->pressure = d->touch_info.data[i].z;
			tdata->width_major = d->touch_info.data[i].major;
			tdata->width_minor = d->touch_info.data[i].minor;
			tdata->orientation = d->touch_info.data[i].angle;

			finger_index++;

			TOUCH_D(ABS, "tdata[ID:%2d Type:%2d X:%4d Y:%4d Z:%4d WM:%4d Wm:%4d Angle:%4d]\n",
					tdata->id,
					tdata->type,
					tdata->x,
					tdata->y,
					tdata->pressure,
					tdata->width_major,
					tdata->width_minor,
					tdata->orientation);

			if (d->lpwg_abs.enable)
				s3618_lpwg_abs_filter(dev, i);
		}
	}

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	if (ts->old_mask != ts->new_mask) {
		TOUCH_I("POWER_IM[%d], NSM_STATE[%d], BASELINE[%d]",
				d->touch_info.power_im,
				d->touch_info.nsm_state,
				d->touch_info.baseline_err_log);
	}

	return ret;
}

static int s3618_get_lpwg_data(struct device *dev, int count) {
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	struct s3618_coord_data *coord_data = NULL;
	int i = 0;

	TOUCH_TRACE();

	ts->lpwg.code_num = count;

	if (!count)
		return 0;

	coord_data = (struct s3618_coord_data *)d->touch_info.lpwg_data.buf;

	for (i = 0; i < count; i++) {
		ts->lpwg.code[i].x = coord_data->point[i].x;
		ts->lpwg.code[i].y = coord_data->point[i].y;

		if ((ts->lpwg.mode == LPWG_PASSWORD) &&
				(ts->role.hide_coordinate))
			TOUCH_I("LPWG data xxxx, xxxx\n");
		else
			TOUCH_I("LPWG data [%d] %d, %d\n",
				d->touch_info.gesture_detected,
				ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	ts->lpwg.code[count].x = -1;
	ts->lpwg.code[count].y = -1;

	return 0;
}

static int s3618_get_swipe_data(struct device *dev) {
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	struct s3618_swipe_coord_data *swipe_coord_data = NULL;
	char str[8] = {0};
	int ret = 0;

	TOUCH_TRACE();

	swipe_coord_data = (struct s3618_swipe_coord_data *)d->touch_info.lpwg_data.buf;

	switch (swipe_coord_data->index) {
		case S3618_SWIPE_U:
			touch_snprintf(str, sizeof(str), "UP");
			ts->intr_status = TOUCH_IRQ_SWIPE_UP;
			break;
		case S3618_SWIPE_L:
			touch_snprintf(str, sizeof(str), "LEFT");
			ts->intr_status = TOUCH_IRQ_SWIPE_LEFT;
			if (d->lpwg_abs.enable) {
				TOUCH_I("%s: lpwg_abs is enabled - skip SWIPE_L gesture\n", __func__);
				ts->intr_status = TOUCH_IRQ_NONE;
				goto exit;
			}
			break;
		case S3618_SWIPE_R:
			touch_snprintf(str, sizeof(str), "RIGHT");
			ts->intr_status = TOUCH_IRQ_SWIPE_RIGHT;
			if (d->lpwg_abs.enable) {
				TOUCH_I("%s: lpwg_abs is enabled - skip SWIPE_R gesture\n",
						__func__);
				ts->intr_status = TOUCH_IRQ_NONE;
				goto exit;
			}
			break;
		case S3618_SWIPE_L2:
			touch_snprintf(str, sizeof(str), "LEFT2");
			ts->intr_status = TOUCH_IRQ_SWIPE_LEFT2;
			break;
		case S3618_SWIPE_R2:
			touch_snprintf(str, sizeof(str), "RIGHT2");
			ts->intr_status = TOUCH_IRQ_SWIPE_RIGHT2;
			break;
		default:
			TOUCH_E("Wrong swipe_coord_data index: %d\n",
					swipe_coord_data->index);
			ts->intr_status = TOUCH_IRQ_ERROR;
			ret = -EINVAL;
			goto exit;
	}

	TOUCH_I("%s: Swipe %s - start(%4d,%4d) end(%4d,%4d) swipe_time(%dms)\n",
			__func__, str,
			swipe_coord_data->start_x, swipe_coord_data->start_y,
			swipe_coord_data->end_x, swipe_coord_data->end_y,
			swipe_coord_data->swipe_time);

	ts->lpwg.code_num = 1;
	ts->lpwg.code[0].x = swipe_coord_data->end_x;
	ts->lpwg.code[0].y = swipe_coord_data->end_y;
	ts->lpwg.code[1].x = -1;
	ts->lpwg.code[1].y = -1;

exit:
	return ret;
}

static bool s3618_check_ai_pick_event(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	struct s3618_active_area *area = &d->ai_pick.total_area;

	int i = 0;
	bool result[2] = {false, false};

	TOUCH_TRACE();

	for (i = 0; i < 2; i++) {
		if ((ts->lpwg.code[i].x >= area->x1)
				&& (ts->lpwg.code[i].x <= area->x2)
				&& (ts->lpwg.code[i].y >= area->y1)
				&& (ts->lpwg.code[i].y <= area->y2)) {
			result[i] = true;
		}
	}

	return (result[0] & result[1]);
}
#if defined(USE_LONGPRESS_AROUND)
static int s3618_check_udf_sensor_inside(struct device *dev, int x, int y)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;

	TOUCH_I("x = %d, y = %d, area x1 = %d, area y1 = %d,  area x2 = %d, area y2 =%d\n",
			x, y,
			d->lpwg_longpress.area.x1,
			d->lpwg_longpress.area.y1,
			d->lpwg_longpress.area.x2,
			d->lpwg_longpress.area.y2);

	if (x >= d->lpwg_longpress.area.x1 &&
			y >= d->lpwg_longpress.area.y1 &&
			x <= d->lpwg_longpress.area.x2 &&
			y <= d->lpwg_longpress.area.y2) {
		ret = (int)true;
	} else {
		ret = (int)false;
	}

	return ret;
}
#endif
static int s3618_report_lpwg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	switch (d->touch_info.gesture_detected) {
		case DETECT_KNOCK_ON:
			ret = s3618_get_lpwg_data(dev, ts->tci.info[TCI_1].tap_count);
			if (ret < 0) {
				TOUCH_E("failed to get DOUBLE_TAP data (reg: %d)\n", ret);
				goto error;
			}
			ts->intr_status = TOUCH_IRQ_KNOCK;
			if (d->ai_pick.enable) {
				if (s3618_check_ai_pick_event(dev) && d->lcd_mode < LCD_MODE_U3) {
					ts->intr_status = TOUCH_IRQ_AI_PICK;
					TOUCH_I("%s: send ai_pick event\n", __func__);
				} else {
					if (ts->lpwg.mode == LPWG_PASSWORD_ONLY) {
						TOUCH_I("%s: ignore knock on event about ai_pick\n", __func__);
						ts->intr_status = TOUCH_IRQ_NONE;
					}
				}
			}
			break;
		case DETECT_KNOCK_CODE:
			ret = s3618_get_lpwg_data(dev, ts->tci.info[TCI_2].tap_count);
			if (ret < 0) {
				TOUCH_E("failed to get PASSWORD data (reg: %d)\n", ret);
				goto error;
			}
			ts->intr_status = TOUCH_IRQ_PASSWD;
			break;
		case DETECT_LONG_PRESS_DOWN:
			ret = s3618_get_lpwg_data(dev, 1);
			if (ret < 0) {
				TOUCH_E("failed to get LONG_PRESS_DOWN data (reg: %d)\n", ret);
				goto error;
			}

#if defined(USE_LONGPRESS_AROUND)
			if(s3618_check_udf_sensor_inside(dev, ts->lpwg.code[0].x, ts->lpwg.code[0].y)) {
				ts->intr_status = TOUCH_IRQ_LPWG_LONGPRESS_DOWN;
			} else {
				ts->intr_status = TOUCH_IRQ_LPWG_LONGPRESS_DOWN_AROUND;
			}
#else
			ts->intr_status = TOUCH_IRQ_LPWG_LONGPRESS_DOWN;
#endif
			d->longpress_uevent_status = ts->intr_status;
			break;
		case DETECT_LONG_PRESS_UP:
			ret = s3618_get_lpwg_data(dev, 1);
			if (ret < 0) {
				TOUCH_E("failed to get LONG_PRESS_UP data (reg: %d)\n", ret);
				goto error;
			}

#if defined(USE_LONGPRESS_AROUND)
			if(d->longpress_uevent_status == TOUCH_IRQ_LPWG_LONGPRESS_DOWN) {
				ts->intr_status = TOUCH_IRQ_LPWG_LONGPRESS_UP;
			} else {
				ts->intr_status = TOUCH_IRQ_LPWG_LONGPRESS_UP_AROUND;
			}
#else
			ts->intr_status = TOUCH_IRQ_LPWG_LONGPRESS_UP;
#endif
			d->longpress_uevent_status = ts->intr_status;
			break;
		case DETECT_ONE_TAP:
			ret = s3618_get_lpwg_data(dev, 1);
			if (ret < 0) {
				TOUCH_E("failed to get ONT_TAP data (reg: %d)\n", ret);
				goto error;
			}
			ts->intr_status = TOUCH_IRQ_LPWG_SINGLE_WAKEUP;
			break;
		case DETECT_SWIPE:
			ret = s3618_get_swipe_data(dev);
			if (ret < 0) {
				TOUCH_E("failed to get swipe data (reg: %d)\n", ret);
				goto error;
			}
			break;
		default:
			TOUCH_E("Wrong gesture_detected id: %d\n",
					d->touch_info.gesture_detected);
			return -EINVAL;
	}

error:
	return ret;
}

static int s3618_report_touch(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = s3618_parse_data(dev);
	if (ret < 0) {
		TOUCH_E("Failed to report touch data\n");
		return ret;
	}

	switch (d->touch_info.gesture_detected) {
		case DETECT_KNOCK_ON:
		case DETECT_KNOCK_CODE:
		case DETECT_LONG_PRESS_DOWN:
		case DETECT_LONG_PRESS_UP:
		case DETECT_SWIPE:
		case DETECT_LONG_PRESS:
		case DETECT_ONE_TAP:
			ret = s3618_report_lpwg(dev);
			if (ret < 0) {
				TOUCH_E("Failed to report lpwg data\n");
				return ret;
			}
			break;
		default:
			ret = s3618_report_abs(dev);
			if (ret < 0) {
				TOUCH_E("Failed to report abs data\n");
				return ret;
			}
			break;

	}

	return 0;
}

static int s3618_dispatch_report(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	mutex_lock(&d->in.buf_mutex);

	switch (d->status_report_code) {
		case REPORT_TOUCH:
			// report directly if touch report is received
			if (atomic_read(&d->state.init) == IC_INIT_DONE) {
				mutex_lock(&d->report.buf_mutex);
				memcpy(d->report.buf, &d->in.buf[MESSAGE_HEADER_SIZE], d->payload_length);
				d->report.data_length = d->payload_length;
				mutex_unlock(&d->report.buf_mutex);

				ret = s3618_report_touch(dev);
				if (ret < 0) {
					TOUCH_E("Failed to report touch data\n");
					goto exit;
				}
			} else {
				TOUCH_I("Touch interrupt occurs before initialization, report_code:0x%x\n",
						d->status_report_code);
				goto exit;
			}
			break;
		case REPORT_IDENTIFY:
			// The device initially powers up, resets, or switch modes,
			// Read identify info for identification
			atomic_set(&d->state.power, POWER_ON);

			memcpy((u8 *)&d->id_info, &d->in.buf[MESSAGE_HEADER_SIZE], d->payload_length);
			d->chunk_size = MIN(d->id_info.max_write_size, CHUNK_SIZE);
			if (d->chunk_size == 0)
				d->chunk_size = d->id_info.max_write_size;

			TOUCH_I("Received identify report (firmware mode = 0x%02x)\n", d->id_info.mode);

			if (atomic_read(&d->command_status) == CMD_BUSY) {
				switch (d->command) {
					case CMD_RESET:
					case CMD_RUN_BOOTLOADER_FIRMWARE:
					case CMD_RUN_APPLICATION_FIRMWARE:
					case CMD_ENTER_PRODUCTION_TEST_MODE:
					case CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE:
						d->response_code = STATUS_OK;
						atomic_set(&d->command_status, CMD_IDLE);
						complete(&d->response_complete);
						break;
					default:
						TOUCH_E("Device has been reset\n");
						atomic_set(&d->command_status, CMD_ERROR);
						complete(&d->response_complete);
						break;
				}

			}

			if (atomic_read(&d->state.init) == IC_INIT_DONE) {
				TOUCH_I("Trigger init_work: Received identify report with IC_INIT_DONE state (IC Reset occured)\n");
				mod_delayed_work(ts->wq, &ts->init_work, 0);
				goto exit;
			}
			break;
		case REPORT_RAW:
		case REPORT_DELTA:
			mutex_lock(&d->report.buf_mutex);
			memcpy(d->report.buf,
					&d->in.buf[MESSAGE_HEADER_SIZE],
					d->payload_length);
			d->report.data_length = d->payload_length;
			mutex_unlock(&d->report.buf_mutex);

			TOUCH_I("report Delta/Raw (0x%x)\n", d->status_report_code);
			d->report_is_ready = true;
			break;
		default:
			break;
	}
exit:
	mutex_unlock(&d->in.buf_mutex);

	return ret;
}

static int s3618_dispatch_response(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);

	TOUCH_TRACE();

	if (atomic_read(&d->command_status) != CMD_BUSY) {
		TOUCH_E("Command status != CMD_BUSY\n");
		return -EINVAL;
	}

	d->response_code = d->status_report_code;

	if (d->payload_length == 0) {
		TOUCH_D(ABS, "Payload_length == 0\n");
		atomic_set(&d->command_status, CMD_IDLE);
		goto exit;
	}

	mutex_lock(&d->response.buf_mutex);
	mutex_lock(&d->in.buf_mutex);
	memset(d->response.buf, 0x00, sizeof(d->response.buf));
	d->response.data_length = 0;

	memcpy(d->response.buf,
			&d->in.buf[MESSAGE_HEADER_SIZE],
			d->payload_length);
	d->response.data_length = d->payload_length;

	mutex_unlock(&d->in.buf_mutex);
	mutex_unlock(&d->response.buf_mutex);

	atomic_set(&d->command_status, CMD_IDLE);

exit:
	TOUCH_D(ABS, "Dispatch message end\n");
	complete(&d->response_complete);

	return 0;
}

static int s3618_continued_read(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;
	u8 marker = 0, code = 0;
	u32 idx = 0, offset = 0, chunks = 0, chunk_space = 0, xfer_length = 0, remaining_length = 0;

	remaining_length = d->payload_length + 1;

	/* available chunk space for payload = total chunk size minus header
	 * marker byte and header code byte */
	if (d->chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = d->chunk_size - 2;

	chunks = ceil_div(remaining_length, chunk_space);
	chunks = (chunks == 0) ? 1 : chunks;
	offset = MESSAGE_HEADER_SIZE;

	mutex_lock(&d->temp.buf_mutex);
	mutex_lock(&d->in.buf_mutex);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		if (xfer_length == 1) {
			d->in.buf[offset] = MESSAGE_PADDING;
			offset += xfer_length;
			remaining_length -= xfer_length;
			continue;
		}

		memset(d->temp.buf, 0x00, sizeof(d->temp.buf));
		d->temp.data_length = 0;

		ret = s3618_read(dev, d->temp.buf, xfer_length + 2);
		if (ret < 0) {
			TOUCH_E("Failed to read from device\n");
			mutex_unlock(&d->temp.buf_mutex);
			mutex_unlock(&d->in.buf_mutex);
			return ret;
		}

		marker = d->temp.buf[0];
		code = d->temp.buf[1];

		if (marker != MESSAGE_MARKER && code != STATUS_CONTINUED_READ) {
			TOUCH_E("Incorrect header marker/code (0x%02x/0x%02x)\n",
					marker, code);
			mutex_unlock(&d->temp.buf_mutex);
			mutex_unlock(&d->in.buf_mutex);
			return -EIO;
		}

		memcpy(&d->in.buf[offset], &d->temp.buf[2], xfer_length);

		offset += xfer_length;

		remaining_length -= xfer_length;
	}

	mutex_unlock(&d->temp.buf_mutex);
	mutex_unlock(&d->in.buf_mutex);

	return 0;
}

int s3618_read_message(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);
	struct s3618_message_header *message_header = NULL;
	int ret = 0;
	bool retry = true;
	u32 total_length = 0;

	TOUCH_TRACE();

	mutex_lock(&d->rw_ctrl_mutex);
retry:
	mutex_lock(&d->in.buf_mutex);
	ret = s3618_read(dev, d->in.buf, MESSAGE_HEADER_SIZE);
	if (ret < 0) {
		TOUCH_E("Failed to read to device, retry: %d\n", retry);
		mutex_unlock(&d->in.buf_mutex);
		if (retry) {
			touch_msleep(READ_RETRY_TIMEOUT);
			retry = false;
			goto retry;
		}
		goto exit;
	}

	message_header = (struct s3618_message_header *)d->in.buf;

	if (message_header->marker != MESSAGE_MARKER) {
		TOUCH_E("Incorrect header marker (0x%02x)\n", message_header->marker);
		mutex_unlock(&d->in.buf_mutex);
		ret = -EIO;
		if (retry) {
			touch_msleep(READ_RETRY_TIMEOUT);
			retry = false;
			goto retry;
		}
		goto exit;
	}

	d->status_report_code = message_header->code;
	d->payload_length = message_header->length;

	TOUCH_D(ABS, "Message header code = 0x%02x, Payload Length = %d\n", message_header->code, d->payload_length);

	if (message_header->code <= STATUS_ERROR
			|| message_header->code == STATUS_INVALID) {
		switch (message_header->code) {
		case STATUS_OK:
			break;
		case STATUS_CONTINUED_READ:
			TOUCH_I("Out-of-sync continued read\n");
			d->payload_length = 0;
			mutex_unlock(&d->in.buf_mutex);
			ret = 0;
			goto exit;
			break;
		case STATUS_IDLE:
			d->payload_length = 0;
			mutex_unlock(&d->in.buf_mutex);
			ret = 0;
			goto exit;
			break;
		case STATUS_BUSY:
			d->payload_length = 0;
			mutex_unlock(&d->in.buf_mutex);
			ret = 0;
			goto exit;
			break;
		default:
			TOUCH_I("Incorrect message code (0x%02x)\n", message_header->code);
			if (message_header->code == STATUS_INVALID) {
				if (retry) {
					touch_msleep(READ_RETRY_TIMEOUT);
					retry = false;
					mutex_unlock(&d->in.buf_mutex);
					goto retry;
				} else {
					d->payload_length = 0;
				}
			}
		}
	}

	total_length = MESSAGE_HEADER_SIZE + d->payload_length + 1;

	// TODO Predictive reading

	if (d->payload_length == 0) {
		d->in.buf[total_length - 1] = MESSAGE_PADDING;
		goto check_padding;
	}

	mutex_unlock(&d->in.buf_mutex);

	ret = s3618_continued_read(dev);
	if (ret < 0) {
		TOUCH_E("Failed to do continued read\n");
		goto exit;
	};

	mutex_lock(&d->in.buf_mutex);

	d->in.buf[0] = MESSAGE_MARKER;
	d->in.buf[1] = d->status_report_code;
	d->in.buf[2] = (u8)d->payload_length;
	d->in.buf[3] = (u8)(d->payload_length >> 8);

check_padding:
	if (d->in.buf[total_length - 1] != MESSAGE_PADDING) {
		TOUCH_E("Incorrect message padding byte (0x%02x)\n",
				d->in.buf[total_length - 1]);
		mutex_unlock(&d->in.buf_mutex);
		ret = -EIO;
		goto exit;
	}

	mutex_unlock(&d->in.buf_mutex);

	// Dispatch Message
	if (d->status_report_code >= REPORT_IDENTIFY) {
		ret = s3618_dispatch_report(dev);
		if (ret < 0) {
			TOUCH_E("Failed to dispatch report\n");
			goto exit;
		}
	} else {
		ret = s3618_dispatch_response(dev);
		if (ret < 0) {
			TOUCH_E("Failed to dispatch response\n");
			if (atomic_read(&d->command_status) == CMD_BUSY) {
				atomic_set(&d->command_status, CMD_ERROR);
				complete(&d->response_complete);
			}
			goto exit;
		}
	}

	ret = 0;
exit:

	mutex_unlock(&d->rw_ctrl_mutex);
	TOUCH_D(ABS, "read message end\n");

	return ret;
}

int s3618_write(struct device *dev, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = size,
			.buf = data,
		},
	};

	int ret = 0;
	// DEBUG
	int i = 0;
	u8 *tmp = (u8*)data;

	// DEBUG
	for (i = 0; i < size; i++) {
		TOUCH_D(GET_DATA, "%s - data[%d]=0x%x\n", __func__, i, tmp[i]);
	}

	mutex_lock(&d->io_mutex);

#if defined(CONFIG_SECURE_TOUCH)
	if (atomic_read(&ts->st_enabled)) {
		TOUCH_E("cannot use i2c, ownership changed!\n");
		ret = -EINVAL;
		goto exit;
	}
#endif

	if (size > CHUNK_SIZE) {
		TOUCH_I("s3618 write size overflow!!!");
		ret = -ENOMEM;
		goto exit;
	}

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		goto exit;
	}

exit:
	mutex_unlock(&d->io_mutex);

	return ret;
}

int s3618_write_message(struct device *dev, u8 command, u8 *payload, u32 length,
		u8 *response_buffer, u32 *response_length)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;
	u32 idx = 0, chunks = 0, chunk_space = 0, xfer_length = 0, remaining_length = 0;

	// TODO response buf null check, romboot, hdl, polling remove

	mutex_lock(&d->command_mutex);
	mutex_lock(&d->rw_ctrl_mutex);
	atomic_set(&d->command_status, CMD_BUSY);

	reinit_completion(&d->response_complete);

	d->command = command;
	TOUCH_D(ABS, "Command=0x%02x, Payload=%s, Length=%d\n",
			command, (payload!=NULL)?"Exist":"None", length);

	// adding two length bytes as part of payload
	remaining_length = length + 2;

	// available chunk space for payload = total chunk size minus command byte
	if (d->chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = d->chunk_size - 1;

	chunks = ceil_div(remaining_length, chunk_space);
	chunks = (chunks == 0) ? 1 : chunks;

	mutex_lock(&d->out.buf_mutex);
	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		memset(d->out.buf, 0x00, sizeof(d->out.buf));
		d->out.data_length = 0;

		if (idx == 0) {
			d->out.buf[0] = command;
			d->out.buf[1] = (u8)length;
			d->out.buf[2] = (u8)(length >> 8);

			if ((xfer_length > 2) && (payload != NULL)) {
				memcpy(&d->out.buf[3], payload, xfer_length-2);
			}
		} else {
			d->out.buf[0] = CMD_CONTINUE_WRITE;

			if (payload != NULL)
				memcpy(&d->out.buf[1], &payload[idx * chunk_space - 2], xfer_length);
		}

		ret = s3618_write(dev, d->out.buf, xfer_length + 1);
		if (ret < 0) {
			TOUCH_E("Failed to write to device\n");
			mutex_unlock(&d->out.buf_mutex);
			mutex_unlock(&d->rw_ctrl_mutex);
			goto exit;
		}

		remaining_length -= xfer_length;

		if (chunks > 1)
			touch_msleep(WRITE_DELAY);
	}
	mutex_unlock(&d->out.buf_mutex);
	mutex_unlock(&d->rw_ctrl_mutex);

	if (command == CMD_GET_BOOT_INFO ||
			command == CMD_GET_APPLICATION_INFO ||
			command == CMD_READ_FLASH ||
			command == CMD_WRITE_FLASH ||
			command == CMD_ERASE_FLASH ||
			command == CMD_PRODUCTION_TEST) {
		ret = wait_for_completion_timeout(&d->response_complete, msecs_to_jiffies(RESPONSE_TIMEOUT_LONG));
	} else {
		ret = wait_for_completion_timeout(&d->response_complete, msecs_to_jiffies(RESPONSE_TIMEOUT));
	}
	if (ret == 0) {
		TOUCH_E("Time out waiting for response (command 0x%02x)\n", d->command);
		s3618_reset_ctrl(dev, HW_RESET);
		ret = -ETIME;
		goto exit;
	}

	if(atomic_read(&d->command_status) != CMD_IDLE) {
		TOUCH_E("command_status is not CMD_IDLE (command 0x%02x, command_status 0x%02x)\n",
				d->command, atomic_read(&d->command_status));
		ret = -EIO;
		goto exit;
	}

	mutex_lock(&d->response.buf_mutex);
	if (d->response_code != STATUS_OK) {
		if (d->response.data_length) {
			TOUCH_E("Error code = 0x%02x (command 0x%02x)\n",
					d->status_report_code, d->command);
		}
		ret = -EIO;
	} else {
		ret = 0;
	}

	if (response_buffer != NULL && response_buffer != d->response.buf
			&& response_length != NULL) {
		memcpy(&response_buffer[0], &d->response.buf[0], d->response.data_length);
		*response_length = d->response.data_length;
	}
	mutex_unlock(&d->response.buf_mutex);

exit:
	TOUCH_D(ABS, "write_message end\n");
	d->command = CMD_NONE;
	atomic_set(&d->command_status, CMD_IDLE);
	mutex_unlock(&d->command_mutex);

	return ret;
}

int s3618_identify(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = s3618_write_message(dev,
			CMD_IDENTIFY,
			NULL,
			0,
			NULL,
			NULL);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n",
				STR(CMD_IDENTIFY));
		goto error;
	}
	TOUCH_I("Response: %d / Write Command: %s \n",
			d->response_code,
			STR(CMD_IDENTIFY));

error:
	return 0;
}

static int s3618_set_report_config(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;
	u32 idx = 0;
	u8 *report_config_buf = NULL;
	int report_config_size = 0;

	TOUCH_TRACE();

#if defined(USE_DEFAULT_TOUCH_REPORT_CONFIG)
	TOUCH_I("%s - Skip, Use default touch report config\n", __func__);
	return 0;
#endif

	report_config_size = d->app_info.max_touch_report_config_size;
	if (report_config_size < TOUCH_REPORT_CONFIG_SIZE) {
		TOUCH_E("Invalid maximum touch report config size\n");
		return -EINVAL;
	}

	report_config_buf = kzalloc(report_config_size, GFP_KERNEL);
	if (!report_config_buf) {
		TOUCH_E("failed to allocate report_config_buf\n");
		ret = -ENOMEM;
		goto error;
	}

	idx = 0;
	report_config_buf[idx] = TOUCH_CUSTOMER_GESTURE_DETECTED;
	idx++;
	report_config_buf[idx] = 0x08; // 8bit
	idx++;
	report_config_buf[idx] = TOUCH_CUSTOMER_GESTURE_INFO;
	idx++;
	report_config_buf[idx] = 0xA0; // 160bit
	idx++;
	report_config_buf[idx] = TOUCH_CUSTOMER_GESTURE_INFO2;
	idx++;
	report_config_buf[idx] = 0xA0; // 160bit
	idx++;
	report_config_buf[idx] = TOUCH_PALM_DETECTED;
	idx++;
	report_config_buf[idx] = 0x10; // 16bit
	idx++;
	report_config_buf[idx] = TOUCH_BASELINE_ERR_LOG;
	idx++;
	report_config_buf[idx] = 0x10; // 16bit
	idx++;
	report_config_buf[idx] = TOUCH_POWER_IM;
	idx++;
	report_config_buf[idx] = 0x10; // 16bit
	idx++;
	report_config_buf[idx] = TOUCH_NSM_STATE;
	idx++;
	report_config_buf[idx] = 0x08; // 8bit
	idx++;
	report_config_buf[idx] = TOUCH_NUM_OF_ACTIVE_OBJECTS;
	idx++;
	report_config_buf[idx] = 0x08; // 8bit
	idx++;
	report_config_buf[idx] = TOUCH_FOREACH_ACTIVE_OBJECT;
	idx++;
	report_config_buf[idx] = TOUCH_OBJECT_N_INDEX;
	idx++;
	report_config_buf[idx] = 0x04; // 4bit
	idx++;
	report_config_buf[idx] = TOUCH_OBJECT_N_CLASSIFICATION;
	idx++;
	report_config_buf[idx] = 0x04; // 4bit
	idx++;
	report_config_buf[idx] = TOUCH_OBJECT_N_X_POSITION;
	idx++;
	report_config_buf[idx] = 0x0C; // 12bit
	idx++;
	report_config_buf[idx] = TOUCH_OBJECT_N_Y_POSITION;
	idx++;
	report_config_buf[idx] = 0x0C; // 12bit
	idx++;
	report_config_buf[idx] = TOUCH_OBJECT_N_Z;
	idx++;
	report_config_buf[idx] = 0x08; // 8bit
	idx++;
	report_config_buf[idx] = TOUCH_OBJECT_N_ANGLE;
	idx++;
	report_config_buf[idx] = 0x08; // 8bit
	idx++;
	report_config_buf[idx] = TOUCH_OBJECT_N_MAJOR;
	idx++;
	report_config_buf[idx] = 0x10; // 16bit
	idx++;
	report_config_buf[idx] = TOUCH_OBJECT_N_MINOR;
	idx++;
	report_config_buf[idx] = 0x10; // 16bit
	idx++;
	report_config_buf[idx] = TOUCH_FOREACH_END;
	idx++;
	report_config_buf[idx] = TOUCH_END;
	idx++;

	ret = s3618_write_message(dev,
			CMD_SET_TOUCH_REPORT_CONFIG,
			report_config_buf,
			report_config_size,
			d->response.buf,
			&d->response.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n",
				STR(CMD_SET_TOUCH_REPORT_CONFIG));
		mutex_unlock(&d->response.buf_mutex);
		kfree(report_config_buf);
		return ret;
	}
	TOUCH_I("Response: %d / Write Command: %s \n",
			d->response_code,
			STR(CMD_SET_TOUCH_REPORT_CONFIG));

error:
	kfree(report_config_buf);

	return ret;
}

static int s3618_get_report_config(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	mutex_lock(&d->config.buf_mutex);

	ret = s3618_write_message(dev,
			CMD_GET_TOUCH_REPORT_CONFIG,
			NULL,
			0,
			d->config.buf,
			&d->config.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n",
				STR(CMD_GET_TOUCH_REPORT_CONFIG));
		mutex_unlock(&d->config.buf_mutex);
		return ret;
	}
	TOUCH_I("Response: %d / Write Command: %s \n",
			d->response_code,
			STR(CMD_GET_TOUCH_REPORT_CONFIG));

	mutex_unlock(&d->config.buf_mutex);

	return 0;
}

int s3618_set_dynamic_config(struct device *dev, u8 dynamic_config_id, u16 value)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;
	u8 payload[3] = {0,};

	TOUCH_TRACE();

	payload[0] = dynamic_config_id;
	payload[1] = (u8)value;
	payload[2] = (u8)(value >> 8);

	ret = s3618_write_message(dev,
			CMD_SET_DYNAMIC_CONFIG,
			payload,
			sizeof(payload),
			NULL,
			NULL);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n",
				STR(CMD_SET_DYNAMIC_CONFIG));
		return ret;
	}
	TOUCH_I("Response: %d / Write Command: %s \n",
			d->response_code,
			STR(CMD_SET_DYNAMIC_CONFIG));

	return 0;
}

static int s3618_sleep_control(struct device *dev, int enable)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;
	u8 command = 0;

	TOUCH_TRACE();

	if (enable) {
		command = CMD_ENTER_DEEP_SLEEP;
		atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
	} else {
		command = CMD_EXIT_DEEP_SLEEP;
		atomic_set(&ts->state.sleep, IC_NORMAL);
	}

	ret = s3618_write_message(dev,
			command,
			NULL,
			0,
			NULL,
			NULL);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n",
				enable ?
				STR(CMD_ENTER_DEEP_SLEEP):
				STR(CMD_EXIT_DEEP_SLEEP));
		return ret;
	}
	TOUCH_I("Response: %d / Write Command: %s \n",
			d->response_code,
			enable ?
			STR(CMD_ENTER_DEEP_SLEEP):
			STR(CMD_EXIT_DEEP_SLEEP));
	TOUCH_I("%s - %s\n", __func__, enable ? "IC_DEEP_SLEEP" : "IC_NORMAL");

	return 0;
}
/*
static int s3618_report_control(struct device *dev, int enable, int report_type)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;
	u8 command = 0;
	u8 payload = 0;

	TOUCH_TRACE();

	if (enable) {
		command = CMD_ENABLE_REPORT;
		payload = report_type;
	} else {
		command = CMD_DISABLE_REPORT;
		payload = report_type;
	}

	ret = s3618_write_message(dev,
			command,
			&payload,
			sizeof(payload),
			NULL,
			NULL);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n",
				enable ?
				STR(CMD_ENABLE_REPORT):
				STR(CMD_DISABLE_REPORT));
		return ret;
	}
	TOUCH_I("Response: %d / Write Command: %s \n",
			d->response_code,
			enable ?
			STR(CMD_ENABLE_REPORT):
			STR(CMD_DISABLE_REPORT));
	TOUCH_I("%s - %s\n", __func__, enable ? "REPORT_ENABLE" : "REPORT_DISABLE");

	return 0;
}
*/
void s3618_reset_ctrl(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	switch (ctrl) {
	case SW_RESET:
		TOUCH_I("%s : SW Reset\n", __func__);
		atomic_set(&d->state.init, IC_INIT_NEED);
		ret = s3618_write_message(dev,
				CMD_RESET,
				NULL,
				0,
				NULL,
				NULL);
		if (ret < 0)
			TOUCH_E("Failed to write command %s\n", STR(CMD_RESET));
		touch_msleep(ts->caps.sw_reset_delay);
		break;
	case HW_RESET:
		TOUCH_I("%s : HW Reset\n", __func__);
		atomic_set(&d->state.power, POWER_OFF);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(10);
		atomic_set(&d->state.power, POWER_SLEEP);
		atomic_set(&d->state.init, IC_INIT_NEED);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(ts->caps.hw_reset_delay);
		break;
	case SW_RESET_NO_INIT:
		TOUCH_I("%s : SW Reset No Init\n", __func__);
		atomic_set(&d->state.init, IC_INIT_NEED);
		ret = s3618_write_message(dev,
				CMD_RESET,
				NULL,
				0,
				NULL,
				NULL);
		if (ret < 0)
			TOUCH_E("Failed to write command %s\n", STR(CMD_RESET));
		touch_msleep(ts->caps.sw_reset_delay);
		return;
		break;
	default:
		break;
	}

	mod_delayed_work(ts->wq, &ts->init_work, 0);
}

static int s3618_tci_enable(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	struct tci_info *info = NULL;
	struct s3618_tci_buf buf = {0, };
	u8 gesture_config_code[2] = {CONFIG_CODE_KNOCK_ON, CONFIG_CODE_KNOCK_CODE};
	int ret = 0;
	int i = 0;

	TOUCH_TRACE();

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("%s : the function is skipped, because IC_DEEP_SLEEP\n", __func__);
		return 0;
	}

	// Do not skip "d->lcd_mode == LCD_MODE_U3" condition for lpwg_mode partial settings

	if (d->isDeviceOpen) {
		TOUCH_I("%s : the function is skipped, because DS6 Connected\n", __func__);
		return 0;
	}

	TOUCH_I("%s: TCI_MODE(%x)\n", __func__, ts->tci.mode);

	for (i = 0; i < 2; i++) {
		if ((ts->tci.mode & (1 << i)) == 0x0) {
			buf.gesture_config_code = gesture_config_code[i];
			buf.data.enable = 0;
			ret = s3618_write_message(dev,
					CMD_SET_LGE_GESTURE_CONFIG,
					(u8*)&buf,
					sizeof(buf),
					NULL,
					NULL);
			if (ret < 0) {
				TOUCH_E("Failed to write command %s\n",
						STR(CMD_SET_LGE_GESTURE_CONFIG));
				return ret;
			}
			TOUCH_I("Response: %d / Write Command: %s / Config Code: %s\n",
					d->response_code,
					STR(CMD_SET_LGE_GESTURE_CONFIG),
					s3618_gesture_config_code_str[gesture_config_code[i]]);
		} else {
			info = &ts->tci.info[i];

			buf.gesture_config_code = gesture_config_code[i];
			buf.data.enable= 1; // Enable
			buf.data.tap_count = info->tap_count;
			buf.data.min_intertap = info->min_intertap;
			buf.data.max_intertap = info->max_intertap;
			buf.data.touch_slop = info->touch_slop;
			buf.data.tap_distance = info->tap_distance;
			buf.data.enable_delay = info->intr_delay ? 1 : 0; // Enable
			buf.data.intr_delay = info->intr_delay;

			ret = s3618_write_message(dev,
					CMD_SET_LGE_GESTURE_CONFIG,
					(u8*)&buf,
					sizeof(buf),
					NULL,
					NULL);
			if (ret < 0) {
				TOUCH_E("Failed to write command %s\n",
						STR(CMD_SET_LGE_GESTURE_CONFIG));
				return ret;
			}
			TOUCH_I("Response: %d / Write Command: %s / Config Code: %s\n",
					d->response_code,
					STR(CMD_SET_LGE_GESTURE_CONFIG),
					s3618_gesture_config_code_str[gesture_config_code[i]]);
		}
	}

	ret = s3618_tci_active_area(dev, 0 + LPWG_SENSELESS_AREA_W,
					0 + LPWG_SENSELESS_AREA_W,
					ts->caps.max_x - LPWG_SENSELESS_AREA_W,
					ts->caps.max_y - LPWG_SENSELESS_AREA_W);
	if (ret < 0)
		TOUCH_E("failed to set tci_active_area (ret: %d)\n", ret);

	return ret;
}

static int s3618_swipe_enable(struct device *dev, bool enable)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	struct swipe_ctrl *ctrl[S3618_SWIPE_NUM] = {	/* U, L, R, L2, R2 */
		&ts->swipe[SWIPE_U],
		&ts->swipe[SWIPE_L],
		&ts->swipe[SWIPE_R],
		&ts->swipe[SWIPE_L2],
		&ts->swipe[SWIPE_R2],
	};
	u8 gesture_config_code[S3618_SWIPE_NUM] = {
		CONFIG_CODE_SWIPE_UP,
		CONFIG_CODE_SWIPE_LEFT,
		CONFIG_CODE_SWIPE_RIGHT,
		CONFIG_CODE_SWIPE_LEFT2,
		CONFIG_CODE_SWIPE_RIGHT2,
	};
	struct s3618_swipe_buf buf = {0, };
	int i = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("%s : the function is skipped, because IC_DEEP_SLEEP\n", __func__);
		return 0;
	}

	if (d->lcd_mode == LCD_MODE_U3) {
		TOUCH_I("%s : the function is skipped, because LCD_MODE_U3\n", __func__);
		return 0;
	}

	if (d->isDeviceOpen) {
		TOUCH_I("%s : the function is skipped, because DS6 Connected\n", __func__);
		return 0;
	}

	TOUCH_I("%s: Enable: %d SWIPE U(%d),L(%d),R(%d),L2(%d),R2(%d)\n", __func__,
			enable,
			ctrl[S3618_SWIPE_U]->enable,
			ctrl[S3618_SWIPE_L]->enable,
			ctrl[S3618_SWIPE_R]->enable,
			ctrl[S3618_SWIPE_L2]->enable,
			ctrl[S3618_SWIPE_R2]->enable);

	for (i = 0; i < S3618_SWIPE_NUM; i++) {	/* U, D, L, R */
		if (enable) {
			memset(&buf, 0, sizeof(buf));

			buf.gesture_config_code = gesture_config_code[i];
			buf.data.enable = ctrl[i]->enable;
			buf.data.distance = ctrl[i]->distance;
			buf.data.ratio_thres = ctrl[i]->ratio_thres;
			buf.data.min_time = ctrl[i]->min_time;
			buf.data.max_time = ctrl[i]->max_time;
			buf.data.wrong_dir_thres = ctrl[i]->wrong_dir_thres;
			buf.data.init_ratio_chk_dist = ctrl[i]->init_ratio_chk_dist;
			buf.data.init_ratio_thres = ctrl[i]->init_ratio_thres;

			buf.data.area.x1 = ctrl[i]->area.x1 - ctrl[i]->border_area.x1;
			if (buf.data.area.x1 < 0)
				buf.data.area.x1 = 0;
			buf.data.area.y1 = ctrl[i]->area.y1 - ctrl[i]->border_area.y1;
			if (buf.data.area.y1 < 0)
				buf.data.area.y1 = 0;
			buf.data.area.x2 = ctrl[i]->area.x2 + ctrl[i]->border_area.x2;
			if (buf.data.area.x2 > ts->caps.max_x)
				buf.data.area.x2 = ts->caps.max_x;
			buf.data.area.y2 = ctrl[i]->area.y2 + ctrl[i]->border_area.y2;
			if (buf.data.area.y2 > ts->caps.max_y)
				buf.data.area.y2 = ts->caps.max_y;

			buf.data.start_area.x1 = ctrl[i]->start_area.x1 - ctrl[i]->start_border_area.x1;
			if (buf.data.start_area.x1 < 0)
				buf.data.start_area.x1 = 0;
			buf.data.start_area.y1 = ctrl[i]->start_area.y1 - ctrl[i]->start_border_area.y1;
			if (buf.data.start_area.y1 < 0)
				buf.data.start_area.y1 = 0;
			buf.data.start_area.x2 = ctrl[i]->start_area.x2 + ctrl[i]->start_border_area.x2;
			if (buf.data.start_area.x2 > ts->caps.max_x)
				buf.data.start_area.x2 = ts->caps.max_x;
			buf.data.start_area.y2 = ctrl[i]->start_area.y2 + ctrl[i]->start_border_area.y2;
			if (buf.data.start_area.y2 > ts->caps.max_y)
				buf.data.start_area.y2 = ts->caps.max_y;

		} else {
			memset(&buf, 0, sizeof(buf));

			buf.gesture_config_code = gesture_config_code[i];
			buf.data.enable = ctrl[i]->enable;

			ctrl[i]->debug_enable = false;
			ctrl[i]->debug_enable = ctrl[i]->enable;
		}

		ret = s3618_write_message(dev,
				CMD_SET_LGE_GESTURE_CONFIG,
				(u8*)&buf,
				sizeof(buf),
				NULL,
				NULL);
		if (ret < 0) {
			TOUCH_E("Failed to write command %s\n",
					STR(CMD_SET_LGE_GESTURE_CONFIG));
			return ret;
		}
		TOUCH_I("Response: %d / Write Command: %s / Config Code: %s\n",
				d->response_code,
				STR(CMD_SET_LGE_GESTURE_CONFIG),
				s3618_gesture_config_code_str[gesture_config_code[i]]);
	}

	return ret;
}

static int s3618_lpwg_abs_enable(struct device *dev, bool enable)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	struct s3618_lpwg_abs_buf buf = {0, };
	int ret = 0;

	TOUCH_TRACE();

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("%s : the function is skipped, because IC_DEEP_SLEEP\n", __func__);
		return 0;
	}

	if (d->lcd_mode == LCD_MODE_U3) {
		TOUCH_I("%s : the function is skipped, because LCD_MODE_U3\n", __func__);
		return 0;
	}

	if (d->isDeviceOpen) {
		TOUCH_I("%s : the function is skipped, because DS6 Connected\n", __func__);
		return 0;
	}

	TOUCH_I("%s: %s\n", __func__,
			(enable ? "enable" : "disable"));

	if (enable) {
		buf.gesture_config_code = CONFIG_CODE_LPWG_ABS;
		buf.enable = 1;

		buf.area.x1 = d->lpwg_abs.area.x1 - d->lpwg_abs.border_area.x1;
		if (buf.area.x1 < 0)
			buf.area.x1 = 0;
		buf.area.y1 = d->lpwg_abs.area.y1 - d->lpwg_abs.border_area.y1;
		if (buf.area.y1 < 0)
			buf.area.y1 = 0;
		buf.area.x2 = d->lpwg_abs.area.x2 + d->lpwg_abs.border_area.x2;
		if (buf.area.x2 > ts->caps.max_x)
			buf.area.x2 = ts->caps.max_x;
		buf.area.y2 = d->lpwg_abs.area.y2 + d->lpwg_abs.border_area.y2;
		if (buf.area.y2 > ts->caps.max_y)
			buf.area.y2 = ts->caps.max_y;

		TOUCH_I("%s: lpwg_abs_active_area(%d,%d)(%d,%d)\n", __func__,
			buf.area.x1, buf.area.y1,
			buf.area.x2, buf.area.y2);
	} else {
		buf.gesture_config_code = CONFIG_CODE_LPWG_ABS;
		buf.enable = 0;
	}

	ret = s3618_write_message(dev,
			CMD_SET_LGE_GESTURE_CONFIG,
			(u8*)&buf,
			sizeof(buf),
			NULL,
			NULL);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n",
				STR(CMD_SET_LGE_GESTURE_CONFIG));
		goto error;
	}
	TOUCH_I("Response: %d / Write Command: %s / Config Code: %s\n",
			d->response_code,
			STR(CMD_SET_LGE_GESTURE_CONFIG),
			s3618_gesture_config_code_str[buf.gesture_config_code]);

	touch_report_all_event(ts);
	ts->tcount = 0;

error:
	return ret;

}

static int s3618_ai_pick_enable(struct device *dev, bool enable)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	int ret = 0;

	TOUCH_TRACE();

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("%s : the function is skipped, because IC_DEEP_SLEEP\n", __func__);
		return 0;
	}

	if (d->lcd_mode == LCD_MODE_U3) {
		TOUCH_I("%s : the function is skipped, because LCD_MODE_U3\n", __func__);
		return 0;
	}

	if (d->isDeviceOpen) {
		TOUCH_I("%s : the function is skipped, because DS6 Connected\n", __func__);
		return 0;
	}

	TOUCH_I("%s: %s\n", __func__,
			(enable ? "enable" : "disable"));

	if (enable) {
		if (ts->lpwg.mode == LPWG_NONE) {
			TOUCH_I("%s: enable ai_pick gesture & area (lpwg_mode : %d)\n",
					__func__, ts->lpwg.mode);

			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
				TOUCH_I("%s: force wake IC by ai_pick\n", __func__);
				s3618_sleep_control(dev, IC_NORMAL);
			}

			ts->tci.mode = 0x01;
			info1->intr_delay = 0;
			info1->tap_distance = 10;

			ret = s3618_tci_enable(dev);
			if (ret < 0) {
				TOUCH_E("failed to set ai_pick gesture (ret: %d)\n", ret);
				goto error;
			}

			ret = s3618_tci_active_area(dev, d->ai_pick.total_area.x1,
					d->ai_pick.total_area.y1,
					d->ai_pick.total_area.x2,
					d->ai_pick.total_area.y2);
			if (ret < 0) {
				TOUCH_E("failed to set ai_pick gesture active_area (ret: %d)\n", ret);
				goto error;
			}

		} else if (ts->lpwg.mode == LPWG_PASSWORD_ONLY) {
			TOUCH_I("%s: enable ai_pick gesture & area (lpwg_mode : %d)\n",
					__func__, ts->lpwg.mode);

			ts->tci.mode = 0x03;
			info1->intr_delay = ts->tci.double_tap_check ? 68 : 0;
			info1->tap_distance = 7;

			ret = s3618_tci_enable(dev);
			if (ret < 0) {
				TOUCH_E("failed to set ai_pick gesture (ret: %d)\n", ret);
				goto error;
			}
		} else {
			TOUCH_I("%s: no need to modify lpwg setting (lpwg_mode : %d)\n",
					__func__, ts->lpwg.mode);
		}
	} else {
		if (ts->lpwg.mode == LPWG_NONE) {
			TOUCH_I("%s: restore lpwg setting (lpwg_mode : %d)\n",
					__func__, ts->lpwg.mode);

			ts->tci.mode = 0x0;

			ret = s3618_tci_enable(dev);
			if (ret < 0) {
				TOUCH_E("failed to set tci_enable (ret: %d)\n", ret);
				goto error;
			}
		} else if (ts->lpwg.mode == LPWG_PASSWORD_ONLY) {
			ts->tci.mode = 0x02;
			info1->intr_delay = 0;
			info1->tap_distance = 10;

			ret = s3618_tci_enable(dev);
			if (ret < 0) {
				TOUCH_E("failed to set LPWG_PASSWORD_ONLY (ret: %d)\n", ret);
				goto error;
			}
		} else {
			TOUCH_I("%s: no need to modify lpwg setting (lpwg_mode : %d)\n",
					__func__, ts->lpwg.mode);
		}
	}

error:
	return ret;
}

static int s3618_lpwg_longpress_enable(struct device *dev, int enable)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	struct s3618_lpwg_longpress_buf buf = {0, };
	int ret = 0;

	TOUCH_TRACE();

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("%s : the function is skipped, because IC_DEEP_SLEEP\n", __func__);
		return 0;
	}

	// Do not skip "d->lcd_mode == LCD_MODE_U3" condition for lpwg_mode partial settings

	if (d->isDeviceOpen) {
		TOUCH_I("%s : the function is skipped, because DS6 Connected\n", __func__);
		return 0;
	}

	TOUCH_I("%s: %s\n", __func__,
			(enable ? "enable" : "disable"));

	if (enable) {
		buf.gesture_config_code = CONFIG_CODE_LONG_PRESS;
		buf.enable = 1;
		buf.touch_slop = d->lpwg_longpress.touch_slop;
		buf.min_press_time = d->lpwg_longpress.min_press_time;
		buf.min_contact_size = d->lpwg_longpress.min_contact_size;

#if defined(USE_LONGPRESS_AROUND)
		buf.area.x1 = d->lpwg_longpress.around_area.x1;
		if (buf.area.x1 < 0)
			buf.area.x1 = 0;
		buf.area.y1 = d->lpwg_longpress.around_area.y1;
		if (buf.area.y1 < 0)
			buf.area.y1 = 0;
		buf.area.x2 = d->lpwg_longpress.around_area.x2;
		if (buf.area.x2 > ts->caps.max_x)
			buf.area.x2 = ts->caps.max_x;
		buf.area.y2 = d->lpwg_longpress.around_area.y2;
		if (buf.area.y2 > ts->caps.max_y)
			buf.area.y2 = ts->caps.max_y;
#else
		buf.area.x1 = d->lpwg_longpress.area.x1;
		if (buf.area.x1 < 0)
			buf.area.x1 = 0;
		buf.area.y1 = d->lpwg_longpress.area.y1;
		if (buf.area.y1 < 0)
			buf.area.y1 = 0;
		buf.area.x2 = d->lpwg_longpress.area.x2;
		if (buf.area.x2 > ts->caps.max_x)
			buf.area.x2 = ts->caps.max_x;
		buf.area.y2 = d->lpwg_longpress.area.y2;
		if (buf.area.y2 > ts->caps.max_y)
			buf.area.y2 = ts->caps.max_y;
#endif

		TOUCH_I("%s: x1: %d, y1: %d, x2: %d, y2: %d, min_press_time: %d, min_contact_size: %d, slop: %d,", __func__,
				buf.area.x1, buf.area.y1,
				buf.area.x2, buf.area.y2,
				buf.min_press_time, buf.min_contact_size, buf.touch_slop);
	} else {
		buf.gesture_config_code = CONFIG_CODE_LONG_PRESS;
		buf.enable = 0;
	}

	ret = s3618_write_message(dev,
			CMD_SET_LGE_GESTURE_CONFIG,
			(u8*)&buf,
			sizeof(buf),
			NULL,
			NULL);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n",
				STR(CMD_SET_LGE_GESTURE_CONFIG));
		goto error;
	}
	TOUCH_I("Response: %d / Write Command: %s / Config Code: %s\n",
			d->response_code,
			STR(CMD_SET_LGE_GESTURE_CONFIG),
			s3618_gesture_config_code_str[buf.gesture_config_code]);

error:
	return ret;
}

static int s3618_lpwg_onetap_enable(struct device *dev, bool enable)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	struct s3618_lpwg_onetap_buf buf = {0, };
	int ret = 0;

	TOUCH_TRACE();

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("%s : the function is skipped, because IC_DEEP_SLEEP\n", __func__);
		return 0;
	}

	// Do not skip "d->lcd_mode == LCD_MODE_U3" condition for lpwg_mode partial settings

	if (d->isDeviceOpen) {
		TOUCH_I("%s : the function is skipped, because DS6 Connected\n", __func__);
		return 0;
	}

	TOUCH_I("%s: %s\n", __func__,
			(enable ? "enable" : "disable"));

	if (enable) {
		buf.gesture_config_code = CONFIG_CODE_ONE_TAP;
		buf.enable = 1;

		buf.area.x1 = d->lpwg_onetap.area.x1;
		if (buf.area.x1 < 0 + LPWG_SENSELESS_AREA_W)
			buf.area.x1 = 0 + LPWG_SENSELESS_AREA_W;
		buf.area.y1 = d->lpwg_onetap.area.y1;
		if (buf.area.y1 < 0 + LPWG_SENSELESS_AREA_W)
			buf.area.y1 = 0 + LPWG_SENSELESS_AREA_W;
		buf.area.x2 = d->lpwg_onetap.area.x2;
		if (buf.area.x2 > ts->caps.max_x - LPWG_SENSELESS_AREA_W)
			buf.area.x2 = ts->caps.max_x - LPWG_SENSELESS_AREA_W;
		buf.area.y2 = d->lpwg_onetap.area.y2;
		if (buf.area.y2 > ts->caps.max_y - LPWG_SENSELESS_AREA_W)
			buf.area.y2 = ts->caps.max_y - LPWG_SENSELESS_AREA_W;

		buf.touch_slop = d->lpwg_onetap.touch_slop;
		buf.max_press_time = d->lpwg_onetap.max_press_time;
		buf.interrupt_delay_time = d->lpwg_onetap.interrupt_delay_time;

		TOUCH_I("%s: x1: %d, y1: %d, x2: %d, y2: %d, slop: %d, max_press_time: %d, interrupt_delay_time: %d", __func__,
				buf.area.x1, buf.area.y1,
				buf.area.x2, buf.area.y2,
				buf.touch_slop, buf.max_press_time, buf.interrupt_delay_time);
	} else {
		buf.gesture_config_code = CONFIG_CODE_ONE_TAP;
		buf.enable = 0;
	}

	ret = s3618_write_message(dev,
			CMD_SET_LGE_GESTURE_CONFIG,
			(u8*)&buf,
			sizeof(buf),
			NULL,
			NULL);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n",
				STR(CMD_SET_LGE_GESTURE_CONFIG));
		goto error;
	}
	TOUCH_I("Response: %d / Write Command: %s / Config Code: %s\n",
			d->response_code,
			STR(CMD_SET_LGE_GESTURE_CONFIG),
			s3618_gesture_config_code_str[buf.gesture_config_code]);

error:
	return ret;
}

static int s3618_tci_active_area(struct device *dev, u16 x1, u16 y1, u16 x2, u16 y2)
{
	struct s3618_data *d = to_s3618_data(dev);
	struct s3618_tci_active_area_buf buf = {0, };
	int ret = 0;

	TOUCH_TRACE();

	buf.gesture_config_code = CONFIG_CODE_TCI_ACTIVE_AREA;
	buf.area.x1 = x1;
	buf.area.y1 = y1;
	buf.area.x2 = x2;
	buf.area.y2 = y2;

	TOUCH_D(TRACE, "%s: x1[%d], y1[%d], x2[%d], y2[%d]\n", __func__,
			buf.area.x1, buf.area.y1, buf.area.x2, buf.area.y2);

	/*
	if (ts->lpwg.qcover == HALL_NEAR)
		memset(&active_area, 0, sizeof(active_area));
	*/

	ret = s3618_write_message(dev,
			CMD_SET_LGE_GESTURE_CONFIG,
			(u8*)&buf,
			sizeof(buf),
			NULL,
			NULL);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n",
				STR(CMD_SET_LGE_GESTURE_CONFIG));
		return ret;
	}
	TOUCH_I("Response: %d / Write Command: %s / Config Code: %s\n",
			d->response_code,
			STR(CMD_SET_LGE_GESTURE_CONFIG),
			s3618_gesture_config_code_str[buf.gesture_config_code]);

	return ret;
}

static int s3618_lpwg_control(struct device *dev,
		int tci_mode, int swipe_enable,
		int lpwg_abs_enable, int ai_pick_enable,
		int lpwg_longpress_enable, int lpwg_onetap_enable)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];

	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s - tci_mode=%d, swipe=%d lpwg_abs=%d, ai_pick=%d, lpwg_longpress=%d, lpwg_onetap=%d\n", __func__,
			tci_mode, swipe_enable,
			lpwg_abs_enable, ai_pick_enable,
			lpwg_longpress_enable, lpwg_onetap_enable);


	// LPWG Setting 1 : Knock on/code setting
	switch (tci_mode) {
		case LPWG_NONE:
			ts->tci.mode = 0x00;
			ret = s3618_tci_enable(dev);
			if (ret < 0) {
				TOUCH_E("failed to set LPWG_NONE (ret: %d)\n", ret);
				goto error;
			}
			break;

		case LPWG_DOUBLE_TAP:
			ts->tci.mode = 0x01;
			info1->intr_delay = 0;
			info1->tap_distance = 10;

			ret = s3618_tci_enable(dev);
			if (ret < 0) {
				TOUCH_E("failed to set LPWG_DOUBLE_TAP (ret: %d)\n", ret);
				goto error;
			}
			break;

		case LPWG_PASSWORD:
			ts->tci.mode = 0x03;
			info1->intr_delay = ts->tci.double_tap_check ? 68 : 0;
			info1->tap_distance = 7;

			ret = s3618_tci_enable(dev);
			if (ret < 0) {
				TOUCH_E("failed to set LPWG_PASSWORD (ret: %d)\n", ret);
				goto error;
			}
			break;

		case LPWG_PASSWORD_ONLY:
			ts->tci.mode = 0x02;
			info1->intr_delay = 0;
			info1->tap_distance = 10;

			ret = s3618_tci_enable(dev);
			if (ret < 0) {
				TOUCH_E("failed to set LPWG_PASSWORD_ONLY (ret: %d)\n", ret);
				goto error;
			}
			break;

		default:
			TOUCH_E("Unknown lpwg control case = %d\n", tci_mode);
			break;
	}

	// LPWG Control 2 : Swipe setting
	ret = s3618_swipe_enable(dev, swipe_enable);
	if (ret < 0) {
		TOUCH_E("failed to set swipe enable (ret: %d)\n", ret);
		goto error;
	}

	// LPWG Control 3 : Lpwg abs setting
	ret = s3618_lpwg_abs_enable(dev, lpwg_abs_enable);
	if (ret < 0) {
		TOUCH_E("failed to set lpwg abs enable (ret: %d)\n", ret);
		goto error;
	}

	// LPWG Control 4 : Ai pick setting
	ret = s3618_ai_pick_enable(dev, ai_pick_enable);
	if (ret < 0) {
		TOUCH_E("failed to ai_pick enable (ret: %d)\n", ret);
		goto error;
	}

	// LPWG Control 5 : lpwg_longpress setting
	ret = s3618_lpwg_longpress_enable(dev, lpwg_longpress_enable);
	if (ret < 0) {
		TOUCH_E("failed to lpwg longpress enable (ret: %d)\n", ret);
		goto error;
	}

	// LPWG Control 6 : lpwg_onetap setting
	ret = s3618_lpwg_onetap_enable(dev, lpwg_onetap_enable);
	if (ret < 0) {
		TOUCH_E("failed to lpwg_onetap enable (ret: %d)\n", ret);
		goto error;
	}


	if (tci_mode || swipe_enable || lpwg_abs_enable || ai_pick_enable || lpwg_longpress_enable || lpwg_onetap_enable) {
		ret = s3618_set_dynamic_config(dev, DC_IN_WAKEUP_GESTURE_MODE, 1);
		if (ret < 0) {
			TOUCH_E("failed to set %s Enable (ret: %d)\n", STR(DC_IN_WAKEUP_GESTURE_MODE), ret);
			goto error;
		}
		TOUCH_I("%s: %s ENABLE\n", __func__, STR(DC_IN_WAKEUP_GESTURE_MODE));
	} else {
		ret = s3618_set_dynamic_config(dev, DC_IN_WAKEUP_GESTURE_MODE, 0);
		if (ret < 0) {
			TOUCH_E("failed to set %s Disable (ret: %d)\n", STR(DC_IN_WAKEUP_GESTURE_MODE), ret);
			goto error;
		}
		TOUCH_I("%s: %s DISABLE\n", __func__, STR(DC_IN_WAKEUP_GESTURE_MODE));
	}

error:
	return ret;
}

static void s3618_longpress_release(struct device *dev) {
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);

#if defined(USE_LONGPRESS_AROUND)
	if (d->longpress_uevent_status == TOUCH_IRQ_LPWG_LONGPRESS_DOWN) {
		touch_send_uevent(ts, TOUCH_UEVENT_LPWG_LONGPRESS_UP);
		d->longpress_uevent_status = TOUCH_UEVENT_LPWG_LONGPRESS_UP;
	} else if (d->longpress_uevent_status == TOUCH_IRQ_LPWG_LONGPRESS_DOWN_AROUND) {
		touch_send_uevent(ts, TOUCH_UEVENT_LPWG_LONGPRESS_UP_AROUND);
		d->longpress_uevent_status = TOUCH_UEVENT_LPWG_LONGPRESS_UP_AROUND;
	}
#else
	if (d->longpress_uevent_status == TOUCH_IRQ_LPWG_LONGPRESS_DOWN) {
		touch_send_uevent(ts, TOUCH_UEVENT_LPWG_LONGPRESS_UP);
		d->longpress_uevent_status = TOUCH_UEVENT_LPWG_LONGPRESS_UP;
	}
#endif
}

static int s3618_lpwg_debug(struct device *dev, int mode)
{
	struct s3618_data *d = to_s3618_data(dev);
	u8 i = 0, failreason = 0;
	int ret = 0;

	TOUCH_TRACE();

	ret = s3618_write_message(dev,
			CMD_GET_LGE_GESTURE_FAILREASON,
			NULL,
			0,
			d->response.buf,
			&d->response.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n",
				STR(CMD_GET_LGE_GESTURE_FAILREASON));
		goto error;
	}

	memcpy((u8*)&d->failreason_info,
			d->response.buf,
			MIN(sizeof(d->failreason_info), d->response.data_length));

	// Knock on failreason
	for (i = 0; i <= sizeof(d->failreason_info.knock_on); i++) {
		if (i == LPWG_FAILREASON_MAX_BUFFER)
			break;

		failreason = d->failreason_info.knock_on[i];
		if (failreason == 0)
			break;
		else if (failreason >= LPWG_FAILREASON_TCI_NUM)
			failreason = 0;

		TOUCH_I("TCI(%d)-Fail[%d] : %s\n", mode, i, s3618_lpwg_failreason_tci_str[failreason]);
	}

	// Swipe failreason
	for (i = 0; i <= sizeof(d->failreason_info.swipe); i++) {
		if (i == LPWG_FAILREASON_MAX_BUFFER)
			break;

		failreason = d->failreason_info.swipe[i];
		if (failreason == 0)
			break;
		else if (failreason >= LPWG_FAILREASON_SWIPE_NUM)
			failreason = 0;

		TOUCH_I("SWIPE-Fail[%d] : %s\n", i, s3618_lpwg_failreason_swipe_str[failreason]);
	}
error:
	return ret;
}


static int s3618_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);

	int ret = 0;

	TOUCH_TRACE();

	if (atomic_read(&d->state.power) == POWER_OFF ||
			atomic_read(&d->state.power) == POWER_SLEEP) {
		TOUCH_I("%s: d.power is POWER_OFF / POWER_SLEEP\n", __func__);
		return 0;
	}

	if (atomic_read(&d->state.init) == IC_INIT_NEED) {
		TOUCH_I("%s: Not Ready, Need IC init\n", __func__);
		return 0;
	}

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->mfts_lpwg) {
			/* Forced lpwg set in minios suspend mode */
			ret = s3618_lpwg_control(dev,
					LPWG_DOUBLE_TAP, SWIPE_ENABLE,
					d->lpwg_abs.enable, d->ai_pick.enable,
					d->lpwg_longpress.enable, d->lpwg_onetap.enable);
			if (ret < 0) {
				TOUCH_E("failed to set lpwg control (ret: %d)\n", ret);
				goto error;
			}
			return 0;
		}

		if (ts->lpwg.screen) {
			TOUCH_I("Skip lpwg_mode\n");

			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
				ret = s3618_sleep_control(dev, IC_NORMAL);
				if (ret < 0) {
					TOUCH_E("failed to set IC_NORMAL (ret: %d)\n", ret);
					goto error;
				}
			}
			s3618_longpress_release(dev);
			ret = s3618_lpwg_debug(dev, ts->lpwg.mode);
			if (ret < 0) {
				TOUCH_E("failed to print lpwg debug (ret: %d)\n", ret);
				goto error;
			}
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			TOUCH_I("suspend sensor == PROX_NEAR\n");
			ret = s3618_sleep_control(dev, IC_DEEP_SLEEP);
			if (ret < 0) {
				TOUCH_E("failed to set IC_DEEP_SLEEP (ret: %d)\n", ret);
				goto error;
			}
		} else if (ts->lpwg.qcover == HALL_NEAR) {
			TOUCH_I("Qcover == HALL_NEAR\n");
			ret = s3618_sleep_control(dev, IC_DEEP_SLEEP);
			if (ret < 0) {
				TOUCH_E("failed to set IC_DEEP_SLEEP (ret: %d)\n", ret);
				goto error;
			}
		} else {
			/* Knock On Case */
			if (ts->lpwg.mode == LPWG_NONE &&
					!ts->swipe[SWIPE_U].enable &&
					!ts->swipe[SWIPE_L].enable &&
					!ts->swipe[SWIPE_R].enable &&
					!ts->swipe[SWIPE_L2].enable &&
					!ts->swipe[SWIPE_R2].enable &&
					!d->ai_pick.enable &&
					!d->lpwg_longpress.enable &&
					!d->lpwg_onetap.enable) {
				ret = s3618_sleep_control(dev, IC_DEEP_SLEEP);
				if (ret < 0) {
					TOUCH_E("failed to set IC_DEEP_SLEEP (ret: %d)\n", ret);
					goto error;
				}
			} else {
				if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
					ret = s3618_sleep_control(dev, IC_NORMAL);
					if (ret < 0) {
						TOUCH_E("failed to set IC_NORMAL (ret: %d)\n", ret);
						goto error;
					}
				}
				ret = s3618_lpwg_control(dev,
						ts->lpwg.mode,
						(ts->swipe[SWIPE_U].enable ||
						 ts->swipe[SWIPE_L].enable ||
						 ts->swipe[SWIPE_R].enable ||
						 ts->swipe[SWIPE_L2].enable ||
						 ts->swipe[SWIPE_R2].enable),
						d->lpwg_abs.enable, d->ai_pick.enable,
						d->lpwg_longpress.enable, d->lpwg_onetap.enable);
				if (ret < 0) {
					TOUCH_E("failed to set lpwg control (ret: %d)\n", ret);
					goto error;
				}
			}
		}
		return 0;
	}

	/* resume */
	touch_report_all_event(ts);
	if (ts->lpwg.screen) {
		/* normal */
		TOUCH_I("resume ts->lpwg.screen on\n");
		s3618_longpress_release(dev);

		if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
			ret = s3618_sleep_control(dev, IC_NORMAL);
			if (ret < 0) {
				TOUCH_E("failed to set IC_NORMAL (ret: %d)\n", ret);
				goto error;
			}
		}
		ret = s3618_lpwg_control(dev,
				LPWG_NONE, SWIPE_DISABLE,
				LPWG_ABS_DISABLE, AI_PICK_DISABLE,
				LPWG_LONGPRESS_DISABLE, LPWG_ONETAP_DISABLE);
		if (ret < 0) {
			TOUCH_E("failed to set lpwg control (ret: %d)\n", ret);
			goto error;
		}
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		/* wake up */
		TOUCH_I("resume ts->lpwg.sensor == PROX_NEAR\n");
		ret = s3618_sleep_control(dev, IC_DEEP_SLEEP);
		if (ret < 0) {
			TOUCH_E("failed to set IC_DEEP_SLEEP (ret: %d)\n", ret);
			goto error;
		}
	} else if (ts->lpwg.qcover == HALL_NEAR) {
		TOUCH_I("resume ts->lpwg.qcover == HALL_NEAR\n");
		ret = s3618_sleep_control(dev, IC_DEEP_SLEEP);
		if (ret < 0) {
			TOUCH_E("failed to set IC_DEEP_SLEEP (ret: %d)\n", ret);
			goto error;
		}
	} else {
		/* partial */
		TOUCH_I("resume Partial\n");
		if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
			ret = s3618_sleep_control(dev, IC_NORMAL);
			if (ret < 0) {
				TOUCH_E("failed to set IC_NORMAL (ret: %d)\n", ret);
				goto error;
			}
		}
		ret = s3618_lpwg_control(dev,
				ts->lpwg.mode,
				(ts->swipe[SWIPE_U].enable ||
				 ts->swipe[SWIPE_L].enable ||
				 ts->swipe[SWIPE_R].enable ||
				 ts->swipe[SWIPE_L2].enable ||
				 ts->swipe[SWIPE_R2].enable),
				d->lpwg_abs.enable, d->ai_pick.enable,
				false, d->lpwg_onetap.enable);
		if (ret < 0) {
			TOUCH_E("failed to set lpwg control (ret: %d)\n", ret);
			goto error;
		}
	}

error:
	return ret;
}

static void s3618_init_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	ts->tci.info[TCI_1].tap_count = 2;
	ts->tci.info[TCI_1].min_intertap = 0;
	ts->tci.info[TCI_1].max_intertap = 70;
	ts->tci.info[TCI_1].touch_slop = 100;
	ts->tci.info[TCI_1].tap_distance = 10;
	ts->tci.info[TCI_1].intr_delay = 0;

	ts->tci.info[TCI_2].min_intertap = 0;
	ts->tci.info[TCI_2].max_intertap = 70;
	ts->tci.info[TCI_2].touch_slop = 100;
	ts->tci.info[TCI_2].tap_distance = 255;
	ts->tci.info[TCI_2].intr_delay = 0;
}

static void s3618_init_swipe_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	float mm_to_point = 15.8f; // 1mm -> pixel

	TOUCH_TRACE();

	ts->swipe[SWIPE_U].enable = false;
	ts->swipe[SWIPE_U].debug_enable = false;
	ts->swipe[SWIPE_U].distance = 20;
	ts->swipe[SWIPE_U].ratio_thres = 150;
	ts->swipe[SWIPE_U].min_time = 4;
	ts->swipe[SWIPE_U].max_time = 150;
	ts->swipe[SWIPE_U].wrong_dir_thres = 5;
	ts->swipe[SWIPE_U].init_ratio_chk_dist = 4;
	ts->swipe[SWIPE_U].init_ratio_thres = 100;
	ts->swipe[SWIPE_U].area.x1 = 0 + LPWG_SENSELESS_AREA_W;
	ts->swipe[SWIPE_U].area.y1 = 0;
	ts->swipe[SWIPE_U].area.x2 = ts->caps.max_x - LPWG_SENSELESS_AREA_W;
	ts->swipe[SWIPE_U].area.y2 = ts->caps.max_y;
	ts->swipe[SWIPE_U].start_area.x1 = (ts->caps.max_x / 2) - (int)(mm_to_point * 18);		// start_area_width 36mm
	ts->swipe[SWIPE_U].start_area.y1 = ts->swipe[SWIPE_U].area.y2 - (int)(mm_to_point * 10);	// start_area_height 10mm
	ts->swipe[SWIPE_U].start_area.x2 = (ts->caps.max_x / 2) + (int)(mm_to_point * 18);		// start_area_width 36mm
	ts->swipe[SWIPE_U].start_area.y2 = ts->swipe[SWIPE_U].area.y2;
	ts->swipe[SWIPE_U].border_area.x1 = 0;
	ts->swipe[SWIPE_U].border_area.y1 = 0;
	ts->swipe[SWIPE_U].border_area.x2 = 0;
	ts->swipe[SWIPE_U].border_area.y2 = 0;
	ts->swipe[SWIPE_U].start_border_area.x1 = 0;
	ts->swipe[SWIPE_U].start_border_area.y1 = 0;
	ts->swipe[SWIPE_U].start_border_area.x2 = 0;
	ts->swipe[SWIPE_U].start_border_area.y2 = 0;

	ts->swipe[SWIPE_L].enable = false;
	ts->swipe[SWIPE_L].debug_enable = false;
	ts->swipe[SWIPE_L].distance = 7;
	ts->swipe[SWIPE_L].ratio_thres = 100;
	ts->swipe[SWIPE_L].min_time = 0;
	ts->swipe[SWIPE_L].max_time = 150;
	ts->swipe[SWIPE_L].wrong_dir_thres = 2;
	ts->swipe[SWIPE_L].init_ratio_chk_dist = 2;
	ts->swipe[SWIPE_L].init_ratio_thres = 100;
	ts->swipe[SWIPE_L].area.x1 = 0;
	ts->swipe[SWIPE_L].area.y1 = 0;
	ts->swipe[SWIPE_L].area.x2 = ts->caps.max_x;
	ts->swipe[SWIPE_L].area.y2 = 200;
	ts->swipe[SWIPE_L].start_area.x1 = 0;
	ts->swipe[SWIPE_L].start_area.y1 = 0;
	ts->swipe[SWIPE_L].start_area.x2 = ts->caps.max_x;
	ts->swipe[SWIPE_L].start_area.y2 = 200;
	ts->swipe[SWIPE_L].border_area.x1 = 150;
	ts->swipe[SWIPE_L].border_area.y1 = 75;
	ts->swipe[SWIPE_L].border_area.x2 = 150;
	ts->swipe[SWIPE_L].border_area.y2 = 300;
	ts->swipe[SWIPE_L].start_border_area.x1 = 75;
	ts->swipe[SWIPE_L].start_border_area.y1 = 75;
	ts->swipe[SWIPE_L].start_border_area.x2 = 75;
	ts->swipe[SWIPE_L].start_border_area.y2 = 150;

	ts->swipe[SWIPE_R].enable = false;
	ts->swipe[SWIPE_R].debug_enable = false;
	ts->swipe[SWIPE_R].distance = 7;
	ts->swipe[SWIPE_R].ratio_thres = 100;
	ts->swipe[SWIPE_R].min_time = 0;
	ts->swipe[SWIPE_R].max_time = 150;
	ts->swipe[SWIPE_R].wrong_dir_thres = 2;
	ts->swipe[SWIPE_R].init_ratio_chk_dist = 2;
	ts->swipe[SWIPE_R].init_ratio_thres = 100;
	ts->swipe[SWIPE_R].area.x1 = 0;
	ts->swipe[SWIPE_R].area.y1 = 0;
	ts->swipe[SWIPE_R].area.x2 = ts->caps.max_x;
	ts->swipe[SWIPE_R].area.y2 = 200;
	ts->swipe[SWIPE_R].start_area.x1 = 0;
	ts->swipe[SWIPE_R].start_area.y1 = 0;
	ts->swipe[SWIPE_R].start_area.x2 = ts->caps.max_x;
	ts->swipe[SWIPE_R].start_area.y2 = 200;
	ts->swipe[SWIPE_R].border_area.x1 = 150;
	ts->swipe[SWIPE_R].border_area.y1 = 75;
	ts->swipe[SWIPE_R].border_area.x2 = 150;
	ts->swipe[SWIPE_R].border_area.y2 = 300;
	ts->swipe[SWIPE_R].start_border_area.x1 = 75;
	ts->swipe[SWIPE_R].start_border_area.y1 = 75;
	ts->swipe[SWIPE_R].start_border_area.x2 = 75;
	ts->swipe[SWIPE_R].start_border_area.y2 = 150;

	ts->swipe[SWIPE_L2].enable = false;
	ts->swipe[SWIPE_L2].debug_enable = false;
	ts->swipe[SWIPE_L2].distance = 12;
	ts->swipe[SWIPE_L2].ratio_thres = 150;
	ts->swipe[SWIPE_L2].min_time = 4;
	ts->swipe[SWIPE_L2].max_time = 150;
	ts->swipe[SWIPE_L2].wrong_dir_thres = 5;
	ts->swipe[SWIPE_L2].init_ratio_chk_dist = 4;
	ts->swipe[SWIPE_L2].init_ratio_thres = 100;
	ts->swipe[SWIPE_L2].area.x1 = 0;
	ts->swipe[SWIPE_L2].area.y1 = 0;
	ts->swipe[SWIPE_L2].area.x2 = ts->caps.max_x;
	ts->swipe[SWIPE_L2].area.y2 = ts->caps.max_y;
	ts->swipe[SWIPE_L2].start_area.x1 = ts->caps.max_x - (int)(mm_to_point * 10);
	ts->swipe[SWIPE_L2].start_area.y1 = 305;
	ts->swipe[SWIPE_L2].start_area.x2 = ts->caps.max_x;
	ts->swipe[SWIPE_L2].start_area.y2 = 1655;
	ts->swipe[SWIPE_L2].border_area.x1 = 0;
	ts->swipe[SWIPE_L2].border_area.y1 = 0;
	ts->swipe[SWIPE_L2].border_area.x2 = 0;
	ts->swipe[SWIPE_L2].border_area.y2 = 0;
	ts->swipe[SWIPE_L2].start_border_area.x1 = 0;
	ts->swipe[SWIPE_L2].start_border_area.y1 = 0;
	ts->swipe[SWIPE_L2].start_border_area.x2 = 0;
	ts->swipe[SWIPE_L2].start_border_area.y2 = 0;

	ts->swipe[SWIPE_R2].enable = false;
	ts->swipe[SWIPE_R2].debug_enable = false;
	ts->swipe[SWIPE_R2].distance = 12;
	ts->swipe[SWIPE_R2].ratio_thres = 150;
	ts->swipe[SWIPE_R2].min_time = 4;
	ts->swipe[SWIPE_R2].max_time = 150;
	ts->swipe[SWIPE_R2].wrong_dir_thres = 5;
	ts->swipe[SWIPE_R2].init_ratio_chk_dist = 4;
	ts->swipe[SWIPE_R2].init_ratio_thres = 100;
	ts->swipe[SWIPE_R2].area.x1 = 0;
	ts->swipe[SWIPE_R2].area.y1 = 0;
	ts->swipe[SWIPE_R2].area.x2 = ts->caps.max_x;
	ts->swipe[SWIPE_R2].area.y2 = ts->caps.max_y;
	ts->swipe[SWIPE_R2].start_area.x1 = 0;
	ts->swipe[SWIPE_R2].start_area.y1 = 305;
	ts->swipe[SWIPE_R2].start_area.x2 = (int)(mm_to_point * 10);
	ts->swipe[SWIPE_R2].start_area.y2 = 1655;
	ts->swipe[SWIPE_R2].border_area.x1 = 0;
	ts->swipe[SWIPE_R2].border_area.y1 = 0;
	ts->swipe[SWIPE_R2].border_area.x2 = 0;
	ts->swipe[SWIPE_R2].border_area.y2 = 0;
	ts->swipe[SWIPE_R2].start_border_area.x1 = 0;
	ts->swipe[SWIPE_R2].start_border_area.y1 = 0;
	ts->swipe[SWIPE_R2].start_border_area.x2 = 0;
	ts->swipe[SWIPE_R2].start_border_area.y2 = 0;
}

static void s3618_init_lpwg_abs_info(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);

	TOUCH_TRACE();

	d->lpwg_abs.border_area.x1 = 75;
	d->lpwg_abs.border_area.y1 = 75;
	d->lpwg_abs.border_area.x2 = 75;
	d->lpwg_abs.border_area.y2 = 19;
}

static void s3618_init_ai_pick_info(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);

	TOUCH_TRACE();

	d->ai_pick.border_area.x1 = 0;
	d->ai_pick.border_area.y1 = 0;
	d->ai_pick.border_area.x2 = 0;
	d->ai_pick.border_area.y2 = 0;
}

static void s3618_init_longpress_info(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);

	TOUCH_TRACE();

	d->lpwg_longpress.enable = LPWG_LONGPRESS_DISABLE;
	d->lpwg_longpress.touch_slop = 16;
	d->lpwg_longpress.min_press_time = 6;
	d->lpwg_longpress.min_contact_size = 1;
	d->lpwg_longpress.area.x1 = 412;
	d->lpwg_longpress.area.y1 = 1980;
	d->lpwg_longpress.area.x2 = 668;
	d->lpwg_longpress.area.y2 = 2235;
#if defined(USE_LONGPRESS_AROUND)
	d->lpwg_longpress.around_area.x1 = d->lpwg_longpress.area.x1 - 200;
	d->lpwg_longpress.around_area.y1 = d->lpwg_longpress.area.y1 - 200;
	d->lpwg_longpress.around_area.x2 = d->lpwg_longpress.area.x2 + 200;
	d->lpwg_longpress.around_area.y2 = d->lpwg_longpress.area.y2 + 200;
#endif

	TOUCH_I("%s, Long Press: %s\n", __func__, d->lpwg_longpress.enable ? "Enable" : "Disable");
#if defined(USE_LONGPRESS_AROUND)
	TOUCH_I("%s, Long Press  sensor_area(%d,%d)(%d,%d), around_area(%d,%d)(%d,%d)\n",
			__func__,
			d->lpwg_longpress.area.x1, d->lpwg_longpress.area.y1,
			d->lpwg_longpress.area.x2, d->lpwg_longpress.area.y2,
			d->lpwg_longpress.around_area.x1, d->lpwg_longpress.around_area.y1,
			d->lpwg_longpress.around_area.x2, d->lpwg_longpress.around_area.y2);
#else
	TOUCH_I("%s, Long Press  sensor_area(%d,%d)(%d,%d)\n",
			__func__,
			d->lpwg_longpress.area.x1, d->lpwg_longpress.area.y1,
			d->lpwg_longpress.area.x2, d->lpwg_longpress.area.y2);
#endif
}

static void s3618_init_onetap_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);

	TOUCH_TRACE();

	d->lpwg_onetap.enable = LPWG_ONETAP_DISABLE;
	d->lpwg_onetap.area.x1 = 0 + LPWG_SENSELESS_AREA_W;
	d->lpwg_onetap.area.y1 = 0 + LPWG_SENSELESS_AREA_W;
	d->lpwg_onetap.area.x2 = ts->caps.max_x - LPWG_SENSELESS_AREA_W;
	d->lpwg_onetap.area.y2 = ts->caps.max_y - LPWG_SENSELESS_AREA_W;
	d->lpwg_onetap.touch_slop = 10;
	d->lpwg_onetap.max_press_time = 50;
	d->lpwg_onetap.interrupt_delay_time = 10;

	TOUCH_I("%s, One tap: %s\n", __func__, d->lpwg_onetap.enable ? "Enable" : "Disable");
	TOUCH_I("%s, One tap  active_area(%d,%d)(%d,%d)\n",
			__func__,
			d->lpwg_onetap.area.x1, d->lpwg_onetap.area.y1,
			d->lpwg_onetap.area.x2, d->lpwg_onetap.area.y2);

}

static int s3618_irq_handler(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);

	int ret = 0;

	TOUCH_TRACE();

	if (atomic_read(&d->state.power) == POWER_OFF)
		return 0;

	pm_qos_update_request(&d->pm_qos_req, 10);
	ret = s3618_read_message(dev);
	if (ret < 0) {
		TOUCH_E("s3618_read_message failed, ret : %d\n", ret);
		goto error;
	}
error:
	pm_qos_update_request(&d->pm_qos_req, PM_QOS_DEFAULT_VALUE);

	return ret;
}

#if defined(CONFIG_DRM) && defined(CONFIG_FB)
#if defined(CONFIG_LGE_TOUCH_USE_PANEL_NOTIFY)
static int s3618_drm_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct touch_core_data *ts =
		container_of(self, struct touch_core_data, drm_notif);
	struct s3618_data *d = to_s3618_data(ts->dev);
	struct msm_drm_notifier *ev = (struct msm_drm_notifier *)data;

	TOUCH_TRACE();

	if (ev && ev->data && event == MSM_DRM_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		d->blank_status.prev = d->blank_status.curr;
		d->blank_status.curr = *blank;
		TOUCH_I("%s: msm_drm_blank - prev[%d] curr[%d]\n",
				__func__, d->blank_status.prev, d->blank_status.curr);
	}
	return 0;
}
#endif
#elif defined(CONFIG_FB)
static int s3618_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct touch_core_data *ts =
		container_of(self, struct touch_core_data, fb_notif);
	struct s3618_data *d = to_s3618_data(ts->dev);
	struct fb_event *ev = (struct fb_event *)data;

	TOUCH_TRACE();

	if (ev && ev->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		d->blank_status.prev = d->blank_status.curr;
		d->blank_status.curr = *blank;
		TOUCH_I("%s: blank_status - prev[%d] curr[%d]\n",
				__func__, d->blank_status.prev, d->blank_status.curr);

		if (d->blank_status.curr == FB_BLANK_UNBLANK) {
			touch_resume(ts->dev);
		} else {
			if (d->blank_status.prev == FB_BLANK_UNBLANK)
				touch_suspend(ts->dev);
		}
	}
	return 0;
}
#endif

void s3618_reflash_function(struct s3618_exp_fn *fn, bool insert)
{
	TOUCH_TRACE();

	reflash_fhandler.insert = insert;

	if (insert) {
		reflash_fhandler.exp_fn = fn;
		reflash_fhandler.insert = true;
		reflash_fhandler.initialized = false;
	} else {
		reflash_fhandler.exp_fn = NULL;
		reflash_fhandler.insert = false;
		reflash_fhandler.initialized = true;
	}
}

static int s3618_reflash_init(struct device *dev)
{
	int ret = 0;

	TOUCH_TRACE();

	if (reflash_fhandler.insert) {
		ret = reflash_fhandler.exp_fn->init(dev);

		if (ret < 0)
			TOUCH_E("Failed to reflash module init (ret: %d)\n", ret);
		else
			reflash_fhandler.initialized = true;
	}

	return ret;
}

void s3618_testing_function(struct s3618_exp_fn *fn, bool insert)
{
	TOUCH_TRACE();

	testing_fhandler.insert = insert;

	if (insert) {
		testing_fhandler.exp_fn = fn;
		testing_fhandler.insert = true;
		testing_fhandler.initialized = false;
	} else {
		testing_fhandler.exp_fn = NULL;
		testing_fhandler.insert = false;
		testing_fhandler.initialized = true;
	}
}

static int s3618_testing_init(struct device *dev)
{
	int ret = 0;

	TOUCH_TRACE();

	if (testing_fhandler.insert) {
		ret = testing_fhandler.exp_fn->init(dev);

		if (ret < 0)
			TOUCH_E("Failed to teseting module init (ret: %d)\n", ret);
		else
			testing_fhandler.initialized = true;
	}

	return ret;
}

void s3618_device_function(struct s3618_exp_fn *fn, bool insert)
{
	TOUCH_TRACE();

	device_fhandler.insert = insert;

	if (insert) {
		device_fhandler.exp_fn = fn;
		device_fhandler.insert = true;
		device_fhandler.initialized = false;
	} else {
		device_fhandler.exp_fn = NULL;
		device_fhandler.insert = false;
		device_fhandler.initialized = true;
	}
}

static int s3618_device_init(struct device *dev)
{
	int ret = 0;

	TOUCH_TRACE();

	if (device_fhandler.insert) {
		ret = device_fhandler.exp_fn->init(dev);

		if (ret < 0)
			TOUCH_E("Failed to device module init (ret: %d)\n", ret);
		else
			device_fhandler.initialized = true;
	}

	return ret;
}


static int s3618_power(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);

	TOUCH_TRACE();

	switch (ctrl) {
	case POWER_OFF:
#if defined(CONFIG_SECURE_TOUCH)
		if (atomic_read(&ts->st_enabled))
			secure_touch_stop(ts, true);
#endif
		atomic_set(&d->state.power, POWER_OFF);
		TOUCH_I("%s, off\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(10);
		touch_power_1_8_vdd(dev, 0);
		touch_msleep(10);
		touch_power_3_3_vcl(dev, 0);
		touch_msleep(10);
		break;

	case POWER_ON:
		TOUCH_I("%s, on\n", __func__);
		atomic_set(&d->state.power, POWER_SLEEP);
		atomic_set(&d->state.init, IC_INIT_NEED);
		touch_power_3_3_vcl(dev, 1); // 3.3v should be preceded by 1.8v
		touch_msleep(10);
		touch_power_1_8_vdd(dev, 1);
		touch_msleep(10);
		touch_gpio_direction_output(ts->reset_pin, 1);
		break;

	case POWER_SW_RESET:
		TOUCH_I("%s, sw reset\n", __func__);
		s3618_reset_ctrl(dev, SW_RESET);
		break;

	case POWER_HW_RESET_ASYNC:
		TOUCH_I("%s, hw reset\n", __func__);
		s3618_reset_ctrl(dev, HW_RESET);
		break;

	default:
		TOUCH_I("%s, Unknown Power ctrl!!\n", __func__);
		break;
	}

	return 0;
}

static int s3618_esd_recovery(struct device *dev)
{
	TOUCH_TRACE();

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY_ESD) || IS_ENABLED(CONFIG_LGE_TOUCH_PANEL_GLOBAL_RESET)
	lge_mdss_report_panel_dead();
#endif
#endif

#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
	mtkfb_esd_recovery();
#endif

	TOUCH_I("s3618_esd_recovery!!\n");

	return 0;
}

static int s3618_suspend(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);

	int boot_mode = TOUCH_NORMAL_BOOT;
	int ret = 0;

	TOUCH_TRACE();

	touch_interrupt_control(dev, INTERRUPT_ENABLE);

#if defined(CONFIG_DRM) && defined(CONFIG_FB)
#if defined(CONFIG_LGE_TOUCH_USE_PANEL_NOTIFY)
#else
	d->lcd_mode = LCD_MODE_U0;
	TOUCH_I("Force LCD Mode setting : d->lcd_mode = %d\n", d->lcd_mode);
#endif
#elif defined(CONFIG_FB)
#endif

	boot_mode = touch_check_boot_mode(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
	case TOUCH_MINIOS_AAT:
		break;
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
		if (!ts->mfts_lpwg) {
			TOUCH_I("%s : touch_suspend - MFTS\n", __func__);
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			s3618_power(dev, POWER_OFF);
			return -EPERM;
		}
		break;
	case TOUCH_CHARGER_MODE:
	case TOUCH_LAF_MODE:
	case TOUCH_RECOVERY_MODE:
		TOUCH_I("%s: Etc boot_mode(%d)!!!\n", __func__, boot_mode);
		return -EPERM;
	default:
		TOUCH_E("%s: invalid boot_mode = %d\n", __func__, boot_mode);
		return -EPERM;
	}

	TOUCH_I("%s : touch_suspend start\n", __func__);

	if (atomic_read(&d->state.power) == POWER_OFF ||
			atomic_read(&d->state.power) == POWER_SLEEP) {
		TOUCH_I("%s : d->state.power is POWER_OFF / POWER_SLEEP\n", __func__);
		return 0;
	}

	if (atomic_read(&d->state.init) == IC_INIT_DONE) {
		ret = s3618_lpwg_mode(dev);
		if (ret < 0) {
			TOUCH_E("failed to set lpwg mode (ret: %d)\n", ret);
			return ret;
		}
	} else {
		TOUCH_I("%s: d->state.init is IC_INIT_NEED\n", __func__);
		ret = 1;
	}

	return ret;
}

static int s3618_resume(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);

	int boot_mode = TOUCH_NORMAL_BOOT;
	int ret = 0;

	TOUCH_TRACE();

	touch_interrupt_control(dev, INTERRUPT_ENABLE);

#if defined(CONFIG_DRM) && defined(CONFIG_FB)
#if defined(CONFIG_LGE_TOUCH_USE_PANEL_NOTIFY)
#else
	d->lcd_mode = LCD_MODE_U3;
	TOUCH_I("Force LCD Mode setting : d->lcd_mode = %d\n", d->lcd_mode);
#endif
#elif defined(CONFIG_FB)
#endif

	boot_mode = touch_check_boot_mode(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
	case TOUCH_MINIOS_AAT:
		break;
	case TOUCH_MINIOS_MFTS_FOLDER:
	case TOUCH_MINIOS_MFTS_FLAT:
	case TOUCH_MINIOS_MFTS_CURVED:
		if (!ts->mfts_lpwg) {
			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			s3618_power(dev, POWER_ON);
			touch_msleep(ts->caps.hw_reset_delay);

			ret = reflash_read_product_info();
			if (ret < 0) {
				TOUCH_E("Failed to read product info\n");
			}
			touch_msleep(10);
		}
		break;
	case TOUCH_CHARGER_MODE:
	case TOUCH_LAF_MODE:
	case TOUCH_RECOVERY_MODE:
		TOUCH_I("%s: Etc boot_mode(%d)!!!\n", __func__, boot_mode);
		touch_interrupt_control(dev, INTERRUPT_DISABLE);
		s3618_power(dev, POWER_OFF);
		return -EPERM;
	default:
		TOUCH_E("%s: invalid boot_mode = %d\n", __func__, boot_mode);
		return -EPERM;
	}

	s3618_reset_ctrl(dev, HW_RESET);

	if (atomic_read(&d->state.power) == POWER_OFF ||
			atomic_read(&d->state.power) == POWER_SLEEP) {
		TOUCH_I("%s: d->state.power is POWER_OFF or POWER_SLEEP\n", __func__);
		return 0;
	}

	if (atomic_read(&d->state.init) == IC_INIT_NEED) {
		TOUCH_I("%s: d->state.init is IC_INIT_NEED\n", __func__);
		return 0;
	}

	return ret;
}

static int s3618_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);

	int *value = (int *)param;
	int ret = 0;

	TOUCH_TRACE();

	switch (code) {
	case LPWG_ACTIVE_AREA:
		ts->tci.area.x1 = value[0];
		ts->tci.area.x2 = value[1];
		ts->tci.area.y1 = value[2];
		ts->tci.area.y2 = value[3];
		TOUCH_I("LPWG_ACTIVE_AREA: x0[%d], x1[%d], x2[%d], x3[%d]\n",
			value[0], value[1], value[2], value[3]);
		break;

	case LPWG_TAP_COUNT:
		ts->tci.info[TCI_2].tap_count = value[0];
		break;

	case LPWG_DOUBLE_TAP_CHECK:
		ts->tci.double_tap_check = value[0];
		break;

	case LPWG_UPDATE_ALL:
		ts->lpwg.mode = value[0];
		ts->lpwg.screen = value[1];
		ts->lpwg.sensor = value[2];
		ts->lpwg.qcover = value[3];
		TOUCH_I("LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
			ts->lpwg.mode,
			ts->lpwg.screen ? "ON" : "OFF",
			ts->lpwg.sensor ? "FAR" : "NEAR",
			ts->lpwg.qcover ? "CLOSE" : "OPEN");
		ret = s3618_lpwg_mode(dev);
		if (ret < 0) {
			TOUCH_E("failed to set lpwg mode (ret: %d)\n", ret);
			return ret;
		}
		break;
	}

	return 0;
}

static int s3618_get_cmd_version(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int offset = 0;
	int ret = 0;
	char str[7] = {0};

	TOUCH_TRACE();

	ret = s3618_identify(ts->dev);
	if (ret < 0) {
		offset += touch_snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
				"Failed to do identification\n");
		return offset;
	}

	ret = s3618_ic_info(ts->dev);
	if (ret < 0) {
		offset += touch_snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
				"Read Fail Touch IC Info\n");
		return offset;
	}

	touch_snprintf(str, sizeof(str), "%s", d->prd_info.product_id);

	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
		"============ Version Info ============\n");
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			" IC Version: v%d.%02d, IC Build_id: %d\n",
			d->app_info.customer_config_id.release,
			d->app_info.customer_config_id.version,
			d->id_info.build_id);
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			" Bin Version: v%d.%02d, Bin Build_id: %d\n",
			d->bin_fw_version_info.release,
			d->bin_fw_version_info.version,
			d->bin_fw_version_info.build_id);
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			" Config ID: 0x%x%x%x%x%x%x%x%x\n",
			d->app_info.customer_config_id.maker,
			d->app_info.customer_config_id.key,
			d->app_info.customer_config_id.supplier,
			d->app_info.customer_config_id.inch0,
			d->app_info.customer_config_id.inch1,
			d->app_info.customer_config_id.panel,
			d->app_info.customer_config_id.release,
			d->app_info.customer_config_id.version);
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			" Product ID: [%s]\n", str);
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			" Chip_ver: %d, FPC_ver: %d, Sensor_ver: %d\n",
			d->prd_info.chip_ver,
			d->prd_info.fpc_ver,
			d->prd_info.sensor_ver);
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			" Inspector_channel: %d\n", d->prd_info.inspect_channel);
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			" Time: 20%d/%d/%d - %dh %dm %ds\n",
			d->prd_info.inspect_date[0],
			d->prd_info.inspect_date[1],
			d->prd_info.inspect_date[2],
			d->prd_info.inspect_time[0],
			d->prd_info.inspect_time[1],
			d->prd_info.inspect_time[2]);
	TOUCH_I("======================================\n");
	offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
			"======================================\n");

	return offset;
}

static int s3618_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int offset = 0;
	int ret = 0;

	TOUCH_TRACE();

	ret = s3618_identify(ts->dev);
	if (ret < 0) {
		offset += touch_snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
				"Failed to do identification\n");
		return offset;
	}

	ret = s3618_ic_info(ts->dev);
	if (ret < 0) {
		offset += touch_snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += touch_snprintf(buf + offset, PAGE_SIZE - offset,
				"Read Fail Touch IC Info\n");
		return offset;
	}

	offset = touch_snprintf(buf + offset, PAGE_SIZE - offset,
			"v%d.%02d\n",
			d->app_info.customer_config_id.release,
			d->app_info.customer_config_id.version);

	return offset;
}

static void s3618_lcd_mode(struct device *dev, u32 mode)
{
	struct s3618_data *d = to_s3618_data(dev);

	d->prev_lcd_mode = d->lcd_mode;
	d->lcd_mode = mode;
	TOUCH_I("lcd_mode: %d (prev: %d)\n", d->lcd_mode, d->prev_lcd_mode);
}

static int s3618_check_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;

	if (atomic_read(&ts->state.core) == CORE_UPGRADE ||
			atomic_read(&ts->state.core) == CORE_SHUTDOWN ||
			atomic_read(&ts->state.core) == CORE_REMOVE) {
		TOUCH_I("%s: state.core is %d, skip fb_notify_work\n", __func__,
				atomic_read(&ts->state.core));
		return 1;
	}

	if (d->lcd_mode != LCD_MODE_U3) {
		if (d->lcd_mode == LCD_MODE_U2) {
			if (d->prev_lcd_mode == LCD_MODE_U2_UNBLANK) {
				TOUCH_I("U2 UNBLANK -> U2\n");
				ret = 1;
			} else {
				TOUCH_I("U2 mode change\n");
			}
		} else if (d->lcd_mode == LCD_MODE_U2_UNBLANK) {
			switch (d->prev_lcd_mode) {
			case LCD_MODE_U2:
				TOUCH_I("U2 -> U2 UNBLANK\n");
				ret = 1;
				break;
			case LCD_MODE_U0:
				TOUCH_I("U0 -> U2 UNBLANK mode change\n");
				ret = 1;
				break;
			default:
				TOUCH_I("%s - Not Defined Mode\n", __func__);
				break;
			}
		} else if (d->lcd_mode == LCD_MODE_U0) {
			if (d->prev_lcd_mode == LCD_MODE_U2_UNBLANK) {
				TOUCH_I("U2 UNBLANK -> U0 mode change\n");
				ret = 1;
			} else
				TOUCH_I("U0 mode change\n");
		} else {
			TOUCH_I("%s - Not defined mode\n", __func__);
		}
	}

	return ret;
}

static int s3618_debug_option(struct device *dev, u32 *data)
{
	u32 chg_mask = data[0];
	u32 enable = data[1];
	int ret = 0;

	TOUCH_TRACE();

	switch (chg_mask) {
	case DEBUG_OPTION_0:
		TOUCH_I("TA Simulator mode %s\n", enable ? "Enable" : "Disable");
		//ret = s3618_connect(dev);
		if (ret < 0)
			TOUCH_E("failed to set charger bit (ret: %d)\n", ret);
		break;
	default:
		TOUCH_E("Not supported debug option\n");
		break;
	}

	return ret;
}

static int s3618_notify(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);

	int ret = 0;

	TOUCH_TRACE();

	TOUCH_D(TRACE, "%s event=0x%x", __func__, (unsigned int)event);

	switch (event) {
	case NOTIFY_TOUCH_RESET:
		break;
	case LCD_EVENT_LCD_MODE:
		TOUCH_I("LCD_EVENT_LCD_MODE!\n");
		s3618_lcd_mode(dev, *(u32 *)data);
		ret = s3618_check_mode(dev);
		if (ret == 0)
			queue_delayed_work(ts->wq, &d->fb_notify_work, 0);
		else
			ret = 0;
		break;
	case NOTIFY_CONNECTION:
		TOUCH_I("%s: NOTIFY_CONNECTION!\n", __func__);
		ret = s3618_set_dynamic_config(dev, DC_CHARGER_CONNECTED, *(u16 *)data);
		if (ret < 0) {
			TOUCH_E("failed to set %s %s (ret: %d)\n",
					STR(DC_CHARGER_CONNECTED),
					*(u16 *)data ? "Enable" : "Disable",
					ret);
			break;
		}
		TOUCH_I("%s: %s %s\n", __func__,
				STR(DC_CHARGER_CONNECTED),
				*(u16 *)data ? "Enable" : "Disable");
		break;
	case NOTIFY_WIRELESS:
		TOUCH_I("%s: NOTIFY_WIRELEES!\n", __func__);
		ret = s3618_set_dynamic_config(dev, DC_WIRELESS_CHARGER_CONNECTED, *(u16 *)data);
		if (ret < 0) {
			TOUCH_E("failed to set %s %s (ret: %d)\n",
					STR(DC_WIRELESS_CHARGER_CONNECTED),
					*(u16 *)data ? "Enable" : "Disable",
					ret);
			break;
		}
		TOUCH_I("%s: %s %s\n", __func__,
				STR(DC_WIRELESS_CHARGER_CONNECTED),
				*(u16 *)data ? "Enable" : "Disable");
		break;
	case NOTIFY_IME_STATE:
		TOUCH_I("%s: NOTIFY_IME_STATE!\n", __func__);
		ret = s3618_set_dynamic_config(dev, DC_IME_MODE_ENABLE, *(u16 *)data);
		if (ret < 0) {
			TOUCH_E("failed to set %s %s (ret: %d)\n",
					STR(DC_IME_MODE_ENABLE),
					*(u16 *)data ? "Enable" : "Disable",
					ret);
			break;
		}
		TOUCH_I("%s: %s %s\n", __func__,
				STR(DC_IME_MODE_ENABLE),
				*(u16 *)data ? "Enable" : "Disable");
		break;
	case NOTIFY_CALL_STATE:
		TOUCH_I("%s: NOTIFY_CALL_STATE!\n", __func__);
		break;
	case NOTIFY_DEBUG_OPTION:
		TOUCH_I("NOTIFY_DEBUG_OPTION!\n");
		ret = s3618_debug_option(dev, (u32 *)data);
		if (ret < 0)
			TOUCH_E("failed to set debug_option (ret: %d)\n", ret);
		break;
	case NOTIFY_FILM_STATE:
		TOUCH_I("%s: NOTIFY_FILM_STATE!\n", __func__);
		ret = s3618_set_dynamic_config(dev, DC_SENSITIVE_MODE_ENABLE, *(u16 *)data);
		if (ret < 0) {
			TOUCH_E("failed to set %s %s (ret: %d)\n",
					STR(DC_SENSITIVE_MODE_ENABLE),
					*(u16 *)data ? "Enable" : "Disable",
					ret);
			break;
		}
		TOUCH_I("%s: %s %s\n", __func__,
				STR(DC_SENSITIVE_MODE_ENABLE),
				*(u16 *)data ? "Enable" : "Disable");
		break;
	case NOTIFY_DUALSCREEN_STATE:
		TOUCH_I("%s: NOTIFY_DUALSCREEN_STATE!\n", __func__);
		ret = s3618_set_dynamic_config(dev, DC_DUALSCREEN_CONNECTED, *(u16 *)data);
		if (ret < 0) {
			TOUCH_E("failed to set %s %s (ret: %d)\n",
					STR(DC_DUALSCREEN_CONNECTED),
					*(u16 *)data ? "Enable" : "Disable",
					ret);
			break;
		}
		TOUCH_I("%s: %s %s\n", __func__,
				STR(DC_DUALSCREEN_CONNECTED),
				*(u16 *)data ? "Enable" : "Disable");
		break;
	default:
		TOUCH_E("%lu is not supported\n", event);
		break;
	}

	return ret;
}

static int s3618_init_pm(struct device *dev)
{
#if defined(CONFIG_DRM) && defined(CONFIG_FB)
#if defined(CONFIG_LGE_TOUCH_USE_PANEL_NOTIFY)
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_I("%s: drm_notif change\n", __func__);
	ts->drm_notif.notifier_call = s3618_drm_notifier_callback;
#endif
#elif defined(CONFIG_FB)
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_I("%s: fb_notif change\n", __func__);
	ts->fb_notif.notifier_call = s3618_fb_notifier_callback;
#endif

	return 0;
}
static void s3618_fb_notify_work_func(struct work_struct *fb_notify_work)
{
	struct s3618_data *d =
			container_of(to_delayed_work(fb_notify_work),
				struct s3618_data, fb_notify_work);
	int ret = 0;

	if (d->lcd_mode == LCD_MODE_U3)
		ret = FB_RESUME;
	else
		ret = FB_SUSPEND;

	touch_notifier_call_chain(NOTIFY_FB, &ret);
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf, size_t count)
{
	int value = 0;

	TOUCH_TRACE();

	if (kstrtos32(buf, 10, &value) < 0)
		return count;

	s3618_reset_ctrl(dev, value);

	return count;
}

static ssize_t show_gpio_pin(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int reset_pin = 0;
	int int_pin = 0;
	int ret = 0;

	TOUCH_TRACE();

	reset_pin = gpio_get_value(ts->reset_pin);
	int_pin = gpio_get_value(ts->int_pin);

	ret += touch_snprintf(buf + ret, PAGE_SIZE - ret,
			"reset_pin = %d , int_pin = %d\n",
			reset_pin, int_pin);
	TOUCH_I("%s: reset_pin = %d , int_pin = %d\n",
			__func__, reset_pin, int_pin);

	return ret;
}



static ssize_t show_lpwg_abs(struct device *dev, char *buf)
{
	int ret = 0;
	struct s3618_data *d = to_s3618_data(dev);

	TOUCH_TRACE();

	ret += touch_snprintf(buf + ret, PAGE_SIZE, "%d\n", d->lpwg_abs.enable);
	TOUCH_I("%s: lpwg_abs.enable = %d\n", __func__, d->lpwg_abs.enable);

	TOUCH_I("%s: active_area(%d,%d)(%d,%d)\n", __func__,
			d->lpwg_abs.area.x1, d->lpwg_abs.area.y1,
			d->lpwg_abs.area.x2, d->lpwg_abs.area.y2);

	return ret;
}

static ssize_t store_lpwg_abs(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);

	int ret = 0;
	int enable = 0;
	int offset_y = 0;
	int start_x = 0;
	int start_y = 0;
	int width = 0;
	int height = 0;
	int end_x = 0;
	int end_y = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d %d %d %d %d %d", &enable, &offset_y, &start_x,
				&start_y, &width, &height) <= 0)
		return count;

	TOUCH_I("%s: enable = %d, offset_y = %d, start_x = %d, start_y = %d, width = %d, height = %d\n",
			__func__, enable,
			offset_y, start_x, start_y, width, height);

	if ((enable > 1) || (enable < 0)) {
		TOUCH_E("invalid enable(%d)\n", enable);
		return count;
	}

	if (enable) {
		end_x = start_x + width - 1;
		end_y = start_y + height - 1;

		d->lpwg_abs.enable = LPWG_ABS_ENABLE;
		d->lpwg_abs.offset_y = offset_y;
		d->lpwg_abs.area.x1 = start_x;
		d->lpwg_abs.area.y1 = start_y;
		d->lpwg_abs.area.x2 = end_x;
		d->lpwg_abs.area.y2 = end_y;
	} else {
		if (d->lpwg_abs.enable == LPWG_ABS_DISABLE) {
			TOUCH_I("%s: skip disable lpwg_abs (d->lpwg_abs.enable:%d)\n",
					__func__, d->lpwg_abs.enable);
			return count;
		}
		d->lpwg_abs.enable = LPWG_ABS_DISABLE;
	}

	mutex_lock(&ts->lock);
	ret = s3618_lpwg_abs_enable(dev, d->lpwg_abs.enable);
	if (ret < 0)
		TOUCH_E("failed to set lpwg_abs enable (ret: %d)\n", ret);

	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t show_ai_pick(struct device *dev, char *buf)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret += touch_snprintf(buf + ret, PAGE_SIZE, "%d\n",
			d->ai_pick.enable);
	TOUCH_I("%s: ai_pick.enable = %d\n",
			__func__, d->ai_pick.enable);

	TOUCH_I("%s: active_area(%d,%d)(%d,%d)\n", __func__,
			d->ai_pick.area.x1, d->ai_pick.area.y1,
			d->ai_pick.area.x2, d->ai_pick.area.y2);

	return ret;
}

static ssize_t store_ai_pick(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int enable = 0;
	int offset_y = 0;
	int start_x = 0;
	int start_y = 0;
	int width = 0;
	int height = 0;
	int end_x = 0;
	int end_y = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d %d %d %d %d %d", &enable, &offset_y, &start_x,
				&start_y, &width, &height) <= 0)
		return count;

	TOUCH_I("%s: enable = %d, offset_y = %d, start_x = %d, start_y = %d, width = %d, height = %d\n",
			__func__, enable,
			offset_y, start_x, start_y, width, height);

	if ((enable > 1) || (enable < 0)) {
		TOUCH_E("invalid enable(%d)\n", enable);
		return count;
	}

	if (enable) {
		end_x = start_x + width - 1;
		end_y = start_y + height - 1;

		d->ai_pick.enable = AI_PICK_ENABLE;

		d->ai_pick.area.x1 = start_x;
		d->ai_pick.area.y1 = start_y;
		d->ai_pick.area.x2 = end_x;
		d->ai_pick.area.y2 = end_y;

		d->ai_pick.total_area.x1 = d->ai_pick.area.x1
			- d->ai_pick.border_area.x1;
		if (d->ai_pick.total_area.x1 < 0)
			d->ai_pick.total_area.x1 = 0;

		d->ai_pick.total_area.y1 = d->ai_pick.area.y1
			- d->ai_pick.border_area.y1;
		if (d->ai_pick.total_area.y1 < 0)
			d->ai_pick.total_area.y1 = 0;

		d->ai_pick.total_area.x2 = d->ai_pick.area.x2
			+ d->ai_pick.border_area.x2;
		if (d->ai_pick.total_area.x2 > ts->caps.max_x)
			d->ai_pick.total_area.x2 = ts->caps.max_x;

		d->ai_pick.total_area.y2 = d->ai_pick.area.y2
			+ d->ai_pick.border_area.y2;
		if (d->ai_pick.total_area.y2 > ts->caps.max_y)
			d->ai_pick.total_area.y2 = ts->caps.max_y;
	} else {
		d->ai_pick.enable = AI_PICK_DISABLE;
	}

	mutex_lock(&ts->lock);
	ret = s3618_ai_pick_enable(dev, d->ai_pick.enable);
	if (ret < 0)
		TOUCH_E("failed to set ai_pick enable (ret: %d)\n", ret);

	mutex_unlock(&ts->lock);

	return count;
}


static ssize_t show_longpress(struct device *dev, char *buf)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret += touch_snprintf(buf + ret, PAGE_SIZE, "%d\n",
			d->lpwg_longpress.enable);
#if defined(USE_LONGPRESS_AROUND)
	TOUCH_I("%s: enable = %d, sensor_area(%d,%d)(%d,%d), around_area(%d,%d)(%d,%d) min_press_time =%d, min_contact_size = %d, touch_slop = %d\n",
			__func__, d->lpwg_longpress.enable,
			d->lpwg_longpress.area.x1, d->lpwg_longpress.area.y1,
			d->lpwg_longpress.area.x2, d->lpwg_longpress.area.y2,
			d->lpwg_longpress.around_area.x1, d->lpwg_longpress.around_area.y1,
			d->lpwg_longpress.around_area.x2, d->lpwg_longpress.around_area.y2,
			d->lpwg_longpress.min_press_time, d->lpwg_longpress.min_contact_size, d->lpwg_longpress.touch_slop);
#else
	TOUCH_I("%s: enable = %d, sensor_area(%d,%d)(%d,%d), min_press_time =%d, min_contact_size = %d, touch_slop = %d\n",
			__func__, d->lpwg_longpress.enable,
			d->lpwg_longpress.area.x1, d->lpwg_longpress.area.y1,
			d->lpwg_longpress.area.x2, d->lpwg_longpress.area.y2,
			d->lpwg_longpress.min_press_time, d->lpwg_longpress.min_contact_size, d->lpwg_longpress.touch_slop);
#endif

	return ret;
}


static ssize_t store_longpress(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int enable = 0;
	int offset_y = 0;
	int start_x = 0;
	int start_y = 0;
	int width = 0;
	int height = 0;
	int end_x = 0;
	int end_y = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d %d %d %d %d %d", &enable, &offset_y, &start_x,
				&start_y, &width, &height) <= 0)
		return count;

	TOUCH_I("%s: enable = %d, offset_y = %d, start_x = %d, start_y = %d, width = %d, height = %d\n",
			__func__, enable, offset_y, start_x,
			start_y, width, height);

	if ((enable > 1) || (enable < 0)) {
		TOUCH_E("invalid enable(%d)\n", enable);
		return count;
	}

	if (enable) {
		d->lpwg_longpress.enable = LPWG_LONGPRESS_ENABLE;
		if (offset_y == 0 && start_x == 0 && start_y == 0
				&& width == 0 && height == 0) {
			TOUCH_D(TRACE, "For CFW calls, the coordinate values are not changed.\n");
		} else {
			end_x = start_x + width - 1;
			end_y = start_y + height - 1;

#if defined(USE_LONGPRESS_AROUND)
			d->lpwg_longpress.around_area.x1 = start_x;
			d->lpwg_longpress.around_area.y1 = start_y;
			d->lpwg_longpress.around_area.x2 = end_x;
			d->lpwg_longpress.around_area.y2 = end_y;
#else
			d->lpwg_longpress.area.x1 = start_x;
			d->lpwg_longpress.area.y1 = start_y;
			d->lpwg_longpress.area.x2 = end_x;
			d->lpwg_longpress.area.y2 = end_y;
#endif

		}
	} else {
		d->lpwg_longpress.enable = LPWG_LONGPRESS_DISABLE;
	}

	mutex_lock(&ts->lock);

	ret = s3618_lpwg_longpress_enable(dev, d->lpwg_longpress.enable);
	if (ret < 0)
		TOUCH_E("failed to set lpwg_longpress enable (ret: %d)\n", ret);

	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t show_onetap(struct device *dev, char *buf)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret += touch_snprintf(buf + ret, PAGE_SIZE, "%d\n",
			d->lpwg_onetap.enable);
	TOUCH_I("%s: enable = %d, active_area(%d,%d)(%d,%d), touch_slop = %d, max_press_time = %d, interrupt_delay_time = %d\n",
			__func__, d->lpwg_onetap.enable, d->lpwg_onetap.area.x1, d->lpwg_onetap.area.y1,
			d->lpwg_onetap.area.x2, d->lpwg_onetap.area.y2, d->lpwg_onetap.touch_slop,
			d->lpwg_onetap.max_press_time, d->lpwg_onetap.interrupt_delay_time);
	return ret;
}


static ssize_t store_onetap(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int enable = 0;
	int offset_y = 0;
	int start_x = 0;
	int start_y = 0;
	int width = 0;
	int height = 0;
	int end_x = 0;
	int end_y = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d %d %d %d %d %d", &enable, &offset_y, &start_x,
				&start_y, &width, &height) <= 0)
		return count;

	TOUCH_I("%s: enable = %d, offset_y = %d, start_x = %d, start_y = %d, width = %d, height = %d\n",
			__func__, enable, offset_y, start_x,
			start_y, width, height);

	if ((enable > 1) || (enable < 0)) {
		TOUCH_E("invalid enable(%d)\n", enable);
		return count;
	}

	if (enable) {
		d->lpwg_onetap.enable = LPWG_ONETAP_ENABLE;

		if (offset_y == 0 && start_x == 0 && start_y == 0
				&& width == 0 && height == 0) {
			TOUCH_D(TRACE, "For CFW calls, the coordinate values are not changed.\n");
		} else {
			end_x = start_x + width - 1;
			end_y = start_y + height - 1;
			d->lpwg_onetap.area.x1 = start_x;
			d->lpwg_onetap.area.y1 = start_y;
			d->lpwg_onetap.area.x2 = end_x;
			d->lpwg_onetap.area.y2 = end_y;
		}
	} else {
		d->lpwg_onetap.enable = LPWG_ONETAP_DISABLE;
	}

	mutex_lock(&ts->lock);

	ret = s3618_lpwg_onetap_enable(dev, d->lpwg_onetap.enable);
	if (ret < 0)
		TOUCH_E("failed to set lpwg_onetap enable (ret: %d)\n", ret);

	mutex_unlock(&ts->lock);

	return count;
}

static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);
static TOUCH_ATTR(gpio_pin, show_gpio_pin, NULL);
static TOUCH_ATTR(lpwg_abs, show_lpwg_abs, store_lpwg_abs);
static TOUCH_ATTR(ai_pick, show_ai_pick, store_ai_pick);
static TOUCH_ATTR(longpress, show_longpress, store_longpress);
static TOUCH_ATTR(onetap, show_onetap, store_onetap);

static struct attribute *s3618_attribute_list[] = {
	&touch_attr_reset_ctrl.attr,
	&touch_attr_gpio_pin.attr,
	&touch_attr_lpwg_abs.attr,
	&touch_attr_ai_pick.attr,
	&touch_attr_longpress.attr,
	&touch_attr_onetap.attr,
	NULL,
};

static const struct attribute_group s3618_attribute_group = {
	.attrs = s3618_attribute_list,
};

static int s3618_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	int ret = 0;

	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &s3618_attribute_group);
	if (ret < 0)
		TOUCH_E("s3618 sysfs register failed\n");

	//s3618_prd_register_sysfs(dev);

	return 0;
}
static int s3618_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int s3618_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
		case CMD_VERSION:
			ret = s3618_get_cmd_version(dev, (char *)output);
			break;

		case CMD_ATCMD_VERSION:
			ret = s3618_get_cmd_atcmd_version(dev, (char *)output);
			break;

		default:
			break;
	}

	return ret;
}

static int s3618_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);

	const struct firmware *fw = NULL;
	char fwpath[256] = {0};
	int ret = 0;

	TOUCH_TRACE();

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	if (atomic_read(&d->state.power) == POWER_OFF ||
			atomic_read(&d->state.power) == POWER_SLEEP) {
		TOUCH_I("%s: d.power is POWER_OFF / POWER_SLEEP\n", __func__);
		return 0;
	}

	if ((ts->force_fwup == 0) && (atomic_read(&d->state.init) == IC_INIT_NEED)) {
		TOUCH_I("%s: d.init is IC_INIT_NEED\n", __func__);
		ret = -EPERM;
		goto error;
	}

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("%s: state.fb is not FB_RESUME\n", __func__);
		ret = -EPERM;
		goto error;
	}

	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n", &ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from default_fwpath:%s\n", ts->def_fwpath[0]);
	} else {
		TOUCH_E("no firmware file\n");
		ret = -EPERM;
		goto error;
	}

	fwpath[sizeof(fwpath) - 1] = '\0';
	if (strlen(fwpath) <= 0) {
		TOUCH_E("error get fw path\n");
		ret = -EPERM;
		goto error;
	}

	TOUCH_I("fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, dev);
	if (ret) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n",
				fwpath, ret);
		goto error;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	ret = s3618_fw_updater(fw->data);
	if (ret < 0) {
		TOUCH_E("Firmware Upgrade failed (ret: %d)\n", ret);
		ret = 0; // Need to Reset
		goto error;
	}

	ret = reflash_read_product_info();
	if (ret < 0) {
		TOUCH_E("Failed to read product info\n");
		ret = 0;
	}
error:
	release_firmware(fw);
	return ret;
}

static void s3618_init_works(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);

	TOUCH_TRACE();

	init_completion(&d->response_complete);
	INIT_DELAYED_WORK(&d->fb_notify_work, s3618_fb_notify_work_func);
}

static void s3618_init_locks(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);

	TOUCH_TRACE();

	// Common
	mutex_init(&d->command_mutex);
	mutex_init(&d->rw_ctrl_mutex);
	mutex_init(&d->io_mutex);
	mutex_init(&d->reset_mutex);
	mutex_init(&d->extif_mutex);

	// Buffer
	mutex_init(&d->in.buf_mutex);
	mutex_init(&d->out.buf_mutex);
	mutex_init(&d->response.buf_mutex);
	mutex_init(&d->report.buf_mutex);
	mutex_init(&d->config.buf_mutex);
	mutex_init(&d->temp.buf_mutex);

}

int s3618_switch_mode(struct device *dev, enum firmware_mode mode)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;
	int command = 0;
	char command_str[128] = {0, };
	int retry = true;

	TOUCH_TRACE();

	switch (mode) {
		case MODE_APPLICATION_FIRMWARE:
			command = CMD_RUN_APPLICATION_FIRMWARE;
			touch_snprintf(command_str, sizeof(command_str), "%s",
					STR(CMD_RUN_APPLICATION_FIRMWARE));
			break;
		case MODE_ROMBOOTLOADER:
			command = CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE;
			touch_snprintf(command_str, sizeof(command_str), "%s",
					STR(CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE));
			break;
		case MODE_BOOTLOADER:
			command = CMD_RUN_BOOTLOADER_FIRMWARE;
			touch_snprintf(command_str, sizeof(command_str), "%s",
					STR(CMD_RUN_BOOTLOADER_FIRMWARE));
			break;
		case MODE_PRODUCTIONTEST_FIRMWARE:
			command = CMD_ENTER_PRODUCTION_TEST_MODE;
			touch_snprintf(command_str, sizeof(command_str), "%s",
					STR(CMD_ENTER_PRODUCTION_TEST_MODE));
			break;
		default:
			TOUCH_E("Invalid switch firmware mode = %d\n", mode);
			ret = -EINVAL;
			goto exit;
	}

retry:
	ret = s3618_write_message(dev,
			command,
			NULL,
			0,
			NULL,
			NULL);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n", command_str);
		goto exit;
	}
	TOUCH_I("Response: %d / Write Command: %s \n",
			d->response_code,
			command_str);

	ret = s3618_ic_info(dev);
	if (ret < 0) {
		TOUCH_E("Failed to get ic_info\n");
		goto exit;
	}

	if (d->id_info.mode != mode) {
		TOUCH_E("Failed to switch firmware mode, current mode=%d\n", d->id_info.mode);

		if (retry) {
			retry = false;
			goto retry;
		}
		ret = -EINVAL;
		goto exit;
	}
	ret = 0;

exit:
	return ret;
}
static int s3618_get_app_info(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;
	int timeout = 0;

	timeout = STATUS_POLL_TIMEOUT_MS;

retry:
	ret = s3618_write_message(dev,
			CMD_GET_APPLICATION_INFO,
			NULL,
			0,
			d->response.buf,
			&d->response.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n", STR(CMD_GET_APPLICATION_INFO));
		goto error;
	}

	memcpy((u8*)&d->app_info,
			d->response.buf,
			MIN(sizeof(d->app_info), d->response.data_length));

	if (d->app_info.status != APP_STATUS_OK) {
		if (timeout >= 0) {
			TOUCH_E("Retry read app_info.status = %d, timeout(%d/%dms)\n",
					d->app_info.status, timeout, STATUS_POLL_TIMEOUT_MS);
			touch_msleep(STATUS_POLL_MS);
			timeout -= STATUS_POLL_MS;
			goto retry;
		} else {
			TOUCH_E("Failed to wait APP_STATUS_OK\n");
			ret = -ETIME;
			goto error;
		}
	}

	ret = 0;

	TOUCH_I("Read app_info.status = %s(%d), timeout(%d/%d)\n",
			(d->app_info.status == 0) ? "STATUS_OK" : "NEED_CHECK",
			d->app_info.status,
			timeout, STATUS_POLL_TIMEOUT_MS);

error:
	return ret;
}

static int s3618_get_boot_info(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;
	int timeout = 0;

	timeout = STATUS_POLL_TIMEOUT_MS;

retry:
	ret = s3618_write_message(dev,
			CMD_GET_BOOT_INFO,
			NULL,
			0,
			d->response.buf,
			&d->response.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n", STR(CMD_GET_BOOT_INFO));
		goto error;
	}

	memcpy((u8*)&d->boot_info,
			d->response.buf,
			MIN(sizeof(d->boot_info), d->response.data_length));

	if (d->boot_info.status != BOOT_STATUS_OK &&
			d->boot_info.status != BOOT_STATUS_BOOTLOADER_REQUEST) {
		if (timeout >= 0) {
			TOUCH_E("Retry read boot_info.status = %d, timeout(%d/%dms)\n",
					d->boot_info.status, timeout, STATUS_POLL_TIMEOUT_MS);
			touch_msleep(STATUS_POLL_MS);
			timeout -= STATUS_POLL_MS;
			goto retry;
		} else {
			TOUCH_E("Failed to wait BOOT_STATUS_OK\n");
			ret = -ETIME;
			goto error;
		}
	}

	ret = 0;

	TOUCH_I("Read boot_info.status = %s(%d), timeout(%d/%d)\n",
					(d->boot_info.status == 0) ? "STATUS_OK" : "NEED_CHECK",
					d->boot_info.status,
					timeout, STATUS_POLL_TIMEOUT_MS);

error:
	return ret;
}

static int s3618_get_romboot_info(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;
	int timeout = 0;

	timeout = STATUS_POLL_TIMEOUT_MS;

retry:
	ret = s3618_write_message(dev,
			CMD_GET_ROMBOOT_INFO,
			NULL,
			0,
			d->response.buf,
			&d->response.data_length);
	if (ret < 0) {
		TOUCH_E("Failed to write command %s\n", STR(CMD_GET_ROMBOOT_INFO));
		goto error;
	}

	memcpy((u8*)&d->romboot_info,
			d->response.buf,
			MIN(sizeof(d->romboot_info), d->response.data_length));

	if (d->romboot_info.status != ROMBOOT_STATUS_OK) {
		if (timeout >= 0) {
			TOUCH_E("Retry read romboot_info.status = %d, timeout(%d/%dms)\n",
					d->romboot_info.status, timeout, STATUS_POLL_TIMEOUT_MS);
			touch_msleep(STATUS_POLL_MS);
			timeout -= STATUS_POLL_MS;
			goto retry;
		} else {
			TOUCH_E("Failed to wait ROMBOOT_STATUS_OK\n");
			ret = -ETIME;
			goto error;
		}
	}

	ret = 0;

	TOUCH_I("Read romboot_info.status = %s(%d), timeout(%d/%d)\n",
					(d->romboot_info.status == 0) ? "STATUS_OK" : "NEED_CHECK",
					d->romboot_info.status,
					timeout, STATUS_POLL_TIMEOUT_MS);

error:
	return ret;
}

int s3618_ic_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int ret = 0;
	char str[7] = {0};
	const struct firmware *fw = NULL;
	char fwpath[256] = {0};

	TOUCH_TRACE();

	switch (d->id_info.mode) {
		case MODE_APPLICATION_FIRMWARE:
		case MODE_HOSTDOWNLOAD_FIRMWARE:
			TOUCH_I("Current FW Mode : Application mode");
			ret = s3618_get_app_info(dev);
			if (ret < 0) {
				TOUCH_E("app_info status is not ok.\n");
				if (d->app_info.status == APP_STATUS_BAD_APP_CONFIG) {
					TOUCH_E("App config wrong!! force_fwup is enabled.\n");
					ts->force_fwup = 1;
					ret = 0;
				}
				goto error;
			}

			if (atomic_read(&ts->state.core) == CORE_PROBE) {
				TOUCH_I("Try to compare fw version for checking exist all command.\n");
				memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
				fwpath[sizeof(fwpath) - 1] = '\0';

				ret = request_firmware(&fw, fwpath, dev);
				if (ret) {
					TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", fwpath, ret);
					release_firmware(fw);
					goto error;
				}

				ret = s3618_get_bin_fw_version(fw->data);
				if (ret < 0) {
					TOUCH_E("fail to get binary firmware version, ret=%d\n", ret);
					release_firmware(fw);
					goto error;
				}

				release_firmware(fw);

				if (d->id_info.build_id != d->bin_fw_version_info.build_id ||
						d->app_info.customer_config_id.release != d->bin_fw_version_info.release ||
						d->app_info.customer_config_id.version != d->bin_fw_version_info.version) {
					TOUCH_I("Firmware version is different! force_fwup is enabled.\n");
					TOUCH_I("%s : Version: v%d.%02d, Build_id: %d\n", __func__,
							d->app_info.customer_config_id.release,
							d->app_info.customer_config_id.version,
							d->id_info.build_id);
					TOUCH_I("%s : Binary Version: v%d.%02d, Binanry Build_id: %d\n", __func__,
							d->bin_fw_version_info.release,
							d->bin_fw_version_info.version,
							d->bin_fw_version_info.build_id);

					ts->force_fwup = 1;
					ret = 0;
					goto error;
				}
			}

			touch_snprintf(str, sizeof(str), "%s", d->prd_info.product_id);
			TOUCH_I("============ Version Info ============\n");
			TOUCH_I(" IC Version: v%d.%02d, IC Build_id: %d\n",
					d->app_info.customer_config_id.release,
					d->app_info.customer_config_id.version,
					d->id_info.build_id);
			TOUCH_I(" Bin Version: v%d.%02d, Bin Build_id: %d\n",
					d->bin_fw_version_info.release,
					d->bin_fw_version_info.version,
					d->bin_fw_version_info.build_id);
			TOUCH_I(" Config ID: 0x%x%x%x%x%x%x%x%x\n",
					d->app_info.customer_config_id.maker,
					d->app_info.customer_config_id.key,
					d->app_info.customer_config_id.supplier,
					d->app_info.customer_config_id.inch0,
					d->app_info.customer_config_id.inch1,
					d->app_info.customer_config_id.panel,
					d->app_info.customer_config_id.release,
					d->app_info.customer_config_id.version);
			TOUCH_I(" Product ID: [%s]\n", str);
			TOUCH_I(" Chip_ver: %d, FPC_ver: %d, Sensor_ver: %d\n",
					d->prd_info.chip_ver,
					d->prd_info.fpc_ver,
					d->prd_info.sensor_ver);
			TOUCH_I(" Inspector_channel: %d\n", d->prd_info.inspect_channel);
			TOUCH_I(" Time: 20%d/%d/%d - %dh %dm %ds\n",
					d->prd_info.inspect_date[0],
					d->prd_info.inspect_date[1],
					d->prd_info.inspect_date[2],
					d->prd_info.inspect_time[0],
					d->prd_info.inspect_time[1],
					d->prd_info.inspect_time[2]);
			TOUCH_I("======================================\n");

			break;
		case MODE_ROMBOOTLOADER:
			TOUCH_I("Current FW Mode : ROM Bootloader mode\n");
			ret = s3618_get_romboot_info(dev);
			if (ret < 0) {
				TOUCH_E("Failed to get romboot_info\n");
				goto error;
			}
			break;
		case MODE_BOOTLOADER:
		case MODE_TDDI_BOOTLOADER:
			TOUCH_I("Current FW Mode : Bootloader mode\n");
			ret = s3618_get_boot_info(dev);
			if (ret < 0) {
				TOUCH_E("Boot_info status is not ok.\n");
				if (d->boot_info.status == BOOT_STATUS_BAD_APP_FIRMWARE ||
						d->boot_info.status == BOOT_STATUS_APP_BAD_DISPLAY_CRC ||
						d->boot_info.status == BOOT_STATUS_BAD_DISPLAY_CONFIG) {
					TOUCH_E("Bootloader Mode!! force_fwup is enabled.\n");
					ts->force_fwup = 1;
					ret = 0;
				}
				goto error;
			}
			break;
		default:
			TOUCH_E("IC is detected, but fw mode is 0x%02x\n",
					d->id_info.mode);
			break;
	}

error:
	return ret;
}

int s3618_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = to_s3618_data(dev);
	int boot_mode = 0;
	int ret = 0;

	TOUCH_TRACE();

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		if (gpio_get_value(ts->int_pin) == 0) {
		// Irq handler is not registered at probe time. Therefore, Read manually.
			TOUCH_I("%s: int_pin is LOW, Read identification packet", __func__);

			ret = s3618_read_message(dev);
			if (ret < 0) {
				TOUCH_E("Failed to read Identificaion packet\n");
				// once the tcm communication interface is not ready,
				// F35 Check and Recovery but, not supported in s3618
				return -ENODEV;
			}
			atomic_set(&d->state.power, POWER_ON);
		}

		ret = s3618_reflash_init(dev);
		if (ret < 0) {
			TOUCH_E("Failed to init reflash module\n");
			goto error;
		}

		ret = s3618_testing_init(dev);
		if (ret < 0) {
			TOUCH_E("Failed to init testing module\n");
			goto error;
		}

		ret = s3618_device_init(dev);
		if (ret < 0) {
			TOUCH_E("Failed to init device module\n");
			goto error;
		}

		ret = reflash_read_product_info();
		if (ret < 0) {
			TOUCH_E("Failed to read product info\n");
			ret = 0;
		}
	}

	if (atomic_read(&d->state.power) == POWER_OFF ||
			atomic_read(&d->state.power) == POWER_SLEEP) {
		TOUCH_I("%s: d.state->power is POWER_OFF / POWER_SLEEP\n", __func__);
		goto error;
	}

	ret = s3618_ic_info(dev);
	if (ts->force_fwup) {
		TOUCH_I("%s : Forcefully trigger f/w Upgrade\n", __func__);
		ret = s3618_upgrade(dev);
		if (ret) {
			TOUCH_E("Failed to f/w upgrade (ret: %d)\n", ret);
			goto error;
		}
		ts->force_fwup = 0;
		ret = s3618_ic_info(dev);
		if (ret < 0) {
			TOUCH_E("Failed to get ic info (ret: %d)\n", ret);
			goto error;
		}
	} else if (ret < 0) {
		TOUCH_E("Failed to get ic_info\n");
		goto error;
	}

	ret = s3618_set_report_config(dev);
	if (ret < 0) {
		TOUCH_E("Failed to set input parameters\n");
		goto error;
	}

	ret = s3618_get_report_config(dev);
	if (ret < 0) {
		TOUCH_E("Failed to get input parameters\n");
		goto error;
	}

	boot_mode = lge_get_mfts_mode();
	if (boot_mode == 1) {
		TOUCH_I("Skip to set dynamic config and lpwg in MFTS");
		atomic_set(&d->state.init, IC_INIT_DONE);
		return ret;
	}

	// Charger Status
	ret = s3618_set_dynamic_config(dev,
			DC_CHARGER_CONNECTED,
			(u16)atomic_read(&ts->state.connect));
	if (ret < 0) {
		TOUCH_E("failed to set %s %s (ret: %d)\n",
				STR(DC_CHARGER_CONNECTED),
				(u16)atomic_read(&ts->state.connect) ? "Enable" : "Disable",
				ret);
		goto error;
	}
	TOUCH_I("%s: %s %s\n", __func__,
			STR(DC_CHARGER_CONNECTED),
			atomic_read(&ts->state.connect) ? "Enable" : "Disable");

	// Wireless Status
	ret = s3618_set_dynamic_config(dev,
			DC_WIRELESS_CHARGER_CONNECTED,
			(u16)atomic_read(&ts->state.wireless));
	if (ret < 0) {
		TOUCH_E("failed to set %s %s (ret: %d)\n",
				STR(DC_WIRELESS_CHARGER_CONNECTED),
				(u16)atomic_read(&ts->state.wireless) ? "Enable" : "Disable",
				ret);
		//goto error;
	}
	TOUCH_I("%s: %s %s\n", __func__,
			STR(DC_WIRELESS_CHARGER_CONNECTED),
			atomic_read(&ts->state.wireless) ? "Enable" : "Disable");

	// IME Status
	ret = s3618_set_dynamic_config(dev,
			DC_IME_MODE_ENABLE,
			(u16)atomic_read(&ts->state.ime));
	if (ret < 0) {
		TOUCH_E("failed to set %s %s (ret: %d)\n",
				STR(DC_IME_MODE_ENABLE),
				(u16)atomic_read(&ts->state.ime) ? "Enable" : "Disable",
				ret);
		goto error;
	}
	TOUCH_I("%s: %s %s\n", __func__,
			STR(DC_IME_MODE_ENABLE),
			atomic_read(&ts->state.ime) ? "Enable" : "Disable");

	// Sensitive (Film) Status
	ret = s3618_set_dynamic_config(dev,
			DC_SENSITIVE_MODE_ENABLE,
			(u16)atomic_read(&ts->state.film));
	if (ret < 0) {
		TOUCH_E("failed to set %s %s (ret: %d)\n",
				STR(DC_SENSITIVE_MODE_ENABLE),
				(u16)atomic_read(&ts->state.film) ? "Enable" : "Disable",
				ret);
		goto error;
	}
	TOUCH_I("%s: %s %s\n", __func__,
			STR(DC_SENSITIVE_MODE_ENABLE),
			atomic_read(&ts->state.film) ? "Enable" : "Disable");

	// Dualscreen Status
	ret = s3618_set_dynamic_config(dev, DC_DUALSCREEN_CONNECTED, atomic_read(&ts->state.dualscreen));
	if (ret < 0) {
		TOUCH_E("failed to set %s %s (ret: %d)\n",
				STR(DC_DUALSCREEN_CONNECTED),
				atomic_read(&ts->state.dualscreen) ? "Enable" : "Disable",
				ret);
		goto error;
	}
	TOUCH_I("%s: %s %s\n", __func__,
			STR(DC_DUALSCREEN_CONNECTED),
			atomic_read(&ts->state.dualscreen) ? "Enable" : "Disable");

	atomic_set(&d->state.init, IC_INIT_DONE);

	ret = s3618_lpwg_mode(dev);
	if (ret < 0)
		TOUCH_E("failed to set lpwg mode (ret: %d)\n", ret);

error:

	return ret;
}

static int s3618_remove(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);

	TOUCH_TRACE();

	pm_qos_remove_request(&d->pm_qos_req);

	if (reflash_fhandler.initialized
		&& reflash_fhandler.insert) {
		reflash_fhandler.exp_fn->remove(dev);
		reflash_fhandler.initialized = false;
	}

	if (testing_fhandler.initialized
		&& testing_fhandler.insert) {
		testing_fhandler.exp_fn->remove(dev);
		testing_fhandler.initialized = false;
	}

	if (device_fhandler.initialized
		&& device_fhandler.insert) {
		device_fhandler.exp_fn->remove(dev);
		device_fhandler.initialized = false;
	}

	return 0;
}

static int s3618_shutdown(struct device *dev)
{
	struct s3618_data *d = to_s3618_data(dev);

	TOUCH_TRACE();

	s3618_power(dev, POWER_OFF);
	pm_qos_remove_request(&d->pm_qos_req);

	return 0;
}

static int s3618_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct s3618_data *d = NULL;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate s3618 data\n");
		return -ENOMEM;
	}

	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_power_init(dev);
	touch_bus_init(dev, 0);

	s3618_init_works(dev);
	s3618_init_locks(dev);

	s3618_init_tci_info(dev);
	s3618_init_swipe_info(dev);
	s3618_init_lpwg_abs_info(dev);
	s3618_init_ai_pick_info(dev);
	s3618_init_longpress_info(dev);
	s3618_init_onetap_info(dev);
	pm_qos_add_request(&d->pm_qos_req, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

	d->chunk_size = CHUNK_SIZE;
	d->lcd_mode = LCD_MODE_U3;
	d->report_touch = s3618_report_touch;

#if defined(CONFIG_SECURE_TOUCH)
	ts->touch_ic_name = "synaptics,s3618";
#endif

	return 0;
}

static struct touch_driver touch_driver = {
	.probe = s3618_probe,
	.remove = s3618_remove,
	.shutdown = s3618_shutdown,
	.suspend = s3618_suspend,
	.resume = s3618_resume,
	.init = s3618_init,
	.upgrade = s3618_upgrade,
	.irq_handler = s3618_irq_handler,
	.power = s3618_power,
	.lpwg = s3618_lpwg,
	.notify = s3618_notify,
	.init_pm = s3618_init_pm,
	.esd_recovery = s3618_esd_recovery,
	.swipe_enable = s3618_swipe_enable,
	.register_sysfs = s3618_register_sysfs,
	.set = s3618_set,
	.get = s3618_get,
};

#define MATCH_NAME			"synaptics,common"

static const struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
	{},
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_I2C,
	.name = LGE_TOUCH_NAME,
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(touch_match_ids),
};

static int __init touch_device_init(void)
{
	TOUCH_TRACE();

	if (is_ddic_name("r66456") || is_ddic_name("r66456a")) {
		TOUCH_I("%s, r66456 Tianma s3618 detected\n", __func__);
	} else {
		TOUCH_I("%s, s3618 not found.\n", __func__);
		return 0;
	}

	return touch_bus_device_init(&hwif, &touch_driver);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
module_exit(touch_device_exit);

MODULE_AUTHOR("BSP-TOUCH@lge.com");
MODULE_DESCRIPTION("LGE touch driver v5");
MODULE_LICENSE("GPL");
