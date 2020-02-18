/*
 * ESS014 Driver for ALSA
 *
 * Copyright (C) 2019 LG Electronics, Inc.
 * Author: Hansun Lee <hansun.lee@lge.com>
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

#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/audio-v2.h>
#include <linux/debugfs.h>

#include <sound/core.h>
#include <sound/control.h>

#include "usbaudio.h"
#include "mixer.h"
#include "helper.h"

#include "mixer_ess014.h"

static ssize_t snd_ess014_read(struct snd_usb_audio *chip,
		u8 addr, u8 *buf, size_t size)
{
	struct snd_ess014_packet_header hdr = {
		.opcode = SND_ESS014_OPCODE_READ,
		.addr = {
			.msb = SND_ESS014_ADDR_MSB,
			.lsb = addr,
		},
	};
	u8 *p = buf;
	int remain = size;
	int ret;

	usb_audio_dbg(chip, "%s: addr:%02x size:%d\n", __func__,
		      addr, size);

	ret = snd_usb_lock_shutdown(chip);
	if (ret < 0)
		return ret;

	while (remain > 0) {
		if (hdr.addr.lsb < addr)
			break;

		hdr.length = min(remain,
				 min((int)SND_ESS014_PACKET_DATA_SIZE,
				     (int)(SND_ESS014_REG_MAX -
					   hdr.addr.lsb + 1)));

		ret = snd_usb_ctl_msg(chip->dev,
				usb_sndctrlpipe(chip->dev, 0),
				0x09,
				USB_DIR_OUT | USB_TYPE_CLASS |
				USB_RECIP_INTERFACE,
				0x0200,
				0x0003,
				&hdr,
				SND_ESS014_PACKET_HDR_SIZE);
		if (ret < 0)
			goto err;

		print_hex_dump(KERN_DEBUG, "OUT ", DUMP_PREFIX_OFFSET, 16, 1,
			       &hdr, SND_ESS014_PACKET_HDR_SIZE, 0);

		ret = snd_usb_ctl_msg(chip->dev,
				usb_sndctrlpipe(chip->dev, 0),
				0x01,
				USB_DIR_IN | USB_TYPE_CLASS |
				USB_RECIP_INTERFACE,
				0x0100,
				0x0003,
				p,
				hdr.length);
		if (ret < 0)
			goto err;

		print_hex_dump(KERN_DEBUG, "IN  ", DUMP_PREFIX_OFFSET, 16, 1,
			       p, hdr.length, 0);

		remain -= hdr.length;
		p += hdr.length;
		hdr.addr.lsb += hdr.length;
	}

	ret = size - remain;
err:
	snd_usb_unlock_shutdown(chip);
	return ret;
}

static ssize_t snd_ess014_write(struct snd_usb_audio *chip,
		u8 addr, const u8 *buf, size_t size)
{
	struct snd_ess014_packet pkt = {
		.hdr = {
			.opcode = SND_ESS014_OPCODE_WRITE,
			.addr = {
				.msb = SND_ESS014_ADDR_MSB,
				.lsb = addr,
			},
		},
	};
	struct snd_ess014_packet_header *hdr = &pkt.hdr;
	const u8 *p = buf;
	int remain = size;
	int ret;

	usb_audio_dbg(chip, "%s: addr:%02x size:%d\n", __func__,
		      addr, size);

	ret = snd_usb_lock_shutdown(chip);
	if (ret < 0)
		return ret;

	while (remain > 0) {
		if (hdr->addr.lsb < addr)
			break;

		hdr->length = min(remain,
				  min((int)SND_ESS014_PACKET_DATA_SIZE,
				      (int)(SND_ESS014_REG_MAX -
					    hdr->addr.lsb + 1)));

		memcpy(pkt.data, p, hdr->length);

		ret = snd_usb_ctl_msg(chip->dev,
				usb_sndctrlpipe(chip->dev, 0),
				0x09,
				USB_DIR_OUT | USB_TYPE_CLASS |
				USB_RECIP_INTERFACE,
				0x0200,
				0x0003,
				&pkt,
				SND_ESS014_PACKET_HDR_SIZE + hdr->length);
		if (ret < 0)
			goto err;

		print_hex_dump(KERN_DEBUG, "OUT ", DUMP_PREFIX_OFFSET, 16, 1,
			       &pkt,
			       SND_ESS014_PACKET_HDR_SIZE + hdr->length,
			       0);

		remain -= hdr->length;
		p += hdr->length;
		hdr->addr.lsb += hdr->length;
	}

	ret = size - remain;
err:
	snd_usb_unlock_shutdown(chip);
	return ret;
}

static ssize_t snd_ess014_masked_write(struct snd_usb_audio *chip,
		u8 addr, u8 mask, u8 val)
{
	u8 tmp;
	u8 orig;
	int ret;

	ret = snd_ess014_read(chip, addr, &orig, 1);
	if (ret < 0)
		goto err;

	tmp = orig & ~mask;
	tmp |= val & mask;

	ret = snd_ess014_write(chip, addr, &tmp, 1);
	if (ret < 0)
		goto err;

err:
	return ret;
}

static int snd_ess014_filter_settings_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	static const char *const texts[] = {
		"Linear Phase Fast Roll-Off",
		"Minimum Phase Slow Roll-Off",
		"Hybrid Min Phase Fast Roll-Off",
		"Custom",
	};

	return snd_ctl_enum_info(uinfo, 1, ARRAY_SIZE(texts), texts);
}

static int snd_ess014_filter_settings_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kcontrol->private_data;

	ucontrol->value.integer.value[0] = elem->cache_val[0];

	return 0;
}

static int snd_ess014_filter_settings_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kcontrol->private_data;
	struct snd_usb_audio *chip = elem->head.mixer->chip;
	int val;
	u8 val_org;
	int ret;

	val = ucontrol->value.enumerated.item[0];

	if (val < 0 || val > 3)
		return -EINVAL;

	if (val < 3)
		val_org = val * 3;
	else
		val_org = 0x0E;

	ret = snd_ess014_masked_write(chip, elem->head.id,
				      SND_ESS014_DAC_FILTER_SHAPE_MASK,
				      val_org);
	if (ret > 0) {
		elem->cached = 1;
		elem->cache_val[0] = val;
	} else {
		usb_audio_dbg(chip, "Failed to set filter %d\n", ret);
	}

	return ret > 0 ? 1 : 0;
}

static struct snd_kcontrol_new snd_ess014_filter_settings_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.count = 1,
	.info = snd_ess014_filter_settings_info,
	.get = snd_ess014_filter_settings_get,
	.put = snd_ess014_filter_settings_put,
};

static int snd_ess014_dac_analog_volume_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	struct usb_mixer_elem_info *elem = kcontrol->private_data;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = elem->channels;
	uinfo->value.integer.max = 24;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.step = 1;
	return 0;
}

static int snd_ess014_dac_analog_volume_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kcontrol->private_data;
	int i;

	for (i = 0; i < elem->channels; i++)
		ucontrol->value.integer.value[i] = elem->cache_val[i];

	return 0;
}

static int snd_ess014_dac_analog_volume_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kcontrol->private_data;
	struct snd_usb_audio *chip = elem->head.mixer->chip;
	int val;
	int i;
	int changed = 0;
	int ret;

	for (i = 0; i < elem->channels; i++) {
		val = ucontrol->value.integer.value[i];
		if (val != elem->cache_val[i]) {
			ret = snd_ess014_masked_write(chip,
					elem->head.id + i,
					SND_ESS014_DAC_ANALOG_VOLUME_MASK,
					val);
			if (ret < 0)
				return ret;

			elem->cached |= 1 << i;
			elem->cache_val[i] = val;

			changed = 1;
		}
	}

	return changed;
}

static struct snd_kcontrol_new snd_ess014_dac_analog_volume_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.count = 2,
	.info = snd_ess014_dac_analog_volume_info,
	.get = snd_ess014_dac_analog_volume_get,
	.put = snd_ess014_dac_analog_volume_put,
};

static int add_new_ctl(struct usb_mixer_interface *mixer,
		const struct snd_kcontrol_new *ncontrol,
		int index, int val_type, int channels,
		const char *name, void *opt,
		struct usb_mixer_elem_info **elem_ret)
{
	struct snd_kcontrol *kctl;
	struct usb_mixer_elem_info *elem;
	int ret;

	usb_audio_dbg(mixer->chip, "%s: add mixer %s\n", __func__, name);

	elem = kzalloc(sizeof(*elem), GFP_KERNEL);
	if (!elem)
		return -ENOMEM;

	elem->head.mixer = mixer;
	elem->head.id = index;
	elem->head.resume = NULL;
	elem->control = 0;
	elem->idx_off = 0;
	elem->channels = channels;
	elem->val_type = val_type;
	elem->private_data = opt;

	kctl = snd_ctl_new1(ncontrol, elem);
	if (!kctl) {
		kfree(elem);
		return -ENOMEM;
	}
	kctl->private_free = snd_usb_mixer_elem_free;

	strlcpy(kctl->id.name, name, sizeof(kctl->id.name));

	ret = snd_usb_mixer_add_control(&elem->head, kctl);
	if (ret < 0)
		return ret;

	if (elem_ret)
		*elem_ret = elem;

	return 0;
}

#ifdef CONFIG_DEBUG_FS
u8 snd_ess014_dump_address = 0;
u8 snd_ess014_dump_count = 1;

#define DEBUGFS_TOT_LEN (2 + 2 + 3) /* address + data + : \n */

static ssize_t snd_ess014_read_debugfs(struct snd_usb_audio *chip,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	char *buf;
	size_t buf_pos = 0;
	loff_t p = *ppos;
	int i;
	unsigned int start_reg;
	u8 val;
	int ret;

	if (*ppos < 0 || !count)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	start_reg = *ppos / DEBUGFS_TOT_LEN;

	for (i = start_reg; i <= SND_ESS014_REG_MAX; i++) {
		if (p >= *ppos) {
			/* ...but not beyond it */
			if ((buf_pos + DEBUGFS_TOT_LEN) > count)
				break;

			/* Format the register */
			snprintf(buf + buf_pos, count - buf_pos, "%02x: ", i);
			buf_pos += 4;

			/* Format the value, write all X if we can't read */
			ret = snd_ess014_read(chip, i, &val, 1);
			if (ret < 0)
				memset(buf + buf_pos, 'X', 2);
			else
				snprintf(buf + buf_pos, count - buf_pos,
					 "%02x", val);
			buf_pos += 2;

			buf[buf_pos++] = '\n';
		}
		p += DEBUGFS_TOT_LEN;
	}

	ret = buf_pos;

	if (copy_to_user(user_buf, buf, buf_pos)) {
		ret = -EFAULT;
		goto out;
	}

	*ppos += buf_pos;

out:
	kfree(buf);
	return ret;
}

static ssize_t snd_ess014_map_read_file(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	struct snd_usb_audio *chip = file->private_data;

	return snd_ess014_read_debugfs(chip, user_buf, count, ppos);
}

static const struct file_operations snd_ess014_map_fops = {
	.open = simple_open,
	.read = snd_ess014_map_read_file,
	.llseek = default_llseek,
};

static ssize_t snd_ess014_data_read_file(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	struct snd_usb_audio *chip = file->private_data;
	int new_count;

	new_count = snd_ess014_dump_count * DEBUGFS_TOT_LEN;
	if (new_count > count)
		new_count = count;

	if (*ppos == 0)
		*ppos = snd_ess014_dump_address * DEBUGFS_TOT_LEN;
	else if (*ppos >= (snd_ess014_dump_address * DEBUGFS_TOT_LEN +
			   snd_ess014_dump_count * DEBUGFS_TOT_LEN))
		return 0;

	return snd_ess014_read_debugfs(chip, user_buf, new_count, ppos);
}

static ssize_t snd_ess014_data_write_file(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct snd_usb_audio *chip = file->private_data;
	char buf[32];
	size_t buf_size;
	char *start = buf;
	u8 value;
	int ret;

	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	while (*start == ' ')
		start++;
	if (kstrtou8(start, 16, &value))
		return -EINVAL;

	ret = snd_ess014_write(chip, snd_ess014_dump_address, &value, 1);
	if (ret < 0)
		return ret;

	return buf_size;
}

static const struct file_operations snd_ess014_data_fops = {
	.open = simple_open,
	.read = snd_ess014_data_read_file,
	.write = snd_ess014_data_write_file,
	.llseek = default_llseek,
};

static void snd_ess014_debugfs_exit(void *data)
{
	struct dentry *root = data;

	pr_debug("%s\n", __func__);

	debugfs_remove_recursive(root);
}

static void snd_ess014_debugfs_init(struct snd_usb_audio *chip)
{
	struct dentry *root;
	struct dentry *file;
	int ret;

	usb_audio_dbg(chip, "%s\n", __func__);

	root = debugfs_create_dir("snd_ess014", NULL);
	if (IS_ERR_OR_NULL(root)) {
		if (!root)
			usb_audio_err(chip, "Can't crate debugfs root\n");
		return;
	}

	file = debugfs_create_file("registers", 0400, root, chip,
				   &snd_ess014_map_fops);
	if (!file)
		usb_audio_dbg(chip, "Can't create debugfs registers\n");

	file = debugfs_create_x8("address", 0600, root,
				  &snd_ess014_dump_address);
	if (!file)
		usb_audio_dbg(chip, "Can't create debugfs address\n");

	file = debugfs_create_u8("count", 0600, root,
				  &snd_ess014_dump_count);
	if (!file)
		usb_audio_dbg(chip, "Can't create debugfs count\n");

	file = debugfs_create_file("data", 0600, root, chip,
				   &snd_ess014_data_fops);
	if (!file)
		usb_audio_dbg(chip, "Can't create debugfs data\n");

	ret = devm_add_action(&(chip)->dev->dev,
			      snd_ess014_debugfs_exit, root);
	if (ret < 0) {
		debugfs_remove_recursive(root);
		usb_audio_err(chip,
			      "Failed to add debugfs cleanup action %d\n",
			      ret);
	}
}
#else
static void snd_ess014_debugfs_init(struct snd_usb_audio *chip) { }
#endif

int snd_ess014_controls_create(struct usb_mixer_interface *mixer)
{

	struct snd_usb_audio *chip = mixer->chip;
	struct usb_mixer_elem_info *elem;
	u8 val;
	int ret;

	usb_audio_info(mixer->chip, "%s\n", __func__);

	/*
	 * setup filter settings control
	 */
	ret = add_new_ctl(mixer,
			  &snd_ess014_filter_settings_ctl,
			  SND_ESS014_FILTER_SETTINGS_REG,
			  USB_MIXER_U8,
			  1,
			  "Filter Settings",
			  NULL,
			  NULL);
	if (ret < 0)
		return ret;

	/*
	 * setup dac analog volume control
	 */
	ret = add_new_ctl(mixer,
			  &snd_ess014_dac_analog_volume_ctl,
			  SND_ESS014_ANALOG_GAIN_CONTROL_1_REG,
			  USB_MIXER_U8,
			  2,
			  "DAC Analog Volume",
			  NULL,
			  &elem);
	if (ret < 0)
		return ret;

	// allow independent L/R analog gain control
	ret = snd_ess014_masked_write(chip,
			SND_ESS014_ANALOG_GAIN_CONTROL_1_REG,
			SND_ESS014_DAC_ANALOG_USE_SECONDARY_BIT |
			SND_ESS014_DAC_ANALOG_VOLUME_MASK,
			SND_ESS014_DAC_ANALOG_USE_SECONDARY_BIT);
	if (ret < 0)
		return ret;

	ret = snd_ess014_masked_write(chip,
			SND_ESS014_ANALOG_GAIN_CONTROL_2_REG,
			SND_ESS014_DAC_ANALOG_VOLUME_MASK, 0);
	if (ret < 0)
		return ret;

	snd_ess014_debugfs_init(chip);

#ifdef DEBUG
	do {
		ret = snd_ess014_read(chip, SND_ESS014_CHIP_ID_REG,
				      &val, 1);
		if (ret) {
			usb_audio_dbg(chip, "ChipID: %02x\n",
				      val & SND_ESS014_CHIP_ID_MASK);
		}
	} while (0);
#endif

	return 0;
}
