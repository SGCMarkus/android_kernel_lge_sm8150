/* touch_ft3518.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hyokmin.kwon@lge.com
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
#define TS_MODULE "[ft3518]"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/delay.h>
//#include <linux/regulator/consumer.h>
//#include <linux/dma-mapping.h>
//#include <linux/async.h>
//#include <mach/board_lge.h>
//#include <linux/i2c.h>
//#include <touch_i2c.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>

/*
 *  Include to Local Header File
 */
#include "touch_ft3518.h"

int ft3518_cmd_read(struct device *dev, void *cmd_data, int cmd_len, void *read_buf, int read_len)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft3518_data *d = to_ft3518_data(dev);
	struct touch_bus_msg msg = {0, };
	int ret = 0;

	mutex_lock(&d->rw_lock);

	memcpy(&ts->tx_buf[0], cmd_data, cmd_len);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = cmd_len;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = read_len;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	memcpy(read_buf, &ts->rx_buf[0], read_len);
	mutex_unlock(&d->rw_lock);
	return 0;

}

int ft3518_reg_read(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft3518_data *d = to_ft3518_data(dev);
	struct touch_bus_msg msg = {0, };
	int ret = 0;

	mutex_lock(&d->rw_lock);

	ts->tx_buf[0] = addr;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = size;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], size);
	mutex_unlock(&d->rw_lock);
	return 0;

}

int ft3518_reg_write(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft3518_data *d = to_ft3518_data(dev);
	struct touch_bus_msg msg = {0, };
	int ret = 0;

	mutex_lock(&d->rw_lock);

	ts->tx_buf[0] = addr;
	memcpy(&ts->tx_buf[1], data, size);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = size + 1;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	mutex_unlock(&d->rw_lock);

	return 0;
}

static void ft3518_init_locks(struct ft3518_data *d)
{
	mutex_init(&d->rw_lock);
	mutex_init(&d->fb_lock);
}

int ft3518_ic_info(struct device *dev)
{
	struct ft3518_data *d = to_ft3518_data(dev);

	int ret = 0;
	u8 chip_id = 0;
	u8 chip_id_low = 0;
	u8 is_official = 0;
	u8 fw_version = 0;
	u8 fw_version_minor = 0;
	u8 fw_version_sub_minor = 0;
	u8 fw_vendor_id = 0;
	u8 lib_version_high = 0;
	u8 lib_version_low = 0;
	int i = 0;

	// If it is failed to get info without error, just return error
	for (i = 0; i < 2; i++) {
		ret |= ft3518_reg_read(dev, FTS_REG_ID, (u8 *)&chip_id, 1);
		ret |= ft3518_reg_read(dev, FTS_REG_ID_LOW, (u8 *)&chip_id_low, 1);
		ret |= ft3518_reg_read(dev, FTS_REG_FW_VER, (u8 *)&fw_version, 1);
		ret |= ft3518_reg_read(dev, FTS_REG_FW_VER_MINOR, (u8 *)&fw_version_minor, 1);
		ret |= ft3518_reg_read(dev, FTS_REG_FW_VER_SUB_MINOR, (u8 *)&fw_version_sub_minor, 1);
		ret |= ft3518_reg_read(dev, FTS_REG_FW_VENDOR_ID, (u8 *)&fw_vendor_id, 1);
		ret |= ft3518_reg_read(dev, FTS_REG_LIB_VER_H, (u8 *)&lib_version_high, 1);
		ret |= ft3518_reg_read(dev, FTS_REG_LIB_VER_L, (u8 *)&lib_version_low, 1);

		if (ret == 0) {
			TOUCH_I("Success to get ic info data\n");
			break;
		}
	}

	if (i >= 2) {
		TOUCH_E("Failed to get ic info data, (need to recover it?)\n");
		return -EPERM; // Do nothing in caller
	}

	is_official = (fw_version & 0x80) >> 7;
	fw_version &= 0x7F;
	d->ic_info.version.major = fw_version;
	d->ic_info.version.minor = fw_version_minor;
	d->ic_info.version.sub_minor = fw_version_sub_minor;

	d->ic_info.chip_id = chip_id; // Device ID
	d->ic_info.chip_id_low = chip_id_low;
	d->ic_info.is_official = is_official;
	d->ic_info.fw_version = fw_version; // Major
	d->ic_info.fw_vendor_id = fw_vendor_id; // Vendor ID
	d->ic_info.lib_version_high = lib_version_high;
	d->ic_info.lib_version_low = lib_version_low;

	d->ic_info.info_valid = 1;

	TOUCH_I("chip_id:%x, chip_id_low:%x, is_official:%d, fw_version:%d.%d.%d, fw_vendor_id:%x, lib_version_high:%x, lib_version_low:%x\n",
		chip_id, chip_id_low, is_official, fw_version, fw_version_minor, fw_version_sub_minor, fw_vendor_id, lib_version_high, lib_version_low);

	return ret;
}

static int ft3518_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft3518_data *d = NULL;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate ft3518 data\n");
		return -ENOMEM;
	}

	d->dev = dev;
	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_power_init(dev);

	touch_bus_init(dev, MAX_BUF_SIZE);

	ft3518_init_locks(d);

	return 0;
}

static int ft3518_remove(struct device *dev)
{
	TOUCH_TRACE();
	return 0;
}

static int ft3518_fw_compare(struct device *dev, const struct firmware *fw)
{
	struct ft3518_data *d = to_ft3518_data(dev);
	u8 ic_fw_version = d->ic_info.fw_version;
	u8 ic_is_official = d->ic_info.is_official;
	u8 bin_fw_version = 0;
	u8 bin_is_official = 0;
	int update = 0;

	if (d->ic_info.info_valid == 0) { // Failed to get ic info
		TOUCH_I("invalid ic info, skip fw upgrade\n");
		return 0;
	}

	bin_fw_version = fw->data[0x5100+0x0e];
	bin_is_official = (bin_fw_version & 0x80) >> 7;
	bin_fw_version &= 0x7F;

	// IF fw ver of bin is not initialized
	if (d->ic_info.fw_version_bin == 0 && d->ic_info.is_official_bin == 0) {
		d->ic_info.fw_version_bin = bin_fw_version;
		d->ic_info.is_official_bin = bin_is_official;
	}

	if ((ic_is_official != bin_is_official) || (ic_fw_version != bin_fw_version))
		update = 1;

	TOUCH_I("%s : binary[V%d.%d] device[V%d.%d] -> update: %d\n", __func__,
		bin_is_official, bin_fw_version, ic_is_official, ic_fw_version,
		update);

	return update;
}

static int ft3518_fwboot_upgrade(struct device *dev, const struct firmware *fw_boot)
{
	const u8 *fw_data = fw_boot->data;
	u32 fw_size = (u32)(fw_boot->size);
	u8 *fw_check_buf = NULL;
	u8 i2c_buf[FTS_PACKET_LENGTH + 12] = {0,};
	int ret;
	int packet_num, i, j, packet_addr, packet_len;
	u8 pramboot_ecc;

	TOUCH_I("%s - START\n", __func__);

	if (fw_size > 0x10000 || fw_size == 0)
		return -EIO;

	fw_check_buf = kmalloc(fw_size + 1, GFP_ATOMIC);
	if (fw_check_buf == NULL)
		return -ENOMEM;

	for (i = 12; i <= 30; i++) {
		// Reset CTPM
		// [Yousung] Need HW Reset before upgrade?
//		touch_gpio_direction_output(ts->reset_pin, 0);
//		touch_msleep(50);
//		touch_gpio_direction_output(ts->reset_pin, 1);
//		touch_msleep(i);

		// Set Upgrade Mode
		ret = ft3518_reg_write(dev, 0x55, i2c_buf, 0);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
		touch_msleep(1);

		// Check ID
		TOUCH_I("%s - Set Upgrade Mode and Check ID : %d ms\n", __func__, i);
		ret = ft3518_reg_read(dev, 0x90, i2c_buf, 2);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}

		TOUCH_I("Check ID : 0x%x , 0x%x\n", i2c_buf[0], i2c_buf[1]);

		if (i2c_buf[0] == 0x80 && i2c_buf[1] == 0x06) {
			touch_msleep(50);
			break;
		}
	}

	if (i > 30) {
		TOUCH_E("timeout to set upgrade mode\n");
		goto FAIL;
	}

	// Write F/W (Pramboot) Binary to CTPM
	TOUCH_I("%s - Write F/W (Pramboot)\n", __func__);
	pramboot_ecc = 0;
	packet_num = (fw_size + FTS_PACKET_LENGTH - 1) / FTS_PACKET_LENGTH;
	for (i = 0; i < packet_num; i++) {
		packet_addr = i * FTS_PACKET_LENGTH;
		i2c_buf[0] = 0; //(u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[1] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[2] = (u8)((u32)(packet_addr) & 0x000000FF);
		if (packet_addr + FTS_PACKET_LENGTH > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = FTS_PACKET_LENGTH;
		i2c_buf[3] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
		i2c_buf[4] = (u8)((u32)(packet_len) & 0x000000FF);
		for (j = 0; j < packet_len; j++) {
			i2c_buf[5 + (j/4)*4 + (3 - (j%4))] = fw_data[packet_addr + j];
			pramboot_ecc ^= i2c_buf[5 + j];
		}
		TOUCH_I("#%d : Writing to %d , %d bytes\n", i, packet_addr, packet_len); //kjh
		ret = ft3518_reg_write(dev, 0xAE, i2c_buf, packet_len + 5);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
	}

	touch_msleep(100);

	// Verify F/W
	TOUCH_I("%s - Verify\n", __func__);
	for (i = 0; i < packet_num; i++) {
		i2c_buf[0] = 0x85;
		packet_addr = i * FTS_PACKET_LENGTH;
		i2c_buf[1] = 0; //(u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[2] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[3] = (u8)((u32)(packet_addr) & 0x000000FF);
		if (packet_addr + FTS_PACKET_LENGTH > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = FTS_PACKET_LENGTH;
		TOUCH_I("#%d : Reading from %d , %d bytes\n", i, packet_addr, packet_len);  //kjh
		ret = ft3518_cmd_read(dev, i2c_buf, 4, fw_check_buf+packet_addr, packet_len);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
	}

	for (i = 0; i < fw_size; i++) {
		//if (fw_check_buf[i] != fw_data[i]) {
		//TOUCH_I("%s - fw_check_buf : %d  fw_data : %d!!\n", __func__, fw_check_buf[i], fw_data[i]); //kjh
			//goto FAIL;
		//}
	}

	for (i = 0; i < fw_size; i++) {
		if (fw_check_buf[i] != fw_data[i]) {
			TOUCH_I("%s - Verify Failed %d %d %d!!\n", __func__, i, fw_check_buf[i], fw_data[i]); //kjh
			goto FAIL;
		}
	}

	TOUCH_I("%s - Verify OK !!\n", __func__);

	// Start App
	TOUCH_I("%s - Start App\n", __func__);
	ret = ft3518_reg_write(dev, 0x08, i2c_buf, 0);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

	touch_msleep(10);

	if (fw_check_buf)
		kfree(fw_check_buf);

	TOUCH_I("===== Firmware (Pramboot) download Okay =====\n");

	return 0;

FAIL:

	if (fw_check_buf)
		kfree(fw_check_buf);

	TOUCH_I("===== Firmware (Pramboot) download FAIL!!! =====\n");

	return -EIO;
}

static int ft3518_fw_upgrade(struct device *dev, const struct firmware *fw)
{
	const u8 *fw_data = fw->data;
	u32 fw_size = (u32)(fw->size);
	u8 i2c_buf[FTS_PACKET_LENGTH + 12] = {0,};
	int ret;
	int packet_num, retry, i, j, packet_addr, packet_len;
	u8 reg_val_id = 0;
	u8 fw_ecc;

	TOUCH_I("%s - START\n", __func__);

	for (i = 0; i < 30; i++) {
		// Enter Upgrade Mode
		TOUCH_I("%s - Enter Upgrade Mode and Check ID\n", __func__);

		ret = ft3518_reg_write(dev, 0x55, i2c_buf, 0);  //Enter upgrade mode of ParamBoot

		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return -EIO;
		}
		touch_msleep(1);

		// Check ID
		ret = ft3518_reg_read(dev, 0x90, i2c_buf, 2);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return -EIO;
		}

		TOUCH_I("Check ID [%d] : 0x%x , 0x%x\n", i, i2c_buf[0], i2c_buf[1]);   // 0x80, 0xC6
		if (i2c_buf[0] == 0x80 && i2c_buf[1] == 0xC6)
			break;

		touch_msleep(10);
	}

	if (i == 30) {
		TOUCH_E("timeout to set upgrade mode\n");
		goto FAIL;
	}

	// Change to write flash mode
	i2c_buf[0] = reg_val_id;
	i2c_buf[1] = 0x00;
	//i2c_buf[1] = 0x01;
	ret = ft3518_reg_write(dev, 0x05, i2c_buf, 2);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

	i2c_buf[0] = 0x80;  //
	i2c_buf[1] = 0x01;
	ret = ft3518_reg_write(dev, 0x05, i2c_buf, 2);

//- Flash : 05
//- Flashe type : 81:FT5003, 80: Winbond
//- Clk speed : 00:48M 01:24M

// Change to write flash set range
	i2c_buf[0] = 0x0A;
	ret = ft3518_reg_write(dev, 0x09, i2c_buf, 1);
//- 0x0A : All
//- 0x0B : App
//- 0x0C : Lcd

	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	touch_msleep(50);

	// Erase App (Panel Parameter Area??)
	TOUCH_I("%s - Erase App and Panel Parameter Area\n", __func__);

	ret = ft3518_reg_write(dev, 0x61, i2c_buf, 0);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	touch_msleep(1000);

	retry = 300;
	i = 0;
	do {
		ret = ft3518_reg_read(dev, 0x6A, i2c_buf, 2);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}

		if (i2c_buf[0] == 0xF0 && i2c_buf[1] == 0xAA) {
			TOUCH_I("Erase Done : %d\n", i);
			break;
		}
		i++;
		//touch_msleep(50);
		mdelay(10);
	} while (--retry);

	// Write F/W (App) Binary to CTPM
	TOUCH_I("%s - Write F/W (App)\n", __func__);
	fw_ecc = 0;
	packet_num = (fw_size + FTS_PACKET_LENGTH - 1) / FTS_PACKET_LENGTH;
	for (i = 0; i < packet_num; i++) {
		packet_addr = i * FTS_PACKET_LENGTH;
		i2c_buf[0] = (u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[1] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[2] = (u8)((u32)(packet_addr) & 0x000000FF);
		if (packet_addr + FTS_PACKET_LENGTH > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = FTS_PACKET_LENGTH;
		i2c_buf[3] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
		i2c_buf[4] = (u8)((u32)(packet_len) & 0x000000FF);
		for (j = 0; j < packet_len; j++) {
			i2c_buf[5 + j] = fw_data[packet_addr + j];
			fw_ecc ^= i2c_buf[5 + j];
		}

		ret = ft3518_reg_write(dev, 0xBF, i2c_buf, packet_len + 5);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}

		// Waiting
		retry = 100;
		do {
			ret = ft3518_reg_read(dev, 0x6A, i2c_buf, 2);
			if (ret < 0) {
				TOUCH_E("i2c error\n");
				goto FAIL;
			}
			if ((u32)(i + 0x1000) == (((u32)(i2c_buf[0]) << 8) | ((u32)(i2c_buf[1])))) {
				if ((i & 0x007F) == 0)
					TOUCH_I("Write Done : %d / %d\n", i+1, packet_num);

				break;
			}
			//touch_msleep(1);
			mdelay(1);
		} while (--retry);
		if (retry == 0) {
			TOUCH_I("Write Fail : %d / %d : [0x%02x] , [0x%02x]\n", i+1, packet_num, i2c_buf[0], i2c_buf[1]);
			//goto FAIL;
		}
	}
	TOUCH_I("Write Finished : Total %d\n", packet_num);

	touch_msleep(50);

	// Read out Checksum
	TOUCH_I("%s - Read out checksum (App) for %d bytes\n", __func__, fw_size);
	ret = ft3518_reg_write(dev, 0x64, i2c_buf, 0);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	touch_msleep(300);

	packet_num = (fw_size + LEN_FLASH_ECC_MAX - 1) / LEN_FLASH_ECC_MAX;
	for (i = 0; i < packet_num; i++) {
		packet_addr = i * LEN_FLASH_ECC_MAX;
		i2c_buf[0] = (u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[1] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[2] = (u8)((u32)(packet_addr) & 0x000000FF);
		if (packet_addr + LEN_FLASH_ECC_MAX > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = LEN_FLASH_ECC_MAX;
		i2c_buf[3] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
		i2c_buf[4] = (u8)((u32)(packet_len) & 0x000000FF);
		ret = ft3518_reg_write(dev, 0x65, i2c_buf, 5);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
		touch_msleep(fw_size/256);

		retry = 200;
		do {
			ret = ft3518_reg_read(dev, 0x6A, i2c_buf, 2);
			if (ret < 0) {
				TOUCH_E("i2c error\n");
				goto FAIL;
			}

			if (i2c_buf[0] == 0xF0 && i2c_buf[1] == 0x55) {
				TOUCH_I("Checksum Calc. Done\n");
				break;
			}
			touch_msleep(1);
		} while (--retry);
	}

	ret = ft3518_reg_read(dev, 0xCC, i2c_buf, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	TOUCH_I("Reg 0xCC : 0x%x\n", i2c_buf[0]);

	ret = ft3518_reg_read(dev, 0x66, i2c_buf, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	TOUCH_I("Reg 0x66 : 0x%x\n", i2c_buf[0]);

	if (i2c_buf[0] != fw_ecc) {
		TOUCH_E("Checksum ERROR : Reg 0x66 [0x%x] , fw_ecc [0x%x]\n", i2c_buf[0], fw_ecc);
		goto FAIL;
	}

	TOUCH_I("Checksum OK : Reg 0x66 [0x%x] , fw_ecc [0x%x]\n", i2c_buf[0], fw_ecc);

	TOUCH_I("===== Firmware download OK!!! =====\n");

	/*SW Reset CMD*/
	ret = ft3518_reg_write(dev, 0x07, i2c_buf, 0);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	/*After SW Reset, not need HW_Reset_Delay becuz it wiil take in CORE side*/

	return 0;

FAIL:

	TOUCH_I("===== Firmware download FAIL!!! =====\n");

	// Reset Anyway
	// [Yousung] Need reset after fw upgrade if fail
//	ft3518_power(dev, POWER_OFF);
//	ft3518_power(dev, POWER_ON);

	return -EIO;

}

static int ft3518_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fw = NULL;
	const struct firmware *fw_boot = NULL;
	char fwpath[256] = {0};
	int ret = 0, retry = 0;
//	int i = 0;

	TOUCH_TRACE();

	return 0;

	if (ts->test_fwpath[0] && (ts->test_fwpath[0] != 't' && ts->test_fwpath[1] != 0)) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n", &ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		memcpy(fwpath, ts->def_fwpath[1], sizeof(fwpath)); // 0 : pramboot bin, 1 : ft3518 all bin
		TOUCH_I("get fwpath from def_fwpath : %s\n", fwpath);
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	fwpath[sizeof(fwpath)-1] = '\0';

	if (strlen(fwpath) <= 0) {
		TOUCH_E("error get fw path\n");
		return -EPERM;
	}

	TOUCH_I("fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, dev);

	if (ret) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", fwpath, ret);
		goto error;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	if (ft3518_fw_compare(dev, fw)) {

		TOUCH_I("fwpath_boot[%s]\n", ts->def_fwpath[0]);  // PARAMBOOT.BIN
		ret = request_firmware(&fw_boot, ts->def_fwpath[0], dev);
		if (ret) {
			TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", ts->def_fwpath[0], ret);
			goto error;
		}

		do {
			ret = ft3518_fwboot_upgrade(dev, fw_boot);
			ret += ft3518_fw_upgrade(dev, fw);

			if (ret >= 0)
				break;

			TOUCH_E("fail to upgrade f/w - ret: %d, retry: %d\n", ret, retry);
		} while (++retry < 3);

		if (retry >= 3) {
			goto error;
		}
		TOUCH_I("f/w upgrade complete\n");
	}
error:
	if(fw_boot ==  NULL)
		release_firmware(fw);
	else{
		release_firmware(fw);
		release_firmware(fw_boot);
	}

	return -EPERM; // Return non-zero to not reset
}

int ft3518_power(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	switch (ctrl) {
	case POWER_OFF:
		TOUCH_I("%s, off\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(5);
		touch_power_3_3_vcl(dev, 0); //2.8V vdd power off
		touch_power_1_8_vdd(dev, 0);
		touch_msleep(5);
		break;
	case POWER_ON:
		TOUCH_I("%s, on\n", __func__);
		touch_power_3_3_vcl(dev, 1); //2.8V vdd power on
		touch_power_1_8_vdd(dev, 1);
		touch_msleep(5);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(200);
		break;
	case POWER_SLEEP:
		TOUCH_I("%s, sleep\n", __func__);
		break;
	case POWER_WAKE:
		TOUCH_I("%s, wake\n", __func__);
		break;
	case POWER_HW_RESET_ASYNC:
		TOUCH_I("%s, HW Reset(%d)\n", __func__, ctrl);
		break;
	case POWER_HW_RESET_SYNC:
		TOUCH_I("%s, HW Reset(%d)\n", __func__, ctrl);
		break;
	case POWER_SW_RESET:
	    TOUCH_I("%s, SW Reset\n", __func__);
		break;
	default:
		TOUCH_I("%s, Unknown Power Ctrl!!!!\n", __func__);
		break;
	}

	return 0;
}

static int ft3518_suspend(struct device *dev)
{
	int ret = 0;

	TOUCH_TRACE();

	ft3518_power(dev, POWER_OFF);
	return ret;
}

static int ft3518_resume(struct device *dev)
{
	TOUCH_TRACE();

	ft3518_power(dev, POWER_ON);
	return 0;
}


static int ft3518_init(struct device *dev)
{
	int ret = 0;

	TOUCH_TRACE();

	ret = ft3518_ic_info(dev);
	if (ret < 0) {
		TOUCH_I("failed to get ic_info, ret:%d\n", ret);
		return ret;
	}

	return 0;
}

static int ft3518_irq_abs_data(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft3518_data *d = to_ft3518_data(dev);
	struct ft3518_touch_data *data = d->info.data;
	struct touch_data *tdata;
	u32 touch_count = 0;
	u8 finger_index = 0;
	int i = 0;
	u8 touch_id, event, palm;
	//static u8 z_toggle;

	touch_count = d->info.touch_cnt;
	ts->new_mask = 0;
	//z_toggle ^= 0x1;

	for (i = 0; i < FTS_MAX_POINTS; i++) {
		touch_id = (u8)(data[i].yh) >> 4;
		if (touch_id >= FTS_MAX_ID) {
			break; // ??
		}

		event = (u8)(data[i].xh) >> 6;
		palm = ((u8)(data[i].xh) >> 4) & 0x01;

		if (palm) {
			if (event == FTS_TOUCH_CONTACT) { // FTS_TOUCH_DOWN
				ts->is_cancel = 1;
				TOUCH_I("Palm Detected\n");
			}
			else if (event == FTS_TOUCH_UP) {
				ts->is_cancel = 0;
				TOUCH_I("Palm Released\n");
			}
			ts->tcount = 0;
			ts->intr_status = TOUCH_IRQ_FINGER;
			return 0;
		}

		if(event == FTS_TOUCH_DOWN || event == FTS_TOUCH_CONTACT) {
			ts->new_mask |= (1 << touch_id);
			tdata = ts->tdata + touch_id;

			tdata->id = touch_id;
			tdata->type = MT_TOOL_FINGER;
			tdata->x = ((u16)(data[i].xh & 0x0F))<<8 | (u16)(data[i].xl);
			tdata->y = ((u16)(data[i].yh & 0x0F))<<8 | (u16)(data[i].yl);
			if (tdata->y < 88)
				tdata->y = 0;
			else
				tdata->y -= 88;
			tdata->pressure = (u8)(data[i].weight);// + z_toggle;
			if ((event == FTS_TOUCH_DOWN) && (tdata->pressure == 0x29))
				tdata->pressure = 0x30;

			tdata->width_major = (u8)((data[i].area)>>4);
			tdata->width_minor = 0;
			tdata->orientation = 0;

			if (tdata->width_major == 0)
				tdata->width_major = 0x09;
			if (tdata->pressure == 0)
				tdata->width_major = 0x3f;

			finger_index++;

			TOUCH_D(ABS, "tdata [id:%d e:%d x:%d y:%d z:%d - %d,%d,%d]\n",\
					tdata->id,\
					event,\
					tdata->x,\
					tdata->y,\
					tdata->pressure,\
					tdata->width_major,\
					tdata->width_minor,\
					tdata->orientation);

		}
	}

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return 0;
}

int ft3518_irq_abs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft3518_data *d = to_ft3518_data(dev);
	struct ft3518_touch_data *data = d->info.data;

	u8 point_buf[POINT_READ_BUF] = { 0, };
	int ret = -1;

	ret = ft3518_reg_read(dev, 0, point_buf, POINT_READ_BUF);

	if (ret < 0) {
		TOUCH_E("Fail to read point regs.\n");
		return ret;
	}

	/* check if touch cnt is valid */
	if (point_buf[FTS_TOUCH_P_NUM] > ts->caps.max_id) {
		TOUCH_I("%s : touch cnt is invalid - %d\n",
			__func__, point_buf[FTS_TOUCH_P_NUM]);
		return -ERANGE;
	}

	d->info.touch_cnt = point_buf[FTS_TOUCH_P_NUM];

	memcpy(data, point_buf+FTS_TOUCH_EVENT_POS, FTS_ONE_TCH_LEN * FTS_MAX_POINTS);

	return ft3518_irq_abs_data(dev);
}

int ft3518_irq_handler(struct device *dev)
{
	int ret = 0;
//	u8 int_status = 0;

	//Not use int_status
	ret = ft3518_irq_abs(dev);

	return ret;
}

static int ft3518_esd_recovery(struct device *dev)
{
	TOUCH_TRACE();
	return 0;
}

static int ft3518_lpwg(struct device *dev, u32 code, void *param)
{
	TOUCH_TRACE();
	return 0;
}

static int ft3518_swipe_enable(struct device *dev, bool enable)
{
	TOUCH_TRACE();
	return 0;
}

static int ft3518_notify(struct device *dev, ulong event, void *data)
{
	int ret = 0;

	TOUCH_TRACE();
	TOUCH_I("%s event=0x%x\n", __func__, (unsigned int)event);
	switch (event) {
	case NOTIFY_TOUCH_RESET:
		TOUCH_I("NOTIFY_TOUCH_RESET! - DO NOTHING (Add-on)\n");
		break;
	case NOTIFY_CONNECTION:
		TOUCH_I("NOTIFY_CONNECTION!\n");
		break;
	case NOTIFY_IME_STATE:
		TOUCH_I("NOTIFY_IME_STATE!\n");
		break;
	case NOTIFY_CALL_STATE:
		TOUCH_I("NOTIFY_CALL_STATE!\n");
		break;
	case NOTIFY_DEBUG_OPTION:
		TOUCH_I("NOTIFY_DEBUG_OPTION!\n");
		break;
	case LCD_EVENT_LCD_BLANK:
		TOUCH_I("LCD_EVENT_LCD_BLANK! - DO NOTHING (Add-on)\n");
		break;
	case LCD_EVENT_LCD_UNBLANK:
		TOUCH_I("LCD_EVENT_LCD_UNBLANK! - DO NOTHING (Add-on)\n");
		break;
	case LCD_EVENT_LCD_MODE:
		TOUCH_I("LCD_EVENT_LCD_MODE! - DO NOTHING (Add-on)\n");
		break;
	case LCD_EVENT_READ_REG:
		TOUCH_I("LCD_EVENT_READ_REG - DO NOTHING (Add-on)\n");
		break;
	default:
		TOUCH_E("%lx is not supported\n", event);
		break;
	}

	return ret;
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value == 0) {
		ft3518_power(dev, POWER_SW_RESET);
	} else if (value == 1) {
		ft3518_power(dev, POWER_HW_RESET_ASYNC);
	} else if (value == 2) {
		ft3518_power(dev, POWER_HW_RESET_SYNC);
	} else if (value == 3) {
		ft3518_power(dev, POWER_OFF);
		ft3518_power(dev, POWER_ON);
		queue_delayed_work(ts->wq, &ts->init_work, 0);
	} else {
		TOUCH_I("Unsupported command %d\n", value);
	}

	return count;
}

static ssize_t show_pinstate(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = snprintf(buf, PAGE_SIZE, "RST:%d, INT:%d\n",
			gpio_get_value(ts->reset_pin), gpio_get_value(ts->int_pin));
	TOUCH_I("%s() buf:%s",__func__, buf);
	return ret;
}

static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);
static TOUCH_ATTR(pinstate, show_pinstate, NULL);

static struct attribute *ft3518_attribute_list[] = {
	&touch_attr_reset_ctrl.attr,
	&touch_attr_pinstate.attr,
	NULL,
};

static const struct attribute_group ft3518_attribute_group = {
	.attrs = ft3518_attribute_list,
};

static int ft3518_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &ft3518_attribute_group);
	if (ret < 0)
		TOUCH_E("ft3518 sysfs register failed\n");

	return 0;
}

static int ft3518_init_pm(struct device *dev)
{
	TOUCH_TRACE();
	return 0;
}

static int ft3518_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int ft3518_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case CMD_VERSION:
		break;

	case CMD_ATCMD_VERSION:
		break;

	default:
		break;
	}

	return ret;
}

static int ft3518_shutdown(struct device *dev){
	TOUCH_TRACE();
	return 0;
}

static struct touch_driver touch_driver = {
	.probe = ft3518_probe,
	.remove = ft3518_remove,
	.suspend = ft3518_suspend,
	.shutdown = ft3518_shutdown,
	.resume = ft3518_resume,
	.init = ft3518_init,
	.irq_handler = ft3518_irq_handler,
	.power = ft3518_power,
	.upgrade = ft3518_upgrade,
	.esd_recovery = ft3518_esd_recovery,
	.lpwg = ft3518_lpwg,
	.swipe_enable = ft3518_swipe_enable,
	.notify = ft3518_notify,
	.init_pm = ft3518_init_pm,
	.register_sysfs = ft3518_register_sysfs,
	.set = ft3518_set,
	.get = ft3518_get,
};

#define MATCH_NAME	"focaltech,ft3518"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
	{ },
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

	if (is_ddic_name("rm692A9")) {
		TOUCH_I("%s, FT3518 detected. But need to return.\n", __func__);
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
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");