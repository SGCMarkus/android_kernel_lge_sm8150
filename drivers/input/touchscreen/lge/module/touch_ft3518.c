/*
 * touch_ft3518.c
 *
 * Copyright (c) 2018 LGE.
 *
 * author : rangkast.jeong@lge.com
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

#include "touch_ft3518.h"
#define MODULE_DEVICE_COMPATIBLE_NAME	    "focaltech,ft3518"

u32 touch_debug_mask = BASE_INFO;

/*
 *******************************************************************************
 * Device specific functions
 *******************************************************************************
 * [brief] specific functions for device to support different IC enviroment.
 *******************************************************************************
 * */

int ft3518_reg_read(struct device *dev, u16 addr, void *data, int size)
{
	struct module_data *md = to_module(dev);
	struct module_bus_msg msg = {0, };
	int ret = 0;

//	mutex_lock(&md->io_lock);
	if (!md) {
		TOUCH_E("module data is not located\n");
		return -ENOMEM;
	}

	md->tx_buf[0] = addr;

	msg.tx_buf = md->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = md->rx_buf;
	msg.rx_size = size;

	ret = module_i2c_read(to_i2c_client(dev), &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		return ret;
	}

	memcpy(data, &md->rx_buf[0], size);

//	mutex_unlock(&md->io_lock);

	return 0;
}

int ft3518_reg_write(struct device *dev, u16 addr, void *data, int size)
{
	struct module_data *md = to_module(dev);
	struct module_bus_msg msg = {0, };
	int ret = 0;

//	mutex_lock(&md->io_lock);
	if (!md) {
		TOUCH_E("module data is not located\n");
		return -ENOMEM;
	}

	md->tx_buf[0] = addr;
	memcpy(&md->tx_buf[1], data, size);

	msg.tx_buf = md->tx_buf;
	msg.tx_size = 1 + size;

	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = module_i2c_write(to_i2c_client(dev), &msg);
	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		return ret;
	}

//	mutex_unlock(&md->io_lock);

	return 0;
}

int ft3518_cmd_read(struct device *dev, void *cmd_data, int cmd_len, void *read_buf, int read_len)
{
	struct module_data *md = to_module(dev);
	struct module_bus_msg msg = {0, };
	int ret = 0;

	if (!md) {
		TOUCH_E("module data is not located\n");
		return -ENOMEM;
	}

	memcpy(&md->tx_buf[0], cmd_data, cmd_len);

	msg.tx_buf = md->tx_buf;
	msg.tx_size = cmd_len;

	msg.rx_buf = md->rx_buf;
	msg.rx_size = read_len;

	ret = module_i2c_read(to_i2c_client(dev), &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		return ret;
	}

	memcpy(read_buf, &md->rx_buf[0], read_len);

	return 0;

}

void ft3518_reset_ctrl(struct device *dev, int ctrl)
{
	struct module_data *md = to_module(dev);

	unsigned int master_data = 0;
	int ret = 0;

	module_interrupt_control(md->dev, MODULE_INTERRUPT_DISABLE);

	switch (ctrl) {
	case HW_RESET_ASYNC:
		TOUCH_I("%s : HW Reset\n", __func__);

		// read register value of touch reset control
		ret = ice40_master_reg_read(global_ice40, 0x00, &master_data);
		if (ret < 0) {
			TOUCH_E("ice40_master_reg_read failed, ret = %d\n", ret);
			break;
		}
		TOUCH_I("test_data : 0x02X\n", master_data);

		// touch reset to low
		ice40_master_reg_write(global_ice40, 0x00, (master_data & 0x7F));
		module_msleep(1);
		ice40_master_reg_write(global_ice40, 0x00, (master_data | 0x80));
		module_msleep(md->dts.hw_reset_delay);

		// check a register value of touch reset control
		ret = ice40_master_reg_read(global_ice40, 0x00, &master_data);
		if (ret < 0) {
			TOUCH_E("ice40_master_reg_read failed, ret = %d\n", ret);
			break;
		}
		TOUCH_I("test_data : 0x02X\n", master_data);

		break;
	default:
		TOUCH_I("%s : invalid ctrl (%d)\n", __func__, ctrl);
		break;
	}

	mod_delayed_work(md->wq, &md->init_work, 0);
}

#define TOUCH_IRQ_FINGER		(1 << 0)
static int ft3518_irq_abs_data(struct device *dev)
{
	struct module_data *md = to_module(dev);
	struct ft3518_data *d = to_ft3518_data(dev);
	struct ft3518_touch_data *data = d->info.data;
	struct touch_data_module *tdata;
	u32 touch_count = 0;
	u8 finger_index = 0;
	int ret = 0;
	int i = 0;
	u8 touch_id, event, palm = 0;

	if (!md) {
		TOUCH_E("module data is not located\n");
		return -ENOMEM;
	}

	//TOUCH_I("%s\n", __func__);
	touch_count = d->info.touch_cnt;
	md->new_mask = 0;

	for (i = 0; i < FTS_MAX_POINTS; i++) {
		touch_id = (u8)(data[i].yh) >> 4;
		if (touch_id >= FTS_MAX_ID) {
			break; // ??
		}

		event = (u8)(data[i].xh) >> 6;
		palm = ((u8)(data[i].xh) >> 4) & 0x01;

		if (palm) {
			if (event == FTS_TOUCH_CONTACT) { // FTS_TOUCH_DOWN
				md->is_cancel = 1;
				TOUCH_I("Palm Detected\n");
			} else if (event == FTS_TOUCH_UP) {
				md->is_cancel = 0;
				TOUCH_I("Palm Released\n");
			}
			md->tcount = 0;
			md->intr_status = TOUCH_IRQ_FINGER;
			return 0;
		}

		if (event == FTS_TOUCH_DOWN || event == FTS_TOUCH_CONTACT) {
			md->new_mask |= (1 << touch_id);
			tdata = md->tdata + touch_id;

			tdata->id = touch_id;
			tdata->type = MT_TOOL_FINGER;
			tdata->x = ((u16)(data[i].xh & 0x0F))<<8 | (u16)(data[i].xl);
			tdata->y = ((u16)(data[i].yh & 0x0F))<<8 | (u16)(data[i].yl);
			if (tdata->y < 88)
				tdata->y = 0;
			else
				tdata->y -= 88;
			tdata->pressure = (u8)(data[i].weight);
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

			/*
			TOUCH_I("tdata [id:%d e:%d x:%d y:%d z:%d - %d,%d,%d]\n",\
					tdata->id,\
					event,\
					tdata->x,\
					tdata->y,\
					tdata->pressure,\
					tdata->width_major,\
					tdata->width_minor,\
					tdata->orientation);
			*/

		}
	}

	md->tcount = finger_index;
	md->intr_status = TOUCH_IRQ_FINGER;

	return ret;
}

int ft3518_irq_abs(struct device *dev)
{
	struct module_data *md = to_module(dev);
	struct ft3518_data *d = to_ft3518_data(dev);
	u8 point_buf[POINT_READ_BUF] = { 0, };
	int ret = 0;

	if (!md) {
		TOUCH_E("module data is not located\n");
		return -ENOMEM;
	}

	//TOUCH_I("%s\n", __func__);
	ret = ft3518_reg_read(dev, 0x02, point_buf + 2, POINT_READ_BUF - 2);
	if (ret < 0) {
		TOUCH_E("Fail to read point regs.\n");
		return ret;
	}
	/* check if touch cnt is valid */
	if (point_buf[FTS_TOUCH_P_NUM] > md->dts.max_id) {
		TOUCH_I("%s : touch cnt is invalid - %d\n",
			__func__, point_buf[FTS_TOUCH_P_NUM]);
		return -ERANGE;
	}

	memcpy(d->info.data, point_buf + FTS_TOUCH_EVENT_POS, FTS_ONE_TCH_LEN * FTS_MAX_POINTS);

	return ft3518_irq_abs_data(dev);
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
//		module_msleep(50);
//		touch_gpio_direction_output(ts->reset_pin, 1);
//		module_msleep(i);

		// Set Upgrade Mode
		ret = ft3518_reg_write(dev, 0x55, i2c_buf, 0);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
		module_msleep(1);

		// Check ID
		TOUCH_I("%s - Set Upgrade Mode and Check ID : %d ms\n", __func__, i);
		ret = ft3518_reg_read(dev, 0x90, i2c_buf, 2);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}

		TOUCH_I("Check ID : 0x%x , 0x%x\n", i2c_buf[0], i2c_buf[1]);

		if (i2c_buf[0] == 0x80 && i2c_buf[1] == 0x06) {
			module_msleep(50);
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

	module_msleep(100);

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

	module_msleep(10);

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
		module_msleep(1);

		// Check ID
		ret = ft3518_reg_read(dev, 0x90, i2c_buf, 2);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return -EIO;
		}

		TOUCH_I("Check ID [%d] : 0x%x , 0x%x\n", i, i2c_buf[0], i2c_buf[1]);   // 0x80, 0xC6
		if (i2c_buf[0] == 0x80 && i2c_buf[1] == 0xC6)
			break;

		module_msleep(10);
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
	module_msleep(50);

	// Erase App (Panel Parameter Area??)
	TOUCH_I("%s - Erase App and Panel Parameter Area\n", __func__);

	ret = ft3518_reg_write(dev, 0x61, i2c_buf, 0);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	module_msleep(1000);

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
		//module_msleep(50);
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
			//module_msleep(1);
			mdelay(1);
		} while (--retry);
		if (retry == 0) {
			TOUCH_I("Write Fail : %d / %d : [0x%02x] , [0x%02x]\n", i+1, packet_num, i2c_buf[0], i2c_buf[1]);
			//goto FAIL;
		}
	}
	TOUCH_I("Write Finished : Total %d\n", packet_num);

	module_msleep(50);

	// Read out Checksum
	TOUCH_I("%s - Read out checksum (App) for %d bytes\n", __func__, fw_size);
	ret = ft3518_reg_write(dev, 0x64, i2c_buf, 0);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	module_msleep(300);

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
		module_msleep(fw_size/256);

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
			module_msleep(1);
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


/*
 *******************************************************************************
 * Module common functions
 *******************************************************************************
 * [brief] common functions to use touch devices
 *******************************************************************************
 * */

int module_get_dts_data(struct module_data *md)
{
	struct device_node *np = md->dev->of_node;

	TOUCH_I("start dev.of_node\n");

	PROPERTY_GPIO(np, "reset-gpio", md->dts.reset_pin);
	PROPERTY_GPIO(np, "irq-gpio", md->dts.int_pin);
	PROPERTY_U32(np, "irqflags", md->dts.irqflags);

	/* */
	PROPERTY_U32(np, "max_x", md->dts.max_x);
	PROPERTY_U32(np, "max_y", md->dts.max_y);
	PROPERTY_U32(np, "max_pressure", md->dts.max_pressure);
	PROPERTY_U32(np, "max_width_major", md->dts.max_width_major);
	PROPERTY_U32(np, "max_width_minor", md->dts.max_width_minor);
	PROPERTY_U32(np, "max_orientation", md->dts.max_orientation);
	PROPERTY_U32(np, "max_id", md->dts.max_id);
	PROPERTY_U32(np, "hw_reset_delay", md->dts.hw_reset_delay);
	PROPERTY_U32(np, "sw_reset_delay", md->dts.sw_reset_delay);
	PROPERTY_GPIO(np, "vcl-gpio", md->dts.vcl_pin);
	PROPERTY_GPIO(np, "vdd-gpio", md->dts.vdd_pin);

	PROPERTY_STRING_ARRAY(np, "fw_image", md->dts.def_fwpath, md->dts.def_fwcnt);
	{
		int i;

		for (i = 0; i < md->dts.def_fwcnt; i++) {
			TOUCH_I("fw_image - %d:%s\n",
				i, md->dts.def_fwpath[i]);
		}
	}
	PROPERTY_STRING(np, "panel_spec", md->dts.panel_spec);
	PROPERTY_STRING(np, "panel_spec_mfts_folder", md->dts.panel_spec_mfts);

	TOUCH_I("end dev.of_node\n");

	return 0;
}

void module_gpio_init(int pin, char *name)
{
	int ret = 0;

	TOUCH_I("%s - pin:%d, name:%s\n", __func__, pin, name);

	if (gpio_is_valid(pin))
		ret = gpio_request(pin, name);
}

void module_gpio_direction_output(int pin, int value)
{
	int ret = 0;

	TOUCH_I("%s - pin:%d, value:%d\n", __func__, pin, value);

	if (gpio_is_valid(pin))
		ret = gpio_direction_output(pin, value);
}

void module_gpio_direction_input(int pin)
{
	int ret = 0;

	TOUCH_I("%s - pin:%d\n", __func__, pin);

	if (gpio_is_valid(pin))
		ret = gpio_direction_input(pin);
}

int module_power_init(struct module_data *md)
{
	int ret = 0;

	if (gpio_is_valid(md->dts.vcl_pin)) {
		ret = gpio_request(md->dts.vcl_pin, "module-touch-vcl");
	} else {
		md->vcl = regulator_get(md->dev, "vcl");
		if (IS_ERR(md->vcl))
			TOUCH_I("regulator \"vcl\" not exist\n");
	}

	if (gpio_is_valid(md->dts.vdd_pin)) {
		ret = gpio_request(md->dts.vdd_pin, "module-touch-vdd");
	} else {
		md->vdd = regulator_get(md->dev, "vdd");
		if (IS_ERR(md->vdd))
			TOUCH_I("regulator \"vdd\" not exist\n");
	}

	return 0;
}

int module_bus_init(struct module_data *md, int buf_size)
{
//	int ret = 0;

	if (buf_size) {
		md->tx_buf = devm_kzalloc(md->dev, buf_size, GFP_KERNEL | GFP_DMA);
		if (!md->tx_buf) {
			TOUCH_E("fail to allocate tx_buf\n");
			return -ENOMEM;
		}

		md->rx_buf = devm_kzalloc(md->dev, buf_size, GFP_KERNEL | GFP_DMA);
		if (!md->rx_buf) {
			TOUCH_E("fail to allocate rx_buf\n");
			return -ENOMEM;
		}
	}

	md->pinctrl.ctrl = devm_pinctrl_get(md->dev);

	if (IS_ERR_OR_NULL(md->pinctrl.ctrl)) {
		if (PTR_ERR(md->pinctrl.ctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		TOUCH_I("Target does not use pinctrl\n");
		md->pinctrl.ctrl = NULL;
	} else {
		md->pinctrl.active = pinctrl_lookup_state(md->pinctrl.ctrl,
				"module_touch_active");

		if (IS_ERR_OR_NULL(md->pinctrl.active))
			TOUCH_E("cannot get pinctrl.active\n");

		md->pinctrl.suspend = pinctrl_lookup_state(md->pinctrl.ctrl,
				"module_touch_sleep");

		if (IS_ERR_OR_NULL(md->pinctrl.suspend))
			TOUCH_E("cannot get pinctrl.suspend\n");

		/* do not set pinctrl for ft3518, s3706 only
		if (!IS_ERR_OR_NULL(md->pinctrl.active)) {
			ret = pinctrl_select_state(md->pinctrl.ctrl,
					md->pinctrl.active);
			if (ret)
				TOUCH_I("cannot set pinctrl.active\n");
			else
				TOUCH_I("pinctrl set active\n");
		}
		*/
	}
	return 0;
}

int module_i2c_read(struct i2c_client *client, struct module_bus_msg *msg)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = msg->tx_size,
			.buf = msg->tx_buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = msg->rx_size,
			.buf = msg->rx_buf,
		},
	};

//	if (msg->rx_size > MAX_BUF_SIZE || msg->tx_size > MAX_BUF_SIZE) {
//		TOUCH_E("buffer overflow\n");
//		return -ENOMEM;
//	}

	ret = i2c_transfer(client->adapter, msgs, 2);

	if (ret == ARRAY_SIZE(msgs)) {
		return 0;
	} else if (ret < 0) {
		TOUCH_E("i2c_transfer - errno[%d]\n", ret);
	} else if (ret != ARRAY_SIZE(msgs)) {
		TOUCH_E("i2c_transfer - size[%d] result[%d]\n",
				(int) ARRAY_SIZE(msgs), ret);
	} else {
		TOUCH_E("unknown error [%d]\n", ret);
	}

	return ret;
}

int module_i2c_write(struct i2c_client *client, struct module_bus_msg *msg)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = msg->tx_size,
			.buf = msg->tx_buf,
		},
	};

//	if (msg->tx_size > MAX_BUF_SIZE) {
//		TOUCH_E("buffer overflow\n");
//		return -ENOMEM;
//	}

	ret = i2c_transfer(client->adapter, msgs, 1);

	return ret;
}

int module_ic_info(struct device *dev)
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

int module_init_input(struct module_data *md)
{
	struct input_dev *input;
	int ret;

	input = input_allocate_device();

	if (!input) {
		TOUCH_E("failed to allocate memory for input\n");
		return -ENOMEM;
	}

	input->name = "touch_dev_2nd";

	TOUCH_I("%s %d-%d-%d-%d-%d-%d-%d\n", __func__,
			md->dts.max_x,
			md->dts.max_y,
			md->dts.max_pressure,
			md->dts.max_width_major,
			md->dts.max_width_minor,
			md->dts.max_orientation,
			md->dts.max_id);
	set_bit(EV_SYN, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);
	set_bit(BTN_TOOL_FINGER, input->keybit);
	set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0,
			md->dts.max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
			md->dts.max_y, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0,
			md->dts.max_pressure, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0,
			md->dts.max_width_major, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MINOR, 0,
			md->dts.max_width_minor, 0, 0);
	input_set_abs_params(input, ABS_MT_ORIENTATION, 0,
			md->dts.max_orientation, 0, 0);

	ret = input_mt_init_slots(input, md->dts.max_id, 0);

	if (ret < 0) {
		TOUCH_E("failed to initialize input device(ret:%d)\n", ret);
		goto error;
	}

	ret = input_register_device(input);

	if (ret < 0) {
		TOUCH_E("failed to register input device(ret:%d)\n", ret);
		goto error_register;
	}

	input_set_drvdata(input, md);
	md->input = input;

	return 0;

error_register:
	input_mt_destroy_slots(input);

error:
	input_free_device(input);

	return ret;
}

irqreturn_t module_irq_handler(int irq, void *dev_id)
{
//	struct module_data *md = (struct module_data *) dev_id;

//	if (atomic_read(&ts->state.pm) >= DEV_PM_SUSPEND) {
//		TOUCH_I("interrupt in suspend[%d]\n",
//				atomic_read(&ts->state.pm));
//		atomic_set(&ts->state.pm, DEV_PM_SUSPEND_IRQ);
//		pm_wakeup_event(ts->dev, 1000);
//		return IRQ_HANDLED;
//	}
//
	return IRQ_WAKE_THREAD;
}

irqreturn_t module_irq_thread(int irq, void *dev_id)
{
	struct module_data *md = (struct module_data *) dev_id;
	u8 int_status = 0;
	int ret = 0;

	md->intr_status = 0;

	pm_qos_update_request(&md->pm_qos_req, 10);

	ret = ft3518_reg_read(md->dev, 0x01, &int_status, 1);
	if (ret < 0)
		TOUCH_I("error in %s 1\n", __func__);

//	TOUCH_I("%s : int_status = %d\n", __func__, int_status);
	if (int_status == 0x00) { // Finger
		ret = ft3518_irq_abs(md->dev);
	} else if (int_status == 0x02) { // TCI_1
		;
//		ret = ft3518_irq_lpwg(dev, TCI_1);
	} else if (int_status == 0x03) { // TCI_2
		;
//		ret = ft3518_irq_lpwg(dev, TCI_2);
	} else if (int_status == 0x04) { // LPWG Fail Reason Report (RT)
		;
//		ret = ft3518_irq_report_tci_fr(dev);
	} else if (int_status == 0x05) { // ESD
		;
//		TOUCH_I("ESD interrupt !!\n");
//		ret = -EGLOBALRESET;
	} else if (int_status == 0x01) {
		;
//		TOUCH_I("No interrupt status\n");
	} else {
		TOUCH_E("Invalid interrupt status : %d\n", int_status);
	}

	if (md->intr_status & TOUCH_MODULE_IRQ_FINGER)
		touch_report_event(md);

	pm_qos_update_request(&md->pm_qos_req, PM_QOS_DEFAULT_VALUE);

	return IRQ_HANDLED;
}

void module_enable_irq(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	TOUCH_I("%s\n", __func__);
	if (desc) {
		if (desc->istate & IRQS_PENDING)
			TOUCH_I("Remove pending irq(%d)\n", irq);
		desc->istate &= ~(IRQS_PENDING);
	}
	enable_irq(irq);
}

void module_disable_irq(unsigned int irq)
{
	TOUCH_I("%s\n", __func__);
	disable_irq_nosync(irq);
}

static void touch_report_cancel_event(struct module_data *md)
{
	u16 old_mask = md->old_mask;
	int i = 0;

	for (i = 0; i < MAX_FINGER; i++) {
		if (old_mask & (1 << i)) {
			input_mt_slot(md->input, i);
			input_mt_report_slot_state(md->input, MT_TOOL_FINGER,
						   true);
			input_report_key(md->input, BTN_TOUCH, 1);
			input_report_key(md->input, BTN_TOOL_FINGER, 1);
			input_report_abs(md->input, ABS_MT_PRESSURE,
					255);
			TOUCH_I("finger canceled:<%d>(%4d,%4d,%4d)\n",
					i,
					md->tdata[i].x,
					md->tdata[i].y,
					md->tdata[i].pressure);
		}
	}

	input_sync(md->input);
}

static void touch_report_event(struct module_data *md)
{
	u16 old_mask = md->old_mask;
	u16 new_mask = md->new_mask;
	u16 press_mask = 0;
	u16 release_mask = 0;
	u16 change_mask = 0;
	int i;

	change_mask = old_mask ^ new_mask;
	press_mask = new_mask & change_mask;
	release_mask = old_mask & change_mask;

//	TOUCH_D(ABS, "mask [new: %04x, old: %04x]\n",
//			new_mask, old_mask);
//	TOUCH_D(ABS, "mask [change: %04x, press: %04x, release: %04x]\n",
//			change_mask, press_mask, release_mask);

	/* Palm state - Report Pressure value 255 */
	if (md->is_cancel) {
		touch_report_cancel_event(md);
		md->is_cancel = 0;
	}

	for (i = 0; i < MAX_FINGER; i++) {
		if (new_mask & (1 << i)) {
			input_mt_slot(md->input, i);
			input_mt_report_slot_state(md->input, MT_TOOL_FINGER,
					true);
			input_report_key(md->input, BTN_TOUCH, 1);
			input_report_key(md->input, BTN_TOOL_FINGER, 1);
			input_report_abs(md->input, ABS_MT_TRACKING_ID, i);
			input_report_abs(md->input, ABS_MT_POSITION_X,
					md->tdata[i].x);
			input_report_abs(md->input, ABS_MT_POSITION_Y,
					md->tdata[i].y);
			input_report_abs(md->input, ABS_MT_PRESSURE,
					md->tdata[i].pressure);
			input_report_abs(md->input, ABS_MT_WIDTH_MAJOR,
					md->tdata[i].width_major);
			input_report_abs(md->input, ABS_MT_WIDTH_MINOR,
					md->tdata[i].width_minor);
			input_report_abs(md->input, ABS_MT_ORIENTATION,
					md->tdata[i].orientation);

			if (press_mask & (1 << i)) {
				TOUCH_I("%d finger pressed:<%d>(%4d,%4d,%4d)\n",
						md->tcount,
						i,
						md->tdata[i].x,
						md->tdata[i].y,
						md->tdata[i].pressure);
			}
		} else if (release_mask & (1 << i)) {
			input_mt_slot(md->input, i);
			//input_report_abs(md->input, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(md->input, MT_TOOL_FINGER, false);
			TOUCH_I(" finger released:<%d>(%4d,%4d,%4d)\n",
					i,
					md->tdata[i].x,
					md->tdata[i].y,
					md->tdata[i].pressure);
		}
	}

	if (!md->tcount) {
		input_report_key(md->input, BTN_TOUCH, 0);
		input_report_key(md->input, BTN_TOOL_FINGER, 0);
	}

	md->old_mask = new_mask;
	input_sync(md->input);
}

void module_msleep(unsigned int msecs)
{
	if (msecs >= 20)
		msleep(msecs);
	else
		usleep_range(msecs * 1000, msecs * 1000);
}
/*
 *******************************************************************************
 * Module driver functions
 *******************************************************************************
 * [brief] module driver functions. should be changed inside of function.
 *******************************************************************************
 * */

void module_interrupt_control(struct device *dev, int on_off)
{
	struct module_data *md = to_module(dev);

	TOUCH_TRACE();

	if (atomic_read(&md->core) != MODULE_NONE) {
		if (on_off) {
			if (atomic_cmpxchg(&md->irq_enable, 0, 1) == 0)
				module_enable_irq(md->irq);
		} else {
			if (atomic_cmpxchg(&md->irq_enable, 1, 0) == 1)
				module_disable_irq(md->irq);
		}
	}
}

static int match_module_ic(struct device *dev)
{
	int ret = 0;
	int retry = 0;
	int retry_link_tr = 0;
	struct module_data *md = to_module(dev);
	u8 value;

	TOUCH_I("%s\n", __func__);

	ret = module_get_dts_data(md);
	if (ret < 0)
		TOUCH_I("get dts data error!\n");

	module_power_init(md);
	ret = module_bus_init(md, 15 * 1024);
	if (ret < 0) {
		TOUCH_I("bus init error %d\n", ret);
		return ret;
	}

	do {
		if (!link_tr_state) {
			module_msleep(100);
			retry_link_tr++;
			TOUCH_I("link training wait..%d\n", retry_link_tr);
		} else {
			break;
		}
	} while (retry_link_tr < 10);

	do {
		ret = ft3518_reg_read(dev, retry, &value, sizeof(value));
		if (ret < 0) {
			retry++;
			continue;
		} else if (ret == 0) {
			TOUCH_I("[%s] find module touch!!",
					MODULE_DEVICE_COMPATIBLE_NAME);
			break;
		}
	} while (retry < 3);

	return ret;
}

static int version_check(struct device *dev)
{
	struct module_data *md = to_module(dev);
	struct ft3518_data *d = to_ft3518_data(dev);

	TOUCH_I("version check %d", d->debug_test_cnt);

	d->debug_test_cnt++;

	if (d->debug_test_cnt > 10) {
		md->m_driver.notify(MODULE_FIRMWARE_UPGRADE, DONE);
		return 0;
	} else {
		md->m_driver.notify(MODULE_FIRMWARE_UPGRADE, WAIT);
		return 1;
	}
}

static int module_upgrade(struct device *dev)
{
	struct module_data *data = to_module(dev);
	const struct firmware *fw = NULL;
	const struct firmware *fw_boot = NULL;
	char fwpath[256] = {0};
	char fwpath_boot[256] = {0};
	int ret = 0;
	int i = 0;

	TOUCH_I("%s\n", __func__);

	/*
	 * [TODO]
	 * 1. compare version between ic / bin
	 * 2. call device specific upgrade function (think about Uevent for module)
	 * */

	if (data->dts.def_fwcnt) {
		memcpy(fwpath_boot, data->dts.def_fwpath[0], sizeof(fwpath_boot));
		TOUCH_I("get fwpath_boot from def_fwpath : %s\n", fwpath_boot);
		memcpy(fwpath, data->dts.def_fwpath[1], sizeof(fwpath));
		TOUCH_I("get fwpath from def_fwpath : %s\n", fwpath);
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	fwpath_boot[sizeof(fwpath_boot) - 1] = '\0';
	fwpath[sizeof(fwpath) - 1] = '\0';

	ret = request_firmware(&fw_boot, fwpath_boot, dev);
	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath_boot: %s (ret:%d)\n",
			fwpath_boot, ret);

		return ret;
	}
	TOUCH_I("fw_boot size:%zu, data: %p\n", fw_boot->size, fw_boot->data);

	ret = request_firmware(&fw, fwpath, dev);
	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n",
			fwpath, ret);
		release_firmware(fw_boot);
		return ret;
	}
	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	if (ft3518_fw_compare(dev, fw)) {
		ret = -EINVAL;
		module_msleep(200);
		for (i = 0; i < 2 && ret; i++) {
			ret = ft3518_fwboot_upgrade(dev, fw_boot);
			if (ret < 0) {
				TOUCH_E("fail to upgrade f/w (pramboot) : %d\n", ret);
				release_firmware(fw);
				release_firmware(fw_boot);
				return -EPERM;
			}

			ret = ft3518_fw_upgrade(dev, fw);
			if (ret < 0) {
				TOUCH_E("fail to upgrade f/w : %d\n", ret);
				release_firmware(fw);
				release_firmware(fw_boot);
				return -EPERM;
			}
		}
	} else {
		release_firmware(fw_boot);
		release_firmware(fw);
		return 0;
	}

	release_firmware(fw_boot);
	release_firmware(fw);

	return ret;
}

static void debug_work_func(struct work_struct *debug_work)
{
	struct module_data *md =
		container_of(to_delayed_work(debug_work),
				struct module_data, debug_work);
	int status = 0;
	int ret = 0;
	int retry = 0;
	u8 value;

	status = version_check(md->dev);

	if (!md->wq)
		status = 0;

	if (status) {
		TOUCH_I("wq run");
		//check reg_read in wq
		ret = ft3518_reg_read(md->dev, retry, &value, sizeof(value));
		if (ret < 0)
			TOUCH_I("failed to read");
		queue_delayed_work(md->wq, &md->debug_work, 50);
	} else {
		TOUCH_I("wq end");
	}
}

static void module_init_work_func(struct work_struct *init_work)
{
	struct module_data *md =
		container_of(to_delayed_work(init_work),
				struct module_data, init_work);
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("touch_ic_init Start\n");
	//mutex_lock(&md->lock);
	ret = module_initialize(md->dev);
	if (ret < 0) {
		TOUCH_E("touch_ic_init failed %d\n", ret);
		//mutex_unlock(&md->lock);
		atomic_set(&md->core, MODULE_NORMAL);
		return;
	}
	module_interrupt_control(md->dev, MODULE_INTERRUPT_ENABLE);
	//mutex_unlock(&ts->lock);

	if (atomic_read(&md->core) == MODULE_PROBE) {
		queue_delayed_work(md->wq, &md->upgrade_work, 0);
		return;
	}

	atomic_set(&md->core, MODULE_NORMAL);
	TOUCH_I("touch_ic_init End\n");
}

static void module_upgrade_work_func(struct work_struct *upgrade_work)
{
	struct module_data *md =
		container_of(to_delayed_work(upgrade_work),
				struct module_data, upgrade_work);

	TOUCH_I("%s : firmware_upgrade\n", __func__);

	atomic_set(&md->core, MODULE_UPGRADE);
	mod_delayed_work(md->wq, &md->init_work, 0);

#if 0 //TBD
	struct module_data *md =
		container_of(to_delayed_work(upgrade_work),
				struct module_data, upgrade_work);

	int ret;

	TOUCH_TRACE();

	atomic_set(&md->state.core, CORE_UPGRADE);

	if (atomic_read(&md->state.fb) >= FB_SUSPEND) {
		TOUCH_I("%s: state.fb is not FB_RESUME\n", __func__);
		atomic_set(&md->state.core, CORE_NORMAL);
		goto exit;
	}

	mutex_lock(&ts->lock);
	touch_interrupt_control(md->dev, INTERRUPT_DISABLE);
	ret = md->driver->upgrade(md->dev);

	if (ret < 0) {
		TOUCH_I("There is no need to reset (ret: %d)\n", ret);
		mutex_unlock(&md->lock);
		mod_delayed_work(md->wq, &md->init_work, 0);
		goto exit;
	}

	md->driver->power(md->dev, POWER_OFF);
	md->driver->power(md->dev, POWER_ON);
	touch_msleep(md->caps.hw_reset_delay);
	mutex_unlock(&md->lock);

	mod_delayed_work(md->wq, &md->init_work, 0);

exit:
	/* init force_upgrade */
	md->force_fwup = 0;
	md->test_fwpath[0] = '\0';
#endif

}

static int module_init_works(struct module_data *md)
{

	int ret = 0;

	TOUCH_I("%s\n", __func__);

	md->wq = create_singlethread_workqueue("fts_wq");

	if (!md->wq) {
		TOUCH_E("failed to create workqueue\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&md->debug_work, debug_work_func);
	INIT_DELAYED_WORK(&md->init_work, module_init_work_func);
	INIT_DELAYED_WORK(&md->upgrade_work, module_upgrade_work_func);
	TOUCH_I("module init works done");
	return ret;
}

static int module_func(struct device *dev, int control, char *data)
{
	struct module_data *md = to_module(dev);
	char buf[LOG_BUF_SIZE] = {0,};
	int ret = 0;
	int i = 0;
	u32 test_data = 0;

	TOUCH_TRACE();

	if (atomic_read(&md->core) == MODULE_NONE) {
		TOUCH_I("%s : not probed, do nothing\n", __func__);
		return ret;
	}

	switch (control) {
	case 1:
		break;
	case 2:
		TOUCH_I("test read registers\n");
		for (i = 0; i < 10; i++) {
			ret = ft3518_reg_read(dev, i, &test_data, sizeof(test_data));
			if (ret < 0)
				TOUCH_I("reg[%d] read error %d\n", i, ret);
			else
				TOUCH_I("test_data[%d] %02X\n", i, test_data);
		}
		return 0;
		break;
	case 3:
		module_upgrade(dev);
		break;
	default:
		TOUCH_I("not support mode: %d\n", control);
		break;
	}

	memcpy(data, buf, sizeof(buf));

	return ret;
}

static ssize_t show_sysnode_test(struct device *dev, char *buf)
{
	int ret = 0;

	TOUCH_I("show_touch_module_test\n");

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "show_module_sysnode_test file write, TIME_INFO_SKIP\n");
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "show_module_sysnode_test file write, TIME_INFO_WRITE\n");
	write_file(dev, buf, TIME_INFO_WRITE);

	return ret;
}

static ssize_t store_sysnode_test(struct device *dev, const char *buf, size_t count)
{
	u32 value = 0;
	char log_buf[LOG_BUF_SIZE] = {0,};
	struct module_data *md = to_module(dev);

	if (kstrtos32(buf, 10, &value) < 0)
		return count;

	TOUCH_I("store_module_sysnode_test! value : %d\n", value);

	md->m_driver.func(dev, value, log_buf);

	return count;
}

static ssize_t show_version_info(struct device *dev, char *buf)
{
	struct ft3518_data *d = to_ft3518_data(dev);
	int ret = 0;
	int result = 0;

	TOUCH_TRACE();

	//mutex_lock(&md->lock);
	result = module_ic_info(dev);
	if (result < 0) {
		TOUCH_E("Failed to get ic info (ret: %d)\n", ret);
		//mutex_unlock(&md->lock);
	}

	TOUCH_I("is_official : %d, fw_version : %d.%d.%d\n",
		d->ic_info.is_official, d->ic_info.fw_version,
		d->ic_info.version.minor, d->ic_info.version.sub_minor);
	ret = snprintf(buf, PAGE_SIZE, "is_official : %d, fw_version : %d.%d.%d\n",
		d->ic_info.is_official, d->ic_info.fw_version,
		d->ic_info.version.minor, d->ic_info.version.sub_minor);

	//mutex_unlock(&md->lock);

	return ret;
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf, size_t count)
{
	int value = 0;

	TOUCH_TRACE();

	if (kstrtos32(buf, 10, &value) < 0)
		return count;

	ft3518_reset_ctrl(dev, value);

	return count;
}

static TOUCH_ATTR(sysnode_test, show_sysnode_test, store_sysnode_test);
static TOUCH_ATTR(version, show_version_info, NULL);
static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);

static struct attribute *module_attribute_list[] = {
	&touch_attr_sysnode_test.attr,
	&touch_attr_version.attr,
	&touch_attr_reset_ctrl.attr,
	NULL,
};

static const struct attribute_group module_attribute_group = {
	.attrs = module_attribute_list,
};

static int module_touch_register_sysfs(struct device *dev)
{
	int ret = 0;
	struct module_data *md = to_module(dev);

	ret = sysfs_create_group(&md->kobj, &module_attribute_group);
	if (ret < 0) {
		TOUCH_E("ft3518 sysfs register failed\n");
		goto error;
	}

	return 0;

error:
	kobject_del(&md->kobj);

	return ret;
}

static int module_probe(struct device *dev)
{
	struct module_data *md = to_module(dev);
	struct ft3518_data *d = NULL;
	int ret = 0;

	TOUCH_I("%s : %s\n", __func__,
			MODULE_DEVICE_COMPATIBLE_NAME);


	/*
	 * [TODO]   1. get dts data
	 *	    2. gpio init
	 *	    3. power init
	 *	    4. bus init
	 *	    5. input init
	 *	    6. uevent / sysfs init [TBD]
	 *	    7. notifier init [TBD]
	 *	    8. init / Upgrade
	 * */

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate synaptics data\n");
		return -ENOMEM;
	}
	d->dev = dev;
	module_set_device(md, d);

//	module_gpio_init(md->dts.reset_pin, "module_touch_reset");
//	module_gpio_direction_output(md->dts.reset_pin, 0);

	module_gpio_init(md->dts.int_pin, "module_touch_int");
	module_gpio_direction_input(md->dts.int_pin);

	module_init_input(md);
	if (ret < 0) {
		TOUCH_E("failed to register input device(ret:%d)\n", ret);
		goto error_init_input;
	}

	ret = request_threaded_irq(md->irq, module_irq_handler,
			module_irq_thread, md->dts.irqflags | IRQF_ONESHOT,
			LGE_MODULE_IRQ_NAME, md);
	if (ret < 0) {
		TOUCH_E("failed to request_thread_irq(irq:%d, ret:%d)\n",
				md->irq, ret);
		goto error_request_irq;
	}

	module_disable_irq(md->irq);
	module_init_works(md);

	/* test for firmware upgrade */
//	queue_delayed_work(md->wq, &md->debug_work, 0);
//	data->m_driver.notify(1);

	pm_qos_add_request(&md->pm_qos_req, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

//	data->m_driver.upgrade(struct device *dev);

//	mutex_init(&data->io_lock);
	md->m_driver.notify(MODULE_ATTACHED_TYPE, 2);

	atomic_set(&md->core, MODULE_PROBE);

	return 0;

error_request_irq:
	free_irq(md->irq, md);
error_init_input:
	if (md->input) {
		input_mt_destroy_slots(md->input);
		input_free_device(md->input);
	}
//error_init_work:
//	if (md->wq)
//		destroy_workqueue(md->wq);

	return 0;
}

static int module_remove(struct device *dev)
{
	struct module_data *md = to_module(dev);

	TOUCH_I("%s\n", __func__);

	if (md->tx_buf)
		devm_kfree(dev, md->tx_buf);
	if (md->rx_buf)
		devm_kfree(dev, md->rx_buf);
	devm_kfree(dev, md);

	return 0;
}

int module_initialize(struct device *dev)
{
	/*
	 *  [TODO] initialize ft3518
	 *
	 *  1. read ic info
	 *  2. activate default sensing (U3)
	 *
	 * */
	int ret = 0;

	TOUCH_I("%s\n", __func__);

	ret = module_ic_info(dev);

	// [Yousung] Need porting IC sensing start
	//           need nothing for IC Sensing (Power on -> IC Sensing Start)

	module_interrupt_control(dev, MODULE_INTERRUPT_ENABLE);

	return 0;
}

static int module_resume(struct device *dev)
{
	TOUCH_I("%s\n", __func__);
	return 0;
}

static int module_suspend(struct device *dev)
{
	TOUCH_I("%s\n", __func__);
	return 0;
}

static int module_lpwg_mode(struct device *dev)
{
	struct module_data *md = to_module(dev);

	if (md->lpwg.screen) {
		TOUCH_I("%s : screen[on] enable irq\n", __func__);
		module_interrupt_control(dev, MODULE_INTERRUPT_ENABLE);
	} else if (!md->lpwg.screen) {
		TOUCH_I("%s : screen[off] disable irq\n", __func__);
		module_interrupt_control(dev, MODULE_INTERRUPT_DISABLE);
	}

	return 0;
}

static int module_lpwg(struct device *dev, u32 code, void *param)
{
	struct module_data *md = to_module(dev);

	int *value = (int *)param;
	int ret = 0;

	switch (code) {
	case 9:
		md->lpwg.mode = value[0];
		md->lpwg.screen = value[1];
		md->lpwg.sensor = value[2];
		md->lpwg.qcover = value[3];
		TOUCH_I("LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
				md->lpwg.mode,
				md->lpwg.screen ? "ON" : "OFF",
				md->lpwg.sensor ? "FAR" : "NEAR",
				md->lpwg.qcover ? "CLOSE" : "OPEN");
		ret = module_lpwg_mode(dev);
		if (ret < 0) {
			TOUCH_E("failed to set lpwg mode (ret: %d)\n", ret);
			return ret;
		}
		break;
	default:
		TOUCH_I("not supported case %d\n", code);
		break;
	}

	return 0;
}

static void touch_module_release(struct device *dev)
{
	TOUCH_I("Module Touch module_release!\n");
	if (dev->platform_data)
		dev->platform_data = NULL;
}

static int touch_bus_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct module_data *md;
	struct platform_device *pdev;

	TOUCH_I("%s\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	TOUCH_I("i2c slave address : %x, platform ptr = %p\n", i2c->addr, i2c->dev.platform_data);

	md = devm_kzalloc(&i2c->dev, sizeof(*md), GFP_KERNEL);

	if (!md) {
		TOUCH_E("Failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	md->dev = &i2c->dev;
	md->irq = i2c->irq;
	TOUCH_I("i2c->irq : %d\n", i2c->irq);

	md->m_driver.match = match_module_ic;
	md->m_driver.probe = module_probe;
	md->m_driver.remove = module_remove;
	md->m_driver.func = module_func;
	md->m_driver.init = module_initialize;
	md->m_driver.suspend = module_suspend;
	md->m_driver.resume = module_resume;
	md->m_driver.register_sysfs = module_touch_register_sysfs;
	md->m_driver.lpwg = module_lpwg;

	dev_set_drvdata(&i2c->dev, md);

	pdev = devm_kzalloc(&i2c->dev, sizeof(*pdev), GFP_KERNEL);
	if (!pdev) {
		TOUCH_E("Failed to allocate memory for module platform_devce\n");
		return -ENOMEM;
	}

	md->pdev = pdev;

	pdev->name = LGE_TOUCH_MODULE_DRIVER_NAME;
	pdev->id = 2;
	pdev->num_resources = 0;
	pdev->dev.parent = &i2c->dev;
	pdev->dev.platform_data = md;
	pdev->dev.release = touch_module_release;

	ret = platform_device_register(pdev);

	if (ret) {
		TOUCH_I("Module Touch Failed to allocate memory for touch platform_devce\n");
		return -ENODEV;
	}

	TOUCH_I("Module Touch platform device registered ...\n");

	return 0;
}

static int touch_bus_remove(struct i2c_client *i2c)
{

	struct module_data *md = to_module(&i2c->dev);
	struct ft3518_data *d = to_ft3518_data(&i2c->dev);

	TOUCH_I("%s\n", __func__);

	if (atomic_read(&md->core) != MODULE_NONE) {
		module_interrupt_control(&i2c->dev, MODULE_INTERRUPT_DISABLE);
		free_irq(md->irq, md);
		pm_qos_remove_request(&md->pm_qos_req);

		if (md->input)
			input_unregister_device(md->input);

		if (md->wq)
			destroy_workqueue(md->wq);

		if (md->m_driver.register_sysfs)
			sysfs_remove_group(&md->kobj, &module_attribute_group);

		kobject_del(&md->kobj);

		devm_kfree(d->dev, d);
	}

	platform_device_unregister(md->pdev);

	return 0;
}

static int module_i2c_pm_suspend(struct device *dev)
{
	TOUCH_I("%s\n", __func__);
	return 0;
}

static int module_i2c_pm_resume(struct device *dev)
{
	TOUCH_I("%s\n", __func__);
	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = MODULE_DEVICE_COMPATIBLE_NAME,},
	{ },
};

static const struct i2c_device_id tp_id[] = {
	{LGE_MODULE_DRIVER_NAME, 0 },
	{},
};

static const struct dev_pm_ops module_pm_ops = {
	.suspend = module_i2c_pm_suspend,
	.resume = module_i2c_pm_resume,
};

MODULE_DEVICE_TABLE(i2c, tp_id);

static void touch_pen_async_init(void *data, async_cookie_t cookie)
{
	int ret = 0;

	touch_module_driver.probe = touch_bus_probe;
	touch_module_driver.remove = touch_bus_remove;
	touch_module_driver.id_table = tp_id;
	touch_module_driver.driver.name = LGE_MODULE_DRIVER_NAME;
	touch_module_driver.driver.owner = THIS_MODULE;
	touch_module_driver.driver.pm = &module_pm_ops;
	touch_module_driver.driver.of_match_table = match_table;

	ret = i2c_add_driver(&touch_module_driver);
}

int touch_module_init(void)
{
	TOUCH_I("%s\n", __func__);
	async_schedule(touch_pen_async_init, NULL);
	return 0;
}

void touch_module_exit(void)
{
	TOUCH_I("%s\n", __func__);
	i2c_del_driver(&touch_module_driver);
	TOUCH_I("%s End!\n", __func__);
}

module_init(touch_module_init);
module_exit(touch_module_exit);

MODULE_AUTHOR("rangkast.jeong@lge.com");
MODULE_DESCRIPTION("LGE module driver v1");
MODULE_LICENSE("GPL");
