/*
 * touch_sw49106.c
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

#include "touch_sw49106.h"

/*
 *******************************************************************************
 * Device specific functions
 *******************************************************************************
 * [brief] specific functions for device to support different IC enviroment.
 *******************************************************************************
 * */
int sw49106_reg_read(struct device *dev, u16 addr, void *data, int size)
{
	struct module_data *md = to_module(dev);
	struct module_bus_msg msg = {0, };
	int ret = 0;

//	mutex_lock(&md->io_lock);
	if (!md) {
		TOUCH_E("module data is not located\n");
		return -ENOMEM;
	}

	md->tx_buf[0] = ((size > 4) ? 0x20 : 0x00);
	md->tx_buf[0] |= ((addr >> 8) & 0x0f);
	md->tx_buf[1] = (addr & 0xff);

	msg.tx_buf = md->tx_buf;
	msg.tx_size = W_HEADER_SIZE;

	msg.rx_buf = md->rx_buf;
	msg.rx_size = R_HEADER_SIZE + size;

	ret = module_i2c_read(to_i2c_client(dev), &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		return ret;
	}

	memcpy(data, &md->rx_buf[R_HEADER_SIZE], size);

//	mutex_unlock(&md->io_lock);

	return 0;
}

int sw49106_reg_write(struct device *dev, u16 addr, void *data, int size)
{
	struct module_data *md = to_module(dev);
	struct module_bus_msg msg = {0, };
	int ret = 0;

//	mutex_lock(&md->io_lock);
	if (!md) {
		TOUCH_E("module data is not located\n");
		return -ENOMEM;
	}

	md->tx_buf[0] = ((size > 4) ? 0x60 : 0x40);
	md->tx_buf[0] |= ((addr >> 8) & 0x0f);
	md->tx_buf[1] = (addr  & 0xff);

	msg.tx_buf = md->tx_buf;
	msg.tx_size = W_HEADER_SIZE + size;

	msg.rx_buf = NULL;
	msg.rx_size = 0;

	memcpy(&md->tx_buf[W_HEADER_SIZE], data, size);

	ret = module_i2c_write(to_i2c_client(dev), &msg);
	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		return ret;
	}

//	mutex_unlock(&md->io_lock);

	return 0;
}

/* (1 << 5)|(1 << 6)|(1 << 7)|(1 << 9)|(1 << 10) */
/*#define INT_RESET_CLR_BIT   0x6C0*/  /*Global reset use only*/
/* (1 << 10) */  /* Global Reset bit */
#define INT_GLOBAL_RESET_CLR_BIT 0x400
/* (1 << 6)|(1 << 7)|(1 << 9) */  /* Touch HW Reset bit */
#define INT_HW_RESET_CLR_BIT 0x2C0
/* LG4894 not set */
#define INT_SW_RESET_CLR_BIT 0x0
/* (1 << 13)|(1 << 15)|(1 << 20)|(1 << 22) */
#define INT_LOGGING_CLR_BIT     0x50A000
/* LG4894 not set */
#define INT_FWU_CLR_BIT      0x0
/* (1 << 5) |(1 << 6) |(1 << 7)|(0 << 9)|(0 << 10)|(0 << 13)|(1 << 15)|(1 << 20)|(1 << 22) */
#define INT_NORMAL_MASK                  0x5080E0
#define IC_DEBUG_SIZE              16       /* byte */
/* During tumble, IC status is abnormal due to I2C error occurs and requires HW reset */
#define ABNORMAL_IC_BIT	0x6F000C0

int sw49106_check_status(struct device *dev)
{
	int ret = 0;
	u32 ic_status = d->info.ic_status; //0x200
	u32 status = d->info.device_status; //0x201
	u32 debugging_num = 0;
	u32 debugging_mask = 0x0;
	u32 status_mask = 0x0;
	int checking_log_flag = 0;
	const int checking_log_size = 1024;
	char *checking_log = NULL;
	int length = 0;

	status_mask = status ^ INT_NORMAL_MASK; //mask 1 is OK bit
	debugging_mask = ((status >> 16) & 0xF);
	if ((status_mask & INT_GLOBAL_RESET_CLR_BIT) || (ic_status & (1 << 0)) || (ic_status & (1 << 3)) || (ic_status & (1 << 6)) || (ic_status & (1 << 7))) {
		TOUCH_I("%s : Need Global Reset, status = %x, ic_status = %x\n",
				__func__, status, ic_status);
		//		     ret = -EGLOBALRESET;
	} else if ((status_mask & INT_HW_RESET_CLR_BIT)
			|| (ic_status & (1 << 5))) {
		TOUCH_I("%s : Need Touch HW Reset, status = %x, ic_status = %x\n",
				__func__, status, ic_status);
		//		     ret = -EHWRESET;
	} else if (status_mask & INT_SW_RESET_CLR_BIT) {
		TOUCH_I("%s : Need Touch SW Reset, status = %x, ic_status = %x\n",
				__func__, status, ic_status);
		//		     ret = -ESWRESET;
	} else if ((status_mask & INT_LOGGING_CLR_BIT) || (debugging_mask == 0x4)) {
		if (status == ABNORMAL_IC_BIT) {
			TOUCH_I("%s : Need Touch HW reset, status = %x, ic_status = %x\n",
					__func__, status, ic_status);
			//		     ret = -EHWRESET;
		} else {
			TOUCH_I("%s : Need Logging, status = %x, ic_status = %x\n",
					__func__, status, ic_status);
			ret = -ERANGE;
		}
	} else if (status_mask & INT_FWU_CLR_BIT) {
		if (d->err_cnt >= 3) {
			ret = -ERANGE;
		} else {
			d->err_cnt++;
			//                                ret = -EUPGRADE;
		}
		TOUCH_I("%s : Need FW Upgrade, status = %x, ic_status = %x err_cnt = %d %s\n",
				__func__, status, ic_status, d->err_cnt,
				d->err_cnt >= 3 ? " skip upgrade":"");
	}

	if (ret != 0) {
		checking_log = kcalloc(checking_log_size, sizeof(*checking_log), GFP_KERNEL);
		if (checking_log == NULL) {
			TOUCH_E("Failed to allocate mem for checking_log\n");
			ret = -ENOMEM;
			goto error;
		}
		if (ic_status & (1 << 0)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[IC_STATUS] [0]ESD detection");
		}
		if (ic_status & (1 << 3)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[IC_STATUS] [3]Watchdog Exception");
		}
		if (ic_status & (1 << 5)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[IC_STATUS] [5]CM3 Fault Status");
		}
		if (ic_status & (1 << 6)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[IC_STATUS] [6]MIPI ESD Error (DIC MIPI ERROR)");
		}
		if (ic_status & (1 << 7)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[IC_STATUS] [7]Check Sum Error (DIC Check Sum ERROR)");
		}
		if (!(status & (1 << 5))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[TC_STATUS] [5]Device_ctl not Set");
		}
		if (!(status & (1 << 6))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[TC_STATUS] [6]Code CRC Invalid");
		}
		if (!(status & (1 << 7))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[TC_STATUS] [7]CFG CRC Invalid");
		}
		if (status & (1 << 9)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[TC_STATUS] [9]Abnormal status Detected");
		}
		if (status & (1 << 10)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[TC_STATUS] [10]System Error Detected");
		}
		if (status & (1 << 13)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[TC_STATUS] [13]Display mode Mismatch");
		}
		if (!(status & (1 << 15))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[TC_STATUS] [15]Interrupt_Pin Invalid");
		}
		if (!(status & (1 << 20))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[TC_STATUS] [20]Touch interrupt status Invalid");
		}
		if (!(status & (1 << 22))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[TC_STATUS] [22]TC driving Invalid");
		}

		if (checking_log_flag) {
			TOUCH_E("%s, status = %x, ic_status = %x\n",
					checking_log, status, ic_status);
		}
		if (checking_log != NULL)
			kfree(checking_log);
	}

	/*
	 *        0x3 abnormal Error
	 *        0x4 debugging INT
	 */
	if (debugging_mask == 0x3 || debugging_mask == 0x4) {
		sw49106_reg_read(dev, ic_debug_info_addr, &d->info.debug[0].ic_debug_info,
				sizeof(d->info.debug[0].ic_debug_info));
		debugging_num = d->info.debug[0].ic_debug_info;
		if (debugging_num < IC_DEBUG_INFO_NUM)
			TOUCH_E("[IC_DEBUG_INFO] [%d]%s\n", debugging_num,
					ic_debug_info_str[debugging_num]);
	}
error:
	return ret;
}

#define TOUCH_IRQ_FINGER		(1 << 0)
static int sw49106_irq_abs_data(struct device *dev)
{
	struct module_data *md = to_module(dev);
	struct sw49106_touch_data *data = d->info.data;
	struct touch_data_module *tdata;
	u32 touch_count = 0;
	u8 finger_index = 0;
	int ret = 0;
	int i = 0;

	if (!md) {
		TOUCH_E("module data is not located\n");
		return -ENOMEM;
	}

	touch_count = d->info.touch_cnt;
	md->new_mask = 0;

	/* check q cover status */
//	if (d->driving_mode == LCD_MODE_U3_QUICKCOVER && !d->q_sensitivity) {
//		TOUCH_I("Interrupt in Qcover closed\n");
//		ts->is_cancel = 1;
//		ts->tcount = 0;
//		ts->intr_status = TOUCH_IRQ_FINGER;
//		return ret;
//	}

	/* check if palm detected */
	if (data[0].track_id == PALM_ID) {
		if (data[0].event == TOUCHSTS_DOWN) {
			md->is_cancel = 1;
			TOUCH_I("Palm Detected\n");
		} else if (data[0].event == TOUCHSTS_UP) {
			md->is_cancel = 0;
			TOUCH_I("Palm Released\n");
		}
		md->tcount = 0;
		md->intr_status = TOUCH_IRQ_FINGER;
		return ret;
	}

	for (i = 0; i < touch_count; i++) {
		if (data[i].track_id >= MAX_FINGER)
			continue;

		if (data[i].event == TOUCHSTS_DOWN
			|| data[i].event == TOUCHSTS_MOVE) {
			md->new_mask |= (1 << data[i].track_id);
			tdata = md->tdata + data[i].track_id;

			tdata->id = data[i].track_id;
			tdata->type = data[i].tool_type;
			tdata->x = data[i].x;
			tdata->y = data[i].y;
			tdata->pressure = data[i].pressure;
			tdata->width_major = data[i].width_major;
			tdata->width_minor = data[i].width_minor;

			if (data[i].width_major == data[i].width_minor)
				tdata->orientation = 1;
			else
				tdata->orientation = (s8)data[i].angle;

			finger_index++;
/*
			TOUCH_I("tdata [id:%d t:%d x:%d y:%d z:%d-%d,%d,%d]\n",
					tdata->id,
					tdata->type,
					tdata->x,
					tdata->y,
					tdata->pressure,
					tdata->width_major,
					tdata->width_minor,
					tdata->orientation);
*/
		}
	}

	md->tcount = finger_index;
	md->intr_status = TOUCH_IRQ_FINGER;

	return ret;
}

int sw49106_irq_abs(struct device *dev)
{
	struct module_data *md = to_module(dev);

	if (!md) {
		TOUCH_E("module data is not located\n");
		return -ENOMEM;
	}

	/* check if touch cnt is valid */
	if (d->info.touch_cnt == 0 || d->info.touch_cnt > md->dts.max_id) {
		TOUCH_I("%s : touch cnt is invalid - %d\n",
			__func__, d->info.touch_cnt);
		return -ERANGE;
	}

	return sw49106_irq_abs_data(dev);
}

static int sw49106_condition_wait(struct device *dev,
				    u16 addr, u32 *value, u32 expect,
				    u32 mask, u32 delay, u32 retry)
{
	u32 data = 0;

	do {
		module_msleep(delay);
		sw49106_read_value(dev, addr, &data);

		if ((data & mask) == expect) {
			if (value)
				*value = data;
			TOUCH_I(
				"%d, addr[%04x] data[%08x], mask[%08x], expect[%08x]\n",
				retry, addr, data, mask, expect);
			return 0;
		}
	} while (--retry);

	if (value)
		*value = data;

	TOUCH_I("%s addr[%04x], expect[%x], mask[%x], data[%x]\n",
		__func__, addr, expect, mask, data);

	return -EPERM;
}

int specific_header_verify(unsigned char *header, int i)
{
	t_cfg_s_header_def *head = (t_cfg_s_header_def *)header;
	char tmp[8] = {0, };

	if (head->cfg_specific_info1.b.chip_rev <= 0
		&& head->cfg_specific_info1.b.chip_rev > 10) {
		TOUCH_I("Invalid Chip revision id %8.8X\n",
			head->cfg_specific_info1.b.chip_rev);
		return -2;
	}

	memset(tmp, 0, 8);
	memcpy((void *)tmp, (void *)&head->cfg_model_name, 4);

	TOUCH_I("==================== SPECIFIC #%d =====================\n",
						i + 1);
	TOUCH_I("chip_rev           : %d\n",
					head->cfg_specific_info1.b.chip_rev);
	TOUCH_I("fpcb_id            : %d\n",
					head->cfg_specific_info1.b.fpcb_id);
	TOUCH_I("lcm_id             : %d\n",
					head->cfg_specific_info1.b.lcm_id);
	TOUCH_I("model_id           : %d\n",
					head->cfg_specific_info1.b.model_id);
	TOUCH_I("model_name         : %s\n", tmp);
	TOUCH_I("lot_id             : %d\n",
					head->cfg_specific_info2.b.lot_id);
	TOUCH_I("ver                : %d\n",
					head->cfg_specific_version);

	return 1;
}

int common_header_verify(t_cfg_info_def *header)
{
	t_cfg_info_def *head = (t_cfg_info_def *)header;
	t_cfg_c_header_def *common_head =
		(t_cfg_c_header_def *)(header + sizeof(t_cfg_info_def));

	if (head->cfg_magic_code != CFG_MAGIC_CODE) {
		TOUCH_I("Invalid CFG_MAGIC_CODE. %8.8X\n",
			head->cfg_magic_code);
		return -1;
	}

	if (head->cfg_chip_id != CFG_CHIP_ID) {
		TOUCH_I("Invalid Chip ID. (49106 != %d)\n",
			head->cfg_chip_id);
		return -2;
	}

	if (head->cfg_struct_version <= 0) {
		TOUCH_I("Invalid cfg_struct_version. %8.8X\n",
			head->cfg_struct_version);
		return -3;
	}

	if (head->cfg_specific_cnt <= 0) {
		TOUCH_I("No Specific Data. %8.8X\n",
			head->cfg_specific_cnt);
		return -4;
	}

	if (head->cfg_size.b.common_cfg_size > CFG_C_MAX_SIZE) {
		TOUCH_I("Over CFG COMMON MAX Size (%d). %8.8X\n",
			CFG_C_MAX_SIZE, head->cfg_size.b.common_cfg_size);
		return -5;
	}

	if (head->cfg_size.b.specific_cfg_size > CFG_S_MAX_SIZE) {
		TOUCH_I("Over CFG SPECIFIC MAX Size (%d). %8.8X\n",
			CFG_S_MAX_SIZE, head->cfg_size.b.specific_cfg_size);
		return -6;
	}

	TOUCH_I("==================== COMMON ====================\n");
	TOUCH_I("magic code         : 0x%8.8X\n", head->cfg_magic_code);
	TOUCH_I("chip id            : %d\n", head->cfg_chip_id);
	TOUCH_I("struct_ver         : %d\n", head->cfg_struct_version);
	TOUCH_I("specific_cnt       : %d\n", head->cfg_specific_cnt);
	TOUCH_I("cfg_c size         : %d\n", head->cfg_size.b.common_cfg_size);
	TOUCH_I("cfg_s size         : %d\n",
					head->cfg_size.b.specific_cfg_size);
	TOUCH_I("date               : 0x%8.8X\n", head->cfg_global_date);
	TOUCH_I("time               : 0x%8.8X\n", head->cfg_global_time);
	TOUCH_I("common_ver         : %d\n", common_head->cfg_common_ver);

	return 1;
}

static int sw49106_img_binary_verify(unsigned char *imgBuf)
{
	unsigned char *specific_ptr;
	unsigned char *cfg_buf_base = &imgBuf[FLASH_FW_SIZE];
	int i;
	t_cfg_info_def *head = (t_cfg_info_def *)cfg_buf_base;

	u32 *fw_crc = (u32 *)&imgBuf[FLASH_FW_SIZE - 4];
	u32 *fw_size = (u32 *)&imgBuf[FLASH_FW_SIZE - 8];

	if (*fw_crc == 0x0
		|| *fw_crc == 0xFFFFFFFF
		|| *fw_size > FLASH_FW_SIZE) {
		TOUCH_I("Firmware Size Invalid READ : 0x%X\n", *fw_size);
		TOUCH_I("Firmware CRC Invalid READ : 0x%X\n", *fw_crc);
		return E_FW_CODE_SIZE_ERR;
	} else {
		TOUCH_I("Firmware Size READ : 0x%X\n", *fw_size);
		TOUCH_I("Firmware CRC READ : 0x%X\n", *fw_crc);
	}

	if (common_header_verify(head) < 0) {
		TOUCH_I("No Common CFG! Firmware Code Only\n");
		return E_FW_CODE_ONLY_VALID;
	}

	specific_ptr = cfg_buf_base + head->cfg_size.b.common_cfg_size;
	for (i = 0; i < head->cfg_specific_cnt; i++) {
		if (specific_header_verify(specific_ptr, i) < 0) {
			TOUCH_I("specific CFG invalid!\n");
			return -2;
		}
		specific_ptr += head->cfg_size.b.specific_cfg_size;
	}

	return E_FW_CODE_AND_CFG_VALID;
}


static int sw49106_fw_upgrade(struct device *dev, const struct firmware *fw)
{
	u8 *fwdata = (u8 *) fw->data;
	u32 data;
	u32 conf_dn_addr;
	u32 conf_specific_dn_index;
	u32 cfg_c_size;
	u32 cfg_s_size;
	t_cfg_info_def *head;
	int ret;
	int i = 0;
	int img_check_result;

	//--------- Binary Check Verification Start-----------------------
		TOUCH_I("%s - Checking FW Image before flashing\n", __func__);
		if (fw->size > FLASH_SIZE) {
			TOUCH_I("%s - FW Image Size is not correct\n", __func__);
			return -EPERM;
		} else {
			TOUCH_I("%s - FLASH_FW_SIZE= 0x%x\n", __func__, FLASH_FW_SIZE);
		}
	if (0) {//CFG area deleted
		img_check_result = sw49106_img_binary_verify((unsigned char *)fwdata);

		switch (img_check_result) {
		case E_FW_CODE_AND_CFG_VALID:
			break;
		case E_FW_CODE_CFG_ERR:
		case E_FW_CODE_SIZE_ERR:
		case E_FW_CODE_ONLY_VALID:
		default:
			TOUCH_I("%s - FW Image Verification fail!!\n", __func__);
			return -EPERM;
		}
		TOUCH_I("%s - FW Image Verification success!!\n", __func__);
	}
	//-----------Binary Check Verification End-------------------------
	/* enable SPI between RAM and ROM */
	sw49106_write_value(dev, 0x15, 0);

	//---------------------------------------------------
	/* Reset Touch CM3 core and put system on hold */
	sw49106_write_value(dev, sys_rst_ctl, 2);

	/* sram write enable */
	sw49106_write_value(dev, sys_sram_ctl, 3);

	/* Write F/W Code to CODE SRAM (80KB) */
	for (i = 0 ; i < FLASH_FW_SIZE ; i += MAX_RW_SIZE) {

		/* Set code sram base address write */
		sw49106_write_value(dev, spr_code_offset, i / 4);

		/* firmware image download to code sram */
		sw49106_reg_write(dev, code_access_addr, &fwdata[i], MAX_RW_SIZE);
	}
	/* sram write disable */
	sw49106_write_value(dev, sys_sram_ctl, 0);

	/* Release Touch CM3 core reset*/
	sw49106_write_value(dev, sys_rst_ctl, 0);

	/* Start CM3 Boot after Code Dump */
	sw49106_write_value(dev, sys_boot_ctl, 1);

	/* Check F/W Boot Done Status */
	ret = sw49106_condition_wait(dev, tc_flash_dn_sts, NULL,
				    FLASH_BOOTCHK_VALUE, 0xFFFFFFFF, 10, 200);
	if (ret < 0) {
		TOUCH_E("failed : \'boot check\'\n");
		return -EPERM;
	} else {
		TOUCH_I("success : boot check\n");
	}
	//---------------------------------------------------

	//--------------F/W Code Flash Download Start---------
		/* Dump F/W Code with Flash DMA */
	sw49106_write_value(dev, tc_flash_dn_ctl, (FLASH_KEY_CODE_CMD << 16) | 1);
	module_msleep(md->dts.hw_reset_delay);

	/* Check F/W Code Flash Download Status */
	ret = sw49106_condition_wait(dev, tc_flash_dn_sts, &data,
			FLASH_CODE_DNCHK_VALUE, 0xFFFFFFFF, 10, 200);
	if (ret < 0) {
		TOUCH_E("failed : \'code check\'\n");
		return -EPERM;
	} else {
		TOUCH_I("success : code check\n");
	}
	//--------------F/W Code Flash Download End---------

	if (0) { //blocked CFG, OTPM Download as per vendor suggestion
		if (img_check_result == E_FW_CODE_AND_CFG_VALID) {
			head = (t_cfg_info_def *)&fwdata[FLASH_FW_SIZE];

			cfg_c_size = head->cfg_size.b.common_cfg_size;
			cfg_s_size = head->cfg_size.b.specific_cfg_size;

			/* conf base address read */
			sw49106_reg_read(dev, tc_confdn_base_addr, (u8 *)&data, sizeof(u32));
			conf_dn_addr =  ((data) & 0xFFFF);
			conf_specific_dn_index = ((data >> 16) & 0xFFFF);
			TOUCH_I("conf_dn_addr : %08x data: %08x conf_specific_dn_index : %08x\n",
					conf_specific_dn_index, conf_dn_addr, data);
			if (conf_specific_dn_index == 0 ||
					((conf_specific_dn_index * cfg_s_size) >
					 (fw->size - FLASH_FW_SIZE - cfg_c_size))) {
				TOUCH_I("Invalid Specific CFG Index => 0x%8.8X\n",
						conf_specific_dn_index);
				return -EPERM;
			}
			if (conf_dn_addr >= (0x1600) || conf_dn_addr < (0x8C0)) {
				TOUCH_E("failed : \'conf base invalid \'\n");
				return -EPERM;
			}
			//--------------Config Data Flash Download Start-------------------
			/* cfg_c sram base address write */
			sw49106_write_value(dev, spr_data_offset, conf_dn_addr);

			/* Conf data download to conf sram */
			sw49106_reg_write(dev, data_access_addr, &fwdata[FLASH_FW_SIZE], cfg_c_size);

			/* cfg_s sram base address write */
			sw49106_write_value(dev, spr_data_offset, conf_dn_addr + cfg_c_size/4);

			// CFG Specific Download to CFG Download buffer (SRAM)
			sw49106_reg_write(dev, data_access_addr, &fwdata[FLASH_FW_SIZE + cfg_c_size +
					(conf_specific_dn_index - 1) * cfg_s_size], cfg_s_size);

			/* Conf Download Start */
			sw49106_write_value(dev, tc_flash_dn_ctl, (FLASH_KEY_CONF_CMD << 16) | 2);

			/* Conf check */
			ret = sw49106_condition_wait(dev, tc_flash_dn_sts, &data,
					FLASH_CONF_DNCHK_VALUE, 0xFFFFFFFF, 10, 200);
			if (ret < 0) {
				TOUCH_E("failed : \'cfg check\'\n");
				return -EPERM;
			} else {
				TOUCH_I("success : cfg_check\n");
			}

			ret = specific_header_verify(&fwdata[FLASH_FW_SIZE + cfg_c_size + (conf_specific_dn_index - 1)*cfg_s_size],
					conf_specific_dn_index - 1);
			if (ret < 0) {
				TOUCH_I("specific header invalid!\n");
				return -EPERM;
			}
			//--------------Config Data Flash down End-------------------

			//------------------OTPM Download Start-----------------
			/* cfg_s sram base address write */
			sw49106_write_value(dev, spr_data_offset, conf_dn_addr);

			// OTPM Data Download to (SRAM)
			sw49106_reg_write(dev, data_access_addr, &fwdata[FLASH_FW_SIZE +
					FLASH_CONF_SIZE], FLASH_OTP_SIZE);

			/* OTPM Download Start */
			sw49106_write_value(dev, tc_flash_dn_ctl, (FLASH_KEY_OTPM_CMD << 16) | 4);

			/*Check OTPM Data Flash Download Status */
			ret = sw49106_condition_wait(dev, tc_flash_dn_sts, &data,
					FLASH_OTP_DNCHK_VALUE, 0xFFFFFFFF, 10, 200);
			if (ret < 0) {
				TOUCH_E("failed : \'OTPM check\'\n");
				return -EPERM;
			} else {
				TOUCH_I("success : OTPM check\n");
			}
			//------------------OTPM Download End------------------
		}
	}

	TOUCH_I("===== Firmware download Okay =====\n");

	return 0;
}

static int sw49106_fw_compare(struct device *dev, const struct firmware *fw)
{
	u8 dev_major = d->fw.version[0];
	u8 dev_minor = d->fw.version[1];
	u32 bin_ver_offset = *((u32 *)&fw->data[0xe8]);
	u32 bin_pid_offset = *((u32 *)&fw->data[0xf0]);
	char pid[12] = {0};
	u8 bin_major;
	u8 bin_minor;
	int update = 0;

	if ((bin_ver_offset > FLASH_FW_SIZE) || (bin_pid_offset > FLASH_FW_SIZE)) {
		TOUCH_I("INVALID OFFSET\n");
		return -1;
	}

	bin_major = fw->data[bin_ver_offset];
	bin_minor = fw->data[bin_ver_offset + 1];
	memcpy(pid, &fw->data[bin_pid_offset], 8);

	if (bin_major != dev_major) {
		update = 1;
	} else {
		if (bin_minor != dev_minor)
			update = 1;
	}

	TOUCH_I("bin-ver: %d.%02d (%s), dev-ver: %d.%02d -> update: %d\n",
		bin_major, bin_minor, pid, dev_major, dev_minor, update);

	return update;
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

	TOUCH_I("reset_pin : %d\n ", md->dts.reset_pin);
	TOUCH_I("int_pin : %d\n ", md->dts.int_pin);
	TOUCH_I("irqflags : %X\n ", md->dts.irqflags);
	TOUCH_I("max_x : %d\n ", md->dts.max_x);
	TOUCH_I("max_y : %d\n ", md->dts.max_y);
	TOUCH_I("max_pressure : %d\n ", md->dts.max_pressure);
	TOUCH_I("max_width_major : %d\n ", md->dts.max_width_major);
	TOUCH_I("minor : %d\n ", md->dts.max_width_minor);
	TOUCH_I("orientation : %d\n ", md->dts.max_orientation);
	TOUCH_I("max_id : %d\n ", md->dts.max_id);
	TOUCH_I("hw_reset_delay : %d\n ", md->dts.hw_reset_delay);
	TOUCH_I("sw_reset_delay : %d\n ", md->dts.sw_reset_delay);

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
	int ret = 0;

	if (buf_size) {
		md->tx_buf = devm_kzalloc(md->dev, buf_size, GFP_KERNEL | GFP_DMA);
		if (!md->tx_buf)
			TOUCH_E("fail to allocate tx_buf\n");

		md->rx_buf = devm_kzalloc(md->dev, buf_size, GFP_KERNEL | GFP_DMA);
		if (!md->rx_buf)
			TOUCH_E("fail to allocate rx_buf\n");
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

		if (!IS_ERR_OR_NULL(md->pinctrl.active)) {
			ret = pinctrl_select_state(md->pinctrl.ctrl,
					md->pinctrl.active);
			if (ret)
				TOUCH_I("cannot set pinctrl.active\n");
			else
				TOUCH_I("pinctrl set active\n");
		}
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

	ret = i2c_transfer(client->adapter, &msgs[0], 1);
	ret += i2c_transfer(client->adapter, &msgs[1], 1);

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

int module_ic_info(struct module_data *md)
{
	struct module_data *data = (struct module_data *)md;
	struct device *dev = data->dev;
	int ret = 0;
	u32 version = 0;
	u32 revision = 0;
	u32 bootmode = 0;
	u32 lcdrevision = 0;
	u32 product[2] = {0};
	char rev_str[32] = {0};

	ret = sw49106_reg_read(dev, tc_version, &version, sizeof(version));
	if (ret < 0) {
		TOUCH_I("version : %x\n", version);
		return ret;
	}

	ret = sw49106_reg_read(dev, info_chip_revision, &revision, sizeof(revision));
	ret = sw49106_reg_read(dev, tc_product_id1, &product[0], sizeof(product));
	ret = sw49106_reg_read(dev, spr_boot_st, &bootmode, sizeof(bootmode));
	ret = sw49106_reg_read(dev, info_lcd_revision, &lcdrevision, sizeof(lcdrevision));

	d->fw.version[0] = ((version >> 8) & 0xFF);
	d->fw.version[1] = version & 0xFF;
	d->fw.revision = revision & 0xFF;
	d->fw.lcd_fpcb_revision = lcdrevision;
	memcpy(&d->fw.product_id[0], &product[0], sizeof(product));

	if (d->fw.revision == 0xFF)
		snprintf(rev_str, 32, "revision: Flash Erased(0xFF)");
	else
		snprintf(rev_str, 32, "revision: %d", d->fw.revision);

	TOUCH_I("version : v%d.%02d, chip : %d, protocol : %d\n" \
		"[Touch] %s\n" \
		"[Touch] product id : %s\n" \
		"[Touch] flash boot : %s, %s, crc : %s\n",
		d->fw.version[0], d->fw.version[1],
		(version >> 16) & 0xFF, (version >> 24) & 0xFF, rev_str,  d->fw.product_id,
		(bootmode >> 0 & 0x1) ? "BUSY" : "idle",
		(bootmode >> 1 & 0x1) ? "done" : "booting",
		(bootmode >> 2 & 0x1) ? "ERROR" : "ok");

	TOUCH_I("lcd fpcb revision : %d\n", d->fw.lcd_fpcb_revision);
	if (((((version >> 16) & 0xFF) != VCHIP_VAL) || (((version >> 24) & 0x0F) != VPROTO_VAL))) {
		TOUCH_I("FW is in abnormal state because of ESD or something.\n");
//		sw49106_reset_ctrl(dev, HW_RESET_ASYNC);
		//sw49106_power(dev, POWER_OFF);
		//sw49106_power(dev, POWER_ON);
		//touch_msleep(ts->caps.hw_reset_delay);
	}

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
	int ret = 0;

	//TOUCH_I("%s\n", __func__);

	ret = sw49106_reg_read(md->dev, tc_ic_status, &d->info,
			sizeof(d->info));
	if (ret < 0)
		TOUCH_I("error in %s 1\n", __func__);

	/*
	 * 2. Check status
	 * */
	ret = sw49106_check_status(md->dev);
	if (ret < 0)
		TOUCH_I("error in %s 2\n", __func__);

	//TOUCH_I("d->info.wakeup_type = %d\n", d->info.wakeup_type);
	if (d->info.wakeup_type == ABS_MODE)
		ret = sw49106_irq_abs(md->dev);
//	else
//		ret = sw49106_irq_lpwg(dev);

	if (md->intr_status & TOUCH_MODULE_IRQ_FINGER)
		touch_report_event(md);

	return IRQ_HANDLED;
}

void module_enable_irq(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	if (desc) {
		if (desc->istate & IRQS_PENDING)
			TOUCH_I("Remove pending irq(%d)\n", irq);
		desc->istate &= ~(IRQS_PENDING);
	}
	enable_irq(irq);
}

void module_disable_irq(unsigned int irq)
{
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
static int match_module_ic(void *md)
{
	int ret = 0;
	int retry = 0;
	struct module_data *data;
	u8 value;

	TOUCH_I("%s\n", __func__);
	data = (struct module_data *)md;

	ret = module_get_dts_data(data);
	if (ret < 0)
		TOUCH_I("get dts data error!\n");

	module_power_init(data);
	module_bus_init(data, 64 * 1024);

	do {
		ret = sw49106_reg_read(data->dev, retry, &value, sizeof(value));
		if (ret == -107 || ret == -110 || ret == -214 || ret == -220) {
			retry++;
			continue;
		} else if (ret == 0) {
			TOUCH_I("find touch module!!");
			break;
		}
	} while (retry < 3);

	return ret;
}

static int module_probe(void *md)
{
	struct module_data *data;
	int ret = 0;

	TOUCH_I("%s\n", __func__);
	data = (struct module_data *)md;

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

//	module_gpio_init(md->dts.reset_pin, "module_touch_reset");
//	module_gpio_direction_output(md->dts.reset_pin, 0);

	module_gpio_init(data->dts.int_pin, "module_touch_int");
	module_gpio_direction_input(data->dts.int_pin);

	module_init_input(data);
	if (ret < 0) {
		TOUCH_E("failed to register input device(ret:%d)\n", ret);
		goto error_init_input;
	}

	ret = request_threaded_irq(data->irq, module_irq_handler,
			module_irq_thread, data->dts.irqflags | IRQF_ONESHOT,
			LGE_MODULE_IRQ_NAME, data);
	if (ret < 0) {
		TOUCH_E("failed to request_thread_irq(irq:%d, ret:%d)\n",
				data->irq, ret);
		goto error_request_irq;
	}
	atomic_set(&d->irq, REQUESTED_IRQ);

	module_disable_irq(data->irq);

	data->m_driver.notify(1);

//	data->m_driver.upgrade((void *)data);

//	mutex_init(&data->io_lock);

	return 0;

error_request_irq:
	free_irq(data->irq, data);
	atomic_set(&d->irq, UNREQUESTED_IRQ);
error_init_input:
	if (data->input) {
		input_mt_destroy_slots(data->input);
		input_free_device(data->input);
	}
//error_init_work:
//	if (md->wq)
//		destroy_workqueue(md->wq);

	return 0;
}

static int module_remove(void *md)
{
	TOUCH_I("%s\n", __func__);

	return 0;
}

static int module_func(int control, char *data)
{
	char buf[LOG_BUF_SIZE] = {0,};
	int ret = 0;
	int i = 0;
	u32 test_data = 0;

	TOUCH_I("module_func : %d\n", control);

	switch (control) {
	case 1:
		break;
	case 2:
		TOUCH_I("test read registers\n");
		for (i = 0; i < 10; i++) {
			ret = sw49106_reg_read(md->dev, i, &test_data, sizeof(test_data));
			if (ret < 0)
				TOUCH_I("reg[%d] read error %d\n", i, ret);
			else
				TOUCH_I("test_data[%d] %02X\n", i, test_data);
		}
		return 0;
		break;
	case 3:
		md->m_driver.upgrade((void *)md);
		break;
	default:
		TOUCH_I("not support mode: %d\n", control);
		break;
	}

	memcpy(data, buf, sizeof(buf));

	return ret;
}

static int module_initialize(void *md)
{
	/*
	 *  [TODO] initialize sw49107
	 *
	 *  1. read ic info
	 *  2. activate default sensing (U3)
	 *
	 * */
	int ret = 0;
	u32 data = 1;
	u32 ctrl = 0;
	struct module_data *mdata = md;
	struct device *dev = mdata->dev;

	TOUCH_I("%s\n", __func__);

	sw49106_write_value(dev, SERIAL_SPI_EN, 0);
	TOUCH_I("Serial Control Addr=[%04x] val=[%04x]\n", SERIAL_SPI_EN, 0);
	sw49106_write_value(dev, SERIAL_I2C_EN, 1);
	TOUCH_I("Serial Control Addr=[%04x] val=[%04x]\n", SERIAL_I2C_EN, 1);
	sw49106_write_value(dev, SPI_TATTN_OPT, 3);
	TOUCH_I("Serial Control Addr=[%04x] val=[%04x]\n", SPI_TATTN_OPT, 3);

	ret = module_ic_info(mdata);

	ret = sw49106_reg_write(dev, tc_device_ctl, &data, sizeof(data));
	if (ret) {
		TOUCH_E("failed to write \'tc_device_ctrl\', ret:%d\n", ret);
		return ret;
	}

	ret = sw49106_reg_write(dev, tc_interrupt_ctl, &data, sizeof(data));
	if (ret) {
		TOUCH_E("failed to write \'tc_interrupt_ctrl\', ret:%d\n", ret);
		return ret;
	}
//	ret = sw49106_reg_write(dev, SPR_CHARGER_STS, &d->charger, sizeof(u32));
//	if (ret)
//		TOUCH_E("failed to write \'spr_charger_sts\', ret:%d\n", ret);
//
//	data = atomic_read(&ts->state.ime);
//	ret = sw49106_reg_write(dev, REG_IME_STATE, &data, sizeof(data));
//	if (ret)
//		TOUCH_E("failed to write \'reg_ime_state\', ret:%d\n", ret);
//	atomic_set(&d->init, IC_INIT_DONE);
//	atomic_set(&ts->state.sleep, IC_NORMAL);
//
//	ret = sw49106_lpwg_mode(dev);
//	if (ret)
//		TOUCH_E("failed to lpwg_control, ret:%d", ret);

	ctrl = 0x181;
	sw49106_reg_write(dev, tc_drive_ctl, &ctrl, sizeof(ctrl));

	module_enable_irq(mdata->irq);

	return 0;
}

static int module_resume(void)
{
	TOUCH_I("%s\n", __func__);
	return 0;
}

static int module_suspend(void)
{
	TOUCH_I("%s\n", __func__);
	return 0;
}

static int module_upgrade(void *md)
{
	struct module_data *data = (struct module_data *)md;
	const struct firmware *fw = NULL;
	char fwpath[256] = {0};
	int ret = 0;
	int i = 0;

	TOUCH_I("%s\n", __func__);

	/*
	 * [TODO]
	 * 1. compare version between ic / bin
	 * 2. call device specific upgrade function (think about Uevent for module)
	 * */

	if (data->dts.def_fwcnt) {
		memcpy(fwpath, data->dts.def_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from def_fwpath : %s\n", fwpath);
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	fwpath[sizeof(fwpath) - 1] = '\0';

	ret = request_firmware(&fw, fwpath, data->dev);
	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n",
			fwpath, ret);

		return ret;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	if (sw49106_fw_compare(data->dev, fw)) {
		ret = -EINVAL;
		module_msleep(200);
		for (i = 0; i < 2 && ret; i++)
			ret = sw49106_fw_upgrade(data->dev, fw);
	} else {
		release_firmware(fw);
		return 0;
	}
	if (!ret) {
		d->err_cnt = 0;
		TOUCH_I("FW upgrade retry err_cnt clear\n");
	}

	release_firmware(fw);

	return ret;
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
	md->m_driver.upgrade = module_upgrade;

	dev_set_drvdata(&i2c->dev, md);

	pdev = devm_kzalloc(&i2c->dev, sizeof(*pdev), GFP_KERNEL);
	if (!pdev) {
		TOUCH_E("Failed to allocate memory for module platform_devce\n");
		return -ENOMEM;
	}

	md->pdev = pdev;

	pdev->name = LGE_TOUCH_DRIVER_NAME;
	pdev->id = 1;
	pdev->num_resources = 0;
	pdev->dev.parent = &i2c->dev;
	pdev->dev.platform_data = md;
	pdev->dev.release = touch_module_release;

	d = devm_kzalloc(&i2c->dev, sizeof(*d), GFP_KERNEL);

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
	struct module_data *data = (struct module_data *)md;

	TOUCH_I("%s\n", __func__);

	if (data->irq && (atomic_read(&d->irq) == REQUESTED_IRQ)) {
		module_disable_irq(data->irq);
		free_irq(data->irq, data);
		atomic_set(&d->irq, UNREQUESTED_IRQ);
	}

	if (data->input)
		input_unregister_device(data->input);

	platform_device_unregister(pdev);

	return 0;
}

#define MODULE_DEVICE_COMPATIBLE_NAME	    "lge,sw49106"
static const struct of_device_id match_table[] = {
	{ .compatible = MODULE_DEVICE_COMPATIBLE_NAME,},
	{ },
};

static const struct i2c_device_id tp_id[] = {
	{LGE_MODULE_DRIVER_NAME, 0 },
	{},
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
}

module_init(touch_module_init);
module_exit(touch_module_exit);

MODULE_AUTHOR("rangkast.jeong@lge.com");
MODULE_DESCRIPTION("LGE module driver v1");
MODULE_LICENSE("GPL");
