/*
 * ice40 Driver functions dedicated to ice40U (mcu)
 *
 * Copyright (c) 2018 LG Electronics, Inc
 *
 * All rights are reserved.
 *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING
 * THE SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *      PROJECT:   ice40 driver
 *      LANGUAGE:  ANSI C
 */

/* ========================================================================== */
/* Includes and Defines */
#ifndef DEBUG
#define DEBUG
#endif
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#ifdef OUT_OF_TREE
#include "ice40-core.h"
#else
#include <linux/ice40/ice40-core.h>
#endif
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>

#define MCU_FW_PATH "mcu/FLASH_SYSTEM.img"
#define LATTICE_FW_PATH "lattice/lg_singlewire_s48_impl_1.img"
#define MCU_UPDATE               0x01
#define LATTICE_UPDATE           0x02
#define I2C_W_MCU_FW_UPDATE      0x0001
#define I2C_W_LATTICE_FW_UPDATE  0x0006

/* ======================================================================== */
/* CHAN's FUNCTION PROTOTYPES */
//static struct regmap *regmap;

/* ======================================================================== */
/* LOCAL FUNCTION PROTOTYPES */
/* sysfs functions */
static ssize_t mcu_i2c2_register_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t mcu_i2c2_register_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size);

/* ======================================================================== */
/* DEVICE ATTRIBUTES */
/* RW */
struct device_attribute mcu_i2c2_register =
	__ATTR(FILE_REG_ACCESS_MCU,
			0660,
			mcu_i2c2_register_show,
			mcu_i2c2_register_store);

/* ========================================================================== */
/* Setup of regmap for ice40S */
/*static const struct regmap_range mcu_i2c2_volatile_ranges[] = {
	regmap_reg_range(MCU_REG_START, MCU_REG_END),
};

static const struct regmap_range mcu_i2c2_writeable_ranges[] = {
	regmap_reg_range(MCU_REG_START, MCU_REG_END),
};

static const struct regmap_range mcu_i2c2_readable_ranges[] = {
	regmap_reg_range(MCU_REG_START, MCU_REG_END),
};

static const struct regmap_access_table mcu_i2c2_volatile_table = {
	.yes_ranges = mcu_i2c2_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(mcu_i2c2_volatile_ranges),
};

static const struct regmap_access_table mcu_i2c2_writeable_table = {
	.yes_ranges = mcu_i2c2_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(mcu_i2c2_writeable_ranges),
};

static const struct regmap_access_table mcu_i2c2_readable_table = {
	.yes_ranges = mcu_i2c2_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(mcu_i2c2_readable_ranges),
};

static const struct regmap_config mcu_i2c2_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.volatile_table = &mcu_i2c2_volatile_table,
	.wr_table = &mcu_i2c2_writeable_table,
	.rd_table = &mcu_i2c2_readable_table,
	.max_register = MCU_REG_END,
	// FIXME: added to suppress errors when reading test regs
	.cache_type = REGCACHE_RBTREE,
};
*/

/*!
 *****************************************************************************
 *  \brief CHAN's regmap
 *****************************************************************************
 */
static ssize_t store_mcu_i2c2_bulkwrite(struct device *dev,
		      struct device_attribute *attr, const char *buf,
		      size_t count)
{
#if 1
	struct ice40 *ice40 = dev_get_drvdata(dev);
	u8 *data;
#endif
	uint addr;
	u16 reg;
	int ret = 0;
	char fwpath[256] = {0};
	char filename[256] = {0};
    const struct firmware *fw = NULL;

	if(sscanf(buf, "0x%hx,%255s", &reg, filename) <= 1) {
        printk(KERN_ERR "Usage> echo \"0xADDR16,ImageName.img\" > mcu_bulkwrite");
		return count;
    }

    if(strlen(filename) > 256) {
        printk(KERN_ERR "filename is too long\n");
        return count;
    }

	sprintf(fwpath, "mcu/%s", filename);
	printk(KERN_ERR "[ice40] mcu_i2c2_bulkwrite address is [0x%hx], filename is [%s]\n", reg, filename);
    if (fwpath == NULL) {
        printk(KERN_ERR "[ice40] ERR: mcu_i2c2's fwpath is null!\n");
        return count;
    }

    ret = request_firmware(&fw, fwpath, dev);
    if (ret < 0) {
        printk(KERN_ERR "[ice40] ERR: fail to mcu_i2c2's request_firmware fwpath!\n");
        return count;
    }
    else  printk(KERN_ERR "[ice40] mcu_i2c2's request_firmware is done!\n");

	addr = reg;

	//ret = regmap_bulk_write(regmap, addr, fw->data, fw->size);
#if 1
	data = kmalloc(fw->size + 2, GFP_KERNEL);
	if(!data) {
		printk(KERN_ERR "[ice40] %s kmalloc error\n", __func__);
		release_firmware(fw);
		return count;
	}
	data[0] = (addr >> 8) & 0xff;
	data[1] = addr & 0xff;
	memcpy(&data[2], fw->data, fw->size);
	ret = i2c_master_send(ice40->client_mcu, data, sizeof(data));
#endif

	if(ret < 0){
		printk(KERN_ERR "[ice40] Failed to write mcu_i2c2_bulkwrite, %d\n", ret);
	}
    else printk(KERN_ERR "[ice40] mcu_i2c2_bulkwrite done\n");

    release_firmware(fw);
#if 1
	kfree(data);
#endif

	return count;
}

static ssize_t store_mcu_i2c2_partialbulkwrite(struct device *dev,
		      struct device_attribute *attr, const char *buf,
		      size_t count)
{
#if 1
	struct ice40 *ice40 = dev_get_drvdata(dev);
	u8 *data;
	u8 *partial_byte_count;
	//u8 sdata[4] = {0,};
	u8 retry = 3;
#endif
	uint addr;
	u16 reg;
	int ret = 0;
    int partial_bytesize;
    int partial_delay;
    int new_offsetcount;
    size_t new_partialsize;
    size_t new_totalsize;
	char fwpath[256] = {0};
	char filename[256] = {0};
    const struct firmware *fw = NULL;

	if(sscanf(buf, "%d,%d,0x%hx,%255s", &partial_bytesize, &partial_delay, &reg, filename) <= 3)
    {
        printk(KERN_ERR "Usage> echo \"PartialSIZE(byte),PartialDELAY(us),0xADDR16,ImageName.img\" > mcu_partialbulkwrite");
		return count;
    }

    if(strlen(filename) > 256) {
        printk(KERN_ERR "filename is too long\n");
        return count;
    }

	sprintf(fwpath, "mcu/%s", filename);
    if(partial_bytesize < 1) partial_bytesize = 256;
	printk(KERN_ERR "[ice40] mcu_i2c2_partialbulkwrite, partialsize[%dbyte] , address[0x%hx], filename[%s]\n", partial_bytesize, reg, filename);
    if (fwpath == NULL) {
        printk(KERN_ERR "[ice40] ERR: mcu_i2c2's fwpath is null!\n");
        return count;
    }

    ret = request_firmware(&fw, fwpath, dev);
    if (ret < 0) {
		printk(KERN_ERR "[ice40] ERR: fail to mcu_i2c2's request_firmware fwpath!\n");
		release_firmware(fw);
		return count;
    }
    else  printk(KERN_ERR "[ice40] mcu_i2c2's request_firmware is done!\n");

	addr = reg;
#if 1
	data = kmalloc(fw->size + 2, GFP_KERNEL);
	if(!data) {
		printk(KERN_ERR "[ice40] %s kmalloc error\n", __func__);
		release_firmware(fw);
		return count;
	}
	data[0] = (addr >> 8) & 0xff;
	data[1] = addr & 0xff;
	memcpy(&data[2], fw->data, fw->size);
#endif

    //new_offsetcount = 0;
#if 1
    new_offsetcount = 2;
#endif
    new_partialsize = partial_bytesize * sizeof(u8);
    new_totalsize = fw->size;
#if 1
	/*sdata[0] = (addr >> 8) & 0xff;
	sdata[1] = addr & 0xff;
	sdata[2] = new_totalsize / partial_bytesize;
	if(new_totalsize % partial_bytesize != 0) {
		sdata[2]++;
		sdata[3]++;
	}
	printk(KERN_ERR "[ice40] sdata[2], %d\n", sdata[2]);
	while(((ret = i2c_master_send(ice40->client_mcu, sdata, sizeof(sdata))) < 0) && retry > 0) {
		if(ret < 0){
			printk(KERN_ERR "[ice40] Failed to write mcu_i2c2_bulkwrite sdata, %d retry %d\n", ret, retry);
		}
		retry--;
	}
	if(partial_delay >= 1) udelay(partial_delay);*/

	partial_byte_count = kzalloc(new_partialsize + 2, GFP_KERNEL);
	if(!partial_byte_count) {
		printk(KERN_ERR "[ice40] %s kmalloc error\n", __func__);
		release_firmware(fw);
		kfree(data);
		return count;
	}
	partial_byte_count[0] = (addr >> 8) & 0xff;
	partial_byte_count[1] = addr & 0xff;
	partial_byte_count[2] = new_totalsize / partial_bytesize;
	if(new_totalsize % partial_bytesize != 0) {
		partial_byte_count[2]++;
		partial_byte_count[3]++;
	}
	printk(KERN_ERR "[ice40] partial_byte_count, %d\n", partial_byte_count[2]);
	while(((ret = i2c_master_send(ice40->client_mcu, partial_byte_count, sizeof(partial_byte_count))) < 0) && retry > 0) {
		if(ret < 0){
			printk(KERN_ERR "[ice40] Failed to write mcu_i2c2_bulkwrite partial_byte_count, %d retry %d\n", ret, retry);
		}
		retry--;
	}
	if(partial_delay >= 1) udelay(partial_delay);
#endif

    while(1)
    {
        if(new_totalsize < new_partialsize) new_partialsize = new_totalsize;
#if 1
		data[new_offsetcount-2] = (addr >> 8) & 0xff;
		data[new_offsetcount-1] = addr & 0xff;
		retry = 3;
#endif
        printk(KERN_ERR "[ice40] partial write [offset:0x%x, size:0x%x]\n", (unsigned int)(new_offsetcount * sizeof(u8)), (unsigned int)new_partialsize);
        //ret = regmap_bulk_write(regmap, addr, fw->data + new_offsetcount * sizeof(u8), new_partialsize);
#if 1
        //ret = i2c_master_send(ice40->client_mcu, &data[new_offsetcount-2] , new_partialsize + 2);
        while(((ret = i2c_master_send(ice40->client_mcu, &data[new_offsetcount-2] , new_partialsize + 2)) < 0) && retry > 0) {
			if(ret < 0){
				printk(KERN_ERR "[ice40] Failed to write mcu_i2c2_bulkwrite, %d retry %d\n", ret, retry);
			}
			retry--;
		}
#endif

        new_offsetcount = new_offsetcount + partial_bytesize;
		new_totalsize = new_totalsize - new_partialsize;
        if(new_totalsize <= 0) break;
		if(partial_delay >= 1) udelay(partial_delay);
    }

	if(ret < 0){
		printk(KERN_ERR "[ice40] Failed to write mcu_i2c2_bulkwrite, %d\n", ret);
	}
    else printk(KERN_ERR "[ice40] mcu_i2c2_bulkwrite done\n");

    release_firmware(fw);
#if 1
	kfree(partial_byte_count);
	kfree(data);
#endif

	return count;
}

int mcu_firmware_partialbulkwrite(struct ice40 *ice40, int select)
{
	u8 *partial_byte_count;
	u8 retry = 3;
	uint addr;
	u16 reg = 0;
	int ret = 0;
	int partial_bytesize = 5120;
	int partial_delay = 100000;
	int new_offsetcount;
	size_t new_partialsize;
	size_t new_totalsize;
	const struct firmware *fw = NULL;

	printk(KERN_ERR "[MCU_FW] mcu_firmware_partialbulkwrite, select[%d]\n", select);

	if (select == MCU_UPDATE) {
		printk(KERN_ERR "[MCU_FW] MCU_UPDATE Start\n");
		reg = I2C_W_MCU_FW_UPDATE;
		ret = request_firmware(&fw, MCU_FW_PATH, ice40->dev);
	} else if (select == LATTICE_UPDATE) {
		printk(KERN_ERR "[MCU_FW] LATTICE_UPDATE Start\n");
		reg = I2C_W_LATTICE_FW_UPDATE;
		ret = request_firmware(&fw, LATTICE_FW_PATH, ice40->dev);
	}

	if (ret < 0) {
		printk(KERN_ERR "[MCU_FW] ERR: fail to mcu_firmware request_firmware\n");
		release_firmware(fw);
		return ret;
	} else
		printk(KERN_ERR "[MCU_FW] mcu_firmware request_firmware is done\n");

	addr = reg;
	new_offsetcount = 0;
	new_partialsize = partial_bytesize * sizeof(u8);
	new_totalsize = fw->size;
	partial_byte_count = kzalloc(new_partialsize + 2, GFP_KERNEL);
	if(!partial_byte_count) {
		printk(KERN_ERR "[MCU_FW] kzalloc error\n");
		release_firmware(fw);
		return -ENOMEM;
	}
	partial_byte_count[0] = (addr >> 8) & 0xff;
	partial_byte_count[1] = addr & 0xff;
	partial_byte_count[2] = new_totalsize / partial_bytesize;
	if(new_totalsize % partial_bytesize != 0) {
		partial_byte_count[2]++;
		partial_byte_count[3]++;
	}
	printk(KERN_ERR "[MCU_FW] partial_byte_count, %d\n", partial_byte_count[2]);
	while(((ret = i2c_master_send(ice40->client_mcu, partial_byte_count, sizeof(partial_byte_count))) < 0) && retry > 0) {
		if(ret < 0){
			printk(KERN_ERR "[MCU_FW] Failed to write mcu_i2c2_bulkwrite partial_byte_count, %d retry %d\n", ret, retry);
			if( ret != -ENOTCONN && ret != -EACCES )
				goto exit;
		}
		retry--;
	}
	if(partial_delay >= 1)
		__udelay(partial_delay);

	while(1)
	{
		if(new_totalsize < new_partialsize) new_partialsize = new_totalsize;

		memcpy(&partial_byte_count[2], &fw->data[new_offsetcount], new_partialsize);
		retry = 3;

		printk(KERN_ERR "[MCU_FW] partial write [offset:0x%x, size:0x%x]\n", (unsigned int)(new_offsetcount * sizeof(u8)), (unsigned int)new_partialsize);

		while(((ret = i2c_master_send(ice40->client_mcu, partial_byte_count, new_partialsize + 2)) < 0) && retry > 0) {
			if(ret < 0){
				printk(KERN_ERR "[MCU_FW] Failed to write mcu_i2c2_bulkwrite, %d retry %d\n", ret, retry);
				if( ret != -ENOTCONN && ret != -EACCES )
					goto exit;
			}
			retry--;
		}
		new_offsetcount = new_offsetcount + partial_bytesize;
		new_totalsize = new_totalsize - new_partialsize;
		if(new_totalsize <= 0) break;
		if(partial_delay >= 1) __udelay(partial_delay);
	}

	if(ret < 0)
		printk(KERN_ERR "[MCU_FW] Failed to write mcu_i2c2_bulkwrite, %d\n", ret);
	else
		printk(KERN_ERR "[MCU_FW] mcu_i2c2_bulkwrite done\n");

exit:
	release_firmware(fw);
	kfree(partial_byte_count);

	return ret;
}


/*!
 *****************************************************************************
 *  \brief initialize ice40M regmap
 *
 *  initializes ice40S register map
 *
 *  \return xx: returns 0 on success
 *
 *****************************************************************************
 */
static int mcu_i2c2_initialize_regmap(struct ice40 *ice40)
{
/*
	int ret = 0;

	if(ice40->regmap_mcu == NULL) {
		ice40->regmap_mcu = devm_regmap_init_i2c(ice40->client_mcu,
			&mcu_i2c2_regmap_config);
		if (IS_ERR(ice40->regmap_mcu)) {
			ret = PTR_ERR(ice40->regmap_mcu);
			dev_err(ice40->dev, "%s: regmap initialization failed: %d\n",
				__func__, ret);
			return ret;
		}
		regmap = ice40->regmap_mcu;
		return ret;
	} else {
		return ret;
	}
*/
	return 0;
}

/* ======================================================================== */
/* CHAN's SYSFS FUNCTION PROTOTYPES */
static DEVICE_ATTR(mcu_bulkwrite, S_IRUGO | S_IWUSR, NULL, store_mcu_i2c2_bulkwrite);
static DEVICE_ATTR(mcu_partialbulkwrite, S_IRUGO | S_IWUSR, NULL, store_mcu_i2c2_partialbulkwrite);

static struct attribute *mcu_i2c2_fs_attrs[] = {
	&dev_attr_mcu_bulkwrite.attr,
	&dev_attr_mcu_partialbulkwrite.attr,
	NULL,
};

static struct attribute_group mcu_i2c2_fs_attrs_group = {
	.attrs = mcu_i2c2_fs_attrs,
};

/*!
 *****************************************************************************
 *  \brief initialize interfaces related to ice40S
 *
 *  initializes sysfs interface related to ice40S
 *
 *  \return xx: returns 0 on success
 *
 *****************************************************************************
 */
static int mcu_i2c2_initialize_sysfs(struct ice40 *ice40)
{
	int ret;

	ret = device_create_file(ice40->dev, &mcu_i2c2_register);
	if (ret) {
		dev_err(ice40->dev, "%s: could not create sysfs attribute file\n",
				__func__);
		device_remove_file(ice40->dev, &mcu_i2c2_register);
	}

	// CHAN's regmap
	ret = sysfs_create_group(&ice40->client_mcu->dev.kobj, &mcu_i2c2_fs_attrs_group);
	if (ret < 0)
		printk(KERN_ERR "ice40 sysfs register failed\n");

	return ret;
}

/* ========================================================================== */
/* GLOBAL functions */

/*!
 *****************************************************************************
 *  \brief initialize ice40S regmap, sysfs files and IRQ
 *
 *  initializes ice40S regmap, sysfs files and IRQ
 *
 *  \return xx: returns 0 on success
 *
 *****************************************************************************
 */
//static int mcu_i2c2_init = 0;
int mcu_i2c2_initialize(struct ice40 *ice40)
{
	int ret;

	//if(mcu_i2c2_init == 1) return 0;

	printk(KERN_ERR "mcu_i2c2_initialize_regmap is called\n");
	ret = mcu_i2c2_initialize_regmap(ice40);
	if (ret < 0)
		return ret;

	printk(KERN_ERR "mcu_i2c2_initialize_sysfs is called\n");
	ret = mcu_i2c2_initialize_sysfs(ice40);
	if (ret < 0)
		return ret;

	//mcu_i2c2_init = 1;
    printk(KERN_ERR "mcu_i2c2_initialize is OK!\n");
	return ret;
}

/*!
 *****************************************************************************
 *  \brief deinitialize sysfs interfaces related to ice40S
 *
 *  deinitializes sysfs interface related to ice40S
 *
 *  \return xx: returns 0 on success
 *
 *****************************************************************************
 */
int mcu_i2c2_deinitialize_sysfs(struct ice40 *ice40)
{
	device_remove_file(ice40->dev, &mcu_i2c2_register);
	sysfs_remove_group(&ice40->client_mcu->dev.kobj, &mcu_i2c2_fs_attrs_group);

	return 0;
}

/*!
 *****************************************************************************
 *  \brief deinitialize ice40S
 *
 *  deinitializes ice40S
 *
 *  \return xx: returns 0 on success
 *
 *****************************************************************************
 */
int mcu_i2c2_deinitialize(struct ice40 *ice40)
{
	int ret;

    ret = 0;
	return ret;
}

/* ========================================================================== */
/* LOCAL SYSFS functions */

/*!
 *****************************************************************************
 *  \brief Reads register from ice40S
 *
 *  Access through sysfs file "reg_access"
 *  Prior to reading reg_access, "R:<REG_ADDR>,<LENGTH_READ>" must be
 *  written to reg_access
 *
 *  \return xx: returns number of bytes read
 *
 *****************************************************************************
 */
static ssize_t mcu_i2c2_register_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ice40 *ice40 = dev_get_drvdata(dev);
	int ret;
	int i;

	dev_dbg(ice40->dev, "%s()\n", __func__);

/*
	if (IS_ERR(ice40->regmap_mcu)) {
		dev_err(ice40->dev, "%s: regmap not initialized\n",
				__func__);
		return -EIO;
	}

	ret = regmap_bulk_read(ice40->regmap_mcu, ice40->reg_rd_addr_mcu, buf,
					 ice40->reg_rd_len_mcu);
*/
	buf[0] = (ice40->reg_rd_addr_mcu >> 8) & 0xff;
	buf[1] = ice40->reg_rd_addr_mcu & 0xff;

	//ret = i2c_master_recv(ice40->client_mcu, buf, ice40->reg_rd_len_mcu );
	ret = ice40_mcu_reg_read(ice40, ice40->reg_rd_addr_mcu, buf, ice40->reg_rd_len_mcu);

	if (ret < 0) {
		dev_err(ice40->dev, "%s: register read failed with err %d\n",
				__func__, ret);

		return ret;
	}
	for( i = 0; i < ice40->reg_rd_len_mcu; i++)
		dev_dbg(ice40->dev, "%s: register read - 0x%hx:0x%hhx, length: %d\n",
			 __func__, ice40->reg_rd_addr_mcu,
			 buf[i], ice40->reg_rd_len_mcu);
	return ice40->reg_rd_len_mcu;
}

/*!
 *****************************************************************************
 *  \brief Writes register from ice40S
 *
 *  Access through sysfs file "reg_access"
 *  Use "W:<REG_ADDR>,<VALUE>" to write register
 *
 *  \return xx: returns number of bytes written
 *
 *****************************************************************************
 */
static ssize_t mcu_i2c2_register_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ice40 *ice40 = dev_get_drvdata(dev);
	int num_extracted = 0;
	int ret;
	u16 reg = 0x00;
	u8 wr_val = 0x00;
	u8 rd_length = 0;
	u8 data[3] = {0,};

	dev_dbg(ice40->dev, "%s()\n", __func__);

/*
	if (IS_ERR(ice40->regmap_mcu)) {
		dev_err(ice40->dev, "%s: regmap not initialized\n",
				__func__);
		return -EIO;
	}
*/
	num_extracted = sscanf(buf, "R:0x%hx,0x%hhx", &reg, &rd_length);
	if (num_extracted == 2) { /* read operation triggered */
		if (rd_length <= 0) {
			dev_err(ice40->dev, "%s: read length 0 not supported\n",
					__func__);
			return -EINVAL;
		}
		ice40->reg_rd_addr_mcu = reg;
		ice40->reg_rd_len_mcu = rd_length;
		dev_dbg(ice40->dev, "%s: register read: %02x\n",
				 __func__, reg);
	} else {
		num_extracted = sscanf(buf, "W:0x%hx,0x%hhx", &reg, &wr_val);
		if (num_extracted == 2) { /* write operation triggered */
			//ret = regmap_write(ice40->regmap_mcu, reg, wr_val);
			data[0] = (reg >> 8) & 0xff;
			data[1] = reg & 0xff;
			data[2] = wr_val;
			ret = i2c_master_send(ice40->client_mcu, data, sizeof(data));
			if (ret < 0) {
				dev_err(ice40->dev,
						"%s: register write failed with err %d\n",
						__func__, ret);
				return ret;
			}
			dev_dbg(ice40->dev,
					 "%s: register write: %02x = %02x\n",
					 __func__, reg, wr_val);
		} else { /* no valid arguments found */
			dev_err(ice40->dev,
					"%s: could not extract valid arguments\n",
					__func__);
			return -EINVAL;
		}
	}
	return size;
}

MODULE_AUTHOR("LG Electronics, Inc");
MODULE_DESCRIPTION("MCU_I2C2 driver");
MODULE_LICENSE("GPL v2");
