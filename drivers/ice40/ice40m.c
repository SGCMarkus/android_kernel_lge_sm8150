/*
 * ice40 Driver functions dedicated to ice40M (master)
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

/* ======================================================================== */
/* LOCAL FUNCTION PROTOTYPES */
/* sysfs functions */
static ssize_t ice40m_register_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t ice40m_register_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size);

/* ======================================================================== */
/* DEVICE ATTRIBUTES */
/* RW */
struct device_attribute ice40m_register =
	__ATTR(FILE_REG_ACCESS_MASTER,
			0660,
			ice40m_register_show,
			ice40m_register_store);

/* ========================================================================== */
/* Setup of regmap for ice40M */
static const struct regmap_range ice40m_volatile_ranges[] = {
	regmap_reg_range(ICE40_REG_START, ICE40_REG_END),
};

static const struct regmap_range ice40m_writeable_ranges[] = {
	regmap_reg_range(ICE40_REG_START, ICE40_REG_END),
};

static const struct regmap_range ice40m_readable_ranges[] = {
	regmap_reg_range(ICE40_REG_START, ICE40_REG_END),
};

static const struct regmap_access_table ice40m_volatile_table = {
	.yes_ranges = ice40m_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(ice40m_volatile_ranges),
};

static const struct regmap_access_table ice40m_writeable_table = {
	.yes_ranges = ice40m_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ice40m_writeable_ranges),
};

static const struct regmap_access_table ice40m_readable_table = {
	.yes_ranges = ice40m_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ice40m_readable_ranges),
};

static const struct regmap_config ice40m_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &ice40m_volatile_table,
	.wr_table = &ice40m_writeable_table,
	.rd_table = &ice40m_readable_table,
	.max_register = ICE40_REG_END,
	/* FIXME: added to suppress errors when reading test regs */
	.cache_type = REGCACHE_RBTREE,
};

/*!
 *****************************************************************************
 *  \brief initialize ice40M regmap
 *
 *  initializes ice40M register map
 *
 *  \return xx: returns 0 on success
 *
 *****************************************************************************
 */
static int ice40m_initialize_regmap(struct ice40 *ice40)
{
	int ret = 0;

	if (ice40->regmap_master == NULL) {
		ice40->regmap_master = devm_regmap_init_i2c(ice40->client_master,
			&ice40m_regmap_config);
		if (IS_ERR(ice40->regmap_master)) {
			ret = PTR_ERR(ice40->regmap_master);
			dev_err(ice40->dev, "%s: regmap initialization failed: %d\n",
				__func__, ret);
		}
		return ret;
	} else {
		return ret;
	}
}

/*!
 *****************************************************************************
 *  \brief initialize interfaces related to ice40M
 *
 *  initializes sysfs interface related to ice40M
 *
 *  \return xx: returns 0 on success
 *
 *****************************************************************************
 */
static int ice40m_initialize_sysfs(struct ice40 *ice40)
{
	int ret;

	ret = device_create_file(ice40->dev, &ice40m_register);
	if (ret) {
		dev_err(ice40->dev, "%s: could not create sysfs attribute file\n",
				__func__);
		device_remove_file(ice40->dev, &ice40m_register);
	}

	return ret;
}

/* ========================================================================== */
/* GLOBAL functions */

/*!
 *****************************************************************************
 *  \brief initialize ice40M regmap, sysfs files and IRQ
 *
 *  initializes ice40M regmap, sysfs files and IRQ
 *
 *  \return xx: returns 0 on success
 *
 *****************************************************************************
 */

//static int ice40m_init = 0;
int ice40m_initialize(struct ice40 *ice40)
{
	int ret;

    //if(ice40m_init == 1) return 0;

	printk("ice40m_initialize_regmap is called\n");
	ret = ice40m_initialize_regmap(ice40);
	if (ret < 0) {
		printk("ice40m_initialize_regmap failed\n");
		return ret;
	}

	printk("ice40m_initialize_sysfs is called\n");
	ret = ice40m_initialize_sysfs(ice40);
	if (ret < 0) {
		printk("ice40m_initialize_sysfs failed\n");
		return ret;
	}

    //ice40m_init = 1;
    printk("ice40m_initialize is OK!\n");
	return ret;
}

/*!
 *****************************************************************************
 *  \brief deinitialize sysfs interfaces related to ice40M
 *
 *  deinitializes sysfs interface related to ice40M
 *
 *  \return xx: returns 0 on success
 *
 *****************************************************************************
 */
int ice40m_deinitialize_sysfs(struct ice40 *ice40)
{
	device_remove_file(ice40->dev, &ice40m_register);
	return 0;
}

/*!
 *****************************************************************************
 *  \brief deinitialize ice40M
 *
 *  deinitializes ice40M
 *
 *  \return xx: returns 0 on success
 *
 *****************************************************************************
 */
int ice40m_deinitialize(struct ice40 *ice40)
{
	int ret;

    ret = 0;
	return ret;
}

/* ========================================================================== */
/* LOCAL SYSFS functions */
/*!
 *****************************************************************************
 *  \brief Reads register from ice40M
 *
 *  Access through sysfs file "reg_access"
 *  Prior to reading reg_access, "R:<REG_ADDR>,<LENGTH_READ>" must be
 *  written to reg_access
 *
 *  \return xx: returns number of bytes read
 *
 *****************************************************************************
 */
static ssize_t ice40m_register_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ice40 *ice40 = dev_get_drvdata(dev);
	int ret;
	int i;

	dev_dbg(ice40->dev, "%s()\n", __func__);

	if (IS_ERR(ice40->regmap_master)) {
		dev_err(ice40->dev, "%s: regmap not initialized\n",
				__func__);
		return -EIO;
	}

	ret = regmap_bulk_read(ice40->regmap_master, ice40->reg_rd_addr_master, buf,
					 ice40->reg_rd_len_master);
	if (ret < 0) {
		dev_err(ice40->dev, "%s: register read failed with err %d\n",
				__func__, ret);

		return ret;
	}
	for( i = 0; i < ice40->reg_rd_len_master; i++)
		dev_dbg(ice40->dev, "%s: register read - 0x%hx:0x%hhx, length: %d\n",
			 __func__, ice40->reg_rd_addr_master,
			 buf[i], ice40->reg_rd_len_master);

	return ice40->reg_rd_len_master;
}

/*!
 *****************************************************************************
 *  \brief Writes register from ice40M
 *
 *  Access through sysfs file "reg_access"
 *  Use "W:<REG_ADDR>,<VALUE>" to write register
 *
 *  \return xx: returns number of bytes written
 *
 *****************************************************************************
 */
static ssize_t ice40m_register_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ice40 *ice40 = dev_get_drvdata(dev);
	int num_extracted = 0;
	int ret;
	u8 reg = 0x00;
	u8 wr_val = 0x00;
	u8 rd_length = 0;

	dev_dbg(ice40->dev, "%s()\n", __func__);

	if (IS_ERR(ice40->regmap_master)) {
		dev_err(ice40->dev, "%s: regmap not initialized\n",
				__func__);
		return -EIO;
	}

	num_extracted = sscanf(buf, "R:0x%hhx,0x%hhx", &reg, &rd_length);
	if (num_extracted == 2) { /* read operation triggered */
		if (rd_length <= 0) {
			dev_err(ice40->dev, "%s: read length 0 not supported\n",
					__func__);
			return -EINVAL;
		}
		ice40->reg_rd_addr_master = reg;
		ice40->reg_rd_len_master = rd_length;
		dev_dbg(ice40->dev, "%s: register read: %02x\n",
				 __func__, reg);
	} else {
		num_extracted = sscanf(buf, "W:0x%hhx,0x%hhx", &reg, &wr_val);
		if (num_extracted == 2) { /* write operation triggered */
			ret = regmap_write(ice40->regmap_master, reg, wr_val);
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
MODULE_DESCRIPTION("ICE40M driver");
MODULE_LICENSE("GPL v2");
