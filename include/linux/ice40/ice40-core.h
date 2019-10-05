/*
 * Core Driver for ice40 Accessory Communication Interface
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


#ifndef ICE40_CORE_H
#define ICE40_CORE_H

/* ========================================================================== */
/* Includes and Defines */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#ifdef OUT_OF_TREE
#include "ice40.h"
#include "mcui2c.h"
#else
#include <linux/ice40/ice40.h>
#include <linux/ice40/mcui2c.h>
#endif
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/completion.h>
#include <linux/workqueue.h>

/* define names for sysfs files */
/* reg_access files end-up in different folders, so no name-conflict occurs */
#define FILE_REG_ACCESS_MASTER  reg_access_m
#define FILE_REG_ACCESS_SLAVE   reg_access_s
#define FILE_REG_ACCESS_MCU reg_access_u
#define FILE_CLOCK              clock

/* ======================================================================== */
/* define for regmap range usage in older kernel versions */
#define regmap_reg_range(low, high) { .range_min = low, .range_max = high, }

#define ICE40_STATUS_POLL_US 5000000
#define ICE40_GET_MCU_LOG_COUNT 5
#define ICE40_RECOVREY_MAX_COUNT 4
/* ======================================================================== */

#define I2C_R_MCU_LOG               0x0003
#define I2C_R_MCU_VER               0x0004
#define I2C_R_LATTICE_VER           0x0005
#define I2C_W_SUB_DEVICE_POWER_OFF  0x000B
#define I2C_R_DISPLAY_ID            0x000C

/*! global driver data structure */
struct ice40 {
    /*! device struct */
    struct device *dev;
    /*! register-map for master */
    struct regmap *regmap_master;
    /*! register-map for mcu */
    struct regmap *regmap_mcu;
    /*! I2C client for master */
    struct i2c_client *client_master;
    /*! I2C client for mcu */
    struct i2c_client *client_mcu;
    /*! read length parameter for master register read through sysfs files */
    u8 reg_rd_len_master;
    /*! read address parameter for master register read through sysfs files */
    u8 reg_rd_addr_master;
    /*! read length parameter for slave register read through sysfs files */
    u8 reg_rd_len_mcu;
    /*! read address parameter for slave register read through sysfs files */
    u8 reg_rd_addr_mcu;
    /*! global driver ice40 enable state */
    bool ice40_on;
    struct delayed_work status_monitor;
    int get_log_count;
    struct mutex mcu_i2c_lock;
    int recovery_count;
    u8 in_recovery;
};

/* ice40 core function prototypes */
int ice40_disable(struct ice40 *ice40);
int ice40_enable(struct ice40 *ice40);
int ice40_master_reg_write(struct ice40 *ice40, uint addr, uint val);
int ice40_master_reg_read(struct ice40 *ice40, uint addr, uint *data);
int ice40_master_bulk_read(struct ice40 *ice40, uint addr, char *data, int len);
int ice40_mcu_reg_write(struct ice40 *ice40, uint addr, uint val);
int ice40_mcu_reg_read(struct ice40 *ice40, uint addr, char *data, int len);

/* ice40m function prototypes */
int ice40m_initialize(struct ice40 *ice40);
int ice40m_deinitialize(struct ice40 *ice40);
int ice40m_deinitialize_sysfs(struct ice40 *ice40);


/* ice40s function prototypes */
int ice40s_initialize(struct ice40 *ice40);
int ice40s_deinitialize(struct ice40 *ice40);
int ice40s_deinitialize_sysfs(struct ice40 *ice40);

/* mcu_i2c2 function prototypes */
int mcu_i2c2_initialize(struct ice40 *ice40);
int mcu_i2c2_deinitialize(struct ice40 *ice40);
int mcu_i2c2_deinitialize_sysfs(struct ice40 *ice40);
int mcu_i2c2_write_backlight(struct ice40 *ice40, uint val);

extern int mcu_firmware_partialbulkwrite(struct ice40 *ice40, int select);
#endif /* ICE40_CORE_H */
