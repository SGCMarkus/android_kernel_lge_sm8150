/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include "qti_virtual_sensor.h"

static const struct virtual_sensor_data qti_virtual_sensors[] = {
	{
		.virt_zone_name = "gpu-virt-max-step",
		.num_sensors = 2,
		.sensor_names = {"gpu0-usr",
				"gpu1-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "silv-virt-max-step",
		.num_sensors = 4,
		.sensor_names = {"cpu0-silver-usr",
				"cpu1-silver-usr",
				"cpu2-silver-usr",
				"cpu3-silver-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "gold-virt-max-step",
		.num_sensors = 4,
		.sensor_names = {"cpu0-gold-usr",
				"cpu1-gold-usr",
				"cpu2-gold-usr",
				"cpu3-gold-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "hexa-silv-max-step",
		.num_sensors = 6,
		.sensor_names = {"cpu0-silver-usr",
				"cpu1-silver-usr",
				"cpu2-silver-usr",
				"cpu3-silver-usr",
				"cpu4-silver-usr",
				"cpu5-silver-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "dual-gold-max-step",
		.num_sensors = 2,
		.sensor_names = {"cpu0-gold-usr",
				"cpu1-gold-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "deca-cpu-max-step",
		.num_sensors = 10,
		.sensor_names = {"apc0-cpu0-usr",
				"apc0-cpu1-usr",
				"apc0-cpu2-usr",
				"apc0-cpu3-usr",
				"apc0-l2-usr",
				"apc1-cpu0-usr",
				"apc1-cpu1-usr",
				"apc1-cpu2-usr",
				"apc1-cpu3-usr",
				"apc1-l2-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "apc-0-max-step",
		.num_sensors = 5,
		.sensor_names = {"cpu-0-0-usr",
				"cpu-0-1-usr",
				"cpu-0-2-usr",
				"cpu-0-3-usr",
				"cpuss-0-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "apc-1-max-step",
		.num_sensors = 9,
		.sensor_names = {"cpu-1-0-usr",
				"cpu-1-1-usr",
				"cpu-1-2-usr",
				"cpu-1-3-usr",
				"cpu-1-4-usr",
				"cpu-1-5-usr",
				"cpu-1-6-usr",
				"cpu-1-7-usr",
				"cpuss-1-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "gpuss-max-step",
		.num_sensors = 2,
		.sensor_names = {"gpuss-0-usr",
				"gpuss-1-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "cpuss-max-step",
		.num_sensors = 5,
		.sensor_names = {"cpuss-0-usr",
				"cpuss-1-usr",
				"cpuss-2-usr",
				"cpuss-3-usr",
				"cpuss-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "cpuss0-max-step",
		.num_sensors = 4,
		.sensor_names = {"cpuss-0-usr",
				"cpuss-1-usr",
				"cpuss-2-usr",
				"cpuss-3-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "apc1-max-step",
		.num_sensors = 4,
		.sensor_names = {"cpu-1-0-usr",
				"cpu-1-1-usr",
				"cpu-1-2-usr",
				"cpu-1-3-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "cpu-0-max-step",
		.num_sensors = 7,
		.sensor_names = {"cpu-0-0-usr",
				"cpu-0-1-usr",
				"cpu-0-2-usr",
				"cpu-0-3-usr",
				"cpu-0-4-usr",
				"cpu-0-5-usr",
				"cpuss-0-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "cpu-1-max-step",
		.num_sensors = 5,
		.sensor_names = {"cpu-1-0-usr",
				"cpu-1-1-usr",
				"cpu-1-2-usr",
				"cpu-1-3-usr",
				"cpuss-1-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "hepta-cpu-max-step",
		.num_sensors = 7,
		.sensor_names = {"cpu-1-0-usr",
				"cpu-1-1-usr",
				"cpu-1-2-usr",
				"cpu-1-3-usr",
				"cpuss-0-usr",
				"cpuss-1-usr",
				"cpuss-2-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "quad-gpuss-max-step",
		.num_sensors = 4,
		.sensor_names = {"gpuss-0-usr",
				"gpuss-1-usr",
				"gpuss-2-usr",
				"gpuss-3-usr"},
		.logic = VIRT_MAXIMUM,
	},
	{
		.virt_zone_name = "hexa-cpu-max-step",
		.num_sensors = 6,
		.sensor_names = {"apc1-cpu0-usr",
				"apc1-cpu1-usr",
				"apc1-cpu2-usr",
				"apc1-cpu3-usr",
				"cpuss0-usr",
				"cpuss1-usr"},
		.logic = VIRT_MAXIMUM,
	},
};

#ifdef CONFIG_LGE_PM
#if defined(CONFIG_MACH_SM8150_ALPHA)
static const struct virtual_sensor_data lge_virtual_sensors_alpha[] = {
	/* 0.410*xo + 0.446*skin + 2.64 */
	{
		.virt_zone_name = "vts-virt-therm",
		.num_sensors = 2,
		.sensor_names = {"xo-therm", "skin-therm"},
		.coefficient_ct = 2,
		.coefficients = {410, 446},
		.avg_offset = 2640000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},
};
static const struct virtual_sensor_data lge_virtual_sensors_alpha_usa[] = {
	/* 0.226*xo + 0.633*skin + 2.47 */
	{
		.virt_zone_name = "vts-virt-therm",
		.num_sensors = 2,
		.sensor_names = {"xo-therm", "skin-therm"},
		.coefficient_ct = 2,
		.coefficients = {226, 633},
		.avg_offset = 2470000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},
};
#else
static const struct virtual_sensor_data lge_virtual_sensors[] = {
#if defined(CONFIG_MACH_SM8150_FLASH)
	/* 0.12*xo + 0.70*skin + 6.26 */
	{
		.virt_zone_name = "vts-virt-therm",
		.num_sensors = 2,
		.sensor_names = {"xo-therm", "skin-therm"},
		.coefficient_ct = 2,
		.coefficients = {12, 70},
		.avg_offset = 626000,
		.avg_denominator = 100,
		.logic = VIRT_WEIGHTED_AVG,
	},
	/* 0.12*xo + 0.70*skin + 6.26 */
	/* Temporarily the same as vts-virt-therm
	 * If necessary, you can change the formula */
	{
		.virt_zone_name = "sub6-vts",
		.num_sensors = 2,
		.sensor_names = {"xo-therm", "skin-therm"},
		.coefficient_ct = 2,
		.coefficients = {12, 70},
		.avg_offset = 626000,
		.avg_denominator = 100,
		.logic = VIRT_WEIGHTED_AVG,
	},
	/* -0.56*xo + 1.31*skin + 0.2*mmw0-usr + 1.75*/
	{
		.virt_zone_name = "mmw0-vts",
		.num_sensors = 3,
		.sensor_names = {"xo-therm", "skin-therm", "modem1-mmw0-usr"},
		.coefficient_ct = 3,
		.coefficients = {-56, 131, 20},
		.avg_offset = 175000,
		.avg_denominator = 100,
		.logic = VIRT_WEIGHTED_AVG,
	},
	/* 0.21*xo + 0.37*skin + 0.25*mmw1-usr + 2.6 */
	{
		.virt_zone_name = "mmw1-vts",
		.num_sensors = 3,
		.sensor_names = {"xo-therm", "skin-therm", "modem1-mmw1-usr"},
		.coefficient_ct = 3,
		.coefficients = {21, 37, 25},
		.avg_offset = 260000,
		.avg_denominator = 100,
		.logic = VIRT_WEIGHTED_AVG,
	},
	/* 0.13*modem1_mmw0-usr + 0.52*skin(quiet) + 10.45 : for tracking mmw0 vts */
	{
		.virt_zone_name = "mmw-ap-vts",
		.num_sensors = 2,
		.sensor_names = {"modem1-mmw0-usr", "skin-therm"},
		.coefficient_ct = 2,
		.coefficients = {13, 52},
		.avg_offset = 1045000,
		.avg_denominator = 100,
		.logic = VIRT_WEIGHTED_AVG,
	},
#elif defined(CONFIG_MACH_SM8150_BETA)
	/* 0.074*xo + 0.743*skin + 6.214 */
	{
		.virt_zone_name = "vts-virt-therm",
		.num_sensors = 2,
		.sensor_names = {"xo-therm", "skin-therm"},
		.coefficient_ct = 2,
		.coefficients = {74, 743},
		.avg_offset = 6214000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},
#elif defined(CONFIG_MACH_SM8150_MH2LM)
	/* 0.185*xo + 0.575*skin + 6.15 */
	{
		.virt_zone_name = "ap-vts",
		.num_sensors = 2,
		.sensor_names = {"xo-therm", "skin-therm"},
		.coefficient_ct = 2,
		.coefficients = {185, 575},
		.avg_offset = 6150000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},
	/* 0.371*skin + 0.366*pm8150b_tz + 6.84 */
	{
		.virt_zone_name = "chg-vts",
		.num_sensors = 2,
		.sensor_names = {"skin-therm", "pm8150b_tz"},
		.coefficient_ct = 2,
		.coefficients = {371, 366},
		.avg_offset = 6840000,
		.avg_denominator = 1000,
		.logic = VIRT_WEIGHTED_AVG,
	},
	{
		.virt_zone_name = "vts-virt-therm",
		.num_sensors = 2,
		.sensor_names = {"ap-vts",
				"chg-vts"},
		.logic = VIRT_MAXIMUM,
	},
#if defined(CONFIG_MACH_SM8150_MH2LM_5G)
	{
		.virt_zone_name = "sub6-vts",
		.num_sensors = 2,
		.sensor_names = {"ap-vts",
				"chg-vts"},
		.logic = VIRT_MAXIMUM,
	},
#endif
#else
	{
		.virt_zone_name = "vts-virt-therm",
		.num_sensors = 2,
		.sensor_names = {"xo-therm", "skin-therm"},
		.coefficient_ct = 2,
		.coefficients = {0, 1},
		.avg_offset = 0,
		.avg_denominator = 1,
		.logic = VIRT_WEIGHTED_AVG,
	},
#endif
};
#endif
#endif

#ifdef CONFIG_LGE_PM
extern bool unified_bootmode_region_usa(void);
#endif

int qti_virtual_sensor_register(struct device *dev)
{
	int sens_ct = 0;
	static int idx;
	struct thermal_zone_device *tz;
#ifdef CONFIG_LGE_PM
#ifdef CONFIG_MACH_SM8150_ALPHA
	bool region_usa = unified_bootmode_region_usa();
	dev_err(dev, "unified_bootmode region_usa : %s\n",
		region_usa ? "true" : "false");
#else
	int vts_cnt = 0;
	static int vts_idx;
#endif
#endif

	sens_ct = ARRAY_SIZE(qti_virtual_sensors);
	for (; idx < sens_ct; idx++) {
		tz = devm_thermal_of_virtual_sensor_register(dev,
				&qti_virtual_sensors[idx]);
		if (IS_ERR(tz))
			dev_dbg(dev, "sensor:%d register error:%ld\n",
					idx, PTR_ERR(tz));
		else
			dev_dbg(dev, "sensor:%d registered\n", idx);
	}

#ifdef CONFIG_LGE_PM
#if defined(CONFIG_MACH_SM8150_ALPHA)
	if (region_usa) {
		tz = devm_thermal_of_virtual_sensor_register(dev,
				&lge_virtual_sensors_alpha_usa[0]);
	}
	else {
		tz = devm_thermal_of_virtual_sensor_register(dev,
				&lge_virtual_sensors_alpha[0]);
	}
#else
	vts_cnt = ARRAY_SIZE(lge_virtual_sensors);
	for (; vts_idx < vts_cnt; vts_idx++) {
		tz = devm_thermal_of_virtual_sensor_register(dev,
				&lge_virtual_sensors[vts_idx]);
	}
#endif

	if (IS_ERR(tz))
		dev_err(dev, "lge_virtual sensor register error:%ld\n",
			PTR_ERR(tz));
	else
		dev_err(dev, "lge_virtual sensor registered\n", idx);
#endif

	return 0;
}
EXPORT_SYMBOL(qti_virtual_sensor_register);
