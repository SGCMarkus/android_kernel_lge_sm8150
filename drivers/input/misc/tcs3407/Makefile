#
# Makefile for ams TCS3407 device.
#
ifdef CONFIG_SENSORS_TCS3407

#
# Feature Options
#
ccflags-$(CONFIG_SENSORS_TCS3407)	:= -DCONFIG_AMS_OPTICAL_SENSOR_3407

#ifeq ($(CONFIG_TCS3407_AMS_DEBUG_LOG), y)
ccflags-$(CONFIG_SENSORS_TCS3407)	+= -DCONFIG_AMS_DEBUG_LOG
#ccflags-$(CONFIG_SENSORS_TCS3407)	+= -DCONFIG_AMS_CORE_LOG
#endif

#ifeq ($(CONFIG_TCS3407_AMS_OPTICAL_SENSOR_ALS), y)
ccflags-$(CONFIG_SENSORS_TCS3407)	+= -DCONFIG_ALS_CAL_TARGET=300
ccflags-$(CONFIG_SENSORS_TCS3407)	+= -DCONFIG_AMS_ALS_CRGBW
ccflags-$(CONFIG_SENSORS_TCS3407)	+= -DCONFIG_AMS_OPTICAL_SENSOR_ALS
ccflags-$(CONFIG_SENSORS_TCS3407)	+= -DCONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
ccflags-$(CONFIG_SENSORS_TCS3407)	+= -DCONFIG_AMS_OPTICAL_SENSOR_ALS_RGB
#endif

#ccflags-$(CONFIG_SENSORS_TCS3407)	+= -DALS_AUTOGAIN
#ccflags-$(CONFIG_SENSORS_TCS3407)	+= -DAMS_SW_FLICKER

ccflags-$(CONFIG_SENSORS_TCS3407)	+= -DAMS_BIN_2048_MODE1
#ccflags-$(CONFIG_SENSORS_TCS3407)	+= -DAMS_BIN_2048_MODE2
 

#
# Include Paths
#
ccflags-$(CONFIG_SENSORS_TCS3407)	+= -I$(src)
ccflags-$(CONFIG_SENSORS_TCS3407)	+= -I$(src)/osal_linux_input/include
ccflags-$(CONFIG_SENSORS_TCS3407)	+= -I$(src)/tcs3407_dev_ctrl_block/include
ccflags-$(CONFIG_SENSORS_TCS3407)	+= -I$(src)/dev_com_iface/include
ccflags-$(CONFIG_SENSORS_TCS3407)	+= -I$(src)/algorithm_als/include
ccflags-$(CONFIG_SENSORS_TCS3407)	+= -I$(src)/ccb_als/include
ccflags-$(CONFIG_SENSORS_TCS3407)	+= -I$(src)/ccb_irbeam/include

#
# Object Artifacts
#
obj-$(CONFIG_SENSORS_TCS3407)	+= tcs3407drv.o
obj-y	:= tcs3407_module.o
obj-y	+= osal_linux_input/src/osal_linux_input.o
obj-y	+= osal_linux_input/src/osal_linux_input_als.o
obj-y	+= tcs3407_dev_ctrl_block/src/ams_device_control_block.o
obj-y	+= tcs3407_dev_ctrl_block/src/ams_device_control_block_init.o
obj-y	+= dev_com_iface/src/ams_device_com_interface.o

obj-y	+= ams_fft_2048.o
#obj-y	+= ams_fft.o


#ifeq ($(CONFIG_TCS3407_AMS_OPTICAL_SENSOR_ALS), y)
obj-y	+= algorithm_als/src/ams_als_API_config.o
obj-y	+= algorithm_als/src/ams_als_API_info.o
obj-y	+= algorithm_als/src/ams_als_API_init.o
obj-y	+= algorithm_als/src/ams_als_API_process.o
obj-y	+= algorithm_als/src/ams_als_API_result.o
obj-y	+= algorithm_als/src/ams_als_core.o
obj-y	+= ccb_als/src/core_control_block_als.o
obj-y	+= ccb_als/src/core_control_block_als_init.o
#endif



endif
