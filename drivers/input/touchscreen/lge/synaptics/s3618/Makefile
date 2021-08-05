#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3618)		+= touch_s3618.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3618)		+= touch_s3618_reflash.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3618)		+= touch_s3618_testing.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3618)		+= touch_s3618_device.o
#obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3618)		+= touch_s3618_rmi_dev.o
#obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3618)		+= touch_s3618_prd.o
#obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3618)		+= touch_s3618_f54_test.o
