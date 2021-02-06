#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3706_MH2)		+= touch_s3706.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3706_MH2)		+= touch_s3706_fw_upgrade.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3706_MH2)		+= touch_s3706_rmi_dev.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3706_MH2)		+= touch_s3706_prd.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3706_MH2)		+= touch_s3706_f54_test.o
