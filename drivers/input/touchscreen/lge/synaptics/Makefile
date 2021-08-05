#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3618)		+= s3618/
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3706)		+= s3706/
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3706_MH2)	+= s3706_mh2/

obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3707)		+= touch_s3707.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3707)		+= touch_s3707_fw_upgrade.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3707)		+= touch_s3707_rmi_dev.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3707)		+= touch_s3707_prd.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3707)		+= touch_s3707_f54_test.o
