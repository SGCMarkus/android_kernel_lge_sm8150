#
## Makefile for the module touchscreen drivers.
#

# Touch Core Driver Common Header Code..!!
#ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

#obj-$(CONFIG_LGE_TOUCH_MODULE_SW49106)	+= touch_module_sw49106.o
#touch_module_sw49106-objs				+= touch_sw49106.o

obj-$(CONFIG_LGE_TOUCH_MODULE_FT3518)	+= touch_module_ft3518.o
touch_module_ft3518-objs				+= touch_ft3518.o

obj-$(CONFIG_LGE_TOUCH_MODULE_S3706)	+= touch_module_s3706.o
touch_module_s3706-objs					+= touch_s3706.o
touch_module_s3706-objs					+= touch_s3706_fw_upgrade.o
touch_module_s3706-objs					+= touch_s3706_prd.o
touch_module_s3706-objs					+= touch_s3706_f54_test.o
touch_module_s3706-objs					+= touch_s3706_rmi_dev.o
