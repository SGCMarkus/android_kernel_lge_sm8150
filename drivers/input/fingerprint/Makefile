#
# Makefile for the fingerprint drivers.
#

# Each configuration option enables a list of files.
ccflags-y += -Idrivers/misc/

ifeq ($(CONFIG_FINGERPRINT_GOODIX_GF3208),y)
obj-$(CONFIG_FINGERPRINT_GOODIX_GF3208) += gf_spi.o platform.o netlink.o gf_common.o
endif

ifeq ($(CONFIG_BBK_ET713),y)
et713-objs := et713_platform.o
obj-$(CONFIG_BBK_ET713) := et713.o
endif