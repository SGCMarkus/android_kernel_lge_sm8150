#
# Makefile for the fingerprint drivers.
#

# Each configuration option enables a list of files.
ccflags-y += -Idrivers/misc/

ifeq ($(CONFIG_FINGERPRINT_GOODIX_GF3208),y)
obj-$(CONFIG_FINGERPRINT_GOODIX_GF3208) += gf_spi.o platform.o netlink.o gf_common.o
endif