ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
# Per Platform
ifeq ($(CONFIG_MACH_SM8150_ALPHA),y)
DTS_PROJECT := sm8150-alpha
endif

ifeq ($(CONFIG_MACH_SM8150_FLASH),y)
DTS_PROJECT := sm8150-flash
endif

ifeq ($(CONFIG_MACH_SM8150_BETA),y)
DTS_PROJECT := sm8150-beta
endif

ifeq ($(CONFIG_MACH_SM8150_MH2LM),y)
DTS_PROJECT := sm8150-mh2lm
endif

# Per Device
ifeq ($(CONFIG_MACH_SM8150_ALPHA_LAO_COM),y)
DTS_TARGET := sm8150-alpha_lao_com
endif

ifeq ($(CONFIG_MACH_SM8150_ALPHA_LAO_LDU),y)
DTS_TARGET := sm8150-alpha_lao_ldu
endif

ifeq ($(CONFIG_MACH_SM8150_FLASH_LAO_COM),y)
DTS_TARGET := sm8150-flash_lao_com
endif

ifeq ($(CONFIG_MACH_SM8150_FLASH_LAO_LDU),y)
DTS_TARGET := sm8150-flash_lao_ldu
endif

ifeq ($(CONFIG_MACH_SM8150_FLASH_DCM_JP),y)
DTS_TARGET := sm8150-flash_dcm_jp
endif

ifeq ($(CONFIG_MACH_SM8150_BETA),y)
DTS_TARGET := sm8150-beta
endif

ifeq ($(CONFIG_MACH_SM8150_BETA_LDU),y)
DTS_TARGET := sm8150-beta_ldu
endif

ifeq ($(CONFIG_MACH_SM8150_MH2LM),y)
ifeq ($(CONFIG_MACH_SM8150_MH2LM_SB_JP),y)
DTS_TARGET := sm8150-mh2lm_sb_jp
else ifeq ($(CONFIG_MACH_SM8150_MH2LM_LDU),y)
DTS_TARGET := sm8150-mh2lm_ldu
else
DTS_TARGET := sm8150-mh2lm
endif
endif

ALL_DTS := $(notdir $(wildcard echo $(srctree)/$(src)/$(DTS_PROJECT)/$(DTS_TARGET)/*.dts))
dtbo-y += $(patsubst %.dts,%.dtbo,$(ALL_DTS))

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtbo,$(DTB_NAMES))
else
DTB_LIST := $(dtbo-y)
endif

always  := $(dtbo-y)

$(obj)/%.dtbo: $(src)/$(DTS_PROJECT)/$(DTS_TARGET)/%.dts FORCE
	$(call if_changed_dep,dtc)
	$(call if_changed,dtbo_verify)

clean-files := *.dtb
endif
