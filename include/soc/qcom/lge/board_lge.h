#ifndef __ASM_ARCH_MSM_BOARD_LGE_H
#define __ASM_ARCH_MSM_BOARD_LGE_H

#ifdef CONFIG_LGE_USB_FACTORY
typedef enum {
	LGE_BOOT_MODE_NORMAL = 0,
	LGE_BOOT_MODE_CHARGER,
	LGE_BOOT_MODE_CHARGERLOGO,
	LGE_BOOT_MODE_QEM_56K,
	LGE_BOOT_MODE_QEM_130K,
	LGE_BOOT_MODE_QEM_910K,
	LGE_BOOT_MODE_PIF_56K,
	LGE_BOOT_MODE_PIF_130K,
	LGE_BOOT_MODE_PIF_910K,
	LGE_BOOT_MODE_MINIOS    /* LGE_UPDATE for MINIOS2.0 */
} lge_boot_mode_t;

typedef enum {
	LGE_FACTORY_CABLE_NONE = 0,
	LGE_FACTORY_CABLE_56K,
	LGE_FACTORY_CABLE_130K,
	LGE_FACTORY_CABLE_910K,
} lge_factory_cable_t;

typedef enum {
	LGE_LAF_MODE_NORMAL = 0,
	LGE_LAF_MODE_LAF,
} lge_laf_mode_t;

typedef enum {
	LT_CABLE_56K = 6,
	LT_CABLE_130K,
	USB_CABLE_400MA,
	USB_CABLE_DTC_500MA,
	ABNORMAL_USB_CABLE_400MA,
	LT_CABLE_910K,
	NONE_INIT_CABLE
} cable_boot_type;

cable_boot_type lge_get_boot_cable(void);
lge_boot_mode_t lge_get_boot_mode(void);
bool lge_get_qemmode_boot(void);
bool lge_get_factory_boot(void);
lge_factory_cable_t lge_get_factory_cable(void);
bool lge_get_android_dlcomplete(void);
lge_laf_mode_t lge_get_laf_mode(void);
#endif
#ifdef CONFIG_LGE_TOUCH_CORE
typedef enum {
	TOUCH_NORMAL = 0,
	SYNAPTICS = 8,
	SIW = 9,
} lge_touch_id_t;
lge_touch_id_t lge_get_touch_id(void);
#endif
enum hw_rev_no {
	HW_REV_EVB1 = 0,
	HW_REV_EVB2,
	HW_REV_EVB3,
	HW_REV_0,
	HW_REV_0_1,
	HW_REV_0_2,
	HW_REV_0_3,
	HW_REV_A,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_E,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_1_2,
	HW_REV_1_3,
	HW_REV_MAX
};

char *lge_get_board_revision(void);
enum hw_rev_no lge_get_board_rev_no(void);

enum hw_subrev_no {
	HW_SUB_REV_0,
	HW_SUB_REV_1,
	HW_SUB_REV_2,
	HW_SUB_REV_3,
	HW_SUB_REV_4,
	HW_SUB_REV_5,
	HW_SUB_REV_6,
	HW_SUB_REV_7,
	HW_SUB_REV_8,
	HW_SUB_REV_9,
	HW_SUB_REV_MAX
};

char *lge_get_board_subrevision(void);
enum hw_subrev_no lge_get_board_subrev_no(void);

extern int lge_get_bootreason(void);

#ifdef CONFIG_MACH_LGE
bool lge_check_recoveryboot(void);
#endif

enum lge_hydra_name lge_get_hydra_name(void);
enum lge_hydra_name {
	HYDRA_ALPHA,
	HYDRA_PRIME,
	HYDRA_PLUS,
	HYDRA_PLUS1,
	HYDRA_PLUS2,
	HYDRA_SIGNATURE,
	HYDRA_NONE
};

#ifdef CONFIG_LGE_ONE_BINARY_SKU
enum lge_sku_carrier_type lge_get_sku_carrier(void);

/*
 * this SKU, SUB revision, VARI main and VARI sub should be sync with
 * boot_images/QcomPkg/SDM845Pkg/Library/LGELib/boot_lge_hw_rev.h
 */
/*--------------------------------
       SKU type
--------------------------------*/
enum lge_sku_carrier_type {
  HW_SKU_KR,             // 0
  HW_SKU_KR_LGU,
  HW_SKU_KR_SKT,
  HW_SKU_KR_KT,
  HW_SKU_JP,
  HW_SKU_NA_GSM,         // 5
  HW_SKU_NA_GSM_ATT,
  HW_SKU_NA_GSM_TMUS,
  HW_SKU_NA_GSM_RESERVED1,
  HW_SKU_NA_GSM_RESERVED2,
  HW_SKU_NA_CDMA,        // 10
  HW_SKU_NA_CDMA_VZW,
  HW_SKU_NA_CDMA_SPR,
  HW_SKU_NA_CDMA_RESERVED1,
  HW_SKU_NA,
  HW_SKU_GLOBAL,         // 15
  HW_SKU_GLOBAL_ASIA,
  HW_SKU_GLOBAL_MEA,
  HW_SKU_AU_TEL,
  HW_SKU_GLOBAL_SCA,
  HW_SKU_CN,             // 20
  HW_SKU_MAX             // 21
};

/*
* this enum and string should be sync with ntcode_op_table at
* android/bootable/bootloader/edk2/QcomModulePkg/Library/LGESharedLib/lge_one_binary.c
*/
enum lge_laop_operator_type {
  OP_OPEN_KR,
  OP_SKT_KR,
  OP_KT_KR,
  OP_LGU_KR,
  OP_ATT_US,
  OP_TMO_US,
  OP_MPCS_US,
  OP_USC_US,
  OP_CCA_LRA_ACG_US, // CCA LRA ACG have same NT code
  OP_TRF_US,
  OP_AMZ_US,  // Amazon
  OP_GFI_US,  // GoogleFi
  OP_VZW_POSTPAID,
  OP_VZW_PREPAID,
  OP_COMCAST_US,
  OP_OPEN_US,
  OP_OPEN_CA, // Canada
  OP_SPR_US,
  OP_GLOBAL,  // Global, SCA, Asia, CIS, MEA...
  OP_OPEN_RU,
  OP_INVALID, // Invalid NT Code
  OP_MAX
};

enum lge_sku_carrier_type lge_get_sku_carrier(void);
enum lge_laop_operator_type lge_get_laop_operator(void);
int lge_get_capsensor(void);
#endif

#endif /* __ASM_ARCH_MSM_BOARD_LGE_H */
