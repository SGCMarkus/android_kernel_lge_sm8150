 /* Copyright (C) 2018 LGE, Inc
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 and
  * only version 2 as published by the Free Software Foundation.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  * GNU General Public License for more details.
  */
#ifndef _LGE_QFUSE_
#define _LGE_QFUSE_

#define RET_OK 0
#define RET_ERR -1

#define FTM_PATH "/dev/block/bootdevice/by-name/ftm"
#define SBL_FUSE_PATH "/vendor/firmware_mnt/image/sdx50m/sbl_fuse.mbn"
#define QFUSE_CHECK_STR "BL_QFPROM_BLOWN"

#define QFPROM_CTRL_BASE        (0x00780000)
#define QFPROM_SEC_HW_KEY       (QFPROM_CTRL_BASE + 0x0508) /* 0x00780508 */
#define QFPROM_HW_KEY_STATUS    (QFPROM_CTRL_BASE + 0x208C) /* 0x0078208C */
/* secondary hw key status flag */
#define SEC_KEY_DERIVATION_BLOWN 0x00000002

typedef enum  {
	SBL_LOAD,
	QFUSE_ALREADY_BLOWNED,
	MAX_STATUS_COUNT
} QFUSE_STATUS;

static const char* qfuse_status_str [] = {"QFPROMmodemnotblown",
						"QFPROMalreadyblown"};

int write_fuse_status(QFUSE_STATUS status);
void check_cp_fused(const char* buf);
bool check_if_sbl_fuse_is_loaded(void);

#endif /* _LGE_QFUSE_ */
