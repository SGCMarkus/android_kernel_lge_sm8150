/**
 * @brief		LC898123F40 Global declaration & prototype declaration
 *
 * @author		Copyright (C) 2016, ON Semiconductor, all right reserved.
 **/

/************************************************/
/*	Command										*/
/************************************************/
#define		CMD_IO_ADR_ACCESS				0xC000
#define		CMD_IO_DAT_ACCESS				0xD000

#define		CMD_RETURN_TO_CENTER			0xF010
	#define		BOTH_SRV_OFF					0x00000000
	#define		XAXS_SRV_ON						0x00000001
	#define		YAXS_SRV_ON						0x00000002
	#define		BOTH_SRV_ON						0x00000003
	#define		ZAXS_SRV_OFF					0x00000004
	#define		ZAXS_SRV_ON						0x00000005
#define		CMD_PAN_TILT					0xF011
	#define		PAN_TILT_OFF					0x00000000
	#define		PAN_TILT_ON						0x00000001
#define		CMD_OIS_ENABLE					0xF012
	#define		OIS_DISABLE						0x00000000
	#define		OIS_ENABLE						0x00000001
	#define		OIS_ENA_NCL						0x00000002
	#define		OIS_ENA_DOF						0x00000004
	#define		OIS_ENA_DCL						0x00000008
#define		CMD_MOVE_STILL_MODE				0xF013
	#define		MOVIE_MODE						0x00000000
	#define		STILL_MODE						0x00000001
	#define		MOVIE_MODE1						0x00000002
	#define		STILL_MODE1						0x00000003
	#define		MOVIE_MODE2						0x00000004
	#define		STILL_MODE2						0x00000005
	#define		MOVIE_MODE3						0x00000006
	#define		STILL_MODE3						0x00000007
#define		CMD_CALIBRATION					0xF014
#define		CMD_CHASE_CONFIRMATION			0xF015
#define		CMD_GYRO_SIG_CONFIRMATION		0xF016
#define		CMD_SSC_ENABLE					0xF01C
	#define		SSC_DISABLE						0x00000000
	#define		SSC_ENABLE						0x00000001
	// Calibration flags
	#define		HALL_CALB_FLG					0x00008000
	#define		HALL_CALB_BIT					0x00FF00FF
	#define		GYRO_GAIN_FLG					0x00004000
	#define		ANGL_CORR_FLG					0x00002000
	#define		OPAF_FST_FLG					0x00001000
	#define		CLAF_CALB_FLG					0x00000800
	#define		HLLN_CALB_FLG					0x00000400
	#define		CROS_TALK_FLG					0x00000200
	#define		ACCL_OFST_FLG					0x00000100

#define		CMD_GYRO_RD_ACCS				0xF01D
#define		CMD_GYRO_WR_ACCS				0xF01E

#define		CMD_READ_STATUS					0xF100
	#define		READ_STATUS_INI					0x01000000


	// Calibration flags
	#define		HALL_CALB_FLG					0x00008000
	#define		HALL_CALB_BIT					0x00FF00FF
	#define		GYRO_GAIN_FLG					0x00004000
	#define		ANGL_CORR_FLG					0x00002000
	#define		OPAF_FST_FLG					0x00001000
	#define		CLAF_CALB_FLG					0x00000800
	#define		HLLN_CALB_FLG					0x00000400
	#define		CROS_TALK_FLG					0x00000200

//==============================================================================
// Calibration Data Memory Map
//==============================================================================
// Calibration Status
#define	CALIBRATION_STATUS		(  0 )
// Hall amplitude Calibration X
#define	HALL_MAX_BEFORE_X		(  1 )
#define	HALL_MIN_BEFORE_X		(  2 )
#define	HALL_MAX_AFTER_X		(  3 )
#define	HALL_MIN_AFTER_X		(  4 )
// Hall amplitude Calibration Y
#define	HALL_MAX_BEFORE_Y		(  5 )
#define	HALL_MIN_BEFORE_Y		(  6 )
#define	HALL_MAX_AFTER_Y		(  7 )
#define	HALL_MIN_AFTER_Y		(  8 )
// Hall Bias/Offset
#define	HALL_BIAS_DAC_X			(  9 )
#define	HALL_OFFSET_DAC_X		( 10 )
#define	HALL_BIAS_DAC_Y			( 11 )
#define	HALL_OFFSET_DAC_Y		( 12 )
// Loop Gain Calibration X
#define	LOOP_GAIN_X				( 13 )
// Loop Gain Calibration Y
#define	LOOP_GAIN_Y				( 14 )
// Lens Center Calibration
#define	MECHA_CENTER_X			( 15 )
#define	MECHA_CENTER_Y			( 16 )
// Optical Center Calibration
#define	OPT_CENTER_X			( 17 )
#define	OPT_CENTER_Y			( 18 )
// Gyro Offset Calibration
#define	GYRO_OFFSET_X			( 19 )
#define	GYRO_OFFSET_Y			( 20 )
// Gyro Gain Calibration
#define	GYRO_GAIN_X				( 21 )
#define	GYRO_GAIN_Y				( 22 )

// Gyro mixing correction
#define MIXING_HX45X			( 32 )
#define MIXING_HX45Y			( 33 )
#define MIXING_HY45Y			( 34 )
#define MIXING_HY45X			( 35 )
#define MIXING_HXSX				( 36 )
#define MIXING_HYSX				( 36 )

// Liniearity correction
#define LN_POS1					( 41 )
#define LN_POS2					( 42 )
#define LN_POS3					( 43 )
#define LN_POS4					( 44 )
#define LN_POS5					( 45 )
#define LN_POS6					( 46 )
#define LN_POS7					( 47 )
#define LN_STEP					( 48 )
// Factory Gyro Gain Calibration
#define	GYRO_FCTRY_OFST_X		( 49 )
#define	GYRO_FCTRY_OFST_Y		( 50 )
// Gyro Offset Calibration
#define	GYRO_OFFSET_Z			( 51 )
// Accl offset
#define	ACCL_OFFSET_X			( 52 )
#define	ACCL_OFFSET_Y			( 53 )
#define	ACCL_OFFSET_Z			( 54 )
// Factory Gyro Gain Calibration
#define	GYRO_FCTRY_OFST_Z		( 55 )
#define	ACCL_FCTRY_OFST_X		( 56 )
#define	ACCL_FCTRY_OFST_Y		( 57 )
#define	ACCL_FCTRY_OFST_Z		( 58 )

// CheckSum
#define	HALL_CAL_CHECKSUM		( 63 )

//==============================================================================
//DMA
//==============================================================================
#define		GyroFilterDelayX_delay3_2		0x01D0
#define		GyroFilterDelayX_GXH1Z2				0x0004 + GyroFilterDelayX_delay3_2
#define		GyroFilterDelayY_delay3_2		0x01F8
#define		GyroFilterDelayY_GYH1Z2				0x0004 + GyroFilterDelayY_delay3_2

#define		GYRO_RAM_X						0x0210

#define			GYRO_RAM_GYROX_OFFSET			0x0000 + GYRO_RAM_X
#define		GYRO_RAM_Y						0x0234

#define			GYRO_RAM_GYROY_OFFSET			0x0000 + GYRO_RAM_Y

#define		GYRO_RAM_COMMON					0x0258

#define			GYRO_RAM_GX_ADIDAT				0x0000 + GYRO_RAM_COMMON
#define			GYRO_RAM_GY_ADIDAT				0x0004 + GYRO_RAM_GX_ADIDAT
#define			GYRO_RAM_SINDX					0x0004 + GYRO_RAM_GY_ADIDAT
#define			GYRO_RAM_SINDY					0x0004 + GYRO_RAM_SINDX
#define			GYRO_RAM_GXLENSZ				0x0004 + GYRO_RAM_SINDY
#define			GYRO_RAM_GYLENSZ				0x0004 + GYRO_RAM_GXLENSZ
#define			GYRO_RAM_GXOX_OUT				0x0004 + GYRO_RAM_GYLENSZ
#define			GYRO_RAM_GYOX_OUT				0x0004 + GYRO_RAM_GXOX_OUT
#define			GYRO_RAM_GXOFFZ					0x0004 + GYRO_RAM_GYOX_OUT
#define			GYRO_RAM_GYOFFZ					0x0004 + GYRO_RAM_GXOFFZ

#define		StMeasureFunc					0x02B0

#define			StMeasFunc_SiSampleNum			0x0000 + StMeasureFunc
#define			StMeasFunc_SiSampleMax			0x0004 + StMeasFunc_SiSampleNum

#define		StMeasureFunc_MFA				0x02B8
#define			StMeasFunc_MFA_SiMax1			0x0000 + StMeasureFunc_MFA
#define			StMeasFunc_MFA_SiMin1			0x0004 + StMeasFunc_MFA_SiMax1
#define			StMeasFunc_MFA_UiAmp1			0x0004 + StMeasFunc_MFA_SiMin1
#define			StMeasFunc_MFA_UiDUMMY1			0x0004 + StMeasFunc_MFA_UiAmp1
#define			StMeasFunc_MFA_LLiIntegral1		0x0004 + StMeasFunc_MFA_UiDUMMY1
#define			StMeasFunc_MFA_LLiAbsInteg1		0x0008 + StMeasFunc_MFA_LLiIntegral1
#define			StMeasFunc_MFA_PiMeasureRam1	0x0008 + StMeasFunc_MFA_LLiAbsInteg1

#define		StMeasureFunc_MFB				0x02E0
#define			StMeasFunc_MFB_SiMax2			0x0000 + StMeasureFunc_MFB
#define			StMeasFunc_MFB_SiMin2			0x0004 + StMeasFunc_MFB_SiMax2
#define			StMeasFunc_MFB_UiAmp2			0x0004 + StMeasFunc_MFB_SiMin2
#define			StMeasFunc_MFB_UiDUMMY1			0x0004 + StMeasFunc_MFB_UiAmp2
#define			StMeasFunc_MFB_LLiIntegral2		0x0004 + StMeasFunc_MFB_UiDUMMY1
#define			StMeasFunc_MFB_LLiAbsInteg2		0x0008 + StMeasFunc_MFB_LLiIntegral2
#define			StMeasFunc_MFB_PiMeasureRam2	0x0008 + StMeasFunc_MFB_LLiAbsInteg2

#define		MeasureFilterA_Delay			0x0308

#define			MeasureFilterA_Delay_z11		0x0000 + MeasureFilterA_Delay
#define			MeasureFilterA_Delay_z12		0x0004 + MeasureFilterA_Delay_z11
#define			MeasureFilterA_Delay_z21		0x0004 + MeasureFilterA_Delay_z12
#define			MeasureFilterA_Delay_z22		0x0004 + MeasureFilterA_Delay_z21

#define		MeasureFilterB_Delay			0x0318

#define			MeasureFilterB_Delay_z11		0x0000 + MeasureFilterB_Delay
#define			MeasureFilterB_Delay_z12		0x0004 + MeasureFilterB_Delay_z11
#define			MeasureFilterB_Delay_z21		0x0004 + MeasureFilterB_Delay_z12
#define			MeasureFilterB_Delay_z22		0x0004 + MeasureFilterB_Delay_z21

#define		WaitTimerData					0x035C

#define			WaitTimerData_UiWaitCounter		0x0000 + WaitTimerData
#define			WaitTimerData_UiTargetCount		0x0004 + WaitTimerData_UiWaitCounter

#define		StMeasureFunc_PMC				0x04B4
#define			StMeasFunc_PMC_UcPhaseMesMode	0x0000 + StMeasureFunc_PMC
#define			StMeasFunc_PMC_UcFRASweepMode	0x0001 + StMeasFunc_PMC_UcPhaseMesMode
#define			StMeasFunc_PMC_UcPrevSign_A		0x0001 + StMeasFunc_PMC_UcFRASweepMode
#define			StMeasFunc_PMC_UcCurrentSign_A	0x0001 + StMeasFunc_PMC_UcPrevSign_A
#define			StMeasFunc_PMC_UcCrossDetectA	0x0001 + StMeasFunc_PMC_UcCurrentSign_A
#define			StMeasFunc_PMC_UcPrevSign_B		0x0001 + StMeasFunc_PMC_UcCrossDetectA
#define			StMeasFunc_PMC_UcCurrentSign_B	0x0001 + StMeasFunc_PMC_UcPrevSign_B
#define			StMeasFunc_PMC_UcCrossDetectB	0x0001 + StMeasFunc_PMC_UcCurrentSign_B
#define			StMeasFunc_PMC_SiFsCountF		0x0001 + StMeasFunc_PMC_UcCrossDetectB
#define			StMeasFunc_PMC_SiFsCountR		0x0004 + StMeasFunc_PMC_SiFsCountF
#define			StMeasFunc_MFA_SiSampleNumA		0x0004 + StMeasFunc_PMC_SiFsCountR
#define			StMeasFunc_MFA_SiSampleMaxA		0x0004 + StMeasFunc_MFA_SiSampleNumA
#define			StMeasFunc_MFB_SiSampleNumB		0x0004 + StMeasFunc_MFA_SiSampleMaxA
#define			StMeasFunc_MFB_SiSampleMaxB		0x0004 + StMeasFunc_MFB_SiSampleNumB

#define			GyroRAM_Z_GYRO_OFFSET		0x0510

#define			GYRO_ZRAM_GZ_ADIDAT			0x0534
#define			GYRO_ZRAM_GZOFFZ			0x0540

#define		AcclFilDly_X					0x0550
#define		AcclFilDly_Y					0x0580
#define		AcclFilDly_Z					0x05B0

#define		AcclRAM_X						0x05E0
#define			ACCLRAM_X_AC_ADIDAT			0x0000 + AcclRAM_X
#define			ACCLRAM_X_AC_OFFSET			0x0004 + AcclRAM_X

#define		AcclRAM_Y						0x060C
#define			ACCLRAM_Y_AC_ADIDAT			0x0000 + AcclRAM_Y
#define			ACCLRAM_Y_AC_OFFSET			0x0004 + AcclRAM_Y

#define		AcclRAM_Z						0x0638
#define			ACCLRAM_Z_AC_ADIDAT			0x0000 + AcclRAM_Z
#define			ACCLRAM_Z_AC_OFFSET			0x0004 + AcclRAM_Z

//==============================================================================
//
//==============================================================================
#define		SiVerNum						0x8000
#define		SiCalID							0x8004
//	#define	TVA707		0x00	/* 1st revision */
//	#define	OAZ01		0x01

#define		StCalibrationData				0x8010
				// Calibration.h  CalibrationData_Type
#define			StCaliData_UsCalibrationStatus	0x0000 + StCalibrationData
#define			StCaliData_SiHallMax_Before_X	0x0004 + StCaliData_UsCalibrationStatus
#define			StCaliData_SiHallMin_Before_X	0x0004 + StCaliData_SiHallMax_Before_X
#define			StCaliData_SiHallMax_After_X	0x0004 + StCaliData_SiHallMin_Before_X
#define			StCaliData_SiHallMin_After_X	0x0004 + StCaliData_SiHallMax_After_X
#define			StCaliData_SiHallMax_Before_Y	0x0004 + StCaliData_SiHallMin_After_X
#define			StCaliData_SiHallMin_Before_Y	0x0004 + StCaliData_SiHallMax_Before_Y
#define			StCaliData_SiHallMax_After_Y	0x0004 + StCaliData_SiHallMin_Before_Y
#define			StCaliData_SiHallMin_After_Y	0x0004 + StCaliData_SiHallMax_After_Y
#define			StCaliData_UiHallBias_X			0x0004 + StCaliData_SiHallMin_After_Y
#define			StCaliData_UiHallOffset_X		0x0004 + StCaliData_UiHallBias_X
#define			StCaliData_UiHallBias_Y			0x0004 + StCaliData_UiHallOffset_X
#define			StCaliData_UiHallOffset_Y		0x0004 + StCaliData_UiHallBias_Y
#define			StCaliData_SiLoopGain_X			0x0004 + StCaliData_UiHallOffset_Y
#define			StCaliData_SiLoopGain_Y			0x0004 + StCaliData_SiLoopGain_X
#define			StCaliData_SiLensCen_Offset_X	0x0004 + StCaliData_SiLoopGain_Y
#define			StCaliData_SiLensCen_Offset_Y	0x0004 + StCaliData_SiLensCen_Offset_X
#define			StCaliData_SiOtpCen_Offset_X	0x0004 + StCaliData_SiLensCen_Offset_Y
#define			StCaliData_SiOtpCen_Offset_Y	0x0004 + StCaliData_SiOtpCen_Offset_X
#define			StCaliData_SiGyroOffset_X		0x0004 + StCaliData_SiOtpCen_Offset_Y
#define			StCaliData_SiGyroOffset_Y		0x0004 + StCaliData_SiGyroOffset_X
#define			StCaliData_SiGyroGain_X			0x0004 + StCaliData_SiGyroOffset_Y
#define			StCaliData_SiGyroGain_Y			0x0004 + StCaliData_SiGyroGain_X
#define			StCaliData_UiHallBias_AF		0x0004 + StCaliData_SiGyroGain_Y
#define			StCaliData_UiHallOffset_AF		0x0004 + StCaliData_UiHallBias_AF
#define			StCaliData_SiLoopGain_AF		0x0004 + StCaliData_UiHallOffset_AF
#define			StCaliData_SiAD_Offset_AF		0x0004 + StCaliData_SiLoopGain_AF
#define			StCaliData_SiMagnification_AF	0x0004 + StCaliData_SiAD_Offset_AF
#define			StCaliData_SiHallMax_Before_AF	0x0004 + StCaliData_SiMagnification_AF
#define			StCaliData_SiHallMin_Before_AF	0x0004 + StCaliData_SiHallMax_Before_AF
#define			StCaliData_SiHallMax_After_AF	0x0004 + StCaliData_SiHallMin_Before_AF
#define			StCaliData_SiHallMin_After_AF	0x0004 + StCaliData_SiHallMax_After_AF

#define		MeasureFilterA_Coeff			0x8380
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterA_Coeff_b1			0x0000 + MeasureFilterA_Coeff
#define			MeasureFilterA_Coeff_c1			0x0004 + MeasureFilterA_Coeff_b1
#define			MeasureFilterA_Coeff_a1			0x0004 + MeasureFilterA_Coeff_c1
#define			MeasureFilterA_Coeff_b2			0x0004 + MeasureFilterA_Coeff_a1
#define			MeasureFilterA_Coeff_c2			0x0004 + MeasureFilterA_Coeff_b2
#define			MeasureFilterA_Coeff_a2			0x0004 + MeasureFilterA_Coeff_c2

#define		MeasureFilterB_Coeff			0x8398
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterB_Coeff_b1			0x0000 + MeasureFilterB_Coeff
#define			MeasureFilterB_Coeff_c1			0x0004 + MeasureFilterB_Coeff_b1
#define			MeasureFilterB_Coeff_a1			0x0004 + MeasureFilterB_Coeff_c1
#define			MeasureFilterB_Coeff_b2			0x0004 + MeasureFilterB_Coeff_a1
#define			MeasureFilterB_Coeff_c2			0x0004 + MeasureFilterB_Coeff_b2
#define			MeasureFilterB_Coeff_a2			0x0004 + MeasureFilterB_Coeff_c2

//==============================================================================
//IO
//==============================================================================
// System Control
#define 		SYSDSP_DSPDIV					0xD00014
#define 		SYSDSP_SOFTRES					0xD0006C
//#define 		OSCRSEL							0xD00090	// OSC Frequency 1
//#define 		OSCCURSEL						0xD00094	// OSC Frequency 2
#define 		SYSDSP_REMAP					0xD000AC
#define 		SYSDSP_CVER						0xD00100

#define FLASHROM_123F40		0xE07000	// Flash Memory I/F配置アドレス
#define 		FLASHROM_F40_RDATL					(FLASHROM_123F40 + 0x00)
#define 		FLASHROM_F40_RDATH					(FLASHROM_123F40 + 0x04)
#define 		FLASHROM_F40_WDATL					(FLASHROM_123F40 + 0x08)
#define 		FLASHROM_F40_WDATH					(FLASHROM_123F40 + 0x0C)
#define 		FLASHROM_F40_ADR					(FLASHROM_123F40 + 0x10)
#define 		FLASHROM_F40_ACSCNT					(FLASHROM_123F40 + 0x14)
#define 		FLASHROM_F40_CMD					(FLASHROM_123F40 + 0x18)
#define 		FLASHROM_F40_WPB					(FLASHROM_123F40 + 0x1C)
#define 		FLASHROM_F40_INT					(FLASHROM_123F40 + 0x20)

#define 		FLASHROM_F40_RSTB_FLA				(FLASHROM_123F40 + 0x4CC)
#define 		FLASHROM_F40_UNLK_CODE1				(FLASHROM_123F40 + 0x554)
#define 		FLASHROM_F40_CLK_FLAON				(FLASHROM_123F40 + 0x664)
#define 		FLASHROM_F40_UNLK_CODE2				(FLASHROM_123F40 + 0xAA8)
#define 		FLASHROM_F40_UNLK_CODE3				(FLASHROM_123F40 + 0xCCC)
