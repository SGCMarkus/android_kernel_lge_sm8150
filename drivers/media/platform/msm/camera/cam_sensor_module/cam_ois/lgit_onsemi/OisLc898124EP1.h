//********************************************************************************
//
//		<< LC898124EP1 Evaluation Soft>>
//		Program Name	: OisLC898124EP1.h
// 		Explanation		: LC898124 Global Declaration & ProtType Declaration
//		Design			: K.abe
//		History			: First edition
//********************************************************************************
/************************************************/
/*	Command										*/
/************************************************/
#define		CMD_IO_ADR_ACCESS				0xC000				// IO Write Access
#define		CMD_IO_DAT_ACCESS				0xD000				// IO Read Access
#define		CMD_REMAP						0xF001				// Remap
#define		CMD_REBOOT						0xF003				// Reboot
#define		CMD_RETURN_TO_CENTER			0xF010				// Center Servo ON/OFF choose axis
#define		BOTH_SRV_OFF					0x00000000			// Both   Servo OFF
#define		XAXS_SRV_ON						0x00000001			// X axis Servo ON
#define		YAXS_SRV_ON						0x00000002			// Y axis Servo ON
#define		BOTH_SRV_ON						0x00000003			// Both   Servo ON
#define		ZAXS_SRV_OFF					0x00000004			// Z axis Servo OFF
#define		ZAXS_SRV_ON						0x00000005			// Z axis Servo ON
#define		CMD_PAN_TILT					0xF011				// Pan Tilt Enable/Disable
#define		PAN_TILT_OFF					0x00000000			// Pan/Tilt OFF
#define		PAN_TILT_ON						0x00000001			// Pan/Tilt ON
#define		CMD_OIS_ENABLE					0xF012				// Ois Enable/Disable
#define		OIS_DISABLE						0x00000000			// OIS Disable
#define		OIS_ENABLE						0x00000001			// OIS Enable
#define		OIS_ENA_NCL						0x00000002			// OIS Enable ( none Delay clear )
#define		OIS_ENA_DOF						0x00000004			// OIS Enable ( Drift offset exec )
#define		CMD_MOVE_STILL_MODE				0xF013				// Select mode
#define		MOVIE_MODE						0x00000000			// Movie mode
#define		STILL_MODE						0x00000001			// Still mode
#define		MOVIE_MODE1						0x00000002			// Movie Preview mode 1
#define		STILL_MODE1						0x00000003			// Still Preview mode 1
#define		MOVIE_MODE2						0x00000004			// Movie Preview mode 2
#define		STILL_MODE2						0x00000005			// Still Preview mode 2
#define		MOVIE_MODE3						0x00000006			// Movie Preview mode 3
#define		STILL_MODE3						0x00000007			// Still Preview mode 3
#define		CMD_CHASE_CONFIRMATION			0xF015				// Hall Chase confirmation
#define		CMD_GYRO_SIG_CONFIRMATION		0xF016				// Gyro Signal confirmation
#define		CMD_FLASH_LOAD					0xF017				// Flash Load
#define		HALL_CALB_FLG					0x00008000
#define		HALL_CALB_BIT					0x00FF00FF
#define		GYRO_GAIN_FLG					0x00004000
#define		ANGL_CORR_FLG					0x00002000
#define		FOCL_GAIN_FLG					0x00001000
#define		CLAF_CALB_FLG					0x00000800			// CLAF Hall calibration
#define		HLLN_CALB_FLG					0x00000400			// Hall linear calibration
#define		CMD_AF_POSITION					0xF01A				// AF Position
#define		CMD_GYRO_RD_ACCS				0xF01D				// Gyro Read Acess
#define		CMD_GYRO_WR_ACCS				0xF01E				// Gyro Write Acess

#define		CMD_READ_STATUS					0xF100				// Status Read
#define		READ_STATUS_INI					0x01000000

//==============================================================================
//E2PROM
//==============================================================================
#define		EEPROM_ONSEMI_LDO					0x00
#define		EEPROM_ONSEMI_CP1					0x01
#define		EEPROM_ONSEMI_CP2					0x02
#define		EEPROM_ONSEMI_CP3					0x03
#define		EEPROM_ONSEMI_OSCS					0x04
#define		EEPROM_ONSEMI_DRVGAINAF				0x05
#define		EEPROM_ONSEMI_DRVOFSTAF				0x06
#define		EEPROM_ONSEMI_DRVOFSTAFM			0x07
#define		EEPROM_ONSEMI_DRVGAINX				0x08
#define		EEPROM_ONSEMI_DRVOFSTX				0x09
#define		EEPROM_ONSEMI_DRVOFSTXM				0x0A
#define		EEPROM_ONSEMI_DRVGAINY				0x0B
#define		EEPROM_ONSEMI_DRVOFSTY				0x0C
#define		EEPROM_ONSEMI_DRVOFSTYM				0x0D
#define		EEPROM_ONSEMI_MARK					0x0E
#define		EEPROM_ONSEMI_CHECKSUM				0x0F		/* target area 0x00~0x0E */

#define		EEPROM_ONSEMI_IDSEL					0x10
#define		EEPROM_Calibration_Status_LSB		0x11
#define		EEPROM_Calibration_Status_MSB		0x12
#define		EEPROM_Calibration_HallMaxX_LSB		0x13
#define		EEPROM_Calibration_HallMaxX_MSB		0x14
#define		EEPROM_Calibration_HallMinX_LSB		0x15
#define		EEPROM_Calibration_HallMinX_MSB		0x16
#define		EEPROM_Calibration_HallMaxY_LSB		0x17
#define		EEPROM_Calibration_HallMaxY_MSB		0x18
#define		EEPROM_Calibration_HallMinY_LSB		0x19
#define		EEPROM_Calibration_HallMinY_MSB		0x1A
#define		EEPROM_Calibration_HallBiasX		0x1B
#define		EEPROM_Calibration_HallOffsetX		0x1C
#define		EEPROM_Calibration_HallBiasY		0x1D
#define		EEPROM_Calibration_HallOffsetY		0x1E
#define		EEPROM_Calibration_LoopGainX_LSB	0x1F

#define		EEPROM_Calibration_LoopGainX_MSB	0x20
#define		EEPROM_Calibration_LoopGainY_LSB	0x21
#define		EEPROM_Calibration_LoopGainY_MSB	0x22
#define		EEPROM_Calibration_LensOffsetX_LSB	0x23
#define		EEPROM_Calibration_LensOffsetX_MSB	0x24
#define		EEPROM_Calibration_LensOffsetY_LSB	0x25
#define		EEPROM_Calibration_LensOffsetY_MSB	0x26
#define		EEPROM_Calibration_GyroGainX_0Byte	0x27
#define		EEPROM_Calibration_GyroGainX_1Byte	0x28
#define		EEPROM_Calibration_GyroGainX_2Byte	0x29
#define		EEPROM_Calibration_GyroGainX_3Byte	0x2A
#define		EEPROM_Calibration_GyroGainY_0Byte	0x2B
#define		EEPROM_Calibration_GyroGainY_1Byte	0x2C
#define		EEPROM_Calibration_GyroGainY_2Byte	0x2D
#define		EEPROM_Calibration_GyroGainY_3Byte	0x2E
#define		EEPROM_Calibration_GyroOffsetX_LSB	0x2F
#define		EEPROM_Calibration_GyroOffsetX_MSB	0x30
#define		EEPROM_Calibration_GyroOffsetY_LSB	0x31
#define		EEPROM_Calibration_GyroOffsetY_MSB	0x32

#define		EEPROM_CheckSum						0x77	/* target area 0x10~0x77 */

//==============================================================================
//DMA
//==============================================================================
#define		HallFilterD_HXDAZ1				0x0080
#define		HallFilterD_HYDAZ1				0x00D0

#define		HALL_RAM_X_COMMON				0x0110
#define			HALL_RAM_HXOFF					(0x0000 + HALL_RAM_X_COMMON)		// 0x0490
#define			HALL_RAM_HXOFF1					(0x0004 + HALL_RAM_X_COMMON)		// 0x0494
#define			HALL_RAM_HXOUT0					(0x0008 + HALL_RAM_X_COMMON)		// 0x0498
#define			HALL_RAM_HXOUT1					(0x000C + HALL_RAM_X_COMMON)		// 0x049C
#define			HALL_RAM_SINDX0					(0x0010 + HALL_RAM_X_COMMON)		// 0x04A0
#define			HALL_RAM_HXLOP					(0x0014 + HALL_RAM_X_COMMON)		// 0x04A4
#define			HALL_RAM_SINDX1					(0x0018 + HALL_RAM_X_COMMON)		// 0x04A8
#define			HALL_RAM_HALL_X_OUT				(0x001C + HALL_RAM_X_COMMON)		// 0x04AC
#define		HALL_RAM_HALL_SwitchX			0x015c

#define		HALL_RAM_Y_COMMON				0x0160
#define			HALL_RAM_HYOFF					(0x0000 + HALL_RAM_Y_COMMON)		// 0x04E0
#define			HALL_RAM_HYOFF1					(0x0004 + HALL_RAM_Y_COMMON)		// 0x04E4
#define			HALL_RAM_HYOUT0					(0x0008 + HALL_RAM_Y_COMMON)		// 0x04E8
#define			HALL_RAM_HYOUT1					(0x000C + HALL_RAM_Y_COMMON)		// 0x04EC
#define			HALL_RAM_SINDY0					(0x0010 + HALL_RAM_Y_COMMON)		// 0x04F0
#define			HALL_RAM_HYLOP					(0x0014 + HALL_RAM_Y_COMMON)		// 0x04F4
#define			HALL_RAM_SINDY1					(0x0018 + HALL_RAM_Y_COMMON)		// 0x04F8
#define			HALL_RAM_HALL_Y_OUT				(0x001C + HALL_RAM_Y_COMMON)		// 0x04FC
#define		HALL_RAM_HALL_SwitchY			0x01AC


#define		HALL_RAM_COMMON					0x01B0
				//  HallFilterDelay.h HALL_RAM_COMMON_t
#define			HALL_RAM_HXIDAT					(0x0000 + HALL_RAM_COMMON)		// 0x0530
#define			HALL_RAM_HYIDAT					(0x0004 + HALL_RAM_COMMON)		// 0x0534
#define			HALL_RAM_GYROX_OUT				(0x0008 + HALL_RAM_COMMON)		// 0x0538
#define			HALL_RAM_GYROY_OUT				(0x000C + HALL_RAM_COMMON)		// 0x053C

#define		GyroFilterDelayX_delay3_2		0x1D0
#define		GyroFilterDelayX_GXH1Z2				(0x0000 + GyroFilterDelayX_delay3_2)
#define		GyroFilterDelayY_delay3_2		0x1F8
#define		GyroFilterDelayY_GYH1Z2				(0x0000 + GyroFilterDelayY_delay3_2)

#define		GYRO_RAM_X						0x0210
				// GyroFilterDelay.h GYRO_RAM_t
#define			GYRO_RAM_GYROX_OFFSET			(0x0000 + GYRO_RAM_X)
#define			GYRO_RAM_GX2X4XF_IN				(0x0004 + GYRO_RAM_GYROX_OFFSET)
#define			GYRO_RAM_GX2X4XF_OUT			(0x0004 + GYRO_RAM_GX2X4XF_IN)
#define			GYRO_RAM_GXFAST					(0x0004 + GYRO_RAM_GX2X4XF_OUT)
#define			GYRO_RAM_GXSLOW					(0x0004 + GYRO_RAM_GXFAST)
#define			GYRO_RAM_GYROX_G1OUT			(0x0004 + GYRO_RAM_GXSLOW)
#define			GYRO_RAM_GYROX_G2OUT			(0x0004 + GYRO_RAM_GYROX_G1OUT)
#define			GYRO_RAM_GYROX_G3OUT			(0x0004 + GYRO_RAM_GYROX_G2OUT)
#define			GYRO_RAM_GYROX_OUT				(0x0004 + GYRO_RAM_GYROX_G3OUT)
#define		GYRO_RAM_Y						0x0234
				// GyroFilterDelay.h GYRO_RAM_t
#define			GYRO_RAM_GYROY_OFFSET			(0x0000 + GYRO_RAM_Y)
#define			GYRO_RAM_GY2X4XF_IN				(0x0004 + GYRO_RAM_GYROY_OFFSET)
#define			GYRO_RAM_GY2X4XF_OUT			(0x0004 + GYRO_RAM_GY2X4XF_IN)
#define			GYRO_RAM_GYFAST					(0x0004 + GYRO_RAM_GY2X4XF_OUT)
#define			GYRO_RAM_GYSLOW					(0x0004 + GYRO_RAM_GYFAST)
#define			GYRO_RAM_GYROY_G1OUT			(0x0004 + GYRO_RAM_GYSLOW)
#define			GYRO_RAM_GYROY_G2OUT			(0x0004 + GYRO_RAM_GYROY_G1OUT)
#define			GYRO_RAM_GYROY_G3OUT			(0x0004 + GYRO_RAM_GYROY_G2OUT)
#define			GYRO_RAM_GYROY_OUT				(0x0004 + GYRO_RAM_GYROY_G3OUT)
#define		GYRO_RAM_COMMON					0x0258
				// GyroFilterDelay.h GYRO_RAM_COMMON_t
#define			GYRO_RAM_GX_ADIDAT				(0x0000 + GYRO_RAM_COMMON)
#define			GYRO_RAM_GY_ADIDAT				(0x0004 + GYRO_RAM_GX_ADIDAT)
#define			GYRO_RAM_SINDX					(0x0004 + GYRO_RAM_GY_ADIDAT)
#define			GYRO_RAM_SINDY					(0x0004 + GYRO_RAM_SINDX)
#define			GYRO_RAM_GXLENSZ				(0x0004 + GYRO_RAM_SINDY)
#define			GYRO_RAM_GYLENSZ				(0x0004 + GYRO_RAM_GXLENSZ)
#define			GYRO_RAM_GXOX_OUT				(0x0004 + GYRO_RAM_GYLENSZ)
#define			GYRO_RAM_GYOX_OUT				(0x0004 + GYRO_RAM_GXOX_OUT)
#define			GYRO_RAM_GXOFFZ					(0x0004 + GYRO_RAM_GYOX_OUT)
#define			GYRO_RAM_GYOFFZ					(0x0004 + GYRO_RAM_GXOFFZ)
#define			GYRO_RAM_LIMITX					(0x0004 + GYRO_RAM_GYOFFZ)
#define			GYRO_RAM_LIMITY					(0x0004 + GYRO_RAM_LIMITX)
#define			GYRO_RAM_GZ_ADIDAT				(0x0004 + GYRO_RAM_LIMITY)
#define			GYRO_RAM_Reserve				(0x0004 + GYRO_RAM_GZ_ADIDAT)
#define			GYRO_RAM_GYRO_Switch			(0x0004 + GYRO_RAM_Reserve)			// 1Byte
#define			GYRO_RAM_GYRO_AF_Switch			(0x0001 + GYRO_RAM_GYRO_Switch)		// 1Byte

#define		StMeasureFunc					0x02B0
#define			StMeasFunc_SiSampleNum			(0x0000 + StMeasureFunc		)			// 0x0708
#define			StMeasFunc_SiSampleMax			(0x0004 + StMeasFunc_SiSampleNum)			// 0x070C

#define		StMeasureFunc_MFA				0x02B8
#define			StMeasFunc_MFA_SiMax1			(0x0000 + StMeasureFunc_MFA		)		// 0x0710
#define			StMeasFunc_MFA_SiMin1			(0x0004 + StMeasFunc_MFA_SiMax1	)		// 0x0714
#define			StMeasFunc_MFA_UiAmp1			(0x0004 + StMeasFunc_MFA_SiMin1	)		// 0x0718
#define			StMeasFunc_MFA_UiDUMMY1			(0x0004 + StMeasFunc_MFA_UiAmp1	)		// 0x071C
#define			StMeasFunc_MFA_LLiIntegral1		(0x0004 + StMeasFunc_MFA_UiDUMMY1)		// 0x0720
#define			StMeasFunc_MFA_LLiAbsInteg1		(0x0008 + StMeasFunc_MFA_LLiIntegral1)	// 0x0728	// 8Byte
#define			StMeasFunc_MFA_PiMeasureRam1	(0x0008 + StMeasFunc_MFA_LLiAbsInteg1)	// 0x0730	// 8Byte
#define			StMeasFunc_MFA_UiDUMMY2			(0x0004 + StMeasFunc_MFA_PiMeasureRam1)	// 0x0734

#define		StMeasureFunc_MFB				0x02E0
#define			StMeasFunc_MFB_SiMax2			(0x0000 + StMeasureFunc_MFB			)	// 0x0738
#define			StMeasFunc_MFB_SiMin2			(0x0004 + StMeasFunc_MFB_SiMax2		)	// 0x073C
#define			StMeasFunc_MFB_UiAmp2			(0x0004 + StMeasFunc_MFB_SiMin2		)	// 0x0740
#define			StMeasFunc_MFB_UiDUMMY1			(0x0004 + StMeasFunc_MFB_UiAmp2		)	// 0x0744
#define			StMeasFunc_MFB_LLiIntegral2		(0x0004 + StMeasFunc_MFB_UiDUMMY1	)	// 0x0748
#define			StMeasFunc_MFB_LLiAbsInteg2		(0x0008 + StMeasFunc_MFB_LLiIntegral2)	// 0x0750	// 8Byte
#define			StMeasFunc_MFB_PiMeasureRam2	(0x0008 + StMeasFunc_MFB_LLiAbsInteg2)	// 0x0758	// 8Byte

#define		MeasureFilterA_Delay			0x0308
				// MeasureFilter.h	MeasureFilter_Delay_Type
#define			MeasureFilterA_Delay_z11		(0x0000 + MeasureFilterA_Delay)
#define			MeasureFilterA_Delay_z12		(0x0004 + MeasureFilterA_Delay_z11)
#define			MeasureFilterA_Delay_z21		(0x0004 + MeasureFilterA_Delay_z12)
#define			MeasureFilterA_Delay_z22		(0x0004 + MeasureFilterA_Delay_z21)

#define		MeasureFilterB_Delay			0x0318
				// MeasureFilter.h	MeasureFilter_Delay_Type
#define			MeasureFilterB_Delay_z11		(0x0000 + MeasureFilterB_Delay)
#define			MeasureFilterB_Delay_z12		(0x0004 + MeasureFilterB_Delay_z11)
#define			MeasureFilterB_Delay_z21		(0x0004 + MeasureFilterB_Delay_z12)
#define			MeasureFilterB_Delay_z22		(0x0004 + MeasureFilterB_Delay_z21)

#define		SinWaveC						0x0328
#define			SinWaveC_Pt						(0x0000 + SinWaveC)
#define			SinWaveC_Regsiter				(0x0004 + SinWaveC_Pt)
//#define			SinWaveC_SignFlag				0x0004 + SinWaveC_Regsiter

#define		SinWave							0x0334
				// SinGenerator.h SinWave_t
#define			SinWave_Offset					(0x0000 + SinWave)
#define			SinWave_Phase					(0x0004 + SinWave_Offset)
#define			SinWave_Gain					(0x0004 + SinWave_Phase)
#define			SinWave_Output					(0x0004 + SinWave_Gain)
#define			SinWave_OutAddr					(0x0004 + SinWave_Output)
#define		CosWave							0x0348
				// SinGenerator.h SinWave_t
#define			CosWave_Offset					(0x0000 + CosWave)
#define			CosWave_Phase					(0x0004 + CosWave_Offset)
#define			CosWave_Gain					(0x0004 + CosWave_Phase)
#define			CosWave_Output					(0x0004 + CosWave_Gain)
#define			CosWave_OutAddr					(0x0004 + CosWave_Output)

#define		WaitTimerData					0x035C
				// CommonLibrary.h  WaitTimer_Type
#define			WaitTimerData_UiWaitCounter		(0x0000 + WaitTimerData	)				// 0x06F4
#define			WaitTimerData_UiTargetCount		(0x0004 + WaitTimerData_UiWaitCounter)	// 0x06F8

#define		PanTilt_DMA_ScTpdSts			0x037C

#define 	PmCheck_CheckSum				0x514
#define 	PmCheck_EndFlag					0x518

#define		DmCheck_CheckSumDMA				0x53C
#define		DmCheck_CheckSumDMB				0x540

//==============================================================================
//DMB
//==============================================================================
#define		SiVerNum						0x8000
#define		ACT_LGIT_N3						0x01
#define		ACT_LGIT_N3V02					0x02

#define		GYRO_ICG1020S					0x00

#define		StCalibrationData				0x8010
				// Calibration.h  CalibrationData_Type
#define			StCaliData_UsCalibrationStatus	(0x0000 + StCalibrationData)
#define			StCaliData_SiHallMax_Before_X	(0x0004 + StCaliData_UsCalibrationStatus)
#define			StCaliData_SiHallMin_Before_X	(0x0004 + StCaliData_SiHallMax_Before_X)
#define			StCaliData_SiHallMax_After_X	(0x0004 + StCaliData_SiHallMin_Before_X)
#define			StCaliData_SiHallMin_After_X	(0x0004 + StCaliData_SiHallMax_After_X)
#define			StCaliData_SiHallMax_Before_Y	(0x0004 + StCaliData_SiHallMin_After_X)
#define			StCaliData_SiHallMin_Before_Y	(0x0004 + StCaliData_SiHallMax_Before_Y)
#define			StCaliData_SiHallMax_After_Y	(0x0004 + StCaliData_SiHallMin_Before_Y)
#define			StCaliData_SiHallMin_After_Y	(0x0004 + StCaliData_SiHallMax_After_Y)
#define			StCaliData_UiHallBias_X			(0x0004 + StCaliData_SiHallMin_After_Y)
#define			StCaliData_UiHallOffset_X		(0x0004 + StCaliData_UiHallBias_X)
#define			StCaliData_UiHallBias_Y			(0x0004 + StCaliData_UiHallOffset_X)
#define			StCaliData_UiHallOffset_Y		(0x0004 + StCaliData_UiHallBias_Y)
#define			StCaliData_SiLoopGain_X			(0x0004 + StCaliData_UiHallOffset_Y)
#define			StCaliData_SiLoopGain_Y			(0x0004 + StCaliData_SiLoopGain_X)
#define			StCaliData_SiLensCen_Offset_X	(0x0004 + StCaliData_SiLoopGain_Y)
#define			StCaliData_SiLensCen_Offset_Y	(0x0004 + StCaliData_SiLensCen_Offset_X)
#define			StCaliData_SiOtpCen_Offset_X	(0x0004 + StCaliData_SiLensCen_Offset_Y)
#define			StCaliData_SiOtpCen_Offset_Y	(0x0004 + StCaliData_SiOtpCen_Offset_X)
#define			StCaliData_SiGyroOffset_X		(0x0004 + StCaliData_SiOtpCen_Offset_Y)
#define			StCaliData_SiGyroOffset_Y		(0x0004 + StCaliData_SiGyroOffset_X)
#define			StCaliData_SiGyroGain_X			(0x0004 + StCaliData_SiGyroOffset_Y)
#define			StCaliData_SiGyroGain_Y			(0x0004 + StCaliData_SiGyroGain_X)
#define			StCaliData_UiHallBias_AF		(0x0004 + StCaliData_SiGyroGain_Y)
#define			StCaliData_UiHallOffset_AF		(0x0004 + StCaliData_UiHallBias_AF)
#define			StCaliData_SiLoopGain_AF		(0x0004 + StCaliData_UiHallOffset_AF)
#define			StCaliData_SiAD_Offset_AF		(0x0004 + StCaliData_SiLoopGain_AF)
#define			StCaliData_SiMagnification_AF	(0x0004 + StCaliData_SiAD_Offset_AF)
#define			StCaliData_SiHallMax_Before_AF	(0x0004 + StCaliData_SiMagnification_AF)
#define			StCaliData_SiHallMin_Before_AF	(0x0004 + StCaliData_SiHallMax_Before_AF)
#define			StCaliData_SiHallMax_After_AF	(0x0004 + StCaliData_SiHallMin_Before_AF)
#define			StCaliData_SiHallMin_After_AF	(0x0004 + StCaliData_SiHallMax_After_AF)

#define		HallFilterCoeffX				0x8090
				// HallFilterCoeff.h  DM_HFC_t
#define			HallFilterCoeffX_HXIGAIN		(0x0000 + HallFilterCoeffX)
#define			HallFilterCoeffX_GYROXOUTGAIN	(0x0004 + HallFilterCoeffX_HXIGAIN)
#define			HallFilterCoeffX_HXOFFGAIN		(0x0004 + HallFilterCoeffX_GYROXOUTGAIN)

#define			HallFilterCoeffX_hxiab			(0x0004 + HallFilterCoeffX_HXOFFGAIN)
#define			HallFilterCoeffX_hxiac			(0x0004 + HallFilterCoeffX_hxiab)
#define			HallFilterCoeffX_hxiaa			(0x0004 + HallFilterCoeffX_hxiac)
#define			HallFilterCoeffX_hxibb			(0x0004 + HallFilterCoeffX_hxiaa)
#define			HallFilterCoeffX_hxibc			(0x0004 + HallFilterCoeffX_hxibb)
#define			HallFilterCoeffX_hxiba			(0x0004 + HallFilterCoeffX_hxibc)
#define			HallFilterCoeffX_hxdab			(0x0004 + HallFilterCoeffX_hxiba)
#define			HallFilterCoeffX_hxdac			(0x0004 + HallFilterCoeffX_hxdab)
#define			HallFilterCoeffX_hxdaa			(0x0004 + HallFilterCoeffX_hxdac)
#define			HallFilterCoeffX_hxdbb			(0x0004 + HallFilterCoeffX_hxdaa)
#define			HallFilterCoeffX_hxdbc			(0x0004 + HallFilterCoeffX_hxdbb)
#define			HallFilterCoeffX_hxdba			(0x0004 + HallFilterCoeffX_hxdbc)
#define			HallFilterCoeffX_hxdcc			(0x0004 + HallFilterCoeffX_hxdba)
#define			HallFilterCoeffX_hxdcb			(0x0004 + HallFilterCoeffX_hxdcc)
#define			HallFilterCoeffX_hxdca			(0x0004 + HallFilterCoeffX_hxdcb)
#define			HallFilterCoeffX_hxpgain0		(0x0004 + HallFilterCoeffX_hxdca)
#define			HallFilterCoeffX_hxigain0		(0x0004 + HallFilterCoeffX_hxpgain0)
#define			HallFilterCoeffX_hxdgain0		(0x0004 + HallFilterCoeffX_hxigain0)
#define			HallFilterCoeffX_hxpgain1		(0x0004 + HallFilterCoeffX_hxdgain0)
#define			HallFilterCoeffX_hxigain1		(0x0004 + HallFilterCoeffX_hxpgain1)
#define			HallFilterCoeffX_hxdgain1		(0x0004 + HallFilterCoeffX_hxigain1)
#define			HallFilterCoeffX_hxgain0		(0x0004 + HallFilterCoeffX_hxdgain1)
#define			HallFilterCoeffX_hxgain1		(0x0004 + HallFilterCoeffX_hxgain0)

#define			HallFilterCoeffX_hxsb			(0x0004 + HallFilterCoeffX_hxgain1)
#define			HallFilterCoeffX_hxsc			(0x0004 + HallFilterCoeffX_hxsb)
#define			HallFilterCoeffX_hxsa			(0x0004 + HallFilterCoeffX_hxsc)

#define			HallFilterCoeffX_hxob			(0x0004 + HallFilterCoeffX_hxsa)
#define			HallFilterCoeffX_hxoc			(0x0004 + HallFilterCoeffX_hxob)
#define			HallFilterCoeffX_hxod			(0x0004 + HallFilterCoeffX_hxoc)
#define			HallFilterCoeffX_hxoe			(0x0004 + HallFilterCoeffX_hxod)
#define			HallFilterCoeffX_hxoa			(0x0004 + HallFilterCoeffX_hxoe)
#define			HallFilterCoeffX_hxpb			(0x0004 + HallFilterCoeffX_hxoa)
#define			HallFilterCoeffX_hxpc			(0x0004 + HallFilterCoeffX_hxpb)
#define			HallFilterCoeffX_hxpd			(0x0004 + HallFilterCoeffX_hxpc)
#define			HallFilterCoeffX_hxpe			(0x0004 + HallFilterCoeffX_hxpd)
#define			HallFilterCoeffX_hxpa			(0x0004 + HallFilterCoeffX_hxpe)

#define		HallFilterCoeffY				0x812c
				// HallFilterCoeff.h  DM_HFC_t
#define			HallFilterCoeffY_HYIGAIN		(0x0000 + HallFilterCoeffY)
#define			HallFilterCoeffY_GYROYOUTGAIN	(0x0004 + HallFilterCoeffY_HYIGAIN)
#define			HallFilterCoeffY_HYOFFGAIN		(0x0004 + HallFilterCoeffY_GYROYOUTGAIN)

#define			HallFilterCoeffY_hyiab			(0x0004 + HallFilterCoeffY_HYOFFGAIN)
#define			HallFilterCoeffY_hyiac			(0x0004 + HallFilterCoeffY_hyiab)
#define			HallFilterCoeffY_hyiaa			(0x0004 + HallFilterCoeffY_hyiac)
#define			HallFilterCoeffY_hyibb			(0x0004 + HallFilterCoeffY_hyiaa)
#define			HallFilterCoeffY_hyibc			(0x0004 + HallFilterCoeffY_hyibb)
#define			HallFilterCoeffY_hyiba			(0x0004 + HallFilterCoeffY_hyibc)
#define			HallFilterCoeffY_hydab			(0x0004 + HallFilterCoeffY_hyiba)
#define			HallFilterCoeffY_hydac			(0x0004 + HallFilterCoeffY_hydab)
#define			HallFilterCoeffY_hydaa			(0x0004 + HallFilterCoeffY_hydac)
#define			HallFilterCoeffY_hydbb			(0x0004 + HallFilterCoeffY_hydaa)
#define			HallFilterCoeffY_hydbc			(0x0004 + HallFilterCoeffY_hydbb)
#define			HallFilterCoeffY_hydba			(0x0004 + HallFilterCoeffY_hydbc)
#define			HallFilterCoeffY_hydcc			(0x0004 + HallFilterCoeffY_hydba)
#define			HallFilterCoeffY_hydcb			(0x0004 + HallFilterCoeffY_hydcc)
#define			HallFilterCoeffY_hydca			(0x0004 + HallFilterCoeffY_hydcb)
#define			HallFilterCoeffY_hypgain0		(0x0004 + HallFilterCoeffY_hydca)
#define			HallFilterCoeffY_hyigain0		(0x0004 + HallFilterCoeffY_hypgain0)
#define			HallFilterCoeffY_hydgain0		(0x0004 + HallFilterCoeffY_hyigain0)
#define			HallFilterCoeffY_hypgain1		(0x0004 + HallFilterCoeffY_hydgain0)
#define			HallFilterCoeffY_hyigain1		(0x0004 + HallFilterCoeffY_hypgain1)
#define			HallFilterCoeffY_hydgain1		(0x0004 + HallFilterCoeffY_hyigain1)
#define			HallFilterCoeffY_hygain0		(0x0004 + HallFilterCoeffY_hydgain1)
#define			HallFilterCoeffY_hygain1		(0x0004 + HallFilterCoeffY_hygain0)
#define			HallFilterCoeffY_hysb			(0x0004 + HallFilterCoeffY_hygain1)
#define			HallFilterCoeffY_hysc			(0x0004 + HallFilterCoeffY_hysb)
#define			HallFilterCoeffY_hysa			(0x0004 + HallFilterCoeffY_hysc)
#define			HallFilterCoeffY_hyob			(0x0004 + HallFilterCoeffY_hysa)
#define			HallFilterCoeffY_hyoc			(0x0004 + HallFilterCoeffY_hyob)
#define			HallFilterCoeffY_hyod			(0x0004 + HallFilterCoeffY_hyoc)
#define			HallFilterCoeffY_hyoe			(0x0004 + HallFilterCoeffY_hyod)
#define			HallFilterCoeffY_hyoa			(0x0004 + HallFilterCoeffY_hyoe)
#define			HallFilterCoeffY_hypb			(0x0004 + HallFilterCoeffY_hyoa)
#define			HallFilterCoeffY_hypc			(0x0004 + HallFilterCoeffY_hypb)
#define			HallFilterCoeffY_hypd			(0x0004 + HallFilterCoeffY_hypc)
#define			HallFilterCoeffY_hype			(0x0004 + HallFilterCoeffY_hypd)
#define			HallFilterCoeffY_hypa			(0x0004 + HallFilterCoeffY_hype)

#define		HallFilterLimitX				0x81c8
#define		HallFilterLimitY				0x81e0
#define		HallFilterShiftX				0x81f8
#define		HallFilterShiftY				0x81fe

#define		HF_MIXING						0x8214
#define			HF_hx45x						(0x0000 + HF_MIXING	)		//0x008005E4 : HallMixingCoeff.hx45x
#define			HF_hx45y						(0x0004 + HF_MIXING	)		//0x008005E8 : HallMixingCoeff.hx45y
#define			HF_hy45y						(0x0008 + HF_MIXING	)		//0x008005EC : HallMixingCoeff.hy45y
#define			HF_hy45x						(0x000C + HF_MIXING	)		//0x008005F0 : HallMixingCoeff.hy45x
#define			HF_ShiftX						(0x0010 + HF_MIXING )

#define		HAL_LN_CORRECT					0x8228
#define			HAL_LN_COEFAX					(0x0000 + HAL_LN_CORRECT)		//0x00800564 : HallLinearCorrAX.zone_coef[6]
#define			HAL_LN_COEFBX					(0x000C + HAL_LN_COEFAX	)	//0x00800570 : HallLinearCorrBX.zone_coef[6]
#define			HAL_LN_ZONEX					(0x000C + HAL_LN_COEFBX	)	//0x0080057C : HallLinearZoneX.zone_area[5]
#define			HAL_LN_COEFAY					(0x000A + HAL_LN_ZONEX	)	//0x00800586 : HallLinearCorrAY.zone_coef[6]
#define			HAL_LN_COEFBY					(0x000C + HAL_LN_COEFAY	)	//0x00800592 : HallLinearCorrBY.zone_coef[6]
#define			HAL_LN_ZONEY					(0x000C + HAL_LN_COEFBY	)	//0x0080059E : HallLinearZoneY.zone_area[5]

#define		GyroFilterTableX				0x8270
				// GyroFilterCoeff.h  DM_GFC_t
#define			GyroFilterTableX_gx45x			(0x0000 + GyroFilterTableX)
#define			GyroFilterTableX_gx45y			(0x0004 + GyroFilterTableX_gx45x)
#define			GyroFilterTableX_gxgyro			(0x0004 + GyroFilterTableX_gx45y)
#define			GyroFilterTableX_gxsengen		(0x0004 + GyroFilterTableX_gxgyro)
#define			GyroFilterTableX_gxl1b			(0x0004 + GyroFilterTableX_gxsengen)
#define			GyroFilterTableX_gxl1c			(0x0004 + GyroFilterTableX_gxl1b)
#define			GyroFilterTableX_gxl1a			(0x0004 + GyroFilterTableX_gxl1c)
#define			GyroFilterTableX_gxl2b			(0x0004 + GyroFilterTableX_gxl1a)
#define			GyroFilterTableX_gxl2c			(0x0004 + GyroFilterTableX_gxl2b)
#define			GyroFilterTableX_gxl2a			(0x0004 + GyroFilterTableX_gxl2c)
#define			GyroFilterTableX_gxigain		(0x0004 + GyroFilterTableX_gxl2a)
#define			GyroFilterTableX_gxh1b			(0x0004 + GyroFilterTableX_gxigain)
#define			GyroFilterTableX_gxh1c			(0x0004 + GyroFilterTableX_gxh1b)
#define			GyroFilterTableX_gxh1a			(0x0004 + GyroFilterTableX_gxh1c)
#define			GyroFilterTableX_gxk1b			(0x0004 + GyroFilterTableX_gxh1a)
#define			GyroFilterTableX_gxk1c			(0x0004 + GyroFilterTableX_gxk1b)
#define			GyroFilterTableX_gxk1a			(0x0004 + GyroFilterTableX_gxk1c)
#define			GyroFilterTableX_gxgain			(0x0004 + GyroFilterTableX_gxk1a)
#define			GyroFilterTableX_gxzoom			(0x0004 + GyroFilterTableX_gxgain)
#define			GyroFilterTableX_gxlenz			(0x0004 + GyroFilterTableX_gxzoom)
#define			GyroFilterTableX_gxt2b			(0x0004 + GyroFilterTableX_gxlenz)
#define			GyroFilterTableX_gxt2c			(0x0004 + GyroFilterTableX_gxt2b)
#define			GyroFilterTableX_gxt2a			(0x0004 + GyroFilterTableX_gxt2c)
#define			GyroFilterTableX_afzoom			(0x0004 + GyroFilterTableX_gxt2a)

#define			GyroFilterTableY				0x82D0
				// GyroFilterCoeff.h  DM_GFC_t
#define			GyroFilterTableY_gy45y			(0x0000 + GyroFilterTableY)
#define			GyroFilterTableY_gy45x			(0x0004 + GyroFilterTableY_gy45y)
#define			GyroFilterTableY_gygyro			(0x0004 + GyroFilterTableY_gy45x)
#define			GyroFilterTableY_gysengen		(0x0004 + GyroFilterTableY_gygyro)
#define			GyroFilterTableY_gyl1b			(0x0004 + GyroFilterTableY_gysengen)
#define			GyroFilterTableY_gyl1c			(0x0004 + GyroFilterTableY_gyl1b)
#define			GyroFilterTableY_gyl1a			(0x0004 + GyroFilterTableY_gyl1c)
#define			GyroFilterTableY_gyl2b			(0x0004 + GyroFilterTableY_gyl1a)
#define			GyroFilterTableY_gyl2c			(0x0004 + GyroFilterTableY_gyl2b)
#define			GyroFilterTableY_gyl2a			(0x0004 + GyroFilterTableY_gyl2c)
#define			GyroFilterTableY_gyigain		(0x0004 + GyroFilterTableY_gyl2a)
#define			GyroFilterTableY_gyh1b			(0x0004 + GyroFilterTableY_gyigain)
#define			GyroFilterTableY_gyh1c			(0x0004 + GyroFilterTableY_gyh1b)
#define			GyroFilterTableY_gyh1a			(0x0004 + GyroFilterTableY_gyh1c)
#define			GyroFilterTableY_gyk1b			(0x0004 + GyroFilterTableY_gyh1a)
#define			GyroFilterTableY_gyk1c			(0x0004 + GyroFilterTableY_gyk1b)
#define			GyroFilterTableY_gyk1a			(0x0004 + GyroFilterTableY_gyk1c)
#define			GyroFilterTableY_gygain			(0x0004 + GyroFilterTableY_gyk1a)
#define			GyroFilterTableY_gyzoom			(0x0004 + GyroFilterTableY_gygain)
#define			GyroFilterTableY_gylenz			(0x0004 + GyroFilterTableY_gyzoom)
#define			GyroFilterTableY_gyt2b			(0x0004 + GyroFilterTableY_gylenz)
#define			GyroFilterTableY_gyt2c			(0x0004 + GyroFilterTableY_gyt2b)
#define			GyroFilterTableY_gyt2a			(0x0004 + GyroFilterTableY_gyt2c)
#define			GyroFilterTableY_afzoom			(0x0004 + GyroFilterTableY_gyt2a)

#define			Gyro_Limiter_X				0x8330
#define			Gyro_Limiter_Y				0x8334
#define			Gyro_ShiftX_RG				0x8338
#define			Gyro_ShiftY_RG				0x833C

#define		MeasureFilterA_Coeff			0x8380
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterA_Coeff_b1			(0x0000 + MeasureFilterA_Coeff)
#define			MeasureFilterA_Coeff_c1			(0x0004 + MeasureFilterA_Coeff_b1)
#define			MeasureFilterA_Coeff_a1			(0x0004 + MeasureFilterA_Coeff_c1)
#define			MeasureFilterA_Coeff_b2			(0x0004 + MeasureFilterA_Coeff_a1)
#define			MeasureFilterA_Coeff_c2			(0x0004 + MeasureFilterA_Coeff_b2)
#define			MeasureFilterA_Coeff_a2			(0x0004 + MeasureFilterA_Coeff_c2)

#define		MeasureFilterB_Coeff			0x8398
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterB_Coeff_b1			(0x0000 + MeasureFilterB_Coeff)
#define			MeasureFilterB_Coeff_c1			(0x0004 + MeasureFilterB_Coeff_b1)
#define			MeasureFilterB_Coeff_a1			(0x0004 + MeasureFilterB_Coeff_c1)
#define			MeasureFilterB_Coeff_b2			(0x0004 + MeasureFilterB_Coeff_a1)
#define			MeasureFilterB_Coeff_c2			(0x0004 + MeasureFilterB_Coeff_b2)
#define			MeasureFilterB_Coeff_a2			(0x0004 + MeasureFilterB_Coeff_c2)

#define		OpenLoopAF_FSTVAL				0x8508

#define		CommandDecodeTable				0x8568
				// Command.cpp  CommandTable in Rom
#define			CommandDecodeTable_08			(0x0020 + CommandDecodeTable)

//==============================================================================
//IO
//==============================================================================
// System Control配置アドレス
#define 	PERICLKON						0xD00000
#define 	SYSDSP_DSPDIV					0xD00014
#define 	IOPLEV							0xD00020
#define 	IOPDIR							0xD00024
#define 	SYSDSP_SOFTRES					0xD0006C
#define 	SYSDSP_DACI						0xD00088
#define 	SYSDSP_OPGSEL					0xD0008C
#define 	OSCRSEL							0xD00090
#define 	OSCCURSEL						0xD00094
#define 	FRQTRM							0xD00098
#define 	SYSDSP_REMAP					0xD000AC
#define 	OSCCNT							0xD000D4
#define 	SYSDSP_CVER						0xD00100
#define 	IOPLEVR							0xD00104
#define 	OSCCKCNT						0xD00108

#define 	ADDA_FSCNT						0xD01004
#define 	ADDA_FSCTRL						0xD01008
#define 	ADDA_ADDAINT					0xD0100C
#define 	ADDA_ADE						0xD01010
#define 	ADDA_ADAV						0xD01014
#define 	ADDA_ADORDER					0xD01018
#define 	ADDA_EXTEND						0xD0101C
#define 	ADDA_AD0O						0xD01020
#define 	ADDA_AD1O						0xD01024
#define 	ADDA_AD2O						0xD01028
#define 	ADDA_AD3O						0xD0102C

#define 	ADDA_DASELW						0xD01040
#define 	ADDA_DASU						0xD01044
#define 	ADDA_DAHD						0xD01048
#define 	ADDA_DASWAP						0xD0104C
#define 	ADDA_DASEL						0xD01050
#define		HLXO							0x00000001			// D/A Converter Channel Select HLXO
#define		HLYO							0x00000002			// D/A Converter Channel Select HLYO
#define		HLXBO							0x00000004			// D/A Converter Channel Select HLXBO
#define		HLYBO							0x00000008			// D/A Converter Channel Select HLYBO
#define		HLAFO							0x00000010			// D/A Converter Channel Select HLAFO
#define		HLAFBO							0x00000020			// D/A Converter Channel Select HLAFBO

#define 	ADDA_DAO						0xD01054

// PWM I/F配置アドレス
#define 	OISDRVFC1						0xD02100
#define 	OISDRVFC4						0xD0210C
#define 	OISDRVFC5						0xD02110
#define 	OISDRVFC6						0xD02114
#define 	OISDRVFC7						0xD02118
#define 	OISDRVFC8						0xD0211C
#define 	OISDRVFC9						0xD02120

#define 	DRVCH1SEL						0xD02128
#define 	DRVCH2SEL						0xD0212C

#define 	OISGAINAM						0xD02190
#define 	OISOFSTAM						0xD02194
#define 	OISGAINBM						0xD02198
#define 	OISOFSTBM						0xD0219C

#define 	AFDRVFC1						0xD02200
#define 	AFDRVFC4						0xD0220C
#define 	AFDRVFC5						0xD02210
#define 	AFDRVFC6						0xD02214
#define 	AFDRVFC7						0xD02218

#define 	DRVCH3SEL						0xD02220

#define 	AFGAINM							0xD02290
#define 	AFSOFSTM						0xD02294

//Periphral
#define 	ROMINFO							0xE0500C

// E2PROM 配置アドレス
#define 	E2P_RDAT						0xE07000
#define 	E2P_ADR							0xE07008
#define 	E2P_ASCNT						0xE0700C
#define 	E2P_CMD							0xE07010
#define 	E2P_WPB							0xE07014
#define 	E2P_INT							0xE07018

#define 	E2P_WDAT00						0xE07040
#define 	E2P_WDAT01						0xE07044
#define 	E2P_WDAT02						0xE07048
#define 	E2P_WDAT03						0xE0704C
#define 	E2P_WDAT04						0xE07050
#define 	E2P_WDAT05						0xE07054
#define 	E2P_WDAT06						0xE07058
#define 	E2P_WDAT07						0xE0705C
#define 	E2P_WDAT08						0xE07060
#define 	E2P_WDAT09						0xE07064
#define 	E2P_WDAT10						0xE07068
#define 	E2P_WDAT11						0xE0706C
#define 	E2P_WDAT12						0xE07070
#define 	E2P_WDAT13						0xE07074
#define 	E2P_WDAT14						0xE07078
#define 	E2P_WDAT15						0xE0707C
#define 	E2P_DFG							0xE07080

#define 	E2P_RSTB						0xE074CC
#define 	E2P_UNLK_CODE1					0xE07554
#define 	E2P_CLKON						0xE07664
#define 	E2P_UNLK_CODE2					0xE07AA8
#define 	E2P_UNLK_CODE3					0xE07CCC