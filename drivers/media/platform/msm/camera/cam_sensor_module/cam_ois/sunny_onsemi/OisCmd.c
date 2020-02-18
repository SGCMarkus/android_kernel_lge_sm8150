/**
 * @brief		OIS system command for LC898123 F40
 *
 * @author		Copyright (C) 2016, ON Semiconductor, all right reserved.
 *
 * @file		OisCmd.c
 * @date		svn:$Date:: 2016-06-22 10:57:58 +0900#$
 * @version	svn:$Revision: 59 $
 * @attention
 **/

//**************************
//	Include Header File
//**************************
#define		__OISCMD__
#define		SEL_SHIFT_COR

#include	"OisLc898123F40.h"
#include 	"PhoneUpdate.h"

#define 	RdStatus 		F40_RdStatus
#define 	ReadCalDataF40 	F40_ReadCalData

stAdjPar	StAdjPar;		//!< Calibration data
stPosOff	StPosOff;		//!< Execute Command Parameter
stAclVal	StAclVal;		//!< Execute Command Parameter

//****************************************************
//	CUSTOMER NECESSARY CREATING LIST
//****************************************************
/* for I2C communication */
extern	int32_t RamWrite32A(uint32_t RamAddr, uint32_t RamData);
extern 	int32_t RamRead32A(uint32_t RamAddr, uint32_t *ReadData);
/* for Wait timer [Need to adjust for your system] */
extern void WitTim(unsigned short	UsWitTim);

//**************************
//	extern  Function LIST
//**************************
uint32_t	UlBufDat[64];							//!< Calibration data write buffer(256 bytes)

//**************************
//	Local Function Prototype
//**************************
void		MesFil(uint8_t);							//!< Measure Filter Setting
void		MeasureStart(uint32_t, uint32_t, uint32_t);	//!< Measure Start Function
void		MeasureWait(void);						//!< Measure Wait
void		MemoryClear(uint16_t, uint16_t);			//!< Memory Cloear
void		SetWaitTime(uint16_t); 					//!< Set Wait Timer
uint32_t	TneGvc(uint8_t	uc_mode);
void		SetTransDataAdr(uint16_t , uint32_t );	//!< Hall VC Offset Adjust
uint8_t		GetInfomation(DSPVER* Info);
void		ClrMesFil(void);

//**************************
//	define
//**************************
//#define 	HALL_ADJ		0
#define 	LOOPGAIN		1
#define 	THROUGH			2
#define 	NOISE			3
#define		OSCCHK			4
#define		GAINCURV		5
#define		SELFTEST		6

//********************************************************************************
// Function Name 	: MemClr
// Retun Value		: void
// Argment Value	: Clear Target Pouint32_ter, Clear Byte Number
// Explanation		: Memory Clear Function
// History			: First edition
//********************************************************************************
void	MemClr(uint8_t	*NcTgtPtr, uint16_t	UsClrSiz)
{
	uint16_t	UsClrIdx;

	for (UsClrIdx = 0; UsClrIdx < UsClrSiz; UsClrIdx++)
	{
		*NcTgtPtr = 0;
		NcTgtPtr++;
	}
}


//********************************************************************************
// Function Name 	: MesFil
// Retun Value		: NON
// Argment Value	: Measure Filter Mode
// Explanation		: Measure Filter Setting Function
// History			: First edition
//********************************************************************************
void	MesFil(uint8_t	UcMesMod)		// 20.019kHz
{
	uint32_t	UlMeasFilaA = 0, UlMeasFilaB = 0, UlMeasFilaC = 0;
	uint32_t	UlMeasFilbA = 0, UlMeasFilbB = 0, UlMeasFilbC = 0;

	if(!UcMesMod) {								// Hall Bias&Offset Adjust

		UlMeasFilaA	= 0x02F19B01;	// LPF 150Hz
		UlMeasFilaB	= 0x02F19B01;
		UlMeasFilaC	= 0x7A1CC9FF;
		UlMeasFilbA	= 0x7FFFFFFF;	// Through
		UlMeasFilbB	= 0x00000000;
		UlMeasFilbC	= 0x00000000;

	} else if(UcMesMod == LOOPGAIN) {				// Loop Gain Adjust

		UlMeasFilaA	= 0x115CC757;	// LPF1000Hz
		UlMeasFilaB	= 0x115CC757;
		UlMeasFilaC	= 0x5D467153;
		UlMeasFilbA	= 0x7F667431;	// HPF30Hz
		UlMeasFilbB	= 0x80998BCF;
		UlMeasFilbC	= 0x7ECCE863;

	} else if(UcMesMod == THROUGH) {				// for Through

		UlMeasFilaA	= 0x7FFFFFFF;	// Through
		UlMeasFilaB	= 0x00000000;
		UlMeasFilaC	= 0x00000000;
		UlMeasFilbA	= 0x7FFFFFFF;	// Through
		UlMeasFilbB	= 0x00000000;
		UlMeasFilbC	= 0x00000000;

	} else if(UcMesMod == NOISE) {				// SINE WAVE TEST for NOISE

		UlMeasFilaA	= 0x02F19B01;	// LPF150Hz
		UlMeasFilaB	= 0x02F19B01;
		UlMeasFilaC	= 0x7A1CC9FF;
		UlMeasFilbA	= 0x02F19B01;	// LPF150Hz
		UlMeasFilbB	= 0x02F19B01;
		UlMeasFilbC	= 0x7A1CC9FF;

	} else if(UcMesMod == OSCCHK) {
		UlMeasFilaA	= 0x05C141BB;	// LPF300Hz
		UlMeasFilaB	= 0x05C141BB;
		UlMeasFilaC	= 0x747D7C88;
		UlMeasFilbA	= 0x05C141BB;	// LPF300Hz
		UlMeasFilbB	= 0x05C141BB;
		UlMeasFilbC	= 0x747D7C88;

	} else if(UcMesMod == SELFTEST) {				// GYRO SELF TEST

		UlMeasFilaA	= 0x115CC757;	// LPF1000Hz
		UlMeasFilaB	= 0x115CC757;
		UlMeasFilaC	= 0x5D467153;
		UlMeasFilbA	= 0x7FFFFFFF;	// Through
		UlMeasFilbB	= 0x00000000;
		UlMeasFilbC	= 0x00000000;

	}

	RamWrite32A (MeasureFilterA_Coeff_a1	, UlMeasFilaA);
	RamWrite32A (MeasureFilterA_Coeff_b1	, UlMeasFilaB);
	RamWrite32A (MeasureFilterA_Coeff_c1	, UlMeasFilaC);

	RamWrite32A (MeasureFilterA_Coeff_a2	, UlMeasFilbA);
	RamWrite32A (MeasureFilterA_Coeff_b2	, UlMeasFilbB);
	RamWrite32A (MeasureFilterA_Coeff_c2	, UlMeasFilbC);

	RamWrite32A (MeasureFilterB_Coeff_a1	, UlMeasFilaA);
	RamWrite32A (MeasureFilterB_Coeff_b1	, UlMeasFilaB);
	RamWrite32A (MeasureFilterB_Coeff_c1	, UlMeasFilaC);

	RamWrite32A (MeasureFilterB_Coeff_a2	, UlMeasFilbA);
	RamWrite32A (MeasureFilterB_Coeff_b2	, UlMeasFilbB);
	RamWrite32A (MeasureFilterB_Coeff_c2	, UlMeasFilbC);
}

//********************************************************************************
// Function Name 	: ClrMesFil
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Clear Measure Filter Function
// History			: First edition
//********************************************************************************
void	ClrMesFil(void)
{
	RamWrite32A (MeasureFilterA_Delay_z11	, 0);
	RamWrite32A (MeasureFilterA_Delay_z12	, 0);

	RamWrite32A (MeasureFilterA_Delay_z21	, 0);
	RamWrite32A (MeasureFilterA_Delay_z22	, 0);

	RamWrite32A (MeasureFilterB_Delay_z11	, 0);
	RamWrite32A (MeasureFilterB_Delay_z12	, 0);

	RamWrite32A (MeasureFilterB_Delay_z21	, 0);
	RamWrite32A (MeasureFilterB_Delay_z22	, 0);
}

//********************************************************************************
// Function Name 	: MeasureStart
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition
//********************************************************************************
void	MeasureStart(uint32_t SlMeasureParameterNum, uint32_t SlMeasureParameterA, uint32_t SlMeasureParameterB)
{
	MemoryClear(StMeasFunc_SiSampleNum, sizeof(MeasureFunction_Type));
	RamWrite32A(StMeasFunc_MFA_SiMax1	, 0x80000000);					// Set Min
	RamWrite32A(StMeasFunc_MFB_SiMax2	, 0x80000000);					// Set Min
	RamWrite32A(StMeasFunc_MFA_SiMin1	, 0x7FFFFFFF);					// Set Max
	RamWrite32A(StMeasFunc_MFB_SiMin2	, 0x7FFFFFFF);					// Set Max

	SetTransDataAdr(StMeasFunc_MFA_PiMeasureRam1	, (uint32_t)SlMeasureParameterA);		// Set Measure Filter A Ram Address
	SetTransDataAdr(StMeasFunc_MFB_PiMeasureRam2	, (uint32_t)SlMeasureParameterB);		// Set Measure Filter B Ram Address
	RamWrite32A(StMeasFunc_MFA_SiSampleNumA	 	, 0);									// Clear Measure Counter
	RamWrite32A(StMeasFunc_MFB_SiSampleNumB	 	, 0);									// Clear Measure Counter
	RamWrite32A(StMeasFunc_PMC_UcPhaseMesMode		, 0);									// Set Phase Measure Mode
	ClrMesFil();																			// Clear Delay Ram
	SetWaitTime(50);
	RamWrite32A(StMeasFunc_MFB_SiSampleMaxB		, SlMeasureParameterNum);				// Set Measure Max Number
	RamWrite32A(StMeasFunc_MFA_SiSampleMaxA		, SlMeasureParameterNum);				// Set Measure Max Number
}

//********************************************************************************
// Function Name 	: MeasureWait
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Wait complete of Measure Function
// History			: First edition
//********************************************************************************
void	MeasureWait(void)
{
	uint32_t	SlWaitTimerStA, SlWaitTimerStB;
	uint16_t	UsTimeOut = 2000;

	do {
		RamRead32A(StMeasFunc_MFA_SiSampleMaxA, &SlWaitTimerStA);
		RamRead32A(StMeasFunc_MFB_SiSampleMaxB, &SlWaitTimerStB);
		UsTimeOut--;
	} while ((SlWaitTimerStA || SlWaitTimerStB) && UsTimeOut);

}

//********************************************************************************
// Function Name 	: MemoryClear
// Retun Value		: NON
// Argment Value	: Top pouint32_ter, Size
// Explanation		: Memory Clear Function
// History			: First edition
//********************************************************************************
void	MemoryClear(uint16_t UsSourceAddress, uint16_t UsClearSize)
{
	uint16_t	UsLoopIndex;

	for (UsLoopIndex = 0; UsLoopIndex < UsClearSize; ) {
		RamWrite32A(UsSourceAddress	, 	0x00000000);				// 4Byte
		UsSourceAddress += 4;
		UsLoopIndex += 4;
	}
}

//********************************************************************************
// Function Name 	: SetWaitTime
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Set Timer wait Function
// History			: First edition
//********************************************************************************
#define 	ONE_MSEC_COUNT	20			// 20.0195kHz * 20 ?à 1ms
void	SetWaitTime(uint16_t UsWaitTime)
{
	RamWrite32A(WaitTimerData_UiWaitCounter	, 0);
	RamWrite32A(WaitTimerData_UiTargetCount	, (uint32_t)(ONE_MSEC_COUNT * UsWaitTime));
}

//********************************************************************************
// Function Name 	: TneGvc
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Gyro VC offset
// History			: First edition
//********************************************************************************
#define 	GYROF_NUM		2048			// 2048times
#define 	GYROF_UPPER		0x0600			// ICM_20690?i?}500[dps]?A65.5[LSB/dps]?j
#define 	GYROF_LOWER		0xFA00			// ?}23.4[dps]
#define 	_GYROF_UPPER_		0x0DAC			// request from customers to change double margin
#define 	_GYROF_LOWER_		0xF254			//

uint32_t	TneGvc(uint8_t	uc_mode)
{
	uint32_t	UlRsltSts;
	uint32_t	SlMeasureParameterA, SlMeasureParameterB;
	uint32_t	SlMeasureParameterNum;
	UnllnVal	StMeasValueA, StMeasValueB;
	uint32_t	SlMeasureAveValueA, SlMeasureAveValueB;

	uint16_t	UsGzoVal;

	MesFil(THROUGH);					// Set Measure Filter

	SlMeasureParameterNum	=	GYROF_NUM;					// Measurement times

	if(uc_mode == 0){

		SlMeasureParameterA		=	GYRO_RAM_GX_ADIDAT;		// Set Measure RAM Address
		SlMeasureParameterB		=	GYRO_RAM_GY_ADIDAT;		// Set Measure RAM Address

	}else{
		SlMeasureParameterA		=	GYRO_ZRAM_GZ_ADIDAT;		// Set Measure RAM Address
		SlMeasureParameterB		=	GYRO_ZRAM_GZ_ADIDAT;		// Set Measure RAM Address
	}


	MeasureStart(SlMeasureParameterNum, SlMeasureParameterA, SlMeasureParameterB);					// Start measure

	MeasureWait();					// Wait complete of measurement

	RamRead32A(StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal);	// X axis
	RamRead32A(StMeasFunc_MFA_LLiIntegral1 + 4		, &StMeasValueA.StUllnVal.UlHigVal);
	RamRead32A(StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal);	// Y axis
	RamRead32A(StMeasFunc_MFB_LLiIntegral2 + 4		, &StMeasValueB.StUllnVal.UlHigVal);

	SlMeasureAveValueA = (uint32_t)((int64_t)StMeasValueA.UllnValue / SlMeasureParameterNum);
	SlMeasureAveValueB = (uint32_t)((int64_t)StMeasValueB.UllnValue / SlMeasureParameterNum);

	SlMeasureAveValueA = (SlMeasureAveValueA >> 16) & 0x0000FFFF;
	SlMeasureAveValueB = (SlMeasureAveValueB >> 16) & 0x0000FFFF;

	UlRsltSts = EXE_END;

	if(uc_mode == 0){

		StAdjPar.StGvcOff.UsGxoVal = (uint16_t)(SlMeasureAveValueA & 0x0000FFFF);		//Measure Result Store
		if(((int16_t)StAdjPar.StGvcOff.UsGxoVal > (int16_t)GYROF_UPPER) || ((int16_t)StAdjPar.StGvcOff.UsGxoVal < (int16_t)GYROF_LOWER)){
			UlRsltSts |= EXE_GXADJ;
		}
		RamWrite32A(GYRO_RAM_GXOFFZ, ((SlMeasureAveValueA << 16) & 0xFFFF0000));		// X axis Gyro offset

		StAdjPar.StGvcOff.UsGyoVal = (uint16_t)(SlMeasureAveValueB & 0x0000FFFF);		//Measure Result Store
		if(((int16_t)StAdjPar.StGvcOff.UsGyoVal > (int16_t)GYROF_UPPER) || ((int16_t)StAdjPar.StGvcOff.UsGyoVal < (int16_t)GYROF_LOWER)){
			UlRsltSts |= EXE_GYADJ;
		}
		RamWrite32A(GYRO_RAM_GYOFFZ, ((SlMeasureAveValueB << 16) & 0xFFFF0000));		// Y axis Gyro offset


		RamWrite32A(GYRO_RAM_GYROX_OFFSET, 0x00000000);			// X axis Drift Gyro offset
		RamWrite32A(GYRO_RAM_GYROY_OFFSET, 0x00000000);			// Y axis Drift Gyro offset
		RamWrite32A(GyroFilterDelayX_GXH1Z2, 0x00000000);		// X axis H1Z2 Clear
		RamWrite32A(GyroFilterDelayY_GYH1Z2, 0x00000000);		// Y axis H1Z2 Clear


	}else{
		UsGzoVal = (uint16_t)(SlMeasureAveValueA & 0x0000FFFF);		//Measure Result Store
		if(((int16_t)UsGzoVal > (int16_t)GYROF_UPPER) || ((int16_t)UsGzoVal < (int16_t)GYROF_LOWER)){
			UlRsltSts |= EXE_GZADJ;
		}
		RamWrite32A(GYRO_ZRAM_GZOFFZ, ((SlMeasureAveValueA << 16) & 0xFFFF0000));		// Z axis Gyro offset

		RamWrite32A(GyroRAM_Z_GYRO_OFFSET, 0x00000000);			// Z axis Drift Gyro offset
	}

	return(UlRsltSts);
}

//********************************************************************************
// Function Name 	: TneAvc
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Accel VC offset for All
// History			: First edition
//********************************************************************************
#define		EXE_ERROR		0x00000003L		// Adjust NG : Execution Failure
#define 	ACCLOF_NUM		4096				// 4096times
#define		SENSITIVITY		2048	// LSB/g
#define		PSENS_MARG		(2048 / 4)	// 1/4g
#define		POSTURETH_P		(SENSITIVITY - PSENS_MARG)	// LSB/g
#define		POSTURETH_M		(-POSTURETH_P)				// LSB/g
#define		ZEROG_MRGN_Z	(409 << 16)			// Zero G tolerance for Z
#define		ZEROG_MRGN_XY	(409 << 16)			// Zero G tolerance for XY
#define		ACCL_SENS		4096
#define		ACCL_SENS_M		-4096
uint32_t	TneAvc(uint8_t ucposture)
{
	uint32_t			UlRsltSts;
	uint32_t			SlMeasureParameterA, SlMeasureParameterB;
	uint32_t			SlMeasureParameterNum;
	UnllnVal			StMeasValueA, StMeasValueB;
	uint32_t			SlMeasureAveValueA, SlMeasureAveValueB;
	uint32_t			SlMeasureRetValueX, SlMeasureRetValueY, SlMeasureRetValueZ;
	uint8_t				i, j, k;
	uint32_t			SlDiff[3];

//--------------------------------------
// Initialize Calibration data
//--------------------------------------
	UlRsltSts = EXE_END;
	if(ucposture < 0x7f){
		for(i=0; i<2; i++)
		{
			MesFil(THROUGH);					// Set Measure Filter

			SlMeasureParameterNum	=	ACCLOF_NUM;					// Measurement times
			switch(i){
			case 0:
				SlMeasureParameterA		=	ACCLRAM_X_AC_ADIDAT;			// Set Measure RAM Address
				SlMeasureParameterB		=	ACCLRAM_Y_AC_ADIDAT;			// Set Measure RAM Address
				break;
			case 1:
				SlMeasureParameterA		=	ACCLRAM_Z_AC_ADIDAT;			// Set Measure RAM Address
				SlMeasureParameterB		=	ACCLRAM_Z_AC_ADIDAT;			// Set Measure RAM Address
				break;
			}

			MeasureStart(SlMeasureParameterNum, SlMeasureParameterA, SlMeasureParameterB);					// Start measure

			MeasureWait();					// Wait complete of measurement

			RamRead32A(StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal);
			RamRead32A(StMeasFunc_MFA_LLiIntegral1 + 4		, &StMeasValueA.StUllnVal.UlHigVal);
			RamRead32A(StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal);
			RamRead32A(StMeasFunc_MFB_LLiIntegral2 + 4		, &StMeasValueB.StUllnVal.UlHigVal);

			SlMeasureAveValueA = (uint32_t)((int64_t)StMeasValueA.UllnValue / SlMeasureParameterNum);
			SlMeasureAveValueB = (uint32_t)((int64_t)StMeasValueB.UllnValue / SlMeasureParameterNum);

			switch(i){
			case 0:
				SlMeasureRetValueX = SlMeasureAveValueA;
				SlMeasureRetValueY = SlMeasureAveValueB;
				break;
			case 1:
				SlMeasureRetValueZ = SlMeasureAveValueA;
				break;
			}

		}

		TRACE("VAL(X,Y,Z) pos = \t%08xh\t%08xh\t%08xh\t0x%02x \n",
			(unsigned int)SlMeasureRetValueX,
			(unsigned int)SlMeasureRetValueY,
			(unsigned int)SlMeasureRetValueZ,
			ucposture);
		if((SlMeasureRetValueZ < (uint32_t)(POSTURETH_P<<16)) && (ucposture == 0x10)){
				UlRsltSts = EXE_ERROR;
			TRACE(" POS14 [ERROR] \t%08xh < %08xh\n",
				(unsigned int)(SlMeasureRetValueZ), (unsigned int)(POSTURETH_P<<16));
		}else if((SlMeasureRetValueZ > (uint32_t)(POSTURETH_M<<16)) && (ucposture == 0x11)){
				UlRsltSts = EXE_ERROR;
			TRACE(" POS14 [ERROR] \t%08xh > %08xh\n",
				(unsigned int)(SlMeasureRetValueZ), (unsigned int)(POSTURETH_M<<16));
		}else{
			TRACE("DEBUG = \t%08xh\t \n", abs((uint32_t)(ACCL_SENS << 16) - abs(SlMeasureRetValueZ)));
			if(abs(SlMeasureRetValueX) > ZEROG_MRGN_XY)									UlRsltSts |= EXE_GXADJ;
			if(abs(SlMeasureRetValueY) > ZEROG_MRGN_XY)									UlRsltSts |= EXE_GYADJ;
			if(abs((uint32_t)(ACCL_SENS << 16) - abs(SlMeasureRetValueZ)) > ZEROG_MRGN_Z)	UlRsltSts |= EXE_GZADJ;
			if(UlRsltSts == EXE_END){
				StPosOff.UlAclOfSt |= 0x0000003F;
				TRACE("POS14(X,Y,Z) st = \t%08xh\t%08xh\t%08xh\t%08xh \n",
					(unsigned int)StPosOff.StPos.Pos[4][0],
					(unsigned int)StPosOff.StPos.Pos[4][1],
					(unsigned int)StPosOff.StPos.Pos[4][2],
					(unsigned int)StPosOff.UlAclOfSt);
				SlDiff[0] = SlMeasureRetValueX - (uint32_t)0;
				SlDiff[1] = SlMeasureRetValueY - (uint32_t)0;
				if(ucposture == 0x10){
					SlDiff[2] = SlMeasureRetValueZ - (uint32_t)(ACCL_SENS << 16);
				}else{
					SlDiff[2] = SlMeasureRetValueZ - (uint32_t)(ACCL_SENS_M << 16);
				}
				StPosOff.StPos.Pos[4][0] = SlDiff[0];
				StPosOff.StPos.Pos[4][1] = SlDiff[1];
				StPosOff.StPos.Pos[4][2] = SlDiff[2];
			}
		}
	}else{
		switch(ucposture){
		case 0x80:
			if(StPosOff.UlAclOfSt == 0x3fL){
				/*X offset*/
				StAclVal.StAccel.SlOffsetX = StPosOff.StPos.Pos[4][0];
				/*Y offset*/
				StAclVal.StAccel.SlOffsetY = StPosOff.StPos.Pos[4][1];
				/*Z offset*/
				StAclVal.StAccel.SlOffsetZ = StPosOff.StPos.Pos[4][2];
#ifdef DEBUG
		TRACE("ACLOFST(X,Y,Z) = \t%08xh\t%08xh\t%08xh \n",
		(unsigned int)StAclVal.StAccel.SlOffsetX,
		(unsigned int)StAclVal.StAccel.SlOffsetY,
		(unsigned int)StAclVal.StAccel.SlOffsetZ);
#endif //DEBUG

				RamWrite32A(ACCLRAM_X_AC_OFFSET, StAclVal.StAccel.SlOffsetX);	// X axis Accel offset
				RamWrite32A(ACCLRAM_Y_AC_OFFSET, StAclVal.StAccel.SlOffsetY);	// Y axis Accel offset
				RamWrite32A(ACCLRAM_Z_AC_OFFSET, StAclVal.StAccel.SlOffsetZ);	// Z axis Accel offset

				for(j=0; j < 6; j++){
					k = 4 * j;
					RamWrite32A(AcclFilDly_X + k, 0x00000000);			// X axis Accl LPF Clear
					RamWrite32A(AcclFilDly_Y + k, 0x00000000);			// Y axis Accl LPF Clear
					RamWrite32A(AcclFilDly_Z + k, 0x00000000);			// Z axis Accl LPF Clear
				}

			}else{
				UlRsltSts = EXE_ERROR;
			}
			break;
		case 0xFF:	/* RAM clear */
			MemClr((uint8_t *)&StPosOff, sizeof(stPosOff));	// Adjust Parameter Clear
			MemClr((uint8_t *)&StAclVal, sizeof(stAclVal));	// Adjust Parameter Clear
			break;
		}
	}

	TRACE(" Result = %08x\n",(uint32_t)UlRsltSts);
	return(UlRsltSts);

}

//********************************************************************************
// Function Name 	: RtnCen
// Retun Value		: Command Status
// Argment Value	: Command Parameter
// Explanation		: Return to center Command Function
// History			: First edition
//********************************************************************************
uint8_t	RtnCen(uint8_t	UcCmdPar)
{
	uint8_t	UcSndDat = FAILURE;

	if(!UcCmdPar){								// X,Y centering
		RamWrite32A(CMD_RETURN_TO_CENTER, BOTH_SRV_ON);
	}else if(UcCmdPar == XONLY_ON){				// only X centering
		RamWrite32A(CMD_RETURN_TO_CENTER, XAXS_SRV_ON);
	}else if(UcCmdPar == YONLY_ON){				// only Y centering
		RamWrite32A(CMD_RETURN_TO_CENTER, YAXS_SRV_ON);
	}else{											// Both off
		RamWrite32A(CMD_RETURN_TO_CENTER, BOTH_SRV_OFF);
	}

	do {
		UcSndDat = RdStatus(1);
	} while(UcSndDat == FAILURE);

//TRACE("RtnCen() = %02x\n", UcSndDat);
	return(UcSndDat);
}

//********************************************************************************
// Function Name 	: OisEna
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function
// History			: First edition
//********************************************************************************
void	OisEna(void)
{
	uint8_t	UcStRd = 1;

	RamWrite32A(CMD_OIS_ENABLE, OIS_ENABLE);
	while(UcStRd) {
		UcStRd = RdStatus(1);
	}
TRACE(" OisEna(Status) = %02x\n", UcStRd);
}

//********************************************************************************
// Function Name 	: OisDis
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Disable Control Function
// History			: First edition
//********************************************************************************
void	OisDis(void)
{
	uint8_t	UcStRd = 1;

	RamWrite32A(CMD_OIS_ENABLE, OIS_DISABLE);
	while(UcStRd) {
		UcStRd = RdStatus(1);
	}
	TRACE(" OisDis(Status) = %02x\n", UcStRd);
}

//********************************************************************************
// Function Name 	: SetTransDataAdr
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Trans Address for Data Function
// History			: First edition
//********************************************************************************
void	SetTransDataAdr(uint16_t UsLowAddress, uint32_t UlLowAdrBeforeTrans)
{
	UnDwdVal	StTrsVal;

	if(UlLowAdrBeforeTrans < 0x00009000){
		StTrsVal.StDwdVal.UsHigVal = (uint16_t)((UlLowAdrBeforeTrans & 0x0000F000) >> 8);
		StTrsVal.StDwdVal.UsLowVal = (uint16_t)(UlLowAdrBeforeTrans & 0x00000FFF);
	}else{
		StTrsVal.UlDwdVal = UlLowAdrBeforeTrans;
	}
//TRACE(" TRANS  ADR = %04xh, DAT = %08xh \n",UsLowAddress, StTrsVal.UlDwdVal);
	RamWrite32A(UsLowAddress	,	StTrsVal.UlDwdVal);

}

//********************************************************************************
// Function Name 	: SetChecSumCalib
// Retun Value		: SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Check Sum calculation and set to memory.
// History			: First edition 									2015.7.14
//********************************************************************************
void SetCheckSumCalib(void)
{
	uint32_t UiChkSum1 = 0,	UiChkSum2 = 0;

	do{
		UiChkSum1 += UlBufDat[ UiChkSum2++ ];							// ChckSum
	}while (UiChkSum2 < 63);											// 64*5 = 320 : NVR sector size
	UlBufDat[ HALL_CAL_CHECKSUM	 ]	= UiChkSum1;      									// CheckSum

}

//********************************************************************************
// Function Name 	: WrGyroOffsetData
// Retun Value		: 0:OK, 1:NG
// Argment Value	: NON
// Explanation		: Flash Write Gyro offset Data Function
// History			: First edition
//********************************************************************************
uint8_t	WrGyroOffsetData(void)
{
	uint32_t	UlFctryX, UlFctryY;
	uint32_t	UlCurrX, UlCurrY;
	uint32_t	UlGofX, UlGofY;
	uint32_t	UiChkSum1,	UiChkSum2;
	uint32_t	UlSrvStat,	UlOisStat;
	uint8_t		ans;
//------------------------------------------------------------------------------------------------
// Servo Off & Get OIS enable status (for F40)
//------------------------------------------------------------------------------------------------
	RamRead32A(CMD_RETURN_TO_CENTER, &UlSrvStat);
	RamRead32A(CMD_OIS_ENABLE, &UlOisStat);
	RtnCen(BOTH_OFF);													// Both OFF
//------------------------------------------------------------------------------------------------
// Backup ALL Calibration data
//------------------------------------------------------------------------------------------------
	ReadCalDataF40(UlBufDat, &UiChkSum2);
//------------------------------------------------------------------------------------------------
// Sector erase NVR2 Calibration area
//------------------------------------------------------------------------------------------------
	ans = EraseCalDataF40();
	if (ans == 0){
//------------------------------------------------------------------------------------------------
// Set Calibration data
//------------------------------------------------------------------------------------------------
		RamRead32A( GYRO_RAM_GXOFFZ, &UlGofX);
		RamWrite32A(StCaliData_SiGyroOffset_X,	UlGofX);

		RamRead32A( GYRO_RAM_GYOFFZ, &UlGofY);
		RamWrite32A(StCaliData_SiGyroOffset_Y,	UlGofY);

		UlCurrX		= UlBufDat[ GYRO_OFFSET_X ];
		UlCurrY		= UlBufDat[ GYRO_OFFSET_Y ];
		UlFctryX	= UlBufDat[ GYRO_FCTRY_OFST_X ];
		UlFctryY	= UlBufDat[ GYRO_FCTRY_OFST_Y ];

		if(UlFctryX == 0xFFFFFFFF)
			UlBufDat[ GYRO_FCTRY_OFST_X ] = UlCurrX;

		if(UlFctryY == 0xFFFFFFFF)
			UlBufDat[ GYRO_FCTRY_OFST_Y ] = UlCurrY;

		UlBufDat[ GYRO_OFFSET_X ] = UlGofX;
		UlBufDat[ GYRO_OFFSET_Y ] = UlGofY;

//------------------------------------------------------------------------------------------------
// Write gyro angle data
//------------------------------------------------------------------------------------------------
		WriteCalDataF40(UlBufDat, &UiChkSum1);
//------------------------------------------------------------------------------------------------
// Calculate calibration data checksum
//------------------------------------------------------------------------------------------------
		ReadCalDataF40(UlBufDat, &UiChkSum2);

		if(UiChkSum1 != UiChkSum2){
			TRACE("CheckSum error\n");
			TRACE("UiChkSum1 = %08X, UiChkSum2 = %08X\n",(uint32_t)UiChkSum1, (uint32_t)UiChkSum2);
			ans = 0x10;
		}
//		TRACE("CheckSum OK\n");
//		TRACE("UiChkSum1 = %08X, UiChkSum2 = %08X\n",(uint32_t)UiChkSum1, (uint32_t)UiChkSum2);
	}
//------------------------------------------------------------------------------------------------
// Resume
//------------------------------------------------------------------------------------------------
	if(!UlSrvStat) {
		RtnCen(BOTH_OFF);
	} else if(UlSrvStat == 3) {
		RtnCen(BOTH_ON);
	} else {
		RtnCen(UlSrvStat);
	}

	if(UlOisStat != 0)               OisEna();

	return(ans);															// CheckSum OK

}

//********************************************************************************
// Function Name 	: WrGyroAcclOffsetData
// Retun Value		: 0:OK, 1:NG
// Argment Value	: NON
// Explanation		: Flash Write Gyro offset Data Function
// History			: First edition
//********************************************************************************
uint8_t	WrGyroAcclOffsetData(void)
{
	uint32_t	UlFctryXg, UlFctryYg, UlFctryZg, UlFctryXa, UlFctryYa, UlFctryZa;
	uint32_t	UlCurrXg, UlCurrYg, UlCurrZg, UlCurrXa, UlCurrYa, UlCurrZa;
	uint32_t	UlGofXg, UlGofYg, UlGofZg, UlGofXa, UlGofYa, UlGofZa;
	uint32_t	UiChkSum1,	UiChkSum2;
	uint32_t	UlSrvStat,	UlOisStat;
	uint8_t	ans;
//------------------------------------------------------------------------------------------------
// Servo Off & Get OIS enable status (for F40)
//------------------------------------------------------------------------------------------------
	RamRead32A(CMD_RETURN_TO_CENTER, &UlSrvStat);
	RamRead32A(CMD_OIS_ENABLE, &UlOisStat);
	RtnCen(BOTH_OFF);													// Both OFF
//------------------------------------------------------------------------------------------------
// Backup ALL Calibration data
//------------------------------------------------------------------------------------------------
	ReadCalDataF40(UlBufDat, &UiChkSum2);
//------------------------------------------------------------------------------------------------
// Sector erase NVR2 Calibration area
//------------------------------------------------------------------------------------------------
	ans = EraseCalDataF40();
	if (ans == 0){
//------------------------------------------------------------------------------------------------
// Set Calibration data
//------------------------------------------------------------------------------------------------
		RamRead32A( GYRO_RAM_GXOFFZ, &UlGofXg);
		RamWrite32A(StCaliData_SiGyroOffset_X,	UlGofXg);
		RamRead32A( GYRO_RAM_GYOFFZ, &UlGofYg);
		RamWrite32A(StCaliData_SiGyroOffset_Y,	UlGofYg);

		RamRead32A( GYRO_ZRAM_GZOFFZ, &UlGofZg);
		RamRead32A( ACCLRAM_X_AC_OFFSET, &UlGofXa);
		RamRead32A( ACCLRAM_Y_AC_OFFSET, &UlGofYa);
		RamRead32A( ACCLRAM_Z_AC_OFFSET, &UlGofZa);

		UlCurrXg = UlBufDat[ GYRO_OFFSET_X ];
		UlCurrYg = UlBufDat[ GYRO_OFFSET_Y ];
		UlCurrZg = UlBufDat[ GYRO_OFFSET_Z ];
		UlCurrXa = UlBufDat[ ACCL_OFFSET_X ];
		UlCurrYa = UlBufDat[ ACCL_OFFSET_Y ];
		UlCurrZa = UlBufDat[ ACCL_OFFSET_Z ];

		UlFctryXg = UlBufDat[ GYRO_FCTRY_OFST_X ];
		UlFctryYg = UlBufDat[ GYRO_FCTRY_OFST_Y ];
		UlFctryZg = UlBufDat[ GYRO_FCTRY_OFST_Z ];
		UlFctryXa = UlBufDat[ ACCL_FCTRY_OFST_X ];
		UlFctryYa = UlBufDat[ ACCL_FCTRY_OFST_Y ];
		UlFctryZa = UlBufDat[ ACCL_FCTRY_OFST_Z ];

		if(UlFctryXg == 0xFFFFFFFF)	UlBufDat[ GYRO_FCTRY_OFST_X ] = UlCurrXg;
		if(UlFctryYg == 0xFFFFFFFF)	UlBufDat[ GYRO_FCTRY_OFST_Y ] = UlCurrYg;
		if(UlFctryZg == 0xFFFFFFFF)	UlBufDat[ GYRO_FCTRY_OFST_Z ] = UlCurrZg;
		if(UlFctryXa == 0xFFFFFFFF)	UlBufDat[ ACCL_FCTRY_OFST_X ] = UlCurrXa;
		if(UlFctryYa == 0xFFFFFFFF)	UlBufDat[ ACCL_FCTRY_OFST_Y ] = UlCurrYa;
		if(UlFctryZa == 0xFFFFFFFF)	UlBufDat[ ACCL_FCTRY_OFST_Z ] = UlCurrZa;

		UlBufDat[ GYRO_OFFSET_X ] = UlGofXg;
		UlBufDat[ GYRO_OFFSET_Y ] = UlGofYg;
		UlBufDat[ GYRO_OFFSET_Z ] = UlGofZg;
		UlBufDat[ ACCL_OFFSET_X ] = UlGofXa;
		UlBufDat[ ACCL_OFFSET_Y ] = UlGofYa;
		UlBufDat[ ACCL_OFFSET_Z ] = UlGofZa;

//------------------------------------------------------------------------------------------------
// Write gyro angle data
//------------------------------------------------------------------------------------------------
		WriteCalDataF40(UlBufDat, &UiChkSum1);
//------------------------------------------------------------------------------------------------
// Calculate calibration data checksum
//------------------------------------------------------------------------------------------------
		ReadCalDataF40(UlBufDat, &UiChkSum2);

		if(UiChkSum1 != UiChkSum2){
			TRACE("CheckSum error\n");
			TRACE("UiChkSum1 = %08X, UiChkSum2 = %08X\n",(uint32_t)UiChkSum1, (uint32_t)UiChkSum2);
			ans = 0x10;
		}
//		TRACE("CheckSum OK\n");
//		TRACE("UiChkSum1 = %08X, UiChkSum2 = %08X\n",(uint32_t)UiChkSum1, (uint32_t)UiChkSum2);
	}
//------------------------------------------------------------------------------------------------
// Resume
//------------------------------------------------------------------------------------------------
	if(!UlSrvStat) {
		RtnCen(BOTH_OFF);
	} else if(UlSrvStat == 3) {
		RtnCen(BOTH_ON);
	} else {
		RtnCen(UlSrvStat);
	}

	if(UlOisStat != 0)               OisEna();

	return(ans);															// CheckSum OK

}

//********************************************************************************
// Function Name 	: LGMC_GyroOffset_ReCalibration
// Retun Value		: SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Gyro Offset recalibration
// History			: First edition 									2018.2.28
//********************************************************************************
uint8_t	LGMC_GyroOffset_ReCalibration(void)
{
	uint32_t	UlRsltSts;
    uint32_t	val_gyro_offset_x, val_gyro_offset_y;
    uint32_t	nvr2_gyro_offset_x, nvr2_gyro_offset_y;
	uint32_t	SlMeasureParameterA, SlMeasureParameterB;
	uint32_t	SlMeasureParameterNum;
	UnllnVal	StMeasValueA, StMeasValueB;
	uint32_t	SlMeasureAveValueA, SlMeasureAveValueB;

	// read back the default values before re-calibration.
	RamRead32A(0x278, &val_gyro_offset_x);
    RamRead32A(0x27C, &val_gyro_offset_y);
	CAM_ERR(CAM_OIS,"Re-calibration start GyroOffsetX: %d, GyroOffsetY: %d",
		(int16_t)(val_gyro_offset_x >> 16), (int16_t)(val_gyro_offset_y >> 16));

	MesFil(THROUGH);					// Set Measure Filter

	SlMeasureParameterNum	=	GYROF_NUM;					// Measurement times
	SlMeasureParameterA		=	GYRO_RAM_GX_ADIDAT;		// Set Measure RAM Address
	SlMeasureParameterB		=	GYRO_RAM_GY_ADIDAT;		// Set Measure RAM Address

	MeasureStart(SlMeasureParameterNum, SlMeasureParameterA, SlMeasureParameterB);					// Start measure

	MeasureWait();					// Wait complete of measurement

	RamRead32A(StMeasFunc_MFA_LLiIntegral1, &StMeasValueA.StUllnVal.UlLowVal);	// X axis
	RamRead32A(StMeasFunc_MFA_LLiIntegral1 + 4, &StMeasValueA.StUllnVal.UlHigVal);
	RamRead32A(StMeasFunc_MFB_LLiIntegral2, &StMeasValueB.StUllnVal.UlLowVal);	// Y axis
	RamRead32A(StMeasFunc_MFB_LLiIntegral2 + 4, &StMeasValueB.StUllnVal.UlHigVal);

	SlMeasureAveValueA = (uint32_t)((int64_t)StMeasValueA.UllnValue / SlMeasureParameterNum);
	SlMeasureAveValueB = (uint32_t)((int64_t)StMeasValueB.UllnValue / SlMeasureParameterNum);

	SlMeasureAveValueA = (SlMeasureAveValueA >> 16) & 0x0000FFFF;
	SlMeasureAveValueB = (SlMeasureAveValueB >> 16) & 0x0000FFFF;

	UlRsltSts = EXE_END;
	StAdjPar.StGvcOff.UsGxoVal = (uint16_t)(SlMeasureAveValueA & 0x0000FFFF);		//Measure Result Store
	if(((int16_t)StAdjPar.StGvcOff.UsGxoVal > (int16_t)GYROF_UPPER) || ((int16_t)StAdjPar.StGvcOff.UsGxoVal < (int16_t)GYROF_LOWER)){
		UlRsltSts |= EXE_GXADJ;
	}
	RamWrite32A(GYRO_RAM_GXOFFZ, ((SlMeasureAveValueA << 16) & 0xFFFF0000));		// X axis Gyro offset

	StAdjPar.StGvcOff.UsGyoVal = (uint16_t)(SlMeasureAveValueB & 0x0000FFFF);		//Measure Result Store
	if(((int16_t)StAdjPar.StGvcOff.UsGyoVal > (int16_t)GYROF_UPPER) || ((int16_t)StAdjPar.StGvcOff.UsGyoVal < (int16_t)GYROF_LOWER)){
		UlRsltSts |= EXE_GYADJ;
	}
	RamWrite32A(GYRO_RAM_GYOFFZ, ((SlMeasureAveValueB << 16) & 0xFFFF0000));		// Y axis Gyro offset
	RamWrite32A(GYRO_RAM_GYROX_OFFSET, 0x00000000);			// X axis Drift Gyro offset
	RamWrite32A(GYRO_RAM_GYROY_OFFSET, 0x00000000);			// Y axis Drift Gyro offset
	RamWrite32A(GyroFilterDelayX_GXH1Z2, 0x00000000);		// X axis H1Z2 Clear
	RamWrite32A(GyroFilterDelayY_GYH1Z2, 0x00000000);		// Y axis H1Z2 Clear

	// read back the default values after re-calibration.
	RamRead32A(0x278, &val_gyro_offset_x);
    RamRead32A(0x27C, &val_gyro_offset_y);
	CAM_ERR(CAM_OIS,"Re-calibration end GyroOffsetX: %d, GyroOffsetY: %d",
		(int16_t)(val_gyro_offset_x >> 16), (int16_t)(val_gyro_offset_y >> 16));

	WrGyroOffsetData();

	ReadNVR2_F40(GYRO_OFFSET_X,  &nvr2_gyro_offset_x );
	ReadNVR2_F40(GYRO_OFFSET_Y,  &nvr2_gyro_offset_y );
	CAM_ERR(CAM_OIS,"NVR2 GyroOffsetX: %d, GyroOffsetY: %d",
		(int16_t)(nvr2_gyro_offset_x >> 16), (int16_t)(nvr2_gyro_offset_y >> 16));

	CAM_ERR(CAM_OIS, "ois calibration done. UlRsltSts 0x%x", UlRsltSts);
	return(UlRsltSts);
}

//********************************************************************************
// Function Name 	: GetInfomation
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Information Data
// History			: First edition
//********************************************************************************
uint8_t GetInfomation(DSPVER* Info)
{
	uint32_t Data;

	RamRead32A((SiVerNum + 0), &Data);
	Info->Vendor 	= (uint8_t)(Data >> 24);
	Info->User 		= (uint8_t)(Data >> 16);
	Info->Model 	= (uint8_t)(Data >> 8);
	Info->Version 	= (uint8_t)(Data >> 0);
	RamRead32A((SiVerNum + 4), &Data);
	Info->GyroType  = (uint8_t)(Data >> 16);
	Info->ActType   = (uint8_t)(Data >> 8);
	Info->LocalVer  = (uint8_t)(Data >> 0);

	TRACE(" (Vender, User  , Model , Version) = (%d,%d,%d,%d) \n", Info->Vendor, Info->User, Info->Model,Info->Version);
	TRACE(" (      , GyrTyp, ActTyp, LocalV ) = ( ,%d,%d,%d) \n", Info->GyroType, Info->ActType, Info->LocalVer);
//	 MonitorInfo(Info);
	return(0);
}
