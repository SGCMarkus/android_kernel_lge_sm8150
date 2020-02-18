//********************************************************************************
//		<< LC898124 Evaluation Soft>>
//		Program Name	: Ois.h
// 		Explanation		: LC898124 Global Declaration & ProtType Declaration
//		Design			: Y.Yamada
//		History			: First edition
//********************************************************************************
#include "OisLc898124EP1.h"

//****************************************************
//	MODE SELECTORS (Compile Switches)
//****************************************************
//#define		__OIS_UIOIS_GYRO_USE__

//#define		HF_LINEAR_ENA		//

//#define		__OIS_BIG_ENDIAN__

//#define		EEPROM_FULL_ERASE		// E2Prom full erase

#define	SELECT_VENDOR		0x04	// 0bit :
									// 1bit :
									// 2bit : LGIT
									// 3bit :
									// 7bit :
#define	SELECT_MODEL		0		// 0 : STORM
									// x : ---

//#define	SELECT_GYRO			0	// --- gyro ---//
//									// 0 : INVN_ICG-1020S
//#define	SELECT_ACT			1	// --- actuator ---//
//									// 0 : not use
//									// 1 : (LGIT  )LGIT N3

#if (SELECT_VENDOR == 0x04)
 #define	FW_VER				0x0B		//ATMEL Version for LGIT
 #define	SUB_VER				0x00		//
#else
 #define	FW_VER				0x0B		//ATMEL Version for ALL
 #define	SUB_VER				0x00		//
#endif

//****************************************************
//	TYPE
//****************************************************
#define		INT16	int16_t
#define		INT32	int32_t
#define		INT64	int64_t
#define		UINT8	uint8_t
#define		UINT16	uint16_t
#define		UINT32	uint32_t
#define		UINT64	uint64_t

//****************************************************
//	Defines
//****************************************************
//Calibration
#define ONE_MSEC_COUNT	20
#define LOOPGAIN		1
#define THROUGH			2
#define NOISE			3
#define	OSCCHK			4

#define	CNT050MS		 676
#define	CNT100MS		1352
#define	CNT200MS		2703

#define LSB 0
#define MSB 8

// Command Status
#define		EXE_END		0x00000002L		// Execute End (Adjust OK)
#define		EXE_ERROR	0x00000003L		// Adjust NG : Execution Failure
#define		EXE_HXADJ	0x00000006L		// Adjust NG : X Hall NG (Gain or Offset)
#define		EXE_HYADJ	0x0000000AL		// Adjust NG : Y Hall NG (Gain or Offset)
#define		EXE_LXADJ	0x00000012L		// Adjust NG : X Loop NG (Gain)
#define		EXE_LYADJ	0x00000022L		// Adjust NG : Y Loop NG (Gain)
#define		EXE_GXADJ	0x00000042L		// Adjust NG : X Gyro NG (offset)
#define		EXE_GYADJ	0x00000082L		// Adjust NG : Y Gyro NG (offset)
#define		EXE_HXMVER	0x06		// X Err
#define		EXE_HYMVER	0x0A		// Y Err
// Gyro Examination of Acceptance
#define		EXE_GXABOVE	0x06		// X Above
#define		EXE_GXBELOW	0x0A		// X Below
#define		EXE_GYABOVE	0x12		// Y Above
#define		EXE_GYBELOW	0x22		// Y Below

// Common Define
#define	SUCCESS			0x00		// Success
#define	FAILURE			0x01		// Failure

#ifndef ON
 #define	ON				0x01		// ON
 #define	OFF				0x00		// OFF
#endif

#define	X_DIR			0x00		// X Direction
#define	Y_DIR			0x01		// Y Direction

// mode
#define		GEA_MINMAX_MODE		0x00		// min, max mode
#define		GEA_MEAN_MODE		0x01		// mean mode

#define		BOTH_ON			0x00
#define		XONLY_ON		0x01
#define		YONLY_ON		0x02
#define		BOTH_OFF		0x03

#define 	GYROF_NUM		2048			// 2048times
#define 	GYROF_UPPER		0x06D6			//
#define 	GYROF_LOWER		0xF92A			//

//****************************************************
//	Generic memory
//****************************************************
#ifdef __OIS_BIG_ENDIAN__
// Big endian
// Word Data Union
union	WRDVAL{
	INT16	SsWrdVal ;
	UINT16	UsWrdVal ;
	UINT8	UcWrkVal[ 2 ] ;
	signed char		ScWrkVal[ 2 ] ;
	struct {
		UINT8	UcHigVal ;
		UINT8	UcLowVal ;
	} StWrdVal ;
} ;


union	DWDVAL {
	UINT32	UlDwdVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsHigVal ;
		UINT16	UsLowVal ;
	} StDwdVal ;
	struct {
		UINT8	UcRamVa3 ;
		UINT8	UcRamVa2 ;
		UINT8	UcRamVa1 ;
		UINT8	UcRamVa0 ;
	} StCdwVal ;
} ;

union	ULLNVAL {
	UINT64	UllnValue ;
	UINT32	UlnValue[ 2 ] ;
	struct {
		UINT32	UlHigVal ;
		UINT32	UlLowVal ;
	} StUllnVal ;
} ;


// Float Data Union
union	FLTVAL {
	float			SfFltVal ;
	UINT32	UlLngVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsHigVal ;
		UINT16	UsLowVal ;
	} StFltVal ;
} ;

#else	// BIG_ENDDIAN
// Little endian
// Word Data Union
union	WRDVAL{
	INT16	SsWrdVal ;
	UINT16	UsWrdVal ;
	UINT8	UcWrkVal[ 2 ] ;
	signed char		ScWrkVal[ 2 ] ;
	struct {
		UINT8	UcLowVal ;
		UINT8	UcHigVal ;
	} StWrdVal ;
} ;

union	DWDVAL {
	UINT32	UlDwdVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsLowVal ;
		UINT16	UsHigVal ;
	} StDwdVal ;
	struct {
		UINT8	UcRamVa0 ;
		UINT8	UcRamVa1 ;
		UINT8	UcRamVa2 ;
		UINT8	UcRamVa3 ;
	} StCdwVal ;
} ;

union	ULLNVAL {
	UINT64	UllnValue ;
	UINT32	UlnValue[ 2 ] ;
	struct {
		UINT32	UlLowVal ;
		UINT32	UlHigVal ;
	} StUllnVal ;
} ;

// Float Data Union
union	FLTVAL {
	float			SfFltVal ;
	UINT32	UlLngVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsLowVal ;
		UINT16	UsHigVal ;
	} StFltVal ;
} ;
#endif	// __OIS_BIG_ENDIAN__

typedef union WRDVAL	UnWrdVal ;
typedef union DWDVAL	UnDwdVal;
typedef union ULLNVAL	UnllnVal;
typedef union FLTVAL	UnFltVal ;


typedef struct STMESRAM {
	INT32	SlMeasureMaxValue ;
	INT32	SlMeasureMinValue ;
	INT32	SlMeasureAmpValue ;
	INT32	SlMeasureAveValue ;
} stMesRam ;									// Struct Measure Ram

typedef struct {
	UINT32 BiasInit;
	UINT32 OffsetInit;
	UINT32 OffsetMargin;
	UINT32 TargetRange;
	UINT32 TargetMax;
	UINT32 TargetMin;
	UINT32 SinNum;
	UINT32 SinFreq;
	UINT32 SinGain;
	UINT32 DecrementStep;
} ADJ_HALL;

typedef struct {
	UINT32 Hxgain;
	UINT32 Hygain;
	UINT32 NoiseNum;
	UINT32 NoiseFreq;
	UINT32 NoiseGain;
	UINT32 Gap;
	UINT32 XJudgeHigh;
	UINT32 XJudgeLow;
	UINT32 YJudgeHigh;
	UINT32 YJudgeLow;
} ADJ_LOPGAN;

typedef struct {
	UINT8 Vendor;
	UINT8 User;
	UINT8 Model;
	UINT8 Version;
	UINT8 Reserve0;
	UINT8 Reserve1;
	UINT8 ActType;
	UINT8 GyroType;
} DSPVER;

typedef struct {
	UINT16	Cmd ;
	const UINT8* DataPM;
	UINT32 LengthPM;
	UINT32 Parity;
	const UINT8* DataDM;
	UINT32 LengthDM;
}DOWNLOAD_TBL ;
//****************************************************
//	Structure of calibration data for GUI
//****************************************************
typedef struct STADJPAR {
	struct {
		UINT32	UlAdjPhs ;				// Hall Adjust Phase

		UINT16	UsHlxCna ;				// Hall Center Value after Hall Adjust
		UINT16	UsHlxMax ;				// Hall Max Value
		UINT16	UsHlxMxa ;				// Hall Max Value after Hall Adjust
		UINT16	UsHlxMin ;				// Hall Min Value
		UINT16	UsHlxMna ;				// Hall Min Value after Hall Adjust
		UINT16	UsHlxGan ;				// Hall Gain Value
		UINT16	UsHlxOff ;				// Hall Offset Value
		UINT16	UsAdxOff ;				// Hall A/D Offset Value
		UINT16	UsHlxCen ;				// Hall Center Value

		UINT16	UsHlyCna ;				// Hall Center Value after Hall Adjust
		UINT16	UsHlyMax ;				// Hall Max Value
		UINT16	UsHlyMxa ;				// Hall Max Value after Hall Adjust
		UINT16	UsHlyMin ;				// Hall Min Value
		UINT16	UsHlyMna ;				// Hall Min Value after Hall Adjust
		UINT16	UsHlyGan ;				// Hall Gain Value
		UINT16	UsHlyOff ;				// Hall Offset Value
		UINT16	UsAdyOff ;				// Hall A/D Offset Value
		UINT16	UsHlyCen ;				// Hall Center Value
	} StHalAdj ;

	struct {
		UINT32	UlLxgVal ;				// Loop Gain X
		UINT32	UlLygVal ;				// Loop Gain Y
	} StLopGan ;

	struct {
		UINT16	UsGxoVal ;				// Gyro A/D Offset X
		UINT16	UsGyoVal ;				// Gyro A/D Offset Y
		UINT16	UsGxoSts ;				// Gyro Offset X Status
		UINT16	UsGyoSts ;				// Gyro Offset Y Status
	} StGvcOff ;

	UINT8		UcOscVal ;				// OSC value

} stAdjPar ;

typedef struct {
	INT32				SiSampleNum ;			// Measure Sample Number
	INT32				SiSampleMax ;			// Measure Sample Number Max

	struct {
		INT32			SiMax1 ;				// Max Measure Result
		INT32			SiMin1 ;				// Min Measure Result
		UINT32	UiAmp1 ;				// Amplitude Measure Result
		INT64		LLiIntegral1 ;			// Integration Measure Result
		INT64		LLiAbsInteg1 ;			// Absolute Integration Measure Result
		INT32			PiMeasureRam1 ;			// Measure Delay RAM Address
	} MeasureFilterA ;

	struct {
		INT32			SiMax2 ;				// Max Measure Result
		INT32			SiMin2 ;				// Min Measure Result
		UINT32	UiAmp2 ;				// Amplitude Measure Result
		INT64		LLiIntegral2 ;			// Integration Measure Result
		INT64		LLiAbsInteg2 ;			// Absolute Integration Measure Result
		INT32			PiMeasureRam2 ;			// Measure Delay RAM Address
	} MeasureFilterB ;
} MeasureFunction_Type ;

//****************************************************
//	Debug
//****************************************************
#ifdef DEBUG
#include <AT91SAM7S.h>
#include <us.h>
 #define TRACE_INIT(x)			dbgu_init(x)
 #define TRACE(fmt, ...)		dbgu_printf(fmt, ## __VA_ARGS__)
 #define TRACE_DUMP(x,y)		dbg_Dump(x,y)
 #define TRACE_USB(fmt, ...)	dbg_UsbData(fmt, ## __VA_ARGS__)
#else
 #define TRACE_INIT(x)
 #define TRACE(...)
 #define TRACE_DUMP(x,y)
 #define TRACE_USB(...)
#endif

