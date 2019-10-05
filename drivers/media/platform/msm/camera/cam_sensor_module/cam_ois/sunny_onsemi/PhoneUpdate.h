/**
 *		LC898123F40 Global declaration & prototype declaration
 *
 *		Copyright (C) 2017, ON Semiconductor, all right reserved.
 *
 **/

#ifndef PHONEUPDATE_H_
#define PHONEUPDATE_H_

#include <cam_sensor_cmn_header.h>
#include "cam_sensor_util.h"

//==============================================================================
//
//==============================================================================
//#define	MODULE_VENDOR	1
//#define	MDL_VER			1

#ifdef DEBUG
 extern void dbg_printf(const char *, ...);
 extern void dbg_Dump(const char *, int);
 #define TRACE_INIT(x)			dbgu_init(x)
 #define TRACE_USB(fmt, ...)	dbg_UsbData(fmt, ## __VA_ARGS__)
 #define TRACE(fmt, ...)		dbg_printf(fmt, ## __VA_ARGS__)
 #define TRACE_DUMP(x,y)		dbg_Dump(x,y)
#else
 #define TRACE_INIT(x)
 #define TRACE(...)
 #define TRACE_DUMP(x,y)
 #define TRACE_USB(...)
#endif

//****************************************************
//	STRUCTURE DEFINE
//****************************************************
typedef struct {
	uint16_t				Index;
	const uint8_t*		MagicCode;
	uint16_t				SizeMagicCode;
	const uint8_t*		FromCode;
	uint16_t				SizeFromCode;
}	DOWNLOAD_TBL;
typedef struct {
	uint8_t Vendor;
	uint8_t User;
	uint8_t Model;
	uint8_t Version;
	uint8_t Reserve0;
	uint8_t GyroType;
	uint8_t ActType;
	uint8_t LocalVer;
} DSPVER;

typedef struct {
	int32_t				SiSampleNum ;			//!< Measure Sample Number
	int32_t				SiSampleMax ;			//!< Measure Sample Number Max

	struct {
		int32_t			SiMax1 ;				//!< Max Measure Result
		int32_t			SiMin1 ;				//!< Min Measure Result
		uint32_t	UiAmp1 ;						//!< Amplitude Measure Result
		int64_t		LLiIntegral1 ;				//!< Integration Measure Result
		int64_t		LLiAbsInteg1 ;				//!< Absolute Integration Measure Result
		int32_t			PiMeasureRam1 ;			//!< Measure Delay RAM Address
	} MeasureFilterA ;

	struct {
		int32_t			SiMax2 ;				//!< Max Measure Result
		int32_t			SiMin2 ;				//!< Min Measure Result
		uint32_t	UiAmp2 ;						//!< Amplitude Measure Result
		int64_t		LLiIntegral2 ;				//!< Integration Measure Result
		int64_t		LLiAbsInteg2 ;				//!< Absolute Integration Measure Result
		int32_t			PiMeasureRam2 ;			//!< Measure Delay RAM Address
	} MeasureFilterB ;
} MeasureFunction_Type ;

union	DWDVAL {
	uint32_t	UlDwdVal ;
	uint16_t	UsDwdVal[ 2 ] ;
	struct {
		uint16_t	UsLowVal ;
		uint16_t	UsHigVal ;
	} StDwdVal ;
	struct {
		uint8_t	UcRamVa0 ;
		uint8_t	UcRamVa1 ;
		uint8_t	UcRamVa2 ;
		uint8_t	UcRamVa3 ;
	} StCdwVal ;
} ;
typedef union DWDVAL	UnDwdVal;

union	ULLNVAL {
	uint64_t	UllnValue ;
	uint32_t	UlnValue[ 2 ] ;
	struct {
		uint32_t	UlLowVal ;
		uint32_t	UlHigVal ;
	} StUllnVal ;
} ;
typedef union ULLNVAL	UnllnVal;

typedef struct STADJPAR {
	struct {
		uint32_t	UlAdjPhs ;				//!< Hall Adjust Phase

		uint16_t	UsHlxCna ;				//!< Hall Center Value after Hall Adjust
		uint16_t	UsHlxMax ;				//!< Hall Max Value
		uint16_t	UsHlxMxa ;				//!< Hall Max Value after Hall Adjust
		uint16_t	UsHlxMin ;				//!< Hall Min Value
		uint16_t	UsHlxMna ;				//!< Hall Min Value after Hall Adjust
		uint16_t	UsHlxGan ;				//!< Hall Gain Value
		uint16_t	UsHlxOff ;				//!< Hall Offset Value
		uint16_t	UsAdxOff ;				//!< Hall A/D Offset Value
		uint16_t	UsHlxCen ;				//!< Hall Center Value

		uint16_t	UsHlyCna ;				//!< Hall Center Value after Hall Adjust
		uint16_t	UsHlyMax ;				//!< Hall Max Value
		uint16_t	UsHlyMxa ;				//!< Hall Max Value after Hall Adjust
		uint16_t	UsHlyMin ;				//!< Hall Min Value
		uint16_t	UsHlyMna ;				//!< Hall Min Value after Hall Adjust
		uint16_t	UsHlyGan ;				//!< Hall Gain Value
		uint16_t	UsHlyOff ;				//!< Hall Offset Value
		uint16_t	UsAdyOff ;				//!< Hall A/D Offset Value
		uint16_t	UsHlyCen ;				//!< Hall Center Value

	} StHalAdj ;

	struct {
		uint32_t	UlLxgVal ;				//!< Loop Gain X
		uint32_t	UlLygVal ;				//!< Loop Gain Y
	} StLopGan ;

	struct {
		uint16_t	UsGxoVal ;				//!< Gyro A/D Offset X
		uint16_t	UsGyoVal ;				//!< Gyro A/D Offset Y
		uint16_t	UsGxoSts ;				//!< Gyro Offset X Status
		uint16_t	UsGyoSts ;				//!< Gyro Offset Y Status
	} StGvcOff ;
} stAdjPar ;
//stAdjPar	StAdjPar ;		//!< Calibration data

typedef struct STPOSOFF {
	struct {
		int32_t	Pos[6][3];
	} StPos;
	uint32_t		UlAclOfSt ;				//!< accel offset status

} stPosOff ;
//stPosOff	StPosOff ;				//!< Execute Command Parameter

typedef struct STACLVAL {
	struct {
		int32_t	SlOffsetX ;
		int32_t	SlOffsetY ;
		int32_t	SlOffsetZ ;
	} StAccel ;

	int32_t	SlInvMatrix[9] ;

} stAclVal ;
//stAclVal	StAclVal ;				//!< Execute Command Parameter

#define	SUCCESS			0x00
#define	FAILURE			0x01

// Command Status
#define		EXE_END		0x00000002L		//!< Execute End (Adjust OK)
#define		EXE_HXADJ	0x00000006L		//!< Adjust NG : X Hall NG (Gain or Offset)
#define		EXE_HYADJ	0x0000000AL		//!< Adjust NG : Y Hall NG (Gain or Offset)
#define		EXE_LXADJ	0x00000012L		//!< Adjust NG : X Loop NG (Gain)
#define		EXE_LYADJ	0x00000022L		//!< Adjust NG : Y Loop NG (Gain)
#define		EXE_GXADJ	0x00000042L		//!< Adjust NG : X Gyro NG (offset)
#define		EXE_GYADJ	0x00000082L		//!< Adjust NG : Y Gyro NG (offset)
#define		EXE_ERR		0x00000099L		//!< Execute Error End
#ifdef	SEL_SHIFT_COR
#define		EXE_GZADJ	0x00400002L		//!< Adjust NG : Z Gyro NG (offset)
#endif	//SEL_SHIFT_COR

//	for RtnCen
#define		BOTH_ON			0x00
#define		XONLY_ON		0x01
#define		YONLY_ON		0x02
#define		BOTH_OFF		0x03
#define		ZONLY_OFF		0x04
#define		ZONLY_ON		0x05
//==============================================================================
//
//==============================================================================

#define		F40_IO_ADR_ACCESS				0xC000
#define		F40_IO_DAT_ACCESS				0xD000

//==============================================================================
// Prototype
//==============================================================================
extern void     F40_IORead32A( uint32_t IOadrs, uint32_t *IOdata );
extern void		F40_IOWrite32A( uint32_t IOadrs, uint32_t IOdata );
extern uint8_t	F40_FlashDownload( uint8_t chiperase, uint8_t ModuleVendor, uint8_t ActVer );
extern uint8_t	F40_FlashUpdate( uint8_t flag, DOWNLOAD_TBL* ptr );
extern uint8_t	F40_FlashBlockErase( uint32_t SetAddress );
extern void		F40_ReadCalData( uint32_t * BufDat, uint32_t * ChkSum );
extern uint8_t	F40_WrGyroOffsetData( void ) ;
extern uint8_t	F40_RdStatus( uint8_t UcStBitChk );
extern uint8_t WriteCalDataF40( uint32_t * BufDat, uint32_t * ChkSum );
extern uint8_t EraseCalDataF40( void );
extern void ReadNVR2_F40( uint8_t Address,  uint32_t * BufDat );

#endif /* #ifndef OIS_H_ */
