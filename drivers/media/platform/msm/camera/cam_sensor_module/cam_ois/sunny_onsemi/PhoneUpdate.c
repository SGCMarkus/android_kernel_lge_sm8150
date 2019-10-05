/**
 *		LC898123F40 Flash update
 *
 *		Copyright (C) 2017, ON Semiconductor, all right reserved.
 *
 **/

#define 		__DOWNLOAD_SPEED_UP__

//**************************
//	Include Header File
//**************************
#include	"PhoneUpdate.h"
#include	"OisLc898123F40.h"

#include	"PmemCode.h"
#include	"FromCode_00_01.h"

#define	USER_RESERVE			3
#define	ERASE_BLOCKS			(16 - USER_RESERVE)

#define BURST_LENGTH (12*5)

#define DMB_COEFF_ADDRESS		0x21
#define BLOCK_UNIT				0x200
#define BLOCK_BYTE				2560
#define SECTOR_SIZE				320
#define HALF_SECTOR_ADD_UNIT	0x20
#define FLASH_ACCESS_SIZE		32
#define	USER_AREA_START			(BLOCK_UNIT * ERASE_BLOCKS)

//****************************************************
//	CUSTOMER NECESSARY CREATING FUNCTION LIST
//****************************************************
/* for I2C communication */
extern int32_t RamWrite32A(uint32_t RamAddr, uint32_t RamData);
extern int32_t RamRead32A(uint32_t RamAddr, uint32_t *ReadData);
extern int32_t ois_i2c_read_seq(uint32_t addr, uint8_t *data, uint16_t num_byte);
/* for I2C Multi Translation: Burst Mode*/
extern int32_t CntWrt(uint8_t *data, uint16_t num_byte);

/* for Wait timer [Need to adjust for your system] */
extern void WitTim(unsigned short	UsWitTim);

//**************************
//	Table of download file
//**************************

const DOWNLOAD_TBL DTbl[] = {
	{0x0001, CcMagicCodeF40_00_01, sizeof(CcMagicCodeF40_00_01), CcFromCodeF40_00_01, sizeof(CcFromCodeF40_00_01)},
	{0xFFFF, (void*)0, 0, (void*)0, 0}
};

//********************************************************************************
// Function Name 	: F40_IOWrite32A
//********************************************************************************
void F40_IORead32A(uint32_t IOadrs, uint32_t *IOdata)
{
	RamWrite32A(F40_IO_ADR_ACCESS, IOadrs);
	RamRead32A (F40_IO_DAT_ACCESS, IOdata);
}

//********************************************************************************
// Function Name 	: F40_IOWrite32A
//********************************************************************************
void F40_IOWrite32A(uint32_t IOadrs, uint32_t IOdata)
{
	RamWrite32A(F40_IO_ADR_ACCESS, IOadrs);
	RamWrite32A(F40_IO_DAT_ACCESS, IOdata);
}

//********************************************************************************
// Function Name 	: WPB level read
//********************************************************************************
uint8_t F40_ReadWPB(void)
{
	uint32_t UlReadVal, UlCnt=0;

	do{
		F40_IORead32A(FLASHROM_F40_WPB, &UlReadVal);
		if((UlReadVal & 0x00000004) != 0)	return (1);
		WitTim(1);
	}while (UlCnt++ < 10);
	return (0);
}

//********************************************************************************
// Function Name 	: F40_UnlockCodeSet
//********************************************************************************
uint8_t F40_UnlockCodeSet(void)
{
	uint32_t UlReadVal;

//	WPBCtrl(WPB_OFF);
	if (F40_ReadWPB() != 1)
		return (5);

	F40_IOWrite32A(FLASHROM_F40_UNLK_CODE1,	0xAAAAAAAA);
	F40_IOWrite32A(FLASHROM_F40_UNLK_CODE2,	0x55555555);
	F40_IOWrite32A(FLASHROM_F40_RSTB_FLA, 0x00000001);
	F40_IOWrite32A(FLASHROM_F40_CLK_FLAON, 0x00000010);
	F40_IOWrite32A(FLASHROM_F40_UNLK_CODE3,	0x0000ACD5);
	F40_IOWrite32A(FLASHROM_F40_WPB, 0x00000001);
	RamRead32A( F40_IO_DAT_ACCESS, &UlReadVal);

	if ((UlReadVal & 0x00000007) != 7)
		return(1);

	return(0);
}

//********************************************************************************
// Function Name 	: F40_UnlockCodeClear
//********************************************************************************
uint8_t F40_UnlockCodeClear(void)
{
	uint32_t UlReadVal;

	F40_IOWrite32A(FLASHROM_F40_WPB, 0x00000010);
	RamRead32A(F40_IO_DAT_ACCESS, &UlReadVal);

	if((UlReadVal & 0x00000080) != 0)
		return(3);

//	WPBCtrl(WPB_ON);

	return(0);
}

//********************************************************************************
// Function Name 	: F40_FlashBlockErase
//********************************************************************************
uint8_t F40_FlashBlockErase(uint32_t SetAddress)
{
	uint32_t	UlReadVal, UlCnt;
	uint8_t	ans	= 0;

	if(SetAddress & 0x00010000)
		return(9);

	ans	= F40_UnlockCodeSet();
	if(ans != 0)
		return(ans);

	F40_IOWrite32A(FLASHROM_F40_ADR, (SetAddress & 0xFFFFFE00));
	F40_IOWrite32A(FLASHROM_F40_CMD, 0x00000006);

	WitTim(5);

	UlCnt	= 0;
	do {
		if(UlCnt++ > 100) {
			ans = 2;
			break;
		}

		F40_IORead32A(FLASHROM_F40_INT, &UlReadVal);
	} while((UlReadVal & 0x00000080) != 0);

	F40_UnlockCodeClear();

	return(ans);
}

//********************************************************************************
// Function Name 	: F40_FlashBurstWrite
//********************************************************************************
uint8_t F40_FlashBurstWrite(const uint8_t *NcDataVal, uint32_t NcDataLength, uint32_t ScNvrMan)
{
	uint32_t	i, j, UlCnt;
	uint8_t	data[163];
	uint32_t	UlReadVal;
	uint8_t	UcOddEvn = 0;
	uint8_t	Remainder;

#ifdef	__DOWNLOAD_SPEED_UP__
	uint32_t SendLength = 0;
	uint8_t ChkSector = 0;
	data[0] = 0xF0;

	RamWrite32A(0xF00A ,ScNvrMan	);

	while((SendLength + BURST_LENGTH) < NcDataLength)
	{
		if(++UcOddEvn >1)  	UcOddEvn = 0;
		if (UcOddEvn == 0) data[1] = 0x0E;
		else 			   data[1] = 0x0F;

		UlCnt = 2;

		if((ChkSector + BURST_LENGTH) > 160){
			for(j = 0; j < (160 - ChkSector); j++)	data[UlCnt++] = *NcDataVal++;
			CntWrt(data, (160 - ChkSector) + 2);
			SendLength += (160 - ChkSector);
			ChkSector = 0;
		}else if((ChkSector + BURST_LENGTH) == 160){
			for(j = 0; j < BURST_LENGTH; j++)	data[UlCnt++] = *NcDataVal++;
			CntWrt(data, BURST_LENGTH + 2);
			SendLength += BURST_LENGTH;
			ChkSector= 0;
		}else{
			for(j = 0; j < BURST_LENGTH; j++)	data[UlCnt++] = *NcDataVal++;
			CntWrt(data, BURST_LENGTH + 2);
			SendLength += BURST_LENGTH;
			ChkSector += BURST_LENGTH;
		}
	}

	Remainder = NcDataLength - SendLength;
	if (Remainder != 0){

		if(++UcOddEvn >1)  	UcOddEvn = 0;
		if (UcOddEvn == 0) data[1] = 0x0E;
		else 			   data[1] = 0x0F;
		WitTim(1);
		UlCnt = 2;

		if((ChkSector + Remainder) > 160){
			i= 160 - ChkSector;

			for(j = 0; j < i; j++)	data[UlCnt++] = *NcDataVal++;
			CntWrt(data, i + 2);

			if(++UcOddEvn >1)  	UcOddEvn = 0;
			if (UcOddEvn == 0) data[1] = 0x0E;
			else 			   data[1] = 0x0F;
			WitTim(1);
			UlCnt = 2;

			for(j = 0; j < (Remainder - i); j++)	data[UlCnt++] = *NcDataVal++;
			CntWrt(data, (Remainder - i) + 2);

		}else{
			for(j = 0; j < Remainder; j++)	data[UlCnt++] = *NcDataVal++;
			CntWrt(data, Remainder + 2);
		}
	}
#else
	data[0] = 0xF0;
	data[1] = 0x08;
	data[2] = BURST_LENGTH;

	for(i = 0; i < (NcDataLength / BURST_LENGTH); i++) {
		UlCnt = 3;

		UcOddEvn =i % 2;
		data[1] = 0x08 + UcOddEvn;

		for(j = 0; j < BURST_LENGTH; j++)
			data[UlCnt++] = *NcDataVal++;

		CntWrt(data, BURST_LENGTH + 3);
		RamWrite32A(0xF00A ,(uint32_t) ((BURST_LENGTH / 5) * i + ScNvrMan));
		RamWrite32A(0xF00B ,(uint32_t) (BURST_LENGTH / 5));

		RamWrite32A(0xF00C , 4 + 4 * UcOddEvn);
	}

	Remainder = NcDataLength % BURST_LENGTH;
	if(Remainder != 0) {
		data[2] = Remainder;
		UlCnt = 3;
		UcOddEvn =i % 2;
		data[1] = 0x08 + UcOddEvn;

		for(j = 0; j < Remainder; j++)
			data[UlCnt++] = *NcDataVal++;

		CntWrt(data, BURST_LENGTH + 3);
		RamWrite32A(0xF00A ,(uint32_t) ((BURST_LENGTH / 5) * i + ScNvrMan));
		RamWrite32A(0xF00B ,(uint32_t) (Remainder /5));
		RamWrite32A(0xF00C , 4 + 4 * UcOddEvn);
	}
#endif

	UlCnt = 0;
	do {
		if(UlCnt++ > 100)
			return (1);

		RamRead32A(0xF00C, &UlReadVal);
	} while (UlReadVal != 0);

	return(0);
}

//********************************************************************************
// Function Name 	: F40_FlashSectorRead
//********************************************************************************
void F40_FlashSectorRead(uint32_t UlAddress, uint8_t *PucData)
{
	uint8_t	UcIndex, UcNum;
	uint8_t	UcReadDat[ 4 ];

	F40_IOWrite32A(FLASHROM_F40_ADR,			(UlAddress & 0xFFFFFFC0));
	F40_IOWrite32A(FLASHROM_F40_ACSCNT,		63);
	UcNum	= 64;

	F40_IOWrite32A(FLASHROM_F40_CMD,			0x00000001);

	for(UcIndex = 0; UcIndex < UcNum; UcIndex++) {
		RamWrite32A(F40_IO_ADR_ACCESS,		FLASHROM_F40_RDATH);
//		RamRead32A( F40_IO_DAT_ACCESS,		UcReadDat);
		ois_i2c_read_seq(F40_IO_DAT_ACCESS, UcReadDat, 4);
		*PucData++		= UcReadDat[ 0 ];
		RamWrite32A(F40_IO_ADR_ACCESS,		FLASHROM_F40_RDATL);
//		RamRead32A(F40_IO_DAT_ACCESS, UcReadDat);
		ois_i2c_read_seq(F40_IO_DAT_ACCESS, UcReadDat, 4);
		*PucData++	= UcReadDat[ 3 ];
		*PucData++	= UcReadDat[ 2 ];
		*PucData++	= UcReadDat[ 1 ];
		*PucData++	= UcReadDat[ 0 ];
	}
}

//********************************************************************************
// Function Name 	: F40_CalcChecksum
//********************************************************************************
void F40_CalcChecksum(const uint8_t *pData, uint32_t len, uint32_t *pSumH, uint32_t *pSumL)
{
	uint64_t sum = 0;
	uint32_t dat;
	uint16_t i;

	for(i = 0; i < len / 5; i++) {
#ifdef _BIG_ENDIAN_

		dat  = (uint32_t)*pData++;
		dat += (uint32_t)*pData++ << 8;
		dat += (uint32_t)*pData++ << 16;
		dat += (uint32_t)*pData++ << 24;

		sum  += (uint64_t)*pData++ << 32;
#else
		sum  += (uint64_t)*pData++ << 32;

		dat  = (uint32_t)*pData++ << 24;
		dat += (uint32_t)*pData++ << 16;
		dat += (uint32_t)*pData++ << 8;
		dat += (uint32_t)*pData++;
#endif
		sum += (uint64_t)dat;
	}

	*pSumH = (uint32_t)(sum >> 32);
	*pSumL = (uint32_t)(sum & 0xFFFFFFFF);
}

//********************************************************************************
// Function Name 	: F40_CalcBlockChksum
//********************************************************************************
void F40_CalcBlockChksum(uint8_t num, uint32_t *pSumH, uint32_t *pSumL)
{
	uint8_t	SectorData[SECTOR_SIZE];
	uint32_t	top;
	uint16_t	sec;
	uint64_t	sum = 0;
	uint32_t	datH, datL;

	top = num * BLOCK_UNIT;

	for(sec = 0; sec < (BLOCK_BYTE / SECTOR_SIZE); sec++) {
		F40_FlashSectorRead(top + sec * 64, SectorData);

		F40_CalcChecksum(SectorData, SECTOR_SIZE, &datH, &datL);
		sum += ((uint64_t)datH << 32) + datL;
	}

	*pSumH = (uint32_t)(sum >> 32);
	*pSumL = (uint32_t)(sum & 0xFFFFFFFF);
}

//********************************************************************************
// Function Name 	: F40_FlashDownload
//********************************************************************************
uint8_t F40_FlashDownload(uint8_t chiperase, uint8_t ModuleVendor, uint8_t ActVer)
{
	DOWNLOAD_TBL* ptr;

	ptr = (DOWNLOAD_TBL *)DTbl;
	do {
		if(ptr->Index == (((uint16_t)ModuleVendor<<8) + ActVer)) {
			return F40_FlashUpdate(chiperase, ptr);
		}
		ptr++;
	} while (ptr->Index != 0xFFFF);

	return 0xF0;
}

//********************************************************************************
// Function Name 	: F40_FlashUpdate
//********************************************************************************
uint8_t F40_FlashUpdate(uint8_t flag, DOWNLOAD_TBL* ptr)
{
	int32_t	SiWrkVl0 ,SiWrkVl1;
	int32_t	SiAdrVal;
	const uint8_t *NcDatVal;
	uint32_t	UlReadVal, UlCnt;
	uint8_t	ans, i;
	uint16_t	UsChkBlocks;
	uint8_t UserMagicCode[ SECTOR_SIZE ];

//--------------------------------------------------------------------------------
// 0.
//--------------------------------------------------------------------------------
	F40_IOWrite32A(SYSDSP_REMAP,				0x00001440);
	WitTim(25);
	F40_IORead32A(SYSDSP_SOFTRES,				(uint32_t *)&SiWrkVl0);
	SiWrkVl0	&= 0xFFFFEFFF;
	F40_IOWrite32A(SYSDSP_SOFTRES,				SiWrkVl0);
	RamWrite32A(0xF006,					0x00000000);
	F40_IOWrite32A(SYSDSP_DSPDIV,				0x00000001);
	RamWrite32A(0x0344,					0x00000014);
	SiAdrVal =0x00100000;

	for(UlCnt = 0;UlCnt < (sizeof(UlPmemCodeF40) /(5*4)); UlCnt++){
		RamWrite32A(0x0340,				SiAdrVal);
		SiAdrVal += 0x00000008;
		RamWrite32A(0x0348,				UlPmemCodeF40[ UlCnt*5   ]);
		RamWrite32A(0x034C,				UlPmemCodeF40[ UlCnt*5+1 ]);
		RamWrite32A(0x0350,				UlPmemCodeF40[ UlCnt*5+2 ]);
		RamWrite32A(0x0354,				UlPmemCodeF40[ UlCnt*5+3 ]);
		RamWrite32A(0x0358,				UlPmemCodeF40[ UlCnt*5+4 ]);
		RamWrite32A(0x033c,				0x00000001);
	}
	for(UlCnt = 0;UlCnt < (sizeof(UpData_CommandFromTable))/6; UlCnt++){
		CntWrt((uint8_t *)&UpData_CommandFromTable[ UlCnt*6 ], 0x00000006);
	}

//--------------------------------------------------------------------------------
// 1.
//--------------------------------------------------------------------------------
	if(flag) {
		ans = F40_UnlockCodeSet();
		if (ans != 0){
			return(ans);
		}

		F40_IOWrite32A(FLASHROM_F40_ADR,		0x00000000);
		F40_IOWrite32A(FLASHROM_F40_CMD,		0x00000005);
		WitTim(13);
		UlCnt=0;
		do {
			if(UlCnt++ > 100) {
				ans=0x10;
				break;
			}
			F40_IORead32A(FLASHROM_F40_INT,	&UlReadVal);
		}while ((UlReadVal & 0x00000080) != 0);

	} else {
		for(i = 0; i < ERASE_BLOCKS; i++) {
			ans	= F40_FlashBlockErase(i * BLOCK_UNIT);
			if(ans != 0) {
				return(ans);
			}
		}
		ans = F40_UnlockCodeSet();
		if (ans != 0){
			return(ans);
		}
	}
//--------------------------------------------------------------------------------
// 2.
//--------------------------------------------------------------------------------
	F40_IOWrite32A(FLASHROM_F40_ADR,			0x00010000);
	F40_IOWrite32A(FLASHROM_F40_CMD,			0x00000004);
	WitTim(5);
	UlCnt=0;
	do {
		if(UlCnt++ > 100) {
			ans = 0x10;
			break;
		}
		F40_IORead32A(FLASHROM_F40_INT,		&UlReadVal);
	} while ((UlReadVal & 0x00000080) != 0);

//--------------------------------------------------------------------------------
// 3.
//--------------------------------------------------------------------------------
	F40_FlashBurstWrite(ptr->FromCode,		ptr->SizeFromCode, 0);

	ans |= F40_UnlockCodeClear();
	if (ans != 0){
		return(ans);
	}

//--------------------------------------------------------------------------------
// 4.
//--------------------------------------------------------------------------------
	UsChkBlocks = (ptr->SizeFromCode / 160) + 1;
	RamWrite32A(0xF00A,					0x00000000);
	RamWrite32A(0xF00B,					UsChkBlocks);
	RamWrite32A(0xF00C,					0x00000100);

	NcDatVal = ptr->FromCode;
	SiWrkVl0 = 0;
	for(UlCnt = 0; UlCnt < ptr->SizeFromCode; UlCnt++) {
		SiWrkVl0 += *NcDatVal++;
	}
	UsChkBlocks *= 160 ;
	for(; UlCnt < UsChkBlocks; UlCnt++) {
		SiWrkVl0 += 0xFF;
	}

	UlCnt=0;
	do {
		if(UlCnt++ > 100){
			return (6);
		}

		RamRead32A(0xF00C,					&UlReadVal);
	} while(UlReadVal != 0) ;

	RamRead32A(0xF00D,						&SiWrkVl1);

	if(SiWrkVl0 != SiWrkVl1){
		return(0x20);
	}

//--------------------------------------------------------------------------------
// X.
//--------------------------------------------------------------------------------

	if (!flag) {
		uint32_t sumH, sumL;
		uint16_t Idx;

		for(UlCnt = 0; UlCnt < ptr->SizeMagicCode; UlCnt++) {
			UserMagicCode[ UlCnt ] = ptr->MagicCode[ UlCnt ];
		}

		for(UlCnt = 0; UlCnt < USER_RESERVE; UlCnt++) {
			F40_CalcBlockChksum(ERASE_BLOCKS + UlCnt, &sumH, &sumL);
			Idx =  (ERASE_BLOCKS + UlCnt) * 2 * 5 + 1 + 40;
			NcDatVal = (uint8_t *)&sumH;

#ifdef _BIG_ENDIAN_
			// for BIG ENDIAN SYSTEM
			UserMagicCode[ Idx++ ] = *NcDatVal++;
			UserMagicCode[ Idx++ ] = *NcDatVal++;
			UserMagicCode[ Idx++ ] = *NcDatVal++;
			UserMagicCode[ Idx++ ] = *NcDatVal++;
			Idx++;
			NcDatVal = (uint8_t *)&sumL;
			UserMagicCode[ Idx++ ] = *NcDatVal++;
			UserMagicCode[ Idx++ ] = *NcDatVal++;
			UserMagicCode[ Idx++ ] = *NcDatVal++;
			UserMagicCode[ Idx++ ] = *NcDatVal++;
#else
			// for LITTLE ENDIAN SYSTEM
			UserMagicCode[ Idx+3 ] = *NcDatVal++;
			UserMagicCode[ Idx+2 ] = *NcDatVal++;
			UserMagicCode[ Idx+1 ] = *NcDatVal++;
			UserMagicCode[ Idx+0 ] = *NcDatVal++;
			Idx+=5;
			NcDatVal = (uint8_t *)&sumL;
			UserMagicCode[ Idx+3 ] = *NcDatVal++;
			UserMagicCode[ Idx+2 ] = *NcDatVal++;
			UserMagicCode[ Idx+1 ] = *NcDatVal++;
			UserMagicCode[ Idx+0 ] = *NcDatVal++;
#endif
		}
		NcDatVal = UserMagicCode;

	} else {
		NcDatVal = ptr->MagicCode;
	}


//--------------------------------------------------------------------------------
// 5.
//--------------------------------------------------------------------------------
	ans = F40_UnlockCodeSet();
	if (ans != 0){
		return(ans);
	}

	F40_FlashBurstWrite(NcDatVal, ptr->SizeMagicCode, 0x00010000);
	F40_UnlockCodeClear();

//--------------------------------------------------------------------------------
// 6.
//--------------------------------------------------------------------------------
	RamWrite32A(0xF00A,					0x00010000);
	RamWrite32A(0xF00B,					0x00000002);
	RamWrite32A(0xF00C,					0x00000100);

	SiWrkVl0 = 0;
	for(UlCnt = 0; UlCnt < ptr->SizeMagicCode; UlCnt++) {
		SiWrkVl0 += *NcDatVal++;
	}
	for(; UlCnt < 320; UlCnt++) {
		SiWrkVl0 += 0xFF;
	}

	UlCnt=0;
	do {
		if(UlCnt++ > 100)
			return(6);

		RamRead32A(0xF00C,					&UlReadVal);
	} while(UlReadVal != 0);
	RamRead32A(0xF00D,						&SiWrkVl1);

	if(SiWrkVl0 != SiWrkVl1)
		return(0x30);

	F40_IOWrite32A(SYSDSP_REMAP,				0x00001000);
	return(0);
}

//********************************************************************************
// Function Name 	: F40_ReadCalData
//********************************************************************************

void F40_ReadCalData(uint32_t * BufDat, uint32_t * ChkSum)
{
	uint16_t	UsSize = 0, UsNum;

	*ChkSum = 0;

	do{
		// Count
		F40_IOWrite32A(FLASHROM_F40_ACSCNT, (FLASH_ACCESS_SIZE-1));

		// NVR2 Addres Set
		F40_IOWrite32A(FLASHROM_F40_ADR, 0x00010040 + UsSize);		// set NVR2 area
		// Read Start
		F40_IOWrite32A(FLASHROM_F40_CMD, 1);  						// Read Start

		RamWrite32A(F40_IO_ADR_ACCESS , FLASHROM_F40_RDATL);		// RDATL data

		for(UsNum = 0; UsNum < FLASH_ACCESS_SIZE; UsNum++)
		{
			RamRead32A( F40_IO_DAT_ACCESS , &(BufDat[ UsSize ]));
			*ChkSum += BufDat[ UsSize++ ];
		}
	}while (UsSize < 64);	// 64*5 = 320: NVR sector size
}

//********************************************************************************
// Function Name 	: F40_RdStatus
//********************************************************************************
uint8_t	F40_RdStatus(uint8_t UcStBitChk)
{
	uint32_t	UlReadVal;

	RamRead32A(0xF100 , &UlReadVal);
	if(UcStBitChk){
		UlReadVal &= READ_STATUS_INI;
	}
	if(!UlReadVal){
		return(SUCCESS);
	}else{
		return(FAILURE);
	}
}

//********************************************************************************
// Function Name 	: EraseCalDataF40
//********************************************************************************
uint8_t EraseCalDataF40(void)
{
	uint32_t	UlReadVal, UlCnt;
	uint8_t ans = 0;


	ans = F40_UnlockCodeSet();
	if ( ans != 0 ) return (ans);


	F40_IOWrite32A( FLASHROM_F40_ADR, 0x00010040 ) ;

	F40_IOWrite32A( FLASHROM_F40_CMD, 4	 ) ;

	WitTim( 5 ) ;
	UlCnt=0;
	do{
		if( UlCnt++ > 100 ){	ans = 2;	break;	} ;
		F40_IORead32A( FLASHROM_F40_INT, &UlReadVal );
	}while ( (UlReadVal & 0x00000080) != 0 );

	ans = F40_UnlockCodeClear();

	return(ans);
}

//********************************************************************************
// Function Name 	: ReadCalDataF40
//********************************************************************************
void ReadCalDataF40(uint32_t * BufDat, uint32_t * ChkSum)
{
	uint16_t	UsSize = 0, UsNum;

	*ChkSum = 0;

	do{

		F40_IOWrite32A( FLASHROM_F40_ACSCNT, (FLASH_ACCESS_SIZE-1) ) ;


		F40_IOWrite32A( FLASHROM_F40_ADR, 0x00010040 + UsSize ) ;

		F40_IOWrite32A( FLASHROM_F40_CMD, 1 ) ;

		RamWrite32A( F40_IO_ADR_ACCESS , FLASHROM_F40_RDATL ) ;

		for( UsNum = 0; UsNum < FLASH_ACCESS_SIZE; UsNum++ )
		{
			RamRead32A(  F40_IO_DAT_ACCESS , &(BufDat[ UsSize ]) ) ;

			*ChkSum += BufDat[ UsSize++ ];
		}
	}while (UsSize < 64);
}

//********************************************************************************
// Function Name 	: WriteCalDataF40
//********************************************************************************
extern void SetCheckSumCalib( void );
uint8_t WriteCalDataF40(uint32_t * BufDat, uint32_t * ChkSum)
{
	uint16_t	UsSize = 0, UsNum;
	uint8_t ans = 0;
	uint32_t	UlReadVal = 0;

//-----------------------------------------------
	SetCheckSumCalib();
//-----------------------------------------------

	*ChkSum = 0;


	ans = F40_UnlockCodeSet();
	if ( ans != 0 ) return (ans);

	F40_IOWrite32A( FLASHROM_F40_WDATH, 0x000000FF ) ;

	do{
		// Count
		F40_IOWrite32A( FLASHROM_F40_ACSCNT, (FLASH_ACCESS_SIZE - 1) ) ;
		// NVR2 Addres Set
		F40_IOWrite32A( FLASHROM_F40_ADR, 0x00010040 + UsSize ) ;
		// Write Start
		F40_IOWrite32A( FLASHROM_F40_CMD, 2) ;



		for( UsNum = 0; UsNum < FLASH_ACCESS_SIZE; UsNum++ )
		{


			F40_IOWrite32A( FLASHROM_F40_WDATL,  BufDat[ UsSize ] ) ;
			do {
				F40_IORead32A( FLASHROM_F40_INT, &UlReadVal );
			}while ( (UlReadVal & 0x00000020) != 0 );

			*ChkSum += BufDat[ UsSize++ ];
		}
	}while (UsSize < 64);

	ans = F40_UnlockCodeClear();

	return( ans );
}

//********************************************************************************
// Function Name 	: ReadNVR2_F40
//********************************************************************************
void ReadNVR2_F40(uint8_t Address,  uint32_t * BufDat)
{
	// Count
	F40_IOWrite32A( FLASHROM_F40_ACSCNT, 0 ) ;
	// NVR2 Addres Set
	F40_IOWrite32A( FLASHROM_F40_ADR, 0x00010040 + Address ) ;		// set NVR2 area
	// Read Start
	F40_IOWrite32A( FLASHROM_F40_CMD, 1 ) ;  						// Read Start
	RamWrite32A( F40_IO_ADR_ACCESS , FLASHROM_F40_RDATL ) ;		// RDATL data

	RamRead32A(  F40_IO_DAT_ACCESS , BufDat ) ;
}

//********************************************************************************
