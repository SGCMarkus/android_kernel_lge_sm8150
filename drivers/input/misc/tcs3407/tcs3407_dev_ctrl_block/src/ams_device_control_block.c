/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/
  
/*
 * Device Control Block 3407
 */

/*
 * @@AMS_REVISION_Id: 562ee7f424a003b8825afa3aa921e8cb5d0e71ab
 */


#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/limits.h>
#else
#include <stdbool.h>
#include <limits.h>
#endif

#include "ams_port_platform.h"
#include "ams_device_control_block.h"

#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
#include "../../ccb_als/include/core_control_block_als.h"
#endif

#define HIGH    0xFF
#define LOW     0x00

deviceRegisterTable_t deviceRegisterDefinition[DEVREG_REG_MAX] = {
    { 0x00, 0x00 },        /* DEVREG_RAM_START */
    { 0x13, 0x00 },        /* DEVREG_SMUX13_PRX_TO_FLICKER */

    { 0x80, 0x00 },          /* DEVREG_ENABLE */
    { 0x81, 0x00 },          /* DEVREG_ATIME */
    { 0x82, 0x00 },          /* DEVREG_PTIME */
    { 0x83, 0x00 },          /* DEVREG_WTIME */
    { 0x84, 0x00 },          /* DEVREG_AILTL */
    { 0x85, 0x00 },          /* DEVREG_AILTH */
    { 0x86, 0x00 },          /* DEVREG_AIHTL */
    { 0x87, 0x00 },          /* DEVREG_AIHTH */

    { 0x90, 0x00 },          /* DEVREG_AUXID */
    { 0x91, AMS_REV_ID },    /* DEVREG_REVID */
    { 0x92, AMS_DEVICE_ID }, /* DEVREG_ID */
    { 0x93, 0x00 },          /* DEVREG_STATUS */
    { 0x94, 0x00 },          /* DEVREG_ASTATUS */
    { 0x95, 0x00 },          /* DEVREG_ADATAOL */
    { 0x96, 0x00 },          /* DEVREG_ADATAOH */
    { 0x97, 0x00 },          /* DEVREG_ADATA1L */
    { 0x98, 0x00 },          /* DEVREG_ADATA1H */
    { 0x99, 0x00 },          /* DEVREG_ADATA2L */
    { 0x9A, 0x00 },          /* DEVREG_ADATA2H */
    { 0x9B, 0x00 },          /* DEVREG_ADATA3L */
    { 0x9C, 0x00 },          /* DEVREG_ADATA3H */
    { 0x9D, 0x00 },          /* DEVREG_ADATA4L */
    { 0x9E, 0x00 },          /* DEVREG_ADATA4H */
    { 0x9F, 0x00 },          /* DEVREG_ADATA5L */

    { 0xA0, 0x00 },          /* DEVREG_ADATA5H */
    { 0xA3, 0x00 },          /* DEVREG_STATUS2 */
    { 0xA4, 0x00 },          /* DEVREG_STATUS3 */
    { 0xA6, 0x00 },          /* DEVREG_STATUS5 */
    { 0xA7, 0x00 },          /* DEVREG_STATUS4 */
    /* 0xA8 Reserved */
    { 0xA9, 0x40 },          /* DEVREG_CFG0 */
    { 0xAA, 0x09 },          /* DEVREG_CFG1 */
    { 0xAC, 0x0C },          /* DEVREG_CFG3 */
    { 0xAD, 0x00 },          /* DEVREG_CFG4 */
    { 0xAF, 0x00 },          /* DEVREG_CFG6 */

    { 0xB1, 0x80 },          /* DEVREG_CFG8 */
    { 0xB2, 0x00 },          /* DEVREG_CFG9 */
    { 0xB3, 0xF2 },          /* DEVREG_CFG10 */
    { 0xB4, 0x4D },          /* DEVREG_CFG11 */
    { 0xB5, 0x00 },          /* DEVREG_CFG12 */
    { 0xB7, 0x00 },          /* DEVREG_PCFG1 */
    { 0xBD, 0x00 },          /* DEVREG_PERS */
    { 0xBE, 0x02 },          /* DEVREG_GPIO */
                             
    { 0xCA, 0xE7 },          /* DEVREG_ASTEPL */
    { 0xCB, 0x03 },          /* DEVREG_ASTEPH */
//    { 0xCF, 0X97 },          /* DEVREG_AGC_GAIN_MAX */
    { 0xCF, 0XBC },          /* DEVREG_AGC_GAIN_MAX */

                             
    { 0xD6, 0xFf },          /* DEVREG_AZ_CONFIG */
    { 0xD7, 0x21},           /*DEVREG_FD_CFG0*/
    { 0xD8, 0x68},           /*DEVREG_FD_CFG1*/
												 
    { 0xD9, 0x64},           /*DEVREG_FD_CFG2*/
    { 0xDA, 0x91 },           /*DEVREG_FD_CFG3*/    	
													   
	
    { 0xDB, 0x00 },          /* DEVREG_FD_STATUS */
    /* 0xEF-0xF8 Reserved */
    { 0xF9, 0x00 },          /* DEVREG_INTENAB */
    { 0xFA, 0x00 },          /* DEVREG_CONTROL */
    { 0xFC, 0x00 },          /* DEVREG_FIFO_MAP */
    { 0xFD, 0x00 },          /* DEVREG_FIFO_STATUS */
    { 0xFE, 0x00 },          /* DEVREG_FDATAL */
    { 0xFF, 0x00 },          /* DEVREG_FDATAH */   
    { 0x6f, 0x00 },          /* DEVREG_FLKR_WA_RAMLOC_1 */
    { 0x71, 0x00 },          /* DEVREG_FLKR_WA_RAMLOC_2 */
    { 0xF3, 0x00 },          /* DEVREG_SOFT_RESET */
    
};

#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
uint32_t alsGain_conversion[] = { 
      1000 / 2,
      1 * 1000,
      2 * 1000,
      4 * 1000,
      8 * 1000,
     16 * 1000,
     32 * 1000,
     64 * 1000,
    128 * 1000,
    256 * 1000,
    512 * 1000,
    1024 * 1000,
    2048 * 1000
};

uint8_t alsIndexToGain(uint8_t x);
uint8_t alsIndexToGain(uint8_t x){
    int i;

    for (i = sizeof(alsGain_conversion)/sizeof(uint32_t)-1; i != 0; i--) {
        if (x == i) break;
    }
    return (i << 3);
}


uint8_t alsGainToReg(uint32_t x);
uint8_t alsGainToReg(uint32_t x){
    int i;

    for (i = sizeof(alsGain_conversion)/sizeof(uint32_t)-1; i != 0; i--) {
        if (x >= alsGain_conversion[i]) break;
    }
    return (i << 0);
}


uint8_t FlickerGainToReg(uint32_t x);
uint8_t FlickerGainToReg(uint32_t x){
    int i;

    for (i = sizeof(alsGain_conversion)/sizeof(uint32_t)-1; i != 0; i--) {
//        if (x >= alsGain_conversion[i]) break;
        if (x == i) break;
		
    }
    return (i );
}


uint16_t alsTimeUsToReg(uint32_t x){
    uint16_t regValue;
    if(x == 0){
		regValue = 0 ;
		return regValue;
    }else{
    		regValue = (x / 2780)-1;
    }
    return regValue;
}
#endif






#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
static bool _3407_alsSetThreshold(ams_deviceCtx_t * ctx, int32_t threshold){
    ams_ccb_als_config_t configData;
    configData.threshold = threshold;
    ccb_alsSetConfig(ctx, &configData);
    return false;
}

bool _3407_alsInit(ams_deviceCtx_t * ctx, ams_calibrationData_t * calibrationData);

bool ams_deviceGetAls(ams_deviceCtx_t * ctx, ams_apiAls_t * exportData){
    ams_ccb_als_result_t result;
    ccb_alsGetResult(ctx, &result);
    exportData->mLux        = result.mLux;
    exportData->saturation   = result.saturation;

    exportData->red         = result.red;
    exportData->green       = result.green;
    exportData->blue        = result.blue;
    exportData->ir          = result.ir;
    exportData->wideband    = result.wideband;
    exportData->rawClear    = result.rawClear;
    exportData->rawRed      = result.rawRed;
    exportData->rawGreen    = result.rawGreen;
    exportData->rawBlue     = result.rawBlue;

    return false;
}

#if 0
static void _3407_handleAlsEvent(ams_deviceCtx_t * ctx){
    ams_ccb_als_dataSet_t ccbAlsData;
    ccbAlsData.statusReg = ctx->shadowStatus1Reg;
    //ams_getWord(ctx->portHndl, DEVREG_ADATA5L,&flickerCtx->flicker_raw_data);
	
    ccb_alsHandle(ctx, &ccbAlsData);
}
#endif
#endif

bool _3407_flickerInit(ams_deviceCtx_t * ctx);

void  ams_deviceGetSWFlicker(ams_deviceCtx_t * ctx, ams_apiAlsFlicker_t * exportData){
   //ams_flicker_ctx_t *flickerCtx = (ams_flicker_ctx_t *)&ctx->flickerCtx;
   //ccb_sw_flicker_GetResult(ctx);
    ams_ccb_als_result_t result;
    ccb_alsGetResult(ctx, &result);
    if(result.wideband <=200)
		result.saturation = 1;/*general low IR light source*/

    if(result.wideband >200 && result.mLux < 600)
		result.saturation = 2;/*high IR light source*/

    if(result.wideband >200 && result.mLux >= 600)
		result.saturation = 3;/*Sun light*/
	
    exportData->wideband    = result.wideband; /*AWB*/
    exportData->clear    = result.clear; 
    exportData->saturation    = result.saturation; 	
    exportData->mHzbysw = ctx->flickerCtx.lastValid.mHzbysw; 
    exportData->mLux    = result.mLux;  /*C/AWB*/
    AMS_PORT_msg_5("ams_deviceGetSWFlicker: freq %u, AWB %d , Clear ch  %d, Sun Light %d\n ,C/AWB %d", ctx->flickerCtx.lastValid.mHzbysw,exportData->wideband,exportData->clear,exportData->saturation,exportData->mLux);
}


bool ams_deviceGetFlicker(ams_deviceCtx_t * ctx, ams_apiAlsFlicker_t * exportData){
    ams_flicker_ctx_t *flickerCtx = (ams_flicker_ctx_t *)&ctx->flickerCtx;

    if (flickerCtx->statusReg & FD_100HZ_VALID)
        exportData->freq100Hz = flickerCtx->lastValid.freq100Hz =
            (flickerCtx->statusReg & FD_100HZ_FLICKER) ? PRESENT : ABSENT;
    else
        exportData->freq100Hz = flickerCtx->lastValid.freq100Hz;

    if (flickerCtx->statusReg & FD_120HZ_VALID)
        exportData->freq120Hz = flickerCtx->lastValid.freq120Hz =
            (flickerCtx->statusReg & FD_120HZ_FLICKER) ? PRESENT : ABSENT;
    else
        exportData->freq120Hz = flickerCtx->lastValid.freq120Hz;

    if ((exportData->freq100Hz == PRESENT) && (exportData->freq120Hz == PRESENT))
        exportData->mHz = flickerCtx->lastValid.mHz = (uint32_t)(ULONG_MAX);
    else if (exportData->freq100Hz == PRESENT)

        exportData->mHz = flickerCtx->lastValid.mHz = 100;
 
    else if (exportData->freq120Hz == PRESENT)
															   
        exportData->mHz = flickerCtx->lastValid.mHz = 120;

    else
        exportData->mHz = 0;


	exportData->flicker_raw_data = flickerCtx->flicker_raw_data;
																																																		
 
    return false;
}



static void _3407_handleFlickerFIFOEvent(ams_deviceCtx_t * ctx){
    bool ret;
																	

    ams_ccb_als_dataSet_t ccbAlsData = {0,};
#if defined (AMS_BIN_2048_MODE1)  ||defined (AMS_BIN_2048_MODE2) 
    ret = ccb_FlickerFIFO4096Event(ctx, &ccbAlsData);
#endif

#if !defined (AMS_BIN_2048_MODE1)  && !defined (AMS_BIN_2048_MODE2) 
    ret = ccb_FlickerFIFOEvent(ctx, &ccbAlsData);
#endif		
}


static bool  _3407_handleFlickerEvent(ams_deviceCtx_t * ctx){
uint8_t status4 ,fd_cfg3;
//uint16_t flicker_raw =0 ;
//    uint8_t     flickerstatus;

    ams_flicker_ctx_t *flickerCtx = (ams_flicker_ctx_t *)&ctx->flickerCtx;
#if 0
   if(ctx->valid_flickerhz_count == 2){
	//ams_deviceSetConfig(ctx, AMS_CONFIG_HW_FLICKER, AMS_CONFIG_ENABLE, 0);/*hw flicker disable*/
	ams_deviceSetConfig(ctx, AMS_CONFIG_SW_FLICKER, AMS_CONFIG_ENABLE, 1);/*sw flicker enable*/
	return 0;
   }
 #endif
 
    ams_getByte(ctx->portHndl, DEVREG_FD_STATUS, &flickerCtx->statusReg);	
    ams_getByte(ctx->portHndl, DEVREG_FD_CFG3, &fd_cfg3);	
    ams_getByte(ctx->portHndl, DEVREG_STATUS4,&status4);
	
    ams_setByte(ctx->portHndl, DEVREG_FD_STATUS, MASK_CLEAR_FLICKER_STATUS);
    ams_setByte(ctx->portHndl, DEVREG_FD_STATUS, 0xff);
    //ams_getWord(ctx->portHndl, DEVREG_ADATA5L,&flickerCtx->flicker_raw_data);
	
//    ams_setByte(ctx->portHndl, DEVREG_FD_STATUS, flickerCtx->statusReg);
//    flickerstatus = 	 flickerCtx->statusReg & (MASK_FLICKER_VALID | MASK_100HZ_FLICKER |MASK_120HZ_FLICKER); /*status & 0x2f*/

    AMS_PORT_msg_2("_3407_handleFlickerEvent:    flickerCtx->statusReg =0x%x, gain %x"
		,  flickerCtx->statusReg,fd_cfg3);
    //AMS_PORT_log_3("_3407_handleFlickerEvent:   FD status = 0x%02x ,status4 0x%x, fd_gain =0x%x \n"
	//	, flickerCtx->statusReg, status4,fd_cfg3);

#if 0
    if (flickerCtx->statusReg == (MASK_FLICKER_VALID | MASK_100HZ_FLICKER)) //100 hz detect 
    {
        ctx->updateAvailable |= (1 << AMS_FLICKER_SENSOR);
	 ctx->valid_flickerhz_count = 0;
	 return 0;
    }

    if (flickerCtx->statusReg == (MASK_FLICKER_VALID | MASK_120HZ_FLICKER)) //120 hz detect 
    {
        ctx->updateAvailable |= (1 << AMS_FLICKER_SENSOR);
	 ctx->valid_flickerhz_count = 0;
	 return 0;
 
    }

    if ( flickerstatus == (MASK_FLICKER_VALID )) //120 hz detect 
    {
	 ctx->valid_flickerhz_count++;
    }
#else
    if (flickerCtx->statusReg  & MASK_FLICKER_VALID) //100 hz detect 
        ctx->updateAvailable |= (1 << AMS_FLICKER_SENSOR);

    AMS_PORT_msg_3("_3407_handleFlickerEvent:   FD status = 0x%02x ,status4 0x%x, fd_gain =0x%x \n"
		, flickerCtx->statusReg, status4,fd_cfg3);

#endif
	
   return 0;
}



bool    ams_deviceSetConfig(ams_deviceCtx_t * ctx, ams_configureFeature_t feature, deviceConfigOptions_t option, uint32_t data){
    int ret = 0;

#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
    if (feature == AMS_CONFIG_ALS_LUX){
        AMS_PORT_log("ams_configureFeature_t  AMS_CONFIG_ALS_LUX\n");
        switch (option)
        {
            case AMS_CONFIG_ENABLE: /* ON / OFF */
                AMS_PORT_log_1("deviceConfigOptions_t   AMS_CONFIG_ENABLE(%u)\n", data);
                AMS_PORT_log_1("current mode            %d\n", ctx->mode);
                if (data == 0) {
                    if (ctx->mode == MODE_ALS_RGB) {
                        /* if no other active features, turn off device */
                        ctx->shadowEnableReg = 0;
                        ctx->shadowIntenabReg = 0;
                        ctx->mode = MODE_OFF;
                    } else {
                        if ((ctx->mode & MODE_ALS_ALL) == MODE_ALS_RGB) {
                            ctx->shadowEnableReg &= ~MASK_AEN;
                            ctx->shadowIntenabReg &= ~MASK_ALS_INT_ALL;
                        }
                        ctx->mode &= ~(MODE_ALS_RGB);
                    }
                } else {
                    if ((ctx->mode & MODE_ALS_ALL) == 0) {
                        ccb_alsInit(ctx, &ctx->ccbAlsCtx.initData);
                        ctx->shadowEnableReg |= (AEN | PON);
						
                        ctx->shadowIntenabReg |= AIEN;
                       
                    } else {
                        /* force interrupt */
                        ams_setWord(ctx->portHndl, DEVREG_AIHTL, 0x00);
                    }
                    ctx->mode |= MODE_ALS_RGB;
                }
                break;
            case AMS_CONFIG_THRESHOLD: /* set threshold */
                AMS_PORT_log(  "deviceConfigOptions_t   AMS_CONFIG_THRESHOLD\n");
                AMS_PORT_log_1("data                    %d\n", data);
                ret |= _3407_alsSetThreshold(ctx, data);
                break;
            default:
                break;
        }
    }
#endif

    if (feature == AMS_CONFIG_HW_FLICKER){
        AMS_PORT_log("ams_configureFeature_t  AMS_CONFIG_HW_FLICKER\n");
        switch (option)
        {
            case AMS_CONFIG_ENABLE: /* power on */
                AMS_PORT_log_1("deviceConfigOptions_t   AMS_CONFIG_ENABLE(%u)\n", data);
                AMS_PORT_log_1("current mode            %d\n", ctx->mode);
                if (data == 0) {
                    //ams_setField(ctx->portHndl, DEVREG_CFG9, LOW, SIEN_FD);
                    ams_setField(ctx->portHndl, DEVREG_CFG9, LOW, MASK_SIEN_FD);
                    if (ctx->mode == MODE_FLICKER) {
                        /* if no other active features, turn off device */
                        ctx->shadowEnableReg = 0;
                        ctx->shadowIntenabReg = 0;
                        ctx->mode = MODE_OFF;
                    } else {
                        ctx->mode &= ~MODE_FLICKER;
                            ctx->shadowEnableReg &= ~(FDEN);
                            if (!(ctx->mode & MODE_IRBEAM)) {
                                ctx->shadowIntenabReg &= ~(SIEN);
                            }
                    }
                } else {
                    ctx->shadowEnableReg |= ( FDEN | PON);
                    ctx->shadowIntenabReg |= SIEN;
                    //ams_setField(ctx->portHndl, DEVREG_CFG9, SIEN_FD, SIEN_FD);
                    ams_setField(ctx->portHndl, DEVREG_CFG9, SIEN_FD, MASK_SIEN_FD);
                    ctx->mode |= MODE_FLICKER;
                     _3407_flickerInit(ctx);
					
                }
                break;
            case AMS_CONFIG_THRESHOLD: /* set threshold */
                AMS_PORT_log(  "deviceConfigOptions_t   AMS_CONFIG_THRESHOLD\n");
                /* TODO?:  set FD_COMPARE value? */
                break;
            default:
                break;
        }
    	}
	
      if (feature == AMS_CONFIG_SW_FLICKER){
        AMS_PORT_log("ams_configureFeature_t  AMS_CONFIG_SW_FLICKER\n");
        switch (option)
        {
            case AMS_CONFIG_ENABLE: /* power on */
                AMS_PORT_log_1("deviceConfigOptions_t   AMS_CONFIG_ENABLE(%u)\n", data);
                AMS_PORT_log_1("current mode            %d\n", ctx->mode);
                if (data == 0) {
                    if (ctx->mode == MODE_FLICKER) {
                        /* if no other active features, turn off device */
                        ctx->shadowEnableReg = 0;
                        ctx->shadowIntenabReg = 0;
                        ctx->mode = MODE_OFF;
                    } else {
                        ctx->mode &= ~MODE_FLICKER;
                            ctx->shadowEnableReg &= ~(FDEN);
                            if (!(ctx->mode & MODE_IRBEAM)) {
                                ctx->shadowIntenabReg &= ~(SIEN);
                            }
                    }
                } else {
                    ctx->shadowEnableReg |= ( AEN |FDEN | PON);
#if !defined(AMS_BIN_2048_MODE1) && !defined(AMS_BIN_2048_MODE2)
                    ctx->shadowIntenabReg |= FIEN;
#endif
                    ctx->mode |= MODE_FLICKER;
                    ccb_alsInit_FIFO(ctx, &ctx->ccbAlsCtx.initData);
                }
                break;
            case AMS_CONFIG_THRESHOLD: /* set threshold */
                AMS_PORT_log(  "deviceConfigOptions_t   AMS_CONFIG_THRESHOLD\n");
                /* TODO?:  set FD_COMPARE value? */
                break;
            default:
                break;
        }
    }

    ams_setByte(ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg);
    mdelay(10);
    ams_setByte(ctx->portHndl, DEVREG_INTENAB, ctx->shadowIntenabReg);

    return 0;
}

#if defined ( AMS_BIN_2048_MODE1) ||defined ( AMS_BIN_2048_MODE2)
bool ams_devicePollingHandler(ams_deviceCtx_t * ctx)
{

      ams_ccb_als_dataSet_t ccbAlsData;

      //AMS_PORT_log( "ams_devicePollingHandler");

     _3407_handleFlickerFIFOEvent(ctx);
	 
     if( ctx->flickerCtx.flicker_finished ==1){
                ccb_alsHandle(ctx, &ccbAlsData);
                ccb_sw_bin4096_flicker_GetResult(ctx);
                ctx->updateAvailable |= (1 << AMS_SW_FLICKER_SENSOR);	 
      }		

return true;
}
#endif

bool ams_deviceEventHandler(ams_deviceCtx_t * ctx)
{
    int ret = 1;
    uint8_t status5 = 0;
    //amsAlsDataSet_t inputDataAls;
    ams_ccb_als_dataSet_t ccbAlsData ={0,};

    ams_getByte(ctx->portHndl, DEVREG_STATUS, &ctx->shadowStatus1Reg);
    ams_getByte(ctx->portHndl, DEVREG_STATUS2, &ctx->shadowStatus2Reg);
	
    if (ctx->shadowStatus1Reg & SINT)
    {
        ams_getByte(ctx->portHndl, DEVREG_STATUS5, &status5);
        AMS_PORT_msg_3( "ctx->shadowStatus1Reg %x, status5 %x, mode %x",ctx->shadowStatus1Reg,status5, ctx->mode);
    }

    if (ctx->shadowStatus1Reg != 0) {
        /* this clears interrupt(s) and STATUS5 */
        ams_setByte(ctx->portHndl, DEVREG_STATUS, ctx->shadowStatus1Reg);
    } else {
        AMS_PORT_msg( "ams_devEventHd Error Case!!!!\n");
        ams_getByte(ctx->portHndl, DEVREG_STATUS, &ctx->shadowStatus1Reg);
        AMS_PORT_msg_1( "ctx->shadowStatus1Reg %x",ctx->shadowStatus1Reg);
        //ams_setByte(ctx->portHndl, DEVREG_STATUS, 0xff);
        ret = 1 ;
        return ret;
    }

loop:
    AMS_PORT_get_timestamp_usec(&ctx->timeStamp);

    //AMS_PORT_msg_3( "ams_devEventHd loop_insde: DCB 0x%02x, STATUS 0x%02x, STATUS2 0x%02x\n", ctx->mode, ctx->shadowStatus1Reg, ctx->shadowStatus2Reg);
#if 0
    if ((ctx->shadowStatus1Reg & ALS_INT_ALL) /*|| ctx->alwaysReadAls*/) {
        if ((ctx->mode & MODE_ALS_ALL) && (!(ctx->mode & MODE_IRBEAM))) {
            AMS_PORT_log_2( "_3407_handleAlsEvent INT:%d alwaysReadAls = %d\n", (ctx->shadowStatus1Reg & ALS_INT_ALL), ctx->alwaysReadAls);
            _3407_handleAlsEvent(ctx);
        }
    }
#endif
    if ((ctx->shadowStatus1Reg & (ASAT_FDSAT |FIFOINT | AINT)) ) {		
            _3407_handleFlickerFIFOEvent(ctx);
            if( ctx->flickerCtx.flicker_finished ==1){
                ccb_alsHandle(ctx, &ccbAlsData);
#if defined ( AMS_BIN_2048_MODE1) ||defined ( AMS_BIN_2048_MODE2) 
                ccb_sw_bin4096_flicker_GetResult(ctx);
#endif

#if !defined ( AMS_BIN_2048_MODE1) && !defined ( AMS_BIN_2048_MODE2) 
        	  ccb_sw_flicker_GetResult(ctx); 
#endif
                ctx->updateAvailable |= (1 << AMS_SW_FLICKER_SENSOR);	 
// shmoon_s
          if (ctx->shadowStatus1Reg != 0) {
              /* this clears interrupt(s) and STATUS5 */
              ams_setByte(ctx->portHndl, DEVREG_STATUS, ctx->shadowStatus1Reg);
          } else {
              AMS_PORT_msg( "ams_devEventHd Error Case!!!!\n");
              ams_getByte(ctx->portHndl, DEVREG_STATUS, &ctx->shadowStatus1Reg);
              //AMS_PORT_log_1( "ctx->shadowStatus1Reg %x",ctx->shadowStatus1Reg);
              //ams_setByte(ctx->portHndl, DEVREG_STATUS, 0xff);
              ret = 1 ;
              return ret;
          }
// shmoon_e
                
		  return ret ;
            }
    }   
	
    if ((status5 & SINT_FD) ) {
        if (ctx->mode & MODE_FLICKER){
            //AMS_PORT_msg_1("_3407_handleFlickerEvent status5:0x%02x  \n", (status5 & SINT_FD));
            _3407_handleFlickerEvent(ctx);
        }
    }   

	
    ams_getByte(ctx->portHndl, DEVREG_STATUS, &ctx->shadowStatus1Reg);
   //AMS_PORT_msg_1( "after ams_devEventHd loop:shadowStatus1Reg 0x%x!!!!!!!!!!\n",ctx->shadowStatus1Reg);

    if (ctx->shadowStatus1Reg & SINT) {
        ams_getByte(ctx->portHndl, DEVREG_STATUS5, &status5);
    } else {
        status5 = 0;
    }
	
    if (ctx->shadowStatus1Reg != 0) {
        /* this clears interrupt(s) and STATUS5 */
        ams_setByte(ctx->portHndl, DEVREG_STATUS, ctx->shadowStatus1Reg);
//#if defined( AMS_SW_FLICKER)
        if (!(ctx->shadowStatus1Reg & PSAT))/*changed to update event data during saturation*/
//#else
       // if (!(ctx->shadowStatus1Reg & (ASAT_FDSAT | PSAT)))/*changed to update event data during saturation*/
//#endif			
        {
             //AMS_PORT_msg_1( "ams_devEventHd loop:go loop shadowStatus1Reg 0x%x!!!!!!!!!!\n",ctx->shadowStatus1Reg);
            goto loop;
        }
    }

    /* the individual handlers may have temporarily disabled things */
    //AMS_REENABLE();

    return ret;
}


bool ams_deviceSelfTest(ams_deviceCtx_t * ctx, AMS_PORT_portHndl * portHndl){
    bool ret = false;
    ams_deviceIdentifier_e id = ams_validateDevice(portHndl);

    if ((id > AMS_UNKNOWN_DEVICE) && (id < AMS_LAST_DEVICE)) {
        ret = ams_deviceInit(ctx, portHndl, NULL);
        if (ams_getResult(ctx) == 0){
            AMS_PORT_log("ams_deviceSelfTest: Set IRQ\n");
            AMS_PORT_log("ams_deviceSelfTest: checking result\n");
            if (ams_getResult(ctx) == 0){
                return false;
            } else {
                return true;
            }
        }
    } else return false;

    return ret;
}

bool ams_getMode(ams_deviceCtx_t * ctx, ams_mode_t * mode) {
    *mode = ctx->mode;
    return false;
}


bool ams_deviceGetRegdump(ams_deviceCtx_t * ctx) {
   uint16_t i = 0;
   uint8_t temp = 0;
   for(i =0x80 ; i<=0xff ; i++){
   	AMS_PORT_getByte(ctx->portHndl, i, &temp,1);
       AMS_PORT_log_2("dump 0x%x , 0x%x\n",i,temp);	
   }
    return false;
}




	
uint32_t ams_getResult(ams_deviceCtx_t * ctx) {
    uint32_t returnValue = ctx->updateAvailable;
    ctx->updateAvailable = 0;
    return returnValue;
}

