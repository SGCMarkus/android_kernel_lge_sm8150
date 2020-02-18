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
 * Core Control Block ALS
 */

/*
 * @@AMS_REVISION_Id: ef30c5e6def3a5c6a6331d1716177dc279679e1f
 */

#include "core_control_block_als.h"
#include "ams_port_platform.h"
#include "ams_device_control_block.h"
#include "ams_fft.h"

typedef struct _fifo{
    uint16_t AdcClear;
    uint16_t AdcRed;
#if defined(CONFIG_AMS_ALS_CRWBI)
    uint16_t AdcGreenWb;
#else
    uint16_t AdcGreen;
#endif
    uint16_t AdcBlue;
#if defined(CONFIG_AMS_ALS_CRGBW)
    uint16_t AdcWb;
#endif
    uint16_t AdcFlicker;
} adcDataSet_t;

#if defined(CONFIG_AMS_ALS_CRGBW)
#define AMS_PORT_LOG_CRGB_W(dataset) \
        AMS_PORT_log_4("ccb_alsHandle: C, R,G,B = %u, %u,%u,%u" \
            , dataset.AdcClear \
            , dataset.AdcRed \
            , dataset.AdcGreen \
            , dataset.AdcBlue \
            ); \
        AMS_PORT_log_1(" WB = %u\n", dataset.AdcWb); \
        AMS_PORT_log_1(" Flicker = %u\n", dataset.AdcFlicker);

#elif defined(CONFIG_AMS_ALS_CRWBI)
#define AMS_PORT_LOG_CRGB_W(dataset) \
        AMS_PORT_log_4("ccb_alsHandle: C, R,G/WB,B = %u, %u,%u,%u\n" \
            , dataset.AdcClear \
            , dataset.AdcRed \
            , dataset.AdcGreenWb \
            , dataset.AdcBlue \
            )
#else
#define AMS_PORT_LOG_CRGB_W(dataset) \
        AMS_PORT_log_4("ccb_alsHandle: C,R,G,B = %u, %u,%u,%u\n" \
            , dataset.AdcClear \
            , dataset.AdcRed \
            , dataset.AdcGreen \
            , dataset.AdcBlue \
            )
#endif

#define FLICKER_SAMPLES 103
#define FFT_BIN_COUNT 512
#define MAX_DATA_SETS (32 / 8)


#define BYTE 2


int16_t FlickerSampleData[FFT_BIN_COUNT];
uint16_t fifodata[128];

void ccb_alsInit_FIFO(void * dcbCtx, ams_ccb_als_init_t* initData){
    ams_deviceCtx_t * ctx = (ams_deviceCtx_t*)dcbCtx;
    ams_ccb_als_ctx_t * ccbCtx = &ctx->ccbAlsCtx;
    amsAlsInitData_t initAlsData = {0,};

    
    if (initData){
        memcpy(&ccbCtx->initData, initData, sizeof(ams_ccb_als_init_t));
    } else {
        ccbCtx->initData.calibrate = false;
    }

    initAlsData.gain = ccbCtx->initData.configData.gain;
#ifdef AMS_ALS_GAIN_V2
    ctx->alsGain = initAlsData.gain;
#endif
    initAlsData.time_us = ccbCtx->initData.configData.uSecTime;
    initAlsData.calibration.adcMaxCount = ccbCtx->initData.calibrationData.adcMaxCount;
    initAlsData.calibration.calibrationFactor = ccbCtx->initData.calibrationData.calibrationFactor;
    initAlsData.calibration.Time_base = ccbCtx->initData.calibrationData.Time_base;
    initAlsData.calibration.thresholdLow = ccbCtx->initData.calibrationData.thresholdLow;
    initAlsData.calibration.thresholdHigh = ccbCtx->initData.calibrationData.thresholdHigh;
    initAlsData.calibration.Wbc = ccbCtx->initData.calibrationData.Wbc;

AMS_PORT_log_3("ccb_alsInit time %d, gain value %d , autogain %d\n",initAlsData.time_us,initAlsData.gain,ctx->ccbAlsCtx.initData.autoGain);
#ifdef   AMS_SW_FLICKER

AMS_PORT_log("!!!!start ccb_alsInit_FIFO!!!!!");
    ams_setField(ctx->portHndl, DEVREG_PCFG1, 0x08, 0x08);
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG0, 0x80); /*fifo mode*/
    ams_setByte(ctx->portHndl, DEVREG_CFG4, 0x80);
    ams_setByte(ctx->portHndl, DEVREG_WTIME, 0x00);
    ams_setByte(ctx->portHndl, DEVREG_CFG8, 0xC8); //FIFO_THR:16, FD_AGC_DISABLE:YES , AUTOAGAIN_DISABLE
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG3, (((AGAIN_16)<<3) | (1<<0)));
    ccbCtx->initData.configData.uSecTime = 1000;
    
    AMS_SET_ALS_TIME(0);
    AMS_SET_ALS_STEP_TIME(ccbCtx->initData.configData.uSecTime); /*2.5msec*/   
    ams_setByte(ctx->portHndl, DEVREG_CONTROL, 0x02); //FIFO Buffer , FINT, FIFO_OV, FIFO_LVL all clear
    AMS_SET_ALS_PERS(0x00);
    /* force interrupt */
    ccbCtx->shadowAiltReg = 0xffff;
    ccbCtx->shadowAihtReg = 0;
    AMS_SET_ALS_THRS_LOW(ccbCtx->shadowAiltReg);
    AMS_SET_ALS_THRS_HIGH(ccbCtx->shadowAihtReg);
     ctx->flickerCtx.flicker_finished = 0;
    AMS_DISABLE_FLICKER_AUTOGAIN(HIGH);
#endif

    AMS_SET_ALS_TIME(ccbCtx->initData.configData.uSecTime);
    AMS_SET_ALS_PERS(0x01);
    AMS_SET_ALS_GAIN(ctx->ccbAlsCtx.initData.configData.gain);

    ccbCtx->shadowAiltReg = 0xffff;
    ccbCtx->shadowAihtReg = 0;
    AMS_SET_ALS_THRS_LOW(ccbCtx->shadowAiltReg);
    AMS_SET_ALS_THRS_HIGH(ccbCtx->shadowAihtReg);
    ccbCtx->state = AMS_CCB_ALS_RGB;

} 

void ccb_alsInit(void * dcbCtx, ams_ccb_als_init_t* initData){
    ams_deviceCtx_t * ctx = (ams_deviceCtx_t*)dcbCtx;
    ams_ccb_als_ctx_t * ccbCtx = &ctx->ccbAlsCtx;
    amsAlsInitData_t initAlsData = {0,};
    amsAlsAlgoInfo_t infoAls = {0,};

    AMS_PORT_log("ccb_alsInit \n");

    if (initData){
        memcpy(&ccbCtx->initData, initData, sizeof(ams_ccb_als_init_t));
    } else {
        ccbCtx->initData.calibrate = false;
    }

    initAlsData.adaptive = false;
    initAlsData.irRejection = false;
    initAlsData.gain = ccbCtx->initData.configData.gain;
#ifdef AMS_ALS_GAIN_V2
    ctx->alsGain = initAlsData.gain;
#endif
    initAlsData.time_us = ccbCtx->initData.configData.uSecTime;
    initAlsData.calibration.adcMaxCount = ccbCtx->initData.calibrationData.adcMaxCount;

    initAlsData.calibration.calibrationFactor = ccbCtx->initData.calibrationData.calibrationFactor;
    initAlsData.calibration.Time_base = ccbCtx->initData.calibrationData.Time_base;
    initAlsData.calibration.thresholdLow = ccbCtx->initData.calibrationData.thresholdLow;
    initAlsData.calibration.thresholdHigh = ccbCtx->initData.calibrationData.thresholdHigh;
    //initAlsData.calibration.calibrationFactor = ccbCtx->initData.calibrationData.calibrationFactor;
#if defined CONFIG_AMS_OPTICAL_SENSOR_ALS_CLEAR
    initAlsData.calibration.D_factor = ccbCtx->initData.calibrationData.D_factor;
    initAlsData.calibration.L0_factor = ccbCtx->initData.calibrationData.L0_factor;
    initAlsData.calibration.L1_factor = ccbCtx->initData.calibrationData.L1_factor;
    initAlsData.calibration.L2_factor = ccbCtx->initData.calibrationData.L2_factor;
    initAlsData.calibration.L3_factor = ccbCtx->initData.calibrationData.L3_factor;
#endif
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_RGB
    initAlsData.calibration.Cc = ccbCtx->initData.calibrationData.Cc;
    initAlsData.calibration.Rc = ccbCtx->initData.calibrationData.Rc;
    initAlsData.calibration.Gc = ccbCtx->initData.calibrationData.Gc;
    initAlsData.calibration.Bc = ccbCtx->initData.calibrationData.Bc;
#ifdef CONFIG_AMS_ALS_CRGBW
    initAlsData.calibration.Wbc = ccbCtx->initData.calibrationData.Wbc;
#endif
#if defined(CONFIG_AMS_ALS_CRGBW) || defined(CONFIG_AMS_ALS_CRGIB) || defined(CONFIG_AMS_ALS_CRGB)
    initAlsData.calibration.C_coef = ccbCtx->initData.calibrationData.C_coef;
#endif
#ifdef CONFIG_AMS_ALS_CRGIB
    initAlsData.calibration.IRc = ccbCtx->initData.calibrationData.IRc;
#endif
    initAlsData.calibration.R_coef = ccbCtx->initData.calibrationData.R_coef;
    initAlsData.calibration.G_coef = ccbCtx->initData.calibrationData.G_coef;
    initAlsData.calibration.B_coef = ccbCtx->initData.calibrationData.B_coef;
    initAlsData.calibration.D_factor = ccbCtx->initData.calibrationData.D_factor;
    initAlsData.calibration.CT_coef = ccbCtx->initData.calibrationData.CT_coef;
    initAlsData.calibration.CT_offset = ccbCtx->initData.calibrationData.CT_offset;
#endif
#ifdef CONFIG_AMS_ALS_CRWBI
    initAlsData.calibration.Wbc = ccbCtx->initData.calibrationData.Wbc;
    initAlsData.calibration.Wideband_C_factor = ccbCtx->initData.calibrationData.Wideband_C_factor;
    initAlsData.calibration.Wideband_R_factor = ccbCtx->initData.calibrationData.Wideband_R_factor;
    initAlsData.calibration.Wideband_B_factor = ccbCtx->initData.calibrationData.Wideband_B_factor;
#endif
    amsAlg_als_getAlgInfo (&infoAls);

#ifdef CONFIG_AMS_OPTICAL_SENSOR_3407
    /* get gain from HW register if so configured */
    if (ctx->ccbAlsCtx.initData.autoGain){
        uint32_t scaledGain;
#ifdef AMS_ALS_GAIN_V2
        uint8_t gainLevel;
        AMS_GET_ALS_GAIN(scaledGain, gainLevel);
#else
        uint8_t gain;
        AMS_GET_ALS_GAIN(scaledGain, gain);
#endif
        ctx->ccbAlsCtx.ctxAlgAls.gain = scaledGain;
    }
#endif
    amsAlg_als_initAlg(&ccbCtx->ctxAlgAls, &initAlsData);

    AMS_SET_ALS_TIME(ccbCtx->initData.configData.uSecTime);
    AMS_SET_ALS_PERS(0x01);
    AMS_SET_ALS_GAIN(ctx->ccbAlsCtx.initData.configData.gain);

    ccbCtx->shadowAiltReg = 0xffff;
    ccbCtx->shadowAihtReg = 0;
    AMS_SET_ALS_THRS_LOW(ccbCtx->shadowAiltReg);
    AMS_SET_ALS_THRS_HIGH(ccbCtx->shadowAihtReg);

    ccbCtx->state = AMS_CCB_ALS_RGB;
}

void ccb_alsGetConfig(void * dcbCtx, ams_ccb_als_config_t * configData){
    ams_ccb_als_ctx_t * ccbCtx = &((ams_deviceCtx_t*)dcbCtx)->ccbAlsCtx;

    configData->threshold = ccbCtx->initData.configData.threshold;
}

void ccb_alsSetConfig(void * dcbCtx, ams_ccb_als_config_t * configData){
    ams_ccb_als_ctx_t * ccbCtx = &((ams_deviceCtx_t*)dcbCtx)->ccbAlsCtx;

    ccbCtx->initData.configData.threshold = configData->threshold;
}



bool ccb_FlickerFIFOEvent(void * dcbCtx, ams_ccb_als_dataSet_t * alsData, uint16_t *data)
{
    ams_deviceCtx_t * ctx = (ams_deviceCtx_t*)dcbCtx;

    int i =0, j =0;
    uint8_t fifo_lvl;
    uint8_t fifo_ov;
    //uint8_t fd_cfg3,astatus;
    uint16_t fifo_size,quotient,remainder;
#ifdef ALS_AUTOGAIN	
     static adcDataSet_t adcData;
     ams_ccb_als_ctx_t * ccbCtx = &((ams_deviceCtx_t*)dcbCtx)->ccbAlsCtx;

#endif

    uint8_t fifo_buffer[256]={0,};	
//    uint16_t fifo_wbuffer[128]={0,};	
#if 0
    if (ctx->ccbAlsCtx.initData.autoGain){
        uint32_t scaledGain;
#ifdef AMS_ALS_GAIN_V2
        uint8_t gainLevel;
        AMS_GET_ALS_GAIN(scaledGain, gainLevel);
#else
        uint8_t gain;
        AMS_GET_ALS_GAIN(scaledGain, gain);
#endif
        ctx->ccbAlsCtx.ctxAlgAls.gain = scaledGain;
    }
#endif

    ams_getByte(ctx->portHndl, DEVREG_FIFO_STATUS,&fifo_lvl); //current fifo count     
    ams_getByte(ctx->portHndl, DEVREG_STATUS4,&fifo_ov);
    //ams_getByte(ctx->portHndl, DEVREG_FD_CFG3, &fd_cfg3);	
    //ams_getByte(ctx->portHndl, DEVREG_ASTATUS, &astatus);	
	
    fifo_size = fifo_lvl * BYTE;
	
    quotient = fifo_size / 32; 
    remainder = fifo_size % 32; 

    //AMS_PORT_log_4("FIFO LVL  %d, FIFO_OV  %d,fd_cfg3 %d, astatus %d\n",fifo_lvl,fifo_ov,fd_cfg3,astatus);

   if(fifo_lvl >=  128)//>128
   {
        //AMS_PORT_log_2("~~~~FIFO ~level ready to %d overflow %d \n",fifo_lvl,fifo_ov);
   
        if(quotient == 0) // fifo size is less than 32 , reading remainder 
        {
        	ams_getBuf(ctx->portHndl, DEVREG_FDATAL, (uint8_t*)&fifo_buffer[0], remainder);
        }
        else
        {
        	for(i=0; i<quotient;i++)
        	{
        		ams_getBuf(ctx->portHndl, DEVREG_FDATAL, (uint8_t*)&fifo_buffer[i*32], 32);
        	}
        	if(remainder != 0)        
        		ams_getBuf(ctx->portHndl, DEVREG_FDATAL, (uint8_t*)&fifo_buffer[i*32], remainder);
        }

	   for(i=0,j=0; i<fifo_lvl*BYTE;j++) // read 256 byte 
         {
         
    	          fifodata[j]= (uint16_t)((fifo_buffer[i]<<0) |(fifo_buffer[i+1]<<8));
    	          //AMS_PORT_log_2("ccb_FlickerFIFOEvent: data[%d] = %d \n" , j, fifodata[j]);
    		   i = i+2;
          }
	   //memcpy(fifodata,data,sizeof(data));
         ctx->flickerCtx.flicker_finished = 1;
         //ctx->shadowIntenabReg &= ~(FIEN|ASIEN_FDSIEN);
         //ams_setField(ctx->portHndl, DEVREG_INTENAB, HIGH,ctx->shadowIntenabReg );/*disable*/

#if 0
         ctx->shadowIntenabReg = 0;
         ctx->shadowEnableReg = 0;	 
         ams_setByte(ctx->portHndl, DEVREG_INTENAB, ctx->shadowIntenabReg);		 
         mdelay(2);
         ams_setByte(ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg);
#endif		 
          //AMS_DISABLE_ALS();
    	   //AMS_PORT_log("ccb_FlickerFIFOEvent: Finished get fifo data!!!!");
return true;
   }


#ifdef ALS_AUTOGAIN  

		// Do S/W AGC 
		AMS_ALS_GET_CRGB_W(&adcData);
		{
				uint64_t temp;
				uint32_t recommendedGain;
				uint32_t max_count;
				uint32_t adcObjective;

         			max_count = (1024 * ccbCtx->initData.configData.uSecTime) / 2780;
				//uint32_t adcObjective = ctx->ccbAlsCtx.ctxAlgAls.saturation * 128;
				adcObjective= max_count * 128;
				adcObjective /= 160; /* about 80% (128 / 160) */
//				adcObjective /= 210; /* about 80% (128 / 160) */				

				if (adcData.AdcClear == 0){
						/* to avoid divide by zero */
						adcData.AdcClear = 1;
				}
				temp = adcObjective * 2048; /* 2048 to avoid floating point operation later on */
#ifdef __KERNEL__
				do_div(temp, adcData.AdcClear);
#else
				temp /= adcData.AdcClear;
#endif
				temp *= ctx->ccbAlsCtx.ctxAlgAls.gain;
#ifdef __KERNEL__
				do_div(temp, 2048);
#else
				temp /= 2048;
#endif

				recommendedGain = temp & 0xffffffff;
#if 0
				AMS_PORT_log_4("ccb_FlickerFIFOEvent: Clear ADC : %d,Red  ADC : %d,Green ADC : %d, Blue ADC : %d\n", adcData.AdcClear,adcData.AdcRed,adcData.AdcGreen,adcData.AdcBlue);

				AMS_PORT_log_4("ccb_FlickerFIFOEvent: AMS_CCB_ALS_AUTOGAIN: sat=%u, objctv=%u, cur=%u, rec=%u"
								, ctx->ccbAlsCtx.ctxAlgAls.saturation
								, adcObjective
								, ctx->ccbAlsCtx.ctxAlgAls.gain
								, recommendedGain
							  );
#endif
				recommendedGain = alsGainToReg(recommendedGain);
				recommendedGain = alsGain_conversion[recommendedGain];
				if (recommendedGain != ctx->ccbAlsCtx.ctxAlgAls.gain){
						//AMS_PORT_log_1("ccb_FlickerFIFOEvent: gain chg to: %u\n", recommendedGain);
						ctx->ccbAlsCtx.ctxAlgAls.gain = recommendedGain;
						//ccbCtx->alg_config.gain = recommendedGain/1000;
                                          AMS_DISABLE_ALS_FLICKER();
                                          //ctx->shadowIntenabReg &= ~(FIEN|ASIEN_FDSIEN);
                                          //ams_setField(ctx->portHndl, DEVREG_INTENAB, HIGH,ctx->shadowIntenabReg );/*disable*/
                                          ams_setByte(ctx->portHndl, DEVREG_CONTROL, 0x02); //FIFO Buffer , FINT, FIFO_OV, FIFO_LVL all clear
        					AMS_SET_ALS_GAIN(ctx->ccbAlsCtx.ctxAlgAls.gain);
						//AMS_SET_FD_GAIN(ctx->ccbAlsCtx.ctxAlgAls.gain);
                                          //ctx->shadowIntenabReg = (FIEN|ASIEN_FDSIEN);
                                          //ams_setField(ctx->portHndl, DEVREG_INTENAB, HIGH,ctx->shadowIntenabReg );/*enable*/

                                          AMS_ENABLE_ALS_FLICKER();
						
				}
				else
						;//AMS_PORT_log_1("ccb_alsHd: no chg, gain %u\n", ctx->ccbAlsCtx.ctxAlgAls.gain);
		}
#endif   

  // AMS_PORT_log_1("Last ccb_alsHd: buffer count %d \n" , buffer_count);
return false;
}

bool ccb_alsHandle(void * dcbCtx, ams_ccb_als_dataSet_t * alsData){
    ams_deviceCtx_t * ctx = (ams_deviceCtx_t*)dcbCtx;
    ams_ccb_als_ctx_t * ccbCtx = &((ams_deviceCtx_t*)dcbCtx)->ccbAlsCtx;
    amsAlsDataSet_t inputDataAls = {0,};
    static adcDataSet_t adcData = {0,}; /* QC - is this really needed? */

    //AMS_PORT_log_Msg_1(AMS_DEBUG, "ccb_alsHandle: case = %d\n", ccbCtx->state);
#if 0
    AMS_ALS_GET_CRGB_W((uint8_t *)&adcData);
    AMS_PORT_LOG_CRGB_W(adcData);
#endif
#ifdef CONFIG_AMS_OPTICAL_SENSOR_3407
    /* get gain from HW register if so configured */
    if (ctx->ccbAlsCtx.initData.autoGain){
        uint32_t scaledGain;
#ifdef AMS_ALS_GAIN_V2
        uint8_t gainLevel;
        AMS_GET_ALS_GAIN(scaledGain, gainLevel);
#else
        uint8_t gain;
        AMS_GET_ALS_GAIN(scaledGain, gain);
#endif
        ctx->ccbAlsCtx.ctxAlgAls.gain = scaledGain;
    }
#endif

    switch (ccbCtx->state){
    case AMS_CCB_ALS_INIT:
        AMS_DISABLE_ALS();
        AMS_SET_ALS_TIME(ccbCtx->initData.configData.uSecTime);
        AMS_SET_ALS_PERS(0x01);
        AMS_SET_ALS_GAIN(ctx->ccbAlsCtx.initData.configData.gain);
        /* force interrupt */
        ccbCtx->shadowAiltReg = 0xffff;
        ccbCtx->shadowAihtReg = 0;
        AMS_SET_ALS_THRS_LOW(ccbCtx->shadowAiltReg);
        AMS_SET_ALS_THRS_HIGH(ccbCtx->shadowAihtReg);
        ccbCtx->state = AMS_CCB_ALS_RGB;
        AMS_REENABLE_ALS();
        break;
    case AMS_CCB_ALS_RGB: /* state to measure RGB */
#ifdef HAVE_OPTION__ALWAYS_READ
    if ((alsData->statusReg & (AINT)) || ctx->alwaysReadAls)
#else
    if (alsData->statusReg & (AINT))
#endif
        {
        AMS_ALS_GET_CRGB_W(&adcData);
        inputDataAls.status = ALS_STATUS_RDY;
        inputDataAls.datasetArray = (alsData_t*)&adcData;
        AMS_PORT_LOG_CRGB_W(adcData);

       /* if (ctx->ccbAlsCtx.ctxAlgAls.previousGain !=
                    ctx->ccbAlsCtx.ctxAlgAls.gain) {
                    AMS_DISABLE_ALS();
                    AMS_PORT_log_Msg(AMS_DEBUG, "ccb_alsHandle: ALS Disalbe to Enable \n");
                    AMS_REENABLE_ALS();
         }*/

        amsAlg_als_processData(&ctx->ccbAlsCtx.ctxAlgAls, &inputDataAls);

        if (ctx->mode & MODE_ALS_LUX) {
            ctx->updateAvailable |= (1 << AMS_AMBIENT_SENSOR);
        }
#if defined ( CONFIG_AMS_OPTICAL_SENSOR_490x )
        /* In order to implement the proper interrupt reporting of LUX vs RGB vs IR HIDs on intel
           we need to clearly differentiate between the different type of available ALS results */
        if (ctx->mode & MODE_ALS_IR) {
            ctx->updateAvailable |= (1 << AMS_AMBIENT_SENSOR_IR);
        }
        if (ctx->mode & MODE_ALS_RGB) {
            ctx->updateAvailable |= (1 << AMS_AMBIENT_SENSOR_RGB);
        }
#endif

#if 0
        AMS_PORT_log_1( "ccb_alsHandle: Unstable?= %d\n", ccbCtx->ctxAlgAls.notStableMeasurement);
        AMS_PORT_log_1( "ccb_alsHandle: Clear    = %d\n", adcData.AdcClear);
        AMS_PORT_log_1( "ccb_alsHandle: Red      = %d\n", adcData.AdcRed);
#ifdef CONFIG_AMS_ALS_CRWBI
        AMS_PORT_log_1( "ccb_alsHandle: Green/Wb = %d\n", adcData.AdcGreenWb);
#else
        AMS_PORT_log_1( "ccb_alsHandle: Green    = %d\n", adcData.AdcGreen);
#endif
        AMS_PORT_log_1( "ccb_alsHandle: Blue     = %d\n", adcData.AdcBlue);
#ifdef CONFIG_AMS_ALS_CRGBW
        AMS_PORT_log_1( "ccb_alsHandle: Wideband = %d\n", adcData.AdcWb);
#endif
        AMS_PORT_log_1( "ccb_alsHandle: IR       = %d\n", ccbCtx->ctxAlgAls.results.IR);
        AMS_PORT_log_1( "ccb_alsHandle: mLux     = %d\n", ccbCtx->ctxAlgAls.results.mLux);
        AMS_PORT_log_1( "ccb_alsHandle: CT       = %d\n", ccbCtx->ctxAlgAls.results.CCT);
        AMS_PORT_log_1( "ccb_alsHandle: factor   = %d\n", ccbCtx->ctxAlgAls.calibration.calibrationFactor);
        AMS_PORT_log_1( "ccb_alsHandle: timeUs   = %d\n", ccbCtx->ctxAlgAls.time_us);
        AMS_PORT_log_1( "ccb_alsHandle: gain     = %d\n", ccbCtx->ctxAlgAls.gain);
        AMS_PORT_log_1( "ccb_alsHandle: cpl      = %d\n", ccbCtx->ctxAlgAls.cpl);
        AMS_PORT_log_1( "ccb_alsHandle: thshld   = %d\n", ccbCtx->initData.configData.threshold);
#endif
        AMS_PORT_log_1( "ccb_alsHandle: timeUs   = %d\n", ccbCtx->ctxAlgAls.time_us);
        AMS_PORT_log_1( "ccb_alsHandle: gain     = %d\n", ccbCtx->ctxAlgAls.gain);
        AMS_PORT_log_1( "ccb_alsHandle: cpl      = %d\n", ccbCtx->ctxAlgAls.cpl);

        ccbCtx->state = AMS_CCB_ALS_RGB;

         //AMS_PORT_log_Msg_2(AMS_DEBUG, "ccb_alsHandle: prevgain %d, gain %d \n",ctx->ccbAlsCtx.ctxAlgAls.previousGain,ctx->ccbAlsCtx.ctxAlgAls.gain);
#if !defined CONFIG_AMS_OPTICAL_SENSOR_3407
        /* Software AGC */
        if (ccbCtx->initData.autoGain == true)
        {
            uint64_t temp;
            uint32_t recommendedGain;
            uint32_t adcObjective = ctx->ccbAlsCtx.ctxAlgAls.saturation * 128;
            adcObjective /= 160; /* about 80% (128 / 160) */

            if (adcData.AdcClear == 0){
                /* to avoid divide by zero */
                adcData.AdcClear = 1;
            }
            temp = adcObjective * 2048; /* 2048 to avoid floating point operation later on */

#ifdef __KERNEL__
    do_div(temp, adcData.AdcClear);
#else
        temp /= adcData.AdcClear;
#endif
        temp *= ctx->ccbAlsCtx.ctxAlgAls.gain;
#ifdef __KERNEL__
    do_div(temp, 2048);
#else
        temp /= 2048;
#endif

        recommendedGain = temp & 0xffffffff;
#if 0
        AMS_PORT_log_Msg_4(AMS_DEBUG, "ccb_alsHandle: AMS_CCB_ALS_AUTOGAIN: sat=%u, objctv=%u, cur=%u, rec=%u\n"
            , ctx->ccbAlsCtx.ctxAlgAls.saturation
            , adcObjective
            , ctx->ccbAlsCtx.ctxAlgAls.gain
            , recommendedGain
            );
#endif
#ifdef AMS_ALS_GAIN_V2
            recommendedGain = alsGainToLevel(recommendedGain);
            recommendedGain = alsLevelToGain(recommendedGain);
#else
            recommendedGain = alsGainToReg(recommendedGain);
            recommendedGain = alsGain_conversion[recommendedGain];
#endif
            if (recommendedGain != ctx->ccbAlsCtx.ctxAlgAls.gain){
                AMS_PORT_log_1( "ccb_alsHandle: AMS_CCB_ALS_AUTOGAIN: change gain to: %u\n", recommendedGain);
                ctx->ccbAlsCtx.ctxAlgAls.gain = recommendedGain;
                AMS_DISABLE_ALS();
                AMS_SET_ALS_GAIN(ctx->ccbAlsCtx.ctxAlgAls.gain);
                AMS_REENABLE_ALS();
                if (ctx->ccbAlsCtx.ctxAlgAls.previousGain !=
                    ctx->ccbAlsCtx.ctxAlgAls.gain) {
                    ctx->updateAvailable |= (1 << AMS_ALS_RGB_GAIN_CHANGED);

                    // run process data over copy just to detemine the new cpl value
                    // in order to compute the next range but don't mess up with the current
                    // measurement which was done over the previous gain.
                    adcDataSet_t tmpAdcData;
                    amsAlsContext_t tmp = ctx->ccbAlsCtx.ctxAlgAls;
                    amsAlsDataSet_t tmpInputDataAls;
                    tmpInputDataAls.status = ALS_STATUS_RDY;

                    tmpInputDataAls.datasetArray = (alsData_t*)&tmpAdcData;
                    // compute the ranges based on the max adc count value
                    tmpAdcData.AdcClear = ctx->ccbAlsCtx.ctxAlgAls.saturation;
                    tmpAdcData.AdcRed   = ctx->ccbAlsCtx.ctxAlgAls.saturation / 3; //divide by more reasonable coefficient
                    tmpAdcData.AdcGreen = ctx->ccbAlsCtx.ctxAlgAls.saturation / 3;
                    tmpAdcData.AdcBlue  = ctx->ccbAlsCtx.ctxAlgAls.saturation / 3;
                    amsAlg_als_processData(&tmp, &tmpInputDataAls);
                    ctx->ccbAlsCtx.ctxAlgAls.results.mMaxLux = tmp.results.mLux;
                }
            }
            else
              AMS_PORT_log_1( "ALS_AUTOGAIN: no change, gain=%u\n", ctx->ccbAlsCtx.ctxAlgAls.gain);

          }
#endif /*CONFIG_AMS_OPTICAL_SENSOR_3407*/
          break;
    }

    default:
        ccbCtx->state = AMS_CCB_ALS_RGB;
    break;
    }
    return false;
}

void ccb_alsGetResult(void * dcbCtx, ams_ccb_als_result_t * exportData){
    ams_ccb_als_ctx_t * ccbCtx = &((ams_deviceCtx_t*)dcbCtx)->ccbAlsCtx;

    /* export data */
    exportData->mLux = ccbCtx->ctxAlgAls.results.mLux;
    exportData->colorTemp = ccbCtx->ctxAlgAls.results.CCT;
    exportData->clear = ccbCtx->ctxAlgAls.results.irrClear;
    exportData->blue = ccbCtx->ctxAlgAls.results.irrBlue;
    exportData->green = ccbCtx->ctxAlgAls.results.irrGreen;
    exportData->red = ccbCtx->ctxAlgAls.results.irrRed;
    exportData->ir = ccbCtx->ctxAlgAls.results.IR;
    exportData->wideband = ccbCtx->ctxAlgAls.results.irrWideband;
    exportData->rawClear = ccbCtx->ctxAlgAls.results.rawClear;
    exportData->rawRed = ccbCtx->ctxAlgAls.results.rawRed;
    exportData->rawGreen = ccbCtx->ctxAlgAls.results.rawGreen;
    exportData->rawBlue = ccbCtx->ctxAlgAls.results.rawBlue;
}



#ifdef AMS_SW_FLICKER

bool  ccb_sw_flicker_GetResult(void * dcbCtx)
{

    ams_deviceCtx_t * ctx = (ams_deviceCtx_t*)dcbCtx;
   // ams_ccb_als_ctx_t * ccbCtx = &((ams_deviceCtx_t*)dcbCtx)->ccbAlsCtx;
    uint8_t fd_cfg3,astatus;


       int i;
//	int div;
//	s32 magRt;
	int minG;
	int maxG;
	int midMaxMinG;
	int shiftFactor;
	int maxGScaled;
	int minGScaled;
	int wbSum;
	
      uint16_t mHz;
      s32 max32;
      s32 magSq;
      s16 imag;
	s16 real;
	uint32_t idx2;
	int idx;
	int max;
	uint32_t sampleFreq;
	uint32_t freqStep;	
       uint8_t si =0;
   /*
	** 1. find min and max of input data
	** 2. scale data down
	** 3. then use max to scale data up
	*/
    ams_getByte(ctx->portHndl, DEVREG_FD_CFG3, &fd_cfg3);	
    ams_getByte(ctx->portHndl, DEVREG_ASTATUS, &astatus);	
	
     if(fifodata[0]==0 ){
              AMS_PORT_log_2("ccb_sw_flicker_GetResult : no fifo data %d, flag %d",fifodata[0],ctx->flickerCtx.flicker_finished );
		return false;
     	}


	// find min and max of input data
	minG  = maxG  = wbSum = fifodata[0];
	for (i = 1; i < FLICKER_SAMPLES; i++)
	{
		if (fifodata[i] > maxG)
			maxG = fifodata[i];
		else if (fifodata[i] < minG)
			minG = fifodata[i];
		wbSum += fifodata[i];
	}

	//	 | maxG
	//	 |	 *
	//	 | *   *   *
	//	 |		 *
	//	 |		minG
	//	 |
	//	 |
	//	 | maxGScaled
	//	 |	 *
	// 0 +-*---*---*-------- X
	//	 |		 *
	//	 |	  minGScaled
	//	 |
	// Scale the input data, so that maxG and minG
	// are around the X axis
	midMaxMinG = (maxG - minG) / 2;
	minGScaled = minG + midMaxMinG;
	maxGScaled = maxG - minGScaled;

	// Find largest shift factor so that:
	// maxG value, scaled down, times 2 raised to the shift factor,
	// will be less than 0x7fff, largest 16 bit positive integer
	shiftFactor = 0;
	for (i = 0; i < 15; i++)
	{
		if (((1 << (i + 1)) * maxGScaled) > 0x7FFF)
		{
			shiftFactor = i;
			break;
		}
	}
#if 1

	for (i = 0; i < FFT_BIN_COUNT; i++)
	{
		FlickerSampleData[i] = (int16_t) (((int16_t) fifodata[i % FLICKER_SAMPLES] - minGScaled) << shiftFactor);

	}

	ams_rfft_int16_512(FlickerSampleData);

/*
	** The sample frequency step for the fft can be calculated as follows:
	**     1 / (((256 - atime) / [1,2,4]) * 2.816ms + 0.286ms)
	** The latter is a constant factor between meaurement starts.
	*/

	// [1,2,3] => [1,2,4] (0 skips this section)
	//div = (chip->flicker.mode == 3) ? 4 : chip->flicker.mode;

	// Mul the sampleFreq by 1000
	ctx->ccbAlsCtx.initData.configData.uSecTime  = 1000;
	sampleFreq = 1000000000 / (1000) ;	 // 2500us and 160us
	freqStep   = sampleFreq / FFT_BIN_COUNT;

     AMS_PORT_log_1("ccb_sw_flicker_GetResult : minG %u", minG);
     AMS_PORT_log_1("ccb_sw_flicker_GetResult : maxG %u", maxG);
     AMS_PORT_log_1("ccb_sw_flicker_GetResult : midMaxMinG %u", midMaxMinG);
     AMS_PORT_log_1("ccb_sw_flicker_GetResult : maxGScaled %u", maxGScaled);	 
     AMS_PORT_log_1("ccb_sw_flicker_GetResult : shiftFactor %u", shiftFactor);
     AMS_PORT_log_1("ccb_sw_flicker_GetResult : freqStep %u", freqStep);
     AMS_PORT_log_1("ccb_sw_flicker_GetResult : fd_gain 0x%x", fd_cfg3);
     AMS_PORT_log_1("ccb_sw_flicker_GetResult : astatus 0x%x", astatus);
	 
       max = -1;
	max32 = -1;
	idx = 0;
	idx2 = 0;

	// Result from FFt is in dataSamples[].
	// The values are complex numbers
	// We need to find the magnitude of each complex number
	// magnitude = sqrt(real^2 + imaginary^2)
	// Since we are only interested in the magnitude,
	// we can use real^2 + imaginary^2
	for (i = (20 * 2); i < FFT_BIN_COUNT - 40; i += 2)
	{
		real = FlickerSampleData[i + 0];
		imag = FlickerSampleData[i + 1];
		magSq = (u32) (((u32) real * real) + ((u32) imag * imag));
		// magRt = (u16)ams_isqrt(magSq);

     //AMS_PORT_log_2("ccb_sw_flicker_GetResult : magSq[%d]	\t%d\n", i/2, magSq);

#if 0
		if (magRt > max) {
			max = magRt;
			idx = i/2;
		}
#endif

		if (magSq > max32)
		{
			max32 = magSq;
			idx2 = i/2;
		}
	}

	// The frequency is the bin index times the frequency step
	mHz = idx2 * freqStep / 1000;

	// Round up to nearest multiple of 5
	mHz = ((mHz+ 5) / 10) * 10;	
	ctx->flickerCtx.mHzbysw = mHz;
	ctx->flickerCtx.flicker_finished = 0;
       //ams_setByte(ctx->portHndl, DEVREG_CONTROL, 0x02); //FIFO Buffer , FINT, FIFO_OV, FIFO_LVL all clear

	// If the frequency is not 100 or 120, report back ZERO.
	// If you want to report back the measured frequency,
	// do NOT do this test!
	/*if ((chip->flicker.freq != 100) && (chip->flicker.freq != 120))
	{
		chip->flicker.freq = 0;
	}*/
AMS_PORT_log_1("ccb_sw_flicker_GetResult:sw_freq= %u \n",ctx->flickerCtx.mHzbysw  );
	
#if 0
         ctx->shadowIntenabReg = 0;
         ctx->shadowEnableReg = 0;	 
         ams_setByte(ctx->portHndl, DEVREG_INTENAB, ctx->shadowIntenabReg);		 
         mdelay(2);
         ams_setByte(ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg);
#endif
	//si = astatus = (astatus & 0x0F);
	si = astatus = ((fd_cfg3 >>3)& 0x1F);

	if(wbSum <=6931){
		astatus = MIN(10,++si);
	}
	if(wbSum > 34560){
		astatus = MIN(0,--si);
	}
	
    //ams_setField(ctx->portHndl, DEVREG_FD_CFG3, 0x08, 0x08);

    //ams_setByte(ctx->portHndl, DEVREG_CFG1, (((FD_GAIN_16)<<3) | (1<<0)));
    AMS_SET_FLICKER_INDEX_TO_FDGAIN(si);
    ams_setByte(ctx->portHndl, DEVREG_CONTROL, 0x02); //FIFO Buffer , FINT, FIFO_OV, FIFO_LVL all clear
	
#endif	
return true;
}
#endif



