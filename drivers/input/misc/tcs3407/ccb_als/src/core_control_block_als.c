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
#if !defined (AMS_BIN_2048_MODE1) && !defined (AMS_BIN_2048_MODE2) 
#include "ams_fft.h"
#endif

#if defined (AMS_BIN_2048_MODE1) ||defined (AMS_BIN_2048_MODE2) 
#include <linux/kfifo.h>
#include "ams_fft_2048.h"
#endif

#if defined (AMS_BIN_2048_MODE1) ||defined (AMS_BIN_2048_MODE2) 
static DECLARE_KFIFO(ams_fifo,u8, 2*PAGE_SIZE);
#endif


#define FLICKER_SAMPLES 128
#define FFT_BIN_COUNT 512
#define FFT_BIN_HALF FFT_BIN_COUNT/2

#define MAX_DATA_SETS (32 / 8)


#define BYTE 2


int16_t FlickerSampleData[FFT_BIN_COUNT];
uint16_t fifodata[FLICKER_SAMPLES];

#define CLOCK_FREQ 720000


void ccb_alsInit_FIFO(void * dcbCtx, ams_ccb_als_init_t* initData){
    ams_deviceCtx_t * ctx = (ams_deviceCtx_t*)dcbCtx;
    ams_ccb_als_ctx_t * ccbCtx = &ctx->ccbAlsCtx;
    amsAlsInitData_t initAlsData = {0,};
    amsAlsAlgoInfo_t infoAls = {0,};
    uint32_t scaledGain = 0;
    uint8_t gain = 0;
    
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
    ccbCtx->ctxAlgAls.time_us = ccbCtx->initData.configData.uSecTime = AMS_ALS_ATIME;/*50msec*/
    initAlsData.calibration.D_factor = AMS_ALS_D_FACTOR;

#ifdef CONFIG_AMS_OPTICAL_SENSOR_3407
        AMS_GET_ALS_GAIN(scaledGain, gain);
        ctx->ccbAlsCtx.ctxAlgAls.gain = scaledGain;
#endif
    amsAlg_als_getAlgInfo (&infoAls);
    amsAlg_als_initAlg(&ccbCtx->ctxAlgAls, &initAlsData);
    AMS_PORT_log_3("ccb_alsInit time %d, gain value %d , autogain %d\n",initAlsData.time_us,initAlsData.gain,ctx->ccbAlsCtx.initData.autoGain);

    ams_setField(ctx->portHndl, DEVREG_PCFG1, 0x08, 0x08);
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG0, 0x80); /*fifo mode*/
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG0,  ( 0x80 | (FD_SAMPLES_128) | (FD_COMPARE_6_32NDS)));		

    ams_setByte(ctx->portHndl, DEVREG_CFG4, 0x80);
    ams_setByte(ctx->portHndl, DEVREG_CFG8, 0xCC); //FIFO_THR:16, FD_AGC_DISABLE:YES , AUTOAGAIN_ENABLE

#if !defined(  AMS_BIN_2048_MODE1) &&!defined(  AMS_BIN_2048_MODE1) 
    ctx->flickerCtx.sampling_time = 1000; // 2k
    ctx->flickerCtx.sampling_time = (CLOCK_FREQ / (2 * ctx->flickerCtx.sampling_time)) - 1;	
    ctx->flickerCtx.gain = AGAIN_16;
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG3, ((ctx->flickerCtx.gain <<3)));
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG3, (ctx->flickerCtx.sampling_time >>8) & 0x03);	
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG1, ctx->flickerCtx.sampling_time & 0xff);
    AMS_PORT_log("!!!!start 128 fft ccb_alsInit_FIFO!!!!!");
	
#endif

   
#ifdef AMS_BIN_2048_MODE1
    ctx->flickerCtx.sampling_time = 2000; // 2k
    ctx->flickerCtx.sampling_time = (CLOCK_FREQ / (2 * ctx->flickerCtx.sampling_time)) - 1;	
    ctx->flickerCtx.gain = AGAIN_16;
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG3, ((ctx->flickerCtx.gain <<3)));
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG3, (ctx->flickerCtx.sampling_time >>8) & 0x03);	
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG1, ctx->flickerCtx.sampling_time & 0xff);
    AMS_PORT_log("!!!!start mode1  fft ccb_alsInit_FIFO!!!!!");
	
#endif

#ifdef AMS_BIN_2048_MODE2
    ctx->flickerCtx.sampling_time = 4000; // 4k
    ctx->flickerCtx.sampling_time = (CLOCK_FREQ / (2 * ctx->flickerCtx.sampling_time)) - 1;	
    ctx->flickerCtx.gain = AGAIN_16;
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG3, ((ctx->flickerCtx.gain <<3)));
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG3, (ctx->flickerCtx.sampling_time >>8) & 0x03);	
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG1, ctx->flickerCtx.sampling_time & 0xff);
    AMS_PORT_log("!!!!start mode2  fft ccb_alsInit_FIFO!!!!!");
	
#endif

#ifdef AMS_BIN_2048_MODE3
    ctx->flickerCtx.sampling_time = 3000; // 3k
    ctx->flickerCtx.sampling_time = (CLOCK_FREQ / (2 * ctx->flickerCtx.sampling_time)) - 1;	
    ctx->flickerCtx.gain = AGAIN_16;
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG3, ((ctx->flickerCtx.gain <<3)));
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG3, (ctx->flickerCtx.sampling_time >>8) & 0x03);	
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG1, ctx->flickerCtx.sampling_time & 0xff);
    AMS_PORT_log("!!!!start mode3  fft ccb_alsInit_FIFO!!!!!");
	
#endif  

    AMS_SET_ALS_TIME(ccbCtx->initData.configData.uSecTime);
    AMS_SET_ALS_STEP_TIME(AMS_ALS_STEP_TIME); 

    ams_setByte(ctx->portHndl, DEVREG_CONTROL, 0x02); //FIFO Buffer , FINT, FIFO_OV, FIFO_LVL all clear
#if defined (AMS_BIN_2048_MODE1) ||defined (AMS_BIN_2048_MODE2)     
    INIT_KFIFO(ams_fifo);/*kfifo init*/
#endif
    /* force interrupt */
     ctx->flickerCtx.flicker_finished = 0;
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

    ccbCtx->initData.configData.uSecTime = AMS_ALS_ATIME;/*50msec*/
    AMS_SET_ALS_TIME(ccbCtx->initData.configData.uSecTime);
    AMS_SET_ALS_STEP_TIME(AMS_ALS_STEP_TIME); 
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

#if defined (AMS_BIN_2048_MODE1)  ||defined (AMS_BIN_2048_MODE2) 
bool ccb_FlickerFIFO4096Event(void * dcbCtx, ams_ccb_als_dataSet_t * alsData)
{
    ams_deviceCtx_t * ctx = (ams_deviceCtx_t*)dcbCtx;

    int i =0;
    int kfifo_size =0 ;
    uint8_t fifo_lvl =0;
    //uint8_t fifo_ov =0 ;
    uint16_t fifo_size,quotient,remainder =0;
    uint8_t fifo_buffer[512]={0,};	/* LGE_CHANGE, fixed kernel stack corruption, 2019-08-26, CST */
//    uint8_t fifo_buffer[256]={0,};
//    uint16_t fifo_wbuffer[128]={0,};	

    ams_getByte(ctx->portHndl, DEVREG_FIFO_STATUS,&fifo_lvl); //current fifo count     
    //ams_getByte(ctx->portHndl, DEVREG_STATUS4,&fifo_ov);
	
    fifo_size = fifo_lvl * BYTE;	
    quotient = fifo_size / 32; 
    remainder = fifo_size % 32; 

   //if(fifo_lvl >=  128)//>128
   if(fifo_lvl <=  128)//>128  /* LGE_CHANGE, fixed kernel stack corruption, 2019-08-26, CST */
   {

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


	   kfifo_in(&ams_fifo,fifo_buffer,fifo_lvl*BYTE);

	   if((kfifo_size = kfifo_len(&ams_fifo)) >= (PAGE_SIZE)) //4096 byte , 2048 level
	   {
	    	ctx->flickerCtx.flicker_finished = 1;
              AMS_PORT_log_1("FIFO now is full!!! ready  to calc freq   fifo size %d",kfifo_size);
	   }	  
      }
  // AMS_PORT_log_1("Last ccb_alsHd: buffer count %d \n" , buffer_count);
return false;
}
#endif 


bool ccb_FlickerFIFOEvent(void * dcbCtx, ams_ccb_als_dataSet_t * alsData)
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
        //AMS_PORT_msg_2("~~~~FIFO ~level ready to %d overflow %d \n",fifo_lvl,fifo_ov);
   
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
    	          //AMS_PORT_msg_2("ccb_FlickerFIFOEvent: data[%d] = %d \n" , j, fifodata[j]);
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

  // AMS_PORT_msg_1("Last ccb_alsHd: buffer count %d \n" , buffer_count);
return false;
}

bool ccb_alsHandle(void * dcbCtx, ams_ccb_als_dataSet_t * alsData){
    ams_deviceCtx_t * ctx = (ams_deviceCtx_t*)dcbCtx;
    ams_ccb_als_ctx_t * ccbCtx = &((ams_deviceCtx_t*)dcbCtx)->ccbAlsCtx;
    amsAlsDataSet_t inputDataAls = {0,};
    static adcDataSet_t adcData = {0,}; /* QC - is this really needed? */

   
#ifdef CONFIG_AMS_OPTICAL_SENSOR_3407
    /* get gain from HW register if so configured */
    if (ctx->ccbAlsCtx.initData.autoGain){
        uint32_t scaledGain;
        uint8_t gain;
        AMS_GET_ALS_GAIN(scaledGain, gain);
        ctx->ccbAlsCtx.ctxAlgAls.gain = scaledGain;
    }
#endif

    switch (ccbCtx->state){
    case AMS_CCB_ALS_INIT:
        break;
    case AMS_CCB_ALS_RGB: /* state to measure RGB */
        {
        AMS_ALS_GET_CRGB_W(&adcData);
        inputDataAls.status = ctx->shadowStatus1Reg; /*saturation condition check*/
        inputDataAls.datasetArray = (alsData_t*)&adcData;
        AMS_PORT_LOG_CRGB_W(adcData);

        amsAlg_als_processData(&ctx->ccbAlsCtx.ctxAlgAls, &inputDataAls);


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
        AMS_PORT_log_1( "ccb_alsHandle: timeUs   = %d\n", ccbCtx->ctxAlgAls.time_us);
        AMS_PORT_log_1( "ccb_alsHandle: gain     = %d\n", ccbCtx->ctxAlgAls.gain);
        AMS_PORT_log_1( "ccb_alsHandle: cpl      = %d\n", ccbCtx->ctxAlgAls.cpl);
#endif
        ccbCtx->state = AMS_CCB_ALS_RGB;
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
    exportData->saturation = ccbCtx->ctxAlgAls.results.saturation;
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



#if defined ( AMS_BIN_2048_MODE1) ||defined ( AMS_BIN_2048_MODE2)

ssize_t read_fifo( char *buf, int size)
{
	int len;
	int kfifo_Len;

       kfifo_Len =  kfifo_len(&ams_fifo);
	   
	if(kfifo_Len >=size){
		len = kfifo_out(&ams_fifo,buf,size);
		//len = kfifo_copy(&ams_fifo,buf,size);
	}else{
		len = 0;
	}
	AMS_PORT_log_2("read_fifo read size  %d , kfifo_Len =%d\n",len,kfifo_Len);
	return len;
}

static void set_gain(void * dcbCtx , uint8_t gain)
{

	ams_deviceCtx_t * ctx = (ams_deviceCtx_t*)dcbCtx;

	AMS_PORT_log_1("set_gain  %d \n",gain);
	ctx->flickerCtx.gain = gain;
	ams_setField(ctx->portHndl, DEVREG_FD_CFG3,   (ctx->flickerCtx.gain<<3) ,       MASK_FD_GAIN);
       ams_setByte(ctx->portHndl, DEVREG_CONTROL, 0x02); //FIFO Buffer , FINT, FIFO_OV, FIFO_LVL all clear

}

static int auto_gain_ctl(void * dcbCtx , uint16_t * data, int size)
{
	ams_deviceCtx_t * ctx = (ams_deviceCtx_t*)dcbCtx;

	uint8_t gain = ctx->flickerCtx.gain ;
	int i;
	int max =0 ;


	for(i=0; i< size;++i){
	//AMS_PORT_log_2("auto_gain_ctl: data[%d] = %d\n",i, data[i]);
		if(data[i] > max){
			max = data[i];
		}
	}

	AMS_PORT_log_3("auto_gain_ctl: gain  %d , max %d ,sample /3 %d ,\n",gain, max,ctx->flickerCtx.sampling_time / 3 );


	if((max < (ctx->flickerCtx.sampling_time / 3)) && gain < 12){
		gain++;
	}else if((max >= ctx->flickerCtx.sampling_time) && gain > 0 ){
		gain--;
	}else{
	       AMS_PORT_log("auto_gain_ctl nothing do!!!\n");
		return 0;
	}
	
	set_gain(ctx,gain);
	return 1;
	
}



int get_fft(void * ctx, char *out)
{

	static int16_t buffer[2048] ;
	ssize_t size = 0;
	
	memset(buffer,0x00,4096);
	size = read_fifo((char*)buffer,4096);

	if(size < 4096){
		return 0;
	}
	kfifo_reset(&ams_fifo);	

	if(!auto_gain_ctl(ctx,buffer,2048)){
		ams_rfft(buffer, 2048);
		ams_get_magnitude((int16_t*)buffer,(uint16_t*)out,1024);
	}
	return 1;


}

typedef struct _filicer_target {
    int target_fre;
    uint16_t min;// tolerrance -2.8%
    uint16_t max;// tolerrance +2.8%
}filicer_target;

#define TARGET_NUM 7 

filicer_target LG_TARGET[TARGET_NUM]={
  {100,96,103},
  {120,115,124},
  {164,158,169},
  {300,290,309},
  {330,319,340},
  {500,484,514},
  {600,581,617},
};
static int Target_fre(uint16_t mhz)
{
  int i; 
  
  for(i =0; i < TARGET_NUM;i++)
  {
    if( mhz > LG_TARGET[i].min && mhz < LG_TARGET[i].max)
    {
        return LG_TARGET[i].target_fre;
    }
  }

  return 0;
  
}

bool  ccb_sw_bin4096_flicker_GetResult(void * dcbCtx)
{
  ams_deviceCtx_t * ctx = (ams_deviceCtx_t*)dcbCtx;
  static uint16_t buf[1024] = {0,};
  int max =0;

  int i =0;
  uint16_t mHz;  
  uint16_t target_fre;  

	if(get_fft(ctx,(char*)buf)){
		buf[0] = 0;
		for(i =0 ; i < 1024; ++i){			
			if(buf[i] > buf[max]){
				max = i;

            	//AMS_PORT_log_1("ccb_sw_bin4096_flicker_GetResult: max  %d \n",max);					
			}
		}
	}
    mHz = ((max * (CLOCK_FREQ / ((ctx->flickerCtx.sampling_time + 1) * 2))) / 2048);

    target_fre = Target_fre(mHz);

    if(!target_fre)
    {
        //Round up to nearest multiple of 5
        mHz = ((mHz+ 5) / 10) * 10;	
    }
    else
    {
        mHz = target_fre;
    }
      
    ctx->flickerCtx.lastValid.mHzbysw = mHz;
	 AMS_PORT_log_1("ccb_sw_bin4096_flicker_GetResult: sw_freq %u \n",ctx->flickerCtx.lastValid.mHzbysw);

	 ams_setByte(ctx->portHndl, DEVREG_CONTROL, 0x02); //FIFO Buffer , FINT, FIFO_OV, FIFO_LVL all clear
	 ctx->flickerCtx.flicker_finished = 0;
    return true;
}

#else

static uint32_t get_sqrt(uint32_t x)
{
    uint32_t result;
    uint32_t tmp;

    result = 0;
    tmp = (1 << 30);
    while (tmp > x) {
        tmp >>= 2;
    }
    while (tmp != 0) {
        if (x >= (result + tmp)) {
            x -= result + tmp;
            result += 2 * tmp;
        }
        result >>= 1;
        tmp >>= 2;
    }
    //AMS_PORT_log_1("get_sqrt : result =  %u", result);
	
    return result;
}

static int get_stdev(uint16_t *buff, int mean, int size)
{
    int i;
    uint32_t sum = 0;

    for (i = 0; i < size; ++i) {
        sum += ((buff[i] - mean) * (buff[i] - mean));
    }
    sum = sum / (size - 1);
    return get_sqrt(sum);
}

static int get_mean(uint16_t *buff, int size)
{
    int i;
    int sum = 0;

    for (i = 0; i < size; ++i) {
        sum += buff[i];
    }
    //AMS_PORT_log_1("get_mean : sum/ size =  %u", sum / size);
    return sum / size;
}
bool  ccb_sw_flicker_GetResult(void * dcbCtx)
{

    ams_deviceCtx_t * ctx = (ams_deviceCtx_t*)dcbCtx;
   // ams_ccb_als_ctx_t * ccbCtx = &((ams_deviceCtx_t*)dcbCtx)->ccbAlsCtx;
    uint8_t fd_cfg3,astatus;


       int i,j;
//	int div;
//	s32 magRt[FFT_BIN_HALF];
	uint16_t magRt[FFT_BIN_HALF];

       int mean;
       int stdev;

	int minG;
	int maxG;
	int midMaxMinG;
	int shiftFactor;
	int maxGScaled;
	int minGScaled;
	int wbSum;
	
      s16 mHz = -1;
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
	uint8_t pre_si =0;
    uint16_t sampling_time = 0;
					 
   /*
	** 1. find min and max of input data
	** 2. scale data down
	** 3. then use max to scale data up
	*/
    ams_getByte(ctx->portHndl, DEVREG_FD_CFG3, &fd_cfg3);	
    ams_getByte(ctx->portHndl, DEVREG_ASTATUS, &astatus);	
	
     

	// find min and max of input data
	minG  = maxG  = wbSum = fifodata[0];
	for (i = 1; i < FLICKER_SAMPLES ; i++)
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
		FlickerSampleData[i] = (int16_t) (((int16_t) fifodata[i % (FLICKER_SAMPLES )] - minGScaled) << shiftFactor);

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
	//ctx->ccbAlsCtx.initData.configData.uSecTime  = 1000;
	if (ctx->flickerCtx.sampling_time == 179)
       sampling_time = 500;
	if (ctx->flickerCtx.sampling_time == 359)
       sampling_time = 1000;
	
	sampleFreq = 1000000000 / (sampling_time) ;	 // 2k(500usec) sampling 
	freqStep   = sampleFreq / FFT_BIN_COUNT;

    /*AMS_PORT_msg_1("ccb_sw_flicker_GetResult : minG %u", minG);
     AMS_PORT_msg_1("ccb_sw_flicker_GetResult : maxG %u", maxG);
     AMS_PORT_msg_1("ccb_sw_flicker_GetResult : midMaxMinG %u", midMaxMinG);
     AMS_PORT_msg_1("ccb_sw_flicker_GetResult : maxGScaled %u", maxGScaled);	 
     AMS_PORT_msg_1("ccb_sw_flicker_GetResult : shiftFactor %u", shiftFactor);
     AMS_PORT_msg_1("ccb_sw_flicker_GetResult : freqStep %u", freqStep);
     AMS_PORT_msg_1("ccb_sw_flicker_GetResult : fd_gain 0x%x", fd_cfg3);
     AMS_PORT_msg_1("ccb_sw_flicker_GetResult : astatus 0x%x", astatus);
     AMS_PORT_msg_1("ccb_sw_flicker_GetResult : fd_gain 0x%x, sampleFreq %d , freqStep %d", fd_cfg3,sampleFreq,freqStep);    
     */
	 
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
	//for (i = (20 * 2); i < FFT_BIN_COUNT - 40; i += 2)
	for (i = 2, j =0; i < FFT_BIN_COUNT ; i += 2, j++)
	{
		real = FlickerSampleData[i + 0];
		imag = FlickerSampleData[i + 1];
		magSq = (u32) (((u32) (real * real)) + ((u32)( imag * imag)));
		magRt[j] = (u16)ams_isqrt(magSq);

     //AMS_PORT_msg_2("ccb_sw_flicker_GetResult : magSq[%d]	\t%d\n", i/2, magSq);

		if (magRt[j] > max) {
			max = magRt[j];
			idx2 = i/2;
                     //AMS_PORT_log_2("ccb_sw_flicker_GetResult : andy_magSq[%d]=%d\n", idx2, magRt[j]);

		}
		
	}

        mean = get_mean(magRt, FFT_BIN_HALF);
        stdev = get_stdev(magRt, mean, FFT_BIN_HALF);
		
		
        if (max> (mean + (stdev * 6))) {
		// The frequency is the bin index times the frequency step
		mHz = idx2 * freqStep / 1000;

	// Round up to nearest multiple of 5
	/*mHz = ((mHz+ 5) / 10) * 10;	*/
	ctx->flickerCtx.lastValid.mHzbysw = mHz; 
		if(((maxG -minG) < 2) || (idx2 <=4 )){
               ctx->flickerCtx.lastValid.mHzbysw = 0;
        	}
        } else{
               ctx->flickerCtx.lastValid.mHzbysw = 0;
        }
	ctx->flickerCtx.flicker_finished = 0;
       //ams_setByte(ctx->portHndl, DEVREG_CONTROL, 0x02); //FIFO Buffer , FINT, FIFO_OV, FIFO_LVL all clear

      

	// If the frequency is not 100 or 120, report back ZERO.
	// If you want to report back the measured frequency,
	// do NOT do this test!
	/*if ((chip->flicker.freq != 100) && (chip->flicker.freq != 120))
	{
		chip->flicker.freq = 0;
	}*/
AMS_PORT_msg_2("ccb_sw_flicker_GetResult:idx2 %d , sw_freq= %u \n",idx2 ,mHz  );
	
#if 0
         ctx->shadowIntenabReg = 0;
         ctx->shadowEnableReg = 0;	 
         ams_setByte(ctx->portHndl, DEVREG_INTENAB, ctx->shadowIntenabReg);		 
         mdelay(2);
         ams_setByte(ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg);
#endif
#if 1
	//si = astatus = (astatus & MASK_AGAIN);
	si = astatus = ((fd_cfg3 >>3)& MASK_AGAIN);
	pre_si = si;
			   

	if(sampling_time == 1000){
		if((wbSum <=6912)&& (si<12)){ /*0.15 %*/
			astatus = MIN(12,++si);
		}
		if(wbSum > 34560){ /*0.8*/
			if(si == 0){
				astatus = MAX(0, si);
			}else{
				astatus = MAX(0,--si);
			}
		}
	}

	if(sampling_time ==500){
		if((wbSum <=3475)&& (si<12)){ /*0.15 %*/
			astatus = MIN(12,++si);
		}
		if(wbSum > 18534){ /*0.8*/
			if(si == 0){
				astatus = MAX(0, si);
			}else{
				astatus = MAX(0,--si);
			}
		}
	}
	
    //ams_setField(ctx->portHndl, DEVREG_FD_CFG3, 0x08, 0x08);

    //ams_setByte(ctx->portHndl, DEVREG_CFG1, (((FD_GAIN_16)<<3) | (1<<0)));
    if(pre_si != si){
       AMS_PORT_msg_4("ccb_sw_flicker_GetResult: wbSum %d , changed gain astatus= %d  si %d , pre_si %d\n", wbSum , astatus , si ,pre_si);
    	//AMS_SET_FLICKER_INDEX_TO_FDGAIN(si);
    	ams_setField(ctx->portHndl, DEVREG_FD_CFG3,   (si<<3) ,       MASK_FD_GAIN);
    }
    //AMS_SET_ALS_INDEX_TO_GAIN(si);
#endif	
    ams_setByte(ctx->portHndl, DEVREG_CONTROL, 0x02); //FIFO Buffer , FINT, FIFO_OV, FIFO_LVL all clear
	
#endif	
return true;
}
#endif


