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
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/delay.h>
#else
#include <stdbool.h>
#endif

#include "ams_port_platform.h"
#if defined CONFIG_AMS_OPTICAL_SENSOR_ALS
#include "../../ccb_als/include/core_control_block_als.h"
#endif
#include "ams_device_control_block.h"



#define HIGH    0xFF
#define LOW     0x00

typedef struct{
    uint8_t                 deviceId;
    uint8_t                 deviceIdMask;
    uint8_t                 deviceRef;
    uint8_t                 deviceRefMask;
    ams_deviceIdentifier_e  device;
}ams_deviceIdentifier_t;

static ams_deviceIdentifier_t deviceIdentifier[]={
    {AMS_DEVICE_ID, AMS_DEVICE_ID_MASK, AMS_REV_ID, AMS_REV_ID_MASK, AMS_TCS3407},
    {AMS_DEVICE_ID, AMS_DEVICE_ID_MASK, AMS_REV_ID_UNTRIM, AMS_REV_ID_MASK, AMS_TCS3407_UNTRIM},
    {AMS_DEVICE_ID2, AMS_DEVICE_ID2_MASK, AMS_REV_ID2, AMS_REV_ID2_MASK, AMS_TCS3408},
    {AMS_DEVICE_ID2, AMS_DEVICE_ID2_MASK, AMS_REV_ID2_UNTRIM, AMS_REV_ID2_MASK, AMS_TCS3408_UNTRIM},
    {0, 0, 0, 0, AMS_LAST_DEVICE}
};

static void _3407_resetAllRegisters(AMS_PORT_portHndl * portHndl);
static void _3407_resetAllRegisters(AMS_PORT_portHndl * portHndl){

	/*ams_deviceRegister_t i;

    for (i = DEVREG_ENABLE; i <= DEVREG_CFG1; i++) {
        ams_setByte(portHndl, i, deviceRegisterDefinition[i].resetValue);
    }
    for (i = DEVREG_STATUS; i < DEVREG_REG_MAX; i++) {
        ams_setByte(portHndl, i, deviceRegisterDefinition[i].resetValue);
    }*/
    // To prevent SIDE EFFECT , below register should be written  
    ams_setByte(portHndl, DEVREG_CFG6, deviceRegisterDefinition[DEVREG_CFG6].resetValue);
    ams_setByte(portHndl, DEVREG_AGC_GAIN_MAX, deviceRegisterDefinition[DEVREG_AGC_GAIN_MAX].resetValue);
    ams_setByte(portHndl, DEVREG_FD_CFG3, deviceRegisterDefinition[DEVREG_FD_CFG3].resetValue);
    
}


#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
bool _3407_alsInit(ams_deviceCtx_t * ctx, ams_calibrationData_t * calibrationData);
bool _3407_alsInit(ams_deviceCtx_t * ctx, ams_calibrationData_t * calibrationData){

    if (calibrationData == NULL) {
        ams_ccb_als_info_t infoData;
        AMS_PORT_log("_3407_alsInit: calibrationData is null\n");
        ccb_alsInfo(&infoData);
        ctx->ccbAlsCtx.initData.calibrationData.luxTarget = infoData.defaultCalibrationData.luxTarget;
        ctx->ccbAlsCtx.initData.calibrationData.luxTargetError = infoData.defaultCalibrationData.luxTargetError;
        ctx->ccbAlsCtx.initData.calibrationData.calibrationFactor = infoData.defaultCalibrationData.calibrationFactor;
        ctx->ccbAlsCtx.initData.calibrationData.Time_base = infoData.defaultCalibrationData.Time_base;
        ctx->ccbAlsCtx.initData.calibrationData.thresholdLow = infoData.defaultCalibrationData.thresholdLow;
        ctx->ccbAlsCtx.initData.calibrationData.thresholdHigh = infoData.defaultCalibrationData.thresholdHigh;
        ctx->ccbAlsCtx.initData.calibrationData.calibrationFactor = infoData.defaultCalibrationData.calibrationFactor;
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_RGB
        ctx->ccbAlsCtx.initData.calibrationData.Cc = infoData.defaultCalibrationData.Cc;
        ctx->ccbAlsCtx.initData.calibrationData.Rc = infoData.defaultCalibrationData.Rc;
        ctx->ccbAlsCtx.initData.calibrationData.Gc = infoData.defaultCalibrationData.Gc;
        ctx->ccbAlsCtx.initData.calibrationData.Bc = infoData.defaultCalibrationData.Bc;
#ifdef CONFIG_AMS_ALS_CRGBW
        ctx->ccbAlsCtx.initData.calibrationData.Wbc = infoData.defaultCalibrationData.Wbc;
        ctx->ccbAlsCtx.initData.calibrationData.C_coef = infoData.defaultCalibrationData.C_coef;
#endif
        ctx->ccbAlsCtx.initData.calibrationData.R_coef = infoData.defaultCalibrationData.R_coef;
        ctx->ccbAlsCtx.initData.calibrationData.G_coef = infoData.defaultCalibrationData.G_coef;
        ctx->ccbAlsCtx.initData.calibrationData.B_coef = infoData.defaultCalibrationData.B_coef;
        ctx->ccbAlsCtx.initData.calibrationData.D_factor = infoData.defaultCalibrationData.D_factor;
        ctx->ccbAlsCtx.initData.calibrationData.CT_coef = infoData.defaultCalibrationData.CT_coef;
        ctx->ccbAlsCtx.initData.calibrationData.CT_offset = infoData.defaultCalibrationData.CT_offset;
#endif
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_WIDEBAND
        ctx->ccbAlsCtx.initData.calibrationData.Wbc = infoData.defaultCalibrationData.Wbc;
        ctx->ccbAlsCtx.initData.calibrationData.Wideband_C_factor = infoData.defaultCalibrationData.Wideband_C_factor;
        ctx->ccbAlsCtx.initData.calibrationData.Wideband_R_factor = infoData.defaultCalibrationData.Wideband_R_factor;
        ctx->ccbAlsCtx.initData.calibrationData.Wideband_B_factor = infoData.defaultCalibrationData.Wideband_B_factor;
#endif
    } else {
        AMS_PORT_log("_3407_alsInit: calibrationData is non-null\n");
        ctx->ccbAlsCtx.initData.calibrationData.luxTarget = calibrationData->alsCalibrationLuxTarget;
        ctx->ccbAlsCtx.initData.calibrationData.luxTargetError = calibrationData->alsCalibrationLuxTargetError;
        ctx->ccbAlsCtx.initData.calibrationData.calibrationFactor = calibrationData->alsCalibrationFactor;
        ctx->ccbAlsCtx.initData.calibrationData.Time_base = calibrationData->timeBase_us;
        ctx->ccbAlsCtx.initData.calibrationData.thresholdLow = calibrationData->alsThresholdLow;
        ctx->ccbAlsCtx.initData.calibrationData.thresholdHigh = calibrationData->alsThresholdHigh;
        ctx->ccbAlsCtx.initData.calibrationData.calibrationFactor = calibrationData->alsCalibrationFactor;
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_RGB
        ctx->ccbAlsCtx.initData.calibrationData.Cc = calibrationData->Cc;
        ctx->ccbAlsCtx.initData.calibrationData.Rc = calibrationData->Rc;
        ctx->ccbAlsCtx.initData.calibrationData.Gc = calibrationData->Gc;
        ctx->ccbAlsCtx.initData.calibrationData.Bc = calibrationData->Bc;
#ifdef CONFIG_AMS_ALS_CRGBW
        ctx->ccbAlsCtx.initData.calibrationData.Wbc = calibrationData->Wbc;
        ctx->ccbAlsCtx.initData.calibrationData.C_coef = calibrationData->alsCoefC;
#endif
        ctx->ccbAlsCtx.initData.calibrationData.R_coef = calibrationData->alsCoefR;
        ctx->ccbAlsCtx.initData.calibrationData.G_coef = calibrationData->alsCoefG;
        ctx->ccbAlsCtx.initData.calibrationData.B_coef = calibrationData->alsCoefB;
        ctx->ccbAlsCtx.initData.calibrationData.D_factor = calibrationData->alsDfg;
        ctx->ccbAlsCtx.initData.calibrationData.CT_coef = calibrationData->alsCctCoef;
        ctx->ccbAlsCtx.initData.calibrationData.CT_offset = calibrationData->alsCctOffset;
#endif
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_WIDEBAND
        ctx->ccbAlsCtx.initData.calibrationData.Wbc = calibrationData->Wbc;
        ctx->ccbAlsCtx.initData.calibrationData.Wideband_C_factor = calibrationData->Wideband_C_factor;
        ctx->ccbAlsCtx.initData.calibrationData.Wideband_R_factor = calibrationData->Wideband_R_factor;
        ctx->ccbAlsCtx.initData.calibrationData.Wideband_B_factor = calibrationData->Wideband_B_factor;
#endif
    }
    ctx->ccbAlsCtx.initData.calibrate = false;
    ctx->ccbAlsCtx.initData.configData.gain = 64000;
    ctx->ccbAlsCtx.initData.configData.uSecTime = AMS_ALS_ATIME; /*ATime changed from 100msec to 60msec*/

    ctx->alwaysReadAls = false;
    ctx->alwaysReadProx = false;
    ctx->alwaysReadFlicker = false;
    ctx->ccbAlsCtx.initData.autoGain = true;
    ctx->ccbAlsCtx.initData.hysteresis = 0x02; /*Lower threshold for adata in AGC */
    if (ctx->ccbAlsCtx.initData.autoGain) {
        AMS_SET_ALS_AUTOGAIN(HIGH);
        AMS_SET_ALS_AGC_HYST(ctx->ccbAlsCtx.initData.hysteresis);
    }
    return false;
}
#endif

bool _3407_flickerInit(ams_deviceCtx_t * ctx);
bool _3407_flickerInit(ams_deviceCtx_t * ctx){
    ams_flicker_ctx_t *flickerCtx = (ams_flicker_ctx_t *)&ctx->flickerCtx;
    flickerCtx->lastValid.freq100Hz = NOT_VALID;
    flickerCtx->lastValid.freq120Hz = NOT_VALID;
    flickerCtx->lastValid.mHz = 0;
    //ctx->valid_flickerhz_count = 0;

//    AMS_DISABLE_FLICKER_AUTOGAIN(HIGH);/*Disable flicker autogain */
//    ams_setByte(ctx->portHndl, DEVREG_FD_CFG0, ((FD_SAMPLES_256) | (FD_COMPARE_4_32NDS)));
    ams_setByte(ctx->portHndl, DEVREG_FD_CFG0, ((FD_SAMPLES_128) | (FD_COMPARE_6_32NDS)));	


    ams_setByte(ctx->portHndl, DEVREG_FD_CFG3, (((AGAIN_128)<<3) | (1<<0)));
//    ams_setByte(ctx->portHndl, DEVREG_FD_CFG3, (((AGAIN_512)<<3) | (1<<0)));
    ams_setField(ctx->portHndl, DEVREG_CFG10, FD_PERS_ALWAYS, MASK_FD_PERS);

    return false;
}

static void ams_deviceSoftReset(ams_deviceCtx_t * ctx)
{
    AMS_PORT_log("ams_deviceSoftReset Start\n");

    // Before S/W reset, the PON has to be asserted 
    ams_setByte(ctx->portHndl, DEVREG_ENABLE, PON);
    ams_setField(ctx->portHndl, DEVREG_SOFT_RESET, HIGH, MASK_SOFT_RESET);

    // Need 1 msec delay
    // sns_busy_wait(sns_convert_ns_to_ticks(1 * 1000 * 1000));
    mdelay(1);

    // Recover the previous enable setting
    ams_setByte(ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg);
}

 void ams_smux_read(ams_deviceCtx_t *ctx )
{
	  
      ams_setByte(ctx->portHndl, DEVREG_ENABLE, 0x00); //sensor off
      ams_setByte(ctx->portHndl, DEVREG_ENABLE, 0x01); //only PON
      
       /* SMUX read command from ram*/
      AMS_READ_S_MUX();
      ams_setByte(ctx->portHndl, DEVREG_ENABLE, 0x11);//PON + SMUXEN excute 
      udelay(1000); //Now 0x80 needs to be read back until SMUXEN has been cleared , wait 1msec      
      ams_getBuf(ctx->portHndl, DEVREG_RAM_START, (uint8_t*)&ctx->smux_buffer[0], sizeof(ctx->smux_buffer));
	  
      		AMS_PORT_log_4("ams_smux_read =%x, %x, %x, %x",ctx->smux_buffer[0],ctx->smux_buffer[1],ctx->smux_buffer[2],ctx->smux_buffer[3]);
      		AMS_PORT_log_4("ams_smux_read =%x, %x, %x, %x",ctx->smux_buffer[4],ctx->smux_buffer[5],ctx->smux_buffer[7],ctx->smux_buffer[8]);
      		AMS_PORT_log_4("ams_smux_read =%x, %x, %x, %x",ctx->smux_buffer[8],ctx->smux_buffer[9],ctx->smux_buffer[10],ctx->smux_buffer[11]);
      		AMS_PORT_log_4("ams_smux_read =%x, %x, %x, %x",ctx->smux_buffer[12],ctx->smux_buffer[13],ctx->smux_buffer[14],ctx->smux_buffer[15]);
      		AMS_PORT_log_4("ams_smux_read =%x, %x, %x, %x",ctx->smux_buffer[16],ctx->smux_buffer[17],ctx->smux_buffer[18],ctx->smux_buffer[19]);
	  
	  
      ams_setByte(ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg); //only PON
	  
}    



 void ams_smux_set(ams_deviceCtx_t *ctx , ams_deviceIdentifier_e devID)
{
    
   if((devID == AMS_TCS3408) ||(devID == AMS_TCS3408_UNTRIM) ){
      ams_setByte(ctx->portHndl, DEVREG_ENABLE, 0x00); //sensor off
      ams_setByte(ctx->portHndl, DEVREG_ENABLE, 0x01); //only PON
      
	  /* SMUX read command from ram*/
	  AMS_READ_S_MUX();
	  ams_setByte(ctx->portHndl, DEVREG_ENABLE, 0x11);//PON + SMUXEN excute 
	  udelay(1000); //Now 0x80 needs to be read back until SMUXEN has been cleared , wait 1msec      
      ams_setByte(ctx->portHndl,DEVREG_SMUX13_PRX_TO_FLICKER,0x66); // 0x66 : ficker+flicker, 0x76: only one flikcer, 0x00 flicker off
      /* SMUX write command */
      AMS_WRITE_S_MUX();//SMUX Write from RAM to chain	  
      ams_setByte(ctx->portHndl, DEVREG_ENABLE, 0x11);//PON + SMUXEN
      udelay(1000); //Now 0x80 needs to be read back until SMUXEN has been cleared , wait 1msec 
      //ams_setByte(ctx->portHndl, DEVREG_ENABLE, 0x11); //PON + SMUXEN
    }else{
      ams_setByte(ctx->portHndl, DEVREG_ENABLE, 0x00); //sensor off
      ams_setByte(ctx->portHndl, DEVREG_ENABLE, 0x01); //only PON
      
	  /* SMUX read command from ram*/
	  AMS_READ_S_MUX();
	  ams_setByte(ctx->portHndl, DEVREG_ENABLE, 0x11);//PON + SMUXEN excute 
	  udelay(1000); //Now 0x80 needs to be read back until SMUXEN has been cleared , wait 1msec      
      ams_setByte(ctx->portHndl,DEVREG_SMUX13_PRX_TO_FLICKER,0x76); // 0x66 : ficker+flicker, 0x76: only one flikcer, 0x00 flicker off
      /* SMUX write command */
      AMS_WRITE_S_MUX();//SMUX Write from RAM to chain	  
      ams_setByte(ctx->portHndl, DEVREG_ENABLE, 0x11);//PON + SMUXEN
      udelay(1000); //Now 0x80 needs to be read back until SMUXEN has been cleared , wait 1msec 
      //ams_setByte(ctx->portHndl, DEVREG_ENABLE, 0x11); //PON + SMUXEN
    }
      ams_setByte(ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg);
	  
}    


bool ams_deviceInit(ams_deviceCtx_t * ctx, AMS_PORT_portHndl * portHndl, ams_calibrationData_t * calibrationData){
    int ret = 0;

    ctx->portHndl = portHndl;
    ctx->mode = MODE_OFF;
    ctx->systemCalibrationData = calibrationData;
    ctx->deviceId = ams_validateDevice(ctx->portHndl);
    ctx->shadowEnableReg = deviceRegisterDefinition[DEVREG_ENABLE].resetValue;
    ams_deviceSoftReset(ctx);
	
    _3407_resetAllRegisters(ctx->portHndl);
    AMS_PORT_get_timestamp_usec(&ctx->timeStamp);


/*
S-MUX Read/Write 
1  read configuration to ram Read smux configuration to RAM from smux chain  
2  write configuration from ram Write smux configuration from RAM to smux chain 
*/

	ams_smux_set(ctx , ctx->deviceId );


#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
    ret |= _3407_alsInit(ctx, calibrationData);
#endif
    ams_setByte(ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg);
    return ret;
}

ams_deviceIdentifier_e ams_validateDevice(AMS_PORT_portHndl * portHndl){
    uint8_t chipId;
    uint8_t revId;
    uint8_t i = 0;

    ams_getByte(portHndl, DEVREG_ID, &chipId);
    ams_getByte(portHndl, DEVREG_REVID, &revId);

    do{
        if (((chipId & deviceIdentifier[i].deviceIdMask) ==
            (deviceIdentifier[i].deviceId & deviceIdentifier[i].deviceIdMask)) &&
            ((revId & deviceIdentifier[i].deviceRefMask) ==
             (deviceIdentifier[i].deviceRef & deviceIdentifier[i].deviceRefMask))){
                return deviceIdentifier[i].device;
        }
        i++;
    }while (deviceIdentifier[i].device != AMS_LAST_DEVICE);

    AMS_PORT_log_2("ams_validateDevice: 0x%02x 0x%02x\n", chipId, revId);
    return AMS_UNKNOWN_DEVICE;
}

bool ams_getDeviceInfo(ams_deviceInfo_t * info){
    memset(info, 0, sizeof(ams_deviceInfo_t));

    info->defaultCalibrationData.timeBase_us = AMS_USEC_PER_TICK;
    info->numberOfSubSensors = 0;
    info->memorySize =  sizeof(ams_deviceCtx_t);
    info->deviceModel = "TCS3407";
    memcpy(info->defaultCalibrationData.deviceName, info->deviceModel, sizeof(info->defaultCalibrationData.deviceName));
    info->deviceName  = "ALS/PRX/FLKR";
    info->driverVersion = "Alpha";
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
    {
        /* TODO */
        ams_ccb_als_info_t infoData;
        ccb_alsInfo(&infoData);
        info->tableSubSensors[info->numberOfSubSensors]= AMS_AMBIENT_SENSOR;
        info->numberOfSubSensors++;

        info->alsSensor.driverName = infoData.algName;
        info->alsSensor.adcBits = 8;
        info->alsSensor.maxPolRate = 50;
        info->alsSensor.activeCurrent_uA = 100;
        info->alsSensor.standbyCurrent_uA = 5;
        info->alsSensor.rangeMax = 1;
        info->alsSensor.rangeMin = 0;

        info->defaultCalibrationData.alsCalibrationFactor = infoData.defaultCalibrationData.calibrationFactor;
        info->defaultCalibrationData.alsCalibrationLuxTarget = infoData.defaultCalibrationData.luxTarget;
        info->defaultCalibrationData.alsCalibrationLuxTargetError = infoData.defaultCalibrationData.luxTargetError;
        info->defaultCalibrationData.alsDfg = infoData.defaultCalibrationData.D_factor;
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_RGB
        info->defaultCalibrationData.Cc = infoData.defaultCalibrationData.Cc;
        info->defaultCalibrationData.Rc = infoData.defaultCalibrationData.Cc;
        info->defaultCalibrationData.Gc = infoData.defaultCalibrationData.Cc;
        info->defaultCalibrationData.Bc = infoData.defaultCalibrationData.Cc;
        info->defaultCalibrationData.Wbc = infoData.defaultCalibrationData.Wbc;
        info->defaultCalibrationData.alsCoefC = infoData.defaultCalibrationData.C_coef;
        info->defaultCalibrationData.alsCoefR = infoData.defaultCalibrationData.R_coef;
        info->defaultCalibrationData.alsCoefG = infoData.defaultCalibrationData.G_coef;
        info->defaultCalibrationData.alsCoefB = infoData.defaultCalibrationData.B_coef;
        info->defaultCalibrationData.alsCctCoef = infoData.defaultCalibrationData.CT_coef;
        info->defaultCalibrationData.alsCctOffset = infoData.defaultCalibrationData.CT_offset;
#endif
#if defined(CONFIG_AMS_ALS_CRWBI) || defined(CONFIG_AMS_ALS_CRGBW)
        info->tableSubSensors[info->numberOfSubSensors]= AMS_WIDEBAND_ALS_SENSOR;
        info->numberOfSubSensors++;
#endif
    }
#endif
    return false;
}
