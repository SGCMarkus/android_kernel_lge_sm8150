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

#ifndef __AMS_CCB_CORE_CONROL_BLOCK_ALS_H__
#define __AMS_CCB_CORE_CONROL_BLOCK_ALS_H__

#ifdef __TESTBENCH__
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <memory.h>
#elif defined VXMICRO_ARCH_Intel
#include <Typedef.h>
#include <math_lib.h>
#include "sysLog.h"
#endif
#ifdef QDSP6
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#endif
#ifdef QT_CORE_LIB
#include <math.h>
#endif
#include "../../algorithm_als/include/ams_als_API.h"

#define HIGH    0xFF
#define LOW     0x00

#if !defined(CONFIG_ALS_CAL_TARGET)
#define CONFIG_ALS_CAL_TARGET          300 /* lux */
#endif
#if !defined(CONFIG_ALS_CAL_TARGET_TOLERANCE)
#define CONFIG_ALS_CAL_TARGET_TOLERANCE  15 /* lux */
#endif

typedef struct {
    uint32_t calibrationFactor;
    int32_t luxTarget;
    int8_t luxTargetError;
    uint32_t Time_base; /* in uSec */
    uint32_t adcMaxCount;
    uint8_t thresholdLow;
    uint8_t thresholdHigh;
#if defined CONFIG_AMS_OPTICAL_SENSOR_ALS_CLEAR
    int32_t D_factor;
    int32_t L1_factor;
    int32_t L2_factor;
    int32_t L3_factor;
#endif
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_RGB
    int32_t Cc;
    int32_t Rc;
    int32_t Gc;
    int32_t Bc;
#ifdef CONFIG_AMS_ALS_CRGBW
    int32_t Wbc;
    int32_t C_coef;
#endif
    int32_t R_coef;
    int32_t G_coef;
    int32_t B_coef;
    int32_t D_factor;
    int32_t CT_coef;
    int32_t CT_offset;
#endif
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_WIDEBAND
    int32_t Wbc;
    int32_t Wideband_C_factor;
    int32_t Wideband_R_factor;
    int32_t Wideband_B_factor;
#endif
}ams_ccb_als_calibration_t;

typedef struct {
    uint32_t uSecTime;
    uint32_t gain;
    uint8_t  threshold;
}ams_ccb_als_config_t;

typedef struct {
    bool calibrate;
    bool autoGain;
    uint8_t hysteresis;
    uint16_t  sampleRate;
    ams_ccb_als_config_t configData;
    ams_ccb_als_calibration_t calibrationData;
}ams_ccb_als_init_t;

typedef enum {
    AMS_CCB_ALS_INIT,
    AMS_CCB_ALS_RGB,
    AMS_CCB_ALS_AUTOGAIN,
    AMS_CCB_ALS_CALIBRATION_INIT,
    AMS_CCB_ALS_CALIBRATION_COLLECT_DATA,
    AMS_CCB_ALS_CALIBRATION_CHECK,
    AMS_CCB_ALS_LAST_STATE
}ams_ccb_als_state_t;

typedef struct {
    char * algName;
    uint16_t contextMemSize;
    uint16_t scratchMemSize;
    ams_ccb_als_calibration_t defaultCalibrationData;
}ams_ccb_als_info_t;

typedef struct {
    ams_ccb_als_state_t state;
    amsAlsContext_t ctxAlgAls;
    ams_ccb_als_init_t initData;
    uint16_t bufferCounter;
    uint16_t shadowAiltReg;
    uint16_t shadowAihtReg;
}ams_ccb_als_ctx_t;

typedef struct {
    uint8_t  statusReg;
}ams_ccb_als_dataSet_t;

typedef struct {
    uint32_t mLux;
    uint32_t saturation;
    uint32_t clear;
    uint32_t red;
    uint32_t green;
    uint32_t blue;
    uint32_t ir;
    uint32_t wideband;
    uint32_t rawClear;
    uint32_t rawRed;
    uint32_t rawGreen;
    uint32_t rawBlue;	
}ams_ccb_als_result_t;

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
        AMS_PORT_msg_4("ccb_alsHandle: C, R,G,B = %u, %u,%u,%u" \
            , dataset.AdcClear \
            , dataset.AdcRed \
            , dataset.AdcGreen \
            , dataset.AdcBlue \
            ); \
        AMS_PORT_msg_1(" WB = %u\n", dataset.AdcWb); \

#elif defined(CONFIG_AMS_ALS_CRWBI)
#define AMS_PORT_LOG_CRGB_W(dataset) \
        AMS_PORT_msg_4("ccb_alsHandle: C, R,G/WB,B = %u, %u,%u,%u\n" \
            , dataset.AdcClear \
            , dataset.AdcRed \
            , dataset.AdcGreenWb \
            , dataset.AdcBlue \
            )
#else
#define AMS_PORT_LOG_CRGB_W(dataset) \
        AMS_PORT_msg_4("ccb_alsHandle: C,R,G,B = %u, %u,%u,%u\n" \
            , dataset.AdcClear \
            , dataset.AdcRed \
            , dataset.AdcGreen \
            , dataset.AdcBlue \
            )
#endif


extern void ccb_alsInit(void * dcbCtx, ams_ccb_als_init_t * initData);
extern void ccb_alsInit_FIFO(void * dcbCtx, ams_ccb_als_init_t * initData);

extern void ccb_alsInfo(ams_ccb_als_info_t* infoData);
extern void ccb_alsGetConfig(void * dcbCtx, ams_ccb_als_config_t * configData);
extern void ccb_alsSetConfig(void * dcbCtx, ams_ccb_als_config_t * configData);
extern bool ccb_alsHandle(void * dcbCtx, ams_ccb_als_dataSet_t * data);
extern bool ccb_FlickerFIFOEvent(void * dcbCtx, ams_ccb_als_dataSet_t * alsData);
extern bool ccb_FlickerFIFO4096Event(void * dcbCtx, ams_ccb_als_dataSet_t * alsData);
extern void ccb_alsGetResult(void * dcbCtx, ams_ccb_als_result_t * result);
extern bool ccb_sw_flicker_GetResult(void * dcbCtx);
extern bool ccb_sw_bin4096_flicker_GetResult(void * dcbCtx);

#endif
