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
 * Device Algorithm ALS
 */

/*
 * @@AMS_REVISION_Id: 94092f58014f4160671b0fcf30e2d665c6fbc02a
 */

#ifndef __AMS_DEVICE_ALG_ALS_API_H__
#define __AMS_DEVICE_ALG_ALS_API_H__

#ifdef __TESTBENCH__
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <memory.h>
#endif
#ifdef VXMICRO_ARCH_Intel
#include <Typedef.h>
#include <math_lib.h>
#include "sysLog.h"
#endif
#ifdef __KERNEL__
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/memory.h>
#endif
#ifdef QDSP6
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#endif
#ifdef __KERNEL__
#include <linux/types.h>
#endif
#ifdef QT_CORE_LIB
#include "stdbool.h"
#include "stdint.h"
#include <memory.h>
#endif
#if defined(__GNUC__) && !defined(__KERNEL__)
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#endif

#define AMS_LUX_AVERAGE_COUNT    8

typedef enum _amsAlsAdaptive {
    ADAPTIVE_ALS_NO_REQUEST,
    ADAPTIVE_ALS_TIME_INC_REQUEST,
    ADAPTIVE_ALS_TIME_DEC_REQUEST,
    ADAPTIVE_ALS_GAIN_INC_REQUEST,
    ADAPTIVE_ALS_GAIN_DEC_REQUEST
}amsAlsAdaptive_t;

typedef enum _amsAlsStatus {
    ALS_STATUS_IRQ  = (1 << 0),
    ALS_STATUS_RDY  = (1 << 1),
    ALS_STATUS_OVFL = (1 << 2)
}amsAlsStatus_t;

typedef struct _alsData{
    uint16_t clearADC;
    uint16_t redADC;
    uint16_t greenADC;
    uint16_t blueADC;
    uint16_t widebandADC;
    uint16_t flickerADC;
} alsData_t;

#if defined CONFIG_AMS_OPTICAL_SENSOR_ALS_CLEAR
typedef struct _alsData{
    uint16_t ch0ADC;
    uint16_t ch1ADC;
} alsData_t;
#endif

typedef struct _amsAlsCalibration {
    uint32_t Time_base; /* in uSec */
    uint32_t adcMaxCount;
    uint16_t calibrationFactor; /* default 1000 */
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
#ifdef CONFIG_AMS_ALS_CRWBI
    int32_t Wbc;
    int32_t Wideband_C_factor;
    int32_t Wideband_R_factor;
    int32_t Wideband_B_factor;
#endif
} amsAlsCalibration_t;

typedef struct _amsAlsInitData {
    bool adaptive;
    bool irRejection;
    uint32_t time_us;
    uint32_t gain;
    amsAlsCalibration_t calibration;
} amsAlsInitData_t;

typedef struct _amsALSConf {
    uint32_t time_us;
    uint32_t gain;
    uint8_t thresholdLow;
    uint8_t thresholdHigh;
} amsAlsConf_t;

typedef struct _amsAlsDataSet {
#if defined(CONFIG_AMS_OPTICAL_SENSOR_ALS_RGB) || defined(CONFIG_AMS_OPTICAL_SENSOR_ALS_CLEAR)
    alsData_t *datasetArray;
#endif
    uint64_t timeStamp;
    uint8_t status;
} amsAlsDataSet_t;

typedef struct _amsAlsResult {
    uint32_t    irrClear;
    uint32_t    irrRed;
    uint32_t    irrGreen;
    uint32_t    irrBlue;
    uint32_t    IR;
    int32_t    irrWideband;
    uint32_t    mLux;
    uint32_t    mLux_ave;
    uint32_t    saturation;
    amsAlsAdaptive_t adaptive;
    uint32_t    rawClear;
    uint32_t    rawRed;
    uint32_t    rawGreen;
    uint32_t    rawBlue;
}amsAlsResult_t;

typedef struct _amsAlsContext {
    uint64_t lastTimeStamp;
    uint32_t ave_lux[AMS_LUX_AVERAGE_COUNT];
    uint32_t ave_lux_index;
    uint32_t cpl;
    uint32_t uvir_cpl;
	
    uint32_t time_us;
    amsAlsCalibration_t calibration;
    amsAlsResult_t results;
    bool adaptive;
    uint16_t saturation;
    uint32_t gain;
    uint32_t previousGain;
    uint32_t previousLux;
    bool notStableMeasurement;
} amsAlsContext_t;

typedef struct _amsAlsAlgInfo {
    char * algName;
    uint16_t contextMemSize;
    uint16_t scratchMemSize;
    amsAlsCalibration_t calibrationData;
    int (*initAlg) (amsAlsContext_t * ctx, amsAlsInitData_t * initData);
    int (*processData) (amsAlsContext_t * ctx, amsAlsDataSet_t * inputData);
    int (*getResult) (amsAlsContext_t * ctx, amsAlsResult_t * outData);
    int (*setConfig) (amsAlsContext_t * ctx, amsAlsConf_t * inputData);
    int (*getConfig) (amsAlsContext_t * ctx, amsAlsConf_t * outputData);
} amsAlsAlgoInfo_t;

extern int amsAlg_als_getAlgInfo (amsAlsAlgoInfo_t * info);
extern int amsAlg_als_initAlg (amsAlsContext_t * ctx, amsAlsInitData_t * initData);
extern int amsAlg_als_processData(amsAlsContext_t * ctx, amsAlsDataSet_t * inputData);
extern int amsAlg_als_getResult(amsAlsContext_t * ctx, amsAlsResult_t * outData);
extern int amsAlg_als_setConfig(amsAlsContext_t * ctx, amsAlsConf_t * inputData);
extern int amsAlg_als_getConfig(amsAlsContext_t * ctx, amsAlsConf_t * outputData);

#endif
