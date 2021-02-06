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

static void als_getDefaultCalibrationData(ams_ccb_als_calibration_t * data)
{
    if (data != NULL) {
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_RGB
        data->Wbc = AMS_ALS_Wbc;
        data->C_coef = AMS_ALS_C_COEF;
        data->R_coef = AMS_ALS_R_COEF;
        data->G_coef = AMS_ALS_G_COEF;
        data->B_coef = AMS_ALS_B_COEF;
        data->Rc = AMS_ALS_Rc;
        data->Gc = AMS_ALS_Gc;
        data->Bc = AMS_ALS_Bc;
        data->Cc = AMS_ALS_Cc;
        data->D_factor = AMS_ALS_D_FACTOR;
        data->CT_coef = AMS_ALS_CT_COEF;
        data->CT_offset = AMS_ALS_CT_OFFSET;
#endif
        data->D_factor  = AMS_ALS_D_FACTOR;
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CLEAR
        data->L1_factor = AMS_ALS_L1_FACTOR;
        data->L2_factor = AMS_ALS_L2_FACTOR;
        data->L3_factor = AMS_ALS_L3_FACTOR;
#endif
        data->Time_base = AMS_ALS_TIMEBASE;
        data->thresholdLow = AMS_ALS_THRESHOLD_LOW;
        data->thresholdHigh = AMS_ALS_THRESHOLD_HIGH;
        data->calibrationFactor = 1000;
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_WIDEBAND
        data->Wbc = AMS_ALS_Wbc;
        data->Wideband_C_factor = AMS_ALS_WB_C_FACTOR;
        data->Wideband_R_factor = AMS_ALS_WB_R_FACTOR;
        data->Wideband_B_factor = AMS_ALS_WB_B_FACTOR;
#endif
    }
}

void ccb_alsInfo(ams_ccb_als_info_t* infoData){
    if (infoData != NULL) {
        infoData->algName = "ALS";
        infoData->contextMemSize = sizeof(ams_ccb_als_ctx_t);
        infoData->scratchMemSize = 0;
        infoData->defaultCalibrationData.calibrationFactor = 1000;
        infoData->defaultCalibrationData.luxTarget = CONFIG_ALS_CAL_TARGET;
        infoData->defaultCalibrationData.luxTargetError = CONFIG_ALS_CAL_TARGET_TOLERANCE;
        als_getDefaultCalibrationData(&infoData->defaultCalibrationData);
    }
}
