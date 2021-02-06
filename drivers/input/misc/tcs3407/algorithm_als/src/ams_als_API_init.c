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

#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS
#include "ams_als_core.h"
#include "ams_port_platform.h"

extern void als_calc_cpl(amsAlsContext_t * ctx);

/*
 * initAlg: is used to initialize the algorithm.
 */
int amsAlg_als_initAlg (amsAlsContext_t * ctx, amsAlsInitData_t * initData){
    int ret = 0;

    memset(ctx, 0, sizeof(amsAlsContext_t));

    if (initData != NULL) {
        ctx->calibration.Time_base = initData->calibration.Time_base;
        ctx->calibration.thresholdLow = initData->calibration.thresholdLow;
        ctx->calibration.thresholdHigh = initData->calibration.thresholdHigh;
        ctx->calibration.calibrationFactor = initData->calibration.calibrationFactor;
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_RGB
        ctx->calibration.Wbc = initData->calibration.Wbc;
        ctx->calibration.C_coef = initData->calibration.C_coef;
        ctx->calibration.R_coef = initData->calibration.R_coef;
        ctx->calibration.G_coef = initData->calibration.G_coef;
        ctx->calibration.B_coef = initData->calibration.B_coef;
        ctx->calibration.Rc = initData->calibration.Rc;
        ctx->calibration.Gc = initData->calibration.Gc;
        ctx->calibration.Bc = initData->calibration.Bc;
        ctx->calibration.Cc = initData->calibration.Cc;
        ctx->calibration.D_factor = initData->calibration.D_factor;
        ctx->calibration.CT_coef = initData->calibration.CT_coef;
        ctx->calibration.CT_offset = initData->calibration.CT_offset;

#endif
    }

    if (initData != NULL) {
        ctx->gain = initData->gain;
        ctx->time_us = initData->time_us;
        ctx->adaptive = initData->adaptive;
    } else {
        AMS_PORT_log("error: initData == NULL\n");
    }

    als_update_statics(ctx);
    return ret;
}
#endif
