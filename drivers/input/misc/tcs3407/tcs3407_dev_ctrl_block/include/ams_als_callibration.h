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
 * @@AMS_REVISION_Id: 562ee7f424a003b8825afa3aa921e8cb5d0e71ab
 */

#ifndef AMS_ALS_CALLIBRATION_H
#define	AMS_ALS_CALLIBRATION_H

#define CPU_FRIENDLY_FACTOR_1024        1
#define AMS_ALS_GAIN_FACTOR             1000

#define AMS_ALS_Cc                      (111 * CPU_FRIENDLY_FACTOR_1024)
//#define AMS_ALS_Cc                      (118 * CPU_FRIENDLY_FACTOR_1024)

#define AMS_ALS_Rc                      (112 * CPU_FRIENDLY_FACTOR_1024)
#define AMS_ALS_Gc                      (172 * CPU_FRIENDLY_FACTOR_1024)
#define AMS_ALS_Bc                      (180 * CPU_FRIENDLY_FACTOR_1024)
#define AMS_ALS_R_COEF                  (195)
#define AMS_ALS_C_COEF                  (195)
#define AMS_ALS_G_COEF                  (1000)
#define AMS_ALS_B_COEF                  (-293)
#define AMS_ALS_D_FACTOR                (436)
#define AMS_ALS_CT_COEF                 (4417)
#define AMS_ALS_CT_OFFSET               (1053)


#define AMS_WIDEBAND_SCALE_FACTOR       1000
#define AMS_ALS_Wbc                     (111 * CPU_FRIENDLY_FACTOR_1024) /* actual value is TBD */
#define AMS_ALS_WB_C_FACTOR             (0.79 * AMS_WIDEBAND_SCALE_FACTOR)
#define AMS_ALS_WB_R_FACTOR             (0.54 * AMS_WIDEBAND_SCALE_FACTOR)
#define AMS_ALS_WB_B_FACTOR             (0.39 * AMS_WIDEBAND_SCALE_FACTOR)

#define AMS_ALS_TIMEBASE                (2780) /* in uSec, see data sheet */
#define AMS_ALS_ADC_MAX_COUNT           (1024) /* see data sheet */
#define AMS_ALS_THRESHOLD_LOW           (5) /* in % */
#define AMS_ALS_THRESHOLD_HIGH          (5) /* in % */

#define AMS_ALS_ATIME                   (50000)
#define AMS_ALS_STEP_TIME                   (999)/*2.78msec*/




#ifdef NOT_IMPLEMENTED
#define AMS_ALS_ADAPTIVE_PREF_ATIME_BEFORE_AGAIN   1
#endif

#endif	/* AMS_ALS_CALLIBRATION_H */
