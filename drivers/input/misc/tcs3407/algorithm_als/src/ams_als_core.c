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

/* see Application Note:
 * DN40-Rev 1.0 – Lux and CCT Calculations using ams Color Sensors
 */
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS
#include "ams_port_platform.h"
#include "ams_als_core.h"

#define AMS_ROUND_SHFT_VAL                      4
#define AMS_ROUND_ADD_VAL                       (1 << (AMS_ROUND_SHFT_VAL - 1))
#define AMS_ALS_GAIN_FACTOR             1000
#define CPU_FRIENDLY_FACTOR_1024        1

#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_RGB
#if defined(CONFIG_AMS_ALS_CRGBW) /* use CRGB here */
static void als_calc_LUX_CCT (amsAlsContext_t * ctx,
                                uint16_t clearADC,
                                uint16_t redADC,
                                uint16_t greenADC,
                                uint16_t blueADC) {
    int64_t lux = 0;

    lux = ctx->calibration.C_coef * clearADC
            + ctx->calibration.R_coef * redADC
            + ctx->calibration.G_coef * greenADC
            + ctx->calibration.B_coef * blueADC;

    /* int64_t is a long long on most platforms but only a long in cygwin64.
     * Promote to long long for portability (on cyg64 they're same size but
     * compiler complains if you use %ll for a long).
     *
     * Regarding 64-bit division in the linux kernel on an ARM:
     * ARM 32 bit toolchains require a function implementation called __aeabi_uldivmod
     * to do the division.  This function is missing. 64-bit division below is done
     * with do_div() for linux kernel builds.
     */
    AMS_PORT_log_2("als_calc_LUX_CCT: tempLux=%lld, cpl=%u\n", (long long int)lux, ctx->cpl);

    lux <<= AMS_ROUND_SHFT_VAL;
    if (lux < (LONG_MAX / ctx->calibration.calibrationFactor)) {
#ifdef __KERNEL__
        lux = lux * ctx->calibration.calibrationFactor;
        do_div(lux, ctx->cpl);
#else
        lux = (lux * ctx->calibration.calibrationFactor) / ctx->cpl;
#endif
    } else {
#ifdef __KERNEL__
        do_div(lux, ctx->cpl);
        lux *= ctx->calibration.calibrationFactor;
#else
        lux = (lux / ctx->cpl ) * ctx->calibration.calibrationFactor;
#endif
    }
    lux += AMS_ROUND_ADD_VAL;
    lux >>= AMS_ROUND_SHFT_VAL;
    ctx->results.mLux = (uint32_t)lux;

    if (redADC == 0 ) redADC = 1;
    ctx->results.CCT = ((ctx->calibration.CT_coef * blueADC) / redADC) + ctx->calibration.CT_offset;
}
#else /* _CRGBI or _CRWBI use derived IR, and derived G if _CRWBI, but not C here */
static void als_calc_LUX_CCT (amsAlsContext_t * ctx,
                                uint16_t redADC,
                                uint16_t greenADC,
                                uint16_t blueADC) {
    int32_t rp1;
    int32_t gp1;
    int32_t bp1;
    int64_t lux = 0;

    rp1 = redADC   - ctx->results.IR;
    gp1 = greenADC - ctx->results.IR;
    bp1 = blueADC  - ctx->results.IR;

    if (redADC > ctx->results.IR)
    {
        lux += (int32_t)(ctx->calibration.R_coef * rp1);
    }
    if (greenADC > ctx->results.IR)
    {
        lux += (int32_t)(ctx->calibration.G_coef * gp1);
    }
    if (blueADC > ctx->results.IR)
    {
        lux += (int32_t)((int32_t)ctx->calibration.B_coef * bp1);
    }

    /* int64_t is a long long on most platforms but only a long in cygwin64.
     * Promote to long long for portability (on cyg64 they're same size but
     * compiler complains if you use %ll for a long).
     */
    AMS_PORT_log_2("als_calc_LUX_CCT: tempLux=%lld, cpl=%u\n", (long long int)lux, ctx->cpl);

    lux <<= AMS_ROUND_SHFT_VAL;
    if (lux < (LONG_MAX / ctx->calibration.calibrationFactor)) {
#ifdef __KERNEL__
        lux = lux * ctx->calibration.calibrationFactor;
        do_div(lux, ctx->cpl);
#else
        lux = (lux * ctx->calibration.calibrationFactor) / ctx->cpl;
#endif
    } else {
#ifdef __KERNEL__
        do_div(lux, ctx->cpl);
        lux *= ctx->calibration.calibrationFactor;
#else
        lux = (lux / ctx->cpl ) * ctx->calibration.calibrationFactor;
#endif
    }
    lux += AMS_ROUND_ADD_VAL;
    lux >>= AMS_ROUND_SHFT_VAL;
    ctx->results.mLux = (uint32_t)lux;

    if (rp1 == 0 ) rp1 = 1;
    ctx->results.CCT = ((ctx->calibration.CT_coef * bp1) / rp1) + ctx->calibration.CT_offset;
}
#endif /* CRGBW vs. CRGBI/CRWBI */
#endif

#define WBD_FACTOR  2266
//#define WBD_FACTOR  AMS_ALS_Cc

void als_update_statics(amsAlsContext_t * ctx) {
    uint64_t tempCpl;
    uint64_t tempTime_us = ctx->time_us;
    uint64_t tempGain = ctx->gain;

    /* test for the potential of overflowing */
    uint32_t maxOverFlow;
#ifdef __KERNEL__
    u64 tmpTerm1;
    u64 tmpTerm2;
#endif

#ifdef __KERNEL__
    u64 tmp = ULLONG_MAX;
    do_div(tmp, ctx->time_us);
    maxOverFlow = (uint32_t)tmp;
#else
    maxOverFlow = (uint64_t)ULLONG_MAX / ctx->time_us;
#endif

    if (maxOverFlow < ctx->gain) {
        /* TODO: need to find use-case to test */
#ifdef __KERNEL__
        tmpTerm1 = tempTime_us;
        do_div(tmpTerm1, 2);
        tmpTerm2 = tempGain;
        do_div(tmpTerm2, 2);
        tempCpl = tmpTerm1 * tmpTerm2;
        do_div(tempCpl, (AMS_ALS_GAIN_FACTOR/4));
#else
        tempCpl = ((tempTime_us / 2) * (tempGain / 2)) / (AMS_ALS_GAIN_FACTOR/4) ;
#endif

    } else {
#ifdef __KERNEL__
        tempCpl = (tempTime_us * tempGain);
        do_div(tempCpl, AMS_ALS_GAIN_FACTOR);
#else
        tempCpl = (tempTime_us * tempGain) / AMS_ALS_GAIN_FACTOR;
#endif
    }
    if (tempCpl > (uint32_t)ULONG_MAX){
        /* if we get here, we have an problem */
        while(1);
    }

#ifdef __KERNEL__
    tmpTerm1 = tempCpl;
    do_div(tmpTerm1, ctx->calibration.D_factor);
    ctx->cpl = (uint32_t)tmpTerm1;


    tempCpl = (tempTime_us * tempGain);
    do_div(tempCpl , AMS_ALS_GAIN_FACTOR);
    do_div(tempCpl , WBD_FACTOR);
    //do_div(tempCpl , ctx->calibration.D_factor);

	
    ctx->uvir_cpl =  tempCpl ;
	
    if(ctx->uvir_cpl ==0)
       	ctx->uvir_cpl = 1;
#else
    ctx->cpl = tempCpl / ctx->calibration.D_factor;
#endif

    {
        uint32_t max_count = ((ctx->calibration.adcMaxCount) * ctx->time_us) / ctx->calibration.Time_base;
        if(max_count > (uint32_t)ULONG_MAX) {
            ctx->saturation = (uint16_t)USHRT_MAX;
        } else {
            ctx->saturation = (uint16_t)max_count; /* TODO: need to validate more all devices */
        }
    }
    ctx->previousGain = ctx->gain;
    {
        /* just changed settings, lux readings could be jumpy */
        ctx->notStableMeasurement = true;
    }
    AMS_PORT_log_1("als_update_statics: uvir_cpl=%u\n", ctx->uvir_cpl);
    AMS_PORT_log_4("als_update_statics: time=%d, gain=%d, dFactor=%d => cpl=%u\n", ctx->time_us, ctx->gain, ctx->calibration.D_factor, ctx->cpl);
}

#define WIDEBAND_CONST 4*AMS_ALS_FACTOR
#define CLEAR_CONST 1.5 * AMS_ALS_FACTOR

/* als_compute_data -- different versions depending on the input data available */
#if defined(CONFIG_AMS_OPTICAL_SENSOR_ALS_RGB) || defined(CONFIG_AMS_OPTICAL_SENSOR_ALS_WIDEBAND)
#if defined(CONFIG_AMS_ALS_CRGBW) /* use CRGB, w/o derived IR, for lux; separate WB channel */
void als_compute_data (amsAlsContext_t * ctx, amsAlsDataSet_t * inputData) {

uint32_t UVIR_clear;
uint32_t UVIR_wideband;

    if (inputData->datasetArray->clearADC < (uint16_t)USHRT_MAX){
        ctx->results.IR = 0;

        /* Compute irradiances in uW/cm^2 */
        ctx->results.rawClear = inputData->datasetArray->clearADC;
        ctx->results.rawRed = inputData->datasetArray->redADC;
        ctx->results.rawGreen = inputData->datasetArray->greenADC;
        ctx->results.rawBlue = inputData->datasetArray->blueADC;
      /*  ctx->results.irrRed = (inputData->datasetArray->redADC * (ctx->calibration.Rc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        ctx->results.irrClear = (inputData->datasetArray->clearADC * (ctx->calibration.Cc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        ctx->results.irrBlue = (inputData->datasetArray->blueADC * (ctx->calibration.Bc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        ctx->results.irrGreen = (inputData->datasetArray->greenADC * (ctx->calibration.Gc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        ctx->results.irrWideband = (inputData->datasetArray->widebandADC * (ctx->calibration.Wbc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;*/

        AMS_PORT_log_4("als_compute_data: RC  %u, CC  %u, BCl %u,  GC %u \n"
                        , AMS_ALS_Rc
                        , AMS_ALS_Cc
                        , AMS_ALS_Bc
                        , AMS_ALS_Gc
                       );                     
        AMS_PORT_log_1("als_compute_data: WC  %u\n"
                        , AMS_ALS_Wbc
                       );                     

/******TEST1 ************/
        ctx->results.irrRed = (inputData->datasetArray->redADC * (AMS_ALS_Rc/ CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        ctx->results.irrClear = (inputData->datasetArray->clearADC * (AMS_ALS_Cc/ CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        ctx->results.irrBlue = (inputData->datasetArray->blueADC * (AMS_ALS_Bc/ CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        ctx->results.irrGreen = (inputData->datasetArray->greenADC * (AMS_ALS_Gc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
	  

        if(ctx->cpl ==0)
			ctx->cpl = 1;

        UVIR_clear = (inputData->datasetArray->clearADC * ( AMS_ALS_Cc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        UVIR_wideband = (inputData->datasetArray->widebandADC * ( AMS_ALS_Wbc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;

	/*ctx->results.irrWideband = ((4 * UVIR_wideband)-((3* UVIR_clear)>>1));*/
        ctx->results.irrWideband = (4000 * UVIR_wideband)-( 1400 * UVIR_clear); //)/AMS_ALS_GAIN_FACTOR);

        if(ctx->results.irrWideband  < 0 ){
			ctx->results.irrWideband = 0;
	 }else{
	 	ctx->results.irrWideband = ctx->results.irrWideband/AMS_ALS_GAIN_FACTOR; 
	 }

        AMS_PORT_log_5("als_compute_data: TEST 1 UVIR_clear = %u, UVIR_wideband = %u, cpl =%u, AWB %d  gain %d\n"
                        , UVIR_clear
                        , UVIR_wideband
                        ,ctx->cpl
                        ,ctx->results.irrWideband
                        ,ctx->gain
                       );                     

/******TEST1 ************/

#if 0
/******TEST2 ************/

        ctx->results.irrRed = (inputData->datasetArray->redADC * ( CPU_FRIENDLY_FACTOR_1024)) / ctx->uvir_cpl;
        ctx->results.irrClear = (inputData->datasetArray->clearADC * ( CPU_FRIENDLY_FACTOR_1024)) / ctx->uvir_cpl;
        ctx->results.irrBlue = (inputData->datasetArray->blueADC * ( CPU_FRIENDLY_FACTOR_1024)) / ctx->uvir_cpl;
        ctx->results.irrGreen = (inputData->datasetArray->greenADC * ( CPU_FRIENDLY_FACTOR_1024)) / ctx->uvir_cpl;
//        ctx->results.irrWideband = (inputData->datasetArray->widebandADC * (ctx->calibration.Wbc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;


        if(ctx->uvir_cpl ==0)
			ctx->uvir_cpl = 1;

        UVIR_clear = (inputData->datasetArray->clearADC * ( AMS_ALS_Cc / CPU_FRIENDLY_FACTOR_1024)) / ctx->uvir_cpl;
        UVIR_wideband = (inputData->datasetArray->widebandADC * ( AMS_ALS_Wbc / CPU_FRIENDLY_FACTOR_1024)) / ctx->uvir_cpl;

	/*ctx->results.irrWideband = ((4 * UVIR_wideband)-((3* UVIR_clear)>>1));*/
        ctx->results.irrWideband = (((4000 * UVIR_wideband)-( 1400 * UVIR_clear))/AMS_ALS_GAIN_FACTOR);



        if(ctx->results.irrWideband  < 0 )
			ctx->results.irrWideband = 0;
	
		
        AMS_PORT_log_5("als_compute_data: TEST 2 UVIR_clear = %u, UVIR_wideband = %u, uvir_cpl =%u, AWB %d ,gain %d\n"
                        , UVIR_clear
                        , UVIR_wideband
                        ,ctx->uvir_cpl
                        ,ctx->results.irrWideband
                        ,ctx->gain
                       );                     

/******TEST2 ************/
#endif

        /*AMS_PORT_log_4("als_compute_data: irrRed = %u, irrGreen = %u, irrBlue = %u, irrWideband = %u\n"
                        , ctx->results.irrRed
                        , ctx->results.irrGreen
                        , ctx->results.irrBlue
                        , ctx->results.irrWideband
                       );*/




        als_calc_LUX_CCT(ctx
            , inputData->datasetArray->clearADC
            , inputData->datasetArray->redADC
            , inputData->datasetArray->greenADC
            , inputData->datasetArray->blueADC
            );
    } else {
        /* measurement is saturated */
        AMS_PORT_log("als_compute_data:  saturated\n");
    }
}
#elif defined(CONFIG_AMS_ALS_CRWBI) /* WB in G channel; use CRB & derived G & derived IR for lux */
void als_compute_data (amsAlsContext_t * ctx, amsAlsDataSet_t * inputData) {
    uint32_t tempIr;

    uint16_t widebandADC = inputData->datasetArray->greenADC;

    AMS_PORT_log_3("als_compute_data: cpl=%u, Wbc factor=%u, greenWbADC=%u\n", ctx->cpl, ctx->calibration.Wbc, widebandADC);
    AMS_PORT_log_3("als_compute_data: WB CRB coefs = (%u, %u, %u)\n", ctx->calibration.Wideband_C_factor, ctx->calibration.Wideband_R_factor, ctx->calibration.Wideband_B_factor);
    ctx->results.irrWideband = (widebandADC * (ctx->calibration.Wbc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;

    ctx->results.IR = 0;
    if (widebandADC < (uint16_t)SHRT_MAX)){
        /* Substitute derived green reading for actual */
        inputData->datasetArray->greenADC = ( (inputData->datasetArray->clearADC * ctx->calibration.Wideband_C_factor)
                                              - (inputData->datasetArray->redADC * ctx->calibration.Wideband_R_factor)
                                              - (inputData->datasetArray->blueADC * ctx->calibration.Wideband_B_factor)
                                            ) / AMS_WIDEBAND_SCALE_FACTOR;
        AMS_PORT_log_3("als_compute_data: IR = %u - %u = %u\n", widebandADC, inputData->datasetArray->clearADC, ctx->results.IR);
        AMS_PORT_log_3("als_compute_data: R = %u, B = %u, G' = %u\n", inputData->datasetArray->redADC, inputData->datasetArray->blueADC, inputData->datasetArray->greenADC);
    }

    if (inputData->datasetArray->clearADC < (uint16_t)USHRT_MAX){
        /* Calculate IR */
        tempIr = inputData->datasetArray->redADC +
                            inputData->datasetArray->greenADC +
                            inputData->datasetArray->blueADC;
        if (tempIr > inputData->datasetArray->clearADC) {
            ctx->results.IR = (tempIr - inputData->datasetArray->clearADC) / 2;
        } else {
            ctx->results.IR = 0;
        }

        /* Compute irradiances in uW/cm^2 */
        ctx->results.rawClear = inputData->datasetArray->clearADC;
        ctx->results.rawRed = inputData->datasetArray->redADC;
        ctx->results.rawGreen = inputData->datasetArray->greenADC;
        ctx->results.rawBlue = inputData->datasetArray->blueADC;
        ctx->results.irrRed = (inputData->datasetArray->redADC * (ctx->calibration.Rc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        ctx->results.irrClear = (inputData->datasetArray->clearADC * (ctx->calibration.Cc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        ctx->results.irrBlue = (inputData->datasetArray->blueADC * (ctx->calibration.Bc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        ctx->results.irrGreen = (inputData->datasetArray->greenADC * (ctx->calibration.Gc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        AMS_PORT_log_2("als_compute_data: irrClear = %u, irrWideband = %u\n"
                        , ctx->results.irrClear
                        , ctx->results.irrWideband
                        );
        AMS_PORT_log_3("als_compute_data: irrRed = %u, irrGreen = %u, irrBlue = %u\n"
                        , ctx->results.irrRed
                        , ctx->results.irrGreen
                        , ctx->results.irrBlue
                       );

        als_calc_LUX_CCT(ctx, inputData->datasetArray->redADC,
            inputData->datasetArray->greenADC,
            inputData->datasetArray->blueADC);
    } else {
        /* measurement is saturated */
        AMS_PORT_log("als_compute_data:  saturated\n");
    }
}
#else /* no WB; use CRGB & derived IR for lux */
void als_compute_data (amsAlsContext_t * ctx, amsAlsDataSet_t * inputData) {
    uint32_t tempIr;

    if (inputData->datasetArray->clearADC < (uint16_t)USHRT_MAX){
        /* Calculate IR */
        tempIr = inputData->datasetArray->redADC +
                            inputData->datasetArray->greenADC +
                            inputData->datasetArray->blueADC;
        if (tempIr > inputData->datasetArray->clearADC) {
            ctx->results.IR = (tempIr - inputData->datasetArray->clearADC) / 2;
        } else {
            ctx->results.IR = 0;
        }

        /* Compute irradiances in uW/cm^2 */
        ctx->results.rawClear = inputData->datasetArray->clearADC;
        ctx->results.rawRed = inputData->datasetArray->redADC;
        ctx->results.rawGreen = inputData->datasetArray->greenADC;
        ctx->results.rawBlue = inputData->datasetArray->blueADC;
        ctx->results.irrRed = (inputData->datasetArray->redADC * (ctx->calibration.Rc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        ctx->results.irrClear = (inputData->datasetArray->clearADC * (ctx->calibration.Cc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        ctx->results.irrBlue = (inputData->datasetArray->blueADC * (ctx->calibration.Bc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        ctx->results.irrGreen = (inputData->datasetArray->greenADC * (ctx->calibration.Gc / CPU_FRIENDLY_FACTOR_1024)) / ctx->cpl;
        AMS_PORT_log_1("als_compute_data: irrClear = %u\n"
                        , ctx->results.irrClear
                        );
        AMS_PORT_log_3("als_compute_data: irrRed = %u, irrGreen = %u, irrBlue = %u\n"
                        , ctx->results.irrRed
                        , ctx->results.irrGreen
                        , ctx->results.irrBlue
                       );

        als_calc_LUX_CCT(ctx, inputData->datasetArray->redADC,
            inputData->datasetArray->greenADC,
            inputData->datasetArray->blueADC);
    } else {
        /* measurement is saturated */
        AMS_PORT_log("als_compute_data:  saturated\n");
    }
}
#endif
#endif /* _RGB || _WIDEBAND */

#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CLEAR
void als_compute_data_clear (amsAlsContext_t * ctx, amsAlsDataSet_t * inputData) {
    int64_t lux1 = 0;
    int64_t lux2 = 0;

    if (inputData->datasetArray->ch0ADC < (uint16_t)USHRT_MAX){

        lux1 = ((inputData->datasetArray->ch0ADC * ctx->calibration.calibrationFactor) - (ctx->calibration.L1_factor * inputData->datasetArray->ch1ADC));
        lux2 = ((ctx->calibration.L2_factor * inputData->datasetArray->ch0ADC) - (ctx->calibration.L3_factor * inputData->datasetArray->ch1ADC));

        if (lux1 > lux2) {
            ctx->results.mLux = (lux1 * ctx->calibration.calibrationFactor) / ctx->cpl;
        } else {
            ctx->results.mLux = (lux2 * ctx->calibration.calibrationFactor) / ctx->cpl;
        }

        if (ctx->results.mLux > (uint32_t)0xFFFFFFFF) {
            ctx->results.mLux = ctx->previousLux;
        }
    }
}
#endif

void als_ave_LUX (amsAlsContext_t * ctx) {

    /* if average queue isn't full (at startup), fill it */
    if(ctx->ave_lux[AMS_LUX_AVERAGE_COUNT - 1] == 0) {
        ctx->results.mLux_ave = 0;
        ctx->ave_lux_index = 0;
        do {
            ctx->ave_lux[ctx->ave_lux_index++] = ctx->results.mLux;
            ctx->results.mLux_ave += ctx->results.mLux;
        } while (ctx->ave_lux_index < AMS_LUX_AVERAGE_COUNT);
        ctx->ave_lux_index = 1;
    } else {
        /* replace the oldest LUX value with the new LUX value */
        ctx->results.mLux_ave -= ctx->ave_lux[ctx->ave_lux_index];
        ctx->ave_lux[ctx->ave_lux_index] = ctx->results.mLux;
        ctx->results.mLux_ave += ctx->ave_lux[ctx->ave_lux_index];
        ctx->ave_lux_index ++;
    }

    if (ctx->ave_lux_index == AMS_LUX_AVERAGE_COUNT) {
        ctx->ave_lux_index = 0;
    }
}

amsAlsAdaptive_t als_adaptive(amsAlsContext_t * ctx, amsAlsDataSet_t * inputData){
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_RGB
    if (inputData->status & ALS_STATUS_OVFL) {
        ctx->results.adaptive = ADAPTIVE_ALS_TIME_DEC_REQUEST;
    } else {
        if (inputData->datasetArray->clearADC == (uint16_t)USHRT_MAX){
            if (ctx->gain != 0){
                return ADAPTIVE_ALS_GAIN_DEC_REQUEST;
            } else {
                return ADAPTIVE_ALS_TIME_DEC_REQUEST;
            }
        } else {
            if (ctx->gain != 0) {
                if (inputData->datasetArray->clearADC >= ctx->saturation){
                    return ADAPTIVE_ALS_GAIN_DEC_REQUEST;
                }
            } else {
                if (inputData->datasetArray->clearADC >= ctx->saturation){
                    return ADAPTIVE_ALS_TIME_DEC_REQUEST;
                }
            }
            if (inputData->datasetArray->clearADC <= 0xff){
                return ADAPTIVE_ALS_GAIN_INC_REQUEST;
            }
        }
    }
#else
    ctx++; inputData++; /* avoid compiler warning */
#endif
    return ADAPTIVE_ALS_NO_REQUEST;
}

#endif
