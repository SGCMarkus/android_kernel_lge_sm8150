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

#ifndef __AMS_DEVICE_CONTROL_BLOCK_H__
#define __AMS_DEVICE_CONTROL_BLOCK_H__


#ifdef __KERNEL__
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/memory.h>
#endif
#ifdef __TESTBENCH__
#include <stdint.h>
#include <stdbool.h>
#endif


typedef struct {
    bool     nearBy;
    uint16_t proximity;
} ams_apiPrx_t;

#include "../../ccb_als/include/core_control_block_als.h"
#include "ams_als_callibration.h"

typedef enum {
    NOT_VALID,
    PRESENT,
    ABSENT
} ternary;

typedef struct {
    uint32_t wideband;
    uint32_t clear;
    uint32_t saturation;
    ternary  freq100Hz;
    ternary  freq120Hz;
    uint32_t mHz;
    uint16_t mHzbysw;    
    uint16_t flicker_raw_data;
    uint32_t mLux;
} ams_apiAlsFlicker_t;

typedef struct {
    uint32_t mLux;
    uint32_t saturation;
    uint32_t red;
    uint32_t green;
    uint32_t blue;
    uint32_t ir;
    uint32_t wideband;
    uint32_t rawClear;
    uint32_t rawRed;
    uint32_t rawGreen;
    uint32_t rawBlue;
} ams_apiAls_t;

#ifdef  __cplusplus
extern "C" {
#endif

#define AMS_USEC_PER_TICK               (2780)
#define ACTUAL_USEC(x)                  (((x + AMS_USEC_PER_TICK / 2) / AMS_USEC_PER_TICK) * AMS_USEC_PER_TICK)
#define AMS_ALS_USEC_TO_REG(x)          (256 - (x / AMS_USEC_PER_TICK))
#define AMS_DEFAULT_REPORTTIME_US       (1000000) /* Max 8 seconds */
#define AMS_PRX_PGLD_TO_REG(x)          ((x-4)/8)

#ifndef UINT_MAX_VALUE
#define UINT_MAX_VALUE      (-1)
#endif

#define AMS_CALIBRATION_DONE                (-1)
#define AMS_CALIBRATION_DONE_BUT_FAILED     (-2)

typedef enum _deviceIdentifier_e {
    AMS_UNKNOWN_DEVICE,
    AMS_TCS3407,
    AMS_TCS3407_UNTRIM,
    AMS_TCS3408,
    AMS_TCS3408_UNTRIM, 
    AMS_LAST_DEVICE
} ams_deviceIdentifier_e;

/*TCS3407 */
/*0x92 ID(0x18), 0x91 REVID(0x51)*/
#define AMS_DEVICE_ID       0x18
#define AMS_DEVICE_ID_MASK  0xFF
#define AMS_REV_ID          0x51
#define AMS_REV_ID_UNTRIM     0x01
#define AMS_REV_ID_MASK     0xFF

/*TMD4907 */
/*0x92 ID(0x18), 0x91 REVID(0x51)*/
#define AMS_DEVICE_ID2       0x18
#define AMS_DEVICE_ID2_MASK  0xFF
#define AMS_REV_ID2          0x53
#define AMS_REV_ID2_UNTRIM     0x48
#define AMS_REV_ID2_MASK 0xFF


#define AMS_DEVICE_ID3       0xEC
#define AMS_DEVICE_ID3_MASK  0xFC
#define AMS_REV_ID3          0x01
#define AMS_REV_ID3_MASK     0x07



#define AMS_PRX_PERS_TO_REG(x)         (x << 4)
#define AMS_PRX_REG_TO_PERS(x)         (x >> 4)
#define AMS_PRX_CURRENT_TO_REG(mA)     ((((mA) > 257) ? 127 : (((mA) - 4) >> 1)) << 0)
#define AMS_ALS_PERS_TO_REG(x)         (x << 0)
#define AMS_ALS_REG_TO_PERS(x)         (x >> 0)

typedef enum _deviceRegisters {
 DEVREG_RAM_START,
 DEVREG_SMUX13_PRX_TO_FLICKER,
 
 DEVREG_ENABLE,
 DEVREG_ATIME, 
 DEVREG_PTIME ,
 DEVREG_WTIME ,
 DEVREG_AILTL ,
 DEVREG_AILTH ,
 DEVREG_AIHTL,
 DEVREG_AIHTH ,
 DEVREG_AUXID ,
 DEVREG_REVID ,
 DEVREG_ID ,
 DEVREG_STATUS ,
 DEVREG_ASTATUS,
 DEVREG_ADATA0L,
 DEVREG_ADATA0H,
 DEVREG_ADATA1L,
 DEVREG_ADATA1H,
 DEVREG_ADATA2L,
 DEVREG_ADATA2H,
 DEVREG_ADATA3L,
 DEVREG_ADATA3H,
 DEVREG_ADATA4L,
 DEVREG_ADATA4H,
 DEVREG_ADATA5L,
 DEVREG_ADATA5H,
 DEVREG_STATUS2,
 DEVREG_STATUS3,
 DEVREG_STATUS5,
 DEVREG_STATUS4,
 DEVREG_CFG0 ,
 DEVREG_CFG1 ,
 DEVREG_CFG3 ,
 DEVREG_CFG4 ,
 DEVREG_CFG6 ,
 DEVREG_CFG8 ,
 DEVREG_CFG9 ,
 DEVREG_CFG10 ,
 DEVREG_CFG11 ,
 DEVREG_CFG12 ,
 DEVREG_PCFG1 ,
 DEVREG_PERS ,
 DEVREG_GPIO ,                             
 DEVREG_ASTEPL,
 DEVREG_ASTEPH,
 DEVREG_AGC_GAIN_MAX,
 DEVREG_AZ_CONFIG ,
 DEVREG_FD_CFG0,
 DEVREG_FD_CFG1,
 DEVREG_FD_CFG2,
 DEVREG_FD_CFG3,   	
 DEVREG_FD_STATUS ,
 DEVREG_INTENAB ,
 DEVREG_CONTROL ,
 DEVREG_FIFO_MAP ,
 DEVREG_FIFO_STATUS ,
 DEVREG_FDATAL ,
 DEVREG_FDATAH ,   
 DEVREG_FLKR_WA_RAMLOC_1,
 DEVREG_FLKR_WA_RAMLOC_2,
 DEVREG_SOFT_RESET,     /* 0xF3 */
    DEVREG_REG_MAX
}ams_deviceRegister_t;

typedef enum _3407_regOptions {

    PON             = 0x01, /* register 0x80 */
    AEN             = 0x02,
    PEN             = 0x04,
    WEN             = 0x08,
    SMUXEN          = 0x10,
    /* reserved: 0x20 */
    FDEN            = 0x40,
    IBEN            = 0x80,

    /* STATUS REG:  Interrupts.  All but FIFOINT are cleared by writing
     * back a '1' bit.  FIFOINT only clearable by emptying the FIFO.
     */
    SINT            = 0x01, /* register 0x93 */
    CINT            = 0x02,
    FIFOINT         = 0x04,
    AINT            = 0x08,
    PINT0           = 0x10,
    PINT1           = 0x20,
    PSAT            = 0x40,
    ASAT_FDSAT      = 0x80,
    ALS_INT_ALL     = (AINT + ASAT_FDSAT),
    PROX_INT_ALL_0  = (CINT + PINT0),
    PROX_INT_ALL_1  = (CINT + PINT1),
    PROX_INT_ALL_0_1= (CINT + PINT0 + PINT1),
    /*PROX_INT_ALL    = (CINT + PINT0),*/
    FLICKER_INT_ALL = (SINT + FIFOINT + ASAT_FDSAT),
    IRBEAM_INT      = SINT,

    FDSAT_DIGITAL   = 0x01, /* register 0xA3 */
    FDSAT_ANALOG    = 0x02,
    ASAT_ANALOG     = 0x08,
    ASAT_DIGITAL    = 0x10,
    AVALID          = 0x40,
    PVALID0         = 0x80,

    PSAT_AMB        = 0x01, /* register 0xA4 */
    PSAT_REFL       = 0x02,
    PSAT_ADC        = 0x04,

    PINT0_PILT      = 0x01, /* register 0xA5 */
    PINT0_PIHT      = 0x02,
    PINT1_PILT      = 0x04,
    PINT1_PIHT      = 0x08,

    /* ambient, prox threshold/hysteresis bits in 0xA4-A5:  not used (yet?) */

    IBUSY           = 0x01, /* register 0xA6 */
    SINT_IRBEAM     = 0x02,
    SINT_SMUX       = 0x04,
    SINT_FD         = 0x08,
    SINT_ALS_MAN_AZ = 0x10,
    SINT_AUX        = 0x20,

    INIT_BUSY       = 0x01, /* register 0xA7 */

    RAM_BANK_0      = 0x00, /* register 0xA9 */
    RAM_BANK_1      = 0x01,
    RAM_BANK_2      = 0x02,
    RAM_BANK_3      = 0x03,
    ALS_TRIG_LONG   = 0x04,
    PRX_TRIG_LONG   = 0x08,
    REG_BANK        = 0x10,
    LOWPWR_IDLE     = 0x20,
    PRX_OFFSET2X    = 0x40,
    PGOFF_HIRES     = 0x80,

    AGAIN_1_HALF    = 0x00, /* register 0xAA */
    AGAIN_1         = 0x01,
    AGAIN_2         = 0x02,
    AGAIN_4         = 0x03,
    AGAIN_8         = 0x04,
    AGAIN_16        = 0x05,
    AGAIN_32        = 0x06,
    AGAIN_64        = 0x07,
    AGAIN_128       = 0x08,
    AGAIN_256       = 0x09,
    AGAIN_512       = 0x0a,
    AGAIN_1024      = 0x0b,    
    ALS_TRIG_FAST   = 0x40,
    S4S_MODE        = 0x80,

    HXTALK_MODE1    = 0x20, /* register 0xAC */

    SMUX_CMD_ROM_INIT   = 0x00, /* register 0xAF */
    SMUX_CMD_READ       = 0x08,
    SMUX_CMD_WRITE      = 0x10,
    SMUX_CMD_ARRAY_MODE = 0x18,

    SWAP_PROX_ALS5  = 0x01, /* register 0xB1 */
    ALS_AGC_ENABLE  = 0x04,
    FD_AGC_ENABLE   = 0x08,
    PROX_BEFORE_EACH_ALS    = 0x10,

    SIEN_AUX        = 0x08, /* register 0xB2 */
    SIEN_SMUX       = 0x10,
    SIEN_ALS_MAN_AZ = 0x20,
    SIEN_FD         = 0x40,
    SIEN_IRBEAM     = 0x80,

    FD_PERS_ALWAYS  = 0x00, /* register 0xB3 */
    FD_PERS_1       = 0x01,
    FD_PERS_2       = 0x02,
    FD_PERS_4       = 0x03,
    FD_PERS_8       = 0x04,
    FD_PERS_16      = 0x05,
    FD_PERS_32      = 0x06,
    FD_PERS_64      = 0x07,

    TRIGGER_APF_ALIGN   = 0x10, /* register 0xB4 */
    PRX_TRIGGER_FAST= 0x20,
    PINT_DIRECT     = 0x40,
    AINT_DIRECT     = 0x80,

    PROX_FILTER_1   = 0x00, /* register 0xB8 */
    PROX_FILTER_2   = 0x01,
    PROX_FILTER_4   = 0x02,
    PROX_FILTER_8   = 0x03,
    HXTALK_MODE2    = 0x80,

    PGAIN_1         = 0x00, /* register 0xBB */
    PGAIN_2         = 0x01,
    PGAIN_4         = 0x02,
    PGAIN_8         = 0x03,
    
    PPLEN_4uS       = 0x00, /* register 0xBC */
    PPLEN_8uS       = 0x01,
    PPLEN_16uS      = 0x02,
    PPLEN_32uS      = 0x03,
    
    FD_COMPARE_32_32NDS = 0x00, /* register 0xD7 */
    FD_COMPARE_24_32NDS = 0x01,
    FD_COMPARE_16_32NDS = 0x02,
    FD_COMPARE_12_32NDS = 0x03,
    FD_COMPARE_8_32NDS  = 0x04,
    FD_COMPARE_6_32NDS  = 0x05,
    FD_COMPARE_4_32NDS  = 0x06,
    FD_COMPARE_3_32NDS  = 0x07,
    FD_SAMPLES_8    = 0x00,
    FD_SAMPLES_16   = 0x08,
    FD_SAMPLES_32   = 0x10,
    FD_SAMPLES_64   = 0x18,
    FD_SAMPLES_128  = 0x20,
    FD_SAMPLES_256  = 0x28,
    FD_SAMPLES_512  = 0x30,
    FD_SAMPLES_1024 = 0x38,
    
    FD_GAIN_1_HALF  = 0x00, /* register 0xDA */
    FD_GAIN_1       = 0x08,
    FD_GAIN_2       = 0x10,
    FD_GAIN_4       = 0x18,
    FD_GAIN_8       = 0x20,
    FD_GAIN_16      = 0x28,
    FD_GAIN_32      = 0x30,
    FD_GAIN_64      = 0x38,
    FD_GAIN_128     = 0x40,
    FD_GAIN_256     = 0x48,
    
    FD_100HZ_FLICKER= 0x01, /* register 0xDB */
    FD_120HZ_FLICKER= 0x02,
    FD_100HZ_VALID  = 0x04,
    FD_120HZ_VALID  = 0x08,
    FD_SAT_DETECTED = 0x10,
    FD_MEAS_VALID   = 0x20,

    ISTART_MOBEAM   = 0x01, /* register 0xE8 */
    ISTART_REMCON   = 0x02,

    START_OFFSET_CALIB  = 0x01, /* register 0xEA */

    DCAVG_AUTO_BSLN = 0x80, /* register 0xEB */
    DCAVG_AUTO_OFFSET_ADJUST = 0x40,
    BINSRCH_SKIP    = 0x08,
    
    BASELINE_ADJUSTED = 0x04, /* register 0xEE */
    OFFSET_ADJUSTED = 0x02,
    CALIB_FINISHED    = 0x01,

    SIEN            = 0x01, /* register 0xF9 */
    CIEN            = 0x02,
    FIEN            = 0x04,
    AIEN            = 0x08,
    PIEN0           = 0x10,
    PIEN1           = 0x20,
    PSIEN           = 0x40,
    ASIEN_FDSIEN    = 0x80,
    AMS_ALL_IENS    = (AIEN+PIEN0+PIEN1+FIEN+CIEN),
    LAST_IN_ENUM_LIST
}ams_regOptions_t;


typedef enum _3407_regMasks {
    MASK_PON            = 0x01, /* register 0x80 */
    MASK_AEN            = 0x02,
    MASK_PEN            = 0x04,
    MASK_WEN            = 0x08,
    MASK_SMUXEN         = 0x10,
    MASK_FDEN           = 0x40,
    MASK_IBEN           = 0x80,

    MASK_ATIME          = 0xFF, /* register 0x81 */

    MASK_PTIME          = 0xFF, /* register 0x82 */

    MASK_WTIME          = 0xFF, /* register 0x83 */

    MASK_AILT           = 0xFFFF, /* register 0x84 */
    MASK_AILH           = 0xFFFF, /* register 0x86 */

    MASK_PILT           = 0xFFFF, /* register 0x88 */
    MASK_PILH           = 0xFFFF, /* register 0x8A */

    MASK_AUXID          = 0x0F, /* register 0x90 */

    MASK_REV_ID         = 0x07, /* register 0x91 */

    MASK_ID             = 0xFC, /* register 0x92 */

    MASK_SINT           = 0x01, /* register 0x93 */
    MASK_CINT           = 0x02,
    MASK_FINT           = 0x04,
    MASK_AINT           = 0x08,
    MASK_PINT0          = 0x10,
    MASK_PINT1          = 0x20,
    MASK_PSAT           = 0x40,
    MASK_ASAT_FDSAT     = 0x80,
    MASK_ALS_INT_ALL    = MASK_AINT,
    MASK_PROX_INT_ALL_0 = (MASK_PINT0 | MASK_PSAT),
    MASK_PROX_INT_ALL_1 = (MASK_PINT1 | MASK_PSAT),
    MASK_PROX_INT_ALL_0_1 = (MASK_PINT0 | MASK_PINT1 | MASK_PSAT),
    /*MASK_PROX_INT_ALL   = (MASK_PINT0),*/
    MASK_INT_ALL        = (0xFF),

    MASK_ADATA          = 0xFFFF, /* registers 0x95-0xA0 */

    MASK_ASAT_ANALOG    = 0x08, /* register 0xA3 */
    MASK_ASAT_DIGITAL   = 0x10,
    MASK_AVALID         = 0x40,
    MASK_PVALID0        = 0x80,

    MASK_PSAT_AMB       = 0x01, /* register 0xA4 */
    MASK_PSAT_REFL      = 0x02,
    MASK_PSAT_ADC       = 0x04,

    MASK_RAM_BANK       = 0x03, /* register 0xA9 */

    MASK_AGAIN          = 0x1F, /* register 0xAA */

    MASK_HXTALK_MODE1    = 0x20, /* register 0xAC */

    MASK_SMUX_CMD       = 0x18, /* register 0xAF */

    MASK_DISABLE_FLICKER_AUTOGAIN               = 0x08, /* register 0xB1 */
    MASK_AUTOGAIN               = 0x04, /* register 0xB1 */
    MASK_PROX_BEFORE_EACH_ALS   = 0x10,

    MASK_SIEN_FD        = 0x40, /* register 0xB2 */
    MASK_SIEN_IRBEAM     = 0x80,

    MASK_FD_PERS        = 0x07, /* register 0xB3 */
    MASK_AGC_HYST_LOW   = 0x30,
    MASK_AGC_HYST_HIGH  = 0xC0,

    MASK_PRX_TRIGGER_FAST   = 0x20, /* register 0xB4 */
    MASK_PINT_DIRECT    = 0x40,
    MASK_AINT_DIRECT    = 0x80,

    MASK_PROX_FILTER    = 0x03, /* register 0xB8 */
    MASK_HXTALK_MODE2   = 0x80,

    MASK_PLDRIVE0       = 0x7f, /* register 0xB9 */

    MASK_APERS          = 0x0F, /* register 0xBD */
    MASK_PPERS          = 0xF0,

    MASK_PPULSE         = 0x3F, /* register 0xBC */
    MASK_PPLEN          = 0xC0,

    MASK_PGAIN          = 0x03, /* register 0xBB */

    MASK_PLDRIVE        = 0x7F, /* Registers 0xB9, 0xBA */

    MASK_FD_GAIN = 0xF8, /* Register 0xDA */
    MASK_FD_TIME_MSBits = 0x07, /* Register 0xDA */

    MASK_100HZ_FLICKER  = 0x05, /* Register 0xDB */
    MASK_120HZ_FLICKER  = 0x0A,
    MASK_CLEAR_FLICKER_STATUS  = 0x3C,
    MASK_FLICKER_VALID  = 0x2C,

    MASK_SLEW           = 0x10, /* register E0 */
    MASK_ISQZT          = 0x07,
    
    MASK_OFFSET_CALIB   = 0x01, /* register 0xEA */

    MASK_DCAVG_AUTO_BSLN = 0x80, /* register 0xEB */
    MASK_BINSRCH_SKIP    = 0x08,

    MASK_PROX_AUTO_OFFSET_ADJUST    = 0x40, /* register 0xEC */
    MASK_PXAVG_AUTO_BSLN    = 0x08, /* register 0xEC */

    MASK_BINSRCH_TARGET = 0xE0, /* register 0xED */

    MASK_SIEN           = 0x01, /* register 0xF9 */
    MASK_CIEN           = 0x02,
    MASK_FIEN           = 0x04,
    MASK_AIEN           = 0x08,
    MASK_PIEN0          = 0x10,
    MASK_PIEN1          = 0x20,
    MASK_PSIEN          = 0x40,
    MASK_ASIEN_FDSIEN   = 0x80,

    MASK_AGC_HYST       = 0x30,
    MASK_SOFT_RESET     = 0x04, /* register 0xF3 */

    MASK_LAST_IN_ENUMLIST
}ams_regMask_t;


#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif




#define AMS_ENABLE_ALS_EX()         {ctx->shadowEnableReg |= (AEN); \
    ams_setByte (ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg); \
    ams_setField (ctx->portHndl, DEVREG_INTENAB, HIGH, (MASK_AIEN | MASK_ASIEN_FDSIEN));\
    }
#define AMS_DISABLE_ALS_EX()        {ctx->shadowEnableReg &= ~(AEN); \
    ams_setByte (ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg); \
    ams_setField (ctx->portHndl, DEVREG_INTENAB, LOW, (MASK_AIEN | MASK_ASIEN_FDSIEN));\
    }
/* REENABLE only enables those that were on record as being enabled */
#define AMS_REENABLE()              {ams_setByte (ctx->portHndl, DEVREG_ENABLE,  ctx->shadowEnableReg);}
/* DISABLE_ALS disables ALS w/o recording that as its new state */
#define AMS_DISABLE_ALS()           {ams_setField(ctx->portHndl, DEVREG_ENABLE,  LOW,                       (MASK_AEN));}
#define AMS_REENABLE_ALS()          {ams_setField(ctx->portHndl, DEVREG_ENABLE,  HIGH,                      (MASK_AEN));}

#define AMS_SET_ALS_TIME(uSec)      {ams_setByte (ctx->portHndl, DEVREG_ATIME,   alsTimeUsToReg(uSec));}

#define AMS_GET_ALS_GAIN(scaledGain, gain) {ams_getByte (ctx->portHndl, DEVREG_ASTATUS, &(gain)); \
    scaledGain = alsGain_conversion[(gain) & 0x0f];}



#define AMS_SET_ALS_STEP_TIME(uSec)      {ams_setWord(ctx->portHndl, DEVREG_ASTEPL,   alsTimeUsToReg(uSec*1000));}

#define AMS_SET_ALS_GAIN(mGain)     {ams_setField(ctx->portHndl, DEVREG_CFG1,    alsGainToReg(mGain),       MASK_AGAIN);}
#define AMS_SET_ALS_INDEX_TO_GAIN(mGain)     {ams_setField(ctx->portHndl, DEVREG_CFG1,    alsIndexToGain(mGain),       MASK_AGAIN);}
#define AMS_SET_FLICKER_INDEX_TO_FDGAIN(mGain)     {ams_setField(ctx->portHndl, DEVREG_FD_CFG3,    alsIndexToGain(mGain) ,       MASK_FD_GAIN);}

#define AMS_DISABLE_ALS_FLICKER()           {ams_setField(ctx->portHndl, DEVREG_ENABLE,  LOW,                       (MASK_AEN|MASK_FDEN));}
#define AMS_ENABLE_ALS_FLICKER()           {ams_setField(ctx->portHndl, DEVREG_ENABLE,  HIGH,                       (MASK_AEN|MASK_FDEN));}



#define AMS_SET_ALS_PERS(persCode)  {ams_setField(ctx->portHndl, DEVREG_PERS,    (persCode),                MASK_APERS);}
#define AMS_CLR_ALS_INT()           {ams_setByte (ctx->portHndl, DEVREG_STATUS,  (AINT | ASAT_FDSAT));}
#define AMS_SET_ALS_THRS_LOW(x)     {ams_setWord (ctx->portHndl, DEVREG_AILTL,   (x));}
#define AMS_SET_ALS_THRS_HIGH(x)    {ams_setWord (ctx->portHndl, DEVREG_AIHTL,   (x));}
#define AMS_SET_ALS_AUTOGAIN(x)     {ams_setField(ctx->portHndl, DEVREG_CFG8,    (x),       MASK_AUTOGAIN);}
#define AMS_SET_ALS_AGC_HYST(x)     {ams_setField(ctx->portHndl, DEVREG_CFG10,  ((x)<<4),   MASK_AGC_HYST);}

#define AMS_DISABLE_FLICKER_AUTOGAIN(x)     {ams_setField(ctx->portHndl, DEVREG_CFG8,    (x),       MASK_DISABLE_FLICKER_AUTOGAIN);}

/* Get CRGB and whatever Wideband it may have */
#define AMS_ALS_GET_CRGB_W(x)       {ams_getBuf  (ctx->portHndl, DEVREG_ADATA0L,(uint8_t*) (x),                       12);}

#define AMS_ENABLE_PROX_EX()        {ctx->shadowEnableReg |= (PEN); \
    ams_setByte (ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg); \
    ams_setField (ctx->portHndl, DEVREG_INTENAB, HIGH, (MASK_PIEN0));\
    }
#define AMS_DISABLE_PROX_EX()       {ctx->shadowEnableReg &= ~(PEN); \
    ams_setByte (ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg); \
    ams_setField (ctx->portHndl, DEVREG_INTENAB, LOW, (MASK_PIEN0));\
    }
#define AMS_SET_PROX_GAIN(mGain)    {ams_setField(ctx->portHndl, DEVREG_PCFG4,   proxGainToReg(mGain),      MASK_PGAIN);}
#define AMS_SET_PROX_PERS(reps)     {ams_setField(ctx->portHndl, DEVREG_PERS,    (uint8_t)(AMS_PRX_PERS_TO_REG(reps)), MASK_PPERS);}
#define AMS_SET_PROX_LEDDRIVE(mA)   {ams_setField(ctx->portHndl, DEVREG_PCFG2,   AMS_PRX_CURRENT_TO_REG(mA),0x1e);}
#define AMS_CLR_PROX_INT()          {ams_setByte (ctx->portHndl, DEVREG_STATUS,  (PINT0 | PINT1 | CINT | PGSAT));}
#define AMS_SET_PROX_THRS_LOW(x)    {ams_setWord (ctx->portHndl, DEVREG_PILT0L,  (x));}
#define AMS_SET_PROX_THRS_HIGH(x)   {ams_setWord (ctx->portHndl, DEVREG_PIHT0L,  (x));}
#define AMS_SET_PROX_PULSE_LEN(x)   {ams_setField(ctx->portHndl, DEVREG_PCFG5,   (x) << 6,                  MASK_PPLEN);}
#define AMS_SET_PROX_PULSE_COUNT(x) {ams_setField(ctx->portHndl, DEVREG_PCFG5,   (x) << 0,                  MASK_PPULSE);}
#define AMS_ENABLE_PROX_INT()       {ams_setField(ctx->portHndl, DEVREG_INTENAB, HIGH,                      MASK_PROX_INT_ALL);}
#define AMS_DISABLE_PROX_INT()      {ams_setField(ctx->portHndl, DEVREG_INTENAB, LOW,                       MASK_PROX_INT_ALL);}

#define AMS_SET_CAL_BINSRCH(x)      {ams_setField(ctx->portHndl, DEVREG_CALIBCFG2,(x) << 5,                 MASK_BINSRCH);}
#define AMS_SET_CAL_PROX_AUTO_OFFSET_ADJUST(x)      {ams_setField(ctx->portHndl, DEVREG_CALIBCFG1,HIGH,     MASK_PROX_AUTO_OFFSET_ADJUST);}
#define AMS_ENABLE_CAL_INT()        {ams_setField(ctx->portHndl, DEVREG_INTENAB, HIGH,                      MASK_CINT);}
#define AMS_DISABLE_CAL_INT()       {ams_setField(ctx->portHndl, DEVREG_INTENAB, LOW,                       MASK_CINT);}

#define AMS_ENABLE_IRBEAM_INT()     {ams_setField(ctx->portHndl, DEVREG_INTENAB, HIGH,                      MASK_SIEN);}
#define AMS_DISABLE_IRBEAM_INT()    {ams_setField(ctx->portHndl, DEVREG_INTENAB, LOW,                       MASK_SIEN);}
#define AMS_SET_IRBEAM_CURRENT(mA)  {ams_setField(ctx->portHndl, DEVREG_PCFG2,   irbeamCurrentToReg(mA),    MASK_PLDRIVE);}


#define AMS_READ_S_MUX() {ams_setField(ctx->portHndl, DEVREG_CFG6,((1)<<3),MASK_SMUX_CMD);}
#define AMS_WRITE_S_MUX() {ams_setField(ctx->portHndl, DEVREG_CFG6,((2)<<3),MASK_SMUX_CMD);}
#define AMS_CLOSE_S_MUX() {ams_setField(ctx->portHndl, DEVREG_CFG6,0x00,MASK_SMUX_CMD);}


enum _tcs3407_calibcfg0 {
    tcs3407_DCAVG_ITERATIONS        = (0x7 << 0),
    tcs3407_BINSRCH_SKIP            = (0x1 << 3),
    tcs3407_ELECTRICAL_CALIBRATION  = (0x1 << 4),
    tcs3407_BINSRCH_NOSUPPRESS_APC  = (0x1 << 5),
    tcs3407_DCAVG_AUTO_OFFSET_ADJUST= (0x1 << 6),
    tcs3407_DCAVG_AUTO_BASELINE     = (0x1 << 7)
};

enum _tcs3407_calibcfg1 {
    tcs3407_PXDCAVG_BASELINE_WINDOW = (0x7 << 0),
    tcs3407_PXDCAVG_AUTO_BASELINE   = (0x1 << 3),
    tcs3407_PX_SUBTRACT_BASELINE    = (0x1 << 4),
    tcs3407_PX_AVERAGING_MODE       = (0x1 << 5),
    tcs3407_PROX_AUTO_OFFSET_ADJUST = (0x1 << 6),
    tcs3407_PXDIR_CNT_ENABLE        = (0x1 << 7)
};

typedef struct _deviceRegisterTable {
    uint8_t address;
    uint8_t resetValue;
}deviceRegisterTable_t;

typedef enum _3407_config_options {
    AMS_CONFIG_ENABLE,
    AMS_CONFIG_THRESHOLD,
    AMS_CONFIG_OPTION_LAST
}deviceConfigOptions_t;

typedef enum _3407_mode {
    MODE_OFF            = (0),
    MODE_ALS_LUX        = (1 << 0),
    MODE_ALS_RGB        = (1 << 1),
    MODE_ALS_CT         = (1 << 2),
    MODE_ALS_WIDEBAND   = (1 << 3),
    MODE_ALS_ALL        = (MODE_ALS_LUX | MODE_ALS_RGB | MODE_ALS_CT | MODE_ALS_WIDEBAND),
    MODE_FLICKER        = (1 << 4), /* is independent of ALS in this model */
    MODE_PROX           = (1 << 5),
    MODE_IRBEAM         = (1 << 6),
    MODE_UNKNOWN    /* must be in last position */
} ams_mode_t;

typedef enum _3407_configureFeature {
    AMS_CONFIG_PROX,
    AMS_CONFIG_ALS_LUX,
    AMS_CONFIG_ALS_RGB,
    AMS_CONFIG_ALS_CT,
    AMS_CONFIG_ALS_WIDEBAND,
    AMS_CONFIG_HW_FLICKER,
    AMS_CONFIG_SW_FLICKER,
    AMS_CONFIG_FEATURE_LAST
}ams_configureFeature_t;

typedef struct _calibrationData {
    uint32_t    timeBase_us;
    uint32_t    adcMaxCount;
    uint8_t     alsThresholdHigh; /* in % */
    uint8_t     alsThresholdLow;  /* in % */
    uint16_t    alsCalibrationLuxTargetHigh;
    uint16_t    alsCalibrationLuxTargetLow;
    uint16_t    alsCalibrationLuxTarget;
    uint16_t    alsCalibrationLuxTargetError;
    uint16_t    alsCalibrationFactor;        /* multiplicative factor default 1000 */
    uint16_t    proxAdaptiveThresholdFactor;
    uint8_t     proxAdaptiveThreshold; /* in bits */
    uint16_t    proxHardThreshold;
    char        deviceName[8];
    int32_t     Cc;
    int32_t     Rc;
    int32_t     Gc;
    int32_t     Bc;
    int32_t     alsCoefC;
    int32_t     alsCoefR;
    int32_t     alsCoefG;
    int32_t     alsCoefB;
    int16_t     alsDfg;
    uint16_t    alsCctOffset;
    uint16_t    alsCctCoef;
    int32_t     Wbc;
} ams_calibrationData_t;

typedef struct _flickerParams {
    uint32_t    sampling_time;
    uint16_t    gain;
    uint8_t     compare;
    uint8_t     statusReg;
    uint8_t flicker_finished;	
    ams_apiAlsFlicker_t lastValid;	
    uint16_t    mHzbysw;
    uint16_t flicker_raw_data;
	
} ams_flicker_ctx_t;

typedef struct _3407Context {
    ams_deviceIdentifier_e deviceId;
    uint64_t timeStamp;
    AMS_PORT_portHndl * portHndl;
    ams_mode_t mode;
#ifdef AMS_PHY_SUPPORT_SHADOW
    uint8_t shadow[DEVREG_REG_MAX];
#endif
    ams_ccb_als_ctx_t ccbAlsCtx;
    ams_flicker_ctx_t flickerCtx;
    ams_calibrationData_t * systemCalibrationData;
	
    bool alwaysReadAls;       /* read ADATA every ams_deviceEventHandler call
                                 regardless of xINT bits */
    bool alwaysReadProx;      /* ditto PDATA */
    bool alwaysReadFlicker;
    //uint8_t valid_flickerhz_count;
	
    uint32_t updateAvailable;
    uint8_t shadowEnableReg;
    uint8_t shadowIntenabReg;
    uint8_t shadowStatus1Reg;
    uint8_t shadowStatus2Reg;
    uint8_t smux_buffer[20];
}ams_deviceCtx_t;

typedef enum _sensorType {
    AMS_NO_SENSOR_AVAILABLE,
    AMS_AMBIENT_SENSOR,
    AMS_FLICKER_SENSOR,
    AMS_SW_FLICKER_SENSOR    ,
    AMS_PROXIMITY_SENSOR,
    AMS_WIDEBAND_ALS_SENSOR,
    AMS_LAST_SENSOR
}ams_sensorType_t;

typedef struct _sensorInfo {
    uint32_t    standbyCurrent_uA;
    uint32_t    activeCurrent_uA;
    uint32_t    rangeMin;
    uint32_t    rangeMax;
    char *      driverName;
    uint8_t     maxPolRate;
    uint8_t     adcBits;
} ams_SensorInfo_t;

typedef struct _deviceInfo {
    uint32_t    memorySize;
    ams_calibrationData_t defaultCalibrationData;
    ams_SensorInfo_t proxSensor;
    ams_SensorInfo_t alsSensor;
    ams_SensorInfo_t mobeamSensor;
    ams_SensorInfo_t remconSensor;
    ams_sensorType_t tableSubSensors[10];
    uint8_t     numberOfSubSensors;
    char *      driverVersion;
    char *      deviceModel;
    char *      deviceName;
}ams_deviceInfo_t;

extern uint32_t alsGain_conversion[];
extern uint8_t alsGainToReg(uint32_t x);
extern uint8_t alsIndexToGain(uint8_t x);

extern uint8_t FlickerGainToReg(uint32_t x);

extern uint16_t alsTimeUsToReg(uint32_t x);
extern uint32_t proxGain_conversion[];
extern uint8_t proxGainToReg(uint32_t x);
extern uint32_t proxRegToTime (uint8_t x);
extern uint8_t irbeamCurrentToReg(uint32_t mA);

extern deviceRegisterTable_t deviceRegisterDefinition[DEVREG_REG_MAX];
extern ams_deviceIdentifier_e ams_validateDevice(AMS_PORT_portHndl * portHndl);
extern bool ams_getDeviceInfo(ams_deviceInfo_t * info);
extern bool ams_getMode(ams_deviceCtx_t * ctx, ams_mode_t * mode);
extern bool ams_deviceInit(ams_deviceCtx_t * ctx, AMS_PORT_portHndl * portHndl, ams_calibrationData_t * calibrationData);
extern bool ams_deviceEventHandler(ams_deviceCtx_t * ctx);
extern bool ams_devicePollingHandler(ams_deviceCtx_t * ctx);
extern uint32_t ams_getResult(ams_deviceCtx_t * ctx);
extern bool ams_deviceSetConfig(ams_deviceCtx_t * ctx, ams_configureFeature_t feature, deviceConfigOptions_t option, uint32_t data);
extern bool ams_deviceGetConfig(ams_deviceCtx_t * ctx);
extern bool ams_deviceCalibrateLux(ams_deviceCtx_t * ctx, ams_calibrationData_t * calibrationData);
extern bool ams_deviceSelfTest(ams_deviceCtx_t * ctx, AMS_PORT_portHndl * portHndl);
extern bool ams_deviceGetAls(ams_deviceCtx_t * ctx, ams_apiAls_t * exportData);
extern bool ams_deviceGetPrx(ams_deviceCtx_t * ctx, ams_apiPrx_t * exportData);
extern bool ams_deviceCalibrateProx(ams_deviceCtx_t * ctx, ams_calibrationData_t * calibrationData);
extern bool ams_deviceGetFlicker(ams_deviceCtx_t * ctx, ams_apiAlsFlicker_t * exportData);
extern void ams_deviceGetSWFlicker(ams_deviceCtx_t * ctx, ams_apiAlsFlicker_t * exportData);
extern bool ams_deviceGetRegdump(ams_deviceCtx_t * ctx) ;
extern void ams_smux_read(ams_deviceCtx_t *ctx ) ;
extern void ams_smux_set(ams_deviceCtx_t *ctx , ams_deviceIdentifier_e devID);

#ifdef  __cplusplus
}
#endif

#else

#endif  /* __AMS_DEVICE_CONTROL_BLOCK_H__ */

