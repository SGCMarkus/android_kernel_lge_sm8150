/*
 * Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

// Topology ID, Module ID, Parameter ID
#define AUDIO_TOPOLOGY_LVACFQ     0x1000BFFA
#define AUDIO_MODULE_AC           0x1000BA01
#define AUDIO_PARAM_AC_CONTROL    0x1000BA21

#define ADM_RAM_STATUS 0x00010D01
#define LGE_SWITCH_RAM_STATUS "ram_status"

#define AC_INPUTGAIN_MIN    (-72)
#define AC_OUTPUTGAIN_MIN   (-96)
#define AC_VOL_VIDEO_MUTE   (-800)
#define AC_VOL_DOWN_WITHDRC (-17)
#define AC_HPF_ENHANCED     2
#define AC_HPF_MIN          50
#define AC_HPF_MAX          350


typedef uint16_t AC_ModeWord_bm;

#define AC_MODE_HPF ((AC_ModeWord_bm)1)
#define AC_MODE_CAL ((AC_ModeWord_bm)2)
#define AC_MODE_NS ((AC_ModeWord_bm)4)
#define AC_MODE_WNS ((AC_ModeWord_bm)8)
#define AC_MODE_AZ ((AC_ModeWord_bm)16)
#define AC_MODE_RMS_LIMITER ((AC_ModeWord_bm)32)
#define AC_MODE_MIC_COVERAGE ((AC_ModeWord_bm)64)
#define AC_MODE_HIGH_SPL ((AC_ModeWord_bm)128)
#define AC_MODE_SAM_MONITORING ((AC_ModeWord_bm)256)
#define AC_MODE_DOWNMIXING ((AC_ModeWord_bm)512)
#define AC_MODE_AUDIO_ROTATION ((AC_ModeWord_bm)1024)
#define AC_MODE_FRONT_BACK ((AC_ModeWord_bm)2048)
#define AC_MODE_AVL ((AC_ModeWord_bm)4096)
#define AC_MODE_FENS ((AC_ModeWord_bm)8192)
#define AC_MODE_LIMITER ((AC_ModeWord_bm)16384)
#define AC_MODE_DRC ((AC_ModeWord_bm)32768)
#define AC_MODEWORD_BM_MIN ((AC_ModeWord_bm)0)
#define AC_MODEWORD_BM_MAX ((AC_ModeWord_bm)((1) | (2) | (4) | (8) | (16) | (32) | (64) | (128) | (256) | (512) | (1024) | (2048) | (4096) | (8192) | (16384) | (32768) | (0)))

struct tx_control_param_t {
    int32_t OperatingMode;
    uint16_t Mode;
    uint16_t Mode2;
    int32_t AudioZoomMode;
    int32_t InputChannelType[4];
    uint16_t MicrophonePositionX[3];
    uint16_t MicrophonePositionY[3];
    uint16_t MicrophonePositionZ[3];
    int16_t HighpassFrequency;
    int32_t InputDelay[4];
    int16_t InputGain[4];
    int16_t OutputGain;
    int16_t ZoomFactor[2];
    int16_t NoiseSuppression[2];
    int16_t IdleNoiseSuppression;
    int16_t IdleNoiseThreshold;
    int16_t WindNoiseSuppression[2];
    int16_t SideGain[2];
    int16_t CenterGain[2];
    int16_t LimiterThreshold;
    int16_t AvlTargetGain;
    uint16_t AvlAttackTime;
    uint16_t AvlReleaseTime;
    uint16_t MdrcBandCount;
    uint16_t MdrcBand0_LowFrequency;
    uint16_t MdrcBand0_KneeCount;
    uint16_t MdrcBand0_AttackTime;
    uint16_t MdrcBand0_ReleaseTime;
    int16_t MdrcBand0_InputLevels[5];
    int16_t MdrcBand0_OutputLevels[5];
    uint16_t MdrcBand1_LowFrequency;
    uint16_t MdrcBand1_KneeCount;
    uint16_t MdrcBand1_AttackTime;
    uint16_t MdrcBand1_ReleaseTime;
    int16_t MdrcBand1_InputLevels[5];
    int16_t MdrcBand1_OutputLevels[5];
    uint16_t MdrcBand2_LowFrequency;
    uint16_t MdrcBand2_KneeCount;
    uint16_t MdrcBand2_AttackTime;
    uint16_t MdrcBand2_ReleaseTime;
    int16_t MdrcBand2_InputLevels[5];
    int16_t MdrcBand2_OutputLevels[5];
    uint16_t MdrcBand3_LowFrequency;
    uint16_t MdrcBand3_KneeCount;
    uint16_t MdrcBand3_AttackTime;
    uint16_t MdrcBand3_ReleaseTime;
    int16_t MdrcBand3_InputLevels[5];
    int16_t MdrcBand3_OutputLevels[5];
    uint16_t MdrcBand4_LowFrequency;
    uint16_t MdrcBand4_KneeCount;
    uint16_t MdrcBand4_AttackTime;
    uint16_t MdrcBand4_ReleaseTime;
    int16_t MdrcBand4_InputLevels[5];
    int16_t MdrcBand4_OutputLevels[5];
    int16_t IM_BeamDetection;
    int16_t IM_BeamGaindB;
    int32_t IM_GainAttackTime;
    int32_t IM_GainReleaseTime;
    int16_t IM_ZoomWidthMic1;
    int16_t IM_ZoomWidthMic2;
    int16_t IM_VADThresholdHigh;
    int16_t IM_VADThresholdLow;
    int16_t IM_ThetaStep;
    int16_t IM_DeltaAmp;
    int16_t IM_BypassFilter;
    int16_t IM_MaskSelect;
    int16_t IM_SmoothCoef;
    int16_t HighSplAlgoChoice;
    int16_t HighSplSaturationCheckType;
    int16_t HighSplSaturationThreshold;
    int16_t HighSplSaturationStrength;
    int16_t HighSplSaturationMax;
    int16_t HighSplSaturationSlowRelease;
    int16_t HighSplSaturationSlowAttack;
    int16_t HighSplSaturationFastThreshold;
    int16_t HighSplSaturationThreshold2;
    int16_t HighSplSaturationStrength2;
    int16_t HighSplSaturationMax2;
    int16_t HighSplSaturationSlowRelease2;
    int16_t HighSplSaturationSlowAttack2;
    int16_t HighSplSaturationFastThreshold2;
    int16_t HighSplMixALowFreq;
    int16_t HighSplMixAHighFreq;
    int16_t HighSplMixAInBandMixingCoeff;
    int16_t HighSplAlgoCoreStrength;
    int16_t HighSplAlgoCoreClippingReduction;
    int16_t HighSplSamBandPassHighFrequency;
    int16_t HighSplCurrentSaturationThreshold;
    int16_t HighSplSamLevelThreshold;
    int16_t HighSplMixCLowFreq;
    int16_t HighSplMixCHighFreq;
    int16_t HighSplSamCalibCoeffFreqStart;
    int16_t HighSplSamCalibCoeffMax;
    int16_t HighSplSamCalibCoeffMid;
    int16_t HighSplSamCalibCoeffMin;
    int16_t HighSplMixCoherence;
    int16_t HighSplMixCoherenceRelease;
    int16_t HighSplMixCoherenceAttack;
    int16_t FENS_limit_NS;
    int16_t FENS_NoiseThreshold;
    int16_t RsvdParams[10];
} __packed;

struct adm_tx_config_param {
	struct param_hdr_v3            param_hdr;
	struct tx_control_param_t           tx_control_param;
} __packed;

int q6adm_set_tx_cfg_parms(int port_id, struct tx_control_param_t *tx_control_param);
