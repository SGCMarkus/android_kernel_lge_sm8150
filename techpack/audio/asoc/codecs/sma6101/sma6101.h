/*
 * sma6101.h -- sma6101 ALSA SoC Audio driver
 *
 * r020, 2019.02.18
 *
 * Copyright 2018 Silicon Mitus Corporation / Iron Device Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _SMA6101_H
#define _SMA6101_H

/* I2C Device Address - 7bit */
#define  SMA6101_I2C_ADDR	0x1e

#define  SMA6101_EXTERNAL_CLOCK_19_2	0x00
#define  SMA6101_EXTERNAL_CLOCK_24_576	0x01
#define  SMA6101_PLL_CLKIN_MCLK         0x02
#define  SMA6101_PLL_CLKIN_BCLK         0x03

/* Class H Mode Scenario */
#define  SMA6101_CLASS_H_VOICE_MODE     0
#define  SMA6101_CLASS_H_MUSIC_MODE     1
#define  SMA6101_CLASS_H_LN_VOICE_MODE  2
#define  SMA6101_CLASS_H_ULN_VOICE_MODE 3
#define  SMA6101_CLASS_H_MODE_OFF       4

/* FDPEC Gain Table */
#define  FDPEC_GAIN_x2P6     0
#define  FDPEC_GAIN_x4       1
#define  FDPEC_GAIN_x6       2
#define  FDPEC_GAIN_x8       3

/*
 * SMA6101 Register Definition
 */

/* SMA6101 Register Addresses */
#define  SMA6101_00_SYSTEM_CTRL	    0x00
#define  SMA6101_01_INPUT1_CTRL1	0x01
#define  SMA6101_02_INPUT1_CTRL2	0x02
#define  SMA6101_03_INPUT1_CTRL3	0x03
#define  SMA6101_04_INPUT1_CTRL4	0x04
#define  SMA6101_05_BROWNOUT_SET1	0x05
#define  SMA6101_06_BROWNOUT_SET2	0x06
#define  SMA6101_07_BROWNOUT_SET3	0x07
#define  SMA6101_08_BROWNOUT_SET4	0x08
#define  SMA6101_09_OUTPUT_CTRL		0x09
#define  SMA6101_0A_SPK_VOL		    0x0a
#define  SMA6101_0B_BROWNOUT_SET5	0x0b
#define  SMA6101_0C_BROWNOUT_SET6	0x0c
#define  SMA6101_0D_CLASS_H_CTRL_LVL3	0x0d
#define  SMA6101_0E_MUTE_VOL_CTRL	0x0e
#define  SMA6101_0F_CLASS_H_CTRL_LVL4	0x0f
#define  SMA6101_10_SYSTEM_CTRL1	0x10
#define  SMA6101_11_SYSTEM_CTRL2	0x11
#define  SMA6101_12_SYSTEM_CTRL3	0x12
#define  SMA6101_13_FDPEC_CTRL1		0x13
#define  SMA6101_14_MODULATOR		0x14
#define  SMA6101_15_BASS_SPK1		0x15
#define  SMA6101_16_BASS_SPK2		0x16
#define  SMA6101_17_BASS_SPK3		0x17
#define  SMA6101_18_BASS_SPK4		0x18
#define  SMA6101_19_BASS_SPK5		0x19
#define  SMA6101_1A_BASS_SPK6		0x1a
#define  SMA6101_1B_BASS_SPK7		0x1b
#define  SMA6101_1C_BROWN_OUT_P16	0x1c
#define  SMA6101_1D_BROWN_OUT_P17	0x1d
#define  SMA6101_1E_BROWN_OUT_P18	0x1e
#define  SMA6101_1F_BROWN_OUT_P19	0x1f
#define  SMA6101_20_BROWN_OUT_P20	0x20
#define  SMA6101_21_TDM_RX			0x21
/* 0x22 : Reserved */
#define  SMA6101_23_COMP_LIM1		0x23
#define  SMA6101_24_COMP_LIM2		0x24
#define  SMA6101_25_COMP_LIM3		0x25
#define  SMA6101_26_COMP_LIM4		0x26
/* 0x27 ~ 0x2A : Reserved */
#define  SMA6101_2B_EQ_MODE		    0x2b
/* 0x2C ~ 0x32 : Reserved */
#define  SMA6101_33_SDM_CTRL		0x33
/* 0x34 ~ 0x35 : Reserved */
#define  SMA6101_36_PROTECTION		0x36
#define  SMA6101_37_SLOPE_CTRL		0x37
#define  SMA6101_38_DIS_CLASSH_LVL12	0x38
#define  SMA6101_39_DIS_CLASSH_LVL34	0x39
#define  SMA6101_3A_DIS_CLASSH_LVL56	0x3a
#define  SMA6101_3B_TEST1		    0x3b
#define  SMA6101_3C_TEST2		    0x3c
#define  SMA6101_3D_TEST3		    0x3d
#define  SMA6101_3E_ATEST1		    0x3e
#define  SMA6101_3F_ATEST2		    0x3f
/* Band1 */
#define	 SMA6101_40_EQ_CTRL1		0x40
#define	 SMA6101_41_EQ_CTRL2		0x41
#define	 SMA6101_42_EQ_CTRL3		0x42
#define	 SMA6101_43_EQ_CTRL4		0x43
#define	 SMA6101_44_EQ_CTRL5		0x44
#define	 SMA6101_45_EQ_CTRL6		0x45
#define	 SMA6101_46_EQ_CTRL7		0x46
#define	 SMA6101_47_EQ_CTRL8		0x47
#define	 SMA6101_48_EQ_CTRL9		0x48
#define	 SMA6101_49_EQ_CTRL10		0x49
#define	 SMA6101_4A_EQ_CTRL11		0x4a
#define	 SMA6101_4B_EQ_CTRL12		0x4b
#define	 SMA6101_4C_EQ_CTRL13		0x4c
#define	 SMA6101_4D_EQ_CTRL14		0x4d
#define	 SMA6101_4E_EQ_CTRL15		0x4e
/* Band2 */
#define	 SMA6101_4F_EQ_CTRL16		0x4f
#define	 SMA6101_50_EQ_CTRL17		0x50
#define	 SMA6101_51_EQ_CTRL18		0x51
#define	 SMA6101_52_EQ_CTRL19		0x52
#define	 SMA6101_53_EQ_CTRL20		0x53
#define	 SMA6101_54_EQ_CTRL21		0x54
#define	 SMA6101_55_EQ_CTRL22		0x55
#define	 SMA6101_56_EQ_CTRL23		0x56
#define	 SMA6101_57_EQ_CTRL24		0x57
#define	 SMA6101_58_EQ_CTRL25		0x58
#define	 SMA6101_59_EQ_CTRL26		0x59
#define	 SMA6101_5A_EQ_CTRL27		0x5a
#define	 SMA6101_5B_EQ_CTRL28		0x5b
#define	 SMA6101_5C_EQ_CTRL29		0x5c
#define	 SMA6101_5D_EQ_CTRL30		0x5d
/* Band3 */
#define	 SMA6101_5E_EQ_CTRL31		0x5e
#define	 SMA6101_5F_EQ_CTRL32		0x5f
#define	 SMA6101_60_EQ_CTRL33		0x60
#define	 SMA6101_61_EQ_CTRL34		0x61
#define	 SMA6101_62_EQ_CTRL35		0x62
#define	 SMA6101_63_EQ_CTRL36		0x63
#define	 SMA6101_64_EQ_CTRL37		0x64
#define	 SMA6101_65_EQ_CTRL38		0x65
#define	 SMA6101_66_EQ_CTRL39		0x66
#define	 SMA6101_67_EQ_CTRL40		0x67
#define	 SMA6101_68_EQ_CTRL41		0x68
#define	 SMA6101_69_EQ_CTRL42		0x69
#define	 SMA6101_6A_EQ_CTRL43		0x6a
#define	 SMA6101_6B_EQ_CTRL44		0x6b
#define	 SMA6101_6C_EQ_CTRL45		0x6c
/* Band4 */
#define	 SMA6101_6D_EQ_CTRL46		0x6d
#define	 SMA6101_6E_EQ_CTRL47		0x6e
#define	 SMA6101_6F_EQ_CTRL48		0x6f
#define	 SMA6101_70_EQ_CTRL49		0x70
#define	 SMA6101_71_EQ_CTRL50		0x71
#define	 SMA6101_72_EQ_CTRL51		0x72
#define	 SMA6101_73_EQ_CTRL52		0x73
#define	 SMA6101_74_EQ_CTRL53		0x74
#define	 SMA6101_75_EQ_CTRL54		0x75
#define	 SMA6101_76_EQ_CTRL55		0x76
#define	 SMA6101_77_EQ_CTRL56		0x77
#define	 SMA6101_78_EQ_CTRL57		0x78
#define	 SMA6101_79_EQ_CTRL58		0x79
#define	 SMA6101_7A_EQ_CTRL59		0x7a
#define	 SMA6101_7B_EQ_CTRL60		0x7b
/* Band5 */
#define	 SMA6101_7C_EQ_CTRL61		0x7c
#define	 SMA6101_7D_EQ_CTRL62		0x7d
#define	 SMA6101_7E_EQ_CTRL63		0x7e
#define	 SMA6101_7F_EQ_CTRL64		0x7f
#define	 SMA6101_80_EQ_CTRL65		0x80
#define	 SMA6101_81_EQ_CTRL66		0x81
#define	 SMA6101_82_EQ_CTRL67		0x82
#define	 SMA6101_83_EQ_CTRL68		0x83
#define	 SMA6101_84_EQ_CTRL69		0x84
#define	 SMA6101_85_EQ_CTRL70		0x85
#define	 SMA6101_86_EQ_CTRL71		0x86
#define	 SMA6101_87_EQ_CTRL72		0x87
#define	 SMA6101_88_EQ_CTRL73		0x88
#define	 SMA6101_89_EQ_CTRL74		0x89
#define	 SMA6101_8A_EQ_CTRL75		0x8a
#define	 SMA6101_8B_PLL_POST_N		0x8b
#define	 SMA6101_8C_PLL_N		    0x8c
#define	 SMA6101_8D_PLL_F1		    0x8d
#define	 SMA6101_8E_PLL_F2		    0x8e
#define	 SMA6101_8F_PLL_F3		    0x8f
#define  SMA6101_90_CLASS_H_CTRL_LVL1   0x90
#define  SMA6101_91_CLASS_H_CTRL_LVL2   0x91
#define  SMA6101_92_FDPEC_CTRL2		0x92
#define  SMA6101_93_BOOST_CTRL0     0x93
#define  SMA6101_94_BOOST_CTRL1     0x94
#define  SMA6101_95_BOOST_CTRL2     0x95
#define  SMA6101_96_BOOST_CTRL3     0x96
#define  SMA6101_97_BOOST_CTRL4     0x97
/* 0x98 ~ 0xA1 : Reserved */
#define	 SMA6101_A2_TOP_MAN1		0xa2
#define	 SMA6101_A3_TOP_MAN2		0xa3
#define	 SMA6101_A4_SDO_OUT_FMT		0xa4
#define	 SMA6101_A5_TDM_TX1		    0xa5
#define	 SMA6101_A6_TDM_TX2			0xa6
#define	 SMA6101_A7_TOP_MAN3		0xa7
#define	 SMA6101_A8_TONE_GENERATOR	0xa8
#define	 SMA6101_A9_TONE_FINE_VOL	0xa9
#define	 SMA6101_AA_PLL_A_SETTING	0xaa
#define	 SMA6101_AB_PLL_D_SETTING	0xab
#define	 SMA6101_AC_PLL_MODE_CTRL	0xac
#define	 SMA6101_AD_SPK_OCP_LVL		0xad
#define	 SMA6101_AE_TOP_MAN4		0xae
#define	 SMA6101_AF_VIN_SENSING		0xaf
#define	 SMA6101_B0_BROWN_OUT_P0	0xb0
#define	 SMA6101_B1_BROWN_OUT_P1	0xb1
#define	 SMA6101_B2_BROWN_OUT_P2	0xb2
#define	 SMA6101_B3_BROWN_OUT_P3	0xb3
#define	 SMA6101_B4_BROWN_OUT_P4	0xb4
#define	 SMA6101_B5_BROWN_OUT_P5	0xb5
#define	 SMA6101_B6_BROWN_OUT_P6	0xb6
#define	 SMA6101_B7_BROWN_OUT_P7	0xb7
#define	 SMA6101_B8_BROWN_OUT_P8	0xb8
#define	 SMA6101_B9_BROWN_OUT_P9	0xb9
#define	 SMA6101_BA_BROWN_OUT_P10	0xba
#define	 SMA6101_BB_BROWN_OUT_P11	0xbb
#define	 SMA6101_BC_BROWN_OUT_P12	0xbc
#define	 SMA6101_BD_BROWN_OUT_P13	0xbd
#define	 SMA6101_BE_BROWN_OUT_P14	0xbe
#define	 SMA6101_BF_BROWN_OUT_P15	0xbf
/* 0xC0 ~ 0xEF : Reserved */
/* Status Register(Read Only) */
#define	 SMA6101_FA_STATUS1		    0xfa
#define	 SMA6101_FB_STATUS2		    0xfb
#define	 SMA6101_FC_STATUS3		    0xfc
#define	 SMA6101_FD_STATUS4		    0xfd
#define	 SMA6101_FE_STATUS5		    0xfe
#define	 SMA6101_FF_VERSION	        0xff

/* SMA6101 Registers Bit Fields */

/* SYSTEM_CTRL : 0x00 */
#define POWER_MASK (1<<0)
#define POWER_ON (1<<0)
#define POWER_OFF (0<<0)

#define CLKSYSTEM_MASK	(7<<5)
#define EXT_19_2	(3<<5)
#define EXT_24_576	(4<<5)

/* INTPUT CTRL1 : 0x01 */
#define MASTER_SLAVE_MASK (1<<7)
#define SLAVE_MODE	(0<<7)
#define MASTER_MODE	(1<<7)

#define I2S_MODE_MASK	(7<<4)
#define STANDARD_I2S	(0<<4)
#define LJ		(1<<4)
#define RJ_16BIT	(4<<4)
#define RJ_18BIT	(5<<4)
#define RJ_20BIT	(6<<4)
#define RJ_24BIT	(7<<4)

#define LEFTPOL_MASK	(1<<3)
#define LOW_FIRST_CH	(0<<3)
#define HIGH_FIRST_CH	(1<<3)

#define SCK_RISING_FALLING_MASK	(1<<2)
#define FALLING_EDGE		(0<<2)
#define RISING_EDGE		(1<<2)

/* INTPUT CTRL2 : 0x02 */
#define INPUT_MODE_MASK (3<<6)
#define I2S	(0<<6)
#define PCM_SHORT (1<<6)
#define PCM_LONG (2<<6)

#define RIGHT_FIRST_MASK (1<<5)
#define LEFT_NORMAL (0<<5)
#define RIGHT_INVERTED (1<<5)

#define PCM_LAW_MASK (1<<4)
#define PCM_U_DECODING (0<<4)
#define PCM_A_DECODING (1<<4)

#define PCM_COMP_MASK (1<<3)
#define PCM_LINEAR (0<<3)
#define PCM_COMPANDING (1<<3)

#define PCM_FS_MASK (1<<2)
#define PCM_8KHZ (0<<2)
#define PCM_16KHZ (1<<2)

#define PCM_STEREO_MASK (1<<1)
#define PCM_MONO (0<<1)
#define PCM_STEREO (1<<1)

#define PCM_DL_MASK (1<<0)
#define PCM_8BIT (0<<0)
#define PCM_16BIT (1<<0)

/* INTPUT CTRL3 : 0x03 */
#define BP_SRC_MASK (1<<4)
#define BP_SRC_NORMAL (0<<4)
#define BP_SRC_BYPASS (1<<4)

#define PCM_N_SLOT_MASK (15<<0)
#define PCM_N_SLOT_1 (0<<0)
#define PCM_N_SLOT_2 (1<<0)
#define PCM_N_SLOT_3 (2<<0)
#define PCM_N_SLOT_4 (3<<0)
#define PCM_N_SLOT_5 (4<<0)
#define PCM_N_SLOT_6 (5<<0)
#define PCM_N_SLOT_7 (6<<0)
#define PCM_N_SLOT_8 (7<<0)
#define PCM_N_SLOT_9 (8<<0)
#define PCM_N_SLOT_10 (9<<0)
#define PCM_N_SLOT_11 (10<<0)
#define PCM_N_SLOT_12 (11<<0)
#define PCM_N_SLOT_13 (12<<0)
#define PCM_N_SLOT_14 (13<<0)
#define PCM_N_SLOT_15 (14<<0)
#define PCM_N_SLOT_16 (15<<0)

/* INTPUT CTRL4 : 0x04 */
#define PCM_SLOT_MASK (15<<4)
#define PCM_SLOT1 (0<<4)
#define PCM_SLOT2 (1<<4)
#define PCM_SLOT3 (2<<4)
#define PCM_SLOT4 (3<<4)
#define PCM_SLOT5 (4<<4)
#define PCM_SLOT6 (5<<4)
#define PCM_SLOT7 (6<<4)
#define PCM_SLOT8 (7<<4)
#define PCM_SLOT9 (8<<4)
#define PCM_SLOT10 (9<<4)
#define PCM_SLOT11 (10<<4)
#define PCM_SLOT12 (11<<4)
#define PCM_SLOT13 (12<<4)
#define PCM_SLOT14 (13<<4)
#define PCM_SLOT15 (14<<4)
#define PCM_SLOT16 (15<<4)

/* OUTPUT CTRL : 0x09 */
#define PORT_CONFIG_MASK (3<<5)
#define INPUT_PORT_ONLY (0<<5)
#define OUTPUT_PORT_ENABLE (2<<5)

#define PORT_OUT_FORMAT_MASK (3<<3)
#define I2S_32SCK (0<<3)
#define I2S_64SCK (1<<3)
#define PCM_SHORT_128FS (2<<3)

#define PORT_OUT_SEL_MASK (7<<0)
#define DISABLE (0<<0)
#define FORMAT_CONVERTER (1<<0)
#define MIXER_OUTPUT (2<<0)
#define SPEAKER_PATH (3<<0)
#define MOD_IN_TONE_OUT (4<<0)

/* MUTE_VOL_CTRL : 0x0E */
#define VOL_SLOPE_MASK (3<<6)
#define VOL_SLOPE_OFF (0<<6)
#define VOL_SLOPE_SLOW (1<<6)
#define VOL_SLOPE_MID (2<<6)
#define VOL_SLOPE_FAST (3<<6)

#define MUTE_SLOPE_MASK (3<<4)
#define MUTE_SLOPE_OFF (0<<4)
#define MUTE_SLOPE_SLOW (1<<4)
#define MUTE_SLOPE_MID (2<<4)
#define MUTE_SLOPE_FAST (3<<4)

#define SPK_MUTE_MASK (1<<0)
#define SPK_MUTE (1<<0)
#define SPK_UNMUTE (0<<0)

/* SYSTEM_CTRL1 :0x10 */
#define SPK_MODE_MASK (7<<2)
#define SPK_OFF (0<<2)
#define SPK_MONO (1<<2)
#define SPK_STEREO (4<<2)

/* SYSTEM_CTRL2 : 0x11 */
#define SPK_EQ_MASK (1<<7)
#define SPK_EQ_BYP (0<<7)
#define SPK_EQ_EN (1<<7)
#define SPK_BS_MASK (1<<6)
#define SPK_BS_BYP (0<<6)
#define SPK_BS_EN (1<<6)
#define SPK_LIM_MASK (1<<5)
#define SPK_LIM_BYP (0<<5)
#define SPK_LIM_EN (1<<5)

#define LR_DATA_SW_MASK (1<<4)
#define LR_DATA_SW_NORMAL (0<<4)
#define LR_DATA_SW_SWAP (1<<4)

#define MONOMIX_MASK (1<<0)
#define MONOMIX_OFF (0<<0)
#define MONOMIX_ON (1<<0)

/* SYSTEM_CTRL3 : 0x12 */
#define INPUT_MASK (3<<6)
#define INPUT_0_DB (0<<6)
#define INPUT_M6_DB (1<<6)
#define INPUT_M12_DB (2<<6)
#define INPUT_INFI_DB (3<<6)
#define INPUT_R_MASK (3<<4)
#define INPUT_R_0_DB (0<<4)
#define INPUT_R_M6_DB (1<<4)
#define INPUT_R_M12_DB (2<<4)
#define INPUT_R_INFI_DB (3<<4)

/* FDPEC CONTROL1 : 0x13 */
#define DIS_SDM_SYNC_MASK (1<<5)
#define DIS_SDM_SYNC_NORMAL (0<<5)
#define DIS_SDM_SYNC_DISABLE (1<<5)

#define EN_FDPEC_CL_MASK (1<<0)
#define EN_FDPEC_CL_DISABLE (0<<0)
#define EN_FDPEC_CL_ENABLE (1<<0)

#define FDPEC_GAIN_MASK (1<<4)
#define FDPEC_GAIN_8 (0<<4)
#define FDPEC_GAIN_6 (1<<4)
#define FDPEC_EXT_GAIN_MASK (1<<1)
#define FDPEC_EXT_GAIN_6 (0<<1)
#define FDPEC_EXT_GAIN_4 (1<<1)
#define FDPEC_EXT_GAIN2_MASK (1<<0)
#define FDPEC_EXT_GAIN2_6 (0<<0)
#define FDPEC_EXT_GAIN2_2P6 (1<<0)

/* Modulator : 0x14 */
#define SPK_HYSFB_MASK (3<<6)
#define HYSFB_625K (0<<6)
#define HYSFB_414K (1<<6)
#define HYSFB_297K (2<<6)
#define HYSFB_226K (3<<6)
#define SPK_BDELAY_MASK (63<<0)

/* EQ_MODE : 0x2B */
#define EQ_MODE_MASK (7<<0)
#define USER_DEFINED (0<<0)
#define CLASSIC (1<<1)
#define ROCK_POP (2<<0)
#define JAZZ (3<<0)
#define RNB (4<<0)
#define DANCE (5<<0)
#define SPEECH (6<<0)
#define PARAMETRIC (7<<0)

/* SDM CONTROL : 0x33 */
#define SDM_VLINK_DIS_MASK (1<<3)
#define VLINK_ENABLE (0<<3)
#define VLINK_DISABLE (1<<3)

#define SDM_Q_SEL_MASK (1<<2)
#define QUART_SEL_1_DIV_4 (0<<2)
#define QUART_SEL_1_DIV_8 (1<<2)

/* PROTECTION : 0x36 */
#define EDGE_DIS_MASK (1<<7)
#define EDGE_DIS_ENABLE (0<<7)
#define EDGE_DIS_DISABLE (1<<7)

#define JITTER_DIS_MASK (1<<4)
#define SRC_JITTER_ADD (0<<4)
#define SRC_JITTER_DISABLE (1<<4)

#define SPK_OCP_DIS_MASK (1<<3)
#define SPK_OCP_ENABLE (0<<3)
#define SPK_OCP_DISABLE (1<<3)

#define OCP_MODE_MASK (1<<2)
#define AUTO_RECOVER (0<<2)
#define SHUT_DOWN_PERMANENT (1<<2)

#define OTP_MODE_MASK (3<<0)
#define OTP_MODE_DISABLE (0<<0)
#define IG_THR1_SHUT_THR2 (1<<0)
#define REC_THR1_SHUT_THR2 (2<<0)
#define SHUT_THR1_SHUT_THR2 (3<<0)

/* TEST2 : 0x3C */
#define SPK_HSDM_BP_MASK (1<<4)
#define SPK_HSDM_ENABLE (0<<4)
#define SPK_HSDM_BYPASS (1<<4)

#define DIS_SDM_SYNC_TEST_MASK (1<<5)
#define DIS_SDM_SYNC_TEST_NORMAL (0<<5)
#define DIS_SDM_SYNC_TEST_DISABLE (1<<5)

/* ATEST2 : 0x3F */
#define THERMAL_ADJUST_MASK (3<<5)
#define THERMAL_150_110 (0<<5)
#define THERMAL_160_120 (1<<5)
#define THERMAL_140_100 (2<<5)

/* CLASS-H CONTROL LEVEL2 : 0x91 */
#define CLASS_H_ATTACK_LVL_MASK (15<<4)

#define CLASS_H_RELEASE_TIME_MASK (15<<0)
#define CLASS_H_RELEASE_TIME_0 (0<<0)
#define CLASS_H_RELEASE_TIME_20 (1<<0)
#define CLASS_H_RELEASE_TIME_40 (2<<0)
#define CLASS_H_RELEASE_TIME_60 (3<<0)
#define CLASS_H_RELEASE_TIME_80 (4<<0)
#define CLASS_H_RELEASE_TIME_100 (5<<0)
#define CLASS_H_RELEASE_TIME_120 (6<<0)
#define CLASS_H_RELEASE_TIME_140 (7<<0)
#define CLASS_H_RELEASE_TIME_160 (8<<0)
#define CLASS_H_RELEASE_TIME_180 (9<<0)
#define CLASS_H_RELEASE_TIME_200 (10<<0)
#define CLASS_H_RELEASE_TIME_220 (11<<0)
#define CLASS_H_RELEASE_TIME_240 (12<<0)
#define CLASS_H_RELEASE_TIME_260 (13<<0)
#define CLASS_H_RELEASE_TIME_280 (14<<0)
#define CLASS_H_RELEASE_TIME_300 (15<<0)

/* FDPEC CONTROL2 : 0x92 */
#define PWMLS_I_MASK (3<<1)
#define PWMLS_I_40U (0<<1)
#define PWMLS_I_80U (1<<1)
#define PWMLS_I_120U (2<<1)
#define PWMLS_I_160U (3<<1)

#define REC_CUR_MODE_MASK (1<<5)
#define REC_CUR_MODE_ENHANCED (0<<5)
#define REC_CUR_MODE_NORMAL (1<<5)

#define REC_CUR_CTRL_MASK (1<<4)
#define REC_CUR_CTRL_ENABLE (0<<4)
#define REC_CUR_CTRL_DISABLE (1<<4)

/* BOOST CONTROL0 : 0x93 */
#define TRM_VBST1_MASK (15<<0)
#define TRM_VBST1_6V (0<<0)
#define TRM_VBST1_7V (1<<0)
#define TRM_VBST1_8V (2<<0)
#define TRM_VBST1_9V (3<<0)
#define TRM_VBST1_10V (4<<0)
#define TRM_VBST1_11V (5<<0)
#define TRM_VBST1_12V (6<<0)
#define TRM_VBST1_13V (7<<0)
#define TRM_VBST1_14V (8<<0)
#define TRM_VBST1_15V (9<<0)
#define TRM_VBST1_16V (10<<0)
#define TRM_VBST1_17V (11<<0)
#define TRM_VBST1_18V (12<<0)
#define TRM_VBST1_19V (13<<0)
#define TRM_VBST1_20V (14<<0)
#define TRM_VBST1_21V (15<<0)

#define TRM_VREF_MASK (15<<4)
#define TRM_VREF_0_IDX (0<<4)
#define TRM_VREF_1_IDX (1<<4)
#define TRM_VREF_2_IDX (2<<4)
#define TRM_VREF_3_IDX (3<<4)
#define TRM_VREF_4_IDX (4<<4)
#define TRM_VREF_5_IDX (5<<4)
#define TRM_VREF_6_IDX (6<<4)
#define TRM_VREF_7_IDX (7<<4)
#define TRM_VREF_8_IDX (8<<4)
#define TRM_VREF_9_IDX (9<<4)
#define TRM_VREF_A_IDX (10<<4)
#define TRM_VREF_B_IDX (11<<4)
#define TRM_VREF_C_IDX (12<<4)
#define TRM_VREF_D_IDX (13<<4)
#define TRM_VREF_E_IDX (14<<4)
#define TRM_VREF_F_IDX (15<<4)

/* TOP_MAN1 : 0xA2 */
#define PLL_LOCK_SKIP_MASK (1<<7)
#define PLL_LOCK_ENABLE (0<<7)
#define PLL_LOCK_DISABLE (1<<7)

#define PLL_PD_MASK (1<<6)
#define PLL_OPERATION (0<<6)
#define PLL_PD (1<<6)

#define MCLK_SEL_MASK (1<<5)
#define PLL_CLK (0<<5)
#define EXTERNAL_CLK (1<<5)

#define PLL_REF_CLK1_MASK (1<<4)
#define REF_EXTERNAL_CLK (0<<4)
#define REF_INTERNAL_OSC (1<<4)

#define PLL_REF_CLK2_MASK (1<<3)
#define PLL_REF_CLK1 (0<<3)
#define PLL_SCK (1<<3)

#define DAC_DN_CONV_MASK (1<<2)
#define DAC_DN_CONV_DISABLE (0<<2)
#define DAC_DN_CONV_ENABLE (1<<2)

#define SDO_IO_MASK (1<<1)
#define HIGH_Z_LRCK_H (0<<1)
#define HIGH_Z_LRCK_L (1<<1)

#define SDO_DATA_MASK (1<<0)
#define SDO_DATA_DAC (0<<0)
#define DIS_SDO_DATA_DAC (1<<0)

/* TOP_MAN2 : 0xA3 */
#define MON_OSC_PLL_MASK (1<<7)
#define PLL_SDO (0<<7)
#define PLL_OSC (1<<7)

#define TEST_CLKO_EN_MASK (1<<6)
#define NORMAL_SDO (0<<6)
#define CLK_OUT_SDO (1<<6)

#define PLL_SDM_PD_MASK (1<<5)
#define SDM_ON (0<<5)
#define SDM_OFF (1<<5)

#define SDO_OUTPUT_MASK (1<<3)
#define NORMAL_OUT (0<<3)
#define HIGH_Z_OUT (1<<3)

#define CLOCK_MON_MASK (1<<1)
#define CLOCK_MON (0<<1)
#define CLOCK_NOT_MON (1<<1)

#define OSC_PD_MASK (1<<0)
#define NORMAL_OPERATION_OSC (0<<0)
#define POWER_DOWN_OSC (1<<0)

/* SDO OUTPUT FORMAT : 0xA4 */
#define O_FORMAT_MASK (7<<5)
#define O_FORMAT_LJ (1<<5)
#define O_FORMAT_I2S (2<<5)
#define O_FORMAT_TDM (4<<5)

#define SCK_RATE_MASK (3<<3)
#define SCK_RATE_64FS (0<<3)
#define SCK_RATE_32FS (2<<3)

#define WD_LENGTH_MASK (3<<1)
#define WL_24BIT (0<<1)
#define WL_20BIT (1<<1)
#define WL_16BIT (2<<1)

/* TDM TX1 FORMAT : 0xA5 */
#define TDM_N_SLOT_MASK (3<<2)
#define TDM_N_SLOT_2 (0<<2)
#define TDM_N_SLOT_4 (1<<2)
#define TDM_N_SLOT_8 (2<<2)
#define TDM_N_SLOT_16 (3<<2)

/* TDM TX2 FORMAT : 0xA6 */
#define TDM_TX_SLOT_MASK (15<<4)
#define TDM_TX_SLOT1 (0<<4)
#define TDM_TX_SLOT2 (1<<4)
#define TDM_TX_SLOT3 (2<<4)
#define TDM_TX_SLOT4 (3<<4)
#define TDM_TX_SLOT5 (4<<4)
#define TDM_TX_SLOT6 (5<<4)
#define TDM_TX_SLOT7 (6<<4)
#define TDM_TX_SLOT8 (7<<4)
#define TDM_TX_SLOT9 (8<<4)
#define TDM_TX_SLOT10 (9<<4)
#define TDM_TX_SLOT11 (10<<4)
#define TDM_TX_SLOT12 (11<<4)
#define TDM_TX_SLOT13 (12<<4)
#define TDM_TX_SLOT14 (13<<4)
#define TDM_TX_SLOT15 (14<<4)
#define TDM_TX_SLOT16 (15<<4)

/* TOP_MAN3 : 0xA7 */
#define CLOCK_MON_SEL_MASK (1<<5)
#define CLOCK_MON_SCK (0<<5)
#define CLOCK_MON_EXTERNAL (1<<5)

#define MAS_EN_MASK (1<<0)
#define MAS_EN_SLAVE (0<<0)
#define MAS_EN_MASTER (1<<0)

/* TONE GENERATOR : 0xA8 */
#define TONE_ON_MASK (1<<0)
#define TONE_OFF (0<<0)
#define TONE_ON (1<<0)

#define TONE_FREQ_MASK (15<<1)
#define TONE_FREQ_50 (0<<1)
#define TONE_FREQ_60 (1<<1)
#define TONE_FREQ_140 (2<<1)
#define TONE_FREQ_150 (3<<1)
#define TONE_FREQ_175 (4<<1)
#define TONE_FREQ_180 (5<<1)
#define TONE_FREQ_200 (6<<1)
#define TONE_FREQ_375 (7<<1)
#define TONE_FREQ_750 (8<<1)
#define TONE_FREQ_1P5K (9<<1)
#define TONE_FREQ_3K (10<<1)
#define TONE_FREQ_6K (11<<1)
#define TONE_FREQ_8K (12<<1)
#define TONE_FREQ_12K (13<<1)
#define TONE_FREQ_1K (14<<1)

/* TONE/FINE VOLUME : 0xA9 */
#define TONE_VOL_MASK (7<<0)
#define TONE_VOL_6 (0<<0)
#define TONE_VOL_0 (1<<0)
#define TONE_VOL_M_6 (2<<0)
#define TONE_VOL_M_12 (3<<0)
#define TONE_VOL_M_18 (4<<0)
#define TONE_VOL_M_24 (5<<0)
#define TONE_VOL_M_30 (6<<0)
#define TONE_VOL_OFF (7<<0)

/* PLL_MODE_CTRL : 0xAC */
#define PLL_LDO_PD_MASK (7<<3)
#define PLL_LDO_POWER_DOWN (5<<3)
#define PLL_LDO_POWER_ON (6<<3)

#define PLL_LDO_BYP_MASK (7<<0)
#define PLL_LDO_BYP_ENABLE (0<<0)
#define PLL_LDO_BYP_DISABLE (7<<0)

/* TOP_MAN4 : 0xAE */
#define SDO_LRCK_MASK (1<<7)
#define SDO_LRCK_HIGH_VALID (0<<7)
#define SDO_LRCK_LOW_VALID (1<<7)

#define DIS_IRQ_MASK (1<<6)
#define NORMAL_OPERATION_IRQ (0<<6)
#define HIGH_Z_IRQ (1<<6)

/* STATUS1 : 0xFA */
#define OT1_OK_STATUS (1<<7)
#define OT2_OK_STATUS (1<<6)
#define REV_NUM_STATUS (7<<3)
#define REV_NUM_REV0 (0<<3)
#define REV_NUM_REV1 (1<<3)
#define REV_NUM_REV2 (2<<3)
#define REV_NUM_REV3 (3<<3)
#define REV_NUM_REV4 (4<<3)

/* STATUS2 : 0xFB */
#define OCP_SPK_STATUS (1<<5)
#define OCP_BST_STATUS (1<<4)
#define UVLO_BST_STATUS (1<<3)
#define CLOCK_MON_STATUS (1<<0)

/* VERSION : 0xFF */
#define CHIP_VERSION (11<<0)

#endif
