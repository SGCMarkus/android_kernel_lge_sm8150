/* Copyright (c) 2015-2016, 2018, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/fs.h>

#include <dsp/q6afe-v2.h>
#include <dsp/smart_amp.h>

//TODO: Move below to smart_amp.h header file
/*Re-calibration*/
#define SMARTAMP_SPEAKER_CALIBDATA_FILE  "/mnt/vendor/persist-lg/audio/smartamp_calib.bin"
#define MAX_STRING		    	200

/*Master Control to Bypass the Smartamp TI CAPIv2 module*/
static int smartamp_bypass = TAS_FALSE;
static int smartamp_enable = TAS_FALSE;
static int calib_state = 0;
static int calib_re = 0;
static int spk_id = 0;
static struct mutex routing_lock;

int get_calibrated_re_tv(uint32_t *Rdc_fix, uint32_t *Tv_fix)
{
	struct file *pF = NULL;
	loff_t pos = 0; 	
	char calib_data[MAX_STRING] = {0};
	mm_segment_t fs;
	int ret = 0;

	fs = get_fs();
	set_fs(get_ds());
	pF = filp_open(SMARTAMP_SPEAKER_CALIBDATA_FILE , O_RDWR, 0666);
	if(!IS_ERR(pF))
	{
		vfs_read(pF, calib_data, MAX_STRING-1, &pos);
		if(sscanf(calib_data, "%d;%d;", Rdc_fix, Tv_fix) != 2)
		{
			pr_err("[Smartamp:%s] file %s read error\n", __func__, SMARTAMP_SPEAKER_CALIBDATA_FILE);
			ret = -1;				
		}
		filp_close(pF, NULL);
	}
	else
	{
		pr_err("[Smartamp:%s] file %s open failed\n", __func__, SMARTAMP_SPEAKER_CALIBDATA_FILE);
		ret = -1;
	}
	set_fs(fs);
	return ret;
}

int smartamp_get_val(int value)
{
	return ((value == TAS_TRUE)?TRUE:FALSE);
}

int smartamp_set_val(int value)
{
	return ((value == TRUE)?TAS_TRUE:TAS_FALSE);
}

int afe_smartamp_get_set(u8 *user_data, uint32_t param_id,
	uint8_t get_set, uint32_t length, uint32_t module_id)
{
	int ret = 0;
	struct afe_smartamp_get_calib calib_resp;

	switch (get_set) {
		case TAS_SET_PARAM:
			if(smartamp_get_val(smartamp_bypass))
			{
				pr_err("[Smartamp:%s] SmartAmp is bypassed no control set", __func__);
				goto fail_cmd;
			}
			ret = afe_smartamp_set_calib_data(param_id, 
				(struct afe_smartamp_set_params_t *)user_data, length, module_id);
		break;	
		case TAS_GET_PARAM:
			memset(&calib_resp, 0, sizeof(calib_resp));
			ret = afe_smartamp_get_calib_data(&calib_resp,
				param_id, module_id);
			memcpy(user_data, calib_resp.res_cfg.payload, length);
		break;
		default:
			goto fail_cmd;
	}
fail_cmd:
	return ret;
}

/*Wrapper arround set/get parameter, all set/get commands pass through this wrapper*/
int afe_smartamp_algo_ctrl(u8 *user_data, uint32_t param_id,
	uint8_t get_set, uint32_t length, uint32_t module_id)
{
	int ret = 0;
	mutex_lock(&routing_lock);
	ret = afe_smartamp_get_set(user_data, param_id, get_set, length, module_id);
	mutex_unlock(&routing_lock);
	return ret;
}

/*Control-1: Set Speaker ID*/
static int tas2562_set_spkid_left(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *pUcontrol)
{
	int ret;
	int user_data = pUcontrol->value.integer.value[0];
	int param_id = 0;
	
	spk_id = user_data;
	pr_info("[Smartamp:%s] Setting Speaker ID %d", __func__, user_data);
	
	param_id = TAS_CALC_PARAM_IDX(TAS_SA_SET_SPKID, 1, CHANNEL0);
	ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
		TAS_SET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
	if(ret < 0)
	{
		pr_err("[Smartamp:%s] Failed to set Speaker ID", __func__);
		return -1;
	}

	return 0;
}

static int tas2562_get_spkid_left(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
	int user_data = 0;

	user_data = spk_id;
	pUcontrol->value.integer.value[0] = user_data;
	pr_info("[Smartamp:%s] Getting Speaker ID %d", __func__, user_data);

	return 0;
}

/*Control-2: Set Profile*/
static const char *profile_index_text[] = {"NONE","MUSIC","RING","VOICE","COMBO","CALIB"};
static const struct soc_enum profile_index_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(profile_index_text), profile_index_text),
};

static int tas2562_set_profile(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *pUcontrol)
{
	int ret;
	int user_data = pUcontrol->value.integer.value[0];
	int param_id = 0;		

	if (user_data < 0 || user_data >= ARRAY_SIZE(profile_index_text)) {
		pr_err("%s: [Smartamp] user_data %d is not valid\n", __func__, user_data);
		return -EINVAL;
	}

	pr_info("[Smartamp:%s] Setting profile %s", __func__, profile_index_text[user_data]);
	//To exclude None
	if(user_data)
		user_data -= 1;
	else
		return 0;	

	param_id = TAS_CALC_PARAM_IDX(TAS_SA_SET_PROFILE, 1, CHANNEL0);
	ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
		TAS_SET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
	if(ret < 0)
	{
		pr_err("[Smartamp:%s] Failed to set profile", __func__);
		return -1;
	}
	if(smartamp_get_val(smartamp_enable))
	{
		pr_info("[Smartamp:%s] Sending Rx Config", __func__);
		param_id = CAPI_V2_TAS_RX_CFG;
		ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
		TAS_SET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
		if(ret < 0)
			pr_err("[Smartamp:%s] Failed to set Rx Config", __func__);	
	}
	return 0;
}

static int tas2562_get_profile(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *pUcontrol)
{
	int ret;
	int user_data = 0;
	int param_id = 0;	

	if(smartamp_get_val(smartamp_enable))
	{
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_SET_PROFILE, 1, CHANNEL0);
		ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
			TAS_GET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
		if(ret < 0)
		{
			pr_err("[Smartamp:%s] Failed to get profile", __func__);
			user_data = 0;
		}
		else
			user_data += 1;
	}	
	pUcontrol->value.integer.value[0] = user_data;
	pr_info("[Smartamp:%s] getting profile %s", __func__, profile_index_text[user_data]);

	return 0;
}

/*Control-3: Calibration and Test(F0,Q,Tv) Controls*/
static const char *tas2562_calib_test_text[] = {
	"NONE",
	"CALIB_START",
	"CALIB_STOP",
	"TEST_START",
	"TEST_STOP"
};

static const struct soc_enum tas2562_calib_test_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tas2562_calib_test_text), tas2562_calib_test_text),
};

static int tas2562_calib_test_set_left(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	int param_id = 0;
	int user_data = pUcontrol->value.integer.value[0];
	int data = 1;

	if (user_data < 0 || user_data >= ARRAY_SIZE(tas2562_calib_test_text)) {
		pr_err("%s: [Smartamp] user_data %d is not valid\n", __func__, user_data);
		return -EINVAL;
	}

	calib_state = user_data;
	pr_info("[Smartamp:%s] case %s", __func__, tas2562_calib_test_text[user_data]);

	switch(user_data) {
		case CALIB_START:
			param_id = TAS_CALC_PARAM_IDX(TAS_SA_CALIB_INIT, 1, CHANNEL0);		
		break;
		case CALIB_STOP:
			param_id = TAS_CALC_PARAM_IDX(TAS_SA_CALIB_DEINIT, 1, CHANNEL0);
		break;
		case TEST_START:
			pr_info("[Smartamp:%s] Not Required!", __func__);
		break;
		case TEST_STOP:
			pr_info("[Smartamp:%s] Not Required!", __func__);
		break;
		default:
			pr_err("[Smartamp:%s] Invalid", __func__);
		break;		
	}
	
	if(param_id)
	{
		ret = afe_smartamp_algo_ctrl((u8 *)&data, param_id,
			TAS_SET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
		if(ret < 0)
		{
			pr_err("[Smartamp:%s] Failed to set calib/test", __func__);
			return 1;
		}
	}

	return ret;			
}

static int tas2562_calib_test_get_left(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	pUcontrol->value.integer.value[0] = calib_state;
	pr_info("[Smartamp:%s] case %s", __func__, tas2562_calib_test_text[calib_state]);
	return ret;
}

/*Control-4: Get Re*/
static int tas2562_get_re_left(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
	int ret;
	int user_data = 0;
	int param_id = 0;	

	if(smartamp_get_val(smartamp_enable))
	{
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_GET_RE, 1, CHANNEL0);
		ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
			TAS_GET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
		if(ret < 0)
		{
			pr_err("[Smartamp:%s] Failed to get Re", __func__);
			return -1;
		}
	}
		
	pUcontrol->value.integer.value[0] = user_data;
	pr_info("[Smartamp:%s] Getting Re %d", __func__, user_data);

	return 0;
}

/*Control-5: Get F0*/
static int tas2562_get_f0_left(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
	int ret;
	int user_data = 0;
	int param_id = 0;	

	if(smartamp_get_val(smartamp_enable))
	{
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_GET_F0, 1, CHANNEL0);
		ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
			TAS_GET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
		if(ret < 0)
		{
			pr_err("[Smartamp:%s] Failed to get F0", __func__);
			return -1;
		}
	}	
	pUcontrol->value.integer.value[0] = user_data;
	pr_info("[Smartamp:%s] Getting F0 %d", __func__, user_data);

	return 0;
}

/*Control-6: Get Q*/
static int tas2562_get_q_left(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
	int ret;
	int user_data = 0;
	int param_id = 0;	

	if(smartamp_get_val(smartamp_enable))
	{
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_GET_Q, 1, CHANNEL0);
		ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
			TAS_GET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
		if(ret < 0)
		{
			pr_err("[Smartamp:%s] Failed to get q", __func__);
			return -1;
		}
	}	
	pUcontrol->value.integer.value[0] = user_data;
	pr_info("[Smartamp:%s] Getting Q %d", __func__, user_data);

	return 0;
}

/*Control-7: Get Tv*/
static int tas2562_get_tv_left(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
	int ret;
	int user_data = 0;
	int param_id = 0;	

	if(smartamp_get_val(smartamp_enable))
	{
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_GET_TV, 1, CHANNEL0);
		ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
			TAS_GET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
		if(ret < 0)
		{
			pr_err("[Smartamp:%s] Failed to get Tv", __func__);
			return -1;
		}
	}	
	pUcontrol->value.integer.value[0] = user_data;
	pr_info("[Smartamp:%s] Getting Tv %d", __func__, user_data);

	return 0;
}

/*Control-8: Smartamp Enable*/
static const char *tas2562_smartamp_enable_text[] = {
	"DISABLE",
	"ENABLE"
};

static const struct soc_enum tas2562_smartamp_enable_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tas2562_smartamp_enable_text), tas2562_smartamp_enable_text),
};
static int tas2562_smartamp_enable_set(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	int param_id = 0;
	int user_data = pUcontrol->value.integer.value[0];
	uint32_t calib_tv = 0;	

	if (user_data < 0 || user_data >= ARRAY_SIZE(tas2562_smartamp_enable_text)) {
		pr_err("%s: [Smartamp] user_data %d is not valid\n", __func__, user_data);
		return -EINVAL;
	}

	pr_info("[Smartamp:%s] case %s", __func__, tas2562_smartamp_enable_text[user_data]);

	smartamp_enable = smartamp_set_val(user_data);
	if(smartamp_get_val(smartamp_enable) == FALSE)
	{
		pr_info("[Smartamp:%s] Disable called", __func__);
		calib_state = 0;
		calib_re = 0;
		return 0;
	}

	ret = get_calibrated_re_tv(&calib_re, &calib_tv);
	if(ret)
		pr_err("[Smartamp:%s] Get Calibrated Rdc failed = 0x%x", __func__, ret);
	else
	{
		pr_info("[Smartamp:%s] Setting Tcal %d", __func__, calib_tv);
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_SET_TCAL, 1, CHANNEL0);
		ret = afe_smartamp_algo_ctrl((u8 *)&calib_tv, param_id,
			TAS_SET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
		if(ret < 0)
			pr_err("[Smartamp:%s] Failed to set Tcal", __func__);
		
		
		pr_info("[Smartamp:%s] Setting Re %d", __func__, calib_re);
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_SET_RE, 1, CHANNEL0);
		ret = afe_smartamp_algo_ctrl((u8 *)&calib_re, param_id,
			TAS_SET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
		if(ret < 0)
			pr_err("[Smartamp:%s] Failed to set Re", __func__);
	}

	pr_info("[Smartamp:%s] Setting the feedback module info for TAS", __func__);		
	ret = afe_spk_prot_feed_back_cfg(TAS_TX_PORT,TAS_RX_PORT, 1, 0, 1);
	if(ret)
		pr_err("[Smartamp:%s] FB Path Info failed ignoring ret = 0x%x", __func__, ret);

	pr_info("[Smartamp:%s] Sending TX Enable", __func__);
	param_id = CAPI_V2_TAS_TX_ENABLE;
	ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
		TAS_SET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_TX);
	if(ret)
	{
		pr_err("[Smartamp:%s] TX Enable Failed ret = 0x%x", __func__, ret);
		goto fail_cmd;
	}
	
	pr_info("[Smartamp:%s] Sending RX Enable", __func__);
	param_id = CAPI_V2_TAS_RX_ENABLE;
	ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
		TAS_SET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
	if(ret)
	{
		pr_err("[Smartamp:%s] RX Enable Failed ret = 0x%x", __func__, ret);
		goto fail_cmd;	
	}

	pr_info("[Smartamp:%s] Sending RX Config", __func__);
	param_id = CAPI_V2_TAS_RX_CFG;
	ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
		TAS_SET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
	if(ret < 0)
		pr_err("[Smartamp:%s] Failed to set config", __func__);
fail_cmd:		
	return ret;
}

static int tas2562_smartamp_enable_get(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	int user_data = smartamp_get_val(smartamp_enable);
	pUcontrol->value.integer.value[0] = user_data;
	pr_info("[Smartamp:%s] case %s", __func__, tas2562_smartamp_enable_text[user_data]);
	return ret;
}

/*Control-9: Smartamp Bypass */
static const char *tas2562_smartamp_bypass_text[] = {
	"FALSE",
	"TRUE"
};

static const struct soc_enum tas2562_smartamp_bypass_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tas2562_smartamp_bypass_text), tas2562_smartamp_bypass_text),
};

static int tas2562_smartamp_bypass_set(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	int user_data = pUcontrol->value.integer.value[0];

	if (user_data < 0 || user_data >= ARRAY_SIZE(tas2562_smartamp_bypass_text)) {
		pr_err("%s: [Smartamp] user_data %d is not valid\n", __func__, user_data);
		return -EINVAL;
	}

	if(smartamp_get_val(smartamp_enable))
	{
		pr_info("[Smartamp:%s] cannot update while smartamp enabled", __func__);
		return ret;
	}	
	smartamp_bypass = smartamp_set_val(user_data);
	pr_info("[Smartamp:%s] case %s", __func__, tas2562_smartamp_bypass_text[user_data]);
	return ret;
}

static int tas2562_smartamp_bypass_get(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	int user_data = smartamp_get_val(smartamp_bypass);
	pUcontrol->value.integer.value[0] = user_data;
	pr_info("[Smartamp:%s] case %s", __func__, tas2562_smartamp_bypass_text[user_data]);
	return ret;
}

#if 0
/*Control-10: Smartamp Status*/
static const char *tas2562_smartamp_status_text[] = {
	"DISABLED",
	"ENABLED"
};

static const struct soc_enum tas2562_smartamp_status_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tas2562_smartamp_status_text), tas2562_smartamp_status_text),
};

static int tas2562_get_smartamp_status(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
	int ret;
	int user_data = 0;
	int param_id = 0;
	int data = 1;	

	param_id = TAS_CALC_PARAM_IDX(TAS_SA_GET_STATUS, 1, CHANNEL0);
	ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
		TAS_GET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
	if(ret < 0)
	{
		pr_err("[Smartamp:%s] Failed to get Status", __func__);
		user_data = 0;
	}
	else
		pr_info("[Smartamp:%s] status = %d", __func__, user_data);
	data &= user_data;
	user_data = 0;
	param_id = CAPI_V2_TAS_RX_ENABLE;
	ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
		TAS_GET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_RX);
	if(ret < 0)
	{
		pr_err("[SmartAmp:%s] Failed to get Rx Enable", __func__);
		user_data = 0;
	}else
		pr_info("[SmartAmp:%s] Rx Enable = %d", __func__, user_data);

	data &= user_data;
	user_data = 0;
	param_id = CAPI_V2_TAS_TX_ENABLE;
	ret = afe_smartamp_algo_ctrl((u8 *)&user_data, param_id,
		TAS_GET_PARAM, sizeof(uint32_t), AFE_SMARTAMP_MODULE_TX);
	if(ret < 0)
	{
		pr_err("[Smartamp:%s] Failed to get Tx Enable", __func__);
		user_data = 0;
	}else
		pr_info("[Smartamp:%s] Tx Enable = %d", __func__, user_data);

	data &= user_data;	
	pUcontrol->value.integer.value[0] = data;
	pr_info("[Smartamp:%s] case %s", __func__, tas2562_smartamp_status_text[data]);

	return 0;
}
#endif
static const struct snd_kcontrol_new msm_smartamp_tas2562_mixer_controls[] = {
	SOC_SINGLE_EXT("TAS2562_SET_SPKID_LEFT", SND_SOC_NOPM, 0, 0x7fffffff, 0,			
	    tas2562_get_spkid_left, tas2562_set_spkid_left),
	SOC_ENUM_EXT("TAS2562_ALGO_PROFILE", profile_index_enum[0],
		tas2562_get_profile, tas2562_set_profile),
	SOC_ENUM_EXT("TAS2562_ALGO_CALIB_TEST", tas2562_calib_test_enum[0],
		tas2562_calib_test_get_left, tas2562_calib_test_set_left),
	SOC_SINGLE_EXT("TAS2562_GET_RE_LEFT", SND_SOC_NOPM, 0, 0x7fffffff, 0,			
	        tas2562_get_re_left, NULL),
	SOC_SINGLE_EXT("TAS2562_GET_F0_LEFT", SND_SOC_NOPM, 0, 0x7fffffff, 0,			
	        tas2562_get_f0_left, NULL),
	SOC_SINGLE_EXT("TAS2562_GET_Q_LEFT", SND_SOC_NOPM, 0, 0x7fffffff, 0,			
	        tas2562_get_q_left, NULL),
	SOC_SINGLE_EXT("TAS2562_GET_TV_LEFT", SND_SOC_NOPM, 0, 0x7fffffff, 0,			
	        tas2562_get_tv_left, NULL),
	SOC_ENUM_EXT("TAS2562_SMARTPA_ENABLE", tas2562_smartamp_enable_enum[0],			
	        tas2562_smartamp_enable_get, tas2562_smartamp_enable_set),
	SOC_ENUM_EXT("TAS2562_ALGO_BYPASS", tas2562_smartamp_bypass_enum[0],
		tas2562_smartamp_bypass_get, tas2562_smartamp_bypass_set),
#if 0
	SOC_ENUM_EXT("TAS2562_ALGO_STATUS", tas2562_smartamp_status_enum[0],
		tas2562_get_smartamp_status, NULL),
#endif
};

void msm_smartamp_add_controls(struct snd_soc_codec *codec)
{
	mutex_init(&routing_lock);
	snd_soc_add_codec_controls(
		codec, msm_smartamp_tas2562_mixer_controls,
		ARRAY_SIZE(msm_smartamp_tas2562_mixer_controls));
}
