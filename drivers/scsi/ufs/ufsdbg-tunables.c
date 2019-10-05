/*
 * Copyright (c) 2017 -, Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

struct ufsdbg_tunables_debugfs_files {
	struct dentry	*debugfs_root;
	struct dentry	*d_gear_fix;
	struct dentry	*d_de_emphasis_lvl;
	struct dentry	*d_pre_emphasis_lvl;
	struct dentry	*d_tx_drv_lvl;
	struct dentry	*d_rx_equ_gain;
	struct dentry	*d_ufs_refclk_drv;
};

struct ufsdbg_phy_tune {
	int de_emphasis_lvl;
	int pre_emphasis_lvl;
	int tx_drv_lvl;
	int rx_equ_gain;
};

struct ufsdbg_hba_tune {
	int gear_fix;
	int ufs_refclk_drv;

	void __iomem *tlmm_ref_clk_ctl;
};

struct ufsdbg_tunables {
	struct ufsdbg_tunables_debugfs_files debugfs_files;

	struct ufsdbg_phy_tune t_phy;
	struct ufsdbg_hba_tune t_hba;
};

#ifdef IMPORT_TO_UFSHCD

//---------------------------------------------------------------------<gear fix>
static int ufsdbg_tunables_fix_gear(struct ufs_hba *hba, enum ufs_hs_gear_tag gear)
{
	int ret;
	struct ufs_pa_layer_attr new_pwr_info;
	bool scale_up = false;
	int old_gear = hba->pwr_info.gear_rx;

	if (hba->pwr_info.gear_rx == gear && hba->pwr_info.gear_tx == gear)
		return 0;

	if (gear > hba->pwr_info.gear_rx || gear > hba->pwr_info.gear_tx)
		scale_up = true;
	else
		scale_up = false;

	memcpy(&new_pwr_info, &hba->pwr_info, sizeof(struct ufs_pa_layer_attr));

	pm_runtime_get_sync(hba->dev);
	ufshcd_hold(hba, false);

	if (hba->clk_scaling.is_allowed) {
		cancel_work_sync(&hba->clk_scaling.suspend_work);
		cancel_work_sync(&hba->clk_scaling.resume_work);

		hba->clk_scaling.is_allowed = false;

		ufshcd_suspend_clkscaling(hba);
	}

	hba->ufs_stats.clk_hold.ctx = CLK_SCALE_WORK;
	ufshcd_hold_all(hba);

	ret = ufshcd_clock_scaling_prepare(hba);
	do {
		if (scale_up) {
			new_pwr_info.gear_rx ++;
			new_pwr_info.gear_tx ++;
		} else {
			new_pwr_info.gear_rx --;
			new_pwr_info.gear_tx --;
		}

		if (!scale_up) {
			ret = ufshcd_change_power_mode(hba, &new_pwr_info);
			if (ret) {
				dev_err(hba->dev, "ufsdbg_fix_gear : ufshcd_change_power_mode fail#1(%d)\n", ret);
				goto out;
			//ret = ufshcd_scale_gear(hba, false);
			} else {
				dev_info(hba->dev, "ufsdbg_fix_gear : change from gear-%d to gear-%d succeeded\n", old_gear, new_pwr_info.gear_rx);
				old_gear = new_pwr_info.gear_rx;
			}
		}
#if 0
		if (ufshcd_is_auto_hibern8_supported(hba)) {
			ret = ufshcd_uic_hibern8_enter(hba);
			if (ret) {
				dev_err(hba->dev, "ufsdbg_fix_gear : ufshcd_uic_hibern8_enter fail(%d)\n", ret);
				goto out;
			}
		}
#endif
		ret = ufshcd_scale_clks(hba, scale_up);
		if (ret) {
			dev_err(hba->dev, "ufsdbg_fix_gear : ufshcd_scale_clks fail(%d)\n", ret);
			goto out;
		} else {
			dev_info(hba->dev, "ufsdbg_fix_gear : scale_clks succeeded\n");
		}
#if 0
		if (ufshcd_is_auto_hibern8_supported(hba)) {
			ret = ufshcd_uic_hibern8_exit(hba);
			if (ret) {
				dev_err(hba->dev, "ufsdbg_fix_gear : ufshcd_uic_hibern8_exit fail(%d)\n", ret);
				goto out;
			}
		}
#endif
		if (scale_up) {
			ret = ufshcd_change_power_mode(hba, &new_pwr_info);
			if (ret) {
				dev_err(hba->dev, "ufsdbg_fix_gear : ufshcd_change_power_mode fail#2(%d)\n", ret);
				ufshcd_scale_clks(hba, false);
				goto out;
			} else {
				dev_info(hba->dev, "ufsdbg_fix_gear : change from gear-%d to gear-%d succeeded\n", old_gear, new_pwr_info.gear_rx);
				old_gear = new_pwr_info.gear_rx;
			}
		}

		if (!ret) {
			hba->clk_scaling.is_scaled_up = scale_up;
			if (scale_up)
				hba->clk_gating.delay_ms =
					hba->clk_gating.delay_ms_perf;
			else
				hba->clk_gating.delay_ms =
					hba->clk_gating.delay_ms_pwr_save;
		}

		msleep(1);

	} while (hba->pwr_info.gear_rx != gear || hba->pwr_info.gear_tx != gear);
out:
	ufshcd_clock_scaling_unprepare(hba);
	ufshcd_release_all(hba);
	hba->ufs_stats.clk_rel.ctx = CLK_SCALE_WORK;

	ufshcd_release(hba, false);
	pm_runtime_put_sync(hba->dev);

	return ret;
}

static int ufsdbg_gear_desc_show(struct seq_file *file, void *data)
{
	struct ufs_hba *hba = (struct ufs_hba *)file->private;
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	seq_printf(file, "current expected gear = %d\n\n", tunable->t_hba.gear_fix);
	return 0;
}

static int ufsdbg_gear_desc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufsdbg_gear_desc_show, inode->i_private);
}

static ssize_t ufsdbg_gear_write(struct file *file,
				const char __user *ubuf, size_t cnt,
				loff_t *ppos)
{
	struct ufs_hba *hba = file->f_mapping->host->i_private;
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	int ret;
	int gear = -1;

	if (!hba)
		return -EINVAL;

	ret = kstrtoint_from_user(ubuf, cnt, 0, &gear);
	if (ret) {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return ret;
	}

	if (gear==0)
		return cnt;

	if (gear > 0 && gear <=3) {
		tunable->t_hba.gear_fix = gear;
	}
	else {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return cnt;
	}

	if (!ufsdbg_tunables_fix_gear(hba, tunable->t_hba.gear_fix))
		dev_info(hba->dev, "gear fix applied successfully\n");
	else
		dev_err(hba->dev, "gear fix failed\n");

	return cnt;
}

static const struct file_operations ufsdbg_gear_desc = {
	.open		= ufsdbg_gear_desc_open,
	.read		= seq_read,
	.write		= ufsdbg_gear_write,
};
//--------------------------------------------------------------------</gear fix>
//--------------------------------------------------------------------<pre emp>
static int ufsdbg_preemp_desc_show(struct seq_file *file, void *data)
{
	struct ufs_hba *hba = (struct ufs_hba *)file->private;
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	seq_printf(file, "current preemp_lvl = %d\n\n", tunable->t_phy.pre_emphasis_lvl);
	return 0;
}

static int ufsdbg_preemp_desc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufsdbg_preemp_desc_show, inode->i_private);
}

static ssize_t ufsdbg_preemp_write(struct file *file,
				const char __user *ubuf, size_t cnt,
				loff_t *ppos)
{
	struct ufs_hba *hba = file->f_mapping->host->i_private;
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	int ret;
	int preemp_lvl = -1;

	if (!hba)
		return -EINVAL;

	ret = kstrtoint_from_user(ubuf, cnt, 0, &preemp_lvl);
	if (ret) {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return ret;
	}

	if (preemp_lvl>=0 && preemp_lvl<32) {
		tunable->t_phy.pre_emphasis_lvl = preemp_lvl;
	}
	else {
		return cnt;
	}

	return cnt;
}

static const struct file_operations ufsdbg_preemp_desc = {
	.open		= ufsdbg_preemp_desc_open,
	.read		= seq_read,
	.write		= ufsdbg_preemp_write,
};

//--------------------------------------------------------------------</pre emp>

//--------------------------------------------------------------------<de_emph>
static int ufsdbg_deemp_lvl_show(struct seq_file *file, void *data)
{
	struct ufs_hba *hba = (struct ufs_hba *)file->private;
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	seq_printf(file, "current deemp_lvl = %d\n\n", tunable->t_phy.de_emphasis_lvl);
	return 0;
}

static int ufsdbg_deemp_lvl_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufsdbg_deemp_lvl_show, inode->i_private);
}

static ssize_t ufsdbg_deemp_lvl_write(struct file *file,
				const char __user *ubuf, size_t cnt,
				loff_t *ppos)
{
	struct ufs_hba *hba = file->f_mapping->host->i_private;
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	int ret;
	int deemp_lvl = -1;

	if (!hba)
		return -EINVAL;

	ret = kstrtoint_from_user(ubuf, cnt, 0, &deemp_lvl);
	if (ret) {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return ret;
	}

	if (deemp_lvl>=0 && deemp_lvl<32) {
		tunable->t_phy.de_emphasis_lvl = deemp_lvl;
	}
	else {
		return cnt;
	}

	return cnt;
}

static const struct file_operations ufsdbg_de_emphasis_lvl_desc = {
	.open		= ufsdbg_deemp_lvl_open,
	.read		= seq_read,
	.write		= ufsdbg_deemp_lvl_write,
};

//-------------------------------------------------------------------</de_emph>

//--------------------------------------------------------------------<tx drv lvl>
static int ufsdbg_tx_drv_lvl_show(struct seq_file *file, void *data)
{
	struct ufs_hba *hba = (struct ufs_hba *)file->private;
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	seq_printf(file, "current tx_drv_lvl = %d\n\n", tunable->t_phy.tx_drv_lvl);
	return 0;
}

static int ufsdbg_tx_drv_lvl_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufsdbg_tx_drv_lvl_show, inode->i_private);
}

static ssize_t ufsdbg_tx_drv_lvl_write(struct file *file,
				const char __user *ubuf, size_t cnt,
				loff_t *ppos)
{
	struct ufs_hba *hba = file->f_mapping->host->i_private;
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	int ret;
	int drv_lvl = -1;

	if (!hba)
		return -EINVAL;

	ret = kstrtoint_from_user(ubuf, cnt, 0, &drv_lvl);
	if (ret) {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return ret;
	}

	if (drv_lvl>=0 && drv_lvl<32) {
		tunable->t_phy.tx_drv_lvl = drv_lvl;
	}
	else {
		return cnt;
	}

	return cnt;
}

static const struct file_operations ufsdbg_tx_drv_lvl_desc = {
	.open		= ufsdbg_tx_drv_lvl_open,
	.read		= seq_read,
	.write		= ufsdbg_tx_drv_lvl_write,
};
//-------------------------------------------------------------------</tx drv lvl>

//-------------------------------------------------------------------<rx eq gain>

static int ufsdbg_rx_equ_gain_show(struct seq_file *file, void *data)
{
	struct ufs_hba *hba = (struct ufs_hba *)file->private;
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	seq_printf(file, "current rx_equ_gain = %d\n\n", tunable->t_phy.rx_equ_gain);
	return 0;
}

static int ufsdbg_rx_equ_gain_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufsdbg_rx_equ_gain_show, inode->i_private);
}

static ssize_t ufsdbg_rx_equ_gain_write(struct file *file,
               const char __user *ubuf, size_t cnt,
               loff_t *ppos)
{
	struct ufs_hba *hba = file->f_mapping->host->i_private;
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	int ret;
	int rx_equ = -1;

	if (!hba)
		return -EINVAL;

	ret = kstrtoint_from_user(ubuf, cnt, 0, &rx_equ);
	if (ret) {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return ret;
	}

	// rx0,rx1 for each 0x00~0x0f
	if (rx_equ>=0 && rx_equ<=0xf) {
		tunable->t_phy.rx_equ_gain = rx_equ;
	}
	else {
		return cnt;
	}

	return cnt;
}

static const struct file_operations ufsdbg_rx_equ_gain_desc = {
   .open       = ufsdbg_rx_equ_gain_open,
   .read       = seq_read,
   .write      = ufsdbg_rx_equ_gain_write,
};
//------------------------------------------------------------------</rx eq gain>

//----------------------------------------------------------------<ref clk drv str>
static int ufsdbg_ufs_refclk_drv_show(struct seq_file *file, void *data)
{
	struct ufs_hba *hba = (struct ufs_hba *)file->private;
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	seq_printf(file, "current ufs_refclk_drv = %d\n\n", tunable->t_hba.ufs_refclk_drv);
	return 0;
}

static int ufsdbg_ufs_refclk_drv_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufsdbg_ufs_refclk_drv_show, inode->i_private);
}

static ssize_t ufsdbg_ufs_refclk_drv_write(struct file *file,
               const char __user *ubuf, size_t cnt,
               loff_t *ppos)
{
	struct ufs_hba *hba = file->f_mapping->host->i_private;
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	int ret;
	int refclk_drv = -1;

	if (!hba)
		return -EINVAL;

	ret = kstrtoint_from_user(ubuf, cnt, 0, &refclk_drv);
	if (ret) {
		dev_err(hba->dev, "%s: Invalid argument\n", __func__);
		return ret;
	}

	if (refclk_drv>=0 && refclk_drv<=7) {
		tunable->t_hba.ufs_refclk_drv = refclk_drv;
	}
	else {
		return cnt;
	}

	return cnt;
}

static const struct file_operations ufsdbg_ufs_refclk_drv_desc = {
   .open       = ufsdbg_ufs_refclk_drv_open,
   .read       = seq_read,
   .write      = ufsdbg_ufs_refclk_drv_write,
};
//---------------------------------------------------------------</ref clk drv str>

static void ufsdbg_tunables_remove_debugfs(struct ufs_hba *hba)
{
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	debugfs_remove_recursive(tunable->debugfs_files.debugfs_root);
	tunable->debugfs_files.debugfs_root = NULL;
}

static void ufsdbg_tunables_add_debugfs(struct ufs_hba *hba, struct dentry *root)
{
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	struct dentry *tunable_root;
	if (!root || !tunable) {
		dev_err(hba->dev, "%s: NULL root, exiting\n", __func__);
		return;
	}

	tunable->debugfs_files.debugfs_root = debugfs_create_dir("tunables", root);
	if (IS_ERR(tunable->debugfs_files.debugfs_root) || !tunable->debugfs_files.debugfs_root)
		goto err_no_root;

	tunable_root = tunable->debugfs_files.debugfs_root;
//--------------------------------------------------------------------
	tunable->debugfs_files.d_gear_fix =
			debugfs_create_file("gear_fix", S_IRUSR | S_IWUSR,
			tunable_root, hba, &ufsdbg_gear_desc);

	if (!tunable->debugfs_files.d_gear_fix) {
		dev_err(hba->dev, "%s:  NULL gear_set desc file, exiting", __func__);
		goto err;
	}
//--------------------------------------------------------------------
	tunable->debugfs_files.d_pre_emphasis_lvl =
			debugfs_create_file("pre_emphasis_lvl", S_IRUSR | S_IWUSR,
			tunable_root, hba, &ufsdbg_preemp_desc);

	if (!tunable->debugfs_files.d_pre_emphasis_lvl) {
		dev_err(hba->dev, "%s:	NULL pre_emphasis_lvl file, exiting", __func__);
		goto err;
	}
//--------------------------------------------------------------------
	tunable->debugfs_files.d_de_emphasis_lvl =
			debugfs_create_file("de_emphasis_lvl", S_IRUSR | S_IWUSR,
			tunable_root, hba, &ufsdbg_de_emphasis_lvl_desc);

	if (!tunable->debugfs_files.d_de_emphasis_lvl) {
		dev_err(hba->dev, "%s:  NULL de_emphasis_lvl desc file, exiting", __func__);
		goto err;
	}
//--------------------------------------------------------------------
	tunable->debugfs_files.d_tx_drv_lvl =
			debugfs_create_file("tx_drv_lvl", S_IRUSR | S_IWUSR,
			tunable_root, hba, &ufsdbg_tx_drv_lvl_desc);

	if (!tunable->debugfs_files.d_tx_drv_lvl) {
		dev_err(hba->dev, "%s:  NULL tx_drv_lvl desc file, exiting", __func__);
		goto err;
	}
//--------------------------------------------------------------------
	tunable->debugfs_files.d_rx_equ_gain =
			debugfs_create_file("rx_equ_gain", S_IRUSR | S_IWUSR,
			tunable_root, hba, &ufsdbg_rx_equ_gain_desc);

	if (!tunable->debugfs_files.d_rx_equ_gain) {
		dev_err(hba->dev, "%s:  NULL rx_equ_gain desc file, exiting", __func__);
		goto err;
	}
//--------------------------------------------------------------------
	tunable->debugfs_files.d_ufs_refclk_drv =
			debugfs_create_file("ufs_refclk_drv", S_IRUSR | S_IWUSR,
			tunable_root, hba, &ufsdbg_ufs_refclk_drv_desc);

	if (!tunable->debugfs_files.d_ufs_refclk_drv) {
		dev_err(hba->dev, "%s:  NULL ufs_refclk_drv desc file, exiting", __func__);
		goto err;
	}
//--------------------------------------------------------------------

	return;
err:
	ufsdbg_tunables_remove_debugfs(hba);

err_no_root:
	dev_err(hba->dev, "%s:  failed to initialize debugfs\n", __func__);
}

// END IMPORT_TO_UFSHCD

#elif defined(IMPORT_TO_UFSQCOM)
#ifdef readl_poll_timeout
#undef readl_poll_timeout
#endif
#include "../../phy/qualcomm/phy-qcom-ufs-i.h"

//-----------------------------------------------------------------//
#if defined(CONFIG_ARCH_SM8150)
/* 80-PD867-2X. (HW register description documents) */
/* UFS	 Reference Clock Drive Strength : 0x03DB5000 */
#define TLMM_BASE								0x03000000
#define TLMM_SOUTH_REG_BASE						(TLMM_BASE + 0x00d00000)
#define HWIO_TLMM_UFS_REF_CLK_CTL_ADDR			(TLMM_SOUTH_REG_BASE + 0x000b5000)
#define HWIO_TLMM_UFS2_REF_CLK_CTL_ADDR			(0x0) // not exists

	/*
		BIT[4:3] : UFS_REF_CLK_PULL
		BIT[2:0] : UFS_REF_CLK_HDRV
	*/
#define HWIO_TLMM_UFS_REF_CLK_CTL_MSK			0x7
//-----------------------------------------------------------------//
#elif defined(CONFIG_ARCH_SDM845) || defined(CONFIG_ARCH_MSM8998)
/* 80-P6348-2X. (HW register description documents) */
/* UFS   Reference Clock Drive Strength : 0x03D9E000 */
/* UFS2 Reference CLock Drive Strength : 0x03DA0000 */

#define TLMM_BASE								0x03400000
#define TLMM_SOUTH_REG_BASE						(TLMM_BASE + 0x00900000)
#define HWIO_TLMM_UFS_REF_CLK_CTL_ADDR			(TLMM_SOUTH_REG_BASE + 0x0009e000)
#define HWIO_TLMM_UFS2_REF_CLK_CTL_ADDR			(TLMM_SOUTH_REG_BASE + 0x000a0000)

/*
	BIT[4:3] : UFS_REF_CLK_PULL
	BIT[2:0] : UFS_REF_CLK_HDRV
*/
#define HWIO_TLMM_UFS_REF_CLK_CTL_MSK			0x7
//-----------------------------------------------------------------//
#elif defined(CONFIG_ARCH_MSM8996)
/* UFS Reference Clock Drive Strength : 0x01146000 */
#define TLMM_BASE								0x01000000
#define TLMM_CSR_REG_BASE						(TLMM_BASE + 0x00010000)
#define HWIO_TLMM_UFS_REF_CLK_CTL_ADDR			(TLMM_CSR_REG_BASE + 0x00136000)
#define HWIO_TLMM_UFS_REF_CLK_CTL_MSK			0x7
//-----------------------------------------------------------------//
#else
#define TLMM_BASE
#define HWIO_TLMM_UFS_REF_CLK_CTL_ADDR
#define HWIO_TLMM_UFS_REF_CLK_CTL_MSK
#endif
//-----------------------------------------------------------------//

/* should be called after ufs qcom phy being initialized */
static int ufsdbg_tunables_init(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct ufs_qcom_phy *ufs_qcom_phy = get_ufs_qcom_phy(host->generic_phy);

	struct ufsdbg_tunables *tunables = kzalloc(sizeof(struct ufsdbg_tunables), GFP_KERNEL);
	if(!tunables)
		return -ENOMEM;

	memset(&tunables->debugfs_files, 0x0, sizeof(tunables->debugfs_files));
	tunables->t_hba.gear_fix 		= -1;
	tunables->t_hba.ufs_refclk_drv 	= -1;
	tunables->t_phy.de_emphasis_lvl	= -1;
	tunables->t_phy.pre_emphasis_lvl	= -1;
	tunables->t_phy.tx_drv_lvl 		= -1;
	tunables->t_phy.rx_equ_gain 	= -1;

	dev_info(hba->dev, "%s : phy(%s)\n", __func__, dev_name(&host->generic_phy->dev));

	if (hba->extcon && HWIO_TLMM_UFS2_REF_CLK_CTL_ADDR != 0x0)
		tunables->t_hba.tlmm_ref_clk_ctl = ioremap(HWIO_TLMM_UFS2_REF_CLK_CTL_ADDR, 4);
	else
		tunables->t_hba.tlmm_ref_clk_ctl = ioremap(HWIO_TLMM_UFS_REF_CLK_CTL_ADDR, 4);

	dev_info(hba->dev, "     tlmm_ref_clk_ctl(%p)\n", (void*)tunables->t_hba.tlmm_ref_clk_ctl);

	hba->ufsdbg_tunables = tunables;
	ufs_qcom_phy->ufsdbg_tunables = tunables;

	return 0;
}

static int ufsdbg_tunables_refclk_drv_apply(struct ufs_hba *hba)
{
	struct ufsdbg_tunables *tunables = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	// ref clk drive strength
	if (tunables->t_hba.ufs_refclk_drv>=0) {
		u32 read_clk_drv = readl_relaxed(tunables->t_hba.tlmm_ref_clk_ctl);
		u32 write_clk_drv = ((read_clk_drv & (unsigned int)(~HWIO_TLMM_UFS_REF_CLK_CTL_MSK)) | ((unsigned int)(tunables->t_hba.ufs_refclk_drv & HWIO_TLMM_UFS_REF_CLK_CTL_MSK)));
		dev_info(hba->dev, "read_clk_drv(0x%x), write_clk_drv(0x%x)\n", read_clk_drv, write_clk_drv);
		writel_relaxed( write_clk_drv, tunables->t_hba.tlmm_ref_clk_ctl);
		mb();
		read_clk_drv = readl_relaxed(tunables->t_hba.tlmm_ref_clk_ctl);
		dev_info(hba->dev, "UFS_REF_CLK_PULL(0x%x), UFS_REF_CLK_HDRV(0x%x)\n", read_clk_drv>>3, read_clk_drv & HWIO_TLMM_UFS_REF_CLK_CTL_MSK);
	} else {
		u32 read_clk_drv = readl_relaxed(tunables->t_hba.tlmm_ref_clk_ctl);
		dev_info(hba->dev, "UFS_REF_CLK_PULL(0x%x), UFS_REF_CLK_HDRV(0x%x)\n", read_clk_drv>>3, read_clk_drv & HWIO_TLMM_UFS_REF_CLK_CTL_MSK);
	}
	return 0;
}

static int ufsdbg_get_fix_gear(struct ufs_hba *hba)
{
	struct ufsdbg_tunables *tunables = (struct ufsdbg_tunables *)hba->ufsdbg_tunables;
	return tunables->t_hba.gear_fix;
}

// END IMPORT_TO_UFSQCOM

#elif defined(IMPORT_TO_UFSMPHY)

//-----------------------------------------------------------------//
#if defined(CONFIG_ARCH_SM8150)
/* 80-PD867-5D-B. (PHY tuning guideline) & 80-PD867-2X. (HW register description documents) */
/* override or add to phy-qcom-ufs-qmp-v4.h */
/* UFS PHY TX registers */
/*
	5.3 Tx emphasis tuning parameters

	5.3.1 Precursor adjustment
	UFS_MEM_MPHY_UFS_QSERDES_TX0_PRE_EMPH: 0x01D87508
	UFS_MEM_MPHY_UFS_QSERDES_TX1_PRE_EMPH: 0x01D87908

	5.3.2 Postcursor adjustment
	UFS_MEM_MPHY_UFS_QSERDES_TX0_TX_EMP_POST1_LVL (0x01D8740C)
	UFS_MEM_MPHY_UFS_QSERDES_TX1_TX_EMP_POST1_LVL (0x01D8780C)
*/
#define QSERDES_TX0_PRE_EMPH	TX_OFF(0, 0x108)
#define QSERDES_TX1_PRE_EMPH	TX_OFF(1, 0x108)

#define QSERDES_TX0_TX_EMP_POST1_LVL	TX_OFF(0, 0xc)
#define QSERDES_TX1_TX_EMP_POST1_LVL	TX_OFF(1, 0xc)

/*
	5.2 Tx swing tuning parameters
	UFS_MEM_MPHY_UFS_QSERDES_TX0_TX_DRV_LVL (0x01D87414)
	UFS_MEM_MPHY_UFS_QSERDES_TX1_TX_DRV_LVL (0x01D87814)
*/
#define QSERDES_TX0_TX_DRV_LVL			TX_OFF(0, 0x14)
#define QSERDES_TX1_TX_DRV_LVL			TX_OFF(1, 0x14)

// ref. RX EQU default values
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2, 0x06),
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL3, 0x04),
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4, 0x1D),

//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL2, 0x06),
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL3, 0x04),
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4, 0x1D),

//-----------------------------------------------------------------//
/* 80-P2169-5D. (PHY tuning guideline) & 80-P6348-2X. (HW register description documents) */
#elif defined(CONFIG_ARCH_SDM845) || defined(CONFIG_ARCH_MSM8998)
/* override or add to phy-qcom-ufs-qmp-v3.h */
/* UFS PHY TX registers */
#define QSERDES_TX0_TX_EMP_POST1_LVL	TX_OFF(0, 0xc)
#define QSERDES_TX1_TX_EMP_POST1_LVL	TX_OFF(1, 0xc)

#define QSERDES_TX0_TX_DRV_LVL			TX_OFF(0, 0x1c)
#define QSERDES_TX1_TX_DRV_LVL			TX_OFF(1, 0x1c)
// ref. RX EQU default values
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2, 0x06),	// BIT[4]==0 -> default to override CNTRL4 register
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4, 0x1D),	// BIT[4]==1 -> override BIT[0:3] : 1dB
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL2, 0x06),	// BIT[4]==0 -> default to override CNTRL4 register
//UFS_QCOM_PHY_CAL_ENTRY(QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4, 0x1D),	// BIT[4]==1 -> override BIT[0:3] : 1dB

//-----------------------------------------------------------------//
#elif defined(CONFIG_ARCH_MSM8996)
/* override or add to phy-qcom-ufs-qmp-14nm.h */

//-----------------------------------------------------------------//
#else

#endif
//-----------------------------------------------------------------//
static int ufsdbg_tunables_mphy_apply(struct ufs_qcom_phy *ufs_qcom_phy)
{
	struct ufsdbg_tunables *tunable = (struct ufsdbg_tunables *)ufs_qcom_phy->ufsdbg_tunables;

	dev_info(ufs_qcom_phy->dev, "%s : ufs_qcom_phy(%p)/mmio(%p)\n", __func__, ufs_qcom_phy, ufs_qcom_phy->mmio);
	// pre-emphasis_lvl
	if (tunable->t_phy.pre_emphasis_lvl >= 0) {
		dev_info(ufs_qcom_phy->dev, "pre_emphasis_lvl(%u)\n",tunable->t_phy.pre_emphasis_lvl);
		writel_relaxed(tunable->t_phy.pre_emphasis_lvl|0x20, ufs_qcom_phy->mmio + QSERDES_TX0_PRE_EMPH);
		writel_relaxed(tunable->t_phy.pre_emphasis_lvl|0x20, ufs_qcom_phy->mmio + QSERDES_TX1_PRE_EMPH);
		mb();
		dev_info(ufs_qcom_phy->dev, "written pre_emph lvl:TX0:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX0_PRE_EMPH));
		dev_info(ufs_qcom_phy->dev, "written pre_emph lvl:TX1:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX1_PRE_EMPH));
	} else {
		dev_info(ufs_qcom_phy->dev, "current pre_emph lvl:TX0:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX0_PRE_EMPH));
		dev_info(ufs_qcom_phy->dev, "written pre_emph lvl:TX1:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX1_PRE_EMPH));
	}

	// de-emphasis_lvl
	if (tunable->t_phy.de_emphasis_lvl >= 0) {
		dev_info(ufs_qcom_phy->dev, "de_emphasis_lvl(%u)\n",tunable->t_phy.de_emphasis_lvl);
		writel_relaxed(tunable->t_phy.de_emphasis_lvl|0x20, ufs_qcom_phy->mmio + QSERDES_TX0_TX_EMP_POST1_LVL);
		writel_relaxed(tunable->t_phy.de_emphasis_lvl|0x20, ufs_qcom_phy->mmio + QSERDES_TX1_TX_EMP_POST1_LVL);
		mb();
		dev_info(ufs_qcom_phy->dev, "written de_emph lvl:TX0:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX0_TX_EMP_POST1_LVL));
		dev_info(ufs_qcom_phy->dev, "written de_emph lvl:TX1:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX1_TX_EMP_POST1_LVL));
	} else {
		dev_info(ufs_qcom_phy->dev, "current de_emph lvl:TX0:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX0_TX_EMP_POST1_LVL));
		dev_info(ufs_qcom_phy->dev, "written de_emph lvl:TX1:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX1_TX_EMP_POST1_LVL));
	}

	// tx_drv_lvl
	if (tunable->t_phy.tx_drv_lvl >= 0) {
		dev_info(ufs_qcom_phy->dev, "tx_drv_lvl(%u)\n", tunable->t_phy.tx_drv_lvl);
		writel_relaxed(tunable->t_phy.tx_drv_lvl|0x20, ufs_qcom_phy->mmio + QSERDES_TX0_TX_DRV_LVL);
		writel_relaxed(tunable->t_phy.tx_drv_lvl|0x20, ufs_qcom_phy->mmio + QSERDES_TX1_TX_DRV_LVL);
		mb();
		dev_info(ufs_qcom_phy->dev, "current large_amp(0x%x)\n" , readl_relaxed(ufs_qcom_phy->mmio + UFS_PHY_TX_LARGE_AMP_DRV_LVL));
		dev_info(ufs_qcom_phy->dev, "        small_amp(0x%x)\n", readl_relaxed(ufs_qcom_phy->mmio + UFS_PHY_TX_SMALL_AMP_DRV_LVL));
		dev_info(ufs_qcom_phy->dev, "written tx_drv_lvl:TX0:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX0_TX_DRV_LVL));
		dev_info(ufs_qcom_phy->dev, "written tx_drv_lvl:TX1:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX1_TX_DRV_LVL));
	} else {
		dev_info(ufs_qcom_phy->dev, "current large_amp(0x%x)\n" , readl_relaxed(ufs_qcom_phy->mmio + UFS_PHY_TX_LARGE_AMP_DRV_LVL));
		dev_info(ufs_qcom_phy->dev, "        small_amp(0x%x)\n", readl_relaxed(ufs_qcom_phy->mmio + UFS_PHY_TX_SMALL_AMP_DRV_LVL));
		dev_info(ufs_qcom_phy->dev, "current tx_drv_lvl:TX0:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX0_TX_DRV_LVL));
		dev_info(ufs_qcom_phy->dev, "current tx_drv_lvl:TX1:0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_TX1_TX_DRV_LVL));
	}

	// rx equ gain : QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4, QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4
	if (tunable->t_phy.rx_equ_gain >= 0) {
		dev_info(ufs_qcom_phy->dev, "rx_equ_gain(%u)\n", tunable->t_phy.rx_equ_gain);

#if defined(CONFIG_ARCH_SM8150)
		/*
		Description: Equalizer gain adaptation must be turned off to apply manual settings. Use the
		following steps:
		1. Set RXn_EQU_ADAPTOR_CNTRL2 [6:4] to 0b101.
		2. Set RXn_EQU_ADAPTOR_CNTRL3 [2:0] to 0b100.
		3. Set RXn_EQU_ADAPTOR_CNTRL4 [6:5] to 0b11.
		4. Use RXn_EQU_ADAPTOR_CNTRL4 [3:0] to set EQ2 peaking.
		*/
		// RX0
		writel_relaxed( (5<<4), ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2);
		writel_relaxed( 4, ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL3);
		writel_relaxed( (3<<5), ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4);
		writel_relaxed(tunable->t_phy.rx_equ_gain, ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4);
		mb();

		// RX1
		writel_relaxed( (5<<4), ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL2);
		writel_relaxed( 4, ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL3);
		writel_relaxed( (3<<5), ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4);
		writel_relaxed(tunable->t_phy.rx_equ_gain, ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4);
		mb();

#elif defined(CONFIG_ARCH_SDM845) || defined(CONFIG_ARCH_MSM8998)
		writel_relaxed(tunable->t_phy.rx_equ_gain, ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4);
		writel_relaxed(tunable->t_phy.rx_equ_gain, ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4);
		mb();
#endif
		dev_info(ufs_qcom_phy->dev, "written rx_equ_gain:(rx0:0x%x, rx1:0x%x)\n",
			readl_relaxed(ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4), readl_relaxed(ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4));
	} else {
		dev_info(ufs_qcom_phy->dev, "current rx_equ_gain - CTL2 : (RX0,RX1)\n");
		dev_info(ufs_qcom_phy->dev, "                            RX0 : 0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2));
		dev_info(ufs_qcom_phy->dev, "                            RX1 : 0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL2));
		dev_info(ufs_qcom_phy->dev, "                      CTL4 : (RX0,RX1)\n");
		dev_info(ufs_qcom_phy->dev, "                            RX0 : 0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4));
		dev_info(ufs_qcom_phy->dev, "                            RX1 : 0x%x\n", readl_relaxed(ufs_qcom_phy->mmio + QSERDES_RX1_RX_EQU_ADAPTOR_CNTRL4));
	}
	return 0;
}

#endif
