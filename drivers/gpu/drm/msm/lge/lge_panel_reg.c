
#define pr_fmt(fmt)	"[Display][lge-panel-reg:%s:%d] " fmt, __func__, __LINE__
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <video/mipi_display.h>

#include "dsi_panel.h"
#include "dsi_ctrl_hw.h"

extern void dsi_panel_destroy_cmd_packets(struct dsi_panel_cmd_set *set);

extern int lge_dsi_panel_parse_cmd_sets_sub(struct dsi_panel_cmd_set *cmd,
					const char *data,
					u32 length);
extern int lge_dsi_panel_tx_cmd_set(struct dsi_panel *panel,
				struct dsi_panel_cmd_set *cmd);

static enum dsi_cmd_set_state command_state = DSI_CMD_SET_STATE_LP;

extern int lge_ddic_dsi_panel_tx_cmd_set(struct dsi_panel *panel,
				enum lge_ddic_dsi_cmd_set_type type);
extern int lge_mdss_dsi_panel_cmd_read(struct dsi_panel *panel,
					u8 cmd, int cnt, char* ret_buf);

static inline bool is_number(char c)
{
	return (c >= '0' && c <= '9') ||
			(c >= 'a' && c <= 'f') ||
			(c >= 'A' && c <= 'F');
}

static inline bool is_whitespace(char c)
{
	return (c == ' ' || c == '\n' || c == '\r' || c == '\t' || c == '\v' || c == '\f');
}

static int get_data(const char *buf, size_t len, u8 *data, u32 *count)
{
	int rc = 0, i = 0, nc = 0;
	u32 c = 0;
	if (!buf || len == 0 || !count)
		return -EINVAL;

	for (i = 0; i < len; i++) {
		if (is_number(buf[i])) {
			nc++;
		} else if (is_whitespace(buf[i])) {
			if (nc == 2) {
				if (data) {
					char tmp[] = {buf[i-2], buf[i-1], 0};
					data[c] = (u8)simple_strtoul(tmp, NULL, 16);
				}
				c++;
				nc = 0;
			} else if (nc > 2) {
				pr_err("[%s] i=%d, nc=%d\n",buf, i, nc);
				rc = -EINVAL;
				break;
			}
		} else {
			pr_err("[%s] %d\n",buf, i);
			rc = -EINVAL;
			break;
		}
	}

	if (!rc && nc == 2) {
		if (data) {
			char tmp[] = {buf[i-2], buf[i-1], 0};
			data[c] = (u8)simple_strtoul(tmp, NULL, 16);
		}
		c++;
		nc = 0;
	}

	if (nc) {
		pr_err("[%s] nc=%d\n",buf, nc);
		rc = -EINVAL;
	}

	if (!rc)
		*count = c;

	return rc;
}

static ssize_t send_command(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t len)
{
	u8 *data = NULL;
	u32 count = 0;
	struct dsi_panel_cmd_set cmd = {0,};
	struct dsi_panel *panel;
	int rc;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return len;
	}

	rc = get_data(buf, len, NULL, &count);
	if (!rc && count) {
		data = kzalloc(sizeof(u8) * count, GFP_KERNEL);
		if (!data) {
			pr_err("Not Enough Memory\n");
			return len;
		}
		get_data(buf, len, data, &count);

		if (!lge_dsi_panel_parse_cmd_sets_sub(&cmd, data, count)) {
			cmd.state = command_state;
			mutex_lock(&panel->panel_lock);
			if (lge_dsi_panel_tx_cmd_set(panel, &cmd))
				pr_err("sending command failed\n");
			mutex_unlock(&panel->panel_lock);
			dsi_panel_destroy_cmd_packets(&cmd);
			cmd.count = 0;
		} else {
			pr_err("Data Parsing Error\n");
		}
		kfree(data);
	} else {
		pr_err("rc=%d, count=%d\n", rc, count);
	}
	return len;
}

static ssize_t get_command_state(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", command_state==DSI_CMD_SET_STATE_LP?"lp":"hs");
}

static ssize_t set_command_state(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t len)
{
	if (!strncmp(buf, "lp", 2)) {
		command_state = DSI_CMD_SET_STATE_LP;
	} else if (!strncmp(buf, "hs", 2)) {
		command_state = DSI_CMD_SET_STATE_HS;
	}

	return len;
}

static u8 tx_buf[256] = {0x0};
static ssize_t read_command_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", tx_buf);
}

static ssize_t read_command(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t len)
{
	ssize_t ret;
	struct dsi_panel *panel;
	u8 reg[256] = {0x0};
	u32 cmd;
	int cnt, i;

	panel = dev_get_drvdata(dev);
	if (!panel) {
		pr_err("panel is NULL\n");
		return -EINVAL;
	}

	sscanf(buf, "%x %d", &cmd, &cnt);
	if ((cmd <= 0) || (cnt <= 0) || (cnt > 256)) {
		pr_err("invalid input, cmd:%x, cnt:%d\n", cmd, cnt);
		return -EINVAL;
	}

	lge_mdss_dsi_panel_cmd_read(panel, (u8)cmd, cnt, &reg[0]);
	if (panel->lge.use_ddic_reg_lock)
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_REGISTER_LOCK);

	ret = scnprintf(tx_buf, 8, "0x%02x: ", (int)cmd);
	for (i = 0; i < cnt; i++)
		ret += scnprintf(tx_buf + ret, 7, "0x%02x ", (int)reg[i]);
	if (ret)
		tx_buf[ret - 1] = '\n';

	return ret;
}

static DEVICE_ATTR(command, S_IWUSR, NULL, send_command);
static DEVICE_ATTR(command_state, S_IWUSR, get_command_state, set_command_state);
static DEVICE_ATTR(read_reg, S_IRUGO | S_IWUSR, read_command_data, read_command);

void lge_panel_reg_create_sysfs(struct dsi_panel *panel, struct class *class_panel)
{
	static struct device *panel_reg_dev = NULL;

	if (!panel_reg_dev) {
		panel_reg_dev = device_create(class_panel, NULL, 0, panel, "reg");
		if (IS_ERR(panel_reg_dev)) {
			pr_err("Failed to create dev(panel_reg_dev)!\n");
		} else {
			if (device_create_file(panel_reg_dev, &dev_attr_command) < 0)
				pr_err("Failed to create panel/reg/command\n");
			if (device_create_file(panel_reg_dev, &dev_attr_command_state) < 0)
				pr_err("Failed to create panel/reg/command_state\n");
			if (device_create_file(panel_reg_dev, &dev_attr_read_reg) < 0)
				pr_err("Failed to create panel/reg/read_reg\n");
		}
	}
}
