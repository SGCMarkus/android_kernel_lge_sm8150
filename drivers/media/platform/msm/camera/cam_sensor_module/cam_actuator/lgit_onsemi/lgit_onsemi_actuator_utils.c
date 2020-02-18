/* ================================================================== */
/*  ONSEMI AF HALL Utils  */
/* ================================================================== */
#include <linux/device.h>
#include <asm/arch_timer.h>
#include <cam_sensor_cmn_header.h>
#include "cam_actuator_core.h"
#include "cam_debug_util.h"
#include "lgit_onsemi_actuator_utils.h"

#ifndef CONFIG_MACH_SM8150_BETA
#define REG_ADDR_SOC_0 0x84
#define REG_ADDR_SOC_3 0x84
#else
#define REG_ADDR_SOC_0 0xF01A
#define REG_ADDR_SOC_3 0x3
#endif

extern struct class* get_camera_class(void);
extern void actuator_create_sysfs(struct cam_actuator_ctrl_t *a_ctrl);
extern void actuator_destroy_sysfs(struct cam_actuator_ctrl_t *a_ctrl);
extern bool msm_act_data_enqueue(uint32_t reg_addr, uint32_t reg_data,
		struct cam_actuator_ctrl_t *a_ctrl);

extern bool msm_act_data_enqueue(uint32_t reg_addr, uint32_t reg_data,
		struct cam_actuator_ctrl_t *a_ctrl)
{
	bool rc;
	struct cam_actuator_ctrl_t *a_ctrl_in = a_ctrl;
	struct msm_act_readout_buffer *o_buf; // = &(a_ctrl_in->buf);
	uint64_t hall_readout_time;
    int16_t  act_hall;

	CAM_DBG(CAM_ACTUATOR, "[AFHALL] reg_addr %d reg_data %d, soc_id : %d", reg_addr, reg_data, a_ctrl->soc_info.index);

    if (a_ctrl == NULL) {
        return true;
    }

    switch(a_ctrl->soc_info.index)
    {
        case 0:
            if (REG_ADDR_SOC_0 != reg_addr) { return true; } break;
        case 3:
            if (REG_ADDR_SOC_3 != reg_addr) { return true; } break;
        default :
            if (REG_ADDR_SOC_0 != reg_addr) { return true; } break;
    }

    o_buf = &(a_ctrl_in->buf);
    hall_readout_time = arch_counter_get_cntvct();
    act_hall = (reg_data & 0x800) ? (0xF000 | (0xFFF & reg_data)) : (0xFFF & reg_data);

	CAM_DBG(CAM_ACTUATOR, "[AFHALL] ACT_HALL : %u soc_id : %d", act_hall, a_ctrl->soc_info.index);

	spin_lock(&a_ctrl_in->hall_lock);

	if ((o_buf->buffer_tail >= 0) &&
            (o_buf->buffer_tail < MSM_ACT_DATA_BUFFER_SIZE)) {
		o_buf->buffer[o_buf->buffer_tail].act_hall = act_hall;
		o_buf->buffer[o_buf->buffer_tail].hall_readout_time = hall_readout_time;

		o_buf->buffer_tail++;
		if (o_buf->buffer_tail >= MSM_ACT_DATA_BUFFER_SIZE)
			o_buf->buffer_tail -= MSM_ACT_DATA_BUFFER_SIZE;

		rc = true;
	} else {
		rc = false;
	}

	spin_unlock(&a_ctrl_in->hall_lock);

	return rc;
}

static ssize_t show_act_vt_hall(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct cam_actuator_ctrl_t *a_ctrl = dev_get_drvdata(dev);

	if(a_ctrl->cam_act_state == CAM_ACTUATOR_START) {
		struct msm_act_readout hall_data[MAX_HALL_QUERY_SIZE];
		uint8_t query_size = MAX_HALL_QUERY_SIZE;
		uint8_t data_get_count = 0;
		uint16_t counter = 0;
		int32_t i;

		memset(hall_data, 0, sizeof(hall_data));

		spin_lock(&a_ctrl->hall_lock);

		counter = a_ctrl->buf.buffer_tail;
		for (i = query_size-1; i >= 0; i-- )
		{
			counter = (counter == 0) ? MSM_ACT_DATA_BUFFER_SIZE-1 : counter-1;
			hall_data[i].act_hall = a_ctrl->buf.buffer[counter].act_hall;
			hall_data[i].hall_readout_time = a_ctrl->buf.buffer[counter].hall_readout_time;
			data_get_count++;
		}
		spin_unlock(&a_ctrl->hall_lock);

		if (data_get_count != 0 && data_get_count < MAX_HALL_QUERY_SIZE + 1) {
			for(i = 0; i < data_get_count; i++) {
				count += sprintf(buf + strlen(buf),"%d,%016llx,\n",hall_data[i].act_hall, hall_data[i].hall_readout_time);
			}
		}

		// Check hall data count
		CAM_DBG(CAM_ACTUATOR,"[AFHALL]:data_get_count = %d count : %d", data_get_count,count);
	}
	else {
        CAM_ERR(CAM_ACTUATOR,"[AFHALL] AF Thread is not working");
	}

	return count;
}
static DEVICE_ATTR(act_vt_hall, S_IRUGO, show_act_vt_hall, NULL);


static ssize_t show_act_tele_hall(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct cam_actuator_ctrl_t *a_ctrl = dev_get_drvdata(dev);

	if(a_ctrl->cam_act_state == CAM_ACTUATOR_START) {
		struct msm_act_readout hall_data[MAX_HALL_QUERY_SIZE];
		uint8_t query_size = MAX_HALL_QUERY_SIZE;
		uint8_t data_get_count = 0;
		uint16_t counter = 0;
		int32_t i;

		memset(hall_data, 0, sizeof(hall_data));

		spin_lock(&a_ctrl->hall_lock);

		counter = a_ctrl->buf.buffer_tail;
		for (i = query_size-1; i >= 0; i-- )
		{
			counter = (counter == 0) ? MSM_ACT_DATA_BUFFER_SIZE-1 : counter-1;
			hall_data[i].act_hall = a_ctrl->buf.buffer[counter].act_hall;
			hall_data[i].hall_readout_time = a_ctrl->buf.buffer[counter].hall_readout_time;
			data_get_count++;
		}
		spin_unlock(&a_ctrl->hall_lock);

		if (data_get_count != 0 && data_get_count < MAX_HALL_QUERY_SIZE + 1) {
			for(i = 0; i < data_get_count; i++) {
				count += sprintf(buf + strlen(buf),"%d,%016llx,\n",hall_data[i].act_hall, hall_data[i].hall_readout_time);
			}
		}

		// Check hall data count
		CAM_DBG(CAM_ACTUATOR,"[AFHALL]:data_get_count = %d count : %d", data_get_count,count);
	}
	else {
        CAM_ERR(CAM_ACTUATOR,"[AFHALL] AF Thread is not working");
	}

	return count;
}
static DEVICE_ATTR(act_tele_hall, S_IRUGO, show_act_tele_hall, NULL);

static ssize_t show_act_normal_hall(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct cam_actuator_ctrl_t *a_ctrl = dev_get_drvdata(dev);

	if(a_ctrl->cam_act_state == CAM_ACTUATOR_START) {
		struct msm_act_readout hall_data[MAX_HALL_QUERY_SIZE];
		uint8_t query_size = MAX_HALL_QUERY_SIZE;
		uint8_t data_get_count = 0;
		uint16_t counter = 0;
		int32_t i;

		memset(hall_data, 0, sizeof(hall_data));

		spin_lock(&a_ctrl->hall_lock);

		counter = a_ctrl->buf.buffer_tail;
		for (i = query_size-1; i >= 0; i-- )
		{
			counter = (counter == 0) ? MSM_ACT_DATA_BUFFER_SIZE-1 : counter-1;
			hall_data[i].act_hall = a_ctrl->buf.buffer[counter].act_hall;
			hall_data[i].hall_readout_time = a_ctrl->buf.buffer[counter].hall_readout_time;
			data_get_count++;
		}
		spin_unlock(&a_ctrl->hall_lock);

		if (data_get_count != 0 && data_get_count < MAX_HALL_QUERY_SIZE + 1) {
			for(i = 0; i < data_get_count; i++) {
				count += sprintf(buf + strlen(buf),"%d,%016llx,\n",hall_data[i].act_hall, hall_data[i].hall_readout_time);
			}
		}

		// Check hall data count
		CAM_DBG(CAM_ACTUATOR, "[AFHALL]:data_get_count = %d count : %d", data_get_count,count);
	}
	else {
		CAM_ERR(CAM_ACTUATOR, "[AFHALL] AF Thread is not working");
	}

	return count;
}
static DEVICE_ATTR(act_normal_hall, S_IRUGO, show_act_normal_hall, NULL);

extern void actuator_create_sysfs(struct cam_actuator_ctrl_t *a_ctrl) {
	struct device*  camera_act_hall_dev;

	if(!a_ctrl->camera_class) {
		a_ctrl->camera_class = get_camera_class();

		if(a_ctrl->soc_info.index == 0 && a_ctrl->camera_class != NULL) {
            CAM_INFO(CAM_ACTUATOR, "Create act_normal_hall_class!");
			camera_act_hall_dev   = device_create(a_ctrl->camera_class, NULL,
					0, a_ctrl, "actuator_normal");
			device_create_file(camera_act_hall_dev, &dev_attr_act_normal_hall);
		}

		if(a_ctrl->soc_info.index == 1 && a_ctrl->camera_class != NULL) {
            CAM_INFO(CAM_ACTUATOR, "Create act_vt_hall_class!");
			camera_act_hall_dev   = device_create(a_ctrl->camera_class, NULL,
					0, a_ctrl, "actuator_vt");
			device_create_file(camera_act_hall_dev, &dev_attr_act_vt_hall);
		}

		if(a_ctrl->soc_info.index == 3 && a_ctrl->camera_class != NULL) {
            CAM_INFO(CAM_ACTUATOR, "Create act_tele_hall_class!");
			camera_act_hall_dev   = device_create(a_ctrl->camera_class, NULL,
					0, a_ctrl, "actuator_tele");
			device_create_file(camera_act_hall_dev, &dev_attr_act_tele_hall);
		}
	}
}

extern void actuator_destroy_sysfs(struct cam_actuator_ctrl_t *a_ctrl) {
	if(a_ctrl->camera_class) {
		CAM_INFO(CAM_ACTUATOR, "[AFHALL] destory_sysfs id : %d", a_ctrl->soc_info.index);
		class_destroy(a_ctrl->camera_class);
		a_ctrl->camera_class = NULL;
	}
}

