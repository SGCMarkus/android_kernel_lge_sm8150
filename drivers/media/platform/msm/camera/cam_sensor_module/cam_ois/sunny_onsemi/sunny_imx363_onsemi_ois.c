/* ================================================================== */
/*  OIS firmware */
/* ================================================================== */
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "sunny_imx363_onsemi_ois.h"
#include <linux/device.h>
#include <linux/spinlock.h>
#include <asm/arch_timer.h>

#define LAST_UPDATE "19-06-24, OIS LC898123F40 0x070e0102"

#define LIMIT_STATUS_POLLING    (15)
#define LIMIT_OIS_ON_RETRY      (5)
#define OIS_READ_STATUS_ADDR    (0xF100)
#define FW_VER                  (0x070e0102)
#define HALL_LIMIT              (314)

#define PROPERTY_VALUE_MAX      (92)

#define OIS_HALL_SEQ_ADDR       (0xE001)
#define OIS_HALL_X_ADDR         (0x01B0)
#define OIS_HALL_Y_ADDR         (0x01B4)
#define OIS_GYRO_ADDR           (0xE002)
#define OIS_HALL_X_OFFSET       (0x0110)
#define OIS_HALL_Y_OFFSET       (0x0160)
#define OIS_GYRO_GAIN_X         (0x82b8)
#define OIS_GYRO_GAIN_Y         (0x8318)
#define OIS_GYRO_DEFAULT_GAIN   (9024)

extern struct class* get_camera_class(void);
static struct class *camera_ois_hall_class = NULL;
static struct ois_timer ois_timer_t;
static void msm_ois_read_work(struct work_struct *work);
static bool msm_ois_data_enqueue(uint64_t x_readout_time, int32_t x_shift,
		uint64_t y_readout_time, int32_t y_shift, struct msm_ois_readout_buffer *o_buf);
static int msm_startGyroThread(struct cam_ois_ctrl_t *o_ctrl);
extern int msm_stopGyroThread(void);
extern int parse_ois_userdata(struct cam_cmd_ois_userdata *ois_userdata,
		struct cam_ois_ctrl_t *o_ctrl);

static struct class *ois_aat_result_class = NULL;
struct device*	ois_aat_result1;
struct device*	ois_aat_result2;
static char aat_selftest_result[PROPERTY_VALUE_MAX] = "000000000000";
static char aat_vendor_name[PROPERTY_VALUE_MAX] = "UNKNOWN";

extern uint8_t F40_FlashDownload(uint8_t chiperase, uint8_t ModuleVendor, uint8_t ActVer);
extern uint8_t LGMC_GyroOffset_ReCalibration(void);

static ssize_t show_ois_aat_selftest_result(struct device *dev, struct device_attribute *attr, char *buf)
{
	CAM_ERR(CAM_OIS, "show_ois_aat_selftest_result: [%s] \n", aat_selftest_result);
	return sprintf(buf, "%s\n", aat_selftest_result);
}
static DEVICE_ATTR(ois_aat_selftest_result, S_IRUGO, show_ois_aat_selftest_result, NULL);

static ssize_t show_ois_aat_vendor_name(struct device *dev,struct device_attribute *attr, char *buf)
{
	CAM_ERR(CAM_OIS, "show_ois_aat_vendor_name: [%s]\n", aat_vendor_name);
	return sprintf(buf, "%s\n", aat_vendor_name);
}
static DEVICE_ATTR(ois_aat_vendor_name, S_IRUGO, show_ois_aat_vendor_name, NULL);

static struct cam_ois_ctrl_t *local_cam_ois_t;

int sunny_imx363_onsemi_ois_poll_ready(int limit)
{
	uint32_t ois_status = 0x01;
	int read_byte = 0;
	int rc = OIS_SUCCESS;

	while ((ois_status != 0x00) && (read_byte < limit)) {
		rc = RamRead32A(OIS_READ_STATUS_ADDR, &ois_status); /* polling status ready */
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS_I2C_ERROR");
			return OIS_INIT_I2C_ERROR;
		}
		usleep_range(5* 1000, 5* 1000 + 10); /* wait 5ms */
		read_byte++;
	}

	if (ois_status == 0x00) {
		return OIS_SUCCESS;
	}
	else {
		CAM_ERR(CAM_OIS, "OIS_TIMEOUT_ERROR");
		return OIS_INIT_TIMEOUT;
	}

	return 0;
}

static ssize_t show_ois_hall(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;

	if(ois_timer_t.ois_timer_state == OIS_TIME_ACTIVE) {
		// Put Hall data
		struct msm_ois_readout gyro_data[MAX_GYRO_QUERY_SIZE];
		uint8_t query_size = MAX_GYRO_QUERY_SIZE;
		uint8_t data_get_count = 0;
		uint16_t counter = 0;
		int i;

		memset(gyro_data, 0, sizeof(gyro_data));

		spin_lock(&local_cam_ois_t->gyro_lock);

		counter = local_cam_ois_t->buf.buffer_tail;
		for (i = query_size-1; i >= 0; i-- )
		{
			counter = (counter == 0) ? MSM_OIS_DATA_BUFFER_SIZE-1 : counter-1;
			gyro_data[i].ois_x_shift = local_cam_ois_t->buf.buffer[counter].ois_x_shift;
			gyro_data[i].ois_y_shift = local_cam_ois_t->buf.buffer[counter].ois_y_shift;
			gyro_data[i].x_readout_time = local_cam_ois_t->buf.buffer[counter].x_readout_time;
			gyro_data[i].y_readout_time = local_cam_ois_t->buf.buffer[counter].y_readout_time;
			data_get_count++;
		}
		spin_unlock(&local_cam_ois_t->gyro_lock);

		if (data_get_count != 0 && data_get_count < MAX_GYRO_QUERY_SIZE + 1) {
			for(i = 0; i < data_get_count; i++) {
				count += sprintf(buf + strlen(buf),"%d,%016llx,%d,%016llx,\n",gyro_data[i].ois_x_shift, gyro_data[i].x_readout_time, gyro_data[i].ois_y_shift, gyro_data[i].y_readout_time);
			}
		}

		// Check Gyro data count
		CAM_DBG(CAM_OIS, "[OIS]:data_get_count = %d count : %d", data_get_count,count);
	}
	else {
		CAM_ERR(CAM_OIS, "OIS is not working");
	}

	return count;
}
static DEVICE_ATTR(ois_hall, S_IRUGO, show_ois_hall, NULL);

/*Get OIS gyro data via i2c.*/
static void msm_ois_read_work(struct work_struct *work)
{
	int32_t rc = 0;
	bool result;
	uint8_t buf[6] = {0,};
	uint64_t x_readout_time;
	uint64_t y_readout_time;
	int32_t x_shift = 0;
	int32_t y_shift = 0;
	struct ois_timer *ois_timer_in_t = NULL;

	ois_timer_in_t = container_of(work, struct ois_timer, g_work);

	// Read Hall data sequencelly (Timestamp -> Hall X & Y)
	rc = ois_i2c_read_seq(OIS_HALL_SEQ_ADDR, buf, 6);
	x_readout_time = y_readout_time  = arch_counter_get_cntvct();

	x_shift = (int32_t)((int16_t)((buf[0] << 8) | (buf[1])));
	y_shift = (int32_t)((int16_t)((buf[2] << 8) | (buf[3])));

	x_shift = x_shift * 32767 /
        (ois_timer_in_t->o_ctrl->gyro_gain_x > 0 ? ois_timer_in_t->o_ctrl->gyro_gain_x : OIS_GYRO_DEFAULT_GAIN);
	y_shift = y_shift * 32767 /
        (ois_timer_in_t->o_ctrl->gyro_gain_y > 0 ? ois_timer_in_t->o_ctrl->gyro_gain_y : OIS_GYRO_DEFAULT_GAIN);

	if (rc != 0) {
		ois_timer_t.i2c_fail_count++;
		CAM_ERR(CAM_OIS, "[OIS] %s : i2c_read_seq fail. cnt = %d",
				__func__, ois_timer_t.i2c_fail_count);
		if (ois_timer_t.i2c_fail_count == MAX_FAIL_CNT) {
			CAM_ERR(CAM_OIS, "[OIS] %s : Too many i2c failed. Stop timer.",
					__func__);
			ois_timer_t.ois_timer_state = OIS_TIME_ERROR;
		}
	} else {
		ois_timer_t.i2c_fail_count = 0;
		result = msm_ois_data_enqueue(x_readout_time, x_shift,
				y_readout_time, y_shift, &ois_timer_in_t->o_ctrl->buf);

		if (!result)
			CAM_ERR(CAM_OIS, "%s %d ois data enqueue ring buffer failed",
					__func__, __LINE__);
	}
}

static bool msm_ois_data_enqueue(uint64_t x_readout_time,
		int32_t x_shift,
		uint64_t y_readout_time,
		int32_t y_shift,
		struct msm_ois_readout_buffer *o_buf)
{
	bool rc;

	spin_lock(&local_cam_ois_t->gyro_lock);

	if (o_buf->buffer_tail >= 0 && o_buf->buffer_tail <
			MSM_OIS_DATA_BUFFER_SIZE) {
		o_buf->buffer[o_buf->buffer_tail].ois_x_shift = x_shift;
		o_buf->buffer[o_buf->buffer_tail].ois_y_shift = y_shift;
		o_buf->buffer[o_buf->buffer_tail].x_readout_time = x_readout_time;
		o_buf->buffer[o_buf->buffer_tail].y_readout_time = y_readout_time;

		o_buf->buffer_tail++;
		if (o_buf->buffer_tail >= MSM_OIS_DATA_BUFFER_SIZE)
			o_buf->buffer_tail -= MSM_OIS_DATA_BUFFER_SIZE;

		rc = true;
	} else {
		rc = false;
	}

	spin_unlock(&local_cam_ois_t->gyro_lock);

	return rc;
}

static enum hrtimer_restart msm_gyro_timer(struct hrtimer *timer)
{
	ktime_t currtime, interval;
	struct ois_timer *ois_timer_in_t;

	ois_timer_in_t = container_of(timer, struct ois_timer, hr_timer);
	if ((ois_timer_in_t->o_ctrl->cam_ois_state >= CAM_OIS_CONFIG)
			&& (ois_timer_t.ois_timer_state != OIS_TIME_ERROR)) {
		queue_work(ois_timer_in_t->ois_wq, &ois_timer_in_t->g_work);
		currtime  = ktime_get();
		interval = ktime_set(0, READ_OUT_TIME);
		hrtimer_forward(timer, currtime, interval);

		return HRTIMER_RESTART;
	} else {
		CAM_ERR(CAM_OIS, "[OIS] %s HRTIMER_NORESTART ois_state : %d timer_state : %d",
				__func__, ois_timer_in_t->o_ctrl->cam_ois_state, ois_timer_t.ois_timer_state);
		return HRTIMER_NORESTART;
	}
}

static int msm_startGyroThread(struct cam_ois_ctrl_t *o_ctrl)
{
	ktime_t  ktime;

	CAM_INFO(CAM_OIS, "[OIS] %s:E", __func__);

	if (ois_timer_t.ois_timer_state == OIS_TIME_ERROR) {
		CAM_ERR(CAM_OIS, "[OIS] %s:Timer error, close befoe create :%d.",
				__func__, ois_timer_t.ois_timer_state);
		msm_stopGyroThread();
	}

	ois_timer_t.i2c_fail_count = 0;
	if (ois_timer_t.ois_timer_state != OIS_TIME_ACTIVE) {
		ois_timer_t.o_ctrl = o_ctrl;
        INIT_WORK(&ois_timer_t.g_work, msm_ois_read_work);
		ois_timer_t.ois_wq = create_workqueue("ois_wq");
		if (!ois_timer_t.ois_wq) {
			CAM_ERR(CAM_OIS, "[OIS]:%s ois_wq create failed.", __func__);
			return -EFAULT;
		}
		ktime = ktime_set(0, READ_OUT_TIME);
		hrtimer_init(&ois_timer_t.hr_timer, CLOCK_MONOTONIC,
				HRTIMER_MODE_REL);
		ois_timer_t.hr_timer.function = &msm_gyro_timer;
		hrtimer_start(&ois_timer_t.hr_timer, ktime,
				HRTIMER_MODE_REL);
		ois_timer_t.ois_timer_state = OIS_TIME_ACTIVE;
	} else
		CAM_ERR(CAM_OIS, "[OIS] invalid timer state = %d.",
				ois_timer_t.ois_timer_state);
	CAM_DBG(CAM_OIS, "[OIS] %s:X", __func__);
	return 0;
}
extern int msm_stopGyroThread(void)
{
	CAM_INFO(CAM_OIS, "[OIS] %s:E", __func__);

	if ((ois_timer_t.ois_timer_state == OIS_TIME_ACTIVE) ||
			(ois_timer_t.ois_timer_state == OIS_TIME_ERROR)) {
		CAM_INFO(CAM_OIS, "[OIS] %s:timer cancel.", __func__);
		hrtimer_cancel(&ois_timer_t.hr_timer);
		destroy_workqueue(ois_timer_t.ois_wq);
		ois_timer_t.ois_timer_state = OIS_TIME_INACTIVE;
	} else
		CAM_ERR(CAM_OIS, "[OIS] invalid timer state = %d",
				ois_timer_t.ois_timer_state);
	CAM_DBG(CAM_OIS, "[OIS] %s:X", __func__);
	return 0;
}

extern int parse_ois_userdata(
        struct cam_cmd_ois_userdata *ois_userdata,
        struct cam_ois_ctrl_t *o_ctrl)
{
    struct cam_ois_ctrl_t *ctrl = o_ctrl;
    int rc = -1;

    switch(ois_userdata->select)
    {
        case HALL_FEEDING:
            if(ois_userdata->oisThreadNeeded == 1) {
                rc = msm_startGyroThread(ctrl);
                ctrl->ois_thread_running = true;
            } else {
                rc = msm_stopGyroThread();
                ctrl->ois_thread_running = false;
            }
            break;
        default:
            break;
    }

    return rc;
}

int32_t sunny_imx363_init_set_onsemi_ois(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t rc = OIS_SUCCESS;
	uint32_t m_fw_ver = 0;
	uint32_t m_act_ver = 0;
	uint8_t gyro_gain_x[2];
	uint8_t gyro_gain_y[2];
	local_cam_ois_t = o_ctrl;

	RamRead32A(0x8000, &m_fw_ver);
	RamRead32A(0x8004, &m_act_ver);

	CAM_ERR(CAM_OIS, "Enter, %s, M_FW_VER 0x%x V_VER 0x%x", LAST_UPDATE, m_fw_ver, m_act_ver);

	if (o_ctrl->is_ois_aat) {
#ifdef CONFIG_MACH_SM8150_BETA
		if (FW_VER != m_fw_ver){
			CAM_ERR(CAM_OIS, "OIS F/W update start");
			rc = F40_FlashDownload(0x01, 0x00, 0x01);
			if (rc) {
				CAM_ERR(CAM_OIS, "OIS F/W update fail rc 0x%x", rc);
			}
			usleep_range(35 * 1000, 35 * 1000 + 10);
			CAM_ERR(CAM_OIS, "OIS F/W update end");
		}
#endif
		CAM_ERR(CAM_OIS, "OIS_VER_calibration");
		usleep_range(20 * 1000, 20 * 1000 + 10);
		LGMC_GyroOffset_ReCalibration();
		ois_selftest();
		ois_selftest2();
	} else {
		// Read Gyro Gain
		ois_i2c_read_seq(OIS_GYRO_GAIN_X, gyro_gain_x, 2);
		local_cam_ois_t->gyro_gain_x = (gyro_gain_x[0] << 8 | gyro_gain_x[1]);
		ois_i2c_read_seq(OIS_GYRO_GAIN_Y, gyro_gain_y, 2);
		local_cam_ois_t->gyro_gain_y = (gyro_gain_y[0] << 8 | gyro_gain_y[1]);

		if(local_cam_ois_t->gyro_gain_x == 0 ||
				local_cam_ois_t->gyro_gain_x == 0xFFFF) {
			local_cam_ois_t->gyro_gain_x = OIS_GYRO_DEFAULT_GAIN;
		}

		if(local_cam_ois_t->gyro_gain_y == 0 ||
				local_cam_ois_t->gyro_gain_y == 0xFFFF) {
			local_cam_ois_t->gyro_gain_y = OIS_GYRO_DEFAULT_GAIN;
		}

	    CAM_ERR(CAM_OIS, " gain %d %d",
	        local_cam_ois_t->gyro_gain_x, local_cam_ois_t->gyro_gain_y);
	}
	CAM_ERR(CAM_OIS, "init done");
	return rc;
}

extern void oeis_create_sysfs(void) {
	struct device*  camera_ois_hall_dev;
	if(!camera_ois_hall_class) {
		CAM_INFO(CAM_OIS, "create ois_hall_class!!");
		camera_ois_hall_class = get_camera_class();
		camera_ois_hall_dev   = device_create(camera_ois_hall_class, NULL,
				0, NULL, "ois_hall");
		device_create_file(camera_ois_hall_dev, &dev_attr_ois_hall);
	}
}

extern void oeis_destroy_sysfs(void) {
	if(camera_ois_hall_class) {
		class_destroy(camera_ois_hall_class);
		camera_ois_hall_class = NULL;
	}
}

void msm_ois_create_sysfs(void){
	if(!ois_aat_result_class){
	  CAM_ERR(CAM_OIS, "create ois_aat_result_class!!");
      ois_aat_result_class = class_create(THIS_MODULE, "ois");
      ois_aat_result1 = device_create(ois_aat_result_class, NULL,0, NULL, "ois_aat_selftest_result");
      device_create_file(ois_aat_result1, &dev_attr_ois_aat_selftest_result);
      ois_aat_result2 = device_create(ois_aat_result_class, NULL,0, NULL, "ois_aat_vendor_name");
      device_create_file(ois_aat_result2, &dev_attr_ois_aat_vendor_name);
	}
}

void msm_ois_destroy_sysfs(void){
	if(ois_aat_result_class){
        device_remove_file(ois_aat_result1, &dev_attr_ois_aat_selftest_result);
        device_remove_file(ois_aat_result2, &dev_attr_ois_aat_vendor_name);
		device_destroy(ois_aat_result_class, 0);
		device_destroy(ois_aat_result_class, 0);
		class_destroy(ois_aat_result_class);
	    ois_aat_result_class = NULL;
		CAM_ERR(CAM_OIS, "del ois_aat_result_class!!");
	}
}

int32_t sunny_imx363_onsemi_ois_stat(sensor_ois_stat_t *data)
{
	sensor_ois_stat_t ois_stat;
	uint32_t val_gyro_x, val_gyro_y;
	uint32_t val_gyro_offset_x, val_gyro_offset_y;

	memset(&ois_stat, 0, sizeof(ois_stat));
	snprintf(ois_stat.ois_provider, ARRAY_SIZE(ois_stat.ois_provider), "SUNNY_ONSEMI");

	/* Gyro Read by reg */
	RamRead32A(0x258, &val_gyro_x);
	RamRead32A(0x25C, &val_gyro_y);
	RamRead32A(0x278, &val_gyro_offset_x);
	RamRead32A(0x27C, &val_gyro_offset_y);

	ois_stat.gyro[0] = (int16_t)(val_gyro_x >> 16);
	ois_stat.gyro[1] = (int16_t)(val_gyro_y >> 16);
	ois_stat.offset[0] = (int16_t)(val_gyro_offset_x >> 16);
	ois_stat.offset[1] = (int16_t)(val_gyro_offset_y >> 16);
	ois_stat.is_stable = 1;

	*data = ois_stat;

	CAM_ERR(CAM_OIS,"gyro x %d gyro y %d offset x %d offset y %d",
		ois_stat.gyro[0], ois_stat.gyro[1], ois_stat.offset[0], ois_stat.offset[1]);

	return 0;
}

int32_t sunny_imx363_onsemi_ois_move_lens(void *data)
{
	int32_t hallx = 0;
	int32_t hally = 0;
	int32_t rHallx = 0;
	int32_t rHally = 0;
	int16_t offset[2];

	memcpy(offset, data, sizeof(offset));
	CAM_ERR(CAM_OIS, "target : %d, %d", offset[0], offset[1]);

	hallx =  offset[0] << 16;
	hally =  offset[1] << 16;

	RamWrite32A(0x114, hallx);
	RamWrite32A(0x164, hally);
	usleep_range(100000, 100010);

	RamRead32A(0x118, (uint32_t *)&rHallx);
	RamRead32A(0x168, (uint32_t *)&rHally);

	CAM_ERR(CAM_OIS, "read : %d(0x%x), %d(0x%x)",
		(rHallx >> 16), (rHallx >> 16), (rHally >> 16), (rHally >> 16));

	if ((abs((rHallx >> 16) - offset[0]) < HALL_LIMIT) && (abs((rHally >> 16) - offset[1]) < HALL_LIMIT))
		return  OIS_SUCCESS;

	CAM_ERR(CAM_OIS, "move fail rHallx: %d, rHally: %d",
		(rHallx >> 16), (rHally >> 16));
	return OIS_FAIL;
}

/*===========================================================================
 * FUNCTION    - ois_selftest_get -
 *
 * DESCRIPTION: ois self-test routine for all-auto-test
 *==========================================================================*/
static uint32_t ois_selftest_get(void)
{
	uint32_t out = 0;
	int i;

	for (i = 0; i < 12; i++)
		if (aat_selftest_result[11 - i] == '1')
			out |= (1 << i);

	CAM_ERR(CAM_OIS," result = %x %s", out, aat_selftest_result);
	return out;
}

/*===========================================================================
 * FUNCTION    - ois_selftest_set -
 *
 * DESCRIPTION: ois self-test routine for all-auto-test
 *==========================================================================*/
static void ois_selftest_set(uint32_t result)
{
	uint8_t rc[12];
	int i;

	for (i=0; i < 12; i++)
		rc[i] = (result & (1 << i)) >> i;

	sprintf(aat_selftest_result, "%d%d%d%d%d%d%d%d%d%d%d%d", rc[11], rc[10], rc[9], rc[8], rc[7],
		rc[6], rc[5], rc[4], rc[3], rc[2], rc[1], rc[0]);

	CAM_ERR(CAM_OIS," result = %x %s",result, aat_selftest_result);
}

/*===========================================================================
 * FUNCTION    - ois_selftest -
 *
 * DESCRIPTION: ois self-test routine for all-auto-test
 *==========================================================================*/
#define GYRO_OFFSET_LIMIT 27   // 26.717[dps]*131
#define GYRO_SCALE_FACTOR 131

#define GYRO_X_VALUE ois_stat.gyro[0] / GYRO_SCALE_FACTOR
#define GYRO_Y_VALUE ois_stat.gyro[1] / GYRO_SCALE_FACTOR
#define CAL_GYRO_X_VALUE (ois_stat.gyro[0] - ois_stat.offset[0]) / GYRO_SCALE_FACTOR
#define CAL_GYRO_Y_VALUE (ois_stat.gyro[1] - ois_stat.offset[1]) / GYRO_SCALE_FACTOR

#define  UNSTABLE_RATIO 40 //0.4 * 100

static int8_t ois_selftest(void)
{
	sensor_ois_stat_t ois_stat;
	uint32_t result = 0;
	int i = 0;
	int16_t hall_target[][2] = {
	{9330, 0}, {6596, 6596}, {0, 9330},
	{-6596, 6596}, {-9330, 0}, {-6596, -6596},
	{0, -9330}, {6596, -6596}, {9330, 0}
	};

	memset(&ois_stat, 0, sizeof(ois_stat));

	CAM_ERR(CAM_OIS, " enter");
	//0. is ois init success ?
	result |= (1 << 11); // <- init success.

	//1. get first stat value.
	sunny_imx363_onsemi_ois_stat(&ois_stat);

	//2. check ois module is alive.
	if (ois_stat.gyro[0] == 0 && ois_stat.gyro[1] == 0 && ois_stat.offset[0] == 0 && ois_stat.offset[1] == 0) goto END;

	//3. check gyro initial dps.
	if (abs(GYRO_X_VALUE) <= GYRO_OFFSET_LIMIT) {
		result |= (1 << 10);
		CAM_ERR(CAM_OIS, "(GYRO_X_VALUE) <= GYRO_OFFSET_LIMIT)");
	}
	else {
		CAM_ERR(CAM_OIS, "spec over! (GYRO_X_VALUE %d) > GYRO_OFFSET_LIMIT)", GYRO_X_VALUE);
	}
	if (abs(GYRO_Y_VALUE) <= GYRO_OFFSET_LIMIT) {
		result |= (1 << 9);
		CAM_ERR(CAM_OIS, "(GYRO_Y_VALUE) <= GYRO_OFFSET_LIMIT)");
	}
	else {
		CAM_ERR(CAM_OIS, "spec over! (GYRO_Y_VALUE %d) > GYRO_OFFSET_LIMIT)", GYRO_Y_VALUE);
	}

	//4. check lens movement range
	for (i = 0; i <= 8; i++) {
		if (sunny_imx363_onsemi_ois_move_lens(&hall_target[i])>= 0)
			result |= (1 << (8 - i));
	}

	//5. reset lens position before ois turn-on.
	{
		uint16_t offset[2] = {0, 0};
		sunny_imx363_onsemi_ois_move_lens(&offset);
	}

	END:
	strcpy(aat_vendor_name, ois_stat.ois_provider);
	ois_selftest_set(result);
	CAM_ERR(CAM_OIS," exit");
	return OIS_SUCCESS;
}

/*===========================================================================
 * FUNCTION    - ois_selftest2 -
 *
 * DESCRIPTION: ois self-test routine #2 for all-auto-test
 *==========================================================================*/
static int8_t ois_selftest2(void)
{
	sensor_ois_stat_t ois_stat;
	int i;
	int unstable = 0;
	uint32_t result = ois_selftest_get();
	int32_t rc = 0;

	memset(&ois_stat, 0, sizeof(ois_stat));

	CAM_ERR(CAM_OIS," enter");

	// if ois init was failed, skip test
	if (result == 0) {
		CAM_ERR(CAM_OIS,"ois init was failed, skip test");
		goto END;
	}

	//check gyro validity, servo stability at least 100 times (~ about 250ms)
	for (i = 0; i < 100; i++) {
		rc = sunny_imx363_onsemi_ois_stat(&ois_stat);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,"i:%d ois_get_stat error", i);
			ois_stat.is_stable = 0;
		}
		if (abs(GYRO_X_VALUE) > GYRO_OFFSET_LIMIT) {
			CAM_ERR(CAM_OIS," %d spec over GYRO_X_VALUE %d", i, GYRO_X_VALUE);
			result &= ~(1 << 10);
			ois_stat.is_stable = 0;
		}
		if (abs(GYRO_Y_VALUE) > GYRO_OFFSET_LIMIT) {
			CAM_ERR(CAM_OIS," %d spec over GYRO_Y_VALUE %d", i, GYRO_Y_VALUE);
			result &= ~(1 << 9);
			ois_stat.is_stable = 0;
		}
		if (!ois_stat.is_stable)
			unstable++;
	}

	if (unstable > UNSTABLE_RATIO)
		result &= ~(1 << 11);

	CAM_ERR(CAM_OIS," unstable -> %d", unstable);

	END:
	ois_selftest_set(result);
	CAM_ERR(CAM_OIS," exit");
	return OIS_SUCCESS;
}

int32_t ois_i2c_read_seq(uint32_t addr, uint8_t *data, uint16_t num_byte)
{
	int32_t ret = 0;

	ret = camera_io_dev_read_seq(
		&(local_cam_ois_t->io_master_info),
		addr,
		data,
		CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		num_byte);

	return ret;
}

int32_t RamRead32A(uint32_t RamAddr, uint32_t *ReadData)
{
	int32_t ret = 0;
	uint8_t buf[4];

	ret = camera_io_dev_read_seq(
		&(local_cam_ois_t->io_master_info),
		RamAddr,
		buf,
		CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		4);
	*ReadData = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
	return ret;
}

int32_t RamWrite32A(uint32_t RamAddr, uint32_t RamData)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	int32_t ret = 0;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * 1, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL)
	{
		CAM_ERR(CAM_OIS,"kzalloc failed");
		ret = OIS_FAIL;
		return ret;
	}
	i2c_reg_setting.reg_setting->reg_addr = RamAddr;
	i2c_reg_setting.reg_setting->reg_data = RamData;
	i2c_reg_setting.reg_setting->delay = 0;
	i2c_reg_setting.reg_setting->data_mask = 0;

	ret = camera_io_dev_write(
		&(local_cam_ois_t->io_master_info),
		&i2c_reg_setting);
	kfree(i2c_reg_setting.reg_setting);
	return ret;
}

int32_t CntWrt(uint8_t *data, uint16_t num_byte)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	uint16_t cnt;
	uint8_t *ptr = NULL;
	int32_t ret = 0;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = num_byte - 1;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * num_byte - 1, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL)
	{
		CAM_ERR(CAM_OIS, "kzalloc failed");
		return -EINVAL;
	}

	for (cnt = 0, ptr = (uint8_t *)data + 1; cnt < num_byte - 1; cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr = data[0];
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	ret = camera_io_dev_write_continuous(&(local_cam_ois_t->io_master_info),
		&i2c_reg_setting, 1);

	if (ret < 0) {
		CAM_ERR(CAM_OIS, "CntWrt fail %d", ret);
	}

	kfree(i2c_reg_setting.reg_setting);
	return ret;
}

void WitTim(unsigned short	UsWitTim)
{
	usleep_range(1000 * UsWitTim, 1000 * UsWitTim + 10); /* wait [UsWitTim]ms */
}
