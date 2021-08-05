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
#include "lgit_imx363_onsemi_ois.h"
#include "onsemi_ois.h"
#include "LC898124EP1_Host_Code0_1.h"	// INVN_ICG-1020S +	LGIT_N3
#include "LC898124EP1_Host_Code0_2.h"	// INVN_ICG-1020S +	LGIT_N3V02
#include <linux/device.h>
#include <linux/spinlock.h>
#include <asm/arch_timer.h>

#define LAST_UPDATE "18-10-11, OIS LC898124EP1"

#define LIMIT_STATUS_POLLING    (15)
#define LIMIT_OIS_ON_RETRY      (5)
#define OIS_READ_STATUS_ADDR    (0xF100)

#define E2P_SID                 (0xA8)
#define MODULE_FW_VER_ADDR      (0x1870)
#define VCM_VER_ADDR            (0xEF1)

#define GYROSELECT              (0x00)
#define HALL_LIMIT              (1219)

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

stAdjPar	StAdjPar; // temporary buffer for caribration data

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

int lgit_imx363_onsemi_ois_poll_ready(int limit)
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

//****************************************************
//	LOCAL RAM LIST
//****************************************************
#define BURST_LENGTH_PM (12*5)
#define BURST_LENGTH_DM (10*6)
#define BURST_LENGTH BURST_LENGTH_PM

void MemoryClear(uint16_t UsSourceAddress, uint16_t UsClearSize)
{
    uint16_t UsLoopIndex;

    for (UsLoopIndex = 0; UsLoopIndex < UsClearSize; UsLoopIndex += 4) {
        RamWrite32A(UsSourceAddress + UsLoopIndex, 0x00000000);				// 4Byte
        //TRACE("MEM CLR ADR = %04xh \n",UsSourceAddress + UsLoopIndex);
	}
}

void SetTransDataAdr(uint16_t UsLowAddress , uint32_t UlLowAdrBeforeTrans)
{
	UnDwdVal StTrsVal;

	if(UlLowAdrBeforeTrans < 0x00009000){
		StTrsVal.UlDwdVal = UlLowAdrBeforeTrans;
	}else{
		StTrsVal.StDwdVal.UsHigVal = (uint16_t)((UlLowAdrBeforeTrans & 0x0000F000) >> 8);
		StTrsVal.StDwdVal.UsLowVal = (uint16_t)(UlLowAdrBeforeTrans & 0x00000FFF);
	}
//TRACE(" TRANS  ADR = %04xh , DAT = %08xh \n",UsLowAddress , StTrsVal.UlDwdVal);
	RamWrite32A(UsLowAddress, StTrsVal.UlDwdVal);
}

void ClrMesFil(void)
{
	RamWrite32A (MeasureFilterA_Delay_z11, 0);
	RamWrite32A (MeasureFilterA_Delay_z12, 0);

	RamWrite32A (MeasureFilterA_Delay_z21, 0);
	RamWrite32A (MeasureFilterA_Delay_z22, 0);

	RamWrite32A (MeasureFilterB_Delay_z11, 0);
	RamWrite32A (MeasureFilterB_Delay_z12, 0);

	RamWrite32A (MeasureFilterB_Delay_z21, 0);
	RamWrite32A (MeasureFilterB_Delay_z22, 0);
}

void SetWaitTime(uint16_t UsWaitTime)
{
	RamWrite32A(WaitTimerData_UiWaitCounter, 0);
	RamWrite32A(WaitTimerData_UiTargetCount, (uint32_t)(ONE_MSEC_COUNT * UsWaitTime));
}

void MeasureStart(int32_t SlMeasureParameterNum , uint32_t SlMeasureParameterA , uint32_t SlMeasureParameterB)
{
	MemoryClear(StMeasFunc_SiSampleNum , sizeof(MeasureFunction_Type));
	RamWrite32A(StMeasFunc_MFA_SiMax1, 0x80000000);					// Set Min
	RamWrite32A(StMeasFunc_MFB_SiMax2, 0x80000000);					// Set Min
	RamWrite32A(StMeasFunc_MFA_SiMin1, 0x7FFFFFFF);					// Set Max
	RamWrite32A(StMeasFunc_MFB_SiMin2, 0x7FFFFFFF);					// Set Max

	SetTransDataAdr(StMeasFunc_MFA_PiMeasureRam1, SlMeasureParameterA);		// Set Measure Filter A Ram Address
	SetTransDataAdr(StMeasFunc_MFB_PiMeasureRam2, SlMeasureParameterB);		// Set Measure Filter B Ram Address
	RamWrite32A(StMeasFunc_SiSampleNum, 0);													// Clear Measure Counter
	ClrMesFil();						// Clear Delay Ram
//	SetWaitTime(50);
	SetWaitTime(1);
	RamWrite32A(StMeasFunc_SiSampleMax, SlMeasureParameterNum);						// Set Measure Max Number
}

void MeasureWait(void)
{
	uint32_t SlWaitTimerSt;

	SlWaitTimerSt = 1;
	while(SlWaitTimerSt){
		RamRead32A(StMeasFunc_SiSampleMax , &SlWaitTimerSt);
	}
}

void MesFil(void)//(uint8_t	UcMesMod)		// 20.019kHz
{
	uint32_t UlMeasFilaA, UlMeasFilaB, UlMeasFilaC;
	uint32_t UlMeasFilbA, UlMeasFilbB, UlMeasFilbC;

	UlMeasFilaA	= 0x7FFFFFFF;	// Through
	UlMeasFilaB	= 0x00000000;
	UlMeasFilaC	= 0x00000000;
	UlMeasFilbA	= 0x7FFFFFFF;	// Through
	UlMeasFilbB	= 0x00000000;
	UlMeasFilbC	= 0x00000000;

	RamWrite32A (MeasureFilterA_Coeff_a1, UlMeasFilaA);
	RamWrite32A (MeasureFilterA_Coeff_b1, UlMeasFilaB);
	RamWrite32A (MeasureFilterA_Coeff_c1, UlMeasFilaC);

	RamWrite32A (MeasureFilterA_Coeff_a2, UlMeasFilbA);
	RamWrite32A (MeasureFilterA_Coeff_b2, UlMeasFilbB);
	RamWrite32A (MeasureFilterA_Coeff_c2, UlMeasFilbC);

	RamWrite32A (MeasureFilterB_Coeff_a1, UlMeasFilaA);
	RamWrite32A (MeasureFilterB_Coeff_b1, UlMeasFilaB);
	RamWrite32A (MeasureFilterB_Coeff_c1, UlMeasFilaC);

	RamWrite32A (MeasureFilterB_Coeff_a2, UlMeasFilbA);
	RamWrite32A (MeasureFilterB_Coeff_b2, UlMeasFilbB);
	RamWrite32A (MeasureFilterB_Coeff_c2, UlMeasFilbC);
}

void DMIOWrite32(uint32_t IOadrs, uint32_t IOdata)
{
#if 0
	uint8_t data[10];
	data[0] = 0xC0;		// Pmem address set
	data[1] = 0x00;		// Command High
	data[2] = (uint8_t)(IOadrs >>24);		// IOadres
	data[3] = (uint8_t)(IOadrs >>16);		// Command High
	data[4] = (uint8_t)(IOadrs >> 8);		// Command High
	data[5] = (uint8_t)(IOadrs >> 0);		// Command High
	data[6] = (uint8_t)(IOdata >>24);		// IOadres
	data[7] = (uint8_t)(IOdata >>16);		// Command High
	data[8] = (uint8_t)(IOdata >> 8);		// Command High
	data[9] = (uint8_t)(IOdata >> 0);		// Command High
	CntWrt(data, 10); 	// I2C 1Byte address.
#else
	RamWrite32A(CMD_IO_ADR_ACCESS, IOadrs);
	RamWrite32A(CMD_IO_DAT_ACCESS, IOdata);
#endif
};

unsigned char DownloadToEP1_Ver2(const uint8_t* DataPM, uint32_t LengthPM,
	uint32_t Parity, const uint8_t* DataDM, uint32_t LengthDM)
{
	uint32_t i, j;
	uint8_t data[64];		// work fifo buffer max size 64 byte
	uint16_t Remainder;
	uint32_t UlReadVal, UlCnt, ver;
	uint32_t ReadVerifyPM, ReadVerifyDM;	// Checksum

//--------------------------------------------------------------------------------
// 0. Start up to boot exection
//--------------------------------------------------------------------------------
	RamWrite32A(CMD_IO_ADR_ACCESS , ROMINFO);
	RamRead32A(CMD_IO_DAT_ACCESS, &UlReadVal);
	RamRead32A(0x8000, &ver);

	CAM_ERR(CAM_OIS, "program execution value 0x%x, DSP code info 0x%x", UlReadVal, ver);
	switch ((uint8_t)UlReadVal){
	case 0x0A:	/* Normal Rom program execution */
		break;
	case 0x01:	/* Normal Ram program execution */
		DMIOWrite32(SYSDSP_REMAP, 0x00001000); 	// CORE_RST
		WitTim(6);
		break;

//	case 0x0B:
//	case 0x08:
	default:
		return(1);
	}
//--------------------------------------------------------------------------------
// 1. Download Program
//--------------------------------------------------------------------------------
	data[0] = 0x30;		// Pmem address set
	data[1] = 0x00;		// Command High
	data[2] = 0x10;		// Command High
	data[3] = 0x00;		// Command High
	data[4] = 0x00;		// Command High
	CntWrt(data, 5); 	// I2C 1Byte address.
	// program start
	data[0] = 0x40;		// Pmem address set
	Remainder = ((LengthPM*5) / BURST_LENGTH_PM);
	for(i=0; i< Remainder; i++)
	{
		UlCnt = 1;
		for(j=0; j < BURST_LENGTH_PM; j++)	data[UlCnt++] = *DataPM++;

		CntWrt(data, BURST_LENGTH_PM+1);  // I2Caddresss 1Byte.
	}
	Remainder = ((LengthPM*5) % BURST_LENGTH_PM);
	if (Remainder != 0)
	{
		UlCnt = 1;
		for(j=0; j < Remainder; j++)	data[UlCnt++] = *DataPM++;
//TRACE("Remainder %d \n", (uint16_t)Remainder);
		CntWrt(data, UlCnt);  // I2C 1Byte address.
	}
	// Chercksum start
	data[0] = 0xF0;											// Pmem address set
	data[1] = 0x0A;											// Command High
	data[2] = (unsigned char)((LengthPM & 0xFF00) >> 8);	// Size High
	data[3] = (unsigned char)((LengthPM & 0x00FF) >> 0);	// Size Low
	CntWrt(data, 4); 	// I2C 2Byte addresss.
#if 0
	UlCnt=0;
	do{
		if(UlCnt++ > 10) break;						// 400kHzÇ≈2âÒñ⁄ÅB
		RamRead32A(PmCheck_EndFlag, &UlReadVal);
	}while (UlReadVal == 0);
	RamRead32A(PmCheck_CheckSum, &ReadVerifyPM);
#endif
//--------------------------------------------------------------------------------
// 2. Download Table Data
//--------------------------------------------------------------------------------
//TRACE("DM Start \n");
	RamWrite32A(DmCheck_CheckSumDMB, 0);		// DMB Parity Clear
	for(i=0; i < LengthDM; i+=6)
	{
		if ((DataDM[0+i] == 0x80) && (DataDM[1+i] == 0x0C))
		{
			RamWrite32A(CommandDecodeTable_08,
				(((uint32_t)(DataDM[3+i])<<16) + ((uint32_t)(DataDM[4+i])<<8) + DataDM[5+i]));
		}
	}
	// Data Send
	Remainder = ((LengthDM*6/4) / BURST_LENGTH_DM);
	for(i=0; i< Remainder; i++)
	{
		CntWrt((uint8_t*)DataDM, BURST_LENGTH_DM);  // I2Caddresss 1Byte.
		DataDM += BURST_LENGTH_DM;
	}
	Remainder = ((LengthDM*6/4) % BURST_LENGTH_DM);
	if (Remainder != 0)
	{
		CntWrt((uint8_t*)DataDM, (uint16_t)Remainder);  // I2Caddresss 1Byte.
//		DataDM += BURST_LENGTH_DM;
	}

//--------------------------------------------------------------------------------
// 3. Verify
//--------------------------------------------------------------------------------
	RamRead32A(PmCheck_CheckSum, &ReadVerifyPM);
	RamRead32A(DmCheck_CheckSumDMB, &ReadVerifyDM);

	RamWrite32A(CommandDecodeTable_08, 0x000418C4);

	if((ReadVerifyPM + ReadVerifyDM) != Parity){
		CAM_ERR(CAM_OIS, "error! %08x %08x",
			(unsigned int)ReadVerifyPM, (unsigned int)ReadVerifyDM);
		return OIS_INIT_DOWNLOAD_ERROR;
	}
	return(0);
}

void RemapMain(void)
{
	RamWrite32A(0xF000, 0x00000000);
}

void MonitorInfo(DSPVER* Dspcode)
{
#if 0
//TRACE("Vendor : %02x \n", Dspcode->Vendor);
//TRACE("User : %02x \n", Dspcode->User);
//TRACE("Model : %02x \n", Dspcode->Model);
//TRACE("Version : %02x \n", Dspcode->Version);

if(Dspcode->ActType == ACT_LGIT_N3	)
//TRACE("actuator type : ACT_LGIT_N3\n");

if(Dspcode->GyroType == GYRO_ICG1020S)
//TRACE("gyro type : INVEN ICG1020S \n");
#endif
}

uint8_t GetInfomationAfterDownload(DSPVER* Info)
{
	uint32_t Data;
	uint32_t UlReadVal;

	RamWrite32A(CMD_IO_ADR_ACCESS , ROMINFO);
	RamRead32A(CMD_IO_DAT_ACCESS, &UlReadVal);
	if((uint8_t)UlReadVal != 0x01) return(1);

	RamRead32A((SiVerNum + 0), &Data);
	Info->Vendor 	= (uint8_t)(Data >> 24);
	Info->User 		= (uint8_t)(Data >> 16);
	Info->Model 		= (uint8_t)(Data >> 8);
	Info->Version 	= (uint8_t)(Data >> 0);
	RamRead32A((SiVerNum + 4), &Data);
	Info->ActType =  (uint8_t)(Data >> 8);
	Info->GyroType = (uint8_t)(Data >> 0);

//	 MonitorInfo(Info);
	return(0);
}

uint8_t GetInfomationBeforeDownlaod(DSPVER* Info, const uint8_t* DataDM,  uint32_t LengthDM)
{
	uint32_t i;
	Info->ActType = 0;
	Info->GyroType = 0;

	for(i=0; i < LengthDM; i+=6)
	{
		if ((DataDM[0+i] == 0x80) && (DataDM[1+i] == 0x00))
		{
			Info->Vendor = DataDM[2+i];
			Info->User = DataDM[3+i];
			Info->Model = DataDM[4+i];
			Info->Version = DataDM[5+i];
			if ((DataDM[6+i] == 0x80) && (DataDM[7+i] == 0x04))
			{
				Info->ActType = DataDM[10+i];
				Info->GyroType = DataDM[11+i];
			}
			MonitorInfo(Info);
			return (0);
		}
	}
	return(1);
}

const DOWNLOAD_TBL DTbl[] = {
#if ((SELECT_VENDOR&0x04) == 0x04)
 {0x0100, LC898124EP1_PM0_1, LC898124EP1_PMSize0_1, (LC898124EP1_PMCheckSum0_1 + LC898124EP1_DMB_CheckSum0_1), LC898124EP1_DM0_1, LC898124EP1_DMB_ByteSize0_1},
 {0x0200, LC898124EP1_PM0_2, LC898124EP1_PMSize0_2, (LC898124EP1_PMCheckSum0_2 + LC898124EP1_DMB_CheckSum0_2), LC898124EP1_DM0_2, LC898124EP1_DMB_ByteSize0_2 },
#endif
 {0xFFFF, (void*)0, 0, 0, (void*)0 ,0}
};

int32_t lgit_imx363_init_set_onsemi_ois(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t rc = OIS_INIT;
	uint8_t ActSelect = 0;
	uint16_t m_fw_ver = 0;
	uint16_t vcm_ver = 0;
	DSPVER Dspcode;
	DOWNLOAD_TBL* ptr;
	uint8_t gyro_gain_x[2];
	uint8_t gyro_gain_y[2];

	local_cam_ois_t = o_ctrl;

	memset(&Dspcode, 0, sizeof(Dspcode));

	ois_i2c_e2p_read(MODULE_FW_VER_ADDR, &m_fw_ver, CAMERA_SENSOR_I2C_TYPE_BYTE);
	ois_i2c_e2p_read(VCM_VER_ADDR, &vcm_ver, CAMERA_SENSOR_I2C_TYPE_BYTE);

	CAM_ERR(CAM_OIS, "Enter, %s, S_VER 0x%x M_VER 0x%x V_VER 0x%x", LAST_UPDATE, FW_VER, m_fw_ver, vcm_ver);

	switch (vcm_ver) {
		case 0x02:
			CAM_ERR(CAM_OIS, "Selet Act 0x01");
			ActSelect = 0x01; // LGIT N3
			break;
		case 0x03:
			CAM_ERR(CAM_OIS, "Selet Act 0x02");
			ActSelect = 0x02; // LGIT N3V02
			break;
		default:
			CAM_ERR(CAM_OIS, "Apply Default");
			rc = -EFAULT;
			break;
	}

	ptr = (DOWNLOAD_TBL *)DTbl;
	while (ptr->Cmd != 0xFFFF){
		if(ptr->Cmd == (((uint16_t)ActSelect<<8) + GYROSELECT)) break;
		ptr++;
	}
	if (ptr->Cmd == 0xFFFF)	return(0xF0);

	if(GetInfomationBeforeDownlaod(&Dspcode, ptr->DataDM, ptr->LengthDM) != 0){
		return(0xF1);
	}

	if((ActSelect != Dspcode.ActType) || (GYROSELECT != Dspcode.GyroType)) return(0xF2);

	rc = DownloadToEP1_Ver2(ptr->DataPM, ptr->LengthPM, ptr->Parity, ptr->DataDM, ptr->LengthDM);
	if (rc)	{
		CAM_ERR(CAM_OIS, "init fail rc = %d", rc);
		return rc;
	}
	RemapMain();

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

	CAM_ERR(CAM_OIS, "done.");

	if (o_ctrl->is_ois_aat) {
		CAM_ERR(CAM_OIS, "OIS_VER_calibration");
			usleep_range(20 * 1000, 20 * 1000 + 10);
			ois_selftest();
			lgit_imx363_onsemi_ois_calibration();
			ois_selftest2();
			}

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

uint32_t lgit_imx363_onsemi_ois_calibration(void)
{
	uint32_t	UlRsltSts;
	int32_t		SlMeasureParameterA , SlMeasureParameterB;
	int32_t		SlMeasureParameterNum;
	UnllnVal	StMeasValueA , StMeasValueB;
	int32_t		SlMeasureAveValueA , SlMeasureAveValueB;
	uint32_t	val_gyro_offset_x, val_gyro_offset_y;
	uint8_t		e2p_gyro_offset_x[2], e2p_gyro_offset_y[2];

	RamRead32A(0x278, &val_gyro_offset_x);
	RamRead32A(0x27C, &val_gyro_offset_y);

	CAM_ERR(CAM_OIS, "lgit_ois_calibration start - val_gyro_offset_x: %d, val_gyro_offset_y: %d",
		(int16_t)(val_gyro_offset_x >> 16),
		(int16_t)(val_gyro_offset_y >> 16));

	MesFil();//(THROUGH); 				// Set Measure Filter

	SlMeasureParameterNum	=	GYROF_NUM; 				// Measurement times
	SlMeasureParameterA 	=	GYRO_RAM_GX_ADIDAT;		// Set Measure RAM Address
	SlMeasureParameterB 	=	GYRO_RAM_GY_ADIDAT;		// Set Measure RAM Address

	MeasureStart(SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB); 				// Start measure

	MeasureWait(); 				// Wait complete of measurement

//TRACE("Read Adr = %04x, %04xh \n",StMeasFunc_MFA_LLiIntegral1 + 4 , StMeasFunc_MFA_LLiIntegral1);
	RamRead32A(StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal);	// X axis
	RamRead32A(StMeasFunc_MFA_LLiIntegral1 + 4 	, &StMeasValueA.StUllnVal.UlHigVal);
	RamRead32A(StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal);	// Y axis
	RamRead32A(StMeasFunc_MFB_LLiIntegral2 + 4 	, &StMeasValueB.StUllnVal.UlHigVal);

//TRACE("GX_OFT = %08x, %08xh \n",(unsigned int)StMeasValueA.StUllnVal.UlHigVal,(unsigned int)StMeasValueA.StUllnVal.UlLowVal);
//TRACE("GY_OFT = %08x, %08xh \n",(unsigned int)StMeasValueB.StUllnVal.UlHigVal,(unsigned int)StMeasValueB.StUllnVal.UlLowVal);
	SlMeasureAveValueA = (int32_t)((INT64)StMeasValueA.UllnValue / SlMeasureParameterNum);
	SlMeasureAveValueB = (int32_t)((INT64)StMeasValueB.UllnValue / SlMeasureParameterNum);
//TRACE("GX_AVEOFT = %08xh \n",(unsigned int)SlMeasureAveValueA);
//TRACE("GY_AVEOFT = %08xh \n",(unsigned int)SlMeasureAveValueB);

	SlMeasureAveValueA = (SlMeasureAveValueA >> 16) & 0x0000FFFF;
	SlMeasureAveValueB = (SlMeasureAveValueB >> 16) & 0x0000FFFF;
//	SlMeasureAveValueA = 0x00010000 - SlMeasureAveValueA;
//	SlMeasureAveValueB = 0x00010000 - SlMeasureAveValueB;

	UlRsltSts = EXE_END;
	StAdjPar.StGvcOff.UsGxoVal = (uint16_t)(SlMeasureAveValueA & 0x0000FFFF); 	//Measure Result Store
	if(((INT16)StAdjPar.StGvcOff.UsGxoVal > (INT16)GYROF_UPPER) || ((INT16)StAdjPar.StGvcOff.UsGxoVal < (INT16)GYROF_LOWER)){
		UlRsltSts |= EXE_GXADJ;
	}
	RamWrite32A(GYRO_RAM_GXOFFZ , ((SlMeasureAveValueA << 16) & 0xFFFF0000)); 	// X axis Gyro offset

	StAdjPar.StGvcOff.UsGyoVal = (uint16_t)(SlMeasureAveValueB & 0x0000FFFF); 	//Measure Result Store
	if(((INT16)StAdjPar.StGvcOff.UsGyoVal > (INT16)GYROF_UPPER) || ((INT16)StAdjPar.StGvcOff.UsGyoVal < (INT16)GYROF_LOWER)){
		UlRsltSts |= EXE_GYADJ;
	}
	RamWrite32A(GYRO_RAM_GYOFFZ , ((SlMeasureAveValueB << 16) & 0xFFFF0000)); 	// Y axis Gyro offset

//TRACE("GX_AVEOFT_RV = %08xh \n",(unsigned int)SlMeasureAveValueA);
//TRACE("GY_AVEOFT_RV = %08xh \n",(unsigned int)SlMeasureAveValueB);

	RamWrite32A(GYRO_RAM_GYROX_OFFSET, 0x00000000); 		// X axis Drift Gyro offset
	RamWrite32A(GYRO_RAM_GYROY_OFFSET, 0x00000000); 		// Y axis Drift Gyro offset
	RamWrite32A(GyroFilterDelayX_GXH1Z2, 0x00000000);		// X axis H1Z2 Clear
	RamWrite32A(GyroFilterDelayY_GYH1Z2, 0x00000000);		// Y axis H1Z2 Clear

	WrGyroOffsetData();

	RamRead32A(0x278, &val_gyro_offset_x);
	RamRead32A(0x27C, &val_gyro_offset_y);

	CAM_ERR(CAM_OIS, "lgit_ois_calibration end - ram_gyro_offset_x: %d, ram_gyro_offset_y: %d",
		(int16_t)(val_gyro_offset_x >> 16),
		(int16_t)(val_gyro_offset_y >> 16));

	BurstReadE2Prom(0x2F, e2p_gyro_offset_x, 2);
	BurstReadE2Prom(0x31, e2p_gyro_offset_y, 2);

	CAM_ERR(CAM_OIS, "lgit_ois_calibration end - e2p_gyro_offset_x: %d, e2p_gyro_offset_y: %d",
		(int16_t)((e2p_gyro_offset_x[1]<<8) + e2p_gyro_offset_x[0]),
		(int16_t)((e2p_gyro_offset_y[1]<<8) + e2p_gyro_offset_y[0]));

	return(UlRsltSts);
}

int32_t lgit_imx363_onsemi_ois_stat(sensor_ois_stat_t *data)
{
	sensor_ois_stat_t ois_stat;
	uint32_t val_gyro_x, val_gyro_y;
	uint32_t val_gyro_offset_x, val_gyro_offset_y;

	memset(&ois_stat, 0, sizeof(ois_stat));
	snprintf(ois_stat.ois_provider, ARRAY_SIZE(ois_stat.ois_provider), "LGIT_ONSEMI");

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

	CAM_ERR(CAM_OIS,"gyro x %d gyro y %d offset x %d offset y %d", ois_stat.gyro[0], ois_stat.gyro[1], ois_stat.offset[0], ois_stat.offset[1]);

	return 0;
}

int32_t lgit_imx363_onsemi_ois_move_lens(void *data)
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

	CAM_ERR(CAM_OIS, "read : %d(0x%x), %d(0x%x)", (rHallx >> 16), (rHallx >> 16), (rHally >> 16), (rHally >> 16));

	if ((abs((rHallx >> 16) - offset[0]) < HALL_LIMIT) && (abs((rHally >> 16) - offset[1]) < HALL_LIMIT))
		return  OIS_SUCCESS;

	CAM_ERR(CAM_OIS, "move fail rHallx: %d, rHally: %d", (rHallx >> 16), (rHally >> 16));
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
#define GYRO_LIMIT        10   // 10[dps]*175
#define GYRO_OFFSET_LIMIT 30   // 30[dps]*175
#define GYRO_SCALE_FACTOR 175

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
	{9900, 0}, {4950, 4950}, {0, 9900},
	{-4950, 4950}, {-9900, 0}, {-4950, -4950},
	{0, -9900}, {4950, -4950}, {9900, 0}
	};

	memset(&ois_stat, 0, sizeof(ois_stat));

	CAM_ERR(CAM_OIS, " enter");
	//0. is ois init success ?
	result |= (1 << 11); // <- init success.

	//1. get first stat value.
	lgit_imx363_onsemi_ois_stat(&ois_stat);

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
		if (lgit_imx363_onsemi_ois_move_lens(&hall_target[i])>= 0)
			result |= (1 << (8 - i));
	}

	//5. reset lens position before ois turn-on.
	{
		uint16_t offset[2] = {0, 0};
		lgit_imx363_onsemi_ois_move_lens(&offset);
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
		rc = lgit_imx363_onsemi_ois_stat(&ois_stat);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,"i:%d ois_get_stat error", i);
			ois_stat.is_stable = 0;
		}
		if (abs(CAL_GYRO_X_VALUE) > GYRO_LIMIT) {
			CAM_ERR(CAM_OIS," %d spec over CAL_GYRO_X_VALUE %d", i, CAL_GYRO_X_VALUE);
			result &= ~(1 << 10);
			ois_stat.is_stable = 0;
		}
		if (abs(CAL_GYRO_Y_VALUE) > GYRO_LIMIT) {
			CAM_ERR(CAM_OIS," %d spec over CAL_GYRO_Y_VALUE %d", i, CAL_GYRO_Y_VALUE);
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

	memset(&i2c_reg_setting, 0, sizeof(i2c_reg_setting));

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

	memset(&i2c_reg_setting, 0, sizeof(i2c_reg_setting));

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

int32_t ois_i2c_e2p_read(uint32_t e2p_addr, uint16_t *e2p_data, enum camera_sensor_i2c_type data_type)
{
	int32_t ret = 0;
	uint32_t data = 0;
	uint16_t temp_sid = 0;

	temp_sid = local_cam_ois_t->io_master_info.cci_client->sid;
	local_cam_ois_t->io_master_info.cci_client->sid = E2P_SID >> 1;

	ret = camera_io_dev_read(
		&(local_cam_ois_t->io_master_info),
		e2p_addr,
		&data,
		CAMERA_SENSOR_I2C_TYPE_WORD,
		data_type);

	local_cam_ois_t->io_master_info.cci_client->sid = temp_sid;
	*e2p_data = (uint16_t)data;

	return ret;
}

void BurstReadE2Prom(unsigned char address, unsigned char * val, unsigned char cnt)
{
	uint32_t UlReadVal;
	unsigned char i;

	DMIOWrite32(E2P_ADR, address);	// Start Address
	DMIOWrite32(E2P_ASCNT, (cnt -1));		// Count Number
	DMIOWrite32(E2P_CMD, 1); 			// Re-Program
	// Read Exe
	RamWrite32A(CMD_IO_ADR_ACCESS, E2P_RDAT);
	for(i=0; i<cnt; i++){
		RamRead32A (CMD_IO_DAT_ACCESS, &UlReadVal);			// Read Access
		val[i] = (unsigned char)UlReadVal;
	}
}

void ReadE2Prom(unsigned char address, unsigned char * val)
{
	uint32_t UlReadVal;

	DMIOWrite32(E2P_ADR, address);	// Start Address
	DMIOWrite32(E2P_ASCNT, 0);		// Count Number
	DMIOWrite32(E2P_CMD, 1); 			// Re-Program
	// Read Exe
	RamWrite32A(CMD_IO_ADR_ACCESS, E2P_RDAT);
	RamRead32A (CMD_IO_DAT_ACCESS, &UlReadVal);			// Read Access

	*val = (unsigned char)UlReadVal;
}

unsigned char UnlockCodeClear(void)
{
	uint32_t UlReadVal;

	RamWrite32A(CMD_IO_ADR_ACCESS, E2P_WPB);				// UNLOCK_CLR(E0_7014h[4])=1
	RamWrite32A(CMD_IO_DAT_ACCESS, 0x00000010);
	RamRead32A(CMD_IO_DAT_ACCESS, &UlReadVal);
	if((UlReadVal & 0x00000080) != 0)	return (3);

	return(0);
}

unsigned char UnlockCodeSet(void)
{
	uint32_t UlReadVal;

	DMIOWrite32(E2P_UNLK_CODE1, 0xAAAAAAAA);	// UNLK_CODE1(E0_7554h) = AAAA_AAAAh
	DMIOWrite32(E2P_UNLK_CODE2, 0x55555555);	// UNLK_CODE2(E0_7AA8h) = 5555_5555h
	DMIOWrite32(E2P_RSTB, 		 0x00000001);	// RSTB_FLA_WR(E0_74CCh[0])=1
	DMIOWrite32(E2P_CLKON, 	 0x00000010);	// FLA_WR_ON(E0_7664h[4])=1
	DMIOWrite32(E2P_UNLK_CODE3, 0x0000ACD5);	// Additional Unllock Code Set

	RamWrite32A(CMD_IO_ADR_ACCESS , E2P_WPB	);
	RamRead32A( CMD_IO_DAT_ACCESS , &UlReadVal);
	if ((UlReadVal & 0x00000002) != 2) return(1);

	return(0);
}

uint8_t	WrGyroOffsetData(void)
{
	uint32_t UlReadVal, UlCnt;
	uint8_t ans, data[104], cnt;
	uint32_t ReadVerify, Parity;

	// Flash write
	ans = UnlockCodeSet();
	if (ans != 0) return (1);							// Unlock Code Set
//------------------------------------------------------------------------------------------------
// Page 1 (0x10-0x1F)
//------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------
// Page 2 (0x20-0x2F)
//------------------------------------------------------------------------------------------------
	DMIOWrite32(E2P_ADR, 0x20);	// Start Address
	DMIOWrite32(E2P_DFG, 0); 		// FLG CLR

	DMIOWrite32(E2P_WDAT15, (uint8_t)((StAdjPar.StGvcOff.UsGxoVal)>>LSB));	// OIS gyro offset X
	DMIOWrite32(E2P_CMD, 2); 			// Re-Program
	WitTim(20);
	UlCnt=0;
	do{
		if(UlCnt++ > 10){
			UnlockCodeClear();							// Unlock Code Clear
			return(3);
		}
		RamWrite32A(CMD_IO_ADR_ACCESS , E2P_INT);
		RamRead32A(CMD_IO_DAT_ACCESS, &UlReadVal);
	}while ((UlReadVal & 0x00000080) != 0);

//------------------------------------------------------------------------------------------------
// Page 3 (0x30-0x32)
//------------------------------------------------------------------------------------------------
	DMIOWrite32(E2P_ADR, 0x30);	// Start Address
	DMIOWrite32(E2P_DFG, 0); 		// FLG CLR

	DMIOWrite32(E2P_WDAT00, (uint8_t)((StAdjPar.StGvcOff.UsGxoVal)>>MSB));	// OIS gyro offset X
	DMIOWrite32(E2P_WDAT01, (uint8_t)((StAdjPar.StGvcOff.UsGyoVal)>>LSB));	// OIS gyro offset Y Lower byte
	DMIOWrite32(E2P_WDAT02, (uint8_t)((StAdjPar.StGvcOff.UsGyoVal)>>MSB));	// OIS gyro offset Y Higher byte
	DMIOWrite32(E2P_CMD, 2); 			// Re-Program
	WitTim(20);
	UlCnt=0;
	do{
		if(UlCnt++ > 10){
			UnlockCodeClear();							// Unlock Code Clear
			return(4);
		}
		RamWrite32A(CMD_IO_ADR_ACCESS , E2P_INT);
		RamRead32A(CMD_IO_DAT_ACCESS, &UlReadVal);
	}while ((UlReadVal & 0x00000080) != 0);

//------------------------------------------------------------------------------------------------
// Page 5 (0x50-0x5F)
//------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------
// Page 6 (0x50-0x5F)
//------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------
// CheckSum Creating
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom(0x10, data, 103);
	Parity = 0;
	for(cnt=0; cnt < 103; cnt++){
		Parity +=  data[cnt];
	}

//------------------------------------------------------------------------------------------------
// Page 7 (0x70-0x7F)
//------------------------------------------------------------------------------------------------
	DMIOWrite32(E2P_ADR, 0x70);	// Start Address
	DMIOWrite32(E2P_DFG, 0); 		// FLG CLR

	DMIOWrite32(E2P_WDAT07, (uint8_t)(Parity)); // CheckSum

	DMIOWrite32(E2P_CMD, 2); 			// Re-Program
	WitTim(20);
	UlCnt=0;
	do{
		if(UlCnt++ > 10){
			UnlockCodeClear();							// Unlock Code Clear
			return(5);
		}
		RamWrite32A(CMD_IO_ADR_ACCESS , E2P_INT);
		RamRead32A(CMD_IO_DAT_ACCESS, &UlReadVal);
	}while ((UlReadVal & 0x00000080) != 0);

	UnlockCodeClear();							// Unlock Code Clear
//------------------------------------------------------------------------------------------------
// Checksum Verification
//------------------------------------------------------------------------------------------------
	BurstReadE2Prom(0x10, data, 103);
	ReadVerify = 0;
	for(cnt=0; cnt < 103; cnt++){
		ReadVerify +=  data[cnt];
	}
	ReadE2Prom(0x77, &cnt);
	Parity = cnt;
	if((uint8_t)ReadVerify != (uint8_t)Parity)	return(6);

	return(0);
}

void WitTim(unsigned short	UsWitTim)
{
	usleep_range(1000 * UsWitTim, 1000 * UsWitTim + 10); /* wait [UsWitTim]ms */
}

