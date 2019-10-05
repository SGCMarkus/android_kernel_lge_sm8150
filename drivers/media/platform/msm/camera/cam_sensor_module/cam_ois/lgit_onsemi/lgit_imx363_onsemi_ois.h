#ifndef LGIT_ONSEMI_OIS_H
#define LGIT_ONSEMI_OIS_H

//OIS ERROR CODE
#define OIS_SUCCESS	 0
#define OIS_FAIL	-1
#define OIS_INIT	-9
#define OIS_INIT_OLD_MODULE		 1
#define OIS_INIT_NOT_SUPPORTED  -2
#define OIS_INIT_CHECKSUM_ERROR -3
#define OIS_INIT_EEPROM_ERROR   -4
#define OIS_INIT_I2C_ERROR      -5
#define OIS_INIT_TIMEOUT		-6
#define OIS_INIT_DOWNLOAD_ERROR -7
#define OIS_INIT_NOMEM			-8
#define OIS_INIT_GYRO_ADJ_FAIL	 2
#define OIS_INIT_SRV_GAIN_FAIL	 4

//****************************************************
//	CUSTOMER NECESSARY CREATING LIST
//****************************************************
/* for I2C communication */
int32_t RamWrite32A(uint32_t RamAddr, uint32_t RamData);
int32_t RamRead32A(uint32_t RamAddr, uint32_t *ReadData);
int32_t ois_i2c_e2p_read(uint32_t e2p_addr, uint16_t *e2p_data, enum camera_sensor_i2c_type data_type);
void BurstReadE2Prom( unsigned char address, unsigned char * val, unsigned char cnt );

/* for I2C Multi Translation : Burst Mode*/
int32_t CntWrt(uint8_t *data, uint16_t num_byte);
/* for Wait timer [Need to adjust for your system] */
void WitTim( unsigned short	UsWitTim );

uint8_t	WrGyroOffsetData( void );
uint32_t lgit_imx363_onsemi_ois_calibration(void);
static int8_t ois_selftest(void);
static int8_t ois_selftest2(void);

void msm_ois_create_sysfs(void);
void msm_ois_destroy_sysfs(void);

int32_t ois_i2c_read_seq(uint32_t addr, uint8_t *data, uint16_t num_byte);

#define MAX_OIS_BIN_FILENAME 128
#define MAX_OIS_BIN_BLOCKS 4
#define MAX_OIS_BIN_FILES 3

struct ois_i2c_bin_addr {
	uint32_t bin_str_addr;
	uint32_t bin_end_addr;
	uint32_t reg_str_addr;
};

struct ois_i2c_bin_entry {
	char  filename[MAX_OIS_BIN_FILENAME];
	uint32_t filesize;
	uint16_t blocks;
	struct ois_i2c_bin_addr addrs[MAX_OIS_BIN_BLOCKS];
};

struct ois_i2c_bin_list {
	uint16_t files;
	struct ois_i2c_bin_entry entries[MAX_OIS_BIN_FILES];
	uint32_t checksum;
};

typedef struct {
    char ois_provider[32];
    int16_t gyro[2];
    int16_t target[2];
    int16_t offset[2];
    uint8_t is_stable;
} sensor_ois_stat_t;

#define MAX_GYRO_QUERY_SIZE 15
#define READ_OUT_TIME 5000000 /*5ms*/
#define MAX_FAIL_CNT 3

enum msm_ois_timer_state_t {
	OIS_TIME_INIT,
	OIS_TIME_ACTIVE,
	OIS_TIME_INACTIVE,
	OIS_TIME_ERROR,
};

enum OISUserData {
	NO_SELECTION,
	HALL_FEEDING,
	USERDATAEND
};

struct ois_timer {
	struct hrtimer hr_timer;
	struct workqueue_struct *ois_wq;
	struct work_struct g_work;
	enum msm_ois_timer_state_t ois_timer_state;
	struct cam_ois_ctrl_t *o_ctrl;
	int i2c_fail_count;
};

int lgit_imx363_onsemi_ois_poll_ready(int limit);

#endif
