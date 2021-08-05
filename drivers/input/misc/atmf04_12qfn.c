#define MODULE_TAG	"<atmf04>"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/pm_wakeup.h>
#include <linux/unistd.h>
#include <linux/async.h>
#include <linux/in.h>

#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/sort.h>

#include <linux/firmware.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif

#include <soc/qcom/lge/board_lge.h>

//#include "bs_log.h"

#include "atmf04_12qfn.h"

#if defined (CONFIG_LGE_USE_DD_SAR_RESET)
#include <linux/dd_sar.h>
#endif
#if defined (CONFIG_LGE_USE_SAR_CONTROLLER)
#define CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM
#define CONFIG_LGE_CAP_SENSOR_AUTO_CAL_ALGORITHM
#define CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE
#define CONFIG_LGE_CAP_SENSOR_UPDATE_SENSITIVITY
#define CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS
#if !defined (CONFIG_SENSOR_ATMF04_2ND) && (defined (CONFIG_MACH_MSM8952_B3_ATT_US) || defined(CONFIG_MACH_MSM8952_B3_BELL_CA) || defined(CONFIG_MACH_MSM8952_B3_RGS_CA))
#define CONFIG_LGE_CAP_SENSOR_CURRENT_ISSUE // b3_att_us rev.B HW event - 2nd cap sensor makes sleep current increasing by always on.
#endif
#endif

#if defined CONFIG_SENSOR_ATMF04_2ND
#define CONFIG_LGE_CAP_SENSOR_BOOTING_TIME_IMPROVEMENT
#endif

#define ATMF04_DRV_NAME     "lge_sar_rf"

#define FAR_STATUS      1
#define NEAR_STATUS     0

#if defined (CONFIG_LGE_ATMF04_2CH)
#define CNT_MAX_CH		3
#else
#define CNT_MAX_CH		2
#endif

#define CH1_FAR         0x2
#define CH1_NEAR        0x1
#if defined (CONFIG_LGE_ATMF04_2CH)
#define CH2_FAR         (0x2 << 2)
#define CH2_NEAR        (0x1 << 2)
#endif

/* I2C Suspend Check */
#define ATMF04_STATUS_RESUME        0
#define ATMF04_STATUS_SUSPEND       1
#define ATMF04_STATUS_QUEUE_WORK    2

/* Calibration Check */
#if defined(CONFIG_LGE_USE_SAR_CONTROLLER) // Device tuning value
#if defined(CONFIG_LGE_ATMF04_2CH)
#if defined (CONFIG_MACH_SM8150_BETA)
#define ATMF04_CR_DUTY_LOW        256  // CR range MIN
#define ATMF04_CR_DUTY_HIGH       3200 // CR Range MAX
#define ATMF04_CS_DUTY_LOW        256  // CS Range MIN
#define ATMF04_CS_DUTY_HIGH       3200 // CS Range MAX
#elif defined ( CONFIG_MACH_SM8150_MH2LM )
#define ATMF04_CR_DUTY_LOW        256  // CR range MIN
#define ATMF04_CR_DUTY_HIGH       3200 // CR Range MAX
#define ATMF04_CS_DUTY_LOW        256  // CS Range MIN
//#define ATMF04_CS_DUTY_HIGH       3200 // CS Range MAX
#define ATMF04_CS_DUTY_HIGH       4032 // CS Range MAX // requested by MH2 NA HW RF ANT shin.choi@lge.com 2019-07-29 mail
#else
#define ATMF04_CR_DUTY_LOW        1900  // CR range MIN
#define ATMF04_CR_DUTY_HIGH       2900 // CR Range MAX
#define ATMF04_CS_DUTY_LOW        1152  // CS Range MIN
#define ATMF04_CS_DUTY_HIGH       2112 // CS Range MAX
#endif
#define ATMF04_CSCR_RESULT          -2
#endif // defined(CONFIG_LGE_USE_SAR_CONTROLLER)
#endif

/* I2C Register */
#if defined(CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM)
#if defined(CONFIG_LGE_ATMF04_2CH)
#define I2C_ADDR_SSTVT_H            0x01
#define I2C_ADDR_SSTVT_L            0x02
#define I2C_ADDR_SSTVT_CH2_H        0x03
#define I2C_ADDR_SSTVT_CH2_L        0x04
#define I2C_ADDR_SAFE_DUTY_CHK      0x1C
#define I2C_ADDR_SYS_CTRL           0x1F
#define I2C_ADDR_SYS_STAT           0x20
#define I2C_ADDR_CR_DUTY_H          0x26
#define I2C_ADDR_CR_DUTY_L          0x27
#define I2C_ADDR_CS_DUTY_H          0x28
#define I2C_ADDR_CS_DUTY_L          0x29
#define I2C_ADDR_CS_DUTY_CH2_H      0x2A
#define I2C_ADDR_CS_DUTY_CH2_L      0x2B
#define I2C_ADDR_PER_H              0x22
#define I2C_ADDR_PER_L              0x23
#define I2C_ADDR_PER_CH2_H          0x24
#define I2C_ADDR_PER_CH2_L          0x25
#define I2C_ADDR_TCH_OUTPUT         0x21
#define I2C_ADDR_PGM_VER_MAIN       0x71
#define I2C_ADDR_PGM_VER_SUB        0x72
#else
#define I2C_ADDR_SSTVT_H            0x01
#define I2C_ADDR_SSTVT_L            0x02
#define I2C_ADDR_TCH_ONOFF_CNT      0x03
#define I2C_ADDR_DIG_FILTER         0x04
#define I2C_ADDR_TEMP_COEF_UP       0x06
#define I2C_ADDR_TEMP_COEF_DN       0x07
#define I2C_ADDR_SAFE_DUTY_CHK      0x0E
#define I2C_ADDR_SYS_CTRL           0x0F
#define I2C_ADDR_SYS_STAT           0x10
#define I2C_ADDR_CR_DUTY_H          0x11
#define I2C_ADDR_CR_DUTY_L          0x12
#define I2C_ADDR_CS_DUTY_H          0x13
#define I2C_ADDR_CS_DUTY_L          0x14
#define I2C_ADDR_PER_H              0x15
#define I2C_ADDR_PER_L              0x16
#define I2C_ADDR_TCH_OUTPUT         0x17
#define I2C_ADDR_PGM_VER_MAIN       0x18
#define I2C_ADDR_PGM_VER_SUB        0x19
#endif
#else
#define I2C_ADDR_SSTVT_H            0x01
#define I2C_ADDR_SSTVT_L            0x02
#define I2C_ADDR_TEMP_COEF_UP       0x03
#define I2C_ADDR_TEMP_COEF_DN       0x04
#define I2C_ADDR_DIG_FILTER         0x05
#define I2C_ADDR_TCH_ONOFF_CNT      0x06
#define I2C_ADDR_SAFE_DUTY_CHK      0x07
#define I2C_ADDR_SYS_CTRL           0x08
#define I2C_ADDR_SYS_STAT           0x09
#define I2C_ADDR_CR_DUTY_H          0x0A
#define I2C_ADDR_CR_DUTY_L          0x0B
#define I2C_ADDR_CS_DUTY_H          0x0C
#define I2C_ADDR_CS_DUTY_L          0x0D
#define I2C_ADDR_PER_H              0x0E
#define I2C_ADDR_PER_L              0x0F
#define I2C_ADDR_TCH_OUTPUT         0x10
#define I2C_ADDR_PGM_VER_MAIN       0x16
#define I2C_ADDR_PGM_VER_SUB        0x17
#endif

#if defined(CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM)
//Calibration Data Backup/Restore
#define I2C_ADDR_CMD_OPT 					0x7E
#define I2C_ADDR_COMMAND 					0x7F
#define I2C_ADDR_REQ_DATA					0x80
#define CMD_R_CD_DUTY						0x04		//Cal Data Duty Read
#define CMD_R_CD_REF						0x05		//Cal Data Ref Read
#define CMD_W_CD_DUTY						0x84		//Cal Data Duty Read
#define CMD_W_CD_REF						0x85		//Cal Data Ref Read
#define SZ_CALDATA_UNIT 					24
static int CalData[6][SZ_CALDATA_UNIT];
#endif

#define BIT_PERCENT_UNIT            8.192
#define MK_INT(X, Y)                (((int)X << 8)+(int)Y)

#define ENABLE_SENSOR_PINS          0
#define DISABLE_SENSOR_PINS         1

#define ON_SENSOR                   1
#define OFF_SENSOR                  2
#define PATH_CAPSENSOR_CAL  "/mnt/vendor/sns/sar_controller_cal.dat"

static struct i2c_driver atmf04_driver;
static struct workqueue_struct *atmf04_workqueue;

//static unsigned char fuse_data[SZ_PAGE_DATA];

#ifdef CONFIG_OF
enum sensor_dt_entry_status {
    DT_REQUIRED,
    DT_SUGGESTED,
    DT_OPTIONAL,
};

enum sensor_dt_entry_type {
    DT_U32,
    DT_GPIO,
    DT_BOOL,
    DT_STRING,
};

struct sensor_dt_to_pdata_map {
    const char			*dt_name;
    void				*ptr_data;
    enum sensor_dt_entry_status status;
    enum sensor_dt_entry_type	type;
    int				default_val;
};
#endif

static struct i2c_client
*atmf04_i2c_client; /* global i2c_client to support ioctl */

struct atmf04_platform_data {
    int (*init)(struct i2c_client *client);
    void (*exit)(struct i2c_client *client);
    unsigned int irq_gpio;
#if defined (CONFIG_MACH_SDM845_JUDYPN)
    unsigned int irq_gpio2;
#endif
    unsigned long chip_enable;
#if defined (CONFIG_LGE_CAP_SENSOR_CURRENT_ISSUE) || defined (CONFIG_LGE_CAP_SENSOR_BOOTING_TIME_IMPROVEMENT)
    unsigned long chip_enable2;
#endif
    int (*power_on)(struct i2c_client *client, bool on);
    u32 irq_gpio_flags;

    bool i2c_pull_up;

    struct regulator *vcc_ana;
    struct regulator *vcc_dig;
    struct regulator *vcc_i2c;

    u32 vdd_ana_supply_min;
    u32 vdd_ana_supply_max;
    u32 vdd_ana_load_ua;

    u32 input_pins_num; // not include ref sensor pin
    const char *fw_name;
};

struct atmf04_data {
    int (*get_nirq_low)(void);
    struct i2c_client *client;
    struct mutex update_lock;
    struct mutex enable_lock;
    struct delayed_work	dwork;		/* for PS interrupt */
    struct input_dev *input_dev_cap;
#ifdef CONFIG_OF
    struct atmf04_platform_data *platform_data;
    int irq;
#endif
    unsigned int enable;
    unsigned int sw_mode;
    atomic_t i2c_status;

    unsigned int cap_detection;
    int touch_out;
};

static bool on_sensor = false;
static bool check_allnear = false;
static bool cal_result = false; // debugging calibration paused
static atomic_t pm_suspend_flag;

#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
static bool probe_end_flag = false;
#endif

static int get_bit(unsigned short x, int n);
//static short get_abs(short x);
#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
static void check_firmware_ready(struct i2c_client *client);
static void check_init_touch_ready(struct i2c_client *client);
#endif


static void chg_mode(unsigned char flag, struct i2c_client *client)
{
    if(flag == ON) {
        i2c_smbus_write_byte_data(client, ADDR_EFLA_STS, 0x80);
        PINFO("ATMF04 change_mode : %d\n",i2c_smbus_read_byte_data(client, ADDR_EFLA_STS));

        //============================================================//
        //[190306] ADS Add
        //[START]=====================================================//
#if defined(USE_ALMF04)
        i2c_smbus_write_byte_data(client, ADDR_ROM_SAFE, VAL_ROM_MASK1);
        i2c_smbus_write_byte_data(client, ADDR_ROM_SAFE, VAL_ROM_MASK2);
#endif
        //[END]======================================================//
    }
    else {
        i2c_smbus_write_byte_data(client, ADDR_EFLA_STS, 0x00);
        PINFO("ATMF04 change_mode : %d\n",i2c_smbus_read_byte_data(client, ADDR_EFLA_STS));
    }
    mdelay(1);
}

#if defined(CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM)
static int Backup_CalData(struct i2c_client *client)
{
    int loop, dloop;
    int ret;

    for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
    {
        ret = i2c_smbus_write_byte_data(client,I2C_ADDR_CMD_OPT, loop);
        if (ret) {
            PINFO("ATMF04 tmf04: i2c_write_fail \n");
            return ret;
        }
        ret = i2c_smbus_write_byte_data(client,I2C_ADDR_COMMAND, CMD_R_CD_DUTY);
        if (ret) {
            PINFO("atmf04: i2c_write_fail \n");
            return ret;
        }

        mdelay(1); 		//1 ms Delay

        for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop++)
            CalData[loop][dloop] = i2c_smbus_read_byte_data(client,I2C_ADDR_REQ_DATA + dloop);
    }

    for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
    {
        ret = i2c_smbus_write_byte_data(client,I2C_ADDR_CMD_OPT, loop);
        if (ret) {
            PINFO("atmf04: i2c_write_fail \n");
            return ret;
        }
        ret = i2c_smbus_write_byte_data(client,I2C_ADDR_COMMAND, CMD_R_CD_REF);
        if (ret) {
            PINFO("atmf04: i2c_write_fail \n");
            return ret;
        }

        mdelay(1); 		//1 ms Delay

        for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop++)
            CalData[CNT_MAX_CH+loop][dloop] = i2c_smbus_read_byte_data(client,I2C_ADDR_REQ_DATA + dloop);
    }
    if(CalData[0][0] == 0xFF || (CalData[0][0] == 0x00 && CalData[0][1] == 0x00))
    {
        PINFO("atmf04: Invalid cal data, Not back up this value.");
        return -1;
    }

    //============================================================//
    //[20180327] ADS Change
    //[START]=====================================================//
    //for(loop =0;loop<3;loop++)
    for(loop =0;loop<6;loop++)
        //[END]======================================================//
    {
        for(dloop=0;dloop < SZ_CALDATA_UNIT ; dloop++)
            PINFO("atmf04: backup_caldata data[%d][%d] : %d \n",loop,dloop,CalData[loop][dloop]);
    }

    PINFO("atmf04 backup_cal success");
    return 0;
}

static int Write_CalData(struct i2c_client *client)
{
    int loop, dloop;
    int ret;

    for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
    {
        ret = i2c_smbus_write_byte_data(client,I2C_ADDR_CMD_OPT, loop);
        if (ret) {
            PINFO("atmf04: i2c_write_fail \n");
            return ret;
        }
        ret =i2c_smbus_write_byte_data(client,I2C_ADDR_COMMAND, CMD_W_CD_DUTY);
        if (ret) {
            PINFO("atmf04: i2c_write_fail \n");
            return ret;
        }

        mdelay(1); 		//1 ms Delay

        for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop++)
        {
            ret = i2c_smbus_write_byte_data(client,I2C_ADDR_REQ_DATA + dloop, CalData[loop][dloop]);
            if (ret) {
                PINFO("atmf04: i2c_write_fail \n");
                return ret;
            }

        }
    }

    for(loop = 0 ; loop < CNT_MAX_CH ; loop++)
    {
        ret = i2c_smbus_write_byte_data(client,I2C_ADDR_CMD_OPT, loop);
        if (ret) {
            PINFO("atmf04: i2c_write_fail \n");
            return ret;
        }
        ret = i2c_smbus_write_byte_data(client,I2C_ADDR_COMMAND, CMD_W_CD_REF);
        if (ret) {
            PINFO("atmf04: i2c_write_fail \n");
            return ret;
        }

        mdelay(1); 		//1 ms Delay

        for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop++)
        {
            ret = i2c_smbus_write_byte_data(client,I2C_ADDR_REQ_DATA + dloop, CalData[CNT_MAX_CH+loop][dloop]);
            if (ret) {
                PINFO("atmf04: i2c_write_fail \n");
                return ret;
            }

        }
    }
    return 0;
}

static int RestoreProc_CalData(struct atmf04_data *data, struct i2c_client *client)
{
    int loop;
    int ret;
    //Power On
    gpio_set_value(data->platform_data->chip_enable, 0);
    mdelay(450);


    //Calibration data write
    ret = Write_CalData(client);
    if(ret)
        return ret;

    //Initial code write
    for(loop = 0 ; loop < CNT_INITCODE ; loop++) {
        ret = i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
        if (ret) {
            PINFO("i2c_write_fail[0x%x]",InitCodeAddr[loop]);
        }
        PINFO("Restore##[0x%x][0x%x]##", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
    }
    //============================================================//
    //[20180327] ADS Change
    //[START]=====================================================//
    ////E-flash Data Save Command old version: 0x80, 2ch version : 0x0c
    ////atmf04_i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x80);
    //atmf04_i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x0c);
    //mdelay(800);		//50ms delay
    //check_firmware_ready(client);
    check_firmware_ready(client);
    i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x80);
    check_firmware_ready(client);
    //[END]======================================================//

    //Software Reset2
    i2c_smbus_write_byte_data(client,I2C_ADDR_SYS_CTRL, 0x02);
    PINFO("atmf04 restore_cal success");
    return 0;
}

#endif

#if defined(CONFIG_LGE_CAP_SENSOR_UPDATE_SENSITIVITY)
static int Update_Sensitivity(struct atmf04_data *data, struct i2c_client *client)
{
    int ret = 0;
#ifdef CONFIG_LGE_ATMF04_2CH
    char sstvt_ch1_l;
    char sstvt_ch2_l;
#endif
    unsigned char loop;

#ifdef CONFIG_LGE_ATMF04_2CH
    sstvt_ch1_l = i2c_smbus_read_byte_data(client, I2C_ADDR_SSTVT_L);
    sstvt_ch2_l = i2c_smbus_read_byte_data(client, I2C_ADDR_SSTVT_CH2_L);
    if (ret) {
        PINFO("i2c_read_fail[0x%x]",I2C_ADDR_SSTVT_CH2_L);
        goto i2c_fail;
    }
#endif
    PINFO(" ATMF04 CH1 sstvt_L > read value:0x%02x, Initvalue:0x%02x\n", sstvt_ch1_l, InitCodeVal[1]);
    PINFO(" ATMF04 CH2 sstvt_L > read value:0x%02x, Initvalue:0x%02x\n", sstvt_ch2_l, InitCodeVal[3]);

    if(sstvt_ch2_l != InitCodeVal[3] || sstvt_ch1_l != InitCodeVal[1]) {

	    //============================================================//
	    //[191123] ADS Change
	    //[START]=====================================================//
	    mdelay(10);
	    PINFO("Update_Sensitivity Start");
	    //[END]======================================================//

	    for(loop = 0 ; loop < CNT_INITCODE ; loop++) {
            ret = i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
            if (ret) {
                PINFO("i2c_write_fail[0x%x]",InitCodeAddr[loop]);
                return ret;
            }
            PINFO("##[0x%x][0x%x]##", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
        }
        check_firmware_ready(client);

        //E-flash Data Save Command old version: 0x80, 2ch version : 0x0c
        //ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x80);
        ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x04);
        if (ret) {
            PINFO("i2c_write_fail[0x%x]",I2C_ADDR_SYS_CTRL);
            goto i2c_fail;
	}
	//============================================================//
	//[191123] ADS Change
	//[START]=====================================================//
	//mdelay(50);		//50ms delay
	check_firmware_ready(client);
	//[END]======================================================//

	PINFO("Update_Sensitivity Finished");
    }

    return 0;

i2c_fail:
    return ret;
}
#endif

static unsigned char chk_done(unsigned int wait_cnt, struct i2c_client *client)
{
    unsigned int trycnt = 0;
    unsigned char rtn;

    do
    {
        if(++trycnt > wait_cnt) {
            PINFO("ATMF04  RTN_TIMEOUT");
            return RTN_TIMEOUT;
        }
        mdelay(1);
        rtn = i2c_smbus_read_byte_data(client, ADDR_EFLA_STS);
    }while((rtn & FLAG_DONE) != FLAG_DONE);

    return RTN_SUCC;
}

static unsigned char chk_done_erase(unsigned int wait_cnt, struct i2c_client *client)
{
    unsigned int trycnt = 0;
    unsigned char rtn;

    do
    {
        if(++trycnt > wait_cnt) return RTN_TIMEOUT;

        //============================================================//
        //[20180327] ADS Change
        //[START]=====================================================//
        //mdelay(1);
        mdelay(5);
        //[END]======================================================//
        rtn = i2c_smbus_read_byte_data(client, ADDR_EFLA_STS);
    }while((rtn & FLAG_DONE_ERASE) != FLAG_DONE_ERASE);

    return RTN_SUCC;
}

static unsigned char erase_eflash(struct i2c_client *client)
{
    //============================================================//
    //[190306] ADS Change
    //[START]=====================================================//
    //i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_ERASE_ALL);
#if defined(USE_ALMF04)
    i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, 0x00);
    i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, EMD_ALL_ERASE);
    i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EEP_START);
#else
    i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_ERASE_ALL);
#endif
    //[END]======================================================//

    if(chk_done_erase(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT)
        return RTN_TIMEOUT; //timeout

    return RTN_SUCC;
}

//=====================================================================================================================================//
//[190306] ADS Change
//[START]==============================================================================================================================//
//static unsigned char write_eflash_page(unsigned char flag, unsigned char * page_addr, unsigned char * wdata, struct i2c_client *client)
//{
//	unsigned char paddr[2];
//
//	if(flag != FLAG_FUSE)
//	{
//		paddr[0] = page_addr[1];
//		paddr[1] = page_addr[0];
//	}
//	else	//Extra User Memory
//	{
//		paddr[0] = 0x00;
//		paddr[1] = 0x00;
//	}
//
//	if(flag != FLAG_FUSE)
//	{
//		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_L_WR);
//	}
//	else
//	{
//		//Erase
//		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_ERASE);
//		if(chk_done_erase(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;
//		//Write
//		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_WR);
//	}
//
//	if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;
//
//	return RTN_SUCC;
//}
//
//static unsigned char read_eflash_page(unsigned char flag, unsigned char * page_addr, unsigned char * rdata, struct i2c_client *client)
//{
//	unsigned char paddr[2];
//
//	if(flag != FLAG_FUSE)
//	{
//		paddr[0] = page_addr[1];
//		paddr[1] = page_addr[0];
//	}
//	else	//Extra User Memory
//	{
//		paddr[0] = 0x00;
//		paddr[1] = 0x00;
//	}
//
//	if(flag != FLAG_FUSE)
//		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_RD);
//	else
//		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_RD);
//
//	if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;
//
//	return RTN_SUCC;
//}
#if 0
static unsigned char write_eflash_page(unsigned char flag, unsigned char * page_addr, unsigned char * wdata, struct i2c_client *client)
{
    for(i=0; i < SZ_PAGE_DATA; i++) i2c_smbus_write_byte_data(client, i, wdata[i]);

#if defined(USE_ALMF04)
    if(flag != FLAG_FUSE)
    {
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, (page_addr[1] | EMD_PG_WRITE));
        i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EEP_START);
    }
    else
    {
        //Erase
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, 0x00);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, EMD_PG_ERASE);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_START);
        if(chk_done_erase(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;
        //write
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, 0x00);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, EMD_PG_WRITE);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_START);
    }
#else
    if(flag != FLAG_FUSE)
    {
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, page_addr[1]);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_L_WR);
    }
    else
    {
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, 0x00);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, 0x00);
        //Erase
        i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_ERASE);
        if(chk_done_erase(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;
        //write
        i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_WR);
    }
#endif

    if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;

    return RTN_SUCC;
}
#endif

#if 0
static unsigned char read_eflash_page(unsigned char flag, unsigned char * page_addr, unsigned char * rdata, struct i2c_client *client)
{
#if defined(USE_ALMF04)
    if(flag != FLAG_FUSE)
    {
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, (page_addr[1] | EMD_PG_READ));
        i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EEP_START);
    }
    else
    {
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, 0x00);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, EMD_PG_READ);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_START);
    }
#else
    if(flag != FLAG_FUSE)
    {
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, page_addr[1]);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_RD);
    }
    else
    {
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, 0x00);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, 0x00);
        i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_RD);
    }
#endif

    if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;

    for(i=0; i < SZ_PAGE_DATA; i++) rdata[i] = i2c_smbus_read_byte_data(client, i);

    return RTN_SUCC;
}
//[END]================================================================================================================================//
#endif

static void onoff_sensor(struct atmf04_data *data, int onoff_mode)
{
    int nparse_mode;

    nparse_mode = onoff_mode;
    PINFO("ATMF04 onoff_sensor: nparse_mode [%d]",nparse_mode);

    if (nparse_mode == ENABLE_SENSOR_PINS) {
#if 1 // fixed bug to support cap sensor HAL
        input_report_abs(data->input_dev_cap, ABS_DISTANCE, 3);/* Initializing input event */
        input_sync(data->input_dev_cap);
#endif
        gpio_set_value(data->platform_data->chip_enable, 0);    /*chip_en pin - low : on, high : off*/
        mdelay(500);
        if (!on_sensor)
            enable_irq_wake(data->irq);
        on_sensor = true;
#if 0 // fixed bug to support cap sensor HAL
        if (gpio_get_value(data->platform_data->irq_gpio)) {
            input_report_abs(data->input_dev_cap, ABS_DISTANCE, FAR_STATUS);/* force FAR detection */
            input_sync(data->input_dev_cap);
        }
#endif
    }
    if (nparse_mode == DISABLE_SENSOR_PINS) {
        gpio_set_value(data->platform_data->chip_enable, 1);    /*chip_en pin - low : on, high : off*/
        if (on_sensor)
            disable_irq_wake(data->irq);
        on_sensor = false;
    }
}

static unsigned char load_firmware(struct atmf04_data *data, struct i2c_client *client, const char *name)
{
    const struct firmware *fw = NULL;
    unsigned char rtn;
    int ret, i, count = 0;
    int max_page;
    unsigned short main_version, sub_version;
    unsigned char page_addr[2];
    unsigned char fw_version, ic_fw_version, page_num;
    int version_addr;
    int chip_id = -1;
#if defined (CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM)
    int restore = 0;
#endif

    unsigned char sys_status = 1;

    PINFO("ATMF04 Load Firmware Entered");

    gpio_set_value(data->platform_data->chip_enable, 1);
    mdelay(50);
    gpio_set_value(data->platform_data->chip_enable, 0);

    ret = request_firmware(&fw, name, &data->client->dev);
    if (ret) {
        PINFO("ATMF04 Unable to open bin [%s]  ret %d", name, ret);
        if (fw) release_firmware(fw);
        return 1;
    } else {
        PINFO("ATMF04 Open bin [%s] ret : %d ", name, ret);
    }

    max_page = (fw->size)/SZ_PAGE_DATA;
    version_addr = (fw->size)-SZ_PAGE_DATA;
    fw_version = MK_INT(fw->data[version_addr], fw->data[version_addr+1]);
    page_num = fw->data[version_addr+3];
    PINFO("###########fw version : %d.%d, fw_version : %d, page_num : %d###########", fw->data[version_addr], fw->data[version_addr+1], fw_version, page_num);

    mdelay(50);

    chip_id= i2c_smbus_read_byte_data(client, 0x00);
    PINFO("ATMF04 SAR controller ATMF04 chip_id = 0x%x\n", chip_id);

    main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
    sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);
    ic_fw_version = MK_INT(main_version, sub_version);
    PINFO("###########ic version : %d.%d, ic_fw_version : %d###########", main_version, sub_version, ic_fw_version);

    if( (fw->data[version_addr] != main_version) || (fw->data[version_addr+1] != sub_version)) {
        PINFO("ATMF04 update firmware");

        mdelay(500);
        check_firmware_ready(client);
        sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

#if defined(CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM)
        if ((ic_fw_version == 0) || ((sys_status & 0x06) ==0)) {
            restore = 0;
            PINFO("skip cal backup");
        }
        else {
            if (Backup_CalData(client) < 0) restore = 0;
            else 							restore = 1;
        }
#endif
        /* IC Download Mode Change */
        chg_mode(ON, client);

        /* firmware write process */
        rtn = erase_eflash(client);
        if(rtn != RTN_SUCC) {
            PINFO("earse fail\n");
            if (fw) release_firmware(fw);
            return rtn;		//earse fail
        }

        while(count < page_num) 
        {
            for(i=0; i < SZ_PAGE_DATA; i++) i2c_smbus_write_byte_data(client, i, fw->data[i + (count*SZ_PAGE_DATA)]);

            page_addr[1] = (unsigned char)((count & 0xFF00) >> 8);
            page_addr[0] = (unsigned char)(count & 0x00FF);

#if defined(USE_ALMF04)
            i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
            i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, (page_addr[1] | EMD_PG_WRITE));
            i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EEP_START);
#else
            i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
            i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, page_addr[1]);
            /*Eflash write command 0xFC -> 0x01 Write*/
            i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_L_WR);
#endif

            if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT)
            {
                if (fw) release_firmware(fw);
                return RTN_TIMEOUT;
            }
            count++;
        }
        chg_mode(OFF, client);

        gpio_set_value(data->platform_data->chip_enable, 1);
        mdelay(50);
        gpio_set_value(data->platform_data->chip_enable, 0);
        mdelay(50);	

        main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
        sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);
        ic_fw_version = MK_INT(main_version, sub_version);
        PINFO("###########ic version : %d.%d, ic_fw_version : %d###########", main_version, sub_version, ic_fw_version);

        if( (fw->data[version_addr] != main_version) || (fw->data[version_addr+1] != sub_version)) 
        {
            PINFO("ATMF04 Firmware update failed.(ic version : %d.%d)", main_version, sub_version);
            if (fw) release_firmware(fw);
            return 3;
        }
        else
            PINFO("ATMF04 Firmware update is done");
    }
    else {
        PINFO("Not update firmware. Firmware version is lower than ic.(Or same version)");
    }

#if defined(CONFIG_LGE_CAP_SENSOR_UPDATE_SENSITIVITY)
    Update_Sensitivity(data, client);
#endif

    gpio_set_value(data->platform_data->chip_enable, 1);

#if defined(CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM)
    mdelay(10);
    if(restore)
    {
        ret = RestoreProc_CalData(data, client);
        if(ret)
            PINFO("restore fail ret: %d",ret);
    }
#endif
    PINFO("disable ok");

    release_firmware(fw);

    return 0;
}
//==================================================================================================================================//

static int write_initcode(struct i2c_client *client)
{
    struct atmf04_data *data = i2c_get_clientdata(client);
    unsigned char loop;
    int en_state;
    int ret = 0;
    //============================================================//
    //[190401] ADS ADD
    //[START]=====================================================//
    unsigned char sys_status = 0;
    //[END]======================================================//

    PINFO("write_initcode entered[%d]",data->platform_data->irq_gpio);

    en_state = gpio_get_value(data->platform_data->chip_enable);

    if (en_state)
        gpio_set_value(data->platform_data->chip_enable, 0);

    //============================================================//
    //[190401] ADS Change
    //[START]=====================================================//

    mdelay(50);

    sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
    if((get_bit(sys_status, 0) == 0) && (get_bit(sys_status, 1) == 1))
        i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x01);

    mdelay(400);

    //[END]======================================================//

#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
    check_firmware_ready(client);
#endif

    for(loop = 0 ; loop < CNT_INITCODE ; loop++) {
        ret = i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
        if (ret) {
            PINFO("i2c_write_fail[0x%x]",InitCodeAddr[loop]);
            return ret;
        }
        PINFO("##[0x%x][0x%x]##", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
    }

    return 0;
}

static bool valid_multiple_input_pins(struct atmf04_data *data)
{
    if (data->platform_data->input_pins_num > 1)
        return true;

    return false;

}

static int write_calibration_data(struct atmf04_data *data, char *filename)
{
    int fd = 0;

    char file_result[5] = {0,}; // debugging calibration paused
    mm_segment_t old_fs = get_fs();

    PINFO("write_calibration_data Entered[%d]",data->platform_data->irq_gpio);
    set_fs(KERNEL_DS);
    fd = sys_open(filename, O_WRONLY|O_CREAT, 0664);

    if (fd >= 0) {
#if 1 // debugging calibration paused
        if(cal_result) {
            strncpy(file_result, CAP_CAL_RESULT_PASS, strlen(CAP_CAL_RESULT_PASS));
            sys_write(fd, file_result, sizeof(file_result));
        } else {
            strncpy(file_result, CAP_CAL_RESULT_FAIL, strlen(CAP_CAL_RESULT_FAIL));
            sys_write(fd, file_result, sizeof(file_result));
        }
        PINFO("%s: write [%s] to %s, cal_result %d", __FUNCTION__, file_result, filename, cal_result);
        sys_close(fd);
        set_fs(old_fs);
#else
        sys_write(fd,0,sizeof(int));
        sys_close(fd);
#endif
    } else {
        PINFO("%s: %s open failed [%d]......", __FUNCTION__, filename, fd);
    }

    //PINFO("sys open to save cal.dat");

    return 0;
}

#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
static void check_firmware_ready(struct i2c_client *client)
{
    unsigned char sys_status = 1;
    int i;

    sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
    for(i = 0 ; i < 100 ; i++) {
        if (get_bit(sys_status, 0) == 1) {
            PINFO("%s: Firmware is busy now.....[%d] sys_status = [0x%x]", __FUNCTION__, i, sys_status);
            mdelay(10);
            sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
        } else {
            break;
        }
    }
    PINFO("%s: sys_status = [0x%x]", __FUNCTION__, sys_status);

    return;

}

static void check_init_touch_ready(struct i2c_client *client)
{
    unsigned char init_touch_md_check = 0;
    int i;

    init_touch_md_check = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
    for(i = 0 ; i < 50 ; i++) {
        if ((get_bit(init_touch_md_check, 2) == 0) && (get_bit(init_touch_md_check, 1) == 0)) {
            PINFO("%s: Firmware init touch is not yet ready.....[%d] sys_statue = [0x%x]", __FUNCTION__, i, init_touch_md_check);
            mdelay(10);
            init_touch_md_check = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
        } else {
            break;
        }
    }
    PINFO("%s: sys_status = [0x%x]", __FUNCTION__, init_touch_md_check);

    return;

}
#endif
static ssize_t atmf04_show_reg(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);
    int loop;
    char buf_line[256] = "";
    int nlength = 0;
    char buf_regproxdata[512] = "";
    client = data->client;

    for (loop = 0; loop < CNT_INITCODE; loop++) {
        memset(buf_line, 0, sizeof(buf_line));
        sprintf(buf_line, "[0x%x:0x%x]", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
        PINFO("###### [0x%x][0x%x]###", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
        nlength = strlen(buf_regproxdata);
        strcpy(&buf_regproxdata[nlength], buf_line);
    }

    return sprintf(buf,"%s\n", buf_regproxdata);
}

static ssize_t atmf04_store_read_reg(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);
    int loop;
    int ret = 0;
    unsigned long val;
    bool flag_match = false;

    client = data->client;
    PINFO("input :%s", buf);

    if (kstrtoul(buf, 0, &val))
        return -EINVAL;

    for (loop = 0; loop < CNT_INITCODE; loop++) {
        PINFO("###### loop:%d [0x%x][0x%x]###", loop,InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
        if( val ==  InitCodeAddr[loop])
        {
            PINFO("match Addr :%s", buf);
            flag_match = true;
            break;
        }
    }

    if(flag_match)
    {
        ret = i2c_smbus_read_byte_data(client, InitCodeAddr[loop]);
        PINFO("read register [0x%x][0x%x]",InitCodeAddr[loop],i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
    }
    else
    {
        PINFO("match Addr fail");
    }

    return count;
}

static ssize_t atmf04_store_write_reg(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);
    unsigned char loop;
    bool flag_match = false;
    unsigned int addr, val;
    client = data->client;

    PINFO("input :%s", buf);

    if (sscanf(buf, "%x %x", &addr, &val) <= 0)
        return count;

    for (loop = 0; loop < CNT_INITCODE; loop++) {
        PINFO("###### [0x%x][0x%x]###", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
        if( addr ==  InitCodeAddr[loop])
        {
            PINFO("match Addr :%s", buf);
            flag_match = true;
            break;
        }
    }

    if(flag_match)
    {
        i2c_smbus_write_byte_data(client, InitCodeAddr[loop], val);
        PINFO("write register ##[0x%x][0x%x]##", InitCodeAddr[loop], val);
        i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x04);
        mdelay(50);		//50ms delay
        i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x01);
        // return sprintf(buf,"0x%02x\n",i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
        return count;
    }
    else
        return count;
}

static ssize_t atmf04_show_regproxctrl0(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    PINFO("atmf04_show_regproxctrl0: %d\n",on_sensor);
    if(on_sensor==true)
        return sprintf(buf,"0x%02x\n",0x0C);
    return sprintf(buf,"0x%02x\n",0x00);
}

static ssize_t atmf04_store_reg(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);
    unsigned char loop;
    client = data->client;

    for (loop = 0; loop < CNT_INITCODE; loop++) {
        i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
        PINFO("##[0x%x][0x%x]##", InitCodeAddr[loop], InitCodeVal[loop]);
    }
    i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x01);
    return count;
}

static ssize_t atmf04_show_proxstatus(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret ;
    struct atmf04_data *data = dev_get_drvdata(dev);
    ret = gpio_get_value(data->platform_data->irq_gpio);
    return sprintf(buf, "%d\n", ret);
}


static ssize_t atmf04_store_onoffsensor(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct atmf04_data *data = dev_get_drvdata(dev);
    unsigned long val;
    struct i2c_client *client = to_i2c_client(dev);
    client = data->client;
    if (kstrtoul(buf, 0, &val))
        return -EINVAL;

    if(val == ON_SENSOR) {
        onoff_sensor(data,ENABLE_SENSOR_PINS);
        PINFO("Store ON_SENSOR[%d]",data->platform_data->irq_gpio);
    }
    else if (val == OFF_SENSOR) {
        PINFO("Store OFF_SENSOR[%d]",data->platform_data->irq_gpio);
        onoff_sensor(data,DISABLE_SENSOR_PINS);
    }
    return count;
}

static ssize_t atmf04_show_docalibration(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);
    unsigned char  safe_duty;
    client = data->client;

    /* check safe duty for validation of cal*/
    safe_duty = i2c_smbus_read_byte_data(client, I2C_ADDR_SAFE_DUTY_CHK);

    return sprintf(buf, "%d\n", get_bit(safe_duty, 7));
}

static ssize_t atmf04_store_docalibration(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);
    int ret, init_state;
    unsigned char  safe_duty;
    //==========================================================================//
    //[190326] ADS Add
    //[START]===================================================================//
    unsigned char loop, rd_val;
    //[END]=====================================================================//

    client = data->client;

    PINFO("atmf04_store_docalibration[%d]",data->platform_data->irq_gpio);
    init_state = write_initcode(client);
    PINFO("write_initcode End[%d]",data->platform_data->irq_gpio);

#if 1 // debugging calibration paused
    if(init_state) {
        PINFO("%s: write_initcode result is failed.....\n", __FUNCTION__);
        cal_result = false;
        write_calibration_data(data, PATH_CAPSENSOR_CAL);
        return count;
    } else {
        PINFO("%s: write_initcode result is successful..... continue next step!!!\n", __FUNCTION__);
    }
#else
    if(init_state)
        return count;
#endif

    mutex_lock(&data->update_lock);
    ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x0C);
    mutex_unlock(&data->update_lock);
    if(ret)
        PINFO("i2c_write_fail\n");

    mdelay(800);
    check_firmware_ready(client);

    //==========================================================================//
    //[190314] ADS Change
    //[START]===================================================================//
    ////==========================================================================//
    ////[20190228] ADS Add
    ////==========================================================================//
    //    mutex_lock(&data->update_lock);
    //	//============================================================//
    //	//[190306] ADS Delete
    //	//[START]=====================================================//
    //    //check_firmware_ready(client);
    //	//[END]=====================================================//
    //    ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x02);       //SW Reset2
    //    mutex_unlock(&data->update_lock);
    //    if(ret)
    //        PINFO("i2c_write_fail: ret = %d\n", ret);
    //
    //    check_firmware_ready(client);
    ////==========================================================================//
    gpio_set_value(data->platform_data->chip_enable, 1);
    mdelay(10);
    gpio_set_value(data->platform_data->chip_enable, 0);
    //==========================================================================//
    //[190319] ADS Add
    //[START]===================================================================//
    mdelay(50);
    //[END]=====================================================================//
    check_firmware_ready(client);
    //[END]=====================================================================//

    safe_duty = i2c_smbus_read_byte_data(client, I2C_ADDR_SAFE_DUTY_CHK);

    PINFO("[%u]safe_duty : %d", data->platform_data->irq_gpio, get_bit(safe_duty, 7));

#if 1 // debugging calibration paused
    if(ret || (get_bit(safe_duty, 7) != 1)) {
        // There is i2c failure or safe duty bit is not 1
        PINFO("%s: calibration result is failed.....\n", __FUNCTION__);
        cal_result = false;
    } else {
        PINFO("%s: calibration result is successful!!!!!\n", __FUNCTION__);
        cal_result = true;
    }
#endif

    //==========================================================================//
    //[190326] ADS Add
    //[START]===================================================================//
    for(loop = 0 ; loop < CNT_INITCODE ; loop++) 
    {
        rd_val = i2c_smbus_read_byte_data(client, InitCodeAddr[loop]);
        //==========================================================================//
        //[190327] ADS Add
        //[START]===================================================================//
        if( (InitCodeAddr[loop] == 0x1C) || (InitCodeAddr[loop] == 0x1D) ) rd_val = rd_val & 0x7F;
        //[END]=====================================================================//

        if (InitCodeVal[loop] != rd_val)
        {
            PINFO("%s: Invalid Init Code[Addr:0x%x][Save Value:0x%x][Read Value:0x%x]", __FUNCTION__,InitCodeAddr[loop], InitCodeVal[loop], rd_val);
            cal_result = false;
        }
    }
    //[END]=====================================================================//

    write_calibration_data(data, PATH_CAPSENSOR_CAL);

    return count;
}

static ssize_t atmf04_store_regreset(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);
    short tmp;
    short cs_per[2], cs_per_result;
    short cr_duty[2], cs_duty[2], cr_duty_val, cs_duty_val;
#ifdef CONFIG_LGE_ATMF04_2CH
    short cs_per_ch2[2], cs_per_result_ch2;
    short cs_duty_ch2[2], cs_duty_val_ch2;
#endif
    int ret;
    client = data->client;

#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
    check_firmware_ready(client);
#endif

#if 0 // debugging calibration paused   /* 2019.04.01, LGE want to keep the result of calibration */
    // Whether cal is pass or fail, make it cal_result true to check raw data/CS/CR/count in bypass mode of AAT
    cal_result = true;
    write_calibration_data(data, PATH_CAPSENSOR_CAL);
#endif

    ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x02);
    if(ret)
        PINFO("[%d]i2c_write_fail\n",data-> platform_data->irq_gpio);

    // CR Duty value 
    cr_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H);
    cr_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L);
    cr_duty_val = MK_INT(cr_duty[1], cr_duty[0]);

    // Ch1 Per value
    cs_per[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_H);
    cs_per[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_L);
    tmp = MK_INT(cs_per[0], cs_per[1]);
    cs_per_result = tmp / 8;    // BIT_PERCENT_UNIT;

    // Ch1 CS Duty value
    cs_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_H);
    cs_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_L);
    cs_duty_val = MK_INT(cs_duty[1], cs_duty[0]);

    PINFO("[%d] Result(ch1): %2d %6d %6d", data-> platform_data->irq_gpio, cs_per_result, cr_duty_val, cs_duty_val);

#ifdef CONFIG_LGE_ATMF04_2CH
    // Ch2 Per value
    cs_per_ch2[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_CH2_H);
    cs_per_ch2[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_CH2_L);
    tmp = MK_INT(cs_per_ch2[0], cs_per_ch2[1]);
    cs_per_result_ch2 = tmp / 8;    // BIT_PERCENT_UNIT;

    // Ch2 CS Duty value
    cs_duty_ch2[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_H);
    cs_duty_ch2[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_L);
    cs_duty_val_ch2 = MK_INT(cs_duty_ch2[1], cs_duty_ch2[0]);

    PINFO("[%d] Result(ch2): %2d %6d %6d", data-> platform_data->irq_gpio, cs_per_result_ch2, cr_duty_val, cs_duty_val_ch2);
#endif

    return count;
}

#if defined (CONFIG_LGE_USE_DD_SAR_RESET)
int atmf04_dd_sar_reset(void) {
    struct i2c_client *client = atmf04_i2c_client;
    int ret = 0;
    if(client != NULL && on_sensor) //only doing reset 1. probing ok 2. sensor enabled
    {
        struct atmf04_data *data = i2c_get_clientdata(client);
        short tmp;
        short cs_per[2], cs_per_result;
        short cr_duty[2], cs_duty[2], cr_duty_val, cs_duty_val;
#ifdef CONFIG_LGE_ATMF04_2CH
        short cs_per_ch2[2], cs_per_result_ch2;
        short cs_duty_ch2[2], cs_duty_val_ch2;
#endif
        PINFO("atmf04 reset enter \n");

        client = data->client;

#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
        check_firmware_ready(client);
#endif

#if 0 // atmf04_dd_sar_reset only reset - debugging calibration paused
        // Whether cal is pass or fail, make it cal_result true to check raw data/CS/CR/count in bypass mode of AAT
        cal_result = true;
        write_calibration_data(data, PATH_CAPSENSOR_CAL);
#endif

        ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x02);
        if(ret)
            PINFO("[%d]i2c_write_fail\n",data-> platform_data->irq_gpio);

        // CR Duty value
        cr_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H);
        cr_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L);
        cr_duty_val = MK_INT(cr_duty[1], cr_duty[0]);

        // Ch1 Per value
        cs_per[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_H);
        cs_per[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_L);
        tmp = MK_INT(cs_per[0], cs_per[1]);
        cs_per_result = tmp / 8;	// BIT_PERCENT_UNIT;

        // Ch1 CS Duty value
        cs_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_H);
        cs_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_L);
        cs_duty_val = MK_INT(cs_duty[1], cs_duty[0]);

        PINFO("[%d] Result(ch1): %2d %6d %6d", data-> platform_data->irq_gpio, cs_per_result, cr_duty_val, cs_duty_val);

#ifdef CONFIG_LGE_ATMF04_2CH
        // Ch2 Per value
        cs_per_ch2[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_CH2_H);
        cs_per_ch2[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_CH2_L);
        tmp = MK_INT(cs_per_ch2[0], cs_per_ch2[1]);
        cs_per_result_ch2 = tmp / 8;	// BIT_PERCENT_UNIT;

        // Ch2 CS Duty value
        cs_duty_ch2[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_H);
        cs_duty_ch2[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_L);
        cs_duty_val_ch2 = MK_INT(cs_duty_ch2[1], cs_duty_ch2[0]);

        PINFO("[%d] Result(ch2): %2d %6d %6d", data-> platform_data->irq_gpio, cs_per_result_ch2, cr_duty_val, cs_duty_val_ch2);
#endif
    }
    else
    {
        PINFO("atmf04 doesn't support, so skip\n");
    }
    return ret;
}
#endif
static int get_bit(unsigned short x, int n) {
    return (x & (1 << n)) >> n;
}

/*
   static short get_abs(short x) {
   return ((x >= 0) ? x : -x);
   }
 */
static ssize_t atmf04_show_regproxdata(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);
    short cs_per[2], cs_per_result;
    short cr_duty[2], cs_duty[2], cr_duty_val, cs_duty_val;
#ifdef CONFIG_LGE_ATMF04_2CH
    short cs_per_ch2[2], cs_per_result_ch2;
    short cs_duty_ch2[2], cs_duty_val_ch2;
    short cap_value_ch2;
#endif
    short tmp, cap_value;
    int nlength = 0;
    char buf_regproxdata[256] = "";
    char buf_line[256] = "";
    unsigned char init_touch_md;
#if defined(CONFIG_LGE_CAP_SENSOR_AUTO_CAL_ALGORITHM)	 /*auto calibration FW*/
    int check_mode;
#endif
    client = data->client;
    memset(buf_line, 0, sizeof(buf_line));
    memset(buf_regproxdata, 0, sizeof(buf_regproxdata));

    // fix that read calibration done register in MID
    check_firmware_ready(client);

    init_touch_md = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

    cs_per[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_H);
    cs_per[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_L);
    tmp = MK_INT(cs_per[0], cs_per[1]);
    cs_per_result = tmp / 8;    // BIT_PERCENT_UNIT;

    cr_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H);
    cr_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L);
    cr_duty_val = MK_INT(cr_duty[1], cr_duty[0]);

    cs_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_H);
    cs_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_L);
    cs_duty_val = MK_INT(cs_duty[1], cs_duty[0]);

    cap_value = (int)cs_duty_val * (int)cs_per_result;

#ifdef CONFIG_LGE_ATMF04_2CH
    // Ch2 Per value
    cs_per_ch2[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_CH2_H);
    cs_per_ch2[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_CH2_L);
    tmp = MK_INT(cs_per_ch2[0], cs_per_ch2[1]);
    cs_per_result_ch2 = tmp / 8;    // BIT_PERCENT_UNIT;

    // Ch2 CS Duty value
    cs_duty_ch2[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_H);
    cs_duty_ch2[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_L);
    cs_duty_val_ch2 = MK_INT(cs_duty_ch2[1], cs_duty_ch2[0]);

    cap_value_ch2 = (int)cs_duty_val_ch2 * (int)cs_per_result_ch2;


#if defined(CONFIG_LGE_CAP_SENSOR_AUTO_CAL_ALGORITHM)	/*auto calibration FW*/
    check_mode = get_bit(init_touch_md, 2);

    /* Normal Mode */
    if (check_mode) {
        sprintf(buf_line, "[R]%6d %6d %6d %6d %6d %6d %6d %6d\n",
                get_bit(init_touch_md,2), cs_per_result, cs_per_result_ch2, cr_duty_val, cs_duty_val, cs_duty_val_ch2, cap_value, cap_value_ch2);
    }
    /* Init Touch Mode */
    else
#endif
        sprintf(buf_line, "[R]%6d %6d %6d %6d %6d %6d %6d %6d\n", 
                get_bit(init_touch_md,1), cs_per_result, cs_per_result_ch2, cr_duty_val, cs_duty_val, cs_duty_val_ch2, cap_value, cap_value_ch2);

#else
    // printk("H: %x L:%x H:%x L:%x\n",cr_duty[1] ,cr_duty[0], cs_duty[1], cs_duty[0]);
#if defined(CONFIG_LGE_CAP_SENSOR_AUTO_CAL_ALGORITHM)	/*auto calibration FW*/
    check_mode = get_bit(init_touch_md, 2);

    if (check_mode)		/* Normal Mode */
        sprintf(buf_line, "[R]%6d %6d %6d %6d %6d\n", get_bit(init_touch_md,2), cs_per_result, cr_duty_val, cs_duty_val, cap_value);
    else		/* Init Touch Mode */
#endif
        sprintf(buf_line, "[R]%6d %6d %6d %6d %6d\n", get_bit(init_touch_md,1), cs_per_result, cr_duty_val, cs_duty_val, cap_value);
#endif
    nlength = strlen(buf_regproxdata);
    strcpy(&buf_regproxdata[nlength], buf_line);

    return sprintf(buf, "%s", buf_regproxdata);
}

static ssize_t atmf04_store_checkallnear(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long val;

    if (kstrtoul(buf, 0, &val))
        return -EINVAL;

    if (val == 0)
        check_allnear = false;
    else if (val == 1)
        check_allnear = true;

    printk("atmf04_store_checkallnear %d\n",check_allnear);
    return count;
}

static ssize_t atmf04_show_count_inputpins(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int count_inputpins = 0;

    struct atmf04_data *data = dev_get_drvdata(dev);

    count_inputpins = data->platform_data->input_pins_num;
    if (count_inputpins > 1) {
        if (valid_multiple_input_pins(data) == false)
            count_inputpins = 1;
    }
    return sprintf(buf, "%d\n", count_inputpins);
}

static ssize_t atmf04_store_firmware(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    const char *fw_name = NULL;
    struct atmf04_data *data = dev_get_drvdata(dev);
    struct i2c_client *client = to_i2c_client(dev);
    client = data->client;

    fw_name = data->platform_data->fw_name;
    load_firmware(data, client, fw_name);
    return count;
}

static ssize_t atmf04_show_version(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned short main_version, sub_version;
    unsigned char ic_fw_version;
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = dev_get_drvdata(dev);
    char buf_line[256] = "";
    int nlength = 0;
    char buf_regproxdata[256] = "";
    client = data->client;

    memset(buf_line, 0, sizeof(buf_line));
    onoff_sensor(data,ENABLE_SENSOR_PINS);

    mdelay(200);

    main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
    sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);
    ic_fw_version = MK_INT(main_version, sub_version);
    PINFO("###########ic version : %d.%d, ic_fw_version : %d###########", main_version, sub_version, ic_fw_version);

    sprintf(buf_line, "%d.%d\n",main_version, sub_version);
    nlength = strlen(buf_regproxdata);
    strcpy(&buf_regproxdata[nlength], buf_line);

    return sprintf(buf,"%s", buf_regproxdata);
}

static ssize_t atmf04_show_check_far(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);
    unsigned char init_touch_md_check;
    short tmp, cs_per[2], cs_per_result;
    short cr_duty[2], cs_duty[2], cr_duty_val, cs1_duty_val;
#if defined(CONFIG_LGE_ATMF04_2CH)
    short cs2_duty_val;
#endif
    int bit_mask = 1; //bit for reading of I2C_ADDR_SYS_STAT
    unsigned int check_result = 0;
    int poor_contact = 0;
    client = data->client;

    mutex_lock(&data->update_lock);

#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
    check_init_touch_ready(client);
#endif

    cs_per[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_H);
    cs_per[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_L);
    tmp = MK_INT(cs_per[0], cs_per[1]);
    cs_per_result = tmp / 8;    // BIT_PERCENT_UNIT;

    cr_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H);
    cr_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L);
    cr_duty_val = MK_INT(cr_duty[1], cr_duty[0]);

    cs_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_H);
    cs_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_L);
    cs1_duty_val = MK_INT(cs_duty[1], cs_duty[0]);

#if defined(CONFIG_LGE_ATMF04_2CH)
    cs_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_H);
    cs_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_L);
    cs2_duty_val = MK_INT(cs_duty[1], cs_duty[0]);
#endif
    init_touch_md_check = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
#if defined(CONFIG_LGE_CAP_SENSOR_AUTO_CAL_ALGORITHM) /*auto calibration FW*/
    if(get_bit(init_touch_md_check, 2))
        bit_mask = 2;  /* Normal Mode */
    else
        bit_mask = 1;  /* Init Touch Mode */
#endif

    /* Check poor contact */
    //============================================================//
    //[190318] ADS Change
    //[START]=====================================================//
    //if (get_bit(init_touch_md_check, 4) == 1)
    if (get_bit(init_touch_md_check, 5) == 1)
        //[END]======================================================//
    {
        poor_contact = 1;
        check_result += 1;
    }

    /* Calibration Check */
    if (get_bit(init_touch_md_check, bit_mask) != 1)
    {
        check_result += 1;
        PINFO("fail calibration check");
    }

    /* CR Duty Range Check */
    if ((cr_duty_val < ATMF04_CR_DUTY_LOW) || (cr_duty_val > ATMF04_CR_DUTY_HIGH)){
        check_result += 1;
        PINFO("fail CR duty range");
    }

    /* CS Duty Rnage Check */
    if ((cs1_duty_val < ATMF04_CS_DUTY_LOW) || (cs1_duty_val > ATMF04_CS_DUTY_HIGH) || (cs2_duty_val < ATMF04_CS_DUTY_LOW) || (cs1_duty_val > ATMF04_CS_DUTY_HIGH)){
        check_result += 1;
        PINFO("fail CS duty range");
    }

    if(gpio_get_value(data->platform_data->irq_gpio) == 1)
    {
        PINFO(" ATMF04 Check_Far IRQ Status NEAR[%d]\n",gpio_get_value(data->platform_data->irq_gpio));
        PINFO("fail gpio state");
        check_result += 1;
    }

    if (check_result != 0)
    {
        PINFO(" ATMF04 [fail] 1.cal: %d, cr: %d, cs_ch1: %d, cs_ch2: %d, poor_contact: %d\n",
                get_bit(init_touch_md_check, bit_mask), cr_duty_val, cs1_duty_val, cs2_duty_val, poor_contact);
        mutex_unlock(&data->update_lock);
        return sprintf(buf,"%d",0);
    }
    else
    {
        PINFO(" ATMF04 [PASS] 2.cal: %d, cr: %d, cs_ch1: %d, cs_ch2: %d, poor_contact: %d\n",
                get_bit(init_touch_md_check, bit_mask), cr_duty_val, cs1_duty_val, cs2_duty_val, poor_contact);
        mutex_unlock(&data->update_lock);
        return sprintf(buf,"%d",1);
    }
}

static ssize_t atmf04_show_check_mid(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);
    unsigned char init_touch_md_check;
    short tmp, cs_per[2], cs_per_result;
    short cr_duty[2], cs_duty[2], cr_duty_val, cs1_duty_val, cs2_duty_val;
    int bit_mask = 1; //bit for reading of I2C_ADDR_SYS_STAT
    unsigned int check_result = 0;
    int poor_contact = 0;

    client = data->client;

    mutex_lock(&data->update_lock);

#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
    check_init_touch_ready(client);
#endif

    cs_per[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_H);
    cs_per[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_L);
    tmp = MK_INT(cs_per[0], cs_per[1]);
    cs_per_result = tmp / 8;    // BIT_PERCENT_UNIT;

    cr_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H);
    cr_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L);
    cr_duty_val = MK_INT(cr_duty[1], cr_duty[0]);

    cs_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_H);
    cs_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_L);
    cs1_duty_val = MK_INT(cs_duty[1], cs_duty[0]);

    cs_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_H);
    cs_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_L);
    cs2_duty_val = MK_INT(cs_duty[1], cs_duty[0]);

    init_touch_md_check = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
#if defined(CONFIG_LGE_CAP_SENSOR_AUTO_CAL_ALGORITHM) /*auto calibration FW*/
    if(get_bit(init_touch_md_check, 2))
        bit_mask = 2;  /* Normal Mode */
    else
        bit_mask = 1;  /* Init Touch Mode */
#endif

    /* Calibration Check */
    if (get_bit(init_touch_md_check, bit_mask) != 1)
    {
        check_result += 1;
    }
    /* CR Duty Range Check */
    if ((cr_duty_val < ATMF04_CR_DUTY_LOW) || (cr_duty_val > ATMF04_CR_DUTY_HIGH))
    {
        check_result += 1;
    }
    /* CS Duty Rnage Check */
    if ((cs1_duty_val < ATMF04_CS_DUTY_LOW) || (cs1_duty_val > ATMF04_CS_DUTY_HIGH) || (cs2_duty_val < ATMF04_CS_DUTY_LOW) || (cs1_duty_val > ATMF04_CS_DUTY_HIGH))
    {
        check_result += 1;
    }

    /* Check poor contact */
    //============================================================//
    //[190318] ADS Change
    //[START]=====================================================//
    //if (get_bit(init_touch_md_check, 4) == 1)
    if (get_bit(init_touch_md_check, 5) == 1)
        //[END]======================================================//
    {
        poor_contact = 1;
        check_result += 1;
    }

    if (check_result != 0)
    {
        PINFO(" ATMF04 [fail] 1.cal: %d, cr: %d, cs_ch1: %d, cs_ch2: %d, poor_contact: %d\n",
                get_bit(init_touch_md_check, bit_mask), cr_duty_val, cs1_duty_val, cs2_duty_val, poor_contact);
        mutex_unlock(&data->update_lock);
        return sprintf(buf,"%d",0);
    }
    else
    {
        PINFO(" ATMF04 [PASS] 2.cal: %d, cr: %d, cs_ch1: %d, cs_ch2: %d, poor_contact: %d\n",
                get_bit(init_touch_md_check, bit_mask), cr_duty_val, cs1_duty_val, cs2_duty_val, poor_contact);
        mutex_unlock(&data->update_lock);
        return sprintf(buf,"%d",1);
    }
}

static DEVICE_ATTR(onoff,        0664, NULL, atmf04_store_onoffsensor);
static DEVICE_ATTR(proxstatus,   0664, atmf04_show_proxstatus, NULL);
static DEVICE_ATTR(docalibration,0664, atmf04_show_docalibration, atmf04_store_docalibration);
static DEVICE_ATTR(reg_ctrl,     0664, atmf04_show_reg, atmf04_store_reg);
static DEVICE_ATTR(write_reg,    0664, NULL, atmf04_store_write_reg);
static DEVICE_ATTR(read_reg,     0664, NULL, atmf04_store_read_reg);
static DEVICE_ATTR(regproxdata,  0664, atmf04_show_regproxdata, NULL);
static DEVICE_ATTR(regreset,     0664, NULL, atmf04_store_regreset);
static DEVICE_ATTR(checkallnear, 0664, NULL, atmf04_store_checkallnear);
static DEVICE_ATTR(cntinputpins, 0664, atmf04_show_count_inputpins, NULL);
static DEVICE_ATTR(regproxctrl0, 0664, atmf04_show_regproxctrl0, NULL);
static DEVICE_ATTR(download,     0664, NULL, atmf04_store_firmware);
static DEVICE_ATTR(version,      0664, atmf04_show_version, NULL);
static DEVICE_ATTR(check_far,    0664, atmf04_show_check_far, NULL);
static DEVICE_ATTR(check_mid,    0664, atmf04_show_check_mid, NULL);

static struct attribute *atmf04_attributes[] = {
    &dev_attr_onoff.attr,
    &dev_attr_docalibration.attr,
    &dev_attr_proxstatus.attr,
    &dev_attr_reg_ctrl.attr,
    &dev_attr_write_reg.attr,
    &dev_attr_read_reg.attr,
    &dev_attr_regproxdata.attr,
    &dev_attr_regreset.attr,
    &dev_attr_checkallnear.attr,
    &dev_attr_cntinputpins.attr,
    &dev_attr_regproxctrl0.attr,
    &dev_attr_download.attr,
    &dev_attr_version.attr,
    &dev_attr_check_far.attr,
    &dev_attr_check_mid.attr,
    NULL,
};

static struct attribute_group atmf04_attr_group = {
    .attrs = atmf04_attributes,
};

static void atmf04_reschedule_work(struct atmf04_data *data,
        unsigned long delay)
{
    int ret;
    struct i2c_client *client = data->client;
    /*
     * If work is already scheduled then subsequent schedules will not
     * change the scheduled time that's why we have to cancel it first.
     */
    if (gpio_get_value(data->platform_data->chip_enable) == 0) { // if power on
        dev_err(&client->dev, "atmf04_reschedule_work : set wake lock timeout!\n");
        pm_wakeup_event(&client->dev,1500);
        //cancel_delayed_work(&data->dwork);
        if (atomic_read(&pm_suspend_flag) == DEV_PM_RESUME){
            ret = queue_delayed_work(atmf04_workqueue, &data->dwork, delay);
            if (ret < 0) {
                PINFO("ATMF04 queue_work fail, ret = %d", ret);
            }
        } else {
            atomic_set(&pm_suspend_flag, DEV_PM_SUSPEND_IRQ);
            PINFO("I2C is not yet ready... try queue_delayed_work after resume");
        }
    } else {
        PINFO("ATMF04 cap sensor enable pin is already high... power off status");
    }
}

/* assume this is ISR */
static irqreturn_t atmf04_interrupt(int vec, void *info)
{
    struct i2c_client *client = (struct i2c_client *)info;
    struct atmf04_data *data = i2c_get_clientdata(client);
    int tmp = -1;
    tmp = atomic_read(&data->i2c_status);

    if (gpio_get_value(data->platform_data->chip_enable) == 0) { // if power on
        dev_err(&client->dev,"atmf04_interrupt\n");
        atmf04_reschedule_work(data, 0);
    }

    return IRQ_HANDLED;
}

static void atmf04_work_handler(struct work_struct *work)
{
    struct atmf04_data *data = container_of(work, struct atmf04_data, dwork.work);
    struct i2c_client *client = data->client;
    int irq_state;

#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
    if(probe_end_flag == true) {
#endif
        data->touch_out = i2c_smbus_read_byte_data(client, I2C_ADDR_TCH_OUTPUT);
        irq_state = gpio_get_value(data->platform_data->irq_gpio);
        PINFO("ATMF04 touch_out[%x] irq_state[%d]", data->touch_out, irq_state);

        /* When I2C fail and abnormal status*/
        if (data->touch_out < 0)
        {
            PINFO("ATMF04 I2C Error[%d]", data->touch_out);
            input_report_abs(data->input_dev_cap, ABS_DISTANCE, NEAR_STATUS);/* FAR-to-NEAR detection */
            input_sync(data->input_dev_cap);

            PINFO("ATMF04 [%d]NEAR ",data->platform_data->irq_gpio);
        }
        else
        {
            if (gpio_get_value(data->platform_data->chip_enable) == 1) { // if power off
                PINFO("ATMF04 cap sensor is already power off : touch_out(0x%x)", data->touch_out);
                return;
            }
#ifdef CONFIG_LGE_ATMF04_2CH
            /* FAR */
            if (irq_state == 0 && data->touch_out == (CH2_FAR|CH1_FAR))
            {
                data->cap_detection = 0;

                input_report_abs(data->input_dev_cap, ABS_DISTANCE, FAR_STATUS);/* NEAR-to-FAR detection */
                input_sync(data->input_dev_cap);

                PINFO("ATMF04 [%d]FAR ",data->platform_data->irq_gpio);
            }
            /* NEAR */
            else
            {
                data->cap_detection = 1;

                input_report_abs(data->input_dev_cap, ABS_DISTANCE, NEAR_STATUS);/* FAR-to-NEAR detection */
                input_sync(data->input_dev_cap);

                PINFO("ATMF04 [%d]NEAR ",data->platform_data->irq_gpio);
            }
            PINFO("ATMF04 [%d] Work Handler done",data->platform_data->irq_gpio);
#else
            /* Occurred touch event */
            if(data->touch_out)
            {
                //	if ((data->touch_out == 1 ) && (data->cap_datection == 0)) {
                data->cap_detection = 1;

                input_report_abs(data->input_dev_cap, ABS_DISTANCE, NEAR_STATUS);/* FAR-to-NEAR detection */
                input_sync(data->input_dev_cap);

                PINFO("ATMF04 [%d]NEAR ",data->platform_data->irq_gpio);
            }
            else
            {
                data->cap_detection = 0;

                input_report_abs(data->input_dev_cap, ABS_DISTANCE, FAR_STATUS);/* NEAR-to-FAR detection */
                input_sync(data->input_dev_cap);

                PINFO("ATMF04 [%d]FAR ",data->platform_data->irq_gpio);
            }
            PINFO("ATMF04 [%d] Work Handler done",data->platform_data->irq_gpio);
#endif
            }
#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
        }
        else{
            PINFO("ATMF04 probe_end_flag = False[%d]",probe_end_flag);
        }
#endif

    }

    static int sensor_regulator_configure(struct atmf04_data *data, bool on)
    {
        struct i2c_client *client = data->client;
        struct atmf04_platform_data *pdata = data->platform_data;
        int rc;

        if (on == false)
            goto hw_shutdown;

        pdata->vcc_ana = regulator_get(&client->dev, "Adsemicon,vdd_ana");
        if (IS_ERR(pdata->vcc_ana)) {
            rc = PTR_ERR(pdata->vcc_ana);
            dev_err(&client->dev,
                    "Regulator get failed vcc_ana rc=%d\n", rc);
            return rc;
        }

        if (regulator_count_voltages(pdata->vcc_ana) > 0) {
            rc = regulator_set_voltage(pdata->vcc_ana, pdata->vdd_ana_supply_min,
                    pdata->vdd_ana_supply_max);

            if (rc) {
                dev_err(&client->dev,
                        "regulator set_vtg failed rc=%d\n", rc);
                goto error_set_vtg_vcc_ana;
            }
        }

        return 0;

error_set_vtg_vcc_ana:
        regulator_put(pdata->vcc_ana);
        return rc;

hw_shutdown:
        if (regulator_count_voltages(pdata->vcc_ana) > 0)
            regulator_set_voltage(pdata->vcc_ana, 0, pdata->vdd_ana_supply_max);

        regulator_put(pdata->vcc_ana);
        regulator_put(pdata->vcc_dig);

        if (pdata->i2c_pull_up) {
            regulator_put(pdata->vcc_i2c);
        }
        return 0;
    }

    static int sensor_regulator_power_on(struct atmf04_data *data, bool on)
    {
        struct i2c_client *client = data->client;
        struct atmf04_platform_data *pdata = data->platform_data;

        int rc;

        if (on == false)
            goto power_off;

        rc = regulator_set_load(pdata->vcc_ana, pdata->vdd_ana_load_ua);
        if (rc < 0) {
            dev_err(&client->dev,
                    "Regulator vcc_ana set_opt failed rc=%d\n", rc);
            return rc;
        }

        rc = regulator_enable(pdata->vcc_ana);
        if (rc) {
            dev_err(&client->dev,
                    "Regulator vcc_ana enable failed rc=%d\n", rc);
        }
        return 0;

power_off:
        regulator_disable(pdata->vcc_ana);
        if (pdata->i2c_pull_up) {
            regulator_disable(pdata->vcc_i2c);
        }
        return 0;
    }

    static int sensor_platform_hw_power_on(struct i2c_client *client, bool on)
    {
        sensor_regulator_power_on(i2c_get_clientdata(client), on);
        return 0;
    }

    static int sensor_platform_hw_init(struct i2c_client *client)
    {
        struct atmf04_data *data = i2c_get_clientdata(client);
        int error;

        error = sensor_regulator_configure(data, true);

        error = gpio_request(data->platform_data->chip_enable, "atmf04_chip_enable");
        if(error) {
            PINFO(" ATMF04 chip_enable request fail\n");
        }
        gpio_direction_output(data->platform_data->chip_enable, 0); // First, set chip_enable gpio direction to output.

        PINFO(" ATMF04 gpio direction output ok\n");

#if defined (CONFIG_LGE_CAP_SENSOR_BOOTING_TIME_IMPROVEMENT)
        error = gpio_request(data->platform_data->chip_enable2, "atmf04_chip_enable2");
        if(error) {
            PINFO(" ATMF04 chip_enable2 request fail\n");
        }
        gpio_direction_output(data->platform_data->chip_enable2, 0); // enable first, to reduce booting time.
        PINFO(" ATMF04 set chip_enable2 power on succesfully\n");
        gpio_free(data->platform_data->chip_enable2);
        PINFO(" ATMF04 set chip_enable2 free\n");
#endif

        if (gpio_is_valid(data->platform_data->irq_gpio)) {
            /* configure touchscreen irq gpio */
            error = gpio_request(data->platform_data->irq_gpio, "atmf04_irq_gpio");
            if (error) {
                PINFO(" ATMF04 unable to request gpio [%d]\n",
                        data->platform_data->irq_gpio);
            }
            error = gpio_direction_input(data->platform_data->irq_gpio);
            if (error) {
                PINFO(" ATMF04 unable to set direction for gpio [%d]\n",
                        data->platform_data->irq_gpio);
            }
            data->irq = client->irq = gpio_to_irq(data->platform_data->irq_gpio);
        } else {
            PINFO(" ATMF04 irq gpio not provided\n");
        }
        PINFO(" ATMF04 sensor_platform_hw_init end\n");
        return 0;
    }

    static void sensor_platform_hw_exit(struct i2c_client *client)
    {
        struct atmf04_data *data = i2c_get_clientdata(client);;

        sensor_regulator_configure(data, false);

        if (gpio_is_valid(data->platform_data->irq_gpio))
            gpio_free(data->platform_data->irq_gpio);
        PINFO(" ATMF04 sensor_platform_hw_exit entered\n");
    }

    static int sensor_parse_dt(struct device *dev,
            struct atmf04_platform_data *pdata)
    {
        struct device_node *np = dev->of_node;

        int ret, err = 0;
        struct sensor_dt_to_pdata_map *itr;
        struct sensor_dt_to_pdata_map map[] = {
#if defined (CONFIG_MACH_SDM845_JUDYPN)
            {"Adsemicon,irq-gpio",		&pdata->irq_gpio,		DT_REQUIRED,	DT_GPIO,	0},
            {"Adsemicon,irq-gpio2",		&pdata->irq_gpio2,		DT_REQUIRED,	DT_GPIO,	0},
#else
            {"Adsemicon,irq-gpio",		&pdata->irq_gpio,		DT_REQUIRED,	DT_GPIO,	0},
#endif
            {"Adsemicon,vdd_ana_supply_min",	&pdata->vdd_ana_supply_min,	DT_SUGGESTED,	DT_U32,		0},
            {"Adsemicon,vdd_ana_supply_max",	&pdata->vdd_ana_supply_max,	DT_SUGGESTED,	DT_U32,		0},
            {"Adsemicon,vdd_ana_load_ua",	&pdata->vdd_ana_load_ua,	DT_SUGGESTED,	DT_U32,		0},
            {"Adsemicon,chip_enable",   &pdata->chip_enable,    DT_SUGGESTED,   DT_GPIO,     0},
            {"Adsemicon,InputPinsNum",         &pdata->input_pins_num,      DT_SUGGESTED,   DT_U32,  0},
            {"Adsemicon,fw_name",              &pdata->fw_name,             DT_SUGGESTED,   DT_STRING,  0},
#if defined (CONFIG_LGE_CAP_SENSOR_CURRENT_ISSUE) || defined (CONFIG_LGE_CAP_SENSOR_BOOTING_TIME_IMPROVEMENT)
            {"Adsemicon,chip_enable2",   &pdata->chip_enable2,    DT_SUGGESTED,   DT_GPIO,     0},
#endif
            /* add */
            {NULL,				NULL,				0,		0,		0},
        };

        for (itr = map; itr->dt_name ; ++itr) {
            switch (itr->type) {
                case DT_GPIO:
                    ret = of_get_named_gpio(np, itr->dt_name, 0);
                    if (ret >= 0) {
                        *((int *) itr->ptr_data) = ret;
                        ret = 0;
                    }
                    break;
                case DT_U32:
                    ret = of_property_read_u32(np, itr->dt_name,
                            (u32 *) itr->ptr_data);
                    break;
                case DT_BOOL:
                    *((bool *) itr->ptr_data) =
                        of_property_read_bool(np, itr->dt_name);
                    ret = 0;
                    break;
                case DT_STRING:
                    ret = of_property_read_string(np, itr->dt_name, itr->ptr_data);
                    break;
                default:
                    PINFO(" ATMF04 %d is an unknown DT entry type\n",
                            itr->type);
                    ret = -EBADE;
            }

            PINFO(" ATMF04 DT entry ret:%d name:%s val:%d\n",
                    ret, itr->dt_name, *((int *)itr->ptr_data));

            if (ret) {
                *((int *)itr->ptr_data) = itr->default_val;

                if (itr->status < DT_OPTIONAL) {
                    PINFO(" ATMF04 Missing '%s' DT entry\n",
                            itr->dt_name);

                    /* cont on err to dump all missing entries */
                    if (itr->status == DT_REQUIRED && !err)
                        err = ret;
                }
            }
        }

        /* set functions of platform data */
        pdata->init = sensor_platform_hw_init;
        pdata->exit = sensor_platform_hw_exit;
        pdata->power_on = sensor_platform_hw_power_on;
        /*pdata->ppcount = 12;	//no need to set, dt_parse */

        return err;

        return 0;
    }

    static int atmf04_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
    {
        struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
        struct atmf04_data *data;
#ifdef CONFIG_OF
        struct atmf04_platform_data *platform_data;
#endif
        int err = 0;

	/* Check for support by carrier */
	if (lge_get_capsensor() == 0)
		return -ENODEV;

        if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
            return -EIO;
        }

        data = devm_kzalloc(&client->dev, sizeof(struct atmf04_data), GFP_KERNEL);
        if (!data) {
            return -ENOMEM;
        }
        PINFO("ATMF04 probe, kzalloc complete\n");

#ifdef CONFIG_OF
        if (client->dev.of_node) {
            platform_data = devm_kzalloc(&client->dev,
                    sizeof(struct atmf04_platform_data), GFP_KERNEL);
            if (!platform_data) {
                dev_err(&client->dev, "Failed to allocate memory\n");
                return -ENOMEM;
            }
            data->platform_data = platform_data;
            client->dev.platform_data = platform_data;
            err = sensor_parse_dt(&client->dev, platform_data);
            if (err)
                return err;
#if defined(CONFIG_MACH_SDM845_JUDYPN)
            if(strcmp(lge_get_board_subrevision(),"subrev_0"))
            {
                PINFO("**Capsensor BLSP Change [%d] to [%d] for	subrev**",platform_data->irq_gpio, platform_data->irq_gpio2);
                platform_data->irq_gpio = platform_data->irq_gpio2;
            }
#endif
        } else {
            platform_data = client->dev.platform_data;
        }
#endif
        data->client = client;
        atmf04_i2c_client = client;
        i2c_set_clientdata(client, data);
        data->cap_detection = 0;

#ifdef CONFIG_OF
        /* h/w initialization */
        if (platform_data->init)
            err = platform_data->init(client);

        if (platform_data->power_on)
            err = platform_data->power_on(client, true);
#endif
        PINFO("**Capsensor BLSP[%d]",platform_data->irq_gpio);

        client->adapter->retries = 15;

        if (client->adapter->retries == 0)
            goto exit;

        atomic_set(&data->i2c_status, ATMF04_STATUS_RESUME);

        mutex_init(&data->update_lock);
        mutex_init(&data->enable_lock);

        INIT_DELAYED_WORK(&data->dwork, atmf04_work_handler);

        err = request_irq(client->irq, atmf04_interrupt, IRQ_TYPE_EDGE_FALLING|IRQ_TYPE_EDGE_RISING,
                ATMF04_DRV_NAME, (void *)client);
        if(err) {
            PINFO("ATMF04 chip_enable request fail\n");
            goto exit_irq_init_failed;
        }

        err = enable_irq_wake(data->irq);

        data->input_dev_cap = input_allocate_device();
        if (!data->input_dev_cap) {
            err = -ENOMEM;
            PINFO("ATMF04 Failed to allocate input device cap !\n");
            goto exit_free_irq;
        }

        set_bit(EV_ABS, data->input_dev_cap->evbit);

        input_set_abs_params(data->input_dev_cap, ABS_DISTANCE, 0, 1, 0, 0);

        data->input_dev_cap->name = ATMF04_DRV_NAME;
        data->input_dev_cap->dev.init_name = ATMF04_DRV_NAME;
        data->input_dev_cap->id.bustype = BUS_I2C;

        input_set_drvdata(data->input_dev_cap, data);

        err = input_register_device(data->input_dev_cap);
        if (err) {
            err = -ENOMEM;
            PINFO("ATMF04 Unable to register input device cap(%s)\n",
                    data->input_dev_cap->name);
            goto exit_free_dev_cap;
        }

        if (data->platform_data->fw_name) {
            err = load_firmware(data, client, data->platform_data->fw_name);
            if (err) {
                PINFO("ATMF04 Failed to request firmware\n");
                //goto exit_free_irq;
                goto exit_irq_init_failed;
            }
        }
        PINFO("sysfs create start!\n");
        err = sysfs_create_group(&data->input_dev_cap->dev.kobj, &atmf04_attr_group);
        if (err)
            PINFO("input sysfs create fail!\n");

        err = sysfs_create_group(&client->dev.kobj, &atmf04_attr_group);
        if (err)
            PINFO("ATMF04 sysfs create fail!\n");

        /* default sensor off */
#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
        probe_end_flag = true;
#endif
        onoff_sensor(data, DISABLE_SENSOR_PINS);
        PINFO("interrupt is hooked\n");

        atomic_set(&pm_suspend_flag, DEV_PM_RESUME); // initial value

        return 0;

exit_irq_init_failed:
        gpio_direction_output(data->platform_data->chip_enable, 1); // To avoid power leakage when probe fail
        mutex_destroy(&data->update_lock);
        mutex_destroy(&data->enable_lock);
        if(data->input_dev_cap != NULL) {
            input_unregister_device(data->input_dev_cap);
        }
exit_free_dev_cap:
exit_free_irq:
        free_irq(data->irq, client);
exit:
        PINFO("ATMF04 Error");
        return err;
    }

    static int atmf04_remove(struct i2c_client *client)
    {

        struct atmf04_data *data = i2c_get_clientdata(client);
        struct atmf04_platform_data *pdata = data->platform_data;


        disable_irq_wake(client->irq);

        free_irq(client->irq, client);

        if (pdata->power_on)
            pdata->power_on(client, false);

        if (pdata->exit)
            pdata->exit(client);

        mutex_destroy(&data->update_lock);
        mutex_destroy(&data->enable_lock);
        PINFO("ATMF04 remove\n");
        return 0;
    }

    static const struct i2c_device_id atmf04_id[] = {
        { "atmf04", 0 },
        { }
    };
    MODULE_DEVICE_TABLE(i2c, atmf04_id);

#ifdef CONFIG_OF
    static struct of_device_id atmf04_match_table[] = {
        { .compatible = "adsemicon,atmf04",},
        { },
    };
#else
#define atmf04_match_table NULL
#endif

    static int atmf04_pm_suspend(struct device *dev)
    {
        atomic_set(&pm_suspend_flag, DEV_PM_SUSPEND);
        return 0;
    }
    static int atmf04_pm_resume(struct device *dev)
    {
        struct i2c_client *client = to_i2c_client(dev);
        struct atmf04_data *data = i2c_get_clientdata(client);
        int ret = 0;

        if (atomic_read(&pm_suspend_flag) == DEV_PM_SUSPEND_IRQ) {
            if (gpio_get_value(data->platform_data->chip_enable) == 0) { // if power on
                PINFO("queue_delayed_work now.");
                ret = queue_delayed_work(atmf04_workqueue, &data->dwork, 0);
                if (ret < 0) {
                    PINFO("queue_work fail, ret = %d", ret);
                }
            }
            else {
                PINFO("cap sensor enable pin is already high... power off status");
            }
        }

        atomic_set(&pm_suspend_flag, DEV_PM_RESUME);
        return 0;
    }

    static const struct dev_pm_ops atmf04_pm_ops = {
        SET_SYSTEM_SLEEP_PM_OPS(
                atmf04_pm_suspend, //Get call when suspend is happening
                atmf04_pm_resume //Get call when resume is happening
                )
    };

    static struct i2c_driver atmf04_driver = {
        .driver = {
            .name   = ATMF04_DRV_NAME,
            .owner  = THIS_MODULE,
            .of_match_table = atmf04_match_table,
            .pm = &atmf04_pm_ops,
            .probe_type = PROBE_PREFER_ASYNCHRONOUS,
        },
        .probe  = atmf04_probe,
        .remove = atmf04_remove,
        .id_table = atmf04_id,
    };
    static void async_atmf04_init(void *data, async_cookie_t cookie)
    {
        PINFO("ATMF04 init Proximity driver: release.\n");
        atmf04_workqueue = create_workqueue("capsensor");
        i2c_add_driver(&atmf04_driver);
    }

    static int __init atmf04_init(void)
    {
        async_schedule(async_atmf04_init, NULL);
        return 0;
    }

    static void __exit atmf04_exit(void)
    {
        PINFO("ATMF04 Proximity driver: release.\n");
        if (atmf04_workqueue)
            destroy_workqueue(atmf04_workqueue);

        atmf04_workqueue = NULL;

        i2c_del_driver(&atmf04_driver);
    }

    MODULE_DESCRIPTION("ATMF04 cap sensor driver");
    MODULE_LICENSE("GPL");

    module_init(atmf04_init);
    module_exit(atmf04_exit);


