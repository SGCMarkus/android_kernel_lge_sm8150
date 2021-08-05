/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/

/*
 * AMS generic device driver
 */

/*
 * @@AMS_REVISION_Id: 969322ab678073812e97aab66a03df1d04e97836
 */
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include "ams_platform.h"
#include "ams_port_platform.h"
#include "osal_linux_input.h"


int i = 0;

 ssize_t osal_flicker_enable_set(struct amsDriver_chip *chip, uint8_t valueToSet)
{
	ssize_t rc = 0;

	rc |= ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_HW_FLICKER, AMS_CONFIG_ENABLE, valueToSet);
	return 0;
}


#if defined(CONFIG_AMS_OPTICAL_SENSOR_ALS)

#if 0
void osal_report_als(struct amsDriver_chip *chip)
{
	ams_apiAls_t outData;
	int mLux;

	if (chip->als_idev) {
		ams_deviceGetAls(chip->deviceCtx, &outData);
		mLux = outData.mLux;
		input_report_abs(chip->als_idev, ABS_MISC, mLux);
		input_sync(chip->als_idev);
	}
}
#endif

void osal_report_flicker(struct amsDriver_chip *chip)
{
    static unsigned int i = 0;   //shmoon_190612
    uint32_t temp = 0;
    uint32_t flicker = 0;

	if (chip->flicker_idev) {
#if 1
        ams_apiAlsFlicker_t outData;

        ams_deviceGetFlicker(chip->deviceCtx, &outData);
        temp = outData.mHz;
#else
        ams_deviceCtx_t * ctx = (ams_deviceCtx_t *)chip->deviceCtx;

        ams_deviceGetSWFlicker(chip->deviceCtx);
        temp = ctx->flickerCtx.lastValid.mHz;
        printk(KERN_INFO"%s temp:%d \n",__func__, temp);
#endif
        temp = temp << 8;
        flicker = temp | i++;

        if (i > 5)
            i = 0;

        printk(KERN_INFO"%s temp:%d flicker:%d \n",__func__, temp, flicker);
		input_report_abs(chip->flicker_idev, ABS_DISTANCE, flicker);
		input_sync(chip->flicker_idev);
	}
}



void osal_report_sw_flicker(struct amsDriver_chip *chip)
{
	ams_apiAlsFlicker_t outData = {0,};
    static unsigned int i = 0;   //shmoon_190612
    int j =0;
    uint32_t temp[4] = {0,};
    uint32_t flicker, satur, clr, wb = 0;

	if (chip->flicker_idev) {
#if 0

        ams_deviceGetFlicker(chip->deviceCtx, &outData);
        temp = outData.mHz;
#else
        //ams_deviceCtx_t * ctx = (ams_deviceCtx_t *)chip->deviceCtx;

        ams_deviceGetSWFlicker(chip->deviceCtx,&outData);
        temp[0] = outData.mHzbysw;
        temp[1] = outData.saturation; /*1:general low IR light source , 2:high IR light source, 3:under sun light  */
        temp[2] = outData.clear;
        temp[3] = outData.wideband;

        for (j=0;j<4;j++)
            temp[j] = temp[j] << 8;

        //printk(KERN_INFO"%s temp:%d \n",__func__, temp);
#endif

        flicker = temp[0] | i;
        satur = temp[1] | i;
        clr = temp[2] | i;
        wb = temp[3] | i;

        AMS_PORT_msg_4("outData (mHzbysw:%u, Sun light :%u, clear:%u, wideband:%u)\n",
            outData.mHzbysw, outData.saturation, outData.clear, outData.wideband);
        /*        
        printk(KERN_INFO"%s temp (flicker:%u, satur:%u, clr:%u, wb:%u)\n",
            __func__, flicker, satur, clr, wb);
        */
		//input_report_abs(chip->flicker_idev, ABS_DISTANCE, flicker);
		input_report_rel(chip->flicker_idev, REL_X, satur);
		input_report_rel(chip->flicker_idev, REL_Y, clr);
		input_report_rel(chip->flicker_idev, REL_Z, wb);
        input_report_rel(chip->flicker_idev, REL_RX, flicker);
		input_sync(chip->flicker_idev);

        i++;
        if (i > 5)
            i = 0;
	}
}


static ssize_t osal_device_als_lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ams_apiAls_t outData;
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	ams_deviceGetAls(chip->deviceCtx, &outData);

	return snprintf(buf, PAGE_SIZE, "%d\n", outData.mLux);
}


static ssize_t osal_als_red_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ams_apiAls_t outData;
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	ams_deviceGetAls(chip->deviceCtx, &outData);
	return snprintf(buf, PAGE_SIZE, "%d\n", outData.rawRed);
}

static ssize_t osal_als_green_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ams_apiAls_t outData;
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	ams_deviceGetAls(chip->deviceCtx, &outData);
	return snprintf(buf, PAGE_SIZE, "%d\n", outData.rawGreen);
}


static ssize_t osal_als_blue_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ams_apiAls_t outData;
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	ams_deviceGetAls(chip->deviceCtx, &outData);
	return snprintf(buf, PAGE_SIZE, "%d\n", outData.rawBlue);
}

static ssize_t osal_als_clear_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ams_apiAls_t outData;
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	ams_deviceGetAls(chip->deviceCtx, &outData);
	return snprintf(buf, PAGE_SIZE, "%d\n", outData.rawClear);
}

static ssize_t osal_flicker_raw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ams_apiAlsFlicker_t outData;
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	ams_deviceGetFlicker(chip->deviceCtx, &outData);
	return snprintf(buf, PAGE_SIZE, "%d\n", outData.flicker_raw_data);
}


static ssize_t osal_als_reg_show(struct device*dev,
	struct device_attribute *attr, char*buf)
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	printk(KERN_INFO"%s debug mode = %u \n",__func__,chip->debug_mode);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->debug_mode);

}


static ssize_t osal_smux_show(struct device*dev,
	struct device_attribute *attr, char*buf)
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);

       ams_smux_read(chip->deviceCtx);
	return snprintf(buf, PAGE_SIZE, "%d\n", 1);

}

static ssize_t osal_smux_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
	
	printk(KERN_INFO"%s osal_smux_store   \n",__func__);

	if (value){
          ams_smux_set(chip->deviceCtx , AMS_TCS3408);
	}
       else
          ams_smux_set(chip->deviceCtx , AMS_TCS3407);
	return size;

}


#if 0
static ssize_t osal_sw_flicker_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
	
	printk(KERN_INFO"%s sw_flicke mode  \n",__func__);

	if (value){
	      ams_deviceGetSWFlicker(chip->deviceCtx);
	}
       else
	   	;
	return size;

}
#endif
static ssize_t osal_als_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	int err;
	s32 mode;

	//mutex_lock(&chip->lock);
	err = kstrtoint(buf,10,&mode);
	if(err<0){
		printk(KERN_ERR"%s- kstrtoint failed%d\n",__func__,err);
		return err;
		}
	chip->debug_mode = (u8)mode;
	printk(KERN_ERR"%s - mode = %d\n",__func__,err);

	if(chip->debug_mode == 1){
	      ams_deviceGetRegdump(chip->deviceCtx);
	}
	//mutex_unlock(&chip->lock);

	return size;

}



	
ssize_t osal_als_enable_set(struct amsDriver_chip *chip, uint8_t valueToSet)
{
	ssize_t rc = 0;
	rc |= ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_ALS_LUX, AMS_CONFIG_ENABLE, valueToSet);
	return 0;
}



ssize_t osal_hw_flicker_enable_set(struct amsDriver_chip *chip, uint8_t valueToSet)
{
	ssize_t rc = 0;
	rc |= ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_HW_FLICKER, AMS_CONFIG_ENABLE, valueToSet);
	return 0;
}


ssize_t osal_sw_flicker_enable_set(struct amsDriver_chip *chip, uint8_t valueToSet)
{
	ssize_t rc = 0;
	rc |= ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_SW_FLICKER, AMS_CONFIG_ENABLE, valueToSet);
	return 0;
}

static ssize_t osal_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	ams_mode_t mode;

	ams_getMode(chip->deviceCtx, &mode);

	if (mode & MODE_ALS_ALL){
		return snprintf(buf, PAGE_SIZE, "%d\n", 1);
	} else {
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}
}

static ssize_t osal_als_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
    // SHMOON : In MH2, als_power_state node is used for making input event for terminating flicker thread.
    struct amsDriver_chip *chip = dev_get_drvdata(dev);
    bool value;

    if (strtobool(buf, &value))
    {
        dev_info(dev, "invalue param!\n");
        return -EINVAL;
    }
#if 0  
    if (value) 
        osal_als_enable_set(chip, AMSDRIVER_ALS_ENABLE);
    else 
        osal_als_enable_set(chip, AMSDRIVER_ALS_DISABLE);
#else
    if (value)
    {
        dev_info(dev, "osal_als_enable_store called!\n");
        input_report_rel(chip->flicker_idev, REL_RY, 1001);
        input_sync(chip->flicker_idev);
    }
#endif

	return size;
}

static ssize_t osal_flicker_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	ams_mode_t mode;

	ams_getMode(chip->deviceCtx, &mode);

	if (mode & MODE_FLICKER){
		return snprintf(buf, PAGE_SIZE, "%d\n", 1);
	} else {
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}
}




static ssize_t osal_flicker_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
#if 1
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
#if 0  // shmoon_190604
	if (value) 
		osal_hw_flicker_enable_set(chip, AMSDRIVER_ALS_ENABLE);
	else 
		osal_hw_flicker_enable_set(chip, AMSDRIVER_ALS_DISABLE);
#else
	if (value)
	{
        int err = osal_flicker_idev_open(chip->flicker_idev);
        dev_info(&chip->client->dev, "osal_flicker_idev_open err:%d\n",err);
	}
    else
        osal_flicker_idev_close(chip->flicker_idev);
#endif

	return size;
}
#else
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	//bool value;
       uint8_t enable;
	ssize_t ret;

	ret = kstrtol(buf,0,(long*)(&(enable)));
	
	
	//if (strtobool(buf, &value))
	if (ret !=0)	
		return -EINVAL;

      dev_err(&chip->client->dev, "enable %d\n",enable);
	  
	if (enable==0){
		ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_HW_FLICKER, AMS_CONFIG_ENABLE, 0);
	}

	else if (enable==1){
		ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_HW_FLICKER, AMS_CONFIG_ENABLE, 1);
	}

	return size;
}
#endif

#if 0
static ssize_t osal_flicker_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	ams_apiAlsFlicker_t outData;
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	ams_deviceGetSWFlicker(chip->deviceCtx);
       //ccb_sw_flicker_GetResult(chip->deviceCtx, uint16_t *data)
	return snprintf(buf, PAGE_SIZE, "%d\n", outData.mHz);


}
#endif

static ssize_t osal_sw_flicker_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	ams_mode_t mode;

	ams_getMode(chip->deviceCtx, &mode);

	if (mode & MODE_FLICKER){
		return snprintf(buf, PAGE_SIZE, "%d\n", 1);
	} else {
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}
}



static ssize_t osal_sw_flicker_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
#if 1
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value) {
		chip->sensor_enable = true;
 		osal_sw_flicker_enable_set(chip, AMSDRIVER_ALS_ENABLE);
		hrtimer_start(&chip->timer, chip->light_poll_delay, HRTIMER_MODE_REL);/*polling start*/
	} else {
		chip->sensor_enable = false;
		hrtimer_cancel(&chip->timer);/*polling stop*/
		osal_sw_flicker_enable_set(chip, AMSDRIVER_ALS_DISABLE);
	}
    
	return size;
}
#else
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	//bool value;
       uint8_t enable;
	ssize_t ret;

	ret = kstrtol(buf,0,(long*)(&(enable)));
	
	
	//if (strtobool(buf, &value))
	if (ret !=0)	
		return -EINVAL;

      dev_err(&chip->client->dev, "enable %d\n",enable);
	  
	if (enable==0){
		ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_SW_FLICKER, AMS_CONFIG_ENABLE, 0);
	}

	else if (enable==1){
		ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_SW_FLICKER, AMS_CONFIG_ENABLE, 1);
	}
 
	return size;
}
#endif
/*
static ssize_t  osal_polling_timer_enable_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	bool value;
					  
			 

	if (strtobool(buf, &value))
 
 
							  
			  
		return -EINVAL;

	if (value)
	{
		printk("starting poll timer %lld ns\n", ktime_to_ns(chip->light_poll_delay));
		hrtimer_start(&chip->timer, chip->light_poll_delay, HRTIMER_MODE_REL);
	}else{
		hrtimer_cancel(&chip->timer);
		cancel_work_sync(&chip->work_light1);
																					
	}
 
	return size;
}
*/

struct device_attribute osal_als_attrs[] = {
	__ATTR(als_lux,           0444, osal_device_als_lux_show,       NULL),
	__ATTR(als_red,           0444, osal_als_red_show,              NULL),
	__ATTR(als_green,         0444, osal_als_green_show,            NULL),
	__ATTR(als_blue,          0444, osal_als_blue_show,             NULL),
	__ATTR(als_clear,         0444, osal_als_clear_show,            NULL),
	__ATTR(flicker_raw_data,         0444, osal_flicker_raw_show,            NULL),
	__ATTR(als_power_state,   0664, osal_als_enable_show,           osal_als_enable_store),
	__ATTR(flicker_power_state,   0664, osal_flicker_enable_show,   osal_flicker_enable_store),
	__ATTR(flicker_sw_power_state,   0664, osal_sw_flicker_enable_show,   osal_sw_flicker_enable_store),
	//__ATTR(polling_enable,         0660, NULL,            osal_polling_timer_enable_store),	
	__ATTR(als_debug,         0664, osal_als_reg_show,            osal_als_reg_store),
	__ATTR(smux_setting,         0664, osal_smux_show,            osal_smux_store),

};

int osal_als_attrs_size = ARRAY_SIZE(osal_als_attrs);

#endif
