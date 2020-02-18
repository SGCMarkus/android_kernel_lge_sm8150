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

 ssize_t osal_flicker_enable_set(struct amsDriver_chip *chip, uint8_t valueToSet)
{
	ssize_t rc = 0;

	rc |= ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_FLICKER, AMS_CONFIG_ENABLE, valueToSet);
	return 0;
}


#if defined(CONFIG_AMS_OPTICAL_SENSOR_ALS)

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

void osal_report_flicker(struct amsDriver_chip *chip)
{
	ams_apiAlsFlicker_t outData;
	int flicker;

	if (chip->flicker_idev) {
		ams_deviceGetFlicker
			(chip->deviceCtx, &outData);
		
		flicker = outData.mHz;
		
		input_report_abs(chip->flicker_idev, ABS_DISTANCE, flicker);
		input_sync(chip->flicker_idev);
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



static ssize_t osal_sw_flicker_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
#ifdef AMS_SW_FLICKER	
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
#endif
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
	
	printk(KERN_INFO"%s sw_flicke mode  \n",__func__);

	if (value){
		#ifdef AMS_SW_FLICKER	
	      ams_deviceGetSWFlicker(chip->deviceCtx);
	      //ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_FLICKER, AMS_CONFIG_ENABLE, 0);
	      #endif
	}
       else
	   	;
	return size;

}

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
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value) 
		osal_als_enable_set(chip, AMSDRIVER_ALS_ENABLE);
	else 
		osal_als_enable_set(chip, AMSDRIVER_ALS_DISABLE);

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
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_FLICKER, AMS_CONFIG_ENABLE, 1);
	else
		ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_FLICKER, AMS_CONFIG_ENABLE, 0);

	return size;
}

static ssize_t osal_flicker_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{

#ifdef AMS_SW_FLICKER    
	ams_apiAlsFlicker_t outData;
	struct amsDriver_chip *chip = dev_get_drvdata(dev);
	ams_deviceGetSWFlicker(chip->deviceCtx);
       //ccb_sw_flicker_GetResult(chip->deviceCtx, uint16_t *data)
	return snprintf(buf, PAGE_SIZE, "%d\n", outData.mHz);
#else
   
    return snprintf(buf, PAGE_SIZE, "%d\n", 1);       
#endif


}


struct device_attribute osal_als_attrs[] = {
	__ATTR(als_lux,           0440, osal_device_als_lux_show,       NULL),
	__ATTR(als_red,           0440, osal_als_red_show,              NULL),
	__ATTR(als_green,         0440, osal_als_green_show,            NULL),
	__ATTR(als_blue,          0440, osal_als_blue_show,             NULL),
	__ATTR(als_clear,         0440, osal_als_clear_show,            NULL),
	__ATTR(flicker_raw_data,         0440, osal_flicker_raw_show,            NULL),
	__ATTR(als_power_state,   0660, osal_als_enable_show,           osal_als_enable_store),
	__ATTR(flicker_power_state,   0660, osal_flicker_enable_show,   osal_flicker_enable_store),
	__ATTR(flicker_sw_debug,         0660, osal_flicker_data_show,            osal_sw_flicker_store),	
	__ATTR(als_debug,         0660, osal_als_reg_show,            osal_als_reg_store),
	__ATTR(smux_setting,         0660, osal_smux_show,            osal_smux_store),

};

int osal_als_attrs_size = ARRAY_SIZE(osal_als_attrs);

#endif
