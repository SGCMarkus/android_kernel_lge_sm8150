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

#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
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
#include "osal_linux_input_als.h"

#define AMS_IRQ 96

/****************************************************************************
 *                     OSAL Device Com Interface
 ****************************************************************************/

uint8_t AMS_PORT_getByte(AMS_PORT_portHndl * handle, uint8_t reg, uint8_t * data, uint8_t len){
    int ret;
    if (handle == NULL)
    {
	    printk(KERN_ERR "\nAMS_Driver: %s: handle is NULL\n", __func__);
        return 0;
    }

    ret = i2c_smbus_read_i2c_block_data(handle, reg, len, data);
    if (ret < 0) 
	    dev_err(&handle->dev, "%s: failed at address %x (%d bytes)\n",
		    __func__, reg, len);

    return ret;
}

uint8_t AMS_PORT_setByte(AMS_PORT_portHndl * handle, uint8_t reg, uint8_t * data, uint8_t len){
    int ret;
    if (handle == NULL)
    {
	    printk(KERN_ERR "\nAMS_Driver: %s: handle is NULL\n", __func__);
        return 0;
    }

    ret = i2c_smbus_write_i2c_block_data(handle, reg, len, data);
    if (ret < 0)
	    dev_err(&handle->dev, "%s: failed at address %x (%d bytes)\n",
		    __func__, reg, len);

    return ret;
}

//extern int ams_smux_set(ams_deviceCtx_t *ctx);
/****************************************************************************
 *                     OSAL Linux Input Driver
 ****************************************************************************/

static int amsdriver_pltf_power_on(struct amsDriver_chip *chip)
{
	int rc = 0;
	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev,
			POWER_ON);
		mdelay(10);
	}
	chip->unpowered = rc != 0;
	dev_err(&chip->client->dev, "\n\n%s: unpowered=%d\n",__func__, chip->unpowered);
	return rc;
}

static int amsdriver_pltf_power_off(struct amsDriver_chip *chip)
{
	int rc = 0;
	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev,
			POWER_OFF);
		chip->unpowered = rc == 0;
	} else {
		chip->unpowered = false;
	}
	dev_err(&chip->client->dev, "\n\n%s: unpowered=%d\n",__func__, chip->unpowered);
	return rc;
}

static int amsdriver_power_on(struct amsDriver_chip *chip)
{
	int rc;
	rc = amsdriver_pltf_power_on(chip);
	if (rc){
		return rc;
	}
	dev_info(&chip->client->dev, "%s: chip was off, restoring regs\n",
			__func__);
	return ams_deviceInit(chip->deviceCtx, chip->client, NULL);
}

static int amsdriver_add_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	dev_err(dev, "%s: failed\n", __func__);
	return -ENODEV;
}

static void amsdriver_remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}

static irqreturn_t amsdriver_irq(int irq, void *handle)
{
	struct amsDriver_chip *chip = handle;
	struct device *dev = &chip->client->dev;
	int ret;
	int interruptsHandled = 0;

	//mutex_lock(&chip->lock);
	if ( (chip->in_suspend) || (chip->sensor_enable == false) ) {
		dev_info(dev, "%s: in suspend\n", __func__);
		chip->irq_pending = 1;
		ret = 0;
		goto bypass;
	}
	ret = ams_deviceEventHandler(chip->deviceCtx);
	interruptsHandled = ams_getResult(chip->deviceCtx);

#if 0
	if (interruptsHandled & (1 << AMS_AMBIENT_SENSOR))
		osal_report_als(chip);
#endif

	if (interruptsHandled & (1 << AMS_FLICKER_SENSOR))
		osal_report_flicker(chip);

    if (interruptsHandled & (1 << AMS_SW_FLICKER_SENSOR))
        osal_report_sw_flicker(chip);
 
//return ret;
return IRQ_HANDLED;
bypass:
	//mutex_unlock(&chip->lock);
	return ret ? IRQ_HANDLED : IRQ_NONE;
}

//static int osal_flicker_idev_open(struct input_dev *idev)     //shmoon_190604
int osal_flicker_idev_open(struct input_dev *idev)
{
#if 1
	struct amsDriver_chip *chip = dev_get_drvdata(&idev->dev);
	int rc = 0;

	dev_info(&idev->dev, "%s\n", __func__);
	AMS_MUTEX_LOCK(&chip->lock);
	if (chip->unpowered) {
		rc = amsdriver_power_on(chip);
		if (rc)
			goto chip_on_err;
	}

        //ams_smux_set(chip->deviceCtx);
	rc = osal_flicker_enable_set(chip, AMSDRIVER_ALS_ENABLE);
	//rc = osal_als_enable_set(chip, AMSDRIVER_ALS_ENABLE);
		
	if (rc)
		amsdriver_pltf_power_off(chip);
    else
        chip->sensor_enable = true;
chip_on_err:
	AMS_MUTEX_UNLOCK(&chip->lock);
#endif	
	return 0;
}

//static void osal_flicker_idev_close(struct input_dev *idev)   //shmoon_190604
void osal_flicker_idev_close(struct input_dev *idev)
{
	int rc = 0;
	struct amsDriver_chip *chip = dev_get_drvdata(&idev->dev);
	dev_info(&idev->dev, "%s\n", __func__);

	AMS_MUTEX_LOCK(&chip->lock);
	rc = osal_flicker_enable_set(chip, AMSDRIVER_ALS_DISABLE);
//	rc = osal_als_enable_set(chip, AMSDRIVER_ALS_DISABLE);
	
	if (rc)
		amsdriver_pltf_power_off(chip);
    else
        chip->sensor_enable = false;
	AMS_MUTEX_UNLOCK(&chip->lock);
}

static enum hrtimer_restart tcs3407_timer_func(struct hrtimer *timer)
{
	struct amsDriver_chip *chip = container_of(timer, struct amsDriver_chip, timer);
	queue_work(chip->wq, &chip->work_light1);
	hrtimer_forward_now(&chip->timer, chip->light_poll_delay);
	return HRTIMER_RESTART;
}


void tcs3407_work_func_light(struct work_struct *work)
{
#if defined ( AMS_BIN_2048_MODE1) ||defined ( AMS_BIN_2048_MODE2)

	int interruptsHandled = 0;
	//ams_apiAlsFlicker_t outData;

  	struct amsDriver_chip *chip
		= container_of(work, struct amsDriver_chip, work_light1);

	//printk("#> tmg49xx_work_func_light \n");

       //AMS_MUTEX_LOCK(&chip->lock);
       if (chip->flicker_idev) {
   	       ams_devicePollingHandler(chip->deviceCtx);
       }

      interruptsHandled = ams_getResult(chip->deviceCtx);
      //AMS_PORT_log_1( "amsdriver_irq !! interruptsHandled %d",interruptsHandled);

	if (interruptsHandled & (1 << AMS_FLICKER_SENSOR))
		osal_report_flicker(chip);

	if (interruptsHandled & (1 << AMS_SW_FLICKER_SENSOR))
		osal_report_sw_flicker(chip);

		
    //AMS_MUTEX_UNLOCK(&chip->lock);
#endif
}
#if 0
static int tcs3407_parse_dt(struct device *dev)
{
	struct amsDriver_chip  *chip = dev_get_drvdata(dev);
	int rc = 0;

	if (of_property_read_u32(dev->of_node, "ams,gpio_irq", &(chip->irq)))
		dev_warn(dev, "irq not specified in dt\n");

	dev_info(dev, "%s: chip->irq = %d\n", __func__, chip->irq);
	chip->irq = AMS_IRQ;
	dev_info(dev, "%s: new chip->irq = %d\n", __func__, chip->irq);

	if (gpio_is_valid(chip->irq)) {
		rc = gpio_request(chip->irq, "ams_irq");
		if (rc)
			dev_err(dev, "%s: unable to request gpio [%d]\n", __func__, chip->irq);
		rc = gpio_direction_input(chip->irq);
		if (rc)
			dev_err(dev, "%s: unable to set direction for gpio [%d]\n", __func__, chip->irq);
		chip->client->irq = gpio_to_irq(chip->irq);
        dev_info(dev, "%s: chip->client->irq = %d\n", __func__, chip->client->irq);
	} else {
		dev_err(dev, "%s: irq gpio not provided\n", __func__);
	}

	return rc;
}

static irqreturn_t handle_twl4030_pih(int irq, void *devid)
{
	return IRQ_HANDLED;
}
#endif

int amsdriver_probe(struct i2c_client *client,
	const struct i2c_device_id *idp)
{
	int ret=0;
	struct device *dev = &client->dev;
	static struct amsDriver_chip *chip;
	struct amsdriver_i2c_platform_data *pdata = dev->platform_data;
	ams_deviceInfo_t amsDeviceInfo = {0,};
	ams_deviceIdentifier_e deviceId;

	printk(KERN_ERR "\nAMS_Driver: probe()\n");

	/****************************************/
	/* Validate bus and device registration */
	/****************************************/

	dev_info(dev, "%s: client->irq = %d\n", __func__, client->irq);
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}

	chip = kzalloc(sizeof(struct amsDriver_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}

	mutex_init(&chip->lock);
	chip->client = client;
	chip->pdata = pdata;
	i2c_set_clientdata(chip->client, chip);

	/****************************************/
	/* parsing dt */
	/****************************************/
	//tcs3407_parse_dt(dev);
	dev_info(dev, "%s: client->irq = %d\n", __func__, client->irq);
	dev_info(dev, "%s: client->addr = %d\n", __func__, client->addr);
	dev_info(dev, "%s: client->name = %s\n", __func__, client->name);

	/********************************************************************/
	/* Validate the appropriate ams device is available for this driver */
	/********************************************************************/
    deviceId = ams_validateDevice(chip->client);
	dev_info(dev, "deviceId: %d\n", deviceId);

	if (deviceId == AMS_UNKNOWN_DEVICE) {
		dev_info(dev, "ams_validateDevice failed: AMS_UNKNOWN_DEVICE\n");
		goto id_failed;
	}

        dev_info(dev, "ams_validateDevice() ok\n");

        ams_getDeviceInfo(&amsDeviceInfo);
        dev_info(dev, "ams_amsDeviceInfo() ok\n");
        dev_info(dev, "%s: name: %s, model: %s, driver ver:%s\n", 
		 __func__, 
		 amsDeviceInfo.deviceName,
		 amsDeviceInfo.deviceModel, 
		 amsDeviceInfo.driverVersion);

        chip->deviceCtx = kzalloc(amsDeviceInfo.memorySize, GFP_KERNEL);
        if(!chip->deviceCtx)
        {
            //SHMOON_190816
            dev_err(dev, "%s: Failed to alloc hip->deviceCtx!\n", __func__);
            ret = -ENOMEM;
            goto malloc_failed;
        }

        ret = ams_deviceInit(chip->deviceCtx, chip->client, NULL);
        if (ret == false){
		dev_info(dev, "ams_amsDeviceInit() ok\n");
        } else {
		dev_info(dev, "ams_deviceInit failed.\n");
        goto id_failed;
        }
        
        chip->sensor_enable = false;


	/*********************/
	/* Initialize ALS    */
	/*********************/
#if 0
//#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS
	/* setup */
	dev_info(dev, "Setup for ALS\n");
	chip->als_idev = input_allocate_device();
	if(!chip->als_idev){
		dev_err(dev, "%s: no memory for input_dev ALS\n", __func__);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->als_idev->name = "ALS";
	chip->als_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->als_idev->evbit);
	set_bit(ABS_MISC, chip->als_idev->absbit);
	input_set_abs_params(chip->als_idev, ABS_MISC, 0, 65535, 0, 0);
	//chip->als_idev->open = osal_als_idev_open;
	//chip->als_idev->close = osal_als_idev_close;
	dev_set_drvdata(&chip->als_idev->dev, chip);
	ret = input_register_device(chip->als_idev);
	if (ret){
		input_free_device(chip->als_idev);
		dev_err(dev, "%s: cant register input 'als'\n",
				__func__);
		goto input_a_alloc_failed;
	}
/*  shmoon_190604
	ret = amsdriver_add_sysfs_interfaces(&chip->als_idev->dev,
			osal_als_attrs, osal_als_attrs_size);
*/
	if (ret){
		goto input_a_sysfs_failed;
	}
#endif

	/*********************/
	/* Initialize Flicker   */
	/*********************/
	/* setup */
	dev_info(dev, "Setup for Flicker\n");
	chip->flicker_idev = input_allocate_device();
	if(!chip->flicker_idev){
		dev_err(dev, "%s: no memory for input_dev Flicker\n", __func__);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->flicker_idev->name = "FLICKER";
	chip->flicker_idev->id.bustype = BUS_I2C;
/*
	set_bit(EV_ABS, chip->flicker_idev->evbit);
	set_bit(ABS_DISTANCE, chip->flicker_idev->absbit);
	input_set_abs_params(chip->flicker_idev, ABS_DISTANCE, 0, 65535, 0, 0);
*/
	input_set_capability(chip->flicker_idev, EV_REL, REL_X); /*Saturation bit*/
	input_set_capability(chip->flicker_idev, EV_REL, REL_Y); /*Clear*/
	input_set_capability(chip->flicker_idev, EV_REL, REL_Z); /*Wide*/
	input_set_capability(chip->flicker_idev, EV_REL, REL_RX); /*Flicker*/
    input_set_capability(chip->flicker_idev, EV_REL, REL_RY); /*Signal for terminating flicker thread*/

/*  shmoon_190604
	chip->flicker_idev->open = osal_flicker_idev_open;
	chip->flicker_idev->close = osal_flicker_idev_close;
*/
	dev_set_drvdata(&chip->flicker_idev->dev, chip);
	ret = input_register_device(chip->flicker_idev);
	if (ret){
		input_free_device(chip->flicker_idev);
		dev_err(dev, "%s: cant register input 'als'\n",
				__func__);
		goto input_a_alloc_failed;
	}

	ret = amsdriver_add_sysfs_interfaces(&chip->flicker_idev->dev,
			osal_als_attrs, osal_als_attrs_size);

	if (ret){
		goto input_a_sysfs_failed;
	}

	INIT_WORK(&chip->work_light1, tcs3407_work_func_light);

	/* hrtimer settings.  we poll for light values using a timer. */
	hrtimer_init(&chip->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
//    	chip->light_poll_delay = ns_to_ktime(3.6 * NSEC_PER_MSEC);  // 60msec Polling time

//	chip->light_poll_delay = ns_to_ktime(20 * NSEC_PER_MSEC);  // 20msec Polling time
//	chip->light_poll_delay = ns_to_ktime(60 * NSEC_PER_MSEC);  // 60msec Polling time
//	chip->light_poll_delay = ns_to_ktime(128 * NSEC_PER_MSEC);  // 100msec Polling time
	chip->light_poll_delay = ns_to_ktime(50 * NSEC_PER_MSEC);  // 50msec Polling time
//	chip->light_poll_delay = ns_to_ktime(10 * NSEC_PER_MSEC);  // 60msec Polling time
	chip->timer.function = tcs3407_timer_func;

	chip->wq = create_singlethread_workqueue("tcs3407_wq");
	if (!chip->wq) {
		ret = -ENOMEM;
		pr_err("%s: could not create workqueue\n", __func__);
		goto init_failed;
	}

	/****************************/
	/* Initialize IRQ & Handler */
	/****************************/
	ret = request_threaded_irq(client->irq, NULL, &amsdriver_irq,
				IRQF_TRIGGER_FALLING | /*IRQF_SHARED |*/IRQF_ONESHOT,
				"ams_tcs3407", chip);

	if (ret) {
		dev_info(dev, "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}


	dev_info(dev, "Probe ok.\n");
	return 0;

	/********************************************************************************/
	/* Exit points for device functional failures (RemCon, Prox, ALS, Gesture)      */
	/* This must be unwound in the correct order, reverse from initialization above */
	/********************************************************************************/

irq_register_fail:

#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS
	if (chip->flicker_idev)
	{
		amsdriver_remove_sysfs_interfaces(&chip->flicker_idev->dev,
						osal_als_attrs, 
						osal_als_attrs_size);
input_a_sysfs_failed:
		input_unregister_device(chip->flicker_idev);
	}
input_a_alloc_failed:
#endif

	/********************************************************************************/
	/* Exit points for general device initialization failures                       */
	/********************************************************************************/

id_failed:
	if (chip->deviceCtx) kfree(chip->deviceCtx);
	i2c_set_clientdata(client, NULL);
malloc_failed:
	kfree(chip);

init_failed:
	dev_err(dev, "Probe failed.\n");
	return ret;
}

int amsdriver_suspend(struct device *dev)
{
	struct amsDriver_chip  *chip = dev_get_drvdata(dev);

	dev_info(dev, "%s chip->sensor_enable:%d\n", __func__, chip->sensor_enable);

    if(chip->sensor_enable == false)
        return 0;

	AMS_MUTEX_LOCK(&chip->lock);
	chip->in_suspend = 1;

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 1);
	} else if (!chip->unpowered) {
		dev_info(dev, "powering off\n");
		/* TODO
		   platform power off */
	}

    osal_sw_flicker_enable_set(chip, AMSDRIVER_ALS_DISABLE);/*POWER OFF*/
    printk(KERN_ERR "\nAMS_Driver: suspend() done\n");
    chip->sensor_enable == false;

	AMS_MUTEX_UNLOCK(&chip->lock);

	return 0;
}

int amsdriver_resume(struct device *dev)
{
	struct amsDriver_chip *chip = dev_get_drvdata(dev);

	//return 0;
	printk(KERN_ERR "\nAMS_Driver: resume()\n");
    if(chip->sensor_enable == false)
        return 0;
    
	AMS_MUTEX_LOCK(&chip->lock);

	chip->in_suspend = 0;

	//amsdriver_power_on(chip); /*Device init & reset */
	osal_sw_flicker_enable_set(chip, AMSDRIVER_ALS_ENABLE); /*POWER ON*/

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
		chip->wake_irq = 0;
	}

/* err_power: */
	AMS_MUTEX_UNLOCK(&chip->lock);
    printk(KERN_ERR "\nAMS_Driver: resume() done\n");

	return 0;
}

int amsdriver_remove(struct i2c_client *client)
{
	struct amsDriver_chip *chip = i2c_get_clientdata(client);

	printk(KERN_ERR "\nAMS_Driver: REMOVE()\n");
	free_irq(client->irq, chip);

	/* TODO
	   platform teardown */

	i2c_set_clientdata(client, NULL);
	kfree(chip->deviceCtx);
	kfree(chip);
	return 0;
}

