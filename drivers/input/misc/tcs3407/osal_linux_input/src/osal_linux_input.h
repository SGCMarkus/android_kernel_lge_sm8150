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

#include <linux/input.h>    //shmoon_190604

/*
 * AMS generic device driver
 */

/*
 * @@AMS_REVISION_Id: 969322ab678073812e97aab66a03df1d04e97836
 */
#ifndef __OSAL_LINUX_INPUT_H
#define __OSAL_LINUX_INPUT_H

struct amsDriver_chip {
	struct mutex lock;
	struct i2c_client *client;
	struct amsdriver_i2c_platform_data *pdata;
	int in_suspend;
	int wake_irq;
	int irq_pending;
	int irq;
	bool unpowered;
	u8 device_index;
	void * deviceCtx;
       u8 debug_mode;	
       u8 sw_flicker_mode;	
	   
	//struct input_dev *als_idev;	
	struct input_dev *flicker_idev;

       struct workqueue_struct *wq;
	struct work_struct work_light1;
       struct hrtimer timer;
	ktime_t light_poll_delay;
    bool sensor_enable;

};

#define AMSDRIVER_ALS_ENABLE 1
#define AMSDRIVER_ALS_DISABLE 0
#define AMSDRIVER_FLICKER_ENABLE 1
#define AMSDRIVER_FLICKER_DISABLE 0

// shmoon_190604
int osal_flicker_idev_open(struct input_dev *idev);
void osal_flicker_idev_close(struct input_dev *idev);


#endif /*__OSAL_LINUX_INPUT_H */
