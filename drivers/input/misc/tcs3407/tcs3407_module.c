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
 * Input Driver Module
 */

/*
 * @@AMS_REVISION_Id: e5a8b52500c3ff445414b03d51f864a184cc76a0
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/device.h>

#include "ams_port_platform.h"

static struct i2c_device_id amsdriver_idtable[] = {
	{"ams_tcs3407", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, amsdriver_idtable);

static const struct dev_pm_ops amsdriver_pm_ops = {
	.suspend = amsdriver_suspend,
	.resume  = amsdriver_resume,
};

static struct i2c_driver amsdriver_driver = {
	.driver = {
		.name = "ams_tcs3407", 
		.pm = &amsdriver_pm_ops,
	},
	.id_table = amsdriver_idtable,
	.probe = amsdriver_probe,
	.remove = amsdriver_remove,
};

static int __init amsdriver_init(void)
{
	int rc;
	printk(KERN_ERR "\nams_tcs3407: init()\n");

	rc = i2c_add_driver(&amsdriver_driver);

	printk(KERN_ERR "ams_tcs3407:  %d", rc);
	return rc;
}

static void __exit amsdriver_exit(void)
{
	printk(KERN_ERR "\nams_tcs3407: exit()\n");
	i2c_del_driver(&amsdriver_driver);
}

module_init(amsdriver_init);
module_exit(amsdriver_exit);

MODULE_AUTHOR("AMS AOS Software<cs.americas@ams.com>");
MODULE_DESCRIPTION("AMS tcs3407 ALS,Flicker sensor driver");
MODULE_LICENSE("GPL");
