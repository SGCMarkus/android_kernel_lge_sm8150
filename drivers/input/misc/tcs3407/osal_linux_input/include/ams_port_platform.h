#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#include <linux/i2c.h>
#include <linux/delay.h>
#endif

#ifndef AMS_PORT_PLATFORM_H
#define	AMS_PORT_PLATFORM_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "ams_platform.h"

#define CONFIG_AMS_LITTLE_ENDIAN 1
#ifdef CONFIG_AMS_LITTLE_ENDIAN
#define AMS_ENDIAN_1    0
#define AMS_ENDIAN_2    8
#else
#define AMS_ENDIAN_2    0
#define AMS_ENDIAN_1    8
#endif

#define AMS_PORT_portHndl   struct i2c_client

#include "ams_device_control_block.h"
#include "ams_device_com_interface.h"

#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS
#include "../../algorithm_als/include/ams_als_API.h"
#endif


#ifdef AMS_MUTEX_DEBUG
#define AMS_MUTEX_LOCK(m) {						\
		printk(KERN_INFO "%s: Mutex Lock\n", __func__);         \
		mutex_lock(m);						\
	}
#define AMS_MUTEX_UNLOCK(m) {                                           \
		printk(KERN_INFO "%s: Mutex Unlock\n", __func__);       \
		mutex_unlock(m);					\
	}
#else
#define AMS_MUTEX_LOCK(m) {                           \
		mutex_lock(m);			      \
	}
#define AMS_MUTEX_UNLOCK(m) {                         \
		mutex_unlock(m);		      \
	}
#endif

#ifdef CONFIG_AMS_DEBUG_LOG
#define AMS_PORT_log(x)                 printk(KERN_ERR "AMS_Driver: " x)
#define AMS_PORT_log_1(x, a)            printk(KERN_ERR "AMS_Driver: " x, a)
#define AMS_PORT_log_2(x, a, b)         printk(KERN_ERR "AMS_Driver: " x, a, b)
#define AMS_PORT_log_3(x, a, b, c)      printk(KERN_ERR "AMS_Driver: " x, a, b, c)
#define AMS_PORT_log_4(x, a, b, c, d)   printk(KERN_ERR "AMS_Driver: " x, a, b, c, d)
#define AMS_PORT_log_5(x, a, b, c, d,e)   printk(KERN_ERR "AMS_Driver: " x, a, b, c, d,e)
#else
#define AMS_PORT_log(x)                 do{ } while (false)
#define AMS_PORT_log_1(x, a)            do{ } while (false)
#define AMS_PORT_log_2(x, a, b)         do{ } while (false)
#define AMS_PORT_log_3(x, a, b, c)      do{ } while (false)
#define AMS_PORT_log_4(x, a, b, c, d)   do{ } while (false)
#define AMS_PORT_log_5(x, a, b, c, d,e)   do{ } while (false)
#endif

#ifdef CONFIG_AMS_CORE_LOG
#define AMS_PORT_msg(x)                 printk(KERN_INFO "AMS_Driver: " x)
#define AMS_PORT_msg_1(x, a)            printk(KERN_INFO "AMS_Driver: " x, a)
#define AMS_PORT_msg_2(x, a, b)         printk(KERN_INFO "AMS_Driver: " x, a, b)
#define AMS_PORT_msg_3(x, a, b, c)      printk(KERN_INFO "AMS_Driver: " x, a, b, c)
#define AMS_PORT_msg_4(x, a, b, c, d)   printk(KERN_INFO "AMS_Driver: " x, a, b, c, d)
#define AMS_PORT_msg_5(x, a, b, c, d,e)   printk(KERN_INFO "AMS_Driver: " x, a, b, c, d,e)
#else
#define AMS_PORT_msg(x)                 do{ } while (false)
#define AMS_PORT_msg_1(x, a)            do{ } while (false)
#define AMS_PORT_msg_2(x, a, b)         do{ } while (false)
#define AMS_PORT_msg_3(x, a, b, c)      do{ } while (false)
#define AMS_PORT_msg_4(x, a, b, c, d)   do{ } while (false)
#define AMS_PORT_msg_5(x, a, b, c, d,e)   do{ } while (false)
#endif



#define AMS_PORT_get_timestamp_usec(x)
extern uint8_t AMS_PORT_getByte(AMS_PORT_portHndl * handle, uint8_t reg, uint8_t * data, uint8_t len);
extern uint8_t AMS_PORT_setByte(AMS_PORT_portHndl * handle, uint8_t reg, uint8_t * data, uint8_t len);

extern int amsdriver_remove(struct i2c_client *client);
extern int amsdriver_resume(struct device *dev);
extern int amsdriver_suspend(struct device *dev);
extern int amsdriver_probe(struct i2c_client *client, const struct i2c_device_id *idp);

#ifdef	__cplusplus
}
#endif

#endif	/* AMS_PORT_PLATFORM_H */


