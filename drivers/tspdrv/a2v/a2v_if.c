#include <linux/module.h>
#include <asm/uaccess.h>
//#include <asm/system.h>
#include <linux/bitops.h>
#include <linux/capability.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/socket.h>
#include <linux/sockios.h>
#include <linux/in.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/if_ether.h>
#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include <linux/init.h>
#include <linux/notifier.h>
#include <linux/inetdevice.h>
#include <linux/ip.h>
#include <linux/kthread.h>

#include <net/arp.h>
#include <net/ip.h>
#include <net/route.h>
#include <net/ip_fib.h>

#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <linux/kfifo.h>
#include <linux/time.h>
#include <linux/delay.h>

#include "a2v_if.h"

#define GUNP_DEBUG 0

#if GUNP_DEBUG
    #define dprint(fmt, x...) printk("A2V %s : " fmt, __FUNCTION__, ## x)
#else
    #define dprint(x...) do {} while (0)
#endif

#define MODULE_NAME a2v

static DEFINE_MUTEX(read_lock);
static DEFINE_MUTEX(write_lock);

static DECLARE_WAIT_QUEUE_HEAD(idle_wait);

static char write_buffer1[20];
static char write_buffer2[20];
static char write_buffer3[20];
static char write_buffer4[20];

/* =====================================================================================
function : PCM Converting Code for example
descript : Android Audio HAL Code
version  : 1.0
release  : 2018.08.24
====================================================================================== */

unsigned char test_data_buf[62] = {
    0x00, 0x0E, 0x1D, 0x2B, 0x39, 0x46, 0x52, 0x5D, 0x66, 0x6E, 0x75, 0x7A, 0x7D, 0x7E, 0x7E, 0x7C, 0x78, 0x73, 0x6C, 0x63,
    0x59, 0x4E, 0x42, 0x35, 0x27, 0x18, 0x09, 0xFC, 0xED, 0xDE, 0xD0,
    0xC2, 0xB6, 0xAA, 0xA0, 0x97, 0x8F, 0x89, 0x85, 0x82, 0x81, 0x82, 0x85, 0x89, 0x8F, 0x97, 0xA0, 0xAA, 0xB6, 0xC2, 0xD0,
    0xDE, 0xED, 0xFC, 0x09, 0x18, 0x27, 0x35, 0x42, 0x4E, 0x59, 0x63
};

/* =====================================================================================
function : write file function in kernel space
descript : write a pcm file from audio HAL, only for test purpose 
version  : 1.0
release  : 2018.08.28
====================================================================================== */

int a2v_open(struct inode *inode, struct file *filp)
{

    int ret = -ENODEV;

    if(!try_module_get(THIS_MODULE)) {
        goto error;
    }

    ret = 0;
    
error:
	return ret;

}

int a2v_release(struct inode *inode, struct file *filp)
{
    module_put(THIS_MODULE);

    return 0;
}

ssize_t a2v_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
    unsigned char reg_val;

	pr_debug("%s, enter\n", __func__);

    reg_val = a2v_byte_read(0x03);

    if (reg_val < 0) {
        pr_err("%s: immersion haptic is busy. skip this function\n", __func__);
        return count;
    }

    if (0 != copy_from_user(write_buffer1, buf, count/4))
    {
        /* Failed to copy all the data, exit */
        pr_err("%s: copy_from_user failed.\n", __func__);
        return -EIO;
    }

    if (0 != copy_from_user(write_buffer2, buf + count/4, count/4))
    {
        /* Failed to copy all the data, exit */
        pr_err("%s: copy_from_user failed.\n", __func__);
        return -EIO;
    }

    if (0 != copy_from_user(write_buffer3, buf + count/4 * 2, count/4))
    {
        /* Failed to copy all the data, exit */
        pr_err("%s: copy_from_user failed.\n", __func__);
        return -EIO;
    }

    if (0 != copy_from_user(write_buffer4, buf + count/4 * 3, count/4))
    {
        /* Failed to copy all the data, exit */
        pr_err("%s: copy_from_user failed.\n", __func__);
        return -EIO;
    }

	pr_debug("%s, count = %d, fifo = %d\n", __func__, (int)count, reg_val);
	a2v_seq_write( (u8*)write_buffer1, 20 );
	a2v_seq_write( (u8*)write_buffer2, 20 );
	a2v_seq_write( (u8*)write_buffer3, 20 );
	a2v_seq_write( (u8*)write_buffer4, 20 );

	pr_debug("%s, exit\n", __func__);
	
	return count;       
}

static void a2v_remove(void)
{
	pr_debug("%s\n", __func__);
}

struct file_operations a2v_fops = {
  .owner    = THIS_MODULE,
  .open     = a2v_open,
  .release  = a2v_release,
  .write = a2v_write,
};


struct miscdevice a2v_miscdev = {
    MISC_DYNAMIC_MINOR, "a2v", &a2v_fops
};

static int __init a2v_init(void)
{
	int ret = 0;
	
	pr_err("%s, enter\n", __func__);

	ret = misc_register(&a2v_miscdev);
	if( ret < 0 )
	{
		printk(KERN_ERR "a2v misc driver register error\n");
		return ret;
	}

	pr_err("%s, exit\n", __func__);

    return 0;
}

static void __exit a2v_exit(void)
{	
    a2v_remove();
    misc_deregister(&a2v_miscdev);
}

module_init(a2v_init);
module_exit(a2v_exit);

MODULE_AUTHOR("gunpguy");
