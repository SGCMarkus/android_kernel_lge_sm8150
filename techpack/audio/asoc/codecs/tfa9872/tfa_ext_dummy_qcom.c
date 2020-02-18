/*
 *Copyright 2016 NXP Semiconductors
 *
 *Licensed under the Apache License, Version 2.0 (the "License");
 *you may not use this file except in compliance with the License.
 *You may obtain a copy of the License at
 *
 *http://www.apache.org/licenses/LICENSE-2.0
 *
 *Unless required by applicable law or agreed to in writing, software
 *distributed under the License is distributed on an "AS IS" BASIS,
 *WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *See the License for the specific language governing permissions and
 *limitations under the License.
 */

/*
 * tfa_ext_DUMMY.c
 *
 *	external DSP/RPC interface dummy for reference demo
 *
 *	The front-end is the tfa_ext pre-fixed code which can be the same for all
 *	platforms.
 *	The remote_*() functions abstract the interface of the communication
 *	between tfa_ext and the remote DSP platform. It is a raw buffer interchange.
 *	In dummy_dsp_*() the remote response is emulated.
 *	debugfs is available for generating remote events and the kernel log will
 *	shows the behavior.
 *	Note that ftrace function trace could be used to trace the call flow.
 *
 *
 */
#define pr_fmt(fmt) "%s(): " fmt, __func__
#include "inc/config.h"
#include "inc/tfa_internal.h"
#include "inc/tfa_ext.h"

#include <linux/module.h>

#define TFA_EXT_DEBUGFS		/* enable the debugfs test interface */
#define RCV_BUFFER_SIZE 2048

static int tfa_ext_event_code(void *buf, int size);

/*
 * module globals
 */
static tfa_event_handler_t tfa_ext_handle_event;

static u8 rcv_buffer[RCV_BUFFER_SIZE];
static int rcv_size;
static void hexdump(int length, u8 *data );

/* in q6afe.c */
int afe_tfa_dsp_send_msg (int dev, int buf_size, char *buf);
int afe_tfa_dsp_read_msg (int dev, int buf_size, char *buf);

#define FAKE_APR_MSG
#ifdef FAKE_APR_MSG
int tfa_dsp_send_msg(int length, void *buffer) {
	int ret;

	pr_info("\n");
	hexdump(length, buffer);

	ret = afe_tfa_dsp_send_msg(0, length, buffer);
	pr_info("afe_tfa_dsp_send_msg() returned:%d\n", ret);

	//FW_PAR_ID_GET_API_VERSION:
//	memset(rcv_buffer, 0, sizeof(rcv_buffer));
//	rcv_buffer[3] = 1;
//	rcv_buffer[4] = 2;
//	rcv_buffer[5] = 3;

	return length;

}
int tfa_dsp_read_msg(int length, void *buffer) {
	int ret;
	int size = min(RCV_BUFFER_SIZE, length);

	pr_info("\n");
	ret = afe_tfa_dsp_read_msg(0, length, buffer);
	pr_info("afe_tfa_dsp_read_msg() returned:%d\n", ret);

	//memcpy(buffer, rcv_buffer, size);
	hexdump(size, buffer);

	return size;

}
#endif

/*
 * debug only
 *   interface *buffer,count order same as for rpmsg
 */
static void hexdump(int length, u8 *data )
{
	int i;

	for (i = 0; i < length; i++) {
		printk("0x%02x ", data[i]);
	}
	printk("\n");
}


/*
 * the trace_level parameter can be used to enabled full buffer payload prints
 */
static int trace_level = 0;
module_param(trace_level, int, S_IRUGO);
MODULE_PARM_DESC(trace_level, "tfa_ext_dummy trace_level (0=off).");

#ifdef TFA_EXT_DEBUGFS
/*
 * debugfs for event testing
 *   /sys/kernel/debug/tfa_ext_dummy/event : event input in ascii
 *   						     refer to the tfa_ext_event_code() below
 *   						      1=dsp powered up
 *   						      2=cmd ready
 *   						      3=dsp powered down
 *   /sys/kernel/debug/tfa_ext_dummy/status: returns msg byte count
 *                                            in ascii since last event
 */
#include <linux/debugfs.h>
/* This directory entry will point to `/sys/kernel/debug/tfa_ext_dummy`.*/
static struct dentry *dir = 0;
/* File `/sys/kernel/debug/tfa_ext_dummy/sum` points to this variable.*/
static u32 debugfs_status = 0;

/*
 This is called when `/sys/kernel/debug/tfa_ext_dummy/event` is written to.

 Executing `echo 1 >> /sys/kernel/debug/tfa_ext_dummy/event` will call
 `debugfs_event(NULL, 1)`.
*/
/*
 *
 */

static int debugfs_event(void *data, u64 value)
{
    int event_call_return, event;

    debugfs_status = 0;

    pr_info("%llx\n",value);

    event = tfa_ext_event_code(&value,  sizeof(u64)); /* interpret code */

	/* call the tfa driver here to handle this event */
    event_call_return = tfa_ext_handle_event(0, event);
	if (event_call_return < 0)
		return TFA_ERROR;

	return 0;

}

DEFINE_SIMPLE_ATTRIBUTE(add_fops, NULL, debugfs_event, "%llu\n");

/* This is called when the module loads.*/
static int debugfs_init_module(void)
{
    struct dentry *junk;

/*     Create directory `/sys/kernel/debug/tfa_ext_dummy`.*/

    dir = debugfs_create_dir("tfa_ext_dummy", 0);
    if (!dir) {
        // Abort module load.
        pr_err("debugfs_tfa_ext_dummy: failed to create /sys/kernel/debug/tfa_ext_dummy\n");
        return TFA_ERROR;
    }

/* Create file `/sys/kernel/debug/tfa_ext_dummy/event`.*/

    junk = debugfs_create_file(
            "event",
            0222,
            dir,
            NULL,
            &add_fops);
    if (!junk) {
        // Abort module load.
        pr_err("debugfs_tfa_ext_dummy: failed to create /sys/kernel/debug/tfa_ext_dummy/event\n");
        return TFA_ERROR;
    }

/*
     Create file `/sys/kernel/debug/tfa_ext_dummy/status`.
*/
    junk = debugfs_create_u32("status", 0444, dir, &debugfs_status);
    if (!junk) {
        pr_err("debugfs_tfa_ext_dummy: failed to create /sys/kernel/debug/tfa_ext_dummy/status\n");
        return TFA_ERROR;
    }

    return 0;
}

/* This is called when the module is removed.*/
static void debugfs_cleanup_module(void)
{
    debugfs_remove_recursive(dir);
}
/*
 * end debugfs
 */
#endif /*debugfs*/


/*
 * remote tfa interfacing
 *
 *  TFADSP_EXT_PWRUP 	: DSP starting
 *  	set cold to receive messages
 *  TFADSP_CMD_READY	: Ready to receive commands
 *  	call tfa_start to send msgs
 *	TFADSP_SPARSESIG_DETECTED :Sparse signal detected
 *		call sparse protection
 *	TFADSP_EXT_PWRDOWN 	: DSP stopping
 *       disable DPS msg forwarding
 *
 */


/*
 * return the event code from the raw input
 *   for this dummy case pick 1st byte
 */
static int tfa_ext_event_code(void *inbuf, int size)
{
	int code;
	u8 *buf = (u8 *)inbuf;

	switch(buf[0]) {
	case 1:
		pr_info("TFADSP_EXT_PWRUP\n");/**< DSP API has started, powered up */
		code = TFADSP_EXT_PWRUP;
		break;
	case 2:
		pr_info("TFADSP_CMD_READY\n");/**< Ready to receive commands */
		code = TFADSP_CMD_READY;
		break;
	case 3:
		pr_info("TFADSP_EXT_PWRDOWN\n");/**< DSP API stopped, power down */
		code = TFADSP_EXT_PWRDOWN;
		break;
	case 4:
		pr_info("TFADSP_CMD_ACK\n");/**< Sparse signal detected */
		code = TFADSP_CMD_ACK;
		break;
	case 5:
		pr_info("TFADSP_SPARSESIG_DETECTED\n");/**< Sparse signal detected */
		code = TFADSP_SPARSESIG_DETECTED;
		break;
	default:
		pr_info("not handled:%d\n", buf[0]);
		code = -1;
		break;
	}

	return code;
}


/**
@brief DSP message interface that sends the RPC to the remote TFADSP

This is the the function that get called by the tfa driver for each
RPC message that is to be send to the TFADSP.
The tfa_ext_registry() function will connect this.

@param [in] devidx : tfa device index that owns the TFADSP
@param [in] length : length in bytes of the message in the buffer
@param [in] buffer : buffer pointer to the RPC message payload

@return 0  success
int apr_send_pkt(void *handle, uint32_t *buf)
*/
static int tfa_ext_dsp_msg(int devidx, int length, const char *buffer)
{
	int error = 0, real_length;
	u8 *buf = (u8 *) buffer;

	if (trace_level > 0)
		pr_debug("id:0x%02x%02x%02x, length:%d \n",
			 buf[0], buf[1], buf[2], length);

	real_length = length;//remote_send((u8 *) buffer, length);
	//
	pr_debug("apr_send_pkt...\n");

	memset(rcv_buffer,0 ,sizeof(rcv_buffer));

	return error;
}

static int tfa_ext_dsp_msg_read(int devidx, int length, char *buffer)
{
	int error = 0;
	int size = min(length, rcv_size);

	pr_debug("%s(%d,%d,%p)\n", __func__, devidx, length, buffer);

	memcpy(buffer, rcv_buffer, size);

	return error;
}

/**
@brief Register at the tfa driver and instantiate the remote interface functions.

This function must be called once at startup after the tfa driver module is
loaded.
The tfa_ext_register() will be called to get the  event handler and dsp message
interface functions and the remote TFADSP will be connected after successful
return.

@param void

@return 0 on success

*/
//#if defined(__x86_64__) | defined(__i386__) /* for on PC testing */
static int tfa_dsp_handle_event(tfa98xx_handle_t handle, enum tfadsp_event_en tfadsp_event)
{
	int retval = 0;// handles_local[handle].rev; /* return revid by default */
	u8 msgbuf[200], readbuf[200], *bptr;
//	enum tfa_error err;
	//pr_info("event:0x%x\n", tfadsp_event);

	switch( tfadsp_event )
	{
	case TFADSP_EXT_PWRUP: /*DSP starting*/
		pr_info("execute read API version\n");
		bptr = msgbuf;
		*bptr++ = 0x00;
		*bptr++ = 0x80;
		*bptr++ = 0xfe; //FW_PAR_ID_GET_API_VERSION
		retval = tfa_dsp_send_msg(3, msgbuf);
		// chek ret ?
		retval = tfa_dsp_read_msg(16, readbuf);
		if ( 0 ) {
			pr_info("ERROR return from API version call: %d\n", retval);
		} else
			pr_info("API version: %d.%d.%d\n",readbuf[5] , readbuf[4] ,   readbuf[3] ); //MSB ok?


	///	handles_local[handle].ext_dsp = 1;
		break;
	case TFADSP_CMD_READY: /*Ready to receive commands*/
		/* confirm */
  	    pr_info("TFADSP_CMD_READY: call tfa_start to send msgs\n");
		pr_info("TFADSP_CMD_READY: set cold\n");
		//handles_local[handle].ext_dsp = 1;
  	    //err = tfa_start(handles_local[handle].profile, handles_local[handle].vstep);
//		if ( err == tfa_error_ok) {
//			handles_local[handle].ext_dsp = 2; /* set warm */
//		} else
//			retval = -1*err;
		break;
	default:
		pr_err("%s: unknown tfadsp event:0x%0x\n",__func__, tfadsp_event);
		retval = -1;
		break;
	}
	return retval;
}

//int tfa_ext_register(dsp_send_message_t tfa_send_message, dsp_read_message_t tfa_read_message, tfa_event_handler_t *tfa_event_handler)
int tfa_ext_register(dsp_write_reg_t tfa_write_reg, dsp_send_message_t tfa_send_message, dsp_read_message_t tfa_read_message, tfa_event_handler_t *tfa_event_handler)
{
	//dsp_send_message_t send;
	//dsp_read_message_t rcv;

	pr_debug("fake %s\n", __func__);

	pr_info(" test only\n");

//	handles_local[0].dev_ops.dsp_msg = (dsp_msg_t)tfa_send_message;
//	handles_local[0].dev_ops.dsp_msg_read = (dsp_msg_read_t)tfa_read_message;
	if ( tfa_event_handler != NULL )
		*tfa_event_handler = tfa_dsp_handle_event;

	return 0;
}
//#endif

static int tfa_ext_registry(void)
{

	pr_debug("%s\n", __func__);

	pr_debug("apr_open / prepare ...\n");
	if (tfa_ext_register(NULL, tfa_ext_dsp_msg, tfa_ext_dsp_msg_read, &tfa_ext_handle_event)) {
		pr_err("Cannot register to tfa driver!\n");
		return 1;
	}

	return 0;
}


/**
@brief Un-register and close the remote interface.

This function must be called once at shutdown of the remote device.

@param void

@return 0 on success

*/

static void tfa_ext_unregister(void)
{
	pr_debug("%s\n", __func__);

	/* send to disabel DSP msg */
	tfa_ext_handle_event(0, TFADSP_EXT_PWRDOWN);

}

/*
 * register at tfa98xx and create the work queue for the events
 */
static int __init tfa_ext_dummy_init(void)
{
//	pr_info("build: %s %s\n", __DATE__, __TIME__);
	pr_info("trace_level:%d\n", trace_level);

	debugfs_init_module();
	tfa_ext_registry();	//TODO add error check

	pr_info("done\n");
	return 0;

}
/*
 * cleanup and disappear
 */
static void __exit tfa_ext_dummy_exit(void)
{
	tfa_ext_unregister();
	debugfs_cleanup_module();
	pr_info("done\n");
}

module_init(tfa_ext_dummy_init);
module_exit(tfa_ext_dummy_exit);

MODULE_DESCRIPTION("TFA98xx remote dummy DSP driver");
MODULE_LICENSE("GPL");
