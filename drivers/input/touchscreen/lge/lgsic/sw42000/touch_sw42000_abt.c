/* touch_sw42000_abt.c
 *
 * Copyright (C) 2015 LGE.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define TS_MODULE "[abt]"

#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/signal.h>
#include <linux/netdevice.h>
#include <linux/ip.h>
#include <linux/in.h>
#include <linux/inet.h>
#include <linux/socket.h>
#include <linux/net.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_sw42000.h"
#include "touch_sw42000_abt.h"

struct mutex abt_socket_lock;
struct sock_comm_t abt_comm;
int abt_socket_mutex_flag;
int abt_socket_report_mode;

static int abt_ksocket_receive(unsigned char *buf, int len)
{
	struct msghdr msg = {0, };
	struct iovec iov = {0, };
	struct socket *sock;
	struct sockaddr_in *addr;
	mm_segment_t oldfs;
	unsigned int flag = 0;
	int iov_count = 1;
	int size = 0;

	sock = abt_comm.ts_sock;
	addr = &abt_comm.ts_addr;

	iov.iov_base = buf;
	iov.iov_len = len;

	iov_iter_init(&msg.msg_iter, READ, &iov, iov_count, len);
	msg.msg_name = addr;
	msg.msg_namelen  = sizeof(struct sockaddr_in);
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	msg.msg_flags = flag;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	size = sock_recvmsg(sock, &msg, msg.msg_flags);
	set_fs(oldfs);

	if (size > 0)
		abt_comm.sock_listener(buf, size);
	else
		TOUCH_I(": sock_recvmsg size invalid %d\n", size);

	return size;
}

static void abt_ksocket_start_for_pctool(struct device *dev)
{
	static int client_connected;
	int size = 0, err = 0;
	unsigned char *buf = 0;
	struct socket *sock;

	/* kernel thread initialization */
	abt_comm.running = 1;
	abt_comm.dev = dev;

	err = sock_create(AF_INET,
			SOCK_STREAM,
			IPPROTO_TCP,
			&abt_comm.ts_sock);
	sock = abt_comm.ts_sock;

	if (err >= 0) {
		memset(&abt_comm.ts_addr, 0, sizeof(struct sockaddr));
		abt_comm.ts_addr.sin_family = AF_INET;
		abt_comm.ts_addr.sin_addr.s_addr = in_aton(abt_comm.send_ip);
		abt_comm.ts_addr.sin_port = htons(TS_TCP_PORT);
	} else {
		TOUCH_I(
			MODULE_NAME": can not create socket %d\n",
			-err);
		goto out;
	}

	err = sock->ops->connect(sock,
				(struct sockaddr *)&abt_comm.ts_addr,
				sizeof(struct sockaddr), !O_NONBLOCK);

	if (err < 0) {
		TOUCH_E(MODULE_NAME": Could not connect to tcp rw socket, error = %d\n", -err);
		goto out;
	} else {
		client_connected = 1;
		TOUCH_I(MODULE_NAME": TCP connected with TS (ip %s,port %d)(\n",
			abt_comm.send_ip, TS_TCP_PORT);
	}

	buf = kzalloc(sizeof(struct s_comm_packet), GFP_KERNEL);


	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);

		if (abt_comm.running != 1)
			break;

		   size = abt_ksocket_receive(&buf[0],
		   sizeof(struct s_comm_packet));

		if (size < 1) {
			TOUCH_E(MODULE_NAME
				": RECEIVE sock_recvmsg invalid = %d\n", size);
			break;
		}
		if (kthread_should_stop()) {
			TOUCH_I(MODULE_NAME": kthread_should_stop\n");
			break;
		}
	}

out:
	if (buf != NULL)
		kfree(buf);

	__set_current_state(TASK_RUNNING);
	abt_comm.running = 0;
	abt_comm.thread = NULL;
	if (abt_comm.ts_sock != NULL) {
		sock_release(abt_comm.ts_sock);
		abt_comm.ts_sock = NULL;
	}
}

static int abt_ksocket_send(struct socket *sock,
			struct sockaddr_in *addr,
			unsigned char *buf, int len)
{
	struct msghdr msg = {0, };
	struct iovec iov = {0, };
	mm_segment_t oldfs;
	unsigned int flag = 0;
	int iov_count = 1;
	int size = 0;

	if (sock == NULL)
		return 0;

	iov.iov_base = buf;
	iov.iov_len = len;

	iov_iter_init(&msg.msg_iter, WRITE, &iov, iov_count, len);
	msg.msg_name = addr;
	msg.msg_namelen  = sizeof(struct sockaddr_in);
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	msg.msg_flags = flag;

	oldfs = get_fs();

	set_fs(KERNEL_DS);
	size = sock_sendmsg(sock, &msg);

	set_fs(oldfs);

	return size;
}

static uint32_t abt_ksocket_rcv_from_pctool(uint8_t *buf, uint32_t len)
{
	int ret;

	abt_comm.recv_packet = (struct s_comm_packet *)buf;

	if (abt_comm.recv_packet->hdr.cmd == TCP_REG_READ) {
		ret = sw42000_reg_read(abt_comm.dev,
					abt_comm.recv_packet->hdr.addr,
					&abt_comm.send_packet.data.frame[0],
					abt_comm.recv_packet->data.value);
		if (ret < 0) {
			TOUCH_I(
				MODULE_NAME
				": TCP REG READ spi_read error : %d\n\n", ret);
		}

		abt_comm.send_packet.hdr.cmd = abt_comm.recv_packet->hdr.cmd;
		abt_comm.send_packet.hdr.addr = abt_comm.recv_packet->hdr.addr;
		abt_comm.send_packet.hdr.size =
					abt_comm.recv_packet->data.value;

		abt_ksocket_send(abt_comm.ts_sock,
				&abt_comm.ts_addr,
				(u8 *)&abt_comm.send_packet,
				sizeof(TPacketHdr) +
				abt_comm.recv_packet->data.value);
	} else if (abt_comm.recv_packet->hdr.cmd == TCP_REG_WRITE) {
		ret = sw42000_reg_write(abt_comm.dev,
					abt_comm.recv_packet->hdr.addr,
					&abt_comm.recv_packet->data.frame[0],
					abt_comm.recv_packet->hdr.size);
		if (ret < 0) {
			TOUCH_I(
				MODULE_NAME
				": TCP REG WRITE bus_read error : %d\n\n", ret);
			abt_comm.send_packet.data.value = eCommRes_WriteFailed;
		} else {
			abt_comm.send_packet.data.value = eCommRes_Success;
		}
		abt_comm.send_packet.hdr.cmd = abt_comm.recv_packet->hdr.cmd;
		abt_comm.send_packet.hdr.addr = abt_comm.recv_packet->hdr.addr;
		abt_comm.send_packet.hdr.size = sizeof(u32);
		abt_ksocket_send(abt_comm.ts_sock,
				&abt_comm.ts_addr,
				(u8 *)&abt_comm.send_packet,
				sizeof(TPacketHdr) + sizeof(u32));
	}

	return 0;
}

static void abt_ksocket_exit(void)
{
	int err;

	if (abt_comm.thread == NULL) {
		if (abt_socket_mutex_flag == 1)
			mutex_destroy(&abt_socket_lock);
		abt_socket_mutex_flag = 0;
		abt_socket_report_mode = 0;
		abt_comm.running = 0;

		TOUCH_I(
			MODULE_NAME
			": no kernel thread to kill\n");

		return;
	}

	TOUCH_I(MODULE_NAME ": start killing thread\n");

	abt_comm.running = 2;

	TOUCH_I(MODULE_NAME": Send disconnect command to Touch solution\n");

	/* send disconnect command to pc server */
	abt_comm.send_packet.hdr.cmd = TCP_DISCONNECT_CMD;
	abt_comm.send_packet.hdr.size = 4;
	abt_ksocket_send(abt_comm.ts_sock,
			&abt_comm.ts_addr,
			(u8 *)&abt_comm.send_packet,
			sizeof(TPacketHdr) +
			abt_comm.send_packet.hdr.size);
	/* abt_comm.ts_sock->ops->disconnect(abt_comm.ts_sock, 0); */

	TOUCH_I(MODULE_NAME ": waiting for killing thread\n");
	err = kthread_stop(abt_comm.thread);
	if (err < 0) {
		TOUCH_I(MODULE_NAME ": unknown error %d while trying to terminate kernel thread\n", -err);
	} else {
		while (abt_comm.running != 0) {
			TOUCH_I(MODULE_NAME ": waiting for killing thread. abt_comm.running.. %d\n", abt_comm.running);
			touch_msleep(10);
		}
		TOUCH_I(MODULE_NAME ": succesfully killed kernel thread!\n");
	}

	abt_comm.thread = NULL;

	if (abt_comm.ts_sock != NULL) {
		sock_release(abt_comm.ts_sock);
		abt_comm.ts_sock = NULL;
	}

	mutex_destroy(&abt_socket_lock);
	abt_socket_mutex_flag = 0;
	abt_socket_report_mode = 0;

	TOUCH_I(": module unloaded\n");
}

static int32_t abt_ksocket_init(struct device *dev,
			char *ip,
			uint32_t (*listener)(uint8_t *buf, uint32_t len))
{
	mutex_init(&abt_socket_lock);
	abt_socket_mutex_flag = 1;
	abt_socket_report_mode = 1;
	memcpy(abt_comm.send_ip, ip, TS_IP_SIZE);

	abt_comm.thread =
		kthread_run((void *)abt_ksocket_start_for_pctool,
					dev, MODULE_NAME);

	if (IS_ERR(abt_comm.thread)) {
		TOUCH_I(
			MODULE_NAME
			": unable to start kernel thread\n");
		abt_comm.thread = NULL;
		return -ENOMEM;
	}

	abt_comm.sock_listener = listener;

	return 0;
}

ssize_t show_abtTool(struct device *dev, char *buf)
{
	int size = 0;

	memcpy((u8 *)&buf[0], (u8 *)&(abt_comm.send_ip[0]), TS_IP_SIZE);
	size += TS_IP_SIZE;
	TOUCH_I(MODULE_NAME ":Touch Solution ip:%s\n", (char *)&buf[0]);

	return size;
}

enum {
	STORE_ABT_MODE_STUDIO = 1,
	STORE_ABT_MODE_TOUCH,
	STORE_ABT_MODE_MAX,
};

ssize_t store_abtTool(struct device *dev,
				const char *buf,
				size_t count)
{
#if (0)
	int mode = buf[0];
	char *ip = (char *)&buf[1];

	TOUCH_I(
		":set raw report mode - mode:%d IP:%s\n", mode, ip);

	if (mode > 47)
		mode -= 48;
#else
	int mode = 0;
	char __ip[16+1] = { '0', };
	char *ip = NULL;

	ip = __ip;
	if (sscanf(buf, "%d %16s", &mode, ip) <= 0) {
		//siw_abt_sysfs_err_invalid_param(abt);
		return count;
	}
	TOUCH_I(
		":set raw report mode - mode:%d IP:%s\n", mode, ip);

#endif

	switch (mode) {
	case STORE_ABT_MODE_STUDIO:
		TOUCH_E(
				"Invalid mode (%d)\n", mode);
		break;
	case STORE_ABT_MODE_TOUCH:
		if (abt_comm.thread == NULL) {
			TOUCH_I(
				": mode Touch Solution Start\n");
			abt_ksocket_init(dev, (u8 *)ip,
					 abt_ksocket_rcv_from_pctool);
		} else {
			TOUCH_I(
				": abt_comm.thread Not NULL\n\n");
			if (memcmp((u8 *)abt_comm.send_ip,
					(u8 *)ip, 16) != 0) {
				abt_ksocket_exit();
				abt_ksocket_init(dev, (u8 *)ip,
						 abt_ksocket_rcv_from_pctool);
			} else {
				TOUCH_I(
					": same IP\n\n");
			}
		}
		break;
	default:
		abt_ksocket_exit();
	}

	return count;
}

static TOUCH_ABT_ATTR(raw_report, show_abtTool, store_abtTool);

static struct attribute *sw42000_abt_attribute_list[] = {
	&touch_attr_raw_report.attr,
	NULL,
};

static const struct attribute_group sw42000_abt_attribute_group = {
	.attrs = sw42000_abt_attribute_list,
};

int sw42000_abt_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = sysfs_create_group(&ts->kobj, &sw42000_abt_attribute_group);

	if (ret < 0)
		TOUCH_E("failed to create sysfs for abt\n");
	return ret;
}
