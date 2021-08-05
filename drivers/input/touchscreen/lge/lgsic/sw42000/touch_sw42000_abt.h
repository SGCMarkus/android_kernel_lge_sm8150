/* touch_sic_abt_spi.h
 *
 * Copyright (C) 2015 LGE.
 *
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

#ifndef TOUCH_SIC_ABT_H
#define TOUCH_SIC_ABT_H

#include <linux/socket.h>
#include <linux/in.h>

#define TOUCH_ABT_ATTR(_name, _show, _store)		\
			struct touch_attribute touch_attr_##_name	\
			= __ATTR(_name, 0660, _show, _store)

#define TS_IP_SIZE		16

#define DEFAULT_PORT	(8095)
#define TS_TCP_PORT		(8097)
#define SEND_PORT		(8090)
#define OMK_BUF			(1000)

#define MODULE_NAME		"ABT_SOCKET"

#pragma pack(push, 1)
/* TCP */
enum ECommCMD {
	TCP_REG_READ = 0x80,
	TCP_REG_WRITE,
	TCP_FRAME_START,
	TCP_REPORT_START,
	TCP_SYNC_START,
	TCP_SYNCDEBUG_START,
	TCP_CAPTURE_STOP,
	TCP_CONNECT_CMD,
	TCP_DISCONNECT_CMD
};

enum ECommRes {
	eCommRes_Success	 = 0,
	eCommRes_WriteFailed = 0x8001
};

typedef struct {
	u8		cmd;
	u16		addr;
	u16		size;
} TPacketHdr, *PPacketHdr;

struct s_comm_packet {
	TPacketHdr	hdr;
	union {
		u32		value;
		u8		frame[4000];
	} data;
};

struct sock_comm_t {
	struct device *dev;

	struct task_struct *thread;

	/* Touch Solution socket */
	struct socket *ts_sock;
	struct sockaddr_in ts_addr;

	uint32_t (*sock_listener)(uint8_t *buf, uint32_t len);

	struct s_comm_packet *recv_packet;
	struct s_comm_packet send_packet;

	char send_ip[20];

	u8 running;
};
#pragma pack(pop)

extern int sw42000_abt_register_sysfs(struct device *dev);
#endif

