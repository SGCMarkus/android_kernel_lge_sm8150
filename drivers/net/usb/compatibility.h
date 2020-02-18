#ifndef LINUX_COMPATIBILITY_H
#define LINUX_COMPATIBILITY_H

/*
 * Definition and macro
 */

#include <linux/init.h>
#include <linux/version.h>
#include <linux/in.h>
#include <linux/mdio.h>
#include <uapi/linux/mdio.h>

#ifndef FALSE
	#define TRUE	1
	#define FALSE	0
#endif

enum rtl_cmd {
	RTLTOOL_PLA_OCP_READ_DWORD = 0,
	RTLTOOL_PLA_OCP_WRITE_DWORD,
	RTLTOOL_USB_OCP_READ_DWORD,
	RTLTOOL_USB_OCP_WRITE_DWORD,
	RTLTOOL_PLA_OCP_READ,
	RTLTOOL_PLA_OCP_WRITE,
	RTLTOOL_USB_OCP_READ,
	RTLTOOL_USB_OCP_WRITE,
	RTLTOOL_USB_INFO,
	RTL_ENABLE_USB_DIAG,
	RTL_DISABLE_USB_DIAG,

	RTLTOOL_INVALID
};

struct usb_device_info {
	__u16		idVendor;
	__u16		idProduct;
	__u16		bcdDevice;
	__u8		dev_addr[8];
	char		devpath[16];
};

struct rtltool_cmd {
	__u32	cmd;
	__u32	offset;
	__u32	byteen;
	__u32	data;
	void	*buf;
	struct usb_device_info nic_info;
	struct sockaddr ifru_addr;
	struct sockaddr ifru_netmask;
	struct sockaddr ifru_hwaddr;
};

#endif /* LINUX_COMPATIBILITY_H */
