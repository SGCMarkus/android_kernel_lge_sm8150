#ifndef __LINUX_TOUCH_NOTIFY_H
#define __LINUX_TOUCH_NOTIFY_H

#include <linux/notifier.h>

#define NOTIFY_NO_EVENT			0x00
#define NOTIFY_TOUCH_RESET		0x01
#define NOTIFY_CONNECTION		0x02
#define NOTIFY_WIRELESS			0x03
#define NOTIFY_IME_STATE		0x04
#define NOTIFY_DEBUG_TOOL		0x05
#define NOTIFY_CALL_STATE		0x06
#define NOTIFY_FB			0x07
#define NOTIFY_EARJACK			0x08
#define NOTIFY_DEBUG_OPTION		0x09
#define NOTIFY_ONHAND_STATE		0x0A
#define NOTIFY_QMEMO_STATE		0x0B
#define NOTIFY_PMEMO_STATE		0x0C
#define NOTIFY_FILM_STATE		0x0D
#define NOTIFY_DD_HALLIC_STATE		0x0E
#define NOTIFY_DUALSCREEN_STATE		0x0F

#define LCD_EVENT_LCD_MODE		0xD0
#define LCD_EVENT_READ_REG		0xD1
#define LCD_EVENT_TOUCH_RESET_START	0xD2
#define LCD_EVENT_TOUCH_RESET_END	0xD3
#define LCD_EVENT_LCD_BLANK		0xD4
#define LCD_EVENT_LCD_UNBLANK		0xD5
#define LCD_EVENT_TOUCH_LPWG_ON		0xD6
#define LCD_EVENT_TOUCH_LPWG_OFF	0xD7

enum {
	ATOMIC_NOTIFY_CONNECTION = 0,
	ATOMIC_NOTIFY_WIRELESS,
	ATOMIC_NOTIFY_EARJACK,
	ATOMIC_NOTIFY_EVENT_SIZE,
};

struct atomic_notify_event {
	unsigned long event;
	int data;
};

int touch_blocking_notifier_register(struct notifier_block *nb);
int touch_blocking_notifier_unregister(struct notifier_block *nb);
int touch_blocking_notifier_call(unsigned long val, void *v);

int touch_atomic_notifier_register(struct notifier_block *nb);
int touch_atomic_notifier_unregister(struct notifier_block *nb);
int touch_atomic_notifier_call(unsigned long val, void *v);

int touch_register_client(struct notifier_block *nb);
int touch_unregister_client(struct notifier_block *nb);
int touch_notifier_call_chain(unsigned long val, void *v);

extern int ignore_compared_event;

#endif /* _LINUX_TOUCH_NOTIFY_H */
