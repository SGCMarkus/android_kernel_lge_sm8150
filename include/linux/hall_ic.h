#ifndef _HALL_IC_H
#define _HALL_IC_H

struct hallic_dev {
	const char	*name;
	struct device	*dev;
	int 		state;
	int		state_front;
	int		state_back;
#ifdef CONFIG_LGE_COVER_DISPLAY
	struct list_head svid_handler_list;
#endif
};

int hallic_register(struct hallic_dev *hdev);
void hallic_unregister(struct hallic_dev *hdev);

static inline int hallic_get_state(struct hallic_dev *hdev)
{ return hdev->state; }
void hallic_set_state(struct hallic_dev *hdev, int state);

#endif
