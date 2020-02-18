#ifndef _LGE_COVER_DISPLAY_H_
#define _LGE_COVER_DISPLAY_H_

enum cover_display_state {
	COVER_DISPLAY_STATE_DISCONNECTED = 0,
	COVER_DISPLAY_STATE_CONNECTED_CHECKING,
	COVER_DISPLAY_STATE_CONNECTED_ERROR,
	COVER_DISPLAY_STATE_CONNECTED_OFF,
	COVER_DISPLAY_STATE_CONNECTED_ON,
	COVER_DISPLAY_STATE_CONNECTED_SUSPEND,
	COVER_DISPLAY_STATE_CONNECTED_POWERDROP,
};

enum cover_display_state get_cover_display_state(void);
int set_cover_display_state(enum cover_display_state state);
void set_dd_skip_uevent(int skip);

#endif // _LGE_COVER_DISPLAY_H_
