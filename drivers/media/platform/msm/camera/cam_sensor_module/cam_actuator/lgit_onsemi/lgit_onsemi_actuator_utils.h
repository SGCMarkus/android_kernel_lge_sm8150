#ifndef LGIT_ONSEMI_AF_H
#define LGIT_ONSEMI_AF_H
/* ================================================================== */
/*  ONSEMI AF HALL Utils  */
/* ================================================================== */

#define MAX_HALL_QUERY_SIZE 15
#define READ_OUT_TIME 5000000 /*5ms*/
#define MAX_FAIL_CNT 3

enum OISUserData {
	NO_SELECTION,
	HALL_FEEDING,
	USERDATAEND
};

enum msm_actuator_timer_state_t {
	ACTUATOR_TIME_INIT,
	ACTUATOR_TIME_ACTIVE,
	ACTUATOR_TIME_INACTIVE,
	ACTUATOR_TIME_ERROR,
};

struct actuator_timer {
	struct hrtimer hr_timer;
	struct workqueue_struct *actuator_wq;
	struct work_struct g_work;
	enum msm_actuator_timer_state_t timer_state;
	int i2c_fail_count;
	void *a_ctrl;
};

#endif /* _LGIT_ONSEMI_AF_UTIL_H */
