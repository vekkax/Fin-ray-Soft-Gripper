// gripper_control.h
#ifndef __GRIPPER_CONTROL_H__
#define __GRIPPER_CONTROL_H__

typedef enum {
	START=0,
	IDLE,
	CONTROL,
	END
} Gripper;

extern Gripper gripperState;

#endif /* __GRIPPER_CONTROL_H__ */
