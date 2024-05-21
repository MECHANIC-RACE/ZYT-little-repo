#ifndef __STATEMACHINE_H__
#define __STATEMACHINE_H__

#include "UpperStart.h"

void Core_xy_StateMachine_Start(void);
void Core_XY_StateMachine_Init();
void pid_reset(PID_t *pid, float kp, float ki, float kd);

#endif // __STATEMACHINE_H__