#ifndef __UPPERSERVO_H__
#define __UPPERSERVO_H__

#include "UpperStart.h"
void Upper_Servo_Start(void);

void Core_xy_Motor_init();

void Core_xy_Servo(CoreXYState Target);

#endif // __UPPERSERVO_H__