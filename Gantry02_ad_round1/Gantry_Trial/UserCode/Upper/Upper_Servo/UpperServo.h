/*
 * @Author: ZYT
 * @Date: 2024-06-13 23:34:03
 * @LastEditors: ZYT
 * @LastEditTime: 2024-06-15 01:04:22
 * @FilePath: \Gantry_Trial\UserCode\Upper\Upper_Servo\UpperServo.h
 * @Brief: 
 * 
 * Copyright (c) 2024 by zyt, All Rights Reserved. 
 */
#ifndef __UPPERSERVO_H__
#define __UPPERSERVO_H__

#include "UpperStart.h"
void Upper_Servo_Start(void);

void Core_xy_Motor_init();


void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle);

#endif // __UPPERSERVO_H__