/*
 * @Author: ZYT
 * @Date: 2024-05-14 22:31:49
 * @LastEditors: ZYT
 * @LastEditTime: 2024-06-10 12:58:11
 * @FilePath: \Gantry_Trial\UserCode\Upper\Upper_StateMachine\Area1State.h
 * @Brief: 
 * 
 * Copyright (c) 2024 by zyt, All Rights Reserved. 
 */
#ifndef __STATEMACHINE_H__
#define __STATEMACHINE_H__

#include "UpperStart.h"

void Area1_StateMachine_Start(void);
void Area1_StateMachine_Init();
void pid_reset(PID_t *pid, float kp, float ki, float kd);
void Cylinder_up(uint16_t i);
void Cylinder_down(uint16_t i);

#endif // __STATEMACHINE_H__