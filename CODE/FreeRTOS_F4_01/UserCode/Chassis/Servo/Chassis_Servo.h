/*
 * @Author: Chen Yitong
 * @Date: 2023-09-23 13:49:46
 * @LastEditors: labbbbbbbbb 
 * @LastEditTime: 2024-04-13 14:56:29
 * @FilePath: \FreeRTOS_F4_01\UserCode\Chassis\Servo\Chassis_Servo.h
 * @Brief: 底盘伺服函数
 *
 * Copyright (c) 2023 by ChenYiTong, All Rights Reserved.
 */

#ifndef __CHASSIS_SERVO_H__
#define __CHASSIS_SERVO_H__
#include"Chassis_Start.h"

void Chassis_Servo_Init();
void Chassis_Servo_TaskStart();
//void Chassis_Servo_Task(void *argument);
#endif // __CHASSIS_SERVO_H__