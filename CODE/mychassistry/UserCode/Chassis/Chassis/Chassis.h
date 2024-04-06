/*
 * @Author: Chen Yitong
 * @Date: 2024-01-08 13:46:49
 * @LastEditors: 15867773077
 * @LastEditTime: 2024-01-08 20:40:18
 * @FilePath: \MyChassisTry - 副本\UserCode\Chassis\Chassis\Chassis.h
 * @Brief: 底盘结构体初始化 底盘逆解算法 舵轮初始化
 *
 * Copyright (c) 2024 by ChenYiTong, All Rights Reserved.
 */
#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "Chassis_Start.h"

void Chassis_Init();

void FourSteeringInverseCalculate();

void WheelCorrect();

#endif // __CHASSIS_H__