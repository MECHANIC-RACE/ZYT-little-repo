/*
 * @Author: Chen Yitong
 * @Date: 2023-09-22 22:19:36
 * @LastEditors: 15867773077
 * @LastEditTime: 2024-01-08 20:26:10
 * @FilePath: \MyChassisTry - 副本\UserCode\Chassis\Start\Chassis_Start.h
 * @brief 底盘启动文件以及所有库
 *
 * Copyright (c) 2023 by ChenYiTong, All Rights Reserved.
 */
#ifndef __CHAEEIS_START_H__
#define __CHAEEIS_START_H__
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "wtr_calculate.h"
#include "wtr_can.h"
#include "wtr_dji.h"
#include "wtr_uart.h"
#include "wtr_vesc.h"
#include "wtr_math.h"
#include "Chassis.h"
#include "Chassis_Callback.h"
#include "Chassis_commen.h"
#include "Chassis_UserConfig.h"
#include "Chassis_RemoteCtl.h"
#include "Chassis_Servo.h"
#include "Chassis_StateMachine.h"
#include "Chassis_Variable.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "uart_printf.c"
#endif