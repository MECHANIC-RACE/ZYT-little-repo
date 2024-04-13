/*
 * @Author Chen Yitong
 * @Date 2023-09-22 22:19:27
 * @LastEditors: labbbbbbbbb 
 * @LastEditTime: 2024-04-13 15:15:04
 * @FilePath: \FreeRTOS_F4_01\UserCode\Chassis\Start\Chassis_Start.c
 * @brief 底盘启动文件
 *
 * Copyright (c) 2023 by ChenYiTong, All Rights Reserved.
 */
#include "Chassis_Start.h"

/**
 * @brief 默认任务，启动其他线程
 * @return {*}
 */
void StartDefaultTask(void const *argument)
{
    //================初始化====================
    Chassis_RemoteCtl_Init();    // 遥控初始化
    Chassis_StateMachine_Init(); // 状态机初始化
    Chassis_Servo_Init();        // 底盘电机初始化
    // ================启动线程==================
    Chassis_Servo_TaskStart(); // 底盘伺服进程启动
    StateMachine_TaskStart();  // 状态机进程启动

    for (;;)
    {
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
        osDelay(1000);
    }
}