/*
 * @Author: ZYT
 * @Date: 2024-05-19 14:41:19
 * @LastEditors: ZYT
 * @LastEditTime: 2024-07-20 22:30:20
 * @FilePath: \Gantry_Trial\UserCode\Upper\Upper_Start\UpperStart.c
 * @Brief: 
 * 
 * Copyright (c) 2024 by zyt, All Rights Reserved. 
 */

#include "UpperStart.h"

void StartDefaultTask(void *argument)
{
    Uart_State = 2; // 后续要删！
    /*等待树莓派数据*/
    RaspReceive_Enable();
    /*串口使能*/
    Usart_start();
    UsartUpdate_Start();

    /*初始化函数*/
    LightFury_Motor_init();
    /*开启线程*/
    StateMachine_Start();
    Upper_Servo_Start();

    HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 1);
    HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 1);
    HAL_GPIO_WritePin(CylinderX_GPIO_Port, CylinderX_Pin, 1);
    osDelay(1000);

    for (;;) {

       

        osDelay(50);
    }
}
