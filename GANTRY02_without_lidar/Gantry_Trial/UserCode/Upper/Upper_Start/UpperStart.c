/*
 * @Author: ZYT
 * @Date: 2024-05-19 14:41:19
 * @LastEditors: ZYT
 * @LastEditTime: 2024-06-15 00:41:47
 * @FilePath: \Gantry_Trial\UserCode\Upper\Upper_Start\UpperStart.c
 * @Brief: 
 * 
 * Copyright (c) 2024 by zyt, All Rights Reserved. 
 */

#include "UpperStart.h"

void StartDefaultTask(void *argument)
{
    /*等待树莓派数据*/
    Uart_State = 2;
    RaspReceive_Enable();
    /*串口使能*/
    Usart_start();
    UsartUpdate_Start();
    while(Uart_State!=2)
    {
        ;       //若未收到上位机数据则一直循环
    }

    /*初始化函数*/
    Core_xy_Motor_init();
    /*开启线程*/
     Area1_StateMachine_Start();
    // Area2_StateMachine_Start();
    // Area3_StateMachine_Start();
    Upper_Servo_Start();
    //HAL_GPIO_WritePin(Cylinder01_GPIO_Port, Cylinder01_Pin, 1);
    

    for (;;) {

        // printf("%f,%f,%f,%f,%f\n",
        //        Core_xy[2].Motor_X->AxisData.AxisAngle_inDegree,
        //        current_pos03[0],
        //        Core_xy[2].Motor_X->speedPID.output,
        //        Core_xy[2].Motor_X->posPID.output,
        //        Core_xy[2].Motor_X->FdbData.rpm);
        printf("%f,%f,%f,%f,%f\n",
               Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree,
               current_pos01[0],
               Core_xy[0].Motor_Y->AxisData.AxisAngle_inDegree,
               current_pos01[1],
               Lidar1.distance
               );

        osDelay(50);
    }
}
