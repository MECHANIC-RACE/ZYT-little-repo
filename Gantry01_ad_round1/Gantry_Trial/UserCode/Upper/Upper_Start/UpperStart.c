/*
 * @Author: ZYT
 * @Date: 2024-05-19 14:41:19
 * @LastEditors: ZYT
 * @LastEditTime: 2024-07-25 21:38:00
 * @FilePath: \Gantry_Trial\UserCode\Upper\Upper_Start\UpperStart.c
 * @Brief: 
 * 
 * Copyright (c) 2024 by zyt, All Rights Reserved. 
 */

#include "UpperStart.h"

void StartDefaultTask(void *argument)
{
    /*等待树莓派数据*/
    //Uart_State = 1;
    RaspReceive_Enable();
    /*串口使能*/
    Usart_start();
    UsartUpdate_Start();
    // HAL_GPIO_WritePin(Cylinder02_GPIO_Port, Cylinder02_Pin, 1);
    // HAL_GPIO_WritePin(Cylinder01_GPIO_Port, Cylinder01_Pin, 1);
     
     /*初始化函数*/
     Core_xy_Motor_init();
     /*开启线程*/
     Reset_Start();
    //  Area1_StateMachine_Start();
    //  Area2_StateMachine_Start();
    //  Area3_StateMachine_Start();
     Upper_Servo_Start();
     // HAL_GPIO_WritePin(Cylinder01_GPIO_Port, Cylinder01_Pin, 1);

     for (;;) {

         // printf("%f,%f,%f,%f,%f\n",
         //        Core_xy[2].Motor_X->AxisData.AxisAngle_inDegree,
         //        current_pos03[0],
         //        Core_xy[2].Motor_X->speedPID.output,
         //        Core_xy[2].Motor_X->posPID.output,
         //        Core_xy[2].Motor_X->FdbData.rpm);
         printf("%f,%f,%f,%f,%f,%f\n",
                Core_xy[1].Motor_X->AxisData.AxisAngle_inDegree,
                Core_xy[1].Motor_X->posPID.output,
                Core_xy[1].Motor_X->FdbData.rpm,
                Core_xy[1].Motor_X->speedPID.output,
                Lidar2.distance_aver,
                Lidar6.distance_aver);
        //  printf("%f,%f,%d,%d,%d\n", weight_placement[0], weight_placement[1],Uart_State,receive_buffer[0],receive_buffer[4]);
         osDelay(50);
    }
}
