/*
 * @Author: ZYT
 * @Date: 2024-05-19 14:41:19
 * @LastEditors: ZYT
 * @LastEditTime: 2024-07-20 00:20:48
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
    // while(Uart_State!=2)
    // {
    //     ;       //若未收到上位机数据则一直循环
    // }
    //osDelay(3000);
    // HAL_GPIO_WritePin(Cylinder02_GPIO_Port, Cylinder02_Pin, 1);
    // HAL_GPIO_WritePin(Cylinder01_GPIO_Port, Cylinder01_Pin, 1);
    
    /*初始化函数*/
    Core_xy_Motor_init();
    /*开启线程*/
    Reset_Start();
    Lidar_sp_Start();

    Area1_StateMachine_Start();
    Area2_StateMachine_Start();
    Upper_Servo_Start();
    //HAL_GPIO_WritePin(Cylinder01_GPIO_Port, Cylinder01_Pin, 1);
    

    for (;;) {

        // printf("%f,%f,%f,%f,%f\n",
        //        Core_xy[2].Motor_X->AxisData.AxisAngle_inDegree,
        //        current_pos03[0],
        //        Core_xy[2].Motor_X->speedPID.output,
        //        Core_xy[2].Motor_X->posPID.output,
        //        Core_xy[2].Motor_X->FdbData.rpm);
        // printf("%f,%f,%f,%f,%f,%f\n",
        //        current_pos02[0],
        //        current_pos01[0],
        //        Core_xy[1].Motor_X->AxisData.AxisAngle_inDegree,
        //        Core_xy[1].Motor_X->AxisData.AxisVelocity,
        //        Core_xy[1].Motor_X->posPID.output,
        //        Lidar6.distance_aver
        //        );
        //printf("%f,%f,%d\n", weight_placement[0], weight_placement[1], Uart_State);
        printf("%f,%f,%f,%f\n", Lidar2.distance_aver, Lidar6.distance_aver, lidar_bad_flag2,lidar_bad_flag6);

        osDelay(50);
    }
}
