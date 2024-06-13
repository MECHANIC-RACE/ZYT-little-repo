/*
 * @Author: ZYT
 * @Date: 2024-05-19 14:41:19
 * @LastEditors: ZYT
 * @LastEditTime: 2024-06-14 00:03:10
 * @FilePath: \Gantry_Trial\UserCode\Upper\Upper_Start\UpperStart.c
 * @Brief: 
 * 
 * Copyright (c) 2024 by zyt, All Rights Reserved. 
 */

#include "UpperStart.h"

void StartDefaultTask(void *argument)
{
    /*等待树莓派数据*/
    //RaspReceive_Enable();
    /*串口使能*/
    Usart_start();

    Uart_State = 2; //后续要删！
    
    __HAL_UART_ENABLE(&huart5);
    UsartUpdate_Start();
    while(Uart_State!=2)
    {
        ;       //若未收到上位机数据则一直循环
    }

    /*初始化函数*/
    Core_xy_Motor_init();
    /*开启线程*/
    //Area1_StateMachine_Start();
    //Area2_StateMachine_Start();
    //Area3_StateMachine_Start();
    Upper_Servo_Start();
   
    
    
    for (;;) {

        // printf("%f,%f,%f,%f,%f\n",
        //        Lidar1.distance_aver,
        //        Core_xy[2].Motor_X->speedPID.output,
        //        current_pos03[0],
        //        Core_xy[2].Motor_X->posPID.output,
        //        Core_xy[2].Motor_X->FdbData.rpm);
        // printf("%f,%f,%f,%f\n",
        //        Lidar4.distance_aver,
        //        current_pos03[0],
        //        Lidar4.distance_aver,
        //        current_pos03[1] );
        printf("%f,%f,%f\n",Lidar3.distance_aver,Lidar6.distance_aver,Lidar1.distance_aver);

        osDelay(50);
    }
}
