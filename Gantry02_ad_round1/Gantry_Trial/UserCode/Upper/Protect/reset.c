/*
 * @Author: ZYT
 * @Date: 2024-07-19 12:40:45
 * @LastEditors: ZYT
 * @LastEditTime: 2024-07-19 14:12:04
 * @FilePath: \Gantry_Trial\UserCode\Upper\Protect\reset.c
 * @Brief: 
 * 
 * Copyright (c) 2024 by zyt, All Rights Reserved. 
 */
#include "reset.h"

void Reset_Task(void *argument)
{
    uint16_t flag = 0;
    while(1){
        uint16_t ResetFlag = HAL_GPIO_ReadPin(Reset_GPIO_Port, Reset_Pin);
        if (ResetFlag)
        {
            // float degree1x = Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree;
            // float degree2x = Core_xy[1].Motor_Y->AxisData.AxisAngle_inDegree;
            // float degree1y = Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree;
            // float degree2y = Core_xy[1].Motor_Y->AxisData.AxisAngle_inDegree;
            if(flag==0){
                osThreadSuspend(Area1_StateHandle);
                osThreadSuspend(Area2_StateHandle);
                pid_reset(&(Core_xy[0].Motor_X->speedPID), 5, 0.4, 0.8);
                pid_reset(&(Core_xy[0].Motor_Y->speedPID), 3.5, 0.3, 0.3);
                pid_reset(&(Core_xy[1].Motor_X->speedPID), 5, 0.4, 0.8);
                pid_reset(&(Core_xy[1].Motor_Y->speedPID), 3.5, 0.3, 0.3);
                pid_reset(&(Core_xy[2].Motor_X->speedPID), 5, 0.4, 0.8);

                pid_reset(&(Core_xy[0].Motor_X->posPID), 200, 0, 0);
                pid_reset(&(Core_xy[0].Motor_Y->posPID), 200, 0, 0);
                pid_reset(&(Core_xy[1].Motor_X->posPID), 200, 0, 0);
                pid_reset(&(Core_xy[1].Motor_Y->posPID), 200, 0, 0);
                pid_reset(&(Core_xy[2].Motor_X->posPID), 200, 0, 0);
                flag = 1;
            }

            HAL_GPIO_WritePin(Cylinder01_GPIO_Port, Cylinder01_Pin, 1);
            HAL_GPIO_WritePin(Cylinder02_GPIO_Port, Cylinder02_Pin, 1);
            HAL_GPIO_WritePin(Electromagnet01_GPIO_Port, Electromagnet01_Pin, 0);
            HAL_GPIO_WritePin(Electromagnet02_GPIO_Port, Electromagnet02_Pin, 0);
            UseLidar01 = 0;
            UseLidar02 = 0;
            current_pos01[0] = 0;
            current_pos02[0] = 0;
            current_pos01[1] = 0;
            current_pos02[1] = 0;
        }
    }
}

void Reset_Start(void)
{
    osThreadId_t ResetHandle;
    const osThreadAttr_t Reset_attributes = {
        .name       = "Reset",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };

    ResetHandle = osThreadNew(Reset_Task, NULL, &Reset_attributes);
}