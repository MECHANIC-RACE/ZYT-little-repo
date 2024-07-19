/*
 * @Author: ZYT
 * @Date: 2024-07-19 12:41:24
 * @LastEditors: ZYT
 * @LastEditTime: 2024-07-19 23:55:49
 * @FilePath: \Gantry_Trial\UserCode\Upper\Protect\lidar_sp.c
 * @Brief: 
 * 
 * Copyright (c) 2024 by zyt, All Rights Reserved. 
 */
#include "Lidar_sp.h"
float lidar_bad_flag2;
float lidar_bad_flag6;

void Lidar_sp_Task(void *argument)
{
    float lidar_tmp2[6]={0};
    float lidar_tmp6[6]={0};
    

    for (;;) {
        lidar_tmp2[0] = Lidar2.distance_aver;
        lidar_tmp6[0] = Lidar6.distance_aver;
        osDelay(100);
        lidar_tmp2[1] = Lidar2.distance_aver;
        lidar_tmp6[1] = Lidar6.distance_aver;
        osDelay(100);
        lidar_tmp2[2] = Lidar2.distance_aver;
        lidar_tmp6[2] = Lidar6.distance_aver;
        osDelay(100);
        lidar_tmp2[3] = Lidar2.distance_aver;
        lidar_tmp6[3] = Lidar6.distance_aver;
        osDelay(100);
        lidar_tmp2[4] = Lidar2.distance_aver;
        lidar_tmp6[4] = Lidar6.distance_aver;
        osDelay(100);
        lidar_tmp2[5] = Lidar2.distance_aver;
        lidar_tmp6[5] = Lidar6.distance_aver;
        osDelay(100);

        if (lidar_tmp2[0] == lidar_tmp2[1] && lidar_tmp2[2] == lidar_tmp2[1] && lidar_tmp2[2] == lidar_tmp2[3]) 
            {lidar_bad_flag2 = 1;}
        if ((lidar_tmp6[0] == lidar_tmp6[1]) && (lidar_tmp6[1] == lidar_tmp6[2]) && lidar_tmp6[2] == lidar_tmp6[3] && lidar_tmp6[3] == lidar_tmp6[4] && lidar_tmp6[4] == lidar_tmp6[5]) { lidar_bad_flag6 = 1; }

        //osDelay(10);
    }
}

void Lidar_sp_Start(void)
{
    osThreadId_t Lidar_spHandle;
    const osThreadAttr_t Lidar_sp_attributes = {
        .name       = "Lidar_sp",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };

    Lidar_spHandle = osThreadNew(Lidar_sp_Task, NULL, &Lidar_sp_attributes);
}
