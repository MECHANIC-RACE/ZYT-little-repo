/*
 * @Author: ZYT
 * @Date: 2024-07-20 21:33:49
 * @LastEditors: ZYT
 * @LastEditTime: 2024-07-29 00:08:52
 * @FilePath: \Gantry_Trial\UserCode\Upper\Upper_StateMachine\Light_fury_state.c
 * @Brief:
 *
 * Copyright (c) 2024 by zyt, All Rights Reserved.
 */
#include "Light_fury_state.h"

#define X_maxvelocity   4000
#define Y_maxvelocity   300
#define X_Acceleration  500
#define Y_Acceleration  60

float initial_posX;
float initial_posYL;
float initial_posYR;
float current_posX;
float current_posYL;
float current_posYR;
uint16_t tickflag;
uint16_t tickflag_x;
uint16_t starttick;

float remem_12xR;
float remem_12xL;

void StateMachine_Task(void *argument)
{
    osDelay(100);
    uint16_t stateflag = 0;
    /*距离变量*/
    float mnq     = 2010; // 中间 内圈 滑行前
    float mnh     = 2370; // 中间 内圈 滑行后
    float mwq     = 1600; // 中间 外圈 滑行前
    float mwh     = 2050; // 中间 外圈 滑行后
    float ywq     = 590;  // 两边区y 外圈 滑行前
    float ywh     = 410;  // 两边区y 外圈 滑行后
    float ynq     = 400;    // 两边区y 内圈 滑行前
    float ynh     = 81;     // 两边区y 内圈 滑行后
    float wx_12   = 3064;   // 12区外圈x
    float nx_12   = 2887;   // 12区内圈x
    float stake_x12 = 3700;   // 12区木桩x
    float wx_34     = 2340;   // 34区外圈x
    float nx_34     = 2520;   // 34区内圈x
    float stake_x34 = 188;    // 34区木桩x
    float stake_y   = 565;    // 木桩y
    float weight_m_x = 2563;//中间木桩x轴
    float wait_ticks    = 2.0;
    float wait_ticks_x  = 1.0;
    uint16_t cur2       = 0;
    weight_placement[0] = 1;
    weight_placement[1] = 0;
    weight_placement[2] = 0;
    weight_placement[3] = 1;
    weight_placement[4] = 1;
    for (;;) {

        if(stateflag==0)//抓取中间的砝码
        {
            LightFury.gantry_t.position.yL = 610;
            LightFury.gantry_t.position.yR = 610;
            if(weight_placement[4]==1){     //neiquan
                LightFury.gantry_t.position.x = mnq;
                osDelay(1200);
            }else{
                LightFury.gantry_t.position.x = mwq;
                osDelay(800);
            }
            
                stateflag = 1;
            
        }else if(stateflag==1){     //往前拖行一段
            HAL_GPIO_WritePin(CylinderX_GPIO_Port, CylinderX_Pin, 0);
            HAL_GPIO_WritePin(ElectromagnetX_GPIO_Port, ElectromagnetX_Pin, 1);
            osDelay(500);
            if (weight_placement[4] == 1) { // neiquan
                LightFury.gantry_t.position.x = mnh;
            } else {
                LightFury.gantry_t.position.x = mwh;
            }
            float diff[1] = {0};
            diff[0]       = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
            //if ((diff[0] < 2)) {
                osDelay(1200);
                stateflag = 2;
            //}
        } else if (stateflag == 2) // 夹取短区的砝码
        {
            pid_reset(&(LightFury.Motor_YL->speedPID), 7, 0.5, 0.3);
            pid_reset(&(LightFury.Motor_YR->speedPID), 7, 0.5, 0.3);
            HAL_GPIO_WritePin(CylinderX_GPIO_Port, CylinderX_Pin, 1);
            osDelay(500);
            
            if(weight_placement[0]==1&&weight_placement[1]==1)      //都在内圈
            {
                LightFury.gantry_t.position.x = nx_12;
                LightFury.gantry_t.position.yL = ynq;
                LightFury.gantry_t.position.yR = ynq;
                float diff[3]                 = {0};

                diff[0] = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
                diff[1] = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
                diff[2] = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
                if (diff[0] < 50) 
                {
                    if (tickflag_x == 0) {
                        starttick  = xTaskGetTickCount();
                        tickflag_x = 1;
                    }
                    cur2 = xTaskGetTickCount() - starttick;
                    pid_reset(&(LightFury.Motor_X->speedPID), 20, 1.8, 0.8);
                }
                
                if ((diff[0] < 4 && diff[1] < 2 && diff[2] < 2)||cur2>wait_ticks_x*1000)
                {
                    osDelay(500);
                    tickflag_x = 0;
                    cur2       = 0;
                    stateflag  = 3;
                }
            }
            if (weight_placement[0] == 1 && weight_placement[1] == 0) // 1区在内 2区在外------------->先抓内圈的1区 即right
            {
            
                LightFury.gantry_t.position.x  = nx_12;
                LightFury.gantry_t.position.yR = ynq;
                float diff[2]                  = {0};
                diff[0] = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
                diff[1] = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
                if (diff[0] < 50) {
                    if (tickflag_x == 0) {
                        starttick  = xTaskGetTickCount();
                        tickflag_x = 1;
                    }
                    cur2 = xTaskGetTickCount() - starttick;
                    pid_reset(&(LightFury.Motor_X->speedPID), 20, 1.8, 0.8);
                }

                if ((diff[0] < 4 && diff[1] < 2) || cur2 > wait_ticks_x * 1000) {
                    osDelay(500);
                    tickflag_x = 0;
                    cur2       = 0;
                    stateflag  = 3;
                }
                
            }
            if (weight_placement[0] == 0 && weight_placement[1] == 1) // 1区在外 2区在内------------->先抓内圈的2区 即left
            {
                
                LightFury.gantry_t.position.x  = nx_12;
                LightFury.gantry_t.position.yL = ynq;
                float diff[2]                  = {0};

                diff[0] = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
                diff[1] = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
                if (diff[0] < 50) {
                    if (tickflag_x == 0) {
                        starttick  = xTaskGetTickCount();
                        tickflag_x = 1;
                    }
                    cur2 = xTaskGetTickCount() - starttick;
                    pid_reset(&(LightFury.Motor_X->speedPID), 20, 1.8, 0.8);
                }

                if ((diff[0] < 4 && diff[1] < 2) || cur2 > wait_ticks_x * 1000) {
                    osDelay(500);
                    tickflag_x = 0;
                    cur2       = 0;
                    stateflag  = 3;
                }
                
            }
            if (weight_placement[0] == 0 && weight_placement[1] == 0) // 都在外圈
            {
                
                LightFury.gantry_t.position.x  = wx_12;
                LightFury.gantry_t.position.yL = ywq;
                LightFury.gantry_t.position.yR = ywq;
                float diff[3]                  = {0};
                diff[0] = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
                if (diff[0] < 50) {
                    if (tickflag_x == 0) {
                        starttick  = xTaskGetTickCount();
                        tickflag_x = 1;
                    }
                    cur2 = xTaskGetTickCount() - starttick;
                    pid_reset(&(LightFury.Motor_X->speedPID), 20, 1.8, 0.8);
                }

                if ((diff[0] < 4 ) || cur2 > wait_ticks_x * 1000) {
                    osDelay(500);
                    tickflag_x = 0;
                    cur2       = 0;
                    stateflag  = 3;
                }
            }

        } else if (stateflag == 3) // 往前滑行一段吸上砝码
        {
            if(tickflag==0){
               starttick = xTaskGetTickCount();
                tickflag                    = 1;
            }
            if (weight_placement[0] == 1 && weight_placement[1] == 1) // 都在内圈 ----一起往中间走
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_GPIO_Port, ElectromagnetYL_Pin, 1);
                HAL_GPIO_WritePin(ElectromagnetYR_GPIO_Port, ElectromagnetYR_Pin, 1);
                osDelay(500);
                LightFury.gantry_t.position.yL = ynh;
                LightFury.gantry_t.position.yR = ynh;
                float diff[2]                  = {0};

                diff[0] = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
                diff[1] = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
                uint16_t cur1 = xTaskGetTickCount() - starttick;
                if ((diff[0] < 2 && diff[1] < 2)||cur1>wait_ticks*1000) {
                    osDelay(500);
                    stateflag = 4;
                }
            }
            if (weight_placement[0] == 1 && weight_placement[1] == 0) // 1区在内 2区在外------------->先抓内圈的1区 即right
            {
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYR_GPIO_Port, ElectromagnetYR_Pin, 1);
                osDelay(500);
                LightFury.gantry_t.position.yR = ynh;
                float diff[1]                  = {0};
                diff[0] = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
                uint16_t cur1                  = xTaskGetTickCount() - starttick;
                if ((diff[0] < 2) || cur1 > wait_ticks * 1000) {
                    osDelay(500);
                    stateflag = 4;
                }
            }
            if (weight_placement[0] == 0 && weight_placement[1] == 1) // 1区在外 2区在内------------->先抓内圈的2区 即left
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_GPIO_Port, ElectromagnetYL_Pin, 1);
                osDelay(500);
                LightFury.gantry_t.position.yL = ynh;
                float diff[1]        = {0};
                diff[0]                        = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
                uint16_t cur1                  = xTaskGetTickCount() - starttick;
                if ((diff[0] < 2) || cur1 > wait_ticks * 1000) {
                    osDelay(500);
                    stateflag = 4;
                }
            }
            if (weight_placement[0] == 0 && weight_placement[1] == 0) // 都在外圈
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_GPIO_Port, ElectromagnetYL_Pin, 1);
                HAL_GPIO_WritePin(ElectromagnetYR_GPIO_Port, ElectromagnetYR_Pin, 1);
                osDelay(500);

                LightFury.gantry_t.position.yL = ywh;
                LightFury.gantry_t.position.yR = ywh;
                float diff[2]                  = {0};
                diff[0]                        = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
                diff[1]                        = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
                uint16_t cur1                  = xTaskGetTickCount() - starttick;
                if ((diff[0] < 2 && diff[1] < 2) || cur1 > wait_ticks * 1000) {
                    osDelay(500);
                    stateflag = 4;
                }
            }
        } else if (stateflag == 4) {
            HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 1);
            HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 1);
            osDelay(500);                                             // 一内一外的情况
            
            if (weight_placement[0] == 1 && weight_placement[1] == 0) // 1区在内 2区在外------------->后抓外圈的2区 即left
            {
                LightFury.gantry_t.position.x  = wx_12;
                LightFury.gantry_t.position.yL = ywq;
             
                float diff[2]                  = {0};
                diff[0] = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
                if (diff[0] < 50) {
                    if (tickflag_x == 0) {
                        starttick  = xTaskGetTickCount();
                        tickflag_x = 1;
                    }
                    cur2 = xTaskGetTickCount() - starttick;
                    pid_reset(&(LightFury.Motor_X->speedPID), 20, 1.8, 0.8);
                }

                if ((diff[0] < 4 && diff[1] < 2) || cur2 > wait_ticks_x * 1000) {
                    osDelay(500);
                    tickflag_x = 0;
                    cur2       = 0;
                    stateflag  = 5;
                }
            }
            else if (weight_placement[0] == 0 && weight_placement[1] == 1) // 1区在外 2区在内------------->后抓wai圈的1区 即right
            {
                LightFury.gantry_t.position.x  = wx_12;
                LightFury.gantry_t.position.yR = ywq;
                float diff[2]        = {0};
                diff[0]                        = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
                if (diff[0] < 50) {
                    if (tickflag_x == 0) {
                        starttick  = xTaskGetTickCount();
                        tickflag_x = 1;
                    }
                    cur2 = xTaskGetTickCount() - starttick;
                    pid_reset(&(LightFury.Motor_X->speedPID), 20, 1.8, 0.8);
                }

                if ((diff[0] < 4 && diff[1] < 2 && diff[2] < 2) || cur2 > wait_ticks_x * 1000) {
                    osDelay(500);
                    tickflag_x = 0;
                    cur2       = 0;
                    stateflag  = 5;
                }
            }else{
            stateflag = 5;
            }
        } else if (stateflag == 5) {
            if (tickflag == 1) {
                starttick = xTaskGetTickCount();
                tickflag  = 2;
            }
            if (weight_placement[0] == 1 && weight_placement[1] == 0) // 1区在内 2区在外------------->后抓2区 left
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_GPIO_Port, ElectromagnetYL_Pin, 1);
                osDelay(500);
                LightFury.gantry_t.position.yL = ywh;
                float diff[1]                  = {0};
                diff[0] = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
                uint16_t cur1                  = xTaskGetTickCount() - starttick;
                if ((diff[0] < 2) || cur1 > wait_ticks * 1000) {
                    osDelay(500);
                    stateflag = 6;
                }
            }
            else if (weight_placement[0] == 0 && weight_placement[1] == 1) // 1区在外 2区在内------------->后抓外圈的1区 即right
            {
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYR_GPIO_Port, ElectromagnetYR_Pin, 1);
                osDelay(500);

                LightFury.gantry_t.position.yR = ywh;

                float diff[1]        = {0};
                diff[0] = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
                uint16_t cur1        = xTaskGetTickCount() - starttick;
                if ((diff[0] < 2 ) || cur1 > wait_ticks * 1000) {
                    osDelay(500);
                    stateflag = 6;
                }
            }
            else{
                stateflag = 6;
            }
        } else if (stateflag == 6) {
            HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 1);
            HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 1);
            osDelay(500);
            pid_reset(&(LightFury.Motor_YL->speedPID),3, 0.3, 0.3);
            pid_reset(&(LightFury.Motor_YR->speedPID),3, 0.3, 0.3);
            //先让左右的爪子退到两边

            LightFury.gantry_t.position.yL = 612;
            LightFury.gantry_t.position.yR = 612;
            osDelay(300);

            // float diff[2]                  = {0};
            // diff[0] = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
            // diff[1] = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
            //if ((diff[0] < 2 && diff[1] < 2 )) {
            stateflag = 7;
             //}
        } else if (stateflag == 7) { // 前往短区木桩
            LightFury.gantry_t.position.x = stake_x12;
            LightFury.gantry_t.position.yL = stake_y;
            LightFury.gantry_t.position.yR = stake_y-2;
            float diff[3]        = {0};
            diff[0]                        = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
            diff[0]                        = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
            diff[1]                        = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
            if ((diff[0] < 2&&diff[1]<2&&diff[2]<2)) {
                osDelay(800);
                stateflag = 8;
            }
        } else if (stateflag == 8) {/*test*/
            osDelay(1700);
            HAL_GPIO_WritePin(ElectromagnetYL_GPIO_Port, ElectromagnetYL_Pin, 0);
            HAL_GPIO_WritePin(ElectromagnetYR_GPIO_Port, ElectromagnetYR_Pin, 0);
            osDelay(100);
            pid_reset(&(LightFury.Motor_YL->speedPID), 3, 0.3, 0.3);
            pid_reset(&(LightFury.Motor_YR->speedPID), 3, 0.3, 0.3);
            LightFury.gantry_t.position.yL = stake_y+1;
            LightFury.gantry_t.position.yR = stake_y+1;
            osDelay(2000);
            LightFury.gantry_t.position.yL = 611;
            LightFury.gantry_t.position.yR = 611;
            float diff01[2]                = {0};
            diff01[0]                      = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
            diff01[1]                      = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
            //if (diff01[0] < 3 && diff01[1] < 4 ) {
                osDelay(1000);
                stateflag = 9;
        } else if (stateflag == 9) {
            pid_reset(&(LightFury.Motor_YL->speedPID), 5, 0.4, 0.3);
            pid_reset(&(LightFury.Motor_YR->speedPID), 5, 0.4, 0.3);
            LightFury.gantry_t.position.x = weight_m_x;       //放下中间的砝码
            float diff[1]                 = {0};
    
            diff[0] = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
            if ((diff[0] < 2)) {
                osDelay(500);
                stateflag = 10;
             }

        } else if (stateflag == 10) // 夹取长区的砝码
        {
            HAL_GPIO_WritePin(ElectromagnetX_GPIO_Port, ElectromagnetX_Pin, 0);
            osDelay(500);
            pid_reset(&(LightFury.Motor_YL->speedPID), 7, 0.5, 0.3);
            pid_reset(&(LightFury.Motor_YR->speedPID), 7, 0.5, 0.3);
            if (weight_placement[2] == 1 && weight_placement[3] == 1) // 都在内圈
            {
                LightFury.gantry_t.position.x  = nx_34;
                LightFury.gantry_t.position.yL = ynq;
                LightFury.gantry_t.position.yR = ynq;
                float diff[3]                  = {0};
                diff[0]                        = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
                diff[1]                        = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
                diff[2]                        = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
                if (diff[0] < 50) {
                    if (tickflag_x == 0) {
                        starttick  = xTaskGetTickCount();
                        tickflag_x = 1;
                    }
                    cur2 = xTaskGetTickCount() - starttick;
                    pid_reset(&(LightFury.Motor_X->speedPID), 20, 1.8, 0.8);
                }

                if ((diff[0] < 4 && diff[1] < 2 && diff[2] < 2) || cur2 > wait_ticks_x * 1000) {
                    osDelay(500);
                    cur2       = 0;
                    stateflag  = 11;
                    tickflag_x = 0;
                }
                 
            }
            if (weight_placement[3] == 1 && weight_placement[2] == 0) // 4区在内 3区在外------------->先抓内圈的4区 即right
            {
                LightFury.gantry_t.position.x = nx_34;
                LightFury.gantry_t.position.yR = ynq;
                float diff[2]                  = {0};

                diff[0] = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
                diff[1] = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
                if (diff[0] < 50) {
                    if (tickflag_x == 0) {
                        starttick  = xTaskGetTickCount();
                        tickflag_x = 1;
                    }
                    cur2 = xTaskGetTickCount() - starttick;
                    pid_reset(&(LightFury.Motor_X->speedPID), 20, 1.8, 0.8);
                }
                if ((diff[0] < 4 && diff[1] < 2 && diff[2] < 2) || cur2 > wait_ticks_x * 1000) {
                    osDelay(500);
                    stateflag  = 11;
                    cur2       = 0;
                    tickflag_x = 0;
                }
            }
            if (weight_placement[3] == 0 && weight_placement[2] == 1) // 4区在外 3区在内------------->先抓内圈的3区 即left
            {
                LightFury.gantry_t.position.x  = nx_34;
                LightFury.gantry_t.position.yL = ynq;
                float diff[2]        = {0};

                diff[0] = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
                diff[1] = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
                if (diff[0] < 50) {
                    if (tickflag_x == 0) {
                        starttick  = xTaskGetTickCount();
                        tickflag_x = 1;
                    }
                    cur2 = xTaskGetTickCount() - starttick;
                    pid_reset(&(LightFury.Motor_X->speedPID), 20, 1.8, 0.8);
                }

                if ((diff[0] < 4 && diff[1] < 2 && diff[2] < 2) || cur2 > wait_ticks_x * 1000) {
                    osDelay(500);
                    stateflag  = 11;
                    cur2       = 0;
                    tickflag_x = 0;
                }
            }
            if (weight_placement[2] == 0 && weight_placement[3] == 0) // 都在外圈
            {
                LightFury.gantry_t.position.x  = wx_34;
                LightFury.gantry_t.position.yL = ywq;
                LightFury.gantry_t.position.yR = ywq;
                float diff[3]                  = {0};
                diff[0]                        = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
                if (diff[0] < 50) {
                    if (tickflag_x == 0) {
                        starttick  = xTaskGetTickCount();
                        tickflag_x = 1;
                    }
                    cur2 = xTaskGetTickCount() - starttick;
                    pid_reset(&(LightFury.Motor_X->speedPID), 20, 1.8, 0.8);
                }

                if ((diff[0] < 4 ) || cur2 > wait_ticks_x * 1000) {
                    osDelay(500);
                    stateflag  = 11;
                    cur2       = 0;
                    tickflag_x = 0;
                }
            }
        } else if (stateflag == 11) // 往前滑行一段吸上砝码
        {
            if (tickflag == 2) {
                starttick = xTaskGetTickCount();
                tickflag  = 3;
            }
            if (weight_placement[2] == 1 && weight_placement[3] == 1) // 都在内圈 ----一起往中间走
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_GPIO_Port, ElectromagnetYL_Pin, 1);
                HAL_GPIO_WritePin(ElectromagnetYR_GPIO_Port, ElectromagnetYR_Pin, 1);
                osDelay(500);
                LightFury.gantry_t.position.yL = ynh;
                LightFury.gantry_t.position.yR = ynh;
                float diff[2]                  = {0};
                diff[0]                        = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
                diff[1]                        = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
                uint16_t cur1                  = xTaskGetTickCount() - starttick;
                if ((diff[0] < 2 && diff[1] < 2) || cur1 > wait_ticks * 1000) {
                    
                        osDelay(500);
                        stateflag = 12;
                    }
                }
            if (weight_placement[3] == 1 && weight_placement[2] == 0) // 4区在内 3区在外------------->先抓内圈的4区 即right
            {
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYR_GPIO_Port, ElectromagnetYR_Pin, 1);
                osDelay(500);

                LightFury.gantry_t.position.yR = ynh;
                float diff[1]                  = {0};
                diff[0]                        = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
                uint16_t cur1                  = xTaskGetTickCount() - starttick;
                if ((diff[0] < 2) || cur1 > wait_ticks * 1000) {
                    osDelay(500);
                    stateflag = 12;
                }
            }
            if (weight_placement[3] == 0 && weight_placement[2] == 1) // 4区在外 3区在内------------->先抓内圈的2区 即left
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_GPIO_Port, ElectromagnetYL_Pin, 1);
                osDelay(500);

                LightFury.gantry_t.position.yL = ynh;
                float diff[1]        = {0};
                diff[0]                        = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
                uint16_t cur1                  = xTaskGetTickCount() - starttick;
                if ((diff[0] < 2) || cur1 > wait_ticks * 1000) {
                    osDelay(500);
                    stateflag = 12;
                }
            }
            if (weight_placement[2] == 0 && weight_placement[3] == 0) // 都在外圈
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_GPIO_Port, ElectromagnetYL_Pin, 1);
                HAL_GPIO_WritePin(ElectromagnetYR_GPIO_Port, ElectromagnetYR_Pin, 1);
                osDelay(500);

                LightFury.gantry_t.position.yL = ywh;
                LightFury.gantry_t.position.yR = ywh;
                
                float diff[2]                  = {0};

                diff[0] = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
                diff[1] = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
                uint16_t cur1 = xTaskGetTickCount() - starttick;
                if ((diff[0] < 2 && diff[1] < 2) || cur1 > wait_ticks * 1000) {
                    osDelay(500);
                    stateflag = 12;
                }
            }
        } else if (stateflag == 12) { // 一内一外的情况
            HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 1);
            HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 1);
            osDelay(500);
            if ((weight_placement[3] == 1 && weight_placement[2] == 0) || (weight_placement[3] == 0 && weight_placement[2] == 1)) {
                LightFury.gantry_t.position.x  = wx_34;
                LightFury.gantry_t.position.yL = ywq;
                LightFury.gantry_t.position.yR = ywq;
                float diff[1]        = {0};
                diff[0]                       = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
                if (diff[0] < 50) {
                    if (tickflag_x == 0) {
                        starttick  = xTaskGetTickCount();
                        tickflag_x = 1;
                    }
                    cur2 = xTaskGetTickCount() - starttick;
                    pid_reset(&(LightFury.Motor_X->speedPID), 20, 1.8, 0.8);
                }

                if ((diff[0] < 4 ) || cur2 > wait_ticks_x * 1000) {
                    osDelay(500);
                    cur2       = 0;
                    stateflag  = 13;
                    tickflag_x = 0;
                }
            }else{
                stateflag = 13;
            }
        } else if (stateflag == 13) {
            if (tickflag == 3) {
                starttick = xTaskGetTickCount();
                tickflag  = 4;
            }
            if (weight_placement[3] == 1 && weight_placement[2] == 0) // 4区在内 3区在外------------->后抓3区 left
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_GPIO_Port, ElectromagnetYL_Pin, 1);
                osDelay(500);

                LightFury.gantry_t.position.yL = ywh;

                float diff[1]                  = {0};
                diff[0]                        = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
                uint16_t cur1                  = xTaskGetTickCount() - starttick;
                if ((diff[0] < 2 ) || cur1 > wait_ticks * 1000) {
                    osDelay(500);
                    stateflag = 14;
                }
            }
            else if (weight_placement[3] == 0 && weight_placement[2] == 1) // 4区在外 3区在内------------->后抓外圈的4区 即right
            {
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYR_GPIO_Port, ElectromagnetYR_Pin, 1);
                osDelay(500);

                LightFury.gantry_t.position.yR = ywh;
                float diff[1]        = {0};
                diff[0]                        = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
                uint16_t cur1                  = xTaskGetTickCount() - starttick;
                if ((diff[0] < 2 ) || cur1 > wait_ticks * 1000) {
                    osDelay(500);
                    stateflag = 14;
                }
            }
            else{
                stateflag = 14;
            }
        } else if (stateflag == 14) {
            HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 1);
            HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 1);
            pid_reset(&(LightFury.Motor_YL->speedPID), 3, 0.3, 0.3);
            pid_reset(&(LightFury.Motor_YR->speedPID), 3, 0.3, 0.3);
            osDelay(500);
            //先让左右的爪子退到两边
            LightFury.gantry_t.position.yL = 612;
            LightFury.gantry_t.position.yR = 612;
            osDelay(100);

            // float diff[2]                  = {0};
            // diff[0] = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
            // diff[1] = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
            //if ((diff[0] < 3 && diff[1] < 3 )) {
                stateflag = 15;
            //}
        } else if (stateflag == 15) { // 前往长区木桩
        
            LightFury.gantry_t.position.x  = stake_x34;
            LightFury.gantry_t.position.yL = stake_y;
            LightFury.gantry_t.position.yR = stake_y-2;
            float diff[3]                  = {0};
            diff[0]                        = fabs(LightFury.gantry_t.position.x - Lidar1.distance_aver);
            diff[0]                        = fabs(LightFury.gantry_t.position.yL - Lidar2.distance_aver);
            diff[1]                        = fabs(LightFury.gantry_t.position.yR - Lidar3.distance_aver);
            if ((diff[0] < 2 && diff[1] < 1 && diff[2] < 1)) {
                osDelay(1500);
                stateflag = 16;
             }
        } else if (stateflag == 16) {
            osDelay(1000);
            HAL_GPIO_WritePin(ElectromagnetYL_GPIO_Port, ElectromagnetYL_Pin, 0);
            HAL_GPIO_WritePin(ElectromagnetYR_GPIO_Port, ElectromagnetYR_Pin, 0);
            osDelay(100);
            LightFury.gantry_t.position.yL = stake_y + 1;
            LightFury.gantry_t.position.yR = stake_y + 1;
            stateflag                      = 17;
        } else if (stateflag == 17) {
            osDelay(2000);
            pid_reset(&(LightFury.Motor_YL->speedPID), 3, 0.3, 0.3);
            pid_reset(&(LightFury.Motor_YR->speedPID), 3, 0.3, 0.3);
            LightFury.gantry_t.position.yL = ywq+5;
            LightFury.gantry_t.position.yR = ywq+5;
        }

        osDelay(2);
    }
}

void StateMachine_Start(void)
{
    osThreadId_t StateMachineHandle;
    const osThreadAttr_t StateMachine_attributes = {
        .name       = "StateMachine",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };
    StateMachineHandle = osThreadNew(StateMachine_Task, NULL, &StateMachine_attributes);
}

