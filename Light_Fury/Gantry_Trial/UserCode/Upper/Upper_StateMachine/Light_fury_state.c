/*
 * @Author: ZYT
 * @Date: 2024-07-20 21:33:49
 * @LastEditors: ZYT
 * @LastEditTime: 2024-07-26 15:36:58
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



void StateMachine_Task(void *argument)
{
    osDelay(100);
    
    /******距离变量******
    float weight5_01 = 0;//5区砝码 滑行前
    float weight5_02 = 0;//5区砝码 滑行后
    if(weight_placement[4]==1)//在内圈
    {
        weight5_01 = 1;
        weight5_02 = 1;
    }else{
        weight5_01 = 1;
        weight5_02 = 1;
    }

    float weight1_01 = 0;
    if(weight_placement[0]==1&&weight_placement[1]) //都在内圈
    {
        weight5_01 = 1;      //取5区砝码  滑行前
    }else if(weight_placement[0]==1&&weight_placement[1]) //1区在内 2区在外
    {
        weight5_01 = 1;      //取5区砝码  滑行前
    }
    else if (weight_placement[0] == 1 && weight_placement[1]) // 都在内圈
    {
        weight5_01 = 1; // 取5区砝码  滑行前
    }
    else if (weight_placement[0] == 1 && weight_placement[1]) // 都在内圈
    {
        weight5_01 = 1; // 取5区砝码  滑行前
    }

    ******距离变量******/

    uint16_t stateflag = 0;
    
    for (;;) {
        // current_posYL = Lidar2.distance_aver;
        // current_posYR = Lidar3.distance_aver;
        pid_reset(&(LightFury.Motor_YL->speedPID),0,0,0);
        pid_reset(&(LightFury.Motor_YR->speedPID),0,0,0);
        LightFury.gantry_t.position.x = 500;
        
        TickType_t StartTick = xTaskGetTickCount();
        initial_posX         = Lidar1.distance_aver; // 电机轴输出角度 单位 度°
        _Bool isArray1       = 0;
        float diff[1]        = {0};
        do {
            TickType_t CurrentTick = xTaskGetTickCount();
            float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
            VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
            diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
            if ((diff[0] < 0.01)) { isArray1 = 1; }

        } while (!isArray1);
        osDelay(200);
        /*
        if(stateflag==0)//抓取中间的砝码
        {
            current_posX  = Lidar1.distance_aver;
            current_posYL = Lidar2.distance_aver;
            current_posYR = Lidar3.distance_aver;
            if(weight_placement[4]==1){     //neiquan
                LightFury.gantry_t.position.x = 1710;
            }else{
                LightFury.gantry_t.position.x = 2115;
            }
            TickType_t StartTick = xTaskGetTickCount();
            initial_posX     = Lidar1.distance_aver; // 电机轴输出角度 单位 度°
            _Bool isArray1       = 0;
            float diff[1]        = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(200);
            HAL_GPIO_WritePin(CylinderX_GPIO_Port, CylinderX_Pin, 0);
            HAL_GPIO_WritePin(ElectromagnetX_GPIO_Port, ElectromagnetX_Pin, 1);
            osDelay(200);
            stateflag = 1;
        }else if(stateflag==1){     //往前拖行一段
            if (weight_placement[4] == 1) { // neiquan
                LightFury.gantry_t.position.x = 1894;
            } else {
                LightFury.gantry_t.position.x = 2290;
            }
            TickType_t StartTick = xTaskGetTickCount();
            initial_posX         = Lidar1.distance_aver; // 电机轴输出角度 单位 度°
            _Bool isArray1       = 0;
            float diff[1]        = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(200);
            HAL_GPIO_WritePin(CylinderX_GPIO_Port, CylinderX_Pin, 1);
            osDelay(200);
            stateflag = 2;
        }else if (stateflag==2)     //夹取短区的砝码
        {
            if(weight_placement[0]==1&&weight_placement[1]==1)      //都在内圈
            {
                LightFury.gantry_t.position.x = 2871;
                LightFury.gantry_t.position.yL = 220;
                LightFury.gantry_t.position.yR = 220;
                TickType_t StartTick          = xTaskGetTickCount();
                initial_posX                  = Lidar1.distance_aver; 
                initial_posYL                  = Lidar2.distance_aver;
                initial_posYR                  = Lidar3.distance_aver;
                _Bool isArray1                 = 0;
                float diff[3]                 = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                    diff[1] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                    diff[2] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01 && diff[1] < 0.01 && diff[2] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[0] == 1 && weight_placement[1] == 0) // 1区在内 2区在外------------->先抓内圈的1区 即right
            {
                LightFury.gantry_t.position.x  = 2871;
                //LightFury.gantry_t.position.yL = 1;
                LightFury.gantry_t.position.yR = 220;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posX                   = Lidar1.distance_aver;
                initial_posYR                  = Lidar3.distance_aver;
                _Bool isArray1                 = 0;
                float diff[2]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    //VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                    diff[1] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01 && diff[1] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[0] == 0 && weight_placement[1] == 1) // 1区在外 2区在内------------->先抓内圈的2区 即left
            {
                LightFury.gantry_t.position.x  = 2871;
                LightFury.gantry_t.position.yL = 220;
                //LightFury.gantry_t.position.yR = 1;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posX                   = Lidar1.distance_aver;
                initial_posYL                  = Lidar2.distance_aver;
                _Bool isArray1                 = 0;
                float diff[2]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    //VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                    diff[1] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                    if ((diff[0] < 0.01 && diff[1] < 0.01)) { isArray1 = 1; }
                

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[0] == 0 && weight_placement[1] == 0) // 都在外圈
            {
                LightFury.gantry_t.position.x  = 3064;
                // LightFury.gantry_t.position.yL = 1;
                // LightFury.gantry_t.position.yR = 1;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posX                   = Lidar1.distance_aver;
                initial_posYL                  = Lidar2.distance_aver;
                initial_posYR                  = Lidar3.distance_aver;
                _Bool isArray1                 = 0;
                float diff[3]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    // VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    // VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                    // diff[1] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                    // diff[2] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01 )) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            stateflag = 3;
        }else if (stateflag==3)         //往前滑行一段吸上砝码
        {
            if (weight_placement[0] == 1 && weight_placement[1] == 1) // 都在内圈 ----一起往中间走
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_Pin, ElectromagnetYL_Pin, 1);
                HAL_GPIO_WritePin(ElectromagnetYR_Pin, ElectromagnetYR_Pin, 1);

                LightFury.gantry_t.position.yL = 81;
                LightFury.gantry_t.position.yR = 81;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posYL                  = Lidar2.distance_aver;
                initial_posYR                  = Lidar3.distance_aver;
                _Bool isArray1                 = 0;
                float diff[2]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                    diff[1] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01 && diff[1] < 0.01 && diff[2] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[0] == 1 && weight_placement[1] == 0) // 1区在内 2区在外------------->先抓内圈的1区 即right
            {
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYR_Pin, ElectromagnetYR_Pin, 1);
                //LightFury.gantry_t.position.x = 1;
                // LightFury.gantry_t.position.yL = 1;
                LightFury.gantry_t.position.yR = 81;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posYR                   = Lidar3.distance_aver; // 电机轴输出角度 单位 度°
                _Bool isArray1                 = 0;
                float diff[1]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    //VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    // VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[0] == 0 && weight_placement[1] == 1) // 1区在外 2区在内------------->先抓内圈的2区 即left
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_Pin, ElectromagnetYL_Pin, 1);
                //LightFury.gantry_t.position.x  = 1;
                LightFury.gantry_t.position.yL = 81;
                // LightFury.gantry_t.position.yR = 1;
                TickType_t StartTick = xTaskGetTickCount();
                initial_posYL         = Lidar2.distance_aver; // 电机轴输出角度 单位 度°
                _Bool isArray1       = 0;
                float diff[1]        = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    //VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    // VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                    if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[0] == 0 && weight_placement[1] == 0) // 都在外圈
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_Pin, ElectromagnetYL_Pin, 1);
                HAL_GPIO_WritePin(ElectromagnetYR_Pin, ElectromagnetYR_Pin, 1);

                LightFury.gantry_t.position.yL = 410;
                LightFury.gantry_t.position.yR = 410;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posYL                  = Lidar2.distance_aver;
                initial_posYR                  = Lidar3.distance_aver;
                _Bool isArray1                 = 0;
                float diff[2]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                    diff[1] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01 && diff[1] < 0.01 && diff[2] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 1);
            HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 1);
            osDelay(500);
            stateflag = 4;
        } else if (stateflag == 4) {        //一内一外的情况
            if (weight_placement[0] == 1 && weight_placement[1] == 0) // 1区在内 2区在外------------->后抓外圈的2区 即left
            {
                LightFury.gantry_t.position.x = 3064;
                //LightFury.gantry_t.position.yL = 410;
                //LightFury.gantry_t.position.yR = 1;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posX                   = Lidar1.distance_aver;
                initial_posYL                  = Lidar2.distance_aver;
                _Bool isArray1                 = 0;
                float diff[2]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    // VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    //VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                    //diff[1] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                    if ((diff[0] < 0.01 )) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[0] == 0 && weight_placement[1] == 1) // 1区在外 2区在内------------->后抓wai圈的1区 即right
            {
                LightFury.gantry_t.position.x  = 3064;
                //LightFury.gantry_t.position.yL = 1;
                //LightFury.gantry_t.position.yR = 410;
                TickType_t StartTick = xTaskGetTickCount();
                initial_posX         = Lidar1.distance_aver;
                initial_posYR        = Lidar3.distance_aver;
                _Bool isArray1       = 0;
                float diff[2]        = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    // VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    //VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                    //diff[1] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01 )) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            stateflag = 5;
        } else if (stateflag == 5){
            if (weight_placement[0] == 1 && weight_placement[1] == 0) // 1区在内 2区在外------------->后抓2区 left
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 1);
                HAL_GPIO_WritePin(ElectromagnetYL_Pin, ElectromagnetYL_Pin, 1);
                //LightFury.gantry_t.position.x = 1;
                //LightFury.gantry_t.position.yL = 1;
                LightFury.gantry_t.position.yR = 410;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posYL                   = Lidar2.distance_aver; 
                _Bool isArray1                 = 0;
                float diff[1]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    //VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    // VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[0] == 0 && weight_placement[1] == 1) // 1区在外 2区在内------------->先抓内圈的2区 即right
            {
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYR_Pin, ElectromagnetYR_Pin, 1);
                //LightFury.gantry_t.position.x  = 1;
                LightFury.gantry_t.position.yR = 410;
                // LightFury.gantry_t.position.yR = 1;
                TickType_t StartTick = xTaskGetTickCount();
                initial_posYR         = Lidar3.distance_aver; 
                _Bool isArray1       = 0;
                float diff[1]        = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    //VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    //VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }

            HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 1);
            HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 1);
            osDelay(500);

            //先让左右的爪子退到两边

            LightFury.gantry_t.position.yL = 612;
            LightFury.gantry_t.position.yR = 612;
            TickType_t StartTick           = xTaskGetTickCount();
            initial_posYL                  = Lidar2.distance_aver;
            initial_posYR                  = Lidar3.distance_aver;
            _Bool isArray1                 = 0;
            float diff[2]                  = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                diff[0] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                diff[1] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                if ((diff[0] < 0.01 && diff[1] < 0.01 && diff[2] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(500);

            stateflag = 6;
        }else if(stateflag==6){     //前往短区木桩
            LightFury.gantry_t.position.x = 3691;
            LightFury.gantry_t.position.yL = 552;
            LightFury.gantry_t.position.yR = 552;
            TickType_t StartTick = xTaskGetTickCount();
            initial_posX         = Lidar1.distance_aver; // 电机轴输出角度 单位 度°
             initial_posYL                  = Lidar2.distance_aver;
            initial_posYR                  = Lidar3.distance_aver;
            _Bool isArray1       = 0;
            float diff[3]        = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                diff[0] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                diff[1] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                if ((diff[0] < 0.01&&diff[1]<0.01&&diff[2]<0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(200);

            LightFury.gantry_t.position.yL = 615;
            LightFury.gantry_t.position.yR = 615;
            TickType_t StartTick01           = xTaskGetTickCount();
            initial_posYL                  = Lidar2.distance_aver;
            initial_posYR                  = Lidar3.distance_aver;
            _Bool isArray2                 = 0;
            float diff01[2]                  = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick01) * 1.0 / 1000.0;
                VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                diff01[0] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                diff01[1] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                if ((diff01[0] < 0.01 && diff01[1] < 0.01 && diff01[2] < 0.01)) { isArray2 = 1; }

            } while (!isArray2);
            osDelay(500);
            HAL_GPIO_WritePin(ElectromagnetYL_GPIO_Port, ElectromagnetYL_Pin, 0);
            HAL_GPIO_WritePin(ElectromagnetYR_GPIO_Port, ElectromagnetYR_Pin, 0);
            stateflag = 7;
        } else if (stateflag == 7) { // 放下中间的砝码
            LightFury.gantry_t.position.x = 2578;
            TickType_t StartTick          = xTaskGetTickCount();
            initial_posX                  = Lidar1.distance_aver; // 电机轴输出角度 单位 度°
            _Bool isArray1                = 0;
            float diff[1]                 = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(500);
            HAL_GPIO_WritePin(ElectromagnetX_GPIO_Port, ElectromagnetX_Pin, 0);
            stateflag = 8;
        } else if (stateflag == 8) // 夹取长区的砝码
        {
            if (weight_placement[2] == 1 && weight_placement[3] == 1) // 都在内圈
            {
                LightFury.gantry_t.position.x  = 2510;
                LightFury.gantry_t.position.yL = 220;
                LightFury.gantry_t.position.yR = 220;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posX                   = Lidar1.distance_aver;
                initial_posYL                  = Lidar2.distance_aver;
                initial_posYR                  = Lidar3.distance_aver;
                _Bool isArray1                 = 0;
                float diff[3]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                    diff[1] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                    diff[2] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01 && diff[1] < 0.01 && diff[2] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[3] == 1 && weight_placement[2] == 0) // 4区在内 3区在外------------->先抓内圈的4区 即right
            {
                LightFury.gantry_t.position.x = 2510;
                // LightFury.gantry_t.position.yL = 1;
                LightFury.gantry_t.position.yR = 220;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posX                   = Lidar1.distance_aver;
                initial_posYR                  = Lidar3.distance_aver;
                _Bool isArray1                 = 0;
                float diff[2]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    // VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                    diff[1] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01 && diff[1] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[3] == 0 && weight_placement[2] == 1) // 4区在外 3区在内------------->先抓内圈的3区 即left
            {
                LightFury.gantry_t.position.x  = 2510;
                LightFury.gantry_t.position.yL = 220;
                // LightFury.gantry_t.position.yR = 1;
                TickType_t StartTick = xTaskGetTickCount();
                initial_posX         = Lidar1.distance_aver;
                initial_posYL        = Lidar2.distance_aver;
                _Bool isArray1       = 0;
                float diff[2]        = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    // VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                    diff[1] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                    if ((diff[0] < 0.01 && diff[1] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[2] == 0 && weight_placement[3] == 0) // 都在外圈
            {
                LightFury.gantry_t.position.x  = 2340;
                // LightFury.gantry_t.position.yL = 1;
                // LightFury.gantry_t.position.yR = 1;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posX                   = Lidar1.distance_aver;
                // initial_posYL                  = Lidar2.distance_aver;
                // initial_posYR                  = Lidar3.distance_aver;
                _Bool isArray1                 = 0;
                float diff[3]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    // VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    // VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                    // diff[1] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                    // diff[2] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01 && diff[1] < 0.01 && diff[2] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            stateflag = 9;
        } else if (stateflag == 9) // 往前滑行一段吸上砝码
        {
            if (weight_placement[2] == 1 && weight_placement[3] == 1) // 都在内圈 ----一起往中间走
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_Pin, ElectromagnetYL_Pin, 1);
                HAL_GPIO_WritePin(ElectromagnetYR_Pin, ElectromagnetYR_Pin, 1);

                LightFury.gantry_t.position.yL = 81;
                LightFury.gantry_t.position.yR = 81;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posYL                  = Lidar2.distance_aver;
                initial_posYR                  = Lidar3.distance_aver;
                _Bool isArray1                 = 0;
                float diff[2]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                    diff[1] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01 && diff[1] < 0.01 && diff[2] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[3] == 1 && weight_placement[2] == 0) // 4区在内 3区在外------------->先抓内圈的4区 即right
            {
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYR_Pin, ElectromagnetYR_Pin, 1);
                // LightFury.gantry_t.position.x = 1;
                //  LightFury.gantry_t.position.yL = 1;
                LightFury.gantry_t.position.yR = 81;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posYR                  = Lidar3.distance_aver; // 电机轴输出角度 单位 度°
                _Bool isArray1                 = 0;
                float diff[1]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    // VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    //  VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[3] == 0 && weight_placement[2] == 1) // 4区在外 3区在内------------->先抓内圈的2区 即left
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_Pin, ElectromagnetYL_Pin, 1);
                // LightFury.gantry_t.position.x  = 1;
                LightFury.gantry_t.position.yL = 81;
                // LightFury.gantry_t.position.yR = 1;
                TickType_t StartTick = xTaskGetTickCount();
                initial_posYL        = Lidar2.distance_aver; // 电机轴输出角度 单位 度°
                _Bool isArray1       = 0;
                float diff[1]        = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    // VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    // VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                    if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[2] == 0 && weight_placement[3] == 0) // 都在外圈
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 0);
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYL_Pin, ElectromagnetYL_Pin, 1);
                HAL_GPIO_WritePin(ElectromagnetYR_Pin, ElectromagnetYR_Pin, 1);

                LightFury.gantry_t.position.yL = 410;
                LightFury.gantry_t.position.yR = 410;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posYL                  = Lidar2.distance_aver;
                initial_posYR                  = Lidar3.distance_aver;
                _Bool isArray1                 = 0;
                float diff[2]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                    diff[1] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01 && diff[1] < 0.01 && diff[2] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 1);
            HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 1);
            osDelay(500);
            stateflag = 10;
        } else if (stateflag == 10) {
            if ((weight_placement[3] == 1 && weight_placement[2] == 0) || (weight_placement[3] == 0 && weight_placement[2] == 1)) {
                LightFury.gantry_t.position.x = 2340;
                // LightFury.gantry_t.position.yL = 410;
                //  LightFury.gantry_t.position.yR = 1;
                TickType_t StartTick = xTaskGetTickCount();
                initial_posX         = Lidar1.distance_aver;
                initial_posYL        = Lidar2.distance_aver;
                _Bool isArray1       = 0;
                float diff[2]        = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                // VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                // VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                // diff[1] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                if ((diff[0] < 0.01 && diff[1] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(500);                                             // 一内一外的情况
            }
            stateflag = 11;
        } else if (stateflag == 11) {
            if (weight_placement[3] == 1 && weight_placement[2] == 0) // 4区在内 3区在外------------->后抓3区 left
            {
                HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 1);
                HAL_GPIO_WritePin(ElectromagnetYL_Pin, ElectromagnetYL_Pin, 1);
                // LightFury.gantry_t.position.x = 1;
                // LightFury.gantry_t.position.yL = 1;
                LightFury.gantry_t.position.yR = 410;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_posYL                  = Lidar2.distance_aver;
                _Bool isArray1                 = 0;
                float diff[1]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    // VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    //  VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }
            if (weight_placement[3] == 0 && weight_placement[2] == 1) // 4区在外 3区在内------------->后抓外圈的4区 即right
            {
                HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 0);
                HAL_GPIO_WritePin(ElectromagnetYR_Pin, ElectromagnetYR_Pin, 1);
                // LightFury.gantry_t.position.x  = 1;
                LightFury.gantry_t.position.yR = 410;
                // LightFury.gantry_t.position.yR = 1;
                TickType_t StartTick = xTaskGetTickCount();
                initial_posYR        = Lidar3.distance_aver; 
                _Bool isArray1       = 0;
                float diff[1]        = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    // VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                    // VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                    VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                    diff[0] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                    if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(500);
            }

            HAL_GPIO_WritePin(CylinderYL_GPIO_Port, CylinderYL_Pin, 1);
            HAL_GPIO_WritePin(CylinderYR_GPIO_Port, CylinderYR_Pin, 1);
            osDelay(500);

            //先让左右的爪子退到两边

            LightFury.gantry_t.position.yL = 612;
            LightFury.gantry_t.position.yR = 612;
            TickType_t StartTick           = xTaskGetTickCount();
            initial_posYL                  = Lidar2.distance_aver;
            initial_posYR                  = Lidar3.distance_aver;
            _Bool isArray1                 = 0;
            float diff[2]                  = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                diff[0] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                diff[1] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                if ((diff[0] < 0.01 && diff[1] < 0.01 && diff[2] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(500);

            stateflag = 12;
        } else if (stateflag == 12) { // 前往长区木桩
            LightFury.gantry_t.position.x  = 3691;
            LightFury.gantry_t.position.yL = 552;
            LightFury.gantry_t.position.yR = 552;
            TickType_t StartTick           = xTaskGetTickCount();
            initial_posX                   = Lidar1.distance_aver; // 电机轴输出角度 单位 度°
            initial_posYL                  = Lidar2.distance_aver;
            initial_posYR                  = Lidar3.distance_aver;
            _Bool isArray1                 = 0;
            float diff[3]                  = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_posX, X_maxvelocity, X_Acceleration, LightFury.gantry_t.position.x, current_time, &(current_posX));
                VelocityPlanning(initial_posYL, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yL, current_time, &(current_posYL));
                VelocityPlanning(initial_posYR, Y_maxvelocity, Y_Acceleration, LightFury.gantry_t.position.yR, current_time, &(current_posYR));
                diff[0] = fabs(LightFury.gantry_t.position.x - current_posX);
                diff[0] = fabs(LightFury.gantry_t.position.yL - current_posYL);
                diff[1] = fabs(LightFury.gantry_t.position.yR - current_posYR);
                if ((diff[0] < 0.01 && diff[1] < 0.01 && diff[2] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(500);

            
            HAL_GPIO_WritePin(ElectromagnetYL_GPIO_Port, ElectromagnetYL_Pin, 0);
            HAL_GPIO_WritePin(ElectromagnetYR_GPIO_Port, ElectromagnetYR_Pin, 0);
            stateflag = 13;
        */
       }

        osDelay(2);
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