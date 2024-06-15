/*
 * @Author: ZYT
 * @Date: 2024-06-06 12:03:15
 * @LastEditors: ZYT
 * @LastEditTime: 2024-06-16 00:23:54
 * @FilePath: \Gantry_Trial\UserCode\Upper\Upper_StateMachine\Area1State.c
 * @Brief: 
 * 
 * Copyright (c) 2024 by zyt, All Rights Reserved. 
 */

#include "Area1State.h"

#define X_maxvelocity 6000   //396  390-600都可，不过时间差不多，而且到后面就有点贴不上了
#define Y_maxvelocity 6000   //264.5 
#define X_Acceleration 3000
#define Y_Acceleration 3000

#define X_offset 100

uint16_t detect01;

float initial_pos01[2];
float current_pos01[2];


/****************线程相关函数********************/

void Area1_State_Task(void *argument)
{
    inner_ring_flag = 1;
    osDelay(100);
    uint16_t stateflag = 0;
    for (;;) {
        /*用于一个分区的路径规划*/
        if (stateflag == 0) // 走到即将取第一个砝码之前
        {

            if (inner_ring_flag == 0) // 在li圈
            {
                pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);
                stateflag = 1;
            } else {
                /*if flag=1 在wai圈*/
                Core_xy[0].gantry_t.position.y = 2950;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_pos01[1]               = Core_xy[0].Motor_Y->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
                _Bool isArray1                 = 0;
                float diff[1]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_pos01[1], Y_maxvelocity, Y_Acceleration, Core_xy[0].gantry_t.position.y, current_time, &(current_pos01[1]));
                    diff[0] = fabs(Core_xy[0].gantry_t.position.y - current_pos01[1]);
                    if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                stateflag            = 1;
               
            }
        } else if (stateflag == 1) // 放下气缸，打开磁铁，往前拖行，吸起砝码，提上气缸
        {

            pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);
            pid_reset(&(Core_xy[0].Motor_Y->speedPID), 0, 0, 0);
            HAL_GPIO_WritePin(Cylinder01_GPIO_Port, Cylinder01_Pin, 1);
            HAL_GPIO_WritePin(Electromagnet01_GPIO_Port, Electromagnet01_Pin, 1); // 放下气缸，打开电磁铁

            osDelay(100);

            pid_reset(&(Core_xy[0].Motor_X->speedPID), 5, 0.4, 0.8);

            if (inner_ring_flag == 0) { Core_xy[0].gantry_t.position.x = -400; } // 往前拖行一段
            else {
                Core_xy[0].gantry_t.position.x = -2200;
            } // 往前拖行一段

            TickType_t StartTick           = xTaskGetTickCount();
            initial_pos01[0]                 = Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
            _Bool isArray1                 = 0;
            float diff[1]                  = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos01[0], 20000, 3000, Core_xy[0].gantry_t.position.x, current_time, &(current_pos01[0]));
                diff[0] = fabs(Core_xy[0].gantry_t.position.x - current_pos01[0]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(50);
            HAL_GPIO_WritePin(Cylinder01_GPIO_Port, Cylinder01_Pin, 0);
            /*GPIO_WRITE_PIN提起气缸*/
            osDelay(50);
            pid_reset(&(Core_xy[0].Motor_Y->speedPID), 3.5, 0.3, 0.3);
            stateflag = 2;
        }

        /*前往木桩*/
        else if (stateflag == 2) {

            Core_xy[0].gantry_t.position.x = -7480;  //7600
            Core_xy[0].gantry_t.position.y = 3880;
            TickType_t StartTick           = xTaskGetTickCount();
            initial_pos01[0]               = Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
            initial_pos01[1]               = Core_xy[0].Motor_Y->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
            _Bool isArray2                 = 0;
            float diff[2]                  = {0};
            detect01                       = 1;
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos01[0], X_maxvelocity, X_Acceleration, Core_xy[0].gantry_t.position.x, current_time, &(current_pos01[0]));
                VelocityPlanning(initial_pos01[1], Y_maxvelocity, Y_Acceleration, Core_xy[0].gantry_t.position.y, current_time, &(current_pos01[1]));
                diff[0] = fabs(Core_xy[0].gantry_t.position.x - current_pos01[0]);
                diff[1] = fabs(Core_xy[0].gantry_t.position.y - current_pos01[1]);
                if ((diff[0] < 0.01) && (diff[1] < 0.01)) { isArray2 = 1; }

        } while (!isArray2);
        stateflag = 3;
        } else if (stateflag == 3) {
            osDelay(100);
            // if(detect01==0){
            // Core_xy[0].gantry_t.position.x = angle_memory01 + X_offset;
            // TickType_t StartTick           = xTaskGetTickCount();
            // initial_pos01[0]               = Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
            // _Bool isArray1                 = 0;
            // float diff[1]                  = {0};
            // do {
            //     TickType_t CurrentTick = xTaskGetTickCount();
            //     float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
            //     VelocityPlanning(initial_pos01[0], X_maxvelocity, X_Acceleration, Core_xy[0].gantry_t.position.x, current_time, &(current_pos01[0]));
            //     diff[0] = fabs(Core_xy[0].gantry_t.position.x - current_pos01[0]);
            //     if ((diff[0] < 0.01)) { isArray1 = 1; }

            // } while (!isArray1);
            // osDelay(100);
            // }
            pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);
            pid_reset(&(Core_xy[0].Motor_Y->speedPID), 0, 0, 0);
            
            osDelay(50);
            HAL_GPIO_WritePin(Electromagnet01_GPIO_Port, Electromagnet01_Pin, 0);
            stateflag = 4;
        } else if (stateflag == 4) {
            pid_reset(&(Core_xy[0].Motor_X->speedPID), 5, 0.4, 0.8);
             
            Core_xy[0].gantry_t.position.x = -7200;
         

            TickType_t StartTick = xTaskGetTickCount();
            initial_pos01[0]       = Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
            _Bool isArray1       = 0;
            float diff[1]        = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos01[0], X_maxvelocity, X_Acceleration, Core_xy[0].gantry_t.position.x, current_time, &(current_pos01[0]));
                diff[0] = fabs(Core_xy[0].gantry_t.position.x - current_pos01[0]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);
            stateflag = 5;
        }
        
        // osDelay(2);
        
    }
}

void Area1_StateMachine_Start(void)
{
    osThreadId_t Area1_StateHandle;
    const osThreadAttr_t Area1_State_attributes = {
        .name       = "Area1_State",
        .stack_size = 128 * 20,
        .priority   = (osPriority_t)osPriorityNormal,
    };

    Area1_StateHandle = osThreadNew(Area1_State_Task, NULL, &Area1_State_attributes);
}

/*******封装函数***********/
void Area1_StateMachine_Init()
{
   
    Core_xy[0].gantry_t.xMutex_control = xSemaphoreCreateRecursiveMutex();
    Core_xy[1].gantry_t.xMutex_control = xSemaphoreCreateRecursiveMutex();
}


void pid_reset(PID_t *pid,float kp,float ki,float kd)
{
    pid->KP = kp;
    pid->KI = ki;
    pid->KD = kd;
    pid->integral = 0;
    pid->output   = 0;
}