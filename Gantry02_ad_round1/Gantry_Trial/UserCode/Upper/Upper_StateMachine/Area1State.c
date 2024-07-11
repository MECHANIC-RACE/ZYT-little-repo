
/*
 * @Author: ZYT
 * @Date: 2024-06-06 12:03:15
 * @LastEditors: ZYT
 * @LastEditTime: 2024-07-11 16:41:15
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

#define X_offset 780
#define Y_offset01 10      //fama //-40
#define Y_offset02 140      //muzhuang

uint16_t detect01xtree;
uint16_t detect01ytree;
uint16_t detect01_weight;

uint16_t UseLidar01;

float initial_pos01[2];
float current_pos01[2];


/****************线程相关函数********************/

void Area1_State_Task(void *argument)
{
    weight_placement[2] = 1; // 到时直接换成weight_detect[]就好
    osDelay(100);
    uint16_t stateflag = 0;
    uint16_t statechoose = Check_LidarStatus(Lidar1,Lidar2);
    for (;;) {
        /*用于一个分区的路径规划*/
        if(1){        /*雷达状态正确*/
        if (stateflag == 0) // 在y轴上走半段，测出砝码的y轴位置
        {
            if(weight_placement[2]==1){
                Core_xy[0].gantry_t.position.y = 3400;  //2950
                TickType_t StartTick           = xTaskGetTickCount();
                initial_pos01[1]               = Core_xy[0].Motor_Y->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
                _Bool isArray1                 = 0;
                float diff[1]                  = {0};
                detect01_weight                = 1;
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_pos01[1], 500, 500, Core_xy[0].gantry_t.position.y, current_time, &(current_pos01[1]));
                    diff[0] = fabs(Core_xy[0].gantry_t.position.y - current_pos01[1]);
                    if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
            }else{
                detect01_weight = 2;
            }
                stateflag = 1;
        }else if(stateflag==1){
            if(weight_placement[2]==1){
                if(detect01_weight==0)
                {
                    Core_xy[0].gantry_t.position.y = angle_memory01weight+Y_offset01;
                }else{
                    Core_xy[0].gantry_t.position.y = 2893;
                }
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
            }
                detect01_weight = 2;//跳开状态，防止受后续进程影响
                stateflag       = 2;

        }
        else if (stateflag == 2) // 放下气缸，打开磁铁，往前拖行，吸起砝码，提上气缸
        {

            pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);
            pid_reset(&(Core_xy[0].Motor_Y->speedPID), 0, 0, 0);
            HAL_GPIO_WritePin(Cylinder01_GPIO_Port, Cylinder01_Pin, 1);
            HAL_GPIO_WritePin(Electromagnet01_GPIO_Port, Electromagnet01_Pin, 1); // 放下气缸，打开电磁铁

            osDelay(100);

            pid_reset(&(Core_xy[0].Motor_X->speedPID), 5, 0.4, 0.8);

            if (weight_placement[2] == 0) { Core_xy[0].gantry_t.position.x = -400; } // 往前拖行一段
            else {
                Core_xy[0].gantry_t.position.x = -2300;
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
            stateflag = 3;
        }

        /*前往木桩*/
        else if (stateflag == 3) {
            
            
            Core_xy[0].gantry_t.position.x = -6900;  //7600
            
            TickType_t StartTick           = xTaskGetTickCount();
            initial_pos01[0]               = Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
            
            _Bool isArray2                 = 0;
            float diff[1]                  = {0};
            
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos01[0], X_maxvelocity, X_Acceleration, Core_xy[0].gantry_t.position.x, current_time, &(current_pos01[0]));
                
                diff[0] = fabs(Core_xy[0].gantry_t.position.x - current_pos01[0]);
                
                if (diff[0] < 0.01) { isArray2 = 1; }
            
        } while (!isArray2);
        stateflag = 4;
        pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);

        } else if (stateflag == 4) {        //往左走一段巡视木桩位置
            Core_xy[0].gantry_t.position.y = 4300; // 2950
            TickType_t StartTick           = xTaskGetTickCount();
            initial_pos01[1]               = Core_xy[0].Motor_Y->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
            _Bool isArray1                 = 0;
            float diff[1]                  = {0};
            detect01ytree                = 1;
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos01[1], 500, 500, Core_xy[0].gantry_t.position.y, current_time, &(current_pos01[1]));
                diff[0] = fabs(Core_xy[0].gantry_t.position.y - current_pos01[1]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);

            stateflag = 5;
            }
        else if(stateflag==5){
            
            osDelay(100);
            if (detect01ytree==0){
                Core_xy[0].gantry_t.position.y = angle_memory01ytree +  Y_offset02;
            }else{
                Core_xy[0].gantry_t.position.y = 3839;
            }
            detect01ytree                  = 2;
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
            osDelay(100);
            
            pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);
            pid_reset(&(Core_xy[0].Motor_Y->speedPID), 0, 0, 0);
            osDelay(100);
            stateflag = 6;
        } else if (stateflag == 6) {        //最后在x轴上的微调，就看是要用雷达伺服（--->置标志位）还是再挂一个雷达
            if(Lidar2.distance_aver!=0)  
            {
                UseLidar01 = 1;        //稍微检验一下雷达是否异常 将雷达标志位置一
                Core_xy[0].gantry_t.position.x = 83;
                initial_pos01[0]               = Lidar2.distance_aver; // 电机轴输出角度 单位 度°
            }else{
                Core_xy[0].gantry_t.position.x = -7390;
                initial_pos01[0]               = Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
            }
            pid_reset(&(Core_xy[0].Motor_X->speedPID), 5, 0.4, 0.8);
        
            TickType_t StartTick = xTaskGetTickCount();
            
            _Bool isArray1       = 0;
            float diff[1]        = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
            
                if(UseLidar01==1)    VelocityPlanning(initial_pos01[0], 1000, 50, Core_xy[0].gantry_t.position.x, current_time, &(current_pos01[0]));
                else    VelocityPlanning(initial_pos01[0], 4000, 500, Core_xy[0].gantry_t.position.x, current_time, &(current_pos01[0]));
                diff[0] = fabs(Core_xy[0].gantry_t.position.x - current_pos01[0]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(200);
            pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);
            osDelay(500);
            HAL_GPIO_WritePin(Electromagnet01_GPIO_Port, Electromagnet01_Pin, 0);
            stateflag = 7;
        } else if (stateflag == 7) {
            pid_reset(&(Core_xy[0].Motor_X->speedPID), 5, 0.4, 0.8);
            UseLidar01                     = 0;
            Core_xy[0].gantry_t.position.x = -6800;

            TickType_t StartTick = xTaskGetTickCount();
            initial_pos01[0]     = Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
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
            stateflag = 8;
        }
        // osDelay(2);
        }else{
            /*雷达状态有误时*/
        }
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