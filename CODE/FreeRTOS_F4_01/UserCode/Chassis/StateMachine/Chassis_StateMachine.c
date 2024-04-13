/*
 * @Author: Chen Yitong 3083697520@qq.com
 * @Date: 2023-09-23 11:33:41
 * @LastEditors: labbbbbbbbb 
 * @LastEditTime: 2024-04-13 14:54:49
 * @FilePath: \FreeRTOS_F4_01\UserCode\Chassis\StateMachine\Chassis_StateMachine.c
 * @brief 底盘状态机
 *
 * Copyright (c) 2023 by ChenYiTong, All Rights Reserved.
 */
#include "Chassis_StateMachine.h"
CHASSIS_MOVING_STATE ChassisControl;
CHASSIS_MOVING_STATE ChassisState;

/**
 * @brief: 状态机线程
 * @return {*}
 */
void StateMachine_Task(void const *argument)            //根据接收到的Remote信息来给ChassisControl赋值，ChassisControl后面会给Servo中的结构体赋值
{
    for (;;) {
        
        DjiRemoteCtl_Decode(); // 大疆遥控器解码

        vPortEnterCritical();
        Remote_t RemoteCtl_RawData_tmp = RemoteCtl_RawData;         //来自decode所得的RemoteCtl结构体
        vPortExitCritical();
        RemoteCtl_RawData_tmp.left=3;
        RemoteCtl_RawData_tmp.ch0=3;
        RemoteCtl_RawData_tmp.ch1=3;
        RemoteCtl_RawData_tmp.ch2=3;
        switch (RemoteCtl_RawData_tmp.left) {
            case Stop:
                xSemaphoreTakeRecursive(ChassisControl.xMutex_control, portMAX_DELAY);
                ChassisControl.velocity.x = 0;
                ChassisControl.velocity.y = 0;
                ChassisControl.velocity.w = 0;
                xSemaphoreGiveRecursive(ChassisControl.xMutex_control);
                break;
            case Run:
                xSemaphoreTakeRecursive(ChassisControl.xMutex_control, portMAX_DELAY);
                DeadBandOneDimensional((RemoteCtl_RawData_tmp.ch0 - 1024) * 0.001, &(ChassisControl.velocity.x), 0.05);
                DeadBandOneDimensional((RemoteCtl_RawData_tmp.ch1 - 1024) * 0.001, &(ChassisControl.velocity.y), 0.05);
                DeadBandOneDimensional((RemoteCtl_RawData_tmp.ch2 - 1024) * 0.001, &(ChassisControl.velocity.w), 0.05);
                xSemaphoreGiveRecursive(ChassisControl.xMutex_control);
                break;
            case Correcting:

                break;
            default:
                break;
        }
        vTaskDelay(10);
    }
}

/**
 * @brief: 状态机线程启动
 * @return {*}
 */
void StateMachine_TaskStart()
{
    osThreadId_t StateMachineHandle;
    const osThreadAttr_t StateMachine_attributes = {
        .name       = "StateMachine",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityAboveNormal,
    };
    StateMachineHandle = osThreadNew(StateMachine_Task, NULL, &StateMachine_attributes);
}
    /**
     * @brief 初始化状态机
     * @return {*}
     */

    void Chassis_StateMachine_Init()
    {
        ChassisControl.xMutex_control = xSemaphoreCreateRecursiveMutex();
        ChassisState.xMutex_control   = xSemaphoreCreateRecursiveMutex();
        WheelComponent.xMutex_wheel   = xSemaphoreCreateRecursiveMutex();
}

void Chassis_SteerinfWheelCorrect(){
}