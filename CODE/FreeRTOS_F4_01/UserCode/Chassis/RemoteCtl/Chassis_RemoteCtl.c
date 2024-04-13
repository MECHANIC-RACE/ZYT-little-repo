/*
 * @Author: Chen Yitong
 * @Date: 2023-09-23 13:47:29
 * @LastEditors: Chen Yitong
 * @LastEditTime: 2023-09-24 18:52:45
 * @FilePath: \WTR_Chassis\麦克纳姆轮\UserCode\Chassis\RemoteCtl\Chassis_RemoteCtl.c
 * @Brief: 底盘遥控代码
 *
 * Copyright (c) 2023 by ChenYiTong, All Rights Reserved.
 */

#include "Chassis_RemoteCtl.h"


/**
 * @brief 底盘遥控初始化
 * @return {*}
 */
void Chassis_RemoteCtl_Init()
{
    HAL_UART_Receive_DMA(&huart_RemoteCtl, JoyStickReceiveData, 18);        //在cube中选择circular模式循环接收JoyStick的控制信息，并且接受后引发中断，开始解码

    //for(uint16_t i=0;i<18;i++)   JoyStickReceiveData[i]=5;              //5:测试数据

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // IDLE 中断使能

}

