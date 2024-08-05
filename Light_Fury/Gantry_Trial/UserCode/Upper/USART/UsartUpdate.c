/*
 * @Author: ZYT
 * @Date: 2024-07-20 21:31:27
 * @LastEditors: ZYT
 * @LastEditTime: 2024-07-31 00:31:53
 * @FilePath: \Gantry_Trial\UserCode\Upper\USART\UsartUpdate.c
 * @Brief: 
 * 
 * Copyright (c) 2024 by zyt, All Rights Reserved. 
 */

#include "UsartUpdate.h"
#include "UpperStart.h"

uint16_t Uart_State = 0;

void UartUpdateTask(void *argument)
{
    /* USER CODE BEGIN UartUpdateTask */
    
    float weight_placement_tmp[5] = {0};
    int switch_flag               = 0; // 判断每次接收到的数组与基准数组是否相等
    int tar_count                 = 0; // 计数连续相同数组的次数
    // osDelay(100);


    /* Infinite loop */
    for (;;) {
        if (Uart_State == 0) {
            if (UartFlag[4] == 1) {
                Upper_Target_Decode();
                UartFlag[4] = 0;
                if (weight_placement[0] == 1) weight_placement_tmp[0] = 1;
                if (weight_placement[1] == 1) weight_placement_tmp[1] = 1;
                if (weight_placement[2] == 1) weight_placement_tmp[2] = 1;
                if (weight_placement[3] == 1) weight_placement_tmp[3] = 1;
                if (weight_placement[4] == 1) weight_placement_tmp[4] = 1;
            if(receive_buffer[0]== 255)    tar_count++; // 首次接收时，计数器初始化为1
                if (tar_count >= 10) {
                    if (weight_placement_tmp[0] == 1) weight_placement[0] = 1;
                    if (weight_placement_tmp[1] == 1) weight_placement[1] = 1;
                    if (weight_placement_tmp[2] == 1) weight_placement[2] = 1;
                    if (weight_placement_tmp[3] == 1) weight_placement[3] = 1;
                    if (weight_placement_tmp[4] == 1) weight_placement[4] = 1;
                    Uart_State = 1;
                }
            }
        } else if (Uart_State == 1) {
            if (UartFlag[0]) {
                STP_23L_Decode(Rxbuffer_1, &Lidar1);
                UartFlag[0] = 0;
            }
            if (UartFlag[1]) {
                STP_23L_Decode(Rxbuffer_2, &Lidar2);
                UartFlag[1] = 0;
            }
            if (UartFlag[2]) {
                STP_23L_Decode(Rxbuffer_3, &Lidar3);
                UartFlag[2] = 0;
            }
            if (UartFlag[3]) {
                STP_23L_Decode(Rxbuffer_6, &Lidar6);
                UartFlag[3] = 0;
            }
            if (UartFlag[4]) {
                STP_23L_Decode(Rxbuffer_4, &Lidar4);
                UartFlag[4] = 0;
            }
            osDelay(6);
        }
    }
    /* USER CODE END UartUpdateTask */
}

void UsartUpdate_Start()
{
    osThreadId_t UsartUpdateHandle;
    const osThreadAttr_t UsartUpdate_attributes = {
        .name       = "UsartUpdate",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityAboveNormal,
    };
    UsartUpdateHandle = osThreadNew(UartUpdateTask, NULL, &UsartUpdate_attributes);
}

void Usart_start()
{
    HAL_UART_Receive_IT(&huart1, usart1_rx, 1);
    HAL_UART_Receive_IT(&huart2, usart2_rx, 1);
    HAL_UART_Receive_IT(&huart3, usart3_rx, 1);
    HAL_UART_Receive_IT(&huart6, usart6_rx, 1);
}


void RaspReceive_Enable()
{
    HAL_UART_Receive_IT(&huart4, receive_buffer, sizeof(receive_buffer));
}