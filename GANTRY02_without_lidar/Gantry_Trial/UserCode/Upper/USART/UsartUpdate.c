
#include "UsartUpdate.h"
#include "UpperStart.h"

uint16_t Uart_State = 0;
uint16_t detect01_cnt;
float angle_memory01;
void UartUpdateTask(void *argument)
{
    /* USER CODE BEGIN UartUpdateTask */
    
    float weight_placement_tmp[5] = {0};
    int switch_flag               = 0; // 判断每次接收到的数组与基准数组是否相等
    int tar_count                 = 0; // 计数连续相同数组的次数
    // osDelay(100);


    /* Infinite loop */
    for (;;) {
        if(Uart_State==0)
        {
            if (UartFlag[5] == 1) {
                Upper_Target_Decode();
                for (int i = 0; i < 5; i++) {
                    weight_placement_tmp[i] = weight_placement[i];
                }
                UartFlag[5]   = 0;
                tar_count = 1; // 首次接收时，计数器初始化为1
                Uart_State    = 1;
            }
        }
        else if(Uart_State==1){
              
                    if (UartFlag[5] == 1) {
                        Upper_Target_Decode();
                        UartFlag[5]     = 0;
                        switch_flag = 0;

                        for (int i = 0; i < 5; i++) {
                            if (weight_placement_tmp[i] != weight_placement[i]) {
                                switch_flag = 1;
                                break;
                            }
                        }

                        // 收到的数组与基准数组相等
                        if (switch_flag == 0) {
                            tar_count++;
                        }
                        // 收到的数组与基准数组不相等
                        else {
                            tar_count = 1; // 重新计数
                            for (int i = 0; i < 5; i++) {
                                weight_placement_tmp[i] = weight_placement[i];
                            }
                        }

                        // 如果连续十次接收到同样的数组，则把这个数组设置为最终值
                        if (tar_count >= 10) {
                            Uart_State = 2;
                            __HAL_UART_DISABLE_IT(&huart5, UART_IT_RXNE);
                        }
                    }
                    osDelay(2);
                }
            
        
        else if(Uart_State==2){
        if (UartFlag[0]) {
            STP_23L_Decode(Rxbuffer_1,&Lidar1);
            if (detect01 && Lidar1.distance_aver < 300) detect01_cnt++;
            if(detect01 && detect01_cnt==5)
            {
                detect01 = 0;
                angle_memory01 = Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree;
            }
            UartFlag[0] = 0;
        }
        if (UartFlag[1]) {
            STP_23L_Decode(Rxbuffer_2,&Lidar2);
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
        osDelay(2);
        }
    }
    /* USER CODE END UartUpdateTask */
}

void UsartUpdate_Start()
{
    osThreadId_t UsartUpdateHandle;
    const osThreadAttr_t UsartUpdate_attributes = {
        .name       = "UsartUpdate",
        .stack_size = 128 * 4,
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
    HAL_UART_Receive_IT(&huart4, usart4_rx, 1);

    // HAL_UART_Receive_DMA(&huart1, usart1_rx, 1);
    // HAL_UART_Receive_DMA(&huart2, usart2_rx, 1);
    // HAL_UART_Receive_DMA(&huart3, usart3_rx, 1);
    // HAL_UART_Receive_DMA(&huart6, usart6_rx, 1);
    /*2，3，6串口的使能函数*/
    __HAL_UART_ENABLE(&huart5);//到时串口5将被用作树莓派接收
   
}


void RaspReceive_Enable()
{
    HAL_UART_Receive_IT(&huart5, receive_buffer, sizeof(receive_buffer));
}