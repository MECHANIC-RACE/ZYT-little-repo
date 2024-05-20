#include "UsartUpdate.h"
#include "UpperStart.h"

void UartUpdateTask(void *argument)
{
    /* USER CODE BEGIN UartUpdateTask */
   

    /* Infinite loop */
    for (;;) {
        if (UartFlag[0]) {
            STP_23L_Decode(Rxbuffer_1,&Lidar1);
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
        osDelay(10);
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
    // HAL_UART_Receive_DMA(&huart1, usart1_rx, 1);
    // HAL_UART_Receive_DMA(&huart2, usart2_rx, 1);
    // HAL_UART_Receive_DMA(&huart3, usart3_rx, 1);
    // HAL_UART_Receive_DMA(&huart6, usart6_rx, 1);
    /*2，3，6串口的使能函数*/
    __HAL_UART_ENABLE(&huart4);
}
