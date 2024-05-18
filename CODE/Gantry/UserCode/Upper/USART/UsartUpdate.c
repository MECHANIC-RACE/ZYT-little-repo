#include "UsartUpdate.h"
#include "UpperStart.h"
uint16_t cnt_queue[4];
void UartUpdateTask(void *argument)
{
    /* USER CODE BEGIN UartUpdateTask */
   

    /* Infinite loop */
    for (;;) {
        if (UartFlag[0]) {
            STP_23L_Decode(0);
            

            //MX_USART1_UART_Init();
            //while (!HAL_DMA_GetState((&huart1)->hdmarx));
            //__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
            //HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rxbuffer[0], sizeof(Rxbuffer[0]));
            //HAL_UART_Receive_DMA(&huart1, Rxbuffer[0], sizeof(Rxbuffer[0]));
            UartFlag[0] = 0;
        }
        if (UartFlag[1]) {
            STP_23L_Decode(1);
            //printf("it");
            //MX_USART2_UART_Init();
            //__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
            //HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Rxbuffer[1], sizeof(Rxbuffer[0]));
            //HAL_UART_Receive_DMA(&huart2, Rxbuffer[1], sizeof(Rxbuffer[0]));
            UartFlag[1] = 0;
        }
        if (UartFlag[2]) {
            STP_23L_Decode(2);

            // MX_USART2_UART_Init();
            //__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
            // HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Rxbuffer[1], sizeof(Rxbuffer[0]));
            //HAL_UART_Receive_DMA(&huart3, Rxbuffer[2], sizeof(Rxbuffer[0]));
            //UartFlag[2] = 0;
        }
        osDelay(10);
    }
    /* USER CODE END UartUpdateTask */
}

UsartUpdate_Start()
{
    osThreadId_t UsartUpdateHandle;
    const osThreadAttr_t UsartUpdate_attributes = {
        .name       = "UsartUpdate",
        .stack_size = 128 * 4,
        .priority   = (osPriority_t)osPriorityAboveNormal,
    };
    UsartUpdateHandle = osThreadNew(UartUpdateTask, NULL, &UsartUpdate_attributes);
}