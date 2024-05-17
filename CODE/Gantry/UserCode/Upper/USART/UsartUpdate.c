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
            UartFlag[0] = 0;
            //MX_USART1_UART_Init();
            __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rxbuffer[0], sizeof(Rxbuffer[0]));
            osDelay(2);
        }
        if (UartFlag[1]) {
            STP_23L_Decode(1);
            UartFlag[1] = 0;
            //MX_USART2_UART_Init();
            __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Rxbuffer[1], sizeof(Rxbuffer[0]));
            osDelay(2);
        }
        osDelay(1);
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