#include "UsartUpdate.h"
#include "UpperStart.h"
//uint16_t cnt[2];
void UartUpdateTask(void *argument)
{
    /* USER CODE BEGIN UartUpdateTask */
    // Usart1_Queue = xQueueCreate(195, 1);
    // Usart2_Queue = xQueueCreate(195, 1);

    /* Infinite loop */
    for (;;) {
        if (UartFlag[0]) {
            
                UartFlag[0] = 0;
                STP_23L_Decode(0);
                
           
        }
        if(UartFlag[1])
        {
            
                //printf("u2\n");
                UartFlag[1] = 0;
                STP_23L_Decode(1);
                
                //HAL_UART_Receive_IT(&huart2, Rxbuffer[1], sizeof(Rxbuffer[0]));
            
        }
        osDelay(100);
    }
    /* USER CODE END UartUpdateTask */
}

void UsartUpdate_Start()
{
    osThreadId_t UsartUpdateHandle;
    const osThreadAttr_t UsartUpdate_attributes = {
        .name       = "UsartUpdate",
        .stack_size = 128 * 4,
        .priority   = (osPriority_t)osPriorityNormal,
    };
    UsartUpdateHandle = osThreadNew(UartUpdateTask, NULL, &UsartUpdate_attributes);
}