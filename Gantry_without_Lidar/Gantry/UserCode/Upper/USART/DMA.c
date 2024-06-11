#include "UpperStart.h"
#include "DMA.h"

uint8_t temp_buffer[4];
void DMA_Enable()
{
    /*串口使能*/
    // HAL_DMA_Start_IT((&huart1)->hdmarx,)
    // HAL_UART_Receive_DMA(&huart1, Rxbuffer[0], sizeof(Rxbuffer[0]));
    // HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rxbuffer[0], sizeof(Rxbuffer[0]));
    //__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    // HAL_UART_Receive_DMA(&huart2, Rxbuffer[1], sizeof(Rxbuffer[1]));
    //HAL_UART_Receive_IT(&huart2, (Rxbuffer[1]), 195);
    // HAL_UART_Receive_DMA(&huart2, (Rxbuffer[1]), 195);

    // HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Rxbuffer[1], sizeof(Rxbuffer[1]));
    //__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    // HAL_UART_Receive_DMA(&huart3, Rxbuffer[2], sizeof(Rxbuffer[2]));
    // HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rxbuffer[2], sizeof(Rxbuffer[2]));
    //__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    // HAL_UART_Receive_DMA(&huart6, Rxbuffer[3], sizeof(Rxbuffer[3]));
    // HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Rxbuffer[3], sizeof(Rxbuffer[3]));
}