#include "UpperStart.h"
#include "stm32f4xx_it.h"
uint16_t UartFlag[4];

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == USART1) {
//         STP_23L_Decode(0);
//         HAL_UARTEx_ReceiveToIdle(&huart1, Rxbuffer[0], sizeof(Rxbuffer[0]), sizeof(Rxbuffer[0]),0xff);
//         HAL_UART_IRQHandler(&huart1); // 调用HAL库中断处理公用函数
//     }
//     if (huart->Instance == USART2) {
//         STP_23L_Decode(1);
//         HAL_UARTEx_ReceiveToIdle(&huart2, Rxbuffer[1], sizeof(Rxbuffer[0]), sizeof(Rxbuffer[0]), 0xff);
//         HAL_UART_IRQHandler(&huart2); // 调用HAL库中断处理公用函数
//     }
//     if (huart->Instance == USART3) {
//         STP_23L_Decode(2);
//         HAL_UARTEx_ReceiveToIdle(&huart3, Rxbuffer[2], sizeof(Rxbuffer[0]), sizeof(Rxbuffer[0]), 0xff);
//         HAL_UART_IRQHandler(&huart3); // 调用HAL库中断处理公用函数
//     }
//     if (huart->Instance == USART6) {
//         STP_23L_Decode(3);
//         HAL_UARTEx_ReceiveToIdle(&huart6, Rxbuffer[3], sizeof(Rxbuffer[0]), sizeof(Rxbuffer[0]), 0xff);
//         HAL_UART_IRQHandler(&huart6); // 调用HAL库中断处理公用函数
//     }
// }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        
        HAL_UART_DMAStop(&huart1);
        STP_23L_Decode(0);
        
        // MX_USART1_UART_Init();
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rxbuffer[0], sizeof(Rxbuffer[0]));
    }
    if (huart->Instance == USART2) {
        
        HAL_UART_DMAStop(&huart2);
        STP_23L_Decode(1);
        
        // MX_USART1_UART_Init();
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Rxbuffer[1], sizeof(Rxbuffer[1]));
    }
    if (huart->Instance == USART3) {
        HAL_UART_DMAStop(&huart3);
        STP_23L_Decode(2);
       
        // MX_USART1_UART_Init();
        __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rxbuffer[2], sizeof(Rxbuffer[2]));
    }
    if (huart->Instance == USART6) {
        HAL_UART_DMAStop(&huart6);
        STP_23L_Decode(3);
        // MX_USART1_UART_Init();
        __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Rxbuffer[3], sizeof(Rxbuffer[3]));
    }
}