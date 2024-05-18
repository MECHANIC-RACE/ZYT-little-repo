#include "UpperStart.h"
#include "stm32f4xx_it.h"
uint16_t UartFlag[4];
uint16_t debug[4];
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

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//     if (huart->Instance == USART1) {

//         HAL_UART_DMAStop(&huart1);
//         __HAL_UART_CLEAR_IDLEFLAG(&huart1);
//         UartFlag[0] = 1;
//     }
//     if (huart->Instance == USART2) {

//         HAL_UART_DMAStop(&huart2);
//         __HAL_UART_CLEAR_IDLEFLAG(&huart2);
//         UartFlag[1] = 1;
//     }
//     if (huart->Instance == USART3) {
//         HAL_UART_DMAStop(&huart3);
//         UartFlag[2] = 1;
//     }
//     if (huart->Instance == USART6) {
//         HAL_UART_DMAStop(&huart6);
//         UartFlag[3] = 1;
//     }
// }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {

        // HAL_UART_DMAStop(&huart1);
        // STP_23L_Decode(0);
        // UartFlag[0] = 1;
        // HAL_UART_Receive_DMA(&huart1, Rxbuffer[0], sizeof(Rxbuffer[0]));
    }
    if (huart->Instance == USART2) {
        // printf("it");
        //  STP_23L_Decode(1);
        //  HAL_UART_DMAStop(&huart2);
        //  HAL_UART_Receive_DMA(&huart2, Rxbuffer[1], sizeof(Rxbuffer[0]));
        UartFlag[1] = 1;
        HAL_UART_Receive_IT(&huart2, (Rxbuffer[1]), 195);
    }
    if (huart->Instance == USART3) {
        // HAL_UART_DMAStop(&huart3);
        UartFlag[2] = 1;
    }
    if (huart->Instance == USART6) {
        // HAL_UART_DMAStop(&huart6);
        UartFlag[3] = 1;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uartHandle)
{
    uartHandle->RxState = HAL_UART_STATE_READY;
    if (uartHandle->Instance == USART2) {
        HAL_UART_Receive_IT(&huart2, (Rxbuffer[1]), 195);
        // if (HAL_OK != HAL_UART_Receive_IT(&huart2, (Rxbuffer[1]), 195)) {
        // assert("HAL_UART_Receive_IT", __FILE__, __LINE__);
        // }
    }
}
