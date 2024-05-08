#include "UpperStart.h"


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        STP_23L_Decode();
    }
    HAL_UART_Receive_IT(&huart1, Rxbuffer, sizeof(Rxbuffer));
}