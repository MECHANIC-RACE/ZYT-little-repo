#include "UpperStart.h"
uint16_t UartFlag[4];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        UartFlag[0] = 1;
        HAL_UART_Receive_IT(&huart1, Rxbuffer[0], sizeof(Rxbuffer[0]));

        //STP_23L_Decode(0);
        //UartFlag[0] = HAL_UART_GetState(&huart1);
        //UartFlag[0] = (&huart1)->gState;
        //HAL_UART_Receive_IT(&huart1, Rxbuffer[0], sizeof(Rxbuffer[0]));
    }
    if (huart->Instance == USART2) { 
        UartFlag[1] = 1;
        HAL_UART_Receive_IT(&huart2, &(Rxbuffer[1]), 195);
        // HAL_UART_Receive_IT(&huart2, (Rxbuffer[1]), sizeof(Rxbuffer[0]));

        // STP_23L_Decode(1);
        // HAL_UART_Receive_IT(&huart2, Rxbuffer[1], sizeof(Rxbuffer[1]));
    }
    // if (huart->Instance == USART3) {
    //     UartFlag[2] = 1;
    //     // STP_23L_Decode(2);
    //     // HAL_UART_Receive_IT(&huart3, Rxbuffer[2], sizeof(Rxbuffer[2]));
    // }
    // if (huart->Instance == USART6) {
    //     UartFlag[3] = 1;
    //     // STP_23L_Decode(3);
    //     // HAL_UART_Receive_IT(&huart6, Rxbuffer[3], sizeof(Rxbuffer[3]));
    // }
}