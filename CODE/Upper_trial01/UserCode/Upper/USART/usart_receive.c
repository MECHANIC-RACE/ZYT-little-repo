#include "UpperStart.h"

uint8_t Rxbuffer[5];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART8) {
        if((Rxbuffer[0]==0xEE)&&(Rxbuffer[4]==0xFF))
        {
        Target.position.x = Rxbuffer[1];
        Target.position.y = Rxbuffer[2];
        Target.position.z = Rxbuffer[3];
        
        //HAL_UART_Transmit(&huart8, Rxbuffer, sizeof(Rxbuffer), 0xFFFF);
        }
    }
    HAL_UART_Receive_IT(&huart8, Rxbuffer, sizeof(Rxbuffer));
}