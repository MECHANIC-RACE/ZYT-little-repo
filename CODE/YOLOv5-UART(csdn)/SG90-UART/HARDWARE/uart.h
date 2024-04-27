#ifndef __UART_H
#define __UART_H

#include "stm32f1xx_hal.h" 

extern UART_HandleTypeDef huart1;

#define USART1_REC_LEN  600

extern int  USART1_RX_BUF[USART1_REC_LEN];
extern uint16_t USART1_RX_STA;
extern int USART1_NewData;

void  HAL_UART_RxCpltCallback(UART_HandleTypeDef  *huart);

#endif