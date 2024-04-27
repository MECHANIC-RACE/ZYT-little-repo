#include "uart.h"
#include "oled.h"

int USART1_RX_BUF[USART1_REC_LEN];		//目标数据
uint16_t USART1_RX_STA=2;
int USART1_NewData;

extern int num;		//标志
extern int num2;
extern int num3;


void  HAL_UART_RxCpltCallback(UART_HandleTypeDef  *huart)
{
    if(huart ==&huart1)
    { 
			
      USART1_RX_BUF[USART1_RX_STA&0X7FFF]=USART1_NewData; 					
      USART1_RX_STA++;  
			
						
      if(USART1_RX_STA>(USART1_REC_LEN-1))USART1_RX_STA=0;

			//num = USART1_RX_BUF[USART1_RX_STA];
			
			HAL_UART_Receive_IT(&huart1,(uint8_t *)&USART1_NewData,1);
			
			num = USART1_RX_BUF[USART1_RX_STA-1];			
			num2 = USART1_RX_BUF[USART1_RX_STA-2];
			num3 = USART1_RX_BUF[USART1_RX_STA-3];			
    }		
}