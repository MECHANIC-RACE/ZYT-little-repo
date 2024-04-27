#include "control.h"
#include "uart.h"
#include "tim.h"
#include "oled.h"

int num;
int num2;
int num3;
int value;
int flag;
int last;


void TargetTracking()
{
	flag = USART1_RX_STA - last;
				
	last = USART1_RX_STA;
	
	value = (num3-48) * 100 + (num2-48) * 10 + (num-48) * 1;
	
	OLED_ShowStr(10,2,"Object Center",2);
	OLED_Showdecimal(45,4,value,3,1,16);
	
	if(flag == 2)
	{
		value = (num2-48) * 10 + (num-48) * 1;
		OLED_Showdecimal(40,4,value,3,1,16);
	}
	
}