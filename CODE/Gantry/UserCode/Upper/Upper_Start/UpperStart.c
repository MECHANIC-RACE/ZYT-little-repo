
#include "UpperStart.h"



void StartDefaultTask(void *argument)
{
/*初始化函数*/
    Core_XY_StateMachine_Init();
//    Core_xy_Motor_init();
/*开启线程*/
    Core_xy_StateMachine_Start();
    //Upper_Servo_Start();
/*串口使能*/
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    HAL_UART_Receive_IT(&huart1, Rxbuffer, sizeof(Rxbuffer));
    __HAL_UART_ENABLE(&huart2);
    
    //HAL_UART_Receive_IT(&huart7, Rxbuffer, sizeof(Rxbuffer));

    for (;;) {

        osDelay(10);
    }
}







