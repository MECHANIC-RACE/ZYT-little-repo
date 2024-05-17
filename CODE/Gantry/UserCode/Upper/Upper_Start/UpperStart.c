
#include "UpperStart.h"



void StartDefaultTask(void *argument)
{
/*初始化函数*/
    Core_XY_StateMachine_Init();
    Core_xy_Motor_init();
/*开启线程*/
    Core_xy_StateMachine_Start();
    Upper_Servo_Start();
/*串口使能*/
    DMA_Enable();
    __HAL_UART_ENABLE(&huart4);

    //HAL_UART_Receive_IT(&huart7, Rxbuffer, sizeof(Rxbuffer));

    for (;;) {
        //printf("%d,%d,%d\n", (int)Core_xy[0].Motor_X->speedPID.output, Core_xy[0].gantry_t.velocity.x, Core_xy[0].Motor_X->AxisData.AxisVelocity);
        printf("%d,%d\n", LidarData[0][0].distance, LidarData[1][0].distance);
        //uint16_t flag = 1;
        //flag=HAL_UART_GetError(&huart2);
        //printf("%d,%d\n", flag,1);
        osDelay(2);
    }
}







