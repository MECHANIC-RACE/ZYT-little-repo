
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
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_RXNE);
    HAL_UART_Receive_IT(&huart8, Rxbuffer, sizeof(Rxbuffer));

    for (;;) {

        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        //printf("%f,%f\n", current_angle[0], Core_xy.Motor_X->speedPID.output);
        //printf("%f,%f,%f,%f\n", Core_xy.Motor_X->AxisData.AxisVelocity, Core_xy.Motor_X->AxisData.AxisAngle_inDegree, Core_xy.Motor_X->speedPID.ref,Core_xy.Motor_X->speedPID.output);

        osDelay(100);
    }
}







