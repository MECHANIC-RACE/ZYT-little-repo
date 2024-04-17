
#include "UpperStart.h"



void StartDefaultTask(void *argument)
{
/*初始化函数*/
    Core_XY_StateMachine_Init();
    Core_xy_Motor_init();
/*开启线程*/
    Core_xy_StateMachine_Start();
    Upper_Servo_Start();

    for (;;) {

        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        //HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        osDelay(100);
    }
}
