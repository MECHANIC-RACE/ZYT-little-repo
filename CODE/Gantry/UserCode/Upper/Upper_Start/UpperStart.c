
#include "UpperStart.h"



void StartDefaultTask(void *argument)
{
    /*初始化函数*/
    Core_XY_StateMachine_Init();
    Core_xy_Motor_init();
    /*开启线程*/
    //Core_xy_StateMachine_Start();
    Upper_Servo_Start();
    UsartUpdate_Start();
    
    /*串口使能*/
    Usart_start();
    

    for (;;) {

        //printf("%f,%f,%f,%f\n", Lidar1.distance_aver, Lidar2.distance_aver, Core_xy[0].Motor_X->speedPID.output, Core_xy[0].Motor_Y->speedPID.output);

        printf("%f,%f,%f\n",
               Lidar2.distance_aver,
               
               Core_xy[0].Motor_Y->speedPID.output,
               
               Core_xy[0].Motor_Y->FdbData.rpm);
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

        osDelay(50);
    }
}







