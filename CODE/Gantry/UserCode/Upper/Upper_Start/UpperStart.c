
#include "UpperStart.h"



void StartDefaultTask(void *argument)
{
    /*初始化函数*/
    Core_xy_Motor_init();
    /*开启线程*/
    Area1_StateMachine_Start();
    Area2_StateMachine_Start();
    Area3_StateMachine_Start();
    Upper_Servo_Start();
    UsartUpdate_Start();
    
    /*串口使能*/
    Usart_start();
    

    for (;;) {

        printf("%f,%f,%f,%f,%d,%d\n", Lidar2.distance_aver, Core_xy[0].Motor_Y->FdbData.rpm, Core_xy[0].Motor_Y->speedPID.ref, Lidar1.distance_aver, *debug_state, debug_time);

        // printf("%f,%f,%f\n",
        //        Lidar2.distance_aver,
               
        //        Core_xy[0].Motor_Y->speedPID.output,
               
        //        Core_xy[0].Motor_Y->FdbData.rpm);
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

        osDelay(50);
    }
}







