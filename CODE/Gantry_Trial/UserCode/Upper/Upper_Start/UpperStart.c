
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

        printf("%f,%f,%f,%f,%f\n",
               Lidar2.distance_aver,
               current_pos01[1],
               Core_xy[0].Motor_Y->speedPID.output,
               Core_xy[0].Motor_Y->posPID.output,
               Core_xy[0].Motor_Y->FdbData.rpm);
        // printf("%f,%f,%f,%f\n",
        //        Lidar1.distance_aver,
        //        current_pos[0],
        //        Lidar2.distance_aver,
        //        current_pos[1] );

        

        osDelay(50);
    }
}
