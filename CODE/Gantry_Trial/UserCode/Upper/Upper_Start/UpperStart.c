
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

        printf("%f,%f,%f,%f,%f\n",
               Lidar1.distance_aver,
               current_pos[0],
               Core_xy[0].Motor_X->speedPID.output,
               Core_xy[0].Motor_X->posPID.output,
               Core_xy[0].Motor_X->FdbData.rpm);

        osDelay(50);
    }
}
