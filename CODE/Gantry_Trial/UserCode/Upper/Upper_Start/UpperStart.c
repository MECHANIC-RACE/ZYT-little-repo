
#include "UpperStart.h"

void StartDefaultTask(void *argument)
{
    /*初始化函数*/
    Core_XY_StateMachine_Init();
    Core_xy_Motor_init();
    /*开启线程*/
    Core_xy_StateMachine_Start();
    Upper_Servo_Start();
    UsartUpdate_Start();
    // osDelay(1000);
    // HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, 1);
    // osDelay(100);
    //HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, 1);
    //HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, 0);
    // HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, 0);
    /*串口使能*/
    Usart_start();

    for (;;) {

        printf("%f,%f,%f,%f,%f\n",
               Lidar2.distance_aver,
               current_pos[1],
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
