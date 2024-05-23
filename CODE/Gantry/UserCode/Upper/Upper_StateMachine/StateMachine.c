
#include "StateMachine.h"

//CoreXYState TargetState[2];     
//两个分区的龙门的目标状态,来自上位机的视觉确认和固定的路径规划--->一个巨大的状态机
//其中的Velocity用来放speedpid的REF，非定量，可以给系数
/*备注：当前状态在全局变量Core_xy[2]中*/
//uint16_t MarchingState;
/****************线程相关函数********************/

void Core_xy_State_Task(void *argument)
{
    // inner_ring_flag = 1;
    osDelay(100);
    uint16_t stateflag = 0;
    for (;;) {
        /*用于一个分区的路径规划*/

        if (stateflag == 0) // 走到即将取第一个砝码之前
        {
            if (inner_ring_flag == 0) // 在外圈
            {
                pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);
                Core_xy[0].gantry_t.position.y = 141;
                if (fabs(Lidar2.distance_aver - Core_xy[0].gantry_t.position.y)<2)
                    stateflag = 1;
            } else {
                /*if flag=1 在里圈*/
                Core_xy[0].gantry_t.position.x = 223.0; //->360
                Core_xy[0].gantry_t.position.y = 498.0;
                if (fabs(Lidar2.distance_aver - Core_xy[0].gantry_t.position.y) < 2 && fabs(Lidar1.distance_aver - Core_xy[0].gantry_t.position.x) < 2)
                    stateflag = 1;
            }
        } else if (stateflag == 1) // 放下气缸，打开磁铁，往前拖行，吸起砝码，提上气缸
        {
            pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);
            pid_reset(&(Core_xy[0].Motor_Y->speedPID), 0, 0, 0);
            // HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, 1);
            // HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, 1);   //放下气缸，打开电磁铁
            stateflag = 2;
        } else if (stateflag == 2) {

            pid_reset(&(Core_xy[0].Motor_X->speedPID), 5, 0.4, 0.8);

            if (inner_ring_flag == 0) { Core_xy[0].gantry_t.position.x = 185; } // 往前拖行一段
            else {
                Core_xy[0].gantry_t.position.x = 325;
            } // 往前拖行一段

            if (fabs(Lidar1.distance_aver - Core_xy[0].gantry_t.position.x) < 2)
                stateflag = 3;

        }

        /*提起气缸，前往木桩*/
        else if (stateflag == 3) {
            HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, 0);
            /*GPIO_WRITE_PIN提起气缸*/
            pid_reset(&(Core_xy[0].Motor_Y->speedPID), 3.5, 0.3, 0.3);
           
            Core_xy[0].gantry_t.position.x = 940.0;
            Core_xy[0].gantry_t.position.y = 551.0;

            if (fabs(Lidar2.distance_aver - Core_xy[0].gantry_t.position.y) < 2 && fabs(Lidar1.distance_aver - Core_xy[0].gantry_t.position.x) < 2)
                    stateflag = 4;

        } else if (stateflag == 4) {
            pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);
            pid_reset(&(Core_xy[0].Motor_Y->speedPID), 0, 0, 0);
            // HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, 1);
            osDelay(1000);
            // HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, 0);
        }
        osDelay(2);
    }

        osDelay(2);
    }


void Core_xy_StateMachine_Start(void)
{
    osThreadId_t Core_xy_StateHandle;
    const osThreadAttr_t Core_xy_State_attributes = {
        .name       = "Core_xy_State",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityAboveNormal,
    };

    Core_xy_StateHandle = osThreadNew(Core_xy_State_Task, NULL, &Core_xy_State_attributes);
}

/*******封装函数***********/
void Core_XY_StateMachine_Init()
{
   
    Core_xy[0].gantry_t.xMutex_control = xSemaphoreCreateRecursiveMutex();
    Core_xy[1].gantry_t.xMutex_control = xSemaphoreCreateRecursiveMutex();
}

void pid_reset(PID_t *pid, float kp, float ki, float kd)
{
    pid->KP       = kp;
    pid->KI       = ki;
    pid->KD       = kd;
    pid->integral = 0;
    pid->output   = 0;
}