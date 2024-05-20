
#include "StateMachine.h"

float initial_pos[3];
float current_pos[3];
//CoreXYState TargetState[2];     
//两个分区的龙门的目标状态,来自上位机的视觉确认和固定的路径规划--->一个巨大的状态机
//其中的Velocity用来放speedpid的REF，非定量，可以给系数
/*备注：当前状态在全局变量Core_xy[2]中*/
//uint16_t MarchingState;
/****************线程相关函数********************/

void Core_xy_State_Task(void *argument)
{
    osDelay(100);
    uint16_t stateflag = 0;
    for (;;) {
        /*用于一个分区的路径规划*/
        if(stateflag==0)
        {
            /*if flag=0 在外圈*/
            // 此处TargetState[i].position.x/y 为预期的与雷达的距离  考虑分开给还是一起给  不过应该没有办法一起给 因为要做不同的路径
            Core_xy[0].gantry_t.position.x = 245.0;
            Core_xy[0].gantry_t.position.y = 245.0;
            /*if flag=1 在里圈*/
            // Core_xy[0].gantry_t.position.x = 1062.5;
            // Core_xy[0].gantry_t.position.y = 675.24;

            TickType_t StartTick = xTaskGetTickCount();
            initial_pos[0]       = Lidar1.distance_aver;
            initial_pos[1]       = Lidar2.distance_aver;
            _Bool isArray1       = 0;
            float diff[2] = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos[0], 2000, 50, Core_xy[0].gantry_t.position.x, current_time, &(current_pos[0]));
                VelocityPlanning(initial_pos[1], 1000, 50, Core_xy[0].gantry_t.position.y, current_time, &(current_pos[1]));
                diff[0] = fabs(Core_xy[0].gantry_t.position.x - current_pos[0]);
                diff[1] = fabs(Core_xy[0].gantry_t.position.y - current_pos[1]);
                if ((diff[0] < 0.01) && (diff[1] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            stateflag = 1;
        }
        else if(stateflag==1)
        {
            Core_xy[0].Motor_X->speedPID.KP = 0;
            Core_xy[0].Motor_X->speedPID.KI = 0;
            Core_xy[0].Motor_X->speedPID.KD = 0;
            Core_xy[0].Motor_Y->speedPID.KP = 0;
            Core_xy[0].Motor_Y->speedPID.KI = 0;
            Core_xy[0].Motor_Y->speedPID.KD = 0;
            HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, 1);
            osDelay(1000);
            Core_xy[0].Motor_X->speedPID.integral = 0;
            Core_xy[0].Motor_Y->speedPID.integral = 0;
            Core_xy[0].Motor_X->speedPID.KP     = 15;
            Core_xy[0].Motor_X->speedPID.KI = 2;
            Core_xy[0].Motor_X->speedPID.KD = 5;
            Core_xy[0].Motor_Y->speedPID.KP = 0;
            Core_xy[0].Motor_Y->speedPID.KI = 0;
            Core_xy[0].Motor_Y->speedPID.KD = 0;
            stateflag = 2;
        }
        /*前往木桩*/
        else if(stateflag==2)
        {
        Core_xy[0].gantry_t.position.x = 875.0;
        Core_xy[0].gantry_t.position.y = 550.0;
        TickType_t StartTick           = xTaskGetTickCount();
        initial_pos[0]                 = Lidar1.distance_aver;
        initial_pos[1]                 = Lidar2.distance_aver;
        _Bool isArray2 = 0;
        float diff[2] = {0};
        do {
            TickType_t CurrentTick = xTaskGetTickCount();
            float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
            VelocityPlanning(initial_pos[0], 2000, 50, Core_xy[0].gantry_t.position.x, current_time, &(current_pos[0]));
            VelocityPlanning(initial_pos[1], 1000, 50, Core_xy[0].gantry_t.position.y, current_time, &(current_pos[1]));
            diff[0] = fabs(Core_xy[0].gantry_t.position.x - current_pos[0]);
            diff[1] = fabs(Core_xy[0].gantry_t.position.y - current_pos[1]);
            if ((diff[0] < 0.01) && (diff[1] < 0.01)) { isArray2 = 1; }

        } while (!isArray2);
        stateflag = 3;
        }
        else if(stateflag==3)
        {
            Core_xy[0].Motor_X->speedPID.KP = 0;
            Core_xy[0].Motor_X->speedPID.KI = 0;
            Core_xy[0].Motor_X->speedPID.KD = 0;
            Core_xy[0].Motor_Y->speedPID.KP = 0;
            Core_xy[0].Motor_Y->speedPID.KI = 0;
            Core_xy[0].Motor_Y->speedPID.KD = 0;
            HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, 1);
            osDelay(1000);
            HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, 0);
        }
        osDelay(2); 
    }
}

void Core_xy_StateMachine_Start(void)
{
    osThreadId_t Core_xy_StateHandle;
    const osThreadAttr_t Core_xy_State_attributes = {
        .name       = "Core_xy_State",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };

    Core_xy_StateHandle = osThreadNew(Core_xy_State_Task, NULL, &Core_xy_State_attributes);
}

/*******封装函数***********/
void Core_XY_StateMachine_Init()
{
   
    Core_xy[0].gantry_t.xMutex_control = xSemaphoreCreateRecursiveMutex();
    Core_xy[1].gantry_t.xMutex_control = xSemaphoreCreateRecursiveMutex();
}