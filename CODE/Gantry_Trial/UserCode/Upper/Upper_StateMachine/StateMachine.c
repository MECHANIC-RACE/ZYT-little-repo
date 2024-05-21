
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
        
        
            if(stateflag==0)  //走到即将取第一个砝码之前
            {
                if (inner_ring_flag == 0) // 在外圈
                {
                    pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);
                    Core_xy[0].gantry_t.position.y = 138.5; // x->185
                    TickType_t StartTick           = xTaskGetTickCount();
                    initial_pos[1]                 = Lidar2.distance_aver;
                    _Bool isArray1                 = 0;
                    float diff[1]                  = {0};
                    do {
                        TickType_t CurrentTick = xTaskGetTickCount();
                        float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                        VelocityPlanning(initial_pos[1], 1000, 50, Core_xy[0].gantry_t.position.y, current_time, &(current_pos[1]));
                        diff[0] = fabs(Core_xy[0].gantry_t.position.y - current_pos[1]);
                        if ((diff[0] < 0.01)) { isArray1 = 1; }

                    } while (!isArray1);
                    stateflag = 1;
                } else {
                    /*if flag=1 在里圈*/
                    Core_xy[0].gantry_t.position.x = 308.0;  //->360
                    Core_xy[0].gantry_t.position.y = 498.0;
                    TickType_t StartTick = xTaskGetTickCount();
                    initial_pos[0]       = Lidar1.distance_aver;
                    initial_pos[1]       = Lidar2.distance_aver;
                    _Bool isArray1       = 0;
                    float diff[2]        = {0};
                    do {
                        TickType_t CurrentTick = xTaskGetTickCount();
                        float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                        VelocityPlanning(initial_pos[0], 10000, 100, Core_xy[0].gantry_t.position.x, current_time, &(current_pos[0]));
                        VelocityPlanning(initial_pos[1], 1000, 50, Core_xy[0].gantry_t.position.y, current_time, &(current_pos[1]));
                        diff[0] = fabs(Core_xy[0].gantry_t.position.x - current_pos[0]);
                        diff[1] = fabs(Core_xy[0].gantry_t.position.y - current_pos[1]);
                        if ((diff[0] < 0.01) && (diff[1] < 0.01)) { isArray1 = 1; }

                    } while (!isArray1);
                    stateflag = 1;
                }
            }
        else if(stateflag==1)       //放下气缸，打开磁铁，往前拖行，吸起砝码，提上气缸
        {
            pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);
            pid_reset(&(Core_xy[0].Motor_Y->speedPID), 0, 0, 0);
            HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, 1);   //放下气缸，打开电磁铁
            osDelay(1000);

            pid_reset(&(Core_xy[0].Motor_X->speedPID), 0.3, 0.01, 0.01);
            pid_reset(&(Core_xy[0].Motor_Y->speedPID), 0.3, 0.01, 0.01);

            if(inner_ring_flag==0)
                {Core_xy[0].gantry_t.position.x = 185;}       //往前拖行一段
            else
                {Core_xy[0].gantry_t.position.x = 360;}       //往前拖行一段

            TickType_t StartTick           = xTaskGetTickCount();
            initial_pos[0]                 = Lidar1.distance_aver;
            _Bool isArray1                 = 0;
            float diff[1]                  = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos[0], 1000, 50, Core_xy[0].gantry_t.position.x, current_time, &(current_pos[0]));
                diff[0] = fabs(Core_xy[0].gantry_t.position.x - current_pos[0]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(500);
            /*GPIO_WRITE_PIN提起气缸*/
            stateflag = 2;
        }
        
        /*前往木桩*/
        else if(stateflag==2)
        {
        Core_xy[0].gantry_t.position.x = 938.0;
        Core_xy[0].gantry_t.position.y = 575.0;
        TickType_t StartTick           = xTaskGetTickCount();
        initial_pos[0]                 = Lidar1.distance_aver;
        initial_pos[1]                 = Lidar2.distance_aver;
        _Bool isArray2 = 0;
        float diff[2] = {0};
        do {
            TickType_t CurrentTick = xTaskGetTickCount();
            float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
            VelocityPlanning(initial_pos[0], 10000, 40, Core_xy[0].gantry_t.position.x, current_time, &(current_pos[0]));
            VelocityPlanning(initial_pos[1], 1000, 50, Core_xy[0].gantry_t.position.y, current_time, &(current_pos[1]));
            diff[0] = fabs(Core_xy[0].gantry_t.position.x - current_pos[0]);
            diff[1] = fabs(Core_xy[0].gantry_t.position.y - current_pos[1]);
            if ((diff[0] < 0.01) && (diff[1] < 0.01)) { isArray2 = 1; }

        } while (!isArray2);
        stateflag = 3;
        }
        else if(stateflag==3)
        {
            pid_reset(&(Core_xy[0].Motor_X->speedPID), 0, 0, 0);
            pid_reset(&(Core_xy[0].Motor_Y->speedPID), 0, 0, 0);
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

void pid_reset(PID_t *pid,float kp,float ki,float kd)
{
    pid->KP = kp;
    pid->KI = ki;
    pid->KD = kd;
    pid->integral = 0;
}