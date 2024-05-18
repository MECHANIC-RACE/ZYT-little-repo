
#include "StateMachine.h"

float initial_angle[3];
float current_angle[3];
//CoreXYState TargetState[2];     
//两个分区的龙门的目标状态,来自上位机的视觉确认和固定的路径规划--->一个巨大的状态机
//其中的Velocity用来放speedpid的REF，非定量，可以给系数
/*备注：当前状态在全局变量Core_xy[2]中*/
//uint16_t MarchingState;
/****************线程相关函数********************/

void Core_xy_State_Task(void *argument)
{
    osDelay(100);

     for (;;) {
            /*用于一个分区的路径规划*/
            
            //     /*if flag=0 在外圈*/
            //     // 此处TargetState[i].position.x/y 为预期的与雷达的距离  考虑分开给还是一起给  不过应该没有办法一起给 因为要做不同的路径
            //     Core_xy[0].gantry_t.position.x = 875.0;
            //     Core_xy[0].gantry_t.position.y = 350.48;
            //     /*if flag=1 在里圈*/
            //     // Core_xy[0].gantry_t.position.x = 1062.5;
            //     // Core_xy[0].gantry_t.position.y = 675.24;

            //     float REF[3];
            //     REF[0]               = (Core_xy[0].gantry_t.position.x) / BELT_LENGTH_PER_ROUND * 360.0f; // 所需要转的角度  单位：度
            //     REF[1]               = (Core_xy[0].gantry_t.position.y) / BELT_LENGTH_PER_ROUND * 360.0f;

            //     TickType_t StartTick = xTaskGetTickCount();

            //     initial_angle[0] = Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree;
            //     initial_angle[1] = Core_xy[0].Motor_Y->AxisData.AxisAngle_inDegree;
    
            //     _Bool isArray = 0;
            //     float diff[3] = {0};
            //     //do {
            //         TickType_t CurrentTick = xTaskGetTickCount();
            //         float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
            //         VelocityPlanning(initial_angle[0], 1000, 500, REF[0], current_time, &(current_angle[0]));
            //         VelocityPlanning(initial_angle[1], 1000, 500, REF[1], current_time, &(current_angle[1]));
            //         for (uint16_t i = 0; i < 3; i++) { diff[i] = fabs(REF[i] - current_angle[i]); }
            //         if ((diff[0] < 0.1) && (diff[1] < 0.1) && (diff[2] < 0.1)) { isArray = 1; }
                    
            //     //} while (!isArray);
           
            // HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, 1);
            // osDelay(5);
            //     /*前往木桩*/

            // //do {
            //     Core_xy[0].gantry_t.position.x = 245;
            //     Core_xy[0].gantry_t.position.y = 245;
            // //} while (fabs(Core_xy[0].gantry_t.position.x - distance_aver[0]) > 1 || fabs(Core_xy[0].gantry_t.position.y - distance_aver[1]) > 1);
            // HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, 1);
            // osDelay(5);
            // HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, 0);
            
            osDelay(200);

            
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