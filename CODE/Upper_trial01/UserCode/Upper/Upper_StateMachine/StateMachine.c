
#include "StateMachine.h"

CoreXYState Target;
float initial_angle[3];
float current_angle[3];
//待赋值
// Target.position.x = 0;
// Target.position.y = 0;
// Target.position.z = 0;

/****************线程相关函数********************/


void Core_xy_State_Task(void *argument)
{
    //osDelay(1000);

     for (;;) {
    //     float REF[3];
    //     REF[0] = (Target.position.x) / BELT_LENGTH_PER_ROUND * 360.0f; // 所需要转的角度  单位：度
    //     REF[1] = (Target.position.y) / BELT_LENGTH_PER_ROUND * 360.0f;
    //     REF[2] = (Target.position.z) / BELT_LENGTH_PER_ROUND * 360.0f;
    //     TickType_t StartTick = xTaskGetTickCount();
    //     initial_angle[0] = Core_xy.Motor_X->AxisData.AxisAngle_inDegree;
    //     initial_angle[1] = Core_xy.Motor_Y->AxisData.AxisAngle_inDegree;
    //     initial_angle[2] = Core_xy.Motor_Z->AxisData.AxisAngle_inDegree;

    //     _Bool isArray         = 0;
    //     float diff[3]        = {0};
    //     do {
    //         TickType_t CurrentTick = xTaskGetTickCount();
    //         float current_time     = (CurrentTick - StartTick) / 1000.0;
    //         VelocityPlanning(initial_angle[0], 1000, 100, REF[0], current_time, &(current_angle[0]));
    //         VelocityPlanning(initial_angle[1], 1000, 100, REF[1], current_time, &(current_angle[1]));
    //         VelocityPlanning(initial_angle[2], 1000, 100, REF[2], current_time, &(current_angle[2]));
    //         for (uint16_t i = 0; i < 3; i++) { diff[i] = fabs(REF[i] - current_angle[i]); }
    //         if ((diff[0] < 0.1) && (diff[1] < 0.1) && (diff[2] < 1)) { isArray = 1; }

    // } while (!isArray);

        osDelay(5);
    }
    
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
    Target.xMutex_control = xSemaphoreCreateRecursiveMutex();
    Core_xy.Corexy_state.xMutex_control = xSemaphoreCreateRecursiveMutex();
    
}