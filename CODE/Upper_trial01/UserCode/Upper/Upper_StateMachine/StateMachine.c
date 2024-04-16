
#include "StateMachine.h"

CoreXYState Target;
//待赋值
//Target.position.x=
//Target.position.y=
//Target.position.z=

/****************线程相关函数********************/


void Core_xy_State_Task(void *argument)
{
    //osDelay(1000);

    for (;;) {

        /*状态机线程，负责接收遥控/上位机的数据并进行一定的处理比如死区等*/

        osDelay(1);
    }
    
}

void Core_xy_StateMachine_Start(void)
{
    osThreadId_t Core_xy_StateHandle;
    const osThreadAttr_t Core_xy_State_attributes = {
        .name       = "Core_xy_State",
        .stack_size = 128 * 4,
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