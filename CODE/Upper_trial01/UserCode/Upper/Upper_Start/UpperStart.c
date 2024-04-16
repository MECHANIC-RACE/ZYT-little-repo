
#include "UpperStart.h"



void StartDefaultTask(void *argument)
{

    Core_XY_StateMachine_Init();
    Core_xy_Motor_init();

    for (;;) {
        osDelay(1);
    }
}
