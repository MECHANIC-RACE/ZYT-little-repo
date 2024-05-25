#include "Area3State.h"
uint16_t *debug_state03;
uint32_t debug_time03;
void Area3_State_Task(void *argument)
{
    // inner_ring_flag = 1;
    osDelay(100);
    uint16_t stateflag   = 0;
    TickType_t StartTick = xTaskGetTickCount();
    TickType_t tick1     = 0;
    TickType_t tick2     = 0;
    TickType_t tick3     = 0;

    debug_state03 = &stateflag;

    for (;;) {
        if (stateflag == 0) 
        {
            HAL_GPIO_WritePin(Cylinder03_GPIO_Port, Cylinder03_Pin, 1);
            HAL_GPIO_WritePin(Electromagnet03_GPIO_Port, Electromagnet03_Pin, 1); // 放下气缸，打开电磁铁
            Core_xy[1].gantry_t.position.x = 141;
            tick1                          = xTaskGetTickCount();
            uint32_t gap                   = (tick1 - StartTick) / 1000;
            debug_time03                   = gap;

            if (fabs(Lidar4.distance_aver - Core_xy[1].gantry_t.position.y) < 2 || gap > 8)
                stateflag = 1;
            
        } else if (stateflag == 1) 
        {
            pid_reset(&(Core_xy[2].Motor_X->speedPID), 0, 0, 0);

            HAL_GPIO_WritePin(Cylinder03_GPIO_Port, Cylinder03_Pin, 0);
            osDelay(100);

            pid_reset(&(Core_xy[0].Motor_X->speedPID), 5, 0.4, 0.8);                       
            stateflag = 2;
        }else if (stateflag == 2)
        {
            Core_xy[1].gantry_t.position.x = 141;
            tick2                          = xTaskGetTickCount();
            uint32_t gap                   = (tick2 - tick1) / 1000;
            debug_time03                   = gap;

            if (fabs(Lidar4.distance_aver - Core_xy[1].gantry_t.position.y) < 2 || gap > 8)
                stateflag = 3;
        }else if (stateflag == 3)
        {
            osDelay(100);
            pid_reset(&(Core_xy[2].Motor_X->speedPID), 0, 0, 0);
            osDelay(100);
            HAL_GPIO_WritePin(Electromagnet03_GPIO_Port, Electromagnet03_Pin, 0);
        }
        osDelay(2);
    }
}

void Area3_StateMachine_Start(void)
{
    osThreadId_t Area3_StateHandle;
    const osThreadAttr_t Area3_State_attributes = {
        .name       = "Area3_State",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityAboveNormal,
    };

    Area3_StateHandle = osThreadNew(Area3_State_Task, NULL, &Area3_State_attributes);
}
