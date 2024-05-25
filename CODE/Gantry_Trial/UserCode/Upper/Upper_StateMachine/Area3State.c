#include "Area3State.h"

#define X_maxvelocity  396 
#define X_Acceleration 130

float initial_pos03[1];
float current_pos03[1];

/****************线程相关函数********************/

void Area3_State_Task(void *argument)
{
    osDelay(100);
    uint16_t stateflag = 0;
    for (;;) {

        if (stateflag == 0) 
        {
            HAL_GPIO_WritePin(Cylinder03_GPIO_Port, Cylinder03_Pin, 1);
            HAL_GPIO_WritePin(Electromagnet03_GPIO_Port, Electromagnet03_Pin, 1); // 放下气缸，打开电磁铁

            Core_xy[2].gantry_t.position.x = 141; //?待定
            TickType_t StartTick           = xTaskGetTickCount();
            initial_pos03[0]               = Lidar4.distance_aver;
            _Bool isArray1                 = 0;
            float diff[1]                  = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos03[0], X_maxvelocity, X_Acceleration, Core_xy[2].gantry_t.position.x, current_time, &(current_pos03[0]));
                diff[0] = fabs(Core_xy[2].gantry_t.position.x - current_pos03[0]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                stateflag = 1;
           
                
        } else if (stateflag == 1) 
        {
            pid_reset(&(Core_xy[2].Motor_X->speedPID), 0, 0, 0);

            HAL_GPIO_WritePin(Cylinder03_GPIO_Port, Cylinder03_Pin, 0);
            osDelay(100);

            pid_reset(&(Core_xy[0].Motor_X->speedPID), 5, 0.4, 0.8);

            Core_xy[2].gantry_t.position.x = 141; //?待定
            TickType_t StartTick           = xTaskGetTickCount();
            initial_pos03[0]               = Lidar4.distance_aver;
            _Bool isArray1                 = 0;
            float diff[1]                  = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos03[0], X_maxvelocity, X_Acceleration, Core_xy[2].gantry_t.position.x, current_time, &(current_pos03[0]));
                diff[0] = fabs(Core_xy[2].gantry_t.position.x - current_pos03[0]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);

            osDelay(100);
            pid_reset(&(Core_xy[2].Motor_X->speedPID), 0, 0, 0);
            osDelay(100);
            HAL_GPIO_WritePin(Electromagnet03_GPIO_Port, Electromagnet03_Pin, 0); 

            // osDelay(2);
    }
        
    }
}

void Area3_StateMachine_Start(void)
{
    osThreadId_t Area3_StateHandle;
    const osThreadAttr_t Area3_State_attributes = {
        .name       = "Area3_State",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };

    Area3_StateHandle = osThreadNew(Area3_State_Task, NULL, &Area3_State_attributes);
}
