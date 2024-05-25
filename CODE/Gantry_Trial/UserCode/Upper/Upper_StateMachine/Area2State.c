#include "Area2State.h"

#define X_maxvelocity  396 // 396  390-600都可，不过时间差不多，而且到后面就有点贴不上了
#define Y_maxvelocity  250 // 264.5
#define X_Acceleration 130
#define Y_Acceleration 50

float initial_pos02[2];
float current_pos02[2];

/****************线程相关函数********************/

void Area2_State_Task(void *argument)
{
    inner_ring_flag = 1;
    osDelay(100);
    uint16_t stateflag = 0;
    for (;;) {
        /*用于一个分区的路径规划*/

        if (stateflag == 0) // 走到即将取第一个砝码之前
        {
            if (inner_ring_flag == 0) // 在外圈
            {
                pid_reset(&(Core_xy[1].Motor_X->speedPID), 0, 0, 0);
                Core_xy[1].gantry_t.position.y = 141; // x->185
                TickType_t StartTick           = xTaskGetTickCount();
                initial_pos02[1]                 = Lidar2.distance_aver;
                _Bool isArray1                 = 0;
                float diff[1]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_pos02[1], Y_maxvelocity, Y_Acceleration, Core_xy[1].gantry_t.position.y, current_time, &(current_pos02[1]));
                    diff[0] = fabs(Core_xy[1].gantry_t.position.y - current_pos02[1]);
                    if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                stateflag = 1;
            } else {
                /*if flag=1 在里圈*/
                // pid_reset(&(Core_xy[1].Motor_X->speedPID), 0, 0, 0);

                Core_xy[1].gantry_t.position.x = 223.0; //->360
                Core_xy[1].gantry_t.position.y = 488.0;
                TickType_t StartTick           = xTaskGetTickCount();
                initial_pos02[0]                 = Lidar1.distance_aver;
                initial_pos02[1]                 = Lidar2.distance_aver;
                _Bool isArray1                 = 0;
                float diff[2]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_pos02[0], X_maxvelocity, 50, Core_xy[1].gantry_t.position.x, current_time, &(current_pos02[0]));
                    VelocityPlanning(initial_pos02[1], Y_maxvelocity, 50, Core_xy[1].gantry_t.position.y, current_time, &(current_pos02[1]));
                    diff[0] = fabs(Core_xy[1].gantry_t.position.x - current_pos02[0]);
                    diff[1] = fabs(Core_xy[1].gantry_t.position.y - current_pos02[1]);
                    if ((diff[0] < 0.01) && (diff[1] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                stateflag = 1;
            }
        } else if (stateflag == 1) // 放下气缸，打开磁铁，往前拖行，吸起砝码，提上气缸
        {
            pid_reset(&(Core_xy[1].Motor_X->speedPID), 0, 0, 0);
            pid_reset(&(Core_xy[1].Motor_Y->speedPID), 0, 0, 0);
            HAL_GPIO_WritePin(Cylinder01_GPIO_Port, Cylinder01_Pin, 1);
            HAL_GPIO_WritePin(Electromagnet01_GPIO_Port, Electromagnet01_Pin, 1); // 放下气缸，打开电磁铁

            osDelay(100);

            pid_reset(&(Core_xy[1].Motor_X->speedPID), 5, 0.4, 0.8);

            if (inner_ring_flag == 0) { Core_xy[1].gantry_t.position.x = 185; } // 往前拖行一段
            else {
                Core_xy[1].gantry_t.position.x = 325;
            } // 往前拖行一段

            TickType_t StartTick = xTaskGetTickCount();
            initial_pos02[0]       = Lidar1.distance_aver;
            _Bool isArray1       = 0;
            float diff[1]        = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos02[0], X_maxvelocity, 50, Core_xy[1].gantry_t.position.x, current_time, &(current_pos02[0]));
                diff[0] = fabs(Core_xy[1].gantry_t.position.x - current_pos02[0]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(100);
            HAL_GPIO_WritePin(Cylinder01_GPIO_Port, Cylinder01_Pin, 0);
            /*GPIO_WRITE_PIN提起气缸*/
            pid_reset(&(Core_xy[1].Motor_Y->speedPID), 3.5, 0.3, 0.3);
            stateflag = 2;
        }

        /*前往木桩*/
        else if (stateflag == 2) {
            Core_xy[1].gantry_t.position.x = 940.0;
            Core_xy[1].gantry_t.position.y = 551.0;
            TickType_t StartTick           = xTaskGetTickCount();
            initial_pos02[0]                 = Lidar1.distance_aver;
            initial_pos02[1]                 = Lidar2.distance_aver;
            _Bool isArray2                 = 0;
            float diff[2]                  = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos02[0], X_maxvelocity, 40, Core_xy[1].gantry_t.position.x, current_time, &(current_pos02[0]));
                VelocityPlanning(initial_pos02[1], Y_maxvelocity, 50, Core_xy[1].gantry_t.position.y, current_time, &(current_pos02[1]));
                diff[0] = fabs(Core_xy[1].gantry_t.position.x - current_pos02[0]);
                diff[1] = fabs(Core_xy[1].gantry_t.position.y - current_pos02[1]);
                if ((diff[0] < 0.01) && (diff[1] < 0.01)) { isArray2 = 1; }

            } while (!isArray2);
            stateflag = 3;
        } else if (stateflag == 3) {
            osDelay(200);
            pid_reset(&(Core_xy[1].Motor_X->speedPID), 0, 0, 0);
            pid_reset(&(Core_xy[1].Motor_Y->speedPID), 0, 0, 0);
            // HAL_GPIO_WritePin(Cylinder01_GPIO_Port, Cylinder01_Pin, 1);
            osDelay(50);
            HAL_GPIO_WritePin(Electromagnet01_GPIO_Port, Electromagnet01_Pin, 0);
            stateflag = 4;
        } else if (stateflag == 4) {
            pid_reset(&(Core_xy[1].Motor_X->speedPID), 5, 0.4, 0.8);

            Core_xy[1].gantry_t.position.x = 850;

            TickType_t StartTick = xTaskGetTickCount();
            initial_pos02[0]       = Lidar1.distance_aver;
            _Bool isArray1       = 0;
            float diff[1]        = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos02[0], X_maxvelocity, 50, Core_xy[1].gantry_t.position.x, current_time, &(current_pos02[0]));
                diff[0] = fabs(Core_xy[1].gantry_t.position.x - current_pos02[0]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            pid_reset(&(Core_xy[1].Motor_X->speedPID), 0, 0, 0);
        }
        // osDelay(2);
    }
}

void Area2_StateMachine_Start(void)
{
    osThreadId_t Area2_StateHandle;
    const osThreadAttr_t Area2_State_attributes = {
        .name       = "Area2_State",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };

    Area2_StateHandle = osThreadNew(Area2_State_Task, NULL, &Area2_State_attributes);
}
