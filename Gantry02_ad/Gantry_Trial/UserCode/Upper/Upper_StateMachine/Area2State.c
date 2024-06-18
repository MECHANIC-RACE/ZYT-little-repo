#include "Area2State.h"

#define X_maxvelocity  5000 // 
#define Y_maxvelocity  6000 // du/s
#define X_Acceleration 1000
#define Y_Acceleration 3000

#define X_offset       780
#define Y_offset01     -135 // fama
#define Y_offset02     95   // muzhuang

uint16_t detect02xtree;
uint16_t detect02ytree;
uint16_t detect02_weight;

float initial_pos02[2];
float current_pos02[2];

/****************线程相关函数********************/

void Area2_State_Task(void *argument)
{
    inner_ring_flag = 1;
    osDelay(100);
    uint16_t stateflag = 0;
    uint16_t statechoose = Check_LidarStatus(Lidar3, Lidar6);
    for (;;) {
        /*用于一个分区的路径规划*/
    if(1){      /*雷达状态正常*/
        if (stateflag == 0) // 走到即将取第一个砝码之前
        {
                Core_xy[1].gantry_t.position.y = -4200;  //-2824.0
                TickType_t StartTick           = xTaskGetTickCount();
                initial_pos02[1]                 = Core_xy[1].Motor_Y->AxisData.AxisAngle_inDegree;
                _Bool isArray1                 = 0;
                float diff[1]                  = {0};
                detect02_weight                  = 1;
                detect02ytree                    = 1;
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_pos02[1], 4000, 1000, Core_xy[1].gantry_t.position.y, current_time, &(current_pos02[1]));
                    diff[0] = fabs(Core_xy[1].gantry_t.position.y - current_pos02[1]);
                    if ((diff[0] < 0.01) ) { isArray1 = 1; }

                } while (!isArray1);
                stateflag = 1;

        } else if (stateflag == 1){
            if(detect02_weight==0){
                Core_xy[1].gantry_t.position.y = -1 * Y_offset01 + angle_memory02weight;
            }else{
                Core_xy[1].gantry_t.position.y = -2824.0;
            }
            TickType_t StartTick = xTaskGetTickCount();
            initial_pos02[1]     = Core_xy[1].Motor_Y->AxisData.AxisAngle_inDegree;
            _Bool isArray1       = 0;
            float diff[1]        = {0};
            detect02_weight      = 1;
            detect02ytree        = 1;
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos02[1], 4000, 1000, Core_xy[1].gantry_t.position.y, current_time, &(current_pos02[1]));
                diff[0] = fabs(Core_xy[1].gantry_t.position.y - current_pos02[1]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(50);
            stateflag = 2;
        }
        else if (stateflag == 2) // 放下气缸，打开磁铁，往前拖行，吸起砝码，提上气缸
        {
                pid_reset(&(Core_xy[1].Motor_X->speedPID), 0, 0, 0);
                pid_reset(&(Core_xy[1].Motor_Y->speedPID), 0, 0, 0);
                HAL_GPIO_WritePin(Cylinder02_GPIO_Port, Cylinder02_Pin, 1);
                HAL_GPIO_WritePin(Electromagnet02_GPIO_Port, Electromagnet02_Pin, 1); // 放下气缸，打开电磁铁

                osDelay(100);

                pid_reset(&(Core_xy[1].Motor_X->speedPID), 5, 0.4, 0.8);

                if (inner_ring_flag == 0) { Core_xy[1].gantry_t.position.x = -200; } // 往前拖行一段
                else {
                    Core_xy[1].gantry_t.position.x = -1929;
                } // 往前拖行一段

                TickType_t StartTick = xTaskGetTickCount();
                initial_pos02[0]     = Core_xy[1].Motor_X->AxisData.AxisAngle_inDegree;
                _Bool isArray1       = 0;
                float diff[1]        = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_pos02[0], 10000, 3000, Core_xy[1].gantry_t.position.x, current_time, &(current_pos02[0]));
                    diff[0] = fabs(Core_xy[1].gantry_t.position.x - current_pos02[0]);
                    if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(100);
                HAL_GPIO_WritePin(Cylinder02_GPIO_Port, Cylinder02_Pin, 0);
                /*GPIO_WRITE_PIN提起气缸*/
                pid_reset(&(Core_xy[1].Motor_Y->speedPID), 3.5, 0.3, 0.3);
                stateflag = 3;
        }

        /*前往木桩*/
        else if (stateflag == 3) {
            Core_xy[1].gantry_t.position.x = -18630.0;
            if(detect02ytree==0){
                Core_xy[1].gantry_t.position.y = -1 * Y_offset02 + angle_memory02weight;
            }else{
                Core_xy[1].gantry_t.position.y = -3748.0;
            }
            TickType_t StartTick           = xTaskGetTickCount();
            initial_pos02[0]                 = Core_xy[1].Motor_X->AxisData.AxisAngle_inDegree;
            initial_pos02[1]                 = Core_xy[1].Motor_Y->AxisData.AxisAngle_inDegree;
            _Bool isArray2                 = 0;
            float diff[2]                  = {0};
            detect02xtree                    = 1;

            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos02[0], X_maxvelocity, X_Acceleration, Core_xy[1].gantry_t.position.x, current_time, &(current_pos02[0]));
                VelocityPlanning(initial_pos02[1], Y_maxvelocity, Y_Acceleration, Core_xy[1].gantry_t.position.y, current_time, &(current_pos02[1]));
                diff[0] = fabs(Core_xy[1].gantry_t.position.x - current_pos02[0]);
                diff[1] = fabs(Core_xy[1].gantry_t.position.y - current_pos02[1]);
                if ((diff[0] < 0.01) && (diff[1] < 0.01)) { isArray2 = 1; }

            } while (!isArray2);
            stateflag = 4;
        } else if (stateflag == 4) {
            osDelay(100);
            if (detect02xtree == 0) {
                Core_xy[1].gantry_t.position.x = angle_memory02xtree + (-1 * X_offset);
                TickType_t StartTick           = xTaskGetTickCount();
                initial_pos02[0]               = Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
                _Bool isArray1                 = 0;
                float diff[1]                  = {0};
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_pos02[0], X_maxvelocity, X_Acceleration, Core_xy[1].gantry_t.position.x, current_time, &(current_pos02[0]));
                    diff[0] = fabs(Core_xy[1].gantry_t.position.x - current_pos02[0]);
                    if ((diff[0] < 0.01)) { isArray1 = 1; }

                } while (!isArray1);
                osDelay(100);
            }
            osDelay(100);
            pid_reset(&(Core_xy[1].Motor_X->speedPID), 0, 0, 0);
            pid_reset(&(Core_xy[1].Motor_Y->speedPID), 0, 0, 0);
            // HAL_GPIO_WritePin(Cylinder02_GPIO_Port, Cylinder02_Pin, 1);
            osDelay(50);
            HAL_GPIO_WritePin(Electromagnet02_GPIO_Port, Electromagnet02_Pin, 0);
            stateflag = 5;
        } else if (stateflag == 5) {
            pid_reset(&(Core_xy[1].Motor_X->speedPID), 5, 0.4, 0.8);

            Core_xy[1].gantry_t.position.x = -18000;

            TickType_t StartTick = xTaskGetTickCount();
            initial_pos02[0]       = Core_xy[1].Motor_X->AxisData.AxisAngle_inDegree;
            _Bool isArray1       = 0;
            float diff[1]        = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos02[0], X_maxvelocity, X_Acceleration, Core_xy[1].gantry_t.position.x, current_time, &(current_pos02[0]));
                diff[0] = fabs(Core_xy[1].gantry_t.position.x - current_pos02[0]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            pid_reset(&(Core_xy[1].Motor_X->speedPID), 0, 0, 0);
            stateflag = 6;
        }
        // osDelay(2);
    }else{
        /*雷达状态异常情况*/
    }
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
