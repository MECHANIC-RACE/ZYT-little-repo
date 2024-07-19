#include "Area2State.h"
osThreadId_t Area2_StateHandle;

#define X_maxvelocity  5000 // 
#define Y_maxvelocity  6000 // du/s
#define X_Acceleration 1000
#define Y_Acceleration 3000

#define X_offset       780
#define Y_offset01     -10 // fama
#define Y_offset02     100   // muzhuang

uint16_t detect02xtree;
uint16_t detect02ytree;
uint16_t detect02_weight;

uint16_t UseLidar02;

float initial_pos02[2];
float current_pos02[2];

/****************线程相关函数********************/

void Area2_State_Task(void *argument)
{
    //weight_placement[1] = 1;
    osDelay(100);
    uint16_t stateflag = 0;
    uint16_t statechoose = Check_LidarStatus(Lidar3, Lidar6);       //检查雷达状态的函数是失败的
    for (;;) {
    /*用于一个分区的路径规划*/
    if(1){      /*雷达状态正常*/
        if (stateflag == 0) 
        {
            if(weight_placement[1]==1){
                HAL_GPIO_WritePin(Cylinder02_GPIO_Port, Cylinder02_Pin, 1);
                osDelay(300);
                Core_xy[1].gantry_t.position.y = -2000;  //-2824.0
                TickType_t StartTick           = xTaskGetTickCount();
                initial_pos02[1]                 = Core_xy[1].Motor_Y->AxisData.AxisAngle_inDegree;
                _Bool isArray1                 = 0;
                float diff[1]                  = {0};
                detect02_weight                  = 1;       //打开查找砝码的标志位
                
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_pos02[1], Y_maxvelocity, Y_Acceleration, Core_xy[1].gantry_t.position.y, current_time, &(current_pos02[1]));
                    diff[0] = fabs(Core_xy[1].gantry_t.position.y - current_pos02[1]);
                    if ((diff[0] < 0.01) ) { isArray1 = 1; }

                } while (!isArray1);
            }else{
                detect02_weight = 2;
                osDelay(1000);
            }
                stateflag = 1;

        } else if (stateflag == 1) 
        {
            if(weight_placement[1]==1){
                Core_xy[1].gantry_t.position.y = -3200;  //-2824.0
                TickType_t StartTick           = xTaskGetTickCount();
                initial_pos02[1]                 = Core_xy[1].Motor_Y->AxisData.AxisAngle_inDegree;
                _Bool isArray1                 = 0;
                float diff[1]                  = {0};
                detect02_weight                  = 1;       //打开查找砝码的标志位
                
                do {
                    TickType_t CurrentTick = xTaskGetTickCount();
                    float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                    VelocityPlanning(initial_pos02[1], 500, 500, Core_xy[1].gantry_t.position.y, current_time, &(current_pos02[1]));
                    diff[0] = fabs(Core_xy[1].gantry_t.position.y - current_pos02[1]);
                    if ((diff[0] < 0.01) ) { isArray1 = 1; }

                } while (!isArray1);
            }else{
            }
                stateflag = 2;

        } else if (stateflag == 2){
            if(weight_placement[1]==1){
            if(detect02_weight==0){
                Core_xy[1].gantry_t.position.y = -1 * Y_offset01 + angle_memory02weight;
            }else{
                Core_xy[1].gantry_t.position.y = -2824.0;
            }
            TickType_t StartTick = xTaskGetTickCount();
            initial_pos02[1]     = Core_xy[1].Motor_Y->AxisData.AxisAngle_inDegree;
            _Bool isArray1       = 0;
            float diff[1]        = {0};
            detect02_weight      = 2;
            
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos02[1], 4000, 1000, Core_xy[1].gantry_t.position.y, current_time, &(current_pos02[1]));
                diff[0] = fabs(Core_xy[1].gantry_t.position.y - current_pos02[1]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            osDelay(50);
            }
            stateflag = 3;
        }
        else if (stateflag == 3) // 放下气缸，打开磁铁，往前拖行，吸起砝码，提上气缸
        {
                pid_reset(&(Core_xy[1].Motor_X->speedPID), 0, 0, 0);
                pid_reset(&(Core_xy[1].Motor_Y->speedPID), 0, 0, 0);
                HAL_GPIO_WritePin(Cylinder02_GPIO_Port, Cylinder02_Pin, 0);
                HAL_GPIO_WritePin(Electromagnet02_GPIO_Port, Electromagnet02_Pin, 1); // 放下气缸，打开电磁铁

                osDelay(100);

                pid_reset(&(Core_xy[1].Motor_X->speedPID), 5, 0.4, 0.8);

                if (weight_placement[1] == 0) { Core_xy[1].gantry_t.position.x = -600; } // 往前拖行一段
                else {
                    Core_xy[1].gantry_t.position.x = -2200;
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
                osDelay(200);
                HAL_GPIO_WritePin(Cylinder02_GPIO_Port, Cylinder02_Pin, 1);
                /*GPIO_WRITE_PIN提起气缸*/
                pid_reset(&(Core_xy[1].Motor_Y->speedPID), 3.5, 0.3, 0.3);
                stateflag = 4;
        }

        /*前往木桩*/
        else if (stateflag == 4) {
            Core_xy[1].gantry_t.position.x = -17730.0;
            Core_xy[1].gantry_t.position.y = -2824;
            TickType_t StartTick           = xTaskGetTickCount();
            initial_pos02[0]                 = Core_xy[1].Motor_X->AxisData.AxisAngle_inDegree;
            initial_pos02[1]                 = Core_xy[1].Motor_Y->AxisData.AxisAngle_inDegree;
            _Bool isArray2                 = 0;
            float diff[2]                  = {0};

            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos02[0], X_maxvelocity, X_Acceleration, Core_xy[1].gantry_t.position.x, current_time, &(current_pos02[0]));
                VelocityPlanning(initial_pos02[1], Y_maxvelocity, Y_Acceleration, Core_xy[1].gantry_t.position.y, current_time, &(current_pos02[1]));
                diff[0] = fabs(Core_xy[1].gantry_t.position.x - current_pos02[0]);
               
                if (diff[0] < 0.01 && diff[1] < 0.01)  { isArray2 = 1; }

            } while (!isArray2);
            stateflag = 5;
        } else if (stateflag == 5) {
            osDelay(100);
            
            Core_xy[1].gantry_t.position.y = -4200;
            TickType_t StartTick           = xTaskGetTickCount();
            initial_pos02[0]               = Core_xy[1].Motor_Y->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
            _Bool isArray1                 = 0;
            float diff[1]                  = {0};
            detect02ytree                  = 1;
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos02[0], 500, 500, Core_xy[1].gantry_t.position.y, current_time, &(current_pos02[1]));
                diff[0] = fabs(Core_xy[1].gantry_t.position.y - current_pos02[1]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }
            } while (!isArray1);  
            osDelay(50);
            stateflag = 6;
        } else if (stateflag == 6) {
            if(detect02ytree==0){
                Core_xy[1].gantry_t.position.y = angle_memory02ytree - Y_offset02;
            }else{
                Core_xy[1].gantry_t.position.y = -3764; 
            }
            initial_pos02[1]     = Core_xy[1].Motor_Y->AxisData.AxisAngle_inDegree;
            TickType_t StartTick = xTaskGetTickCount();
            _Bool isArray1       = 0;
            float diff[1]        = {0};
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                VelocityPlanning(initial_pos02[1], Y_maxvelocity, Y_Acceleration, Core_xy[1].gantry_t.position.y, current_time, &(current_pos02[1]));
                diff[0] = fabs(Core_xy[1].gantry_t.position.y - current_pos02[1]);
                if ((diff[0] < 0.01)) { isArray1 = 1; }

            } while (!isArray1);
            pid_reset(&(Core_xy[1].Motor_Y->speedPID), 0, 0, 0);
            stateflag = 7;
        }else if (stateflag == 7)
        {
            if (Lidar6.distance_aver != 0 && Lidar6.distance_aver<350) {
                UseLidar02 = 1;
                Core_xy[1].gantry_t.position.x = 64;
                initial_pos02[0]               = Lidar6.distance_aver;
                pid_reset(&(Core_xy[1].Motor_X->posPID), 150, 0, 0);

            } else {
                Core_xy[1].gantry_t.position.x = -18400;        //待验证
                initial_pos02[0]               = Core_xy[1].Motor_X->AxisData.AxisAngle_inDegree;
            }
            TickType_t StartTick = xTaskGetTickCount();
            
            _Bool isArray2 = 0;
            float diff[1]  = {0};          
            do {
                TickType_t CurrentTick = xTaskGetTickCount();
                float current_time     = (CurrentTick - StartTick) * 1.0 / 1000.0;
                if(UseLidar02==1)    VelocityPlanning(initial_pos02[0],396, 130, Core_xy[1].gantry_t.position.x, current_time, &(current_pos02[0]));
                else    VelocityPlanning(initial_pos02[0], 4000, 500, Core_xy[1].gantry_t.position.x, current_time, &(current_pos02[0]));
                diff[0] = fabs(Core_xy[1].gantry_t.position.x - current_pos02[0]);
                if (diff[0] < 0.01) { isArray2 = 1; }
            } while (!isArray2);
            osDelay(500);
            pid_reset(&(Core_xy[1].Motor_Y->speedPID), 0, 0, 0);
            osDelay(1000);
            HAL_GPIO_WritePin(Electromagnet02_GPIO_Port, Electromagnet02_Pin, 0);
            stateflag = 8;
        }else if(stateflag==8){
                pid_reset(&(Core_xy[1].Motor_X->speedPID), 5, 0.4, 0.8);
                pid_reset(&(Core_xy[1].Motor_X->posPID), 200, 0, 0);
                UseLidar02 = 0;
                Core_xy[1].gantry_t.position.x = -17200;

                TickType_t StartTick = xTaskGetTickCount();
                initial_pos02[0]     = Core_xy[1].Motor_X->AxisData.AxisAngle_inDegree; // 电机轴输出角度 单位 度°
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
                stateflag = 9;
        }
    }else{
        /*雷达状态异常情况*/
    }
    }
}

void Area2_StateMachine_Start(void)
{
    const osThreadAttr_t Area2_State_attributes = {
        .name       = "Area2_State",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };

    Area2_StateHandle = osThreadNew(Area2_State_Task, NULL, &Area2_State_attributes);
}
