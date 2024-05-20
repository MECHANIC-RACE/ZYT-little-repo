
#include "StateMachine.h"

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
        if(stateflag==0){
            /*if flag=0 在外圈*/
            // 此处TargetState[i].position.x/y 为预期的与雷达的距离  考虑分开给还是一起给  不过应该没有办法一起给 因为要做不同的路径
            Core_xy[0].gantry_t.position.x = 245.0;
            Core_xy[0].gantry_t.position.y = Lidar2.distance_aver;
            /*if flag=1 在里圈*/
            // Core_xy[0].gantry_t.position.x = 1062.5;
            // Core_xy[0].gantry_t.position.y = 675.24;
            if ((fabs(Core_xy[0].gantry_t.position.x - Lidar1.distance_aver)) < 2 && (fabs(Core_xy[0].gantry_t.position.y - Lidar2.distance_aver) < 1)) 
            {
                stateflag = 1;
            }
        }
        //到时这里再补一个状态
        HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, 1);
        osDelay(500);
        /*前往木桩*/

        if (stateflag == 1) {
            /*if flag=0 在外圈*/
            
            Core_xy[0].gantry_t.position.x = 875.0;
            Core_xy[0].gantry_t.position.y = Lidar2.distance_aver;
           
            if ((fabs(Core_xy[0].gantry_t.position.x - Lidar1.distance_aver)) < 2 && (fabs(Core_xy[0].gantry_t.position.y - Lidar2.distance_aver) < 1)) {
                stateflag = 2;
            }
        }
        HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, 1);
        //osDelay(5);
        HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, 0);

        osDelay(2);
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
   
    Core_xy[0].gantry_t.xMutex_control = xSemaphoreCreateRecursiveMutex();
    Core_xy[1].gantry_t.xMutex_control = xSemaphoreCreateRecursiveMutex();
}