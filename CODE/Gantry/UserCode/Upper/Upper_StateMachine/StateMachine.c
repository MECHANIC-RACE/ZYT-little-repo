
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

     for (;;) {
            /*用于一个分区的路径规划*/
            // do {
            //     /*if flag=0 在外圈*/
            //     // 此处TargetState[i].position.x/y 为预期的与雷达的距离  考虑分开给还是一起给  不过应该没有办法一起给 因为要做不同的路径
            //     Core_xy[0].gantry_t.position.x = 875.0;
            //     Core_xy[0].gantry_t.position.y = 350.48;
            //     /*if flag=1 在里圈*/
            //     //Core_xy[0].gantry_t.position.x = 1062.5;
            //     //Core_xy[0].gantry_t.position.y = 675.24;

            // } while (1);
            // //while (fabs(Core_xy[0].gantry_t.position.x - distance_aver[0]) > 1 || fabs(Core_xy[0].gantry_t.position.y - distance_aver[1]) > 1);
            // HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, 1);
            // osDelay(5);
            // /*前往木桩*/

            // do {
            //     Core_xy[0].gantry_t.position.x = 245;
            //     Core_xy[0].gantry_t.position.y = 245;
            // } while (fabs(Core_xy[0].gantry_t.position.x - distance_aver[0]) > 1 || fabs(Core_xy[0].gantry_t.position.y - distance_aver[1]) > 1);
            // HAL_GPIO_WritePin(Cylinder_GPIO_Port, Cylinder_Pin, 1);
            // osDelay(5);
            // HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, 0);
            if(UartFlag[0])
            {
                STP_23L_Decode(0);
                UartFlag[0] = 0;
                HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rxbuffer[0], sizeof(Rxbuffer[0]));
            }
            if(UartFlag[1])
            {
                STP_23L_Decode(1);
                UartFlag[1] = 0;
                HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Rxbuffer[1], sizeof(Rxbuffer[0]));
            }
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