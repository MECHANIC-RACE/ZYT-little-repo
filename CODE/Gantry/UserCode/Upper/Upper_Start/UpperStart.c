
#include "UpperStart.h"



void StartDefaultTask(void *argument)
{
    /*初始化函数*/
    // Core_XY_StateMachine_Init();
    // Core_xy_Motor_init();
    // /*开启线程*/
    // Core_xy_StateMachine_Start();
    // Upper_Servo_Start();
    UsartUpdate_Start();
    /*开启队列*/
    //Queue_Init();
    /*串口使能*/
    DMA_Enable();
    __HAL_UART_ENABLE(&huart4);

    

    for (;;) {
        
        printf("%d,%d\n", LidarData[2][0].distance, LidarData[1][0].distance);
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        // 创建 DMA 句柄结构体
        // 使用 HAL 库提供的函数获取 DMA 状态信息
        //HAL_DMA_StateTypeDef dma_state3 = HAL_DMA_GetState((&huart3)->hdmarx);
        // HAL_DMA_StateTypeDef dma_state2 = HAL_DMA_GetState((&huart2)->hdmarx);
        //printf("%d,%d,%d,%d\n", dma_state3, dma_state2, __HAL_DMA_GET_COUNTER((&huart3)->hdmarx), __HAL_DMA_GET_COUNTER((&huart2)->hdmarx));
        // printf("%d,%d\n", dma_state2, __HAL_DMA_GET_COUNTER((&huart2)->hdmarx));

        // uint16_t flag = 1;
        //printf("%d",Rxbuffer);
        // flag=HAL_UART_GetError(&huart2);
        // printf("%d,%d\n", flag,1);
        osDelay(50);
    }
}







