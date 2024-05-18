
#include "UpperStart.h"



void StartDefaultTask(void *argument)
{
/*初始化函数*/
    //Core_XY_StateMachine_Init();
    //Core_xy_Motor_init();
/*开启线程*/
    // Core_xy_StateMachine_Start();
    //Upper_Servo_Start();
    UsartUpdate_Start();
/*串口使能*/
    // __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    //uint16_t status=HAL_UART_Receive_IT(&huart1, Rxbuffer[0], sizeof(Rxbuffer[0]));
    // HAL_UART_Receive_IT(&huart1, &(Rxbuffer[0]), sizeof(Rxbuffer[0]));
    // __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    // HAL_UART_Receive_IT(&huart2, Rxbuffer[1], sizeof(Rxbuffer[1]));
    HAL_UART_Receive_IT(&huart2, &(Rxbuffer[1]), 195);
    __HAL_UART_ENABLE(&huart4);
    //printf("init:%d", status);

    // HAL_UART_Receive_IT(&huart7, Rxbuffer, sizeof(Rxbuffer));

    for (;;) {
        //printf("it %d\n", UartFlag[0]);

        printf("%d,%d\n", LidarData[0][0].distance, LidarData[1][0].distance);
        //printf("Rx %d", Rxbuffer[0][11]);
        // //printf("%d,%d,he\n", HAL_UART_GetState(&huart1), HAL_UART_GetState(&huart2));
        // HAL_UART_StateTypeDef status1 = HAL_UART_GetState(&huart1);
        // switch (status1) {
        //     case HAL_UART_STATE_READY:
        //         printf("1ready\n");
        //         break;
        //     case HAL_UART_STATE_BUSY_RX:
        //         printf("1Rxbusy\n");
        //         break;
        //     case HAL_UART_STATE_TIMEOUT:
        //         printf("1timeout\n");
        //         break;
        //     case HAL_UART_STATE_ERROR:
        //         printf("1error\n");
        //         break;
        //     default:
        //         printf("1nuknow\n");
        //         break;
        // }
        // HAL_UART_StateTypeDef status2 = HAL_UART_GetState(&huart2);
        // switch (status2) {
        //     case HAL_UART_STATE_READY:
        //         printf("2ready\n");
        //         break;
        //     case HAL_UART_STATE_BUSY_RX:
        //         printf("2Rxbusy\n");
        //         break;
        //     case HAL_UART_STATE_TIMEOUT:
        //         printf("2timeout\n");
        //         break;
        //     case HAL_UART_STATE_ERROR:
        //         printf("2error\n");
        //         break;
        //     default:
        //         printf("2nuknow\n");
        //         break;
        //}
        //printf("%d\n", (&huart1)->Lock);

        osDelay(100);
    }
}







