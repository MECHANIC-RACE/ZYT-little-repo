
#include "UsartUpdate.h"
#include "UpperStart.h"

uint16_t Uart_State = 0;
uint16_t detect01xtree_cnt;
uint16_t detect01ytree_cnt;
uint16_t detect01weight_cnt;
uint16_t detect02xtree_cnt;
uint16_t detect02ytree_cnt;
uint16_t detect02weight_cnt;
float angle_memory01xtree;
float angle_memory01ytree;
float angle_memory01weight;

float angle_memory02xtree;
float angle_memory02ytree;
float angle_memory02weight;
void UartUpdateTask(void *argument)
{
    /* USER CODE BEGIN UartUpdateTask */
    
    float weight_placement_tmp[5] = {0};
    int switch_flag               = 0; // 判断每次接收到的数组与基准数组是否相等
    int tar_count                 = 0; // 计数连续相同数组的次数
    // osDelay(100);


    /* Infinite loop */
    for (;;) {
        if(Uart_State==0)
        {
            if (UartFlag[5] == 1) {
                Upper_Target_Decode();
                for (int i = 0; i < 5; i++) {
                    weight_placement_tmp[i] = weight_placement[i];
                }
                UartFlag[5]   = 0;
                tar_count = 1; // 首次接收时，计数器初始化为1
                Uart_State    = 1;
            }
        }
        else if(Uart_State==1){
              
                    if (UartFlag[5] == 1) {
                        Upper_Target_Decode();
                        UartFlag[5]     = 0;
                        switch_flag = 0;

                        for (int i = 0; i < 5; i++) {
                            if (weight_placement_tmp[i] != weight_placement[i]) {
                                switch_flag = 1;
                                break;
                            }
                        }

                        // 收到的数组与基准数组相等
                        if (switch_flag == 0) {
                            tar_count++;
                        }
                        // 收到的数组与基准数组不相等
                        else {
                            tar_count = 1; // 重新计数
                            for (int i = 0; i < 5; i++) {
                                weight_placement_tmp[i] = weight_placement[i];
                            }
                        }

                        // 如果连续十次接收到同样的数组，则把这个数组设置为最终值
                        if (tar_count >= 10) {
                            Uart_State = 2;
                            __HAL_UART_DISABLE_IT(&huart5, UART_IT_RXNE);
                        }
                    }
                    osDelay(2);
                }
            
        
        else if(Uart_State==2){
        if (UartFlag[0]) {
            STP_23L_Decode(Rxbuffer_1,&Lidar1);
            if (detect01xtree==1 && Lidar1.distance_aver < 300 && Lidar1.distance_aver > 100) detect01xtree_cnt++;
            if(detect01xtree && detect01xtree_cnt==1)
            {
                detect01xtree = 0;
                angle_memory01xtree = Core_xy[0].Motor_X->AxisData.AxisAngle_inDegree;
            }
            UartFlag[0] = 0;
        }
        if (UartFlag[1]) {
            STP_23L_Decode(Rxbuffer_2,&Lidar2);
            if (detect01_weight == 1 && Lidar2.distance_aver < 300 && Lidar2.distance_aver > 100) detect01weight_cnt++;

            if (detect01_weight==1 && detect01weight_cnt == 3) {     //写1风险是比较大的
                detect01_weight       = 0;      //如果为0说明置数成功，否则就是没有识别上
                angle_memory01weight = Core_xy[0].Motor_Y->AxisData.AxisAngle_inDegree;
            }
            if (detect01ytree == 1 && Lidar2.distance_aver < 900 && Lidar2.distance_aver > 700) detect01ytree_cnt++;
            if (detect01ytree==1 && detect01ytree_cnt == 3)
            {
                detect01ytree = 0;
                angle_memory01ytree = Core_xy[0].Motor_Y->AxisData.AxisAngle_inDegree;
            }
            
            UartFlag[1] = 0;
        }
        if (UartFlag[2]) {
            STP_23L_Decode(Rxbuffer_3, &Lidar3);
            if (detect02xtree==1 && Lidar3.distance_aver < 300 && Lidar3.distance_aver > 100) detect02xtree_cnt++;
            if (detect02xtree && detect02xtree_cnt == 1) {
                detect02xtree       = 0;
                angle_memory02xtree = Core_xy[1].Motor_X->AxisData.AxisAngle_inDegree;
            }
            UartFlag[2] = 0;
        }
        if (UartFlag[3]) {
            STP_23L_Decode(Rxbuffer_6, &Lidar6);
            if (detect02_weight == 1 && Lidar6.distance_aver < 300 &&Lidar6.distance_aver>100) detect02weight_cnt++;

            if (detect02_weight == 1 && detect02weight_cnt == 1) { // 写1风险是比较大的
                detect02_weight      = 0;                          // 如果为0说明置数成功，否则就是没有识别上
                angle_memory02weight = Core_xy[1].Motor_Y->AxisData.AxisAngle_inDegree;
            }
            if (detect02ytree == 1 && Lidar6.distance_aver < 2400 && Lidar6.distance_aver > 2600) detect02ytree_cnt++;          /*这个范围内的数可能会很多，不知道可不可行*/
            if (detect02ytree == 1 && detect02ytree_cnt == 3) {
                detect02ytree       = 0;
                angle_memory02ytree = Core_xy[1].Motor_Y->AxisData.AxisAngle_inDegree;
            }
            UartFlag[3] = 0;
        }
        if (UartFlag[4]) {
            STP_23L_Decode(Rxbuffer_4, &Lidar4);
            UartFlag[4] = 0;
        }
        osDelay(2);
        }
    }
    /* USER CODE END UartUpdateTask */
}

void UsartUpdate_Start()
{
    osThreadId_t UsartUpdateHandle;
    const osThreadAttr_t UsartUpdate_attributes = {
        .name       = "UsartUpdate",
        .stack_size = 128 * 4,
        .priority   = (osPriority_t)osPriorityAboveNormal,
    };
    UsartUpdateHandle = osThreadNew(UartUpdateTask, NULL, &UsartUpdate_attributes);
}

void Usart_start()
{
    HAL_UART_Receive_IT(&huart1, usart1_rx, 1);
    HAL_UART_Receive_IT(&huart2, usart2_rx, 1);
    HAL_UART_Receive_IT(&huart3, usart3_rx, 1);
    HAL_UART_Receive_IT(&huart6, usart6_rx, 1);
    HAL_UART_Receive_IT(&huart4, usart4_rx, 1);

    // HAL_UART_Receive_DMA(&huart1, usart1_rx, 1);
    // HAL_UART_Receive_DMA(&huart2, usart2_rx, 1);
    // HAL_UART_Receive_DMA(&huart3, usart3_rx, 1);
    // HAL_UART_Receive_DMA(&huart6, usart6_rx, 1);
    /*2，3，6串口的使能函数*/
    __HAL_UART_ENABLE(&huart5);//到时串口5将被用作树莓派接收
   
}


void RaspReceive_Enable()
{
    HAL_UART_Receive_IT(&huart5, receive_buffer, sizeof(receive_buffer));
}

uint16_t Check_LidarStatus(LidarPointTypedef lidara, LidarPointTypedef lidarb)
{
    uint16_t status1 = 0;
    uint16_t status2 = 0;
    float a[4]={0};
    float b[4]={0};
    a[0]             = lidara.distance_aver;
    b[0]             = lidarb.distance_aver;
    osDelay(3);
    a[1] = lidara.distance_aver;
    b[1] = lidarb.distance_aver;
    osDelay(3);
    a[2] = lidara.distance_aver;
    b[2] = lidarb.distance_aver;
    osDelay(3);
    a[3] = lidara.distance_aver;
    b[3] = lidarb.distance_aver;
    if (a[0] == a[1] && a[1] == a[2] && a[2] == a[3]) status1 = 1;
    if (b[0] == b[1] && b[1] == b[2] && b[2] == b[3]) status2 = 1;
    if (status1 || status2) return 0;        //错误状态
    else
        return 1;                            //OK状态
}