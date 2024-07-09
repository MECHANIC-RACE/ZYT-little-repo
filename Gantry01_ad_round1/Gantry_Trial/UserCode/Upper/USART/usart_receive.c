
#include "UpperStart.h"
#include "stm32f4xx_it.h"

/**************STP_23L********************/
uint8_t Rxbuffer_1[195];
uint8_t Rxbuffer_2[195];
uint8_t Rxbuffer_3[195];
uint8_t Rxbuffer_4[195];
uint8_t Rxbuffer_6[195];

LidarPointTypedef Lidar1;
LidarPointTypedef Lidar2;
LidarPointTypedef Lidar3;
LidarPointTypedef Lidar4;
LidarPointTypedef Lidar6;

uint16_t UartFlag[6];

uint8_t usart1_rx[1];
uint8_t usart2_rx[1];
uint8_t usart3_rx[1];
uint8_t usart4_rx[1];
uint8_t usart6_rx[1];
/*********************STP_23L***********************/
/*********************Ras_pi************************/
uint8_t receive_buffer[24];
float weight_placement[5] = {0};
void Upper_Target_Decode();
/*********************Ras_pi************************/

uint16_t inner_ring_flag;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        static uint16_t u1state = 0; //状态机计数
        static uint16_t crc1    = 0; //校验和
        uint8_t tmp1   = usart1_rx[0];
        if (u1state < 4)
        {
            if (tmp1 == 0xAA)
            {
                Rxbuffer_1[u1state] = tmp1;
                u1state++;
                
            } else {
                u1state = 0;
            }
        } else if (u1state < 194) {
            Rxbuffer_1[u1state] = tmp1;
            u1state++;
            crc1 += tmp1;
        }else if(u1state==194){
            Rxbuffer_1[u1state] = tmp1;
            if (tmp1 == crc1 % 256) 
            {
                UartFlag[0] = 1;
            }
            u1state = 0;
            crc1    = 0;
        } else {
        };

        HAL_UART_Receive_IT(&huart1, usart1_rx, 1);
    }


        if (huart->Instance == USART2) {
            static uint16_t u2state = 0; // 状态机计数
            static uint16_t crc2    = 0; // 校验和
            uint8_t tmp2            = usart2_rx[0];
            if (u2state < 4) {
                if (tmp2 == 0xAA) {
                    Rxbuffer_2[u2state] = tmp2;
                    u2state++;

                } else {
                    u2state = 0;
                }
            } else if (u2state < 194) {
                Rxbuffer_2[u2state] = tmp2;
                u2state++;
                crc2 += tmp2;
            } else if (u2state == 194) {
                Rxbuffer_2[u2state] = tmp2;
                if (tmp2 == crc2 % 256) {
                    UartFlag[1] = 1;
                }
                u2state = 0;
                crc2    = 0;
            } else {
            };
            HAL_UART_Receive_IT(&huart2, usart2_rx, 1);
        }
    if (huart->Instance == USART3) {
        static uint16_t u3state = 0; // 状态机计数
        static uint16_t crc3    = 0; // 校验和
        uint8_t tmp3            = usart3_rx[0];
        if (u3state < 4) {
            if (tmp3 == 0xAA) {
                Rxbuffer_3[u3state] = tmp3;
                u3state++;

            } else {
                u3state = 0;
            }
        } else if (u3state < 194) {
            Rxbuffer_3[u3state] = tmp3;
            u3state++;
            crc3 += tmp3;
        } else if (u3state == 194) {
            Rxbuffer_3[u3state] = tmp3;
            if (tmp3 == crc3 % 256) {
                UartFlag[2] = 1;
            }
            u3state = 0;
            crc3    = 0;
        } else {
        };

        HAL_UART_Receive_IT(&huart3, usart3_rx, 1);
    }
    if (huart->Instance == USART6) {
        static uint16_t u6state = 0; // 状态机计数
        static uint16_t crc6    = 0; // 校验和
        uint8_t tmp6            = usart6_rx[0];
        if (u6state < 4) {
            if (tmp6 == 0xAA) {
                Rxbuffer_6[u6state] = tmp6;
                u6state++;

            } else {
                u6state = 0;
            }
        } else if (u6state < 194) {
            Rxbuffer_6[u6state] = tmp6;
            u6state++;
            crc6 += tmp6;
        } else if (u6state == 194) {
            Rxbuffer_6[u6state] = tmp6;
            if (tmp6 == crc6 % 256) {
                UartFlag[3] = 1;
            }
            u6state = 0;
            crc6    = 0;
        } else {
        };

        HAL_UART_Receive_IT(&huart6, usart6_rx, 1);
    }

    if (huart->Instance == UART4) {
        static uint16_t u4state = 0; // 状态机计数
        static uint16_t crc4    = 0; // 校验和
        uint8_t tmp4            = usart4_rx[0];
        if (u4state < 4) {
            if (tmp4 == 0xAA) {
                Rxbuffer_4[u4state] = tmp4;
                u4state++;

            } else {
                u4state = 0;
            }
        } else if (u4state < 194) {
            Rxbuffer_4[u4state] = tmp4;
            u4state++;
            crc4 += tmp4;
        } else if (u4state == 194) {
            Rxbuffer_4[u4state] = tmp4;
            if (tmp4 == crc4 % 256) {
                UartFlag[4] = 1;
            }
            u4state = 0;
            crc4    = 0;
        } else {
        };

        HAL_UART_Receive_IT(&huart4, usart4_rx, 1);
    }

    if (huart->Instance == UART5) {
        UartFlag[5] = 0;
        HAL_UART_Receive_IT(&huart5, receive_buffer, sizeof(receive_buffer));
    }
}
// void HAL_UART_ErrorCallback(UART_HandleTypeDef *uartHandle)
// {
//     uartHandle->RxState = HAL_UART_STATE_READY;
//     if (uartHandle->Instance == USART2) {
//         HAL_UART_Receive_IT(&huart2, (Rxbuffer[1]), 195);
        
//     }
// }


/*************************树莓派数据解码**********************************/
void Upper_Target_Decode()
{
    static union {
        uint8_t data[20];
        float weight_state[5];
    } state;

    if ((receive_buffer[0] == 0xFF) && (receive_buffer[1] == 0xFE) && (receive_buffer[22] == 0xFE) && (receive_buffer[23] == 0xFF)) {
        for (int i = 0; i < 20; i++) {
            state.data[i] = receive_buffer[i + 2];
        }

        for (int t = 0; t < 5; t++) {
            weight_placement[t] = state.weight_state[t];
        }
    }
}

//另一个版本的解码(int类型)
// void Upper_Target_Decode()
// {
//     if ((receive_buffer[0] == 0xFF) && (receive_buffer[1] == 0xFE) && (receive_buffer[22] == 0xFE) && (receive_buffer[23] == 0xFF)) {
//         for (int i = 0; i < 5; i++) {
//             weight_placement[i] = receive_buffer[2 + 4 * i] + receive_buffer[2 + 4 * i + 1] << 8 + receive_buffer[2 + 4 * i + 2] << 16 + receive_buffer[2 + 4 * i + 3] << 24;
//         }
//     }
// }