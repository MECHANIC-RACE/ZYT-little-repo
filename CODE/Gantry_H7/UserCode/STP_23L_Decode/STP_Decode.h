#ifndef __STP_DECODE_H__
#define __STP_DECODE_H__


#include "stm32h7xx_hal.h"
#include "usart.h"

/*STP_23L数据包命令码宏定义*/
#define PACK_GET_DISTANCE       0x02
#define PACK_RESET_SYSTEM       0x0D
#define PACK_STOP               0x0F
#define PACK_ACK                0x10
#define PACK_VERSIOM            0x14


typedef struct{
    int16_t distance;               //测量目标距离（单位：mm）
    uint16_t noise;                 //环境噪声
    uint32_t peak;                  //接收强度信息
    uint8_t confidence;             //置信度    
    uint32_t intg;                  //积分次数
    int16_t reftof;                 //温度表征值
} LidarPointTypedef;                //获取的测量数据结构体


/*定义接收到的雷达测试数据*/

void STP_23L_Decode(uint8_t *buffer, LidarPointTypedef *lidardata);

#endif // __STP_DECODE_H__