

#include "STP_Decode.h"

void STP_23L_Decode(uint8_t *buffer, LidarPointTypedef*lidardata) // num:指明是第几个雷达，本代码框架中范围为0-3
{
    if((buffer[0]==buffer[1])&&(buffer[1]==buffer[2])&&(buffer[2]==buffer[3])&&(buffer[3]==0xAA))     //检测帧头
    {
        if (buffer[5] == PACK_GET_DISTANCE)                                                                       //检测命令码
         {   uint32_t CS_sum = 0;
            for (uint16_t i = 4; i < 194; i++) CS_sum += buffer[i];
            if (buffer[194] == CS_sum%256)                                                                        //检测校验码
            {
           
                for (uint16_t i = 0; i < 12; i++)
                {
                    // OLED_ShowNum(90, 3, Rxbuffer[5], 2, 8);
                    lidardata->distance   = buffer[10 + 15 * i] + (buffer[11 + 15 * i] * 256); // 只有distance是有用的数据 注意前面是低位后面是高位
                    lidardata->noise      = (buffer[13 + 15 * i] << 8) + buffer[12 + 15 * i];
                    lidardata->peak       = (buffer[17 + 15 * i] << 24) + (buffer[15 + 15 * i] << 16) + (buffer[16 + 15 * i] << 8) + buffer[15 + 15 * i];
                    lidardata->confidence = buffer[18 + 15 * i];
                    lidardata->intg       = (buffer[22 + 15 * i] << 24) + (buffer[21 + 15 * i] << 16) + (buffer[20 + 15 * i] << 8) + buffer[19 + 15 * i];
                    lidardata->reftof     = (buffer[24 + 15 * i] << 8) + buffer[23 + 15 * i];
                }
                
            }
         }
    }
}