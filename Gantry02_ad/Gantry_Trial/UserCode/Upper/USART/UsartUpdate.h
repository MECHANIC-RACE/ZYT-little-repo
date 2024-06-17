

#ifndef __USARTUPDATE_H__
#define __USARTUPDATE_H__
#include "UpperStart.h"

void UsartUpdate_Start();
void Usart_start();
void RaspReceive_Enable();
uint16_t Check_LidarStatus(LidarPointTypedef lidara, LidarPointTypedef lidarb);

#endif // __USARTUPDATE_H__