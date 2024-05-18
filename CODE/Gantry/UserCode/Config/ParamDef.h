
#ifndef __PARAMDEF_H__
#define __PARAMDEF_H__

#include "FreeRTOS.h"
#include "semphr.h"
#include "UpperStart.h"
#define BELT_LENGTH_PER_ROUND 95.5044167  //输出轴转一圈同步带走的长度，单位:mm

typedef __IO struct 
{
    __IO struct {
        double x;
        double y;
        double z;
    } velocity; // 速度结构体//不知道需不需要来着

    __IO struct {
        double x;
        double y;
        double z;
    } position;
    SemaphoreHandle_t xMutex_control; // 互斥锁
}CoreXYState;


typedef __IO struct {
    DJI_t *Motor_X;
    DJI_t *Motor_Y;
    _Bool cylinder;
    _Bool electromagnet;
    CoreXYState gantry_t;
} CoreXY_COMPONENT;

extern CoreXY_COMPONENT Core_xy[2];       //全局的Corexy状态参数
//extern CoreXYState TargetState[2];              //目标状态

extern uint16_t UartFlag[4];
extern uint8_t temp_buffer[4];
extern uint16_t debug[4];
#endif // __PARAMDEF_H__