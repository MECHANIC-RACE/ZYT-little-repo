
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

extern CoreXY_COMPONENT Core_xy[3];       //全局的Corexy状态参数

//extern float initial_pos01[2];
extern float current_pos01[2];
//extern float initial_pos02[2];
extern float current_pos02[2];
//extern float initial_pos03[2];
extern float current_pos03[1];
extern uint16_t UartFlag[6];

extern uint8_t usart1_rx[1];
extern uint8_t usart2_rx[1];
extern uint8_t usart3_rx[1];
extern uint8_t usart4_rx[1];
extern uint8_t usart6_rx[1];

extern uint8_t Rxbuffer_1[195];
extern uint8_t Rxbuffer_2[195];
extern uint8_t Rxbuffer_3[195];
extern uint8_t Rxbuffer_4[195];
extern uint8_t Rxbuffer_6[195];

extern LidarPointTypedef Lidar1;
extern LidarPointTypedef Lidar2;
extern LidarPointTypedef Lidar3;
extern LidarPointTypedef Lidar4;
extern LidarPointTypedef Lidar6;

extern uint16_t inner_ring_flag;
extern uint8_t receive_buffer[24];
extern float weight_placement[5];
extern uint16_t Uart_State;

extern float angle_memory01xtree;
extern float angle_memory01ytree;
extern float angle_memory01weight;

extern uint16_t detect01xtree;
extern uint16_t detect01ytree;
extern uint16_t detect01_weight;
#endif // __PARAMDEF_H__