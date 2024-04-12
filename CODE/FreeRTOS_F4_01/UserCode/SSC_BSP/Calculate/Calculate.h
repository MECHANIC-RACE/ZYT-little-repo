#ifndef _CACULATE_H__
#define _CACULATE_H__
#ifdef __cplusplus
extern "C"
{
#endif
#include "DJI.h"
    void positionServo(float ref, DJI_t *motor);
    void speedServo(float ref, DJI_t *motor);
    void CalculateFourMecanumWheels(double *moter_speed, double vx, double vy, double vw);
    void DeadBand(double x, double y, double *new_x, double *new_y, double threshould);
    void DeadBandOneDimensional(double x, double *new_x, double threshould);
#ifdef __cplusplus
}
#endif

#endif
