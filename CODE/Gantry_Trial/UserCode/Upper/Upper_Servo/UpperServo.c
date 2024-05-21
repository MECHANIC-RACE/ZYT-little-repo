

#include "UpperServo.h"


CoreXY_COMPONENT Core_xy[2];    //两个分区的龙门的数据


/********线程相关部分*************/

void Upper_Servo_Task(void *argument)
{
    osDelay(100);
    for (;;) {
        /*TestCode*/

        // positionServo_lidar(current_pos[0], Core_xy[0].Motor_X,Lidar1);
        // positionServo_lidar(current_pos[1], Core_xy[0].Motor_Y,Lidar2);

        //speedServo(5*(505-Lidar2.distance_aver), Core_xy[0].Motor_Y);
        speedServo(8000, Core_xy[0].Motor_X);
        CanTransmit_DJI_1234(&hcan1,
                             Core_xy[0].Motor_X->speedPID.output,
                             0,
                             0,
                             0);

        osDelay(2);
    }
    
}

void Upper_Servo_Start(void)
{
    osThreadId_t Upper_ServoHandle;
    const osThreadAttr_t Upper_Servo_attributes = {
        .name       = "Upper_Servo",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };
    Upper_ServoHandle = osThreadNew(Upper_Servo_Task, NULL, &Upper_Servo_attributes);
}

/*******封装函数部分********/
void Core_xy_Motor_init()               //电机初始化
{
    
    Core_xy[0].Motor_X = &hDJI[0];
    Core_xy[0].Motor_Y = &hDJI[1];
    Core_xy[1].Motor_X = &hDJI[2];
    Core_xy[1].Motor_Y = &hDJI[3];
    hDJI[0].motorType = M2006;      
    hDJI[1].motorType = M2006;
    hDJI[2].motorType  = M2006;
    hDJI[3].motorType  = M2006;
    DJI_Init();
    pid_reset(&(Core_xy[0].Motor_X->speedPID), 1.4, 0.001, 0.0);
    pid_reset(&(Core_xy[0].Motor_Y->speedPID), 5.0, 0.01, 0.005);

    pid_reset(&(Core_xy[0].Motor_X->posPID), 200, 0, 0);
    pid_reset(&(Core_xy[0].Motor_Y->posPID),-200, 0, 0);
    CANFilterInit(&hcan1);
}




/**
 * @brief T型速度规划函数
 * @param initialAngle 初始角度
 * @param maxAngularVelocity 最大角速度
 * @param AngularAcceleration 角加速度
 * @param targetAngle 目标角度
 * @param currentTime 当前时间
 * @param currentAngle 当前角度
 * @todo 转换为国际单位制
 */
void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle)
{

    float angleDifference = targetAngle - initialAngle;     // 计算到目标位置的角度差
    float sign            = (angleDifference > 0) ? 1 : -1; // 判断角度差的正负(方向)

    float accelerationTime = maxAngularVelocity / AngularAcceleration;                                                      // 加速(减速)总时间
    float constTime        = (fabs(angleDifference) - AngularAcceleration * pow(accelerationTime, 2)) / maxAngularVelocity; // 匀速总时间
    float totalTime        = constTime + accelerationTime * 2;                                                              // 计算到达目标位置所需的总时间

    // 判断能否达到最大速度
    if (constTime > 0) {
        // 根据当前时间判断处于哪个阶段
        if (currentTime <= accelerationTime) {
            // 加速阶段
            *currentAngle = initialAngle + sign * 0.5 * AngularAcceleration * pow(currentTime, 2);
        } else if (currentTime <= accelerationTime + constTime) {
            // 匀速阶段
            *currentAngle = initialAngle + sign * maxAngularVelocity * (currentTime - accelerationTime) + 0.5 * sign * AngularAcceleration * pow(accelerationTime, 2);
        } else if (currentTime <= totalTime) {
            // 减速阶段
            float decelerationTime = currentTime - accelerationTime - constTime;
            *currentAngle          = initialAngle + sign * maxAngularVelocity * constTime + 0.5 * sign * AngularAcceleration * pow(accelerationTime, 2) + sign * (maxAngularVelocity * decelerationTime - 0.5 * AngularAcceleration * pow(decelerationTime, 2));
        } else {
            // 达到目标位置
            *currentAngle = targetAngle;
        }
    } else {
        maxAngularVelocity = sqrt(fabs(angleDifference) * AngularAcceleration);
        accelerationTime   = maxAngularVelocity / AngularAcceleration;
        totalTime          = 2 * accelerationTime;
        constTime          = 0;
        // 根据当前时间判断处于哪个阶段
        if (currentTime <= accelerationTime) {
            // 加速阶段
            *currentAngle = initialAngle + sign * 0.5 * AngularAcceleration * pow(currentTime, 2);
        } else if (currentTime <= totalTime) {
            // 减速阶段
            float decelerationTime = currentTime - accelerationTime; // 减速时间
            *currentAngle          = initialAngle + sign * 0.5 * AngularAcceleration * pow(accelerationTime, 2) + sign * (maxAngularVelocity * decelerationTime - 0.5 * AngularAcceleration * pow(decelerationTime, 2));
        } else {
            // 达到目标位置
            *currentAngle = targetAngle;
           
        }
    }
}
