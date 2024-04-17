#include "UpperServo.h"


/*
问题记录：
已经进StartTask的循环了为什么没有执行MotorInit（PID的参数都是0没有初始化
为什么时而报memcpy的警告有时又不报

并且有的时候会卡在hardfault_handle 

*/


CoreXY_COMPONENT Core_xy;

/********线程相关部分*************/

void Upper_Servo_Task(void *argument)
{
    osDelay(1000);
    for (;;) {
        xSemaphoreTakeRecursive(Target.xMutex_control, portMAX_DELAY);
        CoreXYState target_tmp = Target;
        xSemaphoreGiveRecursive(Target.xMutex_control);     //获取目标控制状态

        
        CoreXY_COMPONENT CORE_TMP;
        vPortEnterCritical();
        memcpy(CORE_TMP.Motor_X, Core_xy.Motor_X, sizeof(DJI_t));
        memcpy(CORE_TMP.Motor_Y, Core_xy.Motor_Y, sizeof(DJI_t));
        memcpy(CORE_TMP.Motor_Z, Core_xy.Motor_Z, sizeof(DJI_t));
        vPortExitCritical();                                   //获取当前状态

        Core_xy_Servo(target_tmp,CORE_TMP); // 伺服函数
        
        CanTransmit_DJI_1234(&hcan1,
                             CORE_TMP.Motor_X->speedPID.output,
                             CORE_TMP.Motor_Y->speedPID.output,
                             CORE_TMP.Motor_Z->speedPID.output,
                             CORE_TMP.Motor_X->speedPID.output); // ee到时候多一个电机数据怎么办呢急急急ID等于4的电调是不是就用不了了

        vPortEnterCritical();
        memcpy(Core_xy.Motor_X, CORE_TMP.Motor_X, sizeof(DJI_t));
        memcpy(Core_xy.Motor_Y, CORE_TMP.Motor_Y, sizeof(DJI_t));
        memcpy(Core_xy.Motor_Z, CORE_TMP.Motor_Z, sizeof(DJI_t));
        vPortExitCritical();                                    //将临时变量的值归还给全局变量

        osDelay(10);

    }
    
}

void Upper_Servo_Start(void)
{
    osThreadId_t Upper_ServoHandle;
    const osThreadAttr_t Upper_Servo_attributes = {
        .name       = "Upper_Servo",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityAboveNormal,
    };
    Upper_ServoHandle = osThreadNew(Upper_Servo_Task, NULL, &Upper_Servo_attributes);
}

/*******封装函数部分********/
void Core_xy_Motor_init()               //电机初始化
{
    
    Core_xy.Motor_X = &hDJI[0];
    Core_xy.Motor_Y = &hDJI[1];
    Core_xy.Motor_Z = &hDJI[2];
    hDJI[0].motorType = M3508;
    hDJI[1].motorType = M2006;
    hDJI[2].motorType = M3508;
    DJI_Init();
    CANFilterInit(&hcan1);
    //HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
}


void Core_xy_Servo(CoreXYState Target,CoreXY_COMPONENT Now)
{
    float REF[3];
    /*
    解算函数，将Target.position.x，y,z与Core_xy.position和REF[]建立起联系
    */
   //注：以下三个值正负待定
   //需要加上每时刻速度的判断吗？T型速度规划？
   //可不可以不建立互斥锁了，直接临界段好了

   //test
    Target.position.x = 360;
    Target.position.y = 360;
    Target.position.z = 360;
    REF[0] = (Target.position.x - Now.Corexy_state.position.x) / BELT_LENGTH_PER_ROUND * 360.0f;//所需要转的角度  单位：度
    REF[1] = (Target.position.y - Now.Corexy_state.position.y) / BELT_LENGTH_PER_ROUND * 360.0f;
    REF[2] = (Target.position.z - Now.Corexy_state.position.z) / BELT_LENGTH_PER_ROUND * 360.0f;
    positionServo(REF[0], Now.Motor_X);
    positionServo(REF[1], Now.Motor_Y);
    positionServo(REF[2], Now.Motor_Z);
}

//还差互斥锁的建立与完整信息链的传输







/**
 * @brief T型速度规划函数
 * @param initialAngle 初始角度
 * @param maxAngularVelocity 最大角速度
 * @param AngularAcceleration 角加速度
 * @param targetAngle 目标角度
 * @param currentTime 当前时间
 * @param currentTime 当前角度
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