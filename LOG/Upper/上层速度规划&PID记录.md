## 上层速度规划&PID记录

位置伺服的几个注意点：

1. 伺服线程的`delay`应该尽可能地短
2. p和i给大了都容易震荡，应该先往小了给

大致调完简单的位置伺服，得到一些相对合适的`pid`整定参数，但是会发现位置伺服得到的`speedref`曲线本身就比较抽象

![屏幕截图 2024-04-28 210046](C:\Users\86189\Pictures\Screenshots\屏幕截图 2024-04-28 210046.png)

表现为有明锐的峰值且后期速度很小(在这个条件下只能尽可能的让实际速度趋近ref曲线但是没有办法改变ref曲线本身的问题)

遂使用速度规划--->

```
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

```



遇到的一些问题：

加速过程会分成一截一截的，就是说加速一段停下来又继续加速，出现这种情况可能是因为`pid`整定得太慢了（p/i给小了) 导致速度规划函数中的`current_angle`一直在增长的同时实际的`Axis_angle`没有跟上这个变化，导致`current_angle`的增长已经完成了但是实际的角度变化并没有完成，所以才会在下一轮循环里再完成剩下的加速，表现为加速分成了一截一截。只要把`pid`的参数再调大一点让它迅速达到每一个`ref(current_angle)`就好啦