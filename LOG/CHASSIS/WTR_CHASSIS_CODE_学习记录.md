## WTR_CHASSIS_CODE_学习记录

## FreeRTOSV1---->V2的修改

CYT靴长的写法是只在Mx中保留一个`StartDefaultTask` ,其他任务都手敲代码自己建，

如在`Chassis_Serve.c` 中的这一段：

``````
void Chassis_Servo_TaskStart()
{
    osThreadDef(Chassis_Servo, Chassis_Servo_Task, osPriorityHigh, 0, 1024);
    osThreadCreate(osThread(Chassis_Servo), NULL);
}
``````

就是用于创建任务的，同个文件里还有`void Chassis_Servo_Task(void const **argument*)` 是对线程内容的定义。

> 注意 创建线程的函数是需要自己去运行的，在`Chassis_Start.c` 中的`void StartDefaultTask(void const **argument*)` 函数包含了初始化以及线程启动（创建）



目前的改法：

把原来的自定义的TaskStart函数都屏蔽掉了，然后在Mx里添加了这两个任务

于是可以在`freertos.c`中自动生成

```

  /* creation of Chassis_Servo */
  Chassis_ServoHandle = osThreadNew(Chassis_Servo_Task, NULL, &Chassis_Servo_attributes);

  /* creation of StateMachine */
  StateMachineHandle = osThreadNew(StateMachine_Task, NULL, &StateMachine_attributes);
  
```

就不用自己去启动线程了，只需要定义任务内容即可

（如果有需要的话可以把自己另把以上两句创建任务的码丢到UserCode里，不直接在`freertos.c`里跑

## Chassis_Servo.c

### void Chassis_Servo_Task(void const *argument)

#### 互斥锁

```
xSemaphoreTakeRecursive( SemaphoreHandle_t xMutex,TickType_t xTicksToWait );
```

官网：

> semphr. h
>
> ```
> xSemaphoreTakeRecursive( SemaphoreHandle_t xMutex,
>                          TickType_t xTicksToWait );
> ```
>
> 递归地获得或“获取”一个互斥锁型信号量的*宏*。 此互斥锁必须已经事先通过调用 xSemaphoreCreateRecursiveMutex() 完成创建；
>
> 必须在 FreeRTOSConfig.h 中将 configUSE_RECURSIVE_MUTEXES 设置为 1， 此宏才可用。
>
> 不得在使用 xSemaphoreCreateMutex() 创建的互斥锁上使用此宏。
>
> 所有者可以反复“获取”递归使用的互斥锁。在所有者 为每个成功的“获取”请求调用 xSemaphoreGiveRecursive() 之前，该互斥锁不会再次变得可用。 例如， 如果一个任务成功地“获取”了同一个互斥锁 5 次， 那么任何其他任务将无法使用这个互斥锁， 直到任务也正好“给”回了这个互斥锁 5 次为止。
>
> 
>
> - **参数：**
>
>   *xMutex* 正在获得的互斥锁的句柄。 这是由 xSemaphoreCreateRecursiveMutex() 返回的句柄。*xTicksToWait* 等待信号量变为可用的时间（以滴答为单位）。 宏 portTICK_PERIOD_MS 可用于将其转换为实时。 可以用一个为零的阻塞时间来轮询信号量。 如果任务已有信号量，则无论 xTicksToWait 的值是多少，xSemaphoreTakeRecursive() 都将立即返回。
>
> - **返回：**
>
>   如果获得信号量，则返回 pdTRUE。 如果 xTicksToWait 过期，信号量不可用，则返回 pdFALSE。

此处递归之意是说可以多次获取互斥锁，只要最后一一还回去就好

互斥锁一般用在保证对重要变量的访问或修改正常进行

比如在`Chassis_Servo_Task`函数中的

```
xSemaphoreTakeRecursive(ChassisControl.xMutex_control, portMAX_DELAY);
        CHASSIS_MOVING_STATE ChassisControl_tmp = ChassisControl;           //用临时变量获取底盘的控制状态
        xSemaphoreGiveRecursive(ChassisControl.xMutex_control);             //在此处即释放互斥锁
```

互斥锁没有用于整个线程，而只是用在访问变量的前后，拿到变量的值后即可释放，防止后面的阻塞浪费CPU资源



#### vPortEnterCritical()

来自`port.c`，当有一部分代码希望不被FreeRTOS所管理的高优先级任务打断时，可以调用该函数进入临界区，进入临界区后将会屏蔽操作系统所能管理的任务切换和中断。



### void VelocityPlanning

在伺服系统以及控制系统的加减速动作中，为了让速度更加平滑，可以引入**T型速度曲线规划**（T-curve velocity profile），T曲线是工业界广泛采用的形式，它是一种时间最优的曲线。一般情况，曲线**加速和减速的过程是对称**的，设给定**速度上限**为。**加速度上限**为，被控对象从**A点运动到B点**，要求生成的轨迹在这些条件下**时间最优**^1。

[一文搞懂T型曲线速度规划的原理和实现 （转载菜刀和小麦） - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/360598384)



## Chassis_StateMachine.c

### void StateMachine_Task(void const **argument*)  



