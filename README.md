Step_motor

一个简易易用的步进电机驱动

## 优势

- 简单易用，仅需调用一个函数即可
  
- 轻量化，不依赖其他外部库
  
- 负载低，空闲时不开启定时器占用资源
  

## 用法

驱动使用定时器输出单脉冲控制步进电机PUL脉冲，DIR和ENA需要自行配置，驱动只包含PUL脉冲输出，注意使用的是高级定时器，定时器主频168MHz

在正确完整导入驱动，正确设置定时器和pwm单脉冲之后，在需要的地方调用

```c
void start(float angle);
```

注意angle为相对角度，默认上电时刻为角度0点，会自动计算梯形加减速

此时驱动会自行开启定时器，进入定时器中断，驱动会自动向TIM8_CH1对应IO发送脉冲

## 开发说明

驱动计算相关参数都在step_motor.c的全局变量中，可供修改的部分如下所示

```c
//参数设置
float accel = 0.5;     //加速度
float decel = 0.5;     //减速度
float speed = 20;      //最高速度
```

此外，驱动提供状态监测接口，全局变量如下所示

```c
float step_count = 0.0f;		 // 当前步数
float total_steps = 0.0f;		 // 总步数

int status = 0; 				 // 0停止 1加速 2匀速 3减速
int mode = 0;							//三角形或梯形
```

另外，如果需要，可以修改定时器相关配置
