/**
 * @file motor_step.c
 * @brief 电机步进驱动，使用定时器输出脉冲数控制电机步进，使用GPIO控制方向和使能，注意使用的是高级定时器，认为定时器主频为168MHz
 * @version 0.1
 * @date 2025-04-10
 * @author SimpleAstronaut
 */

/**
 * @include 需要包含main.h或自行修改start()函数调用htim
 */
#include "main.h"

#include "step_motor.h"
#include "math.h"

float step_count = 0.0f;		 // 当前步数
float total_steps = 0.0f;		 // 总步数
int current_speed = MIN_SPEED;

int status = 0; 				 // 0停止 1加速 2匀速 3减速

float c0 = 0;
float cm = 0;
float ct = 0;

//参数设置
float accel = 0.5;
float decel = 0.5;
float speed = 20;

float min_delay = 0;

float A_SQ = 0;
float T1_FREQ = 0;

float ny = 0;
float n1 = 0;
float n2 = 0;
float n = 0;

float period = 0;

int mode = 0;							//三角形或梯形


/***************************************************************** 
  * 函数功能: 启动函数
  * 输入参数: angle
  * 返 回 值: 无
  * 注意: 默认定时器为tim8，默认包含梯形加减速
  * 特别注意: 非阻塞式启动函数，不可重复调用，角度为相对角度，默认上电时刻角度为0
  ****************************************************************/
void start(float angle)
{
	total_steps = angle / 1.8;
		
	T1_FREQ = 0.676*10000/10;
	//T1_FREQ = 1000000;
	A_SQ = 2* 0.0314 *100000;

	c0 = T1_FREQ * sqrt(A_SQ/accel);
	//c0 = T1_FREQ * sqrt(2 * 1.8 /accel)/10;
	//c0 = ((float)(0.676*T1_FREQ)) * sqrt(((float)(2*10*0.0314)) / accel);

	n1 = speed * speed / (0.0628 * accel);
	n2 = speed * speed / (0.0628 * decel);
	n = n1 + n2;

	HAL_TIM_Base_Start_IT(&htim8);
	//period = (1000000 / speed) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim8, c0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, c0 / 2);
		
	ct = c0;

	if(n >= total_steps)
	{
		//三角形
		mode = 1;
		n1 = decel * total_steps / (accel + decel);		//加速段步数
		n2 = total_steps - n1;							//减速段步数
	}
	else
	{
		//梯形
		mode = 2;
		ny = total_steps - n1 - n2;
		cm = (10 * 0.0314 * T1_FREQ)/speed;
	}
		
	status = 1;
	step_count = 1;
}

/***************************************************************** 
  * 函数功能: 定时器中断
  * 输入参数: TIM_HandleTypeDef
  * 返 回 值: 无
	* htim2: pwm输出定时器 
	* htim3: 计数定时器
  ****************************************************************/
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {
    //pwm输出定时器
    if(htim == (&htim8))
    {
        if(step_count < total_steps)
        {
             //启动电机
             if(mode == 1)
            {
                if(step_count <= n1)
                    {
                        ct = ct - (2 * ct)/(4 * step_count + 1);
                        //ct = ct / 10;
                        //__HAL_TIM_SET_AUTORELOAD(&htim2, ct);
                        //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ct / 2);
                    }
                    else
                    {
                        //减速阶段
                        ct = ct + (2 * ct)/(4 * n2 - 1);
                        n2--;
                    }
            }
            else
            {
                if(step_count <= n1)
                {
                    ct = ct - (2 * ct)/(4 * step_count + 1);
                }
                else if(step_count > n1 && step_count <= (n1 + ny))
                {
                    ct = ct;
                }
                else
                {
                    ct = ct + (2 * ct)/(4 * n2 - 1);
                    n2--;
                }
            }
            __HAL_TIM_SET_AUTORELOAD(&htim8, ct);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, ct / 2);
            __HAL_TIM_ENABLE(&htim8);
            HAL_TIM_OnePulse_Start(&htim8,TIM_CHANNEL_1);
            step_count++;
        }
        else
        {
            //不加Base_Stop在一些版本的cubemx中有概率持续进入HAL_ERROR
            HAL_TIM_Base_Stop_IT(&htim8); 
        }
    }
    if (htim->Instance == TIM7) {
        HAL_IncTick();
    }
 }
 

