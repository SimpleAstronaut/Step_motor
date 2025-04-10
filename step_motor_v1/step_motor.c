/**
 * @file motor_step.c
 * @brief 电机步进驱动，使用定时器输出脉冲数控制电机步进，使用GPIO控制方向和使能
 * @details 
 * - 使用高级定时器（默认TIM8），定时器主频168MHz
 * - 支持梯形加减速算法
 * - 非阻塞式控制
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

/* 电机运动控制相关变量 */
float step_count = 0.0f;         // 当前已执行步数
float total_steps = 0.0f;        // 目标总步数（根据角度计算得出）

int status = 0;                  // 运动状态：0-停止 1-加速 2-匀速 3-减速

/* 定时器参数计算中间变量 */
float c0 = 0;                    // 初始周期值（最大周期，对应最低速度）
float cm = 0;                    // 匀速阶段周期值
float ct = 0;                    // 当前周期值

/* 运动参数配置 */
float accel = 0.5;               // 加速度设置
float decel = 0.5;               // 减速度设置
float speed = 20;                // 目标速度设置

float min_delay = 0;             // 最小延迟（未使用）

/* 运动曲线计算参数 */
float A_SQ = 0;                  // 加速度相关参数
float T1_FREQ = 0;               // 定时器频率参数

/* 步数计算参数 */
float ny = 0;                    // 匀速阶段步数
float n1 = 0;                    // 加速阶段步数
float n2 = 0;                    // 减速阶段步数
float n = 0;                     // 总计算步数

float period = 0;                // 周期参数（未使用）

int mode = 0;                    // 运动模式：1-三角形（无匀速段） 2-梯形（有匀速段）

/***************************************************************** 
 * 函数功能: 启动电机运动
 * 输入参数: angle - 目标角度（相对角度，上电时刻为0度）
 * 返 回 值: 无
 * 注意: 
 * - 使用TIM8定时器
 * - 默认包含梯形加减速
 * - 非阻塞式函数，不可重复调用
 ****************************************************************/
void start(float angle)
{
    /* 将角度转换为步数（1.8度/步） */
    total_steps = angle / 1.8;
    
    /* 计算定时器基础参数 */
    T1_FREQ = 0.676*10000/10;    // 定时器频率计算
    A_SQ = 2* 0.0314 *100000;    // 加速度参数计算
    
    /* 计算初始周期（对应最低速度） */
    c0 = T1_FREQ * sqrt(A_SQ/accel);
    
    /* 计算各阶段理论步数 */
    n1 = speed * speed / (0.0628 * accel);  // 加速段理论步数
    n2 = speed * speed / (0.0628 * decel);  // 减速段理论步数
    n = n1 + n2;                            // 加减速总步数

    /* 启动定时器并设置初始参数 */
    HAL_TIM_Base_Start_IT(&htim8);
    __HAL_TIM_SET_AUTORELOAD(&htim8, c0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, c0 / 2);
    
    ct = c0;  // 设置当前周期

    /* 判断运动模式（三角形/梯形） */
    if(n >= total_steps)
    {
        // 三角形模式（总步数不足，无匀速段）
        mode = 1;
        n1 = decel * total_steps / (accel + decel);  // 重新计算加速段步数
        n2 = total_steps - n1;                      // 剩余为减速段步数
    }
    else
    {
        // 梯形模式（有匀速段）
        mode = 2;
        ny = total_steps - n1 - n2;                 // 计算匀速段步数
        cm = (10 * 0.0314 * T1_FREQ)/speed;         // 计算匀速段周期
    }
    
    /* 设置运动状态为加速阶段 */
    status = 1;
    step_count = 1;  // 初始化步数计数器
}

/***************************************************************** 
 * 函数功能: 定时器中断回调函数
 * 输入参数: htim - 触发中断的定时器句柄
 * 返 回 值: 无
 * 注意: 
 * - htim8: PWM输出定时器
 * - TIM7: 系统滴答定时器（处理HAL_Tick）
 ****************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* PWM定时器中断处理 */
    if(htim == (&htim8))
    {
        if(step_count < total_steps)  // 未达到目标步数
        {
            /* 三角形模式速度曲线计算 */
            if(mode == 1)
            {
                if(step_count <= n1)
                {
                    // 加速阶段：周期递减（速度递增）
                    ct = ct - (2 * ct)/(4 * step_count + 1);
                }
                else
                {
                    // 减速阶段：周期递增（速度递减）
                    ct = ct + (2 * ct)/(4 * n2 - 1);
                    n2--;  // 剩余减速步数递减
                }
            }
            /* 梯形模式速度曲线计算 */
            else
            {
                if(step_count <= n1)
                {
                    // 加速阶段
                    ct = ct - (2 * ct)/(4 * step_count + 1);
                }
                else if(step_count > n1 && step_count <= (n1 + ny))
                {
                    // 匀速阶段：保持周期不变
                    ct = ct;
                }
                else
                {
                    // 减速阶段
                    ct = ct + (2 * ct)/(4 * n2 - 1);
                    n2--;  // 剩余减速步数递减
                }
            }
            
            /* 更新定时器参数 */
            __HAL_TIM_SET_AUTORELOAD(&htim8, ct);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, ct / 2);
            __HAL_TIM_ENABLE(&htim8);
            HAL_TIM_OnePulse_Start(&htim8,TIM_CHANNEL_1);
            
            step_count++;  // 步数递增
        }
        else
        {
            /* 运动完成，停止定时器 */
            HAL_TIM_Base_Stop_IT(&htim8); 
        }
    }
    
    /* 系统滴答定时器处理 */
    if (htim->Instance == TIM7) {
        HAL_IncTick();
    }
}
