#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <stdio.h>   
#include <math.h>

// 定义PID结构体：封装所有PID参数，方便多电机/多回路复用
typedef struct
{
    // 1. PID核心参数（需根据电机调试）
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    
    // 2. 目标值与反馈值
    float target_rpm;  // 设定转速（目标值，单位：rpm）
    float actual_rpm;  // 实际转速（反馈值，传入的电机转速）
    
    // 3. 误差相关（内部计算用）
    float err;         // 当前误差 (target - actual)
    float last_err;    // 上一次误差
    float integral;    // 积分项累积值
    
    // 4. 限幅参数（防止失控）
    float integral_limit;  // 积分限幅（防止积分饱和）
    float output_limit;    // 输出限幅（如PWM范围0~1000）
    
    // 5. 控制周期（单位：s，如10ms=0.01s）
    float dt;          
} PID_HandleTypeDef;

// PID初始化函数：配置参数+清零内部变量
void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd,float output_limit, float dt);

// PID计算函数：输入实际电机转速，输出控制量（如PWM）
// 参数：pid-PID句柄，actual_rpm-当前电机实际转速
// 返回值：PID输出控制量（如0~1000的PWM占空比）
float PID_Calculate(PID_HandleTypeDef *pid, float actual_rpm);

// 示例：设置PID目标转速
void PID_SetTargetRpm(PID_HandleTypeDef *pid, float target_rpm);


#endif
