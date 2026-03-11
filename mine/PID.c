#include <stdint.h>
#include <stdio.h>   
#include <math.h>

#include "PID.h"

// PID初始化函数：配置参数+清零内部变量
void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd,float output_limit, float dt)
{
    if(pid == NULL) return;
    
    // 初始化PID参数
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
	pid->output_limit = output_limit;
    pid->integral_limit = (pid->output_limit*0.8/pid->Ki);
    pid->dt = dt;
    
    // 清零误差和积分项
    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->integral = 0.0f;
    
    // 默认目标转速为0
    pid->target_rpm = 0.0f;
}

// PID计算函数：输入实际电机转速，输出控制量（如PWM）
// 参数：pid-PID句柄，actual_rpm-当前电机实际转速
// 返回值：PID输出控制量（如0~1000的PWM占空比）
float PID_Calculate(PID_HandleTypeDef *pid, float actual_rpm)
{
    if(pid == NULL) return 0.0f;
    
    // 1. 更新实际转速，计算当前误差
    pid->actual_rpm = actual_rpm;
    pid->err = pid->target_rpm - pid->actual_rpm;
    
    // 2. 计算积分项（带积分限幅，防止积分饱和）
    pid->integral += pid->err * pid->dt;
    // 积分限幅：限制积分项在[-integral_limit, integral_limit]
    if(pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if(pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;
    
    // 3. 计算PID输出（位置式公式）
    float output = pid->Kp * pid->err          // 比例项
                 + pid->Ki * pid->integral     // 积分项
                 + pid->Kd * (pid->err - pid->last_err) / pid->dt;  // 微分项
    
    // 4. 输出限幅（保证控制量在硬件允许范围）
    if(output > pid->output_limit)
        output = pid->output_limit;
    else if(output < 0.0f)  // 电机正转时输出非负，反转可调整为-output_limit
        output = 0.0f;
    
    // 5. 更新上一次误差，供下次微分计算
    pid->last_err = pid->err;
    
    return output;
}

// 示例：设置PID目标转速
void PID_SetTargetRpm(PID_HandleTypeDef *pid, float target_rpm)
{
    if(pid == NULL) return;
    pid->target_rpm = target_rpm;
}
