#ifndef __DJ_H
#define __DJ_H

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include <stdint.h>

#include "PID.h"

//========================================= 宏定义 =========================================//
// 通用配置
#define MOTOR_TOTAL_NUM         4U                  // 电机总数
#define MOTOR_MAX_PWM_DUTY      1000U               // PWM最大占空比（定时器ARR值，需与CubeMX配置一致）
#define MOTOR_TIM_IT_HANDLE     &htim7              // 转速计算定时器句柄（100ms中断）
#define MOTOR_IT_CNT_THRESHOLD  10U                 // 中断计数阈值（10*100ms=1000ms，即1s计算一次转速）
#define ENCODER_CNT_MAX         6000U               // 编码器计数器最大值（假设定时器为16位，配置为向上/向下计数，最大值6000）
#define ENCODER_CNT_HALF        (ENCODER_CNT_MAX/2) // 编码器计数中间值（3000）
#define SPEED_CALC_SCALE        60.0f               // 转速计算系数（分钟转秒：60秒/分钟）

// 电机状态枚举
typedef enum {
    MOTOR_STATE_IDLE = 0,       // 0：待机指令
    MOTOR_STATE_EXEC = 1,       // 1：执行命令
    MOTOR_STATE_PID = 2         // 2：PID调速中
} Motor_State_e;

// GPIO引脚结构体
typedef struct {
    GPIO_TypeDef* port;         // GPIO端口（如GPIOA、GPIOB）
    uint16_t pin;               // GPIO引脚（如GPIO_PIN_4）
} GPIO_Pin_t;

// 电机参数结构体（整合单电机所有配置）
typedef struct {
    TIM_HandleTypeDef* pwm_tim;     // PWM定时器句柄
    uint32_t pwm_channel;           // PWM通道
    GPIO_Pin_t gpio_a;              // 正转GPIO
    GPIO_Pin_t gpio_b;              // 反转GPIO
    TIM_HandleTypeDef* enc_tim;     // 编码器定时器句柄
    uint16_t enc_per_round;         // 编码器每圈脉冲数
    float max_speed;                // 电机最大转速（rpm）
    float target_speed;             // 目标转速（rpm）
    float current_speed;            // 当前转速（rpm）
} Motor_Param_t;


//************************************全局变量**********************************************************//
extern Motor_Param_t motor[MOTOR_TOTAL_NUM];	//电机参数

//******************************************函数*******************************************************//
void MY_PID_Init(void);//PID参数初始化（对外接口）

void DJ_init(void);//电机初始化（含PWM、编码器、GPIO、最大转速校准）

void DJ_base_run(uint8_t motor_num, int32_t duty);//单个电机控制函数（核心控制逻辑）

void DJ_Wheel_Set(uint8_t motor_num, float target_speed);//设置电机目标转速（启动PID调速）

void DJ_stop(uint8_t stop_mode);//电机停止控制


#endif
