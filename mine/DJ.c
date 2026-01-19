#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "DJ.h"

#include "My.h" 

//========================================= 电机硬件配置（集中管理） =========================================//
static const Motor_Param_t motor_default_config[MOTOR_TOTAL_NUM] = {
    // 电机1配置
    {
        .pwm_tim = &htim1,
        .pwm_channel = TIM_CHANNEL_1,
        .gpio_a = {GPIOB, GPIO_PIN_4},
        .gpio_b = {GPIOB, GPIO_PIN_5},
        .enc_tim = &htim2,
        .enc_per_round = 138U       // 每圈脉冲数
    },
    // 电机2配置
    {
        .pwm_tim = &htim1,
        .pwm_channel = TIM_CHANNEL_2,
        .gpio_a = {GPIOA, GPIO_PIN_4},
        .gpio_b = {GPIOA, GPIO_PIN_5},
        .enc_tim = &htim3,
        .enc_per_round = 138U
    },
    // 电机3配置
    {
        .pwm_tim = &htim1,
        .pwm_channel = TIM_CHANNEL_3,
        .gpio_a = {GPIOD, GPIO_PIN_10},
        .gpio_b = {GPIOD, GPIO_PIN_11},
        .enc_tim = &htim4,
        .enc_per_round = 138U
    },
    // 电机4配置
    {
        .pwm_tim = &htim1,
        .pwm_channel = TIM_CHANNEL_4,
        .gpio_a = {GPIOA, GPIO_PIN_2},
        .gpio_b = {GPIOA, GPIO_PIN_3},
        .enc_tim = &htim5,
        .enc_per_round = 138U
    }
};

//========================================= 全局变量（精简+初始化） =========================================//
Motor_Param_t motor[MOTOR_TOTAL_NUM] = {0};  // 电机运行参数（静态全局，避免外部篡改）
static PID_HandleTypeDef pid[MOTOR_TOTAL_NUM] = {0};// PID参数（静态全局）
static Motor_State_e motor_state = MOTOR_STATE_IDLE;// 电机状态（替代原DJ_State）
static uint16_t it_cnt = 0U;                        // 中断计数（静态，替代原全局num）
static int32_t encoder_cnt[MOTOR_TOTAL_NUM] = {0};  // 编码器计数缓存（静态）

//========================================= 私有工具函数（参数校验） =========================================//
/**
 * @brief 电机参数合法性校验
 * @param motor_num: 电机编号（0~3）
 * @retval 0：合法，1：非法
 */
static uint8_t Motor_CheckParam(uint8_t motor_num)
{
    if (motor_num >= MOTOR_TOTAL_NUM) {
        return 1; // 电机编号越界
    }
    return 0;
}

//========================================= 电机核心控制函数 =========================================//
/**
 * @brief PID参数初始化（对外接口）
 */
void MY_PID_Init(void)
{
    // 电机1PID参数（其余电机可按需配置）
    PID_Init(&pid[0], 0.015f, 0.085f, 0.05f, MOTOR_MAX_PWM_DUTY - 1, 0.1f);
    // 电机2~4默认初始化（可按需修改）
    for (uint8_t i = 1; i < MOTOR_TOTAL_NUM; i++) {
        PID_Init(&pid[i], 0.1f, 0.1f, 0.1f, MOTOR_MAX_PWM_DUTY - 1, 0.1f);
    }
}

/**
 * @brief 电机初始化（含PWM、编码器、GPIO、最大转速校准）
 */
void DJ_init(void)
{
    // 1. 拷贝默认配置到运行参数，初始化变量
    for (uint8_t i = 0; i < MOTOR_TOTAL_NUM; i++) {
        motor[i] = motor_default_config[i];
        motor[i].max_speed = 0.0f;
        motor[i].target_speed = 0.0f;
        motor[i].current_speed = 0.0f;
    }

    // 2. 启动转速计算定时器中断（带返回值判断）
    if (HAL_TIM_Base_Start_IT(MOTOR_TIM_IT_HANDLE) != HAL_OK) {
        Error_Handler(); // 中断启动失败，执行错误处理
    }

    // 3. 启动所有电机的PWM和编码器
    for (uint8_t i = 0; i < MOTOR_TOTAL_NUM; i++) {
        // 启动PWM输出
        if (HAL_TIM_PWM_Start(motor[i].pwm_tim, motor[i].pwm_channel) != HAL_OK) {
            Error_Handler();
        }
        // 启动编码器
        if (HAL_TIM_Encoder_Start(motor[i].enc_tim, TIM_CHANNEL_ALL) != HAL_OK) {
            Error_Handler();
        }
        // 清零编码器计数
        __HAL_TIM_SET_COUNTER(motor[i].enc_tim, 0U);
    }

    // 4. 校准电机最大转速
    for (uint8_t i = 0; i < MOTOR_TOTAL_NUM; i++) {
        // 输出最大占空比，正转
        HAL_GPIO_WritePin(motor[i].gpio_a.port, motor[i].gpio_a.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor[i].gpio_b.port, motor[i].gpio_b.pin, GPIO_PIN_RESET);
        __HAL_TIM_SetCompare(motor[i].pwm_tim, motor[i].pwm_channel, MOTOR_MAX_PWM_DUTY - 1);
        
        HAL_Delay(1500); // 等待转速稳定
        
        // 记录最大转速（此时current_speed已由中断更新）
        if (motor[i].current_speed > motor[i].max_speed) {
            motor[i].max_speed = motor[i].current_speed;
        }
        
        // 停止电机
        HAL_GPIO_WritePin(motor[i].gpio_a.port, motor[i].gpio_a.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[i].gpio_b.port, motor[i].gpio_b.pin, GPIO_PIN_RESET);
        __HAL_TIM_SetCompare(motor[i].pwm_tim, motor[i].pwm_channel, 0U);
    }
}

/**
 * @brief 电机基本运行控制（占空比+转向）
 * @param motor_num: 电机编号（0~3）
 * @param duty: 占空比（正：正转，负：反转，范围：-MOTOR_MAX_PWM_DUTY ~ MOTOR_MAX_PWM_DUTY）
 */
void DJ_base_run(uint8_t motor_num, int32_t duty)
{
    // 参数校验
    if (Motor_CheckParam(motor_num) != 0) {
        return;
    }

    // 占空比限幅（防止超出最大值）
    if (abs(duty) > MOTOR_MAX_PWM_DUTY) {
        duty = 0;
    }

    // 设置PWM占空比（取绝对值）
    __HAL_TIM_SetCompare(motor[motor_num].pwm_tim, motor[motor_num].pwm_channel, abs(duty));

    // 控制转向
    if (duty > 0) {
        // 正转：A高，B低
        HAL_GPIO_WritePin(motor[motor_num].gpio_a.port, motor[motor_num].gpio_a.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor[motor_num].gpio_b.port, motor[motor_num].gpio_b.pin, GPIO_PIN_RESET);
    } else if (duty < 0) {
        // 反转：A低，B高
        HAL_GPIO_WritePin(motor[motor_num].gpio_a.port, motor[motor_num].gpio_a.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[motor_num].gpio_b.port, motor[motor_num].gpio_b.pin, GPIO_PIN_SET);
    } else {
        // 停止：A、B均低
        HAL_GPIO_WritePin(motor[motor_num].gpio_a.port, motor[motor_num].gpio_a.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor[motor_num].gpio_b.port, motor[motor_num].gpio_b.pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief 设置电机目标转速（启动PID调速）
 * @param motor_num: 电机编号（0~3）
 * @param target_speed: 目标转速（rpm）
 */
void DJ_Wheel_Set(uint8_t motor_num, float target_speed)
{
    // 参数校验
    if (Motor_CheckParam(motor_num) != 0) {
        return;
    }

    // 更新目标转速
    motor[motor_num].target_speed = target_speed;
    PID_SetTargetRpm(&pid[motor_num], target_speed);

    // 计算初始占空比（基于最大转速校准值）
    int32_t init_duty = 0;
    if (motor[motor_num].max_speed > 0.0f) { // 防止除零
        init_duty = (int32_t)(MOTOR_MAX_PWM_DUTY * target_speed / motor[motor_num].max_speed);
    }

    // 启动电机
    DJ_base_run(motor_num, init_duty);
    motor_state = MOTOR_STATE_EXEC;
}

/**
 * @brief 电机停止控制
 * @param stop_mode: 0-软停止（断电），1-硬停止（刹车）
 */
void DJ_stop(uint8_t stop_mode)
{
    for (uint8_t i = 0; i < MOTOR_TOTAL_NUM; i++) {
        // 清零占空比
        __HAL_TIM_SetCompare(motor[i].pwm_tim, motor[i].pwm_channel, 0U);

        if (stop_mode == 0) {
            // 软停止：GPIO均置低，断电自由停止
            HAL_GPIO_WritePin(motor[i].gpio_a.port, motor[i].gpio_a.pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor[i].gpio_b.port, motor[i].gpio_b.pin, GPIO_PIN_RESET);
        } else if (stop_mode == 1) {
            // 硬停止：GPIO均置高，短路刹车
            HAL_GPIO_WritePin(motor[i].gpio_a.port, motor[i].gpio_a.pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor[i].gpio_b.port, motor[i].gpio_b.pin, GPIO_PIN_SET);
        }
    }
    motor_state = MOTOR_STATE_IDLE; // 恢复空闲状态
}

//========================================= 中断回调函数（转速计算+PID调速） =========================================//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == MOTOR_TIM_IT_HANDLE) {
        it_cnt++;

        // 每1s（10次中断）计算一次转速
        if (it_cnt >= MOTOR_IT_CNT_THRESHOLD) {
			
            it_cnt = 0U; // 清零中断计数

            // 遍历所有电机计算转速
            for (uint8_t i = 0; i < MOTOR_TOTAL_NUM; i++) {
                // 读取编码器当前计数值
                uint16_t raw_cnt = __HAL_TIM_GET_COUNTER(motor[i].enc_tim);
                
                // 处理编码器正/反转计数（适配向上/向下计数模式）
                if (raw_cnt > ENCODER_CNT_HALF) {
                    encoder_cnt[i] = ENCODER_CNT_MAX - raw_cnt; // 反转计数
                } else {
                    encoder_cnt[i] = raw_cnt; // 正转计数
                }

                // 计算转速：转速(rpm) = (计数/每圈脉冲数) / 时间(s) * 60s
                // 时间=1s，因此公式为：(cnt / per_round) * 60 = cnt * 60 / per_round
                motor[i].current_speed = (float)(encoder_cnt[i] * SPEED_CALC_SCALE ) / motor[i].enc_per_round;

                // 清零编码器计数，准备下一次采样
                __HAL_TIM_SET_COUNTER(motor[i].enc_tim, 0U);

                // 更新电机最大转速（仅校准阶段有效）
                if (motor[i].current_speed > motor[i].max_speed) {
                    motor[i].max_speed = motor[i].current_speed;
                }
            }
        }

        // PID调速（启用时执行）
        if (motor_state == MOTOR_STATE_EXEC) {
            for (uint8_t i = 0; i < MOTOR_TOTAL_NUM; i++) {
                // 计算PID输出（占空比）
                float pid_duty = PID_Calculate(&pid[i], motor[i].current_speed);
                // 更新PWM占空比
                __HAL_TIM_SetCompare(motor[i].pwm_tim, motor[i].pwm_channel, (uint32_t)pid_duty);
            }
            motor_state = MOTOR_STATE_PID; // 标记为PID调速中
        }
		My_run();
    }
}

//*****************************************************调试******************************************//
void My_vofa_scan(int num)//发送
{
	  float W[5]={motor[num].target_speed,motor[num].current_speed,pid[num].Kp,pid[num].Ki,pid[num].Kd};
	  TX_nums_write(W);
	  Transmit();
}

void My_Uart_scan(int num)//接收
{
		float R[4];
		RX_nums_read(R);
	
		pid[num].Kp=R[0];
		pid[num].Ki=R[1];
		pid[num].Kd=R[2];
		DJ_Wheel_Set(num,R[3]);
}
