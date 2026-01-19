#include "main.h"
#include "tim.h"
#include "gpio.h"

#include <stdio.h>
#include <stdlib.h>
#include "DJ.h"

//******************************************宏定义*******************************************************//
#define MAX_SPEED 1000	//最大占空比
#define DJ_TIM_IT &htim7	//中断定时器

#define DJ_TIM_1 &htim1				//电机1PWM源
#define DJ_TCN_1 TIM_CHANNEL_1		//电机1PWM通道
#define DJ_GPIO_1_A GPIOB,GPIO_PIN_4	//电机1GPIOA
#define DJ_GPIO_1_B GPIOB,GPIO_PIN_5	//电机1GPIOB
#define DJ_ECD_1 &htim2				//电机1编码器
#define DJ_ECT_1 TIM2->CNT			//电机1编码器计数器
#define DJ_Signal_1 138				//电机1每圈信号

#define DJ_TIM_2 &htim1				//电机2PWM源
#define DJ_TCN_2 TIM_CHANNEL_2		//电机2PWM通道
#define DJ_GPIO_2_A GPIOA,GPIO_PIN_4	//电机2GPIOA
#define DJ_GPIO_2_B GPIOA,GPIO_PIN_5	//电机2GPIOB
#define DJ_ECD_2 &htim3			//电机2编码器
#define DJ_ECT_2 TIM3->CNT			//电机2编码器计数器
#define DJ_Signal_2 138				//电机2每圈信号

#define DJ_TIM_3 &htim1				//电机3PWM源
#define DJ_TCN_3 TIM_CHANNEL_3		//电机3PWM通道
#define DJ_GPIO_3_A GPIOD,GPIO_PIN_10	//电机3GPIOA
#define DJ_GPIO_3_B GPIOD,GPIO_PIN_11	//电机3GPIOB
#define DJ_ECD_3 &htim4				//电机3编码器
#define DJ_ECT_3 TIM4->CNT			//电机3编码器计数器
#define DJ_Signal_3 138				//电机3每圈信号

#define DJ_TIM_4 &htim1				//电机4PWM源
#define DJ_TCN_4 TIM_CHANNEL_4		//电机4PWM通道
#define DJ_GPIO_4_A GPIOA,GPIO_PIN_2	//电机4GPIOA
#define DJ_GPIO_4_B GPIOA,GPIO_PIN_3	//电机4GPIOB
#define DJ_ECD_4 &htim5				//电机4编码器
#define DJ_ECT_4 TIM5->CNT			//电机4编码器计数器
#define DJ_Signal_4 138				//电机4每圈信号
		
//******************************************全局变量*******************************************************//
DJ_Parameter DJ[4];	//电机参数
int DJ_State=0;		//0等待指令，1执行命令，2进行PID
//*******************************************PID********************************************************//

PID_HandleTypeDef PID[4];		//电机PID参数

void MY_PID_Init(void)			//初始化PID参数
{
	PID_Init(&PID[0],0.015,0.085,0.05,MAX_SPEED-1,0.1);
//	PID_Init(&PID[1],0.1,0.1,0.1,MAX_SPEED-1,0.1);
//	PID_Init(&PID[2],0.1,0.1,0.1,MAX_SPEED-1,0.1);
//	PID_Init(&PID[3],0.1,0.1,0.1,MAX_SPEED-1,0.1);
	
}

//******************************************函数*******************************************************//

// 子结构体：封装GPIO的“端口+引脚”
typedef struct {
    GPIO_TypeDef* port;  // GPIO端口（如GPIOA、GPIOB）
    uint16_t pin;        // GPIO引脚（如GPIO_PIN_4）
} GPIO_Pin_t;

// 主结构体：封装单台电机的所有参数
typedef struct {
    TIM_HandleTypeDef* pwm_tim;    // PWM对应的定时器句柄（如&htim1）
    uint32_t pwm_channel;          // PWM对应的通道（如TIM_CHANNEL_1）
    GPIO_Pin_t gpio_a;             // 电机控制GPIO_A
    GPIO_Pin_t gpio_b;             // 电机控制GPIO_B
    TIM_HandleTypeDef* enc_tim;    // 编码器对应的定时器句柄（如&htim2）
    uint16_t per_signal;           // 电机每圈信号值（如138）
} DJ_Motor_Params_t;

// 结构体数组：存储4台电机的所有参数（替换原来的宏定义）
DJ_Motor_Params_t dj_motors[4] = {
    // 电机1（对应原DJ_*_1的宏）
    {
        .pwm_tim = DJ_TIM_1,
        .pwm_channel = DJ_TCN_1,
        .gpio_a = {DJ_GPIO_1_A},
        .gpio_b = {DJ_GPIO_1_B},
        .enc_tim = DJ_ECD_1,
        .per_signal = DJ_Signal_1
    },
    // 电机2（对应原DJ_*_2的宏）
    {
        .pwm_tim = DJ_TIM_2,
        .pwm_channel = DJ_TCN_2,
        .gpio_a = {DJ_GPIO_2_A},
        .gpio_b = {DJ_GPIO_2_B},
        .enc_tim = DJ_ECD_2,
        .per_signal = DJ_Signal_2
    },
    // 电机3（对应原DJ_*_3的宏）
    {
        .pwm_tim = DJ_TIM_3,
        .pwm_channel = DJ_TCN_3,
        .gpio_a = {DJ_GPIO_3_A},
        .gpio_b = {DJ_GPIO_3_B},
        .enc_tim = DJ_ECD_3,
        .per_signal = DJ_Signal_3
    },
    // 电机4（对应原DJ_*_4的宏）
    {
        .pwm_tim = DJ_TIM_4,
        .pwm_channel = DJ_TCN_4,
        .gpio_a = {DJ_GPIO_4_A},
        .gpio_b = {DJ_GPIO_4_B},
        .enc_tim = DJ_ECD_4,
        .per_signal = DJ_Signal_4
    }
};

void DJ_init(void)//电机初始化
{
	int i;
	
	//0.1s定时中断开启
	HAL_TIM_Base_Start_IT(DJ_TIM_IT);
	
	//电机定时器开启
	for(i=0;i<4;i++)
	{
		HAL_TIM_PWM_Start(dj_motors[i].pwm_tim,dj_motors[i].pwm_channel);
		HAL_TIM_Encoder_Start(dj_motors[i].enc_tim,TIM_CHANNEL_ALL);
	}

	//电机GPIO初始化
	for(i=0;i<4;i++)
	{
		//测试初始转速最大值
		HAL_GPIO_WritePin(dj_motors[i].gpio_a.port,dj_motors[i].gpio_a.pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(dj_motors[i].gpio_b.port,dj_motors[i].gpio_b.pin,GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(dj_motors[i].pwm_tim,dj_motors[i].pwm_channel,MAX_SPEED-1);
		HAL_Delay(1500);
		DJ[i].DJ_MaxSpeed=DJ[i].DJ_Speed;
		HAL_GPIO_WritePin(dj_motors[i].gpio_a.port,dj_motors[i].gpio_a.pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dj_motors[i].gpio_b.port,dj_motors[i].gpio_b.pin,GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(dj_motors[i].pwm_tim,dj_motors[i].pwm_channel,0);
	}
	
}

void DJ_base_run(int Sn,int duty)//电机基本运行
{
	//防止越界
	if(abs(duty) > MAX_SPEED)duty=0; 
	//设置占空比
	__HAL_TIM_SetCompare(dj_motors[Sn].pwm_tim,dj_motors[Sn].pwm_channel,abs(duty));
	//GPIO设置
	if(duty > 0)
	{
		HAL_GPIO_WritePin(dj_motors[Sn].gpio_a.port,dj_motors[Sn].gpio_a.pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(dj_motors[Sn].gpio_b.port,dj_motors[Sn].gpio_b.pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(dj_motors[Sn].gpio_a.port,dj_motors[Sn].gpio_a.pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dj_motors[Sn].gpio_b.port,dj_motors[Sn].gpio_b.pin,GPIO_PIN_SET);
	}
}

void DJ_Wheel_Set(int num,int speed)//车轮基本转动
{
	//更新目标速度
	DJ[num].DJ_Target_Speed=speed;
	PID_SetTargetRpm(&PID[num],DJ[num].DJ_Target_Speed);
	
	//设置初始占空比
	DJ_base_run(num,MAX_SPEED*DJ[num].DJ_Target_Speed/DJ[num].DJ_MaxSpeed);
	DJ_State=1;
}

void DJ_stop(uint8_t move)//电机停止
{
	int i;
	//软停止
	if(move == 0)
	{
		for(i=0;i<4;i++)
		{
			DJ_base_run(i,0);
			HAL_GPIO_WritePin(dj_motors[i].gpio_a.port,dj_motors[i].gpio_a.pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dj_motors[i].gpio_b.port,dj_motors[i].gpio_b.pin,GPIO_PIN_RESET);
		}
	}
	//硬停止
	else if(move == 1)
	{
		for(i=0;i<4;i++)
		{
			DJ_base_run(i,0);
			HAL_GPIO_WritePin(dj_motors[i].gpio_a.port,dj_motors[i].gpio_a.pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(dj_motors[i].gpio_b.port,dj_motors[i].gpio_b.pin,GPIO_PIN_SET);
		}
	}
}


//******************************************中断函数*******************************************************//
uint16_t num=0;
int DJ_CNT[4];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == DJ_TIM_IT)
	{
		num++;
		if(num==10)
		{
			num=0;
			int i;
			for(i=0;i<4;i++)
			{
				//电机转速计算
				if(dj_motors[i].enc_tim->Instance->CNT > 3000)DJ_CNT[i]=6000-(dj_motors[i].enc_tim->Instance->CNT);
				else DJ_CNT[i]=dj_motors[i].enc_tim->Instance->CNT;
				DJ[i].DJ_Speed = DJ_CNT[i]*60/DJ_Signal_1;
				dj_motors[i].enc_tim->Instance->CNT=0;
				//更新最大值
				if(DJ[i].DJ_MaxSpeed<DJ[i].DJ_Speed)DJ[i].DJ_MaxSpeed=DJ[i].DJ_Speed;
			}
		}
		
//		if(DJ_State == 1)
//		{
//			int i;
//			for(i=0;i<4;i++)
//			{
//				//PID算法
//				__HAL_TIM_SetCompare(dj_motors[i].pwm_tim,dj_motors[i].pwm_channel,PID_Calculate(&PID[i],DJ[i].DJ_Speed));
//			}
//		}
	}
}
