#ifndef __MY_H
#define __MY_H

#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <string.h>
#include <stdlib.h>

#include "DJ.h"
#include "SG.h"
#include "MY_UART.h"

//*******************************结构体**************************************************************//
// 速度参数
typedef struct{
    float Vx; // 左右向 (左正右负)
    float Vy; // 前后向 (前正后负)
    float w;  // 旋转角速度 (顺时针正，逆时针负)
}speed_parameter;

// 车辆参数
typedef struct{
    double length; // 长度
    double wide;   // 宽度
    double radius; // 轮子半径
	double radius_1;//变形轮子半径
}vehicle_parameter;

//*****************************************函数************************************************************//
void My_Init(void);                  // 初始化
void My_run(void);                   // 主逻辑，放在定时器中断/主循环

#endif