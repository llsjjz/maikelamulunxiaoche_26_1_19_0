#ifndef __MY_H
#define __MY_H

#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "DJ.h"
#include "SG.h"

//单位 m/s m RPW(转每分钟) rad/s

//*******************************结构体**************************************************************//

//车速度参数
typedef struct{
	float Vx; //车左正
	float Vy;//车前正
	float w;//车顺正
}speed_parameter;

//车物理参数
typedef struct{
	double length; //车长
	double wide;//车宽
	double radius;//车轮半径
}vehicle_parameter;

//*****************************************函数************************************************************//
void My_Init(void);//初始化
void My_run(void);//总运行,放在接收中断

//*****************************************************调试专用******************************************//
void My_vofa_scan(int num);//查看速度波形
void My_Uart_scan(int num);//通过UART修改PID参数和转速，使用时加入UART中断


#endif
