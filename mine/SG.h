#ifndef __SG_H
#define __SG_H

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


void SG_Init(void);//舵机初始化
uint8_t SG_SetAngle(uint8_t num, float angle);//舵机角度控制

uint8_t SG_Open(void);
uint8_t SG_Close(void);


#endif
