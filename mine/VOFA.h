#ifndef __VOFA_H
#define __VOFA_H

#include <stdint.h>


//******************************************宏定义*******************************************************//

#define UART_head &huart1	//串口头
#define TX_NUM 5				//发送数据数量
#define RX_NUM 4				//接收数据数量

//******************************************发送函数*******************************************************//

void TX_nums_write(float *W);//向发送缓冲区写入
void TX_nums_read(float *W);//从发射缓冲区读出
void Transmit(void);//向VOFA发送数据

//******************************************接收函数*******************************************************//

void Receive_Init(void);//初始化接收函数
void RX_nums_write(float *R);//向接收缓冲区写入
void RX_nums_read(float *R);//接收缓冲区读出

#endif
