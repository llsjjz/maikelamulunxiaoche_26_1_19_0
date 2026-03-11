#include "main.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>
#include <string.h>

#include "VOFA.h"

//******************************************使用******************************************************//
//1.开启串口中断,keil打开MicroLIB,修改宏定义
//2.使用函数

//注意
//接收标准数量小于6 		"A=%f,B=%f,C=%f,D=%f,E=%f\n"
//接收标准数量小于11		"A=%f,B=%f,C=%f,D=%f,E=%f,F=%f,G=%f,H=%f,I=%f,J=%f\n"
//就算没到标准数量也得发送完

//******************************************全局变量*******************************************************//

volatile int run_state = 0;			//程序标准位

char rx_cache[RX_NUM*20]="";

//******************************************发送函数*******************************************************//

float TX_nums[TX_NUM]={0};	//发送数据缓存区

void TX_nums_write(float *W)//向发送缓冲区写入
{
	int i;
	for(i=0;i<TX_NUM;i++)
	{
		TX_nums[i]=W[i];
	}
}

void TX_nums_read(float *W)//从发射缓冲区读出
{
	int i;
	for(i=0;i<TX_NUM;i++)
	{
		W[i]=TX_nums[i];
	}
}

void Transmit(void)//向VOFA发送数据
{
	char txt[TX_NUM*10+5]="";
	int i;
	int len = 0;
	for(i=0; i<TX_NUM; i++)
	{
		if(i==TX_NUM-1)
		{
			len += snprintf(txt+len, sizeof(txt)-len, "%f\n", TX_nums[i]);
			break;
		}
		len += snprintf(txt+len, sizeof(txt)-len, "%f,", TX_nums[i]);
	}
	printf("%s",txt);
}

int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(UART_head,(uint8_t *)&ch,1,1000);
	return ch;
}

//******************************************接收函数*******************************************************//

float RX_nums[RX_NUM]={0};	//接收数据缓存区

void Receive_Init(void)//初始化接收函数
{
	HAL_UARTEx_ReceiveToIdle_IT(UART_head,(uint8_t*)rx_cache,RX_NUM*20);
}

void RX_nums_write(float *R)//向接收缓冲区写入
{
	int i;
	for(i=0;i<TX_NUM;i++)
	{
		RX_nums[i]=R[i];
	}
}

void RX_nums_read(float *R)//接收缓冲区读出
{
	int i;
	for(i=0;i<TX_NUM;i++)
	{
		R[i]=RX_nums[i];
	}
} 

void Receive(void)//从VOFA读出
{
#if RX_NUM <=5 
	float cache[5];
	sscanf(rx_cache,"A=%f,B=%f,C=%f,D=%f,E=%f\n",&cache[0],&cache[1],&cache[2],&cache[3],&cache[4]);
	int i;
	for(i=0;i<RX_NUM;i++)
	{
		RX_nums[i]=cache[i];
	}
#elif RX_NUM >5 && RX_NUM <= 10 
	float cache[10];
	sscanf(rx_cache,"A=%f,B=%f,C=%f,D=%f,E=%f,F=%f,G=%f,H=%f,I=%f,J=%f\n",&cache[0],&cache[1],&cache[2],&cache[3],&cache[4],&cache[5],&cache[6],&cache[7],&cache[8],&cache[9]);
	int i;
	for(i=0;i<RX_NUM;i++)
	{
		RX_nums[i]=cache[i];
	}
#endif

}

//******************************************中断函数*******************************************************//
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	if(huart == UART_head)
//	{	
//		Receive();
//		HAL_UARTEx_ReceiveToIdle_IT(UART_head,(uint8_t*)rx_cache,RX_NUM*20);
//	}
//}
