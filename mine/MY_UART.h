#ifndef __MY_UART_H
#define __MY_UART_H

#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

//****************************************宏定义**********************************************//
// 单位 m/s m RPW(转每分钟) rad/s
#define UART_HANDLE         &huart2          // 串口句柄（根据实际硬件修改）-与树莓派通讯
#define UART_BT		        &huart1          // 串口句柄（根据实际硬件修改）-与蓝牙通讯
#define UART_OLED           &huart3          // 串口句柄（根据实际硬件修改）-与触碰通讯
#define DMA_RX_BUF_SIZE    32              // DMA接收缓冲区大小
#define UART_CMD_FRAME_HEAD 0xAA           // 指令帧头
#define UART_CMD_FRAME_TAIL 0x55           // 指令帧尾

//************************************结构体**********************************************//
// 运动模式枚举
typedef enum {
    MOTION_STOP = 0,        // 停止
    MOTION_FORWARD = 1,     // 前进
    MOTION_BACKWARD = 2,    // 后退
    MOTION_LEFT = 3,        // 左平移
    MOTION_RIGHT = 4,       // 右平移
    MOTION_CW = 5,          // 顺时针旋转
    MOTION_CCW = 6          // 逆时针旋转
} Motion_Mode_e;

// 串口指令结构体
typedef struct {
    Motion_Mode_e mode;     // 运动模式
    float speed;            // 运动速度 (m/s 或 rad/s)
    uint8_t sg_state;       // 舵机状态 0-关闭 1-打开
} UART_Cmd_t;

//*************************************API****************************************************//
void My_UART_DMA_Init(void);         // UART DMA初始化

void My_UART_Cmd_Parse(void);        // 串口指令解析

UART_Cmd_t My_Get_UART_Cmd(void);    // 获取解析后的指令

void My_UART_3(void);				//发送给树莓派



#endif
