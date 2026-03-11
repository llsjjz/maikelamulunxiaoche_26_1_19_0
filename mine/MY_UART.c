#include "MY_UART.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

//**************************************全局变量******************************************************//
static uint8_t uart_rx_buf[DMA_RX_BUF_SIZE] = {0};          // DMA接收缓冲区
static uint8_t uart_tx_buf[DMA_RX_BUF_SIZE] = {0};          // DMA发送缓冲区
static UART_Cmd_t uart_cmd = {MOTION_STOP, 0.0f, 0};        // 解析后的串口指令

//******************************************UART******************************************************************//
/**
 * @brief UART DMA初始化（开启空闲中断+DMA接收）
 */
void My_UART_DMA_Init(void)
{
    // 开启串口DMA接收
    HAL_UART_Receive_DMA(UART_HANDLE, uart_rx_buf, DMA_RX_BUF_SIZE);
	HAL_UART_Receive_DMA(UART_BT, uart_rx_buf, DMA_RX_BUF_SIZE);
	HAL_UARTEx_ReceiveToIdle_DMA(UART_OLED,uart_rx_buf,DMA_RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
    // 开启串口空闲中断
    __HAL_UART_ENABLE_IT(UART_HANDLE, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(UART_BT, UART_IT_IDLE);
}

/**
 * @brief 串口指令解析（帧格式：0xAA + 模式(1B) + 速度(4B,float) + 舵机状态(1B) + 0x55）
 */
void My_UART_Cmd_Parse(void)
{
    // 1. 校验帧头帧尾
    if (uart_rx_buf[0] != UART_CMD_FRAME_HEAD || uart_rx_buf[7] != UART_CMD_FRAME_TAIL) {
        memset(uart_rx_buf, 0, DMA_RX_BUF_SIZE); // 清空缓冲区
        HAL_UART_Receive_DMA(UART_HANDLE, uart_rx_buf, DMA_RX_BUF_SIZE);
        return;
    }

    // 2. 解析指令
    uart_cmd.mode = (Motion_Mode_e)uart_rx_buf[1];                  // 运动模式
    memcpy(&uart_cmd.speed, &uart_rx_buf[2], sizeof(float));        // 速度值
    uart_cmd.sg_state = uart_rx_buf[6];                             // 舵机状态

    // 3. 合法性校验
    if (uart_cmd.mode > MOTION_CCW) uart_cmd.mode = MOTION_STOP;
    if (fabs(uart_cmd.speed) > 0.5f) uart_cmd.speed = 0.5f;        // 速度限制 0.5m/s

    // 4. 清空缓冲区，重新开启DMA接收
    memset(uart_rx_buf, 0, DMA_RX_BUF_SIZE);
    HAL_UART_Receive_DMA(UART_HANDLE, uart_rx_buf, DMA_RX_BUF_SIZE);
}

/**
 * @brief 获取解析后的串口指令
 */
UART_Cmd_t My_Get_UART_Cmd(void)
{
    return uart_cmd;
}

/**
 * @brief 将数据发送给UART3
 */
void My_UART_3(void)
{
	memcpy(uart_tx_buf,uart_rx_buf,8);
    HAL_UART_Transmit_DMA(UART_HANDLE,uart_tx_buf,8);
	HAL_UART_Receive_DMA(UART_BT, uart_rx_buf, DMA_RX_BUF_SIZE);
}

//接收不定长
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart==UART_OLED)
	{
		memcpy(uart_tx_buf,uart_rx_buf,Size);
		HAL_UART_Transmit_DMA(UART_HANDLE,uart_tx_buf,Size);
		My_UART_Cmd_Parse();
		HAL_UARTEx_ReceiveToIdle_DMA(UART_OLED,uart_rx_buf,DMA_RX_BUF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
	}
}