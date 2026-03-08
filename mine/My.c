#include "My.h"
#include <string.h>
#include <stdlib.h>

// 0左前 1右前 2左后 3右后 (麦克纳姆轮编号对应)
#define ZQ 0
#define YQ 1
#define ZH 2
#define YH 3

#define Length 0.2f         // 车辆长度 (m)
#define Wide 0.1f           // 车辆宽度 (m)
#define Radius 0.05f        // 轮子半径 (m)
#define Radius_1 0.05f        // 变形轮子半径 (m)

// 全局变量
vehicle_parameter vehicle_physics = {Length, Wide, Radius, Radius_1}; // 车辆物理参数
static uint8_t uart_rx_buf[DMA_RX_BUF_SIZE] = {0};          // DMA接收缓冲区
static UART_Cmd_t uart_cmd = {MOTION_STOP, 0.0f, 0};        // 解析后的串口指令

/**
 * @brief UART DMA初始化（开启空闲中断+DMA接收）
 */
void My_UART_DMA_Init(void)
{
    // 开启串口DMA接收
    HAL_UART_Receive_DMA(UART_HANDLE, uart_rx_buf, DMA_RX_BUF_SIZE);
    // 开启串口空闲中断
    __HAL_UART_ENABLE_IT(UART_HANDLE, UART_IT_IDLE);
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
 * @brief 麦克纳姆轮速度解算（核心优化）
 * @param speed 运动参数(Vx/Vy/w)
 * @param aim_speed 输出4个轮子的目标转速 (rpm)
 */
static void Mecanum_Wheel_Speed_Calc(speed_parameter *speed, float aim_speed[4])
{
    // 麦克纳姆轮速度公式（适配左前/右前/左后/右后布局）
    float L = vehicle_physics.length / 2 + vehicle_physics.wide / 2; // 轮距半和
    float omega = speed->w;
    float Vx = speed->Vx;
    float Vy = speed->Vy;

    // 转速公式：n (rpm) = (线速度 m/s) / (2πr m/转) * 60 (s/min)
    aim_speed[ZQ] = (Vy - Vx - omega * L) / (2 * 3.14159f * vehicle_physics.radius) * 60.0f;
    aim_speed[YQ] = (Vy + Vx + omega * L) / (2 * 3.14159f * vehicle_physics.radius) * 60.0f;
    aim_speed[ZH] = (Vy + Vx - omega * L) / (2 * 3.14159f * vehicle_physics.radius) * 60.0f;
    aim_speed[YH] = (Vy - Vx + omega * L) / (2 * 3.14159f * vehicle_physics.radius) * 60.0f;
}

/**
 * @brief 变形轮速度解算（核心优化）
 * @param speed 运动参数(Vy)
 * @param aim_speed 输出4个轮子的目标转速 (rpm)
 */
static void Wheel_Speed_Calc(speed_parameter *speed, float aim_speed[4])
{
    float Vy = speed->Vy;

    // 转速公式：n (rpm) = (线速度 m/s) / (2πr m/转) * 60 (s/min)
    aim_speed[ZQ] = Vy / (2 * 3.14159f * vehicle_physics.radius_1) * 60.0f;
    aim_speed[YQ] = Vy / (2 * 3.14159f * vehicle_physics.radius_1) * 60.0f;
    aim_speed[ZH] = Vy / (2 * 3.14159f * vehicle_physics.radius_1) * 60.0f;
    aim_speed[YH] = Vy / (2 * 3.14159f * vehicle_physics.radius_1) * 60.0f;
}

void My_Init(void)// 初始化
{
    // PID初始化
    MY_PID_Init();
    // 电机初始化
    DJ_init();
    // 舵机初始化
    SG_Init();
    // UART DMA初始化
    My_UART_DMA_Init();
}

void My_run(void)// 主逻辑，建议放在10ms定时器中断
{
    speed_parameter vehicle_speed = {0, 0, 0}; // 速度参数初始化
    UART_Cmd_t cmd = My_Get_UART_Cmd();        // 获取串口指令
    float aimspeed[4] = {0};

    // 1. 运动模式解析
    switch (cmd.mode)
    {
        case MOTION_STOP:
            vehicle_speed.Vx = 0;
            vehicle_speed.Vy = 0;
            vehicle_speed.w = 0;
            DJ_stop(1); // 硬停止
            break;
        case MOTION_FORWARD:    // 前进 (Vy正)
            vehicle_speed.Vy = cmd.speed;
            vehicle_speed.Vx = 0;
            vehicle_speed.w = 0;
            break;
        case MOTION_BACKWARD:   // 后退 (Vy负)
            vehicle_speed.Vy = -cmd.speed;
            vehicle_speed.Vx = 0;
            vehicle_speed.w = 0;
            break;
        case MOTION_LEFT:       // 左平移 (Vx正)
            vehicle_speed.Vx = cmd.speed;
            vehicle_speed.Vy = 0;
            vehicle_speed.w = 0;
            break;
        case MOTION_RIGHT:      // 右平移 (Vx负)
            vehicle_speed.Vx = -cmd.speed;
            vehicle_speed.Vy = 0;
            vehicle_speed.w = 0;
            break;
        case MOTION_CW:         // 顺时针旋转 (w正)
            vehicle_speed.w = cmd.speed * 3.14159f; // 速度值转弧度/s
            vehicle_speed.Vx = 0;
            vehicle_speed.Vy = 0;
            break;
        case MOTION_CCW:        // 逆时针旋转 (w负)
            vehicle_speed.w = -cmd.speed * 3.14159f;
            vehicle_speed.Vx = 0;
            vehicle_speed.Vy = 0;
            break;
        default:
            vehicle_speed.Vx = 0;
            vehicle_speed.Vy = 0;
            vehicle_speed.w = 0;
            DJ_stop(1);
            break;
    }

    // 2. 麦克纳姆轮速度解算
    if(cmd.sg_state==0)Mecanum_Wheel_Speed_Calc(&vehicle_speed, aimspeed);
	else if(cmd.sg_state==1)Wheel_Speed_Calc(&vehicle_speed, aimspeed);

    // 3. 速度限幅（防止超过电机最大转速）
    float max_speed = 0;
    uint8_t max_idx = 0;
    for(uint8_t nums=0; nums<4; nums++){
        if(fabs(aimspeed[nums]) > max_speed){
            max_speed = fabs(aimspeed[nums]);
            max_idx = nums;
        }
    }
    // 等比例缩放速度
    if(max_speed > motor[max_idx].max_speed && motor[max_idx].max_speed	 > 0){
        float scale = motor[max_idx].max_speed / max_speed;
        for(uint8_t nums=0; nums<4; nums++){
            aimspeed[nums] *= scale;
        }
    }

    // 4. 舵机控制
    if(cmd.sg_state == 1){
        SG_Open();
    }else{
        SG_Close();
    }

    // 5. 设置电机目标转速
    for(uint8_t nums=0; nums<4; nums++){
        DJ_Wheel_Set(nums, aimspeed[nums]);
    }
}
