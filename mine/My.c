#include "My.h"

//***********************************宏定义************************************************************************//
// 0左前 1右前 2左后 3右后 (麦克纳姆轮编号对应)
#define ZQ 0
#define YQ 1
#define ZH 2
#define YH 3

#define Length 0.2f         // 车辆长度 (m)
#define Wide 0.1f           // 车辆宽度 (m)
#define Radius 0.05f        // 轮子半径 (m)
#define Radius_1 0.05f        // 变形轮子半径 (m)

//**********************************************变量*****************************************************************//
vehicle_parameter vehicle_physics = {Length, Wide, Radius, Radius_1}; // 车辆物理参数

//******************************************私有函数******************************************************************//

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


//****************************************api****************************************************************//
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
