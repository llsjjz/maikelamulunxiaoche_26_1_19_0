#include "SG.h"

//========================================= 宏定义 =========================================//
// 舵机通用配置（魔法数字宏定义，提升可读性和可维护性）
#define SG_PWM_TIMER        &htim8                  // 舵机驱动定时器句柄
#define SG_PWM_MIN_DUTY     5                       // 舵机0°对应的PWM占空比（根据实际舵机调整）
#define SG_PWM_MAX_DUTY     25                      // 舵机180°对应的PWM占空比（5 + 180*20/180 = 25）
#define SG_ANGLE_MIN        0.0f                    // 舵机最小角度（°）
#define SG_ANGLE_MAX        180.0f                  // 舵机最大角度（°）
#define SG_TOTAL_NUM        4                       // 舵机总数
#define SG_OPEN_ANGLE       90.0f                   // 舵机张开角度（°）
#define SG_CLOSE_ANGLE      0.0f                    // 舵机关闭角度（°）
#define SG_TRANSFORM_TIME   3000                    // 变形时间（ms，预留扩展使用）

// 舵机通道定义（命名语义化，注释清晰）
#define SG_CHANNEL_1        TIM_CHANNEL_1           // 舵机1对应的PWM通道
#define SG_CHANNEL_2        TIM_CHANNEL_2           // 舵机2对应的PWM通道
#define SG_CHANNEL_3        TIM_CHANNEL_3           // 舵机3对应的PWM通道
#define SG_CHANNEL_4        TIM_CHANNEL_4           // 舵机4对应的PWM通道

//========================================= 结构体定义 =========================================//
/**
 * @brief 舵机参数结构体（命名语义化，注释完善）
 * @param pwm_timer: 舵机PWM对应的定时器句柄
 * @param pwm_channel: 舵机PWM对应的通道
 */
typedef struct {
    TIM_HandleTypeDef* pwm_timer;    // PWM定时器句柄
    uint32_t pwm_channel;            // PWM通道
} SG_Motor_Config_t;

//========================================= 全局变量 =========================================//
/**
 * @brief 4路舵机的配置数组（初始化各舵机的定时器和通道）
 */
static const SG_Motor_Config_t sg_motor_config[SG_TOTAL_NUM] = {
    {SG_PWM_TIMER, SG_CHANNEL_1},
    {SG_PWM_TIMER, SG_CHANNEL_2},
    {SG_PWM_TIMER, SG_CHANNEL_3},
    {SG_PWM_TIMER, SG_CHANNEL_4}
};

//========================================= 私有函数声明（可选） =========================================//
static uint32_t SG_AngleToDuty(float angle);  // 角度转PWM占空比
static uint8_t SG_CheckParams(uint8_t num, float angle); // 参数合法性校验

//========================================= 公有函数实现 =========================================//
/**
 * @brief 舵机初始化函数
 * @note 启动所有舵机对应的PWM通道输出
 * @retval 无
 */
void SG_Init(void)
{
    // 循环变量定义在for内，符合C99标准
    for (uint8_t i = 0; i < SG_TOTAL_NUM; i++)
    {
        // 启动PWM输出，增加HAL库返回值判断，提升健壮性
        if (HAL_OK != HAL_TIM_PWM_Start(sg_motor_config[i].pwm_timer, sg_motor_config[i].pwm_channel))
        {
            // 可添加错误处理（如日志、指示灯）
            Error_Handler(); // 需确保工程中已实现该错误处理函数
        }
    }
}

/**
 * @brief 单个舵机角度控制函数（核心控制逻辑）
 * @param num: 舵机编号（0~3，对应1~4号舵机）
 * @param angle: 舵机目标角度（0~180°）
 * @retval 0: 成功，1: 参数错误
 */
uint8_t SG_SetAngle(uint8_t num, float angle)
{
    // 第一步：参数合法性校验
    if (SG_CheckParams(num, angle) != 0)
    {
        return 1; // 参数错误，返回错误码
    }

    // 第二步：角度转换为PWM占空比
    uint32_t duty = SG_AngleToDuty(angle);

    // 第三步：设置PWM占空比，控制舵机角度
    __HAL_TIM_SetCompare(sg_motor_config[num].pwm_timer, 
                         sg_motor_config[num].pwm_channel, 
                         duty);

    return 0; // 执行成功
}

/**
 * @brief 所有舵机张开到指定角度（批量控制）
 * @retval 0: 成功，1: 参数错误
 */
uint8_t SG_Open(void)
{
    for (uint8_t i = 0; i < SG_TOTAL_NUM; i++)
    {
        if (SG_SetAngle(i, SG_CLOSE_ANGLE) != 0)
        {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief 所有舵机关闭到初始角度（批量控制）
 * @retval 0: 成功，1: 参数错误
 */
uint8_t SG_Close(void)
{
    for (uint8_t i = 0; i < SG_TOTAL_NUM; i++)
    {
        if (SG_SetAngle(i, SG_OPEN_ANGLE) != 0)
        {
            return 1;
        }
    }
    return 0;
}

//========================================= 私有函数实现 =========================================//
/**
 * @brief 舵机角度转PWM占空比（线性映射）
 * @param angle: 目标角度（已校验合法性）
 * @retval 对应的PWM占空比值
 */
static uint32_t SG_AngleToDuty(float angle)
{
    // 线性映射公式：duty = 最小值 + (角度/最大角度)*(最大值-最小值)
    return (uint32_t)(SG_PWM_MIN_DUTY + (angle / SG_ANGLE_MAX) * (SG_PWM_MAX_DUTY - SG_PWM_MIN_DUTY));
}

/**
 * @brief 舵机控制参数合法性校验
 * @param num: 舵机编号
 * @param angle: 目标角度
 * @retval 0: 合法，1: 编号错误，2: 角度错误
 */
static uint8_t SG_CheckParams(uint8_t num, float angle)
{
    // 校验舵机编号范围
    if (num >= SG_TOTAL_NUM)
    {
        return 1;
    }

    // 校验角度范围（防止超出舵机物理极限）
    if (angle < SG_ANGLE_MIN || angle > SG_ANGLE_MAX)
    {
        return 2;
    }

    return 0;
}