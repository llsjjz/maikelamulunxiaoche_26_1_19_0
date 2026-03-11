#include <stdio.h>    // 控制台输入输出（scanf/printf）
#include <stdint.h>   // 标准整数类型（uint8_t）
#include <stdlib.h>   // atof（字符串转float）、exit（异常退出）
#include <string.h>   // memset（清空缓冲区）

/**
 * @brief 将float类型速度值转换为小端模式的4字节数组
 * @param f_val: 输入的float速度值（如1.0f、2.0f）
 * @param out_bytes: 输出的4字节数组（小端模式，低字节在前）
 * @retval 0=转换成功，1=空指针错误
 */
uint8_t FloatToLittleEndianBytes(float f_val, uint8_t out_bytes[4])
{
    // 空指针校验，防止程序崩溃
    if (out_bytes == NULL) {
        printf("【错误】输出字节数组指针为空！\n");
        return 1;
    }

    // 共用体：共享内存，直接访问float的IEEE 754字节表示（小端原生适配）
    union {
        float f_data;       // 浮点数值
        uint8_t b_data[4];  // 对应4字节（小端：b_data[0] = 最低字节）
    } float2bytes;

    // 赋值并转换（小端模式无需调整字节顺序）
    float2bytes.f_data = f_val;
    out_bytes[0] = float2bytes.b_data[0];
    out_bytes[1] = float2bytes.b_data[1];
    out_bytes[2] = float2bytes.b_data[2];
    out_bytes[3] = float2bytes.b_data[3];

    return 0;
}

/**
 * @brief 控制台输入速度值并做合法性校验
 * @param out_speed: 输出校验后的float速度值
 * @retval 0=输入成功，1=输入非法/失败
 */
uint8_t InputSpeedFromConsole(float *out_speed)
{
    char input_buf[32] = {0};  // 存储控制台输入的字符串
    float speed = 0.0f;

    // 提示用户输入
    printf("\n====================================\n");
    printf("请输入速度值（示例：1.0、2.0、-1.5）：");
    
    // 读取输入的字符串（避免scanf直接读float的格式错误）
    if (fgets(input_buf, sizeof(input_buf), stdin) == NULL) {
        printf("【错误】输入失败！\n");
        return 1;
    }

    // 字符串转float
    speed = atof(input_buf);

    // 速度范围校验（可根据需求修改，示例限制-5.0~5.0）
    const float MIN_SPEED = -10.0f;
    const float MAX_SPEED = 10.0f;
    if (speed < MIN_SPEED || speed > MAX_SPEED) {
        printf("【错误】输入非法！速度范围需在 %.1f ~ %.1f 之间\n", MIN_SPEED, MAX_SPEED);
        return 1;
    }

    // 输出有效速度值
    *out_speed = speed;
    printf("【成功】输入的速度值：%.2f\n", speed);
    return 0;
}

// 主函数：控制台输入 → 转换 → 打印结果
int main(void)
{
    float target_speed = 0.0f;    // 存储输入的速度值
    uint8_t speed_bytes[4] = {0}; // 存储转换后的小端4字节
    uint8_t ret = 0;

    printf("=== Float速度值转小端4字节工具（PC端）===\n");

    // 循环输入（按Ctrl+C退出）
    while (1) {
        // 1. 控制台输入速度值
        ret = InputSpeedFromConsole(&target_speed);
        if (ret != 0) {
            continue; // 输入错误，重新输入
        }

        // 2. 转换为小端4字节
        ret = FloatToLittleEndianBytes(target_speed, speed_bytes);
        if (ret != 0) {
            continue; // 转换错误，重新输入
        }

        // 3. 打印转换结果（十六进制，小端模式）
        printf("【转换结果】小端4字节（低→高）：");
        printf("0x%02X 0x%02X 0x%02X 0x%02X\n", 
               speed_bytes[0], speed_bytes[1], speed_bytes[2], speed_bytes[3]);
    }

    return 0;
}
