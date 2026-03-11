#ifndef __DJ_H
#define __DJ_H

#include <stdint.h>

#include "PID.h"

//******************************************结构体*******************************************************//
typedef struct
{
	int DJ_Speed;			//电机每分转速
	int DJ_MaxSpeed;		//电机最高转速
	int DJ_Target_Speed;	//电机目标转速
}DJ_Parameter;			//电机参数

//******************************************全局变量*******************************************************//

extern DJ_Parameter DJ[4];				//电机参数
extern PID_HandleTypeDef PID[4];		//电机PID参数
extern int DJ_State;					//电机状态机

//******************************************函数*******************************************************//

void MY_PID_Init(void);			//初始化PID参数
void DJ_init(void);//电机初始化

void DJ_base_run(int Sn,int speed);//电机基本运行
void DJ_Wheel_Set(int num,int speed);//车轮基本转动
void DJ_stop(uint8_t move);//电机停止

#endif
