#include "My.h"

//0前左，1前右，2后左，3后右
#define ZQ 0
#define YQ 1
#define ZH 2
#define YH 3

#define Length 0.2		//长
#define Wide 0.1		//宽
#define Radius 0.05			//半径

vehicle_parameter vehicle_physics={Length,Wide,Radius};//车辆物理参数


void My_Init(void)//初始化
{
	//PID
	MY_PID_Init();
	//DJ
	DJ_init();
	//SG
	SG_Init();
	//UART
	Receive_Init();
	
	
}

void My_run(void)//总运行,放在接收中断
{
	speed_parameter vehicle_speed={0,0,0};//车辆速度参数
	float R[4];
	RX_nums_read(R);//从接收缓冲区读出	0-模式，1-速度水平为m/s，旋转为rad/s，2-变形	

	DJ_State=0;
	
	//模式选择
	switch ((int)R[0])
	{
		case 0:		//停止
		{
			DJ_stop(1);
			vehicle_speed.Vx=0;
			vehicle_speed.Vy=0;
			vehicle_speed.w=0;
			break;
		}
		case 1:		//前进
		{
			vehicle_speed.Vx=0;
			vehicle_speed.Vy=R[1];
			vehicle_speed.w=0;
			break;
		}
		case 2:		//后退
		{
			vehicle_speed.Vx=0;
			vehicle_speed.Vy=-R[1];
			vehicle_speed.w=0;
			break;
		}
		case 3:		//左移
		{
			vehicle_speed.Vx=R[1];
			vehicle_speed.Vy=0;
			vehicle_speed.w=0;
			break;
		}
		case 4:		//右移
		{
			vehicle_speed.Vx=-R[1];
			vehicle_speed.Vy=0;
			vehicle_speed.w=0;
			break;
		}
		case 5:		//逆时针
		{
			vehicle_speed.Vx=0;
			vehicle_speed.Vy=0;
			vehicle_speed.w=-R[1];
			break;
		}
		case 6:		//顺时针
		{
			vehicle_speed.Vx=0;
			vehicle_speed.Vy=0;
			vehicle_speed.w=R[1];			
			break;
		}
	}
	
	//速度计算
	float aimspeed[4];
	aimspeed[0]=(0-vehicle_speed.Vx-vehicle_speed.Vy-vehicle_speed.w*(vehicle_physics.length/2+vehicle_physics.wide/2))/vehicle_physics.radius/2/3.14*60;
	aimspeed[1]=(0-vehicle_speed.Vx+vehicle_speed.Vy-vehicle_speed.w*(vehicle_physics.length/2+vehicle_physics.wide/2))/vehicle_physics.radius/2/3.14*60;
	aimspeed[2]=(vehicle_speed.Vx-vehicle_speed.Vy-vehicle_speed.w*(vehicle_physics.length/2+vehicle_physics.wide/2))/vehicle_physics.radius/2/3.14*60;
	aimspeed[3]=(vehicle_speed.Vx+vehicle_speed.Vy-vehicle_speed.w*(vehicle_physics.length/2+vehicle_physics.wide/2))/vehicle_physics.radius/2/3.14*60;	
	
	//找到最大转速
	uint8_t nums;
	uint8_t x;
	float max_speed=0;
	for(nums=0;nums<4;nums++)
	{
		if(max_speed<fabs(aimspeed[nums]))
		{
			max_speed=fabs(aimspeed[nums]);
			x=nums;
		}
	}
	//如果最大速度大于限制，等比缩小
	if(max_speed>DJ[x].DJ_MaxSpeed)
	{
		for(nums=0;nums<4;nums++)
		{
			aimspeed[nums]*=(DJ[x].DJ_MaxSpeed/max_speed);
		}
	}	
	//变形判断
	if(R[2]==1.0f)
	{
		SG_Open();
	}
	else if(R[2]==0.0f)
	{
		SG_Close();
	}
	//更新目标速度
	for(nums=0;nums<4;nums++)DJ_Wheel_Set(nums,aimspeed[nums]);
	
}

//*****************************************************调试专用******************************************//
void My_vofa_scan(int num)//查看速度波形
{
	  float W[5]={DJ[num].DJ_Target_Speed,DJ[num].DJ_Speed,PID[num].Kp,PID[num].Ki,PID[num].Kd};
	  TX_nums_write(W);
	  Transmit();
}

void My_Uart_scan(int num)//通过UART修改PID参数和转速，使用时加入UART中断
{
		float R[4];
		RX_nums_read(R);//从接收缓冲区读出
	
		PID[num].Kp=R[0];
		PID[num].Ki=R[1];
		PID[num].Kd=R[2];
		DJ_Wheel_Set(num,R[3]);
}
