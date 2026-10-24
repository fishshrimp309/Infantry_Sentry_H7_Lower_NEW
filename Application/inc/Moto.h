#ifndef _MOTO_H_
#define _MOTO_H_

#include "USER_Moto.h"
#include "PID.h"
#include "super_cap.h"
#include "Slope.h"

typedef struct CHASSIS
{
	DJI_Motor_t M3508[4];
	
	struct Info
	{
		float wheelbase;	// 轴距
		float wheeltrack;	// 轮距
		float wheelRadius;	// 轮半径
		float R;			// 轮子到中心的距离
	} info;
} ChassisMotor_t;

typedef struct Move
{
	float vx; // 当前左右平移速度 mm/s左为正
	float vy; // 当前前后移动速度 mm/s前为正
	float vw; // 当前旋转速度 rad/逆时针为正

	float maxVx, maxVy, maxVw; // 三个分量最大速度

	float Wheelangle[4];
	Slope xSlope, ySlope, outputSlope, chargeSlope, spinSlope; // 斜坡
	PID buffer_pid;
	float out60_slope;
	float charge_slope;
	uint8_t cap_output;
	float maxPower;
	uint8_t last_power_management;
	uint8_t fastMode; // 快速模式  0-普通模式 1-快速模式

} Move;

extern ChassisMotor_t chassis;
extern Move move;
void Supercap_Update(SuperCap* this, uint8_t* rxdata);

#endif
