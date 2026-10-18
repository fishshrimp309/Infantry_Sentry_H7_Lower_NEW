#ifndef _POWERCTRAL_H_
#define _POWERCTRAL_H_

#include "struct_typedef.h"

#define TOQUE_CONST  600 //地胶摩擦系数

typedef struct
{
	float output;
	float LastOutput[4];//上一次的输出也就是当前的电流或力矩值
	float SumPowerSpeed;//转速平方
	float SumPowerTorque;//力矩平方
	float EffetivePower;//机械功率
	float InitialGivePower[4];//控制前功率
	float InitialTotalPower;//控制前总功率
	float PredictPower;//预测功率
	float MeasurePower;//测量功率
	float TotalPower;//总功率
	float scaleFactor;//放缩系数
	float paramVector[3][1];//动态拟合初始值矩阵
	float transVector[3][3];//动态拟合数据允许变化的范围
	float toque_coefficient;//单位转化，将电流和转速转换成实A和rad/s
	float a; //电流平方项系数
	float k2;//转速平方项系数
	float constant;//拟合常数项
	uint16_t moto_type;//电机类型
	uint16_t UserPowerLimit;
	uint16_t MaxPowerLimit;
	float InputPower;
}ChassisPower;

extern ChassisPower whell_power;
extern uint16_t SET_WHEELSPEED_MAX;

void PowerCtrl(void);
void PowerCtralInit(void);

#endif
