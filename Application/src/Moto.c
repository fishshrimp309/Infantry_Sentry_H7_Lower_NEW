#include "Moto.h"
#include "USER_Moto.h"
#include "bsp_can.h"
#include "moto.h"
#include "super_cap.h"
#include "RLS.h"
#include "arm_math.h"
#include "USER_B2B.h"
#include "PowerCtrl.h"
#include "JUDGE.h"
#include "math.h"

float test_I0,test_I1,test_I2,test_I3;

ChassisMotor_t chassis = {0}; //所有底盘电机结构体 包括四个轮电�?
float sumPowTorque =0;
float sumPowSpeed = 0;
float powerPredict;
float effetive_power;
float toque_coefficient=1;
float measure_power = 0.0;

float p_des = 0;
float p_des_yaw = 0;
Move move = {0};


void Supercap_Update(SuperCap* this, uint8_t* rxdata)
{
	  // 状态变�?
    this->receive_data.power_ctrl_mode = rxdata[0];
    // 状态变�?dcdcmode.autoMode.stage
    this->receive_data.automode_stage = rxdata[1];
    // 电容组电�?CapVoltage
    this->receive_data.cap_voltage = (int16_t)(rxdata[2] | (rxdata[3] << 8));    
		//电感电流
		this->receive_data.L_current = (int16_t)(rxdata[4]|rxdata[5]<<8);
    // 总线电压 Buspower
    this->receive_data.bus_power = (int16_t)(rxdata[6] | (rxdata[7] << 8));
}


static inline float sgn(float val) {
    if (val > 10.0f) return 1.0f;
    if (val < -10.0f) return -1.0f;
    return 0.0f;
}

#define cos45   0.70710678f
float threshold_speed_err = 50.0f; 
float k_NULL = 1.5f; 
float GAMMA = 0.03f;
float LPF_ALPHA = 0.10f;//低通滤波系数(0.05 ~ 0.2，越小越平滑，抗震颤效果越好)

void Torque_Control()
{
    static float scale_smooth[4] = {1.0f, 1.0f, 1.0f, 1.0f};

    move.vy = (-chassis.M3508[0].speed + chassis.M3508[1].speed - chassis.M3508[2].speed + chassis.M3508[3].speed) * 2.0f * PI / 60.0f * 0.075f * cos45 / 4.0f;
    move.vx = ( chassis.M3508[0].speed + chassis.M3508[1].speed - chassis.M3508[2].speed - chassis.M3508[3].speed) * 2.0f * PI / 60.0f * 0.075f * cos45 / 4.0f;
    move.vw = ( chassis.M3508[0].speed + chassis.M3508[1].speed + chassis.M3508[2].speed + chassis.M3508[3].speed) * 2.0f * PI / 60.0f * 0.075f / 0.297f / 4.0f;

    float speed_pre[4];
    speed_pre[0] = -move.vy / cos45 / 0.075f * 60.0f / 2.0f / PI + move.vx / cos45 / 0.075f * 60.0f / 2.0f / PI + move.vw * 0.297f / 0.075f * 60.0f / 2.0f / PI;
    speed_pre[1] =  move.vy / cos45 / 0.075f * 60.0f / 2.0f / PI + move.vx / cos45 / 0.075f * 60.0f / 2.0f / PI + move.vw * 0.297f / 0.075f * 60.0f / 2.0f / PI;
    speed_pre[2] = -move.vy / cos45 / 0.075f * 60.0f / 2.0f / PI - move.vx / cos45 / 0.075f * 60.0f / 2.0f / PI + move.vw * 0.297f / 0.075f * 60.0f / 2.0f / PI;
    speed_pre[3] =  move.vy / cos45 / 0.075f * 60.0f / 2.0f / PI - move.vx / cos45 / 0.075f * 60.0f / 2.0f / PI + move.vw * 0.297f / 0.075f * 60.0f / 2.0f / PI;

    float e_slip[4] = {0.0f};

    for(uint8_t i = 0; i < 4; i++)
    {
        float speed_real_abs   = fabsf((float)chassis.M3508[i].speed);
        float speed_pre_abs    = fabsf(speed_pre[i]);
        float speed_target_abs = fabsf((float)chassis.M3508[i].targetSpeed);

        float dir = sgn((float)chassis.M3508[i].targetSpeed);
        if (dir == 0.0f) dir = sgn((float)chassis.M3508[i].speed);

        // A. 运动学残差
        float e_1 = 0.0f;
        if ((sgn((float)chassis.M3508[i].speed) == dir) && (speed_real_abs > speed_pre_abs + threshold_speed_err)) {
            e_1 = speed_real_abs - speed_pre_abs;
        }

        // B. 目标残差
        float e_2 = 0.0f;
        if ((sgn((float)chassis.M3508[i].speed) == dir) && (speed_real_abs > speed_target_abs + threshold_speed_err)) {
            e_2 = speed_real_abs - speed_target_abs;
        }

        e_slip[i] = fmaxf(e_1, e_2);

        // 计算原始 scale
        float scale_raw = 1.0f;
        if (e_slip[i] > 0.0f) {
            scale_raw = expf(-GAMMA * (e_slip[i] - threshold_speed_err));
            if (scale_raw < 0.01f) scale_raw = 0.1f;
        }

        scale_smooth[i] = scale_smooth[i] * (1.0f - LPF_ALPHA) + scale_raw * LPF_ALPHA;
    }

    const float k_vec[4] = {1.0f, -1.0f, -1.0f, 1.0f};
    float total_cof = 0.0f;

    for(uint8_t i = 0; i < 4; i++) {
        if (e_slip[i] > 0.0f) {
            float dir = sgn((float)chassis.M3508[i].targetSpeed);
            if (dir == 0.0f) dir = sgn((float)chassis.M3508[i].speed);
            
            total_cof += - k_NULL * k_vec[i] * dir * e_slip[i];
        }
    }

    for(uint8_t i = 0; i < 4; i++)
    {
        float effective_target_speed = chassis.M3508[i].targetSpeed * scale_smooth[i];   //低通滤波平滑目标速度

        PID_SingleCalc(&chassis.M3508[i].speedPID, effective_target_speed, chassis.M3508[i].speed);

        float I_final = chassis.M3508[i].speedPID.output + total_cof * k_vec[i];

        chassis.M3508[i].speedPID.output = I_final;
    }
}



/**底盘**/
void Chassis_InitPID()
{
	for (uint8_t i = 0; i < 4; i++)
	{
		PID_Init(&chassis.M3508[i].speedPID, 10, 0, 6, 8000, 16000);
	}
	//		PID_Init(&chassis.move.buffer_pid, 2.5, 0.1, 0, 10, 40);		// 缓冲能量pid 暂时不加
		PID_SetDeadzone(&chassis.M3508[0].speedPID, 0);
		PID_SetDeadzone(&chassis.M3508[1].speedPID, 0);
		PID_SetDeadzone(&chassis.M3508[2].speedPID, 0);
		PID_SetDeadzone(&chassis.M3508[3].speedPID, 0);
}

void Task_CANMotors_Callback()
{
		Torque_Control();
//		for (uint8_t i = 0; i < 4; i++)
//		{
//		 	PID_SingleCalc(&chassis.M3508[i].speedPID,chassis.M3508[i].targetSpeed,chassis.M3508[i].speed);
//		}
		test_I0 = -chassis.M3508[0].speedPID.output;
		test_I1 = chassis.M3508[1].speedPID.output;
		test_I2 = -chassis.M3508[2].speedPID.output;
		test_I3 = chassis.M3508[3].speedPID.output;
	
		PowerCtrl();
	    USER_CAN_SetMotorCurrent(&hfdcan1, 0x200, chassis.M3508[0].speedPID.output,chassis.M3508[1].speedPID.output,chassis.M3508[2].speedPID.output,chassis.M3508[3].speedPID.output);
		USER_CAN_SendCapData(&hfdcan2,0x1aa,1,(int16_t)JUDGE_GetPowerBuffer(),(int16_t)JUDGE_GetChassisPowerLimit(),cap.targetI);
} 



/************************freertos任务*******************  *********/
void OS_MotorCallback(void const * argument)
{
	osDelay(1500);
	Chassis_InitPID();
	PowerCtralInit();
    for(;;)
	    {	
		if(rs485_isvalid){
			Task_CANMotors_Callback();
			rs485_isvalid = 0;
		}
		osDelay(1);
    }
}
