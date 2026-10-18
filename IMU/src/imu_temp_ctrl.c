#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
//#include "tim.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "imu_temp_ctrl.h"
#include "MahonyAHRS.h"
#include "controller.h"
//#include "bsp_PWM.h"

#define cheat TRUE  //作弊模式 去掉较小的gyro值
#define correct_Time_define 1   //上电去0飘 1000次取平均
#define temp_times 1       //探测温度阈值

INS_t INS;
PID_t TempCtrl = {0};


float gyro_correct[3]={0};
float RefTemp = 40;   //Destination
uint8_t attitude_flag=0;
uint32_t correct_times=0;


void INS_Init(void)
{
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f,0); //ekf初始化
		Mahony_Init(1000);  //mahony姿态解算初始化
    while(BMI088_init());  //陀螺仪初始化
}

uint32_t temp_temperature=0;

void IMU_Temperature_Ctrl(void)  //目前没给pid参数  之后可以尝试给给
{
//    PID_Calculate(&TempCtrl, INS.temp, RefTemp);
//		TIM_Set_PWM(&htim3, TIM_CHANNEL_4, float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}

/***
 * @brief: INS_TASK(void const * argument)
 * @param: argument - 任务参数
 * @retval: void
 * @details: IMU姿态控制任务函数
 
*/
static uint8_t first_mahony=0; 
void INS_Task(void)
{
    static uint32_t count = 0;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_read(INS.gyro, INS.accel,&INS.temp);
				if(first_mahony==0)
				{
					first_mahony++;
					MahonyAHRSinit(INS.accel[0],INS.accel[1],INS.accel[2],0,0,0);  
				}
				if(attitude_flag==2)  //ekf的姿态解算
				{
					gyro_correct[0] = 0.00450826762;
					gyro_correct[1] = -0.000528795645;
					gyro_correct[2] = -0.00197213679;
					INS.gyro[0]-=gyro_correct[0];   //减去陀螺仪0飘
					INS.gyro[1]-=gyro_correct[1];
					INS.gyro[2]-=gyro_correct[2];
					
					#if cheat              //作弊 可以让yaw很稳定 去掉比较小的值
						if(fabsf(INS.gyro[2])<0.003f)
							INS.gyro[2]=0;
					#endif
					//===========================================================================
						//ekf姿态解算部分
					IMU_QuaternionEKF_Update(INS.gyro[0],INS.gyro[1],INS.gyro[2],INS.accel[0],INS.accel[1],INS.accel[2]);
					//===============================================================================	
						
					//=================================================================================
					//mahony姿态解算部分
//					Mahony_update(INS.gyro[0],INS.gyro[1],INS.gyro[2],INS.accel[0],INS.accel[1],INS.accel[2],0,0,0);
//					Mahony_computeAngles(); //角度计算   移植到别的平台需要替换掉对应的arm_atan2_f32 和 arm_asin
					//=============================================================================
					//ekf获取姿态角度函数
					INS.pitch=Get_Pitch(); //获得pitch
					INS.roll=Get_Roll();//获得roll
					INS.yaw=Get_Yaw();//获得yaw
					//==============================================================================
				}
				else if(attitude_flag==1)   //状态1 开始1000次的陀螺仪0飘初始化
				{
						//gyro correct
						gyro_correct[0]+=	INS.gyro[0];
						gyro_correct[1]+=	INS.gyro[1];
						gyro_correct[2]+=	INS.gyro[2];
						correct_times++;
						if(correct_times>=correct_Time_define)
						{
							gyro_correct[0]/=correct_Time_define;
							gyro_correct[1]/=correct_Time_define;
							gyro_correct[2]/=correct_Time_define;
							attitude_flag=2; //go to 2 state
						}
				}
    }

    // temperature control
    if ((count % 10) == 0)
    {
        // 100hz 的温度控制pid
//        IMU_Temperature_Ctrl();
				
				static uint32_t temp_Ticks=0;
				if((fabsf(INS.temp-RefTemp)<30.0f)&&attitude_flag==0) //接近额定温度之差小于0.5° 开始计数
				{
					temp_Ticks++;
					if(temp_Ticks>temp_times)   //计数达到一定次数后 才进入0飘初始化 说明温度已经达到目标
					{
						attitude_flag=1;  //go to correct state
					}
				}
    }
    count++;
}

void OS_IMUCallback(void  * argument)
{
    INS_Init();
    /* Infinite loop */
    for (;;)
    {
        // INS_Task();
        osDelay(1);
    }
}


