#include "USER_B2B.h"
#include "usart.h"
#include "cmsis_os.h"
#include "UserFreertos.h"
#include "Moto.h"
#include "judge.h"
#include "USER_Detcet.h"
#include "imu_temp_ctrl.h"
#include <string.h>
#include "PowerCtrl.h"

#define B2B_FRAME_LEN 64U
extern DMA_HandleTypeDef hdma_usart2_rx;
uint8_t rs485_isvalid = 0;
uint8_t vision_mode;  //自瞄模式
uint8_t visionFindcheck;  //自瞄是否锁定
uint8_t chassis_rotate_mode;  //底盘模式
int16_t fricMotor_left_speed;  //Ħ����ת��
uint8_t trigger_block;   //�����Ƿ�����
uint8_t trigger_reverse; //���̹���״̬
int16_t chassis_rotate_angle; //��̨�������ԽǶ�
uint8_t gimbal_fold;//�۵���־λ 0δ�۵� 1�۵�


/* 需要用到的接收变量*/
uint8_t usart2RxBuf[256]; //����2������
uint8_t STOPFLAG = 0;
uint8_t FEEDBACK = 0;


uint32_t receive_times;

/* 需要用到的发送变�?/
*/
uint8_t txbuffer[64] = {0};

void B2B_Init()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2RxBuf, sizeof(usart2RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}


void B2B_ParseUsart()
{
	if (usart2RxBuf[0] == 0xAA && usart2RxBuf[B2B_FRAME_LEN - 1U] == 0xFE)
	{
		/*先解析接收内�?*/
		for (uint8_t i = 0; i < 4; i++)
		{
				chassis.M3508[i].targetSpeed = (int16_t)usart2RxBuf[1 + i * 2] | (int16_t)usart2RxBuf[1 + i * 2 + 1] << 8;
		} //轮电机目标速度	1-8
		move.maxVx = (int16_t)((uint16_t)usart2RxBuf[9] |((uint16_t)usart2RxBuf[10] << 8)) / 1000.0f;
		move.maxVy = (int16_t)((uint16_t)usart2RxBuf[11] |((uint16_t)usart2RxBuf[12] << 8)) / 1000.0f;
		move.xSlope.value = (int16_t)((uint16_t)usart2RxBuf[13] |((uint16_t)usart2RxBuf[14] << 8)) / 1000.0f;
		move.ySlope.value = (int16_t)((uint16_t)usart2RxBuf[15] |((uint16_t)usart2RxBuf[16] << 8)) / 1000.0f;
		move.fastMode = usart2RxBuf[17]; //快速模�?0-普通模�?1-快速模�?		 //自瞄模式
		vision_mode = usart2RxBuf[18]; 
		visionFindcheck = usart2RxBuf[19];//自瞄是否锁定  
		chassis_rotate_mode = usart2RxBuf[20];  //底盘模式
		fricMotor_left_speed = (int16_t)((uint16_t)usart2RxBuf[21] |((uint16_t)usart2RxBuf[22] << 8)) ;  //摩擦轮转�?		trigger_reverse = usart2RxBuf[23];   //拨盘工作状�?		trigger_block = usart2RxBuf[24]; //拨盘是否锁定
		chassis_rotate_angle = (int16_t)((uint16_t)usart2RxBuf[25] |((uint16_t)usart2RxBuf[26] << 8)); //云台与底盘相对角�?		gimbal_fold =   usart2RxBuf[27];
		
		trigger_reverse = usart2RxBuf[23];
		trigger_block = usart2RxBuf[24];
		gimbal_fold = usart2RxBuf[27];
		STOPFLAG = usart2RxBuf[62];
		

		
		/* 发�?   */
		txbuffer[0] = 0xAB;

		for (uint8_t i = 0; i < 4; i++)
		{
				txbuffer[1 + i * 2] = chassis.M3508[i].speed;
				txbuffer[1 + i * 2 + 1] = chassis.M3508[i].speed >> 8;
		} // 1-8 轮电机当前速度
		txbuffer[9] = SET_WHEELSPEED_MAX;
		txbuffer[10] = SET_WHEELSPEED_MAX >> 8; //9-10 轮电机速度上限

		{
			float v = whell_power.InputPower;
			uint8_t *p = (uint8_t *)&v;
			txbuffer[11] = p[0];
			txbuffer[12] = p[1];
			txbuffer[13] = p[2];
			txbuffer[14] = p[3];
		}//11-14 轮电机功率上�?		memcpy(&txbuffer[15], &USER_JudgeData, sizeof(JudgeData_t)); //15-43 裁判系统数据
		
		memcpy(&txbuffer[15], &USER_JudgeData, sizeof(JudgeData_t));
		txbuffer[62] = FEEDBACK;

		txbuffer[B2B_FRAME_LEN - 1U] = 0xFD;	
		rs485_isvalid = 1;
		HAL_UART_Transmit_DMA(&huart2, txbuffer, sizeof(txbuffer));
		
	}
}
	
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart2)
	{
		if (Size == B2B_FRAME_LEN)
		{
			receive_times ++;
			B2B_ParseUsart();
			Detect_Update(DeviceID_B2B);
			detectList[DeviceID_B2B].isLost = 0;
		}
	}
	if(huart == &huart1)
	{
			Judge_Receive();
			Detect_Update(DeviceID_Judge);	
	}
}


void Task_B2B_Callback()
{
	/**********特殊情况处理*********************/
	if (STOPFLAG == 1)
	{
		FEEDBACK = 1;
		B2B_ParseUsart();
		osThreadResume(ErrorTaskHandle); // 恢复错误任务 饿死其他任务
	}
}
int times;
/************************freertos任务****************************/

void OS_Board2BoardCallback(void const *argument)
{
	B2B_Init();
	for (;;)
	{
		times ++;
		if(times >= 100)
		{
			times = 0;
			if(receive_times <= 5)
			{
				HAL_UARTEx_ReceiveToIdle_DMA(&huart2,usart2RxBuf,sizeof(usart2RxBuf));
				__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT); 
			}
			receive_times = 0;
		}
		Task_B2B_Callback();
		osDelay(1);
	}
}
