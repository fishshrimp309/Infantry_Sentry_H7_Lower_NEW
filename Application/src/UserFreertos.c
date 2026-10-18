#include "UserFreertos.h"
#include "USER_Moto.h"
#include "USER_B2B.h"
// #include "cmsis_armcc.h"
#include "usart.h"
#include "bsp_ws2812.h"
#include <stdint.h>

extern DMA_HandleTypeDef hdma_usart2_rx;
extern uint8_t usart2RxBuf[256];


//错误处理(急停)任务 
/*
 无os_delay最高优先级 在程序正常运行时不应该被调用 
 在恢复执行时占据全部时间片饿死其他任务
*/
		
void OS_ErrorCallback(void const * argument)
{
	//B2B_Init();
	osThreadSuspend(ErrorTaskHandle); //第一次执行挂起自身 
	CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
	HAL_Delay(10);
	for(;;)
	{
		USER_CAN_SetMotorCurrent(&hfdcan1,0x200,0,0,0,0);//关断电机	
		USER_CAN_SendCapData(&hfdcan2,0x1aa,0,0,0,0);
		HAL_Delay(1);
		FEEDBACK = 1;
		if(STOPFLAG != 1){
			__set_FAULTMASK(1);//禁止所有的可屏蔽中断
			HAL_NVIC_SystemReset();  //右拨杆回到中间重启系统
		}
		 //红灯闪烁    
		WS2812_Set(0, 50, 0, 0);
		HAL_Delay(75);
		WS2812_Set(0, 0, 0, 0);
		HAL_Delay(75);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,usart2RxBuf,sizeof(usart2RxBuf));
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
	}//无os_delay 最高优先级 
}


