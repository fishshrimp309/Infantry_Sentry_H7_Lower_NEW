#include "bsp_can.h"
#include "fdcan.h"
#include "Moto.h"
#include "USER_Moto.h"
#include "super_cap.h"
#include "USER_Detcet.h"

typedef struct
{
    uint16_t voltage;//0.01V
    uint16_t current;//0.01A
    float power;
} power_measure_t;
power_measure_t power_measure = {0};

CanState can_state;

/**************锟节诧拷锟斤拷锟竭猴拷锟斤拷锟斤拷锟斤拷***********************/
void CAN1_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata);
//can1锟斤拷锟斤拷
void CAN2_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata);
//can2锟斤拷锟斤拷
void CAN3_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata);
//can3锟斤拷锟斤拷
/******************锟斤拷始锟斤拷***************************/
//can锟斤拷锟斤拷锟斤拷锟斤拷始锟斤拷

void CAN_Init()
{
	FDCAN_FilterTypeDef filter;                   	//< 锟斤拷锟斤拷锟街诧拷锟斤拷锟斤拷 can锟斤拷锟斤拷锟斤拷锟结构锟斤拷
	filter.IdType       = FDCAN_STANDARD_ID;       	//< id锟斤拷锟斤拷为锟斤拷准id
	filter.FilterIndex  = 0;                      	//< 锟斤拷值筛选锟斤拷锟侥憋拷牛锟斤拷锟阶糹d选锟斤拷0-127
	filter.FilterType   = FDCAN_FILTER_MASK;       	//< 锟斤拷锟矫癸拷锟斤拷模式为锟斤拷锟斤拷模式
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 	//< 锟斤拷锟斤拷锟斤拷锟斤拷锟剿碉拷锟斤拷锟捷存储锟斤拷 fifo0
	filter.FilterID1    = 0x00000000;                   	//< 筛选锟斤拷锟斤拷id
	filter.FilterID2    = 0x00000000;
	
	HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);   //< 锟斤拷锟矫癸拷锟斤拷锟斤拷	
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使锟斤拷fifo0锟斤拷锟秸碉拷锟斤拷锟斤拷息锟叫讹拷
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1,FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_Start(&hfdcan1);                   //< 使锟斤拷can

	HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);   //< 锟斤拷锟矫癸拷锟斤拷锟斤拷	
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2,FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使锟斤拷fifo0锟斤拷锟秸碉拷锟斤拷锟斤拷息锟叫讹拷
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0);
	HAL_FDCAN_Start(&hfdcan2);                   //< 使锟斤拷can
	
	HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);   //< 锟斤拷锟矫癸拷锟斤拷锟斤拷	
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3,FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使锟斤拷fifo0锟斤拷锟秸碉拷锟斤拷锟斤拷息锟叫讹拷
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_BUS_OFF, 0);
	HAL_FDCAN_Start(&hfdcan3);                   //< 使锟斤拷can
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	HAL_StatusTypeDef	if_can_get_message_ok;
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	if(hfdcan == &hfdcan1)
	{
		if_can_get_message_ok = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		if(if_can_get_message_ok == HAL_OK)
		{
			
			CAN1_Rx0Callback(&rx_header,rx_data);
			
		}
		else
		{
			can_state.can1_receive_error++;
		}
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	}
	else if(hfdcan == &hfdcan2)
	{
		if_can_get_message_ok = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		if(HAL_OK == if_can_get_message_ok)
		{
      		CAN2_Rx0Callback(&rx_header,rx_data);  
		}
		else
		{
			can_state.can2_receive_error++;
		}
		HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	}
	else if(hfdcan == &hfdcan3)
	{
		if_can_get_message_ok = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		if(HAL_OK == if_can_get_message_ok)
		{
			CAN3_Rx0Callback(&rx_header,rx_data);   
			
		}
		else
		{
			can_state.can3_receive_error++;
		}
		HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	}
        
}

//can1锟斤拷锟秸斤拷锟斤拷锟叫讹拷
void CAN1_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata)
{
	uint8_t whichMotor;
	switch(rx_header->Identifier)
	{
		//锟斤拷锟斤拷锟斤拷锟�
		case 0x201:
		case 0x202:
		case 0x203:
		case 0x204:
		{
			whichMotor = rx_header->Identifier - 0x201;
			DJIMotor_Update(&chassis.M3508[whichMotor], (rxdata[0]<<8 | rxdata[1]), (rxdata[2]<<8 | rxdata[3]),(rxdata[4]<<8|rxdata[5]),rxdata[6]);
			if(chassis.M3508[whichMotor].temp>70&&chassis.M3508[whichMotor].temp<100){
				chassis.M3508[whichMotor].ERRORFLAG ++;
			}	
			if(chassis.M3508[whichMotor].temp<50){
				chassis.M3508[whichMotor].ERRORFLAG = 0;
			}	
			Detect_Update(DeviceID_ChassisMotor1+whichMotor);
		}
		break;
		case 0x213:
			  power_measure.voltage = (rxdata[1] << 8) | rxdata[0];
			  power_measure.current = (rxdata[3] << 8) | rxdata[2];
			  power_measure.power = power_measure.voltage * power_measure.current * 0.0001;
			  break;
		default:
		break;
	}
}


////can2锟斤拷锟秸斤拷锟斤拷锟叫讹拷
void CAN2_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata)
{
	switch(rx_header->Identifier)
	{
        case 0x300:
            Supercap_Update(&cap,rxdata);
			break;
				case 0x213:
			  power_measure.voltage = (rxdata[1] << 8) | rxdata[0];
			  power_measure.current = (rxdata[3] << 8) | rxdata[2];
			  power_measure.power = power_measure.voltage * power_measure.current * 0.0001;
			  break;
		default:
		break;
	}
}
            
void CAN3_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata)
{   
	switch(rx_header->Identifier)
	{
		default:
		break;
	}
}


/********************锟解部锟斤拷锟矫猴拷锟斤拷*******************************/
void USER_CAN_Send(FDCAN_HandleTypeDef* hfdcan,int16_t StdId,uint8_t* tx_data)
{
	FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier = StdId;
	tx_header.IdType = FDCAN_STANDARD_ID;
 	tx_header.TxFrameType = FDCAN_DATA_FRAME;
  	tx_header.DataLength = 8; 
 	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
 	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0;
		
    vTaskSuspendAll();
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan,&tx_header,tx_data)!= HAL_OK)
	{
		if(hfdcan==&hfdcan1)
		{
			can_state.can1_send_error++;
		}
		else if(hfdcan==&hfdcan2)
		{
			can_state.can2_send_error++;
		}
		else if(hfdcan==&hfdcan3)
		{
			can_state.can3_send_error++;		
		}
	}
  	xTaskResumeAll();
}

static void check_can_bus(FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_ProtocolStatusTypeDef protocolStatus;

	HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
	if (protocolStatus.BusOff)
	{
		CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
   if (hfdcan == &hfdcan1) {
     if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {

      check_can_bus(hfdcan);
			 
    }
  }
	if (hfdcan == &hfdcan2) {
     if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {

      check_can_bus(hfdcan);
			 
    }
  }
	 if (hfdcan == &hfdcan3) {
     if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {

      check_can_bus(hfdcan);
			 
    }
  }
}
