#include "super_cap.h"
#include "UserFreertos.h"
// #include "graphics.h"
#include "judge.h"
#include <stdio.h>
// #include "Detect.h"
#include "Moto.h"

SuperCap cap;
extern ext_game_robot_status_t                  GameRobotState;
void Cap_AnalysisData()
{    
    if(GameRobotState.power_management_chassis_output==0)
    {
        cap.power_ctrl_mode = TURNOFF;
        cap.target_output_power=0;
        cap.target_charge_power=0;
//        chassis.move.outputSlope.value=0;
//        chassis.move.chargeSlope.value=0;
        cap.last_power_management = 0;
    }
    if(GameRobotState.power_management_chassis_output==1 && cap.last_power_management == 0)
    {
        uint8_t lost = 0;
        // lost += Detect_IsDeviceLost(DeviceID_ChassisMotor1);
        // lost += Detect_IsDeviceLost(DeviceID_ChassisMotor2);
        // lost += Detect_IsDeviceLost(DeviceID_ChassisMotor3);
        // lost += Detect_IsDeviceLost(DeviceID_ChassisMotor4);
        if(lost!=0)
        {
        cap.power_ctrl_mode = TURNOFF;
        cap.target_output_power=0;
        cap.target_charge_power=0;
//        chassis.move.outputSlope.value=0;
//        chassis.move.chargeSlope.value=0;
//        Cap_CanSendData();    
        }
        if(lost==0)
        cap.last_power_management = 1;        
    }
        cap.cap_vot = cap.receive_data.cap_voltage / 100.0f;
        cap.energy = (cap.cap_vot*cap.cap_vot - 5.0f*5.0f) * 0.5f * 11.0f;
        cap.per_energy = cap.energy/1744.875f*100.0f;  //相对7v的百分比能量值 877.5=0.5*5*（25*25-8*8）
//        cap.total_output = cap.receive_data.total_output_power/100.f;
//        cap.cap_output = cap.receive_data.cap_output_power/100.f;
}
void OS_SuperCapCallback(void const * argument)
{
  osDelay(1500);
    uint32_t cnt=0;    
  for(;;)
  {  
        cap.targetI = 200;
        Cap_AnalysisData();         
//    Cap_CanSendData();
        if(cnt%200==0)
        {
            char textBuf[30]={0};
            sprintf(textBuf,"%03d %%",(int)cap.per_energy);
//            Graph_SetText(&chassis.ui.super_cap,"CAP",Color_Orange,4,0,1000,780,textBuf,5,30);
//            Graph_DrawText(&chassis.ui.super_cap,Operation_Change);
        }
        cnt++;
    osDelay(1);
  }

}