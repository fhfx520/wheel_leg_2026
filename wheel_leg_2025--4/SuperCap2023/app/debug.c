#include "debug.h"
#include "usart.h"
#include "data_scope.h"

/* 待添加头文件 */
#include "bsp_adc.h"
#include "power_ctrl_task.h"
#include "bsp_judge.h"
//长期变量
extern float temp;
uint8_t debug_wave = 3; /* 用以选择打印的数据类型 */

void DataWavePkg(void) {
    /* 视觉调试专用部分  */
    switch (debug_wave) 
		{
			case 0: 
			{  /* 查看数据方差 */
					DataScope_Get_Channel_Data(cap_state.bit.chassis_v_over);
					DataScope_Get_Channel_Data(cap_state.bit.bat_v_low );	
					DataScope_Get_Channel_Data(cap_state.bit.cap_v_over);
					DataScope_Get_Channel_Data(cap_state.bit.cap_v_low);
					DataScope_Get_Channel_Data(cap_state.bit.cap_i_over);	
					DataScope_Get_Channel_Data(cap_state.bit.chassis_i_over);				
					DataScope_Get_Channel_Data(cap.I);					
					DataScope_Get_Channel_Data(current.fdb);
					DataScope_Get_Channel_Data(current.output);
					DataScope_Get_Channel_Data(charge_voltage.output);
					break;
			}
			case 1:
			{
					DataScope_Get_Channel_Data(power.ref);	
					DataScope_Get_Channel_Data(power.fdb);
					DataScope_Get_Channel_Data(current.ref);					
					DataScope_Get_Channel_Data(current.fdb);
					DataScope_Get_Channel_Data(charge_voltage.ref);					
					DataScope_Get_Channel_Data(charge_voltage.fdb);
					DataScope_Get_Channel_Data(current.output);
					DataScope_Get_Channel_Data(charge_voltage.output);
					DataScope_Get_Channel_Data(POWER_MODE);
					break;
			}
			case 2:
			{
					DataScope_Get_Channel_Data(chassis.I);
					DataScope_Get_Channel_Data(cap.I);
					break;
			}
			case 3:
			{
					DataScope_Get_Channel_Data(cap.V);
					DataScope_Get_Channel_Data(chassis.V);
					DataScope_Get_Channel_Data(chassis.I);
					DataScope_Get_Channel_Data(bat.V);
					DataScope_Get_Channel_Data(bat.I);
					DataScope_Get_Channel_Data(chassis.P);
					DataScope_Get_Channel_Data(bat.P);
					break;
			}
			case 4:
			{
					DataScope_Get_Channel_Data(temperature.T1);	
					DataScope_Get_Channel_Data(buffer.fdb);
					DataScope_Get_Channel_Data(power.ref);	
					DataScope_Get_Channel_Data(power.fdb);
					DataScope_Get_Channel_Data(current.ref);					
					DataScope_Get_Channel_Data(current.fdb);
					DataScope_Get_Channel_Data(charge_voltage.fdb);
					DataScope_Get_Channel_Data(chassis.V);
					DataScope_Get_Channel_Data(current.output);
					DataScope_Get_Channel_Data(charge_voltage.output);				
					break;
			}
			case 5:
			{
					DataScope_Get_Channel_Data(cap.V);
					DataScope_Get_Channel_Data(chassis.V);
					DataScope_Get_Channel_Data(bat.V);
					DataScope_Get_Channel_Data(cap.I);
					DataScope_Get_Channel_Data(chassis.I);
					DataScope_Get_Channel_Data(bat.I);
					DataScope_Get_Channel_Data(bat.P);
					DataScope_Get_Channel_Data(Power_Heat_Data.chassis_current/1000.0f);	
					DataScope_Get_Channel_Data(Power_Heat_Data.chassis_volt/1000.0f);	
					DataScope_Get_Channel_Data(Power_Heat_Data.chassis_power);	
					break;
			}
			case 6:
			{
					DataScope_Get_Channel_Data(temperature.T1);	
					DataScope_Get_Channel_Data(cap.P/bat.P);
					DataScope_Get_Channel_Data(power.ref);	
					DataScope_Get_Channel_Data(power.fdb);
					DataScope_Get_Channel_Data(current.ref);					
					DataScope_Get_Channel_Data(current.fdb);
					DataScope_Get_Channel_Data(charge_voltage.fdb);
					DataScope_Get_Channel_Data(chassis.V);
					DataScope_Get_Channel_Data(current.output);
					DataScope_Get_Channel_Data(charge_voltage.output);				
					break;
			}
    default: 
			{
          break;
      }
    }
}
