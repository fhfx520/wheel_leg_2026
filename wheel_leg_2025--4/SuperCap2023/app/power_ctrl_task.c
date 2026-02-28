#include "power_ctrl_task.h"
#include "hrtim.h"
#include "bsp_adc.h"
#include "bsp_fdcan.h"
#include "math.h"
#include "bsp_judge.h"
#include "control_def.h"
#include "stdbool.h"


CCMRAM power_mode_e POWER_MODE;
CCMRAM power_calc_t bat, chassis, cap; // 总输入，底盘输出，电容
CCMRAM temperature_t temperature;	   // 温度检测

CCMRAM cap_state_t cap_state;									// 电容错误状态
CCMRAM uint32_t error_time[8] = {0, 0, 0, 0, 0, 0, 1000, 1000}; // 出错计时

CCMRAM pid_t buffer_loop;		  // 缓冲能量pid结构体
CCMRAM pid_t power_loop;		  // 功率环pid结构体
CCMRAM pid_t current_loop;		  // 电流环pid结构体
CCMRAM pid_t charge_voltage_loop; // 充电电压环pid结构体

CCMRAM pid_data_t current, power, buffer, charge_voltage, discharge_voltage; // pid数据

CCMRAM uint16_t boost_duty, buck_duty; // buck_duty boost_duty分别为左上和右上mos管占空比
CCMRAM float voltage_radio;			   // 两端电压差，即pid最终计算的占空比
uint8_t calc_flag, set_flag, toggle_flag, judge_flag;
float temp;
uint8_t chassis_mode; // 底盘的模式

uint8_t jieyue_flag = 1;

float text_current = 4.0f;
float text_power = 80.0f;
CCMRAM float text_voltage = 30.0f; // 电容最大电量
float text_buffer = 45.0f;		   // 缓冲能量恒定在这个设定值

CCMRAM void power_ctrl_task(void)
{
	static uint8_t timetick;

	power_error_check_advanced();
	power_state_machine();
	if (timetick >= 5)
	{
#ifdef Fdcan
		SuperCapMsgSendFdcan(cap.V, chassis.I);
#else
		SuperCapMsgSend(cap.V, chassis.I);
		SuperCapStateSend();
#endif
		timetick = 0;
	}
	timetick++;
}

CCMRAM void bufferCalc(void)
{
	if(cap.V < 15.0f)
		{
				if(Game_Robot_Status.chassis_power_limit > 120.0f)
						buffer.max = 120.0f;
				else
						buffer.max = Game_Robot_Status.chassis_power_limit;
		}
		else
		{
				if(Game_Robot_Status.chassis_power_limit > 200.0f)
						buffer.max = 200.0f;
				else
						buffer.max = Game_Robot_Status.chassis_power_limit;
		}
		buffer.ref = text_buffer;
		buffer.fdb = Power_Heat_Data.buffer_energy;
		buffer.output = buffer.max - pid_calc(&buffer_loop,buffer.fdb,buffer.ref);
		if(buffer.output > buffer.max)
				buffer.output = buffer.max;
		else if(buffer.output < POWER_MIN)
				buffer.output = POWER_MIN;
}

CCMRAM void pwmCalc(void)
{
	static uint32_t timetick;
//	if(cap.V > 18.0f&&jieyue_flag == 1)
//        {
//            jieyue_flag = 0;
//            text_power+=30.0f;
//        }
		if(calc_flag <= 1 && POWER_MODE == NORMAL)
		{
				if(timetick	< 8000)
				{
						if(chassis.I < 1.5f && chassis.I >= -1.5f)
						{
								timetick++;
						}
				}
				else if(timetick	< 16000)
				{
						calc_flag = 1;
						timetick++;
				}
				else
				{
						calc_flag = 2;
						timetick = 0;
				}
		}
//		power.ref = text_power;
		power.ref = buffer.output;
		power.fdb = bat.P;
		power.output = pid_calc(&power_loop,power.fdb,power.ref);
		
		charge_voltage.ref = text_voltage;
		charge_voltage.fdb = cap.V;
		charge_voltage.output = pid_calc(&charge_voltage_loop,charge_voltage.fdb,charge_voltage.ref);
		if(charge_voltage.output < cap.V / 33.0f -0.1f)
				charge_voltage.output = cap.V / 33.0f-0.1f;
		if(calc_flag == 1)
		{
				current.ref = (power.output)* ((float)(timetick - 8000) / 8000);
				if(current.ref > (Game_Robot_Status.chassis_power_limit/33.0f+0.35f))
						current.ref = (Game_Robot_Status.chassis_power_limit/33.0f+0.35f);
		}
		else if(calc_flag == 2)
		{
				current.ref = power.output;
		}
		
//		current.ref = text_current;
		current.fdb = chassis.I;
		current.output = pid_calc(&current_loop,current.fdb,current.ref);
}

void pidInit(void)
{
//		PID_struct_init(&buffer_loop, DELTA_PID, 10, 0, 10, 0, 0,
//                    0.0f, 2.25f, 0.0f);//90
//		PID_struct_init(&power_loop, DELTA_PID, 15, -15, 15, 0, 0,
//                    0.027f, 0.005f, 0.04f);// 0.018f, 0.01f, 0.015f  0.015   0.000085  0.0 \  0.0165f, 0.002f, 0.0f
//    PID_struct_init(&current_loop, DELTA_PID, 1.25, 0, 1.25, 0, 0,
//										0.0057f, 0.0019f, 0.002f);//电流环 0.006f, 0.002f, 0.008f  0.0172f, 0.002f, 0.0048f   0.0092f, 0.001f, 0.0022f采用港科方案的话需要提高限幅值，0.0172f, 0.002f, 0.0f  0.0221f, 0.002f, 0.0f 0.0061f, 0.0005f, 0.0f  dmax0.005f重调pid  0.0056f, 0.002f, 0.0005f
//		PID_struct_init(&charge_voltage_loop, DELTA_PID, 1.25, 0, 1.25, 0, 0,
//										0.006f, 0.001f, 0.0f);//电压环  0.04炸板子
	
//		PID_struct_init(&buffer_loop, DELTA_PID, 10, 0, 10, 0, 0,
//                    0.0f, 2.25f, 0.0f);//90
//		PID_struct_init(&power_loop, DELTA_PID, 15, -15, 15, 0, 0,
//                    0.04268f, 0.006f, 0.058f);// 0.018f, 0.01f, 0.015f  0.015   0.000085  0.0 \  0.0165f, 0.002f, 0.0f
//    PID_struct_init(&current_loop, DELTA_PID, 1.25, 0, 1.25, 0, 0,
//										0.00598f, 0.002f, 0.0158f);//电流环0.00023f, 0.00129f, 0.0005f 0.006f, 0.002f, 0.008f  0.0172f, 0.002f, 0.0048f   0.0092f, 0.001f, 0.0022f采用港科方案的话需要提高限幅值，0.0172f, 0.002f, 0.0f  0.0221f, 0.002f, 0.0f 0.0061f, 0.0005f, 0.0f  dmax0.005f重调pid  0.0056f, 0.002f, 0.0005f
//		PID_struct_init(&charge_voltage_loop, DELTA_PID, 1.25, 0, 1.25, 0, 0,
//										0.006f, 0.001f, 0.0f);//电压环  0.006f, 0.001f, 0.0f	
	
	PID_struct_init(&buffer_loop, DELTA_PID, 10, 0, 10, 0, 0,
                    0.0f, 2.25f, 0.0f);//90
		PID_struct_init(&power_loop, DELTA_PID, 15, -15, 15, 0, 0,
                     0.0395f, 0.0055f, 0.04f);// 0.018f, 0.01f, 0.015f  0.015   0.000085  0.0 \  0.0165f, 0.002f, 0.0f
    PID_struct_init(&current_loop, DELTA_PID, 1.50, 0, 1.50, 0, 0,
										0.00615f, 0.002f, 0.0128f);//电流环0.00023f, 0.00129f, 0.0005f 0.006f, 0.002f, 0.008f  0.0172f, 0.002f, 0.0048f   0.0092f, 0.001f, 0.0022f采用港科方案的话需要提高限幅值，0.0172f, 0.002f, 0.0f  0.0221f, 0.002f, 0.0f 0.0061f, 0.0005f, 0.0f  dmax0.005f重调pid  0.0056f, 0.002f, 0.0005f
		PID_struct_init(&charge_voltage_loop, DELTA_PID, 1.50, 0, 1.50, 0, 0,
										0.0065f, 0.001f, 0.0f);//电压环  0.006f, 0.001f, 0.0f	
	/*********0713 toll **********/
	// 先调电流后调功率和电压
//	PID_struct_init(&buffer_loop, DELTA_PID, 10, 0,
//									10, 0, 0,
//									0.0f, 0.0f, 0.0f); // 90
//	PID_struct_init(&power_loop, DELTA_PID, 15, -15,
//									15, 0, 0,
//									0.0f, 0.0f, 0.0f); // 0.018f, 0.01f, 0.015f  0.015   0.000085  0.0 \  0.0165f, 0.002f, 0.0f
//	PID_struct_init(&buffer_loop, DELTA_PID, 10, 0,
//									10, 0, 0,
//									0.0f, 2.25f, 0.0f); // 90
//	PID_struct_init(&power_loop, DELTA_PID, 15, -15,
//									15, 0, 0,
//									0.02663f, 0.0056f, 0.0655f);
//	PID_struct_init(&current_loop, DELTA_PID, 1.25, 0,
//									1.25, 0, 0,
//									0.00593f, 0.0022f, 0.002194f); // 电流环0.00023f, 0.00129f, 0.0005f 0.006f, 0.002f, 0.008f  0.0172f, 0.002f, 0.0048f   0.0092f, 0.001f, 0.0022f采用港科方案的话需要提高限幅值，0.0172f, 0.002f, 0.0f  0.0221f, 0.002f, 0.0f 0.0061f, 0.0005f, 0.0f  dmax0.005f重调pid  0.0056f, 0.002f, 0.0005f
//	PID_struct_init(&charge_voltage_loop, DELTA_PID, 1.25, 0,
//									1.25, 0, 0,
//									0.0056f, 0.001f, 0.0f); // 电压环  0.006f, 0.001f, 0.0f
}

CCMRAM void pwmSetCompare(void)
{
/*默认占空比左上管等于右下管*/
#if 0// 开环调试1    正常测试0
		buck_duty  = 0.9*DUTY;
		boost_duty = 0.9*DUTY;
		HRTIM_DMA_Buffer[1] = buck_duty;
		HRTIM_DMA_Buffer[0] = boost_duty;
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3,  buck_duty /2);
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3,  boost_duty/2);
#else
	if (POWER_MODE != NORMAL)
	{
		if (cap.V * 0.9f / chassis.V <= 1.0f)
		{
			voltage_radio = cap.V * 0.9f / chassis.V;
			temp = voltage_radio;
		}
		else
		{
			voltage_radio = chassis.V * 0.9f / cap.V;
			temp = voltage_radio;
		}
		
		
		if (voltage_radio <= 1.0f)
		{
			buck_duty = voltage_radio * 0.9f * DUTY;
			boost_duty = 0.9f * DUTY;
		}
		else
		{
			buck_duty = 0.9f * DUTY;
			boost_duty = 0.9f * DUTY / voltage_radio;
		}
		
		
		OUTPUT_LIMIT(buck_duty, MAX_RADIO, MIN_RADIO);
		OUTPUT_LIMIT(boost_duty, MAX_RADIO, MIN_RADIO);

		HRTIM_DMA_Buffer[1] = buck_duty;
		HRTIM_DMA_Buffer[0] = boost_duty;
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, buck_duty / 2);
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, boost_duty / 2);
	}
	
	
	else
	{
		/*防止缓启动后切换为正常工作模式后占空比的突变*/
		if (current.output <= charge_voltage.output)
		{
			if (current.output > temp && current.output < temp + 0.01f) //
			{
				set_flag = 1;
			}
		}
		else
		{
			if (charge_voltage.output > temp && charge_voltage.output < temp + 0.01f)
			{
				set_flag = 1;
			}
		}
		
		
		if (set_flag != 1)
		{
			voltage_radio = temp;
			if (voltage_radio <= 1.0f)
			{
				buck_duty = voltage_radio * 0.9f * DUTY;
				boost_duty = 0.9f * DUTY;
			}
			else
			{
				buck_duty = 0.9f * DUTY;
				boost_duty = 0.9f * DUTY / voltage_radio;
			}
		}
		
		else
		{
			if (current.ref >= 0)
			{
				if (current.output <= charge_voltage.output)
				{
					voltage_radio = current.output;
				}
				else
				{
					voltage_radio = charge_voltage.output;
					toggle_flag = 1;
				}
				
				
				if (voltage_radio <= 0.8f)
				{
					buck_duty = voltage_radio * 0.9f * DUTY;
					boost_duty = 0.9f * DUTY;
				}
				else if(voltage_radio >= 1.25f)
				{
					buck_duty = 0.9f * DUTY;
					boost_duty = 0.9f * DUTY / voltage_radio;
				}
				else
				{
					buck_duty = 4.0f/9.0f*(1+voltage_radio)*DUTY*0.9f;
				  boost_duty = 4.0f/9.0f*(1+1/voltage_radio)*DUTY*0.9f;
				}
				
				
			}
			else
			{
				if (toggle_flag == 1)
				{
					voltage_radio += 0.0000005f;
					if (voltage_radio > current.output)
						toggle_flag = 0;
				}
				else
				{
					voltage_radio = current.output;
				}
				if (voltage_radio <= 1.0f)
				{
					buck_duty = voltage_radio * 0.9f * DUTY;
					boost_duty = 0.9f * DUTY;
				}
				else
				{
					buck_duty = 0.9f * DUTY;
					boost_duty = 0.9f * DUTY / voltage_radio;
				}
			}
		}
		// 最大占空比和最小占空比可以一定程度上防止过充过放
		OUTPUT_LIMIT(buck_duty, MAX_RADIO, MIN_RADIO);
		OUTPUT_LIMIT(boost_duty, MAX_RADIO, MIN_RADIO);

		HRTIM_DMA_Buffer[1] = buck_duty;
		HRTIM_DMA_Buffer[0] = boost_duty;
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, buck_duty / 2);
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, boost_duty / 2);
	}
#endif
}

CCMRAM void PowerControlHandler(void)
{
	switch (POWER_MODE)
	{
	case PROTECT:
	{
		HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
		pwmSetCompare();
		// 进入保护后重置缓启动
		set_flag = 0;
		calc_flag = 0;
		break;
	}
	case RESTART:
	{
		pwmSetCompare();
		break;
	}
	case NORMAL:
	{
		HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
		pwmCalc();
		pwmSetCompare();
		break;
	}
	default:
	{
		POWER_MODE = PROTECT;
		break;
	}
	}
}

CCMRAM void power_error_check_advanced(void)
{
	static uint32_t err_time[ERROR_TYPE_COUNT] = {0};
	static uint32_t recover_tick[2] = {0};
	My_err_timers_update(error_time);
	My_err_handler(err_time, recover_tick);
}

CCMRAM void My_err_timers_update(uint32_t err_time[])
{
	err_time[0] = (cap.V > CAP_MAX_V) ? ++error_time[0] : 0;
	err_time[1] = (cap.V < CAP_MIN_V) ? ++error_time[1] : 0;
	err_time[2] = (chassis.V > CHASSIS_MAX_V) ? ++error_time[2] : 0;
	err_time[3] = (bat.V < BAT_MIN_V) ? ++error_time[3] : 0;
	err_time[4] = (fabs(cap.I) > CAP_MAX_I) ? ++error_time[4] : 0;
	err_time[5] = (fabs(chassis.I) > CHASSIS_MAX_I) ? ++error_time[5] : 0;
	err_time[6]++; // 底盘信息丢失
	err_time[7]++; // 裁判系统信息丢失
}

CCMRAM void My_err_handler(uint32_t err_time[], uint32_t recover_tick[])
{
	cap_state.bit.cap_v_over = (error_time[0] > TIME_CAP_OVER_VOLTAGE_MS);
	cap_state.bit.cap_v_low = (error_time[1] > TIME_CAP_LOW_VOLTAGE_MS) ? 1 : 0;
	cap_state.bit.chassis_v_over = (error_time[2] > TIME_CHASSIS_OVER_VOLTAGE_MS) ? 1 : (cap_state.bit.chassis_v_over && chassis.V >= CHASSIS_V_RECOVER_DUTY) ? 1
																																							  : 0;
	cap_state.bit.bat_v_low = (error_time[3] > TIME_BAT_LOW_VOLTAGE_MS) ? 1 : 0;

	if (error_time[4] > TIME_CAP_OVER_CURRENT_MS)
	{
		cap_state.bit.cap_i_over = 1, recover_tick[0] = 0;
	}
	else if (cap_state.bit.cap_i_over && ++recover_tick[0] > TIME_CURRENT_RECOVER_MS)
	{
		recover_tick[0] = cap_state.bit.cap_i_over = 0;
	}

	if (error_time[5] > TIME_CHASSIS_OVER_CURRENT_MS)
	{
		cap_state.bit.cap_i_over = 1, recover_tick[1] = 0;
	}
	else if (cap_state.bit.cap_i_over && ++recover_tick[1] > TIME_CURRENT_RECOVER_MS)
	{
		recover_tick[1] = cap_state.bit.cap_i_over = 0;
	}

	cap_state.bit.chassis_msg_miss = (error_time[6] > TIME_CHASSIS_MSG_LOST_MS) ? 1 : 0;
	cap_state.bit.judge_msg_miss = (error_time[7] > TIME_JUDGE_MSG_LOST_MS) ? 1 : 0;
}

static bool check_power_fault_conditions(void)
{
	return (cap_state.bit.cap_v_over 
				||cap_state.bit.cap_v_low 
				||cap_state.bit.chassis_v_over 
				||cap_state.bit.bat_v_low 
//				||cap_state.bit.cap_i_over 
				||cap_state.bit.judge_msg_miss
			);
}

CCMRAM void power_state_machine(void)
{
	static uint16_t timetick;
		static power_mode_e last_power_mode;
    last_power_mode = POWER_MODE;
		if(check_power_fault_conditions())
		{
				POWER_MODE = PROTECT;
		}
		else if(last_power_mode == RESTART && timetick > 500 )
		{
				POWER_MODE = NORMAL;
				timetick = 0;
		}
		else if(last_power_mode == PROTECT || last_power_mode == RESTART)
		{
        POWER_MODE = RESTART;
				timetick++;
		}
}


//void three_mode(void)
//{
///*三模式切换*/
//	if(power.output > 0)
//	{
//			if(current.output <= charge_voltage.output)
//					voltage_radio = current.output;
//			else
//					voltage_radio = charge_voltage.output;
//		if(voltage_radio < 0.8)
//		{
//				buck_duty = voltage_radio*DUTY;
//				boost_duty = DUTY-1;
//				HRTIM1->sTimerxRegs[0].DTxR = 3072;//修改buck侧管的死区时间为0
//				HRTIM1->sTimerxRegs[1].DTxR = 396294;//修改boost侧管的死区时间为6
//		}
//		else if(voltage_radio >1.25)
//		{
//				buck_duty = DUTY-1;
//				boost_duty = 1/voltage_radio*DUTY;
//				HRTIM1->sTimerxRegs[0].DTxR = 396294;//修改buck侧管的死区时间为6
//				HRTIM1->sTimerxRegs[1].DTxR = 3072;//修改boost侧管的死区时间为0
//		}
//		else
//		{
//				buck_duty = 4/9*(1+voltage_radio)*DUTY;
//				boost_duty =4/9*(1+1/voltage_radio)*DUTY;
//		}
//	}
//	else
//	{
//		voltage_radio = current.output;
//		if(voltage_radio < 0.8)
//		{
//				buck_duty = voltage_radio*DUTY;
//				boost_duty = DUTY-1;
//				HRTIM1->sTimerxRegs[0].DTxR = 3072;//修改buck侧管的死区时间为0
//				HRTIM1->sTimerxRegs[1].DTxR = 396294;//修改boost侧管的死区时间为6
//		}
//		else if(voltage_radio >1.25)
//		{
//				buck_duty = DUTY-1;
//				boost_duty = 1/voltage_radio*DUTY;
//				HRTIM1->sTimerxRegs[0].DTxR = 396294;//修改buck侧管的死区时间为6
//				HRTIM1->sTimerxRegs[1].DTxR = 3072;//修改boost侧管的死区时间为0
//		}
//		else
//		{
//				buck_duty = 4/9*(1+voltage_radio);
//				boost_duty =4/9*(1+1/voltage_radio);
//				HRTIM1->sTimerxRegs[0].DTxR = 396294;//修改buck侧管的死区时间为6
//				HRTIM1->sTimerxRegs[1].DTxR = 396294;//修改boost侧管的死区时间为6
//		}
//	}
//}
