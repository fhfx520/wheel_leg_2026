#ifndef __POWER_CTRL_TASK_H__
#define __POWER_CTRL_TASK_H__

#include "stm32g4xx.h"
#include "hrtim.h"
#include "pid.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "arm_math.h"

#define CHASSIS_MAX_V	43.0f	
#define BAT_MIN_V		10.0f	

#define CAP_MAX_V		33.0f	
#define CAP_MIN_V		4.0f	
	
#define CAP_MAX_I		40.0f	
#define CHASSIS_MAX_I	25.0f	
#define POWER_MIN		35.0f	//用完20000J之后
#define DUTY 			27200		

#define MAX_RADIO		24480
#define MIN_RADIO		2720
typedef enum
{
    NORMAL,
    PROTECT,
    RESTART,
}power_mode_e;

typedef struct
{
		float V;
		float I;
		float P;
}power_calc_t;

typedef struct
{
		float T1;
		float T2;
		float T3;
}temperature_t;

typedef union
{
    uint8_t state;
    struct
    {
        uint8_t cap_v_over : 1;    //���ݹ�ѹ
		uint8_t cap_v_low : 1;    //����Ƿѹ
        uint8_t chassis_v_over : 1;    //����˹�ѹ
        uint8_t bat_v_low : 1;    //�����Ƿѹ
        uint8_t cap_i_over : 1;    //���ݹ���
		uint8_t chassis_i_over : 1;		//�������
        uint8_t chassis_msg_miss : 1;    //������Ϣ��ʧ
        uint8_t judge_msg_miss : 1;    //����ϵͳ��Ϣ��ʧ		
    }bit;
}cap_state_t;

typedef struct
{
		float ref;
		float fdb;
		float output;
		float max;							//����ϵͳ���Ƶ������
}pid_data_t;

extern power_calc_t bat,chassis,cap;
extern temperature_t temperature;//�¶ȼ��
extern power_mode_e POWER_MODE;
extern cap_state_t cap_state;
extern pid_data_t current,power,buffer,charge_voltage,discharge_voltage;
extern uint32_t error_time[8];
extern uint8_t chassis_mode;
extern float voltage_radio;					//���˵�ѹ���pid���ռ����ռ�ձ�
extern uint16_t boost_duty,buck_duty; //buck_duty boost_duty�ֱ�Ϊ���Ϻ�����mos��ռ�ձ�
extern uint8_t set_flag,calc_flag,toggle_flag,judge_flag;

void power_ctrl_task(void);
void power_error_check_advanced(void);
void My_err_timers_update(uint32_t err_time[]);
void My_err_handler(uint32_t err_time[], uint32_t recover_tick[]);
void power_state_machine(void);
void PowerControlHandler(void);	
void pwmSetCompare(void);
void pwmCalc(void);
void bufferCalc(void);
void powerCalc(void);
void pidInit(void);
#endif
