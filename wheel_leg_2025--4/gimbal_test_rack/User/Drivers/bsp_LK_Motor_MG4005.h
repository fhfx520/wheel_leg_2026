#ifndef __BSP_LK_MOTOR_MG4005_H
#define __BSP_LK_MOTOR_MG4005_H
#include "fdcan.h"

#define SHOOT_CAN_ID_MG4005 0x142



//#define DEVICE_STD_ID						(0x140)  //不知道耶，领控电机厂家例程里面CAN的id就是电机上编码开关的数值加上这个0x140
#define LK_MOTOR_ECD_TO_RAD            0.00549324788281f //     360 /65535   (如果是MG4005 就要360/655350 多一个0是电机减速比1：10）
#define LK_MOTOR_SPD_MAX               36000.0f //限制最大速度为360度每秒， 分辨率为为 0.01dps/LSB  360/0.01=360 00.0
#define LK_MOTOR_SPD_MIN              -36000.0f



typedef enum
{
	MG4005_Run=1,       //LED常亮，此时发送控制指令即可控制电机动作
	MG4005_Stop=0,      //停止电机，但不清除电机运行状态。再次发送控制指令即可控制电机动作。
	MG4005_unable=2,  //清除电机转动圈数及之前接收的控制指令，LED 由常亮转为慢闪。此时电机仍然可以回复控制命令，但不会执行动作。  这把好多数据都毁掉了欸？ 没仔细看过ecd_mode下的电机控制，可能会受影响吧？ 暂时没写函数会使其进入该状态
}MG4005_motor_mode_e;

typedef enum
{
	single_pos_2=0,           //单圈位置闭环控制 2
	vel_2=1,                  //速度闭环控制 2
	tor=2,                    //转矩闭环控制命令
}MG4005_work_mode_e;

typedef enum
{
	LK_RUNNING=0x00,
	LK_OFF=0x10,                      //电机处于开启状态； motorState = 0x10 电机处于关闭状态
}MG4005_motorState_e;


typedef enum
{
	LK_Init      =0x01,
	LK_Init_Fail =0x00,                      //电机处于开启状态； motorState = 0x10 电机处于关闭状态
}MG4005_motorInitState_e;



/*
errorState 位 状态说明 
               0    1
0 低电压状态   正常 低压保护
1 高电压状态   正常 高压保护
2 驱动温度状态 正常 驱动过温
3 电机温度状态 正常 电机过温
4 电机电流状态 正常 电机过流
5 电机短路状态 正常 电机短路
6 堵转状态     正常 电机堵转
7 输入信号状态 正常 输入信号丢失超时
*/
typedef union
{
	uint8_t err_state;
    __packed struct
    {
        uint8_t low_voltage    : 1; // bit0
        uint8_t high_voltage   : 1; // bit1
        uint8_t drive_temp     : 1; // bit2
        uint8_t motor_temp     : 1; // bit3
        uint8_t motor_current  : 1; // bit4
        uint8_t motor_short    : 1; // bit5
        uint8_t motor_stall    : 1; // bit6
        uint8_t signal_loss    : 1; // bit7
     } bits;
} lk_motor_err_state_t;
//extern lk_motor_err_state_t lk_motor_err_state;




typedef struct{
	MG4005_motor_mode_e     MG4005_motor_mode;
	MG4005_work_mode_e      MG4005_work_mode;
	MG4005_motorState_e     MG4005_motorState;
	MG4005_motorInitState_e MG4005_motorInitState;
	lk_motor_err_state_t lk_motor_err_state;
	
	int id;                      //电机id
	int8_t  temperature;         //电机温度  1℃/LSB
	uint32_t error_cnt;
	uint32_t lk_online_cnt;
	
	/*控制值*/
	uint32_t pos_ref;  //位置期望0.01degree/LSB，即 36000 代表 360°。
	int32_t vel_ref;  //速度期望
	int16_t toq_i_ref;  //力矩电流期望
	
	/*读取电机状态 1 和错误标志命令*/
	int16_t voltage;//母线电压 voltage（int16_t 类型，单位 0.01V/LSB）。
	int16_t    current;          //母线电流 current（int16_t 类型，单位 0.01A/LSB）。
	uint8_t  motorState;//电机状态 motorState（为 uint8_t 类型，各个位代表不同的电机状态）
	uint8_t errorState;//错误标志 errorState（为 uint8_t 类型，各个位代表不同的电机错误状态）
	
	/*读取电机状态 2 命令*/
	int16_t  iq;                 //MG电机转矩电流值 iq     分辨率为(66/4096 A) / LSB
	int16_t  speed;              //电机转速 1dps/LSB）
	uint16_t raw_encoder;            //类型，14bit 编码器的数值范围 0~16383，15bit 编码器的数值范围  0~32767，16bit 编码器的数值范围 0~65535,问了客服说18bit编码器的数值范围也是0~65535）。
	uint16_t offset_encoder;     //刚上电的时候check一下初始encoder，用于后续解算总ecd和实际角度
	uint16_t raw_last_encoder;       //这个电机返回的位置是编码器数据，跟dji电机很像所以抄过来了，但一直都不是很懂dji电机关于ecd的计算原理
	int32_t  total_encoder;        //
	int32_t  round_cnt;          //用于计算真正的位置
	int16_t  ecd_pos;
	int16_t  pos;                //位置   单位：度
	int16_t  ele_pos;            //电角度 单位：度
	
	/*读取电机状态 3 命令:相电流数据 iA、iB、iC，MG 电机相电流分辨率为(66/4096 A) / LSB；MF电机相电流分辨率为(33/4096 A) / LSB。*/	
	int16_t iA;
	int16_t iB;
	int16_t iC;
	float ialpha;
	float ibeta;
	float toq_cal;
	
	/*2025年3月12日：工况扭矩测定用新代码*/
	float max_toq; 
	float avrg_toq; 
	float tol_toq;//用于计算平均值
	uint32_t toq_cal_cnt;
	
	/*限幅值*/
	uint32_t angleControl;//对应实际位置为 0.01degree/LSB，即 36000 代表 360°  

	
	float pos_out;      // 本次位置式输出，我加的

	
}Motor_MG4005_t;


extern Motor_MG4005_t trigger_motor_MG4005;

extern uint8_t OFFSET_ECD_FLAG;//校正后置1



void start_MG4005_motor(FDCAN_HandleTypeDef *hfdcan, uint16_t id);
void MG4005_Data_Handler(Motor_MG4005_t* mtr,uint8_t* CAN_Rx_Data);
void MG4005_Toq_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t id, int16_t toq_i_ref);

	

#endif
