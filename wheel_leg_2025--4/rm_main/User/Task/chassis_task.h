#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "stdint.h"

typedef enum
{
    CHASSIS_MODE_PROTECT,

    CHASSIS_MODE_REMOTER_FOLLOW,
    CHASSIS_MODE_REMOTER_ROTATE1,
    CHASSIS_MODE_REMOTER_ROTATE2,
    CHASSIS_MODE_KEYBOARD_FIGHT,

    CHASSIS_MODE_KEYBOARD_FOLLOW,
    CHASSIS_MODE_KEYBOARD_ROTATE,
    CHASSIS_MODE_KEYBOARD_UNFOLLOW,
    CHASSIS_MODE_KEYBOARD_PRONE,  //趴倒模式
} chassis_mode_e;

typedef struct
{
    uint8_t stop_cnt;
    uint8_t reset_flag;
    float last_position;
} motor_reset_t;

typedef struct
{
    float remote, keyboard;
} chassis_scale_t;

typedef struct
{
    float vx, vy;
} chassis_speed_t;

typedef struct
{
    uint32_t rescue_test;			//未使用 用于测试翻倒自起
    uint32_t recover_flag;			// =1进入翻倒自起  =2再根据不同情况决定电机以速度方式输出还是力矩方式
    uint32_t rescue_inter_flag;		// =1车身正在归正  =2进入收腿  =3整车翻倒且保护天鹅颈  =4第二象限启动卡墙
    uint32_t rescue_cnt_L;
    uint32_t rescue_cnt_R;
    uint8_t init;					// = 0底盘未初始化  =1底盘完成初始化
    uint8_t joint_motor_reset;		//未使用
    chassis_mode_e mode;
    float wheel_max;				//未使用
    chassis_speed_t input, output;
} chassis_t;

extern chassis_t chassis;
extern uint8_t rotate_flag;
//extern uint32_t rescue_cnt_T0;
//extern uint8_t recover_flag;

void chassis_task(void const *argu);

#endif

