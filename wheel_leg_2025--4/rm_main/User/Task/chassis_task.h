#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "stdint.h"
#include "kalman_filter.h"
#include "robot_logic.h" // 引入 FSM 大脑

// [修改] 彻底删除了旧的 chassis_mode_e 枚举，直接使用 FSM 的 ChassisState_e

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
    uint8_t recover_flag;           
    uint8_t rescue_inter_flag;      
    uint32_t rescue_cnt_L;
    uint32_t rescue_cnt_R;
    uint8_t init;                   
    uint8_t joint_motor_reset;      
    // [修改] 删除了 chassis_mode_e mode 暂时保留 有耦合模块
    //chassis_mode_e mode;
    float wheel_max;                
    chassis_speed_t input, output;
} chassis_t;

extern chassis_t chassis;
extern uint8_t rotate_flag;
extern kalman_filter_t kal_fusion_vel;

void Fusion_Vel_Acc_Test(void);
void chassis_task(void const *argu);

#endif