#ifndef ROBOT_LOGIC_H
#define ROBOT_LOGIC_H

#include "fsm_core.h" 
#include <stdint.h>

#ifndef __PACKED_STRUCT
#define __PACKED_STRUCT struct __attribute__((packed))
#endif
#ifndef __packed
#define __packed __attribute__((packed))
#endif

// ==========================================
// 1. 输入数据结构
// ==========================================
typedef __PACKED_STRUCT {
    int16_t ch1; int16_t ch2; int16_t ch3; int16_t ch4; int16_t ch5;
    uint8_t sw1; uint8_t sw2;
    __PACKED_STRUCT {
        int16_t x; int16_t y; int16_t z;
        uint8_t l; uint8_t r;
    } mouse;
    __packed union {
        uint16_t key_code;
        __PACKED_STRUCT {
            uint16_t W:1; uint16_t S:1; uint16_t A:1; uint16_t D:1;
            uint16_t SHIFT:1; uint16_t CTRL:1; uint16_t Q:1; uint16_t E:1;
            uint16_t R:1; uint16_t F:1; uint16_t G:1;
            uint16_t Z:1; uint16_t X:1; uint16_t C:1; uint16_t V:1; uint16_t B:1;
        } bit;
    } kb;
} RC_Ctrl_t;

#define RC_SW_UP   1
#define RC_SW_MID  3
#define RC_SW_DOWN 2

// ==========================================
// 2. 输出枚举
// ==========================================
typedef enum {
    CHASSIS_STOP = 0,
    CHASSIS_LOW,        // 普通低腿
    CHASSIS_FIGHT,      // 迎敌模式 (低腿姿态 + 特殊底盘算法)
    CHASSIS_LOW_SPIN,   // 低腿小陀螺
    CHASSIS_HIGH,       // 高腿
    CHASSIS_TERRAIN_READY, 
    CHASSIS_TERRAIN_EXECUTING
} ChassisState_e;

typedef enum { 
    GIMBAL_STOP=0, 
    GIMBAL_GYRO_STABILIZE, // 纯陀螺稳定 (遥控器用)
    GIMBAL_MOUSE_CONTROL,  // [New] 鼠标控制
    GIMBAL_AUTO_AIM        // [New] 自瞄模式
} GimbalState_e;

typedef enum { SHOOT_STOP=0, SHOOT_READY, SHOOT_FIRING } ShootState_e;

// ==========================================
// 3. 全局机器人上下文
// ==========================================
typedef struct {
    RC_Ctrl_t input;
    
    struct {
        ChassisState_e chassis;
        GimbalState_e  gimbal;
        ShootState_e   shoot;
        uint8_t chassis_speed; // 0:低速, 1:高速 (Shift)
    } output;
    
    uint8_t is_online;
    
    // 辅助变量
    int16_t  last_ch3;     
    uint16_t last_key_code; 
    uint32_t ctrl_tick;     
} RobotContext_t;

extern RobotContext_t g_robot_ctx;
extern FsmMachine_t g_top_fsm; 

void robot_logic_init(void);
void robot_logic_update(const RC_Ctrl_t* rc_data);

#endif // ROBOT_LOGIC_H