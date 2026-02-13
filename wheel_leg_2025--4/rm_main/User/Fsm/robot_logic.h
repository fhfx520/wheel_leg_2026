#ifndef ROBOT_LOGIC_H
#define ROBOT_LOGIC_H

#include "fsm_core.h" 
#include <stdint.h>

// --- 编译器宏适配 ---
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
    CHASSIS_LOW,        // 低腿 (sw2 UP)
    CHASSIS_HIGH,       // 高腿 (sw2 MID)
    CHASSIS_TERRAIN,    // 跨越 (sw2 DOWN)
    CHASSIS_INDEPENDENT // 键鼠
} ChassisState_e;

typedef enum { GIMBAL_STOP=0, GIMBAL_GYRO_STABILIZE, GIMBAL_AUTO_AIM } GimbalState_e;
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
    } output;
    uint8_t is_online;
} RobotContext_t;

extern RobotContext_t g_robot_ctx;
extern FsmMachine_t g_top_fsm; // 顶层状态机

// API
void robot_logic_init(void);
void robot_logic_update(const RC_Ctrl_t* rc_data);

#endif // ROBOT_LOGIC_H