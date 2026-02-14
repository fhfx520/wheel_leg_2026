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
// 1. 输入数据结构 (RoboMaster 遥控器)
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

// 拨杆位置定义
#define RC_SW_UP   1
#define RC_SW_MID  3
#define RC_SW_DOWN 2

// ==========================================
// 2. 输出枚举 (各模块目标状态)
// ==========================================
typedef enum {
    TOP_MODE_PROTECT = 0,
    TOP_MODE_REMOTE,
    TOP_MODE_KEYBOARD
} TopMode_e;

typedef enum {
    CHASSIS_STOP = 0,
    CHASSIS_LOW,                // 普通低腿
    CHASSIS_FIGHT,              // 迎敌模式 (低腿)
    CHASSIS_LOW_SPIN,           // 低腿小陀螺
    CHASSIS_HIGH,               // 高腿
    CHASSIS_TERRAIN_READY,      // 跨越地形-准备
    CHASSIS_TERRAIN_EXECUTING   // 跨越地形-执行
} ChassisState_e;

typedef enum { 
    GIMBAL_STOP=0, 
    GIMBAL_GYRO_STABILIZE, // 纯陀螺稳定 (遥控器用)
    GIMBAL_MOUSE_CONTROL,  // 鼠标控制模式
    GIMBAL_AUTO_AIM        // 视觉自瞄模式
} GimbalState_e;

typedef enum { 
    SHOOT_PROTECT = 0, // 拨盘无力，摩擦轮停转 (保护)
    SHOOT_STOP,        // 拨盘有力但锁死，摩擦轮停转
    SHOOT_SINGLE,      // 单发模式 (摩擦轮转，拨盘单步)
    SHOOT_SERIES       // 连发模式 (摩擦轮转，拨盘连续)
} ShootState_e;

// ==========================================
// 3. 全局机器人上下文 (Context)
// ==========================================
typedef struct {
    RC_Ctrl_t input;      // 输入数据
    
    struct {
        TopMode_e      top_mode;   // 顶层大模式
        ChassisState_e chassis;    // 底盘状态
        GimbalState_e  gimbal;     // 云台状态
        ShootState_e   shoot;      // 发射器状态
        
        uint8_t chassis_speed;     // 0:标准速度, 1:高速 (Shift)
    } output;
    
    uint8_t is_online;    // 遥控器在线标志
    
    // --- 内部辅助变量 ---
    int16_t  last_ch3;      // 遥控器 ch3 历史值 (边沿检测)
    uint16_t last_key_code; // 键盘 key_code 历史值 (边沿检测)
    uint32_t ctrl_tick;     // Ctrl 长按计时器
    
} RobotContext_t;

extern RobotContext_t g_robot_ctx;
extern FsmMachine_t g_top_fsm; 

// ==========================================
// 4. API 接口
// ==========================================
void robot_logic_init(void);

/**
 * @brief 逻辑更新函数 (建议 2ms-10ms 调用一次)
 */
void robot_logic_update(const RC_Ctrl_t* rc_data);

#endif // ROBOT_LOGIC_H