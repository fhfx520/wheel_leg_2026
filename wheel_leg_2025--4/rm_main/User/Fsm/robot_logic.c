#include "robot_logic.h"
#include <string.h>
#include <stdlib.h> 

RobotContext_t g_robot_ctx;
FsmMachine_t g_top_fsm; 

// ==========================================
// Forward Declarations
// ==========================================
extern const FsmState_t state_protect;
extern const FsmState_t state_remote;
extern const FsmState_t state_keyboard;

// Keyboard 子状态
static const FsmState_t state_kb_low;
static const FsmState_t state_kb_high;
static const FsmState_t state_kb_fight; 
static const FsmState_t state_kb_spin; 
static const FsmState_t state_kb_ter_ready;
static const FsmState_t state_kb_ter_run;

// Remote 子状态 (保持逻辑一致)
static const FsmState_t state_rem_low;
static const FsmState_t state_rem_spin;
static const FsmState_t state_rem_high;
static const FsmState_t state_rem_ter_ready;
static const FsmState_t state_rem_ter_run;

static FsmMachine_t fsm_remote_sub;   
static FsmMachine_t fsm_keyboard_sub; 

// ==========================================
// Tools
// ==========================================
static uint8_t check_key_trigger(uint16_t key_mask) {
    if ((g_robot_ctx.input.kb.key_code & key_mask) && 
       !(g_robot_ctx.last_key_code & key_mask)) return 1;
    return 0;
}
static uint8_t check_ch3_trigger(void) {
    if (g_robot_ctx.input.ch3 >= 650 && g_robot_ctx.last_ch3 < 650) return 1;
    return 0;
}

// [New] 统一的云台逻辑 helper
// 规则：按住右键 -> 自瞄；否则 -> 鼠标控制
static GimbalState_e get_kb_gimbal_mode(void) {
    if (g_robot_ctx.input.mouse.r) {
        return GIMBAL_AUTO_AIM;
    } else {
        return GIMBAL_MOUSE_CONTROL;
    }
}

#define KEY_SHIFT (1<<4)
#define KEY_CTRL  (1<<5)
#define KEY_R     (1<<8)
#define KEY_F     (1<<9)
#define KEY_C     (1<<13)

// ==========================================
// KEYBOARD FSM (Level 2)
// ==========================================

// --- 1. KB Low (普通低腿) ---
static void kb_low_enter(void) {
    g_robot_ctx.output.chassis = CHASSIS_LOW;
}
static void kb_low_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_LOW;
    g_robot_ctx.output.chassis_speed = (g_robot_ctx.input.kb.bit.SHIFT) ? 1 : 0;
    
    // [Gimbal] 统一逻辑
    g_robot_ctx.output.gimbal = get_kb_gimbal_mode();
    // [Shoot] 左键开火
    g_robot_ctx.output.shoot  = g_robot_ctx.input.mouse.l ? SHOOT_FIRING : SHOOT_READY;

    // [Trans] F -> 迎敌 (仅低腿可进)
    if (check_key_trigger(KEY_F)) { fsm_change(&fsm_keyboard_sub, &state_kb_fight); return; }
    // [Trans] R -> 小陀螺
    if (check_key_trigger(KEY_R)) { fsm_change(&fsm_keyboard_sub, &state_kb_spin); return; }
    // [Trans] C -> 高腿
    if (check_key_trigger(KEY_C)) { fsm_change(&fsm_keyboard_sub, &state_kb_high); return; }
    // [Trans] Ctrl -> 地形
    if (check_key_trigger(KEY_CTRL)) { fsm_change(&fsm_keyboard_sub, &state_kb_ter_ready); return; }
}
static const FsmState_t state_kb_low = { .name = "KB_LOW", .enter = kb_low_enter, .execute = kb_low_execute };

// --- 2. KB Fight (迎敌模式) ---
static void kb_fight_enter(void) {
    g_robot_ctx.output.chassis = CHASSIS_FIGHT; 
}
static void kb_fight_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_FIGHT;
    g_robot_ctx.output.chassis_speed = (g_robot_ctx.input.kb.bit.SHIFT) ? 1 : 0;
    
    // [Gimbal] 统一逻辑 (即使是迎敌底盘，云台也听右键的)
    g_robot_ctx.output.gimbal = get_kb_gimbal_mode();
    g_robot_ctx.output.shoot  = g_robot_ctx.input.mouse.l ? SHOOT_FIRING : SHOOT_READY;

    // [Trans] F -> 退出迎敌 (回普通低腿)
    if (check_key_trigger(KEY_F)) { fsm_change(&fsm_keyboard_sub, &state_kb_low); return; }
    
    // [Trans] R -> 普通小陀螺 (直接切过去，不需要迎敌小陀螺状态)
    if (check_key_trigger(KEY_R)) { fsm_change(&fsm_keyboard_sub, &state_kb_spin); return; }
    
    // [Trans] Ctrl -> 地形 (允许)
    if (check_key_trigger(KEY_CTRL)) { fsm_change(&fsm_keyboard_sub, &state_kb_ter_ready); return; }
    
    // [Constraint] C 键无效 (不能切腿长)
}
static const FsmState_t state_kb_fight = { .name = "KB_FIGHT", .enter = kb_fight_enter, .execute = kb_fight_execute };

// --- 3. KB Spin (普通小陀螺) ---
static void kb_spin_enter(void) {
    g_robot_ctx.output.chassis = CHASSIS_LOW_SPIN;
}
static void kb_spin_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_LOW_SPIN;
    g_robot_ctx.output.chassis_speed = (g_robot_ctx.input.kb.bit.SHIFT) ? 1 : 0;
    
    // [Gimbal] 统一逻辑
    g_robot_ctx.output.gimbal = get_kb_gimbal_mode();
    g_robot_ctx.output.shoot  = g_robot_ctx.input.mouse.l ? SHOOT_FIRING : SHOOT_READY;

    // [Trans] R -> 停止小陀螺 (回普通低腿)
    if (check_key_trigger(KEY_R)) { fsm_change(&fsm_keyboard_sub, &state_kb_low); return; }
    
    // [Trans] F -> 进迎敌 (停止旋转，变成迎敌站立)
    // 如果你希望 F 键无效，就把这行删掉
    if (check_key_trigger(KEY_F)) { fsm_change(&fsm_keyboard_sub, &state_kb_fight); return; }
    
    // [Trans] Ctrl -> 地形
    if (check_key_trigger(KEY_CTRL)) { fsm_change(&fsm_keyboard_sub, &state_kb_ter_ready); return; }
}
static const FsmState_t state_kb_spin = { .name = "KB_SPIN", .enter = kb_spin_enter, .execute = kb_spin_execute };

// --- 4. KB High (高腿) ---
static void kb_high_enter(void) {
    g_robot_ctx.output.chassis = CHASSIS_HIGH;
}
static void kb_high_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_HIGH;
    g_robot_ctx.output.chassis_speed = 0; // 高腿不加速
    
    // [Gimbal] 统一逻辑
    g_robot_ctx.output.gimbal = get_kb_gimbal_mode();
    g_robot_ctx.output.shoot  = SHOOT_STOP; // 高腿通常不射击，或者改为READY

    // [Trans] C -> 回低腿
    if (check_key_trigger(KEY_C)) { fsm_change(&fsm_keyboard_sub, &state_kb_low); return; }
    
    // [Trans] R -> 强切小陀螺 (变相回低腿)
    if (check_key_trigger(KEY_R)) { fsm_change(&fsm_keyboard_sub, &state_kb_spin); return; }
    
    // [Trans] Ctrl -> 地形
    if (check_key_trigger(KEY_CTRL)) { fsm_change(&fsm_keyboard_sub, &state_kb_ter_ready); return; }
    
    // [Constraint] F 键无效 (不能在高腿进迎敌)
}
static const FsmState_t state_kb_high = { .name = "KB_HIGH", .enter = kb_high_enter, .execute = kb_high_execute };

// --- 5. KB Terrain Ready & Run ---
static void kb_ter_ready_enter(void) {
    g_robot_ctx.output.chassis = CHASSIS_TERRAIN_READY;
    g_robot_ctx.ctrl_tick = 0;
}
static void kb_ter_ready_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_TERRAIN_READY;
    // [Gimbal] 地形模式下也允许鼠标看视野
    g_robot_ctx.output.gimbal = get_kb_gimbal_mode();
    
    if (check_key_trigger(KEY_C) || check_key_trigger(KEY_R)) {
        fsm_change(&fsm_keyboard_sub, &state_kb_low);
        return;
    }
    // 长按监测
    if (g_robot_ctx.input.kb.bit.CTRL) {
        g_robot_ctx.ctrl_tick++;
        if (g_robot_ctx.ctrl_tick > 20) fsm_change(&fsm_keyboard_sub, &state_kb_ter_run);
    } else {
        g_robot_ctx.ctrl_tick = 0;
    }
}
static const FsmState_t state_kb_ter_ready = { .name = "KB_TER_RDY", .enter = kb_ter_ready_enter, .execute = kb_ter_ready_execute };

static void kb_ter_run_enter(void) { g_robot_ctx.output.chassis = CHASSIS_TERRAIN_EXECUTING; }
static void kb_ter_run_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_TERRAIN_EXECUTING;
    // 地形执行中也给云台控制权
    g_robot_ctx.output.gimbal = get_kb_gimbal_mode();
    
    if (!g_robot_ctx.input.kb.bit.CTRL) fsm_change(&fsm_keyboard_sub, &state_kb_ter_ready);
}
static const FsmState_t state_kb_ter_run = { .name = "KB_TER_RUN", .enter = kb_ter_run_enter, .execute = kb_ter_run_execute };


// ==========================================
// REMOTE FSM (Level 2)
// ==========================================
// 逻辑保持原样，仅为了完整性列出
static void rem_low_enter(void) { g_robot_ctx.output.chassis = CHASSIS_LOW; }
static void rem_low_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_LOW;
    // Remote 模式下的云台保持陀螺稳定
    g_robot_ctx.output.gimbal = GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot  = SHOOT_STOP;
    
    if (check_ch3_trigger()) { fsm_change(&fsm_remote_sub, &state_rem_spin); return; }
    if (g_robot_ctx.input.sw2 == RC_SW_MID) fsm_change(&fsm_remote_sub, &state_rem_high);
    else if (g_robot_ctx.input.sw2 == RC_SW_DOWN) fsm_change(&fsm_remote_sub, &state_rem_ter_ready);
}
static const FsmState_t state_rem_low = { .name = "REM_LOW", .enter = rem_low_enter, .execute = rem_low_execute };

static void rem_spin_enter(void) { g_robot_ctx.output.chassis = CHASSIS_LOW_SPIN; }
static void rem_spin_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_LOW_SPIN;
    g_robot_ctx.output.gimbal = GIMBAL_GYRO_STABILIZE;
    if (check_ch3_trigger()) fsm_change(&fsm_remote_sub, &state_rem_low);
}
static const FsmState_t state_rem_spin = { .name = "REM_SPIN", .enter = rem_spin_enter, .execute = rem_spin_execute };

static void rem_high_enter(void) { g_robot_ctx.output.chassis = CHASSIS_HIGH; }
static void rem_high_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_HIGH;
    g_robot_ctx.output.gimbal = GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot  = SHOOT_READY;
    if (g_robot_ctx.input.sw2 == RC_SW_UP) fsm_change(&fsm_remote_sub, &state_rem_low);
    else if (g_robot_ctx.input.sw2 == RC_SW_DOWN) fsm_change(&fsm_remote_sub, &state_rem_ter_ready);
}
static const FsmState_t state_rem_high = { .name = "REM_HIGH", .enter = rem_high_enter, .execute = rem_high_execute };

static void rem_ter_ready_enter(void) { g_robot_ctx.output.chassis = CHASSIS_TERRAIN_READY; }
static void rem_ter_ready_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_TERRAIN_READY;
    g_robot_ctx.output.gimbal = GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot  = SHOOT_FIRING;
    if (g_robot_ctx.input.sw2 == RC_SW_UP) { fsm_change(&fsm_remote_sub, &state_rem_low); return; }
    if (g_robot_ctx.input.sw2 == RC_SW_MID) { fsm_change(&fsm_remote_sub, &state_rem_high); return; }
    if (abs(g_robot_ctx.input.ch2) > 500) fsm_change(&fsm_remote_sub, &state_rem_ter_run);
}
static const FsmState_t state_rem_ter_ready = { .name = "REM_TER_RDY", .enter = rem_ter_ready_enter, .execute = rem_ter_ready_execute };

static void rem_ter_run_enter(void) { g_robot_ctx.output.chassis = CHASSIS_TERRAIN_EXECUTING; }
static void rem_ter_run_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_TERRAIN_EXECUTING;
    g_robot_ctx.output.gimbal = GIMBAL_GYRO_STABILIZE;
    if (g_robot_ctx.input.sw2 == RC_SW_UP) { fsm_change(&fsm_remote_sub, &state_rem_low); return; }
    if (g_robot_ctx.input.sw2 == RC_SW_MID) { fsm_change(&fsm_remote_sub, &state_rem_high); return; }
    if (abs(g_robot_ctx.input.ch2) <= 500) fsm_change(&fsm_remote_sub, &state_rem_ter_ready);
}
static const FsmState_t state_rem_ter_run = { .name = "REM_TER_RUN", .enter = rem_ter_run_enter, .execute = rem_ter_run_execute };

// ==========================================
// TOP LEVEL FSM
// ==========================================
static void protect_enter(void) {
    memset(&g_robot_ctx.output, 0, sizeof(g_robot_ctx.output)); // Clear all
}
static void protect_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_STOP;
    if (g_robot_ctx.is_online) {
        if (g_robot_ctx.input.sw1 == RC_SW_MID) fsm_change(&g_top_fsm, &state_remote);
        else if (g_robot_ctx.input.sw1 == RC_SW_DOWN) fsm_change(&g_top_fsm, &state_keyboard);
    }
}
const FsmState_t state_protect = { .name = "PROTECT", .enter = protect_enter, .execute = protect_execute };

static void remote_enter(void) {
    const FsmState_t* init_st;
    switch (g_robot_ctx.input.sw2) {
        case RC_SW_UP:   init_st = &state_rem_low; break;
        case RC_SW_DOWN: init_st = &state_rem_ter_ready; break;
        default:         init_st = &state_rem_high; break;
    }
    fsm_init(&fsm_remote_sub, init_st);
}
static void remote_execute(void) {
    if (g_robot_ctx.input.sw1 == RC_SW_UP) { fsm_change(&g_top_fsm, &state_protect); return; }
    if (g_robot_ctx.input.sw1 == RC_SW_DOWN) { fsm_change(&g_top_fsm, &state_keyboard); return; }
    fsm_run(&fsm_remote_sub);
}
const FsmState_t state_remote = { .name = "REMOTE", .enter = remote_enter, .execute = remote_execute };

static void keyboard_enter(void) { fsm_init(&fsm_keyboard_sub, &state_kb_low); }
static void keyboard_execute(void) {
    if (g_robot_ctx.input.sw1 == RC_SW_UP) { fsm_change(&g_top_fsm, &state_protect); return; }
    if (g_robot_ctx.input.sw1 == RC_SW_MID) { fsm_change(&g_top_fsm, &state_remote); return; }
    fsm_run(&fsm_keyboard_sub);
}
const FsmState_t state_keyboard = { .name = "KEYBOARD", .enter = keyboard_enter, .execute = keyboard_execute };

// ==========================================
// API Implementation
// ==========================================
void robot_logic_init(void) {
    memset(&g_robot_ctx, 0, sizeof(g_robot_ctx));
    fsm_init(&g_top_fsm, &state_protect);
}

void robot_logic_update(const RC_Ctrl_t* rc_data) {
    if (rc_data) {
        g_robot_ctx.input = *rc_data;
        g_robot_ctx.is_online = 1;
    } else {
        g_robot_ctx.is_online = 0;
        fsm_change(&g_top_fsm, &state_protect);
    }
    fsm_run(&g_top_fsm);
    if (rc_data) {
        g_robot_ctx.last_ch3 = rc_data->ch3;
        g_robot_ctx.last_key_code = rc_data->kb.key_code;
    }
}