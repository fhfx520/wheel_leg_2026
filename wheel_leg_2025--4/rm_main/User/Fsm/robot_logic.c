#include "robot_logic.h"
#include <string.h>

// 全局上下文
RobotContext_t g_robot_ctx;
FsmMachine_t g_top_fsm; // 顶层状态机 (Level 1)

// ==========================================
// 前向声明 (Forward Declarations)
// ==========================================
// 顶层状态
extern const FsmState_t state_protect;
extern const FsmState_t state_remote;
extern const FsmState_t state_keyboard;

// 遥控子状态 (Level 2 Sub-States)
// 这些状态属于 Remote 的内部实现，不需要暴露给外部
static const FsmState_t state_rem_low;     // UP
static const FsmState_t state_rem_high;    // MID
static const FsmState_t state_rem_terrain; // DOWN

// 遥控模式专用的子状态机
static FsmMachine_t fsm_remote_sub; 

// ==========================================
// LEVEL 2: 遥控子状态实现 (Sub-States)
// ==========================================

// --- 子状态 1: 低腿模式 (UP) ---
static void rem_low_enter(void) {
    g_robot_ctx.output.chassis = CHASSIS_LOW;
    g_robot_ctx.output.gimbal  = GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot   = SHOOT_STOP;
}
static void rem_low_execute(void) {
    // 持续输出防止被覆盖
    g_robot_ctx.output.chassis = CHASSIS_LOW;
    g_robot_ctx.output.gimbal  = GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot   = SHOOT_STOP;

    // [Transition] 监听 sw2 切换其他子状态
    if (g_robot_ctx.input.sw2 == RC_SW_MID) {
        fsm_change(&fsm_remote_sub, &state_rem_high);
    } else if (g_robot_ctx.input.sw2 == RC_SW_DOWN) {
        fsm_change(&fsm_remote_sub, &state_rem_terrain);
    }
}
static const FsmState_t state_rem_low = {
    .name = "REM_LOW", .enter = rem_low_enter, .execute = rem_low_execute, .exit = NULL
};

// --- 子状态 2: 高腿模式 (MID) ---
static void rem_high_enter(void) {
    g_robot_ctx.output.chassis = CHASSIS_HIGH;
    g_robot_ctx.output.gimbal  = GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot   = SHOOT_READY;
}
static void rem_high_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_HIGH;
    g_robot_ctx.output.gimbal  = GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot   = SHOOT_READY;

    // [Transition]
    if (g_robot_ctx.input.sw2 == RC_SW_UP) {
        fsm_change(&fsm_remote_sub, &state_rem_low);
    } else if (g_robot_ctx.input.sw2 == RC_SW_DOWN) {
        fsm_change(&fsm_remote_sub, &state_rem_terrain);
    }
}
static const FsmState_t state_rem_high = {
    .name = "REM_HIGH", .enter = rem_high_enter, .execute = rem_high_execute, .exit = NULL
};

// --- 子状态 3: 跨越地形 (DOWN) ---
static void rem_terrain_enter(void) {
    g_robot_ctx.output.chassis = CHASSIS_TERRAIN;
    g_robot_ctx.output.gimbal  = GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot   = SHOOT_FIRING;
}
static void rem_terrain_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_TERRAIN;
    g_robot_ctx.output.gimbal  = GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot   = SHOOT_FIRING;

    // [Transition]
    if (g_robot_ctx.input.sw2 == RC_SW_UP) {
        fsm_change(&fsm_remote_sub, &state_rem_low);
    } else if (g_robot_ctx.input.sw2 == RC_SW_MID) {
        fsm_change(&fsm_remote_sub, &state_rem_high);
    }
}
static const FsmState_t state_rem_terrain = {
    .name = "REM_TERRAIN", .enter = rem_terrain_enter, .execute = rem_terrain_execute, .exit = NULL
};

// ==========================================
// LEVEL 1: 顶层状态实现 (Top-States)
// ==========================================

// --- 状态: PROTECT ---
static void protect_enter(void) {
    g_robot_ctx.output.chassis = CHASSIS_STOP;
    g_robot_ctx.output.gimbal  = GIMBAL_STOP;
    g_robot_ctx.output.shoot   = SHOOT_STOP;
}
static void protect_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_STOP;
    g_robot_ctx.output.gimbal  = GIMBAL_STOP;
    g_robot_ctx.output.shoot   = SHOOT_STOP;

    // 监听 sw1 切换大模式
    if (g_robot_ctx.is_online) {
        if (g_robot_ctx.input.sw1 == RC_SW_MID) fsm_change(&g_top_fsm, &state_remote);
        else if (g_robot_ctx.input.sw1 == RC_SW_DOWN) fsm_change(&g_top_fsm, &state_keyboard);
    }
}
const FsmState_t state_protect = {
    .name = "PROTECT", .enter = protect_enter, .execute = protect_execute, .exit = NULL
};

// --- 状态: REMOTE (父状态) ---
// 这里的 Enter/Execute 不负责具体逻辑，只负责“管理子状态机”
static void remote_enter(void) {
    // [Init Sub-FSM] 刚切进遥控模式时，根据当前 sw2 位置决定进入哪个子状态
    // 这样防止切回来时状态不一致
    const FsmState_t* init_sub_state;
    
    switch (g_robot_ctx.input.sw2) {
        case RC_SW_UP:   init_sub_state = &state_rem_low; break;
        case RC_SW_DOWN: init_sub_state = &state_rem_terrain; break;
        case RC_SW_MID:  
        default:         init_sub_state = &state_rem_high; break;
    }
    
    fsm_init(&fsm_remote_sub, init_sub_state);
}

static void remote_execute(void) {
    // 1. [Top Transition] 检查是否要退出 Remote 大模式
    if (g_robot_ctx.input.sw1 == RC_SW_UP) {
        fsm_change(&g_top_fsm, &state_protect);
        return;
    }
    if (g_robot_ctx.input.sw1 == RC_SW_DOWN) {
        fsm_change(&g_top_fsm, &state_keyboard);
        return;
    }

    // 2. [Run Sub-FSM] 驱动子状态机运行
    // 真正的逻辑（Low/High/Terrain）是在这里面执行的
    fsm_run(&fsm_remote_sub);
}

const FsmState_t state_remote = {
    .name = "REMOTE", .enter = remote_enter, .execute = remote_execute, .exit = NULL
};

// --- 状态: KEYBOARD ---
static void keyboard_enter(void) {
    g_robot_ctx.output.chassis = CHASSIS_INDEPENDENT;
    g_robot_ctx.output.gimbal  = GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot   = SHOOT_READY;
}
static void keyboard_execute(void) {
    if (g_robot_ctx.input.sw1 == RC_SW_UP) { fsm_change(&g_top_fsm, &state_protect); return; }
    if (g_robot_ctx.input.sw1 == RC_SW_MID) { fsm_change(&g_top_fsm, &state_remote); return; }

    // 简单的键盘逻辑 (如果这里很复杂，也可以做成子状态机)
    if (g_robot_ctx.input.kb.bit.CTRL) g_robot_ctx.output.chassis = CHASSIS_LOW;
    else g_robot_ctx.output.chassis = CHASSIS_INDEPENDENT;

    g_robot_ctx.output.gimbal = g_robot_ctx.input.mouse.r ? GIMBAL_AUTO_AIM : GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot  = g_robot_ctx.input.mouse.l ? SHOOT_FIRING : SHOOT_READY;
}
const FsmState_t state_keyboard = {
    .name = "KEYBOARD", .enter = keyboard_enter, .execute = keyboard_execute, .exit = NULL
};

// ==========================================
// API Implementations
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
}