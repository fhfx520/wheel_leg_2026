#include "robot_logic.h"
#include <string.h>
#include <stdlib.h> 

// ==========================================================
// [接口隔离] 弱引用底层 rc_fsm_check 接口与宏定义
// 这样不需要 #include 任何底层文件，保证在任何平台都能无痛编译
// ==========================================================
#ifndef RC_LEFT_LD
#define RC_LEFT_LD  (1<<3)
#endif
#ifndef RC_RIGHT_RD
#define RC_RIGHT_RD (1<<6)
#endif
extern uint8_t rc_fsm_check(uint8_t target_status);

RobotContext_t g_robot_ctx;
FsmMachine_t g_top_fsm; 

// ==========================================
// 前向声明
// ==========================================
extern const FsmState_t state_protect;
extern const FsmState_t state_remote;
extern const FsmState_t state_keyboard;

static const FsmState_t state_rem_low;
static const FsmState_t state_rem_spin;
static const FsmState_t state_rem_high;
static const FsmState_t state_rem_ter_ready;
static const FsmState_t state_rem_ter_run;

static const FsmState_t state_kb_low;
static const FsmState_t state_kb_high;
static const FsmState_t state_kb_fight; 
static const FsmState_t state_kb_spin; 
static const FsmState_t state_kb_ter_ready;
static const FsmState_t state_kb_ter_run;

static FsmMachine_t fsm_remote_sub;   
static FsmMachine_t fsm_keyboard_sub; 

// ==========================================
// 工具函数
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
static GimbalState_e get_kb_gimbal_mode(void) {
    return g_robot_ctx.input.mouse.r ? GIMBAL_AUTO_AIM : GIMBAL_MOUSE_CONTROL;
}
static ShootState_e get_kb_shoot_mode(void) {
    return g_robot_ctx.input.mouse.r ? SHOOT_SERIES : SHOOT_SERIES;
}

#define KEY_W 	  (1<<0)
#define KEY_S 	  (1<<1)
#define KEY_A 	  (1<<2)
#define KEY_D 	  (1<<3)
#define KEY_SHIFT (1<<4)
#define KEY_CTRL  (1<<5)
#define KEY_Q  	  (1<<6)
#define KEY_R     (1<<8)
#define KEY_F     (1<<9)
#define KEY_Z	  (1<<11)
#define KEY_X	  (1<<12)
#define KEY_C     (1<<13)
#define KEY_V     (1<<14)

// =========================================================================
// REMOTE 模式子状态机
// =========================================================================
static void rem_low_enter(void) { g_robot_ctx.output.chassis = CHASSIS_LOW; }
static void rem_low_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_LOW;
    g_robot_ctx.output.gimbal  = GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot   = SHOOT_STOP; 
    
    if (check_ch3_trigger()) { fsm_change(&fsm_remote_sub, &state_rem_spin); return; }
    if (g_robot_ctx.input.sw2 == RC_SW_MID) fsm_change(&fsm_remote_sub, &state_rem_high);
    else if (g_robot_ctx.input.sw2 == RC_SW_DOWN) fsm_change(&fsm_remote_sub, &state_rem_ter_ready);
}
static const FsmState_t state_rem_low = { .name = "REM_LOW", .enter = rem_low_enter, .execute = rem_low_execute };

static void rem_spin_enter(void) { g_robot_ctx.output.chassis = CHASSIS_LOW_SPIN; }
static void rem_spin_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_LOW_SPIN;
    g_robot_ctx.output.gimbal  = GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot   = SHOOT_STOP;
    
    if (check_ch3_trigger()) fsm_change(&fsm_remote_sub, &state_rem_low);
}
static const FsmState_t state_rem_spin = { .name = "REM_SPIN", .enter = rem_spin_enter, .execute = rem_spin_execute };

static void rem_high_enter(void) { g_robot_ctx.output.chassis = CHASSIS_HIGH; }
static void rem_high_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_HIGH;
    g_robot_ctx.output.gimbal  = GIMBAL_GYRO_STABILIZE;
    g_robot_ctx.output.shoot   = SHOOT_SINGLE; 
    
    if (g_robot_ctx.input.sw2 == RC_SW_UP) fsm_change(&fsm_remote_sub, &state_rem_low);
    else if (g_robot_ctx.input.sw2 == RC_SW_DOWN) fsm_change(&fsm_remote_sub, &state_rem_ter_ready);
}
static const FsmState_t state_rem_high = { .name = "REM_HIGH", .enter = rem_high_enter, .execute = rem_high_execute };

static void rem_ter_ready_enter(void) { g_robot_ctx.output.chassis = CHASSIS_TERRAIN_READY; }
static void rem_ter_ready_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_TERRAIN_READY;
    g_robot_ctx.output.gimbal  = GIMBAL_GYRO_STABILIZE;
    
    // [架枪模式集成] 左下且非右下 -> 连发 + 底盘急停锁死
    if (rc_fsm_check(RC_LEFT_LD) && !rc_fsm_check(RC_RIGHT_RD)) {
        g_robot_ctx.output.shoot   = SHOOT_SERIES; 
        g_robot_ctx.output.chassis = CHASSIS_STOP; 
    } else {
        g_robot_ctx.output.shoot   = SHOOT_SINGLE;  
    }
    
    if (g_robot_ctx.input.sw2 == RC_SW_UP) { fsm_change(&fsm_remote_sub, &state_rem_low); return; }
    if (g_robot_ctx.input.sw2 == RC_SW_MID) { fsm_change(&fsm_remote_sub, &state_rem_high); return; }
//    if (abs(g_robot_ctx.input.ch2) > 500) fsm_change(&fsm_remote_sub, &state_rem_ter_run);
}
static const FsmState_t state_rem_ter_ready = { .name = "REM_TER_RDY", .enter = rem_ter_ready_enter, .execute = rem_ter_ready_execute };

static void rem_ter_run_enter(void) { g_robot_ctx.output.chassis = CHASSIS_TERRAIN_EXECUTING; }
static void rem_ter_run_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_TERRAIN_EXECUTING;
    g_robot_ctx.output.gimbal  = GIMBAL_GYRO_STABILIZE;
    
    if (rc_fsm_check(RC_LEFT_LD) && !rc_fsm_check(RC_RIGHT_RD)) {
        g_robot_ctx.output.shoot   = SHOOT_SERIES;
        g_robot_ctx.output.chassis = CHASSIS_STOP; 
    } else {
        g_robot_ctx.output.shoot   = SHOOT_SINGLE;
    }
    
    if (g_robot_ctx.input.sw2 == RC_SW_UP) { fsm_change(&fsm_remote_sub, &state_rem_low); return; }
    if (g_robot_ctx.input.sw2 == RC_SW_MID) { fsm_change(&fsm_remote_sub, &state_rem_high); return; }
    if (abs(g_robot_ctx.input.ch2) <= 500) fsm_change(&fsm_remote_sub, &state_rem_ter_ready);
}
static const FsmState_t state_rem_ter_run = { .name = "REM_TER_RUN", .enter = rem_ter_run_enter, .execute = rem_ter_run_execute };

// =========================================================================
// KEYBOARD 模式子状态机 
// =========================================================================
static void kb_low_enter(void) { g_robot_ctx.output.chassis = CHASSIS_LOW; }
static void kb_low_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_LOW;
    g_robot_ctx.output.chassis_speed = (g_robot_ctx.input.kb.bit.SHIFT) ? 1 : 0;
    g_robot_ctx.output.gimbal = get_kb_gimbal_mode();
    g_robot_ctx.output.shoot  = get_kb_shoot_mode();

    if (check_key_trigger(KEY_A) || check_key_trigger(KEY_D)) { fsm_change(&fsm_keyboard_sub, &state_kb_fight); return; }
    if (check_key_trigger(KEY_R)) { fsm_change(&fsm_keyboard_sub, &state_kb_spin); return; }
    if (check_key_trigger(KEY_C)) { fsm_change(&fsm_keyboard_sub, &state_kb_high); return; }
	if (check_key_trigger(KEY_F)) { fsm_change(&fsm_keyboard_sub, &state_kb_ter_ready); return; }
}
static const FsmState_t state_kb_low = { .name = "KB_LOW", .enter = kb_low_enter, .execute = kb_low_execute };

static void kb_fight_enter(void) { g_robot_ctx.output.chassis = CHASSIS_FIGHT; }
static void kb_fight_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_FIGHT;
    g_robot_ctx.output.chassis_speed = (g_robot_ctx.input.kb.bit.SHIFT) ? 1 : 0;
    g_robot_ctx.output.gimbal = get_kb_gimbal_mode();
    g_robot_ctx.output.shoot  = get_kb_shoot_mode();

    if (check_key_trigger(KEY_W) || check_key_trigger(KEY_S)) { fsm_change(&fsm_keyboard_sub, &state_kb_low); return; }
    if (check_key_trigger(KEY_R)) { fsm_change(&fsm_keyboard_sub, &state_kb_spin); return; }
//    if (check_key_trigger(KEY_Z)) { fsm_change(&fsm_keyboard_sub, &state_kb_ter_ready); return; }
}
static const FsmState_t state_kb_fight = { .name = "KB_FIGHT", .enter = kb_fight_enter, .execute = kb_fight_execute };

static void kb_spin_enter(void) { g_robot_ctx.output.chassis = CHASSIS_LOW_SPIN; }
static void kb_spin_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_LOW_SPIN;
    g_robot_ctx.output.chassis_speed = (g_robot_ctx.input.kb.bit.SHIFT) ? 1 : 0;
    g_robot_ctx.output.gimbal = get_kb_gimbal_mode();
    g_robot_ctx.output.shoot  = get_kb_shoot_mode();

    if (check_key_trigger(KEY_R)) { fsm_change(&fsm_keyboard_sub, &state_kb_low); return; }
    if (check_key_trigger(KEY_A) || check_key_trigger(KEY_D)) { fsm_change(&fsm_keyboard_sub, &state_kb_fight); return; }
//    if (check_key_trigger(KEY_Z)) { fsm_change(&fsm_keyboard_sub, &state_kb_ter_ready); return; }
}
static const FsmState_t state_kb_spin = { .name = "KB_SPIN", .enter = kb_spin_enter, .execute = kb_spin_execute };

static void kb_high_enter(void) { g_robot_ctx.output.chassis = CHASSIS_HIGH; }
static void kb_high_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_HIGH;
    g_robot_ctx.output.chassis_speed = (g_robot_ctx.input.kb.bit.SHIFT) ? 1 : 0; 
    g_robot_ctx.output.gimbal = get_kb_gimbal_mode();
    g_robot_ctx.output.shoot  = get_kb_shoot_mode(); 

    if (check_key_trigger(KEY_C)) { fsm_change(&fsm_keyboard_sub, &state_kb_low); return; }
   //if (check_key_trigger(KEY_R)) { fsm_change(&fsm_keyboard_sub, &state_kb_spin); return; } 高腿长不能小陀螺
    if (check_key_trigger(KEY_F)) { fsm_change(&fsm_keyboard_sub, &state_kb_ter_ready); return; }
}
static const FsmState_t state_kb_high = { .name = "KB_HIGH", .enter = kb_high_enter, .execute = kb_high_execute };

static void kb_ter_ready_enter(void) {
    g_robot_ctx.output.chassis = CHASSIS_TERRAIN_READY;
    g_robot_ctx.ctrl_tick = 0;
}
static void kb_ter_ready_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_TERRAIN_READY;
    g_robot_ctx.output.gimbal = get_kb_gimbal_mode();
    g_robot_ctx.output.shoot  = get_kb_shoot_mode(); 
    
//    if (check_key_trigger(KEY_C) || check_key_trigger(KEY_R)) { fsm_change(&fsm_keyboard_sub, &state_kb_low); return; }
//    if (g_robot_ctx.input.kb.bit.CTRL) {
//        g_robot_ctx.ctrl_tick++;
//        if (g_robot_ctx.ctrl_tick > 20) fsm_change(&fsm_keyboard_sub, &state_kb_ter_run);
//    } else { g_robot_ctx.ctrl_tick = 0; }
	if (check_key_trigger(KEY_F)) { fsm_change(&fsm_keyboard_sub, &state_kb_low); return; }
	if (check_key_trigger(KEY_C)) { fsm_change(&fsm_keyboard_sub, &state_kb_high); return; }
	if (check_key_trigger(KEY_V)) { fsm_change(&fsm_keyboard_sub, &state_kb_ter_run); return; }
	
}
static const FsmState_t state_kb_ter_ready = { .name = "KB_TER_RDY", .enter = kb_ter_ready_enter, .execute = kb_ter_ready_execute };

static void kb_ter_run_enter(void) { g_robot_ctx.output.chassis = CHASSIS_TERRAIN_EXECUTING; }
static void kb_ter_run_execute(void) {
    g_robot_ctx.output.chassis = CHASSIS_TERRAIN_EXECUTING;
    g_robot_ctx.output.gimbal = get_kb_gimbal_mode();
    g_robot_ctx.output.shoot  = get_kb_shoot_mode();
//    if (!g_robot_ctx.input.kb.bit.CTRL) fsm_change(&fsm_keyboard_sub, &state_kb_ter_ready);
//	if (check_key_trigger(KEY_C)) { fsm_change(&fsm_keyboard_sub, &state_kb_low); return; }
//	if (check_key_trigger(KEY_Z)) { fsm_change(&fsm_keyboard_sub, &state_kb_high); return; }
	if (g_robot_ctx.sky_finish_flag) { fsm_change(&fsm_keyboard_sub, &state_kb_low); return; }
}
static const FsmState_t state_kb_ter_run = { .name = "KB_TER_RUN", .enter = kb_ter_run_enter, .execute = kb_ter_run_execute };

// =========================================================================
// TOP LEVEL 大模式实现
// =========================================================================
static void protect_enter(void) {
    g_robot_ctx.output.top_mode = TOP_MODE_PROTECT;
    g_robot_ctx.output.chassis  = CHASSIS_STOP;
    g_robot_ctx.output.gimbal   = GIMBAL_STOP;
    g_robot_ctx.output.shoot    = SHOOT_PROTECT;
}
static void protect_execute(void) {
    g_robot_ctx.output.top_mode = TOP_MODE_PROTECT;
    g_robot_ctx.output.chassis  = CHASSIS_STOP;
    g_robot_ctx.output.gimbal   = GIMBAL_STOP;
    g_robot_ctx.output.shoot    = SHOOT_PROTECT;
    
    if (g_robot_ctx.is_online) {
        if (g_robot_ctx.input.sw1 == RC_SW_MID && g_robot_ctx.input.sw2 == RC_SW_UP) {
            fsm_change(&g_top_fsm, &state_remote);
        }
        else if (g_robot_ctx.input.sw1 == RC_SW_DOWN) {
            fsm_change(&g_top_fsm, &state_keyboard);
        }
    }
}
const FsmState_t state_protect = { .name = "PROTECT", .enter = protect_enter, .execute = protect_execute };

static void remote_enter(void) {
    g_robot_ctx.output.top_mode = TOP_MODE_REMOTE;
    fsm_init(&fsm_remote_sub, &state_rem_low);
}
static void remote_execute(void) {
    g_robot_ctx.output.top_mode = TOP_MODE_REMOTE;
    
    // [全局异常检测] 如果在遥控模式下双摇杆内八/右下断电触发
    if (rc_fsm_check(RC_LEFT_LD) && rc_fsm_check(RC_RIGHT_RD)) {
        fsm_change(&g_top_fsm, &state_protect);
        return;
    }

    if (g_robot_ctx.input.sw1 == RC_SW_UP) { fsm_change(&g_top_fsm, &state_protect); return; }
    if (g_robot_ctx.input.sw1 == RC_SW_DOWN) { fsm_change(&g_top_fsm, &state_keyboard); return; }
    
    fsm_run(&fsm_remote_sub);
}
const FsmState_t state_remote = { .name = "REMOTE", .enter = remote_enter, .execute = remote_execute };

static void keyboard_enter(void) {
    g_robot_ctx.output.top_mode = TOP_MODE_KEYBOARD;
    fsm_init(&fsm_keyboard_sub, &state_kb_low);
}
static void keyboard_execute(void) {
    g_robot_ctx.output.top_mode = TOP_MODE_KEYBOARD;
    if (g_robot_ctx.input.sw1 == RC_SW_UP) { fsm_change(&g_top_fsm, &state_protect); return; }
    if (g_robot_ctx.input.sw1 == RC_SW_MID && g_robot_ctx.input.sw2 == RC_SW_UP) { 
		fsm_change(&g_top_fsm, &state_remote); 
		return; 
	}
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