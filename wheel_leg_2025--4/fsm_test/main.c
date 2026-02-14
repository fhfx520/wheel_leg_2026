#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <windows.h> 
#include "robot_logic.h"

// ==========================================
// 1. 宏定义：复刻底层宏，供模拟器自身使用
// ==========================================
#ifndef RC_LEFT_LD
#define RC_LEFT_LU  ( 1<<0 ) 
#define RC_LEFT_RU  ( 1<<1 ) 
#define RC_LEFT_RD  ( 1<<2 ) 
#define RC_LEFT_LD  ( 1<<3 ) // 视觉模式 (左下)
#define RC_RIGHT_LU ( 1<<4 ) 
#define RC_RIGHT_RU ( 1<<5 ) 
#define RC_RIGHT_RD ( 1<<6 ) // 禁止发射 (右下)
#define RC_RIGHT_LD ( 1<<7 ) 
#endif

// ==========================================
// 2. 核心！在 main.c 中为 robot_logic.c 提供 rc_fsm_check 函数
// 这使得 robot_logic.c 可以无缝在 PC 上编译，而到了真机上会自动连上 prot_dr16.c
// ==========================================
RC_Ctrl_t rc_sim;
uint8_t remote_online = 1;      // 遥控器在线状态
uint8_t mock_init_status = 0;   // 锁存的全局状态

// 手搓模拟 prot_dr16.c 里的锁存器
void mock_rc_fsm_init(uint8_t trig_flag) {
    static uint8_t last_trig_flag = 0;
    static uint8_t state = 0;
    
    // 检测到触发信号上升沿 (遥控器刚连上的那一瞬间)
    if (trig_flag == 1 && last_trig_flag == 0) {  
        if (rc_sim.ch3 < -500 && rc_sim.ch4 >  500) state |= RC_LEFT_LU;
        if (rc_sim.ch3 >  500 && rc_sim.ch4 >  500) state |= RC_LEFT_RU;
        if (rc_sim.ch3 >  500 && rc_sim.ch4 < -500) state |= RC_LEFT_RD;
        if (rc_sim.ch3 < -500 && rc_sim.ch4 < -500) state |= RC_LEFT_LD;
        if (rc_sim.ch2 >  500 && rc_sim.ch1 < -500) state |= RC_RIGHT_LU;
        if (rc_sim.ch2 >  500 && rc_sim.ch1 >  500) state |= RC_RIGHT_RU;
        if (rc_sim.ch2 < -500 && rc_sim.ch1 >  500) state |= RC_RIGHT_RD;
        if (rc_sim.ch2 < -500 && rc_sim.ch1 < -500) state |= RC_RIGHT_LD;
    } else if (trig_flag == 0 && last_trig_flag == 1) { 
        state = 0x00;
    }
    last_trig_flag = trig_flag;
    mock_init_status = state;
    
    if (!trig_flag && last_trig_flag) {  
        mock_init_status = 0;
    }
}

// FSM 调用的底层接口，在这里被 PC 接管
uint8_t rc_fsm_check(uint8_t target_status) {
    return (mock_init_status & target_status) ? 1 : 0;
}

// ==========================================
// 辅助打印函数 
// ==========================================
const char* top_mode_str(TopMode_e mode) {
    switch(mode) {
        case TOP_MODE_PROTECT: return "PROTECT (保护急停)";
        case TOP_MODE_REMOTE:  return "REMOTE (遥控)";
        case TOP_MODE_KEYBOARD:return "KEYBOARD (键鼠)";
        default: return "UNKNOWN";
    }
}

const char* chassis_state_str(ChassisState_e state) {
    switch(state) {
        case CHASSIS_STOP: return "STOP (无力/锁死)";
        case CHASSIS_LOW:  return "LOW (低腿长)";
        case CHASSIS_HIGH: return "HIGH (高腿长)";
        case CHASSIS_LOW_SPIN: return "LOW_SPIN (低腿小陀螺)";
        case CHASSIS_FIGHT: return "FIGHT (迎敌模式)";
        case CHASSIS_TERRAIN_READY: return "TERRAIN_READY (跨越准备)";
        case CHASSIS_TERRAIN_EXECUTING: return "TERRAIN_RUN (跨越执行)";
        default: return "UNKNOWN";
    }
}

const char* gimbal_state_str(GimbalState_e state) {
    switch(state) {
        case GIMBAL_STOP: return "STOP";
        case GIMBAL_GYRO_STABILIZE: return "STABILIZE (陀螺稳定)";
        case GIMBAL_MOUSE_CONTROL:  return "MOUSE (鼠标控制)";
        case GIMBAL_AUTO_AIM:       return "AUTO_AIM (自瞄)";
        default: return "UNKNOWN";
    }
}

const char* shoot_state_str(ShootState_e state) {
    switch(state) {
        case SHOOT_PROTECT: return "PROTECT (无力保护)";
        case SHOOT_STOP:    return "STOP (有力锁死)";
        case SHOOT_SINGLE:  return "SINGLE (单发点射)";
        case SHOOT_SERIES:  return "SERIES (连发扫射)";
        case SHOOT_READY:   return "READY (摩擦轮转, 待命)";
        default: return "UNKNOWN";
    }
}

const char* sw_str(uint8_t sw) {
    if (sw == 1) return "1-UP (上)";
    if (sw == 3) return "3-MID (中)";
    if (sw == 2) return "2-DOWN (下)";
    return "0-UNKNOWN";
}

// ==========================================
// 实时外设捕获核心逻辑
// ==========================================
void capture_physical_inputs() {
    rc_sim.kb.key_code = 0; 

    // O 键防抖模拟掉线重连
    static uint8_t key_o_pressed = 0;
    if (GetAsyncKeyState('O') & 0x8000) {
        if (!key_o_pressed) {
            remote_online = !remote_online; 
            key_o_pressed = 1;
        }
    } else {
        key_o_pressed = 0;
    }

    rc_sim.kb.bit.W = (GetAsyncKeyState('W') & 0x8000) ? 1 : 0;
    rc_sim.kb.bit.S = (GetAsyncKeyState('S') & 0x8000) ? 1 : 0;
    rc_sim.kb.bit.A = (GetAsyncKeyState('A') & 0x8000) ? 1 : 0;
    rc_sim.kb.bit.D = (GetAsyncKeyState('D') & 0x8000) ? 1 : 0;
    rc_sim.kb.bit.SHIFT = (GetAsyncKeyState(VK_SHIFT) & 0x8000) ? 1 : 0;
    rc_sim.kb.bit.CTRL  = (GetAsyncKeyState(VK_CONTROL) & 0x8000) ? 1 : 0;
    rc_sim.kb.bit.G     = (GetAsyncKeyState('G') & 0x8000) ? 1 : 0; 
    
    if (rc_sim.kb.bit.SHIFT) rc_sim.kb.key_code |= (1<<4);
    if (rc_sim.kb.bit.CTRL)  rc_sim.kb.key_code |= (1<<5);
    if (rc_sim.kb.bit.G)     rc_sim.kb.key_code |= (1<<10);
    
    if (GetAsyncKeyState('C') & 0x8000) rc_sim.kb.key_code |= (1<<13);
    if (GetAsyncKeyState('R') & 0x8000) rc_sim.kb.key_code |= (1<<8);
    if (GetAsyncKeyState('F') & 0x8000) rc_sim.kb.key_code |= (1<<9);

    rc_sim.mouse.l = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) ? 1 : 0; 
    rc_sim.mouse.r = (GetAsyncKeyState(VK_RBUTTON) & 0x8000) ? 1 : 0;

    if (GetAsyncKeyState('1') & 0x8000) rc_sim.sw1 = RC_SW_UP;   
    if (GetAsyncKeyState('2') & 0x8000) rc_sim.sw1 = RC_SW_MID;  
    if (GetAsyncKeyState('3') & 0x8000) rc_sim.sw1 = RC_SW_DOWN; 

    if (GetAsyncKeyState('4') & 0x8000) rc_sim.sw2 = RC_SW_UP;   
    if (GetAsyncKeyState('5') & 0x8000) rc_sim.sw2 = RC_SW_MID;  
    if (GetAsyncKeyState('6') & 0x8000) rc_sim.sw2 = RC_SW_DOWN; 

    rc_sim.ch1 = 0; rc_sim.ch2 = 0; rc_sim.ch3 = 0; rc_sim.ch4 = 0; rc_sim.ch5 = 0;
    if (GetAsyncKeyState('Q') & 0x8000) rc_sim.ch3 = 660;  
    if (GetAsyncKeyState('E') & 0x8000) rc_sim.ch2 = 600; 
    
    // 模拟摇杆角落
    if (GetAsyncKeyState('Z') & 0x8000) { rc_sim.ch3 = -660; rc_sim.ch4 = -660; }
    if (GetAsyncKeyState('X') & 0x8000) { rc_sim.ch1 =  660; rc_sim.ch2 = -660; }
}

// ==========================================
// 打印仪表盘 
// ==========================================
void print_dashboard() {
    printf("\033[H"); 

    printf("========================================================\n");
    printf("     🚀 RoboMaster 状态机锁存模拟器 (按 ESC 退出) \n");
    printf("========================================================\n");
    printf(" 💡 [锁存测试教程]:\n");
    printf("  1. 按 [O] 键断开遥控器 (状态变为 OFFLINE)。\n");
    printf("  2. 按住 [Z] 键不放 (模拟左摇杆打到左下角)。\n");
    printf("  3. 再次按 [O] 键连接遥控器 (触发上电上升沿)。\n");
    printf("  4. 松开 [Z] 键，你会发现 [视觉模式(LD)] 已经被锁死了！\n");
    printf("--------------------------------------------------------\n");
    
    printf("\n=== [当前外设输入] ===\n");
    printf(" 遥控器状态 : %s\n", remote_online ? "🟢 ONLINE (已连接)" : "🔴 OFFLINE (已掉线)");
    printf(" 拨杆 : 左(SW1)=%s \t 右(SW2)=%s\n", sw_str(rc_sim.sw1), sw_str(rc_sim.sw2));
    
    char ld_str[10] = "  ", rd_str[10] = "  ";
    if (rc_fsm_check(RC_LEFT_LD)) strcpy(ld_str, "LD");
    if (rc_fsm_check(RC_RIGHT_RD)) strcpy(rd_str, "RD");
    printf(" 摇杆 : CH1=%-4d CH2=%-4d CH3=%-4d CH4=%-4d | 角落: [%s] [%s]\n", 
            rc_sim.ch1, rc_sim.ch2, rc_sim.ch3, rc_sim.ch4, ld_str, rd_str);
            
    char latch_str[100] = "";
    if (rc_fsm_check(RC_LEFT_LD)) strcat(latch_str, "[视觉开启(LD)] ");
    if (rc_fsm_check(RC_RIGHT_RD)) strcat(latch_str, "[禁止发射(RD)] ");
    printf(" 🔒 全局锁存状态 : %s\n", strlen(latch_str) > 0 ? latch_str : "无");

    printf("\n>>> [大脑决策 FSM] <<<\n");
    if (g_robot_ctx.output.top_mode == TOP_MODE_PROTECT) {
        printf(" 🚨 大模式   : %-30s\n", top_mode_str(g_robot_ctx.output.top_mode));
    } else {
        printf("    大模式   : %-30s\n", top_mode_str(g_robot_ctx.output.top_mode));
    }
    
    if (g_robot_ctx.output.chassis == CHASSIS_STOP && g_robot_ctx.output.top_mode != TOP_MODE_PROTECT) {
        printf(" ⚓ 底盘状态 : %-30s (架枪断电中!)\n", chassis_state_str(g_robot_ctx.output.chassis));
    } else {
        printf("    底盘状态 : %-30s\n", chassis_state_str(g_robot_ctx.output.chassis));
    }

    printf("    云台状态 : %-30s\n", gimbal_state_str(g_robot_ctx.output.gimbal));
    
    printf("\n=== [★ 发射器输出 ★] ===\n");
    if (g_robot_ctx.output.shoot == SHOOT_SERIES || g_robot_ctx.output.shoot == SHOOT_SINGLE) {
        printf(" 🔥 组合模式 : %-30s\n", shoot_state_str(g_robot_ctx.output.shoot));
    } else {
        printf("    组合模式 : %-30s\n", shoot_state_str(g_robot_ctx.output.shoot));
    }
    printf("========================================================\n");
}

int main() {
    printf("\033[2J"); 
    system("chcp 65001 > nul");
    memset(&rc_sim, 0, sizeof(rc_sim));
    rc_sim.sw1 = RC_SW_UP; 
    rc_sim.sw2 = RC_SW_UP; 
    robot_logic_init();

    while (1) {
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000) break;
        
        capture_physical_inputs();
        mock_rc_fsm_init(remote_online);
        
        if (remote_online) {
            robot_logic_update(&rc_sim);
        } else {
            robot_logic_update(NULL); 
        }
        
        print_dashboard();
        Sleep(20); 
    }

    printf("\n测试结束！\n");
    return 0;
}