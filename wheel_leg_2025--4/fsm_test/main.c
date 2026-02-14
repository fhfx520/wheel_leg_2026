#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <windows.h> 
#include "robot_logic.h"

// ==========================================
// 1. 底层宏定义复刻 (供模拟器自身测试使用)
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

RC_Ctrl_t rc_sim;
uint8_t remote_online = 1;      
uint8_t mock_init_status = 0;   

// 手搓模拟 prot_dr16.c 里的上电锁存器
void mock_rc_fsm_init(uint8_t trig_flag) {
    static uint8_t last_trig_flag = 0;
    static uint8_t state = 0;
    
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
    if (!trig_flag && last_trig_flag) mock_init_status = 0;
}

uint8_t rc_fsm_check(uint8_t target_status) {
    return (mock_init_status & target_status) ? 1 : 0;
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
    } else { key_o_pressed = 0; }

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
    if (GetAsyncKeyState('T') & 0x8000) rc_sim.ch5 = 660; 
    
    // 模拟摇杆角落
    if (GetAsyncKeyState('Z') & 0x8000) { rc_sim.ch3 = -660; rc_sim.ch4 = -660; }
    if (GetAsyncKeyState('X') & 0x8000) { rc_sim.ch1 =  660; rc_sim.ch2 = -660; }
}

// ==========================================
// 打印仪表盘与各Task内部变量透视
// ==========================================
void print_dashboard() {
    printf("\033[H"); // 光标复位

    // --- 1. 帮助说明 ---
    printf("=================================================================\n");
    printf("     🚀 RoboMaster 状态机及底层标志位 综合透视模拟器 (ESC退出) \n");
    printf("=================================================================\n");
    printf(" [按键说明 - 遥控器模拟]\n");
    printf("  O 键 : 断开/连接 遥控器 (测试锁存必用)\n");
    printf("  1/2/3: 左拨杆(SW1) 保护 / 遥控 / 键鼠\n");
    printf("  4/5/6: 右拨杆(SW2) 低腿 / 高腿 / 地形\n");
    printf("  Q 键 : 猛推左摇杆(CH3) -> 触发小陀螺\n");
    printf("  E 键 : 猛推右摇杆(CH2) -> 触发跨越地形\n");
    printf("  T 键 : 拨动拨轮(CH5)   -> 触发开火\n");
    printf("  Z 键 : 左摇杆左下 (测试架枪断电 / 视觉锁存)\n");
    printf("  X 键 : 右摇杆右下 (双杆急停 / 禁止发射)\n");
    printf(" [按键说明 - 键鼠模拟]\n");
    printf("  WASD : 移动   Shift : 加速   Ctrl : 长按跨越地形\n");
    printf("  C/R/F: 切腿长(C) / 小陀螺(R) / 迎敌模式(F)\n");
    printf("  G 键 : 高频射击   鼠标左/右: 开火 / 自瞄\n");
    printf("-----------------------------------------------------------------\n");
    
    // --- 2. 输入状态 ---
    printf("\n>>> [输入层] 当前外设与锁存状态 <<<\n");
    printf(" 遥控器 : %s\n", remote_online ? "🟢 ONLINE" : "🔴 OFFLINE");
    printf(" 拨杆   : 左SW1 = %d, 右SW2 = %d\n", rc_sim.sw1, rc_sim.sw2);
    printf(" 摇杆   : CH1=%-4d CH2=%-4d CH3=%-4d CH4=%-4d CH5=%-4d\n", rc_sim.ch1, rc_sim.ch2, rc_sim.ch3, rc_sim.ch4, rc_sim.ch5);
    char latch_str[100] = "";
    if (rc_fsm_check(RC_LEFT_LD)) strcat(latch_str, "[视觉LD] ");
    if (rc_fsm_check(RC_RIGHT_RD)) strcat(latch_str, "[禁发RD] ");
    printf(" 锁存   : %s\n", strlen(latch_str) > 0 ? latch_str : "无");

    // --- 3. FSM 大脑决策输出 ---
    printf("\n>>> [大脑层] Robot_Logic 状态机输出 <<<\n");
    char* top_str = "UNKNOWN";
    if (g_robot_ctx.output.top_mode == TOP_MODE_PROTECT) top_str = "PROTECT (保护)";
    else if (g_robot_ctx.output.top_mode == TOP_MODE_REMOTE) top_str = "REMOTE (遥控)";
    else if (g_robot_ctx.output.top_mode == TOP_MODE_KEYBOARD) top_str = "KEYBOARD (键鼠)";
    printf(" [Top Mode] %s\n", top_str);

    // --- 4. 模拟 Chassis Task 标志位解析 ---
    printf("\n=== [执行层] Chassis Task (底盘任务) 标志位 ===\n");
    int wlr_ctrl_mode = 2, wlr_high = 0, rotate_flag = 0, recover_flag = 1;
    char wlr_jump[100] = "0", wlr_sky[20] = "WLR_SKY_IDLE";
    float chassis_spd = (g_robot_ctx.output.chassis_speed) ? 2.5f : 2.0f;
    
    switch (g_robot_ctx.output.chassis) {
        case CHASSIS_STOP: 
            wlr_ctrl_mode = 0; recover_flag = 0; chassis_spd = 0; 
            printf(" 状态映射 : CHASSIS_STOP -> 失去动力锁定\n"); break;
        case CHASSIS_LOW: 
            printf(" 状态映射 : CHASSIS_LOW -> 普通低腿\n"); break;
        case CHASSIS_HIGH: 
            wlr_high = 1; chassis_spd = (g_robot_ctx.output.chassis_speed) ? 2.5f : 1.5f;
            printf(" 状态映射 : CHASSIS_HIGH -> 普通高腿\n"); break;
        case CHASSIS_LOW_SPIN: 
            rotate_flag = 1; 
            printf(" 状态映射 : CHASSIS_LOW_SPIN -> 开启小陀螺\n"); break;
        case CHASSIS_FIGHT: 
            printf(" 状态映射 : CHASSIS_FIGHT -> 迎敌底盘算法\n"); break;
        case CHASSIS_TERRAIN_READY: 
            strcpy(wlr_sky, "WLR_SKY_FOLDING"); 
            printf(" 状态映射 : CHASSIS_TERRAIN_READY -> 跨越准备\n"); break;
        case CHASSIS_TERRAIN_EXECUTING: 
            strcpy(wlr_jump, "WLR_SKY_EXTENDING"); 
            printf(" 状态映射 : CHASSIS_TERRAIN_EXECUTING -> 跨越跳跃执行中！\n"); break;
    }
    printf("  ├─ wlr.ctrl_mode  = %d %s\n", wlr_ctrl_mode, wlr_ctrl_mode==0?"(无力)":"(力控)");
    printf("  ├─ wlr.high_flag  = %d\n", wlr_high);
    printf("  ├─ wlr.jump_flag  = %s\n", wlr_jump);
    printf("  ├─ wlr.sky_flag   = %s\n", wlr_sky);
    printf("  ├─ rotate_flag    = %d %s\n", rotate_flag, rotate_flag?"(正在旋转)":"");
    printf("  ├─ recover_flag   = %d %s\n", recover_flag, recover_flag?"(允许自起)":"(禁止自起)");
    printf("  └─ 速度限制上限   = %.1f\n", chassis_spd);

    // --- 5. 模拟 Shoot Task 标志位解析 ---
    printf("\n=== [执行层] Shoot Task (发射任务) 标志位 ===\n");
    char fric_mode[100] = "STOP", trig_mode[100] = "STOP";
    int trigger_period = 0; // 模拟周期
    
    // 射频映射
    if (g_robot_ctx.output.top_mode == TOP_MODE_KEYBOARD && rc_sim.mouse.r) 
        trigger_period = 2; // TRIGGER_PERIOD2
    else 
        trigger_period = 1; // TRIGGER_PERIOD

    switch(g_robot_ctx.output.shoot) {
        case SHOOT_PROTECT: strcpy(fric_mode, "PROTECT"); strcpy(trig_mode, "PROTECT"); break;
        case SHOOT_STOP:    strcpy(fric_mode, "STOP");    strcpy(trig_mode, "STOP"); break;
        case SHOOT_READY:   strcpy(fric_mode, "RUN");     strcpy(trig_mode, "STOP"); break;
        case SHOOT_SINGLE:  strcpy(fric_mode, "RUN");     strcpy(trig_mode, "SINGLE"); break;
        case SHOOT_SERIES:  strcpy(fric_mode, "RUN");     strcpy(trig_mode, "SERIES"); break;
    }
    printf("  ├─ fric_mode      = %-10s %s\n", fric_mode, strcmp(fric_mode,"RUN")==0?"(摩擦轮转动)":"(摩擦轮关闭)");
    printf("  ├─ trigger_mode   = %-10s %s\n", trig_mode, strcmp(trig_mode,"SERIES")==0?"(疯狂拨盘)":"");
    printf("  └─ trigger_period = %-10d %s\n", trigger_period, trigger_period==2?"(高射频模式)":"(标准射频)");

    // --- 6. 模拟 Gimbal Task 标志位解析 ---
    printf("\n=== [执行层] Gimbal Task (云台任务) 标志位 ===\n");
    char aiming[100] = "0 (不自瞄)";
    if (g_robot_ctx.output.gimbal == GIMBAL_AUTO_AIM) strcpy(aiming, "1 (视觉自瞄介入)");
    
    char alpha_spd[100] = "0 (无底盘补偿)";
    if (g_robot_ctx.output.chassis == CHASSIS_LOW_SPIN) {
        strcpy(alpha_spd, "-chassis_wz_fdb (开启小陀螺反向补偿)");
    }
    printf("  ├─ aiming_mode    = %s\n", aiming);
    printf("  └─ alpha_speed    = %s\n", alpha_spd);

    printf("=================================================================\n");
}

int main() {
    // 强制终端输出 UTF-8 避免中文乱码
    system("chcp 65001 > nul");
    printf("\033[2J"); // 清屏

    memset(&rc_sim, 0, sizeof(rc_sim));
    rc_sim.sw1 = RC_SW_UP; 
    rc_sim.sw2 = RC_SW_UP; 
    robot_logic_init();

    while (1) {
        // 监控 ESC 键退出
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000) break;
        
        // 1. 采集物理输入
        capture_physical_inputs();
        
        // 2. 模拟底层的掉线检测和上电锁存
        mock_rc_fsm_init(remote_online);
        
        // 3. 将数据喂给大脑
        if (remote_online) {
            robot_logic_update(&rc_sim);
        } else {
            robot_logic_update(NULL); 
        }
        
        // 4. 打印全局仪表盘
        print_dashboard();
        
        // 5. 模拟 STM32 Task 的执行周期 (50Hz)
        Sleep(20); 
    }

    printf("\n程序安全退出。\n");
    return 0;
}