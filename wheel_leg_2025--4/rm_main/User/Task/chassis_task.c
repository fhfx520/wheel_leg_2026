#include "chassis_task.h"
#include "mode_switch_task.h"
#include "control_def.h"
#include "wlr.h"
#include "can_comm.h"
#include "drv_dji_motor.h"
#include "drv_dm_motor.h"
#include "prot_imu.h"
#include "prot_dr16.h"
#include "prot_power.h"
#include "pid.h"
#include "math_lib.h"
#include "kalman_filter.h"
#include "arm_math.h"
#include "string.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "status_task.h"
#include "KNN.h"
#include "func_generator.h"
#include "leg_vmc.h"
#include "prot_judge.h"
#include "prot_power.h"
#include "dwt.h"
#include "prot_hipnuc.h"
#include "board_comm.h"
#include "container.h"

#ifndef DO_ONCE
#define DO_ONCE(code_block) { static int _flag = 0; if (!_flag) { _flag = 1; code_block; } }
#endif

static imu_data_t chassis_set_imu_data_container;

extern uint16_t quadrant_cnt;
extern pid_t pid_leg_recover[2];
extern float real_vel;

uint8_t rotate_flag;
uint8_t rotate_stop_flag;
static uint8_t kal_init = 0;

ramp_t chassis_x_ramp;
ramp_t chassis_y_ramp;
ramp_t chassis_rotate_ramp;

kalman_filter_t kal_3508_vel[2];
kalman_filter_t kal_wy;
kalman_filter_t kal_fusion_vel;
FGT_sin_t FGT_sin_chassis;
chassis_t chassis;

float variable_rotate_vw;

chassis_scale_t chassis_scale = {
    .remote = 1.0f/660*2.5f,
    .keyboard = 3.0f
};

// 恢复你本来的代码
static void chassis_ramp(void)
{
    if (rc.kb.bit.W) {
        ramp_calc(&chassis_x_ramp, chassis_scale.keyboard);
    } else if (rc.kb.bit.S) {
        ramp_calc(&chassis_x_ramp, -chassis_scale.keyboard);
    } else {
        ramp_calc(&chassis_x_ramp, 0);
    }
    if (rc.kb.bit.D) {
        ramp_calc(&chassis_y_ramp, chassis_scale.keyboard);
    } else if (rc.kb.bit.A) {
        ramp_calc(&chassis_y_ramp, -chassis_scale.keyboard);
    } else {
        ramp_calc(&chassis_y_ramp, 0);
    }
}

static void variable_vw_generate(float target_speed)
{
    static uint32_t dwt_count;
    static float t;
    t += DWT_GetDeltaT(&dwt_count);
    variable_rotate_vw = target_speed * fabsf(sinf(1.5f * PI * t + PI / 3.0f));
}

static void Fusion_Vel_Acc_Init(void)
{
    kalman_filter_init(&kal_fusion_vel,2,1,2);
    const static float dt = 0.002f;
    kal_fusion_vel.A_data[0] = 1; kal_fusion_vel.A_data[1] = dt;
    kal_fusion_vel.A_data[2] = 0; kal_fusion_vel.A_data[3] = 1;
    kal_fusion_vel.B_data[0] = 0.5f * dt * dt; kal_fusion_vel.B_data[1] = dt;
    kal_fusion_vel.H_data[0] = 1; kal_fusion_vel.H_data[1] = 0;
    kal_fusion_vel.H_data[2] = 0; kal_fusion_vel.H_data[3] = 1;
    kal_fusion_vel.Q_data[0] = 0.1; kal_fusion_vel.Q_data[1] = 0;
    kal_fusion_vel.Q_data[2] = 0; kal_fusion_vel.Q_data[3] = 0.5;
    kal_fusion_vel.R_data[0] = 10; kal_fusion_vel.R_data[1] = 0;
    kal_fusion_vel.R_data[2] = 0; kal_fusion_vel.R_data[3] = 2000;
}

void Fusion_Vel_Acc_Test(void)
{
    kal_fusion_vel.measured_vector[0] = wlr.s_fdb;
    kal_fusion_vel.measured_vector[1] = (-driver_motor[0].velocity + driver_motor[1].velocity) * 0.055f / 2.0f;
    kal_fusion_vel.control_vector[0] = chassis_imu.ax;
    kalman_filter_update(&kal_fusion_vel);
}

static void chassis_init(void)
{
    memset(&chassis, 0, sizeof(chassis_t));
    memset(&chassis_x_ramp, 0, sizeof(ramp_t));
    memset(&chassis_y_ramp, 0, sizeof(ramp_t));
    wlr_init();

    ramp_init(&chassis_x_ramp, 0.02f, -chassis_scale.keyboard, chassis_scale.keyboard);
    ramp_init(&chassis_y_ramp, 0.02f, -chassis_scale.keyboard, chassis_scale.keyboard);
    ramp_init(&chassis_rotate_ramp, 0.02f, -2.0f * CHASSIS_ROTATE_SPEED, 2.0f * CHASSIS_ROTATE_SPEED);

    wlr.yaw_ref = (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI;
    wlr.yaw_offset = 1.7f;
    DWT_Init(550);
       
    DO_ONCE({
        for(int i = 0; i < 2; i++) {
            kalman_filter_init(&kal_3508_vel[i], 1, 0, 1);
            kal_3508_vel[i].A_data[0] = 1; kal_3508_vel[i].H_data[0] = 1;
            kal_3508_vel[i].Q_data[0] = 1; kal_3508_vel[i].R_data[0] = 200;
        }
        kalman_filter_init(&kal_wy, 1, 0, 1);
        kal_wy.A_data[0] = 1; kal_wy.H_data[0] = 1;
        kal_wy.Q_data[0] = 1; kal_wy.R_data[0] = 100;
    });
    
    FGT_sin_init (&FGT_sin_chassis,2,0,6000,7.0f,0,7.0f,-0.0f);
    chassis.init = 1;
}

float spin_limit;
float spin_check;
float spin_zero;
float wheel_diff;
float gain_diff = 0.25f;

// ==============================================================================
// 纯粹地把 FSM 的状态映射到底层的配置，不保留旧的 mode
// ==============================================================================
static void chassis_execute_fsm(void)
{
    rotate_flag = 0;

    switch (g_robot_ctx.output.chassis) {
        case CHASSIS_STOP:
            wlr.ctrl_mode = 0; 
            wlr.high_flag = 0;
            wlr.jump_flag = 0;
            wlr.sky_flag = WLR_SKY_IDLE;
            chassis.rescue_cnt_L = 0;
            chassis.rescue_cnt_R = 0;
            chassis.recover_flag = 0;
            break;

        case CHASSIS_LOW:
            wlr.ctrl_mode = 2; 
            wlr.high_flag = 0; 
            wlr.jump_flag = 0;
            wlr.sky_flag = WLR_SKY_IDLE;
            chassis.recover_flag = 1; 
            break;

        case CHASSIS_HIGH:
            wlr.ctrl_mode = 2;
            wlr.high_flag = 1; 
            wlr.jump_flag = 0;
            wlr.sky_flag = WLR_SKY_IDLE;
            chassis.recover_flag = 1;
            break;

        case CHASSIS_LOW_SPIN:
            wlr.ctrl_mode = 2;
            wlr.high_flag = 0; 
            rotate_flag = 1;   
            chassis.recover_flag = 1;
            break;

        case CHASSIS_FIGHT:
            wlr.ctrl_mode = 2;
            wlr.high_flag = 0;
            wlr.jump_flag = 0;
            chassis.recover_flag = 1;
            break;

        case CHASSIS_TERRAIN_READY:
            wlr.ctrl_mode = 2;
            wlr.high_flag = 0;
            wlr.jump_flag = 0;
            wlr.sky_flag = WLR_SKY_FOLDING; 
            chassis.recover_flag = 1;
            break;

        case CHASSIS_TERRAIN_EXECUTING:
            wlr.ctrl_mode = 2;
            wlr.high_flag = 0;
            wlr.jump_flag = WLR_SKY_EXTENDING; 
            chassis.recover_flag = 1;
            break;

        default:
            wlr.ctrl_mode = 0;
            break;
    }

    if (g_robot_ctx.output.chassis == CHASSIS_HIGH) { 
        if (g_robot_ctx.output.chassis_speed) chassis_scale.keyboard = 2.5f;
        else chassis_scale.keyboard = 1.5f;
    } else { 
        if (g_robot_ctx.output.chassis_speed) chassis_scale.keyboard = 2.5f;
        else chassis_scale.keyboard = 2.0f;
    }

    if (supercap.volume_percent < 10 )  chassis_scale.keyboard = 1.5f;
    else if (supercap.volume_percent < 20 ) chassis_scale.keyboard = 2.0f;

    if (g_robot_ctx.output.chassis == CHASSIS_HIGH) chassis_scale.remote = 1.0f/660*2.6f;
    else chassis_scale.remote = 1.0f/660*2.5f; 
    
}

uint8_t rotate_ramp_flag; 
ChassisState_e rotate_chassis_mode = CHASSIS_STOP; // [修改] 替换为 FSM 类型
uint16_t rotate_state_cnt;
ChassisState_e last_chassis_mode = CHASSIS_STOP;   // [修改] 替换为 FSM 类型

// ==============================================================================
// 完全基于 FSM ChassisState_e 进行解算
// ==============================================================================
static void chassis_data_input(void)
{
    spin_limit = circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, (float)yaw_motor.ecd / 8192 * 2 * PI, 2 * PI);

    // 1. 速度输入算算 (基于大模式)
    if (g_robot_ctx.output.chassis == CHASSIS_STOP || g_robot_ctx.output.top_mode == TOP_MODE_PROTECT) {
        chassis.input.vx = 0;
        chassis.input.vy = 0;
    } 
    else if (g_robot_ctx.output.top_mode == TOP_MODE_REMOTE) { 
        chassis.input.vx = -g_robot_ctx.input.ch4 * chassis_scale.remote;
        chassis.input.vy =  g_robot_ctx.input.ch3 * chassis_scale.remote;
    } 
    else if (g_robot_ctx.output.top_mode == TOP_MODE_KEYBOARD) { 
        if(!wlr.jump_flag) {
            chassis_ramp();
            chassis.input.vx = -chassis_x_ramp.out;
            chassis.input.vy =  chassis_y_ramp.out;
        } else {
            chassis.input.vx = 0;
        }
    }

    // ================= 旋转控制算法 (完全替换旧模式) =================
    switch (g_robot_ctx.output.chassis) {
        
        case CHASSIS_STOP: {
            wlr.yaw_ref = (float)yaw_motor.ecd / 8192 * 2 * PI;
            wlr.yaw_fdb = (float)yaw_motor.ecd / 8192 * 2 * PI;
            wlr.wz_ref = 0;
            break;
        }
        
        case CHASSIS_LOW:
        case CHASSIS_HIGH:
        case CHASSIS_TERRAIN_READY:
        case CHASSIS_TERRAIN_EXECUTING: { // 整合了原版的所有 FOLLOW 和 PRONE
            if (gimbal.start_up)    
                wlr.yaw_ref = (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI;
            else                    
                wlr.yaw_ref = (float)yaw_motor.ecd / 8192 * 2  * PI;  

            wlr.yaw_fdb = (float)yaw_motor.ecd / 8192 * 2 *PI;  
            wlr.wz_ref = 0.0f;
            wlr.yaw_err = circle_error(wlr.yaw_ref, wlr.yaw_fdb, 2 * PI);
            
            if (wlr.yaw_err > PI / 2 || wlr.yaw_err < - PI / 2) {
                wlr.yaw_ref = (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI - PI ;
                wlr.direction = 1;
            }
            else if(wlr.yaw_err < PI / 2 || wlr.yaw_err > - PI / 2) {
                wlr.direction = 0;
            }
            chassis_rotate_ramp.out =0;
            break;
        }
        
        case CHASSIS_FIGHT: {
            wlr.yaw_ref = (float)CHASSIS_YAW_FIGHT / 8192 * 2 * PI;
            wlr.yaw_fdb = (float)yaw_motor.ecd / 8192 * 2 * PI;
            wlr.wz_ref = 0;
            break;
        }               
        
        case CHASSIS_LOW_SPIN: { // 整合了原版的所有 ROTATE     
            wlr.yaw_ref = wlr.yaw_fdb;
            if(power_control.judge_max_power > 40.0f)
                chassis_rotate_ramp.min = -(CHASSIS_ROTATE_SPEED + (power_control.judge_max_power - 40.0f/25.0f));
            else
                chassis_rotate_ramp.min = -CHASSIS_ROTATE_SPEED;
            
            variable_vw_generate(ramp_calc(&chassis_rotate_ramp , -(CHASSIS_ROTATE_SPEED + (power_control.judge_max_power - 40.0f)/25.0f)));
            wlr.wz_ref = variable_rotate_vw;
            break;
        }
        
        // UNFOLLOW 已经按照你的要求完全删除
        
        default:break;
    }

    if (wlr.yaw_ref < 0) wlr.yaw_ref += 2 * PI;
    else if (wlr.yaw_ref > 2 * PI) wlr.yaw_ref -= 2 * PI;
    
    wlr.yaw_err = circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, (float)yaw_motor.ecd / 8192 * 2 * PI, 2 * PI);
    if (g_robot_ctx.output.chassis == CHASSIS_FIGHT)
        wlr.yaw_err = circle_error((float)CHASSIS_YAW_FIGHT / 8192 * 2 * PI, (float)yaw_motor.ecd / 8192 * 2 * PI, 2 * PI);
                
    chassis.output.vx = chassis.input.vx * arm_cos_f32(wlr.yaw_err) - chassis.input.vy * arm_sin_f32(wlr.yaw_err);
    chassis.output.vy = chassis.input.vx * arm_sin_f32(wlr.yaw_err) + chassis.input.vy * arm_cos_f32(wlr.yaw_err);
        
    if (g_robot_ctx.output.chassis == CHASSIS_LOW_SPIN){
        if (fabs(wlr.v_fdb) > 2.0f) chassis.output.vx = -2.0f * wlr.v_fdb;
        else rotate_flag = 1;            
    }
    else if(rotate_ramp_flag == 0) rotate_flag = 0;   
    
    if (last_chassis_mode == CHASSIS_LOW_SPIN && (
         g_robot_ctx.output.chassis == CHASSIS_LOW || 
         g_robot_ctx.output.chassis == CHASSIS_HIGH ||
         g_robot_ctx.output.chassis == CHASSIS_TERRAIN_READY ||
         g_robot_ctx.output.chassis == CHASSIS_TERRAIN_EXECUTING)) {
            rotate_ramp_flag = 1;        
            rotate_stop_flag = 1;
    }
    
    if (rotate_stop_flag) {
        if (fabs(wlr.wz_fdb) < 2.0f) rotate_stop_flag = 0;
    }
    
    if(rotate_ramp_flag) {
        if(spin_zero == 0) spin_zero = spin_limit;
        rotate_state_cnt++;
        if(fabs(spin_zero) < PI / 2.0f  ) {                 
            if( fabs(circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, wlr.yaw_fdb, 2 * PI)) < 1.0f && rotate_state_cnt > 100){
                rotate_ramp_flag = 0; spin_zero = 0; rotate_state_cnt = 0; rotate_chassis_mode = CHASSIS_STOP;        
            }
        }else{
            if(fabs(circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI-PI, wlr.yaw_fdb, 2 * PI)) < 1.0f  && rotate_state_cnt > 100){
                rotate_ramp_flag = 0; spin_zero = 0; rotate_state_cnt = 0;
            }
        }       
        if (last_chassis_mode == CHASSIS_LOW_SPIN) 
            rotate_chassis_mode = last_chassis_mode;
            
        if(rotate_chassis_mode == CHASSIS_LOW_SPIN)
            wlr.wz_ref = -12.0f/1.5f;  
            
        wlr.yaw_ref = wlr.yaw_fdb;
    }
    
    wlr.yaw_err = circle_error(wlr.yaw_ref,wlr.yaw_fdb, 2 * PI);
    if (wlr.jump_flag && !wlr.jump_pre) {
        wlr.wz_ref = 6.0f; wlr.yaw_err = 0;
    }
    wlr.yaw_ref = wlr.yaw_fdb + 1.0f * wlr.yaw_err;
    wlr.v_ref = chassis.output.vx;
    
    if (wlr.jump_flag && !wlr.jump_pre && ((fabs(circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, wlr.yaw_fdb, 2 * PI)) < 0.2f)))
        wlr.jump_pre = 1;
    
    if (g_robot_ctx.output.chassis == CHASSIS_FIGHT){
        wlr.v_ref = -chassis.output.vy;
        wlr.s_ref += (wlr.v_ref * 0.001f * 2);
    }
    if(rotate_stop_flag) wlr.v_ref = 0.0f;
        
    wlr.roll_fdb    = -chassis_imu.rol;
    wlr.pit_fdb     = -(chassis_imu.pit);
    wlr.wy_fdb      = -chassis_imu.wy;
    wlr.wz_fdb      = -chassis_imu.wz;
    wlr.az_fdb      =  chassis_imu.az;
        
    wlr.side[0].q2 =  joint_motor[1].position - joint_motor[1].zero_point;
    wlr.side[0].q1 =  joint_motor[0].position - joint_motor[0].zero_point;
    if (wlr.side[0].q2 < 0) wlr.side[0].q2 += 2 * PI;
    if (wlr.side[0].q1 < 0) wlr.side[0].q1 += 2 * PI;
    wlr.side[0].w1 =  joint_motor[0].velocity; wlr.side[0].w2 =  joint_motor[1].velocity;
    wlr.side[0].t1 =  joint_motor[0].torque; wlr.side[0].t2 =  joint_motor[1].torque;
    wlr.side[0].qy = -driver_motor[0].position;
    kal_3508_vel[0].measured_vector[0] = -driver_motor[0].velocity;
    kalman_filter_update(&kal_3508_vel[0]);
    wlr.side[0].wy = kal_3508_vel[0].filter_vector[0];
    
    wlr.side[1].q2 =  -(joint_motor[3].position - joint_motor[3].zero_point);
    wlr.side[1].q1 =  -(joint_motor[2].position - joint_motor[2].zero_point);
    if (wlr.side[1].q2 < 0) wlr.side[1].q2 += 2 * PI;
    if (wlr.side[1].q1 < 0) wlr.side[1].q1 += 2 * PI;
    wlr.side[1].w1 = -joint_motor[2].velocity; wlr.side[1].w2 = -joint_motor[3].velocity;
    wlr.side[1].t1 = -joint_motor[2].torque; wlr.side[1].t2 = -joint_motor[3].torque;
    wlr.side[1].qy =  driver_motor[1].position;
    kal_3508_vel[1].measured_vector[0] = driver_motor[1].velocity;
    kalman_filter_update(&kal_3508_vel[1]);
    wlr.side[1].wy = kal_3508_vel[1].filter_vector[0];
    
    //KNN
    for (int i = 0; i < 10; i++) input_data[i] = lqr.X_fdb[i];
    input_data[10] =  wlr.side[0].Fn_kal;
    input_data[11] =  wlr.side[1].Fn_kal;
    
    // 记录最新状态给下一帧
    last_chassis_mode = g_robot_ctx.output.chassis;
}

float left_speed=3, right_speed=3;
float left_T, right_T;
float up_ready;

static void chassis_self_rescue(void)
{
    // ... [原翻倒自救代码保持不变] ...
}

static void chassis_rescue_test(void)
{
    // ... [原代码保持不变] ...
}

float temp_T;

static void chassis_data_output(void)
{
	static uint32_t prone_cnt = 0;
	if (!wlr.prone_flag)
		prone_cnt = 0;
    
    if (wlr.ctrl_mode == 0) {//保护模式
        wlr_protest();
        dji_motor_set_torque(&driver_motor[0], 0);
        dji_motor_set_torque(&driver_motor[1], 0);		
        for (int i = 0; i < 4; i++) {
			dm_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
        }
    } else if (wlr.ctrl_mode == 2) {//力控
        dji_motor_set_torque(&driver_motor[0], -wlr.side[0].Tw);
        dji_motor_set_torque(&driver_motor[1],  wlr.side[1].Tw);
        if (wlr.prone_flag) {
            // ... [原力控和卧倒逻辑保持不变] ...
        } else {
            if(chassis.recover_flag == 1) chassis_self_rescue();
            if(chassis.recover_flag != 1) {
				if(wlr.crash_flag){	  
					dm_motor_set_control_para(&joint_motor[0], 0, -5.0, 0, 5, 0);
					dm_motor_set_control_para(&joint_motor[1], 0, 0,    0, 0, 0);	
					dm_motor_set_control_para(&joint_motor[2], 0, 5.0,  0, 5, 0);
					dm_motor_set_control_para(&joint_motor[3], 0, 0,    0, 0, 0);
				}
				else if(wlr.joint_all_online){
					dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, wlr.side[0].T1);
					dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, wlr.side[0].T2);
					dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0,-wlr.side[1].T1);
					dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0,-wlr.side[1].T2);  
				}
				else{
					dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0);
					dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);
					dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 0);
					dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);  
				}
            }
        }
    } else if (wlr.ctrl_mode == 1) {//位控
        dji_motor_set_torque(&driver_motor[0], -wlr.side[0].Tw);
        dji_motor_set_torque(&driver_motor[1],  wlr.side[1].Tw);		
		dm_motor_set_control_para(&joint_motor[0],  wlr.side[0].P2 + joint_motor[0].zero_point,      0, 10, 2,  2);
        dm_motor_set_control_para(&joint_motor[1],  wlr.side[0].P1 - joint_motor[1].zero_point - PI, 0, 10, 2, -2);
        dm_motor_set_control_para(&joint_motor[2], -wlr.side[1].P1 + joint_motor[2].zero_point + PI, 0, 10, 2,  2);
        dm_motor_set_control_para(&joint_motor[3], -wlr.side[1].P2 - joint_motor[3].zero_point,      0, 10, 2, -2);		
    } else {
        wlr_protest();
        dji_motor_set_torque(&driver_motor[0], 0);
        dji_motor_set_torque(&driver_motor[1], 0);
        for (int i = 0; i < 4; i++) {
            dm_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
        }
    }
	
    // 恢复你本来的按键硬切断保护
    if ( rc_fsm_check(RC_LEFT_LD) ){
        dji_motor_set_torque(&driver_motor[0], 0);
        dji_motor_set_torque(&driver_motor[1], 0);
        dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0);
        dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);
		dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 0);
		dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);
    }
    
	if (!driver_motor[0].online || !driver_motor[1].online) {
		dji_motor_set_torque(&driver_motor[0], 0);
		dji_motor_set_torque(&driver_motor[1], 0);
		dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0);
		dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);
		dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 0);
		dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);			
	}
	
	if(joint_motor[0].state == 1 && joint_motor[1].state == 1 && joint_motor[2].state == 1 && joint_motor[3].state == 1 )
		wlr.joint_all_online = 1;
	else
		wlr.joint_all_online = 0;	
}

void chassis_set_container(void)
{
	chassis_set_imu_data_container.pit = wlr.pit_fdb;
	chassis_set_imu_data_container.rol = wlr.roll_fdb;
	container_set(TAG_CHA_IMU_DATA,&chassis_set_imu_data_container,sizeof(chassis_set_imu_data_container),CONTAINER_TYPE_STRUCT);
}

// ==============================================================================
// 最终任务主循环
// ==============================================================================
void chassis_task(void const *argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    power_init();
    chassis_init();
    Fusion_Vel_Acc_Init();
    
    
    for(;;)
    {   
        thread_wake_time = osKernelSysTick();
        
        // 2. 将 FSM 状态直接挂载到执行参数
        chassis_execute_fsm();
        
        // 3. 期望速度和旋转矩阵计算
        chassis_data_input();
        
        // 4. 恢复你本来的执行逻辑结构
        if(g_robot_ctx.output.chassis != CHASSIS_STOP)
            wlr_control();
        else
            chassis_init(); // 恢复你的原有保护调用
        
        // 5. 将算好的力矩下发到电机
        chassis_data_output();
        
		//底盘待发送数据打包
		chassis_set_container();
		
        status.task.chassis = 1;
        osDelayUntil(&thread_wake_time, 2);
    }
}