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


//未使用
#define JOINT_MOTOR_RESET_TORQUE 2.0f
#define JOINT_MOTOR_RESET_ERROR 0.005f

extern uint16_t quadrant_cnt;
extern pid_t pid_leg_recover[2];
extern ramp_t recover_ramp;
//用于电机输入角度到q2的映�?
const float Legtemp1 = 0.115f;
const float Legtemp2 = 0.135f;

float rescue_speed;
float test_t[2];
float ro_temp;
uint8_t rotate_flag;
uint8_t rppppp_flag = 0;
//uint8_t last_recover_flag;
//uint8_t recover_flag;
//uint32_t rescue_cnt_L;
//uint32_t rescue_cnt_R;
//uint32_t rescue_cnt_T;
//float dm_p[2];
ramp_t chassis_x_ramp;
ramp_t chassis_y_ramp;
ramp_t chassis_rotate_ramp;

kalman_filter_t kal_3508_vel[2];
kalman_filter_t kal_wy;
FGT_sin_t FGT_sin_chassis;
chassis_t chassis;

chassis_scale_t chassis_scale = {
    .remote = 1.0f/660*3.0f,
    .keyboard = 3.0f
};

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

static void chassis_init()
{
    memset(&chassis, 0, sizeof(chassis_t));
    memset(&chassis_x_ramp, 0, sizeof(ramp_t));
    memset(&chassis_y_ramp, 0, sizeof(ramp_t));
    wlr_init();
    ramp_init(&chassis_x_ramp, 0.02f, -chassis_scale.keyboard, chassis_scale.keyboard);//0.02 0.1s达到最�?
    ramp_init(&chassis_y_ramp, 0.02f, -chassis_scale.keyboard, chassis_scale.keyboard);
	ramp_init(&chassis_rotate_ramp, 0.02f, -2.0f * CHASSIS_ROTATE_SPEED,2.0f * CHASSIS_ROTATE_SPEED);

    wlr.yaw_ref = (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI;
    wlr.yaw_offset = 1.7f;
    
    for(int i = 0; i < 2; i++) {
        kalman_filter_init(&kal_3508_vel[i], 1, 0, 1);
        kal_3508_vel[i].A_data[0] = 1;
        kal_3508_vel[i].H_data[0] = 1;
        kal_3508_vel[i].Q_data[0] = 1;
        kal_3508_vel[i].R_data[0] = 200;
    }
    
    kalman_filter_init(&kal_wy, 1, 0, 1);
    kal_wy.A_data[0] = 1;
    kal_wy.H_data[0] = 1;
    kal_wy.Q_data[0] = 1;
    kal_wy.R_data[0] = 100;
    FGT_sin_init (&FGT_sin_chassis,2,0,6000,7.0f,0,7.0f,-0.0f);
    chassis.init = 1;
}

float spin_limit;
float spin_check;
float spin_zero;
static void chassis_mode_switch(void)
{
    /* 系统历史状态 */
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;

    /* 按键扫描 */
    key_scan(KEY_CHASSIS_ROTATE);
    key_scan(KEY_CHASSIS_HEIGHT);
    key_scan(KEY_CHASSIS_HEIGHT2);
//  key_scan(KEY_CHASSIS_FIGHT);
	key_scan(KEY_CHASSIS_FIGHT_A);
	key_scan(KEY_CHASSIS_FIGHT_D);
	
//  key_scan(KEY_CHASSIS_UNFOLLOW);
    key_scan(KEY_CHASSIS_POWER);
//  key_scan(KEY_CHASSIS_PRONE);

    /* 底盘状态切换 */
    spin_limit = circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, (float)yaw_motor.ecd / 8192 * 2 * PI, 2 * PI);
    switch (ctrl_mode) {
    case PROTECT_MODE: { //能量模式和保护模式下，底盘行为相同
        chassis.mode = CHASSIS_MODE_PROTECT;
        wlr.prone_flag = 0;
        chassis.rescue_cnt_L = 0;
        chassis.rescue_cnt_R = 0;
        chassis.recover_flag = 0;
        chassis.rescue_inter_flag = 0;
        break;
    }
    case REMOTER_MODE: {
        if (last_ctrl_mode == PROTECT_MODE )
            chassis.recover_flag = 1;		//help!!! 这个变量什么意思
        if (last_ctrl_mode != REMOTER_MODE || chassis.mode == CHASSIS_MODE_PROTECT) { //从非遥控模式 或 保护模式切入遥控模式，底盘切为遥控跟随模式
            chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
        }
		
        /* 底盘小陀螺模式 */
        static uint8_t spin_flag; 					//单次触发使能标志
        if (spin_flag == 0 && ABS(rc.ch3) < 10) { 	//使能底盘模式切换
            spin_flag = 1;
        }
				
        if (rc.ch3 == 660 && spin_flag == 1 && wlr.high_flag == 0) {
            if( (chassis.mode == CHASSIS_MODE_REMOTER_ROTATE1 || chassis.mode == CHASSIS_MODE_REMOTER_ROTATE2)	
			/*&& 0 < spin_limit && spin_limit < 1.7f*/  )				
            {//---------------------------为了测试注释 && 0 < spin_limit && spin_limit < 1.7f  
                chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
                spin_flag = 0;
            } else if (chassis.mode == CHASSIS_MODE_REMOTER_FOLLOW) {
                chassis.mode = CHASSIS_MODE_REMOTER_ROTATE1;
                spin_flag = 0;
            }
        }
        if (rc.ch3 == -660 && spin_flag == 1 && wlr.high_flag == 0) {
            if( (chassis.mode == CHASSIS_MODE_REMOTER_ROTATE1 || chassis.mode == CHASSIS_MODE_REMOTER_ROTATE2)
			/*	&& -3.0f < spin_limit && spin_limit < -1.5f */ )				
			{// && -3.0f < spin_limit && spin_limit < -1.5f) {
                chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
                spin_flag = 0;
            } else if (chassis.mode == CHASSIS_MODE_REMOTER_FOLLOW) {
                chassis.mode = CHASSIS_MODE_REMOTER_ROTATE2;
                spin_flag = 0;
            }
        }
        spin_check = spin_flag;				

        /* 遥控器注释底盘 */
        if (rc_fsm_check(RC_LEFT_RU) || rc_fsm_check(RC_RIGHT_RU)) { //遥控器注释底盘
            chassis.mode = CHASSIS_MODE_PROTECT;
        }
        break;
    }
    case KEYBOARD_MODE: { //键盘模式下(跟随，陀螺，迎敌三种模式相互切换),(跟随与补给模式相互切换)  KB_R
        /* 底盘模式切换 */
        if (last_ctrl_mode == PROTECT_MODE )
            chassis.recover_flag = 1;
		
        switch (chassis.mode) {
        case CHASSIS_MODE_KEYBOARD_FOLLOW: { //底盘跟随模式下
			if(kb_status[KEY_CHASSIS_PRONE]) { //趴倒模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_PRONE;
            } else if (kb_status[KEY_CHASSIS_ROTATE]) { //进入�?盘陀螺模�?
                chassis.mode = CHASSIS_MODE_KEYBOARD_ROTATE;
            } else if (kb_status[KEY_CHASSIS_FIGHT] || kb_status[KEY_CHASSIS_FIGHT_A] || kb_status[KEY_CHASSIS_FIGHT_D] ) { //进入迎敌模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;							
			} 
            break;
        }
        case CHASSIS_MODE_KEYBOARD_ROTATE: { //键盘陀螺模式下
            if ((!kb_status[KEY_CHASSIS_ROTATE]) && -1.7f < spin_limit && spin_limit < 0) { //恢复跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            } else if (kb_status[KEY_CHASSIS_FIGHT]) { //进入迎敌模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;
            }
			//2025.3.23
			if (fabs(chassis_imu.pit) > 0.5f)
				chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            break;
        }
        case CHASSIS_MODE_KEYBOARD_FIGHT: { //键盘迎敌模式下
            if (rc.kb.bit.W || rc.kb.bit.S) { //恢复跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            } else if (kb_status[KEY_CHASSIS_ROTATE]) { //进入小陀螺模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_ROTATE;
            } 
            break;
        }
        case CHASSIS_MODE_KEYBOARD_PRONE: {
            if(!kb_status[KEY_CHASSIS_PRONE]) { //趴倒模式
                wlr.prone_flag = 0;
                wlr.high_flag = 0;
                chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            }
            break;
        }		
        default: {
            chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            break;
        }
        }
        break;
    }
    default: break;
    }
    /* 系统历史状态更�? */
    last_ctrl_mode = ctrl_mode;
    /* 状态标志�?�位 */
    if (chassis.mode != CHASSIS_MODE_KEYBOARD_ROTATE)
        key_status_clear(KEY_CHASSIS_ROTATE);
    if (chassis.mode != CHASSIS_MODE_KEYBOARD_FIGHT){
//      key_status_clear(KEY_CHASSIS_FIGHT);
		key_status_clear(KEY_CHASSIS_FIGHT_A);	
		key_status_clear(KEY_CHASSIS_FIGHT_D);	
	}
    if(chassis.mode != CHASSIS_MODE_KEYBOARD_PRONE)
        key_status_clear(KEY_CHASSIS_PRONE);	
}

//help!!! 这四个变量什么意思
uint8_t rotate_ramp_flag; 
chassis_mode_e rotate_chassis_mode = CHASSIS_MODE_PROTECT;
uint16_t rotate_state_cnt;
chassis_mode_e last_chassis_mode = CHASSIS_MODE_PROTECT;

static void chassis_data_input(void)
{
    static uint16_t slow_cnt;
    //速度输入 模式输入
    switch (chassis.mode) {
        case CHASSIS_MODE_PROTECT: {  //底盘保护模式
            chassis.input.vx = 0;
            chassis.input.vy = 0;
            wlr.ctrl_mode = 0;
			wlr.high_flag = 0;
            key_status_clear(KEY_CHASSIS_HEIGHT);
            break;
        }
        case CHASSIS_MODE_REMOTER_FOLLOW:  //底盘遥控跟随模式 底盘遥控陀螺模式
        case CHASSIS_MODE_REMOTER_ROTATE1:
        case CHASSIS_MODE_REMOTER_ROTATE2: {
			if( wlr.high_flag == 1 )
				chassis_scale.remote =	1.0f/660*2.5f;
			else
				chassis_scale.remote =	1.0f/660*2.5f;	
			
            chassis.input.vx = -rc.ch4 * chassis_scale.remote;
            chassis.input.vy = rc.ch3 * chassis_scale.remote;
			wlr.ctrl_mode = 2;
            
           //高度模式
           if (wlr.ctrl_mode == 2) {  //轮腿模式下才可控制腿长
			   if (rc.sw2 == RC_UP) {
                   wlr.jump_flag = 0;
                   wlr.high_flag = 0;	//短腿
                   wlr.power_flag = 1;	//未使用
			   } 
               else if (rc.sw2 == RC_MI ) {
				   wlr.jump_flag = 0;
                   wlr.high_flag = 1;		//中腿
                   wlr.power_flag = 1;
                   chassis.rescue_test = 0;
				   wlr.sky_flag = 0;
				   wlr.sky_over = 0;
				   wlr.jump2_over = 0;
			   } 
//               } else if (rc.sw2 == RC_DN && wlr.jump_flag == 0 && rc_fsm_check(RC_RIGHT_LU))  
//                   wlr.high_flag = 2;
//                 } else if (rc.sw2 == RC_DN && 0)//发射器
//                      chassis.rescue_test = 1;
//               else if (rc.sw2 == RC_DN && wlr.jump_flag == 0 && wlr.jump2_over == 0)//&& !rc_fsm_check(RC_RIGHT_LU))
//                    wlr.jump_flag = 1;
//               else 
//                    wlr.high_flag = 0; 
			   else if (rc.sw2 == RC_DN && wlr.sky_flag == 0 && wlr.sky_over == 0 )
					wlr.sky_flag = 1; 
			   
			   
			   if (rotate_flag == 1) {//小陀螺不能改变腿长
					wlr.jump_flag = 0;
					wlr.high_flag = 0;
					wlr.power_flag = 1;
				    chassis.rescue_test = 0;
			   }
           }
           break;
        }
        case CHASSIS_MODE_KEYBOARD_FOLLOW:
			wlr.ctrl_mode = 2; 
            if (KEY_PRESS_JUMP && wlr.jump_flag == 0){
				wlr.jump_flag = 1;//联盟赛
				wlr.high_flag = 0;
			}else if (!KEY_PRESS_JUMP &&  wlr.jump_flag != 0){
                wlr.jump_flag = 0;
				wlr.high_flag = 0;
                wlr.power_flag = 1;
                chassis.rescue_test = 0;
				wlr.sky_flag = 0;
				wlr.sky_over = 0;
				wlr.jump2_over = 0;	
			}
			if (KEY_PRESS_POWER)
				double_cnt = 3000;
			else
				double_cnt = 0;
						
        case CHASSIS_MODE_KEYBOARD_ROTATE:
        case CHASSIS_MODE_KEYBOARD_FIGHT:					
        case CHASSIS_MODE_KEYBOARD_UNFOLLOW:
        case CHASSIS_MODE_KEYBOARD_PRONE: {
			
/******************************************** 速度选择 ********************************************/
//            if (chassis.mode == CHASSIS_MODE_KEYBOARD_FIGHT ||
//				  chassis.mode == CHASSIS_MODE_KEYBOARD_ROTATE ||
//                chassis.mode == CHASSIS_MODE_KEYBOARD_UNFOLLOW ||
//                wlr.high_flag == 2) {
//                wlr.power_flag = 0;
//                chassis_scale.keyboard = 2.5f;  //迎敌模式					
//							if( wlr.high_flag == 1 )
//						 chassis_scale.keyboard = 2.5f; 							
//            }
			if(wlr.high_flag == 2) {
                wlr.power_flag = 0;
                chassis_scale.keyboard = 1.0f;  //迎敌模式
			}
			else if(wlr.high_flag == 1){
				if(KEY_PRESS_POWER && !wlr.jump_flag) {
					chassis_scale.keyboard = 2.5f;  //飞坡模式下
				} else {
					chassis_scale.keyboard = 1.5f;  //普通模式下----------------------2025.3.4 改2.5
				}
			}
			else if(wlr.high_flag == 0){
				if(KEY_PRESS_POWER && !wlr.jump_flag) 
					chassis_scale.keyboard = 2.5f;  //飞坡模式下
				else 
					chassis_scale.keyboard = 2.0f;  //通模式下----------------------2025.3.4 改2.5
				}
			if (supercap.volume_percent < 10 )  
				chassis_scale.keyboard = 1.5f;
			else if (supercap.volume_percent < 20 )
				chassis_scale.keyboard = 2.0f;
			
/******************************************** 速度输入 ********************************************/
			if(!wlr.jump_flag) {
				chassis_ramp();
				chassis.input.vx = -chassis_x_ramp.out;
				chassis.input.vy = chassis_y_ramp.out;
								
				if(wlr.jump_flag)
					chassis.input.vx = 0;
								
				//控制模式
				wlr.ctrl_mode = 2; 
				if (chassis.mode == CHASSIS_MODE_KEYBOARD_PRONE) {
					wlr.prone_flag = 1;
				}
				
/******************************************** 高度选择 ********************************************/
				if (wlr.high_flag == 0) {
					if (kb_status[KEY_CHASSIS_HEIGHT] == KEY_RUN && 
						chassis.mode != CHASSIS_MODE_KEYBOARD_PRONE) {
						wlr.high_flag = 1;
					}
					if (kb_status[KEY_CHASSIS_HEIGHT2] == KEY_RUN && 
						chassis.mode != CHASSIS_MODE_KEYBOARD_PRONE) {
						wlr.high_flag = 2;
					}
				} else if (wlr.high_flag == 1) {
					if (kb_status[KEY_CHASSIS_HEIGHT2] == KEY_RUN && 
						chassis.mode != CHASSIS_MODE_KEYBOARD_PRONE) {
						wlr.high_flag = 2;
						key_status_clear(KEY_CHASSIS_HEIGHT);
					} else if (kb_status[KEY_CHASSIS_HEIGHT] != KEY_RUN || 
						chassis.mode == CHASSIS_MODE_KEYBOARD_PRONE) {
						wlr.high_flag = 0;
						key_status_clear(KEY_CHASSIS_HEIGHT);
					}
				} else if (wlr.high_flag == 2) {
					chassis_scale.keyboard = 1.0f;
					wlr.power_flag = 0;
					if (kb_status[KEY_CHASSIS_HEIGHT] == KEY_RUN && 
						chassis.mode != CHASSIS_MODE_KEYBOARD_PRONE) {
						wlr.high_flag = 1;
						key_status_clear(KEY_CHASSIS_HEIGHT2);
					} else if (kb_status[KEY_CHASSIS_HEIGHT2] != KEY_RUN || 
						chassis.mode == CHASSIS_MODE_KEYBOARD_PRONE) {
						wlr.high_flag = 0;
						key_status_clear(KEY_CHASSIS_HEIGHT2);
					}
				} else {
					wlr.high_flag = 0;
					key_status_clear(KEY_CHASSIS_HEIGHT);
					key_status_clear(KEY_CHASSIS_HEIGHT2);
				}
				if (rotate_flag == 1) {//小陀螺不能改变腿长
					wlr.jump_flag = 0;
					wlr.high_flag = 0;
					wlr.power_flag = 1;
					chassis.rescue_test = 0;							
				}	
			}
            break;
        }
        default:break;
    }
	
/******************************************** 旋转数据输入 ********************************************/
    switch (chassis.mode) {
        case CHASSIS_MODE_PROTECT: {
            wlr.yaw_ref = (float)yaw_motor.ecd / 8192 * 2 * PI;
            wlr.yaw_fdb = (float)yaw_motor.ecd / 8192 * 2 * PI;
            wlr.wz_ref = 0;
            break;
        }
        case CHASSIS_MODE_REMOTER_FOLLOW:
        case CHASSIS_MODE_KEYBOARD_FOLLOW:
        case CHASSIS_MODE_KEYBOARD_PRONE: {
            if (gimbal.start_up)	//完成起身，前方灯条
                wlr.yaw_ref = (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI;
            else					//起身未完成，目标值等于反馈值
                wlr.yaw_ref = (float)yaw_motor.ecd / 8192 * 2  * PI;     
						
            wlr.yaw_fdb = (float)yaw_motor.ecd / 8192 * 2 *PI;	//-chassis_imu.yaw;
            wlr.wz_ref = 0;
						
            //此yaw_err用于底盘前后都可跟随 算哪边最短路径
            wlr.yaw_err = circle_error(wlr.yaw_ref, wlr.yaw_fdb, 2 * PI);
			
            if ( wlr.yaw_err > PI / 2 || wlr.yaw_err < - PI / 2) {
                wlr.yaw_ref = (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI - PI ;
            }
			chassis_rotate_ramp.out =0;//斜坡清零
            break;
        }
        case CHASSIS_MODE_KEYBOARD_FIGHT: {
            wlr.yaw_ref = (float)CHASSIS_YAW_FIGHT / 8192 * 2 * PI;
            wlr.yaw_fdb = (float)yaw_motor.ecd / 8192 * 2 * PI;
            wlr.wz_ref = 0;
            break;
        }				
        case CHASSIS_MODE_REMOTER_ROTATE1:
        case CHASSIS_MODE_REMOTER_ROTATE2:
        case CHASSIS_MODE_KEYBOARD_ROTATE: {						
            if (last_chassis_mode != CHASSIS_MODE_REMOTER_ROTATE1 && \
				last_chassis_mode != CHASSIS_MODE_REMOTER_ROTATE2 && \
				last_chassis_mode != CHASSIS_MODE_KEYBOARD_ROTATE)
				wlr.yaw_ref = -chassis_imu.yaw;
			
			if (chassis.mode == CHASSIS_MODE_REMOTER_ROTATE2 || chassis.mode == CHASSIS_MODE_KEYBOARD_ROTATE ) {//检录进
				wlr.yaw_ref = wlr.yaw_fdb;
				if(power_control.judge_max_power > 40.0f)
					chassis_rotate_ramp.min = -(CHASSIS_ROTATE_SPEED + (power_control.judge_max_power - 40.0f/25.0f));
				else
					chassis_rotate_ramp.min = -CHASSIS_ROTATE_SPEED ;
				wlr.wz_ref = ramp_calc(&chassis_rotate_ramp , -(CHASSIS_ROTATE_SPEED + (power_control.judge_max_power - 40.0f)/25.0f));
            } else {	//也就是 当前 chassis.mode == CHASSIS_MODE_REMOTER_ROTATE1
				if (rotate_flag) {
					wlr.yaw_ref = wlr.yaw_fdb;
				if(power_control.judge_max_power > 40.0f)
					chassis_rotate_ramp.max = CHASSIS_ROTATE_SPEED + (power_control.judge_max_power - 40.0f)/25.0f;
				else
					chassis_rotate_ramp.max = CHASSIS_ROTATE_SPEED;
				wlr.wz_ref = ramp_calc(&chassis_rotate_ramp,CHASSIS_ROTATE_SPEED + (power_control.judge_max_power - 40.0f)/25.0f);
				
				if (supercap.volume_percent < 35.0f)
					wlr.wz_ref -= 3.5f;
				else if (supercap.volume_percent < 40.0f)
					wlr.wz_ref -= 3.0f;
				else if (supercap.volume_percent < 45.0f)
					wlr.wz_ref -= 2.5f;
				else if (supercap.volume_percent < 50.0f)
					wlr.wz_ref -= 2.0f;
				else if (supercap.volume_percent < 55.0f)
					wlr.wz_ref -= 1.5f;
				else if (supercap.volume_percent < 60.0f)
					wlr.wz_ref -= 1.0f;		
				//pitch歪小陀螺减速
				if (fabs(chassis_imu.pit) > 0.55f)
					wlr.wz_ref -= 5.0f;
				else if (fabs(chassis_imu.pit) > 0.50f)
					wlr.wz_ref -= 4.5f;
				else if (fabs(chassis_imu.pit) > 0.45f)
					wlr.wz_ref -= 4.0f;
				else if (fabs(chassis_imu.pit) > 0.40f)
					wlr.wz_ref -= 3.5f;
				else if (fabs(chassis_imu.pit) > 0.35f)
					wlr.wz_ref -= 3.0f;
				else if (fabs(chassis_imu.pit) > 0.30f)
					wlr.wz_ref -= 2.5f;	
				else if (fabs(chassis_imu.pit) > 0.25f)
					wlr.wz_ref -= 2.0f;	
				else if (fabs(chassis_imu.pit) > 0.20f)
					wlr.wz_ref -= 1.5f;								
				else if (fabs(chassis_imu.pit) > 0.15f)
					wlr.wz_ref -= 1.0f;	
				else if (fabs(chassis_imu.pit) > 0.10f)
					wlr.wz_ref -= 0.5f;	
				if (wlr.wz_ref < 0.0f)
					wlr.wz_ref = 0.0f;
			}
		}
            break;
        }
        case CHASSIS_MODE_KEYBOARD_UNFOLLOW: {
            if (last_chassis_mode != CHASSIS_MODE_KEYBOARD_UNFOLLOW)
                wlr.yaw_ref = -chassis_imu.yaw;
			
				wlr.yaw_fdb = -chassis_imu.yaw;
				wlr.wz_ref = 0;
            break;
        }
        default:break;
    }

    //速度坐标系换 限制 0~2PI
    if (wlr.yaw_ref < 0)
        wlr.yaw_ref += 2 * PI;
    else if (wlr.yaw_ref > 2 * PI)
        wlr.yaw_ref -= 2 * PI;
	
    //此yaw_err用于平移速度体系换算
    wlr.yaw_err = circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, (float)yaw_motor.ecd / 8192 * 2 * PI, 2 * PI);
	if (chassis.mode == CHASSIS_MODE_KEYBOARD_FIGHT)
		wlr.yaw_err = circle_error((float)CHASSIS_YAW_FIGHT / 8192 * 2 * PI, (float)yaw_motor.ecd / 8192 * 2 * PI, 2 * PI);
				
    chassis.output.vx = chassis.input.vx * arm_cos_f32(wlr.yaw_err) - chassis.input.vy * arm_sin_f32(wlr.yaw_err);
    chassis.output.vy = chassis.input.vx * arm_sin_f32(wlr.yaw_err) + chassis.input.vy * arm_cos_f32(wlr.yaw_err);
		
    if (chassis.mode == CHASSIS_MODE_REMOTER_ROTATE1 ||
        chassis.mode == CHASSIS_MODE_REMOTER_ROTATE2 ||
        chassis.mode == CHASSIS_MODE_KEYBOARD_ROTATE){
		if (fabs(wlr.v_fdb) > 2.0f)
			chassis.output.vx = -2.0f * wlr.v_fdb;
		else
			rotate_flag = 1;            
	}
     else if(rotate_ramp_flag == 0)
          rotate_flag = 0;   
    
    //小陀螺缓冲
    if ((last_chassis_mode == CHASSIS_MODE_REMOTER_ROTATE1  ||
         last_chassis_mode == CHASSIS_MODE_REMOTER_ROTATE2  ||
         last_chassis_mode == CHASSIS_MODE_KEYBOARD_ROTATE) && (
         chassis.mode == CHASSIS_MODE_KEYBOARD_FOLLOW       || 
         chassis.mode == CHASSIS_MODE_REMOTER_FOLLOW)) {
			rotate_ramp_flag = 1;  		 
			rppppp_flag = 1;
		 }
	if (rppppp_flag) {
		if (fabs(wlr.wz_fdb) < 2.0f)
			rppppp_flag = 0;
		}
    if(rotate_ramp_flag) {
        if(spin_zero == 0)
            spin_zero =  spin_limit;
		rotate_state_cnt++;
		if(fabs(spin_zero) < PI/2.0f  )//谁大听谁
		{					
			if( fabs(circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, wlr.yaw_fdb, 2 * PI)) < 1.0f && rotate_state_cnt > 100){
				rotate_ramp_flag = 0;
				spin_zero = 0;	
				rotate_state_cnt = 0;
				rotate_chassis_mode	= CHASSIS_MODE_PROTECT ;		
			}
		}else{
			if(fabs(circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI-PI, wlr.yaw_fdb, 2 * PI)) < 1.0f  && rotate_state_cnt > 100){
				rotate_ramp_flag = 0;
				spin_zero = 0;
				rotate_state_cnt = 0;
			}
		}
					
		if (last_chassis_mode == CHASSIS_MODE_REMOTER_ROTATE2 || last_chassis_mode == CHASSIS_MODE_REMOTER_ROTATE1 ||last_chassis_mode == CHASSIS_MODE_KEYBOARD_ROTATE ) 
			rotate_chassis_mode = last_chassis_mode;
		
			if(rotate_chassis_mode == CHASSIS_MODE_REMOTER_ROTATE2 ||rotate_chassis_mode == CHASSIS_MODE_KEYBOARD_ROTATE  )
				wlr.wz_ref = -12.0f/1.0f;  
			else if (rotate_chassis_mode == CHASSIS_MODE_REMOTER_ROTATE1 ) 
				wlr.wz_ref = 12.0f/1.0f;    
			
			wlr.yaw_ref	= wlr.yaw_fdb;
		}
	
		
		wlr.yaw_err = circle_error(wlr.yaw_ref,wlr.yaw_fdb, 2 * PI);//补
		/**********上台阶前让车身转至正对**********/
		if (wlr.jump_flag && !wlr.jump_pre) {
			wlr.wz_ref = 8.0f;
			wlr.yaw_err = 0;
		}
		wlr.yaw_ref = wlr.yaw_fdb + 1.0f * wlr.yaw_err;//同步带哥
		wlr.v_ref = chassis.output.vx;
		
		if (wlr.jump_flag && !wlr.jump_pre && fabs(circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, wlr.yaw_fdb, 2 * PI)) < 0.2f)
			wlr.jump_pre = 1;
		
		
		
		if (chassis.mode == CHASSIS_MODE_KEYBOARD_FIGHT)
			wlr.v_ref = -chassis.output.vy;
			wlr.s_ref += (wlr.v_ref * 0.001f * 2);
		if(rppppp_flag) {
			wlr.v_ref = 0.0f;
		}
    //陀螺仪数据输入
    wlr.roll_fdb    = -chassis_imu.rol;
    wlr.pit_fdb     = -chassis_imu.pit;
    kal_wy.measured_vector[0] = -chassis_imu.wy;
    kalman_filter_update(&kal_wy);
    wlr.wy_fdb = kal_wy.filter_vector[0];
    
//    wlr.wy_fdb      = -chassis_imu.wy;//
    wlr.wz_fdb      = -chassis_imu.wz;//加一下滤波
    wlr.az_fdb      =  chassis_imu.az;
    //电机数据输入joint_motor[1].position
     
     
    wlr.side[0].q2 =  joint_motor[1].position - joint_motor[1].zero_point;//电机原�?�数�?
    wlr.side[0].q1 =  joint_motor[0].position - joint_motor[0].zero_point;
    if (wlr.side[0].q2 < 0)
        wlr.side[0].q2 += 2 * PI;
    if (wlr.side[0].q1 < 0)
        wlr.side[0].q1 += 2 * PI;
    wlr.side[0].w1 =  joint_motor[0].velocity;
    wlr.side[0].w2 =  joint_motor[1].velocity;
    wlr.side[0].t1 =  joint_motor[0].torque;
    wlr.side[0].t2 =  joint_motor[1].torque;
    wlr.side[0].qy = -driver_motor[0].position;
    kal_3508_vel[0].measured_vector[0] = -driver_motor[0].velocity;
    kalman_filter_update(&kal_3508_vel[0]);
    wlr.side[0].wy = kal_3508_vel[0].filter_vector[0];

	
    wlr.side[1].q2 =  -(joint_motor[3].position - joint_motor[3].zero_point);//电机原�?�数�?
    wlr.side[1].q1 =  -(joint_motor[2].position - joint_motor[2].zero_point);
    if (wlr.side[1].q2 < 0)
        wlr.side[1].q2 += 2 * PI;
    if (wlr.side[1].q1 < 0)
        wlr.side[1].q1 += 2 * PI;
    wlr.side[1].w1 = -joint_motor[2].velocity;
    wlr.side[1].w2 = -joint_motor[3].velocity;
    wlr.side[1].t1 = -joint_motor[2].torque;
    wlr.side[1].t2 = -joint_motor[3].torque;
    wlr.side[1].qy =  driver_motor[1].position;
    kal_3508_vel[1].measured_vector[0] = driver_motor[1].velocity;
    kalman_filter_update(&kal_3508_vel[1]);
    wlr.side[1].wy = kal_3508_vel[1].filter_vector[0];
    
    //KNN
    for (int i = 0; i < 10; i++)
    input_data[i] = lqr.X_fdb[i];
    input_data[10] =  wlr.side[0].Fn_kal;
    input_data[11] =  wlr.side[1].Fn_kal;
    
    last_chassis_mode = chassis.mode;//�?了个位置
}

float left_speed=3, right_speed=3;
float left_T, right_T;
float fsjl;
float up_ready;

//原代码
//static void chassis_self_rescue(void)//翻车自救
//{
//	static uint32_t leg_length_cnt;
//	static float rescue_cnt = 0;	
//	static float rescue_T = 4;//翻转力矩
//	
//	if (fabs(chassis_imu.pit) > 1.5f)
//		rescue_T = 6.5;
//	else 
//		rescue_T = 4.5;
//		
//	left_T  = pid_calc(&pid_rescue[0], left_speed , (wlr.side[0].w1));
//	right_T = pid_calc(&pid_rescue[1], right_speed, (wlr.side[1].w1));
//		
////		if (rescue_cnt > 4000) {//----------------10s等待
////				if (fabs(chassis_imu.pit) > 1.5f)
////						rescue_T = 9;
////		}
//	//第四象限卡台阶
//	if ((vmc[0].quadrant ==4 || vmc[1].quadrant == 4) && chassis.rescue_inter_flag != 1)
//		rescue_cnt ++;
////	else
////		rescue_cnt = 0;
//		
//	fsjl = rescue_cnt;
////    if (!chassis.rescue_inter_flag && fabs(chassis_imu.pit) > 1.0f && fabs(chassis_imu.rol) < 0.1f)
////        chassis.rescue_inter_flag = 3;//3阶段 ----整车翻倒且保护天鹅颈
//		
//	if (!chassis.rescue_inter_flag && fabs(chassis_imu.pit) < 0.6f && (vmc[0].quadrant == 2 || vmc[1].quadrant == 2))
//		chassis.rescue_inter_flag = 4;//4阶段 ----第二象限启动卡墙
//		
//    if (!chassis.rescue_inter_flag)
//        chassis.rescue_inter_flag = 1;//1阶段 ----代表车身正在归正
//    
//    if (chassis.rescue_inter_flag == 3) {
//        dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, left_T);//0.03 0.5
//        dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);	
//        if (fabs(chassis_imu.rol) > 0.6f)
//            chassis.rescue_inter_flag = 1;
//    }    
//		
//	if (chassis.rescue_inter_flag == 4) {
//		//左腿归正
//		if (vmc[0].quadrant == 2 && chassis.rescue_cnt_L <= 100) {
//			dm_motor_set_control_para(&joint_motor[0], 0, -rescue_T, 0, 5, 0);//0.03 0.5
//            dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);	
//		} else if (vmc[0].quadrant == 1) 
//			chassis.rescue_cnt_L++;
//		if (chassis.rescue_cnt_L > 100) {
//			dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0);
//			dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0); 		
//		}
//		//右腿归正
//		if (vmc[1].quadrant == 2 && chassis.rescue_cnt_R <= 100 ) {
//			dm_motor_set_control_para(&joint_motor[2], 0, rescue_T, 0, 5, 0);//0.03 0.5
//            dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);	
//		} else if (vmc[1].quadrant == 1) 
//			chassis.rescue_cnt_R++;
//		if (chassis.rescue_cnt_R > 100) {
//            dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 0);
//            dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0); 
//		}
//		//进入收腿阶段
//        if(chassis.rescue_cnt_L > 200 || chassis.rescue_cnt_R > 200) {
//            chassis.rescue_cnt_L = 0;
//            chassis.rescue_cnt_R = 0;
////			pid_rescue[0].i_out = 0;
////			pid_rescue[1].i_out = 0;
//			chassis.rescue_inter_flag = 1;
//		}
//	}
//    
//    if (chassis.rescue_inter_flag == 1) {		
//        //左腿归正
////        if (vmc[0].quadrant == 3 || vmc[0].quadrant == 2 || vmc[0].quadrant == 4 || fabs(chassis_imu.pit) > 2.4f ||fabs(chassis_imu.rol )> 1.6f) {
//		if  ( (vmc[0].quadrant == 1 || vmc[0].quadrant == 2 || vmc[0].quadrant == 3) &&  (fabs(chassis_imu.pit) > 0.3f ||fabs(chassis_imu.rol )> 0.3f) || chassis.rescue_cnt_L > 1250 ) {
////      	dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, left_T);//0.03 0.5
//			dm_motor_set_control_para(&joint_motor[0], 0, 8, 0, 5, 10);//快哥z
//            dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);	
//			if( chassis.rescue_cnt_L < 1000)
//				chassis.rescue_cnt_L = 0;
////				chassis.rescue_cnt_L -=5; 
//        } else if (vmc[0].quadrant == 4 || fabs(chassis_imu.pit) < 0.2f ||fabs(chassis_imu.rol ) <  0.2f) {
//            dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0);
//            dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);        
//            chassis.rescue_cnt_L++;       
//        }
//        //右腿归正        
////        if (vmc[1].quadrant == 3 || vmc[1].quadrant == 2 || vmc[1].quadrant == 4 || fabs(chassis_imu.pit) > 2.4f || fabs(chassis_imu.rol) >1.6f) {
//		if ((vmc[1].quadrant == 1 || vmc[1].quadrant == 2 || vmc[1].quadrant == 3) &&  (fabs(chassis_imu.pit) > 0.3f ||fabs(chassis_imu.rol )> 0.3f) || chassis.rescue_cnt_R > 1250 ) {
////          dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, -right_T);//0.03 0.5
//			dm_motor_set_control_para(&joint_motor[2], 0, -8, 0, 5, 10);//快哥
//            dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);
//			if( chassis.rescue_cnt_R < 1000)
//            chassis.rescue_cnt_R = 0; 
////			chassis.rescue_cnt_R -=5; 
//        } else if (vmc[1].quadrant == 4 || fabs(chassis_imu.pit) < 0.2f ||fabs(chassis_imu.rol ) <  0.2f) {
//            dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 0);//0.03 0.5
//            dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);        
//            chassis.rescue_cnt_R++;               
//        }
//        //进入收腿阶段
//		if((chassis.rescue_cnt_L > 100 && chassis.rescue_cnt_R > 100) || up_ready ){
//			if(!up_ready)
//				up_ready = 1;
//			/*shangjiao*/
//			if(vmc[0].quadrant != 1 ){
//				dm_motor_set_control_para(&joint_motor[0], 0, 6, 0, 5, 0);//快哥z
//				dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);
//			}else{
//				dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0);//0.03 0.5
//				dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);
//			}
//						
//			if(vmc[1].quadrant != 1 ){
//				dm_motor_set_control_para(&joint_motor[2], 0, -6, 0, 5, 0);//快哥
//				dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);
//			}else{
//				dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 0);//0.03 0.5
//				dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);
//			}
//			
//			if(vmc[0].quadrant == 1 && vmc[1].quadrant == 1)
//				up_ready++;
//			if(up_ready > 100)
//				chassis.rescue_inter_flag = 2;
//		}
//    } else if (chassis.rescue_inter_flag == 2) { //开始收腿
////		if (vmc[0].quadrant == 3 || vmc[1].quadrant == 3)
////			wlr.side[0].T1 = wlr.side[0].T2 = wlr.side[1].T1 = wlr.side[1].T2 = 0;
//        dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0.8f*wlr.side[0].T1);//0.03 0.5
//        dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 1.0f*wlr.side[0].T2); 
//        dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0,-0.8f*wlr.side[1].T1);
//        dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0,-1.0f*wlr.side[1].T2); 
//		dji_motor_set_torque(&driver_motor[0], 0);
//		dji_motor_set_torque(&driver_motor[1], 0);	
//			
//        if (fabs(vmc[0].L_fdb - wlr.recover_length) < 0.1f && fabs(vmc[1].L_fdb - wlr.recover_length) < 0.1f ) {
//			leg_length_cnt++;
//			if(leg_length_cnt > 50){
//				leg_length_cnt = 0;
//				rescue_cnt = 0;
//				quadrant_cnt = 0;
//				chassis.rescue_cnt_L = 0;
//				chassis.rescue_cnt_R = 0;
//				chassis.recover_flag = 2; 
//				chassis.rescue_inter_flag = 0;
//				wlr.high_flag = 0;
//				up_ready=0;
//			}
//        }
//    }    
//   
//    dji_motor_set_torque(&driver_motor[0], 0);
//    dji_motor_set_torque(&driver_motor[1], 0);
//	//第四象限卡台阶
//	if (rescue_cnt > 5000 && (vmc[0].quadrant ==4 || vmc[1].quadrant == 4) && chassis.rescue_inter_flag != 2  ) {
//		if (chassis_imu.pit < -0.25f) { 
//			dji_motor_set_torque(&driver_motor[0], -2);
//			dji_motor_set_torque(&driver_motor[1], 2);			
//		}else {			
//			dji_motor_set_torque(&driver_motor[0], 2);
//			dji_motor_set_torque(&driver_motor[1], -2);
//		}
//	}
//	
//	if(ctrl_mode == PROTECT_MODE){
//		leg_length_cnt = 0;
//		rescue_cnt = 0;
//		quadrant_cnt = 0;
//		chassis.rescue_cnt_L = 0;
//		chassis.rescue_cnt_R = 0;
//		chassis.recover_flag = 2; 
//		chassis.rescue_inter_flag = 0;
//		wlr.high_flag = 0;
//		up_ready=0;
//	}
//}

static void chassis_self_rescue(void)//翻车自救
{
	static uint32_t leg_length_cnt;		//腿长到达目标长度之后，变量++，延时0.1s
	static float rescue_cnt = 0;		
	static float rescue_T = 4;			//翻转力矩
	
	if (fabs(chassis_imu.pit) > 1.5f)
		rescue_T = 6.5;
	else
		rescue_T = 4.5;
		
	left_T  = pid_calc(&pid_rescue[0], left_speed , (wlr.side[0].w1));
	right_T = pid_calc(&pid_rescue[1], right_speed, (wlr.side[1].w1));
		
	//第四象限卡台阶
	if ((vmc[0].quadrant ==4 || vmc[1].quadrant == 4) && chassis.rescue_inter_flag != 1)
		rescue_cnt ++;
		
	fsjl = rescue_cnt;
//    if (!chassis.rescue_inter_flag && fabs(chassis_imu.pit) > 1.0f && fabs(chassis_imu.rol) < 0.1f)
//        chassis.rescue_inter_flag = 3;//3阶段 ----整车翻倒且保护天鹅颈
		
	if (!chassis.rescue_inter_flag && fabs(chassis_imu.pit) < 0.6f && (vmc[0].quadrant == 2 || vmc[1].quadrant == 2))
		chassis.rescue_inter_flag = 4;//4阶段 ----第二象限启动卡墙
		
    if (!chassis.rescue_inter_flag)
        chassis.rescue_inter_flag = 1;//1阶段 ----代表车身正在归正
    
	
	
    if (chassis.rescue_inter_flag == 3) {
        dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, left_T);//0.03 0.5
        dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);	
        if (fabs(chassis_imu.rol) > 0.6f)
            chassis.rescue_inter_flag = 1;
    }    
		
	if (chassis.rescue_inter_flag == 4) {	//第二象限启动卡墙
		//左腿归正
		if (vmc[0].quadrant == 2 && chassis.rescue_cnt_L <= 100) {
			dm_motor_set_control_para(&joint_motor[0], 0, -rescue_T, 0, 5, 0);//0.03 0.5
            dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);	
		} else if (vmc[0].quadrant == 1) 
			chassis.rescue_cnt_L++;
		if (chassis.rescue_cnt_L > 100) {
			dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0);
			dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0); 		
		}
		//右腿归正
		if (vmc[1].quadrant == 2 && chassis.rescue_cnt_R <= 100 ) {
			dm_motor_set_control_para(&joint_motor[2], 0, rescue_T, 0, 5, 0);//0.03 0.5
            dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);	
		} else if (vmc[1].quadrant == 1)
			chassis.rescue_cnt_R++;
		if (chassis.rescue_cnt_R > 100) {
            dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 0);
            dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);
		}
		//进入收腿阶段
        if(chassis.rescue_cnt_L > 200 || chassis.rescue_cnt_R > 200) {
            chassis.rescue_cnt_L = 0;
            chassis.rescue_cnt_R = 0;
			chassis.rescue_inter_flag = 1;
		}
	}
    
    if (chassis.rescue_inter_flag == 1) {
        //左腿归正
		if ( (vmc[0].quadrant == 1 || vmc[0].quadrant == 2 || vmc[0].quadrant == 3) && (fabs(chassis_imu.pit) > 0.3f ||fabs(chassis_imu.rol )> 0.3f) || chassis.rescue_cnt_L > 1250 ) {
			dm_motor_set_control_para(&joint_motor[0], 0, 8, 0, 5, 10);//快哥z
            dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);
			if( chassis.rescue_cnt_L < 1000)
				chassis.rescue_cnt_L = 0;
        } else if (vmc[0].quadrant == 4 || fabs(chassis_imu.pit) < 0.2f ||fabs(chassis_imu.rol ) <  0.2f) {
            dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0);
            dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);        
            chassis.rescue_cnt_L++;       
        }
        //右腿归正        
		if ((vmc[1].quadrant == 1 || vmc[1].quadrant == 2 || vmc[1].quadrant == 3) &&  (fabs(chassis_imu.pit) > 0.3f ||fabs(chassis_imu.rol )> 0.3f) || chassis.rescue_cnt_R > 1250 ) {
			dm_motor_set_control_para(&joint_motor[2], 0, -8, 0, 5, 10);//快哥
            dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);
			if( chassis.rescue_cnt_R < 1000)
            chassis.rescue_cnt_R = 0; 
        } else if (vmc[1].quadrant == 4 || fabs(chassis_imu.pit) < 0.2f ||fabs(chassis_imu.rol ) <  0.2f) {
            dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 0);//0.03 0.5
            dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);
            chassis.rescue_cnt_R++;
        }
        //进入收腿阶段
		if((chassis.rescue_cnt_L > 100 && chassis.rescue_cnt_R > 100) || up_ready ){
			if(!up_ready)
				up_ready = 1;
			/*shangjiao*/
			if(vmc[0].quadrant != 1 ){
				dm_motor_set_control_para(&joint_motor[0], 0, 6, 0, 5, 0);//快哥z
				dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);
			}else{
				dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0);//0.03 0.5
				dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);
			}
						
			if(vmc[1].quadrant != 1 ){
				dm_motor_set_control_para(&joint_motor[2], 0, -6, 0, 5, 0);//快哥
				dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);
			}else{
				dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 0);//0.03 0.5
				dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);
			}
			
			if(vmc[0].quadrant == 1 && vmc[1].quadrant == 1)
				up_ready++;
			if(up_ready > 100)
				chassis.rescue_inter_flag = 2;
		}
		

    } else if (chassis.rescue_inter_flag == 2) { //开始收腿
        dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 1.0f*wlr.side[0].T1);//0.03 0.5
        dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 1.0f*wlr.side[0].T2); 
        dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0,-1.0f*wlr.side[1].T1);
        dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0,-1.0f*wlr.side[1].T2);
		dji_motor_set_torque(&driver_motor[0], 0);
		dji_motor_set_torque(&driver_motor[1], 0);
			
		
        if (fabs(vmc[0].L_fdb - wlr.recover_length) < 0.05f && fabs(vmc[1].L_fdb - wlr.recover_length) < 0.05f )  {
			leg_length_cnt++;
			if(leg_length_cnt > 50){
				leg_length_cnt = 0;
				rescue_cnt = 0;
				quadrant_cnt = 0;
				chassis.rescue_cnt_L = 0;
				chassis.rescue_cnt_R = 0;
				chassis.recover_flag = 2;
				chassis.rescue_inter_flag = 0;
				wlr.high_flag = 0;
				up_ready=0;
				
			}
        }
    }    
   
    dji_motor_set_torque(&driver_motor[0], 0);
    dji_motor_set_torque(&driver_motor[1], 0);
	
	//第四象限卡台阶 暂时注释
	if (rescue_cnt > 5000 && (vmc[0].quadrant ==4 || vmc[1].quadrant == 4) && chassis.rescue_inter_flag != 2  && 0 ) {
		if (chassis_imu.pit < -0.25f) { 
			dji_motor_set_torque(&driver_motor[0], -2);
			dji_motor_set_torque(&driver_motor[1], 2);			
		}else {			
			dji_motor_set_torque(&driver_motor[0], 2);
			dji_motor_set_torque(&driver_motor[1], -2);
		}
	}
	
	if(ctrl_mode == PROTECT_MODE){
		leg_length_cnt = 0;
		rescue_cnt = 0;
		quadrant_cnt = 0;
		chassis.rescue_cnt_L = 0;
		chassis.rescue_cnt_R = 0;
		chassis.recover_flag = 2; 
		chassis.rescue_inter_flag = 0;
		wlr.high_flag = 0;
		up_ready=0;

	}
}


static void chassis_rescue_test(void)
{
    if (RC_LEFT_LU_CH_VALUE) {
        dm_motor_set_control_para(&joint_motor[0], 0, 4, 0, 6, 0);//0.03 0.5
        dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);
		dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 0);
		dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);        
    } else if (RC_LEFT_LD_CH_VALUE){
        dm_motor_set_control_para(&joint_motor[0], 0, -4, 0, 6, 0);//0.03 0.5
        dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0); 
		dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 0);
		dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);        
    } else if (RC_LEFT_RU_CH_VALUE){
        dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0);
        dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);        
        dm_motor_set_control_para(&joint_motor[2], 0, -4, 0, 6, 0);//0.03 0.5
        dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);              
    } else if (RC_LEFT_RD_CH_VALUE){
        dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0);
        dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);        
        dm_motor_set_control_para(&joint_motor[2], 0, 4, 0, 6, 0);//0.03 0.5
        dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);          
    } else {
        dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0);
        dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);
		dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 0);
		dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);
    }
        
    dji_motor_set_torque(&driver_motor[0], 0);
    dji_motor_set_torque(&driver_motor[1], 0);    

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
		pid_leg_recover[0].i_out = 0;
		pid_leg_recover[1].i_out = 0;
        for (int i = 0; i < 4; i++) {
			dm_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
        }
    } else if (wlr.ctrl_mode == 2) {//力控
        dji_motor_set_torque(&driver_motor[0], -wlr.side[0].Tw);
        dji_motor_set_torque(&driver_motor[1],  wlr.side[1].Tw);
        if (wlr.prone_flag) {
			prone_cnt++;
			if (prone_cnt < 200) {
				dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, -3.0f);
				dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);
				dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 3.0f);
				dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);				
			} else if (prone_cnt > 600){
				dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 8.0f);
				dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);
				dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, -8.0f);
				dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);
			} else {
				dm_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0, 0);
				dm_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0, 0);
				dm_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0, 0);
				dm_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0, 0);						
			}
			
        } else {
            if(chassis.recover_flag == 1)//恢复至第一象限
                chassis_self_rescue();
            if(chassis.recover_flag != 1) {
				if(wlr.crash_flag){	  //撞到台阶，让两条腿向后方运动，等到摆角到达一定值，再收腿
					dm_motor_set_control_para(&joint_motor[0], 0, -5.0, 0, 5, 0);//快哥
					dm_motor_set_control_para(&joint_motor[1], 0, 0,    0, 0, 0);	
					dm_motor_set_control_para(&joint_motor[2], 0, 5.0,  0, 5, 0);//快哥
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
            };
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
	
	
	if(joint_motor[0].online && joint_motor[1].online && joint_motor[2].online && joint_motor[3].online )
	wlr.joint_all_online = 1;
	else
	wlr.joint_all_online = 0;	
}

void chassis_task(void const *argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    power_init();
    for(;;)
    {	
        if (chassis.init == 0) {
            chassis_init();
        }
        thread_wake_time = osKernelSysTick();
        chassis_mode_switch();
        chassis_data_input();
		if(ctrl_mode != PROTECT_MODE)
        wlr_control();
		else
		wlr_init();
		
        chassis_data_output();
            
        status.task.chassis = 1;
              
        osDelayUntil(&thread_wake_time, 2);
    }
}
