#include "mode_switch_task.h"
#include "status_task.h"
#include "control_def.h"
#include "drv_dji_motor.h"
#include "prot_vision.h"
#include "prot_dr16.h"
#include "prot_imu.h"
#include "math_lib.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "string.h"
#include "func_generator.h"
#include "gimbal_task.h"
#include "status_task.h"
#include "smc.h"


FGT_agl_t yaw_test = {
    .Td = 1,
    .time = 0,
    .T1 = 150,
    .T2 = 0,
    .T1_out = 0,
    .T2_out = 0.175,
    .out = 0
};

gimbal_scale_t gimbal_scale = {
    .ecd_remote = 0.000005f,//待修改sensity
    .ecd_keyboard = 1,
    .angle_remote = 0.000015f,
    .angle_keyboard = 0.00003f
};
gimbal_t gimbal;

static void gimbal_init(void)
{
    memset(&gimbal, 0, sizeof(gimbal_t));

//    pid_init(&gimbal.pit_angle.pid, NONE, 18, 0, 0, 0, 7);//20 0 0 有测速模块
//    pid_init(&gimbal.pit_spd.pid, NONE, -1.2f, -0.006f, 0, 0.4f, 2.2f);//-1.0 -0.008
//    
	  pid_init(&gimbal.pit_angle.pid, CHANG_I_RATE, 30.0f, 0.25, 0, 25, 50);//20 0 0 有测速模块
    pid_init(&gimbal.pit_spd.pid, CHANG_I_RATE, 10000.0f, 0.0f, 0, 8000.0f, 25000.0f);//-1.0 -0.008
		gimbal.pit_angle.pid.threshold_a = 0.002f;
		gimbal.pit_angle.pid.threshold_b = 0.2f;
	
    
//    pid_init(&gimbal.yaw_angle.pid, CHANG_I_RATE,25.0f, 0.15, 0.0f, 50, 100);
	 pid_init(&gimbal.yaw_angle.pid, CHANG_I_RATE,25.0f, 0.15f, 0.0f, 50, 100);//尝试云台补偿算法
    pid_init(&gimbal.yaw_spd.pid, CHANG_I_RATE, 8000.0f, 0.00f, 0, 10000.0f, 25000.0f);
		gimbal.yaw_angle.pid.threshold_a = 0.002f;
		gimbal.yaw_angle.pid.threshold_b = 0.3f;
	
    pid_init(&gimbal.yaw_ecd.pid, NONE, 10.0f, 0, 0, 0.0f, 30.0f);
		pid_init(&gimbal.yaw_spd_ecd.pid, NONE, 6000.0f, 10.00f, 0, 1000.0f, 25000.0f);
	

}
float yaw_err;
float slope_feed,last_yaw_ref,slope_feed_pit,last_pit_ref;
static void gimbal_pid_calc(void)
{
    float  pit_max, pit_min;
    //位置环反馈 陀螺仪 -0.6 0.4
    //速度环反馈 陀螺仪
    //此yaw_err用于云台pit限幅
    yaw_err = circle_error(CHASSIS_YAW_OFFSET / 8192.0f * 2 * PI, yaw_motor.ecd / 8192.0f * 2 * PI, 2 * PI);
    pit_max = -arm_cos_f32(yaw_err) * chassis_imu.pit + 0.60f;
    pit_min = -arm_cos_f32(yaw_err) * chassis_imu.pit - 0.38f;
    data_limit(&gimbal.pit_angle.ref, pit_min, pit_max);
    gimbal.pit_angle.fdb = gimbal_imu.pit;
		
			slope_feed_pit = gimbal.pit_angle.ref - last_pit_ref;
	
//		gimbal.pit_angle.ref = gimbal.pit_angle.ref + (-gimbal.feedback_beta_speed_input * 0.002f);//云台稳定算法

    gimbal.pit_spd.ref = pid_calc(&gimbal.pit_angle.pid, gimbal.pit_angle.ref - gimbal.feedback_beta_speed_input * 0.002f + slope_feed_pit , gimbal.pit_angle.fdb);
//    gimbal.pit_spd.fdb = -gimbal_imu.wy - arm_cos_f32(yaw_err) * chassis_imu.wy;
    gimbal.pit_spd.fdb = gimbal_imu.wy;
    gimbal.pit_output = -pid_calc(&gimbal.pit_spd.pid, gimbal.pit_spd.ref, gimbal.pit_spd.fdb);

    //pid参数选择

    
    if (gimbal.yaw_angle.ref < 0) {
        gimbal.yaw_angle.ref += 2 * PI;
    } else if (gimbal.yaw_angle.ref > 2 * PI) {
        gimbal.yaw_angle.ref -= 2 * PI;
    }
    //视觉测试
//    gimbal.yaw_angle.ref = FGT_agl_calc(&yaw_test);
    gimbal.yaw_angle.fdb = gimbal_imu.yaw;
    //此yaw_err用于云台yaw环形控制
		

		slope_feed = gimbal.yaw_angle.ref - last_yaw_ref;

//		slope_feed = 0;
		
    yaw_err = circle_error(gimbal.yaw_angle.ref + gimbal.feedback_alpha_speed_input * 0.002f + slope_feed  , gimbal.yaw_angle.fdb, 2*PI);
    if (gimbal.start_up == 0 && yaw_err < 0.03f)//-------------------->无云台控制下注释
        gimbal.start_up = 1;
	
    gimbal.yaw_spd.ref = pid_calc(&gimbal.yaw_angle.pid, gimbal.yaw_angle.fdb + yaw_err, gimbal.yaw_angle.fdb);    
    gimbal.yaw_spd.fdb = gimbal_imu.wz;
    gimbal.yaw_output = pid_calc(&gimbal.yaw_spd.pid, gimbal.yaw_spd.ref, gimbal.yaw_spd.fdb);//SMC
//		if (gimbal.start_up == 0 && fabs(yaw_err)  > 0.03f)//-------------------->无云台控制下注释
//       gimbal.yaw_output  *= 0.1f;
    
//    //无云台控制------->无陀螺仪, 通过ecd控制
		if( rc_fsm_check(RC_LEFT_LD)  && 0 ){
    gimbal.yaw_ecd.fdb = (float)yaw_motor.ecd / 8192 * 2  * PI;
    yaw_err = circle_error(gimbal.yaw_ecd.ref, gimbal.yaw_ecd.fdb, 2*PI);
    gimbal.yaw_spd_ecd.ref = pid_calc(&gimbal.yaw_ecd.pid, gimbal.yaw_ecd.fdb + yaw_err, gimbal.yaw_ecd.fdb);
		
    gimbal.yaw_spd_ecd.fdb = gimbal_imu.wz;
		gimbal.yaw_output = pid_calc(&gimbal.yaw_spd_ecd.pid, gimbal.yaw_spd_ecd.ref, gimbal.yaw_spd_ecd.fdb);
		
		
		}
		
		last_yaw_ref = gimbal.yaw_angle.ref;
		last_pit_ref = gimbal.pit_angle.ref;
//		    if (gimbal.start_up == 0 && yaw_err < 0.03f) //无云台时起身判断
//        gimbal.start_up = 1;    

}

static void gimbal_data_output(void)
{
    if(gimbal_imu.online == 0 || yaw_motor.online == 0){
    dji_motor_set_torque(&pit_motor, 0);
    dji_motor_set_torque(&yaw_motor, 0);     
    
    }
        
    else{
    dji_motor_set_torque(&pit_motor, 1.0f*gimbal.pit_output);
    dji_motor_set_torque(&yaw_motor, 1.0f*gimbal.yaw_output);   
    }
}

static void gimbal_get_vision_data(void)
{
    switch (vision.aim_status) {
        case AIMING: {//识别到目标
            if (vision.new_frame_flag) {
                vision.new_frame_flag = 0;
                gimbal.pit_angle.ref = vision.target_pit_angle;
                gimbal.yaw_angle.ref = vision.target_yaw_angle;
            }
            break;
        }
        case FIRST_LOST: {//首次丢失
            vision.aim_status = UNAIMING;
            gimbal.pit_angle.ref = gimbal.pit_angle.fdb;
            gimbal.yaw_angle.ref = gimbal.yaw_angle.fdb;
            break;
        }
        case UNAIMING: {//未识别到目标
            if (ctrl_mode == REMOTER_MODE) {
                gimbal.pit_angle.ref -= rc.ch2 * gimbal_scale.angle_remote;
                gimbal.yaw_angle.ref -= rc.ch1 * gimbal_scale.angle_remote;
            } else if (ctrl_mode == KEYBOARD_MODE) {
                gimbal.pit_angle.ref += rc.mouse.y * gimbal_scale.angle_keyboard * 0.5f;
                gimbal.yaw_angle.ref -= rc.mouse.x * gimbal_scale.angle_keyboard;
            }
            break;
        }
        default: break;
    }
}

void gimbal_task(void const *argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    gimbal_init();
	static float yaw_err_up;
    for(;;) {
        thread_wake_time = osKernelSysTick();
        taskENTER_CRITICAL();
        switch (ctrl_mode) {
            case PROTECT_MODE: {
                gimbal.start_up = 0;//保护模式下，起身标志位置零
                gimbal.yaw_ecd.ref = (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI;
								yaw_err_up = circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, (float)yaw_motor.ecd / 8192 * 2 *PI, 2 * PI);
				if(fabs(yaw_err_up) > PI ||!gimbal.start_up )
				gimbal.yaw_angle.ref = gimbal_imu.yaw - (float)yaw_motor.ecd / 8192 * 2 * PI +  (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI;//yaw轴反馈值+电机与前方灯条差值
				else
//				gimbal.yaw_angle.ref = gimbal_imu.yaw + (float)yaw_motor.ecd / 8192 * 2 * PI +  (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI + PI;	
								gimbal.yaw_angle.ref = gimbal_imu.yaw - (float)yaw_motor.ecd / 8192 * 2 * PI +  (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI ;	
				
				
                gimbal.pit_angle.ref = 0.48f;
                gimbal.pit_output = 0;
                gimbal.yaw_output = 0;
                break;
            }
            case REMOTER_MODE: {
                if ( ( rc_fsm_check(RC_LEFT_LD) && rc_fsm_check(RC_RIGHT_RD) )) { //遥控器开启视觉
                    
                    gimbal_get_vision_data();
                } 
                else {
                    gimbal.yaw_ecd.ref   -= rc.ch1 * gimbal_scale.ecd_remote;
                    gimbal.pit_angle.ref -= -rc.ch2 * gimbal_scale.angle_remote;
                    gimbal.yaw_angle.ref -= rc.ch1 * gimbal_scale.angle_remote;
                }

//								YawSMC.ref = gimbal_imu.yaw* 57.29577f + circle_error(gimbal.yaw_angle.ref* 57.29577f, gimbal_imu.yaw* 57.29577f, 360.0f);;
//								SMC_Tick(&YawSMC,gimbal_imu.yaw* 57.29577f,gimbal_imu.wz* 57.29577f);
//								gimbal.yaw_output = -YawSMC.u;
                gimbal_pid_calc();
                break;
            }
            case KEYBOARD_MODE: {
                if (rc.mouse.r == 1) {
                    gimbal_get_vision_data();
                } else {
                    //一键调头
                    if(key_scan_clear(KEY_GIMBAL_TURN_R)) {
                        gimbal.yaw_angle.ref -= PI/2;
                    } 
                    else if (key_scan_clear(KEY_GIMBAL_TURN_L)) {
                        gimbal.yaw_angle.ref += PI/2;
                    }
                    gimbal.pit_angle.ref += rc.mouse.y * gimbal_scale.angle_keyboard ;
                    gimbal.yaw_angle.ref -= rc.mouse.x * gimbal_scale.angle_keyboard;
                }
//								YawSMC.ref = gimbal_imu.yaw* 57.29577f + circle_error(gimbal.yaw_angle.ref* 57.29577f, gimbal_imu.yaw* 57.29577f, 360.0f);;
//								SMC_Tick(&YawSMC,gimbal_imu.yaw* 57.29577f,gimbal_imu.wz* 57.29577f);
//								gimbal.yaw_output = -YawSMC.u;
								
                gimbal_pid_calc();
                break;
            }
            default:break;
        }
        
        gimbal_data_output();
        status.task.gimbal = 1;
        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 2);
    }
}
