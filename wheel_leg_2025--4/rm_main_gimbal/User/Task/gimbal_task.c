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
#include "drv_dm_motor.h"
#include "gimbal_decoupling.h"
#include "board_comm.h"
float yaw_err_up;
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
    .angle_remote = 0.0000045f,
    .angle_keyboard = 0.00003f
};
gimbal_t gimbal;
float vision_mpc_k = 1.5f;		// MPC：调这个前馈比例系数

float body_vector[3];
float rotate_data[3][3];

static void gimbal_init(void)
{
    memset(&gimbal, 0, sizeof(gimbal_t));
	
	pid_init(&gimbal.pit_angle.pid, NONE, 20.0f, 0.0f, 10, 0, 10); 
    pid_init(&gimbal.pit_spd.pid, NONE, 1.2f, 0.01f, 0, 0.5f, 7.0f);
	
//	pid_init(&gimbal.yaw_angle.pid, NONE,40.0f, 0.2f, 1.0f, 0.0f, 5.0f);//尝试云台补偿算法			MPC：调yaw的pid
//    pid_init(&gimbal.yaw_spd.pid, NONE, 10000.0f, 50.0f, 0, 2000.0f, 25000.0f);
	
	
	//视觉mpc用这个  
	pid_init(&gimbal.yaw_angle.pid, CHANG_I_RATE,15.0f, 0.0f, 0.0f, 0, 3);	
    pid_init(&gimbal.yaw_spd.pid, NONE, 25000.0f, 50.0f, 0, 0.0f, 25000.0f) ;							
	
    pid_init(&gimbal.yaw_ecd.pid, NONE, 10.0f, 0, 0, 0.0f, 30.0f);
	pid_init(&gimbal.yaw_spd_ecd.pid, NONE, 6000.0f, 10.00f, 0, 1000.0f, 25000.0f);
}
float yaw_err;
float slope_feed,last_yaw_ref,slope_feed_pit,last_pit_ref;
float kkkkk = 0.05F;
static void gimbal_pid_calc(void)
{
    float  pit_max, pit_min;
    //位置环反馈 陀螺仪 -0.6 0.4
    //速度环反馈 陀螺仪
    //此yaw_err用于云台pit限幅
    yaw_err = circle_error(CHASSIS_YAW_OFFSET / 8192.0f * 2 * PI, yaw_motor.ecd / 8192.0f * 2 * PI, 2 * PI);
//    pit_max = -arm_cos_f32(yaw_err) * chassis_imu.pit + 0.50f;
//		//1111 测试架，pit没装好，暂时限0.38
//	//pit_max = -arm_cos_f32(yaw_err) * chassis_imu.pit + 0.32f;
//    pit_min = -arm_cos_f32(yaw_err) * chassis_imu.pit - 0.35f;
	
	pit_max = 0.5f + imu_data_rec.pit;		//0.5 滑槽卡头
	pit_min = -0.3f + imu_data_rec.pit; 
	
	gimbal.feedback_alpha_speed_input = gimbal_data_rec.feedback_alpha_speed_input;
	
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
		

//	slope_feed = gimbal.yaw_angle.ref - last_yaw_ref;

	slope_feed = 0;
		
    yaw_err = circle_error(gimbal.yaw_angle.ref + slope_feed  , gimbal.yaw_angle.fdb, 2*PI);
    if (gimbal.start_up == 0 && fabsf(yaw_err) < 0.03f)//-------------------->无云台控制下注释
        gimbal.start_up = 1;
	
    // gimbal.yaw_spd.ref = pid_calc(&gimbal.yaw_angle.pid, gimbal.yaw_angle.fdb + yaw_err, gimbal.yaw_angle.fdb);    
    //MPC
		gimbal.yaw_spd.pid.ki = 50.0f;
		gimbal.yaw_spd.pid.i_max = 1000.0f;
	
	
	if(gimbal.feedback_alpha_speed_input == 0)
		vision_mpc_k = 1.0f;
	else
		vision_mpc_k = 0.75f;
	
	gimbal.yaw_spd.ref = pid_calc(&gimbal.yaw_angle.pid, gimbal.yaw_angle.fdb + yaw_err, gimbal.yaw_angle.fdb) \
                                + vision_mpc_k * gimbal.vision_velocity  + gimbal.feedback_alpha_speed_input * kkkkk ;
	//起身先转pitch yaw阻尼
//	if((!(fabsf(gimbal.pit_angle.ref - gimbal.pit_angle.fdb) < 0.08f) && !gimbal.start_up) && gimbal.start_cnt < 200)
//		 gimbal.yaw_spd.ref = 0.0f;
	
    gimbal.yaw_spd.fdb = gimbal_imu.wz;
    gimbal.yaw_output = pid_calc(&gimbal.yaw_spd.pid, gimbal.yaw_spd.ref, gimbal.yaw_spd.fdb);//SMC
//	//起身先转pitch
//	if((!(fabsf(gimbal.pit_angle.ref - gimbal.pit_angle.fdb) < 0.03f) && !gimbal.start_up) && gimbal.start_cnt < 200)
//		gimbal.yaw_output = 0.0f;
		
//		if (gimbal.start_up == 0 && fabs(yaw_err)  > 0.03f)//-------------------->无云台控制下注释
//       gimbal.yaw_output  *= 0.1f;
    
//    //无云台控制------->无陀螺仪, 通过ecd控制
//	if( rc_fsm_check(RC_LEFT_LD) && (!(rc_fsm_check(RC_RIGHT_RD))))
//	{  //  	调弹道
//		gimbal.yaw_ecd.fdb = (float)yaw_motor.ecd / 8192 * 2  * PI;
//		yaw_err = circle_error(gimbal.yaw_ecd.ref, gimbal.yaw_ecd.fdb, 2*PI);
//		gimbal.yaw_spd_ecd.ref = pid_calc(&gimbal.yaw_ecd.pid, gimbal.yaw_ecd.fdb + yaw_err, gimbal.yaw_ecd.fdb);
//			
//		gimbal.yaw_spd_ecd.fdb = gimbal_imu.wz;
//		gimbal.yaw_output = pid_calc(&gimbal.yaw_spd_ecd.pid, gimbal.yaw_spd_ecd.ref, gimbal.yaw_spd_ecd.fdb);
//	}
		
	last_yaw_ref = gimbal.yaw_angle.ref;
	last_pit_ref = gimbal.pit_angle.ref;
//		    if (gimbal.start_up == 0 && yaw_err < 0.03f) //无云台时起身判断
//        gimbal.start_up = 1;    

}

static void gimbal_data_output(void)
{
    if(gimbal_imu.online == 0 || yaw_motor.online == 0){
		dji_motor_set_torque(&yaw_motor, 0);     
		gimbal.yaw_output = 0.0f;
		dm_motor_set_control_para(&pit_motor,0,0,0,0,0);
    }       
    else{
		dji_motor_set_torque(&yaw_motor, -1.0f*gimbal.yaw_output);
		dm_motor_set_control_para(&pit_motor,0,0,0,0,1.0f*gimbal.pit_output);
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
				gimbal.vision_velocity = vision.rx[0].data.yaw_vel;
            }
            break;
        }
        case FIRST_LOST: {//首次丢失
            vision.aim_status = UNAIMING;
            gimbal.pit_angle.ref = gimbal.pit_angle.fdb;
            gimbal.yaw_angle.ref = gimbal.yaw_angle.fdb;
			gimbal.vision_velocity = 0.0f;
            break;
        }
        case UNAIMING: {//未识别到目标
			gimbal.vision_velocity = 0.0f;
            if (ctrl_mode == REMOTER_MODE) {
                gimbal.pit_angle.ref += rc.ch2 * gimbal_scale.angle_remote;
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
    for(;;) {
        thread_wake_time = osKernelSysTick();
        taskENTER_CRITICAL();
        switch (ctrl_mode) {
            case PROTECT_MODE: {	
                gimbal.start_up = 0;//保护模式下，起身标志位置零
				gimbal.start_cnt = 0;//保护模式下，起身计数置零
                gimbal.yaw_ecd.ref = (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI;
				yaw_err_up = circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, (float)yaw_motor.ecd / 8192 * 2 *PI, 2 * PI);
				
				if(!gimbal.start_up)
				{
					if(fabsf(yaw_err_up) < PI / 2.0f)
						gimbal.yaw_angle.ref = gimbal_imu.yaw + (float)yaw_motor.ecd / 8192 * 2 * PI -  (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI;//yaw轴反馈值+电机与前方灯条差值
					else
						gimbal.yaw_angle.ref = gimbal_imu.yaw + (float)yaw_motor.ecd / 8192 * 2 * PI -  (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI - PI;	
				}
                gimbal.pit_angle.ref = 0.3f;
                gimbal.pit_output = 0;
                gimbal.yaw_output = 0;
                break;
            }
            case REMOTER_MODE: {
				//只开启视觉且右拨杆不在上 或 开启视觉并且注释底盘 时云台听视觉
                if ((rc_fsm_check(RC_RIGHT_RD) && rc.sw2 != RC_UP) || (rc_fsm_check(RC_RIGHT_RD) && rc_fsm_check(RC_LEFT_LD))){ 
                    gimbal_get_vision_data();
//					gimbal.yaw_angle.pid.out_max = 5.0f;
                } 
                else {
//					gimbal.yaw_angle.pid.out_max = 5.0f;
//                   gimbal.yaw_ecd.ref   -= rc.ch1 * gimbal_scale.ecd_remote;
                   gimbal.pit_angle.ref -= -rc.ch2 * gimbal_scale.angle_remote;
                   gimbal.yaw_angle.ref -= rc.ch1 * gimbal_scale.angle_remote;
                }
				if(!gimbal.start_up)
					gimbal.start_cnt++;
				else
					gimbal.start_cnt = 0;
                gimbal_pid_calc();
                break;
            }
            case KEYBOARD_MODE: {
                if (rc.mouse.r == 1) {
                    gimbal_get_vision_data();
					gimbal.yaw_angle.pid.out_max = 5.0f;
                } else {
					gimbal.yaw_angle.pid.out_max = 100.0f;
                    //一键调头
                    if(key_scan_clear(KEY_GIMBAL_TURN_R)) {
                        gimbal.yaw_angle.ref -= PI / 2;
                    } 
                    else if (key_scan_clear(KEY_GIMBAL_TURN_L)) {
                        gimbal.yaw_angle.ref += PI / 2;
                    }
                    gimbal.pit_angle.ref += rc.mouse.y * gimbal_scale.angle_keyboard;
                    gimbal.yaw_angle.ref -= rc.mouse.x * gimbal_scale.angle_keyboard;
                }
								
                gimbal_pid_calc();
                break;
            }
            default:break;
        }
        
        gimbal_data_output();
        status.task.gimbal = 1;
        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 1);
    }
}
