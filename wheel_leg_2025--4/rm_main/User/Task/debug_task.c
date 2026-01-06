#include "debug_task.h"
#include "cmsis_os.h"
#include "data_log.h"
#include "stdint.h"
#include "prot_vision.h"
#include "shoot_task.h"
#include "wlr.h"
#include "leg_vmc.h"
#include "wheel_leg_model.h"
#include "drv_dji_motor.h"
#include "drv_dm_motor.h"
#include "prot_judge.h"
#include "prot_power.h"
#include "prot_imu.h"
#include "kalman_filter.h"
#include "func_generator.h"
#include "prot_dr16.h"
#include "prot_judge.h"
#include "us_time.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "mode_switch_task.h"

#define row_debug 2 * 10
uint8_t debug_wave = 7;
float test_hex = 1;
extern FGT_sin_t FGT_sin_chassis;
extern  uint16_t quadrant_cnt;
extern ctrl_mode_e  ctrl_mode;
extern float x3_balance_zero;
extern float real_vel;
extern float yw_ddot;
extern float Fwy;
extern float F_test[2];
extern float F_wy[2];
void log_scope_data_pkg(void)
{
    switch(debug_wave) {
        case 1: {//LQR
			
			log_scope_get_data(lqr.U_ref[0]);
			log_scope_get_data(lqr.U_ref[1]);
//			log_scope_get_data(lqr.U_ref[2]);
//			log_scope_get_data(lqr.U_ref[3]);
			log_scope_get_data(lqr.X_diff[0] * -1.41378f);
			log_scope_get_data(lqr.X_diff[1] * -2.99569);
			log_scope_get_data(lqr.X_diff[2] * -2.14463);
			log_scope_get_data(lqr.X_diff[3] * -1.56938);
			log_scope_get_data(lqr.X_diff[4] * -12.8504);
			log_scope_get_data(lqr.X_diff[5] * -1.61767);
			log_scope_get_data(lqr.X_diff[6] * -5.38904);
			log_scope_get_data(lqr.X_diff[7] * -0.708452);
			log_scope_get_data(lqr.X_diff[8] * 0);
			log_scope_get_data(lqr.X_diff[9] * 0);
			log_scope_get_data(wlr.sky_flag);
//			log_scope_get_data(lqr.X_diff[4]);
//			log_scope_get_data(lqr.X_diff[5]);
//			log_scope_get_data(lqr.X_diff[6]);
//			log_scope_get_data(lqr.X_diff[7]);
//			log_scope_get_data(lqr.X_diff[8]);
//			log_scope_get_data(lqr.X_diff[9]);
            break;
        } case 2: {//拨盘pid调试
            log_scope_get_data(shoot.trigger_spd.ref);
            log_scope_get_data(shoot.trigger_spd.fdb);
            log_scope_get_data(shoot.trigger_ecd.ref);
            log_scope_get_data(shoot.trigger_ecd.fdb);
//          log_scope_get_data(shoot.trigger_output);
//          log_scope_get_data(trigger_motor.tx_current);
//          log_scope_get_data(wlr.side[0].Tw);
//          log_scope_get_data(wlr.side[1].Tw);
//          log_scope_get_data(wlr.side[0].T_adapt);
//          log_scope_get_data(wlr.side[1].T_adapt);            

            break;
        } case 3: {//底盘yaw roll调试 + 向心力补偿调试
//          log_scope_get_data(wlr.wz_ref);
//          log_scope_get_data(wlr.wz_fdb);
//          log_scope_get_data(wlr.yaw_fdb + wlr.yaw_err);
//          log_scope_get_data(wlr.yaw_fdb);
            
            log_scope_get_data(wlr.inertial_offs);
            log_scope_get_data(wlr.side[0].Fy);
            log_scope_get_data(wlr.side[1].Fy);
            log_scope_get_data(wlr.yaw_err);
            log_scope_get_data(chassis_imu.rol);            
            break;
        } case 4: {//底盘功率调试
//          log_scope_get_data(power_heat_data.chassis_power);
//          log_scope_get_data(power_heat_data.buffer_energy);
//          log_scope_get_data(supercap.volage);
            log_scope_get_data(robot_status.chassis_power_limit);
            log_scope_get_data( power_heat_data.chassis_power);
            break;
        } case 5: {//支持力调试
//          log_scope_get_data(wlr.side[0].Fn_fdb);
            log_scope_get_data(wlr.side[0].Fn_kal);
//          log_scope_get_data(wlr.side[0].fly_cnt);
//          log_scope_get_data(wlr.side[1].Fn_fdb);
            log_scope_get_data(wlr.side[1].Fn_kal);
//          log_scope_get_data(wlr.side[1].fly_cnt);
            break;
        } case 6: {//上台阶测试
//            log_scope_get_data(chassis_imu.pit);
//            log_scope_get_data(wlr.jump_flag);
//            log_scope_get_data(lqr.X_fdb[4]);
//            log_scope_get_data(tlm.l_ref[0]);
//						log_scope_get_data(tlm.l_ref[1]);
//						log_scope_get_data(vmc[0].L_fdb);
//						log_scope_get_data(vmc[1].L_fdb); 
//						log_scope_get_data(chassis_imu.rol  * 180 / PI);
			log_scope_get_data(lqr.U_ref[0]);
			log_scope_get_data(lqr.U_ref[1]);
            break;
        } case 7: {//离地检测测试
//            log_scope_get_data(wlr.high_set);
            log_scope_get_data(wlr.side[0].fly_flag);
            log_scope_get_data(wlr.side[1].fly_flag);
			log_scope_get_data(wlr.side[0].fly_cnt);
            log_scope_get_data(wlr.side[1].fly_cnt);
			log_scope_get_data(wlr.side[0].Fn_kal);
            log_scope_get_data(wlr.side[1].Fn_kal);
			log_scope_get_data(chassis_imu.az);
			log_scope_get_data(F_test[0]);
			log_scope_get_data(F_test[1]);
			log_scope_get_data(F_wy[0]);
			log_scope_get_data(F_wy[1]);
			log_scope_get_data(vmc[0].F_fdb.e.Fy_fdb);
			log_scope_get_data(vmc[1].F_fdb.e.Fy_fdb);
			log_scope_get_data(vmc[0].F_ref.e.Fy_ref);
			log_scope_get_data(vmc[1].F_ref.e.Fy_ref);
//			log_scope_get_data(wlr.side[0].predict_wy);
//            log_scope_get_data(wlr.side[0].wy);
//            log_scope_get_data(wlr.side[1].predict_wy);
//            log_scope_get_data(wlr.side[1].wy);
			
            break;
        } case 8: {//腿部力
            log_scope_get_data(wlr.side[0].Fy);
//            log_scope_get_data(wlr.side[0].T0);
            log_scope_get_data(wlr.side[1].Fy);
//            log_scope_get_data(wlr.side[1].T0);
//            
//            log_scope_get_data(wlr.side[0].Fy);
//            log_scope_get_data(wlr.side[0].T0);
//            log_scope_get_data(wlr.side[1].Fy);
//            log_scope_get_data(wlr.side[1].T0);
            break;
        } case 9: {//底盘状态观测
			log_scope_get_data(gimbal_stable.feedback_alpha_speed);
					
            break;
        } case 10: {//状态预测
            log_scope_get_data(wlr.side[0].predict_wy);
            log_scope_get_data(wlr.side[0].wy);
            log_scope_get_data(wlr.side[1].predict_wy);
            log_scope_get_data(wlr.side[1].wy);
			log_scope_get_data(wlr.side[0].T_adapt);
			log_scope_get_data(wlr.side[1].T_adapt);
			log_scope_get_data(lqr.U_ref[0]);
			log_scope_get_data(lqr.U_ref[1]);
            break;
        } case 11: {//飞天测试
			log_scope_get_data(wlr.sky_flag);
			log_scope_get_data(tlm.l_ref[0]);
			log_scope_get_data(tlm.l_ref[1]);
			log_scope_get_data(vmc[0].L_fdb);
			log_scope_get_data(vmc[1].L_fdb);
			log_scope_get_data(wlr.side[0].Fn_kal);
			log_scope_get_data(wlr.side[1].Fn_kal);
			log_scope_get_data(wlr.side[0].Fy);
			log_scope_get_data(wlr.side[1].Fy);
			log_scope_get_data(vmc[0].q_fdb[0]);
			log_scope_get_data(vmc[1].q_fdb[0]);
			log_scope_get_data(lqr.X_diff[9]);
			log_scope_get_data(lqr.U_ref[2]);
			log_scope_get_data(lqr.U_ref[3]);
			log_scope_get_data(lqr.X_diff[4]);
			log_scope_get_data(lqr.X_diff[5]);
            break;
        } case 12: {//底盘功率模型
            log_scope_get_data(power_heat_data.chassis_power);
            log_scope_get_data(power_heat_data.chassis_voltage);
            log_scope_get_data(power_heat_data.buffer_energy);
            log_scope_get_data(supercap.volage);
            log_scope_get_data(power_control.power_scale);
            break;
        } case 13: {//超电调试
            log_scope_get_data(supercap.volage);
            log_scope_get_data(supercap.current);
            break;
        } case 14: {//大喵电机力矩
			log_scope_get_data(joint_motor[0].t);
			log_scope_get_data(joint_motor[0].torque);
			log_scope_get_data(joint_motor[1].t);
			log_scope_get_data(joint_motor[1].torque);
			log_scope_get_data(joint_motor[2].t);
			log_scope_get_data(joint_motor[2].torque);
			log_scope_get_data(joint_motor[3].t);
			log_scope_get_data(joint_motor[3].torque);
			break;
		} case 15: {// ------------------------------------观测6个电机力矩
			log_scope_get_data(joint_motor[0].t);
			log_scope_get_data(joint_motor[1].t);
			log_scope_get_data(joint_motor[2].t);
			log_scope_get_data(joint_motor[3].t);
			
			log_scope_get_data(driver_motor[0].t);
			log_scope_get_data(driver_motor[1].t);
			break;
		} case 16: {//二阶卡尔曼滤波器观测	
			log_scope_get_data(kal_fusion_vel.filter_vector[1]);
			log_scope_get_data(real_vel);
			log_scope_get_data(wlr.v_fdb);
			log_scope_get_data(chassis_imu.ax);
			log_scope_get_data(kal_fusion_vel.xhatminus_data[1]); 
			log_scope_get_data(wlr.v_ref); 
			break;
		}
        default:break;
    }
}

/* 串口上位机数据发送任务 */
void debug_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        thread_wake_time = osKernelSysTick();
        log_scope_data_output();
        osDelayUntil(&thread_wake_time, 5);
    }
}
