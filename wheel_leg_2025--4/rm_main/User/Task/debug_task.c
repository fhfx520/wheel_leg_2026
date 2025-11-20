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
uint8_t debug_wave = 1;
float test_hex = 1;
extern FGT_sin_t FGT_sin_chassis;
extern  uint16_t quadrant_cnt;
extern ctrl_mode_e  ctrl_mode;
extern float x3_balance_zero;
void log_scope_data_pkg(void)
{
    switch(debug_wave) {
        case 1: {//腿长PID调试
//            log_scope_get_data(wlr.wz_ref);
//            log_scope_get_data(wlr.wz_fdb);            
//            log_scope_get_data(ro_temp);
//			log_scope_get_data(chassis.output.vy);
//            
//            log_scope_get_data(wlr.side[0].T0);
//			log_scope_get_data(wlr.side[1].T0);
//			log_scope_get_data(vmc[0].L_fdb);
//			log_scope_get_data((float)wlr.sky_cnt);
//			log_scope_get_data((float)wlr.sky_flag);
//			log_scope_get_data(vmc[0].L_fdb);
//			log_scope_get_data(vmc[1].L_fdb);
//			log_scope_get_data(joint_motor[0].online);
//			log_scope_get_data(joint_motor[1].online);
//			log_scope_get_data(joint_motor[2].online);
//			log_scope_get_data(joint_motor[3].online);
//			log_scope_get_data(vmc[0].quadrant);
//			log_scope_get_data(vmc[1].quadrant);
//			log_scope_get_data(wlr.side[1].T1);
//			log_scope_get_data(wlr.side[1].T2);
			
//                log_scope_get_data(vmc[0].quadrant);
//                log_scope_get_data(vmc[1].quadrant);
//                log_scope_get_data(chassis.rescue_cnt_L);
//                log_scope_get_data(chassis.rescue_cnt_R);            
//                log_scope_get_data(chassis.recover_flag);
			log_scope_get_data(wlr.sky_flag);
            log_scope_get_data(wlr.side[0].t1);
            log_scope_get_data(wlr.side[0].t2);
            log_scope_get_data(wlr.side[1].t1);
            log_scope_get_data(wlr.side[1].t2);   
            
			log_scope_get_data(wlr.side[0].Fy);
			log_scope_get_data(wlr.side[1].Fy);
			log_scope_get_data(wlr.side[0].T0);
			log_scope_get_data(wlr.side[1].T0);
				
//			log_scope_get_data(wlr.wz_ref);   
//			log_scope_get_data(wlr.wz_fdb);
//			log_scope_get_data(wlr.jump_flag );             
//			log_scope_get_data(wlr.side[0].T0);  
//			log_scope_get_data(lqr.U_ref[0]);  
//			log_scope_get_data(tlm.l_ref[1]);
//			log_scope_get_data(chassis_imu.rol);
//          log_scope_get_data( vmc[0].L_fdb);
//          log_scope_get_data( vmc[1].L_fdb);
//		  	log_scope_get_data(wlr.high_set);
//			log_scope_get_data(wlr.side[0].fly_flag);  
//			log_scope_get_data(wlr.side[1].fly_flag);  
//			log_scope_get_data(wlr.sky_flag);  
					
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
            log_scope_get_data(wlr.jump_flag);
            log_scope_get_data(wlr.high_set);
            log_scope_get_data(wlr.side[0].fly_flag);
            log_scope_get_data(wlr.side[1].fly_flag);
        	 log_scope_get_data(vmc[0].L_fdb);
			 log_scope_get_data(vmc[1].L_fdb); 
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
//            log_scope_get_data(wlr.s_ref);
//            log_scope_get_data(wlr.s_fdb);
//            log_scope_get_data(wlr.v_ref);
//            log_scope_get_data(wlr.v_fdb);
//            log_scope_get_data(wlr.yaw_ref);
//            log_scope_get_data(wlr.yaw_fdb);
//					  log_scope_get_data(wlr.side[0].predict_wy + wlr.side[1].predict_wy);
//            log_scope_get_data(wlr.jump_flag);
//            log_scope_get_data(ctrl_mode);
//            log_scope_get_data(chassis.recover_flag);
//            log_scope_get_data(joint_motor[0].online );
//					log_scope_get_data(joint_motor[1].online );
//					log_scope_get_data(joint_motor[2].online );
//					log_scope_get_data(joint_motor[3].online );
//						log_scope_get_data(driver_motor[0].online);
//						log_scope_get_data(driver_motor[1].online );
//			log_scope_get_data(vision_send_time);
//					log_scope_get_data(wlr.high_set);
//					log_scope_get_data(wlr.high_flag);
					log_scope_get_data(gimbal_stable.feedback_alpha_speed);
					
            break;
        } case 10: {//状态预测
            log_scope_get_data(wlr.side[0].predict_wy);
            log_scope_get_data(-wlr.side[0].wy);
            log_scope_get_data(wlr.side[1].predict_wy);
            log_scope_get_data(-wlr.side[1].wy);
            break;
        } case 11: {
//            log_scope_get_data(driver_motor[0].tx_current);
//            log_scope_get_data(driver_motor[0].rx_current);
//            log_scope_get_data(driver_motor[1].tx_current);
//            log_scope_get_data(driver_motor[1].rx_current);
            
//            log_scope_get_data(driver_motor[0].position);
//            log_scope_get_data(driver_motor[1].position);
//            log_scope_get_data(joint_motor[0].position);
//            log_scope_get_data(joint_motor[1].position);
            log_scope_get_data(wlr.side[0].t1);
            log_scope_get_data(wlr.side[0].t2);
            log_scope_get_data(-wlr.side[1].t1);
            log_scope_get_data(-wlr.side[1].t2);

			log_scope_get_data(supercap.volume_percent);
//			log_scope_get_data(lqr.U_ref[1]);
					
					
			
            break;
        } case 12: {//底盘功率模型
            log_scope_get_data(power_heat_data.chassis_power);
//            log_scope_get_data(power_control.total_power);
//            log_scope_get_data(power_control.scaled_power);
            log_scope_get_data(power_heat_data.chassis_voltage);
            log_scope_get_data(power_heat_data.buffer_energy);
            log_scope_get_data(supercap.volage);
            log_scope_get_data(power_control.power_scale);
            break;
        } case 13: {//超电调试
//            log_scope_get_data(supercap.state.cap_v_over);
//            log_scope_get_data(supercap.state.cap_v_low);
//            log_scope_get_data(supercap.state.bat_v_over);
//            log_scope_get_data(supercap.state.bat_v_low );
//            log_scope_get_data(supercap.state.cap_i_over);
//            log_scope_get_data(supercap.state.chassis_i_over);
//            log_scope_get_data(supercap.state.chassis_msg_miss);
//            log_scope_get_data(supercap.state.judge_msg_miss);
            
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
                
//                log_scope_get_data(driver_motor[0].t);
//                log_scope_get_data(driver_motor[0].torque);
//                log_scope_get_data(driver_motor[1].t);
//                log_scope_get_data(driver_motor[1].torque);
                
//                log_scope_get_data(chassis_imu.rol);
//                log_scope_get_data(chassis_imu.pit);
//                log_scope_get_data(chassis_imu.yaw);
//                log_scope_get_data(chassis_imu.wy);
//                log_scope_get_data(-wlr.wy_fdb);
//                log_scope_get_data(chassis_imu.wz);
                break;
				} case 15: {// ------------------------------------观测6个电机力矩
                log_scope_get_data(joint_motor[0].t);
                log_scope_get_data(joint_motor[1].t);
                log_scope_get_data(joint_motor[2].t);
                log_scope_get_data(joint_motor[3].t);
                
                log_scope_get_data(driver_motor[0].t);
                log_scope_get_data(driver_motor[1].t);
                
                break;
            } case 16: {
//                log_scope_get_data(wlr.high_flag);
//                log_scope_get_data(wlr.jump_flag);
//                log_scope_get_data(lqr.K[row_debug +0] * lqr.X_diff[0]);
//                log_scope_get_data(lqr.K[row_debug +1] * lqr.X_diff[1]);   
//                log_scope_get_data(lqr.K[row_debug +2] * lqr.X_diff[2]);
//                log_scope_get_data(lqr.K[row_debug +3] * lqr.X_diff[3]);   
//                log_scope_get_data(lqr.K[row_debug +4] * lqr.X_diff[4]);
//                log_scope_get_data(lqr.K[row_debug +5] * lqr.X_diff[5]);   
//                log_scope_get_data(lqr.K[row_debug +6] * lqr.X_diff[6]);
//                log_scope_get_data(lqr.K[row_debug +7] * lqr.X_diff[7]);   
//                log_scope_get_data(lqr.K[row_debug +8] * lqr.X_diff[8]);
//                log_scope_get_data(lqr.K[row_debug +9] * lqr.X_diff[9]);
							
							
                log_scope_get_data(lqr.U_ref[0]);
                log_scope_get_data(lqr.U_ref[1]);
							  log_scope_get_data( vmc[0].T_ref.e.T1_ref);
                log_scope_get_data( vmc[0].T_ref.e.T2_ref);
								log_scope_get_data( vmc[1].T_ref.e.T1_ref);
								log_scope_get_data( vmc[1].T_ref.e.T2_ref);
							
							
//                log_scope_get_data(wlr.side[0].Fy);
//                log_scope_get_data(wlr.side[0].T0);
//                log_scope_get_data(wlr.side[1].Fy);
//                log_scope_get_data(wlr.side[1].T0);
                
//                log_scope_get_data(wlr.side[0].Fn_kal);
//                log_scope_get_data(wlr.side[0].fly_cnt);
//                log_scope_get_data(wlr.side[1].Fn_kal);
//                log_scope_get_data(wlr.side[1].fly_cnt);
                    
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
