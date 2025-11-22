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
#include "mode_switch_task.h"

#define row_debug 2 * 10
uint8_t debug_wave = 1;
uint32_t aqdihakdhakjhdaukd;
uint32_t no_online_imu;

extern float feed_torque;
extern float velocity_err;
void log_scope_data_pkg(void)
{
    switch(debug_wave) {
        case 1: {//云台pid调试
//            log_scope_get_data(gimbal.yaw_spd.ref);
//            log_scope_get_data(gimbal.yaw_spd.fdb);
   
            log_scope_get_data(gimbal.yaw_angle.ref);
            log_scope_get_data(gimbal.yaw_angle.fdb);
					  log_scope_get_data(gimbal.pit_angle.ref);
            log_scope_get_data(gimbal.pit_angle.fdb);
//						log_scope_get_data(fabs( circle_error(vision.target_yaw_angle,gimbal.yaw_angle.fdb, 2*PI));
//            log_scope_get_data(yaw_motor.online);
//            
//            log_scope_get_data(aqdihakdhakjhdaukd);
//            log_scope_get_data(no_online_imu);
            
            break;
        } case 2: {//拨盘pid调试

            break;
        }
				case 3: {//发射器调试
						log_scope_get_data(shoot.fric_spd[0].fdb);
						log_scope_get_data(shoot.fric_spd[1].fdb);
            log_scope_get_data(shoot.fric_output[0]);
            log_scope_get_data(shoot.fric_output[1]);
            log_scope_get_data(feed_torque);
            log_scope_get_data(velocity_err);

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
        imu_check_offline();
        dji_motor_check_offline();

            
        osDelayUntil(&thread_wake_time, 5);
    }
}
