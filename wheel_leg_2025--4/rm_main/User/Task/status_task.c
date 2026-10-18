#include "status_task.h"
#include "control_def.h"
#include "cmsis_os.h"
#include "mode_switch_task.h"
#include "chassis_task.h"
#include "wlr.h"
#include "drv_dji_motor.h"
#include "drv_dm_motor.h"
#include "drv_lk_motor.h"
#include "prot_dr16.h"
#include "prot_imu.h"
#include "prot_judge.h"
#include "prot_power.h"
#include "prot_vision.h"
#include "iwdg.h"
#include "robot_logic.h"
#include "container.h"
#include "board_comm.h"
#include "prot_tof.h"
#include "fdcan.h"
#include "status_log.h"
status_t status;
extern uint32_t last_rx_tick;

void status_task(void const* argument)
{
	MX_IWDG1_Init();
	StatusLog_Init(osKernelSysTick());
    for(;;)
    {
        if (status.task.comm == 1 &&
            status.task.chassis == 1 &&
            status.task.shoot == 1 &&
            status.task.mode_switch == 1 &&
			hfdcan3.ErrorCode == 0) {
            status.task.comm = 0;
            status.task.chassis = 0;
            status.task.shoot = 0;
            status.task.mode_switch = 0;
            /* The watchdog may be disabled during debugging. */
            if (hiwdg1.Instance == IWDG1) {
                HAL_IWDG_Refresh(&hiwdg1);
            }
        }

        rc_fsm_init(rc.online);
        status.remote = rc_check_offline();
        status.vision = vision_check_offline();
        status.judge = judge_check_offline();
        status.power = power_check_offline();
        status.imu = imu_check_offline();
		status.board_comm = board_comm_check_offline();
        status.dji_motor = dji_motor_check_offline();
        status.dm_motor = dm_motor_check_offline();
        status.lk_motor = lk_check_offline();
		tof_check_offline();
        
        if (status.remote == 1 && status.vision == 1 && status.judge == 1 && \
            status.power == 1 && status.imu == 1 && status.dji_motor == 0 && \
            status.dm_motor == 1 && status.board_comm == 1 && status.lk_motor == 1) {
            status.all = 1;
        } else {
            status.all = 0;
        }
        
        if (status.dm_motor == 1) {
            chassis.init = 1;
        }
		
		uint32_t now_rtos = osKernelSysTick();
		uint32_t now_hal = HAL_GetTick();
		StatusLog_Observe(LOG_MODULE_REMOTE,1,rc.online,rc.last_rx_tick,0,0,now_rtos,now_hal);
		StatusLog_Observe(LOG_MODULE_JUDGE,1,status.judge,last_rx_tick,0,0,now_rtos,now_hal);
		StatusLog_Observe(LOG_MODULE_SUPERCAP,1,power_control.online,power_control.last_rx_tick,CAN_CHANNEL_2,SUPERCAP_DATA_ID,now_rtos,now_hal);
		StatusLog_Observe(LOG_MODULE_CHASSIS_IMU,1,chassis_imu.online,chassis_imu.last_rx_tick,CAN_CHANNEL_2,IMU_ALL_ID,now_rtos,now_hal);
		StatusLog_Observe(LOG_MODULE_BOARD,1,fdcan_board_comm.online,fdcan_board_comm.last_rx_tick,CAN_CHANNEL_2,FDCAN_GIMBAL_TO_CHA_ID,now_rtos,now_hal);
		
		StatusLog_Observe(LOG_MODULE_DRIVER_0,1,driver_motor[0].online,driver_motor[0].last_rx_tick,CAN_CHANNEL_3,DRIVER_MOTOR_LEFT_ID,now_rtos,now_hal);
		StatusLog_Observe(LOG_MODULE_DRIVER_1,1,driver_motor[1].online,driver_motor[1].last_rx_tick,CAN_CHANNEL_3,DRIVER_MOTOR_RIGHT_ID,now_rtos,now_hal);
		StatusLog_Observe(LOG_MODULE_YAW,1,yaw_motor.online,yaw_motor.last_rx_tick,CAN_CHANNEL_3,YAW_MOTOR_ID,now_rtos,now_hal);
		StatusLog_Observe(LOG_MODULE_JOINT_0,1,joint_motor[0].online,joint_motor[0].last_rx_tick,CAN_CHANNEL_1,JOINT_LB_REC_ID,now_rtos,now_hal);
		StatusLog_Observe(LOG_MODULE_JOINT_1,1,joint_motor[1].online,joint_motor[1].last_rx_tick,CAN_CHANNEL_1,JOINT_LS_REC_ID,now_rtos,now_hal);
		StatusLog_Observe(LOG_MODULE_JOINT_2,1,joint_motor[2].online,joint_motor[2].last_rx_tick,CAN_CHANNEL_1,JOINT_RB_REC_ID,now_rtos,now_hal);
		StatusLog_Observe(LOG_MODULE_JOINT_3,1,joint_motor[3].online,joint_motor[3].last_rx_tick,CAN_CHANNEL_1,JOINT_RS_REC_ID,now_rtos,now_hal);
		StatusLog_Observe(LOG_MODULE_TRIGGER,1,trigger_motor.online,trigger_motor.last_rx_tick,CAN_CHANNEL_3,TRIGGER_MOTOR_ID,now_rtos,now_hal);
        osDelay(33);
    }
}
