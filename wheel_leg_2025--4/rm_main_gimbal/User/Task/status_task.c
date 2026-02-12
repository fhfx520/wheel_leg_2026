#include "status_task.h"
#include "control_def.h"
#include "cmsis_os.h"
#include "mode_switch_task.h"
#include "drv_dji_motor.h"
#include "drv_dm_motor.h"
#include "prot_dr16.h"
#include "prot_imu.h"
#include "prot_judge.h"
#include "prot_power.h"
#include "prot_vision.h"
#include "iwdg.h"
#include "board_comm.h"

status_t status;

int iwdg_test = 1;
void status_task(void const* argument)
{
//	MX_IWDG1_Init();
    for(;;)
    {
        if (status.task.comm == 1 &&
            status.task.gimbal == 1 &&
						//status.task.board == 1 &&				\测试架没有板间通信了，直接不要
            status.task.shoot == 1 &&
            status.task.mode_switch == 1) {
            status.task.comm = 0;
            status.task.gimbal = 0;
            status.task.chassis = 0;
            status.task.shoot = 0;
            status.task.mode_switch = 0;
            HAL_IWDG_Refresh(&hiwdg1);
        }

        rc_fsm_init(rc.online); //板间通信
				
				
        status.remote = rc_check_offline();
        status.vision = vision_check_offline();
        status.judge = judge_check_offline();
        status.power = power_check_offline();
				
				
//        status.imu = imu_check_offline();
//        status.dji_motor = dji_motor_check_offline();
        status.dm_motor = dm_motor_check_offline();
		status.task.board = 0;
        
        if (status.remote == 0 && status.vision == 0 && status.judge == 0 && \
            status.power == 0 && status.imu == 0 && status.dji_motor == 0 && \
            status.dm_motor == 0) {
            status.all = 0;
        } else {
            status.all = 1;
        }
//         HAL_IWDG_Refresh(&hiwdg1);
		board_comm.tx_vis_msg.data.vision_online = 0;
        osDelay(100);
    }
}
