#include "status_task.h"
#include "control_def.h"
#include "cmsis_os.h"
#include "mode_switch_task.h"
#include "chassis_task.h"
#include "wlr.h"
#include "drv_dji_motor.h"
#include "drv_dm_motor.h"
#include "prot_dr16.h"
#include "prot_imu.h"
#include "prot_judge.h"
#include "prot_power.h"
#include "prot_vision.h"
#include "drv_ws2812b.h"
#include "iwdg.h"
#include "robot_logic.h"
#include "container.h"
#include "board_comm.h"

status_t status;

void status_task(void const* argument)
{
//	MX_IWDG1_Init();
    for(;;)
    {
        if (status.task.comm == 1 &&
            status.task.chassis == 1 &&
            status.task.shoot == 1 &&
            status.task.mode_switch == 1) {
            status.task.comm = 0;
            status.task.chassis = 0;
            status.task.shoot = 0;
            status.task.mode_switch = 0;
            HAL_IWDG_Refresh(&hiwdg1);
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
        
        if (status.remote == 1 && status.vision == 1 && status.judge == 1 && \
            status.power == 1 && status.imu == 1 && status.dji_motor == 0 && \
            status.dm_motor == 1) {
            status.all = 1;
        } else {
            status.all = 0;
        }
        
        if (status.dm_motor == 1) {
            chassis.init = 1;
        }
        osDelay(33);
    }
}
