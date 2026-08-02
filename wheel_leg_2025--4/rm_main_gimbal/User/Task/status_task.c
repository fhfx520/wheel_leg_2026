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
#include "prot_vtm.h"

status_t status;

int iwdg_test = 1;
void status_task(void const* argument)
{
//	MX_IWDG1_Init();
    for(;;)
    {
		
        if (status.task.comm == 1 &&
            status.task.gimbal == 1 &&
			status.task.board == 1 &&
            status.task.shoot == 1 &&
            status.task.mode_switch == 1 &&
			status.board == 1        ) {
            status.task.comm = 0;
            status.task.gimbal = 0;
            status.task.chassis = 0;
            status.task.shoot = 0;
            status.task.mode_switch = 0;
			status.task.board = 0;
            HAL_IWDG_Refresh(&hiwdg1);
        }
				
        status.vision = vision_check_offline();
        status.vtm = vtm_check_offline();	
        status.imu = imu_check_offline();
        status.dji_motor = dji_motor_check_offline();
        status.dm_motor = dm_motor_check_offline();
        status.board = board_comm_check_offline();

        if (status.vision == 1 && status.vtm == 1 && status.imu == 1 
            && status.dji_motor == 0 && status.dm_motor == 1 && status.board == 1) {
            status.all = 1;
        } else {
            status.all = 0;
        }
        osDelay(33);
    }
}
