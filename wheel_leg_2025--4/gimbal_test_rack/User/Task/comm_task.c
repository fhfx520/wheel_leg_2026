#include "comm_task.h"
#include "mode_switch_task.h"
#include "cmsis_os.h"
#include "prot_vision.h"
#include "prot_power.h"
#include "drv_dji_motor.h"
#include "drv_dm_motor.h"
#include "prot_dr16.h"
#include "status_task.h"
#include "board_comm.h"

#include "bsp_LK_Motor_MG4005.h"
#include "drv_dm_motor.h"
#include "shoot_task.h"

void comm_task(void const *argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    static uint32_t vision_cnt;

//1111启用新拨盘4005
	start_MG4005_motor(&hfdcan3,SHOOT_CAN_ID_MG4005);//0x142
//	dm_motor_set_control_cmd(&pit_motor, CMD_RESET_MODE);
    for (;;)
    {
        thread_wake_time = osKernelSysTick();

        taskENTER_CRITICAL();
        status.task.comm = 1;
        dji_motor_output_data();
		
		//1111测试架pitch为6020，注释dm
// 		if(pit_motor.state == 0 ){
//			dm_motor_set_control_cmd(&pit_motor, CMD_ENABLE_MODE);	 
//		}
//		else
//			dm_motor_output_single_data(&pit_motor);
		
		//1111测试架yaw为6020开启
// 		if(yaw_motor.state == 0 ){
//			dm_motor_set_control_cmd(&yaw_motor, CMD_ENABLE_MODE);	 
//		}
//		else
//			dm_motor_output_single_data(&yaw_motor);
		
		//1111测试架拨盘电机MG4005，上车注释掉
		if(ctrl_mode == PROTECT_MODE){
			MG4005_Toq_Ctrl(&hfdcan3,SHOOT_CAN_ID_MG4005,0);//电流范围是-2048~2048（超过之后就是上限正数逆时针转）
		}else{
			MG4005_Toq_Ctrl(&hfdcan3,SHOOT_CAN_ID_MG4005,trigger_motor_MG4005.pos_out);//电流范围是-2048~2048（超过之后就是上限正数逆时针转）
		}

		
        if(vision_cnt++ % 2 == 0)
			vision_output_data();
//			superpower_vision_Tx();//同济视觉，待测试

					
        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 1);
    }
}
