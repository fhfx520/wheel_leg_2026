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

//启用新拨盘4005
	start_MG4005_motor(&hfdcan3,SHOOT_CAN_ID_MG4005);//0x142
	dm_motor_set_control_cmd(&pit_motor, CMD_RESET_MODE);

//	dm_motor_init(&trigger_motor_2325, CAN_CHANNEL_3, 0x01, 0.0f, 0x00);//B 3.480
    for (;;)
    {
        thread_wake_time = osKernelSysTick();

        taskENTER_CRITICAL();
        status.task.comm = 1;
        dji_motor_output_data();
        board_comm_send_data();		//板间通信
		
 		if(pit_motor.state == 0 ){
			dm_motor_set_control_cmd(&pit_motor, CMD_ENABLE_MODE);	 
		}
		else
			dm_motor_output_single_data(&pit_motor);
		
	
        if(vision_cnt++ % 2 == 0)
         vision_output_data();
				
		if(ctrl_mode == PROTECT_MODE){
			MG4005_Toq_Ctrl(&hfdcan3,SHOOT_CAN_ID_MG4005,0);//电流范围是-2048~2048（超过之后就是上限正数逆时针转）
//			dm_motor_set_control_para(&pit_motor,0,0,0,0,0);//力矩范围是+-1.35，峰值为5
//			dm_motor_output_single_data(&pit_motor);   

		}else{
			MG4005_Toq_Ctrl(&hfdcan3,SHOOT_CAN_ID_MG4005,trigger_motor_MG4005.pos_out);//电流范围是-2048~2048（超过之后就是上限正数逆时针转）

//			dm_motor_set_control_para(&pit_motor,0,0,0,0,0);//力矩范围是+-1.35，峰值为5
//						dm_motor_set_control_para(&dm_motor2325,P_test,0,0,0,T_test);//力矩范围是+-1.35，峰值为5

//			if( trigger_motor_2325.state == 0 ){
//				dm_motor_set_control_cmd(&trigger_motor_2325, CMD_ENABLE_MODE);	 
//			}
//			else
//				dm_motor_output_single_data(&trigger_motor_2325);   
		}
		

				
        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 1);


//        taskENTER_CRITICAL();
//        vision_output_data();
//        taskEXIT_CRITICAL();

//        osDelayUntil(&thread_wake_time, 1);
    }
}
