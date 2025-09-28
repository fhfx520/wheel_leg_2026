#include "comm_task.h"
#include "mode_switch_task.h"
#include "cmsis_os.h"
#include "prot_vision.h"
#include "prot_power.h"
#include "drv_dji_motor.h"
#include "drv_dm_motor.h"
#include "prot_dr16.h"
#include "status_task.h"

uint32_t board_send_cnt;
void comm_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        thread_wake_time = osKernelSysTick();
			
        taskENTER_CRITICAL();
        status.task.comm = 1;
        dji_motor_output_data();
        
        for (int i = 0; i < 2; i++){
            if( joint_motor[i].state ==0 ){
                dm_motor_set_control_cmd(&joint_motor[i], CMD_ENABLE_MODE);	    
            }else{
				dm_motor_output_single_data(&joint_motor[i]);   
            }        
        }        
        
        taskEXIT_CRITICAL();		
        osDelayUntil(&thread_wake_time, 1);		
        
        taskENTER_CRITICAL();
        
         for (int i = 2; i < 4; i++){
            if( joint_motor[i].state == 0 ){
                dm_motor_set_control_cmd(&joint_motor[i], CMD_ENABLE_MODE);	 
            }else{
            dm_motor_output_single_data(&joint_motor[i]);   
            } 
        }
		board_send_cnt++;
				
		if(board_send_cnt % 2 == 0)
			gimbal_stable_output_data();
				
        if(board_send_cnt % 17 == 0){
			if (ctrl_mode == REMOTER_MODE || ctrl_mode == PROTECT_MODE)
				dr16_output_data();
			else if (ctrl_mode == KEYBOARD_MODE)
				kb_output_data();
        }
		
		if(board_send_cnt % 19 == 0)
			shoot_output_data();
		
		if(board_send_cnt % 32 == 0)
			imu_output_data();
		
		if(board_send_cnt % 37 == 0)
			judge_output_data();

        taskEXIT_CRITICAL();
			
        osDelayUntil(&thread_wake_time, 1);
        
    }
}
