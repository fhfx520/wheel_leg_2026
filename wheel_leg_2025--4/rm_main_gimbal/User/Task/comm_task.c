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


void comm_task(void const *argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    static uint32_t vision_cnt;
    for (;;)
    {
        thread_wake_time = osKernelSysTick();

        taskENTER_CRITICAL();
        status.task.comm = 1;
        dji_motor_output_data();
        board_comm_send_data();		//板间通信
        if(vision_cnt++ % 3 == 0)	//电控发数据频率是333帧
         vision_output_data();
				
        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 1);


//        taskENTER_CRITICAL();
//        vision_output_data();
//        taskEXIT_CRITICAL();

//        osDelayUntil(&thread_wake_time, 1);
    }
}
