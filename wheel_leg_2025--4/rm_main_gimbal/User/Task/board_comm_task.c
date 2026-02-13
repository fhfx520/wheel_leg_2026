#include "board_comm_task.h"
#include "cmsis_os.h"
#include <string.h>
#include "board_comm.h"
	
void board_comm_task(const void *argu)
{
	uint32_t thread_wake_time = osKernelSysTick();
	for(;;)
	{
		taskENTER_CRITICAL();
		
		fdcan_board_comm_send();
		
		taskEXIT_CRITICAL();
		osDelayUntil(&thread_wake_time,2);
	}
}



