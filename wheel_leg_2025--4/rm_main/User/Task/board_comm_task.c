#include "board_comm_task.h"
#include "cmsis_os.h"
#include <string.h>
#include "board_comm.h"
#include "container.h"
#include "container_bus.h"


void board_comm_task(const void *argu)
{
	uint32_t thread_wake_time = osKernelSysTick();
	for(;;)
	{
		taskENTER_CRITICAL();
		
		fdcan_board_comm_send();//set container

		container_bus_poll();//contain_bus 

		taskEXIT_CRITICAL();
		osDelayUntil(&thread_wake_time,5);
	}
}



