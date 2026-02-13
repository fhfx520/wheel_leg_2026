#include "board_comm.h"
#include "string.h"

fdcan_board_comm_t fdcan_board_comm;
//fdcan板间通信上发数据
void fdcan_board_comm_send(void)
{
	
}

//fdcan板间通信收数据
void fdcan_board_comm_get(uint32_t id,uint8_t *pdata)
{
	if(id == FDCAN_GIMBAL_TO_CHA_ID)
	{
		//联合体内存拷贝
		memcpy(&fdcan_board_comm.rx_msg.buff,pdata,FDCAN_BOARD_DATA_LEN);
		//待补充数据分发
		
	}
}
