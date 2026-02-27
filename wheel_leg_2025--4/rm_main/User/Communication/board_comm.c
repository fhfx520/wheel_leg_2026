#include "board_comm.h"
#include "string.h"
#include "container.h"
#include "can_comm.h"

fdcan_board_comm_t fdcan_board_comm;

data_keyboard_t data_keyboard_rec;
gimbal_data_t gimbal_data_rec;
vision_data_t vision_data_rec;


//fdcan板间通信上发数据
void fdcan_board_comm_send(void)
{
	container_set(TAG_KEYBOARD_DATA, &data_keyboard_rec, sizeof(data_keyboard_rec), CONTAINER_TYPE_STRUCT);
	container_set(TAG_GIMBAL_DATA,   &gimbal_data_rec,   sizeof(gimbal_data_rec),   CONTAINER_TYPE_STRUCT);
	container_set(TAG_VISION_DATA,   &vision_data_rec,   sizeof(vision_data_rec),   CONTAINER_TYPE_STRUCT);
	//测试
	static uint8_t ccc = 1;
	for(uint8_t i = 0;i < 64;i++)
	{
		fdcan_board_comm.tx_msg.buff[i] = ccc;
		ccc++;
		if(ccc > 128)
			ccc = 1;
	}
	can_std_transmit(CAN_CHANNEL_2,FDCAN_CHA_TO_GIMBAL_ID,fdcan_board_comm.tx_msg.buff);
}

//fdcan板间通信收数据
void fdcan_board_comm_get(uint32_t id,uint8_t *pdata)
{
	if(id == FDCAN_GIMBAL_TO_CHA_ID)
	{
		//联合体内存拷贝
		memcpy(&fdcan_board_comm.rx_msg.buff,pdata,FDCAN_BOARD_DATA_LEN);
		//待补充数据分发
		memcpy(&data_keyboard_rec,&fdcan_board_comm.rx_msg.data_keyboard,sizeof(data_keyboard_rec));

		memcpy(&gimbal_data_rec,&fdcan_board_comm.rx_msg.gimbal_data,sizeof(gimbal_data_rec));

		memcpy(&vision_data_rec,&fdcan_board_comm.rx_msg.vision_data,sizeof(vision_data_rec));

	}
}
