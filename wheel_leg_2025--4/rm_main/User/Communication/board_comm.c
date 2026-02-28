#include "board_comm.h"
#include "string.h"
#include "container.h"
#include "can_comm.h"

fdcan_board_comm_t fdcan_board_comm;

data_keyboard_t data_keyboard_rec;
gimbal_data_t gimbal_data_rec;
vision_data_t vision_data_rec;

rc_data_t rc_data_tran;
kb_data_t kb_data_tran;
imu_data_t imu_data_tran;
judge_data_t judge_data_tran;
vision_tx_data_t vision_data_tran;
gimbal_tx_data_t gimbal_data_tran;


//fdcan板间通信上发数据
void fdcan_board_comm_send(void)
{
	//测试
//	static uint8_t ccc = 1;
//	for(uint8_t i = 0;i < 64;i++)
//	{
//		fdcan_board_comm.tx_msg.buff[i] = ccc;
//		ccc++;
//		if(ccc > 128)
//			ccc = 1;
//	}
	
}

//fdcan板间通信收数据
void fdcan_board_comm_get(uint32_t id,uint8_t *pdata)
{
	if(id == FDCAN_GIMBAL_TO_CHA_ID)
	{
		//联合体内存拷贝
		memcpy(&fdcan_board_comm.rx_msg.buff,pdata,FDCAN_BOARD_DATA_LEN);
		
		memcpy(&data_keyboard_rec,&fdcan_board_comm.rx_msg.data_keyboard,sizeof(data_keyboard_rec));

		memcpy(&gimbal_data_rec,&fdcan_board_comm.rx_msg.gimbal_data,sizeof(gimbal_data_rec));

		memcpy(&vision_data_rec,&fdcan_board_comm.rx_msg.vision_data,sizeof(vision_data_rec));
	}
}

void board_comm_container_set(void)
{
//	container_set(TAG_KEYBOARD_DATA, &data_keyboard_rec, sizeof(data_keyboard_rec), CONTAINER_TYPE_STRUCT);
//	container_set(TAG_GIMBAL_DATA,   &gimbal_data_rec,   sizeof(gimbal_data_rec),   CONTAINER_TYPE_STRUCT);
	container_set(TAG_VISION_DATA,   &vision_data_rec,   sizeof(vision_data_rec),   CONTAINER_TYPE_STRUCT);
	
	//所有发送信息打包
//	container_set(TAG_TX_ALL_DATA, fdcan_board_comm.tx_msg.buff, FDCAN_BOARD_DATA_LEN,CONTAINER_TYPE_INT);
}
