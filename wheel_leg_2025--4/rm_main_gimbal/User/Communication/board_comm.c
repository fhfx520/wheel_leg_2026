#include "board_comm.h"
#include "string.h"
#include "prot_dr16.h"
#include "mode_switch_task.h"
#include "prot_imu.h"
#include "can_comm.h"
#include "prot_vision.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "prot_judge.h"
#include "status_task.h"
#include "stdlib.h"
#include "math_lib.h"
#include "prot_vtm.h"
#include "container.h"
#include "drv_dji_motor.h"
fdcan_board_comm_t fdcan_board_comm;
uint8_t board_comm_tx_buff[8];

rc_data_t rc_data_rec;
kb_data_t kb_data_rec;
shoot_data_t shoot_data_rec;
imu_data_t imu_data_rec;
judge_data_t judge_data_rec;
vision_data_t vision_data_rec;
gimbal_data_t gimbal_data_rec;

//fdcan板间通信下发数据
void fdcan_board_comm_send(void)
{
	//vtm遥控数据
	memcpy(&fdcan_board_comm.tx_msg.e.data_remote,&vtm.vtm_data.remote_data,sizeof(vtm.vtm_data.remote_data));
//	//图传链路键鼠数据
	memcpy(&fdcan_board_comm.tx_msg.e.data_keyboard.mouse_data,&vtm.vtm_data.mouse_data,sizeof(vtm.vtm_data.mouse_data));
	fdcan_board_comm.tx_msg.e.data_keyboard.key_code = vtm.vtm_data.kb.key_code;
	//云台数据
	fdcan_board_comm.tx_msg.e.gimbal_data.yaw_output = -1.0f * gimbal.yaw_output;
	fdcan_board_comm.tx_msg.e.gimbal_data.gimbal_start_up = gimbal.start_up;
	//视觉数据
	fdcan_board_comm.tx_msg.e.vision_data.vision_enanle = vision.shoot_enable;
	if(vision.aim_status == AIMING)
		fdcan_board_comm.tx_msg.e.vision_data.vision_trace_id = vision.trace_id;
	else
		fdcan_board_comm.tx_msg.e.vision_data.vision_trace_id = 0;
	//在线状态数据
	fdcan_board_comm.tx_msg.e.online_data.vision_online = status.vision;
	fdcan_board_comm.tx_msg.e.online_data.pit_online = status.dm_motor;
	fdcan_board_comm.tx_msg.e.online_data.imu_online = status.imu;
	fdcan_board_comm.tx_msg.e.online_data.vtm_online = status.vtm;
	fdcan_board_comm.tx_msg.e.online_data.fric_online = 
	((fric_motor[0].online && fric_motor[1].online) ? 0 : ((fric_motor[0].online) ? 1 : 2));
	//联合体数组发送
	can_std_transmit(CAN_CHANNEL_1,FDCAN_GIMBAL_TO_CHA_ID,fdcan_board_comm.tx_msg.buff);
	
}

//fdcan板间通信收数据
void fdcan_board_comm_get(uint32_t id, uint8_t *data)
{
	if(id == FDCAN_CHA_TO_GIMBAL_ID)
	{
		//联合体内存拷贝
		memcpy(fdcan_board_comm.rx_msg.buff,data,FDCAN_BOARD_DATA_LEN);
		//数据分发
		memcpy(&rc_data_rec,&fdcan_board_comm.rx_msg.e.rc_data,sizeof(rc_data_rec));
		memcpy(&kb_data_rec,&fdcan_board_comm.rx_msg.e.kb_data,sizeof(kb_data_rec));
		memcpy(&imu_data_rec,&fdcan_board_comm.rx_msg.e.imu_data,sizeof(imu_data_rec));
		memcpy(&judge_data_rec,&fdcan_board_comm.rx_msg.e.judge_data,sizeof(judge_data_rec));
		memcpy(&vision_data_rec,&fdcan_board_comm.rx_msg.e.vision_data,sizeof(vision_data_rec));
		memcpy(&gimbal_data_rec,&fdcan_board_comm.rx_msg.e.gimbal_data,sizeof(gimbal_data_rec));
		
		robot_status.robot_id = judge_data_rec.camp;
		fdcan_board_comm.last_rx_tick = HAL_GetTick();
	}
}

uint8_t board_comm_check_offline(void)
{
    uint32_t current_tick = HAL_GetTick();
    if (fdcan_board_comm.last_rx_tick != 0 && (uint32_t)(current_tick - fdcan_board_comm.last_rx_tick) <= BOARD_COMM_OFFLINE_TIMEOUT_MS) {
        fdcan_board_comm.online = 1;
    } else {
        fdcan_board_comm.online = 0;
    }
	return fdcan_board_comm.online;
}
