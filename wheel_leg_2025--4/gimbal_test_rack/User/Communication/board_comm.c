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
board_comm_t board_comm;
fdcan_board_comm_t fdcan_board_comm;
uint8_t board_comm_tx_buff[8];

rc_data_t rc_data_rec;
kb_data_t kb_data_rec;
shoot_data_t shoot_data_rec;
imu_data_t imu_data_rec;
judge_data_t judge_data_rec;
vision_data_t vision_data_rec;
gimbal_data_t gimbal_data_rec;

uint8_t board_comm_online;

//1111板间通信要把这个改掉，变成直接遥控器输入在中断那里已经收到数据
static void rc_rc_decode(void){	
	if(ctrl_mode == REMOTER_MODE || ctrl_mode == PROTECT_MODE){
		rc.ch1 			 = board_comm.rx_rc_msg.data_remote.chl;
		rc.ch2 		 	 = board_comm.rx_rc_msg.data_remote.ch2;
		rc.sw1 			 = board_comm.rx_rc_msg.data_remote.sw1;
		rc.sw2 			 = board_comm.rx_rc_msg.data_remote.sw2;
		if(abs(rc.ch1) < 30)
			rc.ch1 = 0;
		if(abs(rc.ch2) < 30)
			rc.ch2 = 0;
		
	}else if(ctrl_mode == KEYBOARD_MODE){
		rc.mouse.x 			 = board_comm.rx_rc_msg.data_keyboard.x;
		rc.mouse.y 		 	 = board_comm.rx_rc_msg.data_keyboard.y;
		rc.mouse.l 			 = board_comm.rx_rc_msg.data_keyboard.l;
		rc.mouse.r 			 = board_comm.rx_rc_msg.data_keyboard.r;
		rc.kb.key_code   = board_comm.rx_rc_msg.data_keyboard.key_code;
	}

}

static void rc_cha_decode(void){
	if( fabs(board_comm.rx_cha_msg.data.cha_pit) < 1e-4 && board_comm.rx_cha_msg.data.ctrl_mode==0 )
		return ;
         
//	chassis_imu.pit = board_comm.rx_cha_msg.data.cha_pit;//1111 底盘pit，用于云台pit的上限和下限，给零
	chassis_imu.pit = 0;
	ctrl_mode 	    = board_comm.rx_cha_msg.data.ctrl_mode;//
	robot_status.robot_id  = board_comm.rx_cha_msg.data.camp;//robo id
	rc.init_status  = board_comm.rx_cha_msg.data.rc_init_status;//遥控器初始化检测
}

static void rc_stable_decode(void){
	gimbal.feedback_alpha_speed_input = board_comm.rx_stable_msg.data.feedback_alpha_speed_input;
	gimbal.feedback_beta_speed_input = board_comm.rx_stable_msg.data.feedback_beta_speed_input;
}

static void tx_judge_decode(void){
	 vision_tx_msg.ID = board_comm.rx_judge_msg.data.vision_ID;  	
}

static void tx_cha_decode(void){
	board_comm.tx_cha_msg.data.vision_enanle    = vision.shoot_enable;
	board_comm.tx_cha_msg.data.gimbal_start_up  = gimbal.start_up;
}

float yaw_C;
float pit_C;

static void tx_vis_ui_decode(void){
	if(vision.aim_status == AIMING)
		board_comm.tx_vis_msg.data.vision_trace_id = vision.trace_id;
	else
		board_comm.tx_vis_msg.data.vision_trace_id = 0;
}

void board_comm_get_data(uint32_t id, uint8_t *data){
	
	 status.task.board = 1;
	
	 switch(id) {
		case RC_MSG_ID: {
//			memcpy(board_comm.rx_rc_msg.buff,data,BOARD_DATA_LEN);
//			rc_rc_decode();
			break;
		}
        case CHA_MSG_ID: {
			memcpy(board_comm.rx_cha_msg.buff,data,BOARD_DATA_LEN);
			rc_cha_decode();
			break;
		}
		case JUDGE_MSG_ID: {
			memcpy(board_comm.rx_judge_msg.buff,data,BOARD_DATA_LEN);
			tx_judge_decode();
			break;
		}
		case SHOOT_MSG_ID: {//1111要，改为上面控制
			memcpy(board_comm.rx_shoot_msg.buff,data,BOARD_DATA_LEN);
			break;
		}
		case STABLE_MSG_ID: {
			memcpy(board_comm.rx_stable_msg.buff,data,BOARD_DATA_LEN);
			rc_stable_decode();//前馈，测试架不需要
			break;
		}	
	}
}	 

void board_comm_send_data(void){
		
	static uint32_t cnt;
	static uint8_t id;
	cnt++;
	
	if(cnt % 50 == 0 )	
		id = VISION_UI_MSG_ID;
	else 
		id = VISION_MSG_ID;
	
	switch(id) {
		case VISION_MSG_ID: {
			tx_cha_decode();
			memcpy(board_comm_tx_buff,board_comm.tx_cha_msg.buff,BOARD_DATA_LEN);
			can_std_transmit(CAN_CHANNEL_3,VISION_MSG_ID,board_comm_tx_buff);
			break;
		}
		case VISION_UI_MSG_ID: {
			tx_vis_ui_decode();
			memcpy(board_comm_tx_buff,board_comm.tx_vis_msg.buff,BOARD_DATA_LEN);
			can_std_transmit(CAN_CHANNEL_3,VISION_UI_MSG_ID,board_comm_tx_buff);
			break;
		}
	}
}
//fdcan板间通信下发数据
void fdcan_board_comm_send(void)
{
	//vtm遥控数据
	memcpy(&fdcan_board_comm.tx_msg.e.data_remote,&vtm.vtm_data.remote_data,sizeof(vtm.vtm_data.remote_data));
//	//图传链路键鼠数据
	memcpy(&fdcan_board_comm.tx_msg.e.data_keyboard.mouse_data,&vtm.vtm_data.mouse_data,sizeof(vtm.vtm_data.mouse_data));
	fdcan_board_comm.tx_msg.e.data_keyboard.key_code = vtm.vtm_data.kb.key_code;
	fdcan_board_comm.tx_msg.e.data_keyboard.online = vtm.online;
	//云台数据
	fdcan_board_comm.tx_msg.e.gimbal_data.yaw_output = -1.0f * gimbal.yaw_output;
	fdcan_board_comm.tx_msg.e.gimbal_data.gimbal_start_up = gimbal.start_up;
	//视觉数据
	fdcan_board_comm.tx_msg.e.vision_data.vision_enanle = vision.shoot_enable;
	if(vision.aim_status == AIMING)
		fdcan_board_comm.tx_msg.e.vision_data.vision_trace_id = vision.trace_id;
	else
		fdcan_board_comm.tx_msg.e.vision_data.vision_trace_id = 0;
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
//		memcpy(&rc_data_rec,&fdcan_board_comm.rx_msg.e.rc_data,sizeof(rc_data_rec));
		memcpy(&kb_data_rec,&fdcan_board_comm.rx_msg.e.kb_data,sizeof(kb_data_rec));
		memcpy(&imu_data_rec,&fdcan_board_comm.rx_msg.e.imu_data,sizeof(imu_data_rec));
		memcpy(&judge_data_rec,&fdcan_board_comm.rx_msg.e.judge_data,sizeof(judge_data_rec));
		memcpy(&vision_data_rec,&fdcan_board_comm.rx_msg.e.vision_data,sizeof(vision_data_rec));
		memcpy(&gimbal_data_rec,&fdcan_board_comm.rx_msg.e.gimbal_data,sizeof(gimbal_data_rec));
		
		robot_status.robot_id = judge_data_rec.camp;
		board_comm_online = 1;
	}
}

uint8_t board_comm_check_offline(void)
{
    if (board_comm_online == 0) {
        return 1;
    } else {
        board_comm_online = 0;
        return 0;
    }
}
