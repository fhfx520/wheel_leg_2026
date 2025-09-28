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


board_comm_t board_comm;
uint8_t board_comm_tx_buff[8];


static void rc_rc_decode(void){	
	if(ctrl_mode == REMOTER_MODE || ctrl_mode == PROTECT_MODE){
		rc.ch1 			 = board_comm.rx_rc_msg.data_remote.chl;
		rc.ch2 		 	 = board_comm.rx_rc_msg.data_remote.ch2;
		rc.sw1 			 = board_comm.rx_rc_msg.data_remote.sw1;
		rc.sw2 			 = board_comm.rx_rc_msg.data_remote.sw2;
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
         
	chassis_imu.pit = board_comm.rx_cha_msg.data.cha_pit;
	ctrl_mode 	    = board_comm.rx_cha_msg.data.ctrl_mode;
	robot_status.robot_id  = board_comm.rx_cha_msg.data.camp;
	rc.init_status  = board_comm.rx_cha_msg.data.rc_init_status;
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
			memcpy(board_comm.rx_rc_msg.buff,data,BOARD_DATA_LEN);
			rc_rc_decode();
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
		case SHOOT_MSG_ID: {
			memcpy(board_comm.rx_shoot_msg.buff,data,BOARD_DATA_LEN);
			break;
		}
		case STABLE_MSG_ID: {
			memcpy(board_comm.rx_stable_msg.buff,data,BOARD_DATA_LEN);
			rc_stable_decode();
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
