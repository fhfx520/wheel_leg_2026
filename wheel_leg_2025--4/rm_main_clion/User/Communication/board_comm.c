#include "board_comm.h"
#include "string.h"
#include "prot_dr16.h"
#include "mode_switch_task.h"
#include "prot_imu.h"
#include "can_comm.h"
#include "prot_vision.h"
#include "gimbal_task.h"
#include "chassis_task.h"

board_comm_t board_comm;

uint8_t board_comm_tx_buff[8];


static void rc_rc_decode(void){

	
	if(ctrl_mode == REMOTER_MODE || ctrl_mode == PROTECT_MODE){
	rc.ch1 			 = board_comm.rx_rc_msg.data_remote.chl;
	rc.ch2 		 	 = board_comm.rx_rc_msg.data_remote.ch2;
	rc.sw1 			 = board_comm.rx_rc_msg.data_remote.sw1;
	rc.sw2 			 = board_comm.rx_rc_msg.data_remote.sw2;
	}
	else if(ctrl_mode == KEYBOARD_MODE){
	rc.mouse.x 			 = board_comm.rx_rc_msg.data_keyboard.x;
	rc.mouse.y 		 	 = board_comm.rx_rc_msg.data_keyboard.y;
	rc.mouse.l 			 = board_comm.rx_rc_msg.data_keyboard.l;
	rc.mouse.r 			 = board_comm.rx_rc_msg.data_keyboard.r;
	}

}

static void rc_cha_decode(void){
     if( fabs(board_comm.rx_cha_msg.data.cha_pit) < 1e-4 && board_comm.rx_cha_msg.data.ctrl_mode==0 )
         return ;
             
         
	chassis_imu.pit = board_comm.rx_cha_msg.data.cha_pit;
	ctrl_mode 	    = board_comm.rx_cha_msg.data.ctrl_mode;
	chassis.mode    = board_comm.rx_cha_msg.data.chassis_mode;
    rc.init_status  = board_comm.rx_cha_msg.data.rc_init_status;
}

static void tx_cha_decode(void){
   
	board_comm.tx_cha_msg.data.vision_enanle    = vision.shoot_enable;
	board_comm.tx_cha_msg.data.gimbal_start_up  = gimbal.start_up;
}

void board_comm_get_data(uint32_t id, uint8_t *data){

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
	}
}	 

void board_comm_send_data(uint32_t id){
		
		 switch(id) {
		
		case VISION_MSG_ID: {
					tx_cha_decode();
				  memcpy(board_comm_tx_buff,board_comm.tx_cha_msg.buff,BOARD_DATA_LEN);
					can_std_transmit(CAN_CHANNEL_3,VISION_MSG_ID,board_comm_tx_buff);
				 break;
    }

	}

}
