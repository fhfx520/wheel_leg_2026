#include "prot_vision.h"
#include "shoot_task.h"
#include "gimbal_task.h"
#include "prot_judge.h"
#include "prot_imu.h"
#include "math_lib.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "prot_dr16.h"
#include "control_def.h"
#include "board_comm.h"
#include "crc.h"

vision_t vision;
vision_tx_msg_t vision_tx_msg;
uint8_t vis_e[2];
uint32_t fire_Cnt;
uint32_t shoot_Cnt;
uint32_t send_cnt;

void vision_get_data(uint8_t *data)
{
    static vision_aim_status_e last_aim_status;
    memcpy(vision.rx[0].buff, data, VISION_DATA_LEN);
    if (vision.rx[0].data.eof != 0xCC) {
        vision.rx_status = TAIL_ERROR;
        return;
    } else if (vision.rx[0].data.cnt == vision.rx[1].data.cnt) {
        if (++vision.rx_repeat_cnt > 10) {
            vision.rx_status = REPEAT_ERROR;
            return;
        }
    }
    vision.online = 1;
    vision.rx_status = NORMAL;
    vision.rx_repeat_cnt = 0;
    vision.rx[1].data.cnt = vision.rx[0].data.cnt;
	  vision.trace_id = vision.rx[0].data.trace_id;
    
    NAN_PROCESS(vision.rx[0].data.yaw, vision.rx[1].data.yaw);
    NAN_PROCESS(vision.rx[0].data.pit, vision.rx[1].data.pit);
    NAN_PROCESS(vision.rx[0].data.shoot_yaw_tole, vision.rx[1].data.shoot_yaw_tole);
    NAN_PROCESS(vision.rx[0].data.fire, vision.rx[1].data.fire);
    NAN_PROCESS(vision.rx[0].data.fire_rf, vision.rx[1].data.fire_rf);
    
	if (vision.rx[0].data.aim_flag) {
        vision.aim_status = AIMING;
        vision.new_frame_flag = 1;
        vision.target_yaw_angle = vision.rx[0].data.yaw / 180.00f * PI;
        vision.target_pit_angle = -vision.rx[0].data.pit / 180.0f * PI;
		if( vision.target_pit_angle < -0.40f )
			vision.target_pit_angle = -0.40f;
		
		if(vision.target_yaw_angle > 2 * PI)	
			vision.target_yaw_angle -= 2 * PI;
		else if(vision.target_yaw_angle < 0)	
			vision.target_yaw_angle += 2 * PI;
		
//				if ((vision.target_yaw_angle - gimbal.yaw_angle.fdb) > 1.73f)
		if( fabs(circle_error(vision.target_yaw_angle,gimbal.yaw_angle.fdb, 2*PI)) > 1.73f)
			vision.target_yaw_angle = gimbal.yaw_angle.fdb;
			
        vision.yaw_min_err = vision.rx[0].data.shoot_yaw_tole / 180.0f * PI;
		vision.pit_min_err = vision.rx[0].data.shoot_pit_tole / 180.0f * PI;

        if (ABS(gimbal.yaw_angle.fdb - vision.target_yaw_angle) < vision.yaw_min_err && \
            ABS(gimbal.pit_angle.fdb - vision.target_pit_angle) < vision.pit_min_err && \
            (vision.rx[0].data.fire == 1 || vision.rx[0].data.fire == 2)){				
				vision.shoot_enable = 1;
				shoot_Cnt++;
			}
        else
            vision.shoot_enable = 0;
	} 
	else {
        vision.aim_status = UNAIMING;
        vision.shoot_enable = 0;
	}

		
	if (ABS(gimbal.yaw_angle.fdb - vision.target_yaw_angle) < vision.yaw_min_err)
		 vis_e[0] = 1;
	else
		 vis_e[0] = 0;	
	if (ABS(gimbal.pit_angle.fdb - vision.target_pit_angle) < vision.pit_min_err)
		 vis_e[1] = 1;
	else
		 vis_e[1] = 0;	
	if(vision.rx[0].data.fire == 1 ||vision.rx[0].data.fire == 2) 
		 fire_Cnt++;
    if (last_aim_status == AIMING && vision.aim_status == UNAIMING)
        vision.aim_status = FIRST_LOST;
    last_aim_status = vision.aim_status;
	board_comm.tx_vis_msg.data.vision_online = 1;
	fdcan_board_comm.tx_msg.e.vision_data.vision_online = 1;
}

uint8_t vision_send_buf[40];
float kanan;
void vision_output_data(void)
{
    //    vision.tx.data.imu_pit = -gimbal_imu.pit / PI * 180;
//    vision.tx.data.imu_yaw = gimbal_imu.yaw / PI * 180;
//    vision.tx.data.imu_pit_spd = -gimbal_imu.wy / PI * 180;
//    vision.tx.data.imu_yaw_spd = gimbal_imu.wz / PI * 180;
    
//        vision.tx.data.sof = 0x11;
//        float imu_data_temp_buf;
//    °§=  -gimbal_imu.pit / PI * 180;
//		memcpy(&vision.tx.data.imu_pit, &imu_data_temp_buf, 4);
//    
//		imu_data_temp_buf = gimbal_imu.yaw / PI * 180;
//		memcpy(&vision.tx.data.imu_yaw, &imu_data_temp_buf, 4);
//    
//		imu_data_temp_buf = -gimbal_imu.wy / PI * 180;
//		memcpy(&vision.tx.data.imu_pit_spd, &imu_data_temp_buf, 4);
//		imu_data_temp_buf = gimbal_imu.wz / PI * 180;
//		memcpy(& vision.tx.data.imu_yaw_spd, &imu_data_temp_buf, 4);
//    


//    vision.tx.data.shooter_speed = 3;
//    vision.tx.data.vacancy = 0;
//    
//    if (robot_status.robot_id > 100) {
//        vision.tx.data.camp = 1;
//    } else {
//        vision.tx.data.camp = 0;
//    }
//    
//    vision.tx.data.eof1 = 0x22;
//    vision.tx.data.eof2 = 0x33;
    
    /////////////////////////////////
        vision_tx_msg.sof = 0x11;
        float imu_data_temp_buf;
		float bias_time_temp_buf;
    	imu_data_temp_buf =  gimbal_imu.pit / PI * 180;
			kanan =  -gimbal_imu.pit / PI * 180;
//			imu_data_temp_buf =  -gimbal_imu.pit / PI * 180;
		memcpy(&vision_tx_msg.imu_pit, &imu_data_temp_buf, 4);
    
		imu_data_temp_buf = gimbal_imu.yaw / PI * 180;
		memcpy(&vision_tx_msg.imu_yaw, &imu_data_temp_buf, 4);
    
		imu_data_temp_buf = gimbal_imu.wy / PI * 180;
		memcpy(&vision_tx_msg.imu_pit_spd, &imu_data_temp_buf, 4);
		imu_data_temp_buf = gimbal_imu.wz / PI * 180;
		memcpy(&vision_tx_msg.imu_yaw_spd, &imu_data_temp_buf, 4);
		//Â∞ÑÂáªÂàùÈÄüÂ∫¶
//		imu_data_temp_buf =  board_comm.rx_shoot_msg.data.shoot_speed;
		imu_data_temp_buf = fdcan_board_comm.rx_msg.e.vision_data.shoot_speed;
		memcpy(&vision_tx_msg.shoot_speed, &imu_data_temp_buf, 4);
		
//		bias_time_temp_buf =  board_comm.rx_shoot_msg.data.vision_bias_time;
		bias_time_temp_buf = fdcan_board_comm.rx_msg.e.vision_data.vision_bias_time;
		memcpy(&vision_tx_msg.bias_time, &bias_time_temp_buf, 4);
		
		vision_tx_msg.ID = fdcan_board_comm.rx_msg.e.vision_data.vision_ID;
		
		

    vision_tx_msg.mode_msg.shooter_speed = 3;
    vision_tx_msg.mode_msg.vacancy = 0;
		
    if (robot_status.robot_id > 100) {
        vision_tx_msg.mode_msg.camp = 1;
    } else {
        vision_tx_msg.mode_msg.camp = 0;
    }
    
   vision_tx_msg.eof1 = 0x22;
   vision_tx_msg.eof2 = 0x33;

    memcpy(vision_send_buf, &vision_tx_msg, sizeof(vision_tx_msg));
		
    CDC_Transmit_HS(vision_send_buf, 30);
	send_cnt++;
}


//***********************Õ¨º√ ”æı£¨¥˝≤‚ ‘************************//
GimbalToVision_t vision_tx;
VisionToGimbal_t vision_rx;
//vision_super_power_t vision_super_power; 

void superpower_vision_RxHandler(uint8_t *data)
{
	if(data[0] == 'S' && data[1] == 'P')
	{
		memcpy(&vision_rx,data,sizeof(VisionToGimbal_t));
	}
}

void rpy_to_quaternion(float roll, float pitch, float yaw, float *w, float *x, float *y, float *z) {
    // ???????????????
    float sr = sinf(roll / 2.0f);
    float cr = cosf(roll / 2.0f);
    float sp = sinf(pitch / 2.0f);
    float cp = cosf(pitch / 2.0f);
    float sy = sinf(yaw / 2.0f);
    float cy = cosf(yaw / 2.0f);

    // ?????????????
    *w = cr * cp * cy + sr * sp * sy;
    *x = sr * cp * cy - cr * sp * sy;
    *y = cr * sp * cy + sr * cp * sy;
    *z = cr * cp * sy - sr * sp * cy;
}
uint8_t spvision_send_buf[50];
void superpower_vision_Tx(void)
{
	vision_tx.data.e.head[0] = 'S',vision_tx.data.e.head[1] = 'P';
	vision_tx.data.e.mode = 1;
	rpy_to_quaternion(gimbal_imu.rol,gimbal_imu.pit,gimbal_imu.yaw,&vision_tx.data.e.q[0],&vision_tx.data.e.q[1],&vision_tx.data.e.q[2],&vision_tx.data.e.q[3]);
	vision_tx.data.e.yaw = gimbal_imu.yaw / PI * 180;
	vision_tx.data.e.yaw_vel = gimbal_imu.wz / PI * 180;
	vision_tx.data.e.pitch = gimbal_imu.pit / PI * 180;
	vision_tx.data.e.pitch_vel = gimbal_imu.wy / PI * 180;
	vision_tx.data.e.bullet_speed = fdcan_board_comm.rx_msg.e.vision_data.shoot_speed;
	vision_tx.data.e.bullet_count = 0;
	crc16_set_checksum(vision_tx.data.buff,SUPERPOWER_VISION_TX_DATA_LEN);
//	vision_tx.data.e.crc16 = 0xffff;
	memcpy(spvision_send_buf,&vision_tx.data.e,sizeof(vision_tx.data.e));
	CDC_Transmit_HS(spvision_send_buf, 43);
} 

void superpower_vision_Rx(uint8_t *data)
{
	 
	memcpy(vision_rx.data.buff, data, 29);
	if((vision_rx.data.e.head[0] != 'S')||(vision_rx.data.e.head[1] != 'P'))
	{	
		return;
	}
    vision.online = 1;
	if(vision_rx.data.e.mode == 1 || vision_rx.data.e.mode == 2)
	{
		vision.aim_status = AIMING;
		vision.new_frame_flag = 1;
        vision_rx.target_yaw_angle 	= vision_rx.data.e.yaw;
        vision_rx.target_pit_angle 	= -vision_rx.data.e.pitch;
		vision_rx.target_pit_vel 	= vision_rx.data.e.pitch_vel ;
		vision_rx.target_yaw_vel 	= vision_rx.data.e.yaw_vel;
		
		if( vision_rx.target_pit_angle < -0.40f )
			vision_rx.target_pit_angle = -0.40f;
		if(vision_rx.data.e.mode == 2)
			vision.shoot_enable = 1;
	}else {
        vision.aim_status = UNAIMING;
        vision.shoot_enable = 0;
	}

}


//***********************Õ¨º√ ”æı£¨¥˝≤‚ ‘£¨Ω· ¯************************//


uint8_t vision_check_offline(void)
{
    if (vision.online == 0) {
        return 1;
    } else {
        vision.online = 0;
        return 0;
    }
}

