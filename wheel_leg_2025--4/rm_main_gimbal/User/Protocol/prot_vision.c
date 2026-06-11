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
#include "mode_switch_task.h"
#include "board_comm.h"
#include "crc.h"
#include "drv_dji_motor.h"

//#define VISION_HAND_EYE_CALIBRATION

#define VISION_SP_FIRE_YAW_ERR (2.0f * PI / 180.0f)
#define VISION_SP_FIRE_PIT_ERR (2.0f * PI / 180.0f)

vision_t vision;
vision_tx_msg_t vision_tx_msg;
vision_rx_msg_sp_t vision_rx_msg_sp;
vision_tx_msg_sp_t vision_tx_msg_sp;
uint8_t vis_e[2];
uint32_t fire_Cnt;
uint32_t shoot_Cnt;
uint32_t send_cnt;
static uint8_t vision_get_sp_mode(void)
{
    if (ctrl_mode == KEYBOARD_MODE) {
        if (KEY_PRESS_VISION1) {
            return VISION_SP_MODE_BIG_BUFF;
        } else if (KEY_PRESS_VISION2) {
            return VISION_SP_MODE_SMALL_BUFF;
        } else if (rc.mouse.r == 1) {
            return VISION_SP_MODE_AUTO_AIM;
        }
    } else if (ctrl_mode == REMOTER_MODE) {
        if (rc_fsm_check(RC_LEFT_LD) && rc_fsm_check(RC_RIGHT_RD)) {
            return VISION_SP_MODE_AUTO_AIM;
        }
    }

    return VISION_SP_MODE_IDLE;
}

static void vision_euler_to_quat_wxyz(float yaw, float pitch, float roll, float q[4])
{
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}
static float vision_normalize_yaw(float yaw)
{
    while (yaw > 2.0f * PI) {
        yaw -= 2.0f * PI;
    }
    while (yaw < 0.0f) {
        yaw += 2.0f * PI;
    }
    return yaw;
}

static uint8_t vision_sp_control_enabled(uint8_t mode)
{
    return (mode == VISION_SP_RX_MODE_CONTROL || mode == VISION_SP_RX_MODE_FIRE);
}
static uint8_t vision_sp_float_valid(float value)
{
    return (value > -10000.0f && value < 10000.0f);
}

static float vision_sp_read_float(const uint8_t data[4])
{
    float value;
    memcpy(&value, data, sizeof(value));
    return value;
}

static void vision_sp_write_float(uint8_t data[4], float value)
{
    memcpy(data, &value, sizeof(value));
}

static void vision_sp_write_u16(uint8_t data[2], uint16_t value)
{
    data[0] = (uint8_t)(value & 0xffU);
    data[1] = (uint8_t)((value >> 8) & 0xffU);
}
static void vision_get_data_sp(uint8_t *data, uint32_t len)
{
    vision.last_rx_tick = HAL_GetTick();
    static vision_aim_status_e last_aim_status;

    if (len < sizeof(vision_rx_msg_sp_t)) {
        vision.rx_status = TAIL_ERROR;
        return;
    }

    if (!crc16_verify_checksum(data, sizeof(vision_rx_msg_sp_t))) {
        vision.rx_status = TAIL_ERROR;
        return;
    }

    memcpy(&vision_rx_msg_sp, data, sizeof(vision_rx_msg_sp));

    float rx_yaw = vision_sp_read_float(vision_rx_msg_sp.yaw);
    float rx_yaw_vel = vision_sp_read_float(vision_rx_msg_sp.yaw_vel);
    float rx_yaw_acc = vision_sp_read_float(vision_rx_msg_sp.yaw_acc);
    float rx_pitch = vision_sp_read_float(vision_rx_msg_sp.pitch);
    float rx_pitch_vel = vision_sp_read_float(vision_rx_msg_sp.pitch_vel);
    float rx_pitch_acc = vision_sp_read_float(vision_rx_msg_sp.pitch_acc);

    if (vision_rx_msg_sp.mode > VISION_SP_RX_MODE_FIRE ||
        !vision_sp_float_valid(rx_yaw) ||
        !vision_sp_float_valid(rx_yaw_vel) ||
        !vision_sp_float_valid(rx_yaw_acc) ||
        !vision_sp_float_valid(rx_pitch) ||
        !vision_sp_float_valid(rx_pitch_vel) ||
        !vision_sp_float_valid(rx_pitch_acc)) {
        vision.rx_status = TAIL_ERROR;
        return;
    }

    vision.online = 1;
    vision.rx_status = NORMAL;
    vision.rx_repeat_cnt = 0;
    vision.trace_id = 0;

    if (vision_sp_control_enabled(vision_rx_msg_sp.mode)) {
        vision.aim_status = AIMING;
        vision.new_frame_flag = 1;
        vision.target_yaw_angle = vision_normalize_yaw(rx_yaw);
        vision.target_pit_angle = -rx_pitch;

        if (vision.target_pit_angle < -0.40f) {
            vision.target_pit_angle = -0.40f;
        }
        if (fabs(circle_error(vision.target_yaw_angle, gimbal.yaw_angle.fdb, 2 * PI)) > 1.73f) {
            vision.target_yaw_angle = gimbal.yaw_angle.fdb;
        }

        vision.rx[0].data.yaw = rx_yaw / PI * 180.0f;
        vision.rx[0].data.pit = rx_pitch / PI * 180.0f;
        vision.rx[0].data.yaw_vel = rx_yaw_vel;
        vision.rx[0].data.yaw_acc = rx_yaw_acc;
        vision.rx[0].data.pit_vel = -rx_pitch_vel;
        vision.rx[0].data.pit_acc = -rx_pitch_acc;
        vision.yaw_min_err = VISION_SP_FIRE_YAW_ERR;
        vision.pit_min_err = VISION_SP_FIRE_PIT_ERR;

        if (ABS(circle_error(vision.target_yaw_angle, gimbal.yaw_angle.fdb, 2 * PI)) < vision.yaw_min_err) {
            vis_e[0] = 1;
        } else {
            vis_e[0] = 0;
        }
        if (ABS(gimbal.pit_angle.fdb - vision.target_pit_angle) < vision.pit_min_err) {
            vis_e[1] = 1;
        } else {
            vis_e[1] = 0;
        }

        if (vision_rx_msg_sp.mode == VISION_SP_RX_MODE_FIRE && vis_e[0] && vis_e[1]) {
            vision.shoot_enable = 1;
        } else {
            vision.shoot_enable = 0;
        }
    } else {
        vision.aim_status = UNAIMING;
        vision.new_frame_flag = 0;
        vision.shoot_enable = 0;
        vision.rx[0].data.yaw_vel = 0.0f;
        vision.rx[0].data.yaw_acc = 0.0f;
        vision.rx[0].data.pit_vel = 0.0f;
        vision.rx[0].data.pit_acc = 0.0f;
        vis_e[0] = 0;
        vis_e[1] = 0;
    }

    if (vision_rx_msg_sp.mode == VISION_SP_RX_MODE_FIRE) {
        fire_Cnt++;
    }
    if (last_aim_status == AIMING && vision.aim_status == UNAIMING) {
        vision.aim_status = FIRST_LOST;
    }
    last_aim_status = vision.aim_status;
    vision.last_rx_tick = HAL_GetTick();
}
void vision_get_data(uint8_t *data, uint32_t len)
{
    static vision_aim_status_e last_aim_status;

    if (len >= 2 && data[0] == 'S' && data[1] == 'P') {
        vision_get_data_sp(data, len);
        return;
    }

    if (len < VISION_DATA_LEN) {
        vision.rx_status = TAIL_ERROR;
        return;
    }
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
}

uint8_t vision_send_buf[29];
uint8_t vision_sp_send_buf[sizeof(vision_tx_msg_sp_t)];
float kanan;
void vision_output_data(void)
{
    //    vision.tx.data.imu_pit = -gimbal_imu.pit / PI * 180;
//    vision.tx.data.imu_yaw = gimbal_imu.yaw / PI * 180;
//    vision.tx.data.imu_pit_spd = -gimbal_imu.wy / PI * 180;
//    vision.tx.data.imu_yaw_spd = gimbal_imu.wz / PI * 180;
    
//        vision.tx.data.sof = 0x11;
//        float imu_data_temp_buf;
//    ��=  -gimbal_imu.pit / PI * 180;
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
		//射击初速度
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
		
    CDC_Transmit_HS(vision_send_buf, 29);
	send_cnt++;
}
float yaw_tx;
void vision_output_data_sp(void)
{
    float q[4];
#ifdef VISION_HAND_EYE_CALIBRATION
    yaw_tx = (float)(8192 - yaw_motor.ecd) / 8192.0f * 2.0f * PI;
#else
    yaw_tx = gimbal_imu.yaw;
#endif

    vision_tx_msg_sp.head[0] = 'S';
    vision_tx_msg_sp.head[1] = 'P';
//    vision_tx_msg_sp.mode = vision_get_sp_mode();
	  vision_tx_msg_sp.mode = VISION_SP_MODE_AUTO_AIM;

    vision_euler_to_quat_wxyz(yaw_tx, -gimbal_imu.pit, gimbal_imu.rol, q);
    for (uint8_t i = 0; i < 4; i++) {
        vision_sp_write_float(vision_tx_msg_sp.q[i], q[i]);
    }

    vision_sp_write_float(vision_tx_msg_sp.yaw, yaw_tx);
    vision_sp_write_float(vision_tx_msg_sp.yaw_vel, gimbal_imu.wz);
    vision_sp_write_float(vision_tx_msg_sp.pitch, -gimbal_imu.pit);
    vision_sp_write_float(vision_tx_msg_sp.pitch_vel, -gimbal_imu.wy);
    vision_sp_write_float(vision_tx_msg_sp.bullet_speed, fdcan_board_comm.rx_msg.e.vision_data.shoot_speed);
    vision_sp_write_u16(vision_tx_msg_sp.bullet_count, (uint16_t)shoot_Cnt);
    if (robot_status.robot_id > 100) {
        vision_tx_msg_sp.camp = 1;
    } else {
        vision_tx_msg_sp.camp = 0;
    }
    crc16_set_checksum((uint8_t *)&vision_tx_msg_sp, sizeof(vision_tx_msg_sp));

    memcpy(vision_sp_send_buf, &vision_tx_msg_sp, sizeof(vision_tx_msg_sp));
    CDC_Transmit_HS(vision_sp_send_buf, sizeof(vision_tx_msg_sp));
    send_cnt++;
}

uint8_t vision_check_offline(void)
{
    uint32_t current_tick = HAL_GetTick();
    if (vision.last_rx_tick != 0 && (uint32_t)(current_tick - vision.last_rx_tick) <= VISION_OFFLINE_TIMEOUT_MS)
        vision.online = 1;
    else
        vision.online = 0;
    return vision.online;
}

