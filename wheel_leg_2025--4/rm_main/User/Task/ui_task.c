#include "ui_task.h"
#include "cmsis_os.h"
#include "prot_judge.h"
#include "prot_vision.h"
#include "prot_dr16.h"
#include "prot_power.h"
#include "us_time.h"
#include "wlr.h"
#include "ui_g.h"
#include "ui_interface.h"
#include "ui_types.h"
#include "leg_vmc.h"
#include "math_lib.h"
#include "control_def.h"
#include "drv_dji_motor.h"
#include "drv_dm_motor.h"
#include "drv_lk_motor.h"
#include "chassis_task.h"
#include "board_comm.h"
#include "container.h"
#include "shoot_task.h"
#include "ctype.h"

us_time_t ui_time;

extern ui_string_frame_t ui_g_4_0;
extern ui_string_frame_t ui_g_4_1;
extern ui_string_frame_t ui_g_4_2;
extern ui_string_frame_t ui_g_4_3;
extern ui_string_frame_t ui_g_4_4;
extern ui_string_frame_t ui_g_4_5;

extern ui_string_frame_t ui_g_6_0;
extern ui_string_frame_t ui_g_6_1;
extern ui_string_frame_t ui_g_6_2;
extern ui_string_frame_t ui_g_6_3;
extern ui_string_frame_t ui_g_6_4;
extern ui_string_frame_t ui_g_6_5;
extern ui_string_frame_t ui_g_6_6;

static vision_data_t ui_get_vision_data_container;
static online_data_t ui_get_online_data_container;

float armour_center,start_angle,end_angle;

uint8_t last_group4_online_status[5];
uint8_t last_group6_online_status[6];
uint8_t last_rotate_flag;

// 收到vision数据：
static void vision_data_cb(uint32_t tag_id, void* data, size_t len) {
	if(data == NULL || len != sizeof(vision_data_t))
		return;
    memcpy(&ui_get_vision_data_container,(vision_data_t*)data,len);
}

// 收到online数据：
static void online_data_cb(uint32_t tag_id, void* data, size_t len) {
	if(data == NULL || len != sizeof(online_data_t))
		return;
    memcpy(&ui_get_online_data_container,(online_data_t*)data,len);
}

// --- 回调配置表  ---
static const ContainerBusCfg mb_callback[] = {
    { TAG_TRACE_VISION_DATA, vision_data_cb, NULL },
    { TAG_ONLINE_DATA, online_data_cb, NULL },
};

static int8_t ui_sign_function(void)
{
	return (!wlr.direction ? 1 : -1);
}

void replace_letters_with_spaces(char *str) {
    if (str == NULL) return; // 防止传入空指针

    for (int i = 0; str[i] != '\0'; ++i) {
        // 如果当前字符是字母，则替换成空格
        if (isalpha(str[i]) || str[i] == '_' || str[i] == '!') {
            str[i] = ' ';
        }
    }
}

static void _ui_update_spin_warning() {
    ui_g_4_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_4_0);
    SEND_MESSAGE((uint8_t *) &ui_g_4_0, sizeof(ui_g_4_0));
}

static void _ui_update_leg_rb_offline() {
    ui_g_4_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_4_1);
    SEND_MESSAGE((uint8_t *) &ui_g_4_1, sizeof(ui_g_4_1));
}

static void _ui_update_leg_rs_offline() {
    ui_g_4_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_4_2);
    SEND_MESSAGE((uint8_t *) &ui_g_4_2, sizeof(ui_g_4_2));
}

static void _ui_update_yaw_offline() {
    ui_g_4_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_4_3);
    SEND_MESSAGE((uint8_t *) &ui_g_4_3, sizeof(ui_g_4_3));
}

static void _ui_update_pit_offline() {
    ui_g_4_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_4_4);
    SEND_MESSAGE((uint8_t *) &ui_g_4_4, sizeof(ui_g_4_4));
}

static void _ui_update_gimbal_imu_offline() {
    ui_g_4_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_4_5);
    SEND_MESSAGE((uint8_t *) &ui_g_4_5, sizeof(ui_g_4_5));
}

static void _ui_update_fric_l_offline() {
    ui_g_6_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_0);
    SEND_MESSAGE((uint8_t *) &ui_g_6_0, sizeof(ui_g_6_0));
}

static void _ui_update_fric_r_offline() {
    ui_g_6_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_1);
    SEND_MESSAGE((uint8_t *) &ui_g_6_1, sizeof(ui_g_6_1));
}

static void _ui_update_trigger_offline() {
    ui_g_6_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_2);
    SEND_MESSAGE((uint8_t *) &ui_g_6_2, sizeof(ui_g_6_2));
}

static void _ui_update_wheel_l_offline() {
    ui_g_6_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_3);
    SEND_MESSAGE((uint8_t *) &ui_g_6_3, sizeof(ui_g_6_3));
}

static void _ui_update_wheel_r_offline() {
    ui_g_6_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_4);
    SEND_MESSAGE((uint8_t *) &ui_g_6_4, sizeof(ui_g_6_4));
}

static void _ui_update_leg_lb_offline() {
    ui_g_6_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_5);
    SEND_MESSAGE((uint8_t *) &ui_g_6_5, sizeof(ui_g_6_5));
}

static void _ui_update_leg_ls_offline() {
    ui_g_6_6.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_6);
    SEND_MESSAGE((uint8_t *) &ui_g_6_6, sizeof(ui_g_6_6));
}

void ui_init(void)
{
	ui_init_g_1();
	ui_init_g_2();
	ui_init_g_3();
	ui_init_g_4();
	ui_init_g_5();
	ui_init_g_6();
}

void ui_update(void)
{	
	static uint8_t one_second = 0;
	
	one_second++;
	if(one_second > 4)
		one_second = 0;
	//ui_group1 update begin
	ui_g_1_left_big_leg->end_x = ui_g_1_left_big_leg->start_x - ui_sign_function() * (vmc[((!wlr.direction) ? 1 : 0)].mp_fdb.xd * 300);
	ui_g_1_left_big_leg->end_y = ui_g_1_left_big_leg->start_y - (vmc[((!wlr.direction) ? 1 : 0)].mp_fdb.yd * 300);
	ui_g_1_left_small_leg->start_x = ui_g_1_left_big_leg->end_x;
	ui_g_1_left_small_leg->start_y = ui_g_1_left_big_leg->end_y;
	ui_g_1_left_small_leg->end_x = ui_g_1_left_big_leg->start_x - ui_sign_function() * (vmc[((!wlr.direction) ? 1 : 0)].mp_fdb.xc * 300);
	ui_g_1_left_small_leg->end_y = ui_g_1_left_big_leg->start_y - (vmc[((!wlr.direction) ? 1 : 0)].mp_fdb.yc * 300);
	
	ui_g_1_right_big_leg->end_x = ui_g_1_right_big_leg->start_x + ui_sign_function() * (vmc[((!wlr.direction) ? 0 : 1)].mp_fdb.xd * 300);
	ui_g_1_right_big_leg->end_y = ui_g_1_right_big_leg->start_y - (vmc[((!wlr.direction) ? 0 : 1)].mp_fdb.yd * 300);
	ui_g_1_right_small_leg->start_x = ui_g_1_right_big_leg->end_x;
	ui_g_1_right_small_leg->start_y = ui_g_1_right_big_leg->end_y;
	ui_g_1_right_small_leg->end_x = ui_g_1_right_big_leg->start_x + ui_sign_function() * (vmc[((!wlr.direction) ? 0 : 1)].mp_fdb.xc * 300);
	ui_g_1_right_small_leg->end_y = ui_g_1_right_big_leg->start_y - (vmc[((!wlr.direction) ? 0 : 1)].mp_fdb.yc * 300);
	
	ui_g_1_supercap_capcity->end_x = ui_g_1_supercap_capcity->start_x + (570.0f) * (supercap.volume_percent / 100.0f);
	ui_g_1_supcap_voltage->number = (int)(supercap.volage * 1000.0f) / 100 * 100;
	if(supercap.volage < 20.0f)
	{
		ui_g_1_supercap_capcity->color = 4;
		ui_g_1_supcap_voltage->color = 5;
	}
	else
	{
		ui_g_1_supercap_capcity->color = 2;
		ui_g_1_supcap_voltage->color = 6;
	}

	if(wlr.high_flag == 1)
		ui_g_1_current_high_flag->start_y = 735;
	else if(wlr.jump_flag == WLR_JUMP_ASCEND)
		ui_g_1_current_high_flag->start_y = 685;
	else if(wlr.high_flag == 0)
		ui_g_1_current_high_flag->start_y = 785;

	if(one_second <= 2)
		ui_g_1_current_high_flag->width = 10;
	else		
		ui_g_1_current_high_flag->width = 0;

	ui_update_g_1();
	//ui_group1 update end
	
	//ui_group2 update begin
	ui_g_2_target_trigger->number = (int)((shoot.trigger_ecd.fdb / 65535.0f * 360.0f) * 1000.0f) / 100 * 100;
	ui_g_2_current_trigger->number = (int)((shoot.trigger_ecd.ref / 65535.0f * 360.0f) * 1000.0f) / 100 * 100;
	
	float yaw_err;
    yaw_err = circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, (float)yaw_motor.ecd / 8192 * 2 * PI, 2 * PI);
    armour_center = yaw_err * 180.0f / PI;
	
	start_angle = armour_center - 30.0f;
	if(start_angle < 0.0f)
        start_angle += 360.0f;
    if(start_angle > 360.0f)
        start_angle -= 360.0f;
	ui_g_2_head_position->start_angle = start_angle;     

	end_angle = armour_center + 30.0f;
	if(end_angle < 0.0f)
        end_angle += 360.0f;
    if(end_angle > 360.0f)
        end_angle -= 360.0f;
    ui_g_2_head_position->end_angle = end_angle; 	
	
	ui_g_2_vision_order_id->number = ID_judge;
	ui_g_2_vision_trice_id->number = ui_get_vision_data_container.vision_trace_id;
	
	ui_g_2_current_shoot_mode->start_y = (wlr.energy_flag ? 638 : 673);
	if(one_second <= 2)
		ui_g_2_current_shoot_mode->width = 10;
	else		
		ui_g_2_current_shoot_mode->width = 0;
	
	ui_g_2_vision_frame->color = (ui_get_vision_data_container.vision_trace_id ? (ui_get_vision_data_container.vision_enanle ? 4 : 1) : 8);
	
	ui_update_g_2();
	//ui_group2 update end
	
	//ui_group4 update begin
	if(last_rotate_flag < rotate_flag)//上升沿 0-1
	{
		replace_letters_with_spaces(ui_g_4_spin_warning->string);
		_ui_update_spin_warning();
	}
	else if(last_rotate_flag > rotate_flag)//下降沿1-0
	{
		strcpy(ui_g_4_spin_warning->string, "PLEASE SPIN!");
		_ui_update_spin_warning();
	}

	if(last_group4_online_status[0] < joint_motor[2].online)
	{
		replace_letters_with_spaces(ui_g_4_leg_right_big_warning->string);
		_ui_update_leg_rb_offline();
	}
	else if(last_group4_online_status[0] > joint_motor[2].online)
	{
		strcpy(ui_g_4_leg_right_big_warning->string, "leg_rb_off");
		_ui_update_leg_rb_offline();
	}

	if(last_group4_online_status[1] < joint_motor[3].online)
	{
		replace_letters_with_spaces(ui_g_4_leg_right_small_warning->string);
		_ui_update_leg_rs_offline();
	}
	else if(last_group4_online_status[1] > joint_motor[3].online)
	{
		strcpy(ui_g_4_leg_right_small_warning->string, "leg_rs_off");
		_ui_update_leg_rs_offline();
	}

	if(last_group4_online_status[2] < yaw_motor.online)
	{
		replace_letters_with_spaces(ui_g_4_yaw_warning->string);
		_ui_update_yaw_offline();
	}
	else if(last_group4_online_status[2] > yaw_motor.online)
	{
		strcpy(ui_g_4_yaw_warning->string, "yaw_off");
		_ui_update_yaw_offline();
	}

	if(last_group4_online_status[3] < ui_get_online_data_container.pit_online)
	{
		replace_letters_with_spaces(ui_g_4_pit_warning->string);
		_ui_update_pit_offline();
	}
	else if(last_group4_online_status[3] > ui_get_online_data_container.pit_online)
	{
		strcpy(ui_g_4_pit_warning->string, "pit_off");
		_ui_update_pit_offline();
	}

	if(last_group4_online_status[4] < ui_get_online_data_container.imu_online)
	{
		replace_letters_with_spaces(ui_g_4_gimbal_imu_warning->string);
		_ui_update_gimbal_imu_offline();
	}
	else if(last_group4_online_status[4] > ui_get_online_data_container.imu_online)
	{
		strcpy(ui_g_4_gimbal_imu_warning->string, "g_imu_off");
		_ui_update_gimbal_imu_offline();
	}


	last_rotate_flag = rotate_flag;
	last_group4_online_status[0] = joint_motor[2].online;
	last_group4_online_status[1] = joint_motor[3].online;
	last_group4_online_status[2] = yaw_motor.online;
	last_group4_online_status[3] = ui_get_online_data_container.pit_online;
	last_group4_online_status[4] = ui_get_online_data_container.imu_online;
	//ui_group4 update end
	
	//ui_group5 update begin
	ui_g_5_left_lazer->number = wlr.side[0].Front_dis_kal * 1000.0f;
	ui_g_5_right_lazer->number = wlr.side[1].Front_dis_kal * 1000.0f;
	ui_g_5_left_leg_length->number = vmc[0].L_fdb * 1000.0f;
	ui_g_5_right_leg_length->number = vmc[1].L_fdb * 1000.0f;
	ui_update_g_5();
	//ui_group5 update end
	
	//ui_group6 update begin
	switch(ui_get_online_data_container.fric_online)
	{
		case 0:
		{
			replace_letters_with_spaces(ui_g_6_fric_left_warning->string);
			replace_letters_with_spaces(ui_g_6_fric_right_warning->string);
			break;
		}
		case 1:
		{
			replace_letters_with_spaces(ui_g_6_fric_left_warning->string);
			strcpy(ui_g_6_fric_right_warning->string, "fric_r_off");
			break;
		}
		case 2:
		{
			strcpy(ui_g_6_fric_left_warning->string, "fric_l_off");
			replace_letters_with_spaces(ui_g_6_fric_right_warning->string);
			break;
		}
		case 3:
		{
			strcpy(ui_g_6_fric_left_warning->string, "fric_l_off");
			strcpy(ui_g_6_fric_right_warning->string, "fric_r_off");
			break;
		}
		default:break;
	}
				
	if(last_group6_online_status[0] != ui_get_online_data_container.fric_online) 
	{
		_ui_update_fric_l_offline();
		_ui_update_fric_r_offline();
	}

	if(last_group6_online_status[1] < trigger_motor.online)
	{
		replace_letters_with_spaces(ui_g_6_trigger_warning->string);
		_ui_update_trigger_offline();
	}
	else if(last_group6_online_status[1] > trigger_motor.online)
	{
		strcpy(ui_g_6_trigger_warning->string, "trigger_off");
		_ui_update_trigger_offline();
	}

	if(last_group6_online_status[2] < driver_motor[0].online)
	{
		replace_letters_with_spaces(ui_g_6_wheel_left_warning->string);
		_ui_update_wheel_l_offline();
	}
	else if(last_group6_online_status[2] > driver_motor[0].online)
	{
		strcpy(ui_g_6_wheel_left_warning->string, "wheel_l_off");
		_ui_update_wheel_l_offline();
	}

	if(last_group6_online_status[3] < driver_motor[1].online)
	{
		replace_letters_with_spaces(ui_g_6_wheel_right_warning->string);
		_ui_update_wheel_r_offline();
	}
	else if(last_group6_online_status[3] > driver_motor[1].online)
	{
		strcpy(ui_g_6_wheel_right_warning->string, "wheel_r_off");
		_ui_update_wheel_r_offline();
	}

	if(last_group6_online_status[4] < joint_motor[0].online)
	{
		replace_letters_with_spaces(ui_g_6_leg_left_big_warning->string);
		_ui_update_leg_lb_offline();
	}
	else if(last_group6_online_status[4] > joint_motor[0].online)
	{
		strcpy(ui_g_6_leg_left_big_warning->string, "leg_lb_off");
		_ui_update_leg_lb_offline();
	}

	if(last_group6_online_status[5] < joint_motor[1].online)
	{
		replace_letters_with_spaces(ui_g_6_leg_left_small_warning->string);
		_ui_update_leg_ls_offline();
	}
	else if(last_group6_online_status[5] > joint_motor[1].online)
	{
		strcpy(ui_g_6_leg_left_small_warning->string, "leg_ls_off");
		_ui_update_leg_ls_offline();
	}

	last_group6_online_status[0] = ui_get_online_data_container.fric_online;
	last_group6_online_status[1] = trigger_motor.online;
	last_group6_online_status[2] = driver_motor[0].online;
	last_group6_online_status[3] = driver_motor[1].online;
	last_group6_online_status[4] = joint_motor[0].online;
	last_group6_online_status[5] = joint_motor[1].online;
	//ui_group6 update end
	
}

uint32_t ui_update_cnt = 0;
void ui_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
	container_bus_init(mb_callback, sizeof(mb_callback)/sizeof(ContainerBusCfg));
    for(int i = 0; i < 5; i++) {
        thread_wake_time = osKernelSysTick();
        ui_init();
    }	
    for(;;)
    {
        thread_wake_time = osKernelSysTick();
        us_timer_interval_test_start(&ui_time);
		ui_update_cnt++;
        if (game_status.game_progress == 0 || game_status.game_progress == 1 || game_status.game_progress == 5) 
		{
			if(ui_update_cnt % 5 == 0)	
				ui_update();
			else
				ui_init();
        } 
		else if(rc.kb.bit.X)
		{
			ui_init();
        }
		else
		{
			ui_update();
		}
        us_timer_interval_test_end(&ui_time);
    }
}
