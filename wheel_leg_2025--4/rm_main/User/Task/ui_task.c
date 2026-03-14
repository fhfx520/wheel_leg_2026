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
#include "chassis_task.h"
#include "board_comm.h"
#include "container.h"

us_time_t ui_time;
int32_t lowhz_cnt;

static uint8_t last_status_high;
static uint8_t last_status_jump;
static uint8_t last_status_sky;
static uint8_t last_status_vision_online;
static uint8_t last_status_ID_choice;
static uint8_t last_status_ID_aiming;
static uint8_t fly_flag;
static uint8_t fly_cnt;

static vision_data_t ui_get_vision_data_container;

// 收到vision数据：
static void vision_data_cb(uint32_t tag_id, void* data, size_t len) {
	if(data == NULL || len != sizeof(vision_data_t))
		return;
    memcpy(&ui_get_vision_data_container,(vision_data_t*)data,len);
}

// --- 回调配置表  ---
static const ContainerBusCfg mb_callback[] = {
    { TAG_TRACE_VISION_DATA, vision_data_cb, NULL },
};

static int8_t ui_sign_function(void)
{
	return (!wlr.direction ? 1 : -1);
}


extern chassis_scale_t chassis_scale;

float armour_center;

void ui_init(void)
{
	ui_init_g_1();
	ui_init_g_2();
	ui_init_g_3();
	ui_init_g_4();
	ui_init_g_5();
}

void ui_update(void)
{	
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
	ui_update_g_1();
	
	ui_g_2_target_velocity->number = (int)(chassis_scale.keyboard * 1000.0f) / 100 * 100;
	ui_g_2_current_velocity->number = (int)(wlr.v_fdb * 1000.0f) / 100 * 100;
	
	float yaw_err;
    yaw_err = circle_error((float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI, (float)yaw_motor.ecd / 8192 * 2 * PI, 2 * PI);
    armour_center = -1.0f * yaw_err * 180.0f / PI + 180.0f;
	ui_g_2_head_position->start_angle = armour_center - 15.0f;     
	if(ui_g_2_head_position->start_angle < 0.0f)
        ui_g_2_head_position->start_angle += 360.0f;
    if(ui_g_2_head_position->start_angle > 360.0f)
        ui_g_2_head_position->start_angle -= 360.0f;
    ui_g_2_head_position->end_angle = armour_center + 15.0f; 	
    if(ui_g_2_head_position->end_angle < 0.0f)
        ui_g_2_head_position->end_angle += 360.0f;
    if(ui_g_2_head_position->end_angle > 360.0f)
        ui_g_2_head_position->end_angle -= 360.0f;
	
	ui_g_2_vision_order_id->number = ID_judge;
	ui_g_2_vision_trice_id->number = ui_get_vision_data_container.vision_trace_id;
	ui_g_2_current_shoot_mode->start_y = (wlr.energy_flag ? 638 : 673);
	
	ui_g_2_vision_frame->color = (ui_get_vision_data_container.vision_trace_id ? (ui_get_vision_data_container.vision_enanle ? 4 : 1) : 8);
	
	ui_update_g_2();
	
	if(last_status_high != wlr.high_flag)
		lowhz_cnt += 20;
	if(last_status_jump != wlr.jump_flag && wlr.jump_flag == WLR_JUMP_ASCEND)
		lowhz_cnt += 20;
	if(last_status_sky != wlr.sky_flag && wlr.sky_flag == WLR_SKY_FOLDING)
		lowhz_cnt += 20;
	if(last_status_sky != wlr.sky_flag && wlr.sky_flag >= WLR_SKY_EXTENDING)
		lowhz_cnt += 20;
	if(wlr.side[0].fly_flag == 1 && wlr.side[1].fly_flag == 1)
		fly_cnt += 20;
	if(lowhz_cnt > 60)
		lowhz_cnt = 60;
	if(fly_cnt > 40)
		fly_cnt = 40;
	last_status_high = wlr.high_flag;
	last_status_jump = wlr.jump_flag;
	last_status_sky = wlr.sky_flag;
	
	if(lowhz_cnt > 0){
		lowhz_cnt--;
		if(wlr.jump_flag == WLR_JUMP_ASCEND)
			strcpy(ui_g_3_high_flag->string, "High ");
		else if(wlr.sky_flag == WLR_SKY_FOLDING)
			strcpy(ui_g_3_high_flag->string, "Short");
		else if(wlr.sky_flag > WLR_SKY_FOLDING)
			strcpy(ui_g_3_high_flag->string, "Fly  ");
		else if (wlr.high_flag == 0)
			strcpy(ui_g_3_high_flag->string, "Low  ");
		else if (wlr.high_flag == 1)
			strcpy(ui_g_3_high_flag->string, "Mid  ");
		else
			strcpy(ui_g_3_high_flag->string, "Man  ");
		ui_update_g_3();
	}
	if(fly_cnt >= 15)
	{
		fly_cnt--;
		strcpy(ui_g_4_fly_flag->string, "off_land");
		ui_update_g_4();
	}
	else if(fly_cnt < 15 && fly_cnt > 0)
	{
		fly_cnt--;	
		strcpy(ui_g_4_fly_flag->string, "        ");
		ui_update_g_4();
	}
	
	
}

uint32_t ui_update_cnt = 0;
void ui_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    ui_init();
	container_bus_init(mb_callback, sizeof(mb_callback)/sizeof(ContainerBusCfg));
    for(int i = 0; i < 30; i++) {
        thread_wake_time = osKernelSysTick();
        ui_init();
        osDelayUntil(&thread_wake_time, 1);
    }
    for(;;)
    {
        thread_wake_time = osKernelSysTick();
        us_timer_interval_test_start(&ui_time);
		ui_update_cnt++;
        if (game_status.game_progress == 0 || game_status.game_progress == 1 || game_status.game_progress == 5) 
		{
			if(ui_update_cnt % 20 == 0)
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
        osDelayUntil(&thread_wake_time, 10);
    }
}
