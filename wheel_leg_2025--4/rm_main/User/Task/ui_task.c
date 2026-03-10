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

us_time_t ui_time;
int32_t lowhz_cnt;

static uint8_t last_status_high;
static uint8_t last_status_vision_online;
static uint8_t last_status_ID_choice;
static uint8_t last_status_ID_aiming;

float armour_center;

void ui_init(void)
{
	ui_init_g_1();
	ui_init_g_2();
	ui_init_g_3();
	ui_init_g_4();
}

void ui_update(void)
{	
	ui_g_1_left_big_leg->end_x = ui_g_1_left_big_leg->start_x - (vmc[((!wlr.direction) ? 1 : 0)].mp_fdb.xd * 300);
	ui_g_1_left_big_leg->end_y = ui_g_1_left_big_leg->start_y - (vmc[((!wlr.direction) ? 1 : 0)].mp_fdb.yd * 300);
	ui_g_1_left_small_leg->start_x = ui_g_1_left_big_leg->end_x;
	ui_g_1_left_small_leg->start_y = ui_g_1_left_big_leg->end_y;
	ui_g_1_left_small_leg->end_x = ui_g_1_left_big_leg->start_x - (vmc[((!wlr.direction) ? 1 : 0)].mp_fdb.xc * 300);
	ui_g_1_left_small_leg->end_y = ui_g_1_left_big_leg->start_y - (vmc[((!wlr.direction) ? 1 : 0)].mp_fdb.yc * 300);
	
	ui_g_1_right_big_leg->end_x = ui_g_1_right_big_leg->start_x - (vmc[((!wlr.direction) ? 0 : 1)].mp_fdb.xd * 300);
	ui_g_1_right_big_leg->end_y = ui_g_1_right_big_leg->start_y - (vmc[((!wlr.direction) ? 0 : 1)].mp_fdb.yd * 300);
	ui_g_1_right_small_leg->start_x = ui_g_1_right_big_leg->end_x;
	ui_g_1_right_small_leg->start_y = ui_g_1_right_big_leg->end_y;
	ui_g_1_right_small_leg->end_x = ui_g_1_right_big_leg->start_x - (vmc[((!wlr.direction) ? 0 : 1)].mp_fdb.xc * 300);
	ui_g_1_right_small_leg->end_y = ui_g_1_right_big_leg->start_y - (vmc[((!wlr.direction) ? 0 : 1)].mp_fdb.yc * 300);
	
	ui_g_1_supercap_capcity->end_x = ui_g_1_supercap_capcity->start_x + (570.0f) * (supercap.volume_percent / 100.0f);
	ui_g_1_supcap_voltage->number = (int)(supercap.volage * 1000.0f) / 100 * 100;
	ui_update_g_1();
	
	ui_g_2_target_velocity->number = (int)(wlr.v_ref * 1000.0f) / 100 * 100;
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
	ui_update_g_2();
	
	if(last_status_high != wlr.high_flag)
		lowhz_cnt += 50;
	if(wlr.side[0].fly_flag == 1 && wlr.side[1].fly_flag == 1)
		lowhz_cnt += 50;
	last_status_high = wlr.high_flag;
	
	if(lowhz_cnt > 0){
		lowhz_cnt--;
		if (wlr.high_flag == 0)
			strcpy(ui_g_3_high_flag->string, "Low");
		else if (wlr.high_flag == 1)
			strcpy(ui_g_3_high_flag->string, "Mid");
		else
			strcpy(ui_g_3_high_flag->string, "Man");
		ui_update_g_3();
		if(wlr.side[0].fly_flag == 1 && wlr.side[1].fly_flag == 1)
			strcpy(ui_g_4_fly_flag->string, "off_land");
		else
			strcpy(ui_g_4_fly_flag->string, "	");
		ui_update_g_4();
	}
}

uint32_t ui_update_cnt = 0;
void ui_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    ui_init();
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
