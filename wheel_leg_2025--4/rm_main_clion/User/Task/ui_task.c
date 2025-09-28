#include "ui_task.h"
#include "cmsis_os.h"
#include "prot_judge.h"
#include "prot_vision.h"
#include "prot_dr16.h"
#include "ui_default_group1_0.h"
#include "ui_default_group2_0.h"
#include "ui_default_group2_1.h"
#include "ui_default_group2_2.h"
#include "ui_default_group2_3.h"
#include "ui_default_group2_4.h"
#include "ui_default_group3_0.h"
#include "ui_default_group4_0.h"
#include "ui_default_group5_0.h"
#include "ui_default_group5_1.h"
#include "ui_default_Ungroup_try_new_0.h"
#include "ui_default_Ungroup_vision_rec_0.h"
#include "us_time.h"
#include "ui_default_lock_0.h"
#include "ui_g_Ungroup_0.h"
#include "ui_default_Ungroup_0.h"
#include "wlr.h"
#include "prot_vision.h"
#include "ui_g_Ungroup_leg_0.h"

us_time_t ui_time;
int32_t lowhz_cnt;

static uint8_t last_status_high;
static uint8_t last_status_vision_online;
static uint8_t last_status_ID_choice;
static uint8_t last_status_ID_aiming;

void ui_init(void)
{
			_ui_init_default_group1_0();
			_ui_init_default_group3_0();
	
			_ui_init_g_Ungroup_0();
		
			_ui_init_default_Ungroup_0();
//		_ui_init_default_lock_0();
		_ui_init_g_Ungroup_leg_0();
}

void ui_update(void)
{
	
	
			if(last_status_high != wlr.high_flag)
				lowhz_cnt += 50;
			
			last_status_high = wlr.high_flag;
	
			if(last_status_vision_online != vision.rx_ui_msg.data.vision_online)
				lowhz_cnt += 50;
			last_status_vision_online = vision.rx_ui_msg.data.vision_online;
			
				
			if(last_status_ID_choice !=ID_judge)
				lowhz_cnt += 50;
			last_status_ID_choice = ID_judge;
			
						if(last_status_ID_aiming !=vision.rx_ui_msg.data.vision_trace_id)
				lowhz_cnt += 50;
			last_status_ID_aiming = vision.rx_ui_msg.data.vision_trace_id;
						
			
	
	if(lowhz_cnt > 0){
		lowhz_cnt--;
	_ui_update_g_Ungroup_0();
	_ui_update_default_Ungroup_0();
}
	
	else	{

    _ui_update_default_group3_0();
	}
	
	
	//    _ui_update_default_group4_0();
//    _ui_update_default_group5_0();
//    _ui_update_default_group5_1();
//	
//	_ui_update_default_Ungroup_try_new_0();
// 	_ui_update_default_Ungroup_vision_rec_0();
//	if(vision.rx_ui_msg.data.vision_trace_id != 0)
//		_ui_update_default_lock_0();
//	else
//		_ui_remove_default_lock_0();
	
}

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
        if (game_status.game_progress == 0 || game_status.game_progress == 1 || game_status.game_progress == 5 || rc.kb.bit.F) {
            ui_init();
        } else {
            ui_update();
        }
        us_timer_interval_test_end(&ui_time);
        osDelayUntil(&thread_wake_time, 10);
    }
}
