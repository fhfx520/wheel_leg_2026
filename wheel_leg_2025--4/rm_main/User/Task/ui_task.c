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

#include <stdint.h>
#include <string.h>

#define UI_TASK_PERIOD_MS             5U
#define UI_TX_PERIOD_MS               34U
#define UI_VISION_FRAME_REFRESH_MS    100U
#define UI_RECOVER_REINIT_PERIOD_MS   1000U
#define UI_STARTUP_INIT_ROUNDS        3U
#define UI_MANUAL_INIT_ROUNDS         3U
#define UI_INIT_FRAME_NUM             6U
#define UI_LOW_STATUS_ADD_TICKS       40
#define UI_LOW_STATUS_MAX_TICKS       120
#define UI_FLY_STATUS_TICKS           40

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

extern void _ui_init_g_5_0(void);
extern void _ui_init_g_5_1(void);
extern ui_7_frame_t ui_g_1_0;
extern ui_7_frame_t ui_g_2_0;
extern ui_string_frame_t ui_g_3_0;
extern ui_string_frame_t ui_g_4_0;
extern ui_string_frame_t ui_g_5_0;
extern ui_string_frame_t ui_g_5_1;

typedef struct {
    uint8_t full_init_pending;
    uint8_t init_step;
    uint8_t init_repeat_count;
    uint8_t current_add_pending;
    uint8_t current_add_step;
    uint8_t current_add_repeat_count;

    uint8_t g1_dirty;
    uint8_t g2_dirty;
    uint8_t g3_dirty;
    uint8_t g4_dirty;
    uint8_t vision_frame_dirty;
    uint8_t last_vision_frame_color;

    uint8_t next_dynamic_group;
    uint8_t last_game_progress;
    uint8_t last_force_init;

    uint32_t last_tx_tick;
    uint32_t last_vision_frame_tick;
    uint32_t last_reinit_request_tick;
} ui_sched_t;

static ui_sched_t ui_sched = {
    .last_game_progress = 0xff,
    .last_vision_frame_color = 0xff,
};

static void vision_data_cb(uint32_t tag_id, void* data, size_t len)
{
    if(data == NULL || len != sizeof(vision_data_t))
        return;
    memcpy(&ui_get_vision_data_container, (vision_data_t*)data, len);
}

static const ContainerBusCfg mb_callback[] = {
    { TAG_TRACE_VISION_DATA, vision_data_cb, NULL },
};

static int8_t ui_sign_function(void)
{
    return (!wlr.direction ? 1 : -1);
}

static uint8_t ui_time_is_elapsed(uint32_t now, uint32_t last, uint32_t interval)
{
    return (uint32_t)(now - last) >= interval;
}

static uint8_t ui_set_string_if_changed(char *dst, const char *src)
{
    if (strcmp(dst, src) == 0) {
        return 0;
    }

    strcpy(dst, src);
    return 1;
}

static void ui_request_full_init_rounds(uint8_t rounds)
{
    if (rounds == 0) {
        return;
    }

    ui_sched.full_init_pending = 1;
    ui_sched.init_step = 0;
    ui_sched.init_repeat_count = rounds - 1;
    ui_sched.current_add_pending = 0;
}

static void ui_request_current_add_rounds(uint8_t rounds)
{
    if (rounds == 0) {
        return;
    }

    ui_sched.current_add_pending = 1;
    ui_sched.current_add_step = 0;
    ui_sched.current_add_repeat_count = rounds - 1;
}

static void ui_send_current_7_frame(ui_7_frame_t *frame, uint8_t operate_type)
{
    for (int i = 0; i < 7; i++) {
        frame->data[i].operate_type = operate_type;
    }

    ui_proc_7_frame(frame);
    SEND_MESSAGE((uint8_t *)frame, sizeof(*frame));
}

static void ui_send_current_string_frame(ui_string_frame_t *frame, uint8_t operate_type)
{
    frame->option.operate_type = operate_type;
    ui_proc_string_frame(frame);
    SEND_MESSAGE((uint8_t *)frame, sizeof(*frame));
}

static void ui_send_vision_frame(uint8_t operate_type)
{
    static ui_1_frame_t vision_frame;

    memcpy(&vision_frame.data[0], ui_g_2_vision_frame, sizeof(vision_frame.data[0]));
    vision_frame.data[0].operate_type = operate_type;
    ui_proc_1_frame(&vision_frame);
    SEND_MESSAGE((uint8_t *)&vision_frame, sizeof(vision_frame));
}

static void ui_send_current_add_frame(uint8_t step)
{
    switch (step) {
    case 0:
        ui_send_current_7_frame(&ui_g_2_0, 1);
        ui_sched.g2_dirty = 0;
        ui_sched.vision_frame_dirty = 0;
        break;
    case 1:
        ui_send_current_7_frame(&ui_g_1_0, 1);
        ui_sched.g1_dirty = 0;
        break;
    case 2:
        ui_send_current_string_frame(&ui_g_3_0, 1);
        ui_sched.g3_dirty = 0;
        break;
    case 3:
        ui_send_current_string_frame(&ui_g_4_0, 1);
        ui_sched.g4_dirty = 0;
        break;
    case 4:
        ui_send_current_string_frame(&ui_g_5_0, 1);
        break;
    case 5:
        ui_send_current_string_frame(&ui_g_5_1, 1);
        break;
    default:
        break;
    }
}

static void ui_send_init_frame(uint8_t step)
{
    switch (step) {
    case 0:
        ui_init_g_2();
        break;
    case 1:
        ui_init_g_1();
        break;
    case 2:
        ui_init_g_3();
        break;
    case 3:
        ui_init_g_4();
        break;
    case 4:
        _ui_init_g_5_0();
        break;
    case 5:
        _ui_init_g_5_1();
        break;
    default:
        break;
    }
}

static void ui_service_tx(uint32_t now)
{
    if (!ui_time_is_elapsed(now, ui_sched.last_tx_tick, UI_TX_PERIOD_MS)) {
        return;
    }

    if (ui_sched.full_init_pending) {
        ui_send_init_frame(ui_sched.init_step++);
        if (ui_sched.init_step >= UI_INIT_FRAME_NUM) {
            if (ui_sched.init_repeat_count > 0) {
                ui_sched.init_repeat_count--;
                ui_sched.init_step = 0;
            } else {
                ui_sched.full_init_pending = 0;
                ui_request_current_add_rounds(2);
            }
        }
        ui_sched.last_tx_tick = now;
        return;
    }

    if (ui_sched.current_add_pending) {
        ui_send_current_add_frame(ui_sched.current_add_step++);
        if (ui_sched.current_add_step >= UI_INIT_FRAME_NUM) {
            if (ui_sched.current_add_repeat_count > 0) {
                ui_sched.current_add_repeat_count--;
                ui_sched.current_add_step = 0;
            } else {
                ui_sched.current_add_pending = 0;
            }
        }
        ui_sched.last_tx_tick = now;
        return;
    }

    if (ui_sched.vision_frame_dirty ||
        ui_time_is_elapsed(now, ui_sched.last_vision_frame_tick, UI_VISION_FRAME_REFRESH_MS)) {
        ui_send_vision_frame(2);
        ui_sched.vision_frame_dirty = 0;
        ui_sched.last_vision_frame_tick = now;
    } else if (ui_sched.g3_dirty) {
        ui_update_g_3();
        ui_sched.g3_dirty = 0;
    } else if (ui_sched.g4_dirty) {
        ui_update_g_4();
        ui_sched.g4_dirty = 0;
    } else if (ui_sched.next_dynamic_group == 0 && ui_sched.g1_dirty) {
        ui_update_g_1();
        ui_sched.g1_dirty = 0;
        ui_sched.next_dynamic_group = 1;
    } else if (ui_sched.g2_dirty) {
        ui_update_g_2();
        ui_sched.g2_dirty = 0;
        ui_sched.next_dynamic_group = 0;
    } else if (ui_sched.g1_dirty) {
        ui_update_g_1();
        ui_sched.g1_dirty = 0;
        ui_sched.next_dynamic_group = 1;
    } else {
        return;
    }

    ui_sched.last_tx_tick = now;
}

extern chassis_scale_t chassis_scale;

float armour_center, start_angle, end_angle;

void ui_init(void)
{
    ui_request_full_init_rounds(UI_MANUAL_INIT_ROUNDS);
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
    ui_sched.g1_dirty = 1;

    ui_g_2_target_velocity->number = (int)(lqr.X_ref[1] * 1000.0f) / 100 * 100;
    ui_g_2_current_velocity->number = (int)(fabsf(wlr.v_fdb) * 1000.0f) / 100 * 100;

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

    uint8_t vision_frame_color = (ui_get_vision_data_container.vision_trace_id ? (ui_get_vision_data_container.vision_enanle ? 4 : 1) : 8);
    ui_g_2_vision_frame->color = vision_frame_color;
    if (ui_sched.last_vision_frame_color != vision_frame_color) {
        ui_sched.last_vision_frame_color = vision_frame_color;
        ui_sched.vision_frame_dirty = 1;
    }
    ui_sched.g2_dirty = 1;

    if(last_status_high != wlr.high_flag)
        lowhz_cnt += UI_LOW_STATUS_ADD_TICKS;
    if(last_status_jump != wlr.jump_flag && wlr.jump_flag == WLR_JUMP_ASCEND)
        lowhz_cnt += UI_LOW_STATUS_ADD_TICKS;
    if(last_status_sky != wlr.sky_flag && wlr.sky_flag == WLR_SKY_FOLDING)
        lowhz_cnt += UI_LOW_STATUS_ADD_TICKS;
    if(last_status_sky != wlr.sky_flag && wlr.sky_flag >= WLR_SKY_EXTENDING)
        lowhz_cnt += UI_LOW_STATUS_ADD_TICKS;
    if(wlr.side[0].fly_flag == 1 && wlr.side[1].fly_flag == 1)
        fly_cnt = UI_FLY_STATUS_TICKS;
    if(lowhz_cnt > UI_LOW_STATUS_MAX_TICKS)
        lowhz_cnt = UI_LOW_STATUS_MAX_TICKS;
    if(fly_cnt > UI_FLY_STATUS_TICKS)
        fly_cnt = UI_FLY_STATUS_TICKS;
    last_status_high = wlr.high_flag;
    last_status_jump = wlr.jump_flag;
    last_status_sky = wlr.sky_flag;

    if(lowhz_cnt > 0){
        lowhz_cnt--;
        if(wlr.jump_flag == WLR_JUMP_ASCEND)
            ui_sched.g3_dirty |= ui_set_string_if_changed(ui_g_3_high_flag->string, "High ");
        else if(wlr.sky_flag == WLR_SKY_FOLDING)
            ui_sched.g3_dirty |= ui_set_string_if_changed(ui_g_3_high_flag->string, "Short");
        else if(wlr.sky_flag > WLR_SKY_FOLDING)
            ui_sched.g3_dirty |= ui_set_string_if_changed(ui_g_3_high_flag->string, "Fly  ");
        else if (wlr.high_flag == 0)
            ui_sched.g3_dirty |= ui_set_string_if_changed(ui_g_3_high_flag->string, "Low  ");
        else if (wlr.high_flag == 1)
            ui_sched.g3_dirty |= ui_set_string_if_changed(ui_g_3_high_flag->string, "Mid  ");
        else
            ui_sched.g3_dirty |= ui_set_string_if_changed(ui_g_3_high_flag->string, "Man  ");
        ui_sched.g3_dirty = 1;
    }

    if(fly_cnt > 0)
    {
        fly_cnt--;
        ui_sched.g4_dirty |= ui_set_string_if_changed(ui_g_4_fly_flag->string, "off_land");
        ui_sched.g4_dirty = 1;
    }
    else
    {
        ui_sched.g4_dirty |= ui_set_string_if_changed(ui_g_4_fly_flag->string, "        ");
    }
}

uint32_t ui_update_cnt = 0;
void ui_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    ui_request_full_init_rounds(UI_STARTUP_INIT_ROUNDS);
    container_bus_init(mb_callback, sizeof(mb_callback)/sizeof(ContainerBusCfg));
    ui_sched.last_tx_tick = osKernelSysTick() - UI_TX_PERIOD_MS;
    ui_sched.last_reinit_request_tick = osKernelSysTick();

    for(;;)
    {
        thread_wake_time = osKernelSysTick();
        us_timer_interval_test_start(&ui_time);
        ui_update_cnt++;

        ui_update();

        if (game_status.game_progress != ui_sched.last_game_progress) {
            ui_sched.last_game_progress = game_status.game_progress;
            ui_init();
        }

        if(rc.kb.bit.X && !ui_sched.last_force_init) {
            ui_init();
        }
        ui_sched.last_force_init = rc.kb.bit.X;

        if (!ui_sched.full_init_pending &&
            !ui_sched.current_add_pending &&
            ui_time_is_elapsed(thread_wake_time, ui_sched.last_reinit_request_tick, UI_RECOVER_REINIT_PERIOD_MS)) {
            ui_sched.last_reinit_request_tick = thread_wake_time;
            ui_request_current_add_rounds(1);
        }

        ui_service_tx(thread_wake_time);

        us_timer_interval_test_end(&ui_time);
        osDelayUntil(&thread_wake_time, UI_TASK_PERIOD_MS);
    }
}
