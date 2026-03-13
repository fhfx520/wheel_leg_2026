//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_line_t *ui_g_1_left_big_leg;
extern ui_interface_line_t *ui_g_1_left_small_leg;
extern ui_interface_line_t *ui_g_1_right_big_leg;
extern ui_interface_line_t *ui_g_1_right_small_leg;
extern ui_interface_rect_t *ui_g_1_supercap;
extern ui_interface_rect_t *ui_g_1_supercap_capcity;
extern ui_interface_number_t *ui_g_1_supcap_voltage;

void ui_init_g_1();
void ui_update_g_1();
void ui_remove_g_1();

extern ui_interface_number_t *ui_g_2_current_velocity;
extern ui_interface_number_t *ui_g_2_target_velocity;
extern ui_interface_arc_t *ui_g_2_head_position;
extern ui_interface_number_t *ui_g_2_vision_trice_id;
extern ui_interface_number_t *ui_g_2_vision_order_id;
extern ui_interface_round_t *ui_g_2_current_shoot_mode;

void ui_init_g_2();
void ui_update_g_2();
void ui_remove_g_2();

extern ui_interface_string_t *ui_g_3_high_flag;

void ui_init_g_3();
void ui_update_g_3();
void ui_remove_g_3();

extern ui_interface_string_t *ui_g_4_fly_flag;

void ui_init_g_4();
void ui_update_g_4();
void ui_remove_g_4();

extern ui_interface_rect_t *ui_g_5_vision_frame;
extern ui_interface_string_t *ui_g_5_shoot_mode_aim;
extern ui_interface_string_t *ui_g_5_shoot_mode_energy;

void ui_init_g_5();
void ui_update_g_5();
void ui_remove_g_5();


#endif // UI_g_H
