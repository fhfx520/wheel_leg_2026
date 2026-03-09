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

void ui_init_g_2();
void ui_update_g_2();
void ui_remove_g_2();

extern ui_interface_string_t *ui_g_3_high_flag;

void ui_init_g_3();
void ui_update_g_3();
void ui_remove_g_3();


#endif // UI_g_H
