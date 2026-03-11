//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_g_1_0;

ui_interface_line_t *ui_g_1_left_big_leg = (ui_interface_line_t*)&(ui_g_1_0.data[0]);
ui_interface_line_t *ui_g_1_left_small_leg = (ui_interface_line_t*)&(ui_g_1_0.data[1]);
ui_interface_line_t *ui_g_1_right_big_leg = (ui_interface_line_t*)&(ui_g_1_0.data[2]);
ui_interface_line_t *ui_g_1_right_small_leg = (ui_interface_line_t*)&(ui_g_1_0.data[3]);
ui_interface_rect_t *ui_g_1_supercap = (ui_interface_rect_t*)&(ui_g_1_0.data[4]);
ui_interface_rect_t *ui_g_1_supercap_capcity = (ui_interface_rect_t*)&(ui_g_1_0.data[5]);
ui_interface_number_t *ui_g_1_supcap_voltage = (ui_interface_number_t*)&(ui_g_1_0.data[6]);

void _ui_init_g_1_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_1_0.data[i].figure_name[0] = 0;
        ui_g_1_0.data[i].figure_name[1] = 0;
        ui_g_1_0.data[i].figure_name[2] = i + 0;
        ui_g_1_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_1_0.data[i].operate_type = 0;
    }

    ui_g_1_left_big_leg->figure_type = 0;
    ui_g_1_left_big_leg->operate_type = 1;
    ui_g_1_left_big_leg->layer = 0;
    ui_g_1_left_big_leg->color = 2;
    ui_g_1_left_big_leg->start_x = 639;
    ui_g_1_left_big_leg->start_y = 460;
    ui_g_1_left_big_leg->width = 5;
    ui_g_1_left_big_leg->end_x = 535;
    ui_g_1_left_big_leg->end_y = 460;

    ui_g_1_left_small_leg->figure_type = 0;
    ui_g_1_left_small_leg->operate_type = 1;
    ui_g_1_left_small_leg->layer = 0;
    ui_g_1_left_small_leg->color = 2;
    ui_g_1_left_small_leg->start_x = 463;
    ui_g_1_left_small_leg->start_y = 384;
    ui_g_1_left_small_leg->width = 5;
    ui_g_1_left_small_leg->end_x = 535;
    ui_g_1_left_small_leg->end_y = 461;

    ui_g_1_right_big_leg->figure_type = 0;
    ui_g_1_right_big_leg->operate_type = 1;
    ui_g_1_right_big_leg->layer = 0;
    ui_g_1_right_big_leg->color = 2;
    ui_g_1_right_big_leg->start_x = 1283;
    ui_g_1_right_big_leg->start_y = 460;
    ui_g_1_right_big_leg->width = 5;
    ui_g_1_right_big_leg->end_x = 1392;
    ui_g_1_right_big_leg->end_y = 460;

    ui_g_1_right_small_leg->figure_type = 0;
    ui_g_1_right_small_leg->operate_type = 1;
    ui_g_1_right_small_leg->layer = 0;
    ui_g_1_right_small_leg->color = 2;
    ui_g_1_right_small_leg->start_x = 1450;
    ui_g_1_right_small_leg->start_y = 374;
    ui_g_1_right_small_leg->width = 5;
    ui_g_1_right_small_leg->end_x = 1387;
    ui_g_1_right_small_leg->end_y = 461;

    ui_g_1_supercap->figure_type = 1;
    ui_g_1_supercap->operate_type = 1;
    ui_g_1_supercap->layer = 0;
    ui_g_1_supercap->color = 2;
    ui_g_1_supercap->start_x = 670;
    ui_g_1_supercap->start_y = 780;
    ui_g_1_supercap->width = 5;
    ui_g_1_supercap->end_x = 1270;
    ui_g_1_supercap->end_y = 830;

    ui_g_1_supercap_capcity->figure_type = 1;
    ui_g_1_supercap_capcity->operate_type = 1;
    ui_g_1_supercap_capcity->layer = 0;
    ui_g_1_supercap_capcity->color = 5;
    ui_g_1_supercap_capcity->start_x = 685;
    ui_g_1_supercap_capcity->start_y = 795;
    ui_g_1_supercap_capcity->width = 25;
    ui_g_1_supercap_capcity->end_x = 1255;
    ui_g_1_supercap_capcity->end_y = 815;

    ui_g_1_supcap_voltage->figure_type = 5;
    ui_g_1_supcap_voltage->operate_type = 1;
    ui_g_1_supcap_voltage->layer = 0;
    ui_g_1_supcap_voltage->color = 6;
    ui_g_1_supcap_voltage->start_x = 486;
    ui_g_1_supcap_voltage->start_y = 829;
    ui_g_1_supcap_voltage->width = 4;
    ui_g_1_supcap_voltage->font_size = 40;
    ui_g_1_supcap_voltage->number = 30100;


    ui_proc_7_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}

void _ui_update_g_1_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_1_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}

void _ui_remove_g_1_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_1_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}


void ui_init_g_1() {
    _ui_init_g_1_0();
}

void ui_update_g_1() {
    _ui_update_g_1_0();
}

void ui_remove_g_1() {
    _ui_remove_g_1_0();
}

ui_5_frame_t ui_g_2_0;

ui_interface_number_t *ui_g_2_current_velocity = (ui_interface_number_t*)&(ui_g_2_0.data[0]);
ui_interface_number_t *ui_g_2_target_velocity = (ui_interface_number_t*)&(ui_g_2_0.data[1]);
ui_interface_arc_t *ui_g_2_head_position = (ui_interface_arc_t*)&(ui_g_2_0.data[2]);

void _ui_init_g_2_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_2_0.data[i].figure_name[0] = 0;
        ui_g_2_0.data[i].figure_name[1] = 1;
        ui_g_2_0.data[i].figure_name[2] = i + 0;
        ui_g_2_0.data[i].operate_type = 1;
    }
    for (int i = 3; i < 5; i++) {
        ui_g_2_0.data[i].operate_type = 0;
    }

    ui_g_2_current_velocity->figure_type = 5;
    ui_g_2_current_velocity->operate_type = 1;
    ui_g_2_current_velocity->layer = 0;
    ui_g_2_current_velocity->color = 6;
    ui_g_2_current_velocity->start_x = 914;
    ui_g_2_current_velocity->start_y = 761;
    ui_g_2_current_velocity->width = 3;
    ui_g_2_current_velocity->font_size = 30;
    ui_g_2_current_velocity->number = 2500;

    ui_g_2_target_velocity->figure_type = 5;
    ui_g_2_target_velocity->operate_type = 1;
    ui_g_2_target_velocity->layer = 0;
    ui_g_2_target_velocity->color = 3;
    ui_g_2_target_velocity->start_x = 1285;
    ui_g_2_target_velocity->start_y = 823;
    ui_g_2_target_velocity->width = 3;
    ui_g_2_target_velocity->font_size = 30;
    ui_g_2_target_velocity->number = 2500;

    ui_g_2_head_position->figure_type = 4;
    ui_g_2_head_position->operate_type = 1;
    ui_g_2_head_position->layer = 0;
    ui_g_2_head_position->color = 2;
    ui_g_2_head_position->start_x = 960;
    ui_g_2_head_position->start_y = 540;
    ui_g_2_head_position->width = 5;
    ui_g_2_head_position->start_angle = 0;
    ui_g_2_head_position->end_angle = 60;
    ui_g_2_head_position->rx = 50;
    ui_g_2_head_position->ry = 50;


    ui_proc_5_frame(&ui_g_2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_2_0, sizeof(ui_g_2_0));
}

void _ui_update_g_2_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_2_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_2_0, sizeof(ui_g_2_0));
}

void _ui_remove_g_2_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_2_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_2_0, sizeof(ui_g_2_0));
}


void ui_init_g_2() {
    _ui_init_g_2_0();
}

void ui_update_g_2() {
    _ui_update_g_2_0();
}

void ui_remove_g_2() {
    _ui_remove_g_2_0();
}


ui_string_frame_t ui_g_3_0;
ui_interface_string_t* ui_g_3_high_flag = &(ui_g_3_0.option);

void _ui_init_g_3_0() {
    ui_g_3_0.option.figure_name[0] = 0;
    ui_g_3_0.option.figure_name[1] = 2;
    ui_g_3_0.option.figure_name[2] = 0;
    ui_g_3_0.option.operate_type = 1;

    ui_g_3_high_flag->figure_type = 7;
    ui_g_3_high_flag->operate_type = 1;
    ui_g_3_high_flag->layer = 0;
    ui_g_3_high_flag->color = 1;
    ui_g_3_high_flag->start_x = 101;
    ui_g_3_high_flag->start_y = 774;
    ui_g_3_high_flag->width = 5;
    ui_g_3_high_flag->font_size = 50;
    ui_g_3_high_flag->str_length = 3;
    strcpy(ui_g_3_high_flag->string, "Man");


    ui_proc_string_frame(&ui_g_3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_3_0, sizeof(ui_g_3_0));
}

void _ui_update_g_3_0() {
    ui_g_3_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_3_0, sizeof(ui_g_3_0));
}

void _ui_remove_g_3_0() {
    ui_g_3_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_3_0, sizeof(ui_g_3_0));
}

void ui_init_g_3() {
    _ui_init_g_3_0();
}

void ui_update_g_3() {
    _ui_update_g_3_0();
}

void ui_remove_g_3() {
    _ui_remove_g_3_0();
}


ui_string_frame_t ui_g_4_0;
ui_interface_string_t* ui_g_4_fly_flag = &(ui_g_4_0.option);

void _ui_init_g_4_0() {
    ui_g_4_0.option.figure_name[0] = 0;
    ui_g_4_0.option.figure_name[1] = 3;
    ui_g_4_0.option.figure_name[2] = 0;
    ui_g_4_0.option.operate_type = 1;

    ui_g_4_fly_flag->figure_type = 7;
    ui_g_4_fly_flag->operate_type = 1;
    ui_g_4_fly_flag->layer = 0;
    ui_g_4_fly_flag->color = 6;
    ui_g_4_fly_flag->start_x = 806;
    ui_g_4_fly_flag->start_y = 357;
    ui_g_4_fly_flag->width = 4;
    ui_g_4_fly_flag->font_size = 40;
    ui_g_4_fly_flag->str_length = 8;
    strcpy(ui_g_4_fly_flag->string, "        ");


    ui_proc_string_frame(&ui_g_4_0);
    SEND_MESSAGE((uint8_t *) &ui_g_4_0, sizeof(ui_g_4_0));
}

void _ui_update_g_4_0() {
    ui_g_4_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_4_0);
    SEND_MESSAGE((uint8_t *) &ui_g_4_0, sizeof(ui_g_4_0));
}

void _ui_remove_g_4_0() {
    ui_g_4_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_4_0);
    SEND_MESSAGE((uint8_t *) &ui_g_4_0, sizeof(ui_g_4_0));
}

void ui_init_g_4() {
    _ui_init_g_4_0();
}

void ui_update_g_4() {
    _ui_update_g_4_0();
}

void ui_remove_g_4() {
    _ui_remove_g_4_0();
}

