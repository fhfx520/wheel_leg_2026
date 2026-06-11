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
ui_interface_round_t *ui_g_1_current_high_flag = (ui_interface_round_t*)&(ui_g_1_0.data[3]);
ui_interface_line_t *ui_g_1_right_small_leg = (ui_interface_line_t*)&(ui_g_1_0.data[4]);
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
    ui_g_1_left_big_leg->start_x = 652;
    ui_g_1_left_big_leg->start_y = 440;
    ui_g_1_left_big_leg->width = 5;
    ui_g_1_left_big_leg->end_x = 540;
    ui_g_1_left_big_leg->end_y = 440;

    ui_g_1_left_small_leg->figure_type = 0;
    ui_g_1_left_small_leg->operate_type = 1;
    ui_g_1_left_small_leg->layer = 0;
    ui_g_1_left_small_leg->color = 2;
    ui_g_1_left_small_leg->start_x = 468;
    ui_g_1_left_small_leg->start_y = 363;
    ui_g_1_left_small_leg->width = 5;
    ui_g_1_left_small_leg->end_x = 540;
    ui_g_1_left_small_leg->end_y = 440;

    ui_g_1_right_big_leg->figure_type = 0;
    ui_g_1_right_big_leg->operate_type = 1;
    ui_g_1_right_big_leg->layer = 0;
    ui_g_1_right_big_leg->color = 2;
    ui_g_1_right_big_leg->start_x = 1283;
    ui_g_1_right_big_leg->start_y = 440;
    ui_g_1_right_big_leg->width = 5;
    ui_g_1_right_big_leg->end_x = 1392;
    ui_g_1_right_big_leg->end_y = 440;

    ui_g_1_current_high_flag->figure_type = 2;
    ui_g_1_current_high_flag->operate_type = 1;
    ui_g_1_current_high_flag->layer = 0;
    ui_g_1_current_high_flag->color = 5;
    ui_g_1_current_high_flag->start_x = 96;
    ui_g_1_current_high_flag->start_y = 785;
    ui_g_1_current_high_flag->width = 10;
    ui_g_1_current_high_flag->r = 5;

    ui_g_1_right_small_leg->figure_type = 0;
    ui_g_1_right_small_leg->operate_type = 1;
    ui_g_1_right_small_leg->layer = 0;
    ui_g_1_right_small_leg->color = 2;
    ui_g_1_right_small_leg->start_x = 1450;
    ui_g_1_right_small_leg->start_y = 356;
    ui_g_1_right_small_leg->width = 5;
    ui_g_1_right_small_leg->end_x = 1387;
    ui_g_1_right_small_leg->end_y = 443;

    ui_g_1_supercap_capcity->figure_type = 1;
    ui_g_1_supercap_capcity->operate_type = 1;
    ui_g_1_supercap_capcity->layer = 0;
    ui_g_1_supercap_capcity->color = 2;
    ui_g_1_supercap_capcity->start_x = 666;
    ui_g_1_supercap_capcity->start_y = 282;
    ui_g_1_supercap_capcity->width = 11;
    ui_g_1_supercap_capcity->end_x = 1240;
    ui_g_1_supercap_capcity->end_y = 283;

    ui_g_1_supcap_voltage->figure_type = 5;
    ui_g_1_supcap_voltage->operate_type = 1;
    ui_g_1_supcap_voltage->layer = 0;
    ui_g_1_supcap_voltage->color = 6;
    ui_g_1_supcap_voltage->start_x = 911;
    ui_g_1_supcap_voltage->start_y = 249;
    ui_g_1_supcap_voltage->width = 3;
    ui_g_1_supcap_voltage->font_size = 30;
    ui_g_1_supcap_voltage->number = 28000;


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

ui_7_frame_t ui_g_2_0;

ui_interface_number_t *ui_g_2_current_trigger = (ui_interface_number_t*)&(ui_g_2_0.data[0]);
ui_interface_number_t *ui_g_2_target_trigger = (ui_interface_number_t*)&(ui_g_2_0.data[1]);
ui_interface_arc_t *ui_g_2_head_position = (ui_interface_arc_t*)&(ui_g_2_0.data[2]);
ui_interface_number_t *ui_g_2_vision_trice_id = (ui_interface_number_t*)&(ui_g_2_0.data[3]);
ui_interface_rect_t *ui_g_2_vision_frame = (ui_interface_rect_t*)&(ui_g_2_0.data[4]);
ui_interface_number_t *ui_g_2_vision_order_id = (ui_interface_number_t*)&(ui_g_2_0.data[5]);
ui_interface_round_t *ui_g_2_current_shoot_mode = (ui_interface_round_t*)&(ui_g_2_0.data[6]);

void _ui_init_g_2_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_2_0.data[i].figure_name[0] = 0;
        ui_g_2_0.data[i].figure_name[1] = 1;
        ui_g_2_0.data[i].figure_name[2] = i + 0;
        ui_g_2_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_2_0.data[i].operate_type = 0;
    }

    ui_g_2_current_trigger->figure_type = 5;
    ui_g_2_current_trigger->operate_type = 1;
    ui_g_2_current_trigger->layer = 0;
    ui_g_2_current_trigger->color = 1;
    ui_g_2_current_trigger->start_x = 344;
    ui_g_2_current_trigger->start_y = 777;
    ui_g_2_current_trigger->width = 3;
    ui_g_2_current_trigger->font_size = 25;
    ui_g_2_current_trigger->number = 65535000;

    ui_g_2_target_trigger->figure_type = 5;
    ui_g_2_target_trigger->operate_type = 1;
    ui_g_2_target_trigger->layer = 0;
    ui_g_2_target_trigger->color = 5;
    ui_g_2_target_trigger->start_x = 344;
    ui_g_2_target_trigger->start_y = 711;
    ui_g_2_target_trigger->width = 3;
    ui_g_2_target_trigger->font_size = 25;
    ui_g_2_target_trigger->number = 65535000;

    ui_g_2_head_position->figure_type = 4;
    ui_g_2_head_position->operate_type = 1;
    ui_g_2_head_position->layer = 0;
    ui_g_2_head_position->color = 2;
    ui_g_2_head_position->start_x = 960;
    ui_g_2_head_position->start_y = 540;
    ui_g_2_head_position->width = 5;
    ui_g_2_head_position->start_angle = 15;
    ui_g_2_head_position->end_angle = 345;
    ui_g_2_head_position->rx = 50;
    ui_g_2_head_position->ry = 50;

    ui_g_2_vision_trice_id->figure_type = 6;
    ui_g_2_vision_trice_id->operate_type = 1;
    ui_g_2_vision_trice_id->layer = 0;
    ui_g_2_vision_trice_id->color = 4;
    ui_g_2_vision_trice_id->start_x = 945;
    ui_g_2_vision_trice_id->start_y = 680;
    ui_g_2_vision_trice_id->width = 3;
    ui_g_2_vision_trice_id->font_size = 30;
    ui_g_2_vision_trice_id->number = 1;

    ui_g_2_vision_frame->figure_type = 1;
    ui_g_2_vision_frame->operate_type = 1;
    ui_g_2_vision_frame->layer = 0;
    ui_g_2_vision_frame->color = 8;
    ui_g_2_vision_frame->start_x = 690;
    ui_g_2_vision_frame->start_y = 380;
    ui_g_2_vision_frame->width = 3;
    ui_g_2_vision_frame->end_x = 1240;
    ui_g_2_vision_frame->end_y = 700;

    ui_g_2_vision_order_id->figure_type = 6;
    ui_g_2_vision_order_id->operate_type = 1;
    ui_g_2_vision_order_id->layer = 0;
    ui_g_2_vision_order_id->color = 3;
    ui_g_2_vision_order_id->start_x = 945;
    ui_g_2_vision_order_id->start_y = 430;
    ui_g_2_vision_order_id->width = 3;
    ui_g_2_vision_order_id->font_size = 30;
    ui_g_2_vision_order_id->number = 1;

    ui_g_2_current_shoot_mode->figure_type = 2;
    ui_g_2_current_shoot_mode->operate_type = 1;
    ui_g_2_current_shoot_mode->layer = 0;
    ui_g_2_current_shoot_mode->color = 5;
    ui_g_2_current_shoot_mode->start_x = 720;
    ui_g_2_current_shoot_mode->start_y = 673;
    ui_g_2_current_shoot_mode->width = 10;
    ui_g_2_current_shoot_mode->r = 5;


    ui_proc_7_frame(&ui_g_2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_2_0, sizeof(ui_g_2_0));
}

void _ui_update_g_2_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_2_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_2_0, sizeof(ui_g_2_0));
}

void _ui_remove_g_2_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_2_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_2_0);
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

ui_1_frame_t ui_g_3_0;

ui_interface_rect_t *ui_g_3_supercap = (ui_interface_rect_t*)&(ui_g_3_0.data[0]);

void _ui_init_g_3_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_3_0.data[i].figure_name[0] = 0;
        ui_g_3_0.data[i].figure_name[1] = 2;
        ui_g_3_0.data[i].figure_name[2] = i + 0;
        ui_g_3_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_g_3_0.data[i].operate_type = 0;
    }

    ui_g_3_supercap->figure_type = 1;
    ui_g_3_supercap->operate_type = 1;
    ui_g_3_supercap->layer = 0;
    ui_g_3_supercap->color = 0;
    ui_g_3_supercap->start_x = 658;
    ui_g_3_supercap->start_y = 273;
    ui_g_3_supercap->width = 5;
    ui_g_3_supercap->end_x = 1248;
    ui_g_3_supercap->end_y = 293;


    ui_proc_1_frame(&ui_g_3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_3_0, sizeof(ui_g_3_0));
}

void _ui_update_g_3_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_3_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_g_3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_3_0, sizeof(ui_g_3_0));
}

void _ui_remove_g_3_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_3_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_g_3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_3_0, sizeof(ui_g_3_0));
}

ui_string_frame_t ui_g_3_1;
ui_interface_string_t* ui_g_3_low_leg_length = &(ui_g_3_1.option);

void _ui_init_g_3_1() {
    ui_g_3_1.option.figure_name[0] = 0;
    ui_g_3_1.option.figure_name[1] = 2;
    ui_g_3_1.option.figure_name[2] = 1;
    ui_g_3_1.option.operate_type = 1;

    ui_g_3_low_leg_length->figure_type = 7;
    ui_g_3_low_leg_length->operate_type = 1;
    ui_g_3_low_leg_length->layer = 0;
    ui_g_3_low_leg_length->color = 1;
    ui_g_3_low_leg_length->start_x = 122;
    ui_g_3_low_leg_length->start_y = 806;
    ui_g_3_low_leg_length->width = 3;
    ui_g_3_low_leg_length->font_size = 30;
    ui_g_3_low_leg_length->str_length = 3;
    strcpy(ui_g_3_low_leg_length->string, "low");


    ui_proc_string_frame(&ui_g_3_1);
    SEND_MESSAGE((uint8_t *) &ui_g_3_1, sizeof(ui_g_3_1));
}

void _ui_update_g_3_1() {
    ui_g_3_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_3_1);
    SEND_MESSAGE((uint8_t *) &ui_g_3_1, sizeof(ui_g_3_1));
}

void _ui_remove_g_3_1() {
    ui_g_3_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_3_1);
    SEND_MESSAGE((uint8_t *) &ui_g_3_1, sizeof(ui_g_3_1));
}
ui_string_frame_t ui_g_3_2;
ui_interface_string_t* ui_g_3_shoot_mode_aim = &(ui_g_3_2.option);

void _ui_init_g_3_2() {
    ui_g_3_2.option.figure_name[0] = 0;
    ui_g_3_2.option.figure_name[1] = 2;
    ui_g_3_2.option.figure_name[2] = 2;
    ui_g_3_2.option.operate_type = 1;

    ui_g_3_shoot_mode_aim->figure_type = 7;
    ui_g_3_shoot_mode_aim->operate_type = 1;
    ui_g_3_shoot_mode_aim->layer = 0;
    ui_g_3_shoot_mode_aim->color = 8;
    ui_g_3_shoot_mode_aim->start_x = 740;
    ui_g_3_shoot_mode_aim->start_y = 685;
    ui_g_3_shoot_mode_aim->width = 2;
    ui_g_3_shoot_mode_aim->font_size = 15;
    ui_g_3_shoot_mode_aim->str_length = 3;
    strcpy(ui_g_3_shoot_mode_aim->string, "aim");


    ui_proc_string_frame(&ui_g_3_2);
    SEND_MESSAGE((uint8_t *) &ui_g_3_2, sizeof(ui_g_3_2));
}

void _ui_update_g_3_2() {
    ui_g_3_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_3_2);
    SEND_MESSAGE((uint8_t *) &ui_g_3_2, sizeof(ui_g_3_2));
}

void _ui_remove_g_3_2() {
    ui_g_3_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_3_2);
    SEND_MESSAGE((uint8_t *) &ui_g_3_2, sizeof(ui_g_3_2));
}
ui_string_frame_t ui_g_3_3;
ui_interface_string_t* ui_g_3_shoot_mode_energy = &(ui_g_3_3.option);

void _ui_init_g_3_3() {
    ui_g_3_3.option.figure_name[0] = 0;
    ui_g_3_3.option.figure_name[1] = 2;
    ui_g_3_3.option.figure_name[2] = 3;
    ui_g_3_3.option.operate_type = 1;

    ui_g_3_shoot_mode_energy->figure_type = 7;
    ui_g_3_shoot_mode_energy->operate_type = 1;
    ui_g_3_shoot_mode_energy->layer = 0;
    ui_g_3_shoot_mode_energy->color = 8;
    ui_g_3_shoot_mode_energy->start_x = 740;
    ui_g_3_shoot_mode_energy->start_y = 650;
    ui_g_3_shoot_mode_energy->width = 2;
    ui_g_3_shoot_mode_energy->font_size = 15;
    ui_g_3_shoot_mode_energy->str_length = 6;
    strcpy(ui_g_3_shoot_mode_energy->string, "energy");


    ui_proc_string_frame(&ui_g_3_3);
    SEND_MESSAGE((uint8_t *) &ui_g_3_3, sizeof(ui_g_3_3));
}

void _ui_update_g_3_3() {
    ui_g_3_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_3_3);
    SEND_MESSAGE((uint8_t *) &ui_g_3_3, sizeof(ui_g_3_3));
}

void _ui_remove_g_3_3() {
    ui_g_3_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_3_3);
    SEND_MESSAGE((uint8_t *) &ui_g_3_3, sizeof(ui_g_3_3));
}
ui_string_frame_t ui_g_3_4;
ui_interface_string_t* ui_g_3_mid_leg_length = &(ui_g_3_4.option);

void _ui_init_g_3_4() {
    ui_g_3_4.option.figure_name[0] = 0;
    ui_g_3_4.option.figure_name[1] = 2;
    ui_g_3_4.option.figure_name[2] = 4;
    ui_g_3_4.option.operate_type = 1;

    ui_g_3_mid_leg_length->figure_type = 7;
    ui_g_3_mid_leg_length->operate_type = 1;
    ui_g_3_mid_leg_length->layer = 0;
    ui_g_3_mid_leg_length->color = 1;
    ui_g_3_mid_leg_length->start_x = 122;
    ui_g_3_mid_leg_length->start_y = 756;
    ui_g_3_mid_leg_length->width = 3;
    ui_g_3_mid_leg_length->font_size = 30;
    ui_g_3_mid_leg_length->str_length = 3;
    strcpy(ui_g_3_mid_leg_length->string, "mid");


    ui_proc_string_frame(&ui_g_3_4);
    SEND_MESSAGE((uint8_t *) &ui_g_3_4, sizeof(ui_g_3_4));
}

void _ui_update_g_3_4() {
    ui_g_3_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_3_4);
    SEND_MESSAGE((uint8_t *) &ui_g_3_4, sizeof(ui_g_3_4));
}

void _ui_remove_g_3_4() {
    ui_g_3_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_3_4);
    SEND_MESSAGE((uint8_t *) &ui_g_3_4, sizeof(ui_g_3_4));
}
ui_string_frame_t ui_g_3_5;
ui_interface_string_t* ui_g_3_high_leg_length = &(ui_g_3_5.option);

void _ui_init_g_3_5() {
    ui_g_3_5.option.figure_name[0] = 0;
    ui_g_3_5.option.figure_name[1] = 2;
    ui_g_3_5.option.figure_name[2] = 5;
    ui_g_3_5.option.operate_type = 1;

    ui_g_3_high_leg_length->figure_type = 7;
    ui_g_3_high_leg_length->operate_type = 1;
    ui_g_3_high_leg_length->layer = 0;
    ui_g_3_high_leg_length->color = 1;
    ui_g_3_high_leg_length->start_x = 122;
    ui_g_3_high_leg_length->start_y = 706;
    ui_g_3_high_leg_length->width = 3;
    ui_g_3_high_leg_length->font_size = 30;
    ui_g_3_high_leg_length->str_length = 4;
    strcpy(ui_g_3_high_leg_length->string, "high");


    ui_proc_string_frame(&ui_g_3_5);
    SEND_MESSAGE((uint8_t *) &ui_g_3_5, sizeof(ui_g_3_5));
}

void _ui_update_g_3_5() {
    ui_g_3_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_3_5);
    SEND_MESSAGE((uint8_t *) &ui_g_3_5, sizeof(ui_g_3_5));
}

void _ui_remove_g_3_5() {
    ui_g_3_5.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_3_5);
    SEND_MESSAGE((uint8_t *) &ui_g_3_5, sizeof(ui_g_3_5));
}
ui_string_frame_t ui_g_3_6;
ui_interface_string_t* ui_g_3_trigger = &(ui_g_3_6.option);

void _ui_init_g_3_6() {
    ui_g_3_6.option.figure_name[0] = 0;
    ui_g_3_6.option.figure_name[1] = 2;
    ui_g_3_6.option.figure_name[2] = 6;
    ui_g_3_6.option.operate_type = 1;

    ui_g_3_trigger->figure_type = 7;
    ui_g_3_trigger->operate_type = 1;
    ui_g_3_trigger->layer = 0;
    ui_g_3_trigger->color = 2;
    ui_g_3_trigger->start_x = 331;
    ui_g_3_trigger->start_y = 825;
    ui_g_3_trigger->width = 2;
    ui_g_3_trigger->font_size = 20;
    ui_g_3_trigger->str_length = 7;
    strcpy(ui_g_3_trigger->string, "trigger");


    ui_proc_string_frame(&ui_g_3_6);
    SEND_MESSAGE((uint8_t *) &ui_g_3_6, sizeof(ui_g_3_6));
}

void _ui_update_g_3_6() {
    ui_g_3_6.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_3_6);
    SEND_MESSAGE((uint8_t *) &ui_g_3_6, sizeof(ui_g_3_6));
}

void _ui_remove_g_3_6() {
    ui_g_3_6.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_3_6);
    SEND_MESSAGE((uint8_t *) &ui_g_3_6, sizeof(ui_g_3_6));
}

void ui_init_g_3() {
    _ui_init_g_3_0();
    _ui_init_g_3_1();
    _ui_init_g_3_2();
    _ui_init_g_3_3();
    _ui_init_g_3_4();
    _ui_init_g_3_5();
    _ui_init_g_3_6();
}

void ui_update_g_3() {
    _ui_update_g_3_0();
    _ui_update_g_3_1();
    _ui_update_g_3_2();
    _ui_update_g_3_3();
    _ui_update_g_3_4();
    _ui_update_g_3_5();
    _ui_update_g_3_6();
}

void ui_remove_g_3() {
    _ui_remove_g_3_0();
    _ui_remove_g_3_1();
    _ui_remove_g_3_2();
    _ui_remove_g_3_3();
    _ui_remove_g_3_4();
    _ui_remove_g_3_5();
    _ui_remove_g_3_6();
}


ui_string_frame_t ui_g_4_0;
ui_interface_string_t* ui_g_4_spin_warning = &(ui_g_4_0.option);

void _ui_init_g_4_0() {
    ui_g_4_0.option.figure_name[0] = 0;
    ui_g_4_0.option.figure_name[1] = 3;
    ui_g_4_0.option.figure_name[2] = 0;
    ui_g_4_0.option.operate_type = 1;

    ui_g_4_spin_warning->figure_type = 7;
    ui_g_4_spin_warning->operate_type = 1;
    ui_g_4_spin_warning->layer = 0;
    ui_g_4_spin_warning->color = 6;
    ui_g_4_spin_warning->start_x = 790;
    ui_g_4_spin_warning->start_y = 763;
    ui_g_4_spin_warning->width = 3;
    ui_g_4_spin_warning->font_size = 30;
    ui_g_4_spin_warning->str_length = 12;
    strcpy(ui_g_4_spin_warning->string, "PLEASE SPIN!");


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
ui_string_frame_t ui_g_4_1;
ui_interface_string_t* ui_g_4_leg_right_big_warning = &(ui_g_4_1.option);

void _ui_init_g_4_1() {
    ui_g_4_1.option.figure_name[0] = 0;
    ui_g_4_1.option.figure_name[1] = 3;
    ui_g_4_1.option.figure_name[2] = 1;
    ui_g_4_1.option.operate_type = 1;

    ui_g_4_leg_right_big_warning->figure_type = 7;
    ui_g_4_leg_right_big_warning->operate_type = 1;
    ui_g_4_leg_right_big_warning->layer = 0;
    ui_g_4_leg_right_big_warning->color = 8;
    ui_g_4_leg_right_big_warning->start_x = 1574;
    ui_g_4_leg_right_big_warning->start_y = 599;
    ui_g_4_leg_right_big_warning->width = 2;
    ui_g_4_leg_right_big_warning->font_size = 20;
    ui_g_4_leg_right_big_warning->str_length = 10;
    strcpy(ui_g_4_leg_right_big_warning->string, "leg_rb_off");


    ui_proc_string_frame(&ui_g_4_1);
    SEND_MESSAGE((uint8_t *) &ui_g_4_1, sizeof(ui_g_4_1));
}

void _ui_update_g_4_1() {
    ui_g_4_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_4_1);
    SEND_MESSAGE((uint8_t *) &ui_g_4_1, sizeof(ui_g_4_1));
}

void _ui_remove_g_4_1() {
    ui_g_4_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_4_1);
    SEND_MESSAGE((uint8_t *) &ui_g_4_1, sizeof(ui_g_4_1));
}
ui_string_frame_t ui_g_4_2;
ui_interface_string_t* ui_g_4_leg_right_small_warning = &(ui_g_4_2.option);

void _ui_init_g_4_2() {
    ui_g_4_2.option.figure_name[0] = 0;
    ui_g_4_2.option.figure_name[1] = 3;
    ui_g_4_2.option.figure_name[2] = 2;
    ui_g_4_2.option.operate_type = 1;

    ui_g_4_leg_right_small_warning->figure_type = 7;
    ui_g_4_leg_right_small_warning->operate_type = 1;
    ui_g_4_leg_right_small_warning->layer = 0;
    ui_g_4_leg_right_small_warning->color = 8;
    ui_g_4_leg_right_small_warning->start_x = 1574;
    ui_g_4_leg_right_small_warning->start_y = 565;
    ui_g_4_leg_right_small_warning->width = 2;
    ui_g_4_leg_right_small_warning->font_size = 20;
    ui_g_4_leg_right_small_warning->str_length = 10;
    strcpy(ui_g_4_leg_right_small_warning->string, "leg_rs_off");


    ui_proc_string_frame(&ui_g_4_2);
    SEND_MESSAGE((uint8_t *) &ui_g_4_2, sizeof(ui_g_4_2));
}

void _ui_update_g_4_2() {
    ui_g_4_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_4_2);
    SEND_MESSAGE((uint8_t *) &ui_g_4_2, sizeof(ui_g_4_2));
}

void _ui_remove_g_4_2() {
    ui_g_4_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_4_2);
    SEND_MESSAGE((uint8_t *) &ui_g_4_2, sizeof(ui_g_4_2));
}
ui_string_frame_t ui_g_4_3;
ui_interface_string_t* ui_g_4_yaw_warning = &(ui_g_4_3.option);

void _ui_init_g_4_3() {
    ui_g_4_3.option.figure_name[0] = 0;
    ui_g_4_3.option.figure_name[1] = 3;
    ui_g_4_3.option.figure_name[2] = 3;
    ui_g_4_3.option.operate_type = 1;

    ui_g_4_yaw_warning->figure_type = 7;
    ui_g_4_yaw_warning->operate_type = 1;
    ui_g_4_yaw_warning->layer = 0;
    ui_g_4_yaw_warning->color = 8;
    ui_g_4_yaw_warning->start_x = 1567;
    ui_g_4_yaw_warning->start_y = 530;
    ui_g_4_yaw_warning->width = 2;
    ui_g_4_yaw_warning->font_size = 20;
    ui_g_4_yaw_warning->str_length = 7;
    strcpy(ui_g_4_yaw_warning->string, "yaw_off");


    ui_proc_string_frame(&ui_g_4_3);
    SEND_MESSAGE((uint8_t *) &ui_g_4_3, sizeof(ui_g_4_3));
}

void _ui_update_g_4_3() {
    ui_g_4_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_4_3);
    SEND_MESSAGE((uint8_t *) &ui_g_4_3, sizeof(ui_g_4_3));
}

void _ui_remove_g_4_3() {
    ui_g_4_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_4_3);
    SEND_MESSAGE((uint8_t *) &ui_g_4_3, sizeof(ui_g_4_3));
}
ui_string_frame_t ui_g_4_4;
ui_interface_string_t* ui_g_4_pit_warning = &(ui_g_4_4.option);

void _ui_init_g_4_4() {
    ui_g_4_4.option.figure_name[0] = 0;
    ui_g_4_4.option.figure_name[1] = 3;
    ui_g_4_4.option.figure_name[2] = 4;
    ui_g_4_4.option.operate_type = 1;

    ui_g_4_pit_warning->figure_type = 7;
    ui_g_4_pit_warning->operate_type = 1;
    ui_g_4_pit_warning->layer = 0;
    ui_g_4_pit_warning->color = 8;
    ui_g_4_pit_warning->start_x = 1569;
    ui_g_4_pit_warning->start_y = 496;
    ui_g_4_pit_warning->width = 2;
    ui_g_4_pit_warning->font_size = 20;
    ui_g_4_pit_warning->str_length = 7;
    strcpy(ui_g_4_pit_warning->string, "pit_off");


    ui_proc_string_frame(&ui_g_4_4);
    SEND_MESSAGE((uint8_t *) &ui_g_4_4, sizeof(ui_g_4_4));
}

void _ui_update_g_4_4() {
    ui_g_4_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_4_4);
    SEND_MESSAGE((uint8_t *) &ui_g_4_4, sizeof(ui_g_4_4));
}

void _ui_remove_g_4_4() {
    ui_g_4_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_4_4);
    SEND_MESSAGE((uint8_t *) &ui_g_4_4, sizeof(ui_g_4_4));
}
ui_string_frame_t ui_g_4_5;
ui_interface_string_t* ui_g_4_gimbal_imu_warning = &(ui_g_4_5.option);

void _ui_init_g_4_5() {
    ui_g_4_5.option.figure_name[0] = 0;
    ui_g_4_5.option.figure_name[1] = 3;
    ui_g_4_5.option.figure_name[2] = 5;
    ui_g_4_5.option.operate_type = 1;

    ui_g_4_gimbal_imu_warning->figure_type = 7;
    ui_g_4_gimbal_imu_warning->operate_type = 1;
    ui_g_4_gimbal_imu_warning->layer = 0;
    ui_g_4_gimbal_imu_warning->color = 8;
    ui_g_4_gimbal_imu_warning->start_x = 1569;
    ui_g_4_gimbal_imu_warning->start_y = 461;
    ui_g_4_gimbal_imu_warning->width = 2;
    ui_g_4_gimbal_imu_warning->font_size = 20;
    ui_g_4_gimbal_imu_warning->str_length = 9;
    strcpy(ui_g_4_gimbal_imu_warning->string, "g_imu_off");


    ui_proc_string_frame(&ui_g_4_5);
    SEND_MESSAGE((uint8_t *) &ui_g_4_5, sizeof(ui_g_4_5));
}

void _ui_update_g_4_5() {
    ui_g_4_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_4_5);
    SEND_MESSAGE((uint8_t *) &ui_g_4_5, sizeof(ui_g_4_5));
}

void _ui_remove_g_4_5() {
    ui_g_4_5.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_4_5);
    SEND_MESSAGE((uint8_t *) &ui_g_4_5, sizeof(ui_g_4_5));
}

void ui_init_g_4() {
    _ui_init_g_4_0();
    _ui_init_g_4_1();
    _ui_init_g_4_2();
    _ui_init_g_4_3();
    _ui_init_g_4_4();
    _ui_init_g_4_5();
}

void ui_update_g_4() {
    _ui_update_g_4_0();
    _ui_update_g_4_1();
    _ui_update_g_4_2();
    _ui_update_g_4_3();
    _ui_update_g_4_4();
    _ui_update_g_4_5();
}

void ui_remove_g_4() {
    _ui_remove_g_4_0();
    _ui_remove_g_4_1();
    _ui_remove_g_4_2();
    _ui_remove_g_4_3();
    _ui_remove_g_4_4();
    _ui_remove_g_4_5();
}

ui_5_frame_t ui_g_5_0;

ui_interface_number_t *ui_g_5_left_lazer = (ui_interface_number_t*)&(ui_g_5_0.data[0]);
ui_interface_number_t *ui_g_5_right_lazer = (ui_interface_number_t*)&(ui_g_5_0.data[1]);
ui_interface_number_t *ui_g_5_left_leg_length = (ui_interface_number_t*)&(ui_g_5_0.data[2]);
ui_interface_number_t *ui_g_5_right_leg_length = (ui_interface_number_t*)&(ui_g_5_0.data[3]);

void _ui_init_g_5_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_5_0.data[i].figure_name[0] = 0;
        ui_g_5_0.data[i].figure_name[1] = 4;
        ui_g_5_0.data[i].figure_name[2] = i + 0;
        ui_g_5_0.data[i].operate_type = 1;
    }
    for (int i = 4; i < 5; i++) {
        ui_g_5_0.data[i].operate_type = 0;
    }

    ui_g_5_left_lazer->figure_type = 5;
    ui_g_5_left_lazer->operate_type = 1;
    ui_g_5_left_lazer->layer = 0;
    ui_g_5_left_lazer->color = 1;
    ui_g_5_left_lazer->start_x = 1368;
    ui_g_5_left_lazer->start_y = 780;
    ui_g_5_left_lazer->width = 3;
    ui_g_5_left_lazer->font_size = 25;
    ui_g_5_left_lazer->number = 65535000;

    ui_g_5_right_lazer->figure_type = 5;
    ui_g_5_right_lazer->operate_type = 1;
    ui_g_5_right_lazer->layer = 0;
    ui_g_5_right_lazer->color = 4;
    ui_g_5_right_lazer->start_x = 1370;
    ui_g_5_right_lazer->start_y = 730;
    ui_g_5_right_lazer->width = 3;
    ui_g_5_right_lazer->font_size = 25;
    ui_g_5_right_lazer->number = 65535000;

    ui_g_5_left_leg_length->figure_type = 5;
    ui_g_5_left_leg_length->operate_type = 1;
    ui_g_5_left_leg_length->layer = 0;
    ui_g_5_left_leg_length->color = 5;
    ui_g_5_left_leg_length->start_x = 131;
    ui_g_5_left_leg_length->start_y = 556;
    ui_g_5_left_leg_length->width = 2;
    ui_g_5_left_leg_length->font_size = 20;
    ui_g_5_left_leg_length->number = 160;

    ui_g_5_right_leg_length->figure_type = 5;
    ui_g_5_right_leg_length->operate_type = 1;
    ui_g_5_right_leg_length->layer = 0;
    ui_g_5_right_leg_length->color = 1;
    ui_g_5_right_leg_length->start_x = 130;
    ui_g_5_right_leg_length->start_y = 596;
    ui_g_5_right_leg_length->width = 2;
    ui_g_5_right_leg_length->font_size = 20;
    ui_g_5_right_leg_length->number = 160;


    ui_proc_5_frame(&ui_g_5_0);
    SEND_MESSAGE((uint8_t *) &ui_g_5_0, sizeof(ui_g_5_0));
}

void _ui_update_g_5_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_5_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_5_0);
    SEND_MESSAGE((uint8_t *) &ui_g_5_0, sizeof(ui_g_5_0));
}

void _ui_remove_g_5_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_5_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_5_0);
    SEND_MESSAGE((uint8_t *) &ui_g_5_0, sizeof(ui_g_5_0));
}

ui_string_frame_t ui_g_5_1;
ui_interface_string_t* ui_g_5_distance = &(ui_g_5_1.option);

void _ui_init_g_5_1() {
    ui_g_5_1.option.figure_name[0] = 0;
    ui_g_5_1.option.figure_name[1] = 4;
    ui_g_5_1.option.figure_name[2] = 4;
    ui_g_5_1.option.operate_type = 1;

    ui_g_5_distance->figure_type = 7;
    ui_g_5_distance->operate_type = 1;
    ui_g_5_distance->layer = 0;
    ui_g_5_distance->color = 2;
    ui_g_5_distance->start_x = 1351;
    ui_g_5_distance->start_y = 825;
    ui_g_5_distance->width = 2;
    ui_g_5_distance->font_size = 20;
    ui_g_5_distance->str_length = 8;
    strcpy(ui_g_5_distance->string, "distance");


    ui_proc_string_frame(&ui_g_5_1);
    SEND_MESSAGE((uint8_t *) &ui_g_5_1, sizeof(ui_g_5_1));
}

void _ui_update_g_5_1() {
    ui_g_5_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_5_1);
    SEND_MESSAGE((uint8_t *) &ui_g_5_1, sizeof(ui_g_5_1));
}

void _ui_remove_g_5_1() {
    ui_g_5_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_5_1);
    SEND_MESSAGE((uint8_t *) &ui_g_5_1, sizeof(ui_g_5_1));
}
ui_string_frame_t ui_g_5_2;
ui_interface_string_t* ui_g_5_leg_length = &(ui_g_5_2.option);

void _ui_init_g_5_2() {
    ui_g_5_2.option.figure_name[0] = 0;
    ui_g_5_2.option.figure_name[1] = 4;
    ui_g_5_2.option.figure_name[2] = 5;
    ui_g_5_2.option.operate_type = 1;

    ui_g_5_leg_length->figure_type = 7;
    ui_g_5_leg_length->operate_type = 1;
    ui_g_5_leg_length->layer = 0;
    ui_g_5_leg_length->color = 2;
    ui_g_5_leg_length->start_x = 116;
    ui_g_5_leg_length->start_y = 638;
    ui_g_5_leg_length->width = 2;
    ui_g_5_leg_length->font_size = 20;
    ui_g_5_leg_length->str_length = 6;
    strcpy(ui_g_5_leg_length->string, "length");


    ui_proc_string_frame(&ui_g_5_2);
    SEND_MESSAGE((uint8_t *) &ui_g_5_2, sizeof(ui_g_5_2));
}

void _ui_update_g_5_2() {
    ui_g_5_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_5_2);
    SEND_MESSAGE((uint8_t *) &ui_g_5_2, sizeof(ui_g_5_2));
}

void _ui_remove_g_5_2() {
    ui_g_5_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_5_2);
    SEND_MESSAGE((uint8_t *) &ui_g_5_2, sizeof(ui_g_5_2));
}

void ui_init_g_5() {
    _ui_init_g_5_0();
    _ui_init_g_5_1();
    _ui_init_g_5_2();
}

void ui_update_g_5() {
    _ui_update_g_5_0();
    // _ui_update_g_5_1();
    // _ui_update_g_5_2();
}

void ui_remove_g_5() {
    _ui_remove_g_5_0();
    _ui_remove_g_5_1();
    _ui_remove_g_5_2();
}


ui_string_frame_t ui_g_6_0;
ui_interface_string_t* ui_g_6_fric_left_warning = &(ui_g_6_0.option);

void _ui_init_g_6_0() {
    ui_g_6_0.option.figure_name[0] = 0;
    ui_g_6_0.option.figure_name[1] = 5;
    ui_g_6_0.option.figure_name[2] = 0;
    ui_g_6_0.option.operate_type = 1;

    ui_g_6_fric_left_warning->figure_type = 7;
    ui_g_6_fric_left_warning->operate_type = 1;
    ui_g_6_fric_left_warning->layer = 0;
    ui_g_6_fric_left_warning->color = 8;
    ui_g_6_fric_left_warning->start_x = 1583;
    ui_g_6_fric_left_warning->start_y = 846;
    ui_g_6_fric_left_warning->width = 2;
    ui_g_6_fric_left_warning->font_size = 20;
    ui_g_6_fric_left_warning->str_length = 10;
    strcpy(ui_g_6_fric_left_warning->string, "fric_l_off");


    ui_proc_string_frame(&ui_g_6_0);
    SEND_MESSAGE((uint8_t *) &ui_g_6_0, sizeof(ui_g_6_0));
}

void _ui_update_g_6_0() {
    ui_g_6_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_0);
    SEND_MESSAGE((uint8_t *) &ui_g_6_0, sizeof(ui_g_6_0));
}

void _ui_remove_g_6_0() {
    ui_g_6_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_6_0);
    SEND_MESSAGE((uint8_t *) &ui_g_6_0, sizeof(ui_g_6_0));
}
ui_string_frame_t ui_g_6_1;
ui_interface_string_t* ui_g_6_fric_right_warning = &(ui_g_6_1.option);

void _ui_init_g_6_1() {
    ui_g_6_1.option.figure_name[0] = 0;
    ui_g_6_1.option.figure_name[1] = 5;
    ui_g_6_1.option.figure_name[2] = 1;
    ui_g_6_1.option.operate_type = 1;

    ui_g_6_fric_right_warning->figure_type = 7;
    ui_g_6_fric_right_warning->operate_type = 1;
    ui_g_6_fric_right_warning->layer = 0;
    ui_g_6_fric_right_warning->color = 8;
    ui_g_6_fric_right_warning->start_x = 1583;
    ui_g_6_fric_right_warning->start_y = 808;
    ui_g_6_fric_right_warning->width = 2;
    ui_g_6_fric_right_warning->font_size = 20;
    ui_g_6_fric_right_warning->str_length = 10;
    strcpy(ui_g_6_fric_right_warning->string, "fric_r_off");


    ui_proc_string_frame(&ui_g_6_1);
    SEND_MESSAGE((uint8_t *) &ui_g_6_1, sizeof(ui_g_6_1));
}

void _ui_update_g_6_1() {
    ui_g_6_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_1);
    SEND_MESSAGE((uint8_t *) &ui_g_6_1, sizeof(ui_g_6_1));
}

void _ui_remove_g_6_1() {
    ui_g_6_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_6_1);
    SEND_MESSAGE((uint8_t *) &ui_g_6_1, sizeof(ui_g_6_1));
}
ui_string_frame_t ui_g_6_2;
ui_interface_string_t* ui_g_6_trigger_warning = &(ui_g_6_2.option);

void _ui_init_g_6_2() {
    ui_g_6_2.option.figure_name[0] = 0;
    ui_g_6_2.option.figure_name[1] = 5;
    ui_g_6_2.option.figure_name[2] = 2;
    ui_g_6_2.option.operate_type = 1;

    ui_g_6_trigger_warning->figure_type = 7;
    ui_g_6_trigger_warning->operate_type = 1;
    ui_g_6_trigger_warning->layer = 0;
    ui_g_6_trigger_warning->color = 8;
    ui_g_6_trigger_warning->start_x = 1577;
    ui_g_6_trigger_warning->start_y = 769;
    ui_g_6_trigger_warning->width = 2;
    ui_g_6_trigger_warning->font_size = 20;
    ui_g_6_trigger_warning->str_length = 11;
    strcpy(ui_g_6_trigger_warning->string, "trigger_off");


    ui_proc_string_frame(&ui_g_6_2);
    SEND_MESSAGE((uint8_t *) &ui_g_6_2, sizeof(ui_g_6_2));
}

void _ui_update_g_6_2() {
    ui_g_6_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_2);
    SEND_MESSAGE((uint8_t *) &ui_g_6_2, sizeof(ui_g_6_2));
}

void _ui_remove_g_6_2() {
    ui_g_6_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_6_2);
    SEND_MESSAGE((uint8_t *) &ui_g_6_2, sizeof(ui_g_6_2));
}
ui_string_frame_t ui_g_6_3;
ui_interface_string_t* ui_g_6_wheel_left_warning = &(ui_g_6_3.option);

void _ui_init_g_6_3() {
    ui_g_6_3.option.figure_name[0] = 0;
    ui_g_6_3.option.figure_name[1] = 5;
    ui_g_6_3.option.figure_name[2] = 3;
    ui_g_6_3.option.operate_type = 1;

    ui_g_6_wheel_left_warning->figure_type = 7;
    ui_g_6_wheel_left_warning->operate_type = 1;
    ui_g_6_wheel_left_warning->layer = 0;
    ui_g_6_wheel_left_warning->color = 8;
    ui_g_6_wheel_left_warning->start_x = 1578;
    ui_g_6_wheel_left_warning->start_y = 736;
    ui_g_6_wheel_left_warning->width = 2;
    ui_g_6_wheel_left_warning->font_size = 20;
    ui_g_6_wheel_left_warning->str_length = 11;
    strcpy(ui_g_6_wheel_left_warning->string, "wheel_l_off");


    ui_proc_string_frame(&ui_g_6_3);
    SEND_MESSAGE((uint8_t *) &ui_g_6_3, sizeof(ui_g_6_3));
}

void _ui_update_g_6_3() {
    ui_g_6_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_3);
    SEND_MESSAGE((uint8_t *) &ui_g_6_3, sizeof(ui_g_6_3));
}

void _ui_remove_g_6_3() {
    ui_g_6_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_6_3);
    SEND_MESSAGE((uint8_t *) &ui_g_6_3, sizeof(ui_g_6_3));
}
ui_string_frame_t ui_g_6_4;
ui_interface_string_t* ui_g_6_wheel_right_warning = &(ui_g_6_4.option);

void _ui_init_g_6_4() {
    ui_g_6_4.option.figure_name[0] = 0;
    ui_g_6_4.option.figure_name[1] = 5;
    ui_g_6_4.option.figure_name[2] = 4;
    ui_g_6_4.option.operate_type = 1;

    ui_g_6_wheel_right_warning->figure_type = 7;
    ui_g_6_wheel_right_warning->operate_type = 1;
    ui_g_6_wheel_right_warning->layer = 0;
    ui_g_6_wheel_right_warning->color = 8;
    ui_g_6_wheel_right_warning->start_x = 1575;
    ui_g_6_wheel_right_warning->start_y = 702;
    ui_g_6_wheel_right_warning->width = 2;
    ui_g_6_wheel_right_warning->font_size = 20;
    ui_g_6_wheel_right_warning->str_length = 11;
    strcpy(ui_g_6_wheel_right_warning->string, "wheel_r_off");


    ui_proc_string_frame(&ui_g_6_4);
    SEND_MESSAGE((uint8_t *) &ui_g_6_4, sizeof(ui_g_6_4));
}

void _ui_update_g_6_4() {
    ui_g_6_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_4);
    SEND_MESSAGE((uint8_t *) &ui_g_6_4, sizeof(ui_g_6_4));
}

void _ui_remove_g_6_4() {
    ui_g_6_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_6_4);
    SEND_MESSAGE((uint8_t *) &ui_g_6_4, sizeof(ui_g_6_4));
}
ui_string_frame_t ui_g_6_5;
ui_interface_string_t* ui_g_6_leg_left_big_warning = &(ui_g_6_5.option);

void _ui_init_g_6_5() {
    ui_g_6_5.option.figure_name[0] = 0;
    ui_g_6_5.option.figure_name[1] = 5;
    ui_g_6_5.option.figure_name[2] = 5;
    ui_g_6_5.option.operate_type = 1;

    ui_g_6_leg_left_big_warning->figure_type = 7;
    ui_g_6_leg_left_big_warning->operate_type = 1;
    ui_g_6_leg_left_big_warning->layer = 0;
    ui_g_6_leg_left_big_warning->color = 8;
    ui_g_6_leg_left_big_warning->start_x = 1575;
    ui_g_6_leg_left_big_warning->start_y = 665;
    ui_g_6_leg_left_big_warning->width = 2;
    ui_g_6_leg_left_big_warning->font_size = 20;
    ui_g_6_leg_left_big_warning->str_length = 10;
    strcpy(ui_g_6_leg_left_big_warning->string, "leg_lb_off");


    ui_proc_string_frame(&ui_g_6_5);
    SEND_MESSAGE((uint8_t *) &ui_g_6_5, sizeof(ui_g_6_5));
}

void _ui_update_g_6_5() {
    ui_g_6_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_5);
    SEND_MESSAGE((uint8_t *) &ui_g_6_5, sizeof(ui_g_6_5));
}

void _ui_remove_g_6_5() {
    ui_g_6_5.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_6_5);
    SEND_MESSAGE((uint8_t *) &ui_g_6_5, sizeof(ui_g_6_5));
}
ui_string_frame_t ui_g_6_6;
ui_interface_string_t* ui_g_6_leg_left_small_warning = &(ui_g_6_6.option);

void _ui_init_g_6_6() {
    ui_g_6_6.option.figure_name[0] = 0;
    ui_g_6_6.option.figure_name[1] = 5;
    ui_g_6_6.option.figure_name[2] = 6;
    ui_g_6_6.option.operate_type = 1;

    ui_g_6_leg_left_small_warning->figure_type = 7;
    ui_g_6_leg_left_small_warning->operate_type = 1;
    ui_g_6_leg_left_small_warning->layer = 0;
    ui_g_6_leg_left_small_warning->color = 8;
    ui_g_6_leg_left_small_warning->start_x = 1575;
    ui_g_6_leg_left_small_warning->start_y = 633;
    ui_g_6_leg_left_small_warning->width = 2;
    ui_g_6_leg_left_small_warning->font_size = 20;
    ui_g_6_leg_left_small_warning->str_length = 10;
    strcpy(ui_g_6_leg_left_small_warning->string, "leg_ls_off");


    ui_proc_string_frame(&ui_g_6_6);
    SEND_MESSAGE((uint8_t *) &ui_g_6_6, sizeof(ui_g_6_6));
}

void _ui_update_g_6_6() {
    ui_g_6_6.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_6_6);
    SEND_MESSAGE((uint8_t *) &ui_g_6_6, sizeof(ui_g_6_6));
}

void _ui_remove_g_6_6() {
    ui_g_6_6.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_6_6);
    SEND_MESSAGE((uint8_t *) &ui_g_6_6, sizeof(ui_g_6_6));
}

void ui_init_g_6() {
    _ui_init_g_6_0();
    _ui_init_g_6_1();
    _ui_init_g_6_2();
    _ui_init_g_6_3();
    _ui_init_g_6_4();
    _ui_init_g_6_5();
    _ui_init_g_6_6();
}

void ui_update_g_6() {
    _ui_update_g_6_0();
    _ui_update_g_6_1();
    _ui_update_g_6_2();
    _ui_update_g_6_3();
    _ui_update_g_6_4();
    _ui_update_g_6_5();
    _ui_update_g_6_6();
}

void ui_remove_g_6() {
    _ui_remove_g_6_0();
    _ui_remove_g_6_1();
    _ui_remove_g_6_2();
    _ui_remove_g_6_3();
    _ui_remove_g_6_4();
    _ui_remove_g_6_5();
    _ui_remove_g_6_6();
}

