//
// Created by RM UI Designer
//

#include "ui_g_Ungroup_leg_0.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 0
#define OBJ_NUM 2
#define FRAME_OBJ_NUM 2

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_g_Ungroup_leg_0;
ui_interface_line_t *ui_g_Ungroup_leg_bit_leg = (ui_interface_line_t *)&(ui_g_Ungroup_leg_0.data[0]);
ui_interface_line_t *ui_g_Ungroup_leg_small_leg = (ui_interface_line_t *)&(ui_g_Ungroup_leg_0.data[1]);

void _ui_init_g_Ungroup_leg_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Ungroup_leg_0.data[i].figure_name[0] = FRAME_ID;
        ui_g_Ungroup_leg_0.data[i].figure_name[1] = GROUP_ID;
        ui_g_Ungroup_leg_0.data[i].figure_name[2] = i + START_ID;
        ui_g_Ungroup_leg_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_g_Ungroup_leg_0.data[i].operate_tpyel = 0;
    }

    ui_g_Ungroup_leg_bit_leg->figure_tpye = 0;
    ui_g_Ungroup_leg_bit_leg->layer = 0;
    ui_g_Ungroup_leg_bit_leg->start_x = 1535;
    ui_g_Ungroup_leg_bit_leg->start_y = 980;
    ui_g_Ungroup_leg_bit_leg->end_x = 1685;
    ui_g_Ungroup_leg_bit_leg->end_y = 780;
    ui_g_Ungroup_leg_bit_leg->color = 0;
    ui_g_Ungroup_leg_bit_leg->width = 3;

    ui_g_Ungroup_leg_small_leg->figure_tpye = 0;
    ui_g_Ungroup_leg_small_leg->layer = 0;
    ui_g_Ungroup_leg_small_leg->start_x = 1685;
    ui_g_Ungroup_leg_small_leg->start_y = 780;
    ui_g_Ungroup_leg_small_leg->end_x = 1535;
    ui_g_Ungroup_leg_small_leg->end_y = 660;
    ui_g_Ungroup_leg_small_leg->color = 0;
    ui_g_Ungroup_leg_small_leg->width = 3;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Ungroup_leg_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_leg_0, sizeof(ui_g_Ungroup_leg_0));
}

void _ui_update_g_Ungroup_leg_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Ungroup_leg_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Ungroup_leg_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_leg_0, sizeof(ui_g_Ungroup_leg_0));
}

void _ui_remove_g_Ungroup_leg_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Ungroup_leg_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Ungroup_leg_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_leg_0, sizeof(ui_g_Ungroup_leg_0));
}
