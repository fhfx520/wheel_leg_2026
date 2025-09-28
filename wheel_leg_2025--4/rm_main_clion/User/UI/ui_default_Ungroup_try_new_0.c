//
// Created by RM UI Designer
//

#include "ui_default_Ungroup_try_new_0.h"
#include "prot_vision.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 0
#define OBJ_NUM 1
#define FRAME_OBJ_NUM 1


CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Ungroup_try_new_0;
ui_interface_number_t *ui_default_Ungroup_try_new_new_try = (ui_interface_number_t *)&(ui_default_Ungroup_try_new_0.data[0]);

void _ui_init_default_Ungroup_try_new_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Ungroup_try_new_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_Ungroup_try_new_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_Ungroup_try_new_0.data[i].figure_name[2] = i + START_ID;
        ui_default_Ungroup_try_new_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Ungroup_try_new_0.data[i].operate_tpyel = 0;
    }

    ui_default_Ungroup_try_new_new_try->figure_tpye = 6;
    ui_default_Ungroup_try_new_new_try->layer = 0;
    ui_default_Ungroup_try_new_new_try->font_size = 50;
    ui_default_Ungroup_try_new_new_try->start_x = 400;
    ui_default_Ungroup_try_new_new_try->start_y = 800;
    ui_default_Ungroup_try_new_new_try->color = 0;
    ui_default_Ungroup_try_new_new_try->number = 12345;
    ui_default_Ungroup_try_new_new_try->width = 5;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Ungroup_try_new_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Ungroup_try_new_0, sizeof(ui_default_Ungroup_try_new_0));
}

void _ui_update_default_Ungroup_try_new_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Ungroup_try_new_0.data[i].operate_tpyel = 2;
    }
		
		ui_default_Ungroup_try_new_new_try->number = ID_judge;
		
    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Ungroup_try_new_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Ungroup_try_new_0, sizeof(ui_default_Ungroup_try_new_0));
}

void _ui_remove_default_Ungroup_try_new_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Ungroup_try_new_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Ungroup_try_new_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Ungroup_try_new_0, sizeof(ui_default_Ungroup_try_new_0));
}
