//
// Created by RM UI Designer
//

#include "ui_g_Ungroup_0.h"
#include "prot_vision.h"


#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 0
#define OBJ_NUM 2
#define FRAME_OBJ_NUM 2

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_g_Ungroup_0;
ui_interface_number_t *ui_g_Ungroup_Number_choice = (ui_interface_number_t *)&(ui_g_Ungroup_0.data[0]);
ui_interface_number_t *ui_g_Ungroup_Number_rec = (ui_interface_number_t *)&(ui_g_Ungroup_0.data[1]);

void _ui_init_g_Ungroup_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Ungroup_0.data[i].figure_name[0] = FRAME_ID;
        ui_g_Ungroup_0.data[i].figure_name[1] = GROUP_ID;
        ui_g_Ungroup_0.data[i].figure_name[2] = i + START_ID;
        ui_g_Ungroup_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_g_Ungroup_0.data[i].operate_tpyel = 0;
    }

    ui_g_Ungroup_Number_choice->figure_tpye = 6;
    ui_g_Ungroup_Number_choice->layer = 0;
    ui_g_Ungroup_Number_choice->font_size = 50;
    ui_g_Ungroup_Number_choice->start_x = 512;
    ui_g_Ungroup_Number_choice->start_y = 808;
    ui_g_Ungroup_Number_choice->color = 6;
    ui_g_Ungroup_Number_choice->number = 12345;
    ui_g_Ungroup_Number_choice->width = 5;

    ui_g_Ungroup_Number_rec->figure_tpye = 6;
    ui_g_Ungroup_Number_rec->layer = 0;
    ui_g_Ungroup_Number_rec->font_size = 50;
    ui_g_Ungroup_Number_rec->start_x = 1352;
    ui_g_Ungroup_Number_rec->start_y = 804;
    ui_g_Ungroup_Number_rec->color = 2;
    ui_g_Ungroup_Number_rec->number = 12345;
    ui_g_Ungroup_Number_rec->width = 5;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_update_g_Ungroup_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Ungroup_0.data[i].operate_tpyel = 2;
    }
	ui_g_Ungroup_Number_choice->number = ID_judge;
	ui_g_Ungroup_Number_rec->number = vision.rx_ui_msg.data.vision_trace_id;
	if(vision.rx_ui_msg.data.vision_online == 0)
		ui_g_Ungroup_Number_rec->color = 7;
	else
			if(vision.rx_ui_msg.data.vision_online == 1)
		ui_g_Ungroup_Number_rec->color = 2;
	
		
    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
			

			
}

void _ui_remove_g_Ungroup_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Ungroup_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}
