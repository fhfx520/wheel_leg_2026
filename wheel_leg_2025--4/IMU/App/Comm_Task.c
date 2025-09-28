#include "Comm_Task.h"
#include "Status_Task.h"

void can_msg_send_task(void)
{
    if(Data_Ready_Flag==1) {
//        send_pit_message();//底盘IMU
//        send_yaw_message();
//        send_rol_message();//云台IMU注释这两行即可
//        send_acc_message();
    }
}
