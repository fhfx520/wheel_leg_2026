#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__

#include "fdcan.h"

/* CAN receive ID */
#define CAN_POWER_ID	0x010  //主控板反馈超级电容的信息

//#define Fdcan //操作此宏定义决定是否用fdcan发送


void SuperCapMsgSend(float cap_voltage,float cap_current);
void SuperCapStateSend(void);
void SuperCapMsgSendFdcan(float cap_voltage,float cap_current);
void ChassisMsgGet(uint32_t can_id,uint8_t * CAN_Rx_data);
void can_filter_init(void);


#endif
