#include "bsp_can.h"

uint8_t pit_can_tx_data[8];
uint8_t yaw_can_tx_data[8];
uint8_t rol_can_tx_data[8];
uint8_t acc_can_tx_data[8];
CAN_TxHeaderTypeDef TxMessage;

static void can_filter_init(void)
{
    //can filter config
    CAN_FilterTypeDef  can_filter;
    
    can_filter.FilterBank           = 0;
    can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh         = 0x0000;
    can_filter.FilterIdLow          = 0x0000;
    can_filter.FilterMaskIdHigh     = 0x0000;
    can_filter.FilterMaskIdLow      = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.SlaveStartFilterBank = 0;  
    can_filter.FilterActivation     = ENABLE;
    
    HAL_CAN_ConfigFilter(&hcan, &can_filter);
    while (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK);
}

void can_device_init(void)
{
    can_filter_init();
    HAL_Delay(100);
    HAL_CAN_Start(&hcan);
}

void send_pit_message(void)
{
    uint8_t FreeTxNum = 0;
    TxMessage.StdId   = 0x011;
    TxMessage.IDE     = CAN_ID_STD;
    TxMessage.RTR     = CAN_RTR_DATA;
    TxMessage.DLC     = 0x08;
    
//    pit_can_tx_data[0] = angle_tx_data[4];//åº•ç›˜
//    pit_can_tx_data[1] = angle_tx_data[5];
//    pit_can_tx_data[2] = angle_tx_data[6];
//    pit_can_tx_data[3] = angle_tx_data[7];
//    pit_can_tx_data[4] = palstance_tx_data[4];
//    pit_can_tx_data[5] = palstance_tx_data[5];
//    pit_can_tx_data[6] = palstance_tx_data[6];
//    pit_can_tx_data[7] = palstance_tx_data[7];
    pit_can_tx_data[0] = angle_tx_data[0];//äº‘å°
    pit_can_tx_data[1] = angle_tx_data[1];
    pit_can_tx_data[2] = angle_tx_data[2];
    pit_can_tx_data[3] = angle_tx_data[3];
    pit_can_tx_data[4] = palstance_tx_data[0];
    pit_can_tx_data[5] = palstance_tx_data[1];
    pit_can_tx_data[6] = palstance_tx_data[2];
    pit_can_tx_data[7] = palstance_tx_data[3];
    //²éÑ¯·¢ËÍÓÊÏäÊÇ·ñÎª¿Õ
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    while(FreeTxNum == 0) {  
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    }
    HAL_CAN_AddTxMessage(&hcan, &TxMessage, pit_can_tx_data, (uint32_t*)CAN_TX_MAILBOX1);
}

void send_yaw_message(void)
{
    uint8_t FreeTxNum = 0;
    TxMessage.StdId   = 0x012;
    TxMessage.IDE     = CAN_ID_STD;
    TxMessage.RTR     = CAN_RTR_DATA;
    TxMessage.DLC     = 0x08;
    
    yaw_can_tx_data[0] = angle_tx_data[8];
    yaw_can_tx_data[1] = angle_tx_data[9];
    yaw_can_tx_data[2] = angle_tx_data[10];
    yaw_can_tx_data[3] = angle_tx_data[11];
    yaw_can_tx_data[4] = palstance_tx_data[8];
    yaw_can_tx_data[5] = palstance_tx_data[9];
    yaw_can_tx_data[6] = palstance_tx_data[10];
    yaw_can_tx_data[7] = palstance_tx_data[11];
    //²éÑ¯·¢ËÍÓÊÏäÊÇ·ñÎª¿Õ
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    while(FreeTxNum == 0) {  
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    }
    HAL_CAN_AddTxMessage(&hcan, &TxMessage, yaw_can_tx_data, (uint32_t*)CAN_TX_MAILBOX1);
}

void send_rol_message(void)
{
    uint8_t FreeTxNum = 0;
    TxMessage.StdId   = 0x013;
    TxMessage.IDE     = CAN_ID_STD;
    TxMessage.RTR     = CAN_RTR_DATA;
    TxMessage.DLC     = 0x08;
    
    rol_can_tx_data[0] = angle_tx_data[4];
    rol_can_tx_data[1] = angle_tx_data[5];
    rol_can_tx_data[2] = angle_tx_data[6];
    rol_can_tx_data[3] = angle_tx_data[7];
    rol_can_tx_data[4] = palstance_tx_data[4];
    rol_can_tx_data[5] = palstance_tx_data[5];
    rol_can_tx_data[6] = palstance_tx_data[6];
    rol_can_tx_data[7] = palstance_tx_data[7];
//    rol_can_tx_data[0] = angle_tx_data[0];
//    rol_can_tx_data[1] = angle_tx_data[1];
//    rol_can_tx_data[2] = angle_tx_data[2];
//    rol_can_tx_data[3] = angle_tx_data[3];
//    rol_can_tx_data[4] = palstance_tx_data[0];
//    rol_can_tx_data[5] = palstance_tx_data[1];
//    rol_can_tx_data[6] = palstance_tx_data[2];
//    rol_can_tx_data[7] = palstance_tx_data[3];
    //²éÑ¯·¢ËÍÓÊÏäÊÇ·ñÎª¿Õ
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    while(FreeTxNum == 0) {  
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    }
    HAL_CAN_AddTxMessage(&hcan, &TxMessage, rol_can_tx_data, (uint32_t*)CAN_TX_MAILBOX1);
}

void send_acc_message(void)
{
    uint8_t FreeTxNum = 0;
    TxMessage.StdId   = 0x014;
    TxMessage.IDE     = CAN_ID_STD;
    TxMessage.RTR     = CAN_RTR_DATA;
    TxMessage.DLC     = 0x08;
    
    acc_can_tx_data[0] = accelerat_tx_data[0];
    acc_can_tx_data[1] = accelerat_tx_data[1];
    acc_can_tx_data[2] = accelerat_tx_data[2];
    acc_can_tx_data[3] = accelerat_tx_data[3];
    acc_can_tx_data[4] = accelerat_tx_data[8];
    acc_can_tx_data[5] = accelerat_tx_data[9];
    acc_can_tx_data[6] = accelerat_tx_data[10];
    acc_can_tx_data[7] = accelerat_tx_data[11];
    //²éÑ¯·¢ËÍÓÊÏäÊÇ·ñÎª¿Õ
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    while(FreeTxNum == 0) {  
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    }
    HAL_CAN_AddTxMessage(&hcan, &TxMessage, acc_can_tx_data, (uint32_t*)CAN_TX_MAILBOX1);
}
