#include "bsp_fdcan.h"
#include "power_ctrl_task.h"
#include "fdcan.h"
#include "pid.h"
#include "string.h"

FDCAN_FilterTypeDef	FDCAN2_RXFilter;
FDCAN_TxHeaderTypeDef Tx1Message;
FDCAN_RxHeaderTypeDef Rx1Message;
FDCAN_TxHeaderTypeDef Tx2Message;
FDCAN_RxHeaderTypeDef Rx2Message;

uint8_t CAN1_Rx_data[8];
uint8_t CAN1_Tx_data[8];
uint8_t CAN2_Rx_data[8];
uint8_t CAN2_Tx_data[8];

uint8_t FDCAN2_Rx_data[64];
uint8_t FDCAN2_Tx_data[64];



/**
  * @brief     CAN????????
  * @param     CAN_Rx_data :CAN????????
  * @attention 
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if(hfdcan->Instance == FDCAN2)
	{
#ifdef Fdcan
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Rx2Message, FDCAN2_Rx_data);
#else
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Rx2Message, CAN2_Rx_data);
#endif
		switch (Rx2Message.Identifier)
		{
			case CAN_POWER_ID:
			{
			#ifdef Fdcan
				ChassisMsgGet(CAN_POWER_ID, FDCAN2_Rx_data);
			#else
				ChassisMsgGet(CAN_POWER_ID, CAN2_Rx_data);	
			#endif
				
				break;
			}
			
			default:
			{
				break;
			}
		}
		__HAL_FDCAN_ENABLE_IT(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
	}
}
/**
  * @brief     CAN????
  * @param     ?????,??????
  * @attention  Specifies the identifier.
							 This parameter must be a number between:
								- 0 and 0x7FF, if IdType is FDCAN_STANDARD_ID
								- 0 and 0x1FFFFFFF, if IdType is FDCAN_EXTENDED_ID 
	*/

void SuperCapMsgSend(float cap_voltage,float cap_current)
{
		uint8_t msg_uint8_t[8];
	
    Tx2Message.Identifier=0x100;                           //32位ID
    Tx2Message.IdType=FDCAN_STANDARD_ID;                  //标准ID
    Tx2Message.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    Tx2Message.DataLength=FDCAN_DLC_BYTES_8;              //数据长度
    Tx2Message.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    Tx2Message.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    Tx2Message.FDFormat=FDCAN_CLASSIC_CAN;                //传统can模式
    Tx2Message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //?????
    Tx2Message.MessageMarker=0x52;                           
    
//		memcpy(msg_uint8_t,&cap_state,sizeof(cap_state));
		memcpy(msg_uint8_t,&cap_voltage,sizeof(cap_voltage));
		memcpy(msg_uint8_t+4,&cap_current,sizeof(cap_current));
		for(uint8_t i=0;i<8;i++)
		{
				CAN2_Tx_data[i] = msg_uint8_t[i];
		}
		if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Tx2Message,CAN2_Tx_data);
}

void SuperCapStateSend(void)
{
		uint8_t msg_uint8_t[8];
	
    Tx2Message.Identifier=0x101;                           //32位ID
    Tx2Message.IdType=FDCAN_STANDARD_ID;                  //标准ID
    Tx2Message.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    Tx2Message.DataLength=FDCAN_DLC_BYTES_8;              //数据长度
    Tx2Message.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    Tx2Message.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    Tx2Message.FDFormat=FDCAN_CLASSIC_CAN;                //传统can模式
    Tx2Message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //?????
    Tx2Message.MessageMarker=0x52;                           
    
		memcpy(msg_uint8_t,&cap_state,sizeof(cap_state));
		memcpy(msg_uint8_t+1,&POWER_MODE,sizeof(POWER_MODE));
		for(uint8_t i=0;i<8;i++)
		{
				CAN2_Tx_data[i] = msg_uint8_t[i];
		}
		if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Tx2Message,CAN2_Tx_data);
}

void SuperCapMsgSendFdcan(float cap_voltage,float cap_current)
{
	uint8_t msg_uint8_t[12] = { 0 };
	
    Tx2Message.Identifier=0x100;                           //32位ID
    Tx2Message.IdType=FDCAN_STANDARD_ID;                   //标准ID
    Tx2Message.TxFrameType=FDCAN_DATA_FRAME;               //数据帧
    Tx2Message.DataLength=FDCAN_DLC_BYTES_12;              //12字节数据长度
    Tx2Message.ErrorStateIndicator=FDCAN_ESI_ACTIVE;       //主动错误状态发送     
    Tx2Message.BitRateSwitch=FDCAN_BRS_ON;                 //开启速率切换
    Tx2Message.FDFormat=FDCAN_FD_CAN;                	   //fdcan模式
    Tx2Message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;      //不开启发送事件控制
    Tx2Message.MessageMarker=0x52;                           
    
	memcpy(msg_uint8_t,&cap_voltage,sizeof(cap_voltage));
	memcpy(msg_uint8_t+4,&cap_current,sizeof(cap_current));
	memcpy(msg_uint8_t+8,&cap_state,sizeof(cap_state));
	memcpy(msg_uint8_t+9,&POWER_MODE,sizeof(POWER_MODE));
	memcpy(FDCAN2_Tx_data,msg_uint8_t,12);
	if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Tx2Message,FDCAN2_Tx_data);
}

void ChassisMsgGet(uint32_t can_id,uint8_t * CAN_Rx_data)
{
		switch(can_id)
		{
				case CAN_POWER_ID:
				{
						memcpy(&chassis_mode,CAN_Rx_data,1);
						error_time[6] = 0;
						break;
				}
		}
}

/**
  * @brief  init the can transmit and receive
  * @param  None
  */
void can_filter_init(void)
{
  /* can filter config */
    FDCAN2_RXFilter.IdType=FDCAN_STANDARD_ID;                       //??ID
    FDCAN2_RXFilter.FilterIndex=0;                                  //?????                   
    FDCAN2_RXFilter.FilterType=FDCAN_FILTER_MASK;                   //?????
    FDCAN2_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //???0???FIFO0  
    FDCAN2_RXFilter.FilterID1=0x0000;                               //32?ID
    FDCAN2_RXFilter.FilterID2=0x0000;                               //??FDCAN?????????,???32???
    if(HAL_FDCAN_ConfigFilter(&hfdcan2,&FDCAN2_RXFilter)!=HAL_OK) 	//??????
		{
			Error_Handler();
		}
	/* start the can transmit and receive */
		HAL_FDCAN_Start(&hfdcan2);                               //??FDCAN
    HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);

}

