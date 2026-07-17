#include "can_comm.h"
#include "fdcan.h"
#include "iwdg.h"

FDCAN_FilterTypeDef can_filter;
FDCAN_TxHeaderTypeDef tx_message;


imu_msg_t imu_msg_send;

void can_comm_init(void)
{

  // can过滤器设置
  // 底盘imu数据接收
//  can_filter.IdType = FDCAN_STANDARD_ID; // 标准帧
//  can_filter.FilterIndex = 0;
//  can_filter.FilterType = FDCAN_FILTER_RANGE; // 范围过滤
//  can_filter.FilterID1 = 0x0000;              // 32位ID
//  can_filter.FilterID2 = 0x0000;
//  can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 通过过滤后给邮箱0
//  HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter);
//  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 使能邮箱0新消息中断
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	// 初始化FDCAN时启用传输完成中断
	HAL_FDCAN_Start(&hfdcan1);
    // 配置标准发送参数
    tx_message.IdType = FDCAN_STANDARD_ID;
    tx_message.TxFrameType = FDCAN_DATA_FRAME;
    tx_message.DataLength = FDCAN_DLC_BYTES_8;
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_message.MessageMarker = 0;	
}

void fdcan_comm_init(void)
{

  // can过滤器设置
  // 底盘imu数据接收
//  can_filter.IdType = FDCAN_STANDARD_ID; // 标准帧
//  can_filter.FilterIndex = 0;
//  can_filter.FilterType = FDCAN_FILTER_RANGE; // 范围过滤
//  can_filter.FilterID1 = 0x0000;              // 32位ID
//  can_filter.FilterID2 = 0x0000;
//  can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 通过过滤后给邮箱0
//  HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter);
//  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 使能邮箱0新消息中断
//	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	// 初始化FDCAN时启用传输完成中断
	HAL_FDCAN_Start(&hfdcan1);
    // 配置标准发送参数
    tx_message.IdType = FDCAN_STANDARD_ID;
    tx_message.TxFrameType = FDCAN_DATA_FRAME;
    tx_message.DataLength = FDCAN_DLC_BYTES_64;
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_message.BitRateSwitch = FDCAN_BRS_ON;
    tx_message.FDFormat = FDCAN_FD_CAN;
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_message.MessageMarker = 0;	
}

/*
 * @brief     can发送标准数据统一接口，提供给其它文件调用，8字节数据长度
 * @param[in] can_periph: can通道
 * @param[in] id        : 帧id
 * @param[in] data      : 数据指针
 * @retval    void
 */
void can_std_transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
	static int32_t iwdg_cnt;
	tx_message.Identifier = id;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_message, data) == HAL_OK){
		if(iwdg_cnt>0)
			iwdg_cnt--;
	}
	else
		iwdg_cnt++;
	
	if(iwdg_cnt < 500)
		HAL_IWDG_Refresh(&hiwdg);
}
