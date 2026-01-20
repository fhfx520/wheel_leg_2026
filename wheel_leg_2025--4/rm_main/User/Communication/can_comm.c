#include "can_comm.h"
#include "drv_dji_motor.h"
#include "drv_ht_motor.h"
#include "prot_imu.h"
#include "prot_power.h"
#include "fdcan.h"
#include "drv_dm_motor.h"
#include "prot_vision.h"


FDCAN_TxHeaderTypeDef tx_message;
FDCAN_RxHeaderTypeDef rx_fifo0_message, rx_fifo1_message;
//注意FDCAN只能设置64个字节给用，设置8个会数组越界进硬件错误中断
uint8_t rx_fifo0_data[64], rx_fifo1_data[64];


/*
 * @brief  can总线初始化
 * @retval void
 * @note   设置过滤器，添加各驱动的初始化函数
 */
void can_comm_init(void)
{
    FDCAN_FilterTypeDef can_filter;    
	//can1 filter config
	//step motor -- dm8009p * 4
	can_filter.IdType = FDCAN_STANDARD_ID;//STANDARD
    can_filter.FilterIndex = 0;
    can_filter.FilterType = FDCAN_FILTER_RANGE;//RANGE
    can_filter.FilterID1 = 0x011;
    can_filter.FilterID2 = 0x014;
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//To Fifo0
    HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter);
    
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//使能邮箱0新消息中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能邮箱1新消息中断
    HAL_FDCAN_Start(&hfdcan1);
	//can1 filter config begin
    
    //can2 filter config begin
    //imu
    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
    can_filter.FilterIndex = 0;
    can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤
    can_filter.FilterID1 = 0x011;
    can_filter.FilterID2 = 0x014;
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//通过过滤后给邮箱1
	HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter);

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//使能邮箱0新消息中断
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能邮箱1新消息中断
    HAL_FDCAN_Start(&hfdcan2);
	//can2 filter config finish

    //can3 filter config begin
	//wheel motor
	can_filter.IdType = FDCAN_STANDARD_ID;//STANDARD
    can_filter.FilterIndex = 0;
    can_filter.FilterType = FDCAN_FILTER_DUAL;//RANGE
    can_filter.FilterID1 = 0x203;
    can_filter.FilterID2 = 0x204;
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//To Fifo0
    HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter);
	//vision_shoot_enable
    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
    can_filter.FilterIndex = 1;
    can_filter.FilterType = FDCAN_FILTER_DUAL;//等于过滤
    can_filter.FilterID1 = 0x003;
	can_filter.FilterID2 = 0x006;
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//通过过滤后给邮箱1
    HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter); 
	//yaw电机
    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
    can_filter.FilterIndex = 2;
    can_filter.FilterType = FDCAN_FILTER_DUAL;//等于过滤
    can_filter.FilterID1 = 0x206;
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤后给邮箱1
    HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter);   

	//power
    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
    can_filter.FilterIndex = 3;
    can_filter.FilterType = FDCAN_FILTER_DUAL;//等于过滤
    can_filter.FilterID1 = 0x100;
	can_filter.FilterID2 = 0x101; 
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//通过过滤后给邮箱0
    HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter);
		
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//使能邮箱0新消息中断
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能邮箱1新消息中断
    HAL_FDCAN_Start(&hfdcan3);
	//can3 filter config finish
    
    //tx_message config
    tx_message.IdType = FDCAN_STANDARD_ID;  
    tx_message.TxFrameType = FDCAN_DATA_FRAME;
    tx_message.DataLength = FDCAN_DLC_BYTES_8;
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_message.BitRateSwitch = FDCAN_BRS_ON;
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_message.MessageMarker = 0;
    
    //driver init
	dm_motor_init(&joint_motor[0], CAN_CHANNEL_1, 0x03, -0.422233582f, 0x13);//B 3.480 -0.422233582
	dm_motor_init(&joint_motor[1], CAN_CHANNEL_1, 0x04, 4.51793051f, 0x14);//S 3.564  4.51793051
	dm_motor_init(&joint_motor[2], CAN_CHANNEL_1, 0x02, 4.62540722f, 0x12);//B 4.394 4.62540722
	dm_motor_init(&joint_motor[3], CAN_CHANNEL_1, 0x01, 3.35368752f, 0x11);//S 3.770  3.35368752

//    dji_motor_init(&fric_motor[0], DJI_3508_MOTOR, CAN_CHANNEL_3, 0x201, 1.0f);
//    dji_motor_init(&fric_motor[1], DJI_3508_MOTOR, CAN_CHANNEL_3, 0x202, 1.0f);
//    dji_motor_init(&trigger_motor, DJI_2006_MOTOR, CAN_CHANNEL_2, 0x205, 36.0f);
//    dji_motor_init(&driver_motor[0], DJI_3508_MOTOR, CAN_CHANNEL_2, 0x204, 19.0f);
//    dji_motor_init(&driver_motor[1], DJI_3508_MOTOR, CAN_CHANNEL_2, 0x203, 19.0f);
    dji_motor_init(&driver_motor[0], DJI_3508_MOTOR, CAN_CHANNEL_3, 0x204, DJI_3508_TAURUS_REDUCTION_RATIO);
    dji_motor_init(&driver_motor[1], DJI_3508_MOTOR, CAN_CHANNEL_3, 0x203, DJI_3508_TAURUS_REDUCTION_RATIO);
//    dji_motor_init(&yaw_motor, DJI_6020_MOTOR, CAN_CHANNEL_3, 0x206, 1.0f);
//    dji_motor_init(&pit_motor, DJI_6020_MOTOR, CAN_CHANNEL_3, 0x206, 1.0f);
}

/*
 * @brief  邮箱0接收回调函数
 * @retval void
 * @note   在其中添加各驱动的数据接收函数
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_fifo0_message, rx_fifo0_data);
        if (hfdcan->Instance == FDCAN1) {
			dm_motor_get_data(rx_fifo0_message.Identifier, rx_fifo0_data);
        } else if (hfdcan->Instance == FDCAN2) {
			
        } else if (hfdcan->Instance == FDCAN3) {
			dji_motor_get_data(CAN_CHANNEL_3, rx_fifo0_message.Identifier, rx_fifo0_data);
//			if (rx_fifo0_message.Identifier == 0x100) 
//				power_get_data(rx_fifo0_data);
//			else if(rx_fifo0_message.Identifier == 0x101)
//				power_get_status(rx_fifo0_data);
//			else		
//				vision_gimbal_get_data(&vision, rx_fifo0_message.Identifier, rx_fifo0_data);
        }
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}

/*
 * @brief  邮箱1接收回调函数
 * @retval void
 * @note   在其中添加各驱动的数据接收函数
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_fifo1_message, rx_fifo1_data);
        if (hfdcan->Instance == FDCAN1) {
//			 dji_motor_get_data(CAN_CHANNEL_1, rx_fifo1_message.Identifier, rx_fifo1_data);
        } else if (hfdcan->Instance == FDCAN2) {
           imu_get_data(&chassis_imu, rx_fifo1_message.Identifier, rx_fifo1_data);
        } else if (hfdcan->Instance == FDCAN3) {
           if (rx_fifo1_message.Identifier == 0x100) 
				power_get_data(rx_fifo1_data);
			else if(rx_fifo1_message.Identifier == 0x101)
				power_get_status(rx_fifo1_data);
			else		
				vision_gimbal_get_data(&vision, rx_fifo1_message.Identifier, rx_fifo1_data);
        }
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    }
}

/*
 * @brief     can发送标准数据统一接口，提供给其它文件调用，8字节数据长度
 * @param[in] can_periph: can通道
 * @param[in] id        : 帧id
 * @param[in] data      : 数据指针
 * @retval    v oid
 */
void can_std_transmit(can_channel_e can_periph, uint32_t id, uint8_t *data)
{
    tx_message.Identifier = id;
    if (can_periph == CAN_CHANNEL_1)
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, data);
    else if (can_periph == CAN_CHANNEL_2)
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_message, data);
    else if (can_periph == CAN_CHANNEL_3)
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_message, data);
}
