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
    
    //can1过滤器设置
//底盘imu数据接收
//    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
//    can_filter.FilterIndex = 0;
//    can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤
//    can_filter.FilterID1 = 0x001;
//    can_filter.FilterID2 = 0x005;
//    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤后给邮箱0
//    HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter);
//底盘关节电机接收
    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
    can_filter.FilterIndex = 0;
    can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤
    can_filter.FilterID1 = 0x011;
    can_filter.FilterID2 = 0x014;
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//通过过滤后给邮箱1
	HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter);
    
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//使能邮箱0新消息中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能邮箱1新消息中断
    HAL_FDCAN_Start(&hfdcan1);
    
    //can2过滤器设置
//底盘2个驱动电机 trigger
    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
    can_filter.FilterIndex = 0;
    can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤
    can_filter.FilterID1 = 0x203;
    can_filter.FilterID2 = 0x205;
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//通过过滤后给邮箱1
    HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter);
    //功率控制
//    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
//    can_filter.FilterIndex = 1;
//    can_filter.FilterType = FDCAN_FILTER_DUAL;//等于过滤
//    can_filter.FilterID1 = 0x020;
//    can_filter.FilterID2 = 0x100;
//    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤后给邮箱0
//    HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter);
//底盘imu
    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
    can_filter.FilterIndex = 2;
    can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤
    can_filter.FilterID1 = 0x011;
    can_filter.FilterID2 = 0x015; 
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤后给邮箱0
    HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter);
    
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//使能邮箱0新消息中断
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能邮箱1新消息中断
    HAL_FDCAN_Start(&hfdcan2);

    //can3过滤器设置
    //云台陀螺仪
//    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
//    can_filter.FilterIndex = 0;
//    can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤
//    can_filter.FilterID1 = 0x011;
//    can_filter.FilterID2 = 0x014;
//    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤后给邮箱0
//    HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter);

//vision_shoot_enable
    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
    can_filter.FilterIndex = 0;
    can_filter.FilterType = FDCAN_FILTER_DUAL;//等于过滤
    can_filter.FilterID1 = 0x003;
	can_filter.FilterID2 = 0x006;
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤后给邮箱1
    HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter); 
//yaw电机
    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
    can_filter.FilterIndex = 1;
    can_filter.FilterType = FDCAN_FILTER_DUAL;//等于过滤
    can_filter.FilterID1 = 0x206;
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//通过过滤后给邮箱1
    HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter);   

//power
    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
    can_filter.FilterIndex = 2;
    can_filter.FilterType = FDCAN_FILTER_DUAL;//等于过滤
    can_filter.FilterID1 = 0x100;
	can_filter.FilterID2 = 0x101; 
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤后给邮箱0
    HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter);
		
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//使能邮箱0新消息中断
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能邮箱1新消息中断
    HAL_FDCAN_Start(&hfdcan3);
    
    //配置标准发送参数
    tx_message.IdType = FDCAN_STANDARD_ID;  
    tx_message.TxFrameType = FDCAN_DATA_FRAME;
    tx_message.DataLength = FDCAN_DLC_BYTES_8;
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_message.BitRateSwitch = FDCAN_BRS_ON;
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_message.MessageMarker = 0;
    
    //各驱动初始化
	//膝关节朝前，左腿：[0][1]，右腿[2][3] chuan
	dm_motor_init(&joint_motor[0], CAN_CHANNEL_1, 0x03, 2.79840f, 0x13);//B 3.480
	dm_motor_init(&joint_motor[1], CAN_CHANNEL_1, 0x04, 5.68684f, 0x14);//S 3.564
	dm_motor_init(&joint_motor[2], CAN_CHANNEL_1, 0x02, 1.49280f, 0x12);//B 4.394 
	dm_motor_init(&joint_motor[3], CAN_CHANNEL_1, 0x01, 1.96688f, 0x11);//S 3.770 
//	
//	dm_motor_init(&joint_motor[0], CAN_CHANNEL_1, 0x03, 2.79840f, 0x13);//B 3.480
//	dm_motor_init(&joint_motor[1], CAN_CHANNEL_1, 0x04, 2.5285f, 0x14);//S 3.564
//	dm_motor_init(&joint_motor[2], CAN_CHANNEL_1, 0x02, 1.49280f, 0x12);//B 4.394 
//	dm_motor_init(&joint_motor[3], CAN_CHANNEL_1, 0x01, 5.12156f, 0x11);//S 3.770 
	

//    dji_motor_init(&fric_motor[0], DJI_3508_MOTOR, CAN_CHANNEL_3, 0x201, 1.0f);
//    dji_motor_init(&fric_motor[1], DJI_3508_MOTOR, CAN_CHANNEL_3, 0x202, 1.0f);
    dji_motor_init(&trigger_motor, DJI_2006_MOTOR, CAN_CHANNEL_2, 0x205, 36.0f);
//    dji_motor_init(&driver_motor[0], DJI_3508_MOTOR, CAN_CHANNEL_2, 0x204, 19.0f);
//    dji_motor_init(&driver_motor[1], DJI_3508_MOTOR, CAN_CHANNEL_2, 0x203, 19.0f);
    dji_motor_init(&driver_motor[0], DJI_3508_MOTOR, CAN_CHANNEL_2, 0x203, 15.06f);
    dji_motor_init(&driver_motor[1], DJI_3508_MOTOR, CAN_CHANNEL_2, 0x204, 15.06f);
    dji_motor_init(&yaw_motor, DJI_6020_MOTOR, CAN_CHANNEL_3, 0x206, 1.0f);
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
//          imu_get_data(&chassis_imu, rx_fifo0_message.Identifier, rx_fifo0_data);
        } else if (hfdcan->Instance == FDCAN2) {
			imu_get_data(&chassis_imu, rx_fifo0_message.Identifier, rx_fifo0_data);
        } else if (hfdcan->Instance == FDCAN3) {
			if (rx_fifo0_message.Identifier == 0x100) 
				power_get_data(rx_fifo0_data);
			else if(rx_fifo0_message.Identifier == 0x101)
				power_get_status(rx_fifo0_data);
			else		
				vision_gimbal_get_data(&vision, rx_fifo0_message.Identifier, rx_fifo0_data);
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
//            ht_motor_get_data(rx_fifo1_data[0], rx_fifo1_data);
			dm_motor_get_data(rx_fifo1_message.Identifier, rx_fifo1_data);
        } else if (hfdcan->Instance == FDCAN2) {
            dji_motor_get_data(CAN_CHANNEL_2, rx_fifo1_message.Identifier, rx_fifo1_data);
        } else if (hfdcan->Instance == FDCAN3) {
            dji_motor_get_data(CAN_CHANNEL_3, rx_fifo1_message.Identifier, rx_fifo1_data);
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
