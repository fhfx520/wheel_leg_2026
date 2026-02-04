/**
  * @file    bsp_LK_Motor_MG4005.c
  * @version A44A_gimbal_MG4005版
  * @date    2025.1.6
  * @brief 	 领控MG4005驱动程序									
  *	@author  空中组
  * @warning 电机调参请注意以下几点：
  *           1.
	*@note  
	*           命令报文ID：0x140 + ID(1~32)
	            回复报文ID：0x180 + ID(1~32)
  */
	
#include "main.h"
#include "can_comm.h"



#include "fdcan.h"

#include "bsp_LK_Motor_MG4005.h"
//#include "modeswitch_task.h"
#include "gimbal_task.h"
//#include "math_calcu.h"
//#include "math_lib.h"
#include "control_def.h"
//#include "remote_msg.h"

//新拨盘需要4005
typedef enum
{
    CAN_3508_FRIC_L_ID       = 0x201,
    CAN_3508_FRIC_R_ID       = 0x202,
    CAN_TRIGGER_MOTOR1_ID = 0x207,
	
    CAN_YAW_MOTOR_ID      = 0x206,//2025年1月17日 观察发现目前
    CAN_ROLL_MOTOR_ID      = 0x000,
	
	  CAN_J4310_ID=0X06,//J4310电机控制帧ID   
		CAN_MG4005_T_ID=0x142 ,					//0x147					//2025年1月6日：预计新pit电机MG4005的id为0x07（电机背面编码开关的数值 亦即motorId）   即 0x140(厂家规定) + 0x07 
	  CAN_MG4005_R_ID=0x187,//2025年1月10日 ：厂家规定的CAN反馈id是0x180+id，假的，其实还是0x140  后面在问问师兄


} can_msg_id_e;


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 电机参数结构体

#define     Kt                 0.06;          // 参考MG4005参数表 ：扭矩常数 0.06 Nm/A
#define     Ifriction          0   ;          // 标定获得的摩擦补偿电流 AI说要标定不动时候的扭矩，暂时还没想通，我要的是电机的输出，是不是应该包含了全部的扭矩输出，而不是云台动起来所需的额外扭矩？
#define     I_offset           0   ;          // 零点漂移补偿
#define     motor_pp           26  ;          //也有可能是28？
/* USER CODE END PD */



/* USER CODE BEGIN PV */
uint8_t OFFSET_ECD_FLAG=0;//校正后置1
// uint32_t lk_motor_debug_cnt=0;//隔一段时间就查询一次状态，如果过热关机了，而且查到电机关机， 就重启，并报错给UI和log
 uint8_t LK_REINIT_FLAG;

//定义拨盘电机MG4005
Motor_MG4005_t trigger_motor_MG4005;

//1111
///**
//	* @brief	电机运行命令（解锁）
//	* @param 	can句柄
//	* @param 	电机id   CAN_MG4005_ID
//	* @return 
//	* @note 将电机从关闭状态切换到开启状态，LED 由慢闪转为常亮。此时再发送控制指令即可控制电机动作。驱动回复和主机发送相同。
//	*/
//void start_MG4005_motor(CAN_HandleTypeDef *hcan,uint16_t id)
//{	
//	uint8_t FreeTxNum=0;
//	static uint32_t MailBox;
//	uint8_t CAN_Tx_data[8];
//	CAN_TxHeaderTypeDef Tx1Message;
//	
//	Tx1Message.StdId=id;
//	Tx1Message.IDE=CAN_ID_STD;
//	Tx1Message.RTR=CAN_RTR_DATA;
//	Tx1Message.DLC=0x08;
//	
//	CAN_Tx_data[0]=0x88;
//	CAN_Tx_data[1]=0x00;
//	CAN_Tx_data[2]=0x00;
//	CAN_Tx_data[3]=0x00;
//	CAN_Tx_data[4]=0x00;
//	CAN_Tx_data[5]=0x00;
//	CAN_Tx_data[6]=0x00;
//	CAN_Tx_data[7]=0x00;
//	 
//	
//	uint8_t block_cnt = 0;
//    //查询发送邮箱是否为空
//    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
//    while(FreeTxNum == 0)
//    {
//		block_cnt++;
//        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
//		if(block_cnt >= 20)
//		{
//			return;
//		}
//    }
//    HAL_CAN_AddTxMessage(hcan, &Tx1Message, CAN_Tx_data, &MailBox);

//	
//}	

void start_MG4005_motor(FDCAN_HandleTypeDef *hfdcan, uint16_t id)
{
    uint8_t FreeTxNum = 0;
    static uint32_t MailBox;
    uint8_t FDCAN_Tx_data[8];
    FDCAN_TxHeaderTypeDef Tx1Message;

    Tx1Message.Identifier = id;
    Tx1Message.IdType = FDCAN_STANDARD_ID;
    Tx1Message.TxFrameType = FDCAN_DATA_FRAME;
    Tx1Message.DataLength = FDCAN_DLC_BYTES_8;  // 数据长度
    Tx1Message.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 错误状态指示器
    Tx1Message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    FDCAN_Tx_data[0] = 0x88;
    FDCAN_Tx_data[1] = 0x00;
    FDCAN_Tx_data[2] = 0x00;
    FDCAN_Tx_data[3] = 0x00;
    FDCAN_Tx_data[4] = 0x00;
    FDCAN_Tx_data[5] = 0x00;
    FDCAN_Tx_data[6] = 0x00;
    FDCAN_Tx_data[7] = 0x00;

    uint8_t block_cnt = 0;
    // 查询发送邮箱是否为空
    FreeTxNum = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
    while (FreeTxNum == 0)
    {
        block_cnt++;
        FreeTxNum = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
        if (block_cnt >= 20)
        {
            return;
        }
    }
		if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &Tx1Message, FDCAN_Tx_data) != HAL_OK)
		{
				// 发送失败，处理错误
				Error_Handler();
		}
}

//1111
/**
	* @brief	MG4005电机数据处理
	* @param 	电机指针？
	* @param 	CAN数据指针
	* @return 
	* @note 
	*/
void MG4005_Data_Handler(Motor_MG4005_t* mtr,uint8_t* CAN_Rx_Data)
{	
   switch (CAN_Rx_Data[0])
    {
			
		    case 0x88://启动成功
				{
					mtr->MG4005_motor_mode = MG4005_Run;
					mtr->MG4005_motorState = LK_RUNNING;  //现在有两个标志电机运行与否姿态的枚举，好臃肿，改掉！
					break;
				}
			  case 0x81://电机停止命令
				{
					 mtr->MG4005_motor_mode=MG4005_Stop;
					break;
				}
				
			 
			  case 0x9A:// 数据读取1
        {
						mtr->temperature = ((int8_t)CAN_Rx_Data[1]);
						mtr->voltage = ((int16_t)(CAN_Rx_Data[3]<<8)|(CAN_Rx_Data[2]))*0.01;//母线电压 voltage
						mtr->current =((CAN_Rx_Data[5]<<8)|(CAN_Rx_Data[4])/100);//母线电流 current        
						mtr->MG4005_motorState =(MG4005_motorState_e)(CAN_Rx_Data[6]);//电机状态 motorState
						mtr->lk_motor_err_state.err_state =(uint8_t)(CAN_Rx_Data[7]);//错误标志 errorState
					  
//					 	if(lk_motor_debug_cnt!= 0x00)
//						{
//							LK_REINIT_FLAG++;
//							MG4005_ReInit(&hcan2,mtr);
//						}
					
            break;
        }
				
				 case 0x9B:// 数据读取1
        {
//						mtr->temperature = ((int8_t)CAN_Rx_Data[1]);
//						mtr->voltage = ((int16_t)(CAN_Rx_Data[3]<<8)|(CAN_Rx_Data[2]))*0.01;//母线电压 voltage
//						mtr->current =((CAN_Rx_Data[5]<<8)|(CAN_Rx_Data[4])/100);//母线电流 current        
//						mtr->MG4005_motorState =(MG4005_motorState_e)(CAN_Rx_Data[6]);//电机状态 motorState
//						mtr->lk_motor_err_state.err_state =(uint8_t)(CAN_Rx_Data[7]);//错误标志 errorState
//					  
////					 	if(lk_motor_debug_cnt!= 0x00)
////						{
////							LK_REINIT_FLAG++;
//							MG4005_ReInit(&hcan2,mtr);
//						}
					
            break;
        }
				
        case 0x9C:// 数据读取2
        {
					
					 /*校正offset_ecd*/
					  if(OFFSET_ECD_FLAG==0)//上电后仅校准一次offset ecd
							{
                   mtr->raw_encoder        = (uint16_t)(CAN_Rx_Data[7] << 8 | CAN_Rx_Data[6]);
                   mtr->offset_encoder = mtr->raw_encoder;	
								
								OFFSET_ECD_FLAG=1;
							}

							
						mtr->temperature = ((int8_t)CAN_Rx_Data[1]);
						mtr->iq = (CAN_Rx_Data[3]<<8)|(CAN_Rx_Data[2])*66/4096;
						mtr->speed =(CAN_Rx_Data[5]<<8)|(CAN_Rx_Data[4]);//1dps/LSB
  
						mtr->raw_last_encoder = mtr->raw_encoder;//机械角度
						mtr->raw_encoder      = (uint16_t)(CAN_Rx_Data[7] << 8 | CAN_Rx_Data[6]);
					
					
					 break;
        }
        case 0x9D:// 数据读取2
        {
            mtr->temperature = (int8_t)CAN_Rx_Data[1];
						mtr->iA = (CAN_Rx_Data[3]<<8)|(CAN_Rx_Data[2])*66/4096;
						mtr->iB = (CAN_Rx_Data[5]<<8)|(CAN_Rx_Data[4])*66/4096;
						mtr->iC = (CAN_Rx_Data[7]<<8)|(CAN_Rx_Data[6])*66/4096;
					  
					  //2025年3月12日 临时新增三相电流——》扭矩 的计算函数用于测试工况力矩，实际使用中可注释掉
//					  MG4005_CalculateTorque(mtr);
					
					
            break;
        }
				
				

			
				case 0xA1:
				case 0xA2:
				case 0xA6:// 力矩闭环|速度闭环|位置闭环， 返回的数据都相同，都在这个case里解算
        {
			
					  mtr->MG4005_motor_mode=MG4005_Run;
						mtr->temperature = ((int8_t)CAN_Rx_Data[1]);
						mtr->iq = (CAN_Rx_Data[3]<<8)|(CAN_Rx_Data[2])*33/4096;//还没写数值转换
						mtr->speed =(CAN_Rx_Data[5]<<8)|(CAN_Rx_Data[4]);//1dps/LSB
  

    

		
		        //电机返回的encoder数据（减速前位置；0~65535）
						mtr->raw_last_encoder = mtr->raw_encoder;//机械角度
						mtr->raw_encoder      = ((uint16_t)(CAN_Rx_Data[7] << 8 | CAN_Rx_Data[6]));	
					
					  //过零判断
            if (mtr->raw_encoder - mtr->raw_last_encoder > 32767) 
                  mtr->round_cnt++;
            else if (mtr->raw_encoder - mtr->raw_last_encoder < -32767)
                  mtr->round_cnt--;
						
            mtr->total_encoder = mtr->round_cnt * (-65535) + mtr->raw_encoder - mtr->offset_encoder;
						
						//单圈内电机的绝对角度
		        mtr->ecd_pos = trigger_motor_MG4005.total_encoder*LK_MOTOR_ECD_TO_RAD;//编码器转到角度制(0°~360°)  校正后，本来此时电机ecd位置为0。若运动范围囊括了零点，则PID计算的情况会比较复杂。 故手动给total_ecd添加120度，将零点“驱逐出运动范围”。（运动范围为0°~60°，故需要给total_ecd添加至少60度）							
						mtr->pos=mtr->ecd_pos/65535;  
						mtr->ele_pos=mtr->pos*motor_pp; //FOC计算用，电角度=机械角度*极对数 
					 break;
        }
        default: break;
    }
}


//1111
///**
//	* @brief	MG4005电机力矩闭环控制命令
//	* @param 	can句柄
//	* @param 	电机数据指针
//	* @return 
//	* @note   1. 该命令中的控制值 iqControl 不受上位机中的 Max Torque Current 值限制
//            2. 未特殊校准前 toq_i_ref正，电机产生顺时针力矩  与imu的坐标系相反（这里不做注释，坐标系正负号混乱会导致严重的失控 疯车！ 每次调参之前必须自己重新推演一遍！）
//	*/
//void MG4005_Toq_Ctrl(CAN_HandleTypeDef *hcan,   uint16_t id,  int16_t toq_i_ref)
//{
//	uint8_t FreeTxNum=0;
//	uint8_t CAN_Tx_data[8];
//		static uint32_t MailBox;
//	CAN_TxHeaderTypeDef Tx1Message;

//	static int16_t temp;
//	temp=toq_i_ref;//这行代码有毒！ 每次使用这个函数之前必须自己推演一遍正负号！

//	
//	Tx1Message.StdId = id;
//	Tx1Message.IDE = CAN_ID_STD;
//	Tx1Message.RTR = CAN_RTR_DATA;
//	Tx1Message.DLC = 0x08;
//	
//	CAN_Tx_data[0] = 0xA1;
//	CAN_Tx_data[1] = 0x00;
//	CAN_Tx_data[2] = 0x00;
//	CAN_Tx_data[3] = 0x00;
//	CAN_Tx_data[4] = *(uint8_t  *)(& temp);
//	CAN_Tx_data[5] = *((uint8_t *)(& temp)+1);
//	CAN_Tx_data[6] = 0x00;
//	CAN_Tx_data[7] = 0x00;
//	
//		uint8_t block_cnt = 0;
//    //查询发送邮箱是否为空
//    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
//    while(FreeTxNum == 0)
//    {
//		block_cnt++;
//        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
//		if(block_cnt >= 20)
//		{
//			return;
//		}
//    }
//    HAL_CAN_AddTxMessage(hcan, &Tx1Message, CAN_Tx_data, &MailBox);
//}	

/**
  * @brief  MG4005 电机力矩闭环控制命令
  * @param  hfdcan FDCAN 句柄
  * @param  id 电机 ID
  * @param  toq_i_ref 目标力矩电流参考值
  * @note   此函数将目标力矩电流 `toq_i_ref` 发送到电机进行控制
  */
void MG4005_Toq_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t id, int16_t toq_i_ref)
{
    uint8_t FDCAN_Tx_data[8];  // 数据缓冲区
    static uint32_t MailBox;    // 邮箱，用于发送消息

    // FDCAN 发送消息的消息头
    FDCAN_TxHeaderTypeDef Tx1Message;

    // 设置消息头
    Tx1Message.Identifier = id;  // 电机 ID
    Tx1Message.IdType = FDCAN_STANDARD_ID;  // 使用标准帧（标准 11 位标识符）
    Tx1Message.TxFrameType = FDCAN_DATA_FRAME;  // 数据帧
    Tx1Message.DataLength = FDCAN_DLC_BYTES_8;  // 数据长度为 8 字节
    Tx1Message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 错误状态指示器（正常工作状态）
    Tx1Message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;  // 不生成发送事件

    // 填充数据：目标力矩电流参考值 toq_i_ref
    FDCAN_Tx_data[0] = 0xA1;  // 控制命令标识符
    FDCAN_Tx_data[1] = 0x00;  // 保留
    FDCAN_Tx_data[2] = 0x00;  // 保留
    FDCAN_Tx_data[3] = 0x00;  // 保留
    FDCAN_Tx_data[4] = *(uint8_t *)(&toq_i_ref);  // 将目标力矩电流的低字节填入
    FDCAN_Tx_data[5] = *((uint8_t *)(&toq_i_ref) + 1);  // 将目标力矩电流的高字节填入
    FDCAN_Tx_data[6] = 0x00;  // 保留
    FDCAN_Tx_data[7] = 0x00;  // 保留

    uint8_t block_cnt = 0;
    // 查询发送邮箱是否为空
    uint8_t FreeTxNum = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
    while (FreeTxNum == 0)
    {
        block_cnt++;
        FreeTxNum = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
        if (block_cnt >= 20)  // 超过最大尝试次数，退出
        {
            return;
        }
    }

    // 发送消息
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &Tx1Message, FDCAN_Tx_data) != HAL_OK)
    {
        // 发送失败，处理错误
        Error_Handler();
    }
}