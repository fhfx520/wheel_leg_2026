#include "drv_lk_motor.h"
#ifndef PI
	#define PI 3.14159265358979323846f
#endif
#define LIMIT_MIN_MAX(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

static list_t object_list = {&object_list, &object_list};

/*
 * @brief     瓴控电机初始化设置
 * @param[in] motor     : 电机数据结构体
 * @param[in] can_periph: 电机所在can通道
 * @param[in] id        : 电机发送报文id
 * @param[in] zero_point: 电机安装零点
 * @param[in] mst_id	: 电机反馈报文id
 * @retval    void
 */
void lk_motor_init(lk_motor_t *motor, can_channel_e can_channel, uint32_t id, float zero_point, uint32_t mst_id, uint8_t resoul_bits)
{
	motor->can_channel = can_channel;
	motor->can_id = id;
	motor->zero_point = zero_point;
    motor->mst_id = mst_id;
	motor->resoul_bits = resoul_bits;
	motor->max_ecd = 1;
	for(uint8_t i = 0;i < motor->resoul_bits;i++)
		motor->max_ecd *= 2;
	motor->max_ecd *= 4;
	lk_set_cmd(motor,START_MOTOR);
	list_add(&(motor->list),&object_list);
}
/*
 * @brief    	瓴控电机命令设置
 * @param[in] *motor    : 电机数据结构体指针
 * @param[in] cmd    	: 电机命令
 * @retval    void
 */
void lk_set_cmd(lk_motor_t* motor,lk_motor_cmd_e cmd)
{
	static uint8_t cmd_buff[8] = { 0 };
	cmd_buff[0] = cmd;
	if(cmd != READ_ERR || cmd != CLEAR_ERR || cmd != CLOSE_MOTOR || cmd != START_MOTOR)
		return;
	can_std_transmit(motor->can_channel,motor->can_id,cmd_buff);
}
/*
 * @brief    	瓴控电机电流发送
 * @param[in] *motor    : 电机数据结构体指针
 * @retval    void
 */
void lk_set_current(lk_motor_t* motor)
{
	static uint8_t iq_buff[8] = { 0 };
	iq_buff[0] = CURRENT_MODE;
	iq_buff[4] = (uint8_t)motor->tx_current;		//电流低8位
	iq_buff[5] = (uint8_t)(motor->tx_current >> 8);	//电流高8位
	can_std_transmit(motor->can_channel,motor->can_id,iq_buff);
	motor->send_cnt++;
	motor->online = 0;
}
/*
 * @brief    	瓴控电机接收数据处理
 * @param[in] *motor    : 电机数据结构体指针
 * @param[in] *data		: 电机接收数据指针
 * @retval    void
 */
void lk_get_single_data(lk_motor_t *motor,uint8_t *data)
{
	motor->receive_cnt++;
	motor->online = 1;
    motor->err_percent = ((float)motor->send_cnt - (float)motor->receive_cnt) / (float)motor->send_cnt;
	switch(data[0])
	{
		case CURRENT_MODE:
		{
			motor->temp = data[1];
			motor->rx_current = data[3] << 8 | data[2];
			motor->speed = data[5] << 8 | data[4];
			motor->ecd = data[7] << 8 | data[6];
			break;
		}
		case READ_ERR:
		{
			motor->temp = data[1];
			motor->err_code = data[7];
			break;
		}
		default:break;
	}
	motor->p = (float)motor->ecd / motor->max_ecd * 2 * PI;
	motor->v = motor->speed * PI / 180.0f;
	motor->t = motor->rx_current * 66.0f / 4096.0f / 2.75f;
}
