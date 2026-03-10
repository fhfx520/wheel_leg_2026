#include "mode_switch_task.h"
#include "prot_dr16.h"
#include "prot_judge.h"
#include "cmsis_os.h"
#include "status_task.h"
#include "drv_dji_motor.h"
#include "prot_vision.h"
#include "chassis_task.h"
#include "board_comm.h"
#include "container.h"
#include "prot_imu.h"

uint8_t lock_flag = 0;
uint8_t reset_flag = 0;
ctrl_mode_e ctrl_mode;

static data_keyboard_t modesw_get_keyboard_data_container;

static rc_data_t modesw_set_rc_data_container;
static kb_data_t modesw_set_kb_data_container;
static judge_data_t modesw_set_judge_data_container;

// 收到vision数据：
static void keyboard_data_cb(uint32_t tag_id, void* data, size_t len) {
	if(data == NULL || len != sizeof(data_keyboard_t))
		return;
	memcpy(&modesw_get_keyboard_data_container,(data_keyboard_t*)data,len);
}

// --- 回调配置表  ---
static const ContainerBusCfg mb_callback[] = {
    { TAG_VTM_KEYBOARD_DATA, keyboard_data_cb, NULL }
};


static void unlock_init(void) {
    if (rc.sw1 == RC_UP && rc.sw2 == RC_UP ) { //左拨杆置上，右拨杆置上
        if (rc.ch4 < -600 && rc.ch3 > 600) {
            lock_flag = 1;  //左控制杆拨至右下
        }
    }
}

uint16_t chassis_power_cnt;
static void sw1_mode_handler(void) { //由拨杆1决定系统模式切换，主要是云台、底盘和发射器
	
	static uint8_t last_chassis_power = 0;
    switch (rc.sw1) {
        case RC_UP: {
            ctrl_mode = PROTECT_MODE;break;
        }
        case RC_MI: {
            ctrl_mode = REMOTER_MODE;break;
        }
        case RC_DN: {
//        if (rc.mouse.r == 1) {
//            ctrl_mode = VISION_MODE;    //视觉模式，右键开启
//        } else {
            ctrl_mode = KEYBOARD_MODE;break;
        }
        default:break;
    }
		
//		if ( (last_chassis_power ==0 && robot_status.power_management_chassis_output == 1 && status.judge == 0) || (chassis_power_cnt != 0 && robot_status.power_management_chassis_output == 1 ) ){
//			if (!driver_motor[0].online || !driver_motor[1].online) {
//				 ctrl_mode = PROTECT_MODE;
//		}
		if (chassis.recover_flag == 1 && rotate_flag)
			ctrl_mode = PROTECT_MODE;
				
		last_chassis_power = robot_status.power_management_chassis_output ;
}

static void remote_reset(void)
{
    //保护模式下右拨杆拨至左下
    if (rc.sw1 == RC_UP && rc.sw2 == RC_UP && rc.ch1 == -660 && rc.ch2 == -660) {
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }
}

void modesw_set_container(void)
{
	modesw_set_rc_data_container.ch1 = rc.ch1;
	modesw_set_rc_data_container.ch2 = rc.ch2;
	modesw_set_rc_data_container.sw1 = rc.sw1;
	modesw_set_rc_data_container.sw2 = rc.sw2;
	modesw_set_rc_data_container.rc_init_status = rc.init_status;
	modesw_set_rc_data_container.ctrl_mode = ctrl_mode;
	if (chassis.recover_flag == 1 && fabs(chassis_imu.pit) > 0.3f)
		modesw_set_rc_data_container.ctrl_mode = PROTECT_MODE;
	container_set(TAG_DR16_RC_DATA,&modesw_set_rc_data_container,sizeof(modesw_set_rc_data_container),CONTAINER_TYPE_STRUCT);
	
	modesw_set_kb_data_container.l = rc.mouse.l;
	modesw_set_kb_data_container.r = rc.mouse.r;
	modesw_set_kb_data_container.x = rc.mouse.x;
	modesw_set_kb_data_container.y = rc.mouse.y;
	modesw_set_kb_data_container.z = rc.mouse.z;
	modesw_set_kb_data_container.key_code = rc.kb.key_code;
	container_set(TAG_DR16_KB_DATA,&modesw_set_kb_data_container,sizeof(modesw_set_kb_data_container),CONTAINER_TYPE_STRUCT);
	
	modesw_set_judge_data_container.camp = robot_status.robot_id;
	container_set(TAG_JUDGE_DATA,&modesw_set_judge_data_container,sizeof(modesw_set_judge_data_container),CONTAINER_TYPE_STRUCT);
}

//决定使用什么源头的键鼠数据 ： rc or vtm ?
void decide_to_use_Witch_KbData(void)
{
	//如果遥控器离线，图传链路在线，使用图传链路键鼠数据
	if(status.remote && modesw_get_keyboard_data_container.online)
	{
		rc.mouse.l = modesw_get_keyboard_data_container.mouse_data.mouse_l;
		rc.mouse.r = modesw_get_keyboard_data_container.mouse_data.mouse_r;
		rc.mouse.x = modesw_get_keyboard_data_container.mouse_data.mouse_x;
		rc.mouse.y = modesw_get_keyboard_data_container.mouse_data.mouse_y;
		rc.mouse.z = modesw_get_keyboard_data_container.mouse_data.mouse_z;
		rc.kb.key_code = modesw_get_keyboard_data_container.key_code;
	}
	else if(status.remote && !modesw_get_keyboard_data_container.online) //都不在线 
	{
		//无法解锁
		lock_flag = 0;
	}
	else //（遥控器在线 && 图传链路离线） || 都在线
	{
		//直接使用rc数据，那就返回
		return;
	}
}

void mode_switch_task(void const *argu)
{
    ctrl_mode = PROTECT_MODE;
    lock_flag = 0;
    container_bus_init(mb_callback, sizeof(mb_callback)/sizeof(ContainerBusCfg));

    robot_logic_init();
    for (;;) {
		remote_reset();
        if (!lock_flag) {
            if (game_status.game_progress == 4) {//比赛中直接解锁
                lock_flag = 1;
            } else {
                unlock_init();  //解锁操作
            }
        }
        else {
            remote_reset();
			sw1_mode_handler();
        }
		if(status.dji_motor)
		{
			lock_flag = 0;
			ctrl_mode = PROTECT_MODE;
		}
		//决定键鼠数据来源
		decide_to_use_Witch_KbData();
		//运行 FSM 大脑 解锁后激活 否则一直保护
        robot_logic_update((lock_flag == 1) ? (const RC_Ctrl_t*)&rc : NULL);
		//遥控数据打包发送
		modesw_set_container();
		
        status.task.mode_switch = 1;
        osDelay(10);
    }
}
