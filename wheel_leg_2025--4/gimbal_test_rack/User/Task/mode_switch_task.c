#include "mode_switch_task.h"
#include "prot_dr16.h"
#include "prot_judge.h"
#include "cmsis_os.h"
#include "status_task.h"
#include "board_comm.h"
uint8_t lock_flag = 0;
uint8_t reset_flag = 0;
ctrl_mode_e ctrl_mode;

static void unlock_init(void) {
    if (rc.sw1 == RC_UP && rc.sw2 == RC_UP ) { //左拨杆置上，右拨杆置上
        if (rc.ch4 < -600 && rc.ch3 > 600) {
            lock_flag = 1;  //左控制杆拨至右下
        }
    }
}

//获取底盘发上来的遥控数据 此处的遥控器数据已经经过处理，无需判断数据源
static void rc_data_get(void)
{
	rc.ch1 = rc_data_rec.ch1;
	rc.ch2 = rc_data_rec.ch2;
	rc.init_status = rc_data_rec.rc_init_status;
	rc.sw1 = rc_data_rec.sw1;
	rc.sw2 = rc_data_rec.sw2;
	
	ctrl_mode = rc_data_rec.ctrl_mode;
	
	rc.mouse.l = kb_data_rec.l;
	rc.mouse.r = kb_data_rec.r;
	rc.mouse.x = kb_data_rec.x;
	rc.mouse.y = kb_data_rec.y;
	rc.mouse.z = kb_data_rec.z;
	rc.kb.key_code = kb_data_rec.key_code;
}

static void sw1_mode_handler(void) { //由拨杆1决定系统模式切换，主要是云台、底盘和发射器
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
}

static void remote_reset(void)
{
    //保护模式下右拨杆拨至右上
     if (rc.sw1 == RC_UP && rc.sw2 == RC_UP && rc.ch1 == -660 && rc.ch2 == -660) {
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }
}

void mode_switch_task(void const *argu)
{
    ctrl_mode = PROTECT_MODE;
    lock_flag = 0;
    for (;;) {
		
//		rc_data_get();	//1111测试架不需要
		
		remote_reset();
		
//        if (!lock_flag) {
//            if (game_status.game_progress == 4) {//比赛中直接解锁
//                lock_flag = 1;
//            } else {
//                unlock_init();  //解锁操作
//            }
//        }
//        else {
//            remote_reset();
            sw1_mode_handler();  //根据左拨杆切换系统模式 1111测试架解开注释了
//        }
        status.task.mode_switch = 1;
        osDelay(10);
    }
}
