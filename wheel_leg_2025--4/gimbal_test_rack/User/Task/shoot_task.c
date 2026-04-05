#include "shoot_task.h"
#include "mode_switch_task.h"
#include "control_def.h"
#include "drv_dji_motor.h"
#include "prot_judge.h"
#include "prot_dr16.h"
#include "prot_vision.h"
#include "data_buffer.h"
#include "cmsis_os.h"
#include "status_task.h"
#include "math_lib.h"
#include "bsp_LK_Motor_MG4005.h"
#include "drv_dm_motor.h"


#define SHOOT_SPEED_NUM 15	//没用上
//*******射击拨盘有关变量，比2025多添加了新电机的变量******//
static uint16_t frequency_cnt = 0;	//射击周期计算
static uint8_t  shoot_enable  = 1;  //单发使能标志
static float trigger_ecd_error;

//用于卡弹反转
static uint32_t back_cnt = 0;
static uint32_t err_cnt  = 0;
static uint8_t back_flag = 0;

float MIN_HEAT = 40;        //热量控制裕量
shoot_t shoot;

//static buffer_t *shoot_speed_buffer;
//交叉耦合
float velocity_err;
float K_c;
float feed_torque;

//测试
float rpm_test = 0.0f;

//新添加
float ref_motor_revolutions = 0;		//目标圈数

uint16_t prev_encoder = 0;				//上一次编码器值，用于计算圈数
float prev_encoder_2325 = 0;
float total_motor_revolutions = 0.0f; 	// 用于记录电机总圈数（带小数），一圈就是一颗子弹，实际圈数
float real_motor_revolutions = 0.0f;	//输出轴圈数,暂时没有用上

//微电流预制
uint8_t microcurrent_flag = 0;
float micro_t = 0;

//**********选择拨盘电机**********//
//#define DJI2006
#define MG4005
//#define DM2325	烂
//#define DJI3508

//**********定义正侧供，减速比不一样**********//
#define SIDE_SUPPLY
//#define CENTRAL_SUPPLY	//中供
//**********宏定义部分**********//
#ifdef DJI2006

//#define ABS(x) ((x>0)? (x): (-(x)))
//#define TRIGGER_MOTOR_ECD_SINGLE   (68027.0f)  
//#define TRIGGER_MOTOR_ECD_SERIES   (68027.0f)  
#define TRIGGER_MOTOR_ECD_SINGLE   (58975.0f)  //拨盘一颗子弹转过的编码值 8191 * 36 *2/10 = 58975.2f
#define TRIGGER_MOTOR_ECD_SERIES   (58975.0f)  //拨盘一颗子弹转过的编码值 8191 * 36 *2/10 = 58975.2f
static int motor_type = MOTOR_DJI2006;
#endif

#ifdef MG4005
static int motor_type = MOTOR_MG4005;

		#ifdef SIDE_SUPPLY	//侧供
		
		#define	TRIGGER_MOTOR_ECD_SINGLE 1
		#define TRIGGER_MOTOR_ECD_SERIES 1	//子弹发射，侧供子弹，因为电机与齿轮减速比为1：10，而一圈输出轴为10颗
		#define SHOOT_TRIGGER_PERIOD 35		//射频，32ms   为31.25hz，若33.333ms难以计算			测试30ms即33hz第一发偶尔会卡
		 
		#endif
		
		#ifdef CENTRAL_SUPPLY	//中供

		#define	TRIGGER_MOTOR_ECD_SINGLE -1.6f
		#define TRIGGER_MOTOR_ECD_SERIES -1.6f	//因为多了个减速齿轮所以反转
		#define SHOOT_TRIGGER_PERIOD 	38//射频，40ms  25hz 最高为31.25hz，但是目前速度我限幅了15000，26.04hz

		#endif

#define ENCODER_MAX 65535.0f    // 编码器最大值,用于计算圈数
#define REDUCTION_RATIO 10.0f   // 减速比

//#define TRIGGER_ECD    65535 //拨盘一颗子弹转过的编码值 65535f暂时没有用到			104,856f这是中供的因为有1.6减速比
#endif
#ifdef DM2325
static int motor_type =  MOTOR_DM2325;

	#define TRIGGER_MOTOR_ECD_SERIES 5	
	#define TRIGGER_MOTOR_ECD_SINGLE 5
	#define SHOOT_TRIGGER_PERIOD 40		

	#define ENCODER_MAX (2*PI)  // 编码器最大值	标记
	#define REDUCTION_RATIO 25.0f   // 减速比
#endif

#ifdef DJI3508

#define TRIGGER_MOTOR_ECD_SINGLE   (8192.0f)  
#define TRIGGER_MOTOR_ECD_SERIES   (8192.0f)  
static int motor_type = MOTOR_DJI3508;


#endif
//***********函数部分**********//

//记录电机的圈数
void update_position(void)
{
	#ifdef MG4005
		uint16_t current_encoder = trigger_motor_MG4005.raw_encoder;
	    // 计算差值，考虑溢出
		int16_t delta = (int16_t)(current_encoder - prev_encoder);  
		// 如果差值过大，说明发生了溢出
		// 例如从65530走到10，正常差值是10-65530 = -65520，这是一个巨大的负变化
		if(delta > ENCODER_MAX / 2) {
			delta -= ENCODER_MAX; // 正溢出，应减去一圈
		} else if(delta < -ENCODER_MAX / 2) {
			delta += ENCODER_MAX; // 负溢出，应加上一圈
		}
		// 更新电机总圈数 (delta / ENCODER_MAX 是电机转动的圈数)
		total_motor_revolutions += (float)delta / ENCODER_MAX;
		// 更新输出轴圈数
		real_motor_revolutions = total_motor_revolutions / REDUCTION_RATIO;
		prev_encoder = current_encoder;
	#endif
		
	#ifdef DM2325
		float current_encoder = trigger_motor_2325.position;
		float delta = (float)(current_encoder - prev_encoder_2325);  
		static uint8_t rec_cnt = 0;
		static float p_offset = 0.0f;
//		static float total_p = 0.0f;
//		static float caccaa = 0.0f;
//		fuck = delta;
//		if(ABS(delta)>0.5)
//		{
		if(rec_cnt < 50)
		{
			rec_cnt++;
			p_offset = current_encoder;
		}
		else{
				if(delta > ENCODER_MAX / 2) {
						delta -= ENCODER_MAX; // 正溢出，应减去一圈
				} else if(delta < -ENCODER_MAX / 2) {
						delta += ENCODER_MAX; // 负溢出，应加上一圈
				}
//				caccaa = delta / (ENCODER_MAX);
//    total_motor_revolutions += caccaa;
			total_motor_revolutions	+= delta / (ENCODER_MAX);
			real_motor_revolutions = total_motor_revolutions / 25.0f;
		}
		prev_encoder_2325 = current_encoder;
	#endif
}


static void pre_fabricated_trigger_position(void)
{
	static uint8_t recover_flag = 0;
	static uint16_t init_cnt = 0;
	if(init_cnt < 1000)
	{	
		init_cnt++;
		if(recover_flag == 0)
		{ 
#ifdef MG4005
			shoot.trigger_ecd.ref = 54500.0f/65535.0f;			//38400	50%的连发几率			//改这里改变预制的位置，通过读编码值

#endif
//
#ifdef DJI3508
//			trigger_motor.total_ecd = trigger_motor.ecd;
			shoot.trigger_ecd.ref = 4717.0f;	//5976
#endif
			recover_flag++; 
		}
	}
}

//2006微电流预制弹位，要加上3508代码电机微电流预制弹位
static void microcurrent_pre_fabricated_firing_position(void)
{
	static uint16_t microcurrent_cnt = 5;	
	static uint16_t microcurrent_wait_cnt = 100;	
	
		
		if(ABS(trigger_motor.rx_current) > 700 && ABS(trigger_motor.speed_rpm) < 50 && microcurrent_flag == 0)
		{
			micro_t = 0.0f;
			if(microcurrent_cnt>0)
				microcurrent_cnt--;
			else 
				microcurrent_flag = 1;
				
		}
		else if(microcurrent_flag == 1 && microcurrent_wait_cnt > 0)
		{
			micro_t = 0.0f;
			microcurrent_wait_cnt--;	
		}else if(microcurrent_flag == 1 && microcurrent_wait_cnt == 0){
			microcurrent_flag = 2;		
		}
		else if(microcurrent_flag == 0)
		{
			if(ABS(trigger_motor.speed_rpm) < 300)	
				micro_t = 0.15f;
			else
				micro_t = 0.05f;
		}
		
		
		if(microcurrent_flag == 2)
		{
			shoot.trigger_ecd.ref -= 7000.0f;	
			microcurrent_flag = 3;
			microcurrent_wait_cnt++;
		}else if(microcurrent_flag == 3 && microcurrent_wait_cnt>=100)
		{
			microcurrent_flag = 4;		
		}
}

//**********4005和2325多的函数定义结束**********//


static uint8_t single_shoot_reset(void)
{
    return (
        (rc.mouse.l == 0 && ctrl_mode == KEYBOARD_MODE)
        || (ABS(rc.ch4) < 10 && ctrl_mode == REMOTER_MODE)
    );
}

static uint8_t single_shoot_enable(void)
{
    return (
        shoot_enable
//1111测试架注释热量        && shoot.barrel.heat_remain >= MIN_HEAT
        && ((rc.mouse.l && ctrl_mode == KEYBOARD_MODE) || (rc.ch4 > 500 && ctrl_mode == REMOTER_MODE))
        && ABS(trigger_ecd_error) < TRIGGER_MOTOR_ECD_SINGLE
    );
}

static uint8_t series_shoot_enable(void)
{
    return (
        (      (ctrl_mode == REMOTER_MODE && vision.shoot_enable )//&& (rc_fsm_check(RC_LEFT_LD) || rc_fsm_check(RC_RIGHT_RD))) //开启视觉连发
            || (ctrl_mode == REMOTER_MODE && rc.sw2 == RC_DN && rc_fsm_check(RC_LEFT_LD) && !rc_fsm_check(RC_RIGHT_RD)) //开启遥控连发
            || (ctrl_mode == KEYBOARD_MODE && rc.mouse.l && rc.mouse.r && vision.shoot_enable)	 
            || (ctrl_mode == KEYBOARD_MODE && rc.mouse.l && rc.mouse.r == 0)
        )
//1111测试架注释热量        && shoot.barrel.heat_remain >= MIN_HEAT  //热量控制   
        && frequency_cnt * SHOOT_PERIOD >= shoot.trigger_period  //射频控制
        && ABS(trigger_ecd_error) < 1.0f * TRIGGER_MOTOR_ECD_SERIES  //拨盘误差控制
    );
}

static void shoot_control(void)
{
    switch (shoot.trigger_mode) {
        case TRIGGER_MODE_PROTECT: { //拨盘保护模式，保持惯性，无力
            frequency_cnt = 0; //计时变量置0，打出当前一发，禁止
            shoot.barrel.shoot_period = 0;
			
			if(motor_type == MOTOR_DJI2006||MOTOR_DJI3508)
				shoot.trigger_ecd.ref = trigger_motor.total_ecd;
			else
				shoot.trigger_ecd.ref = total_motor_revolutions;           
				shoot.trigger_spd.pid.i_out = 0;		
				shoot.trigger_output = 0;
            break;
        }
        case TRIGGER_MODE_STOP: { //拨盘停止模式，保持静止，有力
            frequency_cnt = 0; //计时变量置0
            shoot.barrel.shoot_period = 0;   
			if(motor_type == MOTOR_DJI2006)
			{
				microcurrent_pre_fabricated_firing_position();
				
				if(	microcurrent_flag != 3)
					shoot.trigger_ecd.ref = trigger_motor.total_ecd;
				
			}else if(motor_type == MOTOR_DJI3508){
				shoot.trigger_ecd.ref = trigger_motor.total_ecd;
			}
			else{
				shoot.trigger_ecd.ref = total_motor_revolutions;//4005
			}

			
            shoot.trigger_spd.pid.i_out = 0;
            break;
        }
        case TRIGGER_MODE_SINGLE: { //拨盘单发模式，连续开枪请求，只响应一次
						
#ifdef MG4005
			pre_fabricated_trigger_position();
			update_position();
#endif
#ifdef DM2325
			update_position();
	
#endif
#ifdef DJI3508
			pre_fabricated_trigger_position();
#endif
            frequency_cnt++;
            trigger_ecd_error = shoot.trigger_ecd.ref - shoot.trigger_ecd.fdb;
            if (single_shoot_reset()) {
                shoot_enable = 1; 
            }
            if (single_shoot_enable()) { //热量控制
                shoot_enable = 0;
				
				if(motor_type == MOTOR_DM2325 ||motor_type == MOTOR_DJI3508)
					shoot.trigger_ecd.ref -= TRIGGER_MOTOR_ECD_SERIES;//拨一颗子弹2325
				else
					shoot.trigger_ecd.ref += TRIGGER_MOTOR_ECD_SERIES;//拨一颗子弹4005和2006
	
				shoot.barrel.heat += 10;
            }
            break;
        }
        case TRIGGER_MODE_SERIES: { //拨盘连发模式，连续开枪请求，连续响应	
#ifdef MG4005
			pre_fabricated_trigger_position();
			update_position();
#endif		
#ifdef DM2325
			update_position();
	
#endif
#ifdef DJI3508
			pre_fabricated_trigger_position();
#endif
            frequency_cnt++;
            trigger_ecd_error = shoot.trigger_ecd.ref - shoot.trigger_ecd.fdb;
            if (series_shoot_enable()) { //一个周期打一颗
                frequency_cnt = 0;
				if(motor_type == MOTOR_DM2325 ||motor_type == MOTOR_DJI3508)
					shoot.trigger_ecd.ref -= TRIGGER_MOTOR_ECD_SERIES;//拨一颗子弹2325和3508 
				else
					shoot.trigger_ecd.ref += TRIGGER_MOTOR_ECD_SERIES;//拨一颗子弹4005和2006
				
				shoot.barrel.heat += 10;
            }
			
//卡弹反转	
#ifdef DJI2006				
			if(ABS(trigger_motor.rx_current) > 7000 && ABS(trigger_motor.speed_rpm) < 100)
				back_cnt ++;
			if (back_cnt > 200){
				back_flag = 1;
				err_cnt ++;
				shoot.trigger_ecd.ref = trigger_motor.total_ecd + TRIGGER_MOTOR_ECD_SERIES;
				if (err_cnt > 200) {
					back_cnt = 0;
					err_cnt = 0;
					back_flag = 0;
				}
			}
#endif	

#ifdef MG4005
			if(ABS(trigger_motor_MG4005.iq) > 700 && ABS(trigger_motor_MG4005.speed) < 1000)//950
					back_cnt ++;
			if (back_cnt > 100 || back_flag) { 
					back_flag = 1;
					err_cnt ++;
					ref_motor_revolutions = total_motor_revolutions - TRIGGER_MOTOR_ECD_SERIES;
					shoot.trigger_spd.ref = ref_motor_revolutions;//11111
					if (err_cnt > 20) {
							back_cnt = 0;
							err_cnt = 0;
							back_flag = 0;
					}					
			}
#endif

#ifdef DM2325
			if(ABS(trigger_motor_2325.torque) > 4.5 && ABS(trigger_motor_2325.velocity) < 10)
				back_cnt ++;
//						else
//								back_cnt = 0;
			if (back_cnt > 100 || back_flag) {
				back_flag = 1;
				err_cnt ++;
				ref_motor_revolutions = total_motor_revolutions - TRIGGER_MOTOR_ECD_SERIES;
				if (err_cnt > 20) {
					back_cnt = 0;
					err_cnt = 0;
					back_flag = 0;
				}				
			}
#endif
			
#ifdef DJI3508
			
			if(ABS(trigger_motor.rx_current) > 7000 && ABS(trigger_motor.speed_rpm) < 100)
				back_cnt ++;
			if (back_cnt > 100){ 
				back_flag = 1;
				err_cnt ++;
				shoot.trigger_ecd.ref = trigger_motor.total_ecd + TRIGGER_MOTOR_ECD_SERIES;
				if (err_cnt > 50) {
					back_cnt = 0;
					err_cnt = 0;
					back_flag = 0;
				}
			}
			
#endif
			
            break;
        }
        default:break;
    }
	//摩擦轮
    switch (shoot.fric_mode) {
        case FRIC_MODE_PROTECT:
        case FRIC_MODE_STOP: {
            shoot.fric_spd[0].ref = 0;
            shoot.fric_spd[1].ref = 0;
            break;
        }
        case FRIC_MODE_RUN: {
            shoot.fric_spd[0].ref = -shoot.fric_speed_set;
            shoot.fric_spd[1].ref = shoot.fric_speed_set;
            break;
        }
        default:break;
    }
}
  
static void shoot_init(void)
{
    memset(&shoot, 0, sizeof(shoot_t));
    //发射器底层初始化
    pid_init(&shoot.fric_spd[0].pid, NONE, 0.00045f, 0, 0, 0, 0.8);
    pid_init(&shoot.fric_spd[1].pid, NONE, 0.00045f, 0, 0, 0, 0.8);

#ifdef DJI2006 	
	
    pid_init(&shoot.trigger_ecd.pid, NONE, 0.3f, 0, 0.3f, 0, 5000);
    pid_init(&shoot.trigger_spd.pid, NONE, 0.0015f, 0, 0, 0, 1);
#endif

#ifdef MG4005
    pid_init(&shoot.trigger_ecd.pid, NONE, 7500.0f, 0.0f, 0.0f, 10000, 12600);
    pid_init(&shoot.trigger_spd.pid, NONE, 0.18f, 0.00012f, 0, 200, 2048);
#endif	

#ifdef DM2325
    pid_init(&shoot.trigger_ecd.pid, NONE,  8.0f, 0.0f, 0.0f, 10000, 50);
    pid_init(&shoot.trigger_spd.pid, NONE, 250.0f,0.05f, 0.0f, 5000, 5000);
#endif	

#ifdef DJI3508
	pid_init(&shoot.trigger_ecd.pid, NONE, 0.25f, 0.0f, 0.0f, 100, 4000);
    pid_init(&shoot.trigger_spd.pid, NONE, 0.00065f, 0.0000008f, 0, 0.3f, 5.0); 

#endif
    //发射器模式初始化
    shoot.trigger_mode  = TRIGGER_MODE_PROTECT;
    shoot.fric_mode     = FRIC_MODE_PROTECT;
    //枪管参数初始化
    shoot.trigger_period = TRIGGER_PERIOD;
    shoot.fric_speed_set = 650;//650
//	shoot.fric_speed_set = 800;//650
//	  shoot.fric_speed_set = 200;
	
    shoot.barrel.cooling_rate   = 10;
    shoot.barrel.heat_max       = 50;
    //历史射速反馈缓存区
//    shoot_speed_buffer = buffer_create(SHOOT_SPEED_NUM, sizeof(float));
}

static void shoot_pid_calc(void)
{
	
    for (int i = 0; i < 2; i++) {
        shoot.fric_spd[i].fdb = fric_motor[i].velocity;
        shoot.fric_output[i] = pid_calc(&shoot.fric_spd[i].pid, shoot.fric_spd[i].ref, shoot.fric_spd[i].fdb);
//		shoot.fric_output[i] = 0;//1111

    }
		velocity_err = shoot.fric_spd[0].fdb + shoot.fric_spd[1].fdb;
		feed_torque = K_c * velocity_err;
//拨盘的代码		
		
#ifdef DJI2006
    shoot.trigger_ecd.fdb = trigger_motor.total_ecd;
    shoot.trigger_spd.ref = pid_calc(&shoot.trigger_ecd.pid, shoot.trigger_ecd.ref, shoot.trigger_ecd.fdb);
    shoot.trigger_spd.fdb = trigger_motor.speed_rpm;
    shoot.trigger_output = pid_calc(&shoot.trigger_spd.pid, shoot.trigger_spd.ref, shoot.trigger_spd.fdb);
#endif
		
#ifdef MG4005
	shoot.trigger_ecd.fdb = total_motor_revolutions;
    shoot.trigger_spd.ref = pid_calc(&shoot.trigger_ecd.pid, shoot.trigger_ecd.ref, shoot.trigger_ecd.fdb);
    shoot.trigger_spd.fdb = trigger_motor_MG4005.speed;
    shoot.trigger_output = pid_calc(&shoot.trigger_spd.pid, shoot.trigger_spd.ref, shoot.trigger_spd.fdb);
#endif
#ifdef DM2325
	shoot.trigger_ecd.fdb = total_motor_revolutions;
    shoot.trigger_spd.ref = pid_calc(&shoot.trigger_ecd.pid, shoot.trigger_ecd.ref, shoot.trigger_ecd.fdb);
    shoot.trigger_spd.fdb = trigger_motor_2325.velocity;
    shoot.trigger_output = pid_calc(&shoot.trigger_spd.pid, shoot.trigger_spd.ref, shoot.trigger_spd.fdb);

#endif

#ifdef DJI3508
	shoot.trigger_ecd.fdb = trigger_motor.total_ecd;
    shoot.trigger_spd.ref = pid_calc(&shoot.trigger_ecd.pid, shoot.trigger_ecd.ref, shoot.trigger_ecd.fdb);

//    shoot.trigger_spd.ref = rpm_test;
    shoot.trigger_spd.fdb = trigger_motor.speed_rpm;
    shoot.trigger_output = pid_calc(&shoot.trigger_spd.pid, shoot.trigger_spd.ref, shoot.trigger_spd.fdb);
	
#endif
	
}

static void shoot_data_output(void)
{
    if (shoot.fric_mode == FRIC_MODE_PROTECT) {
        dji_motor_set_torque(&fric_motor[0], 0);
        dji_motor_set_torque(&fric_motor[1], 0);
    } else {
        dji_motor_set_torque(&fric_motor[0], shoot.fric_output[0]-feed_torque);
        dji_motor_set_torque(&fric_motor[1], shoot.fric_output[1]-feed_torque);
    }
    if (shoot.trigger_mode == TRIGGER_MODE_PROTECT) {
        dji_motor_set_torque(&trigger_motor, 0);
		trigger_motor_MG4005.pos_out = 0;//4005电机
    } else {
		
		if(motor_type == MOTOR_DJI2006 )//2006有微电流预制
		{
			if(microcurrent_flag == 0||microcurrent_flag == 1)
				dji_motor_set_torque(&trigger_motor, micro_t);
			else//2 3
				dji_motor_set_torque(&trigger_motor, shoot.trigger_output);
		}
		else if(motor_type == MOTOR_DJI3508)
			dji_motor_set_torque(&trigger_motor, shoot.trigger_output);//其他电机比如3508
		else if(motor_type == MOTOR_MG4005)
			trigger_motor_MG4005.pos_out = shoot.trigger_output;//4005电机

    }		
}

static void shoot_param_update(void)
{
    //更新裁判系统数据
    if (robot_status.shooter_barrel_heat_limit != 0) {
        shoot.barrel.heat_max = robot_status.shooter_barrel_heat_limit;//枪管热量上限
        shoot.barrel.cooling_rate = robot_status.shooter_barrel_cooling_value;//枪管冷却速率
    }
    //更新模拟裁判系统数据
    shoot.barrel.heat -= shoot.barrel.cooling_rate * SHOOT_PERIOD * 0.001f;  //当前枪管(理论)热量
    if (shoot.barrel.heat < 0) shoot.barrel.heat = 0;
    shoot.barrel.heat_remain = shoot.barrel.heat_max - shoot.barrel.heat;  //当前枪管(理论)剩余热量
    shoot.barrel.heat_remain = 50;
}

static void shoot_mode_switch(void)
{
    /* 更新裁判系统参数 */
    shoot_param_update();
    /* 模式切换 */
    switch (ctrl_mode) {
        case PROTECT_MODE: {
            shoot.trigger_period = TRIGGER_PERIOD;
            shoot.fric_mode = FRIC_MODE_STOP;
            shoot.trigger_mode = TRIGGER_MODE_PROTECT;
            break;
        }
        case REMOTER_MODE: {
            shoot.trigger_period = TRIGGER_PERIOD;
            /* 摩擦轮和拨盘模式切换 */
            switch (rc.sw2) {
                case RC_UP: {
                    shoot.fric_mode = FRIC_MODE_STOP;
                    shoot.trigger_mode = TRIGGER_MODE_STOP;
                    break;
                }
                case RC_MI: {
                    shoot.fric_mode = FRIC_MODE_RUN;   //正常
                    shoot.trigger_mode = TRIGGER_MODE_SINGLE;                
                    
                    break;
                }
                case RC_DN: {
                    shoot.fric_mode = FRIC_MODE_RUN;
                    shoot.trigger_mode = TRIGGER_MODE_SERIES;                 
                    
                    break;
                }
                default: break;
            }
			if(rc_fsm_check(RC_RIGHT_LU)){	//右摇杆左上关闭发射 
				shoot.fric_mode = FRIC_MODE_STOP;
				shoot.trigger_mode = TRIGGER_MODE_PROTECT;
			}
            break;
        }
        case KEYBOARD_MODE: {
            /* 射频切换 */
            if (rc.mouse.r)
                shoot.trigger_period = TRIGGER_PERIOD2;
            else
                shoot.trigger_period = TRIGGER_PERIOD;
            /* 摩擦轮模式切换 */
            if (robot_status.power_management_shooter_output || 1) {  //发射机构得到供电  展示底盘没发上云台
                shoot.fric_mode = FRIC_MODE_RUN;  //开关摩擦轮         
            } else {
                shoot.fric_mode = FRIC_MODE_PROTECT;  //摩擦轮断电，软件保护，禁用摩擦轮
            }
            /* 视觉模式切换 */
            if (KEY_PRESS_VISION2 ) {
				vision_tx_msg.mode_msg.aiming_status = 2;
//              vision.tx.data.aiming_mode = 2;
            } else if (KEY_PRESS_VISION1 ) {
               vision_tx_msg.mode_msg.aiming_status = 1;
            } else {
                vision_tx_msg.mode_msg.aiming_status = 0;
            }
            /* 拨盘模式切换 */
            if (shoot.fric_mode != FRIC_MODE_RUN) {
                shoot.trigger_mode = TRIGGER_MODE_STOP;
            } else if (vision.tx.data.aiming_mode != 0) {
                shoot.trigger_mode = TRIGGER_MODE_SINGLE;
            } else {
                shoot.trigger_mode = TRIGGER_MODE_SERIES;
            }
            break;
        }
        default: break;
    }
}

void shoot_task(void const *argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    shoot_init();
    for(;;)
    {
        thread_wake_time = osKernelSysTick();
//        taskENTER_CRITICAL();		//本来是没有保护的，现在加了拨盘，把他加回去了
        shoot_mode_switch();    /* 发射器模式切换 */
        shoot_control();
        shoot_pid_calc();
        shoot_data_output();
        status.task.shoot = 1;
//        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 2);
    }
}
