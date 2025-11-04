#ifndef __WHEEL_LEG_ROBOT_H
#define __WHEEL_LEG_ROBOT_H

#include "stdint.h"
#include "pid.h"
#include "kalman_filter.h"

typedef struct
{
    float X_ref[10];
    float X_fdb[10];
    float X_diff[10];
    float U_ref[4];
    float K[4 * 10];
    //leg_w为腿摆角角速度 用来差分计算dot_leg_w腿摆角加速度
    float dot_leg_w[2], last_leg_w[2];
} lqr_t;

//全身运动控制
typedef struct
{
    //起身目标长度
    float recover_length;
    //宏观控制数据 其它数据像腿摆角、机体倾角默认为0
    float s_ref, v_ref, yaw_ref, wz_ref;
    float high_set;
    //反馈数据
    float s_fdb, v_fdb, yaw_fdb, wz_fdb;
    float roll_fdb, pit_fdb, wy_fdb, az_fdb ,front_dis_fdb;
    //中间变量
    float yaw_err, s_adapt;
    //补偿
    float K_adapt;//状态预测补偿系数
    float roll_offs, inertial_offs;//roll补偿 惯性力补偿
    float yaw_offset;//小陀螺偏置
    //期望限制系数
    float K_ref[2];
    //控制标志
    uint8_t jump_flag, jump_pre, high_flag, power_flag, prone_flag, ctrl_mode, quarand_, crash_flag ,joint_all_online;
	/*	jump_flag 	=1开始上台阶 =2和=3成功上台阶		
		jump_pre 	上台阶膝关节朝后完成标志位		
		high_flag 	=0短腿 =1中腿 =2长腿
		power_flag  未使用		
		prone_flag  =0保护 =1趴下				
		ctrl_mode 	=0 保护模式 =1位控 =2力控
		quarand_ 	未使用			
		crash_flag  =1两条腿磕到台阶
		joint_all_online = 1 所有关节电机都在线
		*/
		
	uint16_t jump_cnt, jump_run;
		//jump_cnt 用于软件延时变化腿长		jump_run 用于软件延时判断两条腿是否撞击台阶
	
	uint16_t sky_flag, sky_cnt, sky_over, jump2_over;
	/*	sky_flag 	=1在地面收腿 =2伸腿 =3在空中收腿 =4伸腿准备落地缓冲
		sky_cnt		跳跃计数变量
		sky_over 	=1完成跳跃
		jump2_over  =1机体已磕上第二级台阶 */
	
	int16_t s_wait;//定点wait
		
    //单侧控制参数
    struct
    {
        //接收数据
        float q1, q2;
        float q21;
        float temp;
        float w1, w2;
        float t1, t2;
        float qy, wy;
        //发送数据
        float T1, T2, Tw;
        float P1, P2;
        //中间数据
        uint8_t  fly_flag;
        uint8_t  fly_adapt;
        int16_t  fly_cnt;
        uint16_t fly_flag_cnt;
        float 	 Fn_kal, Fn_fdb;
        float 	 T0, Fy;
        //状态预测
        float predict_wy, T_adapt;
    } side[2];
} wlr_t;

extern wlr_t wlr;
extern lqr_t lqr;
extern pid_t pid_leg_length[2];
extern pid_t pid_leg_length_fast[2];
extern pid_t pid_roll;
extern pid_t pid_rescue[2];
extern int32_t double_cnt;

void wlr_init(void);
void wlr_protest(void);
void wlr_control(void);

#endif

