#include "leg_vmc.h"
#include "math_matrix.h"
#include "arm_math.h"

vmc_t vmc[2];

/*
 * @brief     vmc解算初始化
 * @param[in] v : vmc实例
 * @param[in] legLength: 大小腿长度
 * @retval    void
 */
void vmc_init(vmc_t* v, const float legLength[2])
{
    for(int i = 1; i < 3; i++)
        v->L[i] = legLength[i-1];
    //写出髋部中点
    v->mp_fdb.xm = 0;
    v->mp_fdb.ym = 0;
    v->mp_ref.xm = 0;
    v->mp_ref.ym = 0;
    v->L_M[1] = 0.09675f;
    v->L_M[2] = 0.1161f;
    v->L_M[3] = 0.1161f;
    v->L_M[4] = 0.09675f;
}

/*
 * @brief     vmc正解算
 * @param[in] v : vmc实例
 * @param[in] q1 q2: 实际杆角度      解算得到虚拟杆长度和角度 q1大腿 q2小腿连杆
 * @param[in] w1 w2: 实际杆角速度    解算得到虚拟杆伸长速度和角速度
 * @param[in] t1 t2: 实际杆力矩      解算得到虚拟杆转动力矩和支持力
 * @retval    void
 */
void vmc_forward_solution(vmc_t* v, float q1, float q2, float w1, float w2, float t1, float t2)
{
    //------------------------位置运动学正解算------------------------//
    //反馈关节角度
    v->q_fdb[1] = q1;
//    v->q_fdb[2] = q2;
//    
	//通过大小腿电机实际得到角度 解算得到等效大小腿模型 小腿关节角
	v->mp_fdb_mid.xb = v->L_M[1] * arm_cos_f32(q2);
    v->mp_fdb_mid.yb = v->L_M[1] * arm_sin_f32(q2);
    v->mp_fdb_mid.xd = v->L_M[4] * arm_cos_f32(q1);
    v->mp_fdb_mid.yd = v->L_M[4] * arm_sin_f32(q1);
    //计算解算所需中间系数
    v->mp_fdb_mid.Lbd = sqrtf(powf(v->mp_fdb_mid.xd - v->mp_fdb_mid.xb, 2) + powf(v->mp_fdb_mid.yd - v->mp_fdb_mid.yb, 2));
    v->mp_fdb_mid.A0 = 2 * v->L_M[2] * (v->mp_fdb_mid.xd - v->mp_fdb_mid.xb);
    v->mp_fdb_mid.B0 = 2 * v->L_M[2] * (v->mp_fdb_mid.yd - v->mp_fdb_mid.yb);
    v->mp_fdb_mid.C0 = powf(v->L_M[2], 2) + powf(v->mp_fdb_mid.Lbd, 2) - powf(v->L_M[3], 2);
    //解算左膝关节角
    v->q_fdb_mid[2] = 2 * atan2f(v->mp_fdb_mid.B0 + sqrtf(powf(v->mp_fdb_mid.A0, 2) + powf(v->mp_fdb_mid.B0, 2) \
                    - powf(v->mp_fdb_mid.C0, 2)), v->mp_fdb_mid.A0 + v->mp_fdb_mid.C0);
    //得到足端坐标
    v->mp_fdb_mid.xc = v->mp_fdb_mid.xb + v->L_M[2] * arm_cos_f32(v->q_fdb_mid[2]);
    v->mp_fdb_mid.yc = v->mp_fdb_mid.yb + v->L_M[2] * arm_sin_f32(v->q_fdb_mid[2]);
    //解算右膝关节角
    v->q_fdb_mid[3] = atan2f(v->mp_fdb_mid.yc - v->mp_fdb_mid.yd, v->mp_fdb_mid.xc - v->mp_fdb_mid.xd);
    if(v->q_fdb_mid[3] < 0)
    v->q_fdb_mid[3]+=2*PI;

	v->q_fdb[2] = v->q_fdb_mid[3];  //-10.0f 为机械固定偏差
     
     
     
	
    //得到足端坐标
    v->mp_fdb.xc = v->L[1] * arm_cos_f32(v->q_fdb[1]) + v->L[2] * arm_cos_f32(v->q_fdb[2]);
    v->mp_fdb.yc = v->L[1] * arm_sin_f32(v->q_fdb[1]) + v->L[2] * arm_sin_f32(v->q_fdb[2]);
    
    //象限判断
    if (v->mp_fdb.xc > 0.0f && v->mp_fdb.yc > 0.0f)
        v->quadrant = 1;
    else if (v->mp_fdb.xc < 0.0f && v->mp_fdb.yc > 0.0f)
        v->quadrant = 2;
    else if (v->mp_fdb.xc < 0.0f && v->mp_fdb.yc < 0.0f)
        v->quadrant = 3;
    else if (v->mp_fdb.xc > 0.0f && v->mp_fdb.yc < 0.0f)
        v->quadrant = 4;
    else v->quadrant = 0;

    //计算虚拟腿连杆角度与长度 (此时拟腿连杆角度是相对于机体而言)
    v->L_fdb = sqrtf(powf(v->mp_fdb.xc - v->mp_fdb.xm, 2) + powf(v->mp_fdb.yc - v->mp_fdb.ym, 2));
    v->q_fdb[0] = atan2f(v->mp_fdb.yc - v->mp_fdb.ym, v->mp_fdb.xc - v->mp_fdb.xm);
        
    if (v->q_fdb[0] < 0) //0~360
        v->q_fdb[0]+=2*PI;
        
    //------------------------速度运动学正解算 ------------------------//
    //反馈关节角速度
    v->W_fdb.e.w1_fdb = w1;
//    v->W_fdb.e.w2_fdb = w2*0.851f;
		 v->W_fdb.e.w2_fdb = w2;//7.7
    //写出角速度正解算矩阵 [vw0 vy0] = Jwv [w1 w2]
//    v->Jwv.e.x11 = (-v->L[1] * arm_sin_f32(v->q_fdb[1]) * arm_sin_f32(v->q_fdb[0]) \
//                    +v->L[2] * arm_sin_f32(v->q_fdb[2]) * arm_cos_f32(v->q_fdb[0]));
//    v->Jwv.e.x12 = (-v->L[1] * arm_sin_f32(v->q_fdb[1]) * arm_cos_f32(v->q_fdb[0]) \
//                    -v->L[2] * arm_sin_f32(v->q_fdb[2]) * arm_sin_f32(v->q_fdb[0]));
//    v->Jwv.e.x21 = ( v->L[1] * arm_cos_f32(v->q_fdb[1]) * arm_sin_f32(v->q_fdb[0]) \
//                    -v->L[2] * arm_cos_f32(v->q_fdb[2]) * arm_cos_f32(v->q_fdb[0]));
//    v->Jwv.e.x22 = ( v->L[1] * arm_cos_f32(v->q_fdb[1]) * arm_cos_f32(v->q_fdb[0]) \
//                    +v->L[2] * arm_cos_f32(v->q_fdb[2]) * arm_sin_f32(v->q_fdb[0]));
//      v->Jwv.e.x11 = (-v->L[1] * arm_sin_f32(v->q_fdb[1]));
//      v->Jwv.e.x12 = (-v->L[2] * arm_sin_f32(v->q_fdb[2]));
//      v->Jwv.e.x21 = ( v->L[1] * arm_cos_f32(v->q_fdb[1]));
//      v->Jwv.e.x22 = ( v->L[2] * arm_cos_f32(v->q_fdb[2]));
      v->Jwv.e.x11 = (+v->L[1] * arm_sin_f32(v->q_fdb[1]) * arm_sin_f32(v->q_fdb[0]) \
                      +v->L[1] * arm_cos_f32(v->q_fdb[1]) * arm_cos_f32(v->q_fdb[0]));
      v->Jwv.e.x12 = (+v->L[2] * arm_sin_f32(v->q_fdb[2]) * arm_sin_f32(v->q_fdb[0]) \
                      +v->L[2] * arm_cos_f32(v->q_fdb[2]) * arm_cos_f32(v->q_fdb[0]));
      v->Jwv.e.x21 = (-v->L[1] * arm_sin_f32(v->q_fdb[1]) * arm_cos_f32(v->q_fdb[0]) \
                      +v->L[1] * arm_cos_f32(v->q_fdb[1]) * arm_sin_f32(v->q_fdb[0]));
      v->Jwv.e.x22 = (-v->L[2] * arm_sin_f32(v->q_fdb[2]) * arm_cos_f32(v->q_fdb[0]) \
                      +v->L[2] * arm_cos_f32(v->q_fdb[2]) * arm_sin_f32(v->q_fdb[0]));
    
    //速度正解算
    aMartix_Mul(v->Jwv.array, v->W_fdb.array, v->V_fdb.array, 2, 2, 1);
    v->V_fdb.e.vw0_fdb *= 1.0f / v->L_fdb; //线速度转换为角速度
    
    //腿长二阶导数(差分)解算
    v->Acc_fdb.L0_ddot = (v->V_fdb.e.vy0_fdb - v->Acc_fdb.vy0_fdb_last) / (2 * 0.001f);
    v->Acc_fdb.vy0_fdb_last = v->V_fdb.e.vy0_fdb;
    
    //------------------------动力学正解算------------------------//
    //反馈关节力矩
    v->T_fdb.e.T1_fdb = t1;
    v->T_fdb.e.T2_fdb = t2;
    
    //写出力的正解算矩阵 [T0 Fy] = Jtf-1 [T1 T2]
    v->Jtf.e.x11 = v->L_fdb * arm_sin_f32(v->q_fdb[0] - v->q_fdb[2]) \
                    / (v->L[1] * arm_sin_f32(v->q_fdb[1] - v->q_fdb[2]));
    v->Jtf.e.x12 =-v->L_fdb * arm_sin_f32(v->q_fdb[0] - v->q_fdb[1]) \
                    / (v->L[2] * arm_sin_f32(v->q_fdb[1] - v->q_fdb[2]));
    v->Jtf.e.x21 =-arm_cos_f32(v->q_fdb[0] - v->q_fdb[2]) \
                    / (v->L[1] * arm_sin_f32(v->q_fdb[1] - v->q_fdb[2]));
    v->Jtf.e.x22 = arm_cos_f32(v->q_fdb[0] - v->q_fdb[1]) \
                    / (v->L[2] * arm_sin_f32(v->q_fdb[1] - v->q_fdb[2]));
           
                    
    //力正解算
    aMartix_Mul(v->Jtf.array, v->T_fdb.array, v->F_fdb.array, 2, 2, 1);
}

/*
 * @brief     vmc逆解算
 * @param[in] v : vmc实例
 * @param[in] L_ref q0_ref: 虚拟杆长度和角度实际杆角度   解算得到实际杆角度
 * @param[in] T0 Fy:        虚拟杆转动力矩和支持力       解算得到实际杆力矩
 * @retval    void
 */
void vmc_inverse_solution(vmc_t* v, float L_ref, float q0_ref, float T0, float Fy)
{
    //------------------------位置运动学逆解算------------------------//
    //目标关节角度
    v->L_ref = L_ref;
    v->q_ref[0] = q0_ref;
    //得到足端坐标
    v->mp_ref.xc = v->mp_ref.xm + v->L_ref * arm_cos_f32(v->q_ref[0]);
    v->mp_ref.yc = v->mp_ref.ym + v->L_ref * arm_sin_f32(v->q_ref[0]);
    
    //q12为L1延长线与L2外部夹角
    //[0 pi]
    v->mp_ref.q12 = acosf((powf(v->mp_ref.xc, 2) + powf(v->mp_ref.yc, 2) - powf(v->L[1], 2)\
                    - powf(v->L[2], 2)) / (2 * v->L[1] * v->L[2]));

    //计算解算所需中间系数 得到大腿与水平面角度
    v->mp_ref.tp1 = v->L[2] * arm_sin_f32(v->mp_ref.q12);
    v->mp_ref.tp2 = v->L[2] * arm_cos_f32(v->mp_ref.q12) + v->L[1];
    v->mp_ref.tp3 = sqrtf(powf(v->mp_ref.tp1, 2) + powf(v->mp_ref.tp2, 2));   
    v->mp_ref.q01 = -acosf(v->mp_ref.tp2 / v->mp_ref.tp3) + acosf(v->mp_ref.xc / v->mp_ref.tp3);
    
	//得到目标各角度
    v->q_ref[1] = v->mp_ref.q01;
    v->q_ref[2] = v->mp_ref.q01 + v->mp_ref.q12;

    //------------------------动力学逆解算------------------------//
    //目标关节力矩
    v->F_ref.e.T0_ref = T0;
    v->F_ref.e.Fy_ref = Fy;
    
    //写出力的正解算矩阵 [T1 T2] = Jft [T0 Fy]
    v->Jft.e.x11 = v->L[1] * arm_cos_f32(v->q_fdb[0] - v->q_fdb[1]) / v->L_fdb;
    v->Jft.e.x12 = v->L[1] * arm_sin_f32(v->q_fdb[0] - v->q_fdb[1]);
    v->Jft.e.x21 = v->L[2] * arm_cos_f32(v->q_fdb[0] - v->q_fdb[2]) / v->L_fdb;
    v->Jft.e.x22 = v->L[2] * arm_sin_f32(v->q_fdb[0] - v->q_fdb[2]);
    
    
    //力逆解算
    aMartix_Mul(v->Jft.array, v->F_ref.array, v->T_ref.array, 2, 2, 1); 
}
