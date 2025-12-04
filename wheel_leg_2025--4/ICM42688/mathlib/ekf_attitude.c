#include "ekf_attitude.h"
#include <math.h>
#include <string.h>

#define MIN_NORM 1e-12f

// 四元数归一化
static void quat_renorm(float* q) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if(norm < MIN_NORM) return;
    norm = 1.0f / norm;
    q[0] *= norm;
    q[1] *= norm;
    q[2] *= norm;
    q[3] *= norm;
}

// 创建单位矩阵
static void mat_identity(arm_matrix_instance_f32* mat, uint16_t size) {
    memset(mat->pData, 0, size*size*sizeof(float));
    for(uint16_t i=0; i<size; i++) {
        mat->pData[i*size + i] = 1.0f;
    }
}

void EKF_Init(EKF_Instance* ekf, float dt) {
    // 初始化状态
    ekf->x[0] = 1.0f;
    ekf->x[1] = ekf->x[2] = ekf->x[3] = 0.0f;
    ekf->x[4] = ekf->x[5] = ekf->x[6] = 0.0f;
    ekf->dt = dt;

    // 初始化协方差矩阵
    arm_mat_init_f32(&ekf->P, 7, 7, ekf->P_data);
    memset(ekf->P_data, 0, 49*sizeof(float));
    for(int i=0; i<7; i++) ekf->P_data[i*7+i] = (i<4) ? 0.01f : 0.001f;

    // 过程噪声（角度随机游走 + 偏置噪声）
    arm_mat_init_f32(&ekf->Q, 7, 7, ekf->Q_data);
    memset(ekf->Q_data, 0, 49*sizeof(float));
    for(int i=0; i<7; i++) ekf->Q_data[i*7+i] = (i<4) ? 1e-4f : 1e-6f;

    // 观测噪声
    arm_mat_init_f32(&ekf->R, 3, 3, ekf->R_data);
    memset(ekf->R_data, 0, 9*sizeof(float));
    for(int i=0; i<3; i++) ekf->R_data[i*3+i] = 0.3f*0.3f;
}

void EKF_Predict(EKF_Instance* ekf, float wx, float wy, float wz,float dt) {
    // 去偏置
    wx -= ekf->x[4];
    wy -= ekf->x[5];
    wz -= ekf->x[6];

    // 四元数积分
//    const float dt = ekf->dt;
		
    float dq[4] = {
        (-ekf->x[1]*wx - ekf->x[2]*wy - ekf->x[3]*wz) * 0.5f * dt,
        ( ekf->x[0]*wx + ekf->x[2]*wz - ekf->x[3]*wy) * 0.5f * dt,
        ( ekf->x[0]*wy - ekf->x[1]*wz + ekf->x[3]*wx) * 0.5f * dt,
        ( ekf->x[0]*wz + ekf->x[1]*wy - ekf->x[2]*wx) * 0.5f * dt
    };
    
    // 更新四元数
    ekf->x[0] += dq[0];
    ekf->x[1] += dq[1];
    ekf->x[2] += dq[2];
    ekf->x[3] += dq[3];
    quat_renorm(ekf->x);

    // 状态转移矩阵F（简化模型）
    arm_mat_init_f32(&ekf->F, 7, 7, ekf->F_data);
    mat_identity(&ekf->F, 7);

    // 协方差预测: P = F*P*F^T + Q
    arm_matrix_instance_f32 FP, FPF_T;
    float FP_data[49], FPF_T_data[49];
    arm_mat_init_f32(&FP, 7, 7, FP_data);
    arm_mat_init_f32(&FPF_T, 7, 7, FPF_T_data);
    
    arm_mat_mult_f32(&ekf->F, &ekf->P, &FP);
    arm_mat_trans_f32(&ekf->F, &ekf->F);
    arm_mat_mult_f32(&FP, &ekf->F, &FPF_T);
    arm_mat_add_f32(&FPF_T, &ekf->Q, &ekf->P);
}

void EKF_Update(EKF_Instance* ekf, float ax, float ay, float az) {
		
			// 在状态更新后添加偏置限幅
		for(int i=4; i<7; i++) {
				ekf->x[i] = fmaxf(fminf(ekf->x[i], 0.1f), -0.1f); // 限制偏置在±0.1rad/s内
		}
    // 加速度计归一化
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if(norm < MIN_NORM) return;
    ax /= norm; ay /= norm; az /= norm;

    // 预测的重力方向
    float g_pred[3] = {
        2.0f*(ekf->x[1]*ekf->x[3] - ekf->x[0]*ekf->x[2]),
        2.0f*(ekf->x[0]*ekf->x[1] + ekf->x[2]*ekf->x[3]),
        ekf->x[0]*ekf->x[0] - ekf->x[1]*ekf->x[1] 
           - ekf->x[2]*ekf->x[2] + ekf->x[3]*ekf->x[3]
    };

    // 残差
    ekf->y_data[0] = ax - g_pred[0];
    ekf->y_data[1] = ay - g_pred[1];
    ekf->y_data[2] = az - g_pred[2];
    arm_mat_init_f32(&ekf->y, 3, 1, ekf->y_data);

    // 观测矩阵H (3x7)
    arm_mat_init_f32(&ekf->H, 3, 7, ekf->H_data);
    memset(ekf->H_data, 0, 21*sizeof(float));
    ekf->H_data[0]  = -2*ekf->x[2];  // ?h0/?q0
    ekf->H_data[1]  =  2*ekf->x[3];  // ?h0/?q1
    ekf->H_data[2]  = -2*ekf->x[0];  // ?h0/?q2
    ekf->H_data[7]  =  2*ekf->x[1];  // ?h1/?q0
    ekf->H_data[8]  =  2*ekf->x[0];  // ?h1/?q1
    ekf->H_data[9]  =  2*ekf->x[3];  // ?h1/?q2
    ekf->H_data[14] =  2*ekf->x[0];  // ?h2/?q0
    ekf->H_data[15] = -2*ekf->x[1];  // ?h2/?q1
    ekf->H_data[16] = -2*ekf->x[2];  // ?h2/?q2

    // 卡尔曼增益计算
    arm_matrix_instance_f32 PH_T, S_inv;
    float PH_T_data[21], S_inv_data[9];
    
    arm_mat_trans_f32(&ekf->H, &ekf->H);
    arm_mat_mult_f32(&ekf->P, &ekf->H, &PH_T);
    
    arm_mat_init_f32(&ekf->S, 3, 3, ekf->S_data);
    arm_mat_mult_f32(&ekf->H, &PH_T, &ekf->S);
    arm_mat_add_f32(&ekf->S, &ekf->R, &ekf->S);

    // 矩阵求逆保护
//    arm_status inv_status = arm_mat_inverse_f32(&ekf->S, &S_inv);
//    if(inv_status != ARM_MATH_SUCCESS) return ;
    
				// 使用伪逆避免奇异矩阵
		arm_status status = arm_mat_inverse_f32(&ekf->S, &S_inv);
		if(status != ARM_MATH_SUCCESS) {
				// 求逆失败时采用标称增益
				arm_mat_scale_f32(&ekf->K, 0.5f, &ekf->K);
		}

    arm_mat_init_f32(&ekf->K, 7, 3, ekf->K_data);
    arm_mat_mult_f32(&PH_T, &S_inv, &ekf->K);

    // 状态更新
    arm_matrix_instance_f32 X_update;
    float X_update_data[7];
    arm_mat_init_f32(&X_update, 7, 1, X_update_data);
    arm_mat_mult_f32(&ekf->K, &ekf->y, &X_update);
    
    for(int i=0; i<7; i++) ekf->x[i] += X_update_data[i];
    quat_renorm(ekf->x);

    // 协方差更新
    arm_matrix_instance_f32 I, KH;
    float I_data[49], KH_data[49];
    arm_mat_init_f32(&I, 7, 7, I_data);
    mat_identity(&I, 7);
    
    arm_mat_init_f32(&KH, 7, 7, KH_data);
    arm_mat_mult_f32(&ekf->K, &ekf->H, &KH);
    arm_mat_sub_f32(&I, &KH, &KH);
    arm_mat_mult_f32(&KH, &ekf->P, &ekf->P);
//		for(int i=0; i<49; i++)
//		{
//			
//		if(ekf->P_data[i] > 100.0f )
//			 ekf->P_data[i] = 100.0f;
//		}
		
				// 协方差对称性保持
		for(int i=0; i<7; i++) {
				for(int j=i+1; j<7; j++) {
						float avg = 0.5f*(ekf->P_data[i*7+j] + ekf->P_data[j*7+i]);
						ekf->P_data[i*7+j] = ekf->P_data[j*7+i] = avg;
				}
		}

		// 协方差限幅
		for(int i=0; i<7; i++) {
				ekf->P_data[i*7+i] = fmaxf(fminf(ekf->P_data[i*7+i], 1.0f), 1e-6f);
		}

}

void EKF_GetEulerAngles(const EKF_Instance* ekf, float* roll, float* pitch, float* yaw) {
    const float q0 = ekf->x[0], q1 = ekf->x[1], q2 = ekf->x[2], q3 = ekf->x[3];
    
    // Roll (x-axis)
    *roll = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
    
    // Pitch (y-axis)
    float sinp = 2*(q0*q2 - q3*q1);
    if(fabsf(sinp) >= 1) {
        *pitch = copysignf(PI/2, sinp);
    } else {
        *pitch = asinf(sinp);
    }
    
    // Yaw (z-axis)
    *yaw = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
}
