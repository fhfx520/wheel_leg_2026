#ifndef EKF_ATTITUDE_H
#define EKF_ATTITUDE_H

#include "arm_math.h"

typedef struct {
    // 状态：四元数 + 陀螺偏置 [q0, q1, q2, q3, bias_x, bias_y, bias_z]
    float x[7];
    float dt;  // 采样周期
    
    // 协方差矩阵 (7x7)
    arm_matrix_instance_f32 P;
    float P_data[49];
    
    // 过程噪声 (7x7)
    arm_matrix_instance_f32 Q;
    float Q_data[49];
    
    // 观测噪声 (3x3)
    arm_matrix_instance_f32 R;
    float R_data[9];
    
    // 临时矩阵
    arm_matrix_instance_f32 F, H, K, S, y;
    float F_data[49], H_data[21], K_data[21], S_data[9], y_data[3];
} EKF_Instance;

// 初始化滤波器
void EKF_Init(EKF_Instance* ekf, float dt);

// 预测步骤（陀螺仪输入：rad/s）
void EKF_Predict(EKF_Instance* ekf, float wx, float wy, float wz,float dt);

// 更新步骤（加速度计输入：m/s2）
void EKF_Update(EKF_Instance* ekf, float ax, float ay, float az);

// 获取欧拉角（弧度）
void EKF_GetEulerAngles(const EKF_Instance* ekf, float* roll, float* pitch, float* yaw);

#endif
