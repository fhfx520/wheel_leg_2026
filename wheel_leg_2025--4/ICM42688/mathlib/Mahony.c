#include "Mahony.h"
#include <math.h>



// 初始化滤波器参数
void Mahony_Init(MahonyFilter* filter, float sampleFreq, float Kp, float Ki) {
    filter->q0 = 1.0f;
    filter->q1 = filter->q2 = filter->q3 = 0.0f;
    filter->integralFBx = filter->integralFBy = filter->integralFBz = 0.0f;
    filter->Kp = Kp;
    filter->Ki = Ki;
    filter->sampleFreq = sampleFreq;
}

// 四元数归一化
static void QuaternionNormalize(float* q0, float* q1, float* q2, float* q3) {
    float norm = sqrtf(*q0 * *q0 + *q1 * *q1 + *q2 * *q2 + *q3 * *q3);
    if(norm < DEFAULT_ZERO_TOLERANCE) return;
    norm = 1.0f / norm;
    *q0 *= norm;
    *q1 *= norm;
    *q2 *= norm;
    *q3 *= norm;
}

// 主更新函数
void Mahony_Update(MahonyFilter* filter, float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    
    // 加速度计数据归一化
    recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // 计算当前姿态预估的重力方向
    halfvx = filter->q1 * filter->q3 - filter->q0 * filter->q2;
    halfvy = filter->q0 * filter->q1 + filter->q2 * filter->q3;
    halfvz = filter->q0 * filter->q0 - 0.5f + filter->q3 * filter->q3;
    
    // 计算加速度计测量值与预估值的误差
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    
    // 积分误差计算
    if(filter->Ki > 0.0f) {
        filter->integralFBx += filter->Ki * halfex * (1.0f / filter->sampleFreq);
        filter->integralFBy += filter->Ki * halfey * (1.0f / filter->sampleFreq);
        filter->integralFBz += filter->Ki * halfez * (1.0f / filter->sampleFreq);
        
        // 应用积分反馈
        gx += filter->integralFBx;
        gy += filter->integralFBy;
        gz += filter->integralFBz;
    }
    
    // 应用比例反馈
    gx += filter->Kp * halfex;
    gy += filter->Kp * halfey;
    gz += filter->Kp * halfez;
    
    // 积分四元数微分方程
    float dt = 1.0f / filter->sampleFreq;
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    
    qa = filter->q0;
    qb = filter->q1;
    qc = filter->q2;
    
    filter->q0 += (-qb * gx - qc * gy - filter->q3 * gz);
    filter->q1 += (qa * gx + qc * gz - filter->q3 * gy);
    filter->q2 += (qa * gy - qb * gz + filter->q3 * gx);
    filter->q3 += (qa * gz + qb * gy - qc * gx);
    
    // 四元数归一化
    QuaternionNormalize(&filter->q0, &filter->q1, &filter->q2, &filter->q3);
}

// 获取欧拉角(弧度)
void Mahony_GetEulerAngles(const MahonyFilter* filter, float* roll, float* pitch, float* yaw) {
    QuaternionToEuler(filter->q0, filter->q1, filter->q2, filter->q3, roll, pitch, yaw);
}

// 四元数转欧拉角(Z-Y-X顺序)
void QuaternionToEuler(float q0, float q1, float q2, float q3, 
                      float* roll, float* pitch, float* yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if(fabsf(sinp) >= 1.0f)
        *pitch = copysignf(PI/2, sinp); // 使用90度近似
    else
        *pitch = asinf(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp);

}
			
