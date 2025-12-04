#ifndef MAHONY_H
#define MAHONY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#define PI 3.14159265358979323846f
#define DEFAULT_ZERO_TOLERANCE 1e-6f


typedef struct {
    float q0, q1, q2, q3;   // 四元数
    float integralFBx, integralFBy, integralFBz; // 积分误差
    float Kp;               // 比例增益
    float Ki;               // 积分增益
    float sampleFreq;       // 采样频率(Hz)
} MahonyFilter;
 
// 初始化滤波器
void Mahony_Init(MahonyFilter* filter, float sampleFreq, float Kp, float Ki);

// 更新姿态解算（需定时调用）
void Mahony_Update(MahonyFilter* filter, 
                  float gx, float gy, float gz,  // 陀螺仪数据(rad/s)
                  float ax, float ay, float az); // 加速度计数据(归一化)

// 获取欧拉角(弧度)
void Mahony_GetEulerAngles(const MahonyFilter* filter, 
                          float* roll, 
                          float* pitch, 
                          float* yaw);

// 四元数转欧拉角(直接使用)
void QuaternionToEuler(float q0, float q1, float q2, float q3,
                      float* roll, float* pitch, float* yaw);

#ifdef __cplusplus
}
#endif

#endif /* MAHONY_FILTER_H */
