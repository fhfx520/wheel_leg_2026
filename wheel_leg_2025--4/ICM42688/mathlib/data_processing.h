#ifndef __DATA_PROCESSING_H
#define __DATA_PROCESSING_H

#include "stm32g4xx_hal.h"
//#include "FreeRTOS.h"


#include <math.h>
#include <stdint.h>

#define PI 3.14159265358979323846f

#define STATIC_DETECT_WINDOW 20    // 静止检测窗口大小
#define STATIC_THRESHOLD      0.02f // 静止状态判断阈值(rad/s)
#define GYRO_DT 0.001f  // 1ms采样周期
#define MOTION_THRESHOLD 2.0f  // 运动检测阈值(rad/s)

typedef struct {
    float q;    // 过程噪声协方差
    float r;    // 测量噪声协方差
    float p;    // 估计误差协方差
    float k;    // 卡尔曼增益
    float x;    // 最优估计值
    float b;    // 零偏估计
} KalmanFilter;

typedef struct {
    float window[5];    // 滑动窗口
    uint8_t index;      // 当前索引
    float sum;          // 窗口求和
} MovingAverage;

typedef struct {
    KalmanFilter kf[3];  // XYZ三轴卡尔曼滤波器
    MovingAverage ma[3]; // XYZ三轴滑动平均
    float adaptive_gain; // 自适应增益系数
    float last_output[3];// 上次输出值
	
	    float static_bias[3];      // 静态零偏补偿值
    float variance[3];         // 方差计算缓存
    uint8_t static_counter;    // 静止状态计数器
	
	
} GyroFilter;


void Float2Byte(float *target,unsigned char *buf,unsigned char beg);
void GyroFilter_Update(GyroFilter* gf, float* raw_data, float* filtered_data);
void GyroFilter_Init(GyroFilter* gf) ;
float invSqrt(float x);

#endif

