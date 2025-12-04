/**
	* @file data_processing.c
	* @version 1.0
	* @date 2019.11.26
  *
  * @brief  一些数据处理函数
  *
  *	@author YY
  *
  */
	
#include "data_processing.h"


/***************************************************************/
/*
 * 函数名：Float2Byte
 * 描述  ：将单精度浮点数据转成4字节数据并存入指定地址
 * 输入  ：target:目标单精度数据
					 buf:待写入数组
					 beg:指定从数组第几个元素开始写入
 * 输出  ：无
 */ 
/***************************************************************/
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
// Fast inverse square-root
*******************************************************************************/
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


// 卡尔曼滤波器初始化
void Kalman_Init(KalmanFilter* kf, float q, float r) {
    kf->q = q;
    kf->r = r;
    kf->p = 1.0f;
    kf->k = 0.0f;
    kf->x = 0.0f;
    kf->b = 0.0f;
}

// 卡尔曼滤波器更新
float Kalman_Update(KalmanFilter* kf, float measurement) {
    // 预测阶段
    kf->p += kf->q;
    
    // 更新阶段
    kf->k = kf->p / (kf->p + kf->r);
    kf->x += kf->k * (measurement - kf->x - kf->b);
    kf->p *= (1 - kf->k);
    
    // 零偏自适应估计
    kf->b += 0.001f * (measurement - kf->x - kf->b);
    
    return kf->x;
}

// 滑动平均初始化
void MA_Init(MovingAverage* ma) {
    for(int i=0; i<5; i++) ma->window[i] = 0;
    ma->index = 0;
    ma->sum = 0;
}

// 滑动平均更新
float MA_Update(MovingAverage* ma, float input) {
    ma->sum -= ma->window[ma->index];
    ma->window[ma->index] = input;
    ma->sum += input;
    ma->index = (ma->index + 1) % 5;
    return ma->sum / 5.0f;
}

// 陀螺仪滤波器初始化
void GyroFilter_Init(GyroFilter* gf) {
    for(int i=0; i<3; i++) {
        Kalman_Init(&gf->kf[i], 0.001f, 0.03f);
        MA_Init(&gf->ma[i]);
        gf->last_output[i] = 0.0f;
    }
    gf->adaptive_gain = 1.0f;
		
		    for(int i=0; i<3; i++) {
        gf->static_bias[i] = 0.0f;
        gf->variance[i] = 0.0f;
    }
    gf->static_counter = 0;
		
		
}

// 主滤波函数
void GyroFilter_Update(GyroFilter* gf, float* raw_data, float* filtered_data) {
  float motion_level = 0;
    float ma_output[3];
    static float history[STATIC_DETECT_WINDOW][3];
    
    // 阶段1：滑动平均预处理（保持不变）
    for(int i=0; i<3; i++) {
        ma_output[i] = MA_Update(&gf->ma[i], raw_data[i]);
    }

    // 新增：Yaw轴专项处理 -----------------------------------
    // 检测静止状态（基于滑动窗口方差计算）
    static uint8_t history_index = 0;
    for(int i=0; i<3; i++){
        // 更新历史数据队列
        history[history_index][i] = ma_output[i];
        
        // 计算窗口方差
        float mean = gf->ma[i].sum / 5.0f;
        gf->variance[i] = 0.0f;
        for(int j=0; j<STATIC_DETECT_WINDOW; j++){
            gf->variance[i] += powf(history[j][i] - mean, 2);
        }
        gf->variance[i] /= STATIC_DETECT_WINDOW;
    }
    history_index = (history_index + 1) % STATIC_DETECT_WINDOW;

    // 静止状态判断（所有轴方差均低于阈值）
    uint8_t is_static = 1;
    for(int i=0; i<3; i++){
        if(gf->variance[i] > STATIC_THRESHOLD) {
            is_static = 0;
            break;
        }
    }

    // Yaw轴零偏动态补偿（仅在静止时更新）
    if(is_static) {
        gf->static_counter = (gf->static_counter < 255) ? gf->static_counter+1 : 255;
        
        // 指数滑动平均更新零偏
        float alpha = (gf->static_counter > 50) ? 0.01f : 0.0f;
        for(int i=0; i<3; i++){
            gf->static_bias[i] = (1-alpha)*gf->static_bias[i] + alpha*ma_output[i];
        }
        
        // 特别增强Yaw轴补偿（Z轴）
        gf->kf[2].b = 0.95f*gf->kf[2].b + 0.05f*gf->static_bias[2];
    } else {
        gf->static_counter = 0;
    }

    // 阶段2：带补偿的卡尔曼滤波 -----------------------------
    for(int i=0; i<3; i++) {
        // 注入静态补偿值
        float compensated_meas = ma_output[i] - gf->static_bias[i];
        filtered_data[i] = Kalman_Update(&gf->kf[i], compensated_meas);
        motion_level += fabsf(filtered_data[i] - gf->last_output[i]);
    }
    
    // 第三阶段：动态响应调整
    motion_level /= 3.0f * GYRO_DT;
    gf->adaptive_gain = (motion_level > MOTION_THRESHOLD) ? 
                        0.9f : 0.1f;
    
    // 混合输出
    for(int i=0; i<3; i++) {
        filtered_data[i] = gf->adaptive_gain * filtered_data[i] + 
                          (1 - gf->adaptive_gain) * gf->last_output[i];
        gf->last_output[i] = filtered_data[i];
    }
}
