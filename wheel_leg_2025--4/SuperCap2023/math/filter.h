#ifndef __FILTER_H
#define __FILTER_H
#include "stdint.h"
#include "math.h"
/*----------------------------------------------------------------------------------------------------------------------/
        *               
        *               一阶低通滤波器
        *               
----------------------------------------------------------------------------------------------------------------------*/
/*一阶低通滤波器模式*/
#define LPFO_MODE_N 0
#define LPFO_MODE_D 1       //使用动态调节


/*一阶低通滤波器结构体类型*/
typedef struct
{
    uint8_t mode : 1;       //是否使用动态调节
    float K;                //滤波系数

    float last_data;        //上一次获取数据
    float now_data;         //本次获取的数据

    float last_after_data;        //上一次获取数据
    float now_after_data;         //本次获取的数据

    float Kw;               //数据稳定时系数
    uint8_t dir_last;   //数据变化方向
    uint8_t dir_now;
    uint8_t cnt;            //稳定增长（减少）持续时间
    uint8_t Threshold_A;    //阈值1用于一阶带参滤波器，变化角度大于此值时，计数增加
    uint8_t Threshold_T;    //阈值2用于一阶带参滤波器，计数值大于此值时，增大参数，增强滤波跟随
} LPFOfilter_t;


/*一阶低通滤波器初始化结构体类型*/
typedef struct {
    uint8_t mode : 1;
    float K;
    float Kw;
    uint8_t Threshold_A;
    uint8_t Threshold_T;
} LPFO_initstruct;


float LPFOfilter_cal(LPFOfilter_t* pft, float get);
void LPFOfilter_init(LPFOfilter_t* pft,LPFO_initstruct* pftinit);
/*----------------------------------------------------------------------------------------------------------------------/
        *               
        *               平均滤波器
        *               
----------------------------------------------------------------------------------------------------------------------*/
#ifndef __AVG_FILTER_H
#define __AVG_FILTER_H


#define AVG_FILTER_NUM 10


typedef struct{
    uint8_t num;
    float array[AVG_FILTER_NUM];
    uint8_t index;
    float after_data;
}AVGfilter_t;

extern AVGfilter_t avg_filter1;
extern AVGfilter_t avg_filter2;
extern AVGfilter_t avg_filter3;
extern AVGfilter_t avg_filter4;
extern AVGfilter_t avg_filter5;
extern AVGfilter_t avg_filter6;

extern LPFOfilter_t chassis_PA2_filter;
extern LPFOfilter_t cap_PA3_filter;
extern LPFOfilter_t bat_PA6_filter;
extern LPFOfilter_t vint_PA0_filter;
extern LPFOfilter_t vout_PA1_filter;
extern LPFOfilter_t vbat_PA5_filter;

void filterInit(void);
void avg_filter_init(AVGfilter_t* avg_filter);
float AVGfilter_cal(AVGfilter_t* avg, float get);
#endif

#endif


