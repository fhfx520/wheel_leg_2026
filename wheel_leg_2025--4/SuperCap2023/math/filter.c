#include "filter.h"
#include "main.h"

LPFO_initstruct LPFO_init=
{
	.mode = 0,
	.K = 0.8627,    //0.00625  0.8627
};

void filterInit(void)
{
		LPFOfilter_init(&chassis_PA2_filter,&LPFO_init);
		LPFOfilter_init(&cap_PA3_filter,&LPFO_init);
		LPFOfilter_init(&bat_PA6_filter,&LPFO_init);
		LPFOfilter_init(&vint_PA0_filter,&LPFO_init);
		LPFOfilter_init(&vout_PA1_filter,&LPFO_init);
		LPFOfilter_init(&vbat_PA5_filter,&LPFO_init);
}

/**
  *************************************************************************
  * @brief  一阶低通滤波器
  * @author ZZJ
  * @date   2020/9/27
  * @note   使用时，在需要处理的数据变量文件中，包含头文件，
  *         编写初始化函数，主函数中完成初始化，在需要滤波的地方调用滤波器函数
  *         调参：一般模式下，只用调K； 动态模式下，需要调A，T，和Kw
  *************************************************************************
  */
/**
  * @brief  一阶低通滤波器结构体初始化函数
  * @param  pft      滤波器结构体指针变量
  * @param  pftinit  滤波器结构体初始化结构体指针变量
  * @retval 无
  */
void LPFOfilter_init(LPFOfilter_t* pft,LPFO_initstruct* pftinit)
{
    pft->mode = pftinit->mode;
    pft->K    = pftinit->K;
    pft->last_data = pft->now_data =
                               pft->last_after_data = pft->now_after_data = 0;
    if(pftinit->mode)  //如果启用滤波系数的动态调节
    {
        pft->Kw = pftinit->Kw;
        pft->Threshold_A = pftinit->Threshold_A;
        pft->Threshold_T = pftinit->Threshold_T;
        pft->cnt = 0;
        pft->dir_last = pft->dir_now = 0;
    }
}


/**
  * @brief  一阶低通滤波器功能函数
  * @param  pft  滤波器结构体指针变量
  * @retval 无
  */
CCMRAM float LPFOfilter_cal(LPFOfilter_t* pft, float get)
{
    pft->now_data = get;

    if(pft->mode)  //如果使用动态调整
    {
        /*根据连续真实数据判断数据状态*/
        if(pft->now_data > pft->last_data + pft->Threshold_A)  //判断变化方向
            pft->dir_now = 1;
        else if(pft->now_data < pft->last_data - pft->Threshold_A)
            pft->dir_now = 2;
        else
            pft->dir_now = 0;
        pft->last_data = pft->now_data;

        /*增量稳定性判断*/
        if(pft->dir_now == pft->dir_last)  //判断前后两次变化方向是否相等  // && pft->dir_last != 0
        {
            pft->cnt+=1;
            if(pft->cnt >= pft->Threshold_T)  //比较 单向变化持续时长
            {
                pft->K += 0.1f;  //提高灵敏度，增强跟随效果
                if(pft->K >= 1) pft->K = 1;
                pft->cnt = pft->Threshold_T;  //防止溢出
            }
        }
        else
        {
            //pft->cnt = 0;
            pft->K = pft->Kw;  //数据围绕某值波动时，降低灵敏度，增强滤波效果
        }
        pft->dir_last = pft->dir_now;
    }
    /*一阶低通滤波*/
    pft->now_after_data = pft->K*pft->now_data + (1-pft->K)*pft->last_after_data;

    pft->last_after_data = pft->now_after_data;

    return pft->now_after_data;
}


/**
  *************************************************************************
  * @brief  平均滤波器
  * @author ZZJ
  * @date   2020/11/19
  * @note   
  *************************************************************************
  */
CCMRAM float AVGfilter_cal(AVGfilter_t* avg, float get)
{
    float res = 0;
    if(avg->index < avg->num)
    {
        avg->array[avg->index++] = get;
        res = get;
    }
    else
    {
        float sum = 0;
        avg->array[(avg->index++)%avg->num] = get;
        for(uint8_t i = 0; i<avg->num; i++)
            sum += avg->array[i];
        res = sum/avg->num;
    }
    return res;
}


void avg_filter_init(AVGfilter_t* avg_filter)
{
    avg_filter->num = 5;
    avg_filter->after_data = 0;
    avg_filter->index = 0;
    
    for(uint8_t i = 0; i<avg_filter->num; i++)
        avg_filter->array[i] = 0;
}

