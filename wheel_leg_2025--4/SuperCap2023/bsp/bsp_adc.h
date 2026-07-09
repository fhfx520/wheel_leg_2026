#ifndef __BSP_ADC_H__
#define __BSP_ADC_H__

#include "adc.h"

#define ID010

#define ADC1_BUFF_SIZE	4
#define ADC2_BUFF_SIZE	4
#define ADC3_BUFF_SIZE	1
#define ADC4_BUFF_SIZE  1

#define V_GAIN 				11.0f				//电压增益
#define I_GAIN				20.0f				//电流增益
#define I_STANDARD 		1.65f			//电流的测量电压标准值
#define V_REFERENCE		3.3f					//基准电压
#define R_SAMPLE			0.003f		//采样电阻
#define T_GAIN				100.0f
#define T_OFFSET			50.0f

void adcInit(void);
void adcGetValue(void);
void acdGetTemperature(void);
void adcGetVoltageStander(void);
#endif
