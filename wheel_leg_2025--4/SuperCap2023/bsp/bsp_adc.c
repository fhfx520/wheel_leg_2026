#include "bsp_adc.h"
#include "adc.h"
#include "filter.h"
#include "hrtim.h"
#include "power_ctrl_task.h"

uint16_t adc1_value_buf[ADC1_BUFF_SIZE];
uint16_t adc2_value_buf[ADC2_BUFF_SIZE];
uint16_t adc3_value_buf[ADC3_BUFF_SIZE];
uint16_t adc4_value_buf[ADC4_BUFF_SIZE];

CCMRAM LPFOfilter_t chassis_PA2_filter;
CCMRAM LPFOfilter_t cap_PA3_filter;
CCMRAM LPFOfilter_t bat_PA6_filter;
CCMRAM LPFOfilter_t vint_PA0_filter;
CCMRAM LPFOfilter_t vout_PA1_filter;
CCMRAM LPFOfilter_t vbat_PA5_filter;

CCMRAM float v_reference;



void adcInit(void)
{
		HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);		/*??adc??*/
		HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
		HAL_ADCEx_Calibration_Start(&hadc3,ADC_SINGLE_ENDED);		/*??adc??*/
		HAL_ADCEx_Calibration_Start(&hadc4,ADC_SINGLE_ENDED);		/*??adc??*/
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&adc1_value_buf,ADC1_BUFF_SIZE);	/*??ADCdma??*/
		HAL_ADC_Start_DMA(&hadc2,(uint32_t*)&adc2_value_buf,ADC2_BUFF_SIZE);
		HAL_ADC_Start_DMA(&hadc3,(uint32_t*)&adc3_value_buf,ADC3_BUFF_SIZE);
		HAL_ADC_Start_DMA(&hadc4,(uint32_t*)&adc4_value_buf,ADC4_BUFF_SIZE);
}

CCMRAM void adcGetValue(void)
{
		/*????*/
		/*????*/
		/*????*/
		/*???????????????*/
/*001*/
#ifdef ID000
		bat.V = LPFOfilter_cal(&vbat_PA5_filter,((float)(adc1_value_buf[3])* v_reference/65535) * V_GAIN);0.9955x + 0.0959
		bat.I = LPFOfilter_cal(&bat_PA6_filter,(((float)(adc4_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));1.0096x + 0.1289
		bat.P =	bat.V *bat.I;
		chassis.V = LPFOfilter_cal(&vint_PA0_filter,((float)(adc1_value_buf[1])* v_reference/65535) * 14.0f);
		chassis.I = LPFOfilter_cal(&chassis_PA2_filter,(((float)(adc3_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));
		chassis.P = chassis.V *chassis.I;
		cap.V = LPFOfilter_cal(&vout_PA1_filter,((float)(adc1_value_buf[2])* v_reference/65535) * V_GAIN);
		cap.I = LPFOfilter_cal(&cap_PA3_filter,(((float)(adc1_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));//*1.018871f +0.01565f
		cap.P = cap.V * cap.I ;
#endif
#ifdef ID001
		bat.V = LPFOfilter_cal(&vbat_PA5_filter,((float)(adc1_value_buf[3])* v_reference/65535) * V_GAIN)*0.9955f+ 0.0959f;
		bat.I = LPFOfilter_cal(&bat_PA6_filter,(((float)(adc4_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE))*1.0096f + 0.1289f;
		bat.P =	bat.V *bat.I;
		chassis.V = LPFOfilter_cal(&vint_PA0_filter,((float)(adc1_value_buf[1])* v_reference/65535) * 14.0f);
		chassis.I = LPFOfilter_cal(&chassis_PA2_filter,(((float)(adc3_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));
		chassis.P = chassis.V *chassis.I;
		cap.V = LPFOfilter_cal(&vout_PA1_filter,((float)(adc1_value_buf[2])* v_reference/65535) * V_GAIN);
		cap.I = LPFOfilter_cal(&cap_PA3_filter,(((float)(adc1_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));//*1.018871f +0.01565f
		cap.P = cap.V * cap.I ;
#endif
#ifdef ID002
		bat.V = LPFOfilter_cal(&vbat_PA5_filter,((float)(adc1_value_buf[3])* v_reference/65535) * V_GAIN)*0.9939f + 0.1052f;
		bat.I = LPFOfilter_cal(&bat_PA6_filter,(((float)(adc4_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE))*1.0066f + 0.1249f;
		bat.P =	bat.V *bat.I;
		chassis.V = LPFOfilter_cal(&vint_PA0_filter,((float)(adc1_value_buf[1])* v_reference/65535) * 14.0f);
		chassis.I = LPFOfilter_cal(&chassis_PA2_filter,(((float)(adc3_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));
		chassis.P = chassis.V *chassis.I;
		cap.V = LPFOfilter_cal(&vout_PA1_filter,((float)(adc1_value_buf[2])* v_reference/65535) * V_GAIN);
		cap.I = LPFOfilter_cal(&cap_PA3_filter,(((float)(adc1_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));//*1.018871f +0.01565f
		cap.P = cap.V * cap.I ;
#endif
#ifdef ID003
		bat.V = LPFOfilter_cal(&vbat_PA5_filter,((float)(adc1_value_buf[3])* v_reference/65535) * V_GAIN)*0.9939f+ 0.0949f;
		bat.I = LPFOfilter_cal(&bat_PA6_filter,(((float)(adc4_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE))*0.998f +0.0451f;
		bat.P =	bat.V *bat.I;
		chassis.V = LPFOfilter_cal(&vint_PA0_filter,((float)(adc1_value_buf[1])* v_reference/65535) * 14.0f);
		chassis.I = LPFOfilter_cal(&chassis_PA2_filter,(((float)(adc3_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));
		chassis.P = chassis.V *chassis.I;
		cap.V = LPFOfilter_cal(&vout_PA1_filter,((float)(adc1_value_buf[2])* v_reference/65535) * V_GAIN);
		cap.I = LPFOfilter_cal(&cap_PA3_filter,(((float)(adc1_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));//*1.018871f +0.01565f
		cap.P = cap.V * cap.I ;
#endif
#ifdef ID004
		bat.V = LPFOfilter_cal(&vbat_PA5_filter,((float)(adc1_value_buf[3])* v_reference/65535) * V_GAIN)*0.9988f + 0.0753f;
		bat.I = LPFOfilter_cal(&bat_PA6_filter,(((float)(adc4_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE))*0.9897f - 0.1032f;
		bat.P =	bat.V *bat.I;
		chassis.V = LPFOfilter_cal(&vint_PA0_filter,((float)(adc1_value_buf[1])* v_reference/65535) * 14.0f);
		chassis.I = LPFOfilter_cal(&chassis_PA2_filter,(((float)(adc3_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));
		chassis.P = chassis.V *chassis.I;
		cap.V = LPFOfilter_cal(&vout_PA1_filter,((float)(adc1_value_buf[2])* v_reference/65535) * V_GAIN);
		cap.I = LPFOfilter_cal(&cap_PA3_filter,(((float)(adc1_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));//*1.018871f +0.01565f
		cap.P = cap.V * cap.I ;
#endif
#ifdef ID005
		bat.V = LPFOfilter_cal(&vbat_PA5_filter,((float)(adc1_value_buf[3])* v_reference/65535) * V_GAIN)*0.975f + 0.0234f;
		bat.I = LPFOfilter_cal(&bat_PA6_filter,(((float)(adc4_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE))*1.0023f + 0.0812f;
		bat.P =	bat.V *bat.I;
		chassis.V = LPFOfilter_cal(&vint_PA0_filter,((float)(adc1_value_buf[1])* v_reference/65535) * 14.0f);
		chassis.I = LPFOfilter_cal(&chassis_PA2_filter,(((float)(adc3_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));
		chassis.P = chassis.V *chassis.I;
		cap.V = LPFOfilter_cal(&vout_PA1_filter,((float)(adc1_value_buf[2])* v_reference/65535) * V_GAIN);
		cap.I = LPFOfilter_cal(&cap_PA3_filter,(((float)(adc1_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));//*1.018871f +0.01565f
		cap.P = cap.V * cap.I ;
#endif
#ifdef ID006
		bat.V = LPFOfilter_cal(&vbat_PA5_filter,((float)(adc1_value_buf[3])* v_reference/65535) * V_GAIN)*1.0073f - 0.1124f;
		bat.I = LPFOfilter_cal(&bat_PA6_filter,(((float)(adc4_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE))*1.0111f + 0.1155f;
		bat.P =	bat.V *bat.I;
		chassis.V = LPFOfilter_cal(&vint_PA0_filter,((float)(adc1_value_buf[1])* v_reference/65535) * 14.0f);
		chassis.I = LPFOfilter_cal(&chassis_PA2_filter,(((float)(adc3_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));
		chassis.P = chassis.V *chassis.I;
		cap.V = LPFOfilter_cal(&vout_PA1_filter,((float)(adc1_value_buf[2])* v_reference/65535) * V_GAIN);
		cap.I = LPFOfilter_cal(&cap_PA3_filter,(((float)(adc1_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));//*1.018871f +0.01565f
		cap.P = cap.V * cap.I ;
#endif

#ifdef ID011
		bat.V = LPFOfilter_cal(&vbat_PA5_filter,((float)(adc1_value_buf[3])* v_reference/65535) * V_GAIN)*0.9833f + 0.144f;
		bat.I = LPFOfilter_cal(&bat_PA6_filter,(((float)(adc4_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE))*0.9868f - 0.0302f;
		bat.P =	bat.V *bat.I;
		chassis.V = LPFOfilter_cal(&vint_PA0_filter,((float)(adc1_value_buf[1])* v_reference/65535) * 14.0f);
		chassis.I = LPFOfilter_cal(&chassis_PA2_filter,(((float)(adc3_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));
		chassis.P = chassis.V *chassis.I;
		cap.V = LPFOfilter_cal(&vout_PA1_filter,((float)(adc1_value_buf[2])* v_reference/65535) * V_GAIN);
		cap.I = LPFOfilter_cal(&cap_PA3_filter,(((float)(adc1_value_buf[0])* v_reference/65535) - I_STANDARD) / (I_GAIN * R_SAMPLE));//*1.018871f +0.01565f
		cap.P = cap.V * cap.I ;
#endif

}

uint8_t value = 0;

uint16_t VREFINT_GetCalibrationValue(void) {
    return *VREFINT_CAL_ADDR;
}

const float SCALE_FACTOR = (3.3f * V_REFERENCE * 4095.0f) / (65535.0f * 3.0f);

CCMRAM void adcGetVoltageStander(void)
{
	v_reference = SCALE_FACTOR * (float)(adc1_value_buf[0]) / (float)VREFINT_GetCalibrationValue();
}
CCMRAM void acdGetTemperature(void)
{
		temperature.T1 = ((float)(adc2_value_buf[1])* V_REFERENCE/65535) * T_GAIN - T_OFFSET;
		temperature.T2 = ((float)(adc2_value_buf[2])* V_REFERENCE/65535) * T_GAIN - T_OFFSET;
		temperature.T3 = ((float)(adc2_value_buf[3])* V_REFERENCE/65535) * T_GAIN - T_OFFSET;
}
