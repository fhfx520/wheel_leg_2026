#include "Status_Task.h"
//#include "tim.h"
//#include "BMI088_Read.h"
//#include "IST8310_Read.h"
//#include "IMU_AHRSupdate.h"
//#include "Comm_Task.h"
//#include "bsp_imu.h"

uint8_t Data_Ready_Flag=0;	// 0:数据未就绪	 1：数据已就绪
uint8_t led_dir=0;	//LED呼吸方向
uint16_t led_pwm=0;

void status_task(void)
{
    Status_LED_PWM_Set(0);
    if(fabs(Temperature-calibration_temperature)>=5) {
        imu_mode = temperature_error;
    } else {
        imu_mode = normal;
    }
    if(imu_mode == normal) {
        if(led_dir == 0) {
            led_pwm++;
            if(led_pwm == 1000) {
                led_dir = 1;
            }
        } else {
            led_pwm--;
            if(led_pwm == 0) {
                led_dir = 0;
            }
        }
    }
    if(imu_mode == temperature_error){
        led_pwm=800;
    }
    Status_LED_PWM_Set(led_pwm);	//数据就绪后变为呼吸灯
}

void Data_Not_Ready(void)
{
    Data_Ready_Flag = 0;
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
    Status_LED_PWM_Set(0);	//数据未就绪状态灯常亮
}

void Data_Ready(void)
{
    Status_LED_PWM_Set(1000);
    HAL_Delay(200);
    Data_Ready_Flag = 1;
}
