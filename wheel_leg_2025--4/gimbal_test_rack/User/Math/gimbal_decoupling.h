#ifndef __GIMBAL_DECOUPLING_H
#define __GIMBAL_DECOUPLING_H

#include "stdint.h"
#include "stdlib.h"
#include "arm_math.h"

#ifndef user_malloc
    #ifdef _CMSIS_OS_H
        #define user_malloc pvPortMalloc
    #else
        #define user_malloc malloc
    #endif
#endif

#define mat         arm_matrix_instance_f32
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

typedef struct
{
	//矩阵结构体
	mat G_W_Rotate_mat;//云台系到世界系的旋转矩阵
	mat G_B_Rotate_mat;//云台系到基座系的旋转矩阵
	mat	B_W_Rotate_mat;//基座系到世界系的旋转矩阵
	mat W_target_vector;//世界系下的目标向量
	mat B_target_vector;//解耦Roll到基座系下的目标向量
	//绕三轴的旋转矩阵
	mat yaw_rotate_mat;
	mat pitch_rotate_mat;
	mat roll_rotate_mat;
	//中间矩阵
	mat temp_vector1;
	mat temp_vector2;
	//矩阵存储空间定义
	float *G_W_Rotate_data;
	float *G_B_Rotate_data;
	float *B_W_Rotate_data;
	float *W_target_data;
	float *B_target_data;
	float *yaw_rotate_data;
	float *pitch_rotate_data;
	float *roll_rotate_data;
	float *temp_data1,*temp_data2;
	//外部接口[yaw pitch]2阶列向量
	float *imu_mea_vector,*motor_mea_vector,*vision_mea_vector;
	float *decoupling_vector;
}decoupling_t;

void decoupling_init(decoupling_t* dp);
void decoupling_inverse_calc(decoupling_t* dp);

#endif
