#include "gimbal_decoupling.h"


void decoupling_init(decoupling_t* dp)
{
	//世界系目标向量初始化
	dp->W_target_data = (float*)user_malloc(sizeof(float) * 3);
	memset(dp->W_target_data,0,sizeof(float) * 3);
	mat_init(&dp->W_target_vector,3,1,dp->W_target_data);
	//基座系目标向量初始化
	dp->B_target_data = (float*)user_malloc(sizeof(float) * 3);
	memset(dp->B_target_data,0,sizeof(float) * 3);
	mat_init(&dp->B_target_vector,3,1,dp->B_target_data);
	//云台系到世界系旋转矩阵初始化
	dp->G_W_Rotate_data = (float*)user_malloc(sizeof(float) * 3 * 3);
	memset(dp->G_W_Rotate_data,0,sizeof(float) * 3 * 3);
	mat_init(&dp->G_W_Rotate_mat,3,3,(float*)dp->G_W_Rotate_data);
	//云台系到基座系旋转矩阵初始化
	dp->G_B_Rotate_data = (float*)user_malloc(sizeof(float) * 3 * 3);
	memset(dp->G_B_Rotate_data,0,sizeof(float) * 3 * 3);
	mat_init(&dp->G_B_Rotate_mat,3,3,(float*)dp->G_B_Rotate_data);
	//基座系到世界系的旋转矩阵初始化
	dp->B_W_Rotate_data = (float*)user_malloc(sizeof(float) * 3 * 3);
	memset(dp->B_W_Rotate_data,0,sizeof(float) * 3 * 3);
	mat_init(&dp->B_W_Rotate_mat,3,3,(float*)dp->B_W_Rotate_data);
	//绕三轴旋转矩阵初始化
	dp->yaw_rotate_data = (float*)user_malloc(sizeof(float) * 3 * 3);
	memset(dp->yaw_rotate_data,0,sizeof(float) * 3 * 3);
	mat_init(&dp->yaw_rotate_mat,3,3,(float*)dp->yaw_rotate_data);

	dp->pitch_rotate_data = (float*)user_malloc(sizeof(float) * 3 * 3);
	memset(dp->pitch_rotate_data,0,sizeof(float) * 3 * 3);
	mat_init(&dp->pitch_rotate_mat,3,3,(float*)dp->pitch_rotate_data);

	dp->roll_rotate_data = (float*)user_malloc(sizeof(float) * 3 * 3);
	memset(dp->roll_rotate_data,0,sizeof(float) * 3 * 3);
	mat_init(&dp->roll_rotate_mat,3,3,(float*)dp->roll_rotate_data);
	//外部接口初始化[roll pitch]
	//陀螺仪测量
	dp->imu_mea_vector = (float*)user_malloc(sizeof(float) * 2);
	memset(dp->imu_mea_vector,0,sizeof(float) * 2);
	//电机测量
	dp->motor_mea_vector = (float*)user_malloc(sizeof(float) * 2);
	memset(dp->motor_mea_vector,0,sizeof(float) * 2);
	//视觉测量
	dp->vision_mea_vector = (float*)user_malloc(sizeof(float) * 2);
	memset(dp->vision_mea_vector,0,sizeof(float) * 2);
	//输出
	dp->decoupling_vector = (float*)user_malloc(sizeof(float) * 2);
	memset(dp->decoupling_vector,0,sizeof(float) * 2);
	//中间向量初始化
	dp->temp_data1 = (float*)user_malloc(sizeof(float) * 3);
	dp->temp_data1[0] = 1,dp->temp_data1[1] = 0,dp->temp_data1[2] = 0;
	mat_init(&dp->temp_vector1,3,1,(float*)dp->temp_data1);

	dp->temp_data2 = (float*)user_malloc(sizeof(float) * 3 * 3);
	memset(dp->temp_data2,0,sizeof(float) * 3 * 3);
	mat_init(&dp->temp_vector2,3,3,(float*)dp->temp_data2);
}

void rotate_yaw_matrix(float *arr,float theta)
{
	float temp_arr[3][3] = 
	{
		{arm_cos_f32(theta),-arm_sin_f32(theta),0},
		{arm_sin_f32(theta),arm_cos_f32(theta),0},
		{0,0,1}
	};
	memcpy(arr,(float*)temp_arr,sizeof(float) * 9);
}

void rotate_pitch_matrix(float *arr,float theta)
{
	float temp_arr[3][3] = 
	{
		{arm_cos_f32(theta),0,arm_sin_f32(theta)},
		{0,1,0},
		{-arm_sin_f32(theta),0,arm_cos_f32(theta)}
	};
	memcpy(arr,(float*)temp_arr,sizeof(float) * 9);
}

void rotate_roll_matrix(float *arr,float theta)
{
	float temp_arr[3][3] = 
	{
		{1,0,0},
		{0,arm_cos_f32(theta),-arm_sin_f32(theta)},
		{0,arm_sin_f32(theta),arm_cos_f32(theta)}
	};
	memcpy(arr,(float*)temp_arr,sizeof(float) * 9);
}

void decoupling_build_target_vector(decoupling_t* dp)
{
	rotate_yaw_matrix(dp->yaw_rotate_data,dp->vision_mea_vector[0]);
	rotate_pitch_matrix(dp->pitch_rotate_data,dp->vision_mea_vector[1]);
	//先绕yaw再pitch
	mat_mult(&dp->yaw_rotate_mat,&dp->pitch_rotate_mat,&dp->temp_vector2);
	mat_mult(&dp->temp_vector2,&dp->temp_vector1,&dp->W_target_vector);
}

void decoupling_build_rotate_matrix(decoupling_t* dp)
{
	//求云台系对基座系旋转矩阵
	rotate_yaw_matrix(dp->yaw_rotate_data,dp->motor_mea_vector[0]);
	rotate_pitch_matrix(dp->pitch_rotate_data,dp->motor_mea_vector[1]);
	mat_mult(&dp->yaw_rotate_mat,&dp->pitch_rotate_mat,&dp->G_B_Rotate_mat);
	mat_trans(&dp->G_B_Rotate_mat,&dp->temp_vector2);
	//求基座系对世界系旋转矩阵
	rotate_yaw_matrix(dp->yaw_rotate_data,dp->imu_mea_vector[0]);
	rotate_pitch_matrix(dp->pitch_rotate_data,dp->imu_mea_vector[1]);
	mat_mult(&dp->yaw_rotate_mat,&dp->pitch_rotate_mat,&dp->G_W_Rotate_mat);
	mat_mult(&dp->G_W_Rotate_mat,&dp->temp_vector2,&dp->B_W_Rotate_mat);
}

void decoupling_build_body_vector(decoupling_t* dp)
{
	//世界系目标向量投影到基座系下
	mat_trans(&dp->B_W_Rotate_mat,&dp->temp_vector2);
	mat_mult(&dp->temp_vector2,&dp->W_target_vector,&dp->B_target_vector);
}

void decoupling_build_cmd_vector(decoupling_t* dp)
{
	dp->decoupling_vector[0] = atan2f(dp->B_target_data[1],dp->B_target_data[0]);
	dp->decoupling_vector[1] = atan2f(-dp->B_target_data[2],sqrtf(powf(dp->B_target_data[0],2) + powf(dp->B_target_data[1],2)));
}

void decoupling_inverse_calc(decoupling_t* dp)
{
	decoupling_build_target_vector(dp);
	decoupling_build_rotate_matrix(dp);
	decoupling_build_body_vector(dp);
	decoupling_build_cmd_vector(dp);
}


