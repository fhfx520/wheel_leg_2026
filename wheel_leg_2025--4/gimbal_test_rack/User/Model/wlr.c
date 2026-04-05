#include "wlr.h"
#include "chassis_task.h"
#include "leg_vmc.h"
#include "wheel_leg_model.h"
#include "prot_imu.h"
#include "prot_power.h"
#include "drv_dji_motor.h"
#include "prot_dr16.h"
#include "pid.h"
#include "kalman_filter.h"
#include "math_lib.h"
#include "math_matrix.h"
#include "KNN.h"


extern uint32_t rescue_cnt_L;
extern uint32_t rescue_cnt_R;

#define WLR_SIGN(x) ((x) > 0? (1): (-1))

#define CHASSIS_PERIOD_DU 2

const float LegLengthParam[2] = {0.250f, 0.300f};//大小腿长度
float mb = 22.75f;
float ml = 2.09f, mw = 0.715f;//机体质量 腿部质量 轮子质量 14.5
const float BodyWidth = 0.48f;//两轮间距
const float WheelRadius = 0.075f;//轮子半径 气胎
const float LegLengthMax = 0.40f, LegLengthMin = 0.18f;

const float LegLengthHighFly = 0.35f;//长腿腿长腾空 0.28
const float LegLengthFly = 0.20f;//正常腿长腾空
const float LegLengthHigh2 = 0.28f;//超长腿
const float LegLengthHigh = 0.24f;//长腿 0.23
const float LegLengthNormal = 0.18f;//正常

float x3_balance_zero = 0.12f, x5_balance_zero = 0.00f;//腿摆角角度偏置 机体俯仰角度偏置
float x3_fight_zero = 0.00f;
float Normal_balance_zero = 0.18f, High_balance_zero = 0.14f;//两种腿长摆角偏置

uint8_t state;
uint16_t T0_state;
float L_REF = 0.24f;
float W0_REF;
float recover_length = 0.18f;
float feed_leg;
float wlr_test_t0;
float wlr_test_fy;
//位移 速度 yaw wz 左腿摆角 左腿摆角速度 右腿摆角 右腿摆角速度 机体倾角 机体倾角速度 
//左轮转矩 右轮转矩 左腿转矩 右腿转矩
const float K_Array_Wheel[4][10] =
{{-1.57274, -3.15546, -1.2459, -0.144771, -21.6562, -3.41203, 6.88813, 0.587245, -2.08519, -0.764707}, 
{-1.57274, -3.15546, 1.2459, 0.144771, 6.88813, 0.587245, -21.6562, -3.41203, -2.08519, -0.764707}, 
{0.0727682, 
  0.137973, -1.47971, -0.727712, 9.58579, 
  1.29604, -8.82474, -1.18285, -7.44177, -2.14868}, {0.0727682, 
  0.137973, 1.47971, 0.727712, -8.82474, -1.18285, 9.58579, 
  1.29604, -7.44177, -2.14868}};

const float K_Array_Fly[4][10] = 
{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
{0, 0, 0, 0, 25.9508, 4.06359, -9.01401, -0.995014, 0, 0}, 
{0, 0, 0, 0, -9.01401, -0.995014, 25.9508, 4.06359, 0, 0}};

const float K_Array_Prone[4][10] =
{{10, 10, -10, -5, 0, 0, 0, 0, 0, 0}, 
{10, 10, 10, 5, 0, 0, 0, 0, 0, 0}, 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

float K_Array_Leg[4][10] = 
{{-1.55655, -3.14219, -1.99572, -0.138159, -13.4528, -2.31259, 1.30728, -0.0103002, -3.18659, -1.21733},
{-1.55655, -3.14219, 1.99572, 0.138159, 1.30728, -0.0103002, -13.4528, -2.31259, -3.18659, -1.21733}, \
{0.124211, 0.237821, -1.78981, -0.700304, 5.75135, 0.867569, -4.84826, -0.705913, -6.6076, -2.05861},
{0.124211, 0.237821, 1.78981, 0.700304, -4.84826, -0.705913, 5.75135,  0.867569, -6.6076, -2.05861}};  
// 0805 : 4000, 2000, 5000, 200, 20000, 500, 20000, 500, 20000, 800
float K_Array_Leg_030[4][10] = 
//{{-0.19617, -1.0652, -1.5498, -0.46736, -4.9982, -0.82762, -4.491, -0.74389, -1.4064, -0.25726},
//{-0.19617, -1.0652, 1.5498, 0.46736, -4.491, -0.74389, -4.9982, -0.82762, -1.4064, -0.25726},
//{0.681276, 3.61974, -3.79064, -1.24925, 31.3068, 4.71577, -11.3452, -1.29091, -38.2282, -2.7542},
//{0.681276, 3.61974, 3.79064, 1.24925, -11.3452, -1.29091, 31.3068, 4.71577, -38.2282, -2.7542}
//}
{{-0.56925, -1.4405, -1.4055, -0.43498, -6.0207, -0.95311, -5.2747, -0.83286, -1.2715, -0.21246},
{-0.56925, -1.4405, 1.4055, 0.43498, -5.2747, -0.83286, -6.0207, -0.95311, -1.2715, -0.21246},
{2.37229, 5.77853, -5.38309, -1.86632, 40.6899, 5.85835, -13.0458, -0.904859, -38.0731, -2.75411},
{2.37229, 5.77853, 5.38309, 1.86632, -13.0458, -0.904859, 40.6899, 5.85835, -38.0731, -2.75411}
};

 float K_Array_Leg_020[4][10] =  
//{{-1.0497, -1.4291, -0.64372, -0.29693, -5.3697, -0.92753, -4.9721, -0.81436, -1.1707, -0.23443},
//{-1.0497, -1.4291, 0.64372, 0.29693, -4.9721, -0.81436, -5.3697, -0.92753, -1.1707, -0.23443},
//{7.61433, 9.36142, -3.02651, -1.16868, 44.3472, 7.24347, -3.88754, 0.166469, -40.844, -3.27858},
//{7.61433, 9.36142, 3.02651, 1.16868, -3.88754, 0.166469, 44.3472, 7.24347, -40.844, -3.27858}
//};//没装云台前
//{{-2.78922, -4.85939, -3.58771, -0.91893, -13.3172, -2.36485, -7.5601, -1.39973, -4.76936, -1.55292}, 
//{-2.78922, -4.85939, 3.58771, 0.91893, -7.5601, -1.39973, -13.3172, -2.36485, -4.76936, -1.55292},
//{4.0807, 6.4926, -5.33503, -1.52053, 30.8647, 4.97539, -7.7491, -0.730829, -19.1712, -3.94271},
//{4.0807, 6.4926, 5.33503, 1.52053, -7.7491, -0.730829, 30.8647, 4.97539, -19.1712, -3.94271}};
//{{-0.59882, -1.2686, -2.89154, -0.3225, -10.2331, -0.77993, -4.646, -0.7126, -2.295, -0.0317},
//{-0.59882, -1.2686, 2.89154, 0.3225, -4.646, -0.7126, -10.2331, -0.77993, -2.295, -0.0317},
//{4.35022, 7.74413, -3.17977, -1.20175, 70.9728, 8.67419, -11.5223, -0.0714763, -34.4414, -4.79244},
//{4.35022, 7.74413, 3.17977, 1.20175, -11.5223, -0.0714763, 70.9728, 8.67419, -34.4414, -4.79244}
//};
{{-1.37844, -2.52876, -2.16979, -0.560691, -11.0529, -1.17598, -4.85553, -0.601282, -5.54793, -0.869989},
{-1.37844, -2.52876, 2.16979, 0.560691, -4.85553, -0.601282, -11.0529, -1.17598, -5.54793, -0.869989},
{6.16969, 10.9905, -7.00728, -1.89944, 66.874, 6.71345, -10.1126, 0.238247, -33.3638, -3.56917},
{6.16969, 10.9905, 7.00728, 1.89944, -10.1126, 0.238247, 66.874, 6.71345, -33.3638, -3.56917}
};


float K_Array_Leg_018[4][10] = 
//{{-0.56825, -1.461, -1.4371, -0.44484, -5.2815, -0.89706, -4.8312, -0.82134, -1.4749, -0.27332},
//{-0.56825, -1.461, 1.4371, 0.44484, -4.8312, -0.82134, -5.2815, -0.89706, -1.4749, -0.27332},
//{2.41804, 5.92769, -3.63422, -1.2151, 34.1083, 5.29012, -8.59389, -0.724047, -37.5237, -2.61496},
//{2.41804, 5.92769, 3.63422, 1.2151, -8.59389, -0.724047, 34.1083, 5.29012, -37.5237, -2.61496}
//}
{{-1.4151, -2.446, -2.0509, -0.52929, -9.3631, -0.81999, -3.762, -0.44196, -4.9215, -0.90387},
{-1.4151, -2.446, 2.0509, 0.52929, -3.762, -0.44196, -9.3631, -0.81999, -4.9215, -0.90387},
{3.99933, 6.62307, -5.44248, -1.52353, 35.2167, 2.76628, -7.0165, 0.0209892, -20.7844, -2.58235},
{3.99933, 6.62307, 5.44248, 1.52353, -7.0165, 0.0209892, 35.2167, 2.76628, -20.7844, -2.58235}
};

float K_Array_Leg_recover[4][10] = 
{{-0.4245, -0.81374, -0.76408, -0.27706, -5.1376, -0.4533, -2.8227, -0.27615, -2.2582, -0.46523},
{-0.4245, -0.81374, 0.76408, 0.27706, -2.8227, -0.27615, -5.1376, -0.4533, -2.2582, -0.46523},
{3.01284, 5.48211, -3.10421, -1.25049, 33.0514, 2.80351, -4.59781, 0.228048, -10.0927, -1.23066},
{3.01284, 5.48211, 3.10421, 1.25049, -4.59781, 0.228048, 33.0514, 2.80351, -10.0927, -1.23066}
};


const float K_Fit_Array[40][3][3] = 
{{{-1.50772,-0.201288,1.74873},{-0.174393,-3.76534,0},{2.48358,0,0}},{{-3.24058,-0.347737,3.66648},{1.46602,-9.05607,0},{2.57215,0,0}},{{-4.91297,-4.4151,-0.53714},{22.322,21.6979,0},{-33.8252,0,0}},{{-1.27698,-1.59112,0.0465232},{8.25481,7.83571,0},{-12.5106,0,0}},{{-2.61953,10.6752,11.36},{-81.2722,-103.514,0},{98.6671,0,0}},{{-1.21525,1.47504,1.8075},{-7.82801,-14.9352,0},{9.59973,0,0}},{{-5.83705,-16.7586,-6.51043},{59.0041,123.214,0},{-98.0578,0,0}},{{-0.952356,-3.52993,1.66281},{9.96847,8.02318,0},{-13.3617,0,0}},{{-5.75986,2.8643,3.66412},{16.2791,-28.6718,0},{2.21668,0,0}},{{-2.29619,0.953316,1.8891},{6.9863,-12.3387,0},{0.36472,0,0}},{{-1.50772,-0.174393,2.48358},{-0.201288,-3.76534,0},{1.74873,0,0}},{{-3.24058,1.46602,2.57215},{-0.347737,-9.05607,0},{3.66648,0,0}},{{4.91297,-22.322,33.8252},{4.4151,-21.6979,0},{0.53714,0,0}},{{1.27698,-8.25481,12.5106},{1.59112,-7.83571,0},{-0.0465232,0,0}},{{-5.83705,59.0041,-98.0578},{-16.7586,123.214,0},{-6.51043,0,0}},{{-0.952356,9.96847,-13.3617},{-3.52993,8.02318,0},{1.66281,0,0}},{{-2.61953,-81.2722,98.6671},{10.6752,-103.514,0},{11.36,0,0}},{{-1.21525,-7.82801,9.59973},{1.47504,-14.9352,0},{1.8075,0,0}},{{-5.75986,16.2791,2.21668},{2.8643,-28.6718,0},{3.66412,0,0}},{{-2.29619,6.9863,0.36472},{0.953316,-12.3387,0},{1.8891,0,0}},{{0.199645,2.23515,-3.25561},{-2.85191,0.0883145,0},{3.7967,0,0}},{{0.398063,4.96975,-7.30163},{-6.28643,0.106763,0},{8.6668,0,0}},{{-0.930465,-3.21701,6.30601},{-4.35736,-4.18175,0},{10.1397,0,0}},{{-0.509699,-0.932181,1.7568},{-1.25037,-1.25889,0},{3.07325,0,0}},{{1.56636,15.7418,-29.8362},{20.2154,28.388,0},{-40.6369,0,0}},{{0.663193,2.78914,-4.79139},{-0.884351,3.17267,0},{-0.243134,0,0}},{{-0.56926,-15.7434,25.1009},{-20.7429,-27.7502,0},{43.8098,0,0}},{{-0.422595,0.948891,-0.72993},{-3.4919,-3.54838,0},{6.83277,0,0}},{{-7.12249,7.53699,-7.6356},{-9.45193,-0.49529,0},{10.9732,0,0}},{{-2.32959,3.55112,-3.91312},{-4.39327,-0.217266,0},{5.39055,0,0}},{{0.199645,-2.85191,3.7967},{2.23515,0.0883145,0},{-3.25561,0,0}},{{0.398063,-6.28643,8.6668},{4.96975,0.106763,0},{-7.30163,0,0}},{{0.930465,4.35736,-10.1397},{3.21701,4.18175,0},{-6.30601,0,0}},{{0.509699,1.25037,-3.07325},{0.932181,1.25889,0},{-1.7568,0,0}},{{-0.56926,-20.7429,43.8098},{-15.7434,-27.7502,0},{25.1009,0,0}},{{-0.422595,-3.4919,6.83277},{0.948891,-3.54838,0},{-0.72993,0,0}},{{1.56636,20.2154,-40.6369},{15.7418,28.388,0},{-29.8362,0,0}},{{0.663193,-0.884351,-0.243134},{2.78914,3.17267,0},{-4.79139,0,0}},{{-7.12249,-9.45193,10.9732},{7.53699,-0.49529,0},{-7.6356,0,0}},{{-2.32959,-4.39327,5.39055},{3.55112,-0.217266,0},{-3.91312,0,0}}};

const float K_Fit_Array2[40][3][3] = 
{{{-2.01738,0.619157,0.561419},{-1.14183,-3.81284,0},{3.92777,0,0}},{{-4.169,1.22219,1.23516},{0.0687191,-9.18838,0},{4.45458,0,0}},{{-5.19993,-4.09213,-0.850394},{22.5581,22.4976,0},{-34.3481,0,0}},{{-1.29569,-1.28832,-0.277364},{7.94806,7.84303,0},{-12.0676,0,0}},{{-3.25979,11.1385,9.25514},{-82.7641,-103.138,0},{97.9959,0,0}},{{-1.36892,1.60971,1.30682},{-7.92552,-14.6136,0},{8.51295,0,0}},{{-6.48552,-17.0253,-5.67787},{56.7834,120.048,0},{-94.1846,0,0}},{{-1.1207,-3.21235,0.565515},{9.59504,7.62117,0},{-12.7821,0,0}},{{-6.26313,4.25921,1.11382},{16.4493,-27.7753,0},{1.58917,0,0}},{{-2.55444,1.65792,0.638259},{7.15658,-11.9863,0},{-0.0380284,0,0}},{{-2.01738,-1.14183,3.92777},{0.619157,-3.81284,0},{0.561419,0,0}},{{-4.169,0.0687191,4.45458},{1.22219,-9.18838,0},{1.23516,0,0}},{{5.19993,-22.5581,34.3481},{4.09213,-22.4976,0},{0.850394,0,0}},{{1.29569,-7.94806,12.0676},{1.28832,-7.84303,0},{0.277364,0,0}},{{-6.48552,56.7834,-94.1846},{-17.0253,120.048,0},{-5.67787,0,0}},{{-1.1207,9.59504,-12.7821},{-3.21235,7.62117,0},{0.565515,0,0}},{{-3.25979,-82.7641,97.9959},{11.1385,-103.138,0},{9.25514,0,0}},{{-1.36892,-7.92552,8.51295},{1.60971,-14.6136,0},{1.30682,0,0}},{{-6.26313,16.4493,1.58917},{4.25921,-27.7753,0},{1.11382,0,0}},{{-2.55444,7.15658,-0.0380284},{1.65792,-11.9863,0},{0.638259,0,0}},{{0.285877,2.49044,-3.64987},{-3.36299,0.260743,0},{4.29609,0,0}},{{0.542454,5.41621,-7.98522},{-7.17667,0.391461,0},{9.5801,0,0}},{{-0.949327,-3.46488,6.47959},{-4.60436,-3.96777,0},{10.4229,0,0}},{{-0.510705,-0.982773,1.71583},{-1.26871,-1.07161,0},{2.99343,0,0}},{{1.6327,16.3883,-30.1977},{19.9453,28.1593,0},{-40.0339,0,0}},{{0.682502,2.91383,-4.89648},{-0.996772,3.16041,0},{-0.0729002,0,0}},{{-0.468162,-15.3817,24.1509},{-21.6527,-27.342,0},{44.3754,0,0}},{{-0.404202,1.04296,-0.84007},{-3.67243,-3.5285,0},{6.98738,0,0}},{{-7.0644,7.65992,-7.7833},{-9.85481,-0.385766,0},{11.4317,0,0}},{{-2.30102,3.62527,-4.01723},{-4.60912,-0.162355,0},{5.65669,0,0}},{{0.285877,-3.36299,4.29609},{2.49044,0.260743,0},{-3.64987,0,0}},{{0.542454,-7.17667,9.5801},{5.41621,0.391461,0},{-7.98522,0,0}},{{0.949327,4.60436,-10.4229},{3.46488,3.96777,0},{-6.47959,0,0}},{{0.510705,1.26871,-2.99343},{0.982773,1.07161,0},{-1.71583,0,0}},{{-0.468162,-21.6527,44.3754},{-15.3817,-27.342,0},{24.1509,0,0}},{{-0.404202,-3.67243,6.98738},{1.04296,-3.5285,0},{-0.84007,0,0}},{{1.6327,19.9453,-40.0339},{16.3883,28.1593,0},{-30.1977,0,0}},{{0.682502,-0.996772,-0.0729002},{2.91383,3.16041,0},{-4.89648,0,0}},{{-7.0644,-9.85481,11.4317},{7.65992,-0.385766,0},{-7.7833,0,0}},{{-2.30102,-4.60912,5.65669},{3.62527,-0.162355,0},{-4.01723,0,0}}};

	
float P_Array[2][8] = 
{{15.3846, -3.6923, -0.5488, -0.6627, 0.2536, 0.1212, -0.0351, -0.0424}, 
{15.3846, 3.6923, -0.6627, -0.5488, 0.1212, 0.2536, -0.0424, -0.0351}};
    
const float P_Fit_Array[16][3][3] = 
{{{15.3846,0,0},{0,0,0},{0,0,0}},{{-3.86154,0,0},{0,0,0},{0,0,0}},{{0.0133423,0.00586344,-0.000413327},{-0.316935,-0.00202615,0},{-0.025402,0,0}},{{0.050454,-0.990208,-0.0361836},{0.00192015,-0.000677019,0},{-0.00012967,0,0}},{{0.106136,0.0767992,-0.00588455},{0.0736214,0.00376196,0},{-0.0421193,0,0}},{{-0.0508608,0.241513,-0.155217},{0.0251495,0.00104189,0},{-0.00184647,0,0}},{{-0.00132327,0.00125407,-0.0000458756},{-0.0299709,-0.00317036,0},{0.0495896,0,0}},{{-0.00165051,-0.1033,0.169188},{0.000410662,-0.00103957,0},{-0.0000143599,0,0}},{{15.3846,0,0},{0,0,0},{0,0,0}},{{3.86154,0,0},{0,0,0},{0,0,0}},{{0.050454,0.00192015,-0.00012967},{-0.990208,-0.000677019,0},{-0.0361836,0,0}},{{0.0133423,-0.316935,-0.025402},{0.00586344,-0.00202615,0},{-0.000413327,0,0}},{{-0.0508608,0.0251495,-0.00184647},{0.241513,0.00104189,0},{-0.155217,0,0}},{{0.106136,0.0736214,-0.0421193},{0.0767992,0.00376196,0},{-0.00588455,0,0}},{{-0.00165051,0.000410662,-0.0000143599},{-0.1033,-0.00103957,0},{0.169188,0,0}},{{-0.00132327,-0.0299709,0.0495896},{0.00125407,-0.00317036,0},{-0.0000458756,0,0}}};

wlr_t wlr;
lqr_t lqr;

kalman_filter_t kal_fn[2];
    
ramp_t height_ramp;

pid_t pid_leg_recover[2];
pid_t pid_leg_length[2];
pid_t pid_leg_length_fast[2];
pid_t pid_leg_length_fly[2];
pid_t pid_leg_vy[2];
pid_t pid_roll;
pid_t pid_ph0_test_0;
pid_t pid_L_test[2]; 
pid_t pid_ph0_test_1;
pid_t pid_L_test_1; 
pid_t pid_w0_test;
static float wlr_fn_calc(float az, float Fy_fdb, float T0_fdb, float L0[3], float theta[3])
{
    float Fwy = Fy_fdb * cosf(theta[0]) + T0_fdb * sinf(theta[0]) / L0[0];//轮子受到腿部机构竖直方向的作用力
    float yw_ddot = az
                    - L0[2] * cosf(theta[0])
                    + 2 * L0[1] * theta[1] * sinf(theta[0])
                    + L0[0] * theta[2] * sinf(theta[0])
                    + L0[0] * powf(theta[1], 2) * cosf(theta[0]);//轮子竖直方向的加速度
    return Fwy + mw * GRAVITY + mw * yw_ddot;
    
}

static void k_array_fit(float K[4][10], float Ll_fdb, float Lr_fdb)
{
    float temp;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 10; j++) {
            temp = 0;
            for(int x = 0; x < 3; x++)
                for(int y = 0; x + y < 3; y++)
                    temp += (K_Fit_Array[i * 10 + j][x][y] * powf(Ll_fdb, x) * powf(Lr_fdb, y));
            K[i][j] = temp;
        }
}

static void p_array_fit(float P[2][8], float Ll_fdb, float Lr_fdb)
{
    float temp;
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 8; j++) {
            temp = 0;
            for(int x = 0; x < 3; x++)
                for(int y = 0; x + y < 3; y++)
                    temp += (P_Fit_Array[i * 8 + j][x][y] * powf(Ll_fdb, x) * powf(Lr_fdb, y));
            P[i][j] = temp;
        }
}

static void state_predict(void)
{
    for (int i = 0; i < 2; i++) {
        wlr.side[i].predict_wy = 0;
        wlr.side[i].predict_wy += (P_Array[i][0] * lqr.X_fdb[1] + P_Array[i][1] * lqr.X_fdb[3] + P_Array[i][2] * lqr.X_fdb[4] + P_Array[i][3] * lqr.X_fdb[6]);
//        for (int j = 0; j < 4; j++) {
//            wlr.side[i].predict_wy -= (P_Array[i][j + 4] * wlr.side[i].Tw);
//        }   
        wlr.side[i].predict_wy = P_Array[i][4] * wlr.side[0].Tw + P_Array[i][5] * wlr.side[1].Tw \
                               + P_Array[i][6] * wlr.side[0].T0 + P_Array[i][7] * wlr.side[1].T0;//待校对极性 
    }
}

void wlr_init(void)
{
	wlr.high_set = LegLengthNormal;
	wlr.K_adapt = 0.1f;
    wlr.recover_length = 0.20f;
    
	twm_init(&twm, BodyWidth, WheelRadius);
	tlm_init(&tlm, LegLengthMax, LegLengthMin, BodyWidth);
    
    ramp_init(&height_ramp, 0.001f, 0.1f, 0.3f);
	for(int i = 0; i < 2; i++) {
		//腿部长度初始化
		vmc_init(&vmc[i], LegLengthParam);
		//卡尔曼滤波器初始化
        kalman_filter_init(&kal_fn[i], 1, 0, 1);
        kal_fn[i].A_data[0] = 1;
        kal_fn[i].H_data[0] = 1;
        kal_fn[i].Q_data[0] = 1;
        kal_fn[i].R_data[0] = 100;
        
		//PID参数初始化      
//        pid_init(&pid_leg_recover[i], NONE, 2000, 1.0f, 1000.0f, 0, 200);//起身专用pid
		pid_init(&pid_leg_recover[i], NONE, 1000, 1.0f, 30000.0f, 50, 100);//起身专用pid
        pid_init(&pid_leg_length[i], NONE, 1500, 1.0f,  0.0f, 50, 100);//500 0/2.5f 10000
        pid_init(&pid_leg_length_fast[i], NONE, 100, 0,0, 0, 20);
        pid_init(&pid_leg_length_fly[i], NONE, 1000, 0.0, 20000, 0, 100);
        pid_init(&pid_leg_vy[i], NONE, 5, 0, 0, 0, 50);
        pid_init(&pid_ph0_test_0, NONE, 30, 0, 1000, 0, 20);
        pid_init(&pid_L_test[i], NONE, 800, 2.0, 30000, 10, 100);
        
        pid_init(&pid_ph0_test_1, NONE, 30, 0, 5000, 0, 20);
        pid_init(&pid_L_test_1, NONE, 2000, 1.0, 10000, 20, 100);
        
        pid_init(&pid_w0_test, NONE, 0, 0, 0, 0, 30);
      
        
	}
	//卡尔曼滤波器初始化

	//PID参数初始化
        pid_init(&pid_roll, NONE, 300, 0, 4000, 0, 50);
	//pid_init(&pid_roll, NONE, 1500, 0, 3000, 0, 100);//与VMC的腿长控制协同  1000 0 3500
}

void wlr_protest(void)
{
	pid_leg_length[0].i_out = 0;
	pid_leg_length[1].i_out = 0;
    height_ramp.out = 0.1f;
    wlr.s_ref = wlr.s_fdb;
	wlr.s_adapt = wlr.s_fdb;
}

float pid_p = 1000.0f, pid_i = 1.0f, pid_d = 10000.0f;
float pid_p_fly = 3000.0f,pid_d_fly = 80000.0f;

//轮子：位移、速度   摆角：角度、角速度   机体俯仰：角度、角速度
void wlr_control(void)
{
    //------------------------反馈数据更新------------------------//
    wlr.s_fdb = (wlr.side[0].qy * WheelRadius + wlr.side[1].qy * WheelRadius)/2.0f;
    wlr.v_fdb = (wlr.side[0].wy * WheelRadius + wlr.side[1].wy * WheelRadius)/2.0f;
    
//    if (fabs(wlr.v_fdb) > fabs(wlr.v_ref))//加强超速控制
//        wlr.v_ref = data_fusion(wlr.v_ref, 0, fabs(wlr.v_fdb - wlr.v_ref));
    //两侧轮腿分别更新数据
	for(int i = 0; i < 2; i++) {
		//更新腿部VMC模型
		vmc_forward_solution(&vmc[i], wlr.side[i].q1, wlr.side[i].q2, wlr.side[i].w1, \
									  wlr.side[i].w2, wlr.side[i].t1, wlr.side[i].t2);
        //更新预测补偿力矩 用上一个时刻所预测的状态来补偿
        wlr.side[i].T_adapt = wlr.K_adapt * (wlr.side[i].predict_wy + wlr.side[i].wy);
    }
    lqr.X_fdb[0] = wlr.s_fdb;
    lqr.X_fdb[1] = wlr.v_fdb;
    lqr.X_fdb[2] = -wlr.yaw_fdb;
    lqr.X_fdb[3] = -wlr.wz_fdb;
    //机体
    lqr.X_fdb[8] = x5_balance_zero + wlr.pit_fdb;
    lqr.X_fdb[9] = wlr.wy_fdb;
    //左腿
    lqr.X_fdb[4] = x3_balance_zero + (-PI / 2 + lqr.X_fdb[8] + vmc[0].q_fdb[0]);
    lqr.X_fdb[5] = lqr.X_fdb[9] + vmc[0].V_fdb.e.vw0_fdb;
    lqr.dot_leg_w[0] = (lqr.X_fdb[5] - lqr.last_leg_w[0]) / 0.002f;
    lqr.last_leg_w[0] = lqr.X_fdb[5];
    //右腿
    lqr.X_fdb[6] = x3_balance_zero + (-PI / 2 + lqr.X_fdb[8] + vmc[1].q_fdb[0]);
    lqr.X_fdb[7] = lqr.X_fdb[9] + vmc[1].V_fdb.e.vw0_fdb;
    lqr.dot_leg_w[1] = (lqr.X_fdb[7] - lqr.last_leg_w[1]) / 0.002f;
    lqr.last_leg_w[1] = lqr.X_fdb[7];
    
    
    
    //支持力解算
    for(int i = 0; i < 2; i++) {
		float L0_array[3] = {vmc[i].L_fdb, vmc[i].V_fdb.e.vy0_fdb, vmc[i].Acc_fdb.L0_ddot};
		float theta_array[3] = {lqr.X_fdb[4+2*i], lqr.X_fdb[5+2*i], lqr.dot_leg_w[i]};
		wlr.side[i].Fn_fdb = wlr_fn_calc(wlr.az_fdb, vmc[i].F_fdb.e.Fy_fdb, vmc[i].F_fdb.e.T0_fdb, L0_array, theta_array);
        kal_fn[i].measured_vector[0] = wlr.side[i].Fn_fdb;
        kalman_filter_update(&kal_fn[i]);
        wlr.side[i].Fn_kal = kal_fn[i].filter_vector[0];
//		//离地检测
       
        
        if (wlr.high_flag != 2) {
           if(wlr.side[i].Fn_kal < 8.0f  )
//           if( knn_classify(input_data, 8))
                wlr.side[i].fly_cnt++;
            else if(wlr.side[i].fly_cnt > 0)
                wlr.side[i].fly_cnt-=2;
            if(wlr.side[i].fly_cnt > 30) {
                wlr.side[i].fly_cnt = 30;
                wlr.side[i].fly_flag = 1;//离地标志位
            } else if(wlr.side[i].fly_cnt == 0)
                wlr.side[i].fly_flag = 0;
            
//            if(wlr.side[i].fly_flag)
//            wlr.side[i].fly_adapt = 1;//累计腾空250ms
//            else if(!wlr.side[i].fly_flag)
//            wlr.side[i].fly_adapt = 0 ;
//            
//             if(wlr.side[i].fly_adapt == 1)//增益落地500ms
//             {
//               wlr.side[i].fly_flag_cnt = 100; 
//             }
//             else if(wlr.side[i].fly_adapt == 0 && wlr.side[i].fly_flag_cnt>0)
//                wlr.side[i].fly_flag_cnt--;    
        }      
        else {
            wlr.side[i].fly_flag = 0;
            wlr.side[i].fly_cnt = 0;
        }
    }
    
//    state = knn_classify(input_data, 25);
//    wlr.side[0].fly_flag =  state;
//    wlr.side[1].fly_flag =  state;
	//高度选择
    if (wlr.power_flag == 0) {
        if (fabs(wlr.v_ref) > fabs(wlr.v_fdb)) {
            data_limit(&wlr.v_ref,wlr.v_fdb-1.0f,wlr.v_fdb+1.0f);
        }
    }
	if (wlr.high_flag == 2) { //站高高
        
        wlr.high_set = ramp_calc(&height_ramp, LegLengthHigh2);
//		  x3_balance_zero = 0.20f;	
    } else if (wlr.high_flag == 1) { //高
        if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {
//            wlr.high_set = 0.35f;
            wlr.high_set = ramp_calc(&height_ramp, LegLengthHighFly);
        } else {
            wlr.high_set = ramp_calc(&height_ramp, LegLengthHigh);
        } 
        x3_balance_zero = High_balance_zero;
//        L_REF = 0.18f;  
//				x3_balance_zero = 0.20f;			
    } else { //正常腿长
        if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {
            wlr.high_set = ramp_calc(&height_ramp, LegLengthFly);
        } else {
            wlr.high_set = ramp_calc(&height_ramp, LegLengthNormal);
        }
//        L_REF = 0.18f;
		x3_balance_zero = Normal_balance_zero;	
    }
    //上台阶
    if (wlr.jump_flag == 1) {  
        wlr.high_set = 0.30f;
        wlr.v_ref = -1.0f;
        if (fabs(chassis_imu.pit) > 0.20f && fabs(lqr.X_fdb[4]) > 0.10f && fabs(lqr.X_fdb[6]) > 0.10f)
            wlr.jump_flag = 2;
    } else if (wlr.jump_flag == 2) { 
        wlr.high_set = 0.15f;
        if (fabs(wlr.high_set - vmc[0].L_fdb) < 0.02f && fabs(wlr.high_set - vmc[1].L_fdb) < 0.02f)
            wlr.jump_flag = 3;
    } else if (wlr.jump_flag == 3) {
        wlr.high_set = 0.18f;
        wlr.v_ref = -wlr.v_fdb;
        if (fabs(wlr.high_set - vmc[0].L_fdb) < 0.02f && fabs(wlr.high_set - vmc[1].L_fdb) < 0.02f)
            wlr.jump_flag = 4;
    } else if(wlr.jump_flag == 4) { //落地
            wlr.high_set = 0.20f;
            wlr.jump_cnt++;
            wlr.v_ref = -wlr.v_fdb;
        if (wlr.jump_cnt > 500) {
            wlr.jump_cnt = 0;
            wlr.jump_flag = 5;
        }
    } 
    //更新两腿模型
	tlm_gnd_roll_calc(&tlm, -wlr.roll_fdb, vmc[0].L_fdb, vmc[1].L_fdb);//计算地形倾角
//    if (wlr.jump_flag != 0 || (wlr.side[0].fly_flag && wlr.side[1].fly_flag))
    if ((wlr.side[0].fly_flag && wlr.side[1].fly_flag))
		tlm.l_ref[0] = tlm.l_ref[1] = wlr.high_set;
	else
        tlm_leg_length_cacl(&tlm, wlr.high_set, 0);//计算腿长设定值

	//------------------------状态选择------------------------//
	//根据当前状态选择合适的控制矩阵
    if (wlr.ctrl_mode == 2) {//力控
        if (wlr.prone_flag && 0) {
            aMartix_Cover(lqr.K, (float*)K_Array_Prone, 4, 10);
        } else if (wlr.side[0].fly_flag && wlr.side[1].fly_flag && wlr.jump_flag == 0 && !chassis.recover_flag) {//腾空
            aMartix_Cover(lqr.K, (float*)K_Array_Fly, 4, 10);
        } 
 		  else if (chassis.recover_flag > 1) {
            aMartix_Cover(lqr.K, (float*)K_Array_Leg_recover, 4, 10);
			if(fabs(lqr.X_fdb[4]) < Normal_balance_zero && fabs(lqr.X_fdb[6]) < Normal_balance_zero )
                chassis.recover_flag ++;
            if (chassis.recover_flag > 200)
				chassis.recover_flag = 0; //起立完成						
        } 	
		  else if (wlr.jump_flag == 1 || wlr.jump_flag == 2) {
//            k_array_fit(K_Array_Leg, vmc[0].L_fdb, vmc[1].L_fdb);
//            aMartix_Cover(lqr.K, (float*)K_Array_Leg, 4, 10);
            aMartix_Cover(lqr.K, (float*)K_Array_Leg_030, 4, 10);
        } else if (wlr.high_flag == 2) 
            aMartix_Cover(lqr.K, (float*)K_Array_Leg_030, 4, 10);			
          else if (wlr.high_flag == 0)
            aMartix_Cover(lqr.K, (float*)K_Array_Leg_018, 4, 10);		
          else
            aMartix_Cover(lqr.K, (float*)K_Array_Leg_020, 4, 10);    
    } else if (wlr.ctrl_mode == 1) {//位控
        if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {//腾空
            aMartix_Cover(lqr.K, (float*)K_Array_Fly, 4, 10);
        } else {
            aMartix_Cover(lqr.K, (float*)K_Array_Wheel, 4, 10);
        }
    }
	//------------------------控制数据更新------------------------//
	//全身运动控制
	wlr.roll_offs = pid_calc(&pid_roll, 0, wlr.roll_fdb);
    wlr.inertial_offs = (mb/2) * wlr.high_set * lqr.X_fdb[3] * lqr.X_fdb[1] / (BodyWidth/2) / 2;//惯性力补偿
    if(fabs(wlr.v_ref) < 1e-3 && (wlr.jump_flag == 0 || wlr.jump_flag == 5))//|| wlr.jump_flag == 4))// || !wlr.jump_flag == 4))
    {
        lqr.X_ref[0] = wlr.s_ref = wlr.s_adapt;
    } else {
        lqr.X_ref[0] = wlr.s_ref = wlr.s_fdb;
        wlr.s_adapt = wlr.s_fdb;
    }
    if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {
        lqr.X_ref[0] = wlr.s_ref = wlr.s_adapt = wlr.s_fdb;
    }
	if (wlr.side[0].fly_flag || wlr.side[1].fly_flag) //腾空
		wlr.v_ref = 0;
    
    lqr.X_ref[1] = wlr.v_ref;
    lqr.X_ref[2] = -wlr.yaw_ref;
    lqr.X_ref[3] = -wlr.wz_ref;
    
    aMartix_Add(1, lqr.X_ref, -1, lqr.X_fdb, lqr.X_diff, 10, 1);
    data_limit(&lqr.X_diff[1], -2.0f, 2.0f); //<! 速度窗口
    aMartix_Mul(lqr.K, lqr.X_diff, lqr.U_ref, 4, 10, 1);
    
    //预测下一个时刻的状态
    p_array_fit(P_Array, vmc[0].L_fdb, vmc[1].L_fdb);
    state_predict();
	//虚拟力映射
	for (int i = 0; i < 2; i++) {
        if (wlr.side[0].fly_flag && wlr.side[1].fly_flag && wlr.jump_flag==0 && !chassis.recover_flag) {
            wlr.side[i].Fy = pid_calc(&pid_leg_length_fly[i], tlm.l_ref[i], vmc[i].L_fdb) ;
		} else if (wlr.jump_flag == 1 && 0){
            wlr.side[i].Fy = pid_calc(&pid_leg_length[i], tlm.l_ref[i], vmc[i].L_fdb)
                                 + 24.0f + WLR_SIGN(i) * (wlr.roll_offs - wlr.inertial_offs);
        } else if (wlr.jump_flag == 2){
            wlr.side[i].Fy = pid_calc(&pid_leg_length_fast[i], tlm.l_ref[i], vmc[i].L_fdb); 
            lqr.U_ref[i] *= 0.1;           
		} else if (wlr.jump_flag == 3 && 0){		
            wlr.side[i].Fy =  pid_calc(&pid_leg_length[i], tlm.l_ref[i], vmc[i].L_fdb);            
		} else if (wlr.jump_flag == 4 && 0){
			wlr.side[i].Fy = pid_calc(&pid_leg_length[i], tlm.l_ref[i], vmc[i].L_fdb)
                                 + 24.0f + WLR_SIGN(i) * (wlr.roll_offs - wlr.inertial_offs)+pid_calc(&pid_leg_vy[i], 0.0f, vmc[i].V_fdb.e.vy0_fdb);		        
		} else if (chassis.recover_flag == 1){
			wlr.side[i].Fy = pid_calc(&pid_leg_recover[i], wlr.recover_length, vmc[i].L_fdb) ;  
//            wlr.side[i].Fy = pid_calc(&pid_leg_length[i],0.18f, vmc[i].L_fdb);  
		} else if (wlr.high_flag == 0){
			wlr.side[i].Fy = pid_calc(&pid_L_test[i],tlm.l_ref[i], vmc[i].L_fdb) + 5.0f + WLR_SIGN(i) * (wlr.roll_offs - wlr.inertial_offs);  
		}
		else{         
            wlr.side[i].Fy = pid_calc(&pid_L_test[i],tlm.l_ref[i], vmc[i].L_fdb) + 25.0f + WLR_SIGN(i) * (wlr.roll_offs - wlr.inertial_offs)+pid_calc(&pid_leg_vy[i], 0.0f, vmc[i].V_fdb.e.vy0_fdb);  
//            wlr.side[i].Fy = pid_calc(&pid_L_test[i],L_REF, vmc[i].L_fdb) + 25.0f + WLR_SIGN(i) * (wlr.roll_offs - wlr.inertial_offs)+pid_calc(&pid_leg_vy[i], 0.0f, vmc[i].V_fdb.e.vy0_fdb);  
        }
         if(chassis.recover_flag == 1) 
            wlr.side[i].T0 = 0;   
         else   
            wlr.side[i].T0 = lqr.U_ref[2+i];


				 
		 vmc_inverse_solution(&vmc[i], wlr.high_set, PI / 2 + x3_balance_zero, wlr.side[i].T0, wlr.side[i].Fy);
				 
	}
    //------------------------控制数据输出------------------------//
    for (int i = 0; i < 2; i++) {
        wlr.side[i].T1 =  vmc[i].T_ref.e.T1_ref ;
        wlr.side[i].T2 =  vmc[i].T_ref.e.T2_ref ;
        wlr.side[i].Tw =  lqr.U_ref[i];// - 0.2f * wlr.side[i].T_adapt;    
        wlr.side[i].P1 =  vmc[i].q_ref[1];
        wlr.side[i].P2 =  vmc[i].q_ref[2];
    }
}
