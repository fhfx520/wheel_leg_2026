#include "LADRC.h"
#include <math.h>

// 浮点数限幅辅助函数
static float clamp(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

void ADRC_Speed_Init(ADRC_Speed *adrc, float Ts, float b0, float wc, float wo, float u_max) {
    adrc->Ts = Ts;
    adrc->b0 = b0;
    adrc->wc = wc;
    adrc->wo = wo;
    adrc->u_max = u_max;

    // 计算控制器和观测器增益
    adrc->kp = wc;
    adrc->beta1 = 2.0f * wo;
    adrc->beta2 = wo * wo;

    // 重置状态
    adrc->z1 = 0.0f;
    adrc->z2 = 0.0f;
    adrc->u_prev = 0.0f;
    adrc->output = 0.0f;
}

void ADRC_Speed_Reset(ADRC_Speed *adrc) {
    adrc->z1 = 0.0f;
    adrc->z2 = 0.0f;
    adrc->u_prev = 0.0f;
    adrc->output = 0.0f;
}

void ADRC_Speed_SetB0(ADRC_Speed *adrc, float b0) {
    adrc->b0 = b0;
    // 注意：b0 改变不需要重新计算 beta1/beta2/kp，因为它们与 b0 无关
}

void ADRC_Speed_SetWc(ADRC_Speed *adrc, float wc) {
    adrc->wc = wc;
    adrc->kp = wc;   // 比例增益同步更新
    // 注意：wc 改变一般不需要改 wo，但实际应用中可能希望保持 wo/wc 比例
    // 如果你希望自动调整 wo 比例，可以在这里添加逻辑，但这里不做自动处理
}

float ADRC_Speed_Update(ADRC_Speed *adrc, float omega_cmd, float omega_act) {
    // 1. 观测误差: e = 估计速度 - 实际速度
    float e = adrc->z1 - omega_act;

    // 2. 扩张状态观测器 (LESO) 更新 (前向欧拉)
    float z1_dot = adrc->z2 + adrc->b0 * adrc->u_prev + adrc->beta1 * e;
    float z2_dot = adrc->beta2 * e;
    adrc->z1 += adrc->Ts * z1_dot;
    adrc->z2 += adrc->Ts * z2_dot;

    // 3. 控制律 (P 控制，基于估计速度)
    float u0 = adrc->kp * (omega_cmd - adrc->z1);

    // 4. 扰动补偿，得到最终力矩指令
    float u = (u0 - adrc->z2) / adrc->b0;

    // 5. 输出限幅
    u = clamp(u, -adrc->u_max, adrc->u_max);

    // 6. 保存当前控制量供下一周期观测器使用
    adrc->u_prev = u;
    adrc->output = u;

    return u;
}