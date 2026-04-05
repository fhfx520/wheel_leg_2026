#ifndef ADRC_SPEED_H
#define ADRC_SPEED_H

#include <stdint.h>

// ADRC 速度环控制对象
typedef struct {
    float Ts;           // 采样周期 (s)
    float b0;           // 控制增益估计 (rad/s² per 控制量单位)
    float wc;           // 控制器带宽 (rad/s)
    float wo;           // 观测器带宽 (rad/s)
    float u_max;        // 最大输出力矩限幅 (与控制量单位一致)

    // 内部计算参数
    float kp;           // 比例增益 = wc
    float beta1;        // 观测器增益1 = 2 * wo
    float beta2;        // 观测器增益2 = wo * wo

    // 状态变量
    float z1;           // 估计速度 (rad/s)
    float z2;           // 估计总扰动 (rad/s²)
    float u_prev;       // 上一周期控制量

    // 输出
    float output;       // 当前周期控制量 (力矩)
} ADRC_Speed;

// 初始化 ADRC 速度环
// Ts: 采样周期 (秒)，例如 0.001
// b0: 控制增益估计，通过开环实验测得
// wc: 控制器带宽 (rad/s)，决定响应速度
// wo: 观测器带宽 (rad/s)，通常 3~10 倍 wc
// u_max: 输出力矩限幅 (绝对值)
void ADRC_Speed_Init(ADRC_Speed *adrc, float Ts, float b0, float wc, float wo, float u_max);

// 重置观测器状态 (例如云台掉电重启后)
void ADRC_Speed_Reset(ADRC_Speed *adrc);

// 速度环核心更新函数
// omega_cmd: 期望速度 (rad/s) - 来自上位机 MPC 速度 + 外环位置修正
// omega_act: 实际速度 (rad/s) - 来自编码器差分或速度传感器
// 返回值: 力矩指令 (与 b0 中的控制量单位一致)
float ADRC_Speed_Update(ADRC_Speed *adrc, float omega_cmd, float omega_act);

// 可选：动态修改 b0 (用于负载惯量变化时在线调整)
void ADRC_Speed_SetB0(ADRC_Speed *adrc, float b0);

// 可选：动态修改 wc (改变响应速度)
void ADRC_Speed_SetWc(ADRC_Speed *adrc, float wc);

#endif // ADRC_SPEED_H