#ifndef RL_POLICY_H
#define RL_POLICY_H

#include <stdint.h>

/* ============================================================
 * RL Policy configuration
 * ============================================================ */

#define RL_POLICY_OBS_SIZE          25U
#define RL_POLICY_OBS_HISTORY_SIZE  125U
#define RL_POLICY_ACTION_SIZE       6U


/* ============================================================
 * RL Model
 * ============================================================ */

typedef enum
{
    RL_POLICY_MODEL_STABLE = 0,
    RL_POLICY_MODEL_UPSTAIRS,
    RL_POLICY_MODEL_PIN,
    RL_POLICY_MODEL_JUMP
} RLPolicyModel_t;


/* ============================================================
 * RL Policy context
 * ============================================================ */

typedef struct
{
    uint8_t ready;
    uint32_t last_inference_us;
} RLPolicy_t;


/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 获取 RL Policy 全局实例
 */
RLPolicy_t *RLPolicy_GetInstance(void);


/**
 * @brief 初始化所有 RL 模型
 *
 * @return 1: 初始化成功
 *         0: 初始化失败
 */
uint8_t RLPolicy_Init(RLPolicy_t *policy);


/**
 * @brief 执行一次 RL 推理
 *
 * @param policy        RL Policy 实例
 * @param model         使用的模型
 * @param obs           当前观测值，25 个 float
 * @param obs_history   历史观测值，125 个 float
 * @param actions       输出动作，6 个 float
 *
 * @return 1: 推理成功
 *         0: 推理失败
 */
uint8_t RLPolicy_Run(
    RLPolicy_t *policy,
    RLPolicyModel_t model,
    const float obs[RL_POLICY_OBS_SIZE],
    const float obs_history[RL_POLICY_OBS_HISTORY_SIZE],
    float actions[RL_POLICY_ACTION_SIZE]
);


/**
 * @brief 判断 RL Policy 是否已经初始化
 */
uint8_t RLPolicy_IsReady(const RLPolicy_t *policy);


/**
 * @brief 获取上一次推理耗时
 */
uint32_t RLPolicy_GetLastInferenceUs(
    const RLPolicy_t *policy
);

#endif /* RL_POLICY_H */
