#ifndef RL_DEPLOY_H
#define RL_DEPLOY_H

#include <stdint.h>

#include "rl_policy_design.h"

#define AI_CONTROL

/*
 * RL shadow deployment:
 * - samples the chassis state at 500 Hz;
 * - runs the Stable policy at 100 Hz;
 * - exposes observations/actions for logging and debugger inspection;
 * - never writes motor commands.
 */

typedef struct
{
    uint8_t initialized;
    uint8_t policy_ready;
    uint8_t inference_ok;
    uint8_t history_initialized;

    uint32_t sample_count;
    uint32_t inference_count;
    uint32_t inference_fail_count;
    uint32_t last_inference_us;

    float command[3];
    float projected_gravity[3];
    float q[RL_POLICY_ACTION_SIZE];
    float qd[RL_POLICY_ACTION_SIZE];
    float obs[RL_POLICY_OBS_SIZE];
    float obs_history[RL_POLICY_OBS_HISTORY_SIZE];
    float actions[RL_POLICY_ACTION_SIZE];
    float action_clipped[RL_POLICY_ACTION_SIZE];
    float target_q[RL_POLICY_ACTION_SIZE];
    float target_qd[RL_POLICY_ACTION_SIZE];
    /* Policy-coordinate torques: [lf0, lf1_virtual, lw, rf0, rf1_virtual, rw]. */
    float tau_virtual[RL_POLICY_ACTION_SIZE];
    /* Five-bar Jacobian: [left_shank, left_thigh, right_shank, right_thigh]. */
    float leg_jacobian[4];
    float leg_length[2];
    float force_map_det[2];
    uint8_t force_map_valid[2];
    /* Physical motor order: [left_thigh, left_shank, left_wheel,
       right_thigh, right_shank, right_wheel]. */
    float tau_motor_raw[RL_POLICY_ACTION_SIZE];
    float tau_motor_shadow[RL_POLICY_ACTION_SIZE];
} RLDeployDebug_t;

extern RLDeployDebug_t rl_deploy_debug;

void RLDeploy_Init(void);
void RLDeploy_Step500Hz(void);
void RLDeploy_ResetHistory(void);

#endif /* RL_DEPLOY_H */
