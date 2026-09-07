#ifndef RL_DEPLOY_H
#define RL_DEPLOY_H

#include <stdint.h>

#include "rl_policy_design.h"

#define AI_CONTROL

typedef enum
{
    RL_DEPLOY_NUMERIC_FAULT_NONE = 0,
    RL_DEPLOY_NUMERIC_FAULT_JOINT_STATE = 1,
    RL_DEPLOY_NUMERIC_FAULT_IMU_STATE = 2,
    RL_DEPLOY_NUMERIC_FAULT_OBSERVATION = 3,
    RL_DEPLOY_NUMERIC_FAULT_HISTORY = 4,
    RL_DEPLOY_NUMERIC_FAULT_POLICY_OUTPUT = 5,
    RL_DEPLOY_NUMERIC_FAULT_CONTROL_TORQUE = 6
} RLDeployNumericFaultStage_t;

/*
 * RL shadow deployment:
 * - samples the chassis state at 500 Hz;
 * - runs the selected policy at 100 Hz;
 * - exposes observations/actions for logging and debugger inspection;
 * - never writes motor commands.
 */

typedef struct
{
    uint8_t initialized;
    uint8_t policy_ready;
    uint8_t inference_ok;
    uint8_t history_initialized;
    uint8_t numeric_fault;
    uint8_t numeric_fault_stage;
    uint8_t numeric_valid_streak;
    uint8_t requested_model;
    uint8_t active_model;
    uint8_t keyboard_normal_model;
    uint8_t keyboard_jump_phase;
    uint16_t keyboard_jump_cycles;

    uint32_t sample_count;
    uint32_t inference_count;
    uint32_t inference_fail_count;
    uint32_t numeric_fault_count;
    uint32_t model_switch_count;
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

/*
 * Keil Watch can write this variable directly:
 * 0=Stable, 1=Upstairs, 2=Spin, 3=Jump.
 * The new selection is applied safely in the next 500 Hz step.
 * Remote control: while the left switch is UP (protection mode), push the
 * ch5 dial above +500 for the next model or below -500 for the previous one,
 * then release it back to centre before selecting again.
 * Keyboard control follows the existing chassis FSM: R/LOW_SPIN selects Spin;
 * Z/ASCEND automatically runs Upstairs crouch -> Jump -> Upstairs; otherwise
 * wheel up selects Upstairs and wheel down selects Stable.
 */
extern volatile RLPolicyModel_t rl_deploy_model_select;

void RLDeploy_Init(void);
void RLDeploy_Step500Hz(void);
void RLDeploy_ResetHistory(void);
uint8_t RLDeploy_SetModel(RLPolicyModel_t model);
RLPolicyModel_t RLDeploy_GetModel(void);

#endif /* RL_DEPLOY_H */
