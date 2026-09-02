#include "rl_deploy.h"

#include <math.h>
#include <string.h>

#include "prot_imu.h"
#include "wlr.h"
#include "drv_dm_motor.h"
#include "drv_dji_motor.h"

#define RL_DEPLOY_INFERENCE_DIVIDER       5U
#define RL_DEPLOY_HISTORY_FRAMES          5U

#define RL_DEPLOY_PI                      3.14159265358979323846f
#define RL_DEPLOY_TWO_PI                  (2.0f * RL_DEPLOY_PI)
#define RL_DEPLOY_MIN_SIN                 1.0e-4f

/* Constants retained from the released Stable policy deployment. */
#define RL_DEPLOY_SOURCE_L1               0.21f
#define RL_DEPLOY_SOURCE_L2               0.25f
#define RL_DEPLOY_THIGH_OFFSET            0.506f
#define RL_DEPLOY_SHANK_OFFSET            1.175f

#define RL_DEPLOY_ACTION_CLIP             100.0f
#define RL_DEPLOY_POSITION_ACTION_SCALE   0.5f
#define RL_DEPLOY_VELOCITY_ACTION_SCALE   10.0f
#define RL_DEPLOY_VIRTUAL_TORQUE_LIMIT    1000.0f

enum
{
    RL_DOF_LF0 = 0,
    RL_DOF_LF1 = 1,
    RL_DOF_LW  = 2,
    RL_DOF_RF0 = 3,
    RL_DOF_RF1 = 4,
    RL_DOF_RW  = 5
};

typedef struct
{
    float phi2;
    float phi3;
    float relative_phi3;
    float relative_phi3_dot;
} RLDeployLegState_t;

RLDeployDebug_t rl_deploy_debug;

static uint8_t rl_deploy_initialized = 0U;
static uint8_t rl_inference_divider = 0U;

static const float rl_stable_default_dof_pos[RL_POLICY_ACTION_SIZE] = {
    -0.23f, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f
};

static const float rl_stable_p_gains[RL_POLICY_ACTION_SIZE] = {
    15.0f, 15.0f, 0.0f, 15.0f, 15.0f, 0.0f
};

static const float rl_stable_d_gains[RL_POLICY_ACTION_SIZE] = {
    1.0f, 1.0f, 0.1f, 1.0f, 1.0f, 0.1f
};

static float rl_clip(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static float rl_wrap_to_pi(float angle)
{
    angle = fmodf(angle, RL_DEPLOY_TWO_PI);
    if (angle > RL_DEPLOY_PI)
    {
        angle -= RL_DEPLOY_TWO_PI;
    }
    else if (angle < -RL_DEPLOY_PI)
    {
        angle += RL_DEPLOY_TWO_PI;
    }
    return angle;
}

static RLDeployLegState_t rl_solve_leg(float phi1,
                                       float phi4,
                                       float shank_velocity,
                                       float thigh_velocity,
                                       uint8_t is_right)
{
    RLDeployLegState_t state = {0.0f, 0.0f, 0.0f, 0.0f};
    const float xb = RL_DEPLOY_SOURCE_L1 * cosf(phi1);
    const float yb = RL_DEPLOY_SOURCE_L1 * sinf(phi1);
    const float xd = RL_DEPLOY_SOURCE_L1 * cosf(phi4);
    const float yd = RL_DEPLOY_SOURCE_L1 * sinf(phi4);
    const float dx = xd - xb;
    const float dy = yd - yb;
    const float a0 = 2.0f * RL_DEPLOY_SOURCE_L2 * dx;
    const float b0 = 2.0f * RL_DEPLOY_SOURCE_L2 * dy;
    const float c0 = dx * dx + dy * dy;
    float discriminant = a0 * a0 + b0 * b0 - c0 * c0;
    float sin_phi3_phi2;
    float dphi3_dphi1 = 0.0f;
    float dphi3_dphi4_relative = 0.0f;
    float xc;
    float yc;

    if (discriminant < 0.0f)
    {
        discriminant = 0.0f;
    }

    state.phi2 = 2.0f * atan2f(b0 + sqrtf(discriminant), a0 + c0);
    xc = xb + RL_DEPLOY_SOURCE_L2 * cosf(state.phi2);
    yc = yb + RL_DEPLOY_SOURCE_L2 * sinf(state.phi2);
    state.phi3 = atan2f(yc - yd, xc - xd);

    sin_phi3_phi2 = sinf(state.phi3 - state.phi2);
    if (fabsf(sin_phi3_phi2) >= RL_DEPLOY_MIN_SIN)
    {
        const float denominator = RL_DEPLOY_SOURCE_L2 * sin_phi3_phi2;
        dphi3_dphi1 = RL_DEPLOY_SOURCE_L1 * sinf(phi1 - state.phi2) / denominator;
        dphi3_dphi4_relative =
            -RL_DEPLOY_SOURCE_L1 * sinf(phi4 - state.phi2) / denominator - 1.0f;
    }

    if (is_right)
    {
        state.relative_phi3 = -state.phi3 + phi4 + 0.5f * RL_DEPLOY_PI;
    }
    else
    {
        state.relative_phi3 = state.phi3 - phi4 - 0.5f * RL_DEPLOY_PI;
    }
    state.relative_phi3 = rl_wrap_to_pi(state.relative_phi3);
    state.relative_phi3_dot =
        dphi3_dphi1 * shank_velocity + dphi3_dphi4_relative * thigh_velocity;

    return state;
}

static void rl_update_joint_state(void)
{
	// 使left_thigh是RL_DEPLOY_THIGH_OFFSET
    const float left_thigh =
        rl_wrap_to_pi(joint_motor[0].position - 2.35201263f - RL_DEPLOY_THIGH_OFFSET);
    const float left_shank =
        rl_wrap_to_pi(joint_motor[1].position - 0.542654037f + RL_DEPLOY_SHANK_OFFSET);
    const float right_thigh =
        rl_wrap_to_pi(joint_motor[2].position - 4.65292311f + RL_DEPLOY_THIGH_OFFSET);
    const float right_shank =
        rl_wrap_to_pi(joint_motor[3].position - 1.28655005f - RL_DEPLOY_SHANK_OFFSET);

    const float left_thigh_velocity = joint_motor[0].velocity;
    const float left_shank_velocity = joint_motor[1].velocity;
    const float right_thigh_velocity = joint_motor[2].velocity;
    const float right_shank_velocity = joint_motor[3].velocity;

    const RLDeployLegState_t left_leg =
        rl_solve_leg(left_shank,
                     left_thigh,
                     left_shank_velocity,
                     left_thigh_velocity,
                     0U);
    const RLDeployLegState_t right_leg =
        rl_solve_leg(-right_shank,
                     -right_thigh,
                     right_shank_velocity,
                     right_thigh_velocity,
                     1U);

    rl_deploy_debug.q[RL_DOF_LF0] = left_thigh;
    rl_deploy_debug.q[RL_DOF_LF1] = left_leg.relative_phi3;
    rl_deploy_debug.q[RL_DOF_LW] = 0.0f;
    rl_deploy_debug.q[RL_DOF_RF0] = right_thigh;
    rl_deploy_debug.q[RL_DOF_RF1] = right_leg.relative_phi3;
    rl_deploy_debug.q[RL_DOF_RW] = 0.0f;

    rl_deploy_debug.qd[RL_DOF_LF0] = left_thigh_velocity;
    rl_deploy_debug.qd[RL_DOF_LF1] = left_leg.relative_phi3_dot;
    rl_deploy_debug.qd[RL_DOF_LW] = -driver_motor[0].velocity;
    rl_deploy_debug.qd[RL_DOF_RF0] = right_thigh_velocity;
    rl_deploy_debug.qd[RL_DOF_RF1] = right_leg.relative_phi3_dot;
    rl_deploy_debug.qd[RL_DOF_RW] = -driver_motor[1].velocity;;
}

static void rl_update_projected_gravity(void)
{
    const float roll = -chassis_imu.rol;
    const float pitch = chassis_imu.pit;
    const float sin_roll = sinf(roll);
    const float cos_roll = cosf(roll);
    const float sin_pitch = sinf(pitch);
    const float cos_pitch = cosf(pitch);

    /* R^T * [0, 0, -1], using the chassis roll/pitch convention. */
    rl_deploy_debug.projected_gravity[0] = sin_pitch;
    rl_deploy_debug.projected_gravity[1] = -sin_roll * cos_pitch;
    rl_deploy_debug.projected_gravity[2] = -cos_roll * cos_pitch;
}

static void rl_build_observation(void)
{
    // 策略角
    static const float default_obs_dof_pos[4] = {
        -0.23f, -0.65f, 0.23f, 0.65f
    };
	//roll pit yaw
    const float gyro[3] = {
        -chassis_imu.wx,
        chassis_imu.wy,
        chassis_imu.wz
    };
    uint32_t index = 0U;
    uint32_t i;

    rl_deploy_debug.command[0] = wlr.v_ref * 3.0f;
    rl_deploy_debug.command[1] = wlr.wz_ref * 0.25f;
    rl_deploy_debug.command[2] = wlr.high_set * 5.0f;

    for (i = 0U; i < 3U; ++i)
    {
        rl_deploy_debug.obs[index++] = gyro[i] * 0.25f;
    }
    for (i = 0U; i < 3U; ++i)
    {
        rl_deploy_debug.obs[index++] = rl_deploy_debug.projected_gravity[i];
    }
    for (i = 0U; i < 3U; ++i)
    {
        rl_deploy_debug.obs[index++] = rl_deploy_debug.command[i];
    }

    rl_deploy_debug.obs[index++] = rl_deploy_debug.q[RL_DOF_LF0] - default_obs_dof_pos[0];
    rl_deploy_debug.obs[index++] = rl_deploy_debug.q[RL_DOF_LF1] - default_obs_dof_pos[1];
    rl_deploy_debug.obs[index++] = rl_deploy_debug.q[RL_DOF_RF0] - default_obs_dof_pos[2];
    rl_deploy_debug.obs[index++] = rl_deploy_debug.q[RL_DOF_RF1] - default_obs_dof_pos[3];

    for (i = 0U; i < RL_POLICY_ACTION_SIZE; ++i)
    {
        rl_deploy_debug.obs[index++] = rl_deploy_debug.qd[i] * 0.05f;
    }
    for (i = 0U; i < RL_POLICY_ACTION_SIZE; ++i)
    {
        rl_deploy_debug.obs[index++] = rl_deploy_debug.actions[i];
    }
}

static void rl_calculate_shadow_pd(void)
{
    uint32_t i;

    for (i = 0U; i < RL_POLICY_ACTION_SIZE; ++i)
    {
        const float action = rl_clip(rl_deploy_debug.actions[i], RL_DEPLOY_ACTION_CLIP);
        float position_offset = 0.0f;
        float velocity_target = 0.0f;
        float torque;

        if ((i == RL_DOF_LF0) || (i == RL_DOF_LF1) ||
            (i == RL_DOF_RF0) || (i == RL_DOF_RF1))
        {
            position_offset = action * RL_DEPLOY_POSITION_ACTION_SCALE;
        }
        else
        {
            velocity_target = action * RL_DEPLOY_VELOCITY_ACTION_SCALE;
        }

        rl_deploy_debug.action_clipped[i] = action;
        rl_deploy_debug.target_q[i] = rl_stable_default_dof_pos[i] + position_offset;
        rl_deploy_debug.target_qd[i] = velocity_target;

        torque =
            rl_stable_p_gains[i] *
                (rl_deploy_debug.target_q[i] - rl_deploy_debug.q[i]) +
            rl_stable_d_gains[i] *
                (rl_deploy_debug.target_qd[i] - rl_deploy_debug.qd[i]);
        rl_deploy_debug.tau_virtual[i] =
            rl_clip(torque, RL_DEPLOY_VIRTUAL_TORQUE_LIMIT);
    }
}

static void rl_update_history(void)
{
    uint32_t frame;

    if (!rl_deploy_debug.history_initialized)
    {
        for (frame = 0U; frame < RL_DEPLOY_HISTORY_FRAMES; ++frame)
        {
            memcpy(&rl_deploy_debug.obs_history[frame * RL_POLICY_OBS_SIZE],
                   rl_deploy_debug.obs,
                   sizeof(rl_deploy_debug.obs));
        }
        rl_deploy_debug.history_initialized = 1U;
        return;
    }

    memmove(rl_deploy_debug.obs_history,
            &rl_deploy_debug.obs_history[RL_POLICY_OBS_SIZE],
            sizeof(float) * (RL_POLICY_OBS_HISTORY_SIZE - RL_POLICY_OBS_SIZE));
    memcpy(&rl_deploy_debug.obs_history[RL_POLICY_OBS_HISTORY_SIZE - RL_POLICY_OBS_SIZE],
           rl_deploy_debug.obs,
           sizeof(rl_deploy_debug.obs));
}

void RLDeploy_ResetHistory(void)
{
    memset(rl_deploy_debug.obs_history, 0, sizeof(rl_deploy_debug.obs_history));
    memset(rl_deploy_debug.actions, 0, sizeof(rl_deploy_debug.actions));
    memset(rl_deploy_debug.action_clipped, 0, sizeof(rl_deploy_debug.action_clipped));
    memset(rl_deploy_debug.target_q, 0, sizeof(rl_deploy_debug.target_q));
    memset(rl_deploy_debug.target_qd, 0, sizeof(rl_deploy_debug.target_qd));
    memset(rl_deploy_debug.tau_virtual, 0, sizeof(rl_deploy_debug.tau_virtual));
    rl_deploy_debug.history_initialized = 0U;
    rl_inference_divider = 0U;
}

void RLDeploy_Init(void)
{
    if (rl_deploy_initialized)
    {
        return;
    }

    memset(&rl_deploy_debug, 0, sizeof(rl_deploy_debug));
    rl_deploy_debug.policy_ready = RLPolicy_Init(RLPolicy_GetInstance());
    rl_deploy_debug.initialized = 1U;
    rl_deploy_initialized = 1U;
}

void RLDeploy_Step500Hz(void)
{
    RLPolicy_t *policy;

    if (!rl_deploy_initialized)
    {
        RLDeploy_Init();
    }

    ++rl_deploy_debug.sample_count;
    rl_update_joint_state();
    rl_update_projected_gravity();

    ++rl_inference_divider;
    if (rl_inference_divider < RL_DEPLOY_INFERENCE_DIVIDER)
    {
        rl_calculate_shadow_pd();
        return;
    }
    rl_inference_divider = 0U;

    rl_build_observation();
    rl_update_history();

    policy = RLPolicy_GetInstance();
    rl_deploy_debug.inference_ok = RLPolicy_Run(
        policy,
        RL_POLICY_MODEL_STABLE,
        rl_deploy_debug.obs,
        rl_deploy_debug.obs_history,
        rl_deploy_debug.actions);
    rl_deploy_debug.policy_ready = RLPolicy_IsReady(policy);
    rl_deploy_debug.last_inference_us = RLPolicy_GetLastInferenceUs(policy);
    ++rl_deploy_debug.inference_count;
    if (!rl_deploy_debug.inference_ok)
    {
        ++rl_deploy_debug.inference_fail_count;
    }

    rl_calculate_shadow_pd();

    /* Shadow mode: PD torques remain debug-only and never reach motor commands. */
}
