#include "rl_deploy.h"

#include <math.h>
#include <string.h>

#include "prot_imu.h"
#include "wlr.h"
#include "drv_dm_motor.h"
#include "drv_dji_motor.h"
#include "robot_logic.h"
#include "math_lib.h"

#define RL_DEPLOY_INFERENCE_DIVIDER       5U
#define RL_DEPLOY_HISTORY_FRAMES          5U
#define RL_DEPLOY_FAULT_RECOVERY_RUNS     3U
#define RL_DEPLOY_JUMP_CROUCH_CYCLES       750U
#define RL_DEPLOY_JUMP_ACTIVE_CYCLES       240U
#define RL_DEPLOY_JUMP_HEIGHT              0.05f
#define RL_DEPLOY_NORMAL_HEIGHT            0.16f

#define RL_DEPLOY_PI                      3.14159265358979323846f
#define RL_DEPLOY_TWO_PI                  (2.0f * RL_DEPLOY_PI)
#define RL_DEPLOY_MIN_SIN                 1.0e-4f
#define RL_DEPLOY_MIN_LEG_LENGTH          1.0e-2f
#define RL_DEPLOY_MIN_FORCE_MAP_DET       1.0e-6f

/* Constants retained from the released Stable policy deployment. */
#define RL_DEPLOY_SOURCE_L1               0.21f
#define RL_DEPLOY_SOURCE_L2               0.25f
#define RL_DEPLOY_THIGH_OFFSET            0.506f
#define RL_DEPLOY_SHANK_OFFSET            1.175f

#define RL_DEPLOY_ACTION_CLIP             100.0f
#define RL_DEPLOY_POSITION_ACTION_SCALE   0.5f
#define RL_DEPLOY_VELOCITY_ACTION_SCALE   10.0f
#define RL_DEPLOY_VIRTUAL_TORQUE_LIMIT    1000.0f
#define RL_DEPLOY_REAL_TORQUE_LIMIT       100.0f
#define RL_DEPLOY_PARALLEL_TORQUE_LIMIT   35.0f
#define RL_DEPLOY_WHEEL_TORQUE_LIMIT      5.0f
#define RL_DEPLOY_LEFT_GAS_SPRING_K       600.1f
#define RL_DEPLOY_RIGHT_GAS_SPRING_K      600.1f

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
    float j11;
    float j12;
    float j21;
    float j22;
    float det;
    uint8_t valid;
} RLDeployForceTorqueMap_t;

typedef struct
{
    float phi2;
    float phi3;
    float phi0;
    float l0;
    float relative_phi3;
    float relative_phi3_dot;
    float jacobian_shank;
    float jacobian_thigh;
    RLDeployForceTorqueMap_t force_torque_map;
} RLDeployLegState_t;

typedef struct
{
    float default_dof_pos[RL_POLICY_ACTION_SIZE];
    float default_obs_dof_pos[4];
    float p_gains[RL_POLICY_ACTION_SIZE];
    float d_gains[RL_POLICY_ACTION_SIZE];
    float command_scale[3];
} RLDeployModelParams_t;

typedef enum
{
    RL_DEPLOY_JUMP_IDLE = 0,
    RL_DEPLOY_JUMP_CROUCH,
    RL_DEPLOY_JUMP_ACTIVE,
    RL_DEPLOY_JUMP_COMPLETE
} RLDeployJumpPhase_t;

RLDeployDebug_t rl_deploy_debug;
volatile RLPolicyModel_t rl_deploy_model_select = RL_POLICY_MODEL_UPSTAIRS;

static uint8_t rl_deploy_initialized = 0U;
static uint8_t rl_inference_divider = 0U;
static uint8_t rl_model_keyboard_armed = 1U;
static RLPolicyModel_t rl_active_model = RL_POLICY_MODEL_UPSTAIRS;
static RLPolicyModel_t rl_keyboard_normal_model = RL_POLICY_MODEL_UPSTAIRS;
static RLDeployJumpPhase_t rl_keyboard_jump_phase = RL_DEPLOY_JUMP_IDLE;
static uint16_t rl_keyboard_jump_cycles = 0U;
RLDeployLegState_t rl_left_leg;
RLDeployLegState_t rl_right_leg;

static const RLDeployModelParams_t rl_model_params[] = {
    /* Stable */
    {
        {-0.23f, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f},
        {-0.23f, -0.65f, 0.23f, 0.65f},
        {15.0f, 15.0f, 0.0f, 15.0f, 15.0f, 0.0f},
        {1.0f, 1.0f, 0.1f, 1.0f, 1.0f, 0.1f},
        {3.0f, 0.25f, 5.0f}
    },
    /* Upstairs / MiniRecover */
    {
        {-0.23f, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f},
        {-0.23f, -0.65f, 0.23f, 0.65f},
        {15.0f, 15.0f, 0.0f, 15.0f, 15.0f, 0.0f},
        {1.0f, 1.0f, 0.1f, 1.0f, 1.0f, 0.1f},
        {2.0f, 0.25f, 5.0f}
    },
    /* Spin (the public enum retains the open-source name Pin). */
    {
        {-0.23f, -0.65f, 0.0f, 0.23f, 0.65f, 0.0f},
        {-0.23f, -0.65f, 0.23f, 0.65f},
        {10.0f, 10.0f, 0.0f, 10.0f, 10.0f, 0.0f},
        {1.0f, 1.0f, 0.1f, 1.0f, 1.0f, 0.1f},
        {2.0f, 0.25f, 5.0f}
    },
    /* Jump */
    {
        {0.2f, 0.4f, 0.0f, -0.2f, -0.4f, 0.0f},
        {0.2f, 0.4f, -0.2f, -0.4f},
        {6.0f, 6.0f, 0.0f, 6.0f, 6.0f, 0.0f},
        {0.5f, 0.5f, 0.2f, 0.5f, 0.5f, 0.2f},
        {3.0f, 0.25f, 5.0f}
    }
};

static uint8_t rl_model_is_valid(RLPolicyModel_t model)
{
    return (uint8_t)((model >= RL_POLICY_MODEL_STABLE) &&
                     (model <= RL_POLICY_MODEL_JUMP));
}

static const RLDeployModelParams_t *rl_get_model_params(void)
{
    return &rl_model_params[(uint32_t)rl_active_model];
}

static void rl_update_remote_model_selection(void)
{
    if ((!g_robot_ctx.is_online) ||
        (g_robot_ctx.output.top_mode != TOP_MODE_REMOTE))
    {
        return;
    }

    /* Follow the existing remote-control chassis FSM automatically. */
    if (g_robot_ctx.output.chassis == CHASSIS_LOW_SPIN)
    {
        (void)RLDeploy_SetModel(RL_POLICY_MODEL_PIN);
    }
    else if (g_robot_ctx.output.chassis == CHASSIS_ASCEND)
    {
        (void)RLDeploy_SetModel(RL_POLICY_MODEL_UPSTAIRS);
    }
	else if (g_robot_ctx.output.chassis == CHASSIS_HIGH)
    {
        (void)RLDeploy_SetModel(RL_POLICY_MODEL_STABLE);
    }
    else
    {
        (void)RLDeploy_SetModel(RL_POLICY_MODEL_UPSTAIRS);
    }
}

static void rl_update_keyboard_model_selection(void)
{
    int16_t wheel;

    wheel = g_robot_ctx.input.mouse.z;

    if ((!g_robot_ctx.is_online) ||
        (g_robot_ctx.output.top_mode != TOP_MODE_KEYBOARD))
    {
        rl_keyboard_jump_phase = RL_DEPLOY_JUMP_IDLE;
        rl_keyboard_jump_cycles = 0U;
        if (wheel == 0)
        {
            rl_model_keyboard_armed = 1U;
        }
        return;
    }

    /* In WLR torque mode, RL must not advance or finish keyboard actions. */
    if (g_robot_ctx.output.torque_source != CHASSIS_TORQUE_RL)
    {
        rl_keyboard_jump_phase = RL_DEPLOY_JUMP_IDLE;
        rl_keyboard_jump_cycles = 0U;
        if (wheel == 0)
        {
            rl_model_keyboard_armed = 1U;
        }
        return;
    }

    /* R is interpreted by the existing FSM as CHASSIS_LOW_SPIN. */
    if (g_robot_ctx.output.chassis == CHASSIS_LOW_SPIN)
    {
        rl_keyboard_jump_phase = RL_DEPLOY_JUMP_IDLE;
        rl_keyboard_jump_cycles = 0U;
        (void)RLDeploy_SetModel(RL_POLICY_MODEL_PIN);
        rl_deploy_debug.keyboard_normal_model = (uint8_t)rl_keyboard_normal_model;
        rl_deploy_debug.keyboard_jump_phase = (uint8_t)rl_keyboard_jump_phase;
        rl_deploy_debug.keyboard_jump_cycles = rl_keyboard_jump_cycles;
        return;
    }

    /*
     * Z is interpreted by the existing FSM as CHASSIS_ASCEND.  First crouch
     * for 300 ms with Upstairs, run Jump for 480 ms, then return to Upstairs
     * and notify the FSM that the automatic sequence has finished.
     */
    if (g_robot_ctx.output.chassis == CHASSIS_ASCEND)
    {
//        if (rl_keyboard_jump_phase == RL_DEPLOY_JUMP_IDLE)
//        {
//            rl_keyboard_jump_phase = RL_DEPLOY_JUMP_CROUCH;
//            rl_keyboard_jump_cycles = 0U;
//            g_robot_ctx.jump_finish_flag = 0U;
//            (void)RLDeploy_SetModel(RL_POLICY_MODEL_UPSTAIRS);
//        }
//        else if (rl_keyboard_jump_phase == RL_DEPLOY_JUMP_CROUCH)
//        {
//            ++rl_keyboard_jump_cycles;
//            if (rl_keyboard_jump_cycles >= RL_DEPLOY_JUMP_CROUCH_CYCLES)
//            {
//                rl_keyboard_jump_phase = RL_DEPLOY_JUMP_ACTIVE;
//                rl_keyboard_jump_cycles = 0U;
//                (void)RLDeploy_SetModel(RL_POLICY_MODEL_JUMP);
//            }
//        }
//        else if (rl_keyboard_jump_phase == RL_DEPLOY_JUMP_ACTIVE)
//        {
//            ++rl_keyboard_jump_cycles;
//            if (rl_keyboard_jump_cycles >= RL_DEPLOY_JUMP_ACTIVE_CYCLES)
//            {
//                rl_keyboard_jump_phase = RL_DEPLOY_JUMP_COMPLETE;
//                rl_keyboard_jump_cycles = 0U;
//                rl_keyboard_normal_model = RL_POLICY_MODEL_UPSTAIRS;
//                (void)RLDeploy_SetModel(RL_POLICY_MODEL_UPSTAIRS);
//                g_robot_ctx.jump_finish_flag = 1U;
//            }
//        }
//        else
//        {
//            (void)RLDeploy_SetModel(RL_POLICY_MODEL_UPSTAIRS);
//            g_robot_ctx.jump_finish_flag = 1U;
//        }

//        rl_deploy_debug.keyboard_normal_model = (uint8_t)rl_keyboard_normal_model;
//        rl_deploy_debug.keyboard_jump_phase = (uint8_t)rl_keyboard_jump_phase;
//        rl_deploy_debug.keyboard_jump_cycles = rl_keyboard_jump_cycles;
//        return;
		  (void)RLDeploy_SetModel(RL_POLICY_MODEL_UPSTAIRS);
    }

    rl_keyboard_jump_phase = RL_DEPLOY_JUMP_IDLE;
    rl_keyboard_jump_cycles = 0U;

    if ((rl_deploy_model_select == RL_POLICY_MODEL_STABLE) ||
        (rl_deploy_model_select == RL_POLICY_MODEL_UPSTAIRS))
    {
        rl_keyboard_normal_model = rl_deploy_model_select;
    }

    if (wheel == 0)
    {
        rl_model_keyboard_armed = 1U;
    }
    else if (rl_model_keyboard_armed)
    {
        /* Wheel up selects Upstairs; wheel down selects Stable. */
        rl_keyboard_normal_model =
            (wheel > 0) ? RL_POLICY_MODEL_UPSTAIRS : RL_POLICY_MODEL_STABLE;
        rl_model_keyboard_armed = 0U;
    }

    (void)RLDeploy_SetModel(rl_keyboard_normal_model);
    rl_deploy_debug.keyboard_normal_model = (uint8_t)rl_keyboard_normal_model;
    rl_deploy_debug.keyboard_jump_phase = (uint8_t)rl_keyboard_jump_phase;
    rl_deploy_debug.keyboard_jump_cycles = rl_keyboard_jump_cycles;
}

static uint8_t rl_array_is_finite(const float *data, uint32_t size)
{
    uint32_t i;

    for (i = 0U; i < size; ++i)
    {
        if (!isfinite(data[i]))
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t rl_force_map_is_valid(const RLDeployForceTorqueMap_t *map)
{
    return (uint8_t)(
        map->valid &&
        isfinite(map->j11) &&
        isfinite(map->j12) &&
        isfinite(map->j21) &&
        isfinite(map->j22) &&
        isfinite(map->det));
}

static uint8_t rl_leg_state_is_valid(const RLDeployLegState_t *leg)
{
    return (uint8_t)(
        isfinite(leg->phi2) &&
        isfinite(leg->phi3) &&
        isfinite(leg->phi0) &&
        isfinite(leg->l0) &&
        (leg->l0 >= RL_DEPLOY_MIN_LEG_LENGTH) &&
        isfinite(leg->relative_phi3) &&
        isfinite(leg->relative_phi3_dot) &&
        isfinite(leg->jacobian_shank) &&
        isfinite(leg->jacobian_thigh) &&
        rl_force_map_is_valid(&leg->force_torque_map));
}

static void rl_zero_torque_outputs(void)
{
    memset(rl_deploy_debug.action_clipped, 0, sizeof(rl_deploy_debug.action_clipped));
    memset(rl_deploy_debug.target_q, 0, sizeof(rl_deploy_debug.target_q));
    memset(rl_deploy_debug.target_qd, 0, sizeof(rl_deploy_debug.target_qd));
    memset(rl_deploy_debug.tau_virtual, 0, sizeof(rl_deploy_debug.tau_virtual));
    memset(rl_deploy_debug.tau_motor_raw, 0, sizeof(rl_deploy_debug.tau_motor_raw));
    memset(rl_deploy_debug.tau_motor_shadow, 0, sizeof(rl_deploy_debug.tau_motor_shadow));
}

static void rl_clear_policy_runtime(void)
{
    memset(rl_deploy_debug.obs, 0, sizeof(rl_deploy_debug.obs));
    memset(rl_deploy_debug.obs_history, 0, sizeof(rl_deploy_debug.obs_history));
    memset(rl_deploy_debug.actions, 0, sizeof(rl_deploy_debug.actions));
    rl_zero_torque_outputs();
    rl_deploy_debug.history_initialized = 0U;
    rl_inference_divider = 0U;
}

static void rl_enter_numeric_fault(RLDeployNumericFaultStage_t stage)
{
    if (!rl_deploy_debug.numeric_fault)
    {
        ++rl_deploy_debug.numeric_fault_count;
        rl_deploy_debug.numeric_fault_stage = (uint8_t)stage;
    }

    rl_deploy_debug.numeric_fault = 1U;
    rl_deploy_debug.numeric_valid_streak = 0U;
    rl_deploy_debug.inference_ok = 0U;
    rl_clear_policy_runtime();
}

static void rl_note_valid_inference(void)
{
    if (rl_deploy_debug.numeric_fault)
    {
        if (rl_deploy_debug.numeric_valid_streak < RL_DEPLOY_FAULT_RECOVERY_RUNS)
        {
            ++rl_deploy_debug.numeric_valid_streak;
        }

        if (rl_deploy_debug.numeric_valid_streak >= RL_DEPLOY_FAULT_RECOVERY_RUNS)
        {
            rl_deploy_debug.numeric_fault = 0U;
        }
    }
    else
    {
        rl_deploy_debug.numeric_valid_streak = RL_DEPLOY_FAULT_RECOVERY_RUNS;
    }
}

static float rl_clip(float value, float limit)
{
    if ((!isfinite(value)) || (!isfinite(limit)) || (limit <= 0.0f))
    {
        return 0.0f;
    }

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
    RLDeployLegState_t state;
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
    float force_map_sin;
    float dphi3_dphi1 = 0.0f;
    float dphi3_dphi4_relative = 0.0f;
    float xc;
    float yc;

    memset(&state, 0, sizeof(state));

    if (discriminant < 0.0f)
    {
        discriminant = 0.0f;
    }

    state.phi2 = 2.0f * atan2f(b0 + sqrtf(discriminant), a0 + c0);
    xc = xb + RL_DEPLOY_SOURCE_L2 * cosf(state.phi2);
    yc = yb + RL_DEPLOY_SOURCE_L2 * sinf(state.phi2);
    state.phi3 = atan2f(yc - yd, xc - xd);
    state.phi0 = atan2f(yc, xc);
    state.l0 = sqrtf(xc * xc + yc * yc);

    sin_phi3_phi2 = sinf(state.phi3 - state.phi2);
    if (fabsf(sin_phi3_phi2) >= RL_DEPLOY_MIN_SIN)
    {
        const float denominator = RL_DEPLOY_SOURCE_L2 * sin_phi3_phi2;
        dphi3_dphi1 = RL_DEPLOY_SOURCE_L1 * sinf(phi1 - state.phi2) / denominator;
        dphi3_dphi4_relative =
            -RL_DEPLOY_SOURCE_L1 * sinf(phi4 - state.phi2) / denominator - 1.0f;
    }

    state.jacobian_shank = dphi3_dphi1;
    state.jacobian_thigh = dphi3_dphi4_relative;

    force_map_sin = sin_phi3_phi2;
    if (fabsf(force_map_sin) < RL_DEPLOY_MIN_SIN)
    {
        force_map_sin =
            (force_map_sin < 0.0f) ? -RL_DEPLOY_MIN_SIN : RL_DEPLOY_MIN_SIN;
    }
    {
        const float safe_l0 =
            (state.l0 > RL_DEPLOY_MIN_LEG_LENGTH) ?
            state.l0 : RL_DEPLOY_MIN_LEG_LENGTH;
        const float common_phi1 =
            RL_DEPLOY_SOURCE_L1 * sinf(phi1 - state.phi2) / force_map_sin;
        const float common_phi4 =
            RL_DEPLOY_SOURCE_L1 * sinf(state.phi3 - phi4) / force_map_sin;

        state.force_torque_map.j11 =
            sinf(state.phi0 - state.phi3) * common_phi1;
        state.force_torque_map.j12 =
            cosf(state.phi0 - state.phi3) * common_phi1 / safe_l0;
        state.force_torque_map.j21 =
            sinf(state.phi0 - state.phi2) * common_phi4;
        state.force_torque_map.j22 =
            cosf(state.phi0 - state.phi2) * common_phi4 / safe_l0;
        state.force_torque_map.det =
            state.force_torque_map.j11 * state.force_torque_map.j22 -
            state.force_torque_map.j12 * state.force_torque_map.j21;
        state.force_torque_map.valid =
            (fabsf(state.force_torque_map.det) >= RL_DEPLOY_MIN_FORCE_MAP_DET) ? 1U : 0U;
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
float left_shank_debug = 0.0f;
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
	left_shank_debug = left_shank;
    rl_left_leg =
        rl_solve_leg(left_shank,
                     left_thigh,
                     left_shank_velocity,
                     left_thigh_velocity,
                     0U);
    rl_right_leg =
        rl_solve_leg(-right_shank,
                     -right_thigh,
                     right_shank_velocity,
                     right_thigh_velocity,
                     1U);

    rl_deploy_debug.q[RL_DOF_LF0] = left_thigh;
    rl_deploy_debug.q[RL_DOF_LF1] = rl_left_leg.relative_phi3;
    rl_deploy_debug.q[RL_DOF_LW] = 0.0f;
    rl_deploy_debug.q[RL_DOF_RF0] = right_thigh;
    rl_deploy_debug.q[RL_DOF_RF1] = rl_right_leg.relative_phi3;
    rl_deploy_debug.q[RL_DOF_RW] = 0.0f;

    rl_deploy_debug.qd[RL_DOF_LF0] = left_thigh_velocity;
    rl_deploy_debug.qd[RL_DOF_LF1] = rl_left_leg.relative_phi3_dot;
    rl_deploy_debug.qd[RL_DOF_LW] = -driver_motor[0].velocity;
    rl_deploy_debug.qd[RL_DOF_RF0] = right_thigh_velocity;
    rl_deploy_debug.qd[RL_DOF_RF1] = rl_right_leg.relative_phi3_dot;
    rl_deploy_debug.qd[RL_DOF_RW] = -driver_motor[1].velocity;;

    rl_deploy_debug.leg_jacobian[0] = rl_left_leg.jacobian_shank;
    rl_deploy_debug.leg_jacobian[1] = rl_left_leg.jacobian_thigh;
    rl_deploy_debug.leg_jacobian[2] = rl_right_leg.jacobian_shank;
    rl_deploy_debug.leg_jacobian[3] = rl_right_leg.jacobian_thigh;
    rl_deploy_debug.leg_length[0] = rl_left_leg.l0;
    rl_deploy_debug.leg_length[1] = rl_right_leg.l0;
    rl_deploy_debug.force_map_det[0] = rl_left_leg.force_torque_map.det;
    rl_deploy_debug.force_map_det[1] = rl_right_leg.force_torque_map.det;
    rl_deploy_debug.force_map_valid[0] = rl_left_leg.force_torque_map.valid;
    rl_deploy_debug.force_map_valid[1] = rl_right_leg.force_torque_map.valid;
}

static uint8_t rl_joint_state_is_valid(void)
{
    return (uint8_t)(
        rl_array_is_finite(rl_deploy_debug.q, RL_POLICY_ACTION_SIZE) &&
        rl_array_is_finite(rl_deploy_debug.qd, RL_POLICY_ACTION_SIZE) &&
        rl_leg_state_is_valid(&rl_left_leg) &&
        rl_leg_state_is_valid(&rl_right_leg));
}

static uint8_t rl_imu_state_is_valid(void)
{
    return (uint8_t)(
        isfinite(chassis_imu.rol) &&
        isfinite(chassis_imu.pit) &&
        isfinite(chassis_imu.wx) &&
        isfinite(chassis_imu.wy) &&
        isfinite(chassis_imu.wz) &&
        rl_array_is_finite(rl_deploy_debug.projected_gravity, 3U));
}

static void rl_update_projected_gravity(void)
{
    const float roll = chassis_imu.rol;
    const float pitch = -chassis_imu.pit;
    const float sin_roll = sinf(roll);
    const float cos_roll = cosf(roll);
    const float sin_pitch = sinf(pitch);
    const float cos_pitch = cosf(pitch);

    /* R^T * [0, 0, -1], using the chassis roll/pitch convention. */
    rl_deploy_debug.projected_gravity[0] = sin_pitch;
    rl_deploy_debug.projected_gravity[1] = -sin_roll * cos_pitch;
    rl_deploy_debug.projected_gravity[2] = -cos_roll * cos_pitch;
}
float k = 5.0f;
static void rl_build_observation(void)
{
	const RLDeployModelParams_t *params = rl_get_model_params();
	//roll pit yaw
    const float gyro[3] = {
        chassis_imu.wx,
        -chassis_imu.wy,
        chassis_imu.wz
    };
    uint32_t index = 0U;
    uint32_t i;

    rl_deploy_debug.command[0] =
	(wlr.v_ref) * params->command_scale[0];
    if (rl_active_model == RL_POLICY_MODEL_PIN)
    {
        rl_deploy_debug.command[1] =
           9.0f * params->command_scale[1];
    }
    else
    {
        rl_deploy_debug.command[1] =
            k * circle_error(wlr.yaw_ref,wlr.yaw_fdb,2 * PI) * params->command_scale[1];
    }

    if ((rl_keyboard_jump_phase == RL_DEPLOY_JUMP_CROUCH) ||
        (rl_keyboard_jump_phase == RL_DEPLOY_JUMP_ACTIVE))
    {
        rl_deploy_debug.command[2] =
            RL_DEPLOY_JUMP_HEIGHT * params->command_scale[2];
    }
	else if (g_robot_ctx.output.chassis == CHASSIS_LOW)
    {
        rl_deploy_debug.command[2] = 0.16f * params->command_scale[2];
    }
    else if (g_robot_ctx.output.chassis == CHASSIS_HIGH)
    {
        rl_deploy_debug.command[2] = 0.18f * params->command_scale[2];
    }
    else if (g_robot_ctx.output.chassis == CHASSIS_ASCEND)
    {
        rl_deploy_debug.command[2] =
            0.3F * params->command_scale[2];
    }
    else
    {
        rl_deploy_debug.command[2] = wlr.high_set * params->command_scale[2];
    }

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

    rl_deploy_debug.obs[index++] =
        rl_deploy_debug.q[RL_DOF_LF0] - params->default_obs_dof_pos[0];
    rl_deploy_debug.obs[index++] =
        rl_deploy_debug.q[RL_DOF_LF1] - params->default_obs_dof_pos[1];
    rl_deploy_debug.obs[index++] =
        rl_deploy_debug.q[RL_DOF_RF0] - params->default_obs_dof_pos[2];
    rl_deploy_debug.obs[index++] =
        rl_deploy_debug.q[RL_DOF_RF1] - params->default_obs_dof_pos[3];

    for (i = 0U; i < RL_POLICY_ACTION_SIZE; ++i)
    {
        rl_deploy_debug.obs[index++] = rl_deploy_debug.qd[i] * 0.05f;
    }
    for (i = 0U; i < RL_POLICY_ACTION_SIZE; ++i)
    {
        rl_deploy_debug.obs[index++] = rl_deploy_debug.actions[i];
    }
}

static uint8_t rl_calculate_shadow_pd(void)
{
    const RLDeployModelParams_t *params = rl_get_model_params();
    uint32_t i;

    if (!rl_array_is_finite(rl_deploy_debug.actions, RL_POLICY_ACTION_SIZE))
    {
        return 0U;
    }

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
        rl_deploy_debug.target_q[i] = params->default_dof_pos[i] + position_offset;
        rl_deploy_debug.target_qd[i] = velocity_target;

        torque =
            params->p_gains[i] *
                (rl_deploy_debug.target_q[i] - rl_deploy_debug.q[i]) +
            params->d_gains[i] *
                (rl_deploy_debug.target_qd[i] - rl_deploy_debug.qd[i]);

        if (!isfinite(torque))
        {
            return 0U;
        }

        rl_deploy_debug.tau_virtual[i] =
            rl_clip(torque, RL_DEPLOY_VIRTUAL_TORQUE_LIMIT);
    }

    return (uint8_t)(
        rl_array_is_finite(rl_deploy_debug.action_clipped, RL_POLICY_ACTION_SIZE) &&
        rl_array_is_finite(rl_deploy_debug.target_q, RL_POLICY_ACTION_SIZE) &&
        rl_array_is_finite(rl_deploy_debug.target_qd, RL_POLICY_ACTION_SIZE) &&
        rl_array_is_finite(rl_deploy_debug.tau_virtual, RL_POLICY_ACTION_SIZE));
}

static float rl_gas_spring_calcu(uint8_t is_right)
{
	uint8_t i = 0;
	if(is_right)
		i = 1;
	else
		i = 0;
	
	vmc_forward_solution_five(&vmc[i], wlr.side[i].q2, wlr.side[i].q1, wlr.side[i].w2,
							  wlr.side[i].w1, wlr.side[i].t2, wlr.side[i].t1);
	
	return gas_spring_F_Calc(vmc[i]);
}

static void rl_apply_gas_spring_compensation(const RLDeployLegState_t *leg,
                                             float gas_spring_k,
                                             float force_sign,
                                             float *tau_thigh,
                                             float *tau_shank)
{
    const RLDeployForceTorqueMap_t *map = &leg->force_torque_map;

    if (map->valid)
    {
        float foot_force =
            (map->j22 * (*tau_shank) - map->j12 * (*tau_thigh)) / map->det;
        const float foot_torque =
            (-map->j21 * (*tau_shank) + map->j11 * (*tau_thigh)) / map->det;
		
        foot_force += force_sign * gas_spring_k;
        *tau_shank = map->j11 * foot_force + map->j12 * foot_torque;
        *tau_thigh = map->j21 * foot_force + map->j22 * foot_torque;
    }
}

static uint8_t rl_calculate_shadow_motor_torques(void)
{
    float left_thigh =
        rl_deploy_debug.tau_virtual[RL_DOF_LF0] +
        rl_deploy_debug.tau_virtual[RL_DOF_LF1] * rl_left_leg.jacobian_thigh;
    float left_shank =
        rl_deploy_debug.tau_virtual[RL_DOF_LF1] * rl_left_leg.jacobian_shank;
    float right_thigh =
        rl_deploy_debug.tau_virtual[RL_DOF_RF0] +
        rl_deploy_debug.tau_virtual[RL_DOF_RF1] * rl_right_leg.jacobian_thigh;
    float right_shank =
        rl_deploy_debug.tau_virtual[RL_DOF_RF1] * rl_right_leg.jacobian_shank;
    uint32_t i;

    /* Exact left/right signs used by the released deployment. */
    rl_apply_gas_spring_compensation(&rl_left_leg,
                                     rl_gas_spring_calcu(0),
                                     -1.0f,
                                     &left_thigh,
                                     &left_shank);
    rl_apply_gas_spring_compensation(&rl_right_leg,
                                     rl_gas_spring_calcu(1),
                                     1.0f,
                                     &right_thigh,
                                     &right_shank);

    rl_deploy_debug.tau_motor_raw[RL_DOF_LF0] = left_thigh;
    rl_deploy_debug.tau_motor_raw[RL_DOF_LF1] = left_shank;
    rl_deploy_debug.tau_motor_raw[RL_DOF_LW] =
        -rl_deploy_debug.tau_virtual[RL_DOF_LW];
    rl_deploy_debug.tau_motor_raw[RL_DOF_RF0] = right_thigh;
    rl_deploy_debug.tau_motor_raw[RL_DOF_RF1] = right_shank;
    rl_deploy_debug.tau_motor_raw[RL_DOF_RW] =
        -rl_deploy_debug.tau_virtual[RL_DOF_RW];

    for (i = 0U; i < RL_POLICY_ACTION_SIZE; ++i)
    {
        const float output_torque =
            rl_clip(rl_deploy_debug.tau_motor_raw[i], RL_DEPLOY_REAL_TORQUE_LIMIT);
        rl_deploy_debug.tau_motor_shadow[i] =
            rl_clip(output_torque,
                    ((i == RL_DOF_LW) || (i == RL_DOF_RW)) ?
                    RL_DEPLOY_WHEEL_TORQUE_LIMIT : RL_DEPLOY_PARALLEL_TORQUE_LIMIT);
    }

    return (uint8_t)(
        rl_array_is_finite(rl_deploy_debug.tau_motor_raw, RL_POLICY_ACTION_SIZE) &&
        rl_array_is_finite(rl_deploy_debug.tau_motor_shadow, RL_POLICY_ACTION_SIZE));
}

static uint8_t rl_calculate_shadow_control(void)
{
    if (!rl_calculate_shadow_pd())
    {
        return 0U;
    }

    return rl_calculate_shadow_motor_torques();
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
    rl_clear_policy_runtime();
    rl_deploy_debug.numeric_fault = 0U;
    rl_deploy_debug.numeric_fault_stage = RL_DEPLOY_NUMERIC_FAULT_NONE;
    rl_deploy_debug.numeric_valid_streak = 0U;
}

uint8_t RLDeploy_SetModel(RLPolicyModel_t model)
{
    if (!rl_model_is_valid(model))
    {
        return 0U;
    }

    rl_deploy_model_select = model;
    return 1U;
}

RLPolicyModel_t RLDeploy_GetModel(void)
{
    return rl_active_model;
}

void RLDeploy_Init(void)
{
    if (rl_deploy_initialized)
    {
        return;
    }

    memset(&rl_deploy_debug, 0, sizeof(rl_deploy_debug));
    if (rl_model_is_valid(rl_deploy_model_select))
    {
        rl_active_model = rl_deploy_model_select;
    }
    else
    {
        rl_deploy_model_select = RL_POLICY_MODEL_STABLE;
        rl_active_model = RL_POLICY_MODEL_STABLE;
    }
    rl_deploy_debug.requested_model = (uint8_t)rl_deploy_model_select;
    rl_deploy_debug.active_model = (uint8_t)rl_active_model;
    rl_deploy_debug.keyboard_normal_model = (uint8_t)rl_keyboard_normal_model;
    rl_deploy_debug.keyboard_jump_phase = (uint8_t)rl_keyboard_jump_phase;
    rl_deploy_debug.keyboard_jump_cycles = rl_keyboard_jump_cycles;
    rl_deploy_debug.policy_ready = RLPolicy_Init(RLPolicy_GetInstance());
    rl_deploy_debug.initialized = 1U;
    rl_deploy_initialized = 1U;
}

void RLDeploy_Step500Hz(void)
{
    RLPolicy_t *policy;
    RLPolicyModel_t requested_model;

    if (!rl_deploy_initialized)
    {
        RLDeploy_Init();
    }

    rl_update_remote_model_selection();
    rl_update_keyboard_model_selection();
    requested_model = rl_deploy_model_select;
    rl_deploy_debug.requested_model = (uint8_t)requested_model;
    if (rl_model_is_valid(requested_model) &&
        (requested_model != rl_active_model))
    {
        rl_active_model = requested_model;
        rl_deploy_debug.active_model = (uint8_t)rl_active_model;
        ++rl_deploy_debug.model_switch_count;
        rl_clear_policy_runtime();
    }

    ++rl_deploy_debug.sample_count;
    rl_update_joint_state();
    if (!rl_joint_state_is_valid())
    {
        rl_enter_numeric_fault(RL_DEPLOY_NUMERIC_FAULT_JOINT_STATE);
        return;
    }

    rl_update_projected_gravity();
    if (!rl_imu_state_is_valid())
    {
        rl_enter_numeric_fault(RL_DEPLOY_NUMERIC_FAULT_IMU_STATE);
        return;
    }

    ++rl_inference_divider;
    if (rl_inference_divider < RL_DEPLOY_INFERENCE_DIVIDER)
    {
        if (!rl_calculate_shadow_control())
        {
            rl_enter_numeric_fault(RL_DEPLOY_NUMERIC_FAULT_CONTROL_TORQUE);
        }
        else if (rl_deploy_debug.numeric_fault)
        {
            rl_zero_torque_outputs();
        }
        return;
    }
    rl_inference_divider = 0U;

    rl_build_observation();
    if (!rl_array_is_finite(rl_deploy_debug.obs, RL_POLICY_OBS_SIZE))
    {
        rl_enter_numeric_fault(RL_DEPLOY_NUMERIC_FAULT_OBSERVATION);
        return;
    }

    rl_update_history();
    if (!rl_array_is_finite(rl_deploy_debug.obs_history, RL_POLICY_OBS_HISTORY_SIZE))
    {
        rl_enter_numeric_fault(RL_DEPLOY_NUMERIC_FAULT_HISTORY);
        return;
    }

    policy = RLPolicy_GetInstance();
    rl_deploy_debug.inference_ok = RLPolicy_Run(
        policy,
        rl_active_model,
        rl_deploy_debug.obs,
        rl_deploy_debug.obs_history,
        rl_deploy_debug.actions);
    rl_deploy_debug.policy_ready = RLPolicy_IsReady(policy);
    rl_deploy_debug.last_inference_us = RLPolicy_GetLastInferenceUs(policy);
    ++rl_deploy_debug.inference_count;
    if (!rl_deploy_debug.inference_ok)
    {
        ++rl_deploy_debug.inference_fail_count;
        rl_enter_numeric_fault(RL_DEPLOY_NUMERIC_FAULT_POLICY_OUTPUT);
        return;
    }

    if (!rl_array_is_finite(rl_deploy_debug.actions, RL_POLICY_ACTION_SIZE))
    {
        ++rl_deploy_debug.inference_fail_count;
        rl_enter_numeric_fault(RL_DEPLOY_NUMERIC_FAULT_POLICY_OUTPUT);
        return;
    }

    rl_note_valid_inference();

    if (!rl_calculate_shadow_control())
    {
        rl_enter_numeric_fault(RL_DEPLOY_NUMERIC_FAULT_CONTROL_TORQUE);
        return;
    }

    if (rl_deploy_debug.numeric_fault)
    {
        rl_zero_torque_outputs();
    }

    /* Shadow mode: mapped physical torques remain debug-only. */
}
