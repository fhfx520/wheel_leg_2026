#include "rl_policy_design.h"

#include <math.h>
#include <string.h>

#include "dwt.h"
#include "main.h"

/* ============================================================
 * CubeAI generated model headers
 * ============================================================ */
#include "stable.h"
#include "stable_data.h"

/* ============================================================
 * Network Context
 * ============================================================ */

typedef struct
{
    ai_handle network;

    ai_buffer *inputs;
    ai_buffer *outputs;

    uint8_t ready;

} NetworkContext_t;


/* ============================================================
 * Network contexts
 * ============================================================ */

NetworkContext_t stable_ctx = {
    AI_HANDLE_NULL,
    NULL,
    NULL,
    0
};

static uint8_t rl_policy_array_is_finite(const float *data, uint32_t size)
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

/* ============================================================
 * Activation buffers
 *
 * CubeAI requires these buffers to be aligned.
 * ============================================================ */

AI_ALIGNED(4) static ai_u8 stable_activations[AI_STABLE_DATA_ACTIVATION_1_SIZE];

/* ============================================================
 * Global RL Policy instance
 * ============================================================ */

RLPolicy_t rl_policy = {
    0,
    0
};


/* ============================================================
 * Initialize Stable network
 * ============================================================ */

uint8_t RLPolicy_InitStable(void)
{
    const ai_handle activation_buffers[] = {
        AI_HANDLE_PTR(stable_activations)
    };

    ai_error create_error;
    ai_u16 input_count = 0;
    ai_u16 output_count = 0;

    if (stable_ctx.ready)
    {
        return 1U;
    }

    create_error = ai_stable_create_and_init(
        &stable_ctx.network,
        activation_buffers,
        NULL
    );

    if (create_error.type != AI_ERROR_NONE)
    {
        stable_ctx.ready = 0U;
        return 0U;
    }

    stable_ctx.inputs = ai_stable_inputs_get(
        stable_ctx.network,
        &input_count
    );

    stable_ctx.outputs = ai_stable_outputs_get(
        stable_ctx.network,
        &output_count
    );

    stable_ctx.ready =
        (stable_ctx.inputs != NULL) &&
        (stable_ctx.outputs != NULL) &&
        (input_count == AI_STABLE_IN_NUM) &&
        (output_count == AI_STABLE_OUT_NUM);

    return stable_ctx.ready;
}


/* ============================================================
 * Initialize Upstairs network
 * ============================================================ */

/* ============================================================
 * Initialize Pin network
 * ============================================================ */

/* ============================================================
 * Initialize Jump network
 * ============================================================ */

/* ============================================================
 * Get context according to model
 * ============================================================ */

NetworkContext_t *RLPolicy_GetContext(
    RLPolicyModel_t model
)
{
    switch (model)
    {
        case RL_POLICY_MODEL_STABLE:
            return &stable_ctx;
        default:
            return NULL;
    }
}


/* ============================================================
 * Get singleton instance
 * ============================================================ */

RLPolicy_t *RLPolicy_GetInstance(void)
{
    return &rl_policy;
}


/* ============================================================
 * Initialize
 * ============================================================ */

uint8_t RLPolicy_Init(RLPolicy_t *policy)
{
    uint8_t stable_ok;

    if (policy == NULL)
    {
        return 0U;
    }

    if (policy->ready)
    {
        return 1U;
    }

    /* CubeAI runtime requires the CRC peripheral clock on STM32H7. */
    __HAL_RCC_CRC_CLK_ENABLE();

    /*
     * Do not use:
     *
     * stable_ok && upstairs_ok && pin_ok && jump_ok
     *
     * directly here, because && has short-circuit evaluation.
     *
     * We want all four models to be initialized.
     */

    stable_ok = RLPolicy_InitStable();

    policy->ready = stable_ok;
    return policy->ready;
}


/* ============================================================
 * Run inference
 * ============================================================ */

uint8_t RLPolicy_Run(
    RLPolicy_t *policy,
    RLPolicyModel_t model,
    const float obs[RL_POLICY_OBS_SIZE],
    const float obs_history[RL_POLICY_OBS_HISTORY_SIZE],
    float actions[RL_POLICY_ACTION_SIZE]
)
{
    NetworkContext_t *ctx;

    ai_float *obs_input;
    ai_float *obs_history_input;
    ai_float *actions_output;

    ai_i32 processed_batches = 0;
    uint64_t inference_start_us;

    if ((policy == NULL) ||
        (obs == NULL) ||
        (obs_history == NULL) ||
        (actions == NULL))
    {
        return 0U;
    }

    /* Never copy NaN/Inf into the CubeAI activation buffers. */
    if ((!rl_policy_array_is_finite(obs, RL_POLICY_OBS_SIZE)) ||
        (!rl_policy_array_is_finite(obs_history, RL_POLICY_OBS_HISTORY_SIZE)))
    {
        memset(actions, 0, sizeof(float) * RL_POLICY_ACTION_SIZE);
        return 0U;
    }

    /*
     * Make sure all networks are initialized.
     */
    if (!policy->ready)
    {
        if (!RLPolicy_Init(policy))
        {
            memset(
                actions,
                0,
                sizeof(float) * RL_POLICY_ACTION_SIZE
            );

            return 0U;
        }
    }

    /*
     * Select network according to model.
     */
    ctx = RLPolicy_GetContext(model);

    if ((ctx == NULL) || (!ctx->ready))
    {
        memset(
            actions,
            0,
            sizeof(float) * RL_POLICY_ACTION_SIZE
        );

        return 0U;
    }

    /*
     * Get actual input/output data pointers.
     */
    obs_input = AI_BUFFER_DATA(
        &ctx->inputs[0],
        ai_float
    );

    obs_history_input = AI_BUFFER_DATA(
        &ctx->inputs[1],
        ai_float
    );

    actions_output = AI_BUFFER_DATA(
        &ctx->outputs[0],
        ai_float
    );

    if ((obs_input == NULL) ||
        (obs_history_input == NULL) ||
        (actions_output == NULL))
    {
        memset(
            actions,
            0,
            sizeof(float) * RL_POLICY_ACTION_SIZE
        );

        return 0U;
    }


    /* ========================================================
     * Copy input data
     * ======================================================== */

    inference_start_us = DWT_GetTimeline_us();

    switch (model)
    {
        case RL_POLICY_MODEL_STABLE:

            memcpy(
                obs_input,
                obs,
                AI_STABLE_IN_1_SIZE_BYTES
            );

            memcpy(
                obs_history_input,
                obs_history,
                AI_STABLE_IN_2_SIZE_BYTES
            );

            processed_batches = ai_stable_run(
                ctx->network,
                ctx->inputs,
                ctx->outputs
            );

            break;

        default:
            memset(actions, 0, sizeof(float) * RL_POLICY_ACTION_SIZE);
            return 0U;
    }

    policy->last_inference_us =
        (uint32_t)(DWT_GetTimeline_us() - inference_start_us);


    /* ========================================================
     * Check inference result
     * ======================================================== */

    if (processed_batches != 1)
    {
        /*
         * Read network error to preserve diagnostic chain.
         */
        switch (model)
        {
            case RL_POLICY_MODEL_STABLE:
                (void)ai_stable_get_error(ctx->network);
                break;
            default:
                break;
        }

        memset(
            actions,
            0,
            sizeof(float) * RL_POLICY_ACTION_SIZE
        );

        return 0U;
    }

    /* A successful ai_run only means the graph executed; validate its numbers. */
    if (!rl_policy_array_is_finite(actions_output, RL_POLICY_ACTION_SIZE))
    {
        memset(actions, 0, sizeof(float) * RL_POLICY_ACTION_SIZE);
        return 0U;
    }


    /* ========================================================
     * Copy output data
     * ======================================================== */

    switch (model)
    {
        case RL_POLICY_MODEL_STABLE:

            memcpy(
                actions,
                actions_output,
                AI_STABLE_OUT_1_SIZE_BYTES
            );

            break;

        default:
            memset(actions, 0, sizeof(float) * RL_POLICY_ACTION_SIZE);
            return 0U;
    }

    return 1U;
}


/* ============================================================
 * Check ready state
 * ============================================================ */

uint8_t RLPolicy_IsReady(const RLPolicy_t *policy)
{
    if (policy == NULL)
    {
        return 0U;
    }

    return policy->ready;
}


/* ============================================================
 * Get last inference time
 * ============================================================ */

uint32_t RLPolicy_GetLastInferenceUs(
    const RLPolicy_t *policy
)
{
    if (policy == NULL)
    {
        return 0U;
    }

    return policy->last_inference_us;
}
