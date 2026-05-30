#include "Trace.h"

static TraceState state = {0};
static float last_error = 0.0f;
static float last_search_direction = 1.0f;
static uint8_t filter_level[TRACE_SENSOR_COUNT] = {0};
static uint8_t stable_mask = 0;
static uint8_t pending_mask = 0;
static uint8_t pending_confirm_count = 0;
static uint8_t center_hold_count = 0;
static uint8_t left_curve_hint_count = 0;
static uint8_t right_curve_hint_count = 0;
static uint8_t lost_hold_count = 0;

static float trace_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t read_raw_mask(void)
{
    uint8_t mask = 0;

    if (HAL_GPIO_ReadPin(S1_PORT, S1_PIN) == GPIO_PIN_SET) mask |= (uint8_t)(1U << 0);
    if (HAL_GPIO_ReadPin(S2_PORT, S2_PIN) == GPIO_PIN_SET) mask |= (uint8_t)(1U << 1);
    if (HAL_GPIO_ReadPin(S3_PORT, S3_PIN) == GPIO_PIN_SET) mask |= (uint8_t)(1U << 2);
    if (HAL_GPIO_ReadPin(S4_PORT, S4_PIN) == GPIO_PIN_SET) mask |= (uint8_t)(1U << 3);
    if (HAL_GPIO_ReadPin(S5_PORT, S5_PIN) == GPIO_PIN_SET) mask |= (uint8_t)(1U << 4);
    if (HAL_GPIO_ReadPin(S6_PORT, S6_PIN) == GPIO_PIN_SET) mask |= (uint8_t)(1U << 5);

    return mask;
}

// 多次采样取多数决，抗无灯红外的瞬态噪声
static uint8_t read_oversampled_mask(void)
{
    uint8_t vote[TRACE_SENSOR_COUNT] = {0};

    for (uint8_t s = 0; s < TRACE_OVERSAMPLE; s++) {
        uint8_t raw = read_raw_mask();
        for (uint8_t i = 0; i < TRACE_SENSOR_COUNT; i++) {
            if ((raw & (uint8_t)(1U << i)) != 0U) {
                vote[i]++;
            }
        }
    }

    uint8_t result = 0;
    for (uint8_t i = 0; i < TRACE_SENSOR_COUNT; i++) {
        if (vote[i] >= TRACE_OVERSAMPLE_ON_VOTES) {
            result |= (uint8_t)(1U << i);
        }
    }
    return result;
}

static uint8_t filter_raw_mask(uint8_t raw_mask)
{
    uint8_t candidate_mask = stable_mask;

    for (uint8_t i = 0; i < TRACE_SENSOR_COUNT; i++) {
        uint8_t bit = (uint8_t)(1U << i);

        if ((raw_mask & bit) != 0U) {
            if (filter_level[i] < TRACE_FILTER_MAX) {
                filter_level[i]++;
            }
        } else if (filter_level[i] > 0U) {
            filter_level[i]--;
        }

        if (filter_level[i] >= TRACE_FILTER_ON_LEVEL) {
            candidate_mask |= bit;
        } else if (filter_level[i] <= TRACE_FILTER_OFF_LEVEL) {
            candidate_mask &= (uint8_t)~bit;
        }
    }

    if (candidate_mask == stable_mask) {
        pending_mask = stable_mask;
        pending_confirm_count = TRACE_CONFIRM_CYCLES;
    } else {
        if (candidate_mask != pending_mask) {
            pending_mask = candidate_mask;
            pending_confirm_count = 1U;
        } else if (pending_confirm_count < TRACE_CONFIRM_CYCLES) {
            pending_confirm_count++;
        }

        if (pending_confirm_count >= TRACE_CONFIRM_CYCLES) {
            stable_mask = pending_mask;
            pending_confirm_count = TRACE_CONFIRM_CYCLES;
        }
    }

    return stable_mask;
}

static uint8_t count_black_groups(uint8_t mask)
{
    uint8_t groups = 0;
    uint8_t in_group = 0;

    for (uint8_t i = 0; i < TRACE_SENSOR_COUNT; i++) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            if (!in_group) {
                groups++;
                in_group = 1;
            }
        } else {
            in_group = 0;
        }
    }

    return groups;
}

static uint8_t keep_center_group(uint8_t mask)
{
    uint8_t result = (uint8_t)(mask & TRACE_CENTER_MASK);

    if (result == TRACE_CENTER_MASK) {
        return TRACE_CENTER_MASK;
    }

    if ((mask & TRACE_SENSOR_BIT(2)) != 0U) {
        if ((mask & TRACE_SENSOR_BIT(1)) != 0U) {
            result |= TRACE_SENSOR_BIT(1);
            if ((mask & TRACE_SENSOR_BIT(0)) != 0U) {
                result |= TRACE_SENSOR_BIT(0);
            }
        }
    }

    if ((mask & TRACE_SENSOR_BIT(3)) != 0U) {
        if ((mask & TRACE_SENSOR_BIT(4)) != 0U) {
            result |= TRACE_SENSOR_BIT(4);
            if ((mask & TRACE_SENSOR_BIT(5)) != 0U) {
                result |= TRACE_SENSOR_BIT(5);
            }
        }
    }

    return result;
}

static uint8_t update_curve_hint(uint8_t mask, uint8_t raw_mask)
{
    uint8_t seen = (uint8_t)(mask | raw_mask);

    if ((seen & TRACE_LEFT_INNER_MASK) != 0U) {
        left_curve_hint_count = TRACE_CURVE_HINT_CYCLES;
    } else if (left_curve_hint_count > 0U) {
        left_curve_hint_count--;
    }

    if ((seen & TRACE_RIGHT_INNER_MASK) != 0U) {
        right_curve_hint_count = TRACE_CURVE_HINT_CYCLES;
    } else if (right_curve_hint_count > 0U) {
        right_curve_hint_count--;
    }

    if ((left_curve_hint_count > 0U) && (right_curve_hint_count == 0U)) {
        return 1U;
    }
    if ((right_curve_hint_count > 0U) && (left_curve_hint_count == 0U)) {
        return 2U;
    }

    return 0U;
}

static uint8_t normalize_trace_mask(uint8_t mask, uint8_t raw_mask,
                                    uint8_t *center_hold, uint8_t *curve_hint,
                                    uint8_t *uncertain)
{
    uint8_t center_seen = (uint8_t)((raw_mask | mask) & TRACE_CENTER_MASK);
    uint8_t turn_hint = update_curve_hint(mask, raw_mask);

    *center_hold = 0U;
    *curve_hint = turn_hint;
    *uncertain = 0U;

    if (center_seen != 0U) {
        center_hold_count = TRACE_CENTER_HOLD_CYCLES;
    } else if (center_hold_count > 0U) {
        center_hold_count--;
    }

    if ((mask & TRACE_CENTER_MASK) != 0U) {
        uint8_t centered_mask = keep_center_group(mask);
        if (centered_mask != mask) {
            *center_hold = 1U;
        }
        return centered_mask;
    }

    // 有真实的非中心读数(如内侧 S2/S5、外侧 S1/S6 亮)：直接放行真实加权误差，
    // 不再用固定 BLEND(±2) 或 CENTER(0) 覆盖。这是修复急弯欠转露线的关键——
    // 线偏到 S1/S6 时误差应为 ±5，旧逻辑却把它钉死在 ±2，转向力只有真实值的40%。
    if (mask != 0U) {
        if (count_black_groups(mask) >= 2U) {
            *uncertain = 1U;
        }
        return mask;
    }

    // 以下分支仅在当前帧滤波掩码完全为空(线落在传感器缝隙)时生效，
    // 用历史提示维持上一次的转向方向/居中，避免立刻判丢线。
    if ((turn_hint == 1U) && (last_error <= TRACE_CENTER_HOLD_ERROR_LIMIT)) {
        *center_hold = 1U;
        return TRACE_LEFT_BLEND_MASK;
    }
    if ((turn_hint == 2U) && (last_error >= -TRACE_CENTER_HOLD_ERROR_LIMIT)) {
        *center_hold = 1U;
        return TRACE_RIGHT_BLEND_MASK;
    }

    if ((center_hold_count > 0U) &&
        ((trace_absf(last_error) <= TRACE_CENTER_HOLD_ERROR_LIMIT) ||
         ((raw_mask & TRACE_CENTER_MASK) != 0U))) {
        *center_hold = 1U;
        return TRACE_CENTER_MASK;
    }

    return mask;
}

static float guard_sudden_turn(float new_error, uint8_t *uncertain)
{
    if ((trace_absf(last_error) <= TRACE_CENTER_HOLD_ERROR_LIMIT) &&
        (trace_absf(new_error - last_error) >= TRACE_ERROR_JUMP_LIMIT)) {
        *uncertain = 1U;
    }

    return new_error;
}

static int pick_error_from_mask(uint8_t mask, float reference_error)
{
    const int weights[TRACE_SENSOR_COUNT] = {-5, -3, -1, 1, 3, 5};
    int best_weight = 0;
    float best_distance = 100.0f;

    for (uint8_t i = 0; i < TRACE_SENSOR_COUNT; i++) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            float distance = (float)weights[i] - reference_error;
            if (distance < 0.0f) {
                distance = -distance;
            }
            if (distance < best_distance) {
                best_distance = distance;
                best_weight = weights[i];
            }
        }
    }

    return best_weight;
}

float trace_get_error(void) {
    const int weights[TRACE_SENSOR_COUNT] = {-5, -3, -1, 1, 3, 5};
    int sum = 0;
    uint8_t count = 0;
    uint8_t raw_mask = read_oversampled_mask();
    uint8_t filtered_mask = filter_raw_mask(raw_mask);
    uint8_t center_hold = 0U;
    uint8_t curve_hint = 0U;
    uint8_t uncertain = 0U;
    uint8_t mask = normalize_trace_mask(filtered_mask, raw_mask,
                                        &center_hold, &curve_hint, &uncertain);

    for (uint8_t i = 0; i < TRACE_SENSOR_COUNT; i++) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            sum += weights[i];
            count++;
        }
    }

    state.raw_mask = raw_mask;
    state.mask = mask;
    state.stable_mask = filtered_mask;
    state.pending_mask = pending_mask;
    state.count = count;
    state.confirm_count = pending_confirm_count;
    state.center_hold = center_hold;
    state.curve_hint = curve_hint;
    state.uncertain = uncertain;
    state.wide = 0;
    state.split = 0;

    if (count == 0) {
        if (lost_hold_count < TRACE_LOST_HOLD_CYCLES) {
            lost_hold_count++;
        }
        state.line_lost = (lost_hold_count >= TRACE_LOST_HOLD_CYCLES) ? 1U : 0U;
        state.error = last_error;
        return last_error;
    }

    lost_hold_count = 0;
    state.line_lost = 0;

    uint8_t groups = count_black_groups(mask);
    state.split = (groups >= 2U) ? 1U : 0U;
    state.wide = (count >= 4U) ? 1U : 0U;

    float new_error;
    if (state.split) {
        new_error = (float)pick_error_from_mask(mask, last_error);
    } else {
        new_error = (float)sum / count;
    }

    new_error = guard_sudden_turn(new_error, &uncertain);
    state.uncertain = uncertain;

    // 记录最后一次非居中方向用于丢线原地搜索
    if (new_error > 0.5f) {
        last_search_direction = 1.0f;
    } else if (new_error < -2.0f) {
        // 只有明显偏左才切换到左转搜索，避免轻微左偏覆盖默认右转
        last_search_direction = -1.0f;
    } else {
        uint8_t search_mask = (uint8_t)(mask | raw_mask);
        uint8_t left_seen = (uint8_t)(search_mask & (TRACE_SENSOR_BIT(0) | TRACE_SENSOR_BIT(1)));
        uint8_t right_seen = (uint8_t)(search_mask & (TRACE_SENSOR_BIT(4) | TRACE_SENSOR_BIT(5)));

        if ((right_seen != 0U) && (left_seen == 0U)) {
            last_search_direction = 1.0f;
        } else if ((left_seen != 0U) && (right_seen == 0U)) {
            last_search_direction = -1.0f;
        }
    }

    // 不做额外低通滤波，过采样+迟滞已抗噪
    last_error = new_error;

    state.error = last_error;
    return last_error;
}

uint8_t trace_is_line_lost(void) {
    return state.line_lost;
}

float trace_get_last_error(void) {
    return last_error;
}

float trace_get_search_direction(void) {
    return last_search_direction;
}

uint8_t trace_get_raw_mask(void) {
    return state.raw_mask;
}

uint8_t trace_get_mask(void) {
    return state.mask;
}

uint8_t trace_get_count(void) {
    return state.count;
}

const TraceState *trace_get_state(void) {
    return &state;
}
