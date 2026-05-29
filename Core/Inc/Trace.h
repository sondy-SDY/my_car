#ifndef __TRACE_H
#define __TRACE_H

#include "main.h"

// 6路循迹传感器引脚定义 (高电平=检测到黑线)
#define S1_PIN  GPIO_PIN_12  // PB12 最左
#define S2_PIN  GPIO_PIN_13  // PB13
#define S3_PIN  GPIO_PIN_14  // PB14
#define S4_PIN  GPIO_PIN_15  // PB15
#define S5_PIN  GPIO_PIN_8   // PA8
#define S6_PIN  GPIO_PIN_9   // PA9 最右

#define S1_PORT GPIOB
#define S2_PORT GPIOB
#define S3_PORT GPIOB
#define S4_PORT GPIOB
#define S5_PORT GPIOA
#define S6_PORT GPIOA

#define TRACE_SENSOR_COUNT 6U
#define TRACE_SENSOR_BIT(index) ((uint8_t)(1U << (index)))
#define TRACE_CENTER_MASK       ((uint8_t)(TRACE_SENSOR_BIT(2) | TRACE_SENSOR_BIT(3)))
#define TRACE_LEFT_INNER_MASK   TRACE_SENSOR_BIT(1)
#define TRACE_RIGHT_INNER_MASK  TRACE_SENSOR_BIT(4)
#define TRACE_LEFT_BLEND_MASK   ((uint8_t)(TRACE_SENSOR_BIT(1) | TRACE_SENSOR_BIT(2)))
#define TRACE_RIGHT_BLEND_MASK  ((uint8_t)(TRACE_SENSOR_BIT(3) | TRACE_SENSOR_BIT(4)))

typedef struct {
    uint8_t mask;
    uint8_t raw_mask;
    uint8_t stable_mask;
    uint8_t pending_mask;
    uint8_t count;
    uint8_t confirm_count;
    uint8_t center_hold;
    uint8_t curve_hint;
    uint8_t uncertain;
    uint8_t line_lost;
    uint8_t wide;
    uint8_t split;
    float error;
} TraceState;

// 多级确认：过采样 + 积分迟滞 + 候选状态连续确认
#define TRACE_FILTER_MAX       5U
#define TRACE_FILTER_ON_LEVEL  2U
#define TRACE_FILTER_OFF_LEVEL 0U
#define TRACE_CONFIRM_CYCLES   2U
#define TRACE_LOST_HOLD_CYCLES 6U
#define TRACE_CENTER_HOLD_CYCLES 7U
#define TRACE_CURVE_HINT_CYCLES 3U

// 单次调用内多次采样取多数 (抗瞬态干扰)
#define TRACE_OVERSAMPLE       5U
#define TRACE_OVERSAMPLE_ON_VOTES 2U

// 中间两路闪烁时宁可迟钝一点，也不要立刻按边缘误判转弯
#define TRACE_CENTER_HOLD_ERROR_LIMIT 1.5f
#define TRACE_ERROR_JUMP_LIMIT        2.5f

float trace_get_error(void);
uint8_t trace_is_line_lost(void);
float trace_get_last_error(void);
float trace_get_search_direction(void);
uint8_t trace_get_raw_mask(void);
uint8_t trace_get_mask(void);
uint8_t trace_get_count(void);
const TraceState *trace_get_state(void);

#endif /* __TRACE_H */
