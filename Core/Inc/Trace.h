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
// 注意：主循环周期由 10ms 改为 5ms(200Hz)后，下列"按周期计"的窗口
// 已相应加倍以保持实际毫秒时长不变(纯提升控制分辨率，不改变保持/丢线行为)。
// 唯一例外是 TRACE_FILTER_MAX，有意调小以加快边缘位释放、减少过弯露线。
#define TRACE_FILTER_MAX       4U   // 3→4: 迟滞更厚，单帧误读不够累积到置位
#define TRACE_FILTER_ON_LEVEL  2U
#define TRACE_FILTER_OFF_LEVEL 0U
#define TRACE_CONFIRM_CYCLES   4U   // 2→4: 配合 5ms 周期，确认时长保持 ~20ms
#define TRACE_LOST_HOLD_CYCLES 10U  // 50ms 内用 last_error 差速滑行，普通弯道不触发自转
#define TRACE_CENTER_HOLD_CYCLES 6U  // 降低保持时间，减少中间灯闪烁影响
#define TRACE_CURVE_HINT_CYCLES 1U  // 内侧灯灭后立刻解除弯道提示，不影响直道速度

// 单次调用内多次采样取多数 (抗瞬态干扰/褶皱反光)
// 9次采样需5票才置位，褶皱短暂反光难以凑够多数
#define TRACE_OVERSAMPLE       9U
#define TRACE_OVERSAMPLE_ON_VOTES 5U

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
