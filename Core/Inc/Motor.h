#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include <stdint.h>

// 左电机 (TIM3_CH1 = PA6)
#define MOTOR_L_IN1_PIN  GPIO_PIN_0   // PB0
#define MOTOR_L_IN2_PIN  GPIO_PIN_1   // PB1
#define MOTOR_L_PORT     GPIOB

// 右电机 (TIM3_CH2 = PA7)
#define MOTOR_R_IN1_PIN  GPIO_PIN_10  // PB10
#define MOTOR_R_IN2_PIN  GPIO_PIN_11  // PB11
#define MOTOR_R_PORT     GPIOB

// PWM parameters
#define PWM_MAX       999
#define BASE_SPEED    360
#define MAX_RUN_SPEED 440   // 直道速度
#define MIN_RUN_SPEED 0     // 转弯内轮可停到0，一轮定住另一轮转
#define CROSS_SPEED   260
#define TURN_SLOWDOWN 30    // 降低斜率，直道判定范围更宽，速度更稳
#define SEARCH_SPEED  320
#define MOTOR_PWM_STEP 300  // 加快差速建立

// Open-loop trim for motors without encoders. Tune these on a straight line.
#define LEFT_TRIM   0
#define RIGHT_TRIM  0

void motor_set(int16_t left, int16_t right);
void motor_stop(void);

#endif /* __MOTOR_H */
