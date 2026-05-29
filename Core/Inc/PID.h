#ifndef __PID_H
#define __PID_H

#include <stdint.h>

#define KP    35.0f
#define KI    0.0f
#define KD    70.0f
#define MAX_I 60.0f
#define PID_OUTPUT_LIMIT 350.0f

// D项滤波适中
#define PID_D_FILTER_ALPHA 0.8f

void calc_pid(float error, uint8_t line_lost);
float get_pid_output(void);
void pid_reset(void);

#endif /* __PID_H */
