#pragma once

typedef struct PID {
    float kp;
    float ki;
    float dt;
    float max_out;

    float integrated;
} pi_t;

float pid_calc(pi_t *pid, float input, float setpoint);