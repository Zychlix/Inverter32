#include "PID.h"

float pid_calc(pi_t *pid, float input, float setpoint) {
    float error = setpoint - input;

    pid->integrated += error * pid->ki * pid->dt;

    return error * pid->kp + pid->integrated;
}