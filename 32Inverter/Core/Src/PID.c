#include "PID.h"

float pid_calc(pi_t *pid, float input, float setpoint) {
    float error = setpoint - input;

    pid->integrated += error * pid->ki * pid->dt;
    if(pid->integrated > pid->max_out) pid->integrated = pid->max_out;
    else if(pid->integrated < -(pid->max_out)) pid->integrated = -(pid->max_out);

    return error * pid->kp + pid->integrated;
}