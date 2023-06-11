#include "PID.h"

void pid_recalculate(pid_t * pid)
{
    double error = pid->setpoint - pid->input;

    pid->integrated += error * pid->ki * DELTA_T;

    pid->output = error * pid->kp + pid->integrated;
}