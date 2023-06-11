#pragma once

#define DELTA_T 0.001   //1ms

typedef struct PID
{

    double kp;
    double ki;

    double integrated;
    double input;
    double output;
    double setpoint;

}pid_t;

void pid_recalculate(pid_t * pid);