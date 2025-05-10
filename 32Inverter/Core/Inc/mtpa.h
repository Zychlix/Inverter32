#pragma once
#include "math.h"
#include "vectors.h"

#define L_d ((149e-6f)) //H
#define R_d 0.039f //Ohm

#define L_q ((372e-6f))
#define R_Q 0.073f

#define LAMBDA (0.27f/4)// approx 43.7V per 162 rad/s

/*
 * output Iq and Id in motor reference frame
 * current reference [A]
 * omega [rad/s] ELECTRICAL we = wm* pole count
 */


vec_t calculate_mtpa(float i_r,float v_max, float omega)
{
    float root = (LAMBDA*L_d)*(LAMBDA*L_d)-(L_d*L_d- L_q*L_q)*(LAMBDA*LAMBDA+L_q*L_q*i_r*i_r-v_max*v_max/(omega*omega));
    float i_d = (-LAMBDA*L_d + sqrtf(root))/(L_d*L_d-L_q*L_q);
    float i_q = sqrtf(i_r*i_r-i_d*i_d);
    return (vec_t){i_d,i_q};
}
