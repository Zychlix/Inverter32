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


vec_t calculate_mtpa_stage_1(float i_m)
{

    float root_i_d = (LAMBDA*LAMBDA)/(16*(L_q-L_d)*(L_q-L_d))+i_m*i_m/2; //always positive
    float i_d_mtpa = LAMBDA/(4*(L_q-L_d))- sqrtf(root_i_d);
    float root_i_q = (i_m*i_m)-(i_d_mtpa*i_d_mtpa);
    if(root_i_q >=0)
    {
        float i_q_mtpa = sqrtf(root_i_q);
        return (vec_t){i_d_mtpa,i_q_mtpa};

    }
    return (vec_t){i_d_mtpa,0};
}

float calculate_required_vbus(vec_t mtpa_current, float omega)
{
    return omega*sqrtf((mtpa_current.y*L_q)*(mtpa_current.y*L_q)+(L_d*mtpa_current.x+LAMBDA)*(L_d*mtpa_current.x+LAMBDA));
}

uint32_t calculate_mtpa_weakening_regime(float i_r, float v_max, float omega, vec_t *output)
{
    float root = (LAMBDA*L_d)*(LAMBDA*L_d)-(L_d*L_d- L_q*L_q)*(LAMBDA*LAMBDA+L_q*L_q*i_r*i_r-v_max*v_max/(omega*omega));

    if(root< 0 )    return -1;
    float i_d = (-LAMBDA*L_d + sqrtf(root))/(L_d*L_d-L_q*L_q);
    float i_q = sqrtf(i_r*i_r-i_d*i_d);

    output->x = i_d;
    output->y = i_q;
    return 0;
}

uint32_t mtpa_complete(float i_m, float omega, float v_m, vec_t * output)
{
    vec_t mtpa_preliminary = calculate_mtpa_stage_1(i_m);
    if(mtpa_preliminary.y == 0)
    {
        return -1;
    }
    float dc_link_required = calculate_required_vbus(mtpa_preliminary, omega);

    if(dc_link_required < v_m)      //Under base speed
    {
        output->x = mtpa_preliminary.x;
        output->y = mtpa_preliminary.y;
        return 0;
    }

                                    //In omega > omega_m regime
    if(calculate_mtpa_weakening_regime(i_m, v_m, omega, output)==0)
    {
        return 0;
    }

    return -1;


}