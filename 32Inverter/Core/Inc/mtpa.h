#pragma once
#include "math.h"
#include "vectors.h"

#define L_d ((149e-6f)) //H

#define L_q ((372e-6f))


static volatile float lambda = ((0.05f)); // To be designated
#define LAMBDA lambda// approx 43.7V per 162 rad/s

/*
 * output Iq and Id in motor reference frame
 * current reference [A]
 * omega [rad/s] ELECTRICAL we = wm* pole count
 */

#define MTPA_OK ((0))
#define MTPA_FAIL ((-1))


uint32_t calculate_mtpa(float i_m, vec_t * current)
{

    float root_i_d = (LAMBDA*LAMBDA)/(16*(L_q-L_d)*(L_q-L_d))+i_m*i_m/2; //always positive
    if(root_i_d<0)
    {
        return MTPA_FAIL;     //Invalid root arg
    }
    float i_d_mtpa = LAMBDA/(4*(L_q-L_d))- sqrtf(root_i_d);

    float root_i_q = (i_m*i_m)-(i_d_mtpa*i_d_mtpa);
    if(root_i_q >=0)
    {
        float i_q_mtpa = sqrtf(root_i_q);
        current->x  = i_d_mtpa;
        current->y  = i_q_mtpa;
        return MTPA_OK;

    }
    return MTPA_FAIL;
}

float calculate_required_vbus(vec_t mtpa_current, float omega)
{
    return omega*sqrtf((mtpa_current.y*L_q)*(mtpa_current.y*L_q)+(L_d*mtpa_current.x+LAMBDA)*(L_d*mtpa_current.x+LAMBDA));
}

uint32_t calculate_mtpa_weakening_regime(float i_r, float v_max, float omega, vec_t *output)
{
    float root = (LAMBDA*L_d)*(LAMBDA*L_d)-(L_d*L_d- L_q*L_q)*(LAMBDA*LAMBDA+L_q*L_q*i_r*i_r-v_max*v_max/(omega*omega));

    if(root< 0 ){
        __asm__("nop");
        return MTPA_FAIL;
    }

    float i_d = (-LAMBDA*L_d + sqrtf(root))/(L_d*L_d-L_q*L_q);
    if(i_d < -fabsf(i_r)){
        i_d = -fabsf(i_r);
    }

    root = i_r*i_r-i_d*i_d;
    if(root< 0 ){
        __asm__("nop");
        return MTPA_FAIL;
    }

    float i_q = sqrtf(root);

    *output = (vec_t){i_d,i_q};
    return MTPA_OK;
}

uint32_t mtpa_current_controller(float i_m, float omega, float v_m, vec_t * output)
{
    static volatile bool weakening_enabled;

    vec_t result_current={0,0};
    uint32_t return_value = calculate_mtpa(i_m,&result_current);
    if(return_value == MTPA_FAIL)
    {
        *output = (vec_t){0,0};
        return MTPA_FAIL;
    }

    static volatile float dc_link_required = 0;
    dc_link_required = calculate_required_vbus(result_current, omega);

    if(dc_link_required < v_m)      //Under base speed
    {
        weakening_enabled = false;
        *output = result_current;
        return MTPA_OK;
    }

    return_value = calculate_mtpa_weakening_regime(i_m, v_m, omega, &result_current);
                                    //In omega > omega_m regime
    if( return_value == MTPA_FAIL)
    {
        *output = (vec_t){0,0};
        return MTPA_FAIL;
    }

    weakening_enabled = true;
    *output = result_current;
    return MTPA_OK;


}