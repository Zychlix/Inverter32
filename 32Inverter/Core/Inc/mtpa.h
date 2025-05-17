#pragma once
#include "math.h"
#include "vectors.h"

#define L_d ((149e-6f)) //H

#define L_q ((372e-6f))

//MCD Motor Current Dispatcher

static volatile float lambda = ((0.05f)); // To be designated
#define LAMBDA 0.05f// approx 43.7V per 162 rad/s

#define CONVERGANCE 1.f
#define POLE_COUNT  ((4))
#define OMEGA       ((p.Omega))
#define VBUS        ((p.V_bus))
#define I_Q         ((p.I_q))
#define T_REF       ((p.T_ref))

#define MTPA_I0     ((300.f))

#define MTPA_A ((9 * POLE_COUNT*POLE_COUNT * (L_d - L_q)*(L_d - L_q)*(L_q*L_q) * (OMEGA*OMEGA)))
#define MTPA_B 0
#define MTPA_C (9 * (POLE_COUNT *POLE_COUNT) * (LAMBDA*LAMBDA) * (L_q *L_q) * (OMEGA *OMEGA) - 9* (POLE_COUNT *POLE_COUNT) * (L_d-L_q)*(L_d-L_q) * (VBUS*VBUS))
#define MTPA_D (-12 * T_REF * POLE_COUNT * LAMBDA * L_d * L_q*OMEGA *OMEGA)
#define MTPA_E (4 * T_REF *T_REF * L_d *L_d * OMEGA *OMEGA)

#define MTPA_NEWTON_ITERATIONS ((10))

#define MTPA_OK ((0))
#define MTPA_FAIL ((-1))

typedef struct
{
    float Omega;    //Electrical omega
    float T_ref;    //Reference torque
    float V_bus;    //Supply voltage
    float I_q;      //Iq
}mtpa_parameters;

float mtpa_expression(mtpa_parameters p)
{
    return MTPA_A * (I_Q *I_Q*I_Q*I_Q) + MTPA_C * I_Q * I_Q + MTPA_D * I_Q + MTPA_E;
}
float mtpa_derivative_expression(mtpa_parameters p)
{
    return 4 * MTPA_A *(I_Q*I_Q*I_Q) + 2 * MTPA_C * I_Q + MTPA_D;
}

uint32_t mtpa_newton(mtpa_parameters parameters, float* I_q)    // Iq is treated as first solution, also as a return value
{
    parameters.I_q= MTPA_I0;    //Important
    float delta_I = CONVERGANCE+1;
    for(int i = 0; (i< MTPA_NEWTON_ITERATIONS); i++)

    {
        delta_I = -mtpa_expression(parameters)/ mtpa_derivative_expression(parameters);
        parameters.I_q += delta_I;
        //printf("Newton i: %d, %f  \r\n", i, delta_I);
    }
    if (delta_I*delta_I >CONVERGANCE*CONVERGANCE)   //ABS(delta_I)>CONVERGENCE
    {
        return MTPA_FAIL;
    }
    *I_q = parameters.I_q;
    return MTPA_OK;
}
/*
 * output Iq and Id in motor reference frame
 * current reference [A]
 * omega [rad/s] ELECTRICAL we = wm* pole count
 */



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



float mtpa_get_im(float T_ref,float  I_max)
{
    float Im_ref = 2.f * T_ref /( 3.f * POLE_COUNT * LAMBDA);
    if(Im_ref > I_max)
    {
        return I_max;
    }
    return Im_ref;
}

uint32_t mtpa_current_controller_newton(float t_ref, float i_m, float omega, float v_m, vec_t * output)
{
    static volatile bool weakening_enabled;

    float i_ref = mtpa_get_im(t_ref,i_m);

    vec_t result_current={0,0};
    uint32_t return_value = calculate_mtpa(i_ref,&result_current);
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

    mtpa_parameters parameters = {.Omega = omega, .V_bus = v_m, .T_ref = t_ref};

    float mtpa_i_q_weakening, mtpa_i_d_weakening = 0;
    return_value = mtpa_newton(parameters,&mtpa_i_q_weakening);

    if(return_value == MTPA_FAIL)
    {
        *output = (vec_t){0,0};
        return MTPA_FAIL;
    }

    float root = i_ref*i_ref-mtpa_i_q_weakening*mtpa_i_q_weakening;

    if(root<0)
    {
        *output = (vec_t){0,0};
        return MTPA_FAIL;
    }

    mtpa_i_d_weakening = -sqrtf(root);
    *output = (vec_t){mtpa_i_d_weakening, mtpa_i_q_weakening};

    weakening_enabled = true;
    return MTPA_OK;


}