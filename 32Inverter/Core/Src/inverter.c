//
// Created by michal on 03.05.23.
//

#include <stdio.h>
#include "inverter.h"
#include "resolver.h"
#include "vectors.h"
#include "PID.h"
#include "oscilloscope.h"
#include "swo_scope.h"
#include "stm32f3xx_hal_tim_ex.h"
#include "can_debug_interface.h"
#include "fast_data_logger.h"
#include "stimuli.h"
#define INV_MIN_VOLTAGE_HYSTERESIS 5.f



#define TRACE_FREQUENCY_DIVIDER 16

#define INV_LOOP_SPEED 13800.f

#define INV_MOTOR_POLES 4


extern inv_t inv;
extern cdi_t can_debugger;
extern fdl_t fast_data ;

#define SYSCLK ((64000000))

void fdl_acquisition_complete()
{
//    printf("acquisition complete! \r\n");
}


__inline void inv_delay_cycles(uint32_t delay)
{
    uint32_t start_time = DWT->CYCCNT;
    delay = delay*8;
    while ((DWT->CYCCNT - start_time) < delay);
}


inv_ret_val_t inv_state_machine_update(inv_t * inverter)
{
    if(inverter == NULL)
    {
        return INV_FAIL;

    }

//    inverter_status_t current_status = inverter.sta
    return INV_OK;
}

void inv_reset_controllers(inv_t * inverter)
{
    inverter->pid_d.integrated = 0;
    inverter->pid_q.integrated = 0;
    inverter->pid_a.integrated = 0;
    inverter->pid_b.integrated = 0;
}

void inv_enable_pwm_outputs(inv_t *inverter, uint32_t Channel)
{
    inverter->timer->Instance->CCER |= (1 | 1<<2)<<Channel;

}

void inv_disable_pwm_outputs(inv_t *inverter, uint32_t Channel)
{
    inverter->timer->Instance->CCER &= ~(1 | 1<<2)<<Channel;

}


inv_ret_val_t inv_init(inv_t *inverter) {

    if(inverter->relay_box.main_contactor.pin == 0) return INV_FAIL;
    if(inverter->relay_box.main_contactor.port == 0) return INV_FAIL;
    if(inverter->relay_box.precharge_contactor.pin == 0) return INV_FAIL;
    if(inverter->relay_box.precharge_contactor.port == 0)return INV_FAIL;

    TIM_OC_InitTypeDef oc_config;

    oc_config.OCMode = TIM_OCMODE_PWM1;
    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc_config.OCFastMode = TIM_OCFAST_DISABLE;
    oc_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    oc_config.OCIdleState = TIM_OCIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(inverter->timer, &oc_config, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(inverter->timer, &oc_config, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(inverter->timer, &oc_config, TIM_CHANNEL_3);

    HAL_ADC_Start_DMA(inverter->current_adc, (uint32_t *) &inverter->raw_current_adc, 2);

    inverter->current_filter_alpha = DEFAULT_CURRENT_FILTER_ALPHA;

    inverter->filter_d = (iir_filter_t){
        .a = {1.0f, -0.10528975341038055f, 0.07419736034296573f},
        .b = {0.24222690173314645f, 0.4844538034662929f, 0.24222690173314645f},
    };
    inverter->filter_q = inverter->filter_d;

    iir_filter_init(&inverter->filter_d);
    iir_filter_init(&inverter->filter_q);


    // current PI
    inverter->pid_d.kp = 0.8f;
    inverter->pid_d.ki = 200.f;
    inverter->pid_d.dt = (float) INV_MAX_PWM_PULSE_VAL / (float) SystemCoreClock;
    inverter->pid_d.integrated = 0;
    inverter->pid_d.max_out = INV_PID_MAX_OUT;

    inverter->pid_q.kp = 0.8f;
    inverter->pid_q.ki = 200.f;
    inverter->pid_q.dt = (float) INV_MAX_PWM_PULSE_VAL / (float) SystemCoreClock;
    inverter->pid_q.integrated = 0;
    inverter->pid_q.max_out = INV_PID_MAX_OUT;


    inverter->pid_a.kp = 0;
    inverter->pid_a.ki = 1;
    inverter->pid_a.dt = (float) INV_MAX_PWM_PULSE_VAL / (float) SystemCoreClock;
    inverter->pid_a.integrated = 0;
    inverter->pid_a.max_out = INV_PID_MAX_OUT;

    inverter->pid_b.kp = 0;
    inverter->pid_b.ki = 1;
    inverter->pid_b.dt = (float) INV_MAX_PWM_PULSE_VAL / (float) SystemCoreClock;
    inverter->pid_b.integrated = 0;
    inverter->pid_b.max_out = INV_PID_MAX_OUT;

    inv_set_fault();
    inverter->adc_readings_ready = false;
    inv_start(inverter);

    inverter->main_status = INV_STATUS_IDLE;

    env_init(&inverter->stimuli);
    inverter->stimuli.inputs = &inverter->inputs;

    return INV_OK;
}


inv_ret_val_t inv_start(inv_t * inv)
{
    if(!inv)
{
    return INV_FAIL;
}

    HAL_TIM_Base_Start_IT(inv->timer);

    return INV_OK;
}

#define READ_SPI_MAX_CLK_DELAY 72000  //1ms
uint16_t spi_read_word(SPI_TypeDef *spi)
{
    while (spi->SR & SPI_SR_BSY)
    {
    }
    *(volatile uint16_t*)(&spi->DR) = 0xffff;
    uint32_t start_tick = DWT->CYCCNT;
    uint32_t current_tick = DWT->CYCCNT;
    while (!(spi->SR & SPI_SR_RXNE) && (current_tick-start_tick)<READ_SPI_MAX_CLK_DELAY)
    {
        current_tick = DWT->CYCCNT;
    }

    if((current_tick-start_tick)>READ_SPI_MAX_CLK_DELAY)
    {
        Error_Handler();
    }
    uint16_t data = *(volatile uint16_t*)(&spi->DR);

    return data;
}

#define RES_OFFSET 2*(float) M_PI-5.51f

inline float wrap(float new, float old)
{
    if(new-old < -(float) M_PI) {
        return new + 2 * (float) M_PI;
    }
    else if(new-old > (float) M_PI){
        return new + 2 * -(float) M_PI;
    } else
    {
        return new;
    }
}

void res_init(resolver_t * res)
{
    HAL_GPIO_WritePin(RESET_RES_GPIO_Port, RESET_RES_Pin, false);
    HAL_Delay(10);
    HAL_GPIO_WritePin(RESET_RES_GPIO_Port, RESET_RES_Pin, true);

    CLEAR_BIT(SPI1->CR1, SPI_CR1_BIDIOE);
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);
}

void res_read_position(resolver_t *res) {
    // TODO: simplify GPIO toggling
    HAL_GPIO_WritePin(SAMPLE_GPIO_Port, SAMPLE_Pin, 0);
    inv_delay_cycles(3);
    HAL_GPIO_WritePin(SAMPLE_GPIO_Port, SAMPLE_Pin, 1);
    inv_delay_cycles(3);
    HAL_GPIO_WritePin(RDVEL_GPIO_Port, RDVEL_Pin, 1);
    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 1);
    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 0);

     static volatile float resolver_offset = RES_OFFSET;
//    static volatile float resolver_offset = -2.9f-3.14152f/2+0.1f;  // Silnik w samochodzie

    uint16_t pos = spi_read_word(res->spi_handler->Instance) >> 4;

    //    res->fi = (float) pos / 4096.f * 2 * (float) M_PI + resolver_offset;  //w samochodzie





    float new_fi = (1-(float) pos / 4096.f )* 2 * (float) M_PI + resolver_offset;


//    if(new_fi-res->fi>)

//    new_fi = wrap(new_fi, res->fi);

    float fi_delta = (new_fi - res->fi) ;


    if (fi_delta < -(float)M_PI ) {
        fi_delta += 2 * (float)M_PI  ;
    }
    if (fi_delta > (float)M_PI ) {
        fi_delta  -= 2 * (float)M_PI ;
    }
    res->derived_electrical_velocity_rad_s = fi_delta * INV_LOOP_SPEED;
    res->derived_mechanical_velocity_rad_s = res->derived_electrical_velocity_rad_s/ INV_MOTOR_POLES ; // Calculate motor velocity in rad/s

    res->fi = new_fi;

    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 1);
    inv_delay_cycles(1);
    HAL_GPIO_WritePin(RDVEL_GPIO_Port, RDVEL_Pin, 0);
    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 0);
    int16_t speed = (int16_t) (spi_read_word(res->spi_handler->Instance) & 0xfff0) / 16; // Zrob cos z tym
//    res->velocity = speed * 7; // TODO: rad/s, find a better factor

    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 1);
    HAL_GPIO_WritePin(SAMPLE_GPIO_Port, SAMPLE_Pin, 1);
}

bool inv_get_fault() {
    return HAL_GPIO_ReadPin(FAULT_SR_3V3_GPIO_Port, FAULT_SR_3V3_Pin);
}

void inv_clear_fault() {
    HAL_GPIO_WritePin(FAULT_RST_GPIO_Port, FAULT_RST_Pin, true);
    HAL_Delay(10);
    HAL_GPIO_WritePin(FAULT_RST_GPIO_Port, FAULT_RST_Pin, false);
}

void inv_set_fault() {
    HAL_GPIO_WritePin(GPIO_A_GPIO_Port, GPIO_A_Pin, true);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIO_A_GPIO_Port, GPIO_A_Pin, false);
}

inline float constrain(float x, const float min, const float max) {
    if (x > max) return max;
    if (x < min) return min;
    return x;
}


void inv_set_pwm(inv_t *inverter, float u, float v, float w) {
    inverter->timer->Instance->CCR1 = INV_MAX_PWM_PULSE_VAL * (0.5f + u / 2.0f);
    inverter->timer->Instance->CCR2 = INV_MAX_PWM_PULSE_VAL * (0.5f + v / 2.0f);
    inverter->timer->Instance->CCR3 = INV_MAX_PWM_PULSE_VAL * (0.5f + w / 2.0f);
}

typedef enum {
    MEASURED_CURRENT_D = 1,
    MEASURED_CURRENT_Q,
    FI,
    SET_VOLTAGE_D,
    SET_VOLTAGE_Q,
    CALCULATED_VELOCITY,
    BUS_VOLTAGE
} graph_channel_t;

static void inv_send_trace_data(inv_t *inverter) {
    static int tick = 0;
    ++tick;

    if (tick > TRACE_FREQUENCY_DIVIDER) {
        tick = 0;

        swo_send_float(MEASURED_CURRENT_D, inverter->current.x);
        swo_send_float(MEASURED_CURRENT_Q, inverter->current.y);
        swo_send_float(FI, inverter->resolver.fi);
        swo_send_float(SET_VOLTAGE_D, inverter->voltage.x);
        swo_send_float(SET_VOLTAGE_Q, inverter->voltage.y);
//        swo_send_float(CALCULATED_VELOCITY, inverter->resolver.velocity/100.f);
        swo_send_float(BUS_VOLTAGE, inverter->vbus);
    }
}

static uint32_t start_times[1024];

void inv_pwm_tick(inv_t *inverter) {
    static uint32_t counter123 = 0;


    res_read_position(&inverter->resolver); //Change speed calculation method
    vec_t phi = angle(inverter->resolver.fi);

    static volatile abc_t current_abc;
    static volatile vec_t current_ab;
    static volatile vec_t current_dq;

    current_abc = inv_read_current(inverter);
    current_ab = clarkeTransform(current_abc);
    current_dq = parkTransform(current_ab, phi);

    inverter->current.x = iir_filter_calculate(&inverter->filter_d, current_dq.x);
    inverter->current.y = iir_filter_calculate(&inverter->filter_q, current_dq.y);


    static volatile float setpoint_alpha = 0.02f;
    inverter->smooth_set_current.x = (setpoint_alpha * inverter->set_value.x) + (1.0f - setpoint_alpha) * inverter->smooth_set_current.x;
    inverter->smooth_set_current.y = (setpoint_alpha * inverter->set_value.y) + (1.0f - setpoint_alpha) * inverter->smooth_set_current.y;

    if (inverter->motor_control_mode == MODE_DQ)
    {
        inverter->voltage = (vec_t){
            pid_calc(&inverter->pid_d, inverter->current.x, inverter->smooth_set_current.x),
            pid_calc(&inverter->pid_q, inverter->current.y, inverter->smooth_set_current.y),
        };



        vec_t pwm = {
            inverter->voltage.x / inverter->vbus,
            inverter->voltage.y / inverter->vbus,
        };

        pwm = limit_amplitude(pwm, 1);

        //Add phi correction as a function of speed
        pwm = inverseParkTransform(pwm, phi);

        static volatile abc_t pwmABC = {0};
        pwmABC = inverseClarkeTransform(pwm);

        inv_set_pwm(inverter, pwmABC.a, pwmABC.b, pwmABC.c);


    } else if (inverter->motor_control_mode == MODE_AB) {
        inverter->voltage = (vec_t){
            pid_calc(&inverter->pid_a, current_ab.x, inverter->set_value.x),
            pid_calc(&inverter->pid_b, current_ab.y, inverter->set_value.y),
        };

        vec_t pwm = {
            inverter->voltage.x / inverter->vbus,
            inverter->voltage.y / inverter->vbus,
        };
        pwm = limit_amplitude(pwm, 1);
        abc_t pwmABC = inverseClarkeTransform(pwm);
        inv_set_pwm(inverter, pwmABC.a, pwmABC.b, pwmABC.c);
    } else if(inverter->motor_control_mode == MODE_DQ_FREQUENCY){
        static float fi = 0;
        fi += 2.f * 3.14152f * inverter->frequency_setpoint / INV_LOOP_SPEED ;
        if(fi >=2*3.14152f) fi = 0;
        float a = sinf(fi);
        inverter->voltage = (vec_t){
                a * inverter->set_value.x,
                a * inverter->set_value.y,
        };

        vec_t pwm = {
                inverter->voltage.x / inverter->vbus,
                inverter->voltage.y / inverter->vbus,
        };
        pwm = limit_amplitude(pwm, 1);
        pwm = inverseParkTransform(pwm, phi);
        abc_t pwmABC = inverseClarkeTransform(pwm);
        inv_set_pwm(inverter, pwmABC.a, pwmABC.b, pwmABC.c);


        if(fast_data.x_channel==CHANNEL_0)
        {
            fdl_data_t data = {inverter->voltage.y,inverter->current.y};
            fdl_add_datapoint(&fast_data, &data);

        } else {
            fdl_data_t data = {inverter->voltage.x,inverter->current.x};
            fdl_add_datapoint(&fast_data, &data);
        }


    }

    inv_send_trace_data(inverter);

//    HAL_GPIO_WritePin(X_OUT_GPIO_Port, X_OUT_Pin, false);

    start_times[(counter123++) % 1024] = DWT->CYCCNT;

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
//        HAL_GPIO_WritePin(X_OUT_GPIO_Port, X_OUT_Pin, true);
        inv_pwm_tick(&inv);
//        res_read_position(&inv.resolver);
    }
}

int32_t inv_calibrate_current(inv_t *inverter) {
    if (inverter->active) return -1;

    const uint16_t avg_cycles = 256;
    static uint32_t sum[2];
    sum[0] = 0;
    sum[1] = 0;
    HAL_Delay(1);

//    while (!inverter->adc_readings_ready);
    for (int i = 0; i < avg_cycles; i++) {
        HAL_ADC_Start_DMA(inverter->current_adc,sum,2);
        HAL_ADC_PollForConversion(inverter->current_adc, 100);
        sum[0] += inverter->raw_current_adc[0];
        sum[1] += inverter->raw_current_adc[1];
        HAL_Delay(1);
    }
    sum[0] /= avg_cycles;
    sum[1] /= avg_cycles;

    const uint32_t MIN_OFFSET = 1500;
    const uint32_t MAX_OFFSET = 2500;

    if (sum[0] > MAX_OFFSET || sum[0] < MIN_OFFSET) return -1;
    if (sum[1] > MAX_OFFSET || sum[1] < MIN_OFFSET) return -1;

    inverter->current_adc_offset[0] = sum[0];
    inverter->current_adc_offset[1] = sum[1];

    return 0;
}


abc_t inv_read_current(inv_t *inverter) {
    const float amps_per_bit = 1.15f;
    const float symmetry_compensation = 1.00f;
    abc_t result;
    result.c = (float) (inverter->raw_current_adc[1] - inverter->current_adc_offset[1]) * amps_per_bit;
    result.b = symmetry_compensation * (float) (inverter->raw_current_adc[0] - inverter->current_adc_offset[0]) *
               amps_per_bit;
    result.a = -result.b - result.c;

    return result;
}

void inv_enable(inv_t *inverter, bool status) {
    if (status) {
        if(!inverter->active)
        {
            HAL_TIM_PWM_Start(inverter->timer, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Start(inverter->timer, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(inverter->timer, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Start(inverter->timer, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(inverter->timer, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(inverter->timer, TIM_CHANNEL_3);

            for(int i = 0; i< 300; i++)                 //IT CAN'T BE DONE THIS WAY. Add a state machine waiting for it to be done
            {
                adc4_read(&inverter->inputs);
            }
            inv_reset_controllers(inverter);
            inv_clear_fault();

        }

    } else {
        if(inverter->active)
        {
            inv_set_fault();
            inv_reset_controllers(inverter);
            inverter->set_value = (vec_t){0,0};
            HAL_TIM_PWM_Stop(inverter->timer, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(inverter->timer, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(inverter->timer, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(inverter->timer, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(inverter->timer, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(inverter->timer, TIM_CHANNEL_3);
        }

    }
    inverter->active = status;
}

void inv_set_mode_and_current(inv_t *inverter, inverter_mode_t mode, vec_t current)
{
    inverter->motor_control_mode = mode;
    inverter->set_value = current;


}

void inv_vbus_update(inv_t * inverter)
{
    float current_vbus = inverter->inputs.bus_voltage;
#ifdef BENCH_DEBUG_MODE
//    float current_vbus = 40;
#endif

    inverter->vbus = current_vbus;

    if(inverter->inputs.bus_voltage < 30.f)
    {
        #ifdef BENCH_DEBUG_MODE
        inv_command_state(inverter, INV_COMMAND_IDLE);
        #endif
    }

}

void inv_temperature_check(inv_t * inverter)
{
    if( inverter->inputs.igbt_A_temperature > INV_MAX_TEMPERATURE_DISABLE)
    {
        #ifdef BENCH_DEBUG_MODE
        inv_command_state(inverter,INV_COMMAND_IDLE);
        inv_enable(inverter,false);
        #endif

    }

}


/*
 * This function runs in certain intervals. Less often than main tick. Not time critical
 */
void inv_auxiliary_tick(inv_t * inverter)
{
    volatile static uint32_t tick_counter = 0;

//    HAL_GPIO_WritePin(GPIO)

    if(!inverter)
    {
        return;
    }


    adc4_read(&inverter->inputs);
    adc2_read(&inverter->inputs);
    //Make it more human

    env_update(&inverter->stimuli);


    inv_vbus_update(inverter);


    //Signal that ADC reading has already acquired enough samples
    if(tick_counter > 20)
    {
        inv.adc_readings_ready = true;
    }


//    if(tick_counter%10)
//    {
//        printf(" test %d \r\n", tick_counter);
//    }

    tick_counter++;
}


#define INV_PRECHARGE_WAIT_TIME 5000

//Remember to start the ADCS
inv_ret_val_t inv_connect_supply(inv_t * inverter)
{
    float initial_voltage = inverter->inputs.bus_voltage;
    HAL_GPIO_WritePin(inverter->relay_box.precharge_contactor.port, inverter->relay_box.precharge_contactor.pin, 1);
    printf("Precharge engaged \n");

    uint32_t start_time = HAL_GetTick();
    uint32_t current_time = start_time;

    adc4_read(&inverter->inputs);

    float current_voltage = inverter->inputs.bus_voltage;

    while(inverter->inputs.bus_voltage < ENV_MIN_VBUS_VALUE && (current_time - start_time) < INV_PRECHARGE_WAIT_TIME) //later change to derivative
    {

        current_time = HAL_GetTick();
        current_voltage = inverter->inputs.bus_voltage;
        HAL_Delay(1);

    }


    if(inverter->inputs.bus_voltage < ENV_MIN_VBUS_VALUE)
    {
        HAL_GPIO_WritePin(inverter->relay_box.precharge_contactor.port, inverter->relay_box.precharge_contactor.pin, 0);

        inverter->power_status = INV_POWER_DISCONNECTED;
        log_info("Precharge failed. TIMEOUT");
        return INV_PRECHARGE_FAIL;
    }
    HAL_Delay(500);

    HAL_GPIO_WritePin(inverter->relay_box.main_contactor.port, inverter->relay_box.main_contactor.pin, 0);  //Contactor enabled

    HAL_Delay(50);
    HAL_GPIO_WritePin(inverter->relay_box.precharge_contactor.port, inverter->relay_box.precharge_contactor.pin, 0);
    HAL_Delay(100);

    inverter->power_status = INV_POWER_ENGAGED;

    return INV_OK;
}
inv_ret_val_t inv_disconnect_supply(inv_t * inverter)
{

    HAL_GPIO_WritePin(inverter->relay_box.main_contactor.port, inverter->relay_box.main_contactor.pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(inverter->relay_box.precharge_contactor.port, inverter->relay_box.precharge_contactor.pin, 0);
    printf("Contactor disengaged\n");
    inverter->power_status = INV_POWER_DISCONNECTED;

    HAL_Delay(500);

    return INV_OK;
}

#define INV_THROTTLE_THRESHOLD 0.2f
#define INV_THROTTLE_TRIGGER 1.5f

float inv_get_throttle(inv_t * inverter)
{
    float input = (inverter->inputs.throttle_b_voltage - INV_THROTTLE_THRESHOLD)/(INV_THROTTLE_TRIGGER-INV_THROTTLE_THRESHOLD);

    if(input>1.f) input = 1.f;
    if(input<1.f) input = 0.f;

    return input;


}

