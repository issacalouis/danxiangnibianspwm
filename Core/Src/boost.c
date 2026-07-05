#include "boost.h"
#include "main.h"
#include "gpio.h"
#include <math.h>

Boost_t boost = {
    .v_target_rms = VAC_TARGET_DEFAULT,
    .i_limit = IOUT_LIMIT_DEFAULT,
    .v_out = 0.0f,
    .v_out_raw = 0.0f,
    .i_out = 0.0f,
    .i_out_raw = 0.0f,
    .i_in = 0.0f,
    .i_in_raw = 0.0f,
    .phase = 0.0f,
    .modulation = 0.0f,
    .duty_a = INV_DUTY_CENTER,
    .duty_b = INV_DUTY_CENTER,
    .edit_mode = BOOST_EDIT_VOLTAGE,
    .adc_vout_raw = 0U,
    .adc_iout_raw = 0U,
    .adc_iin_raw = 0U,
    .v_pi = {
        .kp = V_LOOP_KP,
        .ki = V_LOOP_KI,
        .integral = 0.0f,
        .integral_max = PI_INTEGRAL_MAX,
        .integral_min = PI_INTEGRAL_MIN,
        .output = 0.0f,
    },
    .i_pi = {
        .kp = I_LOOP_KP,
        .ki = I_LOOP_KI,
        .integral = 0.0f,
        .integral_max = PI_INTEGRAL_MAX,
        .integral_min = PI_INTEGRAL_MIN,
        .output = 0.0f,
    },
};


static float clampf(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float adc_to_voltage(uint32_t raw)
{
    float vadc = (float)raw * ADC_VREF / ADC_RESOLUTION;
    return (vadc - SAMPLE_BIAS) * VOLTAGE_SAMPLE_RATIO;
}

static float adc_to_current(uint32_t raw)
{
    float vadc = (float)raw * ADC_VREF / ADC_RESOLUTION;
    return (vadc - SAMPLE_BIAS) * CURRENT_SAMPLE_RATIO;
}

static float pi_step(PI_Controller_t *pi, float error)
{
    pi->integral += pi->ki * error;
    pi->integral = clampf(pi->integral, pi->integral_min, pi->integral_max);
    pi->output = pi->kp * error + pi->integral;
    return pi->output;
}

static void pi_reset(PI_Controller_t *pi)
{
    pi->integral = 0.0f;
    pi->output = 0.0f;
}

static void pwm_write(float duty_a, float duty_b)
{
    duty_a = clampf(duty_a, INV_DUTY_MIN, INV_DUTY_MAX);
    duty_b = clampf(duty_b, INV_DUTY_MIN, INV_DUTY_MAX);

    boost.duty_a = duty_a;
    boost.duty_b = duty_b;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                          (uint32_t)(duty_a * (float)(INV_PWM_ARR + 1U)));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,
                          (uint32_t)(duty_b * (float)(INV_PWM_ARR + 1U)));

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (INV_PWM_ARR + 1U) / 2U);
}

static void inverter_shutdown(void)
{
    boost.modulation = 0.0f;
    pwm_write(INV_DUTY_CENTER, INV_DUTY_CENTER);
    pi_reset(&boost.v_pi);
    pi_reset(&boost.i_pi);
}

void Boost_Init(void)
{
    inverter_shutdown();
    if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADC_Start_IT(&hadc2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADC_Start_IT(&hadc3) != HAL_OK) {
        Error_Handler();
    }

    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    __HAL_TIM_SET_COUNTER(&htim8, 0U);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        Error_Handler();
    }
}

void Boost_SetTarget(float v_target_rms)
{
    boost.v_target_rms = clampf(v_target_rms, VAC_TARGET_MIN, VAC_TARGET_MAX);
    pi_reset(&boost.v_pi);
}

void Boost_SetCurrentLimit(float i_limit)
{
    boost.i_limit = clampf(i_limit, IOUT_LIMIT_MIN, IOUT_LIMIT_MAX);
    pi_reset(&boost.i_pi);
}

void Boost_ControlLoop(void)
{
    boost.v_out_raw = adc_to_voltage(boost.adc_vout_raw);
    boost.i_out_raw = adc_to_current(boost.adc_iout_raw);
    boost.i_in_raw = adc_to_current(boost.adc_iin_raw);

    boost.v_out += ADC_FILTER_ALPHA * (boost.v_out_raw - boost.v_out);
    boost.i_out += ADC_FILTER_ALPHA * (boost.i_out_raw - boost.i_out);
    boost.i_in += ADC_FILTER_ALPHA * (boost.i_in_raw - boost.i_in);

    boost.phase += INV_TWO_PI * INV_OUTPUT_FREQ_HZ / INV_CONTROL_RATE_HZ;
    if (boost.phase >= INV_TWO_PI) {
        boost.phase -= INV_TWO_PI;
    }

    float sine = sinf(boost.phase);
    float v_ref = boost.v_target_rms * 1.41421356f * sine;
    float i_ref_amp = pi_step(&boost.v_pi, v_ref - boost.v_out);
    i_ref_amp = clampf(i_ref_amp, -boost.i_limit, boost.i_limit);

    float i_ref = i_ref_amp;
    float modulation = pi_step(&boost.i_pi, i_ref - boost.i_out);
    modulation = clampf(modulation, -0.96f, 0.96f);
    boost.modulation = modulation;

    float duty_a = INV_DUTY_CENTER + 0.5f * modulation;
    float duty_b = INV_DUTY_CENTER - 0.5f * modulation;
    pwm_write(duty_a, duty_b);
}

void Boost_KeyScan(void)
{
    static uint8_t key_last = 0U;
    uint8_t key = get_keyboard_value();

    if (key != 0U && key != key_last) {
        if (key == 'A') {
            boost.edit_mode = BOOST_EDIT_VOLTAGE;
        } else if (key == 'B') {
            boost.edit_mode = BOOST_EDIT_CURRENT;
        } else if (key == '1') {
            if (boost.edit_mode == BOOST_EDIT_VOLTAGE) {
                Boost_SetTarget(boost.v_target_rms + VAC_TARGET_STEP);
            } else {
                Boost_SetCurrentLimit(boost.i_limit + IOUT_LIMIT_STEP);
            }
        } else if (key == '2') {
            if (boost.edit_mode == BOOST_EDIT_VOLTAGE) {
                Boost_SetTarget(boost.v_target_rms - VAC_TARGET_STEP);
            } else {
                Boost_SetCurrentLimit(boost.i_limit - IOUT_LIMIT_STEP);
            }
        }
    }

    key_last = key;
}

float Boost_GetTarget(void) { return boost.v_target_rms; }
float Boost_GetVout(void) { return boost.v_out; }
float Boost_GetIout(void) { return boost.i_out; }
float Boost_GetIin(void) { return boost.i_in; }
float Boost_GetCurrentLimit(void) { return boost.i_limit; }
float Boost_GetDuty(void) { return boost.modulation; }
Boost_EditMode_t Boost_GetEditMode(void) { return boost.edit_mode; }

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        boost.adc_vout_raw = HAL_ADC_GetValue(hadc);
    } else if (hadc->Instance == ADC2) {
        boost.adc_iout_raw = HAL_ADC_GetValue(hadc);
    } else if (hadc->Instance == ADC3) {
        boost.adc_iin_raw = HAL_ADC_GetValue(hadc);
    }
}

