#include "boost.h"
#include "main.h"
#include "gpio.h"
#include <math.h>

Boost_t boost = {
    .v_target_rms = VAC_TARGET_DEFAULT,
    .v_target_active_rms = 0.0f,
    .i_limit = IOUT_LIMIT_DEFAULT,
    .output_freq_hz = INV_OUTPUT_FREQ_DEFAULT_HZ,
    .v_ref = 0.0f,
    .v_out = 0.0f,
    .v_out_raw = 0.0f,
    .i_out = 0.0f,
    .i_out_raw = 0.0f,
    .i_in = 0.0f,
    .i_in_raw = 0.0f,
    .v_sample_bias = XTQ3_VOLTAGE_BIAS,
    .iout_sample_bias = XTQ3_CURRENT_BIAS,
    .iin_sample_bias = XTQ3_CURRENT_BIAS,
    .outer_integrator = 0.0f,
    .inner_integrator = 0.0f,
    .outer_output = 0.0f,
    .inner_error = 0.0f,
    .qpr_e1 = 0.0f,
    .qpr_e2 = 0.0f,
    .qpr_y1 = 0.0f,
    .qpr_y2 = 0.0f,
    .qpr_b0 = BOOST_QPR_B0,
    .qpr_b1 = BOOST_QPR_B1,
    .qpr_b2 = BOOST_QPR_B2,
    .qpr_a1 = BOOST_QPR_A1,
    .qpr_a2 = BOOST_QPR_A2,
    .phase = 0.0f,
    .modulation = 0.0f,
    .modulation_limit = 0.0f,
    .modulation_peak = 0.0f,
    .modulation_peak_work = 0.0f,
    .duty_a = INV_DUTY_CENTER,
    .duty_b = INV_DUTY_CENTER,
    .arming_ticks = 0U,
    .fault_recover_ticks = 0U,
    .edit_mode = BOOST_EDIT_VOLTAGE,
    .run_state = BOOST_STATE_STOPPED,
    .cal_mode = BOOST_CAL_NONE,
    .adc_vout_raw = 0U,
    .adc_iout_raw = 0U,
    .adc_iin_raw = 0U,
};

static float clampf(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float adc_raw_to_vadc(uint32_t raw)
{
    return (float)raw * ADC_VREF / ADC_RESOLUTION;
}

static float adc_to_voltage(uint32_t raw)
{
    return (adc_raw_to_vadc(raw) - boost.v_sample_bias) * XTQ3_VOLTAGE_GAIN;
}

static float adc_to_current(uint32_t raw, float bias)
{
    return (adc_raw_to_vadc(raw) - bias) * XTQ3_CURRENT_GAIN;
}

static void reset_control_state(void)
{
    boost.v_ref = 0.0f;
    boost.outer_integrator = 0.0f;
    boost.inner_integrator = 0.0f;
    boost.outer_output = 0.0f;
    boost.inner_error = 0.0f;
    boost.qpr_e1 = 0.0f;
    boost.qpr_e2 = 0.0f;
    boost.qpr_y1 = 0.0f;
    boost.qpr_y2 = 0.0f;
    boost.phase = 0.0f;
    boost.modulation = 0.0f;
    boost.modulation_limit = 0.0f;
    boost.modulation_peak = 0.0f;
    boost.modulation_peak_work = 0.0f;
}

static float spwm_triangle_cut_duty(float modulation_ref)
{
    modulation_ref = clampf(modulation_ref,
                            -BOOST_MODULATION_MAX,
                            BOOST_MODULATION_MAX);

    return INV_DUTY_CENTER + 0.5f * modulation_ref;
}

static void pwm_write(float duty_a, float duty_b)
{
    uint32_t arr1 = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t arr8 = __HAL_TIM_GET_AUTORELOAD(&htim8);

    duty_a = clampf(duty_a, INV_DUTY_MIN, INV_DUTY_MAX);
    duty_b = clampf(duty_b, INV_DUTY_MIN, INV_DUTY_MAX);

    boost.duty_a = duty_a;
    boost.duty_b = duty_b;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                          (uint32_t)(duty_a * (float)(arr1 + 1U)));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,
                          (uint32_t)(duty_b * (float)(arr8 + 1U)));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (arr1 + 1U) / 2U);
}

static void inverter_shutdown(void)
{
    reset_control_state();
    pwm_write(INV_DUTY_CENTER, INV_DUTY_CENTER);
}

static void boost_enter_stop(void)
{
    boost.run_state = BOOST_STATE_STOPPED;
    boost.v_target_active_rms = 0.0f;
    boost.arming_ticks = 0U;
    boost.fault_recover_ticks = 0U;
    inverter_shutdown();
}

static void boost_enter_arming(void)
{
    boost.run_state = BOOST_STATE_ARMING;
    boost.v_target_active_rms = 0.0f;
    boost.arming_ticks = 0U;
    boost.fault_recover_ticks = 0U;
    inverter_shutdown();
}

static void boost_enter_run(void)
{
    boost.run_state = BOOST_STATE_RUNNING;
    boost.v_target_active_rms = 0.0f;
    boost.arming_ticks = 0U;
    boost.fault_recover_ticks = 0U;
    reset_control_state();
}

static void boost_enter_fault(void)
{
    boost.run_state = BOOST_STATE_FAULT;
    boost.v_target_active_rms = 0.0f;
    boost.arming_ticks = 0U;
    boost.fault_recover_ticks = 0U;
    inverter_shutdown();
}

static void update_measurements(void)
{
    boost.v_out_raw = adc_to_voltage(boost.adc_vout_raw);
    boost.i_out_raw = adc_to_current(boost.adc_iout_raw, boost.iout_sample_bias);
    boost.i_in_raw = adc_to_current(boost.adc_iin_raw, boost.iin_sample_bias);

    boost.v_out += ADC_FILTER_ALPHA * (boost.v_out_raw - boost.v_out);
    boost.i_out += ADC_FILTER_ALPHA * (boost.i_out_raw - boost.i_out);
    boost.i_in += ADC_FILTER_ALPHA * (boost.i_in_raw - boost.i_in);
}

static void clear_qpr_history(void)
{
    boost.qpr_e1 = 0.0f;
    boost.qpr_e2 = 0.0f;
    boost.qpr_y1 = 0.0f;
    boost.qpr_y2 = 0.0f;
}

static void update_qpr_coefficients(float frequency_hz)
{
    float t = INV_CONTROL_TS;
    float wc = INV_TWO_PI * BOOST_QPR_BANDWIDTH_HZ;
    float w0 = INV_TWO_PI * frequency_hz;
    float w0t = w0 * t;
    float w0t2 = w0t * w0t;
    float den = 4.0f + 4.0f * wc * t + w0t2;
    float b0 = (4.0f * BOOST_QPR_KR * wc * t) / den;

    boost.qpr_b0 = b0;
    boost.qpr_b1 = 0.0f;
    boost.qpr_b2 = -b0;
    boost.qpr_a1 = (-8.0f + 2.0f * w0t2) / den;
    boost.qpr_a2 = (4.0f - 4.0f * wc * t + w0t2) / den;
}

static uint8_t overcurrent_active(void)
{
    return (fabsf(boost.i_out_raw) >= boost.i_limit ||
            fabsf(boost.i_out) >= boost.i_limit) ? 1U : 0U;
}

static uint8_t overcurrent_cleared(void)
{
    float recover_level = boost.i_limit - IOUT_RECOVER_HYSTERESIS_A;
    if (recover_level < IOUT_LIMIT_MIN) {
        recover_level = IOUT_LIMIT_MIN;
    }
    return (fabsf(boost.i_out) <= recover_level) ? 1U : 0U;
}

static float qpr_resonant_predict(float error)
{
    float resonant = boost.qpr_b0 * error
                   + boost.qpr_b1 * boost.qpr_e1
                   + boost.qpr_b2 * boost.qpr_e2
                   - boost.qpr_a1 * boost.qpr_y1
                   - boost.qpr_a2 * boost.qpr_y2;

    return clampf(resonant,
                  -BOOST_QPR_RESONANT_MAX_V,
                  BOOST_QPR_RESONANT_MAX_V);
}

static void qpr_resonant_commit(float error, float resonant)
{
    boost.qpr_e2 = boost.qpr_e1;
    boost.qpr_e1 = error;
    boost.qpr_y2 = boost.qpr_y1;
    boost.qpr_y1 = resonant;
}

static void update_soft_start(void)
{
    float step = BOOST_SOFT_START_V_PER_S * INV_CONTROL_TS;
    float modulation_step = BOOST_MODULATION_SOFT_START_PER_S * INV_CONTROL_TS;

    if (boost.v_target_active_rms < boost.v_target_rms) {
        boost.v_target_active_rms += step;
        if (boost.v_target_active_rms > boost.v_target_rms) {
            boost.v_target_active_rms = boost.v_target_rms;
        }
    } else if (boost.v_target_active_rms > boost.v_target_rms) {
        boost.v_target_active_rms -= step;
        if (boost.v_target_active_rms < boost.v_target_rms) {
            boost.v_target_active_rms = boost.v_target_rms;
        }
    }

    if (boost.modulation_limit < BOOST_MODULATION_MAX) {
        boost.modulation_limit += modulation_step;
        if (boost.modulation_limit > BOOST_MODULATION_MAX) {
            boost.modulation_limit = BOOST_MODULATION_MAX;
        }
    }
}

static uint8_t arming_samples_ok(void)
{
    return (fabsf(boost.v_out) <= BOOST_ARMING_V_MAX &&
            fabsf(boost.i_out) <= BOOST_ARMING_I_MAX &&
            fabsf(boost.i_in) <= BOOST_ARMING_I_MAX) ? 1U : 0U;
}

static void update_modulation_peak(float modulation, uint8_t phase_wrapped)
{
    float abs_m = fabsf(modulation);

    if (abs_m > boost.modulation_peak_work) {
        boost.modulation_peak_work = abs_m;
    }

    if (phase_wrapped) {
        boost.modulation_peak = boost.modulation_peak_work;
        boost.modulation_peak_work = 0.0f;
    }
}

static void adjust_current_cal_bias(float delta)
{
    if (boost.run_state != BOOST_STATE_STOPPED) return;

    if (boost.cal_mode == BOOST_CAL_VOLTAGE) {
        boost.v_sample_bias = clampf(boost.v_sample_bias + delta, 0.0f, ADC_VREF);
    } else if (boost.cal_mode == BOOST_CAL_IOUT) {
        boost.iout_sample_bias = clampf(boost.iout_sample_bias + delta, 0.0f, ADC_VREF);
    } else if (boost.cal_mode == BOOST_CAL_IIN) {
        boost.iin_sample_bias = clampf(boost.iin_sample_bias + delta, 0.0f, ADC_VREF);
    }
}

static void next_cal_mode(void)
{
    if (boost.run_state != BOOST_STATE_STOPPED) return;

    if (boost.cal_mode == BOOST_CAL_NONE) {
        boost.cal_mode = BOOST_CAL_VOLTAGE;
    } else if (boost.cal_mode == BOOST_CAL_VOLTAGE) {
        boost.cal_mode = BOOST_CAL_IOUT;
    } else if (boost.cal_mode == BOOST_CAL_IOUT) {
        boost.cal_mode = BOOST_CAL_IIN;
    } else {
        boost.cal_mode = BOOST_CAL_NONE;
    }
}

void Boost_Init(void)
{
    update_qpr_coefficients(boost.output_freq_hz);
    boost_enter_stop();

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
}

void Boost_SetCurrentLimit(float i_limit)
{
    boost.i_limit = clampf(i_limit, IOUT_LIMIT_MIN, IOUT_LIMIT_MAX);
}

void Boost_SetOutputFrequency(float freq_hz)
{
    float new_freq = clampf(freq_hz, INV_OUTPUT_FREQ_MIN_HZ, INV_OUTPUT_FREQ_MAX_HZ);

    if (new_freq != boost.output_freq_hz) {
        boost.output_freq_hz = new_freq;
        update_qpr_coefficients(new_freq);
        clear_qpr_history();
    }
}

void Boost_ControlLoop(void)
{
    update_measurements();

    if (boost.run_state == BOOST_STATE_FAULT) {
        inverter_shutdown();
        if (overcurrent_cleared()) {
            boost.fault_recover_ticks++;
            if (boost.fault_recover_ticks >= BOOST_FAULT_RECOVER_TICKS) {
                boost_enter_arming();
            }
        } else {
            boost.fault_recover_ticks = 0U;
        }
        return;
    }

    if (boost.run_state == BOOST_STATE_STOPPED) {
        boost.v_target_active_rms = 0.0f;
        inverter_shutdown();
        return;
    }

    if (overcurrent_active()) {
        boost_enter_fault();
        return;
    }

    if (boost.run_state == BOOST_STATE_ARMING) {
        inverter_shutdown();
        if (arming_samples_ok()) {
            boost.arming_ticks++;
            if (boost.arming_ticks >= BOOST_ARMING_TICKS) {
                boost_enter_run();
            }
        } else {
            boost.arming_ticks = 0U;
        }
        return;
    }

    update_soft_start();

    uint8_t phase_wrapped = 0U;
    boost.phase += INV_TWO_PI * boost.output_freq_hz * INV_CONTROL_TS;
    if (boost.phase >= INV_TWO_PI) {
        boost.phase -= INV_TWO_PI;
        phase_wrapped = 1U;
    }

    boost.v_ref = boost.v_target_active_rms * INV_SQRT2 * sinf(boost.phase);

    float v_err = boost.v_ref - boost.v_out;
    float resonant = qpr_resonant_predict(v_err);
    float inverter_voltage_cmd = boost.v_ref
                               + BOOST_QPR_KP * v_err
                               + resonant
                               - BOOST_QPR_IOUT_DAMPING_V_PER_A * boost.i_out;
    float modulation_unclamped = inverter_voltage_cmd / XTQ3_VBUS_NORM;
    float active_modulation_limit = clampf(boost.modulation_limit,
                                           0.0f,
                                           BOOST_MODULATION_MAX);
    float modulation = clampf(modulation_unclamped,
                              -active_modulation_limit,
                              active_modulation_limit);

    uint8_t high_saturated = (modulation_unclamped > active_modulation_limit) ? 1U : 0U;
    uint8_t low_saturated = (modulation_unclamped < -active_modulation_limit) ? 1U : 0U;
    uint8_t hold_resonant = ((high_saturated && v_err > 0.0f) ||
                             (low_saturated && v_err < 0.0f)) ? 1U : 0U;

    if (hold_resonant) {
        resonant = boost.qpr_y1;
        inverter_voltage_cmd = boost.v_ref
                             + BOOST_QPR_KP * v_err
                             + resonant
                             - BOOST_QPR_IOUT_DAMPING_V_PER_A * boost.i_out;
        modulation_unclamped = inverter_voltage_cmd / XTQ3_VBUS_NORM;
        modulation = clampf(modulation_unclamped,
                            -active_modulation_limit,
                            active_modulation_limit);
    } else {
        qpr_resonant_commit(v_err, resonant);
    }

    boost.outer_output = BOOST_QPR_KP * v_err + resonant;
    boost.inner_error = 0.0f;
    boost.modulation = modulation;
    update_modulation_peak(modulation, phase_wrapped);

    float duty_a = spwm_triangle_cut_duty(modulation);
    float duty_b = spwm_triangle_cut_duty(-modulation);
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
        } else if (key == 'C') {
            next_cal_mode();
        } else if (key == '#') {
            if (boost.run_state == BOOST_STATE_STOPPED) {
                boost.cal_mode = BOOST_CAL_NONE;
            }
            boost.edit_mode = BOOST_EDIT_FREQUENCY;
        } else if (key == 'D') {
            boost.cal_mode = BOOST_CAL_NONE;
            if (boost.run_state == BOOST_STATE_RUNNING ||
                boost.run_state == BOOST_STATE_ARMING ||
                boost.run_state == BOOST_STATE_FAULT) {
                boost_enter_stop();
            } else {
                boost_enter_arming();
            }
        } else if (key == '1') {
            if (boost.run_state == BOOST_STATE_STOPPED && boost.cal_mode != BOOST_CAL_NONE) {
                adjust_current_cal_bias(SAMPLE_BIAS_STEP);
            } else if (boost.edit_mode == BOOST_EDIT_VOLTAGE) {
                Boost_SetTarget(boost.v_target_rms + VAC_TARGET_STEP);
            } else if (boost.edit_mode == BOOST_EDIT_FREQUENCY) {
                Boost_SetOutputFrequency(boost.output_freq_hz + INV_OUTPUT_FREQ_STEP_HZ);
            } else {
                Boost_SetCurrentLimit(boost.i_limit + IOUT_LIMIT_STEP);
            }
        } else if (key == '2') {
            if (boost.run_state == BOOST_STATE_STOPPED && boost.cal_mode != BOOST_CAL_NONE) {
                adjust_current_cal_bias(-SAMPLE_BIAS_STEP);
            } else if (boost.edit_mode == BOOST_EDIT_VOLTAGE) {
                Boost_SetTarget(boost.v_target_rms - VAC_TARGET_STEP);
            } else if (boost.edit_mode == BOOST_EDIT_FREQUENCY) {
                Boost_SetOutputFrequency(boost.output_freq_hz - INV_OUTPUT_FREQ_STEP_HZ);
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
float Boost_GetOutputFrequency(void) { return boost.output_freq_hz; }
float Boost_GetDuty(void) { return boost.modulation_peak; }
Boost_EditMode_t Boost_GetEditMode(void) { return boost.edit_mode; }
Boost_RunState_t Boost_GetRunState(void) { return boost.run_state; }
Boost_CalMode_t Boost_GetCalMode(void) { return boost.cal_mode; }

float Boost_GetCalBias(void)
{
    if (boost.cal_mode == BOOST_CAL_VOLTAGE) return boost.v_sample_bias;
    if (boost.cal_mode == BOOST_CAL_IOUT) return boost.iout_sample_bias;
    if (boost.cal_mode == BOOST_CAL_IIN) return boost.iin_sample_bias;
    return 0.0f;
}

float Boost_GetCalValue(void)
{
    if (boost.cal_mode == BOOST_CAL_VOLTAGE) return boost.v_out;
    if (boost.cal_mode == BOOST_CAL_IOUT) return boost.i_out;
    if (boost.cal_mode == BOOST_CAL_IIN) return boost.i_in;
    return 0.0f;
}

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