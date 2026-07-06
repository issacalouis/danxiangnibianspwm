#ifndef __BOOST_H
#define __BOOST_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ============================================================
 * Single-phase inverter control ported from Simulink/C2000 xtq3
 *
 * PWM bridge A : TIM1_CH1  -> PE9, TIM1_CH1N -> PA7
 * PWM bridge B : TIM8_CH2  -> PC7, TIM8_CH2N -> PB0
 * ADC output V : ADC1_CH5  -> PA5, V = (Vadc - v_bias) * 55.8
 * ADC output I : ADC2_CH14 -> PC4, I = (Vadc - iout_bias) * 2.828
 * ADC input  I : ADC3_CH10 -> PC0, I = (Vadc - iin_bias) * 2.828
 * ============================================================ */

#define INV_DUTY_MIN                (0.02f)
#define INV_DUTY_MAX                (0.98f)
#define INV_DUTY_CENTER             (0.50f)

#define INV_CONTROL_RATE_HZ         (20000.0f)
#define INV_CONTROL_TS              (1.0f / INV_CONTROL_RATE_HZ)
#define INV_OUTPUT_FREQ_HZ          (50.0f)
#define INV_TWO_PI                  (6.28318530718f)
#define INV_SQRT2                   (1.41421356237f)

#define ADC_VREF                    (3.3f)
#define ADC_RESOLUTION              (4095.0f)
#define SAMPLE_BIAS_STEP            (0.0001f)
#define ADC_FILTER_ALPHA            (0.2f)

#define XTQ3_PID1_P                 (0.03115f)
#define XTQ3_PID1_I                 (21.0f)
#define XTQ3_PID2_P                 (27.066f)
#define XTQ3_PID2_I                 (6.67f)
#define XTQ3_VOLTAGE_BIAS           (1.6900f)
#define XTQ3_CURRENT_BIAS           (1.6900f)
#define XTQ3_VOLTAGE_GAIN           (55.8f)
#define XTQ3_CURRENT_GAIN           (2.828f)
#define XTQ3_VBUS_NORM              (18.0f)

#define OUTPUT_L_H                  (0.001f)
#define OUTPUT_C_F                  (0.000010f)

#define BOOST_SOFT_START_V_PER_S    (20.0f)
#define BOOST_MODULATION_MAX        (0.90f)
#define BOOST_MODULATION_SOFT_START_PER_S (1.8f)
#define BOOST_OUTER_INTEGRATOR_MAX  (10.0f)
#define BOOST_INNER_INTEGRATOR_MAX  (20.0f)

#define BOOST_QPR_KP                (0.20f)
#define BOOST_QPR_KR                (3.00f)
#define BOOST_QPR_BANDWIDTH_HZ      (5.0f)
#define BOOST_QPR_B0                (0.0047047086f)
#define BOOST_QPR_B1                (0.0f)
#define BOOST_QPR_B2                (-0.0047047086f)
#define BOOST_QPR_A1                (-1.9966171896f)
#define BOOST_QPR_A2                (0.9968635276f)
#define BOOST_QPR_RESONANT_MAX_V    (6.0f)

#define BOOST_ARMING_TICKS          (1000U)
#define BOOST_ARMING_V_MAX          (5.0f)
#define BOOST_ARMING_I_MAX          (1.0f)

#define VAC_TARGET_DEFAULT          (10.0f)
#define VAC_TARGET_MIN              (0.0f)
#define VAC_TARGET_MAX              (10.0f)
#define VAC_TARGET_STEP             (0.5f)

#define IOUT_LIMIT_DEFAULT          (1.6f)
#define IOUT_LIMIT_MIN              (0.2f)
#define IOUT_LIMIT_MAX              (10.0f)
#define IOUT_LIMIT_STEP             (0.2f)

typedef enum {
    BOOST_EDIT_VOLTAGE = 0,
    BOOST_EDIT_CURRENT = 1,
} Boost_EditMode_t;

typedef enum {
    BOOST_STATE_STOPPED = 0,
    BOOST_STATE_ARMING = 1,
    BOOST_STATE_RUNNING = 2,
} Boost_RunState_t;

typedef enum {
    BOOST_CAL_NONE = 0,
    BOOST_CAL_VOLTAGE = 1,
    BOOST_CAL_IOUT = 2,
    BOOST_CAL_IIN = 3,
} Boost_CalMode_t;

typedef struct {
    float v_target_rms;
    float v_target_active_rms;
    float i_limit;
    float v_ref;
    float v_out;
    float v_out_raw;
    float i_out;
    float i_out_raw;
    float i_in;
    float i_in_raw;
    float v_sample_bias;
    float iout_sample_bias;
    float iin_sample_bias;
    float outer_integrator;
    float inner_integrator;
    float outer_output;
    float inner_error;
    float qpr_e1;
    float qpr_e2;
    float qpr_y1;
    float qpr_y2;
    float phase;
    float modulation;
    float modulation_limit;
    float modulation_peak;
    float modulation_peak_work;
    float duty_a;
    float duty_b;
    uint32_t arming_ticks;
    Boost_EditMode_t edit_mode;
    Boost_RunState_t run_state;
    Boost_CalMode_t cal_mode;
    uint32_t adc_vout_raw;
    uint32_t adc_iout_raw;
    uint32_t adc_iin_raw;
} Boost_t;

extern Boost_t boost;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

void Boost_Init(void);
void Boost_SetTarget(float v_target_rms);
void Boost_SetCurrentLimit(float i_limit);
void Boost_ControlLoop(void);
void Boost_KeyScan(void);
float Boost_GetTarget(void);
float Boost_GetVout(void);
float Boost_GetIout(void);
float Boost_GetIin(void);
float Boost_GetCurrentLimit(void);
float Boost_GetDuty(void);
Boost_EditMode_t Boost_GetEditMode(void);
Boost_RunState_t Boost_GetRunState(void);
Boost_CalMode_t Boost_GetCalMode(void);
float Boost_GetCalBias(void);
float Boost_GetCalValue(void);

#endif /* __BOOST_H */
