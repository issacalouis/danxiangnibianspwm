#ifndef __BOOST_H
#define __BOOST_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ============================================================
 * Single-phase inverter SPWM voltage/current dual-loop control
 *
 * PWM bridge A : TIM1_CH1  -> PE9, TIM1_CH1N -> PA7
 * PWM bridge B : TIM8_CH2  -> PC7, TIM8_CH2N -> PB0
 * ADC output V : ADC1_CH5  -> PA5, V = (Vadc - 1.72) * 55.8
 * ADC output I : ADC2_CH14 -> PC4, I = (Vadc - 1.72) * 2.82
 * ADC input  I : ADC2_CH15 -> PC5, I = (Vadc - 1.72) * 2.82
 * ============================================================ */

#define INV_PWM_ARR                 (1679U)
#define INV_DUTY_MIN                (0.05f)
#define INV_DUTY_MAX                (0.95f)
#define INV_DUTY_CENTER             (0.50f)

#define INV_CONTROL_RATE_HZ         (2000.0f)
#define INV_OUTPUT_FREQ_HZ          (50.0f)
#define INV_TWO_PI                  (6.28318530718f)

#define ADC_VREF                    (3.3f)
#define ADC_RESOLUTION              (4095.0f)
#define SAMPLE_BIAS                 (1.72f)
#define VOLTAGE_SAMPLE_RATIO        (55.8f)
#define CURRENT_SAMPLE_RATIO        (2.82f)
#define ADC_FILTER_ALPHA            (0.2f)

#define VAC_TARGET_DEFAULT          (12.0f)
#define VAC_TARGET_MIN              (0.0f)
#define VAC_TARGET_MAX              (60.0f)
#define VAC_TARGET_STEP             (1.0f)

#define IOUT_LIMIT_DEFAULT          (2.0f)
#define IOUT_LIMIT_MIN              (0.2f)
#define IOUT_LIMIT_MAX              (10.0f)
#define IOUT_LIMIT_STEP             (0.2f)
#define IIN_OC_LIMIT_DEFAULT        (8.0f)

#define V_LOOP_KP                   (0.020f)
#define V_LOOP_KI                   (0.001f)
#define I_LOOP_KP                   (0.030f)
#define I_LOOP_KI                   (0.002f)
#define PI_INTEGRAL_MAX             (1.0f)
#define PI_INTEGRAL_MIN            (-1.0f)

typedef struct {
    float kp;
    float ki;
    float integral;
    float integral_max;
    float integral_min;
    float output;
} PI_Controller_t;

typedef enum {
    BOOST_EDIT_VOLTAGE = 0,
    BOOST_EDIT_CURRENT = 1,
} Boost_EditMode_t;

typedef struct {
    float v_target_rms;
    float i_limit;
    float iin_oc_limit;
    float v_out;
    float v_out_raw;
    float i_out;
    float i_out_raw;
    float i_in;
    float i_in_raw;
    float phase;
    float modulation;
    float duty_a;
    float duty_b;
    uint8_t fault_oc;
    Boost_EditMode_t edit_mode;
    uint32_t adc_vout_raw;
    uint32_t adc_iout_raw;
    uint32_t adc_iin_raw;
    PI_Controller_t v_pi;
    PI_Controller_t i_pi;
} Boost_t;

extern Boost_t boost;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

void Boost_Init(void);
void Boost_SetTarget(float v_target_rms);
void Boost_SetCurrentLimit(float i_limit);
void Boost_ClearFault(void);
void Boost_ControlLoop(void);
void Boost_KeyScan(void);
float Boost_GetTarget(void);
float Boost_GetVout(void);
float Boost_GetIout(void);
float Boost_GetIin(void);
float Boost_GetCurrentLimit(void);
float Boost_GetDuty(void);
uint8_t Boost_GetFault(void);
Boost_EditMode_t Boost_GetEditMode(void);

#endif /* __BOOST_H */
