#include "boost.h"
#include "main.h"
#include "gpio.h"

/* ============================================================
 *  boost.c - Boost closed-loop voltage control (PI)
 *
 *  Call sequence:
 *    1. Call Boost_Init() in main() to initialize PWM and ADC.
 *    2. Call Boost_ControlLoop() periodically, recommended 1 ms.
 *    3. Call Boost_KeyScan() slowly in the main loop to adjust target voltage.
 * ============================================================ */

/* ---------- Global instance ---------- */
Boost_t boost = {
    .v_target = VOUT_TARGET_DEFAULT,
    .v_out    = 0.0f,
    .duty     = 0.0f,
    .adc_raw  = 0,
    .pi = {
        .kp           = PI_KP,
        .ki           = PI_KI,
        .integral     = 0.0f,
        .integral_max = PI_INTEGRAL_MAX,
        .integral_min = PI_INTEGRAL_MIN,
        .output       = 0.0f,
    },
};

/* Convert ADC sample to actual output voltage. */
static float ADC_ToVout(uint32_t raw)
{
    float vadc = (float)raw * ADC_VREF / ADC_RESOLUTION;
    float vout = (vadc - SAMPLE_BIAS) * SAMPLE_RATIO;
    if (vout < 0.0f) vout = 0.0f;
    return vout;
}

/* Set TIM1 CH1 duty. */
static void PWM_SetDuty(float duty)
{
    if (duty > BOOST_DUTY_MAX) duty = BOOST_DUTY_MAX;
    if (duty < BOOST_DUTY_MIN) duty = BOOST_DUTY_MIN;

    boost.duty = duty;
    uint32_t ccr = (uint32_t)(duty * (float)(BOOST_PWM_ARR + 1));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}

void Boost_Init(void)
{
    /* Start TIM1 CH1 PWM (PE9), and complementary CH1N (PA7). */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

    PWM_SetDuty(BOOST_DUTY_MIN);

    /* Start ADC in polling mode. */
    HAL_ADC_Start(&hadc1);
}

void Boost_SetTarget(float v_target)
{
    if (v_target > VOUT_TARGET_MAX) v_target = VOUT_TARGET_MAX;
    if (v_target < VOUT_TARGET_MIN) v_target = VOUT_TARGET_MIN;

    boost.v_target = v_target;

    /* Reset integral when target changes to avoid windup. */
    boost.pi.integral = 0.0f;
}

void Boost_ControlLoop(void)
{
    /* 1. ADC sampling */
    HAL_ADC_PollForConversion(&hadc1, 1);
    boost.adc_raw = HAL_ADC_GetValue(&hadc1);
    boost.v_out = ADC_ToVout(boost.adc_raw);
    HAL_ADC_Start(&hadc1);

    /* 2. Error */
    float error = boost.v_target - boost.v_out;

    /* 3. Integral with limit */
    boost.pi.integral += boost.pi.ki * error;
    if (boost.pi.integral > boost.pi.integral_max)
        boost.pi.integral = boost.pi.integral_max;
    if (boost.pi.integral < boost.pi.integral_min)
        boost.pi.integral = boost.pi.integral_min;

    /* 4. PI output, added to current duty */
    float delta = boost.pi.kp * error + boost.pi.integral;
    float new_duty = boost.duty + delta;

    /* 5. Limit output and update PWM */
    PWM_SetDuty(new_duty);
}

void Boost_KeyScan(void)
{
    static uint8_t key_last = 0;
    uint8_t key = get_keyboard_value();

    /* Only act once when a new key value is detected. */
    if (key != key_last) {
        if (key == '1') {
            Boost_SetTarget(boost.v_target + VOUT_TARGET_STEP);
        } else if (key == '2') {
            Boost_SetTarget(boost.v_target - VOUT_TARGET_STEP);
        }
    }

    key_last = key;
}

float Boost_GetVout(void)   { return boost.v_out;    }
float Boost_GetDuty(void)   { return boost.duty;     }
float Boost_GetTarget(void) { return boost.v_target; }
