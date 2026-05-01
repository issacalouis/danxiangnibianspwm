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

/* Incremental PI state: last control error.
 * Keep it file-local so Boost_ControlLoop() interface stays unchanged.
 */
static float boost_pi_error_last = 0.0f;

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

    /* Reset PI internal state when target changes to avoid stale error history. */
    boost.pi.integral = 0.0f;
    boost.pi.output = boost.duty;
    boost_pi_error_last = 0.0f;
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

    /* 3. Pure incremental PI:
     *      du(k) = Kp * [e(k) - e(k-1)] + Ki * e(k)
     *      u(k)  = u(k-1) + du(k)
     *
     * No separate integral accumulator is used here.  The "integral" effect
     * is naturally accumulated in the output duty itself.
     */
    float delta = boost.pi.kp * (error - boost_pi_error_last)
                + boost.pi.ki * error;
    float new_duty = boost.duty + delta;

    /* 4. Limit output and update PWM */
    PWM_SetDuty(new_duty);

    /* 5. Save state for next incremental PI step.
     * Keep legacy fields consistent for debugging/UI, but they are not used
     * as a positional PI integral any more.
     */
    boost_pi_error_last = error;
    boost.pi.integral = 0.0f;
    boost.pi.output = boost.duty;
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
