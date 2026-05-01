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
    .v_target     = VOUT_TARGET_DEFAULT,
    .v_out        = 0.0f,
    .v_out_raw    = 0.0f,
    .v_in         = VIN_DEFAULT,
    .duty         = 0.0f,
    .adc_raw      = 0,
    .adc_vin_raw  = 0,
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

/* Convert ADC sample to input voltage. */
static float ADC_ToVin(uint32_t raw)
{
    float vadc = (float)raw * ADC_VREF / ADC_RESOLUTION;
    float vin = (vadc - VIN_SAMPLE_BIAS) * VIN_SAMPLE_RATIO;
    if (vin < 0.0f) vin = 0.0f;
    return vin;
}

/* Calculate feedforward duty based on ideal Boost equation: D = 1 - Vin/Vout */
static float Calc_Feedforward(float v_in, float v_target)
{
    if (v_target <= v_in) return BOOST_DUTY_MIN;
    float duty_ff = 1.0f - v_in / v_target;
    if (duty_ff > BOOST_DUTY_MAX) duty_ff = BOOST_DUTY_MAX;
    if (duty_ff < BOOST_DUTY_MIN) duty_ff = BOOST_DUTY_MIN;
    return duty_ff;
}

/* Set TIM1 CH1 duty. */
static void PWM_SetDuty(float duty)
{
    if (duty > BOOST_DUTY_MAX) duty = BOOST_DUTY_MAX;
    if (duty < BOOST_DUTY_MIN) duty = BOOST_DUTY_MIN;

    boost.duty = duty;
    uint32_t ccr = (uint32_t)(duty * (float)(BOOST_PWM_ARR + 1));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);

    /* TIM1_CH2 is used as ADC1 trigger only (no GPIO output).
     * Put the ADC trigger at the middle of the PWM off-time:
     *   trigger = CCR1 + (ARR + 1 - CCR1) / 2
     * This keeps the sample away from the CH1 switching edge while duty changes.
     */
    uint32_t adc_trigger_ccr = ccr + ((BOOST_PWM_ARR + 1U - ccr) / 2U);
    if (adc_trigger_ccr > BOOST_PWM_ARR) {
        adc_trigger_ccr = BOOST_PWM_ARR;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, adc_trigger_ccr);
}

void Boost_Init(void)
{
    PWM_SetDuty(BOOST_DUTY_MIN);

    /* Start ADC1 in interrupt mode. Conversions are triggered by TIM1_CC2. */
    if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /* Start TIM1 CH1 PWM (PE9), and complementary CH1N (PA7). */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    /* Start TIM1 CH2 as the ADC trigger source, no GPIO is configured for CH2. */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
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
    /* 1. ADC sampling - Vout
     * ADC1 raw value is updated by TIM1_CC2-triggered conversion complete IRQ.
     * Control loop only consumes the latest sampled value here.
     */
    boost.v_out_raw = ADC_ToVout(boost.adc_raw);

    /* ADC一阶低通滤波，减少开关纹波干扰 */
    boost.v_out = (1.0f - ADC_FILTER_ALPHA) * boost.v_out
                + ADC_FILTER_ALPHA * boost.v_out_raw;

    /* TODO: 如果硬件支持Vin采样，在此处读取并更新boost.v_in
     * 示例代码（需配置ADC多通道）：
     * HAL_ADC_PollForConversion(&hadc1, 1);
     * boost.adc_vin_raw = HAL_ADC_GetValue(&hadc1);
     * boost.v_in = ADC_ToVin(boost.adc_vin_raw);
     */

    /* 2. Error */
    float error = boost.v_target - boost.v_out;

#if ENABLE_FEEDFORWARD
    /* 3. 前馈 + 增量式PI控制
     *    duty_ff = 1 - Vin/Vout (理想Boost占空比)
     *    du(k) = Kp * [e(k) - e(k-1)] + Ki * e(k)
     *    duty(k) = duty_ff + du(k)
     */
    float duty_ff = Calc_Feedforward(boost.v_in, boost.v_target);
    float delta = boost.pi.kp * (error - boost_pi_error_last)
                + boost.pi.ki * error;
    float new_duty = duty_ff + delta;
#else
    /* 3. 纯增量式PI (无前馈)
     *    du(k) = Kp * [e(k) - e(k-1)] + Ki * e(k)
     *    u(k)  = u(k-1) + du(k)
     */
    float delta = boost.pi.kp * (error - boost_pi_error_last)
                + boost.pi.ki * error;
    float new_duty = boost.duty + delta;
#endif

    /* 4. 抗积分饱和：占空比饱和时，如果误差方向还在让输出继续饱和，
     *    则回退本次的积分增量，避免积分项继续累积 */
    if ((new_duty > BOOST_DUTY_MAX && error > 0.0f) ||
        (new_duty < BOOST_DUTY_MIN && error < 0.0f)) {
        /* 饱和且误差方向不利 → 冻结积分，回退Ki项 */
        new_duty -= boost.pi.ki * error;
    }

    /* 5. Limit output and update PWM */
    PWM_SetDuty(new_duty);

    /* 6. Save state for next incremental PI step */
    boost_pi_error_last = error;
    boost.pi.integral = 0.0f;  /* 增量式PI不使用此字段，保留用于调试 */
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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        boost.adc_raw = HAL_ADC_GetValue(hadc);
    }
}
