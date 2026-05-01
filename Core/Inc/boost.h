#ifndef __BOOST_H
#define __BOOST_H

#include "stm32f4xx_hal.h"

/* ============================================================
 *  Boost Closed-Loop Voltage Control
 *  Board: STM32F407VGT6 V2.0
 *
 *  PWM  : TIM1_CH1  → PE9  (主通道)
 *         TIM1_CH1N → PE8  (互补通道，可选死区)
 *  ADC  : ADC1_CH5  → PA5  (Vout采样，比例1/20，偏置1.65V)
 *  KEYS : KEY_UP    → PB10 (Binary_Switch1, 目标电压+)
 *         KEY_DOWN  → PB11 (Binary_Switch2, 目标电压-)
 * ============================================================ */

/* ---------- 硬件参数 ---------- */
#define BOOST_TIM_FREQ_HZ       100000U     /* PWM频率 100kHz           */
#define BOOST_PWM_PERIOD        (168U)      /* ARR = 168-1, 168MHz/168=1MHz? 
                                               实际: APB2=168MHz, TIM1预分频=0
                                               ARR=1679 → 100kHz          */
#define BOOST_PWM_ARR           (1679U)     /* 168MHz / (0+1) / (1679+1) = 100kHz */
#define BOOST_DUTY_MAX          (0.90f)     /* 最大占空比 90%             */
#define BOOST_DUTY_MIN          (0.05f)     /* 最小占空比 5%              */

/* ---------- 采样电路参数 ---------- */
/* Vadc = Vout/20 + 1.65
 * => Vout = (Vadc - 1.65) * 20
 * ADC 12bit, Vref=3.3V => Vadc = adc_raw * 3.3 / 4095
 */
#define ADC_VREF                (3.3f)
#define ADC_RESOLUTION          (4095.0f)
#define SAMPLE_RATIO            (20.0f)     /* 分压比: Vout:Vadc = 20:1  */
#define SAMPLE_BIAS             (1.65f)     /* 偏置电压 1.65V            */

/* Vin采样参数（如果硬件支持，需配置ADC第二通道） */
#define VIN_SAMPLE_RATIO        (10.0f)     /* Vin分压比，根据实际电路调整 */
#define VIN_SAMPLE_BIAS         (0.0f)      /* Vin偏置电压               */
#define VIN_DEFAULT             (21.0f)      /* 默认输入电压（无采样时）   */

/* ADC滤波系数 */
#define ADC_FILTER_ALPHA        (0.2f)      /* 一阶低通滤波系数 0.2      */

/* ---------- PI 控制器参数 ---------- */
#define PI_KP                   (0.005f)
#define PI_KI                   (0.0002f)
#define PI_INTEGRAL_MAX         (0.5f)
#define PI_INTEGRAL_MIN        (-0.5f)

/* 前馈控制开关 */
#define ENABLE_FEEDFORWARD      (1)         /* 1=启用前馈, 0=禁用        */

/* ---------- 目标电压范围 ---------- */
#define VOUT_TARGET_DEFAULT     (12.0f)     /* 默认目标 12V              */
#define VOUT_TARGET_MAX         (30.0f)     /* 最大目标 30V              */
#define VOUT_TARGET_MIN         (5.0f)      /* 最小目标 5V               */
#define VOUT_TARGET_STEP        (0.5f)      /* 每次按键调节 0.5V         */

/* ---------- PI 控制器结构体 ---------- */
typedef struct {
    float kp;
    float ki;
    float integral;
    float integral_max;
    float integral_min;
    float output;
} PI_Controller_t;

/* ---------- Boost 状态结构体 ---------- */
typedef struct {
    float v_target;         /* 目标输出电压 (V)   */
    float v_out;            /* 实测输出电压 (V)   */
    float v_out_raw;        /* 未滤波的输出电压   */
    float v_in;             /* 输入电压 (V)       */
    float duty;             /* 当前占空比 0.0~1.0 */
    uint32_t adc_raw;       /* ADC原始值           */
    uint32_t adc_vin_raw;   /* Vin ADC原始值      */
    PI_Controller_t pi;
} Boost_t;

/* ---------- 全局实例 ---------- */
extern Boost_t boost;
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;

/* ---------- 函数声明 ---------- */
void Boost_Init(void);
void Boost_SetTarget(float v_target);
void Boost_ControlLoop(void);   /* 在定时器/主循环中周期调用 */
float Boost_GetVout(void);
float Boost_GetDuty(void);
float Boost_GetTarget(void);

/* 按键处理（放在主循环扫描） */
void Boost_KeyScan(void);

#endif /* __BOOST_H */
