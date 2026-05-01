# ADC与PWM同步采样配置指南

## 问题说明
当前代码使用 `HAL_ADC_PollForConversion` 在1ms软件节拍中轮询ADC，采样点与PWM相位是漂移的，会引入随机扰动和开关噪声。

## 解决方案
使用TIM1触发ADC，在每个PWM周期的固定相位采样（建议在开关管关断中点），避开开关噪声。

---

## STM32CubeMX配置步骤

### 1. TIM1配置（已完成）
- 频率: 100kHz
- ARR: 1679
- PWM输出: CH1 (PE9)

### 2. ADC1配置修改

#### 2.1 触发源设置
在STM32CubeMX中：
- **ADC1 → Parameter Settings → External Trigger Conversion Source**
  - 选择: `Timer 1 Trigger Out event (TIM1_TRGO)`
  - 或者: `Timer 1 Capture Compare 1 event (TIM1_CC1)`

- **External Trigger Conversion Edge**
  - 选择: `Trigger detection on the rising edge`

#### 2.2 采样时机选择
**方案A: 使用TIM1_CC1事件（推荐）**
- 在PWM周期的特定相位触发
- 设置TIM1_CCR1为采样点（例如ARR的50%，即关断中点）
- 优点：精确控制采样时刻，避开开关噪声

**方案B: 使用TIM1_TRGO事件**
- 在PWM周期的更新事件触发
- 采样点固定在周期开始
- 配置简单，但可能采到开关瞬间噪声

### 3. DMA配置（可选，提高效率）
- 启用ADC1的DMA
- Mode: Circular
- Data Width: Half Word (16-bit)
- 自动将ADC结果搬运到内存，无需轮询

---

## 代码修改

### 方案1: 使用TIM1_CC1触发 + 中断模式

#### 修改 `adc.c` 初始化
```c
void MX_ADC1_Init(void)
{
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;  // 关闭连续转换
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;  // TIM1_CC1触发
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }
  
  // 配置通道
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_5;  // PA5
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;  // 根据需要调整
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}
```

#### 修改 `tim.c` - 配置TIM1触发ADC的时刻
```c
void MX_TIM1_Init(void)
{
  // ... 现有配置 ...
  
  // 配置TIM1_CCR1为采样触发点（例如：在PWM周期50%处采样）
  // 这样可以在开关管关断中点采样，避开开关噪声
  TIM_OC_InitTypeDef sConfigOC_Trigger = {0};
  sConfigOC_Trigger.OCMode = TIM_OCMODE_PWM1;
  sConfigOC_Trigger.Pulse = (BOOST_PWM_ARR + 1) / 2;  // 50%位置触发ADC
  sConfigOC_Trigger.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC_Trigger.OCFastMode = TIM_OCFAST_DISABLE;
  
  // 注意：如果CH1已用于PWM输出，可以用CH2/CH3/CH4作为触发源
  // 这里假设用CH2作为ADC触发（不输出到引脚）
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC_Trigger, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
}
```

#### 修改 `boost.c` - 使用中断模式
```c
void Boost_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    
    // 启动TIM1_CH2用于触发ADC（不输出到引脚）
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
    
    PWM_SetDuty(BOOST_DUTY_MIN);
    
    // 启动ADC中断模式，等待TIM1触发
    HAL_ADC_Start_IT(&hadc1);
}

// ADC转换完成中断回调
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        boost.adc_raw = HAL_ADC_GetValue(&hadc1);
        // 可以在这里设置标志位，在主循环中处理控制算法
        // 或者直接在中断中执行控制（注意中断执行时间）
    }
}

void Boost_ControlLoop(void)
{
    // ADC采样由TIM1自动触发，无需轮询
    // 直接使用boost.adc_raw（由中断更新）
    
    boost.v_out_raw = ADC_ToVout(boost.adc_raw);
    boost.v_out = (1.0f - ADC_FILTER_ALPHA) * boost.v_out
                + ADC_FILTER_ALPHA * boost.v_out_raw;
    
    // ... 其余控制逻辑不变 ...
}
```

---

### 方案2: 使用DMA模式（最高效）

#### 启用DMA
在CubeMX中：
- ADC1 → DMA Settings → Add
- DMA Request: ADC1
- Mode: Circular
- Data Width: Half Word to Half Word

#### 代码修改
```c
uint16_t adc_dma_buffer[2];  // [0]=Vout, [1]=Vin (如果有多通道)

void Boost_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);  // 触发源
    
    PWM_SetDuty(BOOST_DUTY_MIN);
    
    // 启动ADC DMA模式
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, 1);
}

void Boost_ControlLoop(void)
{
    // 直接读取DMA缓冲区，无需等待
    boost.adc_raw = adc_dma_buffer[0];
    
    boost.v_out_raw = ADC_ToVout(boost.adc_raw);
    boost.v_out = (1.0f - ADC_FILTER_ALPHA) * boost.v_out
                + ADC_FILTER_ALPHA * boost.v_out_raw;
    
    // ... 其余控制逻辑 ...
}
```

---

## 优化效果对比

| 方案 | 采样时刻 | CPU占用 | 噪声抑制 | 实现难度 |
|------|---------|---------|---------|---------|
| 当前轮询 | 随机漂移 | 高 | 差 | 简单 |
| TIM触发+中断 | 固定相位 | 中 | 好 | 中等 |
| TIM触发+DMA | 固定相位 | 低 | 好 | 较高 |

**推荐**: 先实现TIM触发+中断模式，验证效果后再考虑DMA。

---

## 调试建议

1. **验证触发时刻**: 用示波器观察ADC采样时刻与PWM波形的相位关系
2. **调整采样点**: 修改TIM1_CCR2的值，找到噪声最小的采样位置
3. **采样时间**: 根据ADC输入阻抗调整SamplingTime，确保采样准确
4. **多通道采样**: 如需同时采Vin和Vout，配置ADC扫描模式+DMA

---

## 注意事项

- TIM1的CCR2仅用于触发ADC，不需要连接到物理引脚
- 如果使用TIM1_TRGO，需在TIM1配置中设置Trigger Output为Update Event
- 采样频率 = PWM频率 = 100kHz，确保控制环路1ms调用一次时有足够的新数据

