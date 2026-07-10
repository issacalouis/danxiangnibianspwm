# FULL 单相逆变控制固件用户手册

## 1. 适用范围

本手册适用于当前 `FULL` 工程，目标芯片为 STM32F407。固件实现单相全桥 SPWM 输出、QPR 电压环控制、OLED 状态显示、矩阵键盘参数设置和三路 ADC 采样显示。

当前控制假设：直流输入为固定 `36V`，输出频率默认 `50Hz`，输出 LC 为 `1mH + 10uF`，控制周期为 `20kHz / 50us`。模型归一化母线参数使用 `XTQ3_VBUS_NORM = 36.0`。调试时必须从低压、限流开始。

## 2. 安全注意事项

1. 首次运行必须使用限流电源，并先断开重载。
2. 接功率级前，先用示波器确认 PE9/PA7、PC7/PB0 互补 PWM、死区和占空比变化正常。
3. 修改控制参数、采样比例、采样偏置或目标电压后，都要从低目标电压重新验证。
4. OLED 显示来自软件换算，不能替代万用表、示波器或电流探头。
5. 功率级仍需要外部限流或硬件保护。

## 3. 硬件接口

### PWM 输出

| 功能 | 定时器通道 | 引脚 |
| --- | --- | --- |
| 桥臂 A 主 PWM | TIM1_CH1 | PE9 |
| 桥臂 A 互补 PWM | TIM1_CH1N | PA7 |
| 桥臂 B 主 PWM | TIM8_CH2 | PC7 |
| 桥臂 B 互补 PWM | TIM8_CH2N | PB0 |

### ADC 采样

| 测量量 | STM32 通道 | 引脚 | C2000/Simulink 信号 | 换算 |
| --- | --- | --- | --- | --- |
| 输出电压 `Vout` | ADC1_CH5 | PA5 | ADC7 | `(Vadc - v_bias) * v_gain` |
| 输出电流 `Iout` | ADC2_CH14 | PC4 | ADC2 | `(Vadc - iout_bias) * iout_gain` |
| 输入电流 `Iin` | ADC3_CH10 | PC0 | ADC6 | `(Vadc - iin_bias) * iin_gain` |

默认偏置为 `1.6900V`，默认比例为 `v_gain=55.8`、`iout_gain=2.828`、`iin_gain=2.828`。在 `STOP` 状态下可用键盘按 `0.0001V` 步进微调三路偏置，或按 `0.1` 步进微调三路比例。

## 4. 控制算法

RUN 状态下执行 QPR 电压环 + 输出电流阻尼 + SPWM：

```c
v_ref = SV_active * sqrt(2) * sin(phase);
v_err = v_ref - v_out_raw;  // QPR waveform loop uses unfiltered instantaneous sample
control_v_err = (abs(v_err) < 0.05V) ? 0 : v_err;
resonant = deadbanded ? resonant_last : QPR(control_v_err);
rms_trim = PI(SV_active - Vrms);  // RMS loop uses filtered RMS
feedforward = (SV_active + rms_trim) * sqrt(2) * sin(phase);
inverter_voltage_cmd = feedforward + KP * control_v_err + resonant - iout_damping * i_out_raw;
M = clamp(inverter_voltage_cmd / 36.0, -modulation_limit, modulation_limit);
M_comp = clamp(M + deadtime_compensation(Io), -modulation_limit, modulation_limit);
duty_a = 0.5 + 0.5 * M_comp;
duty_b = 0.5 - 0.5 * M_comp;
```

说明：

- 输出频率可在 `20Hz ~ 100Hz` 范围内步进调整，默认 `50Hz`。
- QPR 谐振系数会跟随输出频率重算；示波器若看到的是开关沿附近尖峰/振铃，优先按开关瞬态和周期级抖动处理。当前 `BOOST_QPR_KP = 0.012`，`BOOST_QPR_KR = 0.80`，误差死区 `BOOST_QPR_ERROR_DEADBAND_V = 0.05V`。
- PWM 输出使用 TIM1/TIM8 硬件互补 PWM，载波保持 `20kHz`。ADC 仍由 `TIM1_CC2` 触发，采样点设在约 `85%` PWM 周期位置，用于避开疑似开关沿噪声。
- ADC_FILTER_ALPHA=0.25，电压/电流瞬时一阶滤波启用，用于 RMS 显示、Irms 过流和 RMS 幅值外环；QPR/比例波形环使用未滤波的 v_out_raw，输出电流阻尼使用未滤波的 i_out_raw。OLED 主界面显示的 Vr/Ir 为 RMS 值，使用滤波后的瞬时值平方后再由 RMS_FILTER_ALPHA=0.001 做均值低通。慢速 RMS 幅值外环使用 SV_active - Vrms 修正前馈幅值：KP=0.05、KI=1.0、最大修正 2Vrms。死区误差补偿代码已保留，但默认补偿量为 0.0，等效关闭。
- OLED 的 `M` 显示为最近一个输出周期内的补偿后调制峰值，避免瞬时过零时长期显示 `0.00`。

## 5. 上电与运行流程

1. 上电后 OLED 显示 `INITIALIZING`。
2. 初始化完成后默认进入 `STOP`，PWM 保持中心占空比，`M=0.00`。
3. 默认目标电压 `SV=5.0Vrms`。
4. 按 `D` 进入 `ARM`，采样稳定后自动进入 `RUN`。
5. `RUN` 后目标电压从 0 按软启动斜率爬升到 `SV`。
6. 运行中再按 `D` 立即回到 `STOP`，PWM 回中心占空比，控制状态清零。

## 6. OLED 字段

| 字段 | 含义 |
| --- | --- |
| `STOP` | 停止状态，PWM 中心占空比 |
| `ARM` | 启动自检状态，PWM 仍为中心占空比 |
| `RUN` | 闭环运行状态，QPR + SPWM 生效 |
| `OC` | 输出过流保护状态 |
| `SV` | 输出电压目标，单位按 RMS 理解 |
| `F` | 输出频率 |
| `LI` | 输出过流保护阈值 |
| Vr | 输出电压 RMS，来自输出电压瞬时值的平方均值低通 |
| Ir | 输出电流 RMS，来自输出电流瞬时值的平方均值低通 |
| `Ii` | 输入电流采样换算值，来自 PC0 |
| M | 最近一个输出周期内的调制峰值，用于观察是否接近饱和 |

校准界面：`CAL V`、`CAL IO`、`CAL II` 分别对应电压、输出电流、输入电流采样。每个校准页同时显示 `BIAS`、`GAIN` 和 `VAL`；`1/2` 调偏置，`A/B` 调比例。`BIAS` 为当前偏置电压，`GAIN` 为当前换算比例，`VAL` 为当前换算值。

## 7. 键盘操作

| 按键 | 功能 |
| --- | --- |
| `A` | 普通界面切换到电压目标编辑；校准界面比例 `+0.1` |
| `B` | 普通界面切换到电流限制编辑；校准界面比例 `-0.1` |
| `C` | 在 `STOP` 下循环 `CAL V / CAL IO / CAL II / NORMAL` |
| `D` | `STOP -> ARM -> RUN`，运行中或故障中再按回到 `STOP` |
| `1` | 普通界面增加当前编辑项；校准界面偏置 `+0.0001V` |
| `2` | 普通界面减少当前编辑项；校准界面偏置 `-0.0001V` |
| `#` | 切换到频率编辑，OLED 显示 `SET F` |

参数范围：

| 参数 | 默认 | 最小 | 最大 | 步进 |
| --- | ---: | ---: | ---: | ---: |
| `SV` | 5.0 | 0.0 | 26.0 | 0.1 |
| `F` | 50 | 20 | 100 | 5 |
| `LI` | 2.5 | 0.2 | 2.5 | 0.1 |
| `BIAS` | 1.6900 | 0.0 | 3.3 | 0.0001 |
| `GAIN` | 55.8 / 2.828 | 0.0 | 200.0 | 0.1 |

当前参数不会掉电保存，重新上电后恢复源码默认值。

## 8. 推荐调试流程

1. 保持 `STOP`，确认 `Vr/Ir/Ii` 零输入接近 0。
2. 若零点不准，按 `C` 进入对应校准界面，用 `1/2` 调整偏置；若比例不准，在同一校准界面用 `A/B` 按 `0.1` 步进调整比例。
3. 示波器测 PE9/PA7、PC7/PB0，确认 `STOP` 下中心占空比和互补输出正常。
4. 确认 TIM1/TIM8 仍为 `20kHz`，死区约 `1us`，ADC 触发点在约 `85%` PWM 周期位置，并相对主要开关沿移开。
5. 确认死区补偿前后过零附近输出电压畸变、电流尖峰和器件温升没有变差。
6. 将 `SV` 降到较低值，按 `D` 进入 `ARM/RUN`。
7. 观察 `M` 是否从小值随软启动上升，不应直接长时间顶到上限。
8. 输出稳定后再逐步升高 `SV`。

## 9. 常见现象

| 现象 | 可能原因 | 处理建议 |
| --- | --- | --- |
| 零输入时 `Vr/Ir/Ii` 不接近 0 | 采样偏置不准 | 在 `STOP` 下用校准界面调整偏置 |
| 实测电压/电流与显示成比例偏差 | 采样比例不准 | 在 `STOP` 下进入对应 `CAL` 页面，用 `A/B` 调整 `GAIN` |
| 按 `D` 后停在 `ARM` | 启动自检未通过 | 检查零点、采样线和噪声 |
| RUN 后 M 长期接近上限 | 目标过高、母线不足、负载过重或采样极性错误 | 降低 SV，限流低压重新测 |
| 输出端几乎无电压 | PWM 未到驱动、驱动未使能或功率路径断开 | 先用示波器逐点测 PE9/PA7/PC7/PB0 和驱动输出 |
| 输出波形异常 | 采样极性、桥臂接线、LC 接线或驱动死区问题 | 断开负载，逐项确认硬件路径 |
| 相邻 PWM 周期占空比轻微跳动 | ADC 采到开关尖峰、QPR 对噪声过度反应或误差极限环 | 先确认 TIM1_CC2 触发点，再小幅调整 ADC_FILTER_ALPHA、BOOST_QPR_KP 和误差死区；不要把 ADC_FILTER_ALPHA 调得过小以免基波相位滞后 |

## 10. 构建

推荐使用 STM32CubeIDE 构建，也可在有 `make` 的环境下执行：

```sh
cd Debug
make -j16 all
```

关键文件：

- `Core/Inc/boost.h`：控制参数、ADC 换算、目标范围。
- `Core/Src/boost.c`：QPR 控制、状态机、PWM 写入、ADC 回调。
- `Core/Src/main.c`：初始化、按键扫描、OLED 刷新。
- `Core/Src/adc.c` / `Core/Src/tim.c`：CubeMX 生成的 ADC/PWM/定时器配置。

## 11. 当前限制

1. 没有母线电压 ADC，调制仍按模型归一化 `36V` 处理；`SV` 设到 `26Vrms` 时会接近或进入满调制饱和，不能保证无削顶。
2. 参数不会掉电保存。
3. 不重新生成 CubeMX；若后续用 CubeMX 生成代码，需要重点检查 TIM1/TIM8/TIM6 和 ADC1/2/3 配置是否被覆盖。

## 本次新增功能说明

### 当前控制逻辑

- RUN 状态下实际执行 QPR 电压环 + 输出电流阻尼 + RMS 幅值外环 + SPWM。QPR 波形环用未滤波瞬时采样，RMS 幅值外环用滤波 RMS。
- QPR 谐振系数会跟随输出频率重算，调制量按固定归一化母线 `XTQ3_VBUS_NORM = 36V` 限幅。
- STOP、ARM 和 OC 故障态保持 50%/50% 中点 PWM，等效关闭有效调制，但不是关断栅极驱动。
- TIM1_CH1/CH1N 与 TIM8_CH2/CH2N 使用高级定时器硬件互补输出，BDTR 死区为 84 tick，按当前 168MHz 定时器且 CKD=DIV2 约 1us；上功率前需用示波器确认栅极波形。
- 载波频率保持 `20kHz` 不变；固件保留基于 `Io` 极性的死区误差前馈补偿入口，但当前默认关闭，避免补偿方向错误导致波形异常。
- 若补偿后电压尖峰、电流毛刺、噪声或温升变差，应先把 `BOOST_DEADTIME_COMP_MODULATION` 调为 `0.0f` 回退补偿，再继续检查硬件波形。

### 输出频率可步进调整

- 输出频率由固定 `50Hz` 改为运行参数 `output_freq_hz`。
- 默认频率：`50Hz`。
- 可调范围：`20Hz ~ 100Hz`。
- 步进值：`5Hz`，满足“步进值不大于 5Hz”的要求。
- 按键操作：按 `#` 进入 `SET F` 频率编辑模式，按 `1` 增加 5Hz，按 `2` 减少 5Hz。
- 频率改变时，程序会同步重算 QPR 谐振控制器系数，使谐振中心跟随目标输出频率，而不是继续固定在 50Hz。

### Vrms / Irms 计算方式

采样中断里先把 ADC 原始值换算成瞬时电压/电流，再做一阶瞬时滤波，再用滤波后的瞬时值计算 RMS：

```c
v_inst = (Vadc - v_bias) * v_gain;
i_inst = (Iadc - iout_bias) * iout_gain;

v_f += ADC_FILTER_ALPHA * (v_inst - v_f);
i_f += ADC_FILTER_ALPHA * (i_inst - i_f);

v_rms_sq += RMS_FILTER_ALPHA * (v_f * v_f - v_rms_sq);
i_rms_sq += RMS_FILTER_ALPHA * (i_f * i_f - i_rms_sq);

Vrms = sqrt(v_rms_sq);
Irms = sqrt(i_rms_sq);
```

其中 v_f/i_f 是滤波后的瞬时量，v_rms_sq/i_rms_sq 是平方值的低通均值。这个算法是指数滑动 RMS，不是严格整周期窗口 RMS，显示会比瞬时值更稳。RMS 幅值外环使用这个 Vrms 来缓慢修正前馈幅值，使输出 RMS 接近 SV。

### 采样偏置与比例可调

- 三路采样使用运行时参数换算：`V = (Vadc - v_bias) * v_gain`，`Io = (Vadc - iout_bias) * iout_gain`，`Ii = (Vadc - iin_bias) * iin_gain`。
- 在 `STOP` 状态下按 `C` 循环 `CAL V/CAL IO/CAL II`；每个页面同时显示并调整偏置和比例，`1/2` 调 `BIAS`，`A/B` 调 `GAIN`。
- 偏置步进为 `0.0001V`，比例步进为 `0.1`，比例限制在 `0.0 ~ 200.0`。
- 当前校准值不会掉电保存，重新上电后恢复源码默认值。

### 输出过流保护与自动恢复

- 输出过流保护量：Ir = boost.i_rms，由 ADC2_CH14 / PC4 的滤波后输出电流计算得到。
- 保护判断使用低通后的输出电流 RMS：Ir。
- 默认过流动作阈值：`2.5A`。
- `LI` 作为输出过流保护阈值显示和设置项，允许调低做安全测试，但最大值被限制为 `2.5A`。
- 当 `Ir >= LI` 时，控制进入 `BOOST_STATE_FAULT`，OLED 显示 `OC`，PWM 输出回到中点并关闭有效调制。
- 故障恢复采用滞回：当 `Ir <= LI - 0.2A` 且保持 `1000` 个控制周期后，系统自动从 `OC` 进入 `ARM`，再经过原有上电确认流程恢复 `RUN`。
- 20kHz 控制周期下，`1000` 个周期约为 `50ms`。
- 运行中按 `D` 可从 `OC` / `ARM` / `RUN` 手动回到 `STOP`。

### OLED 显示变化

- 第一行状态新增 `OC`，表示输出过流保护状态。
- 编辑模式新增 `SET F`，表示正在调整输出频率。
- 主界面第二行显示 `SV` 和 `F`；第四行显示 `LI` 和 `M`。
