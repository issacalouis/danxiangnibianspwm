# Repository Guidelines

## Project Structure & Module Organization

This is an STM32CubeIDE project for an STM32F407 target. Application source lives in `Core/Src`, with public headers in `Core/Inc`. Board startup code is in `Core/Startup`. STM32 HAL and CMSIS vendor code are under `Drivers/`; treat these as generated/vendor files unless a hardware package update requires changes. IMU-related code is split between `Core/IMU` and `Core/ICM45686`. `FULL.ioc` is the CubeMX peripheral configuration source of truth. `Debug/` contains generated build products and makefiles.

## Build, Test, and Development Commands

Use STM32CubeIDE for normal development and code generation from `FULL.ioc`.

```sh
cd Debug
make -j16 all
make -j16 clean
```

`make -j16 all` builds `FULL.elf`, `FULL.map`, and `FULL.list`. `make -j16 clean` removes generated objects and build outputs. If CubeMX regenerates files, review changes inside `USER CODE BEGIN/END` blocks carefully.

## Coding Style & Naming Conventions

Use C11-compatible C and the STM32 HAL style already present in the project. Keep four-space indentation for new application logic, avoid tabs in new code, and keep comments short and technical. Use `MX_*` names for Cube-generated initialization, `HAL_*` callbacks for HAL hooks, and module-prefixed names such as `Boost_*`, `OLED_*`, or `IMU_*` for application APIs. Keep hardware constants in headers such as `Core/Inc/boost.h`.

## Testing Guidelines

There is no host-side unit test framework. Validation is firmware-build plus hardware test. At minimum, build successfully with `make -j16 all`, then verify PWM pins, ADC scaling, protection thresholds, OLED display, and keyboard controls on current-limited hardware. For power-stage changes, test first with low input voltage and oscilloscope checks before connecting the full load.

## Commit & Pull Request Guidelines

No consistent commit convention is visible in this workspace, so use concise imperative messages, for example `Tune inverter PI defaults` or `Add TIM8 complementary PWM setup`. PRs should describe the hardware behavior changed, list CubeMX regeneration if any, include build results, and note bench-test conditions such as input voltage, load, current limit, and observed waveforms.

## Agent-Specific Instructions

Do not overwrite user changes or regenerate CubeMX files unless explicitly requested. Prefer edits inside application modules and `USER CODE` regions. Before changing control-loop constants, state the target hardware assumptions and the exact files/defines being changed.
