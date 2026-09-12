# 传感器载荷与 Iris roll PID 稳定性报告

日期：2026-08-04

## 范围

本轮将 Gazebo 飞行器 dynamics 与 navigation backend 隔离。Gazebo `SIM2` truth 仅用于选择 airborne interval 和评估物理运动，绝不输入 FAST-LIO 或 unified estimator。

D435i、optical-flow assembly 与 MID360 是固定 measurement link。SDF 要求 dynamic link 具有正质量和 positive-definite inertia，因此“zero mass”用每个 link `1e-6 kg`、diagonal inertia `1e-9 kg m^2` 表示；Iris base、rotor link 和上游 `imu_link` 不变。

| Payload link | 旧 mass | 最终 mass | 旧 diagonal inertia | 最终 diagonal inertia |
|---|---:|---:|---:|---:|
| `flow_camera_link` | 0.001 kg | 1e-6 kg | 1e-6 kg m^2 | 1e-9 kg m^2 |
| `front_d435i_link` | 0.001 kg | 1e-6 kg | 1e-6 kg m^2 | 1e-9 kg m^2 |
| `mid360_link` | 0.001 kg | 1e-6 kg | 1e-6 kg m^2 | 1e-9 kg m^2 |

## 受控飞行结果

每次保留的 trial 都清除 SITL EEPROM，保持 roll P/I、pitch PID、filter、motor model、route 和 sensor 配置不变，完成一条 22.77 m S 航线并返航自动降落。DataFlash 使用 `tools/analyze_apm_attitude_jitter.py` 分析。

| Trial | DataFlash | Airborne | Roll D | Roll RMS | Roll >3 Hz RMS | Dominant peak | Rate error RMS | Correlation | Clips |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Original 1 g payload | `00000309.BIN` | 91.56 s | 0.0036 | 22.68 deg/s | 22.41 deg/s | 7.54 Hz | 23.84 deg/s | -0.553 | 0 |
| 1 mg payload only | `00000310.BIN` | 123.64 s | 0.0036 | 23.18 deg/s | 22.91 deg/s | 7.55 Hz | 24.34 deg/s | -0.572 | 0 |
| First PID trial | `00000311.BIN` | 123.63 s | 0.0027 | 18.07 deg/s | 17.89 deg/s | 6.73 Hz | 18.95 deg/s | -0.466 | 0 |
| Retained PID | `00000312.BIN` | 123.62 s | 0.0018 | 1.38 deg/s | 0.35 deg/s | 0.52 Hz | 0.33 deg/s | 0.975 | 0 |

仅降低 payload mass 使 roll RMS 增加 2.2%，没有消除 limit cycle。保留的 D gain 消除 7.5 Hz peak，相比 equal-mass native-PID run 将 roll RMS 降低 94.1%。最终 roll actual/desired 标准差比为 0.943；四个 motor output 在 1501–1597 us，未接近 1050/1950 us saturation。Roll PID contribution RMS：P=0.000509、I=0.000124、D=0.000457（normalized output units）。

## Optical-flow 影响

Flight interval 的 median gyro-equivalent image velocity 从 D=0.0036 时的 0.271 m/s 降到 D=0.0018 时的 0.037 m/s，下降 86.3%。Median optical-flow quality 仍约 200/255，说明 quality score 没有发现原 rotation-driven error。

最终运行的 image/gyro integration period 仍不规则（median 约 0.132 s，p95 0.264 s）。PID tuning 消除了 aircraft limit cycle，但未解决独立的 optical-flow scheduling/timestamp 问题。

## 保留配置

`params/iris_roll_stability.parm` 仅设置：

```text
ATC_RAT_RLL_D 0.0018
```

Profile 只在 SITL launcher 中默认启用。设置 `WIPE_EEPROM=1 ENABLE_IRIS_ROLL_STABILITY_PROFILE=0` 可恢复 ArduPilot native default 做 A/B test。已有 workspace 若存储旧 PID 值，启用 profile 时也需先 `WIPE_EEPROM=1`；real-hardware PID 必须在真实机体上调参。

ArduPilot 官方建议将 rapid oscillation 视为过高 gain 边界，观察到后降低 D value。参考：

- https://ardupilot.org/copter/docs/ac_rollpitchtuning.html
- https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html

## 证据

JSON report 位于 `logs/sensor_mass_pid_20260804/`。本轮验证 simulated flight plant 与 optical-flow input，不是新的 unified-SLAM accuracy 或 ExternalNav closed-loop 验收。
