# Sensor payload and Iris roll PID stability report

Date: 2026-08-04

## Scope

This iteration isolates the Gazebo aircraft dynamics from the navigation
backend. Gazebo `SIM2` ground truth is used only to select the airborne interval
and evaluate physical motion; it is never fed to FAST-LIO or the unified
estimator.

The D435i, optical-flow assembly, and MID360 are fixed measurement links. SDF
requires a positive mass and positive-definite inertia for a dynamic link, so
"zero mass" is represented by `1e-6 kg` and diagonal inertia `1e-9 kg m^2` per
link. The Iris base, rotor links, and the upstream `imu_link` remain unchanged.

| Payload link | Previous mass | Final mass | Previous diagonal inertia | Final diagonal inertia |
| --- | ---: | ---: | ---: | ---: |
| `flow_camera_link` | 0.001 kg | 1e-6 kg | 1e-6 kg m^2 | 1e-9 kg m^2 |
| `front_d435i_link` | 0.001 kg | 1e-6 kg | 1e-6 kg m^2 | 1e-9 kg m^2 |
| `mid360_link` | 0.001 kg | 1e-6 kg | 1e-6 kg m^2 | 1e-9 kg m^2 |

## Controlled flight results

Each retained trial wipes SITL EEPROM, keeps roll P/I, pitch PID, filters,
motor model, route, and sensor configuration unchanged, and flies one complete
22.77 m S pass followed by return and automatic landing. DataFlash is analyzed
with `tools/analyze_apm_attitude_jitter.py`.

| Trial | DataFlash | Airborne | Roll D | Roll RMS | Roll >3 Hz RMS | Dominant peak | Rate error RMS | Correlation | Clips |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Original 1 g payload | `00000309.BIN` | 91.56 s | 0.0036 | 22.68 deg/s | 22.41 deg/s | 7.54 Hz | 23.84 deg/s | -0.553 | 0 |
| 1 mg payload only | `00000310.BIN` | 123.64 s | 0.0036 | 23.18 deg/s | 22.91 deg/s | 7.55 Hz | 24.34 deg/s | -0.572 | 0 |
| First PID trial | `00000311.BIN` | 123.63 s | 0.0027 | 18.07 deg/s | 17.89 deg/s | 6.73 Hz | 18.95 deg/s | -0.466 | 0 |
| Retained PID | `00000312.BIN` | 123.62 s | 0.0018 | 1.38 deg/s | 0.35 deg/s | 0.52 Hz | 0.33 deg/s | 0.975 | 0 |

Reducing payload mass alone changes roll RMS by +2.2 percent and does not
remove the limit cycle. The retained D gain removes the 7.5 Hz peak and reduces
roll RMS by 94.1 percent relative to the equal-mass native-PID run.

The final roll actual/desired standard-deviation ratio is 0.943, so the result
is not obtained by making the axis unresponsive. Four motor outputs remain
between 1501 and 1597 us with zero samples near the 1050/1950 us saturation
bounds. Roll PID contribution RMS values are P=0.000509, I=0.000124, and
D=0.000457 in normalized output units.

## Optical-flow effect

The flight-interval median gyro-equivalent image velocity falls from 0.271 m/s
at D=0.0036 to 0.037 m/s at D=0.0018, an 86.3 percent reduction. Median optical
flow quality remains about 200/255, showing that the quality score did not
detect the original rotation-driven error.

The image/gyro integration period is still irregular (about 0.132 s median and
0.264 s p95 in the final run). PID tuning removes the aircraft limit cycle but
does not solve this separate optical-flow scheduling/timestamp issue.

## Retained configuration

`params/iris_roll_stability.parm` sets only:

```text
ATC_RAT_RLL_D 0.0018
```

The profile is enabled by default only in the SITL launcher. Set
`WIPE_EEPROM=1 ENABLE_IRIS_ROLL_STABILITY_PROFILE=0` to recover native
ArduPilot defaults for controlled A/B tests. An existing workspace that has
stored older PID values must also use `WIPE_EEPROM=1` once when enabling the
profile, because stored SITL parameters override defaults files. Real-hardware
PID values must be tuned on the real airframe and are not inherited from this
profile.

ArduPilot's official guidance treats rapid oscillation as an excessive-gain
boundary and recommends reducing the D value after it is observed. SITL default
overrides should be loaded from a parameter file with EEPROM wiped for a clean
comparison:

- https://ardupilot.org/copter/docs/ac_rollpitchtuning.html
- https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html

## Evidence

JSON reports are under `logs/sensor_mass_pid_20260804/` in the active workspace.
This iteration validates the simulated flight plant and its optical-flow input;
it is not a new unified-SLAM accuracy or ExternalNav closed-loop acceptance.
