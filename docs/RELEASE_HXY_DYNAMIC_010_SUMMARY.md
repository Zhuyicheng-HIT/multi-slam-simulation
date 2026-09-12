# RELEASE-HXY-DYNAMIC-010 Stage Summary

## Baseline

- Branch: `feat/lidar-horizontal-degeneracy-v1`
- Release commit before this summary: `88203bf22e12887e1aacecebe99219d550e89b9e`
- Intended base: current `main` (`c7c1adcd92a7fdd3b5b38aa47e48a10ea3552981`)
- Repository: `Zhuyicheng-HIT/multi-slam-simulation`

## Delivered

- MID360 native IMU is the common IMU source for FAST-LIO and the Ultra-Fusion backend; FCU IMU remains inside ArduPilot.
- HXY horizontal LiDAR degeneracy handling with arbitrary rotated weak-subspace attenuation.
- Prediction recovery and scan-prediction contract fail-closed protection.
- PR15 Dynamic Observer v2 and Clean Scan Gateway migration, preserving the raw cloud path for safety/avoidance.
- Gazebo clock startup/ownership guard and FAST-LIO clean-topic runtime configuration fix.

## Evidence

- Dynamic benchmark (PR15-compatible synthetic benchmark): micro precision `99.8439%`, recall `97.0854%`, F1 `98.4454%`; macro precision `93.5449%`, recall `85.7748%`, F1 `88.8424%`.
- Static-point preservation `99.9859%`; dynamic contamination `1.8083%`; observer latency p50 `7.186 ms`, p95 `9.784 ms`.
- HXY long-tunnel replay reduced approximately `6.5 m` horizontal drift to `0.79 m` on the frozen comparison bag.
- Dynamic-enabled static replay completed 60 s ROS time: maximum 3D deviation `0.028 m` (about `2.8 cm`); 10/30/60 s XYZ displacement `0.019/0.017/0.023 m`; XY displacement `0.019/0.015/0.021 m`.
- Static replay recorded 600 FAST-LIO and 600 truth samples with no invalid timestamps. Native LiDAR factors: `598`; GNSS factors: `41`; IMU factors: `597`; optimization rollbacks: `0`.
- Feature repeatability median: `100%`.

## Known limits and follow-up

- Feature repeatability is currently a median diagnostic only. Formal P5/minimum and fraction-of-frames-below-95% scoring is still required.
- Dynamic benchmark macro recall is below the micro result and needs scene-diverse validation before claiming universal recall.
- Marginal-prior weak-direction attribution and further replay coverage remain follow-up work; no marginalization retuning is included here.
- No merge is requested as part of this release PR.
