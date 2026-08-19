# Frozen Low-Altitude Figure-Eight Baseline

This is the stable demonstration and regression entry point after the
2026-08-19 low-altitude figure-eight run. Use:

```bash
cd /home/zyc/multi-slam-pr12-audit
source /opt/ros/humble/setup.bash
source /home/zyc/multi-slam-deps/mid360_ws/install/setup.bash
source install/setup.bash
bash tools/run_frozen_low_figure8_validation.sh
```

The frozen profile uses the low indoor world, a 2.2 m nominal takeoff, RGB-D
direct factors, a 10 m simulation depth limit, and directional LiDAR axis
handoff. It keeps online time calibration in shadow/diagnostic mode.

The following experimental paths are retained but not invoked by this entry
point: Range-Facet, barometer fallback, and active relocalization triggers.
EKF3 ExternalNav control is enabled by this entry point. One-variable experiments must use a new `LOG_DIR`
and must not overwrite the frozen run.

Reference run: `logs/low_indoor_figure8_rangefacet_20260819`.

| Metric | Reference |
| --- | ---: |
| Simulation duration | 279.807 s |
| Matched samples | 2617 |
| Causal 3D RMSE | 3.232 cm |
| Causal 3D P95 | 4.973 cm |
| Maximum 3D error | 11.460 cm |
| Vertical RMSE | 2.685 cm |
| Solver P95 | about 33 ms |
| Backend callback P95 | about 81 ms |

The recorded run had the Range-Facet switch present but no accepted
Range-Facet factor. The frozen profile disables it explicitly to prevent an
experimental path from consuming runtime budget. The reference metrics are
therefore an archived comparison point, not a claim that every future run is
identical.
