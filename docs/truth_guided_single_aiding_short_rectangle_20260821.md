# Truth-guided IMU + single-aiding short-rectangle validation (2026-08-21)

## Scope and isolation

Four estimator profiles were launched with fixed reliability weights after the
stationary IMU initialization (`observability_bootstrap_mode=S`):

| Profile | Active estimator modalities | Fixed weights (LiDAR/IMU/GNSS/flow/vision) |
| --- | --- | --- |
| LiDAR | IMU + LiDAR | 1/1/0/0/0 |
| GNSS | IMU + GNSS | 0/1/1/0/0 |
| Optical flow | IMU + MTF-01P-like optical flow | 0/1/0/1/0 |
| Vision | IMU + D435i RGB-D direct | 0/1/0/0/1 |

All runs disabled EKF3 ExternalNav feedback. Gazebo world truth was used only
by the rectangle route controller and the accuracy observer. The unified
estimator remained observer-only. The route log records
`DIAGNOSTIC CONTROL ISOLATION ENABLED`, and every completed accuracy report
records `truth_used_by_estimator=false`.

The route was one 2.0 m x 1.2 m rectangle at 2.2 m altitude. Collectors were
configured to stop after landing, with no rosbag, map builder, SLAM-drift
recorder, or reliability timeline. FAST-LIO remained alive as the native
frontend/trigger, but non-target estimator factor weights were fixed to zero
and the profile checker required non-target accepted counts to remain zero.

## Results

| Profile | Mission result | Accepted target factors | Causal RMSE / P95 / max | Endpoint | Output rate / max gap / age P95 | Result |
| --- | --- | ---: | --- | ---: | --- | --- |
| IMU + LiDAR | takeoff, 4 waypoints, land, disarm | 864 | 3D 0.0288 / 0.0409 / 0.0755 m | 0.0149 m | 9.99 Hz / 0.240 s / 0.080 s | Precision pass; strict run fails only 6 native queue discards |
| IMU + GNSS | takeoff, 4 waypoints, land, disarm | 464 | 3D 0.0722 / 0.1535 / 0.3514 m | 0.0409 m | 5.06 Hz / 0.250000001 s / 0.080 s | Mean/P95 useful; strict fail from Z spike, one gap, and 57 native triggers without commit |
| IMU + optical flow | takeoff, 4 waypoints, land, disarm | 37 | **XY** 48.617 / 56.598 / 67.924 m | **XY** 56.598 m | 0.277 Hz / 47.02 s / 1.680 s | Failed: severe horizontal divergence and 33 optimization rollbacks |
| IMU + RGB-D direct | takeoff only; stuck in post-takeoff hold; manually stopped | 0 | unavailable | unavailable | no observed unified odometry | Failed before accuracy evaluation: 798/798 visual candidates dropped pre-bootstrap |

Optical-flow Z was diagnostic only (Z RMSE 0.449 m) and was not used by its
acceptance gates. Its physical range path recovered after takeoff
(`published_range=771`), so the divergence is not a no-range placeholder
result. The backend reported 97 valid LOS diagnostic samples, a 3.106 rad/s
LOS-residual P95, 37 accepted flow factors, and 33 excessive-translation
rollback events.

The GNSS maximum is dominated by Z: horizontal RMSE was 0.0313 m while Z RMSE
was 0.0651 m and the 3D maximum reached 0.3514 m. The endpoint remained 0.0409
m. This profile is not a strict pass because the absolute-position tail and
state-clock gap are real.

The RGB-D run did not produce a defensible accuracy number. Gazebo reached
about 204 s of simulation while the route node remained in its nominal 3 s
post-takeoff hold. The final backend summary reported 798 visual candidates,
798 `visual_prebootstrap_dropped`, zero visual factors, 982 IMU
`interval_not_covered` failures, and a 4.94 s maximum prepare time. This is the
visual cross-topic/state-window bootstrap deadlock, not a completed localization
trial.

Representative backend timing (median/P95/max) was 10.13/14.50/14.96 ms
solver and 36.03/41.32/46.67 ms callback for LiDAR; 4.94/17.36/40.06 ms
solver and 17.79/25.69/26.94 ms callback for GNSS; and 1.91/51.19/51.19 ms
solver but 3013/9259/11283 ms callback for optical flow. The interrupted RGB-D
run already reached 1648/2733/2733 ms callback timing before any visual factor
was accepted.

## Runtime evidence

| Profile | Wall/sim ratio | Host CPU P50/P95 | Validation CPU P50/P95 | Validation RSS P50/P95 | Backend CPU P50/P95 |
| --- | ---: | --- | --- | --- | --- |
| LiDAR | 1.03 | 17.5/19.4% | 704.8/762.7% | 2543/2555 MiB | 60.2/63.3% |
| GNSS | 1.03 | 13.1/15.3% | 685.8/703.8% | 2542/2552 MiB | 70.7/80.4% |
| Optical flow | 2.69 | 13.8/15.2% | 536.0/557.9% | 3095/3118 MiB | 109.4/111.4% |
| RGB-D (failed run) | 24.55 observed | 16.4/18.7% | 691.5/799.1% | 3835/4729 MiB | 102.5/121.8% |

The GPU utilization P50/P95 was 15.5/97.3% (LiDAR), 4.0/31.0% (GNSS),
13.0/97.4% (flow), and 19.0/99.0% (RGB-D). GPU memory is host-global NVML
usage and is not attributable to this validation alone.

## Evidence paths

- LiDAR: `/home/ld666/multi-slam-simulation/logs/truth_nav_single_lidar_20260821_125138`
- GNSS: `/home/ld666/multi-slam-simulation/logs/truth_nav_single_gnss_20260821_125405`
- Optical flow: `/home/ld666/multi-slam-simulation/logs/truth_nav_single_optical_flow_20260821_130126`
- RGB-D: `/home/ld666/multi-slam-simulation/logs/truth_nav_single_vision_20260821_130622`

The optical-flow-only and vision-only truth-guided profiles defer the
continuous-odometry preflight gate until takeoff. This does not relax their
post-flight target-factor or accuracy gates. It avoids rejecting a stationary
single-aiding estimator before a displacement observation exists; for the
landed simulated MTF-01P, physical range rays are also below the floor until
ground clearance is established.
