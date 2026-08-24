# Long Indoor Tunnel Baseline 2026-08-24

The long tunnel world is now part of the main simulation package:
`src/multi_slam_uav_sim/worlds/large_indoor_tunnel_apm_rgbd_mid360.sdf`.
It is a 96 m repetitive tunnel with periodic ribs, floor bands, colored
landmarks, textured people, RGB-D, optical flow, and MID360 sensors.

## Current run

The current stable backend was run with ExternalNav FCU consumption disabled,
`fcu_local` route feedback, and a short 8 m longitudinal rectangle inside the
96 m tunnel. This keeps the experiment measurable while retaining the tunnel
geometry and its longitudinal degeneracy.

The mission completed takeoff, all four route segments, landing, and disarm.
The estimator result failed the strict accuracy gate:

| Metric | Result |
| --- | ---: |
| 3D RMSE | 2.721 m |
| 3D P95 | 5.029 m |
| 3D maximum / endpoint | 15.117 m |
| XY RMSE | 2.719 m |
| Z RMSE | 0.095 m |

The dominant error is horizontal Y drift. FAST-LIO diagnostic odometry was
worse than the unified output, with 22.153 m 3D RMSE. The run also recorded a
0.501 s maximum unified-odometry gap and two stale stamps. The run advanced
132.849 s of simulation time in 397.235 s of wall time, for an effective RTF
of 0.334.

## Baseline upload gate

The exact worktree containing this world was also run in the ordinary indoor
world before upload. The mission completed and the strict validation passed:

| Metric | Result |
| --- | ---: |
| 3D RMSE | 0.0258 m |
| 3D P95 | 0.0346 m |
| 3D maximum | 0.0419 m |
| XY RMSE | 0.0141 m |
| Z RMSE | 0.0216 m |
| Endpoint error | 0.0226 m |

All five source-factor gates passed. The unified output had 828 samples at
10.00 Hz, a 0.133 s maximum gap, and no stale, duplicate, or regressing source
stamps.

## Directional evidence

The stable run did not perform directional handoff:

- `axis_information_handoff_enabled=false`;
- LiDAR axis information scale remained `1,1,1`;
- axis handoff frames and per-axis handoff counts were all zero;
- GNSS used a scalar reliability/information scale, not an XYZ handoff;
- LiDAR prediction-gate recovery produced 320 recovery factors after 322
  prediction rejections, but this is an all-factor recovery path, not a
  per-axis handoff.

The LiDAR diagnostic nevertheless exported directional evidence:

```text
profile information XYZ = 12120, 22849, 43942
raw information XYZ     = 60399, 26752, 151849
observability degradation XYZ = 0.848, 0.713, 0.448
condition number = 66.4
normalized Hessian eigenvalues = 0.015, 0.025, 0.060, 0.094, 0.348, 1.0
```

These values are useful diagnostics, but the current backend does not convert
them into a source-by-axis information allocation. GNSS was received 654 times
and formed 289 factors, with scalar effective information scale 0.885 and
scalar reliability weight 0.941.

## Research direction

The next implementation should be a controlled directional-information A/B,
not a new global weight. For every sensor and every sliding-window update:

1. project each factor Jacobian into the common body/world XYZ subspace;
2. accumulate a per-axis information matrix over a short causal history;
3. compare each sensor's normalized axis information against the predicted
   covariance and innovation in that axis;
4. apply a continuous diagonal information transfer, with hysteresis and a
   conservation bound, so a weak LiDAR axis can be taken over by healthy GNSS,
   RGB-D, or optical flow without changing strong LiDAR axes;
5. retain cross-axis terms and use eigenvectors when the weak direction is
   rotated relative to world X/Y/Z.

The first acceptance test should inject a single rotated weak direction, then
two simultaneous weak directions, and verify per-axis factor information,
causal error, and no improvement from future truth. A successful result must
show strong axes unchanged, only weak axes transferred, GNSS innovation bounded,
and no factor duplication. Only after that should the handoff be enabled in the
default baseline.
