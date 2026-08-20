# Structural Relocalization Scene Variant

## Purpose

The frozen low-altitude five-source scene is visually repetitive and gives
relocalization very little structural stress. This branch adds a separate
scene for structural-degradation experiments. The original
`low_indoor_apm_rgbd_mid360.sdf` is unchanged.

## Geometry

- World name: `low_indoor_apm_rgbd_mid360_window`
- Route: the existing single-pass large figure-eight, centered at the room
  origin and rotated by 158 degrees.
- Ceiling: four panels leave a `4 m x 4 m` central skylight at `z=4.0 m`.
  The route's central section therefore transitions from covered indoor space
  to an open-sky view while the outer route remains covered.
- North wall: the middle has a `3 m x 1.6 m` aperture. The sill, header, and
  jambs remain as collision and visual geometry. The outer facade is split at
  the same location so it cannot mask the opening.
- The window has no glass visual or collision. RGB-D and LiDAR both observe
  the real opening and the background beyond it.

## Reproduction

Build the simulation package first:

```bash
cd /home/zyc/multi-slam-passive-relocalization
source /opt/ros/humble/setup.bash
source /home/zyc/multi-slam-deps/mid360_ws/install/setup.bash
colcon build --symlink-install --packages-select multi_slam_uav_sim
```

Run the scene with the passive relocalization baseline:

```bash
bash tools/run_window_opening_relocalization_validation.sh
```

The entry point defaults to `hold` motion and
`stationary_zero/preserve`, so it does not introduce EGO-style active motion.
For the later active-motion group, set for example:

```bash
VALIDATION_RELOCALIZATION_CHECKPOINTS=8 \
VALIDATION_RELOCALIZATION_MOTION_PROFILE=figure8 \
bash tools/run_window_opening_relocalization_validation.sh
```

Do not mix these logs with the frozen indoor scores. Record the scene name,
checkpoint, motion profile, and whether the run completed its full duration.
