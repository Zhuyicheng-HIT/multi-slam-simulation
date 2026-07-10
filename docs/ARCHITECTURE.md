# Architecture

## Flight State

```text
Gazebo sensors -> ArduPilot SITL -> MAVROS -> /uav/* -> companion nodes
```

Upper-level navigation nodes must not replace FCU state with Gazebo ground
truth. Gazebo pose is used only by explicitly named simulation diagnostics.

## Companion Sensors

MID360 and the front D435i are direct companion-computer sensors:

```text
Gazebo MID360 -> gz_mid360_pointcloud_bridge -> /sim/mid360/points_raw
Gazebo D435i  -> d435i_sim_bridge            -> /front/d435i/*
```

The downward optical-flow camera models an MTF-01P-like input. Diagnostic flow
and FCU injection remain separate launch modes.

## TF

```text
base_link
  front_d435i_link
    front_d435i_color_frame
      front_d435i_color_optical_frame
    front_d435i_depth_frame
      front_d435i_depth_optical_frame
    front_d435i_imu_frame
```

Optical frames use ROS conventions: +Z forward, +X right and +Y down.

## FAST-LIO

```text
/sim/mid360/points_raw -> Livox CustomMsg adapter -> FAST-LIO
/uav/imu               -> /livox/imu              -> FAST-LIO
/cloud_registered      -> reliable mapper         -> map and occupancy grid
```

FAST-LIO and Livox driver source stay in a separate external workspace. The
small project-owned reliable mapper is included in this repository.

