# Simulation versus real-hardware performance

## Transferable compute

The marginal-prior block transform, transaction snapshot reduction, batched
analytic visual factor, batched voxel indexing, structured PointCloud2 packing,
bounded diagnostics and `RelWithDebInfo` build apply unchanged on the companion
computer.  The measured estimator, visual frontend, reliability, mapping, ROS
transport and memory/copy costs are therefore relevant to hardware sizing.

## SIM_ONLY cost

Gazebo rendering/physics, gz-to-ROS bridges, ArduPilot SITL and WSL scheduling
do not transfer to the aircraft.  In the representative profile Gazebo alone
used about 5.2% of total WSL capacity and roughly 0.54 GiB RSS, while bridges
used about 1.2% and SITL about 0.18%.

The machine exposed `/dev/dxg` and an RTX 4070, but OpenCV reported zero CUDA
devices and no OpenCL.  EGL could not open `/dev/dri/renderD128` (permission
denied) and fell back to `kms_swrast`; `glxinfo` was unavailable.  Headless
rendering was already enabled.  Fixing device permissions or the WSL graphics
stack requires a host/system change and was intentionally not attempted with
`sudo`.

Consequently, the solver improvement is a valid algorithm result, while the
small live RTF change must be labelled SIM_ONLY and must not be projected to
the vehicle.  A short deterministic backend replay would further isolate the
estimator, but no new rosbag was committed and no fabricated replay result is
reported.
