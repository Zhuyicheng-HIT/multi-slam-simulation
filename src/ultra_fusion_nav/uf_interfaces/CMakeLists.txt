cmake_minimum_required(VERSION 3.8)
project(uf_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FaultState.msg"
  "msg/FusionEpoch.msg"
  "msg/GnssIntegrity.msg"
  "msg/LioDiagnostics.msg"
  "msg/LidarCalibrationMotion.msg"
  "msg/ReliabilityScore.msg"
  "msg/SchedulerState.msg"
  "msg/RelocalizationResult.msg"
  "msg/RgbdGeometryTrack.msg"
  "msg/RgbdGeometryTracks.msg"
  "msg/RgbdDirectTrack.msg"
  "msg/RgbdDirectTracks.msg"
  "msg/VisualFeatureTrack.msg"
  "msg/VisualFeatureTracks.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
