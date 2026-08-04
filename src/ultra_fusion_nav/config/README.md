# Configuration Ownership

Shared algorithm configuration will live here until a ROS package has a clearer owner. Files must use relative/package-resolved paths and declare units in parameter names or comments.

Planned files include sensor topic contracts, fault-injection profiles, scheduler thresholds, and evaluation scenario matrices. Simulator-specific source parameters remain in `multi_slam_uav_sim`; backend and node parameters move into their owning ROS package when implemented.
