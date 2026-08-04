# Test Strategy

Tests will be added with the stage that owns the behavior:

- unit tests for score normalization, hysteresis, state transitions, geodesy, and factor residuals;
- launch tests for topic type, QoS, frame, timestamp, and ground-truth isolation;
- rosbag replay regression tests for deterministic counts and metrics;
- scenario tests for single and concurrent degradation;
- fixed-weight versus dynamic-weight backend ablations.

Large bags are external artifacts described by manifest and checksum, not Git-tracked test fixtures.
