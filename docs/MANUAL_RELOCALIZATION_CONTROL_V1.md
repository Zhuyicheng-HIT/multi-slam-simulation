# Manual Relocalization Control v1

The manual operator entry point is the `uf_reliability` service
`/relocalization/manual_control` (`uf_interfaces/srv/ManualRelocalization`).
`START` and `CANCEL` requests require `source=manual_control`, an episode id,
and a current timestamp. Active requests use a bounded lease (0.20--5.0 s)
renewed while the control node is alive; process loss therefore fails closed.

The node publishes only `RelocalizationRequestIntent` on
`/relocalization/request_intent`. `relocalization_request_arbiter` remains the
only producer of `/relocalization/request`. The existing active controller
consumes that aggregate request and enforces HOLD, candidate/transaction,
FusionEpoch and recovery-dwell gates. It also keeps Raw Obstacle Safety as the
motion veto. No manual node publishes MAVROS setpoints; those remain owned by
`flight_command_arbiter`.

Repeated START while active and repeated CANCEL while idle are idempotently
rejected. Timestamp regression, stale/future timestamps, unknown source and
invalid lease are rejected before an intent is emitted. The RC rising-edge
adapter is intentionally reserved and disabled in this server candidate.

Validation performed on the frozen-base worktree includes the real ROS service
smoke (START, duplicate START, CANCEL, aggregate output transition and source
ownership), arbiter core ordering/lease tests, full 22-package build and the
full 263-test colcon suite. Hardware and flight validation remain pending.
