# Manual Relocalization Control v1

The manual operator entry point is the `uf_reliability` service
`/relocalization/manual_control` (`uf_interfaces/srv/ManualRelocalization`).
`START` and `CANCEL` requests require `source=manual_control`, an episode id,
and a current timestamp. Active requests use a bounded lease (0.20--5.0 s)
renewed while the control node is alive; process loss therefore fails closed.

The ROS CLI uses the existing production launch; no direct request topic
publication is permitted. For example, start and cancel one operator episode
with:

```bash
ros2 service call /relocalization/manual_control uf_interfaces/srv/ManualRelocalization \
  "{command: 1, source: manual_control, episode_id: 1001, lease_duration_s: 1.0}"
ros2 service call /relocalization/manual_control uf_interfaces/srv/ManualRelocalization \
  "{command: 2, source: manual_control, episode_id: 1001, lease_duration_s: 1.0}"
```

An omitted timestamp is stamped by the control node's current ROS clock; a
supplied timestamp must be current and monotonic. `command: 1` is START and
`command: 2` is CANCEL.

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
adapter is intentionally reserved and disabled in this server candidate. The
existing command arbiter retains the mission intent while the active controller
temporarily owns its relocalization intent; after the existing recovery gate
releases it, normal priority selects the still-fresh mission intent again.

The operational state mapping is: `IDLE` is the inactive service state,
`MANUAL_REQUESTED` is the owned manual intent, followed by the existing
`HOLD`, `ACTIVE_RELOCALIZATION`, `RECOVERY_VALIDATION` and `RESUME` states.
Any cancel during HOLD/ACTIVE, missing candidate, failed transaction, epoch
mismatch, unsafe Raw Obstacle Safety state or recovery timeout ends in the
existing fail-closed `HOVER_REQUIRED`/localization-hold path. Manual override
and FCU failsafe remain above all automatic command intents in the command
arbiter.

Validation performed on the frozen-base worktree includes real ROS service
tests (START, duplicate START, CANCEL, timestamp regression, source-loss
lease expiry, aggregate output transition and publisher ownership), arbiter
ordering/lease tests and deterministic active-controller tests for candidate,
epoch, obstacle, cancellation and recovery failure. Hardware and flight
validation remain pending.
