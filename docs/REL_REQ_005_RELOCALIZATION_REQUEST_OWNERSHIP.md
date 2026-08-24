# REL-REQ-005: Relocalization Request Ownership and Arbitration

Date: 2026-08-24

## Scope and baseline

- Baseline: `integration/current-complete-pr14-20260824` at `054a6744cf2265bd4dc1bd4cee0be6287cd2dbc1`.
- REL-TRIGGER-004 at `94b05baca783c3dbeca6d6f239dac5253a6564b9` was used only as an audit reference. Its LEVEL 0--4 model is not present in this production branch and remains shadow-only.
- This change repairs request ownership only. Reliability Scheduler and Localization Safety thresholds, relocalization cooldowns, FusionEpoch/transaction/candidate validation, Active Relocalization, Raw Obstacle Safety, flight-command priority, Dynamic, LiDAR directional reliability, and Z-COV are unchanged.

## Reproduced defect

The old production topic was a source-less `std_msgs/Bool` with two publishers:

1. Reliability Scheduler published `true` and still required relocalization.
2. Localization Safety independently published `false` while releasing or recovering.
3. DDS delivered the latest `false` to subscribers, so the aggregate request became false even though Reliability Scheduler still owned an active condition.

The relocalization node's edge/transaction guard prevented some duplicate transactions, but it could not reconstruct lost source ownership. A deterministic regression test reproduces the last-writer-wins loss before applying the new OR-owned state machine.

## Architecture

```text
Reliability Scheduler ---- RelocalizationRequestIntent --+
                                                         |
Localization Safety ----- RelocalizationRequestIntent ---+-- request arbiter
                                                               |
                                                               +-- /relocalization/request Bool
                                                                   (only production publisher)

/relocalization/request --> relocalization node, scheduler state tracking,
                            Active Relocalization, mission hold consumers
```

`RelocalizationRequestIntent` contains:

- `source_id`: allow-listed ownership identity;
- `source_instance_id`: unique process incarnation;
- monotonically increasing `sequence`;
- per-source `episode_id`;
- `active` level;
- bounded `lease_duration_s`;
- source timestamp and diagnostic reason.

The final Bool interface is preserved, so the relocalization transaction consumer and existing safety/flight behavior do not require semantic changes.

## Lease and release contract

- Production sources: `reliability_scheduler` and `localization_safety`.
- Both use a `1.0 s` lease and refresh active ownership every `0.25 s`.
- Arbiter accepts active leases only in the configured `[0.20, 5.0] s` range.
- Explicit `active=false` releases only that source. The final request remains true while any other valid source is active.
- A crashed/disappeared source is automatically released when its monotonic lease deadline expires; it cannot leave a permanent request.
- Repeated heartbeats refresh a lease without publishing a new final edge, so one risk episode cannot create repeated relocalization transactions.
- Duplicate or reordered sequence numbers, retired process instances, timestamp regression, stamps older than `2.0 s`, stamps more than `0.50 s` in the future, nonfinite timing, unknown sources, and invalid leases are rejected and do not extend ownership.
- A source restart uses a new instance ID, atomically replaces its previous lease, and retires the old instance so delayed packets from the dead process cannot reacquire ownership.
- ROS clock regression clears all leases and publishes false. Producers also create a new instance contract on clock rewind. This avoids carrying pre-reset requests across simulation epochs.
- Arbiter expiry uses a steady monotonic clock, so wall/simulation timestamp manipulation cannot extend a lease.

## Producer behavior

### Reliability Scheduler

Its existing trigger computation is unchanged. On a valid automatic trigger it acquires the `reliability_scheduler` lease. It refreshes only while it owns the source request and releases its own lease after its existing matching epoch commit, result failure, or commit timeout logic. Observing a transaction initiated by another source does not grant it release ownership.

### Localization Safety

Its existing loss dwell, HOLD, readiness, recovery dwell, and retry cooldown are unchanged. It acquires and refreshes `localization_safety` only while its existing state machine requests recovery, and explicitly releases only that source after validated recovery. The final aggregate Bool remains separately subscribed for mission HOLD semantics.

## Concurrency and fault validation

| Case | Expected and observed result |
|---|---|
| Reliability true, Safety false | final true; inactive Safety release cannot clear Reliability |
| Reliability false, Safety true | final true, owned by Safety |
| Both true | one final true edge; two active owners |
| One releases while peer remains active | final remains true; zero lost request |
| Interleaved acquire/release | exactly one true edge at first acquire and one false edge at last release |
| Active heartbeats/repeated packets | lease refresh only; no extra final edge or transaction |
| Duplicate/reordered packet | rejected; cannot extend lease |
| Source crash/disappearance | source expires at lease deadline; no permanent stale request |
| Source restart | new instance replaces old lease; late old-instance packets rejected |
| Recovery/cooldown then new episode | new edge occurs only after aggregate false and the source's unchanged cooldown permits reacquire |
| Epoch commit or relocalization failure | only the owning source releases; peer ownership is preserved |
| Obstacle Safety simultaneous with request | request aggregation remains correct; existing Raw Obstacle Safety/flight arbiter veto and priority are unchanged |
| Nonfinite/stale/future/regressed time | rejected deterministically; no lease extension |

Final measured counters across the deterministic and ROS integration cases:

- lost requests: `0`;
- duplicate final true edges / duplicate transactions: `0`;
- permanent stale requests: `0`.

## Publisher ownership

Minimal production `reliability.launch.py` runtime graph:

- `/relocalization/request` publisher count: `1`;
- publisher node: `/relocalization_request_arbiter`;
- Reliability Scheduler publishes only `/relocalization/request_intent` and subscribes to the aggregate Bool;
- Localization Safety publishes only its typed intent and subscribes to the aggregate Bool.

The arbiter is included in the three existing stack entry points that independently start Reliability Scheduler: standalone reliability, unified online backend, and visual tight-coupling stack. Test-only smoke publishers remain under test/tools and are not production ownership paths.

## Regression and runtime

- Full workspace build: 20 packages passed.
- Full xUnit result: 192 tests, 0 errors, 0 failures, 0 skipped.
- Reliability package regression: 83 tests passed, including original automatic trigger, readiness, failure, FusionEpoch correlation, and commit-timeout behavior.
- New arbiter tests: 13 pure-state tests plus 2 ROS graph/integration tests passed.
- Existing relocalization, Active Relocalization, command ownership, obstacle safety, local avoidance, Dynamic, backend, and visual tests all passed in the full workspace run.
- Minimal ROS launch exited cleanly with no remaining processes.
- Pure arbitration update benchmark, 10,000 active heartbeats: P50/P95/P99 `3.747/4.183/8.674 us`; observed maximum `240.340 us` under the desktop test scheduler.

## Promotion decision

`PROMOTE_RELOCALIZATION_REQUEST_ARBITER`

The ownership defect is directly reproduced, the fix preserves downstream interfaces and all trigger/safety thresholds, final publisher ownership is unique in the production runtime graph, and concurrent release, expiry, restart, timestamp-fault, epoch/failure, and obstacle-conflict contracts pass without lost, duplicate, or permanent stale requests.
