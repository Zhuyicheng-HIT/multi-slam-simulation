# Active Relocalization Experiment

Date: 2026-08-07

Branch: `exp/active-relocalization-oai-ego`

## Isolation boundary

This branch is for pre-integration experiments. The first round adds a pure
decision core, tests, and an experiment-only parameter file. It does not:

- subscribe to ROS topics;
- publish position, velocity, attitude, or trajectory commands;
- load EGO-Planner;
- alter the current relocalization node, scheduler, backend, launch files, or
  default configuration.

`ego_motion_enabled` is false in both the policy default and the experiment
configuration.

## Mainline synchronization checkpoint

The project mainline is `feature/ultra-fusion-stage3`, not the Git branch named
`main`. Round 1 was created directly from
`feature/ultra-fusion-stage3@21e656d`; at branch creation, the local mainline and
`origin/feature/ultra-fusion-stage3` pointed to the same commit. The experiment
therefore began from a synchronized project-mainline baseline.

The branches named `main` and `origin/main` belong to an older, reduced history
and are not integration sources for this experiment. They must not be merged
into the experiment merely because of their names.

Before every new ablation group or structural rewrite:

1. Record `git status --short --branch` and preserve unrelated user changes.
2. Fetch and inspect `feature/ultra-fusion-stage3` and
   `origin/feature/ultra-fusion-stage3` and record their merge base.
3. Review incoming commits and file-level conflicts before integrating them.
4. Synchronize reviewed mainline commits into the experiment branch before
   beginning the new group; never merge `main` or `origin/main` by name alone.
5. Run the relocalization unit tests before changing the next group.

## Round 1 policy

The experiment policy selects actions in this order:

1. Passive search while its attempt budget remains.
2. Fixed yaw-scan views while attitude and altitude remain healthy.
3. EGO safe motion only when explicitly enabled, local odometry is healthy,
   and the obstacle map is fresh.
4. Hold position when EGO gates are not satisfied.
5. Enter failsafe when attitude or altitude stabilization is unavailable.

This policy does not decide that relocalization succeeded. Candidate retrieval,
registration, multi-frame consistency, and backend epoch commit remain owned by
their existing modules.

## Planned ablation groups

| Group | Observation action | Recovery behavior |
|---|---|---|
| A1 | Passive only | Existing transactional backend reset |
| A2 | Passive only | Recovery bootstrap for velocity and IMU biases |
| A3 | Fixed yaw scan | Recovery bootstrap |
| A4 | EGO safe motion | Recovery bootstrap |

Round 1 implements only the action-selection core needed to separate A1, A3,
and A4. No flight-control integration is claimed.
