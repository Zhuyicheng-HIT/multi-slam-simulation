# Offline Map Maintenance and Loop Closure Design

## Goal

Add a reproducible, offline-first historical mapping path to PR #18 without
changing online localization ownership. Raw observations and original poses
remain immutable; map products are rebuilt whenever corrected poses change.

## Considered approaches

### Extend the online rolling mapper

This has the smallest file count, but its data model is irreversible and tied
to callback order, rolling eviction, and already registered points. It cannot
rebuild history after a loop correction. Rejected.

### Put loop edges into the current fixed-lag backend

This would mix long-horizon graph ownership with a bounded window whose old
states have been marginalized. It would require reconstructing historical
states or misrepresenting a loop as a current-state correction. Rejected.

### Immutable archive + derived map + independent pose graph

This preserves causal online estimation, reuses tested keyframe retrieval and
registration, and makes every map revision reproducible. It requires two small
new packages and explicit schemas, but gives a clean ownership boundary.
Selected.

## Data contracts

The immutable session archive owns raw MID360 messages, IMU, calibration,
original timestamped poses, quality evidence, frame IDs, epoch/reset IDs, and
content hashes. A deterministic extractor may cache deskewed body-frame clouds.

The map-maintenance output owns voxel evidence and a cleaned map revision. A
voxel records support from distinct scans, temporal span, view diversity,
centroid/covariance, and removal/admission reason.

The relocalization layer owns descriptor retrieval and geometric verification.
Its loop output is a relative SE(3) constraint with source/target keyframe IDs,
timestamp provenance, registration metrics, information/covariance, and gate
status.

The global pose graph owns original pose priors, sequential odometry edges,
accepted loop edges, batch optimization, and corrected pose revisions. It does
not publish flight control, reset the fixed-lag estimator, or alter raw data.

## Failure behavior

- Missing or ambiguous timestamp association excludes the scan and records a
  reason; it never guesses a pose.
- Non-finite scan, pose, calibration, graph edge, or optimizer result fails the
  current derived revision without touching the previous one.
- A descriptor hit cannot create a loop edge without geometric verification.
- An unobservable or inconsistent loop remains a diagnostic candidate.
- Map cleanup reports removed geometry and must pass static-preservation tests;
  low support alone is not enough to erase raw data.
- Interrupted builds use a temporary revision directory and atomic final
  manifest rename.

## MVP acceptance

The first MVP is deliberately map-only: a deterministic tiny session containing
raw/body scans and poses can be archived, validated, rebuilt twice byte-for-byte,
and rebuilt from a supplied corrected-pose table. It reports voxel support,
isolated/floating removals, completeness, runtime, and memory. It does not yet
run a pose graph or alter online loop closure.

## Later loop-closure acceptance

Candidate retrieval, verification, graph optimization, and rebuild are tested
as separate gates. A loop is accepted only if descriptor, overlap,
correspondence, residual, observability, ambiguity, and cycle-consistency gates
pass. Corrected trajectories and maps are compared with the uncorrected build;
truth remains evaluator-only.
