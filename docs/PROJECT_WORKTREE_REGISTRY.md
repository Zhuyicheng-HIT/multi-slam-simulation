# Project worktree registry

This registry prevents the dynamic-environment and Z-axis work lines from
being confused or modified together.

## Dynamic line

- Branch: `feat/dynamic-static-map-freedom-v1`
- Baseline commit: `64dd899cb5ef0f11a8e3324626971f000a8a5e8c`
- Baseline tag: `dyn-map-006-long-term-static-refinement-20260820`
- Absolute WSL path: `/home/zyc/projects/multi-slam-ultrafusion-visual-tight-20260807`
- Current stage: `DYN-LOC-007`

The PR #14 frozen tag remains
`baseline-pr14-low-altitude-five-source-20260819`. DYN-LOC-007 must not modify
the Z-axis worktree or the frozen tag.

## Latest relocalization integration line

- Branch: `integration/dynamic-latest-relocalization-v1`
- Upstream baseline: `origin/exp/passive-relocalization-reinit-five-source`
- Upstream baseline commit: `35e234dd063b16e47c1f995fce9a1758349be581`
- Frozen Dynamic source commit: `29e580448b33cd8bd1a5815808435c2ac4a9342f`
- Frozen Dynamic source tag: `dyn-loc-007-dynamic-localization-20260820`
- Absolute WSL path:
  `/home/zyc/projects/multi-slam-dynamic-latest-relocalization-20260820`
- Current stage: latest-relocalization integration and local validation

This line is local-only. It does not rewrite the frozen Dynamic branch/tag and
must not be confused with the existing Z-axis worktree.
