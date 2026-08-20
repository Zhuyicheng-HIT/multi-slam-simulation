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
