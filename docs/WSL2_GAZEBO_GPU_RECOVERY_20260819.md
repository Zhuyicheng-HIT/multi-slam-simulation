# WSL2 Gazebo GPU Recovery (2026-08-19)

## Scope and frozen baseline

This work was performed on `feat/dynamic-static-map-freedom-v1`. The annotated
tag `baseline-pr14-low-altitude-five-source-20260819` remains unchanged at
`50e96f63d19e8d9292b15a684f0cc8a76f55e5bd`. No dynamic-point algorithm,
fusion threshold, sensor model, ExternalNav contract, or one-observation-one-
factor rule was changed.

## Root cause

The original host used WSL `2.3.24.0`, kernel `5.15.153.1-2`, and WSLg
`1.0.65`. The WSLg log reported that the Wayland GBM requirements for glamor
were missing and that Xwayland fell back to software. Gazebo/Ogre2 logs then
showed Mesa falling back to `kms_swrast`; no usable hardware renderer context
could pass the existing preflight. The old preflight also depended on the
optional `glxinfo` program and only rejected a subset of Mesa software
renderers. Its OpenCV CUDA/OpenCL report was unrelated to Gazebo rendering.

The apparent `/dev/dri/renderD128` permission issue was not the root cause.
After the repair, WSL exposes no `/dev/dri` node, while `/dev/dxg` and Mesa's
D3D12 Gallium driver provide a working hardware context. Xwayland's compositor
may still report its own glamor software fallback; the renderer created by the
Gazebo application must be inspected independently.

## Environment repair and adapter selection

`wsl --update --web-download` followed by `wsl --shutdown` updated the chain to:

- WSL `2.7.12.0`
- Linux kernel `6.18.33.2-2`
- WSLg `1.0.73.2`
- MSRDC `1.2.7214`
- Direct3D component `1.611.1`

The host adapters are an NVIDIA GeForce RTX 4070 Laptop GPU and a RayLink
virtual display adapter. The Windows NVIDIA driver is `32.0.15.6094`
(`560.94`, dated 2024-08-14). `/dev/dxg`, `/usr/lib/wsl/lib`, and Mesa
`d3d12_dri.so` are present. `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` selects the
physical NVIDIA adapter explicitly. No Linux NVIDIA kernel driver was installed
inside WSL and no `sudo` or system security change was used.

Temporary user-space extraction of `mesa-utils` confirmed:

```text
Vendor: Microsoft Corporation
Device: D3D12 (NVIDIA GeForce RTX 4070 Laptop GPU)
Accelerated: yes
OpenGL renderer: D3D12 (NVIDIA GeForce RTX 4070 Laptop GPU)
```

`eglinfo` initialized both Wayland and X11 EGL 1.5 displays. Its GBM platform
failed because `/dev/dri` is absent; that is expected for this working WSLg
D3D12 route. Vulkan was not selected or required by the tested Ogre2 OpenGL
backend.

## Preflight implementation

`gpu_renderer_probe.py` creates a real EGL pbuffer/OpenGL ES context using only
the system `libEGL` and `libGLESv2` libraries, then queries `GL_VENDOR`,
`GL_RENDERER`, and `GL_VERSION`. On WSL it requires both `/dev/dxg` and `D3D12`
in the actual renderer. It rejects `kms_swrast`, `llvmpipe`, `softpipe`,
`lavapipe`, generic software rasterizers, and the Microsoft Basic Render
Driver. `check_gpu_acceleration.sh` treats this EGL result as authoritative for
headless Ogre2; `glxinfo` is now an optional cross-check. OpenCV CUDA/OpenCL is
not queried and cannot affect the result.

Four deterministic unit tests cover WSL D3D12 acceptance, all recognized
software renderers, missing `/dev/dxg`/D3D12, and native Linux hardware.

## Gazebo/Ogre2 hardware proof

A minimal headless Ogre2 depth-camera scene produced color/depth/point topics.
Ogre2 reported:

```text
GL_VENDOR = Microsoft Corporation
GL_RENDERER = D3D12 (NVIDIA GeForce RTX 4070 Laptop GPU)
Device Name: D3D12 (NVIDIA GeForce RTX 4070 Laptop GPU)
```

Five minimal-scene RTF samples were `1.000184`, `0.998775`, `0.999532`,
`0.998794`, and `1.000017`. No `kms_swrast`, `llvmpipe`, or software-rasterizer
message appeared. During the full five-source run, NVIDIA utilization sampled
`13..76%` (mean `34.6%`), with about `1482 MiB` in use. Windows attributed a
separate 3D GPU load to the WSL virtual-machine worker, which corroborates the
renderer string.

A representative full-stack process sample recorded Gazebo at about
`150..168%` of one CPU core and `545..689 MiB` RSS, the backend at about `85%`
and `105 MiB`, FAST-LIO at about `55%` and `183 MiB`, the visual frontend at
about `42%` and `193 MiB`, and MAVROS at about `224%` and `221 MiB`. These
figures are process samples rather than a controlled performance benchmark;
they show why the full stack remains CPU-contention limited even though its
rendering is on the GPU.

## PR #14 frozen validation result

The external FAST-LIO checkout had been patched before the repository added its
fifth official patch. Consequently its NativeLidarFactor publisher was still
Best Effort while the frozen backend subscriber was Reliable. Applying the
already-versioned `0005-reliable-native-factor-qos.patch` to that dependency
and rebuilding restored the formal Reliable/Reliable contract. This was a
deployment repair; no estimator source or threshold in this repository changed.

Two full low-altitude figure-eight runs then reached all 14 route checkpoints,
LAND, and confirmed FCU disarm. The final no-bag run recorded:

- Native LiDAR factors: `2797`
- IMU factors: `2842`
- GNSS factors: `1440`
- optical-flow factors: `909`
- visual factors: `484`
- optimization errors: `0`
- native worker errors/overflows: `0/0`
- causal 3D RMSE/P95/max: `0.03071 / 0.05030 / 0.08842 m`
- endpoint error: `0.02322 m`
- full-stack RTF: `280 / 757.956 = 0.3694`

GPU acceleration is therefore fully reproduced, and the flight path itself is
reproduced. The complete strict PR #14 gate is **not** reproduced. Remaining
failed gates are:

1. the unified odometry maximum source-time gap was `0.277 s` (the unchanged
   gate is `0.25 s`);
2. CPU contention caused `89` superseded native worker queue items and `19`
   native inputs consumed without a state commit, with zero optimization error,
   integrity rollback, worker error, or overflow;
3. the frozen checker requires a route of at least `35 m` and checkpoints
   `1..19`, but the same frozen launcher generates a `29.08 m` route and the
   controller completes it at checkpoint `14` before LAND;
4. the auxiliary raw FAST-LIO drift checker exits nonzero for the full mission,
   even though the unified causal accuracy gate passes.

These are recorded as real blockers. No acceptance threshold or frozen route
was changed to manufacture a PASS. Diagnostic run outputs remain under ignored
`logs/gpu_recovery_pr14_*` directories and are not committed.

## Build and test closure

All 17 workspace packages built in Release mode. `colcon test` completed for
all 17 packages, and `colcon test-result --all --verbose` reported 81 test
records with zero errors, failures, or skips. The backend aggregate runner also
reported 283 tests OK and the visual frontend reported 13 tests OK. The direct
GPU probe's four unit tests, Shell syntax checks, Python compilation, and
`git diff --check` all passed.
