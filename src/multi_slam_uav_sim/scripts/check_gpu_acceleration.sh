#!/usr/bin/env bash
set -u

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
required=${REQUIRE_GAZEBO_GPU:-0}
adapter=${GAZEBO_GPU_ADAPTER:-${MESA_D3D12_DEFAULT_ADAPTER_NAME:-}}
renderer="unknown"
accelerated="unknown"
status=0

printf 'Gazebo render engine: %s\n' "${GZ_RENDER_ENGINE:-ogre2}"
printf 'Requested WSLg adapter: %s\n' "${adapter:-auto}"

if [[ -e /dev/dxg ]]; then
  printf 'WSLg GPU interface: /dev/dxg present\n'
else
  printf 'WSLg GPU interface: /dev/dxg absent (native Linux or no WSL vGPU)\n'
fi

if [[ "${LIBGL_ALWAYS_SOFTWARE:-0}" == "1" ]]; then
  printf 'ERROR: LIBGL_ALWAYS_SOFTWARE=1 forces software rendering.\n' >&2
  status=1
fi

if command -v python3 >/dev/null 2>&1; then
  probe_output=$(python3 "$SCRIPT_DIR/gpu_renderer_probe.py" 2>&1)
  probe_status=$?
  egl_vendor=$(sed -n 's/^egl_vendor=//p' <<<"$probe_output" | head -n 1)
  egl_version=$(sed -n 's/^egl_version=//p' <<<"$probe_output" | head -n 1)
  gl_vendor=$(sed -n 's/^gl_vendor=//p' <<<"$probe_output" | head -n 1)
  renderer=$(sed -n 's/^gl_renderer=//p' <<<"$probe_output" | head -n 1)
  gl_version=$(sed -n 's/^gl_version=//p' <<<"$probe_output" | head -n 1)
  accelerated=$(sed -n 's/^hardware_accelerated=//p' <<<"$probe_output" | head -n 1)
  hardware_reason=$(sed -n 's/^hardware_reason=//p' <<<"$probe_output" | head -n 1)
  printf 'EGL probe backend: egl_gles_pbuffer\n'
  printf 'EGL vendor: %s\n' "${egl_vendor:-unknown}"
  printf 'EGL version: %s\n' "${egl_version:-unknown}"
  printf 'OpenGL vendor: %s\n' "${gl_vendor:-unknown}"
  printf 'OpenGL renderer: %s\n' "${renderer:-unknown}"
  printf 'OpenGL version: %s\n' "${gl_version:-unknown}"
  printf 'Hardware accelerated: %s (%s)\n' \
    "${accelerated:-unknown}" "${hardware_reason:-unknown}"
  if [[ $probe_status -ne 0 || "$accelerated" != "yes" ]]; then
    printf 'ERROR: EGL/OpenGL hardware probe failed.\n' >&2
    printf '%s\n' "$probe_output" >&2
    status=1
  fi
else
  printf 'EGL/OpenGL probe: python3 not installed\n'
  status=1
fi

# glxinfo is a useful WSLg cross-check, but it is not part of the base Ubuntu
# install. The EGL context above is authoritative for headless Ogre2.
if command -v glxinfo >/dev/null 2>&1; then
  glx_output=$(glxinfo -B 2>&1)
  glx_status=$?
  if [[ $glx_status -eq 0 ]]; then
    glx_renderer=$(awk -F': ' '/OpenGL renderer string/ {print $2; exit}' <<<"$glx_output")
    glx_accelerated=$(awk -F': ' '/Accelerated/ {print tolower($2); exit}' <<<"$glx_output")
    printf 'GLX renderer cross-check: %s\n' "${glx_renderer:-unknown}"
    printf 'GLX accelerated cross-check: %s\n' "${glx_accelerated:-unknown}"
    glx_renderer_lower=${glx_renderer,,}
    if [[ "$glx_renderer_lower" == *llvmpipe* ||
          "$glx_renderer_lower" == *softpipe* ||
          "$glx_renderer_lower" == *swrast* ||
          "$glx_renderer_lower" == *lavapipe* ||
          "$glx_accelerated" == "no" ]]; then
      printf 'ERROR: GLX cross-check reports software rendering.\n' >&2
      status=1
    fi
  else
    printf 'GLX cross-check: failed (%s)\n' "$glx_status"
    status=1
  fi
else
  printf 'GLX cross-check: glxinfo not installed (optional)\n'
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia_name=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -n 1)
  printf 'NVIDIA compute device: %s\n' "${nvidia_name:-unavailable}"
else
  printf 'NVIDIA compute device: nvidia-smi not available\n'
fi

renderer_lower=${renderer,,}
adapter_lower=${adapter,,}
if [[ "$renderer_lower" == *llvmpipe* ||
      "$renderer_lower" == *softpipe* ||
      "$renderer_lower" == *swrast* ||
      "$renderer_lower" == *lavapipe* ||
      "$accelerated" != "yes" ]]; then
  printf 'ERROR: Gazebo would use software rendering.\n' >&2
  status=1
fi
if [[ -e /dev/dxg && "$renderer_lower" != *d3d12* ]]; then
  printf 'ERROR: WSLg renderer is not using the Mesa D3D12 backend.\n' >&2
  status=1
fi
if [[ -n "$adapter_lower" && "$renderer" != "unknown" && "$renderer_lower" != *"$adapter_lower"* ]]; then
  printf 'ERROR: requested adapter %s does not match renderer %s.\n' "$adapter" "$renderer" >&2
  status=1
fi

if [[ $status -ne 0 && "$required" != "1" ]]; then
  printf 'WARNING: GPU validation failed; continuing because REQUIRE_GAZEBO_GPU=%s.\n' "$required" >&2
  exit 0
fi
exit "$status"
