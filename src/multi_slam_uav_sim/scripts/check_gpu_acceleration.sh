#!/usr/bin/env bash
set -u

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
  printf 'WSLg GPU interface: not applicable\n'
fi

if command -v glxinfo >/dev/null 2>&1; then
  glx_output=$(glxinfo -B 2>&1)
  glx_status=$?
  if [[ $glx_status -eq 0 ]]; then
    renderer=$(awk -F': ' '/OpenGL renderer string/ {print $2; exit}' <<<"$glx_output")
    accelerated=$(awk -F': ' '/Accelerated/ {print $2; exit}' <<<"$glx_output")
    printf 'OpenGL renderer: %s\n' "${renderer:-unknown}"
    printf 'OpenGL accelerated: %s\n' "${accelerated:-unknown}"
  else
    printf 'OpenGL probe: failed (%s)\n' "$glx_status"
    status=1
  fi
else
  printf 'OpenGL probe: glxinfo not installed\n'
  status=1
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia_name=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -n 1)
  printf 'NVIDIA compute device: %s\n' "${nvidia_name:-unavailable}"
else
  printf 'NVIDIA compute device: nvidia-smi not available\n'
fi

if command -v python3 >/dev/null 2>&1; then
  opencv_report=$(python3 - <<'PY' 2>/dev/null
try:
    import cv2
    cuda_count = cv2.cuda.getCudaEnabledDeviceCount() if hasattr(cv2, "cuda") else 0
    opencl = bool(cv2.ocl.haveOpenCL()) if hasattr(cv2, "ocl") else False
    print(f"OpenCV {cv2.__version__}; CUDA devices={cuda_count}; OpenCL={opencl}")
except Exception as exc:
    print(f"unavailable ({exc})")
PY
)
  printf 'Optical-flow compute backend: %s\n' "$opencv_report"
fi

renderer_lower=${renderer,,}
adapter_lower=${adapter,,}
if [[ "$renderer_lower" == *llvmpipe* || "$renderer_lower" == *softpipe* || "$accelerated" == "no" ]]; then
  printf 'ERROR: Gazebo would use software rendering.\n' >&2
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
