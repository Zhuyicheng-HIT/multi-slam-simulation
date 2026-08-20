#!/usr/bin/env bash
set -Eeuo pipefail

if (( $# < 2 || $# > 4 )); then
  printf 'usage: %s URL OUTPUT [EXPECTED_SIZE_BYTES] [SHA256]\n' "$0" >&2
  exit 2
fi

url=$1
output=$2
expected_size=${3:-}
expected_sha256=${4:-}
part="${output}.part"
retry_delay_s=${DATASET_DOWNLOAD_RETRY_DELAY_S:-5}
maximum_attempts=${DATASET_DOWNLOAD_MAXIMUM_ATTEMPTS:-0}

if [[ -n "$expected_size" && ! "$expected_size" =~ ^[1-9][0-9]*$ ]]; then
  printf 'EXPECTED_SIZE_BYTES must be a positive integer.\n' >&2
  exit 2
fi
if [[ -n "$expected_sha256" && ! "$expected_sha256" =~ ^[0-9a-fA-F]{64}$ ]]; then
  printf 'SHA256 must contain exactly 64 hexadecimal characters.\n' >&2
  exit 2
fi
if [[ ! "$retry_delay_s" =~ ^[0-9]+$ || ! "$maximum_attempts" =~ ^[0-9]+$ ]]; then
  printf 'Dataset download retry settings must be non-negative integers.\n' >&2
  exit 2
fi

mkdir -p "$(dirname "$output")"
if [[ -e "$output" ]]; then
  printf 'Refusing to overwrite completed output: %s\n' "$output" >&2
  exit 2
fi

attempt=0
while :; do
  attempt=$((attempt + 1))
  current_size=0
  [[ -f "$part" ]] && current_size=$(stat -c '%s' "$part")
  printf 'dataset_download attempt=%d resumed_bytes=%d output=%s\n' \
    "$attempt" "$current_size" "$output"

  set +e
  curl --location --fail --show-error \
    --connect-timeout 30 --speed-time 120 --speed-limit 1024 \
    --continue-at - --output "$part" "$url"
  curl_status=$?
  set -e

  current_size=0
  [[ -f "$part" ]] && current_size=$(stat -c '%s' "$part")
  size_complete=true
  if [[ -n "$expected_size" && "$current_size" != "$expected_size" ]]; then
    size_complete=false
  fi
  if [[ "$curl_status" == 0 && "$size_complete" == true ]]; then
    break
  fi

  printf 'dataset_download incomplete status=%d bytes=%d\n' \
    "$curl_status" "$current_size" >&2
  if (( maximum_attempts > 0 && attempt >= maximum_attempts )); then
    printf 'Dataset download exhausted %d attempts; partial file retained: %s\n' \
      "$maximum_attempts" "$part" >&2
    exit 1
  fi
  sleep "$retry_delay_s"
done

if [[ -n "$expected_sha256" ]]; then
  actual_sha256=$(sha256sum "$part" | awk '{print $1}')
  if [[ "${actual_sha256,,}" != "${expected_sha256,,}" ]]; then
    printf 'SHA256 mismatch for %s: expected=%s actual=%s\n' \
      "$part" "$expected_sha256" "$actual_sha256" >&2
    exit 1
  fi
fi

mv "$part" "$output"
printf 'dataset_download complete bytes=%d output=%s\n' \
  "$(stat -c '%s' "$output")" "$output"
