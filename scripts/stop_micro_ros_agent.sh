#!/usr/bin/env bash
set -euo pipefail

IMAGE_PATTERN="microros/micro-ros-agent"

# Find running containers that match image pattern
mapfile -t matches < <(docker ps --format '{{.ID}} {{.Image}} {{.Names}}' | awk -v pat="$IMAGE_PATTERN" '$2 ~ pat {print $1" "$3}')

if [ "${#matches[@]}" -eq 0 ]; then
  echo "No running containers found matching image: $IMAGE_PATTERN" >&2
  exit 1
fi

if [ "${#matches[@]}" -gt 1 ]; then
  echo "Multiple matching containers found:" >&2
  printf '  %s\n' "${matches[@]}" >&2
  echo "Refine IMAGE_PATTERN in this script or stop one manually." >&2
  exit 2
fi

cid_name="${matches[0]}"
cid="${cid_name%% *}"
name="${cid_name#* }"

echo "Stopping container: $name ($cid)"
docker stop "$name"
