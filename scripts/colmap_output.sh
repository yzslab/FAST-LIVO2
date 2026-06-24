#!/bin/bash

BASE_DIR=${1}
if [[ -z "$BASE_DIR" || "$BASE_DIR" =~ ^[[:space:]]+$ ]]; then
    echo "A non-empty path to the target directory is required" >&2
    exit 1
fi

TARGET_DIRS=(
    "${BASE_DIR}/PCD"
    "${BASE_DIR}/PCD_frames"
    "${BASE_DIR}/Colmap/images"
    "${BASE_DIR}/Colmap/sparse/0"
)

for dir in "${TARGET_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        rm -rf "$dir"
        echo "Removed: $dir"
    else
        echo "Not found: $dir"
    fi
done

for dir in "${TARGET_DIRS[@]}"; do
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        echo "Created: $dir"
    else
        echo "Exists: $dir"
    fi
done

rosparam get / >"${BASE_DIR}/config.yaml"