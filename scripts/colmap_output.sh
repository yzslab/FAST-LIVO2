#!/bin/bash

BASE_DIR=$(/opt/ros/noetic/bin/rospack find fast_livo)

TARGET_DIRS=(
    "${BASE_DIR}/Log/PCD_frames"
    "${BASE_DIR}/Log/Colmap/images"
    "${BASE_DIR}/Log/Colmap/sparse/0"
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

