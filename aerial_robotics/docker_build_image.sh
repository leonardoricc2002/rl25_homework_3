#!/bin/bash

if [[ "$@" == "--help" || "$@" == "-h" ]]; then
    echo "Usage: $0 [IMAGE_NAME]"
    exit 0
fi

if [[ $# -eq 0 ]]; then
    echo "Usage: $0 [IMAGE_NAME]"
    exit 1
fi

IMAGE_NAME=$1

BUILD_DIR="build_files"

if [[ ! -f "$BUILD_DIR/Dockerfile" ]]; then
    echo "Dockerfile not found in $BUILD_DIR/"
    exit 1
fi

echo "Building image: $IMAGE_NAME"
echo "Using Dockerfile from: $BUILD_DIR/Dockerfile"

docker build \
    -t "$IMAGE_NAME" \
    -f "$BUILD_DIR/Dockerfile" \
    --build-arg USER_ID=$(id -u) \
    --build-arg GROUP_ID=$(id -g) \
    "$BUILD_DIR"