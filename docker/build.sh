#!/usr/bin/env bash

set -e
    
docker build \
    --rm \
    --network=host \
    --build-arg=HTTP_PROXY=$http_proxy \
    --build-arg=HTTPS_PROXY=$https_proxy \
    --tag slam_ws:ubuntu18.04 \
    --file Dockerfile .

echo "已构建新镜像"

    # --network=host \
    # --build-arg=HTTP_PROXY=$http_proxy \
    # --build-arg=HTTPS_PROXY=$https_proxy \