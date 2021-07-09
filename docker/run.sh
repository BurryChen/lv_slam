#!/usr/bin/env bash

set -e

# Default settings
HOST_SOURCE_DIR="/home/chenshoubin/code_ws/slam_ws"
IMAGE="slam_ws:ubuntu18.04"

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
VOLUMES="--volume=$XSOCK:$XSOCK:rw
         --volume=$HOST_SOURCE_DIR:/home/chenshoubin/code_ws/slam_ws:rw "
#$VOLUMES \
    
set -x

docker run \
    --privileged \
    -it --rm \
    --name slam_ws \
    --net host \
    --env HTTP_PROXY=$http_proxy \
    --env HTTPS_PROXY=$https_proxy \
    $VOLUMES \
    --env DISPLAY=${DISPLAY} \
    -w /home/chenshoubin/code_ws/slam_ws \
    $IMAGE

set +x
