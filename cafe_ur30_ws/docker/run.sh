#!/bin/bash
# Convenience wrapper:   docker/run.sh [fortress|harmonic] [extra launch args, e.g. gui:=false]
set -e
TAG=${1:-fortress}; shift || true
xhost +local:docker >/dev/null 2>&1 || true
GPU=(); [ -d /dev/dri ] && GPU+=(--device /dev/dri)
command -v nvidia-smi >/dev/null 2>&1 && GPU+=(--gpus all)
docker run --rm -it --net=host --ipc=host \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw "${GPU[@]}" \
  "cafe-ur30:$TAG" ros2 launch cafe_bringup cafe.launch.xml "$@"
