#!/bin/bash

echo "Usage: $0 <SSH_PORT> <PORT_MAP> <CONTAINER_DIR> <IMAGE_NAME> <CONTAINER_NAME>"

SSH_PORT=${1:-2200}
PORT_MAP=${2:-6000-6099:6000-6099}
CONTAINER_DIR=${3:-/root/volume}
IMAGE_NAME=${4:-ketirobotvision/ketinet:v1}
CONTAINER_NAME=${5:-vision}

# ======================= BUILD PROCESS =======================
DOCKERFILE=Dockerfile
VOLUME_DIR=$(dirname $(dirname "$(pwd)"))

echo "======================================="
echo "🚀 Building Docker container"
echo "SSH Port:    ${PORT}"
echo "Port Map:      ${PORT_MAP}"
echo "Container dir:     ${CONTAINER_DIR}"
echo "Volume dir:     ${VOLUME_DIR}"
echo "Image: ${IMAGE_NAME}"
echo "Container: 	 ${CONTAINER_NAME}"
echo "======================================="

sudo docker run \
--name $CONTAINER_NAME \
-it \
-d \
--gpus all \
--privileged \
--env="DISPLAY=:0.0" \
-v=/tmp/.X11-unix:/tmp/.X11-unix:ro \
-v=/dev:/dev \
-v=$VOLUME_DIR:$CONTAINER_DIR \
-p ${SSH_PORT}:20 \
-p ${PORT_MAP} \
-w $CONTAINER_DIR \
-e DISPLAY=$DISPLAY \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /run/udev:/run/udev:ro \
--group-add video \
$IMAGE_NAME \
bash -c "mkdir -p $SHARE_DIR && cd $SHARE_DIR && exec bash"

#functions
echo "xhost +local:host" >> ~/.bashrc
echo "alias sb='source ~/.bashrc'" >> ~/.bashrc
echo "alias run_vision='xhost local: & docker start -ai vision'" >> ~/.bashrc
echo 'sb'
