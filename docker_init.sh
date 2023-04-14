#!/bin/bash

xhost +local:docker
docker run -it --rm --name lidar-filtering --net=host -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v ${PWD}:/workspace lidar-filtering bash -c "roscore & > /dev/null & sleep 2; bash"
