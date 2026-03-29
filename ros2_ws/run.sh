xhost +
docker run -it --rm --net host --ipc host --privileged \
    --device=/dev/video0:/dev/video0 \
    --device=/dev/ttyUSB0:/dev/ttyUSB0 \
    --group-add dialout \
    --shm-size=2g \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority \
    -v /dev:/dev \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTHORITY \
    -v ./ros_ws/:/root/ros_workspace \
    --name gimbal \
    ros:gimbal bash
    