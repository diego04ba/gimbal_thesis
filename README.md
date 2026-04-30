# Gimbal Target Tracking System (ROS2)

This project implements a real-time automated target tracking system based on ROS2 Humble. The system integrates industrial computer vision and PID control to enable a 2-axis gimbal to follow a visual reference (ArUco Marker).

## System Overview
The system captures frames from a FLIR camera, detects the ArUco marker's position, and calculates the pixel error relative to the frame center. A PID controller processes this error to generate velocity commands, which are sent to the gimbal controller via a serial binary protocol.

### Hardware Specifications
*   **Camera:** FLIR Firefly S (FFY-U3-16S2C-CS) - Industrial vision via Spinnaker SDK.
*   **Gimbal Controller:** BaseCam SimpleBGC 2.2 (Firmware 2.2b2 - 8-bit).
*   **Protocol:** SimpleBGC Binary Protocol via Serial/USB.

## Software Architecture
The project consists of four main ROS2 nodes:
1.  **spinnaker_camera_driver**: Industrial driver for high-speed image acquisition.
2.  **aruco_detector**: Processes camera frames to detect markers and publishes pixel errors. 
3.  **pid_controller**: Implements the control logic. 
4.  **gimbal_driver**: The hardware interface that translates ROS Twist messages into SimpleBGC binary packets for the AlexMos board.

## Installation and Setup

### 1. Prerequisites
Ensure X11 Server Utils are installed on the host machine to enable graphical output (rqt_image_view) from the Docker container:
```bash
sudo apt update && sudo apt install -y x11-xserver-utils
xhost +local:root
```

### 2. Spinnaker SDK Installation
To ensure full hardware compatibility and optimal performance with the FLIR Firefly S camera, the Spinnaker SDK should be installed on the host system. This is necessary to correctly configure udev rules for USB device permissions and to manage the kernel's USBFS memory limits, which are critical for high-resolution industrial image streaming.
**Note**: To ensure the camera is detected, modify the complete.launch.py file by entering your camera's specific serial number as a string (e.g., '123456789').

### 3. Correct Gimbal Interfacing
The gimbal is typically assigned to /dev/ttyUSB0. If the device is assigned to a different port, locate it and update the serial_port parameter in complete.launch.py.

```bash
# To identify the port assigned to the gimbal run in the terminal
ls -l /dev/ttyUSB* /dev/ttyACM*
```

To grant the necessary permissions for the Docker container to access the serial port:

```bash
# Run this command using the correct port identified above
sudo chmod 666 /dev/ttyUSB0 
sudo usermod -a -G dialout $USER
```

To verify that the container correctly recognizes the serial port, you can run the following inside the container:

```bash
python3 -m serial.tools.list_ports
```

### 4. Docker Build and Deployment
The project uses a containerized environment to manage ROS2 Humble dependencies and OpenCV libraries using Docker. Use the provided automation scripts to build the image and launch the system:

```bash
# Build the Docker image containing ROS2 Humble and vision drivers
./docker_ws/build_gimbal.sh

# Grant X11 permissions and start the container with hardware access
./run.sh
```
Inside the container, navigate to the ros_workspace directory (running the container should already take you there), build the package, and launch the system:

```bash
# Build the workspace
cd ros_workspace/ # You should already be in this directory
colcon build --symlink-install
source install/setup.bash

# To deploy the system
ros2 launch gimbal_tracker complete.launch.py
```