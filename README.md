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

### 3. Docker Build and Deployment
The project uses a containerized environment to manage ROS2 Humble dependencies and OpenCV libraries using Docker. Use the provided automation scripts to build the image and launch the system:

```bash
# Build the Docker image containing ROS2 Humble and vision drivers
./docker_ws/build_gimbal.sh

# Grant X11 permissions and start the container with hardware access
./run.sh
```