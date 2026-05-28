# Target Tracking System (ROS2)

This project implements a real-time automated target tracking system based on ROS2 Humble. The system integrates industrial computer vision and PID control to enable a 2-axis gimbal to follow a visual reference (ArUco Marker).
**Note**: The project initially utilized a FLIR camera and a separate gimbal stabilizer. However, due to hardware limitations, it has been transitioned to a fully integrated PTZ (Pan-Tilt-Zoom) camera system.

## System Overview
The system captures video frames directly from the PTZ camera, detects the ArUco marker's position, and calculates the pixel error relative to the frame center. A PID controller processes this error to generate velocity commands, which are sent to the camera's Pan-Tilt motors via HTTP requests (Axis VAPIX protocol).

## Hardware Specifications
*   **PTZ Camera:** PTZ AXIS M5525-E - Network Camera provided by Axis Communication.
*   **Communication Protocol:** HTTP (VAPIX API).

## Software Architecture
The project consists of four main ROS2 nodes:
1.  **axis_camera**: Official driver for image acquisition from the Axis PTZ Camera.
2.  **aruco_detector**: Processes camera frames to detect markers and publishes pixel errors. 
3.  **pid_controller**: Implements the control logic. 
4.  **axis_driver**: The hardware interface that translates ROS Twist messages into HTTP requests to drive the PTZ motors.

## Installation and Setup

### 1. Prerequisites
Ensure X11 Server Utils are installed on the host machine to enable graphical output (rqt_image_view) from the Docker container:
```bash
sudo apt update && sudo apt install -y x11-xserver-utils
xhost +local:root
```


### 2. Axis PTZ Camera Configuration
This system is designed to be compatible with the Axis M55 series (e.g., M5525-E).

**Network Setup**: Use the AXIS IP Utility software to find the camera's IP address and ensure the IPv4 address matches the one specified in `ptz_complete.launch.py`. By default, the camera might receive a dynamic IP from your local DHCP server. To ensure the ROS nodes can always reach the camera without manually updating the launch files after every reboot, please log into the camera's web interface via browser. From there, set the network configuration to "Static / Manual", specifying and locking in the exact IP address you want to use for your system.
The system uses encrypted Digest authentication by default. If you have set up a custom username and password on the camera, remember to update these parameters in the launch file.

**Troubleshooting Network Connectivity (Ubuntu)**: If your PC fails to reach the camera (e.g., if connected directly via Ethernet without a DHCP router), your network interface might require a static IP configuration. You can force the connection using `nmcli` in the terminal:
```bash
# Create a static profile for the Ethernet port (adjust 'eth0' if your interface name differs)
sudo nmcli con add con-name "Axis_Camera" ifname eth0 type ethernet ipv4.addresses 192.168.0.100/24 ipv4.method manual
# Activate the connection
sudo nmcli con up "Axis_Camera"
# Verify the connection (the default Axis fallback IP is usually 192.168.0.90)
ping 192.168.0.90 # if it worked you should see something such as "64 bytes from 192.168.0.90: icmp_seq=1..."
```


### 3. Docker Build and Deployment
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
ros2 launch gimbal_tracker ptz_complete.launch.py
```

**Performance Tuning**: To mitigate system latency and reduce CPU load during execution, you can adjust the resize factor within the `complete.launch.py` file in the ArUco Detector section.  
**Note**: This parameter requires a floating-point value between 0.0 and 1.0 (e.g. {'resize_factor': 0.5}).


## System Overview (OLD)
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
**Note**: To ensure the camera is detected, modify the `complete.launch.py` file by entering your camera's specific serial number as a string (e.g., '123456789').

### 3. Gimbal Interfacing
The gimbal is typically assigned to `/dev/ttyUSB0` with a default baudrate of `115200`. If the device is assigned to a different port or the internal baudrate has been modified via the SimpleBGC GUI, update the corresponding parameters in `complete.launch.py`.

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

**Note on Baudrate**: While `115200` is the standard for BGC 2.2 firmware, if you experience communication issues, verify the board settings in the SimpleBGC GUI and ensure the baudrate parameter in the launch file matches the hardware configuration.

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
ros2 launch gimbal_tracker gimbal_complete.launch.py
```

To mitigate system latency and reduce CPU load during execution, you can adjust the resize factor within the `complete.launch.py` file in the ArUco Detector section.  
**Note**: This parameter requires a floating-point value between 0.0 and 1.0 (e.g. {'resize_factor': 0.5}).
