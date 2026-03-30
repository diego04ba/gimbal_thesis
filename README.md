# gimbal_thesis
My first automated project. Given a certain reference, the camera (in my case the FFY-U3-16S2C-CS, FLIR firefly S) has to follow it by giving a Gimbal (BGC 2.2 (2.2b2)) control commands.
Started by using ArUco markers as a reference, the idea is to also add the possibility to follow any reference.
It has also nodes for simulation of ArUco detection (with an .mp4 file) and tuning of the PID controller.

Use the docker build script to build the necessary image to run the container.
To run the first command on the run script you also need to have installed the X11 Server Utils, run this command on the bash:
sudo apt update && sudo apt install -y x11-xserver-utils
