**Host OS does not matter.** \
(For the most part) \
All development and deployment is done inside Docker containers.

**To Launch the Docker Container**

Go into the ./docker/ folder in the repository. Then:

For Development: docker compose run dev\
For Orin: docker compose run orin\
For Nano: docker compose run nano 

**Specs and Info for Design**

Central Unit: NVIDIA Jetson Orin Nano Super Dev Kit\
Coprocessors: 2x NVIDIA Jetson Nano Dev Kit

Drone Processing Unit: NVIDIA Jetson Nano Dev Kit

Main Stereoscopic Camera: ZED 2\
Main Camera Framework: ZED SDK 5.1

Drive Motors: 6x 5304 Series Saturn Planetary Gear Motor\
Drive Motor Controllers: 2x BILDA 2x40A Motor Controller

**Compilation and Build** 

cd ros2_ws
colcon build --symlink-install
