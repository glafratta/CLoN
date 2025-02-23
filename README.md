Reactive library simplified from CLoCK, template for experimenting with closed-loop task execution.

Background:
- a Task is a closed-loop behaviour with a fixed motor output (Direction), but centered around a specific object (Disturbance)
- the properties of the Disturbance (position, dimensions) determine the duration of a Task
- by simulating Tasks in Box2D, the Configurator can identify Disturbances in the environment (e.g. obstacles)

## Hardware
The indoor robot is equipped with 
* 360 Parallax Continuous Rotation Servo motors (see [here](https://github.com/berndporr/alphabot/blob/main/alphabot.cpp) for wiring)
* A1 SLAMTEC LIDAR (see [here](https://github.com/berndporr/rplidar_rpi) for wiring)
* Raspberry Pi model 3b+

## Prerequisites
### Development packages

* G++ compiler
* CMake
* PiGPIO library
* OpenCV
* Boost
* XOrg
* LibGLU1

`sudo apt install g++ cmake libpigpio-dev libopencv-dev libboost-all-dev xorg-dev libglu1-mesa-dev`

### Compile from source

* [LIDAR API](https://github.com/berndporr/rplidar_rpi)
* [Motors API](https://github.com/berndporr/alphabot)
* [Cpp Timer](https://github.com/berndporr/cppTimer)
* [Box2D v2.4.1](https://github.com/erincatto/box2d)
  ** if not installed automatically, go to `box2d/build` and run `sudo make install`

## Build
```
cd CloCK
cmake .
sudo make install
```

## Run
### Navigation demo (Raspberry Pi)
* `sudo ./targetless` : this program demonstrates planning over a 1m distance horizon for a control goal that is not a target location but rather an objective to drive straight for the longest time with the least amount of disturbances
* `sudo ./target`: this program (under construction) demonstrates target seeking behaviour, where the target is imaginary and located at x=1.0m, y=0m.
Run with options `0`: for turning debug options off. In debug mode, LIDAR coordinates, Box2D objects and robot trajectories are dumped into the `/tmp` folder.
