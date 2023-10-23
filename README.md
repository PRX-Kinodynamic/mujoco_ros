# mujoco_ros
A minimal MuJoCo simulation with ROS communication.

## Usage
Inside a fresh catkin workspace, clone this repo inside `/path/to/ws/src/`. Then, run `catkin_make`.

## TODOs
- [*] Visualization support
- [*] Executing an open-loop plan from a file
- [ ] Closed-loop execution via a service (control in, next observation out) 
- [ ] Add viam-like robot
- [ ] Check if Xacro can be used to configure MJ models (i.e. change environment)
