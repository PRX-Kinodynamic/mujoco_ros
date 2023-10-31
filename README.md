# mujoco_ros
A minimal MuJoCo simulation with ROS communication.

## Usage
Inside a fresh catkin workspace, clone this repo inside `/path/to/ws/src/`. Then, run `catkin_make`.

### Mujoco
Download, compile and install mujoco:
```
cd /path/to/mujoco
mkdir build & cd build
cmake -DCMAKE_INSTALL_PREFIX="../install" ..
make -j
make install
cd ..
export MJ_PATH=$(pwd) 
```

## TODOs
- [*] Visualization support
- [*] Executing an open-loop plan from a file
- [ ] Closed-loop execution via a service (control in, next observation out) 
- [ ] Check if Xacro can be used to configure MJ models (i.e. change environment)