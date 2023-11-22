# mujoco_ros
A minimal MuJoCo simulation with ROS communication.

## Usage
Inside a fresh catkin workspace, clone this repo inside `/path/to/ws/src/` (or inside `/path/to/ws` and rename this repo to `src`). Then, run `catkin_make`.

### Mujoco
Download, compile and install mujoco:
```
cd /path/to/mujoco
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release ..
make -j
make install
cd ..
export MJ_PATH=$(pwd) 
```

## Tests
Running tests for all packages:
```
catkin_make               # Compile and generate msgs
catkin_make run_tests     # Run tests 
catkin_test_results       # Check for failures
```

## TODOs
- [x] Visualization support
- [x] Executing an open-loop plan from a file
- [x] Closed-loop execution via a service (control in, next observation out) 
- [ ] Check if Xacro can be used to configure MJ models (i.e. change environment)
- [ ] Use nodelets for faster message copies?
- [ ] Check for unnecessary copies
- [ ] Replace the current `feedback_client` (or at least refactor it) so it spawns `mushr_simulation` and connects via topics.