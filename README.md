# mujoco_ros
A minimal MuJoCo simulation with ROS communication.

## Usage
Inside a fresh catkin workspace, clone this repo inside `/path/to/ws/src/` (or inside `/path/to/ws` and rename this repo to `src`). Then, run `catkin_make`.

### GTSAM

Install: 
```
cd PATH/TO/DIR
git clone https://github.com/borglab/gtsam.git
cd /path/to/gtsam
git checkout 4.2.1
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release ..
make -j
make install
cd ..
```


### ML4KP

Install: 
```
cd PATH/TO/DIR
git clone https://github.com/PRX-Kinodynamic/ML4KP-devel.git
cd /path/to/ML4KP
git checkout v2Beta
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release ..
make -j
make install
cd ..
export MJ_PATH=$(pwd) 
```



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


## Run Experiments

* Launch Mujoco Mushr in an specify environment: 
  ``` roslaunch mujoco_ros mushr.launch environment:=*environment* ```
* Launch Stela window Replanning with Dirt:
  ``` roslaunch interface stela_replanning.launch  ``` 
* Read Mujoco trajectories, propagate on mushr model and publish marker
  ``` roslaunch interface mj_data_comparison.launch  ``` 