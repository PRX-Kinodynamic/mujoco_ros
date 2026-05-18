# Running on real robot

roslaunch motion_planning stela.launch  environment:=environments/prx_lab_3.yaml obstacle_mode:=sdf obstacle_factor_include_distance:=1.0 obstacle_distance_tolerance:=0.5 obstacle_sigma:=.0 data_dir:=data/ experiment_id:=exp_000 Plant:=mushr plant_file:=/tmp/mushr.yaml robot_frame:=robot_0 Simulator:=false


roslaunch motion_planning stela.launch planner:=StelaWindowed Stepper:=None sim_clock:=false visualize:=true report_control_frequency:=true stela_future_nodes:=10 stela_past_nodes:=10  environment:=environments/prx_lab_3.yaml obstacle_mode:=sdf obstacle_factor_include_distance:=0.5 obstacle_distance_tolerance:=0.5 obstacle_sigma:=1.0 data_dir:=data/ experiment_id:=exp_000 Plant:=mushr plant_file:=/tmp/mushr.yaml robot_frame:=robot_0 Simulator:=false




obstacle_params="obstacle_mode:=sdf obstacle_factor_include_distance:=0.5 obstacle_distance_tolerance:=0.5 obstacle_sigma:=1.0"



python $(rospack find utils)/scripts/stela_experiments.py -e /common/home/eg585/perception/stela_ws/src/mujoco_ros/motion_planning/config/experiments/prx_lab_3.yaml -p /common/home/eg585/perception/stela_ws/src/mujoco_ros/motion_planning/config/mushr.yaml -i 0 -o /tmp/mushr.yaml

# Collecting Data
(This only collects the markers & controls for postprocessing, no video, no images)

Rosbag record:
``` 
rosbag record /perception/logitech_brio_73B6FC82/aruco/markers /perception/logitech_brio_96214279/aruco/markers /mushr/vesc/commands/motor/speed /mushr/mux/ackermann_cmd_mux/output /mushr/plan_stepper/control /mushr/plan_stepper/control_stamped -O /common/users/eg585/stela_kraft/test.bag
``` 
```
roslaunch interface cameras_303_desktop_10.launch
roslaunch interface plan_from_file.launch simulation:=false

``` 

# Cameras

``` 
roslaunch interface cameras_303_desktop_10.launch
``` 

# ROBOT

ssh robot@mushr.cs.rutgers.edu

cd prx_ws/
source devel/setup.bash
roslaunch mushr_base prx_mushr.launch

export ROS_MASTER_URI=http://1spring-303-desktop-10.cs.rutgers.edu:11611/

# Foxglove
Using foxglove bridge (to allow calling services from foxglove)
``` 
roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765 send_buffer_limit:=1000000000
``` 