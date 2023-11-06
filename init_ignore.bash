#!/usr/bin/env bash
# Populate `CATKIN_IGNORE` files according to where this is `catkin_make`

# Touch CATKIN_IGNORE in a given list of packages
# Use: `populate_catkin_ignore "LIST_OF_PACKAGES[@]" `
populate_catkin_ignore () {
  pakage_list=("$@")
	for package in "${pakage_list[@]}"
	do
  	echo "Ignoring ${package}"
	  touch $(pwd)/$(dirname $0)/${package}/CATKIN_IGNORE
	done
}

## List packages needed
# Simulation (Mujoco)
simulation=("world/mujoco_ros")
simulation+=("world/mushr/mushr_sim")
# Mushr
mushr=("world/mushr/mushr/mushr_base/mushr_base/mushr_base")
mushr+=("world/mushr/mushr/mushr_hardware/mushr_hardware")
mushr+=("world/mushr/mushr/mushr_hardware/realsense/realsense2_description")
mushr+=("world/mushr/mushr/mushr_base/vesc/vesc") # Metapackage
mushr+=("world/mushr/mushr/mushr_base/vesc/vesc_main")
mushr+=("world/mushr/mushr/mushr_base/vesc/vesc_msgs")
mushr+=("world/mushr/mushr/mushr_hardware/push_button_utils")
mushr+=("world/mushr/mushr/mushr_base/mushr_base/ackermann_cmd_mux")
mushr+=("world/mushr/mushr/mushr_hardware/rplidar_ros")
mushr+=("world/mushr/mushr/mushr_hardware/realsense/realsense2_camera")
mushr+=("world/mushr/mushr/mushr_base/vesc/vesc_ackermann")
mushr+=("world/mushr/mushr/mushr_base/vesc/vesc_driver")
mushr+=("world/mushr/mushr/mushr_hardware/ydlidar")
mushr+=("world/mushr/mushr/mushr_description")
# mushr+=("")

# Perception
# perception=("")
# 

if [ "${1}" = "mushr" ]; then
  echo "Built is in Mushr, ignoring simulation-specific packages"
	populate_catkin_ignore "${simulation[@]}"
elif [ "${1}" = "PC" ]; then
  echo "Ignoring robot-specific packages"
	populate_catkin_ignore "${mushr[@]}"
fi