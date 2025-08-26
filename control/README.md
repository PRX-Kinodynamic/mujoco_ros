README.md


Publish grid and environment
```
roslaunch utils sdf_grid_publisher.launch
```

Setup MPPI for ackermann system
```
rosrun control mppi_ackermann_fo.py _sample_rollouts:=1000
```

Call the MPPI service
```
rosservice call /mppi/run "control:
  point: [0,0]
start:
  point: [0,0,0]
goal:
  point: [10,10,0]"
```