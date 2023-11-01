#!/usr/bin/env python

import rospy
import numpy as np
import rospkg
import os
from mj_models.msg import MushrPlan, MushrControl
from std_msgs.msg import Float64

def talker():
    rospy.init_node('mushr_open_loop_publisher', anonymous=True)
    pub = rospy.Publisher('control', MushrPlan, queue_size=10)
    
    # wait for simulation to start
    rospy.sleep(2.0)

    full_param_name = rospy.search_param('plan_file')
    package_path = rospkg.RosPack().get_path('mujoco_ros')
    plan_file = os.path.join(package_path, rospy.get_param(full_param_name))
    rospy.loginfo('Loading plan from %s', plan_file)
    plan = np.loadtxt(plan_file, delimiter=',')
    
    rate = rospy.Rate(1) 
    plan_msg = MushrPlan()
    controls = []
    durations = []
    for i in range(plan.shape[0]):
        ctrl_msg = MushrControl()
        ctrl_msg.steering_angle.data = plan[i, 0]
        ctrl_msg.velocity.data = plan[i, 1]
        duration_msg = Float64()
        duration_msg.data = plan[i,2]
        controls.append(ctrl_msg)
        durations.append(duration_msg)

    plan_msg.controls = controls 
    plan_msg.durations = durations

    pub.publish(plan_msg)
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
