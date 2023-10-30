#!/usr/bin/env python

import rospy
import numpy as np
import rospkg
import os
from mujoco_ros.msg import MushrControl

def talker():
    rospy.init_node('mushr_open_loop_publisher', anonymous=True)
    pub = rospy.Publisher('control', MushrControl, queue_size=10)
    
    # wait for simulation to start
    rospy.sleep(2.0)

    full_param_name = rospy.search_param('plan_file')
    package_path = rospkg.RosPack().get_path('mujoco_ros')
    plan_file = os.path.join(package_path, rospy.get_param(full_param_name))
    rospy.loginfo('Loading plan from %s', plan_file)
    plan = np.loadtxt(plan_file, delimiter=',')
    
    rate = rospy.Rate(1) 
    for i in range(plan.shape[0]):
        msg = MushrControl()
        msg.steering_angle.data = plan[i, 0]
        msg.velocity.data = plan[i, 1]
        msg.duration.data = plan[i, 2]
        pub.publish(msg)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
