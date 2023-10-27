#!/usr/bin/env python

import rospy
import numpy as np
import rospkg
import os
from mj_models.msg import MushrControl
from mj_models.msg import MushrPlan
from std_msgs.msg import Float64

def talker():
    rospy.init_node('mushr_open_loop_publisher', anonymous=True)
    pub = rospy.Publisher('/mushr/plan', MushrPlan, queue_size=10)
    
    # wait for simulation to start
    rospy.sleep(5.0)

    full_param_name = rospy.search_param('plan_file')
    delimiter = rospy.search_param('delimiter')
    package_path = rospkg.RosPack().get_path('mujoco_ros')
    plan_file = os.path.join(package_path, rospy.get_param(full_param_name))
    rospy.loginfo('Loading plan from %s', plan_file)
    plan = np.loadtxt(plan_file, delimiter=delimiter)
    
    rate = rospy.Rate(1) 
    msg = MushrPlan()
    for i in range(plan.shape[0]):
        ctrl = MushrControl()
        ctrl.steering_angle.data = float(plan[i, 1])
        ctrl.velocity.data = float(plan[i, 2])
        msg.control.append(ctrl)
        msg.durations.append(Float64(float(plan[i, 0])))
    # print(msg)
    pub.publish(msg)
    rate.sleep()
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
