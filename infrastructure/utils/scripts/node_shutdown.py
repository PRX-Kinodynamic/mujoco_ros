#!/usr/bin/env python
import rospy
import roslaunch
import rosnode
import time
from std_msgs.msg import Bool
from prx_models.msg import Tree
import subprocess

import rosbag

bag_dir="/home/gary/motion_planning/catkin_ws/bags/stela/"
bag_file="simple_obstacle_aorrt_003.bag"
# bag_file="test_cost_aorrt_v2.bag"


tree_topic="/stela/sbmp/sln_tree"
node_name="node_shutdown"

class launcher: 
    def bag_publisher(self):
        bag = rosbag.Bag(bag_dir + bag_file)
        for topic, msg, t in bag.read_messages(topics=[tree_topic]):
            self.tree_pub.publish(msg)
            # print(msg)
        bag.close()
    def callback(self, data):
        if data.data:
            self.launch.shutdown()
            time.sleep(5)
            nodes = rosnode.get_node_names()
            rosnode.kill_nodes(nodes);
            exit(0)
    def __init__(self):
        rospy.init_node(node_name, anonymous=False)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file="/home/gary/motion_planning/catkin_ws/src/mujoco_ros/motion_planning/launch/stela.launch"
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        self.launch.start()

        rospy.Subscriber('/stela/finished', Bool, self.callback)
        self.tree_pub = rospy.Publisher(tree_topic, Tree, queue_size=1,latch=True)
        
        time.sleep(5)
        self.bag_publisher();
        rospy.spin()

if __name__ == '__main__':
    try:
        launcher = launcher()
    except rospy.ROSInterruptException:
        pass