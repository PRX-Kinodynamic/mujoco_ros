#!/usr/bin/env python
import rospy
import roslaunch
import rosnode
import time
from std_msgs.msg import Bool
from prx_models.msg import Tree
import subprocess
import random
import rosbag

ros_dir="/Users/Gary/pracsys/catkin_ws/"
bag_dir=ros_dir+"bags/"
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
        launch_file=ros_dir+"/src/mujoco_ros/motion_planning/launch/stela.launch"
        # self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])

        # args="planner:=AORRT Stepper:=None sim_clock:=false visualize:=true"
        seed_arg = 'random_seed:=' + str(random.randrange(1,9999999))
        print(seed_arg)
        cli_args = [launch_file,seed_arg,args]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
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