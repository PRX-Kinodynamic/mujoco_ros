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
import argparse

ros_dir="/Users/Gary/pracsys/catkin_ws/"
bag_dir=ros_dir+"bags/"
bag_file="simple_obstacle_aorrt_003.bag"
# bag_file="test_cost_aorrt_v2.bag"


tree_topic="/stela/sbmp/sln_tree"
node_name="bag_tree_publisher"

class launcher: 
    def bag_publisher(self):
        # bag = rosbag.Bag(bag_dir + bag_file)
        bag = rosbag.Bag(self.bagfile)
        for topic, msg, t in bag.read_messages(topics=[tree_topic]):
            self.tree_pub.publish(msg)
            # print(msg)
        bag.close()

    def __init__(self):
        rospy.init_node(node_name, anonymous=False)
        # private_param = 
        self.bagfile = rospy.get_param('~bagfile')
        print("Using bagfile:", self.bagfile)
        self.tree_pub = rospy.Publisher(tree_topic, Tree, queue_size=1,latch=True)
        
        time.sleep(5)
        self.bag_publisher();
        rospy.spin()

if __name__ == '__main__':
    # parser = argparse.ArgumentParser(
    #                 prog='bag_tree_publisher',
    #                 description='What the program does',
    #                 epilog='Text at the bottom of help')
    # parser.add_argument('bagfile')           # positional argument
    # # The ArgumentParser.parse_args() method runs the parser and places the extracted data in a argparse.Namespace object:

    # args = parser.parse_args()
    # print(args.filename, args.count, args.verbose)
    try:
        launcher = launcher()
    except rospy.ROSInterruptException:
        pass