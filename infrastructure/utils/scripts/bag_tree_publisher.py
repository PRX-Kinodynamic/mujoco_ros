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

ros_dir = "/common/home/st1122/Projects/ros_workspace/"
bag_dir = ros_dir + "data/bags/"
# bag_file = "ltv_sde_forest_aorrt_1m_000.bag"
# bag_file_path = bag_dir + bag_file

publishing_topic = "/stela/sbmp/sln_tree"
tree_topic = "/scate/sbmp/sln_tree"
node_name = "bag_tree_publisher"


class launcher:
    def __init__(self):
        rospy.init_node(node_name, anonymous=False)
        # private_param =
        # self.bagfile = bag_dir + bag_file
        self.bagfile = rospy.get_param("~bagfile")
        print("Using bagfile:", self.bagfile)
        self.tree_pub = rospy.Publisher(tree_topic, Tree, queue_size=1, latch=True)

        time.sleep(5)
        self.bag_publisher()
        rospy.spin()

    def bag_publisher(
        self,
    ):
        bag = rosbag.Bag(self.bagfile)
        print("Publishing tree from bagfile:", self.bagfile)
        print("Publishing from topic:", publishing_topic)
        print("Publishing to topic:", tree_topic)
        for topic, msg, t in bag.read_messages(topics=[publishing_topic]):
            self.tree_pub.publish(msg)
            # print(msg)
        bag.close()


if __name__ == "__main__":
    # parser = argparse.ArgumentParser(
    #     prog="bag_tree_publisher",
    #     description="What the program does",
    #     epilog="Text at the bottom of help",
    # )
    # parser.add_argument("bagfile")  # positional argument
    # The ArgumentParser.parse_args() method runs the parser and places the extracted data in a argparse.Namespace object:

    # args = parser.parse_args()
    # print(args.filename, args.count, args.verbose)
    try:
        launcher = launcher()
    except rospy.ROSInterruptException:
        pass
