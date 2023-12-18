import os
import sys
import cv2
import rosbag

from std_msgs.msg import Int32, String
from ackermann_msgs.msg import AckermannDriveStamped

from perception.msg import stamped_markers
from argparse import ArgumentParser
from cv_bridge import CvBridge


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-b", "--bag", dest="bag", help="Rosbag to read from", metavar="bag")
    parser.add_argument("-o", "--out", dest="out_dir", default="", help="Out directory")
    parser.add_argument("-f", "--file", dest="file_prefix", default="", help="Prefix of the file")
    parser.add_argument("-t", "--topic", dest="image_topic", default="",help="Topic")

    args = parser.parse_args()
    bag_name = args.bag;

    bag = rosbag.Bag(bag_name, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2.imwrite(os.path.join(args.out_dir, args.file_prefix + "_%06i.png" % count), cv_img)
        count += 1

    bag.close()