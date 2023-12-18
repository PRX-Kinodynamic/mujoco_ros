import os
import sys

import rosbag
from std_msgs.msg import Int32, String
# from ackermann_msgs.msg import AckermannDriveStamped
# from utilities.msg import omnirobot
# from utilities.msg import stamped_markers
from interface.msg import SensorDataStamped
from prx_models.msg import MushrPlan

from argparse import ArgumentParser

sep = " "

def header_to_txt(msg, out):
    global sep
    if msg._type == "std_msgs/Header":
        return out + str(msg.seq) + sep + str(msg.stamp.secs) + sep + str(msg.stamp.nsecs)

def sensor_data_stamped_to_txt(msg, out):
    global sep
    if msg._type == "mujoco_ros/SensorDataStamped" or msg._type == "interface/SensorDataStamped":
        out = header_to_txt(msg.header, out) + sep
        for d in msg.raw_sensor_data:
            out += str(d.data) + sep 
        return out
def mushr_plan_to_txt(msg, out):
    global sep
    if msg._type == "prx_models/MushrPlan":
        out = header_to_txt(msg.header, out) + sep
        out += "controls" + sep
        for d in msg.controls:
            out += mushr_control_to_txt(d, out) + sep
        out += "durations" + sep
        for d in msg.durations:
            out += mushr_duration_to_txt(d, out) + sep
        return out

def mushr_duration_to_txt(msg, out):
    global sep
    if msg._type == "std_msgs/Duration":
        return str(msg.data.to_sec()) + sep

def mushr_control_to_txt(msg, out):
    global sep
    if msg._type == "prx_models/MushrControl":
        out = str(msg.steering_angle.data) + sep 
        out += str(msg.velocity.data) + sep 
        return out

def marker_to_txt(msg, out):
    global sep
    if msg._type == "utilities/marker":    
        out += str(msg.id) + sep 
        out += str(msg.x1) + sep + str(msg.y1) + sep 
        out += str(msg.x2) + sep + str(msg.y2) + sep 
        out += str(msg.x3) + sep + str(msg.y3) + sep 
        out += str(msg.x4) + sep + str(msg.y4)
        return out

def stamped_markers_to_txt(msg, out):
    global sep
    if msg._type == "utilities/stamped_markers":
        out_m = ""
        out = header_to_txt(msg.header, out) + sep
        for marker in msg.markers:
            if (out_m != ""):
                out_m += "\n"
            out_m += marker_to_txt(marker, out)
        return out_m

def omnirobot_to_txt(msg, out):
    global sep
    if msg._type == "utilities/omnirobot":
        out = header_to_txt(msg.header, out) + sep
        out += str(msg.m1) + sep
        out += str(msg.m2) + sep
        out += str(msg.m3) + sep
        out += str(msg.m4)
        return out

def ackermann_drive_to_txt(msg, out):
    global sep
    if msg._type == "ackermann_msgs/AckermannDrive":
        out += str(msg.steering_angle) + sep
        out += str(msg.steering_angle_velocity) + sep
        out += str(msg.speed) + sep
        out += str(msg.acceleration) + sep
        out += str(msg.jerk)
        return out

def ackermann_drive_stamped_to_txt(msg, out):
    global sep
    if msg._type == "ackermann_msgs/AckermannDriveStamped":
        out = header_to_txt(msg.header, out) + sep
        out = ackermann_drive_to_txt(msg.drive, out)
        return out
        
if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-b", "--bag", dest="bag",
                    help="Rosbag to read from", metavar="bag")
    parser.add_argument("-o", "--out", dest="out_filename", default="",
                    help="Out file")

    args = parser.parse_args()
    bag_name = args.bag;
    out_filename = args.out_filename
    if(out_filename == ""):
        path = os.path.dirname(os.path.abspath(os.path.abspath(bag_name)));
        out_filename = path + "/" + os.path.basename(bag_name).replace("bag", "txt")

    print(args.bag)
    print(out_filename)

    bag = rosbag.Bag(bag_name, 'r')
    outfile = open(out_filename, 'w')

    to_txt_fs = [omnirobot_to_txt, header_to_txt, marker_to_txt, stamped_markers_to_txt, ackermann_drive_to_txt, ackermann_drive_stamped_to_txt,sensor_data_stamped_to_txt,mushr_plan_to_txt,mushr_duration_to_txt, mushr_control_to_txt]

    try:
        for topic, msg, t in bag.read_messages():
            # out = 
            for f in to_txt_fs:
                out = str(topic) + sep + str(msg._type) + sep + str(t.secs) + sep + str(t.nsecs) + sep;
                line = f(msg, out)
                if line != None:
                    outfile.write( line + "\n" )

    finally:
        bag.close()
        outfile.close()