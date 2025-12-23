import glob
import traceback

import os
import cv2
import math
import rospy
from interface.msg import NodeStatus
import numpy as np

class NodeStatusHelper:
    def __init__(self, node_id):
        self.msg = NodeStatus()
        self.msg.status = NodeStatus.PREPARING;
        change_topic =  node_id + "/status/change";
        current_topic =  node_id + "/status/current";
        self.status_subscriber = rospy.Subscriber(change_topic, NodeStatus, self.callback)
        self.status_publisher = rospy.Publisher(current_topic, NodeStatus, queue_size=1, latch=True)

        self.timer = rospy.Timer(rospy.Duration(1.0), self.update)

        self.status_publisher.publish(self.msg);
    
    @property
    def status(self):
        return self.msg.status

    @status.setter
    def status(self, new_status):
        self.status_change(new_status)

    def callback(self, msg):
        # print(f"NodeStatus {msg}")
        self.status_change(msg.status)

    def update(self, event):
        if self.msg.status == NodeStatus.FINISH:
            rospy.signal_shutdown("Status FINISH received");
        self.status_publisher.publish(self.msg);

    def status_change(self, new_status):
        # current = self.msg.status 
        # next_status = current
        self.msg.status = new_status;
        self.status_publisher.publish(self.msg);
