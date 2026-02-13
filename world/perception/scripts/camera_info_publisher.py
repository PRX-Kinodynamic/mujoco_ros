import glob
import traceback

import os
import cv2
import rospy
import tf
import tf2_ros
import geometry_msgs
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
import numpy as np

# Mostly adapted from:
#    https://github.com/ros-perception/image_pipeline/blob/rolling/camera_calibration/src/camera_calibration/calibrator.py
class CameraInfoNode(object):

    def __init__(self):
        # self.focal_length = rospy.get_param("~focal_length", "")
        # self.optical_center = rospy.get_param("~optical_center", "")

        self.width = rospy.get_param("~width", "")
        self.height = rospy.get_param("~height", "")

        fx = rospy.get_param("~fx", "") # float(self.focal_length[0])
        fy = rospy.get_param("~fy", "") #float(self.focal_length[1])
        
        cx = rospy.get_param("~cx", "") # float(self.optical_center[0])
        cy = rospy.get_param("~cy", "") # float(self.optical_center[1])

        self.intrinsic = np.array([[fx, 0, cx],\
                                   [0, fy, cy],\
                                   [0,  0,  1]])
        if rospy.has_param('~distortion'):
            distortion = rospy.get_param('~distortion')
            self.distortion = np.array(distortion, np.float32)
        else:
            self.distortion = np.array([]);

        if rospy.has_param('~rotation'):
            rotation = rospy.get_param('~rotation')
            print("rotation not implemented yet")
        else:
            # Identity for monocular
            self.rotation = np.array([[1, 0, 0],\
                                      [0, 1, 0],\
                                      [0, 0, 1]])
        if rospy.has_param('~projection'):
            projection = rospy.get_param('~projection')
            print("projection not implemented yet")
        else:
            # Identity for monocular
            self.projection = np.array([[fx, 0, cx, 0],\
                                        [0, fy, cy, 0],\
                                        [0,  0,  1, 0]])

        self.distortion_model = "plumb_bob"
        if rospy.has_param('cam_model'):
            self.distortion_model = rospy.get_param('~cam_model')
        
        self.camera_name = rospy.get_param('~camera_name') 
        self.camera_name_topic = "/" + self.camera_name  + "/info"
        self.publisher_camera_info = rospy.Publisher(self.camera_name_topic, CameraInfo, queue_size=1, latch=True)

        self.msg_camera_info = self.create_msg();

        self.publisher_camera_info.publish(self.msg_camera_info)
        

        if rospy.has_param('~camera_extrinsics'):
            self.camera_extrinsics = rospy.get_param("~camera_extrinsics")
            self.extrinsics_to_tf()


        # print(f"params: {rospy.has_param('~pose/publish_static')}")
        # print(f"publish_static: {rospy.has_param('~pose/publish_static')}")
        self.publish_static = False;
        if rospy.has_param('~pose/publish_static'):
            self.publish_static = rospy.get_param("~pose/publish_static")
            # print(f"publish_static: {publish_static}")
            if self.publish_static:
                position = rospy.get_param("~pose/position")
                quaternion = rospy.get_param("~pose/quaternion")
                position = np.array(position, np.float32)
                quaternion = np.array(quaternion, np.float32)
                self.pose_to_tf(position, quaternion)

        rospy.Timer(rospy.Duration(1.0), self.timer_callback)

    def timer_callback(self, event=None):
        self.publisher_camera_info.publish(self.msg_camera_info)
        if self.publish_static:
            self.broadcaster.sendTransform(self.static_transformStamped)

    def extrinsics_to_matrix(self):
        extrinsic = np.zeros((4,4))
        for i in range(3):
          for j in range(4):
            extrinsic[i, j] = float(self.camera_extrinsics[4 * i + j]);
        extrinsic[3, 3] = 1.0
        return extrinsic


    def pose_to_tf(self, position, quat):
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = geometry_msgs.msg.TransformStamped()

        self.static_transformStamped.header.stamp = rospy.Time.now()
        self.static_transformStamped.header.frame_id = "world"
        self.static_transformStamped.child_frame_id = self.camera_name 

        self.static_transformStamped.transform.translation.x = position[0]
        self.static_transformStamped.transform.translation.y = position[1]
        self.static_transformStamped.transform.translation.z = position[2]

        # quat = tf.transformations.quaternion_from_matrix(mat)
        self.static_transformStamped.transform.rotation.w = quat[0]
        self.static_transformStamped.transform.rotation.x = quat[1]
        self.static_transformStamped.transform.rotation.y = quat[2]
        self.static_transformStamped.transform.rotation.z = quat[3]

        # print(static_transformStamped)

    def extrinsics_to_tf(self):

        mat = self.extrinsics_to_matrix()
        # print(f"extrinsic: {mat}")
        t = mat[0:3,3]
        R = mat[0:3,0:3]
        # print(f"t {t}")
        # print(f"R {R}")

        mat_inv = np.zeros_like(mat)
        mat_inv[0:3,0:3] = R.T
        # print(t)
        # print(t.reshape((3,1)))
        # print(R.T.shape)
        # print(t.reshape((3,1)).shape)
        # print(R.T @ t.reshape((3,1)))
        mat_inv[0:3,3] = (R.T @ t.reshape((3,1))).T
        mat_inv[3, 3] = 1.0

        tinv = mat_inv[0:3,3]
        Rinv = mat_inv[0:3,0:3]

        # print(f"t {tinv}")
        # print(f"R {Rinv}")

        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = self.camera_name 

        static_transformStamped.transform.translation.x = t[0]
        static_transformStamped.transform.translation.y = t[1]
        static_transformStamped.transform.translation.z = t[2]

        # print(f"mat: {mat}")
        # print(f"R: {R}")
        # quat = tf.transformations.quaternion_from_matrix(mat_inv)
        quat = tf.transformations.quaternion_from_matrix(mat)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(static_transformStamped)

    def create_msg(self):
        msg = CameraInfo()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.camera_name

        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = self.distortion_model

        msg.D = np.ravel(self.distortion).copy().tolist()
        msg.K = np.ravel(self.intrinsic).copy().tolist()
        msg.R = np.ravel(self.rotation).copy().tolist()
        msg.P = np.ravel(self.projection).copy().tolist()

        # print(msg)
        return msg


if __name__ == '__main__':
    rospy.init_node("CameraInfoPublisher")
    ros_node = CameraInfoNode()
    rospy.spin()