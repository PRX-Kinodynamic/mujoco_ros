import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class ImageToFile(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        frequency = float(rospy.get_param("~frequency", 10))
        self.img = None

        self.timer = rospy.Timer(rospy.Duration(1.0/frequency), self.timer_callback)
        # self.loop_rate = rospy.Rate(1)

        # Publishers
        # self.pub = rospy.Publisher('imagetimer', Image,queue_size=10)

        camera_topic = rospy.get_param("~camera_topic", "")

        self.path = rospy.get_param("~output_dir");
        self.idx = 0;

        # Subscribers
        self.subs = rospy.Subscriber(camera_topic, Image, self.callback)



    # def back(*args):

    def callback(self, msg):
        # rospy.loginfo('Image received...')
        self.img = self.br.imgmsg_to_cv2(msg)

    def timer_callback(self, event):
        if (self.img is not None):
            cv2.imshow('image', self.img)
            c = cv2.waitKey(10)
            if (c == ord('s')):
                self.save_img();

    def save_img(self):
        img_name = f"{self.path}/img_{self.idx:04}.png" 
        # print(f"Saving img: {img_name}")
        cv2.imwrite(img_name, self.img)
        self.idx += 1 

            
if __name__ == '__main__':
    rospy.init_node("ImageTopicToFile", anonymous=True)
    my_node = ImageToFile()

    # cv2.namedWindow("Frame")
    # cv2.createButton("Back",save_img,my_node,cv2.QT_PUSH_BUTTON,1)
    rospy.spin();