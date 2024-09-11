#!/usr/bin/env python3
import rospy
import numpy as np
import cv2 as cv
import glob
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraCalibration:

  def __init__(self):
    self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    self.images = [];
    self.time_between_frames = rospy.Duration.from_sec(1)
    self.images_to_collect = 100

    rospy.init_node('camera_calibration', anonymous=True)
    camera_video_param = "~calibration_video"
    image_topic_param = "~image_topic"
    pattern_size = rospy.get_param("~pattern_size", [9,6]) # 33mm
    square_length = rospy.get_param("~square_length", 32) # 33mm
    display = rospy.get_param("~display", True)

    print(cv.__version__)
    if rospy.has_param(camera_video_param):
      filename = rospy.get_param(camera_video_param)
      prob = rospy.get_param("~sampling_prob", 0.1)
      self.sample_frames_from_video(filename, prob, display);
      self.checkboard_calibration(pattern_size, square_length)
    elif rospy.has_param(image_topic_param):
      image_topic = rospy.get_param(image_topic_param)
      frame_capture_time = rospy.get_param("~frame_capture_time", 1)
      self.images_to_collect = rospy.get_param("~images_to_collect", 10)
      self.time_between_frames = rospy.Duration.from_sec(frame_capture_time)
      self.next_time = rospy.get_rostime() + self.time_between_frames
      images_subscriber = rospy.Subscriber(image_topic, Image, self.sample_frames_from_topic)

      while self.images_to_collect > 0:
        pass
      images_subscriber.unregister()
      self.checkboard_calibration(pattern_size, square_length, display)

    else:
      rospy.logfatal("No video!")

  def find_checkboard(self, img, patter_size):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, patter_size, None)
    # If found, add object points, image points (after refining them)
    corners2 = None
    if ret == True:
      corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), self.criteria)
    return ret, corners2

  def calibrate(self, objpoints, imgpoints, img_size):
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, img_size, None, None)
    print("Succesful Calibration:\n", ret)
    print("Matrix:\n",mtx)
    print("Dist:\n", dist)
    # print("rvecs:\n", rvecs)
    # print("tvecs:\n", tvecs)
    mean_error = 0
    for i in range(len(objpoints)):
      imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
      error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
      mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )
    cv.destroyAllWindows()

  def checkboard_calibration(self, pattern_size, square_length, display = False):
    # termination criteria
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    pattern_w = pattern_size[0]
    pattern_h = pattern_size[1]
    size_tuple = (pattern_size[0], pattern_size[1])
    objp = np.zeros((pattern_h*pattern_w,3), np.float32)
    objp[:,:2] = np.mgrid[0:pattern_w, 0:pattern_h].T.reshape(-1,2)
    objp = objp * square_length
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    # images = glob.glob('*.jpg')
    img_size = None
    tot_good_imgs = 0
    for img in self.images:
      img_size = img.shape[1::-1]
      ret, corners = self.find_checkboard(img, size_tuple)

      if ret == True:
        tot_good_imgs += 1
        objpoints.append(objp)
        imgpoints.append(corners)
        if display:
          # Draw and display the corners
          cv.drawChessboardCorners(img, size_tuple, corners, ret)
          cv.imshow('img', img)
          cv.waitKey(5)

    print("tot_good_imgs:", tot_good_imgs)
    print("img_size:", img_size)
    print("objpoints:", len(objpoints))
    print("imgpoints:", len(imgpoints))
    if len(objpoints) == 0:
      print("No checkboards found")
      exit(-1)
    # ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, img_size, None, None)
    self.calibrate(objpoints, imgpoints, img_size)


  def sample_frames_from_topic(self, msg):
    now = rospy.get_rostime()
    if now > self.next_time:
      self.images.append(np.array(CvBridge().imgmsg_to_cv2(msg)))
      self.next_time = now + self.time_between_frames
      print(self.images_to_collect)
      self.images_to_collect -= 1


  def sample_frames_from_video(self, filename, prob, display=False):
    cap = cv.VideoCapture(filename)
    while cap.isOpened():
      ret, frame = cap.read()
      if not ret:
        break;
      if random.random() < prob:
        self.images.append(frame);
        if display:
          cv.imshow('frame', frame)
          wait_ret = cv.waitKey(1)
          if wait_ret == ord('q'):
            break
    if display:
      cap.release()
      cv.destroyAllWindows()

if __name__ == '__main__':
  try:
    camera_calibration = CameraCalibration()
  except rospy.ROSInterruptException:
    pass