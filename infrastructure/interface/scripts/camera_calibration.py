import rospy
import numpy as np
import cv2 as cv
import glob
import random
from sensor_msgs.msg import Image

class CameraCalibration:

  def __init__(self):
    self.patter_size = (8,6)

  def checkers_calibration(images, display = False):
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    pattern_w = 8
    pattern_h = 6
    patter_size = (pattern_w,pattern_h)
    objp = np.zeros((pattern_h*pattern_w,3), np.float32)
    objp[:,:2] = np.mgrid[0:pattern_w,0:pattern_h].T.reshape(-1,2)
    objp = objp * 33
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    # images = glob.glob('*.jpg')
    size = None
    tot_good_imgs = 0
    for img in images:
      # img = cv.imread(fname)
      gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
      size = gray.shape[::-1]
      # Find the chess board corners
      ret, corners = cv.findChessboardCorners(gray, patter_size, None)
      # If found, add object points, image points (after refining them)
      if ret == True:
        tot_good_imgs += 1
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv.drawChessboardCorners(img, patter_size, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(5)
    print("tot_good_imgs:", tot_good_imgs)
    print("size:", size)
    print("objpoints:", len(objpoints))
    print("imgpoints:", len(imgpoints))
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, size, None, None)
    mean_error = 0
    for i in range(len(objpoints)):
      imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
      error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
      mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )
    print("Matrix:\n",mtx)
    print("Dist:\n", dist)
    cv.destroyAllWindows()

def sample_frames_from_topic(filename, duration_between_frames, display=False):
  
  image = self.br.imgmsg_to_cv2(msg)

def sample_frames_from_video(filename, prob, display=False):
  cap = cv.VideoCapture(filename)
  frames = []
  while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
      break;
    if random.random() < prob:
      frames.append(frame);
      if display:
        cv.imshow('frame', frame)
        wait_ret = cv.waitKey(1)
        if wait_ret == ord('q'):
          break
  if display:
    cap.release()
    cv.destroyAllWindows()
  return frames

if __name__ == '__main__':
  rospy.init_node('camera_calibration', anonymous=True)
  camera_video_param = "~calibration_video"

  if rospy.has_param(camera_video_param):
    filename = rospy.get_param(camera_video_param)
    prob = rospy.get_param("~sampling_prob", 0.1)
    display = rospy.get_param("~display", False)
    frames = sample_frames_from_video(filename, prob, display);
    print(len(frames))
    checkers_calibration(frames)
  elif:
    image_topic = rospy.get_param("~image_topic")
    rospy.Subscriber(image_topic, Image, sample_frames_from_topic)

  else:
    rospy.logfatal("No video!")

