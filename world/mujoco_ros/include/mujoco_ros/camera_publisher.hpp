#pragma once
#include "defs.h"
#include <sstream>
#include <cstdio>
#include <mutex>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

#include <mujoco_ros/simulator.hpp>
namespace mj_ros
{
class simulator_t;

class camera_rgb_publisher_t
{
public:
  camera_rgb_publisher_t(ros::NodeHandle& nh, SimulatorPtr sim, const std::string camera_name, double frequency = 30,
                         const std::string topic_name = ros::this_node::getNamespace() + "/camera/rgb")
    : _sim(sim)
    , _rate(frequency)
    , _root(ros::this_node::getNamespace())
    , _topic_name(topic_name)
    , _publisher(nh.advertise<sensor_msgs::Image>(_topic_name, 1000))
    , _cam_name(camera_name)
    , _header()
  {
    mjv_defaultOption(&_mj_option);
    mjv_defaultScene(&_mj_scene);
    mjr_defaultContext(&_mj_context);

    _header.seq = 0;
    _header.stamp = ros::Time::now();
    _header.frame_id = _topic_name;

    if (!glfwInit())
      ROS_ERROR("Error in initializing GLFW.");

    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
// Macos works better if Null as 4th arg, this seems to be OS dependent
#ifdef __APPLE__
    _glfw_window = glfwCreateWindow(1280, 720, _cam_name.c_str(), NULL, NULL);
#else
    _glfw_window = glfwCreateWindow(1280, 720, _cam_name.c_str(), glfwGetPrimaryMonitor(), NULL);
#endif

    if (!_glfw_window)
      ROS_ERROR("Error in creating GLFW window.");

    glfwMakeContextCurrent(_glfw_window);
    // The next three calls need to be AFTER glfwMakeContextCurrent
    mjv_defaultCamera(&_mj_camera);
    mjv_makeScene(_sim->m, &_mj_scene, 1000);
    mjr_makeContext(_sim->m, &_mj_context, mjFONTSCALE_150);

    for (int i = 0; i < _sim->m->ncam; ++i)
    {
      const std::string name_i{ _sim->m->names + _sim->m->name_camadr[i] };
      // std::cout << "name_i: " << name_i << std::endl;
      if (_cam_name == name_i)
      {
        _mj_viewport.left = 0;
        _mj_viewport.bottom = 0;
        // _mj_viewport.width = _sim->m->cam_resolution[i];
        // _mj_viewport.height = _sim->m->cam_resolution[i + 1];
        _mj_viewport.width = 1280;
        _mj_viewport.height = 720;
        _mj_camera.type = mjtCamera::mjCAMERA_FIXED;                                   // camera type (mjtCamera)
        _mj_camera.fixedcamid = mj_name2id(_sim->m, mjOBJ_CAMERA, _cam_name.c_str());  // fixed camera id
        _cv_pixels = cv::Mat(_mj_viewport.height, _mj_viewport.width, CV_8UC3);
        // _mj_camera.trackbodyid = _sim->m->cam_targetbodyid[i];                   // body id to track
      }
    }
  }

  void get_frame()
  {
    glfwMakeContextCurrent(_glfw_window);

    mjv_updateScene(_sim->m, _sim->d, &_mj_option, NULL, &_mj_camera, mjCAT_ALL, &_mj_scene);
    mjr_render(_mj_viewport, &_mj_scene, &_mj_context);
    glfwSwapBuffers(_glfw_window);

    mjr_readPixels(_cv_pixels.data, nullptr, _mj_viewport, &_mj_context);
    cvtColor(_cv_pixels, _cv_pixels, cv::COLOR_RGB2BGR);
    cv::flip(_cv_pixels, _cv_pixels, 0);
    _header.seq++;
    _header.stamp = ros::Time::now();
    _message = cv_bridge::CvImage(_header, "bgr8", _cv_pixels).toImageMsg();
  }

  void run()
  {
    ros::Duration(1).sleep();
    while (ros::ok())
    {
      get_frame();
      _publisher.publish(_message);
      _rate.sleep();
    }
  }

private:
  const std::string _root;
  const std::string _topic_name;
  const std::string _cam_name;

  mjrRect _mj_viewport;
  mjvCamera _mj_camera;
  mjvOption _mj_option;
  mjvScene _mj_scene;
  mjrContext _mj_context;

  ros::Rate _rate;

  SimulatorPtr _sim;
  ros::Publisher _publisher;

  std_msgs::Header _header;
  sensor_msgs::ImagePtr _message;

  cv::Mat _cv_pixels;

  GLFWwindow* _glfw_window;
};
}  // namespace mj_ros
