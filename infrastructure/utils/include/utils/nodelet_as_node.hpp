#pragma once

#include <ros/ros.h>
#include <regex>

namespace utils
{

// Class to "convert" a nodelet into a node. Useful for debuging
class nodelet_as_node_t
{
public:
  nodelet_as_node_t() : _priv_nh("~")
  {
  }
  virtual ~nodelet_as_node_t(){};

  void init()
  {
    onInit();
  }

  ros::NodeHandle& getPrivateNodeHandle()
  {
    return _priv_nh;
  }

private:
  virtual void onInit() = 0;

  ros::NodeHandle _priv_nh;
};

}  // namespace utils
