#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <tf2_ros/transform_listener.h>

#include <utils/dbg_utils.hpp>
#include <utils/std_utils.hpp>
#include <utils/execution_status.hpp>
#include <utils/rosparams_utils.hpp>
#include <ml4kp_bridge/defs.h>
#include <prx_models/mushr.hpp>
#include <interface/node_status.hpp>

struct nodes_checker_t
{
  ros::Timer _timer;
  std::vector<std::shared_ptr<interface::node_status_t>> _all_ns;

  nodes_checker_t(ros::NodeHandle& nh)
  {
    XmlRpc::XmlRpcValue nodes_ids;
    PARAM_SETUP(nh, nodes_ids);

    for (int i = 0; i < nodes_ids.size(); ++i)
    {
      const std::string id(nodes_ids[i]);
      _all_ns.push_back(interface::node_status_t::create(nh, id, false));
    }

    _timer = nh.createTimer(ros::Duration(1.0), &nodes_checker_t::timer_callback, this);
  }

  void timer_callback(const ros::TimerEvent& event)
  {
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "NodeStatusTimeout");
  ros::NodeHandle nh("~");

  return 0;
}