#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <ml4kp_bridge/plan_bridge.hpp>
#include <std_srvs/Empty.h>
namespace ml4kp_bridge
{
class plan_from_file_t : public nodelet::Nodelet
{
public:
  plan_from_file_t()
  {
  }

protected:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ getPrivateNodeHandle() };
    private_nh.getParam("publisher_topic", _publisher_topic);
    private_nh.getParam("plan_file", _plan_file);

    private_nh.getParam("control_dimension", _dim);
    private_nh.getParam("control_topology", _topology);

    _ctrl.resize(_dim);
    for (int i = 0; i < _dim; ++i)
    {
      _address.push_back(&_ctrl[i]);
    }
    _control_space = std::make_shared<prx::space_t>(_topology, _address, "plan_control_space");
    _plan = std::make_shared<prx::plan_t>(_control_space.get());
    _plan->from_file(_plan_file);
    copy(_plan_msg, _plan);

    // const std::string publisher_name{ ros::this_node::getNamespace() + "/plan" };
    const std::string service_name{ ros::this_node::getNamespace() + "/publish_plan" };

    _timer = private_nh.createTimer(ros::Duration(1.0), &plan_from_file_t::timer_callback, this, true, false);
    _publisher = private_nh.advertise<ml4kp_bridge::PlanStamped>(_publisher_topic, 1, true);

    _service = private_nh.advertiseService(service_name, &plan_from_file_t::service_callback, this);
  }
  bool service_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    _timer.start();
    return true;
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    _publisher.publish(_plan_msg);
    _timer.stop();
  }

  std::string _plan_file;
  std::string _publisher_topic;

  int _dim;
  std::string _topology;
  Eigen::VectorXd _ctrl;
  std::vector<double*> _address;
  std::shared_ptr<prx::plan_t> _plan;
  std::shared_ptr<prx::space_t> _control_space;

  ml4kp_bridge::PlanStamped _plan_msg;

  ros::Timer _timer;
  ros::Publisher _publisher;
  ros::ServiceServer _service;
};
}  // namespace ml4kp_bridge
PLUGINLIB_EXPORT_CLASS(ml4kp_bridge::plan_from_file_t, nodelet::Nodelet);
