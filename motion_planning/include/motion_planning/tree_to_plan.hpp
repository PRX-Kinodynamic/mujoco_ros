#include <unordered_set>
#include <prx_models/Graph.h>
#include <prx_models/Tree.h>
#include <visualization_msgs/Marker.h>

#include <utils/dbg_utils.hpp>
#include <utils/rosparams_utils.hpp>
#include <motion_planning/motion_planning_types.hpp>
#include <interface/ControlsPlot.h>

namespace motion_planning
{

template <class Base>
class tree_to_plan_t : public Base
{
  using Derived = tree_to_plan_t<Base>;

public:
  tree_to_plan_t()
  {
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::string tree_topic_name{};

    PARAM_SETUP(private_nh, tree_topic_name);

    const std::string plan_topic_name{ tree_topic_name + "/controls" };

    // publishers
    _plan_publisher = private_nh.advertise<interface::ControlsPlot>(plan_topic_name, 1, true);

    // subscribers
    _tree_subscriber = private_nh.subscribe(tree_topic_name, 1, &Derived::get_graph, this);
  }

protected:
  void get_graph(prx_models::TreePtr msg)
  {
    interface::ControlsPlot plan;

    prx_models::Node& node{ motion_planning::get_root(*msg) };

    ros::Duration total_duration{ 0.0 };
    while (node.children.size() > 0)
    {
      node = motion_planning::get_node(*msg, node.children[0]);
      const prx_models::Edge& edge{ motion_planning::get_edge(*msg, node.parent_edge) };
      for (auto& step : edge.plan.steps)
      {
        plan.stamps.push_back(total_duration.toSec());
        plan.controls.push_back(step.control);
        total_duration += step.duration.data;
        // plan.steps.push_back(step);
      }
    }

    _plan_publisher.publish(plan);
  }

  // Topic names

  // Subscribers
  ros::Subscriber _tree_subscriber;

  // Publishers
  ros::Publisher _plan_publisher;

  // Timers
  ros::Timer _tree_timer;
};
}  // namespace motion_planning