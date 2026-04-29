#include <chrono>

#include <ros/ros.h>
#include <ros/time.h>

#include <iterator>
#include <memory>
#include <string>
#include <std_msgs/Bool.h>
#include <prx/utilities/general/prx_assert.hpp>
#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

#include <interface/PlannerClock.h>

#include <motion_planning/utils.hpp>

#include <prx/factor_graphs/utilities/dbg_utills.hpp>
#include <utils/dbg_utils.hpp>

namespace motion_planning
{

class goal_checker_t
{
  double _radius;
  gtsam::Pose2 _goal;
  Eigen::Matrix3d _weights;

public:
  goal_checker_t(ros::NodeHandle nh) : _weights(Eigen::Matrix3d::Identity())
  {
    double& radius{ _radius };
    std::vector<double> goal, weights;

    PARAM_SETUP(nh, radius);
    PARAM_SETUP_WITH_DEFAULT(nh, weights, weights);
    GLOBAL_PARAM_BLOCKER(goal);

    prx_assert(goal.size() == 3, "[goal_checkers] Wrong goal size ");

    _goal = gtsam::Pose2(goal[0], goal[1], goal[2]);
    if (weights.size() == 3)
    {
      _weights(0, 0) = weights[0];
      _weights(1, 1) = weights[1];
      _weights(2, 2) = weights[2];
    }
  }

  ~goal_checker_t()
  {
  }

  bool goal_reached(const gtsam::Pose2 x)
  {
    const gtsam::Pose2 x_btw{ gtsam::traits<gtsam::Pose2>::Between(x, _goal) };
    const Eigen::Vector3d v_err{ gtsam::traits<gtsam::Pose2>::Logmap(x_btw) };
    const double err2{ v_err.transpose() * _weights * v_err };
    return std::sqrt(err2) < _radius;
  }

private:
  mutable std::mutex _clock_mutex;

  ros::Publisher _planner_clock_publisher;

  ros::Timer _clock_timer;

  interface::PlannerClock _planner_clock_msg;
};
}  // namespace motion_planning
