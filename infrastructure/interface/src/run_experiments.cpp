#include <ros/ros.h>

#include <ml4kp_bridge/defs.h>
#include <prx_models/mj_mushr.hpp>
#include <prx_models/MushrObservation.h>

#include <interface/mushr_translation.hpp>
#include <interface/msg_translator.hpp>
#include <interface/NodeStatus.h>
#include <interface/node_status.hpp>
#include <utils/rosparams_utils.hpp>
#include <utils/std_utils.hpp>
#include <ackermann_msgs/AckermannDriveStamped.h>

struct collision_t
{
  std::string pose_topic;
  std::string collision_topic;
  ros::Subscriber collision_subscriber;
  ros::Subscriber pose_subscriber;
  bool status, goal_reached;

  double goal_radius;
  std::vector<double> goal;

  collision_t(ros::NodeHandle& nh) : status(false), goal_reached(false)
  {
    PARAM_SETUP(nh, pose_topic);
    PARAM_SETUP(nh, collision_topic);
    PARAM_SETUP(nh, goal);
    PARAM_SETUP(nh, goal_radius);

    pose_subscriber = nh.subscribe(pose_topic, 1, &collision_t::pose_callback, this);
    collision_subscriber = nh.subscribe(collision_topic, 1, &collision_t::callback, this);
  }

  void callback(const std_msgs::BoolConstPtr msg)
  {
    status = msg->data or status;
  }

  void pose_callback(const prx_models::MushrObservationConstPtr msg)
  {
    const double xdiff{ msg->pose.position.x - goal[0] };
    const double ydiff{ msg->pose.position.y - goal[1] };
    const double dist{ std::sqrt(xdiff * xdiff + ydiff * ydiff) };
    if (not goal_reached)
    {
      goal_reached = dist < goal_radius;
      if (goal_reached)
      {
        DEBUG_VARS(msg->pose.position);
      }
    }
  }
};

int main(int argc, char** argv)
{
  using NodeStatus = interface::node_status_t;
  using NodeStatusPtr = std::shared_ptr<NodeStatus>;

  const std::string node_name{ "mushr_stop" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");
  const std::string root{ ros::this_node::getName() };

  // std::string stop_topic{};
  std::string sim_status_topic;
  std::string mppi_status_topic;
  std::string filename_prefix;
  int total_experiments{ 30 };

  // PARAM_SETUP(nh, stop_topic);
  PARAM_SETUP(nh, sim_status_topic);
  PARAM_SETUP(nh, mppi_status_topic);
  PARAM_SETUP(nh, filename_prefix);

  interface::NodeStatus sim_status;
  interface::NodeStatus mppi_status;

  NodeStatusPtr node_status{ interface::node_status_t::create(nh, "/nodes/experiment") };
  // NodeStatusPtr sim_node_status{ interface::node_status_t::create(nh, "/nodes/mj/") };
  // NodeStatusPtr mppi_node_status{ interface::node_status_t::create(nh, "/nodes/mppi/") };

  collision_t collision(nh);

  ros::Publisher sim_status_publisher{ nh.advertise<interface::NodeStatus>(sim_status_topic, 1, true) };
  ros::Publisher mppi_status_publisher{ nh.advertise<interface::NodeStatus>(mppi_status_topic, 1, true) };

  std::string filename{ filename_prefix + "_" + utils::timestamp() + ".txt" };
  std::ofstream ofs(filename, std::ios::trunc);

  ofs << "curr_experiment ";
  ofs << "collision.status ";
  ofs << "timeout ";
  ofs << "collision.goal_reached ";
  ofs << "\n";

  int restart_iters{ 5 };

  int curr_experiment{ 0 };
  int curr_restart_iters{ 0 };
  ros::Duration max_duration(120.0);

  bool timeout{ false };
  bool running_pub{ true };
  ros::Time start{ ros::Time::now() };

  node_status->status(interface::NodeStatus::READY);

  while (ros::ok())
  {
    auto curr = ros::Time::now() - start;
    if (max_duration <= curr)
    {
      PRINT_MSG("Timeout!");
      timeout = true;
    }
    else if (collision.goal_reached)
    {
      PRINT_MSG("Goal Reached!");
      node_status->status(interface::NodeStatus::RESTART);
    }

    if (curr_experiment >= total_experiments)
    {
      PRINT_MSG("Finished");
      sim_status.status = interface::NodeStatus::FINISH;
      mppi_status.status = interface::NodeStatus::FINISH;
      node_status->status(interface::NodeStatus::FINISH);
      sim_status_publisher.publish(sim_status);
      mppi_status_publisher.publish(mppi_status);
    }
    else if (node_status->status() == interface::NodeStatus::FINISH)
    {
      sim_status_publisher.publish(sim_status);
      mppi_status_publisher.publish(mppi_status);
      if (curr_restart_iters < restart_iters)
      {
        ros::spinOnce();
        ros::Rate(1.0).sleep();
        curr_restart_iters++;
      }
      else
      {
        ros::shutdown();
      }
    }
    else if (node_status->status() == interface::NodeStatus::RUNNING)
    {
      // PRINT_MSG("Running");

      if (collision.status or timeout)
      {
        node_status->status(interface::NodeStatus::RESTART);
      }
      // if (running_pub)
      // {
      sim_status.status = interface::NodeStatus::RUNNING;
      mppi_status.status = interface::NodeStatus::RUNNING;
      sim_status_publisher.publish(sim_status);
      mppi_status_publisher.publish(mppi_status);
      running_pub = false;
      // }
    }
    else if (node_status->status() == interface::NodeStatus::RESTART)
    {
      DEBUG_VARS(curr_experiment);
      PRINT_MSG("Restart");
      sim_status.status = interface::NodeStatus::RESTART;
      mppi_status.status = interface::NodeStatus::RESTART;
      sim_status_publisher.publish(sim_status);
      mppi_status_publisher.publish(mppi_status);
      if (curr_restart_iters < restart_iters)
      {
        ros::spinOnce();
        ros::Rate(1.0).sleep();
        curr_restart_iters++;
      }
      else
      {
        ofs << curr_experiment << " ";
        ofs << collision.status << " ";
        ofs << timeout << " ";
        ofs << collision.goal_reached << " ";
        ofs << curr.toSec() << " ";
        ofs << "\n";

        curr_experiment++;
        curr_restart_iters = 0;
        node_status->status(interface::NodeStatus::RUNNING);
        timeout = false;
        start = ros::Time::now();
        collision.goal_reached = false;
        collision.status = false;
        running_pub = true;
      }
    }

    ros::spinOnce();
  }

  return 0;
}
