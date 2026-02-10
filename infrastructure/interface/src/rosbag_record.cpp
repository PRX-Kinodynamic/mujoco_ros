#include <atomic>
#include <chrono>
#include <iterator>
#include <memory>
#include <prx/utilities/general/prx_assert.hpp>
#include <thread>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <rosbag/bag.h>

#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <ml4kp_bridge/TrajectoryStamped.h>
#include <ml4kp_bridge/PlanStamped.h>
#include <ml4kp_bridge/defs.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

#include <cv_bridge/cv_bridge.h>

#include <XmlRpcValue.h>

#include <tf2_msgs/TFMessage.h>

#include <tuple>
#include <utils/std_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <utils/rosparams_utils.hpp>
#include <utils/execution_status.hpp>
// #include <interface/defs.hpp>
#include <interface/rosbag_record.hpp>
#include <interface/StampedMarkers.h>
#include <visualization_msgs/MarkerArray.h>
#include <interface/node_status.hpp>
#include "interface/ControlsPlot.h"
#include "interface/SensorDataStamped.h"
#include "ml4kp_bridge/StelaTrajectory.h"
#include "prx_models/MushrControl.h"
#include "prx_models/MushrObservation.h"
#include "prx_models/MushrPlan.h"
#include <interface/NodeStatus.h>
#include <interface/PlannerClock.h>
#include <interface/StelaStatus.h>

#include <prx_models/Tree.h>

struct bag_writer_t
{
  std::atomic<bool> stop = false;
  std::string rosbag_directory;
  std::string rosbag_prefix;

  interface::queues_t<ackermann_msgs::AckermannDriveStamped> ackermann_drive_stamped_queue;

  interface::queues_t<std_msgs::String> string_queue;
  interface::queues_t<std_msgs::Int32> int32_queue;
  interface::queues_t<std_msgs::Float64> float64_queue;
  interface::queues_t<std_msgs::Bool> bool_queue;

  interface::queues_t<geometry_msgs::TwistStamped> twist_stamped_queue;
  interface::queues_t<geometry_msgs::Pose2D> pose2d_queue;
  interface::queues_t<geometry_msgs::PoseStamped> pose_stamped_queue;

  interface::queues_t<sensor_msgs::Image> image_queue;
  interface::queues_t<sensor_msgs::CameraInfo> cam_info_queue;
  interface::queues_t<sensor_msgs::Imu> imu_queue;

  interface::queues_t<ml4kp_bridge::Plan> plan_queue;
  interface::queues_t<ml4kp_bridge::PlanStamped> plan_st_queue;
  interface::queues_t<ml4kp_bridge::Trajectory> traj_queue;
  interface::queues_t<ml4kp_bridge::TrajectoryStamped> traj_st_queue;
  interface::queues_t<ml4kp_bridge::SpacePoint> spoint_queue;
  interface::queues_t<ml4kp_bridge::SpacePointStamped> spoint_st_queue;
  interface::queues_t<ml4kp_bridge::StelaTrajectory> stela_traj_queue;

  interface::queues_t<prx_models::Tree> prx_tree_queue;
  interface::queues_t<prx_models::MushrPlan> prx_mushr_plan_queue;
  interface::queues_t<prx_models::MushrControl> prx_mushr_ctrl_queue;
  interface::queues_t<prx_models::MushrObservation> prx_mushr_obs_queue;

  interface::queues_t<tf2_msgs::TFMessage> tf_queue;

  interface::queues_t<interface::StampedMarkers> stamped_markers_queue;
  interface::queues_t<interface::NodeStatus> node_status_queue;
  interface::queues_t<interface::PlannerClock> planner_clock_queue;
  interface::queues_t<interface::StelaStatus> stela_status_queue;
  interface::queues_t<interface::ControlsPlot> ctrls_plot_queue;
  interface::queues_t<interface::SensorDataStamped> sensor_data_stamped_queue;

  interface::queues_t<visualization_msgs::Marker> marker_queue;
  interface::queues_t<visualization_msgs::MarkerArray> marker_array_queue;

  rosbag::Bag bag;
  XmlRpc::XmlRpcValue topics;

  ros::Timer _timer;

  int _bag_num;
  bool _first;

  std::shared_ptr<interface::node_status_t> _node_status;

  bag_writer_t(ros::NodeHandle& nh) : rosbag_directory(""), rosbag_prefix(""), _bag_num(0), _first(true)
  {
    PARAM_SETUP(nh, topics);
    PARAM_SETUP(nh, rosbag_directory);
    PARAM_SETUP_WITH_DEFAULT(nh, rosbag_prefix, rosbag_prefix);

    _node_status = interface::node_status_t::create(nh);
    register_topics(nh);

    _timer = nh.createTimer(ros::Duration(1.0 / 30.0), &bag_writer_t::timer_callback, this);
    init_bag();

    _node_status->status(interface::NodeStatus::READY);
  }

  void init_bag()
  {
    const std::string bn{ prx::utilities::convert_to<std::string>(_bag_num) };
    interface::init_bag(&bag, rosbag_directory, rosbag_prefix + "_" + bn);
    _bag_num++;
  }

  void register_topics(ros::NodeHandle& nh)
  {
    for (int i = 0; i < topics.size(); ++i)
    {
      // for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = topics.begin(); it != topics.end(); ++it)
      // PRX_DEBUG_VARS(topics[i]);
      auto topic_i = topics[i];
      const std::string topic_name(topic_i["name"]);
      const std::string topic_type(topic_i["type"]);

      // std::cout << "topic_name: " << topic_name << std::endl;
      // ROS_INFO_STREAM("Topic: " << topic_name << " - " << topic_type);
      bool registred{ false };
      registred |= bool_queue.register_topic(topic_name, topic_type, "std_msgs::Bool", nh);
      registred |= float64_queue.register_topic(topic_name, topic_type, "std_msgs::Float64", nh);
      registred |= int32_queue.register_topic(topic_name, topic_type, "std_msgs::int32", nh);
      registred |= string_queue.register_topic(topic_name, topic_type, "std_msgs::string", nh);

      registred |= ackermann_drive_stamped_queue.register_topic(topic_name, topic_type,
                                                                "ackermann_msgs::AckermannDriveStamped", nh);

      registred |= image_queue.register_topic(topic_name, topic_type, "sensor_msgs::Image", nh);
      registred |= imu_queue.register_topic(topic_name, topic_type, "sensor_msgs::Imu", nh);
      registred |= cam_info_queue.register_topic(topic_name, topic_type, "sensor_msgs::CameraInfo", nh);

      registred |= traj_queue.register_topic(topic_name, topic_type, "ml4kp_bridge::Trajectory", nh);
      registred |= traj_st_queue.register_topic(topic_name, topic_type, "ml4kp_bridge::TrajectoryStamped", nh);
      registred |= plan_queue.register_topic(topic_name, topic_type, "ml4kp_bridge::Plan", nh);
      registred |= plan_st_queue.register_topic(topic_name, topic_type, "ml4kp_bridge::PlanStamped", nh);
      registred |= spoint_queue.register_topic(topic_name, topic_type, "ml4kp_bridge::SpacePoint", nh);
      registred |= spoint_st_queue.register_topic(topic_name, topic_type, "ml4kp_bridge::SpacePointStamped", nh);
      registred |= stela_traj_queue.register_topic(topic_name, topic_type, "ml4kp_bridge::StelaTrajectory", nh);

      registred |= pose2d_queue.register_topic(topic_name, topic_type, "geometry_msgs::Pose2D", nh);
      registred |= pose_stamped_queue.register_topic(topic_name, topic_type, "geometry_msgs::PoseStamped", nh);
      registred |= twist_stamped_queue.register_topic(topic_name, topic_type, "geometry_msgs::TwistStamped", nh);

      registred |= prx_mushr_obs_queue.register_topic(topic_name, topic_type, "prx_models::MushrObservation", nh);
      registred |= prx_mushr_plan_queue.register_topic(topic_name, topic_type, "prx_models::MushrPlan", nh);
      registred |= prx_mushr_ctrl_queue.register_topic(topic_name, topic_type, "prx_models::MushrControl", nh);
      registred |= prx_tree_queue.register_topic(topic_name, topic_type, "prx_models::Tree", nh);

      registred |= tf_queue.register_topic(topic_name, topic_type, "tf2_msgs::TFMessage", nh);

      registred |= sensor_data_stamped_queue.register_topic(topic_name, topic_type, "interface::SensorDataStamped", nh);
      registred |= stamped_markers_queue.register_topic(topic_name, topic_type, "interface::StampedMarkers", nh);
      registred |= node_status_queue.register_topic(topic_name, topic_type, "interface::NodeStatus", nh);
      registred |= planner_clock_queue.register_topic(topic_name, topic_type, "interface::PlannerClock", nh);
      registred |= stela_status_queue.register_topic(topic_name, topic_type, "interface::StelaStatus", nh);
      registred |= ctrls_plot_queue.register_topic(topic_name, topic_type, "interface::ControlsPlot", nh);

      registred |= marker_queue.register_topic(topic_name, topic_type, "visualization_msgs::Marker", nh);
      registred |= marker_array_queue.register_topic(topic_name, topic_type, "visualization_msgs::MarkerArray", nh);

      DEBUG_VARS(topic_name, registred)
      prx_assert(registred, "Topic not supported: " << topic_name)
      // std::cout << "Unsupported topic '" << topic_name << "' type: " << topic_type << std::endl;
    }
  }

  auto all_qs()
  {
    return std::forward_as_tuple(float64_queue, string_queue, int32_queue, bool_queue,               // std_msgs
                                 ackermann_drive_stamped_queue,                                      // ackermann
                                 image_queue, imu_queue, cam_info_queue,                             // Sensor::msgs
                                 twist_stamped_queue, pose2d_queue, pose_stamped_queue,              // geometry_msgs
                                 plan_queue, plan_st_queue, traj_queue, traj_st_queue,               // ml4kp
                                 spoint_queue, spoint_st_queue, stela_traj_queue, stela_traj_queue,  // ml4kp
                                 prx_tree_queue, prx_mushr_ctrl_queue, prx_mushr_plan_queue,         // prx_models 1
                                 prx_mushr_obs_queue,                                                // prx_models 2
                                 tf_queue,                                                           // TF
                                 stamped_markers_queue, node_status_queue, planner_clock_queue,      // interface 1
                                 stela_status_queue, ctrls_plot_queue, sensor_data_stamped_queue,    // interface 2
                                 marker_queue, marker_array_queue                                    // vis_msgs
    );
  }

  template <typename Queue>
  static std::size_t process_queue(rosbag::Bag& bag, const Queue& queue)
  {
    std::size_t msgs_left{ 0 };
    for (std::size_t idx = 0; idx < queue.size(); ++idx)
    {
      if (!queue[idx]._queue.empty())
      {
        const auto msg = queue[idx]._queue.front();
        const std::string topic_name{ queue[idx].topic_name() };
        bag.write(topic_name, std::get<0>(msg), std::get<1>(msg));
        queue[idx]._queue.pop();
        msgs_left += queue[idx]._queue.size();
      }
    }

    return msgs_left;
  }

  // template <typename... Queues>
  template <typename TupleQueue, std::size_t... Is>
  std::size_t process_all_queues(TupleQueue& qs, std::index_sequence<Is...>)
  {
    // int tot{ 0 };
    // for (auto& q : queues...)
    // {
    //   tot += process_queue(q);
    // }
    // return tot;
    // msgs_left = std::apply(&process_queue, all_qs());
    // std::apply([](auto&&... args) {((process_queue(bag, ))+ ...);}, t);
    // std::apply([](auto&&... args) {((process_queue(bag, ))+ ...);}, t);
    // return (process_queue(bag, queues) + ...);
    // auto qs = all_qs();
    // const std::size_t qs_size{ std::tuple_size_v<decltype(qs)> };
    // auto seq = std::make_index_sequence<qs_size>{};

    return (process_queue(bag, std::get<Is>(qs)) + ...);
  }

  // template <typename Q, typename... Queues>
  template <typename TupleQueue, std::size_t... Is>
  void pause_queues_impl(const bool pause, TupleQueue& qs, std::index_sequence<Is...>)
  {
    (std::get<Is>(qs).pause(pause), ...);
  }

  void pause_queues(const bool pause)
  {
    auto qs = all_qs();
    const std::size_t qs_size{ std::tuple_size_v<decltype(qs)> };
    pause_queues_impl(pause, qs, std::make_index_sequence<qs_size>{});
  }
  // template <typename Q, typename... Queues>
  template <typename TupleQueue, std::size_t... Is>
  void reset_queues_impl(TupleQueue& qs, std::index_sequence<Is...>)
  {
    (std::get<Is>(qs).reset(), ...);
  }

  void reset_queues()
  {
    auto qs = all_qs();
    const std::size_t qs_size{ std::tuple_size_v<decltype(qs)> };
    reset_queues_impl(qs, std::make_index_sequence<qs_size>{});
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    if (_node_status->status() == interface::NodeStatus::RUNNING)
    {
      if (_first)
      {
        pause_queues(false);
        reset_queues();
      }
      write();
    }
    else if (_node_status->status() == interface::NodeStatus::READY)
    {
      _first = true;
      pause_queues(true);
    }
    else if (_node_status->status() == interface::NodeStatus::RESET)
    {
      pause_queues(true);
      write();
      _node_status->status(interface::NodeStatus::WAITING);
    }
    else if (_node_status->status() == interface::NodeStatus::WAITING)
    {
      const std::size_t msgs_left{ write() };
      if (msgs_left == 0)
      {
        bag.close();
        init_bag();
        _node_status->status(interface::NodeStatus::READY);
      }
    }
    else if (_node_status->status() == interface::NodeStatus::FINISH)
    {
      pause_queues(true);
      const std::size_t msgs_left{ write() };

      if (msgs_left == 0)
      {
        bag.close();
        ros::shutdown();
      }
    }
    else
    {
      auto invalid_status = _node_status;
      DEBUG_VARS(invalid_status);
    }
  }

  std::size_t write()
  {
    auto qs = all_qs();
    const std::size_t qs_size{ std::tuple_size_v<decltype(qs)> };
    const std::size_t msgs_left{ process_all_queues(qs, std::make_index_sequence<qs_size>{}) };
    return msgs_left;
  }

  void bag_write()
  {
    interface::init_bag(&bag, rosbag_directory, rosbag_prefix);

    ros::Time msg_t;

    std::size_t msgs_left{ 0 };

    while (msgs_left > 0)
    {
      auto qs = all_qs();
      const std::size_t qs_size{ std::tuple_size_v<decltype(qs)> };
      // auto seq = std::make_index_sequence<qs_size>{};
      msgs_left = process_all_queues(qs, std::make_index_sequence<qs_size>{});
      // msgs_left = std::apply(&process_all_queues, all_qs());
      // msgs_left = process_all_queues(bag,                                                    // no-lint
      //                                float64_queue, string_queue, int32_queue, bool_queue,   // std_msgs
      //                                ackermann_drive_stamped_queue,                          // ackermann
      //                                image_queue, imu_queue, cam_info_queue,                 // Sensor::msgs
      //                                twist_stamped_queue, pose2d_queue, pose_stamped_queue,  // geometry_msgs
      //                                plan_queue, plan_st_queue, traj_queue, traj_st_queue,   // ml4kp
      //                                spoint_queue, spoint_st_queue, stela_traj_queue, stela_traj_queue,  // ml4kp
      //                                prx_tree_queue, prx_mushr_ctrl_queue, prx_mushr_plan_queue,         //
      //                                prx_models 1 prx_mushr_obs_queue, // prx_models 2 tf_queue, // TF
      //                                stamped_markers_queue, node_status_queue, planner_clock_queue,      // interface
      //                                1 stela_status_queue, ctrls_plot_queue, sensor_data_stamped_queue,    //
      //                                interface 2 marker_queue, marker_array_queue // vis_msgs
      // );
      if (stop)
      {
        ROS_INFO_STREAM_ONCE("Remaining messages: " << msgs_left);
      }
    }

    bag.close();
    ROS_INFO_STREAM("Rosbag closed.");
  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosbag_record");

  ros::NodeHandle nh("~");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // PARAM_SETUP(nh, rosbag_directory);
  // PARAM_SETUP_WITH_DEFAULT(nh, rosbag_prefix, rosbag_prefix);
  // ROS_PARAM_SETUP(nh, stop_topic);

  // std::vector<ros::Subscriber> subscribers;
  // utils::execution_status_t execution_status(nh, stop_topic);

  // PRX_DEBUG_VARS(rosbag_directory);
  // PRX_DEBUG_VARS(stop_topic);

  // interface::node_status_t node_status(nh);

  // PRX_DEBUG_VARS(topics.size());

  // DEBUG_VARS(subscribers.size());
  // std::thread thread_b(bag_writter);
  bag_writer_t bag_writter(nh);
  // ros::AsyncSpinner spinner(4);
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  // ros::waitForShutdown();

  // node_status.status(interface::NodeStatus::RUNNING);
  // while (ros::ok())
  // {
  //   if (node_status.status() == interface::NodeStatus::RUNNING)
  //   {
  //     continue;
  //   }
  //   else if (node_status.status() == interface::NodeStatus::RESET)
  //   {
  //   }
  //   else if (node_status.status() == interface::NodeStatus::FINISH)
  //   {
  //     stop = true;
  //     break;
  //   }
  //   else
  //   {
  //     auto invalid_status = node_status;
  //     DEBUG_VARS(invalid_status);
  //   }
  //   //    ros::spinOnce();
  // }
  // stop = true;
  // ROS_INFO_STREAM("Joining bag writter thread");
  // spinner.stop();
  // thread_b.join();

  return 0;
}
