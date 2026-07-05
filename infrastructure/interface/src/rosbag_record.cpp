#include <atomic>
#include <chrono>
#include <iterator>
#include <memory>
#include <prx/utilities/general/prx_assert.hpp>
#include <thread>

#include <ros/duration.h>
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
#include "interface/ReplannerStatus.h"
#include "interface/SensorDataStamped.h"
#include <ml4kp_bridge/StelaTrajectory.h>
#include <ml4kp_bridge/PlanStepStampedArray.h>
#include "prx_models/MushrControl.h"
#include "prx_models/MushrObservation.h"
#include "prx_models/MushrPlan.h"
#include <prx_models/PlannerStats.h>
#include <interface/NodeStatus.h>
#include <interface/PlannerClock.h>
#include <interface/StelaStatus.h>
#include <prx_models/Tree.h>

struct ros_qs_types_t
{
  interface::rosbag_queue_t<ackermann_msgs::AckermannDriveStamped> ackermann_drive_stamped_queue;

  interface::rosbag_queue_t<std_msgs::String> string_queue;
  interface::rosbag_queue_t<std_msgs::Int32> int32_queue;
  interface::rosbag_queue_t<std_msgs::Float64> float64_queue;
  interface::rosbag_queue_t<std_msgs::Bool> bool_queue;
  interface::rosbag_queue_t<std_msgs::Empty> empty_queue;
  interface::rosbag_queue_t<std_msgs::Duration> duration_queue;

  interface::rosbag_queue_t<geometry_msgs::TwistStamped> twist_stamped_queue;
  interface::rosbag_queue_t<geometry_msgs::Pose2D> pose2d_queue;
  interface::rosbag_queue_t<geometry_msgs::PoseStamped> pose_stamped_queue;

  interface::rosbag_queue_t<sensor_msgs::Image> image_queue;
  interface::rosbag_queue_t<sensor_msgs::CameraInfo> cam_info_queue;
  interface::rosbag_queue_t<sensor_msgs::Imu> imu_queue;

  interface::rosbag_queue_t<ml4kp_bridge::Plan> plan_queue;
  interface::rosbag_queue_t<ml4kp_bridge::PlanStamped> plan_st_queue;
  interface::rosbag_queue_t<ml4kp_bridge::Trajectory> traj_queue;
  interface::rosbag_queue_t<ml4kp_bridge::TrajectoryStamped> traj_st_queue;
  interface::rosbag_queue_t<ml4kp_bridge::SpacePoint> spoint_queue;
  interface::rosbag_queue_t<ml4kp_bridge::SpacePointStamped> spoint_st_queue;
  interface::rosbag_queue_t<ml4kp_bridge::StelaTrajectory> stela_traj_queue;
  interface::rosbag_queue_t<ml4kp_bridge::PlanStepStampedArray> plan_step_stamped_array_queue;

  interface::rosbag_queue_t<prx_models::Tree> prx_tree_queue;
  interface::rosbag_queue_t<prx_models::MushrPlan> prx_mushr_plan_queue;
  interface::rosbag_queue_t<prx_models::MushrControl> prx_mushr_ctrl_queue;
  interface::rosbag_queue_t<prx_models::MushrObservation> prx_mushr_obs_queue;
  interface::rosbag_queue_t<prx_models::PlannerStats> planner_stats_queue;

  interface::rosbag_queue_t<tf2_msgs::TFMessage> tf_queue;

  interface::rosbag_queue_t<interface::StampedMarkers> stamped_markers_queue;
  interface::rosbag_queue_t<interface::NodeStatus> node_status_queue;
  interface::rosbag_queue_t<interface::PlannerClock> planner_clock_queue;
  interface::rosbag_queue_t<interface::StelaStatus> stela_status_queue;
  interface::rosbag_queue_t<interface::ControlsPlot> ctrls_plot_queue;
  interface::rosbag_queue_t<interface::SensorDataStamped> sensor_data_stamped_queue;
  interface::rosbag_queue_t<interface::ReplannerStatus> replanner_status_queue;

  interface::rosbag_queue_t<visualization_msgs::Marker> marker_queue;
  interface::rosbag_queue_t<visualization_msgs::MarkerArray> marker_array_queue;

  ros_qs_types_t()
    :  //  ACKERMANN MSGS
    ackermann_drive_stamped_queue("ackermann_msgs::AckermannDriveStamped")
    // STD MSGS
    , bool_queue("std_msgs::Bool")
    , float64_queue("std_msgs::Float64")
    , int32_queue("std_msgs::int32")
    , string_queue("std_msgs::string")
    , empty_queue("std_msgs::Empty")
    , duration_queue("std_msgs::Duration")
    // GEOMETRY MSGS
    , pose2d_queue("geometry_msgs::Pose2D")
    , pose_stamped_queue("geometry_msgs::PoseStamped")
    , twist_stamped_queue("geometry_msgs::TwistStamped")
    // SENSOR MSGS
    , image_queue("sensor_msgs::Image")
    , imu_queue("sensor_msgs::Imu")
    , cam_info_queue("sensor_msgs::CameraInfo")
    // ML4KP BRIDGE
    , traj_queue("ml4kp_bridge::Trajectory")
    , traj_st_queue("ml4kp_bridge::TrajectoryStamped")
    , plan_queue("ml4kp_bridge::Plan")
    , plan_st_queue("ml4kp_bridge::PlanStamped")
    , spoint_queue("ml4kp_bridge::SpacePoint")
    , spoint_st_queue("ml4kp_bridge::SpacePointStamped")
    , stela_traj_queue("ml4kp_bridge::StelaTrajectory")
    , plan_step_stamped_array_queue("ml4kp_bridge::PlanStepStampedArray")
    // PRX MODELS
    , prx_mushr_obs_queue("prx_models::MushrObservation")
    , prx_mushr_plan_queue("prx_models::MushrPlan")
    , prx_mushr_ctrl_queue("prx_models::MushrControl")
    , planner_stats_queue("prx_models::PlannerStats")
    , prx_tree_queue("prx_models::Tree")
    // TF
    , tf_queue("tf2_msgs::TFMessage")
    // INTERFACE
    , sensor_data_stamped_queue("interface::SensorDataStamped")
    , stamped_markers_queue("interface::StampedMarkers")
    , node_status_queue("interface::NodeStatus")
    , planner_clock_queue("interface::PlannerClock")
    , stela_status_queue("interface::StelaStatus")
    , ctrls_plot_queue("interface::ControlsPlot")
    , replanner_status_queue("interface::ReplannerStatus")
    // VISUALIZATION MSGS
    , marker_queue("visualization_msgs::Marker")
    , marker_array_queue("visualization_msgs::MarkerArray")
  {
  }

  auto all_qs()
  {
    return std::forward_as_tuple(float64_queue, string_queue, int32_queue, bool_queue, empty_queue,  // std_msgs
                                 duration_queue,                                                     // std_msgs
                                 ackermann_drive_stamped_queue,                                      // ackermann
                                 image_queue, imu_queue, cam_info_queue,                             // Sensor::msgs
                                 twist_stamped_queue, pose2d_queue, pose_stamped_queue,              // geometry_msgs
                                 plan_queue, plan_st_queue, traj_queue, traj_st_queue,               // ml4kp
                                 spoint_queue, spoint_st_queue, stela_traj_queue, stela_traj_queue,  // ml4kp
                                 plan_step_stamped_array_queue,                                      // ml4kp
                                 prx_tree_queue, prx_mushr_ctrl_queue, prx_mushr_plan_queue,         // prx_models 1
                                 prx_mushr_obs_queue, planner_stats_queue,                           // prx_models 2
                                 tf_queue,                                                           // TF
                                 stamped_markers_queue, node_status_queue, planner_clock_queue,      // interface 1
                                 stela_status_queue, ctrls_plot_queue, sensor_data_stamped_queue,    // interface 2
                                 replanner_status_queue,                                             // interface 3
                                 marker_queue, marker_array_queue                                    // vis_msgs
    );
  }

  template <typename TupleQueue, std::size_t... Is>
  bool register_topic_impl(const TupleQueue& qs, const std::string& topic_name, const std::string topic_type,
                           ros::NodeHandle& nh, std::index_sequence<Is...>)
  {
    // return q.register_topic(topic_name, topic_type,  nh);
    return (std::get<Is>(qs).register_topic(topic_name, topic_type, nh) or ...);
  }

  void register_topics(ros::NodeHandle& nh, XmlRpc::XmlRpcValue topics)
  {
    const auto qs = all_qs();
    const std::size_t qs_size{ std::tuple_size_v<decltype(qs)> };
    for (int i = 0; i < topics.size(); ++i)
    {
      auto topic_i = topics[i];
      const std::string topic_name(topic_i["name"]);
      const std::string topic_type(topic_i["type"]);

      bool registred{ register_topic_impl(qs, topic_name, topic_type, nh, std::make_index_sequence<qs_size>{}) };

      // DEBUG_VARS(topic_name, registred)
      prx_assert(registred, "Topic not supported: " << topic_name)
      // std::cout << "Unsupported topic '" << topic_name << "' type: " << topic_type << std::endl;
    }
  }

  template <typename TupleQueue, std::size_t... Is>
  std::size_t total_msgs_impl(const TupleQueue& qs, std::index_sequence<Is...>)
  {
    return (std::get<Is>(qs).total_msgs() + ...);
  }

  std::size_t total_msgs()
  {
    const auto qs = all_qs();
    const std::size_t qs_size{ std::tuple_size_v<decltype(qs)> };
    return total_msgs_impl(qs, std::make_index_sequence<qs_size>{});
  }
};

struct bag_writer_t
{
  std::atomic<bool> stop = false;
  std::string rosbag_directory;
  std::string rosbag_prefix;

  rosbag::Bag bag;
  XmlRpc::XmlRpcValue topics;

  ros::Timer _timer;

  int _bag_num;
  bool _first;

  std::shared_ptr<interface::node_status_t> _node_status;
  bool _verbose;

  double _msgs_in_queue;

  ros_qs_types_t _qs;
  ros::NodeHandle _nh;

  bag_writer_t(ros::NodeHandle& nh, std::shared_ptr<interface::node_status_t> node_status)
    : rosbag_directory("")
    , rosbag_prefix("")
    , _bag_num(0)
    , _first(true)
    , _verbose(false)
    , _nh(nh)
    , _node_status(node_status)
  {
    bool& verbose{ _verbose };
    PARAM_SETUP(nh, topics);
    // PARAM_SETUP(nh, rosbag_directory);
    // PARAM_SETUP_WITH_DEFAULT(nh, rosbag_prefix, rosbag_prefix);
    PARAM_SETUP_WITH_DEFAULT(nh, verbose, verbose);

    _qs.register_topics(nh, topics);
    pause_queues(true);

    _timer = nh.createTimer(ros::Duration(5.0), &bag_writer_t::timer_callback, this);
    // init_bag();

    dbg::set_log_filename("log_rosbag_record.txt");

    _node_status->status(interface::NodeStatus::READY);
  }

  ~bag_writer_t()
  {
    _node_status->status(interface::NodeStatus::FINISH);
    ros::Duration(1.0).sleep();
  }

  bool init_bag()
  {
    const bool dir_set{ ros::param::get("/rosbag/directory", rosbag_directory) };
    const bool prefix_set{ ros::param::get("/rosbag/prefix", rosbag_prefix) };

    // DEBUG_VARS(dir_set, prefix_set)
    if (dir_set and prefix_set)
    {
      const std::string bn{ prx::utilities::convert_to<std::string>(_bag_num) };
      interface::init_bag(&bag, rosbag_directory, rosbag_prefix + "_" + bn);
      _bag_num++;

      // ros::param::del("/rosbag/directory");
      // ros::param::del("/rosbag/prefix");
      return true;
    }
    return false;
  }

  template <typename Queue>
  static std::size_t process_queue(rosbag::Bag& bag, Queue& queue)
  {
    std::size_t msgs_left{ 0 };
    try
    {
      // for (std::size_t idx = 0; idx < queue.size(); ++idx)
      // {
      if (!queue.empty())
      {
        // const auto msg = queue[idx]._queue.front();
        const auto msg = queue.get_next();
        // const std::string topic_name{ queue[idx].topic_name() };
        // const ros::Time& ti{ std::get<0>(msg) };
        const std::string topic_name{ msg.getConnectionHeaderPtr()->at("topic") };

        // auto msg_p = *(msg.getConnectionHeaderPtr());
        // LOG_VARS(topic_name, msg_p)
        // bag.write(topic_name, ti, std::get<1>(msg));
        bag.write(topic_name, msg);
        // queue[idx]._queue.pop();
        msgs_left += queue.size();
      }
      // }
    }
    catch (const std::exception& ex)
    {
      std::cout << ex.what() << std::endl;
      // prx_warn("Error at [bag_writter::process_queue]");
    }
    catch (...)
    {
      prx_warn("Error at [bag_writter::process_queue]");
    }
    return msgs_left;
  }

  // template <typename... Queues>
  template <typename TupleQueue, std::size_t... Is>
  std::size_t process_all_queues(TupleQueue& qs, std::index_sequence<Is...>)
  {
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
    auto qs = _qs.all_qs();
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
    auto qs = _qs.all_qs();
    const std::size_t qs_size{ std::tuple_size_v<decltype(qs)> };
    reset_queues_impl(qs, std::make_index_sequence<qs_size>{});
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    if (_verbose)
    {
      const double ROSBAG_MSGS_IN_Q{ _msgs_in_queue };
      const std::size_t total_msgs{ _qs.total_msgs() };
      DEBUG_VARS(_node_status, ROSBAG_MSGS_IN_Q, total_msgs);
    }
  }

  void process_req_status()
  {
    if (_node_status->new_request())
    {
      const interface::node_status_t::StatusType current_status{ _node_status->status() };
      const interface::node_status_t::StatusType req_status{ _node_status->requested_status() };
      if (current_status == interface::NodeStatus::RUNNING or current_status == interface::NodeStatus::READY)
      {
        _node_status->status(_node_status->requested_status());
        _node_status->request_acknowledged();
      }
    }
  }

  void run()
  {
    while (ros::ok())
    {
      process_req_status();
      if (_node_status->status() == interface::NodeStatus::RUNNING)
      {
        if (_first)
        {
          init_bag();

          pause_queues(false);
          reset_queues();
          _first = false;
          LOG_MSG("RUNNING");
        }
        // const std::size_t msgs_left{ write() };
        // DEBUG_VARS(msgs_left)

        _msgs_in_queue = write();
      }
      else if (_node_status->status() == interface::NodeStatus::PAUSED)
      {
        _first = true;
        pause_queues(true);
      }
      else if (_node_status->status() == interface::NodeStatus::READY)
      {
        _first = true;
        pause_queues(true);
      }
      else if (_node_status->status() == interface::NodeStatus::RESET)
      {
        LOG_MSG("RESET");
        pause_queues(true);
        //   write();
        //   _node_status->status(interface::NodeStatus::WAITING);
        // }
        // else if (_node_status->status() == interface::NodeStatus::WAITING)
        // {
        //   pause_queues(true);
        if (_msgs_in_queue > 0)
        {
          _msgs_in_queue = write();

          if (_msgs_in_queue == 0)
          {
            bag.close();
            _first = true;
          }
        }
        else
        {
          if (_node_status->new_request())
          {
            const interface::node_status_t::StatusType current_status{ _node_status->status() };
            const interface::node_status_t::StatusType req_status{ _node_status->requested_status() };
            _node_status->status(_node_status->requested_status());
            _node_status->request_acknowledged();
          }
        }
        // DEBUG_VARS(msgs_left)
        // // LOG_MSG("WAITING")
        // if (init_bag())
        // {
        //   _node_status->status(interface::NodeStatus::READY);
        //   // }
        // }
      }
      else if (_node_status->status() == interface::NodeStatus::FINISH)
      {
        pause_queues(true);
        _msgs_in_queue = write();

        if (_msgs_in_queue == 0)
        {
          LOG_MSG("FINISH");
          bag.close();
          return;
          // ros::shutdown();
        }
      }
      else
      {
        auto invalid_status = _node_status;
        DEBUG_VARS(invalid_status);
      }
    }
  }

  std::size_t write()
  {
    auto qs = _qs.all_qs();
    const std::size_t qs_size{ std::tuple_size_v<decltype(qs)> };
    const std::size_t msgs_left{ process_all_queues(qs, std::make_index_sequence<qs_size>{}) };
    return msgs_left;
  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosbag_record");

  ros::NodeHandle nh("~");
  ros::Duration(10.0).sleep();

  std::string experiments_node_id;
  PARAM_SETUP(nh, experiments_node_id)

  std::shared_ptr<interface::node_status_t> experiments_node_status;
  experiments_node_status = interface::node_status_t::create(nh, experiments_node_id, true);

  std::shared_ptr<interface::node_status_t> node_status{ interface::node_status_t::create(nh) };

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (experiments_node_status->status() != interface::NodeStatus::FINISH)
  {
    node_status->status(interface::NodeStatus::INITIALIZING, experiments_node_status->sequence_id());
    bag_writer_t bag_writter(nh, node_status);
    bag_writter.run();
  }
  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
