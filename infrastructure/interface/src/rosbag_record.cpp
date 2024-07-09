#include <atomic>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <ml4kp_bridge/PlanStamped.h>
#include <ml4kp_bridge/SpacePointStamped.h>
#include <ml4kp_bridge/TrajectoryStamped.h>

#include <prx_models/Tree.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

#include <cv_bridge/cv_bridge.h>

#include <XmlRpcValue.h>

#include <tf2_msgs/TFMessage.h>

#include <utils/rosparams_utils.hpp>
#include <utils/execution_status.hpp>
// #include <interface/defs.hpp>
#include <interface/rosbag_record.hpp>
#include <interface/StampedMarkers.h>

std::atomic<bool> stop = false;
std::string rosbag_directory = "";
std::string bagname = "";

std::unordered_map<std::string, std::shared_ptr<interface::queues_base_t>> all_queues;
// interface::queues_t<std_msgs::String> string_queue;
// interface::queues_t<std_msgs::Int32> int32_queue;
// interface::queues_t<std_msgs::Float64> float64_queue;
// interface::queues_t<interface::StampedMarkers> stamped_markers_queue;
// interface::queues_t<geometry_msgs::TwistStamped> twist_stamped_queue;
// interface::queues_t<geometry_msgs::Pose2D> pose2d_queue;
// interface::queues_t<geometry_msgs::PoseStamped> pose_stamped_queue;
// interface::queues_t<ackermann_msgs::AckermannDriveStamped> ackermann_drive_stamped_queue;
// interface::queues_t<sensor_msgs::Image> image_queue;
// interface::queues_t<sensor_msgs::Imu> imu_queue;
// interface::queues_t<ml4kp_bridge::PlanStamped> plan_queue;
// interface::queues_t<ml4kp_bridge::TrajectoryStamped> traj_queue;
// interface::queues_t<tf2_msgs::TFMessage> tf_queue;

void init_all_queues()
{
  using interface::queues_t;
  all_queues["std_msgs::String"] = std::make_shared<queues_t<std_msgs::String>>();
  all_queues["std_msgs::Int32"] = std::make_shared<queues_t<std_msgs::Int32>>();
  all_queues["std_msgs::Float64"] = std::make_shared<queues_t<std_msgs::Float64>>();
  all_queues["interface::StampedMarkers"] = std::make_shared<queues_t<interface::StampedMarkers>>();
  all_queues["geometry_msgs::TwistStamped"] = std::make_shared<queues_t<geometry_msgs::TwistStamped>>();
  all_queues["geometry_msgs::Pose2D"] = std::make_shared<queues_t<geometry_msgs::Pose2D>>();
  all_queues["geometry_msgs::PoseStamped"] = std::make_shared<queues_t<geometry_msgs::PoseStamped>>();
  all_queues["ackermann_msgs::AckermannDriveStamped"] =
      std::make_shared<queues_t<ackermann_msgs::AckermannDriveStamped>>();
  all_queues["sensor_msgs::Image"] = std::make_shared<queues_t<sensor_msgs::Image>>();
  all_queues["sensor_msgs::Imu"] = std::make_shared<queues_t<sensor_msgs::Imu>>();
  all_queues["ml4kp_bridge::Plan"] = std::make_shared<queues_t<ml4kp_bridge::Plan>>();
  all_queues["ml4kp_bridge::PlanStamped"] = std::make_shared<queues_t<ml4kp_bridge::PlanStamped>>();
  all_queues["ml4kp_bridge::Trajectory"] = std::make_shared<queues_t<ml4kp_bridge::Trajectory>>();
  all_queues["ml4kp_bridge::TrajectoryStamped"] = std::make_shared<queues_t<ml4kp_bridge::TrajectoryStamped>>();
  all_queues["ml4kp_bridge::SpacePoint"] = std::make_shared<queues_t<ml4kp_bridge::SpacePoint>>();
  all_queues["ml4kp_bridge::SpacePointStamped"] = std::make_shared<queues_t<ml4kp_bridge::SpacePointStamped>>();
  all_queues["tf2_msgs::TFMessage"] = std::make_shared<queues_t<tf2_msgs::TFMessage>>();
  all_queues["prx_models::Tree"] = std::make_shared<queues_t<prx_models::Tree>>();
}

// template <typename Queue>
// std::size_t process_queue(rosbag::Bag& bag, const std::shared_ptr<Queue>& queue_ptr)
// {
//   std::size_t msgs_left{ 0 };
//   Queue& queue{ *queue_ptr };
//   for (std::size_t idx = 0; idx < queue.size(); ++idx)
//   {
//     if (!queue[idx]._queue.empty())
//     {
//       msgs_left += queue[idx]._queue.size();
//       auto msg = queue[idx]._queue.front();
//       bag.write(std::get<0>(msg), std::get<1>(msg), std::get<2>(msg));
//       queue[idx]._queue.pop();
//     }
//   }
//   return msgs_left;
// }

// template <typename... Queues>
std::size_t process_all_queues(rosbag::Bag& bag)
{
  std::size_t total_msgs{ 0 };
  for (auto q : all_queues)
  {
    total_msgs += q.second->process_queue(bag);
  }
  return total_msgs;
  // return (process_queue(bag, queues) + ...);
}

void bag_writter()
{
  rosbag::Bag bag;
  interface::init_bag(&bag, rosbag_directory, bagname);

  ros::Time msg_t;

  std::size_t msgs_left{ 0 };

  while (msgs_left > 0 || !stop)
  {
    msgs_left = process_all_queues(bag);
    // msgs_left =
    //     process_all_queues(bag, stamped_markers_queue, string_queue, twist_stamped_queue,
    //     ackermann_drive_stamped_queue,
    //                        image_queue, imu_queue, plan_queue, traj_queue, float64_queue, pose2d_queue, tf_queue);
    if (stop)
    {
      ROS_INFO_STREAM_ONCE("Remaining messages: " << msgs_left);
    }
  }
  bag.close();
  ROS_INFO_STREAM("Rosbag closed.");
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosbag_record");

  ros::NodeHandle nh("~");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  XmlRpc::XmlRpcValue topics;
  std::string stop_topic;
  ROS_PARAM_SETUP(nh, topics);
  ROS_PARAM_SETUP(nh, rosbag_directory);
  ROS_PARAM_SETUP(nh, stop_topic);
  PARAM_SETUP_WITH_DEFAULT(nh, bagname, bagname)

  std::vector<ros::Subscriber> subscribers;
  utils::execution_status_t execution_status(nh, stop_topic);

  init_all_queues();
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
    std::shared_ptr<interface::queues_base_t> queue{ all_queues[topic_type] };
    if (queue != nullptr)
    {
      registred |= queue->register_topic(topic_name, topic_type, topic_type, subscribers, nh);
    }

    // registred |= string_queue.register_topic(topic_name, topic_type, "std_msgs::string", subscribers, nh);
    // registred |= int32_queue.register_topic(topic_name, topic_type, "std_msgs::int32", subscribers, nh);
    // registred |=
    //     stamped_markers_queue.register_topic(topic_name, topic_type, "interface::stamped_markers", subscribers, nh);
    // registred |=
    //     twist_stamped_queue.register_topic(topic_name, topic_type, "geometry_msgs::TwistStamped", subscribers, nh);
    // registred |= ackermann_drive_stamped_queue.register_topic(topic_name, topic_type,
    //                                                           "ackermann_msgs::AckermannDriveStamped", subscribers,
    //                                                           nh);
    // registred |= image_queue.register_topic(topic_name, topic_type, "sensor_msgs::Image", subscribers, nh);
    // registred |= imu_queue.register_topic(topic_name, topic_type, "sensor_msgs::Imu", subscribers, nh);
    // registred |= traj_queue.register_topic(topic_name, topic_type, "ml4kp_bridge::TrajectoryStamped", subscribers,
    // nh); registred |= plan_queue.register_topic(topic_name, topic_type, "ml4kp_bridge::PlanStamped", subscribers,
    // nh); registred |= pose2d_queue.register_topic(topic_name, topic_type, "geometry_msgs::Pose2D", subscribers, nh);
    // registred |=
    //     pose_stamped_queue.register_topic(topic_name, topic_type, "geometry_msgs::PoseStamped", subscribers, nh);
    // registred |= float64_queue.register_topic(topic_name, topic_type, "std_msgs::Float64", subscribers, nh);
    // registred |= tf_queue.register_topic(topic_name, topic_type, "tf2_msgs::TFMessage", subscribers, nh);

    if (!registred)
    {
      std::cout << "Unsupported topic '" << topic_name << "' type: " << topic_type << std::endl;
    }
  }

  std::thread thread_b(bag_writter);

  while (execution_status.ok())
  {
    ros::spinOnce();
  }
  stop = true;
  ROS_INFO_STREAM("Joining bag writter thread");
  thread_b.join();

  return 0;
}