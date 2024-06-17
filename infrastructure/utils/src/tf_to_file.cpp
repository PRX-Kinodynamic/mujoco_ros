#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <tf2_ros/transform_listener.h>

#include <utils/dbg_utils.hpp>
#include <utils/std_utils.cpp>
#include <utils/execution_status.hpp>
#include <utils/rosparams_utils.hpp>

std::stringstream strstr_plan;
std::stringstream strstr_ctrl;
std::ofstream ofs_poses;

ros::Time initial_time;

inline std::string to_string(const std_msgs::Header& header)
{
  std::stringstream strstr{};
  strstr << header.stamp << " ";
  return strstr.str();
}

inline std::string to_string(const geometry_msgs::TransformStamped& tf)
{
  std::stringstream strstr{};
  const geometry_msgs::Vector3& vec{ tf.transform.translation };
  const geometry_msgs::Quaternion& quat{ tf.transform.rotation };

  strstr << to_string(tf.header);
  strstr << vec.x << " ";
  strstr << vec.y << " ";
  strstr << vec.z << " ";
  strstr << quat.w << " ";
  strstr << quat.x << " ";
  strstr << quat.y << " ";
  strstr << quat.z << " ";

  return strstr.str();
}

std::string get_transform(tf2_ros::Buffer& tf_buffer, const std::string root_frame, const std::string child_frame,
                          ros::Time& last)
{
  std::string res{ "" };

  try
  {
    const geometry_msgs::TransformStamped tf{ tf_buffer.lookupTransform(root_frame, child_frame, ros::Time(0)) };
    if (last < tf.header.stamp)
    {
      res = to_string(tf);
      last = tf.header.stamp;
    }
  }
  catch (tf2::TransformException& ex)
  {
  }

  return res;
}

void set_ofs(std::ofstream& ofs, const std::filesystem::path& path)
{
  if (ofs.is_open())
  {
    ofs.close();
  }

  std::filesystem::path filename{ path };
  filename += "_" + utils::timestamp() + ".txt";
  // const std::filesystem::path filename{ fileprefix + "_" + utils::timestamp() + ".txt" };
  DEBUG_VARS(filename);

  ofs.open(filename, std::ios::trunc);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TfToFile");
  ros::NodeHandle nh("~");
  const std::string root{ ros::this_node::getNamespace() };

  std::string directory;
  std::string fileprefix;
  std::string reset_topic;
  XmlRpc::XmlRpcValue transforms;
  double window_duration{ 0.0 };

  // DEBUG_VARS();

  ROS_PARAM_SETUP(nh, directory);
  ROS_PARAM_SETUP(nh, fileprefix);
  ROS_PARAM_SETUP(nh, transforms);
  ROS_PARAM_SETUP(nh, reset_topic);
  PARAM_SETUP_WITH_DEFAULT(nh, window_duration, window_duration);

  utils::execution_status_t::FunctionOnReset reset_function = [&]() { ofs_poses << "\n"; };
  utils::execution_status_t status(nh, root + "/TfToFile", reset_topic, reset_function);
  std::filesystem::path path{ directory };
  path /= fileprefix + "_poses";
  // const std::string filename_poses{ fileprefix + "_poses" };

  std::vector<std::pair<std::string, std::string>> frames{};
  for (int i = 0; i < transforms.size(); ++i)
  {
    auto tf_i = transforms[i];
    const std::string frame_root(tf_i["root"]);
    const std::string frame_child(tf_i["child"]);
    DEBUG_VARS(frame_root, frame_child);
    frames.push_back(std::make_pair(frame_root, frame_child));
  }

  set_ofs(ofs_poses, path);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tfListener(tf_buffer);

  ros::Time now{ ros::Time(0) };
  initial_time = ros::Time::now();
  ros::Duration window_size(window_duration);
  ros::Time window{ ros::Time::now() + window_size };
  while (status.ok())
  {
    if (window_duration > 0 and window < ros::Time::now())
    {
      set_ofs(ofs_poses, path);
      window = ros::Time::now() + window_size;
    }
    for (auto fr : frames)
    {
      const std::string root{ fr.first };
      const std::string child{ fr.second };
      const std::string tf_str{ get_transform(tf_buffer, root, child, now) };
      if (tf_str.size() > 0)
      {
        ofs_poses << root << " " << child << " " << tf_str << "\n";
      }
    }
    ros::spinOnce();
  }

  ofs_poses.close();

  return 0;
}