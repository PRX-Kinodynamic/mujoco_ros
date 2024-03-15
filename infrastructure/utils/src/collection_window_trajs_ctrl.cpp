#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <utils/dbg_utils.hpp>
#include <utils/std_utils.cpp>
#include <utils/execution_status.hpp>
#include <utils/rosparams_utils.hpp>

#include <ml4kp_bridge/defs.h>

std::stringstream strstr_plan;
std::stringstream strstr_ctrl;
std::ofstream ofs_poses, ofs_plan, ofs_control;

ros::Time initial_time;

std::string tf_to_str(geometry_msgs::TransformStamped& tf)
{
  std::stringstream strstr;
  const geometry_msgs::Vector3& vec{ tf.transform.translation };
  const geometry_msgs::Quaternion& quat{ tf.transform.rotation };
  // tf::Vector3& vec{ tf.getOrigin() };
  // tf::Quaternion quat{ tf.getRotation() };
  strstr << tf.header.stamp << " ";
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
    geometry_msgs::TransformStamped tf{ tf_buffer.lookupTransform(root_frame, child_frame, ros::Time(0)) };
    if (last < tf.header.stamp)
    {
      res = tf_to_str(tf);
      last = tf.header.stamp;
    }
  }
  catch (tf2::TransformException& ex)
  {
  }

  return res;
}
void get_plan(const ml4kp_bridge::PlanStampedConstPtr message)
{
  const ros::Time now{ ros::Time::now() };
  for (auto step : message->plan.steps)
  {
    ofs_plan << now << " ";
    for (auto ctrl : step.control.point)
    {
      ofs_plan << ctrl.data << " ";
    }
    ofs_plan << step.duration.data << "\n";
  }
}

void get_control(const ml4kp_bridge::SpacePointConstPtr control)
{
  const ros::Time now{ ros::Time::now() };
  ofs_control << now << " ";
  for (auto e : control->point)
  {
    ofs_control << e.data << " ";
  }
  ofs_control << "\n";
}

void set_ofs(std::ofstream& ofs, const std::string& fileprefix)
{
  if (ofs.is_open())
  {
    ofs.close();
  }

  const std::string filename{ fileprefix + "_" + utils::timestamp() + ".txt" };
  DEBUG_VARS(filename);

  ofs.open(filename.c_str(), std::ios::trunc);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TfToFile");
  ros::NodeHandle nh("~");
  const std::string root{ ros::this_node::getNamespace() };

  std::string filename;
  XmlRpc::XmlRpcValue transforms;
  double window_duration{ 0.0 };

  ROS_PARAM_SETUP(nh, filename);
  ROS_PARAM_SETUP(nh, transforms);
  ROS_PARAM_SETUP(nh, window_duration);
  utils::execution_status_t status(nh, root + "/TfToFile");
  const std::string filename_poses{ filename + "_poses" };
  const std::string filename_plan{ filename + "_plan" };
  const std::string filename_ctrl{ filename + "_ctrl" };

  std::vector<std::pair<std::string, std::string>> frames{};
  for (int i = 0; i < transforms.size(); ++i)
  {
    auto tf_i = transforms[i];
    const std::string frame_root(tf_i["root"]);
    const std::string frame_child(tf_i["child"]);
    DEBUG_VARS(frame_root, frame_child);
    frames.push_back(std::make_pair(frame_root, frame_child));
  }

  tf::TransformListener tf_listener;
  tf::StampedTransform tf;

  set_ofs(ofs_poses, filename_poses);
  set_ofs(ofs_plan, filename_plan);
  set_ofs(ofs_control, filename_ctrl);

  ros::Subscriber plan_subscriber{ nh.subscribe("/mushr/ml4kp_plan", 1, &get_plan) };
  ros::Subscriber ctrl_subscriber{ nh.subscribe("/mushr/plan_stepper/control", 1, &get_control) };

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tfListener(tf_buffer);

  ros::Time now{ ros::Time(0) };
  initial_time = ros::Time::now();
  ros::Duration window_size(window_duration);
  ros::Time window{ ros::Time::now() + window_size };
  while (status.ok())
  {
    // DEBUG_VARS(window, ros::Time::now());
    if (window < ros::Time::now())
    {
      set_ofs(ofs_poses, filename_poses);
      set_ofs(ofs_plan, filename_plan);
      set_ofs(ofs_control, filename_ctrl);
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
  ofs_plan.close();
  ofs_control.close();
  // ofs.close();
  // std::ofstream ofs_plan(filename_plan.c_str(), std::ofstream::trunc);
  // ofs_plan << strstr_plan.str();
  // ofs_plan.close();

  // std::ofstream ofs_control(filename_ctrl.c_str(), std::ofstream::trunc);
  // ofs_control << strstr_ctrl.str();
  // ofs_control.close();

  return 0;
}